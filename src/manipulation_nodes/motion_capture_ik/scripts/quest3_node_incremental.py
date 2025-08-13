#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import signal
import rospy
import rospkg
import numpy as np
from sensor_msgs.msg import JointState
from tools.drake_trans import *
from tools.quest3_utils import Quest3ArmInfoTransformer
import argparse
import enum
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam, sensorsData
from kuavo_msgs.srv import changeTorsoCtrlMode, changeTorsoCtrlModeRequest, changeArmCtrlMode, changeArmCtrlModeRequest
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList
from kuavo_msgs.msg import JoySticks
from kuavo_msgs.msg import robotHandPosition
from kuavo_msgs.srv import controlLejuClaw, controlLejuClawRequest
from kuavo_msgs.msg import lejuClawCommand
from kuavo_msgs.srv import fkSrv
from kuavo_msgs.msg import twoArmHandPose
from std_msgs.msg import Float32MultiArray
from enum import Enum

class IncrementalMpcCtrlMode(Enum):
    """表示Kuavo机器人 Manipulation MPC 控制模式的枚举类"""
    NoControl = 0
    """无控制"""
    ArmOnly = 1
    """仅控制手臂"""
    BaseOnly = 2
    """仅控制底座"""
    BaseArm = 3
    """同时控制底座和手臂"""
    ERROR = -1
    """错误状态"""
    
def get_package_path(package_name):
    try:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(package_name)
        return package_path
    except rospkg.ResourceNotFound:
        return None

def reset_mm_mpc():
    rospy.wait_for_service('/reset_mm_mpc')
    try:
        reset_mpc = rospy.ServiceProxy('/reset_mm_mpc', changeTorsoCtrlMode)
        req = changeTorsoCtrlModeRequest()
        res = reset_mpc(req)
        if res.result:
            rospy.loginfo("Mobile manipulator MPC reset successfully")
        else:
            rospy.logerr("Failed to reset mobile manipulator MPC")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to %s failed: %s", '/reset_mm_mpc', e)
    except rospy.ROSException as e:
        rospy.logerr("Failed to connect to service %s: %s", '/reset_mm_mpc', e)
    except Exception as e:
        rospy.logerr("Failed to reset mobile manipulator MPC: %s", e)

class Quest3Node:
    class ControlMode(enum.Enum):
        NONE_MODE = 0
        INCREMENTAL_MODE = 1      # 增量控制模式
        FOLLOW_MODE = 2           # 跟随模式

    def __init__(self):
        rospy.init_node('quest3_node')
        
        self.model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
        self.use_custom_ik_param = True
        self.ik_solve_param = ikSolveParam()
        self.incremental_control = False
        
        # Initialize IK solver parameters
        self.set_ik_solver_params()

        self.end_effector_type = "qiangnao"
        self.send_srv = True
        self.last_quest_running_state = False
        self.joySticks_data = None
        self.button_y_last = False
        self.freeze_finger = False
        self.ik_error_norm = [0.0, 0.0]
        self.arm_joint_angles = None
        self.control_mode  = Quest3Node.ControlMode.NONE_MODE
        # 发送给IK求解的目标位姿
        self._left_target_pose = (None, None)   # tuple(pos, quat), quat(x, y, z, w)
        self._right_target_pose = (None, None)  # tuple(pos, quat), quat(x, y, z, w) 
        # 计算增量的 VR 锚点
        self._left_anchor_pose = (None, None)
        self._right_anchor_pose = (None, None)
        
        # Marker可视化开关
        self.enable_markers = rospy.get_param('/quest3/enable_markers', True)
        self.enable_safety = rospy.get_param('/quest3/enable_safety', True)
        # 安全保护参数
        self.max_pos_diff = rospy.get_param('/quest3/max_pos_diff', 0.45)
        self.max_quat_diff = rospy.get_param('/quest3/max_quat_diff', 0.1)  
        
        print(f"安全保护参数:")
        print(f"  目标与当前位置最大差异: {self.max_pos_diff}m")
        print(f"  目标与当前姿态最大差异: {self.max_quat_diff}rad ({np.degrees(self.max_quat_diff):.1f}度)")

        kuavo_assests_path = get_package_path("kuavo_assets")
        robot_version = os.environ.get('ROBOT_VERSION', '40')
        model_config_file = kuavo_assests_path + f"/config/kuavo_v{robot_version}/kuavo.json"
        import json
        with open(model_config_file, 'r') as f:
            model_config = json.load(f)
        upper_arm_length = model_config["upper_arm_length"]
        lower_arm_length = model_config["lower_arm_length"]
        shoulder_width = model_config["shoulder_width"] 
        print(f"upper_arm_length: {upper_arm_length}, lower_arm_length: {lower_arm_length}, shoulder_width: {shoulder_width}")
        rospy.set_param("/quest3/upper_arm_length", upper_arm_length)
        rospy.set_param("/quest3/lower_arm_length", lower_arm_length)
        rospy.set_param("/quest3/shoulder_width", shoulder_width)
        
        # Get hand reference mode from parameter or use default
        hand_reference_mode = rospy.get_param('~hand_reference_mode', 'thumb_index')
        print(f"Hand reference mode: {hand_reference_mode}")
        
        self.quest3_arm_info_transformer = Quest3ArmInfoTransformer(self.model_path, hand_reference_mode=hand_reference_mode)
        
        self.control_robot_hand_position_pub = rospy.Publisher("control_robot_hand_position", robotHandPosition, queue_size=10)
        self.pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
        self.leju_claw_command_pub = rospy.Publisher("leju_claw_command", lejuClawCommand, queue_size=10)

        # Marker发布器 - 用于可视化末端位置
        self.marker_pub = rospy.Publisher('/quest3/end_effector_markers', MarkerArray, queue_size=10)

        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/leju_quest_bone_poses", PoseInfoList, self.quest_bone_poses_callback)
        rospy.Subscriber("/quest_joystick_data", JoySticks, self.joySticks_data_callback)
        rospy.Subscriber("/sensors_data_raw", sensorsData, self.sensors_data_raw_callback)
        rospy.Subscriber("/ik/error_norm", Float32MultiArray, self.ik_error_norm_callback)

    def set_control_torso_mode(self, mode: bool):
        self.quest3_arm_info_transformer.control_torso = mode

    def set_ik_solver_params(self):
        self.ik_solve_param.major_optimality_tol = 9e-3
        self.ik_solve_param.major_feasibility_tol = 9e-3
        self.ik_solve_param.minor_feasibility_tol = 9e-3
        self.ik_solve_param.major_iterations_limit = 50
        self.ik_solve_param.oritation_constraint_tol = 9e-3
        self.ik_solve_param.pos_constraint_tol = 9e-3
        self.ik_solve_param.pos_cost_weight = 10.0

    def pub_robot_end_hand(self, joyStick_data=None, hand_finger_data=None):
        left_hand_position = [0 for _ in range(6)]
        right_hand_position = [0 for _ in range(6)]
        robot_hand_position = robotHandPosition()
        robot_hand_position.header.stamp = rospy.Time.now()

        if self.end_effector_type == "qiangnao":
            self.handle_qiangnao(joyStick_data, hand_finger_data, left_hand_position, right_hand_position, robot_hand_position)
        elif self.end_effector_type == "jodell":
            self.handle_jodell(hand_finger_data, left_hand_position, right_hand_position, robot_hand_position)
        elif self.end_effector_type == "lejuclaw":
            self.handle_lejuclaw(hand_finger_data)

    def pub_leju_claw_command(self, pos:list, vel:list, effort:list) -> None:
        msg = lejuClawCommand()
        msg.data.name = ['left_claw', 'right_claw']
        msg.data.position = pos
        msg.data.velocity = vel
        msg.data.effort = effort
        self.leju_claw_command_pub.publish(pos)

    @staticmethod
    def control_lejuclaw(pos:list, vel:list, effort:list):
        service_name = "/control_robot_leju_claw"
        try:
            rospy.wait_for_service("/control_robot_leju_claw", timeout=1)
            control_lejucalw_srv = rospy.ServiceProxy(
                service_name, controlLejuClaw
            )
            req = controlLejuClawRequest()
            req.data.name = ['left_claw', 'right_claw']
            req.data.position = pos
            req.data.velocity = vel
            req.data.effort = effort
            control_lejucalw_srv(pos)
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
        except Exception as e:
            rospy.logerr(f"Error: {e}")  
            
    def handle_qiangnao(self, joyStick_data, hand_finger_data, left_hand_position, right_hand_position, robot_hand_position):
        if joyStick_data is not None:
            if joyStick_data.left_second_button_pressed and not self.button_y_last:
                print(f"\033[91mButton Y is pressed.\033[0m")
                self.freeze_finger = not self.freeze_finger
            self.button_y_last = joyStick_data.left_second_button_pressed

            for i in range(6):
                if i <= 2:
                    left_hand_position[i] = int(100.0 * joyStick_data.left_trigger)
                    right_hand_position[i] = int(100.0 * joyStick_data.right_trigger)
                else:
                    left_hand_position[i] = int(100.0 * joyStick_data.left_grip)
                    right_hand_position[i] = int(100.0 * joyStick_data.right_grip)

                # Clamp values to [0, 100]
                left_hand_position[i] = max(0, min(left_hand_position[i], 100))
                right_hand_position[i] = max(0, min(right_hand_position[i], 100))

            left_hand_position[1] = 100 if joyStick_data.left_first_button_touched else 0
            right_hand_position[1] = 100 if joyStick_data.right_first_button_touched else 0

        elif hand_finger_data is not None:
            left_qpos = hand_finger_data[0]
            right_qpos = hand_finger_data[1]
            for i in range(6):
                left_hand_position[i] = int(100.0 * left_qpos[i] / 1.70)
                right_hand_position[i] = int(100.0 * right_qpos[i] / 1.70)
                left_hand_position[i] = max(0, min(left_hand_position[i], 100))
                right_hand_position[i] = max(0, min(right_hand_position[i], 100))

        robot_hand_position.left_hand_position = left_hand_position
        robot_hand_position.right_hand_position = right_hand_position
        if not self.freeze_finger:
            self.control_robot_hand_position_pub.publish(robot_hand_position)

    def handle_jodell(self, hand_finger_data, left_hand_position, right_hand_position, robot_hand_position):
        if hand_finger_data is not None:
            left_qpos = hand_finger_data[0]
            right_qpos = hand_finger_data[1]
            left_hand_position[0] = max(0, min(int(255.0 * left_qpos[2] / 1.70), 255))
            right_hand_position[0] = max(0, min(int(255.0 * right_qpos[2] / 1.70), 255))
        else:
            return

        robot_hand_position.left_hand_position = left_hand_position
        robot_hand_position.right_hand_position = right_hand_position
        if not self.freeze_finger:
            self.control_robot_hand_position_pub.publish(robot_hand_position)

    def handle_lejuclaw(self, hand_finger_data, vel=[90, 90], tor = [1.0, 1.0]):
        pos = [0.0, 0.0] 
        if hand_finger_data is not None:
            left_qpos = hand_finger_data[0]
            right_qpos = hand_finger_data[1]
            pos[0] = max(0, min(int(100.0 * left_qpos[2] / 1.70), 100))
            pos[1] = max(0, min(int(100.0 * right_qpos[2] / 1.70), 100))
            self.pub_leju_claw_command(pos, vel, tor)
        else:
            return

    def change_arm_ctrl_mode(self, mode: int):
        service_name = "/change_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name)
            changeHandTrackingMode_srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            changeHandTrackingMode_srv(mode)
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")

    def change_mobile_ctrl_mode(self, mode: int):
        # print(f"change_mobile_ctrl_mode: {mode}")
        mobile_manipulator_service_name = "/mobile_manipulator_mpc_control"
        try:
            rospy.wait_for_service(mobile_manipulator_service_name)
            changeHandTrackingMode_srv = rospy.ServiceProxy(mobile_manipulator_service_name, changeArmCtrlMode)
            changeHandTrackingMode_srv(mode)
        except rospy.ROSException:
            rospy.logerr(f"Service {mobile_manipulator_service_name} not available")

    def change_mm_wbc_arm_ctrl_mode(self, mode: int):
        # print(f"change_wbc_arm_ctrl_mode: {mode}")
        service_name = "/enable_mm_wbc_arm_trajectory_control"
        try:
            rospy.wait_for_service(service_name)
            changeHandTrackingMode_srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            changeHandTrackingMode_srv(mode)
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")

    def ik_error_norm_callback(self, msg):
        """
        Callback for left hand error norm messages.
        """
        self.ik_error_norm = msg.data

    def sensors_data_raw_callback(self, msg):
        if len(msg.joint_data.joint_q) >= 26:
            self.arm_joint_angles = msg.joint_data.joint_q[12:26]

    def fk_srv_client(self, joint_angles):
        service_name = "/ik/fk_srv"
        try:
            rospy.wait_for_service(service_name)
            fk_srv = rospy.ServiceProxy(service_name, fkSrv)
            fk_result = fk_srv(joint_angles)
            rospy.loginfo(f"FK result: {fk_result.success}")
            return fk_result.hand_poses
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return None
    
    def get_current_end_effector_poses(self):
        """通过TF获取机器人当前末端位姿"""
        try:
            # 获取左手末端位姿
            left_transform = self.tf_buffer.lookup_transform(
                'base_link', 'zarm_l7_link', rospy.Time(0), rospy.Duration(0.1)
            )
            left_pos = np.array([
                left_transform.transform.translation.x,
                left_transform.transform.translation.y,
                left_transform.transform.translation.z
            ])
            left_quat = np.array([
                left_transform.transform.rotation.x,
                left_transform.transform.rotation.y,
                left_transform.transform.rotation.z,
                left_transform.transform.rotation.w
            ])
            
            # 获取右手末端位姿
            right_transform = self.tf_buffer.lookup_transform(
                'base_link', 'zarm_r7_link', rospy.Time(0), rospy.Duration(0.1)
            )
            right_pos = np.array([
                right_transform.transform.translation.x,
                right_transform.transform.translation.y,
                right_transform.transform.translation.z
            ])
            right_quat = np.array([
                right_transform.transform.rotation.x,
                right_transform.transform.rotation.y,
                right_transform.transform.rotation.z,
                right_transform.transform.rotation.w
            ])
            
            return {
                'left_pos': left_pos,
                'left_quat': left_quat,
                'right_pos': right_pos,
                'right_quat': right_quat
            }
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed: {e}")
            return None

    def create_end_effector_markers(self, left_target_pose, right_target_pose, left_current_pose=None, right_current_pose=None):
        """创建末端执行器的可视化Marker"""
        marker_array = MarkerArray()
        
        # 创建左手目标位置Marker
        if left_target_pose[0] is not None:
            left_target_marker = Marker()
            left_target_marker.header.frame_id = "base_link"
            left_target_marker.header.stamp = rospy.Time.now()
            left_target_marker.ns = "left_target"
            left_target_marker.id = 0
            left_target_marker.type = Marker.SPHERE
            left_target_marker.action = Marker.ADD
            
            # 设置位置
            left_target_marker.pose.position.x = left_target_pose[0][0]
            left_target_marker.pose.position.y = left_target_pose[0][1]
            left_target_marker.pose.position.z = left_target_pose[0][2]
            
            # 设置姿态
            left_target_marker.pose.orientation.x = left_target_pose[1][0]
            left_target_marker.pose.orientation.y = left_target_pose[1][1]
            left_target_marker.pose.orientation.z = left_target_pose[1][2]
            left_target_marker.pose.orientation.w = left_target_pose[1][3]
            
            # 设置大小和颜色
            left_target_marker.scale.x = 0.05
            left_target_marker.scale.y = 0.05
            left_target_marker.scale.z = 0.05
            left_target_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # 红色，目标位置
            
            marker_array.markers.append(left_target_marker)
        
        # 创建右手目标位置Marker
        if right_target_pose[0] is not None:
            right_target_marker = Marker()
            right_target_marker.header.frame_id = "base_link"
            right_target_marker.header.stamp = rospy.Time.now()
            right_target_marker.ns = "right_target"
            right_target_marker.id = 1
            right_target_marker.type = Marker.SPHERE
            right_target_marker.action = Marker.ADD
            
            # 设置位置
            right_target_marker.pose.position.x = right_target_pose[0][0]
            right_target_marker.pose.position.y = right_target_pose[0][1]
            right_target_marker.pose.position.z = right_target_pose[0][2]
            
            # 设置姿态
            right_target_marker.pose.orientation.x = right_target_pose[1][0]
            right_target_marker.pose.orientation.y = right_target_pose[1][1]
            right_target_marker.pose.orientation.z = right_target_pose[1][2]
            right_target_marker.pose.orientation.w = right_target_pose[1][3]
            
            # 设置大小和颜色
            right_target_marker.scale.x = 0.05
            right_target_marker.scale.y = 0.05
            right_target_marker.scale.z = 0.05
            right_target_marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # 绿色，目标位置
            
            marker_array.markers.append(right_target_marker)
        
        # 创建左手当前位置Marker
        if left_current_pose is not None:
            left_current_marker = Marker()
            left_current_marker.header.frame_id = "base_link"
            left_current_marker.header.stamp = rospy.Time.now()
            left_current_marker.ns = "left_current"
            left_current_marker.id = 2
            left_current_marker.type = Marker.SPHERE
            left_current_marker.action = Marker.ADD
            
            # 设置位置
            left_current_marker.pose.position.x = left_current_pose[0]
            left_current_marker.pose.position.y = left_current_pose[1]
            left_current_marker.pose.position.z = left_current_pose[2]
            
            # 设置姿态
            left_current_marker.pose.orientation.x = left_current_pose[3]
            left_current_marker.pose.orientation.y = left_current_pose[4]
            left_current_marker.pose.orientation.z = left_current_pose[5]
            left_current_marker.pose.orientation.w = left_current_pose[6]
            
            # 设置大小和颜色
            left_current_marker.scale.x = 0.03
            left_current_marker.scale.y = 0.03
            left_current_marker.scale.z = 0.03
            left_current_marker.color = ColorRGBA(1.0, 0.5, 0.0, 0.6)  # 橙色，当前位置
            
            marker_array.markers.append(left_current_marker)
        
        # 创建右手当前位置Marker
        if right_current_pose is not None:
            right_current_marker = Marker()
            right_current_marker.header.frame_id = "base_link"
            right_current_marker.header.stamp = rospy.Time.now()
            right_current_marker.ns = "right_current"
            right_current_marker.id = 3
            right_current_marker.type = Marker.SPHERE
            right_current_marker.action = Marker.ADD
            
            # 设置位置
            right_current_marker.pose.position.x = right_current_pose[0]
            right_current_marker.pose.position.y = right_current_pose[1]
            right_current_marker.pose.position.z = right_current_pose[2]
            
            # 设置姿态
            right_current_marker.pose.orientation.x = right_current_pose[3]
            right_current_marker.pose.orientation.y = right_current_pose[4]
            right_current_marker.pose.orientation.z = right_current_pose[5]
            right_current_marker.pose.orientation.w = right_current_pose[6]
            
            # 设置大小和颜色
            right_current_marker.scale.x = 0.03
            right_current_marker.scale.y = 0.03
            right_current_marker.scale.z = 0.03
            right_current_marker.color = ColorRGBA(0.0, 0.5, 1.0, 0.6)  # 蓝色，当前位置
            
            marker_array.markers.append(right_current_marker)
        
        return marker_array

    def quest_bone_poses_callback(self, quest_bone_poses_msg):
        self.quest3_arm_info_transformer.read_msg(quest_bone_poses_msg)
        left_pose, left_elbow_pos = self.quest3_arm_info_transformer.get_hand_pose("Left")
        right_pose, right_elbow_pos = self.quest3_arm_info_transformer.get_hand_pose("Right")
        
        left_finger_joints = self.quest3_arm_info_transformer.get_finger_joints("Left")
        right_finger_joints = self.quest3_arm_info_transformer.get_finger_joints("Right")
        
        eef_pose_msg = None

        if(self.quest3_arm_info_transformer.check_if_vr_error()):
            print("\033[91mDetected VR ERROR!!! Please restart VR app in quest3 or check the battery level of the joystick!!!\033[0m")
            return

        if self.incremental_control:
            def is_incremental_control(joySticks_data):
                if joySticks_data is not None:
                    if joySticks_data.left_first_button_touched and joySticks_data.right_first_button_touched:
                        if joySticks_data.left_first_button_pressed and joySticks_data.right_first_button_pressed:
                            # 触摸左右第一个按键，并且不是按下则认为是增量控制
                            return False
                        return True
                return False

            if is_incremental_control(self.joySticks_data):
                if self.control_mode != Quest3Node.ControlMode.INCREMENTAL_MODE:

                    if self.enable_safety and not self.safe_check_enter_incremental_mode(left_pose, right_pose):
                        return

                    print("\033[93m++++++++++++++++++++开始增量模式+++++++++++++++++\033[0m")
                    # 刚开始切换到增量控制模式
                    self.control_mode = Quest3Node.ControlMode.INCREMENTAL_MODE
                    
                    # 重置MPC状态
                    print("\033[94m重置 Mobile Manipulator MPC...\033[0m")
                    reset_mm_mpc()
                    
                    self.change_mobile_ctrl_mode(IncrementalMpcCtrlMode.ArmOnly.value)
                    # 设置当前VR的末端位姿为锚点
                    self._left_anchor_pose = left_pose
                    self._right_anchor_pose = right_pose
                    
                    # 只有在目标位姿为空时才重新获取FK，避免抖动
                    def is_pose_empty(pose):
                        return pose[0] is None or pose[1] is None
                    
                    if is_pose_empty(self._left_target_pose) or is_pose_empty(self._right_target_pose):
                        hand_poses = self.fk_srv_client(self.arm_joint_angles)
                        if hand_poses is not None:
                            print("********************************* FK (First Time) *********************************")
                            self._left_target_pose = (hand_poses.left_pose.pos_xyz, hand_poses.left_pose.quat_xyzw)
                            self._right_target_pose = (hand_poses.right_pose.pos_xyz, hand_poses.right_pose.quat_xyzw)
                            # Print target poses from FK
                            print("\033[92m左手FK目标位置: {}\033[0m".format(self._left_target_pose[0]))
                            print("\033[92m左手FK目标姿态(四元数): {}\033[0m".format(self._left_target_pose[1]))
                            print("\033[92m右手FK目标位置: {}\033[0m".format(self._right_target_pose[0]))
                            print("\033[92m右手FK目标姿态(四元数): {}\033[0m".format(self._right_target_pose[1]))
                        else:
                            print("********************************* FK ERROR *********************************")
                            self._left_target_pose = left_pose
                            self._right_target_pose = right_pose
                    else:
                        print("********************************* 使用上次目标位姿 (避免抖动) *********************************")
                        print("\033[92m左手保持目标位置: {}\033[0m".format(self._left_target_pose[0]))
                        print("\033[92m左手保持目标姿态(四元数): {}\033[0m".format(self._left_target_pose[1]))
                        print("\033[92m右手保持目标位置: {}\033[0m".format(self._right_target_pose[0]))
                        print("\033[92m右手保持目标姿态(四元数): {}\033[0m".format(self._right_target_pose[1]))

                # 计算位置增量
                l_xyz_delta = left_pose[0] - self._left_anchor_pose[0]
                r_xyz_delta = right_pose[0] - self._right_anchor_pose[0]
                
                # 添加位置变化阈值，减少噪声影响
                position_threshold = 0.001  # 1mm
                if np.linalg.norm(l_xyz_delta) < position_threshold:
                    l_xyz_delta = np.zeros(3)
                if np.linalg.norm(r_xyz_delta) < position_threshold:
                    r_xyz_delta = np.zeros(3)

                def quaternion_inverse(q):
                    norm = np.sum(np.array(q)**2)
                    return np.array([-q[0], -q[1], -q[2], q[3]]) / norm

                def quaternion_multiply(q1, q2):
                    x1, y1, z1, w1 = q1
                    x2, y2, z2, w2 = q2
                    return np.array([
                        w1*x2 + x1*w2 + y1*z2 - z1*y2,
                        w1*y2 - x1*z2 + y1*w2 + z1*x2,
                        w1*z2 + x1*y2 - y1*x2 + z1*w2,
                        w1*w2 - x1*x2 - y1*y2 - z1*z2
                    ])
                def normalize_quaternion(q):
                    norm = np.linalg.norm(q)
                    return q / norm if norm > 0 else q

                l_anchor_quat_inv = quaternion_inverse(self._left_anchor_pose[1])
                r_anchor_quat_inv = quaternion_inverse(self._right_anchor_pose[1])
                l_delta_quat = quaternion_multiply(left_pose[1], l_anchor_quat_inv)
                r_delta_quat = quaternion_multiply(right_pose[1], r_anchor_quat_inv)
                
                # 添加姿态变化阈值，减少噪声影响
                def quaternion_angle(q):
                    # 计算四元数对应的旋转角度
                    w = abs(q[3])  # 确保w为正
                    if w > 1.0:
                        w = 1.0
                    return 2.0 * np.arccos(w)
                
                orientation_threshold = 0.01  # 约0.57度
                if quaternion_angle(l_delta_quat) < orientation_threshold:
                    l_delta_quat = np.array([0, 0, 0, 1])  # 单位四元数
                if quaternion_angle(r_delta_quat) < orientation_threshold:
                    r_delta_quat = np.array([0, 0, 0, 1])  # 单位四元数
                
                # 只有在有显著变化时才更新锚点
                if np.linalg.norm(l_xyz_delta) > 0 or quaternion_angle(l_delta_quat) > 0:
                    self._left_anchor_pose = left_pose
                if np.linalg.norm(r_xyz_delta) > 0 or quaternion_angle(r_delta_quat) > 0:
                    self._right_anchor_pose = right_pose
                    
                l_target_quat = quaternion_multiply(l_delta_quat, self._left_target_pose[1])
                r_target_quat = quaternion_multiply(r_delta_quat, self._right_target_pose[1])

                l_target_quat = normalize_quaternion(l_target_quat)
                r_target_quat = normalize_quaternion(r_target_quat)

                self._left_target_pose = (self._left_target_pose[0] + l_xyz_delta, l_target_quat)
                self._right_target_pose = (self._right_target_pose[0] + r_xyz_delta, r_target_quat)
                left_elbow_pos = np.zeros(3)
                right_elbow_pos = np.zeros(3)

                eef_pose_msg = twoArmHandPoseCmd()
                eef_pose_msg.hand_poses.left_pose.pos_xyz = self._left_target_pose[0]
                eef_pose_msg.hand_poses.left_pose.quat_xyzw = self._left_target_pose[1]
                eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow_pos
                eef_pose_msg.hand_poses.right_pose.pos_xyz = self._right_target_pose[0]
                eef_pose_msg.hand_poses.right_pose.quat_xyzw = self._right_target_pose[1]
                eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow_pos

                # 发布Marker用于可视化
                # current_poses = self.get_current_end_effector_poses()
                # left_current_pose = None
                # right_current_pose = None
                # if current_poses is not None:
                #     left_current_pose = np.concatenate([current_poses['left_pos'], current_poses['left_quat']])
                #     right_current_pose = np.concatenate([current_poses['right_pos'], current_poses['right_quat']])
                
                # marker_array = self.create_end_effector_markers(
                #     self._left_target_pose, self._right_target_pose,
                #     left_current_pose, right_current_pose
                # )
                # self.marker_pub.publish(marker_array)

            else: # Incremental mode OFF
                if self.control_mode == Quest3Node.ControlMode.INCREMENTAL_MODE:
                    print("\033[93m--------------------退出增量模式-----------------\033[0m")
                    self.change_mobile_ctrl_mode(IncrementalMpcCtrlMode.NoControl.value)
                    # 注意：这里不清空目标位姿，保持状态以避免下次进入时的抖动
                    print("\033[94m保持目标位姿以避免下次进入增量模式时的抖动\033[0m")
                self.control_mode = Quest3Node.ControlMode.NONE_MODE
        
        if eef_pose_msg is not None:
            eef_pose_msg.ik_param = self.ik_solve_param
            eef_pose_msg.use_custom_ik_param = self.use_custom_ik_param

            if any(error > 75.0 for error in self.ik_error_norm):
                rospy.logwarn(f"Hand error norm too large: {self.ik_error_norm}, skipping arm trajectory publication")
            elif self.quest3_arm_info_transformer.is_runing:
                self.pub.publish(eef_pose_msg)

        if self.send_srv and (self.last_quest_running_state != self.quest3_arm_info_transformer.is_runing):
            print(f"Quest running state change to: {self.quest3_arm_info_transformer.is_runing}")
            mode = 2 if self.quest3_arm_info_transformer.is_runing else 0
            mobile_mode = 1 if self.quest3_arm_info_transformer.is_runing else 0
            wbc_mode = 1 if self.quest3_arm_info_transformer.is_runing else 0
            self.change_arm_ctrl_mode(mode)
            self.change_mobile_ctrl_mode(mobile_mode)
            self.change_mm_wbc_arm_ctrl_mode(wbc_mode)
            print("Received service response of changing arm control mode.")
            self.last_quest_running_state = self.quest3_arm_info_transformer.is_runing
        
        if self.joySticks_data is None:  # 优先使用手柄数据
            self.pub_robot_end_hand(hand_finger_data=[left_finger_joints, right_finger_joints])
        
        # self.joySticks_data = None

    def joySticks_data_callback(self, msg):
        self.quest3_arm_info_transformer.read_joySticks_msg(msg)
        self.joySticks_data = msg
        self.pub_robot_end_hand(joyStick_data=self.joySticks_data)

    def safe_check_enter_incremental_mode(self, left_pose, right_pose):

        # 额外的安全检查：确保目标位姿与机器人当前位姿的差异不会过大
        def check_target_vs_current_safety(target_pos, target_quat, current_pos, current_quat, hand_name):
            # 如果禁用安全保护，直接返回True
            if not self.enable_safety:
                return True
            
            if current_pos is None or current_quat is None:
                return True  # 如果没有当前位姿信息，跳过检查
            
            # 使用可配置的安全阈值
            max_pos_diff = self.max_pos_diff
            max_quat_diff = self.max_quat_diff
            
            # 检查位置差异
            pos_diff = np.linalg.norm(target_pos - current_pos)
            if pos_diff > max_pos_diff:
                print(f"\033[91m[安全警告] {hand_name}目标位置与当前位置差异过大: {pos_diff:.3f}m > {max_pos_diff}m\033[0m")
                return False
            
            # 检查姿态差异
            def quaternion_angle_difference(q1, q2):
                dot_product = np.clip(np.abs(np.dot(q1, q2)), -1.0, 1.0)
                return 2.0 * np.arccos(dot_product)
            
            quat_diff = quaternion_angle_difference(target_quat, current_quat)
            if quat_diff > max_quat_diff:
                print(f"\033[91m[安全警告] {hand_name}目标姿态与当前姿态差异过大: {quat_diff:.3f}rad > {max_quat_diff}rad\033[0m")
                return False
            
            return True
        
        # 获取当前机器人位姿（通过TF）
        current_left_pos = None
        current_left_quat = None
        current_right_pos = None
        current_right_quat = None
        
        current_poses = self.get_current_end_effector_poses()
        if current_poses is not None:
            current_left_pos = current_poses['left_pos']
            current_left_quat = current_poses['left_quat']
            current_right_pos = current_poses['right_pos']
            current_right_quat = current_poses['right_quat']
        
        # 执行安全检查
        left_safe = check_target_vs_current_safety(
            left_pose[0],
            left_pose[1],
            current_left_pos, current_left_quat, "左手"
        )
        right_safe = check_target_vs_current_safety(
            right_pose[0],
            right_pose[1],
            current_right_pos, current_right_quat, "右手"
        )
        return left_safe and right_safe

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    parser = argparse.ArgumentParser()
    parser.add_argument("--send_srv", type=int, default=1, help="Send arm control service, True or False.")
    parser.add_argument("--ee_type", "--end_effector_type", dest="end_effector_type", type=str, default="", help="End effector type, jodell, qiangnao or lejuclaw.")
    parser.add_argument("--control_torso", type=int, default=0, help="0: do NOT control, 1: control torso.")
    parser.add_argument("--incremental_control", type=int, default=0, help="0: direct control, 1: incremental control.")
    args, unknown = parser.parse_known_args()
    
    quest3_node = Quest3Node()
    quest3_node.end_effector_type = args.end_effector_type
    print(f"end effector type: {quest3_node.end_effector_type}")
    quest3_node.send_srv = args.send_srv
    print(f"Send srv?: {quest3_node.send_srv}")
    quest3_node.set_control_torso_mode(args.control_torso)
    print(f"Control torso?: {args.control_torso}")
    quest3_node.incremental_control = bool(args.incremental_control)
    print(f"Incremental control?: {quest3_node.incremental_control}")
    print("Quest3 node started")
    rospy.spin()
