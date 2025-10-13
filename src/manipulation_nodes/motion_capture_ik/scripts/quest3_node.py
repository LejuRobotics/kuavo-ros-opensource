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
from tools.kalman_filter import TwoArmPosePredictor
from visualization_msgs.msg import MarkerArray, Marker
import argparse
from copy import deepcopy
from std_msgs.msg import Float64
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeArmCtrlMode, fkSrv
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.msg import JoySticks
from kuavo_msgs.msg import robotHandPosition
from kuavo_msgs.srv import controlLejuClaw, controlLejuClawRequest
from kuavo_msgs.msg import lejuClawCommand
def get_package_path(package_name):
    try:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(package_name)
        return package_path
    except rospkg.ResourceNotFound:
        return None
class Quest3Node:
    def __init__(self):
        rospy.init_node('quest3_node')
        
        self.use_custom_ik_param = True
        self.ik_solve_param = ikSolveParam()
        
        # Initialize IK solver parameters
        self.set_ik_solver_params()

        self.end_effector_type = "qiangnao"
        self.send_srv = True
        self.rl = False
        self.last_quest_running_state = False
        self.joySticks_data = None
        self.button_y_last = False
        self.freeze_finger = False
        self.arm_joint_angles = None
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
        rospy.set_param("/quest3/base_height_offset", model_config.get("base_height_offset", 0.23))
    
        rospy.set_param("/quest3/shoulder_width", shoulder_width)
        
        # Get hand reference mode from parameter or use default
        hand_reference_mode = rospy.get_param('~hand_reference_mode', 'thumb_index')
        print(f"Hand reference mode: {hand_reference_mode}")
        self.model_path = kuavo_assests_path + "/models/biped_s" + str(robot_version)
        print(f"vis model path: {self.model_path}")

        import json
        with open(model_config_file, 'r') as f:
            model_config = json.load(f)
        eef_visual_stl_files = model_config["eef_visual_stl_files"]
        print(f"eef_visual_stl_files: {eef_visual_stl_files}")
        self.quest3_arm_info_transformer = Quest3ArmInfoTransformer(self.model_path, eef_visual_stl_files=eef_visual_stl_files, hand_reference_mode=hand_reference_mode)
        
        self.control_robot_hand_position_pub = rospy.Publisher("control_robot_hand_position", robotHandPosition, queue_size=10)
        self.pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
        self.pub_mm_traj = rospy.Publisher('/mm/end_effector_trajectory', armTargetPoses, queue_size=10)
        self.leju_claw_command_pub = rospy.Publisher("leju_claw_command", lejuClawCommand, queue_size=10)

        rospy.Subscriber("/leju_quest_bone_poses", PoseInfoList, self.quest_bone_poses_callback)
        rospy.Subscriber("/quest_joystick_data", JoySticks, self.joySticks_data_callback)
        rospy.Subscriber("/sensors_data_raw", sensorsData, self.sensors_data_raw_callback)
        rospy.Subscriber("/mm/control_type", Float64, self.mm_control_type_callback)
        self.filtered_marker_array_pub = rospy.Publisher("/quest3_node/filtered_marker_array", MarkerArray, queue_size=10)

        self.arm_pose_predictor = None
        self.predict_time = 0.0 # if predict_time is 0, no predict, just publish the filtered pose.

        self.need_interpolate = False
        self.last_mm_control_type = 0
        self.mm_control_type = 0

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
                left_hand_position[i] = int(100.0 * joyStick_data.left_trigger)
                right_hand_position[i] = int(100.0 * joyStick_data.right_trigger)

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

    def quest_bone_poses_callback(self, quest_bone_poses_msg):
        self.quest3_arm_info_transformer.read_msg(quest_bone_poses_msg)
        left_pose, left_elbow_pos = self.quest3_arm_info_transformer.get_hand_pose("Left")
        right_pose, right_elbow_pos = self.quest3_arm_info_transformer.get_hand_pose("Right")
        if left_pose is None or right_pose is None:
            rospy.logwarn("left_pose or right_pose is None!")
            return
        
        left_finger_joints = self.quest3_arm_info_transformer.get_finger_joints("Left")
        right_finger_joints = self.quest3_arm_info_transformer.get_finger_joints("Right")

        if(self.quest3_arm_info_transformer.check_if_vr_error()):
            print("\033[91mDetected VR ERROR!!! Please restart VR app in quest3 or check the battery level of the joystick!!!\033[0m")
            return

        if self.send_srv and (self.last_quest_running_state != self.quest3_arm_info_transformer.is_runing):
            print(f"Quest running state change to: {self.quest3_arm_info_transformer.is_runing}")
            mode = 2 if self.quest3_arm_info_transformer.is_runing else 0
            mobile_mode = 1 if self.quest3_arm_info_transformer.is_runing else 0
            wbc_mode = 1 if self.quest3_arm_info_transformer.is_runing else 0
            self.change_mobile_ctrl_mode(mobile_mode)
            if not self.rl:
                self.change_arm_ctrl_mode(mode)
                self.change_mm_wbc_arm_ctrl_mode(wbc_mode)
                print("Received service response of changing arm control mode.")
            self.last_quest_running_state = self.quest3_arm_info_transformer.is_runing

        if self.quest3_arm_info_transformer.is_runing and self.predict_time <= 1e-5:
            eef_pose_msg = twoArmHandPoseCmd()
            eef_pose_msg.frame = 3
            eef_pose_msg.hand_poses.left_pose.pos_xyz = left_pose[0]
            eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_pose[1]
            eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow_pos

            eef_pose_msg.hand_poses.right_pose.pos_xyz = right_pose[0]
            eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_pose[1]
            eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow_pos
            eef_pose_msg.ik_param = self.ik_solve_param
            eef_pose_msg.use_custom_ik_param = self.use_custom_ik_param

            if self.rl and self.need_interpolate:
                eef_pose_msg = self.interpolate_eef_pose(eef_pose_msg)
            self.pub.publish(eef_pose_msg)

        if self.joySticks_data is None:  # 优先使用手柄数据
            self.pub_robot_end_hand(hand_finger_data=[left_finger_joints, right_finger_joints])
        
        self.joySticks_data = None
        if self.arm_pose_predictor is None:
            dt = 0.01 # 认为是稳定的100Hz的频率
            process_noise_scale=0.1
            measurement_noise_scale=0.01
            self.arm_pose_predictor = TwoArmPosePredictor(left_pose[0], right_pose[0], left_pose[1], right_pose[1], dt,
                                                          process_noise_scale=process_noise_scale,
                                                          measurement_noise_scale=measurement_noise_scale)
            print("Initialized arm pose predictor")
        else:
            predict_time = self.predict_time
            times = [0.0, predict_time]
            values = []
            pos_left, pos_right, quat_left, quat_right = self.arm_pose_predictor.filter(left_pose[0], right_pose[0], left_pose[1], right_pose[1])
            values.append([*pos_left, *quat_left, *pos_right, *quat_right])
            # print(f"pos_left: {pos_left}, pos_right: {pos_right}, quat_left: {quat_left}, quat_right: {quat_right}")
            # visualize the filtered pose
            marker_array = MarkerArray()
            marker_array.markers.append(self.quest3_arm_info_transformer.construct_marker(pos_left, quat_left, rgba=[1,0,0,0.9], side="Left", marker_id=0))
            marker_array.markers.append(self.quest3_arm_info_transformer.construct_marker(pos_right, quat_right, rgba=[1,0,0,0.9], side="Right", marker_id=1))
            
            pos_left, pos_right, quat_left, quat_right = self.arm_pose_predictor.predict_next_pose(predict_time)
            values.append([*pos_left, *quat_left, *pos_right, *quat_right])
            marker_array.markers.append(self.quest3_arm_info_transformer.construct_marker(pos_left, quat_left, rgba=[0,0.8,0,0.3], side="Left", marker_id=2))
            marker_array.markers.append(self.quest3_arm_info_transformer.construct_marker(pos_right, quat_right, rgba=[0,0.8,0,0.3], side="Right", marker_id=3))
            self.filtered_marker_array_pub.publish(marker_array)
            
            values = np.array(values)
            values = values.flatten()
            # publish mm traj
            if self.quest3_arm_info_transformer.is_runing and self.predict_time > 1e-5:
                msg = armTargetPoses()
                msg.times = times
                msg.values = values
                self.pub_mm_traj.publish(msg)
                # print(f"publishing mm traj values: {values}")

    def joySticks_data_callback(self, msg):
        self.quest3_arm_info_transformer.read_joySticks_msg(msg)
        self.joySticks_data = msg
        self.pub_robot_end_hand(joyStick_data=self.joySticks_data)

    def fk_srv_client(self, joint_angles):
        service_name = "/ik/fk_srv"
        try:
            rospy.wait_for_service(service_name, timeout=3.0)
            fk_srv = rospy.ServiceProxy(service_name, fkSrv)
            fk_result = fk_srv(joint_angles)
            # rospy.loginfo(f"FK result: {fk_result.success}")
            return fk_result.hand_poses
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return None

    def sensors_data_raw_callback(self, msg):
        if len(msg.joint_data.joint_q) >= 26:
            self.arm_joint_angles = msg.joint_data.joint_q[12:26]

    def mm_control_type_callback(self, msg):
        self.mm_control_type = int(msg.data)
        if self.mm_control_type == 1 and self.last_mm_control_type == 0:
            print("Interpolation started")
            self.need_interpolate = True
        self.last_mm_control_type = self.mm_control_type

    def interpolate_eef_pose(self, eef_pose_msg: 'twoArmHandPoseCmd', max_dist_threshold: float = 0.3, interp_alpha: float = 0.3
) -> 'twoArmHandPoseCmd':
        """对双手末端位姿进行线性插值，返回插值后的消息。"""

        if self.arm_joint_angles is None:
            return None

        # 拷贝原始消息，避免副作用
        interp_msg = deepcopy(eef_pose_msg)

        # 目标位姿（VR输入）
        l_target_pos = np.array(eef_pose_msg.hand_poses.left_pose.pos_xyz)
        r_target_pos = np.array(eef_pose_msg.hand_poses.right_pose.pos_xyz)

        # 获取当前实际末端（通过FK服务）
        hand_poses = self.fk_srv_client(self.arm_joint_angles)
        if hand_poses is not None:
            l_current_pos = np.array(hand_poses.left_pose.pos_xyz)
            r_current_pos = np.array(hand_poses.right_pose.pos_xyz)

            l_dist = np.linalg.norm(l_target_pos - l_current_pos)
            r_dist = np.linalg.norm(r_target_pos - r_current_pos)

            left_need_interp = l_dist > max_dist_threshold
            right_need_interp = r_dist > max_dist_threshold
            
            if left_need_interp:
                new_l_pos = l_current_pos + interp_alpha * (l_target_pos - l_current_pos)
            else:
                new_l_pos = l_target_pos
                
            if right_need_interp:
                new_r_pos = r_current_pos + interp_alpha * (r_target_pos - r_current_pos)
            else:
                new_r_pos = r_target_pos

            if not left_need_interp and not right_need_interp:
                self.need_interpolate = False
                print(f"Interpolation completed - both hands within threshold")
            
        else:
            # FK失败，直接使用目标
            new_l_pos = l_target_pos
            new_r_pos = r_target_pos

        # 填入新数据
        interp_msg.hand_poses.left_pose.pos_xyz = new_l_pos.tolist()
        interp_msg.hand_poses.right_pose.pos_xyz = new_r_pos.tolist()

        return interp_msg

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    parser = argparse.ArgumentParser()
    parser.add_argument("--send_srv", type=int, default=1, help="Send arm control service, True or False.")
    parser.add_argument("--rl", type=int, default=0, help="RL version.")
    parser.add_argument("--ee_type", "--end_effector_type", dest="end_effector_type", type=str, default="", help="End effector type, jodell, qiangnao or lejuclaw.")
    parser.add_argument("--control_torso", type=int, default=0, help="0: do NOT control, 1: control torso.")
    parser.add_argument("--predict_time", type=float, default=0.0, help="predict time, if predict_time is 0, no predict, just publish the filtered pose.")
    args, unknown = parser.parse_known_args()
    
    quest3_node = Quest3Node()
    quest3_node.end_effector_type = args.end_effector_type
    print(f"end effector type: {quest3_node.end_effector_type}")
    quest3_node.send_srv = args.send_srv
    print(f"Send srv?: {quest3_node.send_srv}")
    quest3_node.set_control_torso_mode(args.control_torso)
    print(f"Control torso?: {args.control_torso}")
    quest3_node.rl = args.rl
    print(f"RL version?: {quest3_node.rl}")
    quest3_node.predict_time = args.predict_time
    print(f"predict time: {quest3_node.predict_time}")
    if quest3_node.predict_time > 0:
        print("Predict time is set, will publish mm traj instead of single-frame /mm/two_arm_hand_pose_cmd")
    else:
        print("Predict time is not set, will publish single-frame /mm/two_arm_hand_pose_cmd instead of mm traj")
    print("Quest3 node started")
    rospy.spin()
