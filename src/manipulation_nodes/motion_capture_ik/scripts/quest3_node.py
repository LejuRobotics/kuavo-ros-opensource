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

from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
from kuavo_msgs.srv import changeArmCtrlMode
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList, JoySticks
from handcontrollerdemorosnode.msg import robotHandPosition
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
        
        self.model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
        self.use_custom_ik_param = True
        self.ik_solve_param = ikSolveParam()
        
        # Initialize IK solver parameters
        self.set_ik_solver_params()

        self.end_effector_type = "qiangnao"
        self.send_srv = True
        self.last_quest_running_state = False
        self.joySticks_data = None
        self.button_y_last = False
        self.freeze_finger = False

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
        self.quest3_arm_info_transformer = Quest3ArmInfoTransformer(self.model_path)
        
        self.control_robot_hand_position_pub = rospy.Publisher("control_robot_hand_position", robotHandPosition, queue_size=10)
        self.pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
        self.leju_claw_command_pub = rospy.Publisher("leju_claw_command", lejuClawCommand, queue_size=10)

        rospy.Subscriber("/leju_quest_bone_poses", PoseInfoList, self.quest_bone_poses_callback)
        rospy.Subscriber("/quest_joystick_data", JoySticks, self.joySticks_data_callback)

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

    def quest_bone_poses_callback(self, quest_bone_poses_msg):
        self.quest3_arm_info_transformer.read_msg(quest_bone_poses_msg)
        left_pose, left_elbow_pos = self.quest3_arm_info_transformer.get_hand_pose("Left")
        right_pose, right_elbow_pos = self.quest3_arm_info_transformer.get_hand_pose("Right")
        
        left_finger_joints = self.quest3_arm_info_transformer.get_finger_joints("Left")
        right_finger_joints = self.quest3_arm_info_transformer.get_finger_joints("Right")
        
        eef_pose_msg = twoArmHandPoseCmd()
        eef_pose_msg.hand_poses.left_pose.pos_xyz = left_pose[0]
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_pose[1]
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow_pos

        eef_pose_msg.hand_poses.right_pose.pos_xyz = right_pose[0]
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_pose[1]
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow_pos
        eef_pose_msg.ik_param = self.ik_solve_param
        eef_pose_msg.use_custom_ik_param = self.use_custom_ik_param

        if(self.quest3_arm_info_transformer.check_if_vr_error()):
            print("\033[91mDetected VR ERROR!!! Please restart VR app in quest3 or check the battery level of the joystick!!!\033[0m")
            return
        if self.quest3_arm_info_transformer.is_runing:
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
        
        self.joySticks_data = None

    def joySticks_data_callback(self, msg):
        self.quest3_arm_info_transformer.read_joySticks_msg(msg)
        self.joySticks_data = msg
        self.pub_robot_end_hand(joyStick_data=self.joySticks_data)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    parser = argparse.ArgumentParser()
    parser.add_argument("--send_srv", type=int, default=1, help="Send arm control service, True or False.")
    parser.add_argument("--ee_type", "--end_effector_type", dest="end_effector_type", type=str, default="", help="End effector type, jodell, qiangnao or lejuclaw.")
    parser.add_argument("--control_torso", type=int, default=0, help="0: do NOT control, 1: control torso.")
    args, unknown = parser.parse_known_args()
    
    quest3_node = Quest3Node()
    quest3_node.end_effector_type = args.end_effector_type
    print(f"end effector type: {quest3_node.end_effector_type}")
    quest3_node.send_srv = args.send_srv
    print(f"Send srv?: {quest3_node.send_srv}")
    quest3_node.set_control_torso_mode(args.control_torso)
    print(f"Control torso?: {args.control_torso}")
    print("Quest3 node started")
    rospy.spin()
