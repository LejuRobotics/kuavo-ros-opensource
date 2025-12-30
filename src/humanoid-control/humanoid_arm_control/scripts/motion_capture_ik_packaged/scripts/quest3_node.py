#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import signal

import rospkg
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from tools.drake_trans import *
from tools.quest3_utils import Quest3ArmInfoTransformer
import argparse

from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
from kuavo_msgs.srv import changeArmCtrlMode
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList, JoySticks
from handcontrollerdemorosnode.msg import robotHandPosition

class Quest3Node:
    def __init__(self):
        rospack = rospkg.RosPack()
        kuavo_assests_path = rospack.get_path("kuavo_assets")
        robot_version = os.environ.get('ROBOT_VERSION', '40')
        self.model_path = kuavo_assests_path + f"/models/biped_s{robot_version}"
        self.quest3_arm_info_transformer = Quest3ArmInfoTransformer(self.model_path)
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

        rospy.init_node('quest3_node')
        self.control_robot_hand_position_pub = rospy.Publisher("control_robot_hand_position", robotHandPosition, queue_size=10)
        self.pub = rospy.Publisher('/ik/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)

        rospy.Subscriber("/leju_quest_bone_poses", PoseInfoList, self.quest_bone_poses_callback)
        rospy.Subscriber("/quest_joystick_data", JoySticks, self.joySticks_data_callback)

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

    def change_arm_ctrl_mode(self, mode: bool):
        service_name = "/change_arm_ctrl_mode"
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
            print("\033[91mDetected VR ERROR!!! Please restart VR app in quest3!!!\033[0m")
            return
        if self.quest3_arm_info_transformer.is_runing:
            self.pub.publish(eef_pose_msg)

        if self.send_srv and (self.last_quest_running_state != self.quest3_arm_info_transformer.is_runing):
            print(f"Quest running state change to: {self.quest3_arm_info_transformer.is_runing}")
            self.change_arm_ctrl_mode(self.quest3_arm_info_transformer.is_runing)
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
    parser.add_argument("--ee_type", "--end_effector_type", dest="end_effector_type", type=str, default="", help="End effector type, jodell or qiangnao.")
    args, unknown = parser.parse_known_args()
    
    quest3_node = Quest3Node()
    quest3_node.end_effector_type = args.end_effector_type
    print(f"end effector type: {quest3_node.end_effector_type}")
    quest3_node.send_srv = args.send_srv
    print(f"Send srv?: {quest3_node.send_srv}")
    print("Quest3 node started")
    rospy.spin()
