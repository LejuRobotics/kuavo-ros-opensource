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
from handcontrollerdemorosnode.msg import robotHandPosition
import argparse

arm_joint_position = np.zeros(14)
finger_joint_position = [np.zeros(6), np.zeros(6)]

JODELL_SCALE = 1.7/255.0
QIANGNAO_SCALE = 1.7/100.0
QIANGNAO = "qiangnao"
JODELL = "jodell"

end_effector_type = QIANGNAO

def kuavo_arm_traj_callback(msg):
    global arm_joint_position
    arm_joint_position = np.array(msg.position) * np.pi / 180.0

def finger_joint_callback(msg):
    global finger_joint_position, end_effector_type
    if end_effector_type == QIANGNAO:
        for i in range(6):
            finger_joint_position[0][i] = msg.left_hand_position[i] * QIANGNAO_SCALE
            finger_joint_position[1][i] = msg.right_hand_position[i] * QIANGNAO_SCALE
    else:
        for i in range(6):
            finger_joint_position[0][i] = msg.left_hand_position[0] * JODELL_SCALE
            finger_joint_position[1][i] = msg.right_hand_position[0] * JODELL_SCALE

arm_joint_sub = rospy.Subscriber("/kuavo_arm_traj", JointState, kuavo_arm_traj_callback)
finger_joint_sub = rospy.Subscriber("/control_robot_hand_position", robotHandPosition, finger_joint_callback)


if __name__ == '__main__':
    rospy.init_node('visulize_traj')
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    parser = argparse.ArgumentParser()
    parser.add_argument("--ee_type", "--end_effector_type", dest="end_effector_type", type=str, default="", help="End effector type, jodell or qiangnao.")
    args, unknown = parser.parse_known_args()
    end_effector_type = args.end_effector_type
    print(f"end effector type: {end_effector_type}")

    rospack = rospkg.RosPack()
    kuavo_assests_path = rospack.get_path("kuavo_assets")
    robot_version = os.environ.get('ROBOT_VERSION', '40')
    model_path = kuavo_assests_path + f"/models/biped_s{robot_version}"
    quest3_arm_info_transformer = Quest3ArmInfoTransformer(model_path)
    # rospy.spin()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        quest3_arm_info_transformer.pub_whole_body_joint_state_msg(arm_joint_position, finger_joint_position[0], finger_joint_position[1])
        rate.sleep()
