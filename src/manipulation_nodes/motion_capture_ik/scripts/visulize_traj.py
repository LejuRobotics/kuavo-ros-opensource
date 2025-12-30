#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import signal
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from tools.drake_trans import *
from tools.utils import get_package_path
from tools.quest3_utils import Quest3ArmInfoTransformer
from handcontrollerdemorosnode.msg import robotHandPosition
from kuavo_msgs.msg import headBodyPose
import argparse
from kuavo_msgs.msg import twoArmHandPoseCmd, twoArmHandPose, ikSolveParam
from visualization_msgs.msg import Marker

arm_joint_position = np.zeros(14)
finger_joint_position = [np.zeros(6), np.zeros(6)]

JODELL_SCALE = 1.7/255.0
QIANGNAO_SCALE = 1.7/100.0
QIANGNAO = "qiangnao"
JODELL = "jodell"
torso_ypr = [0, 0, 0]
torso_height = 0.0

class HandPose:
    l_p = [0, 0, 0]
    l_q = [0, 0, 0, 1]
    r_p = [0, 0, 0]
    r_q = [0, 0, 0, 1]

hand_pose = HandPose()
hand_pose_res = HandPose()
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

def torso_pose_callback(msg):
    global torso_ypr, torso_height
    torso_ypr = [msg.body_yaw, msg.body_pitch, msg.body_roll]
    torso_height = msg.body_height

def hand_cmd_callback(msg):
    global hand_pose
    hand_pose.l_p = msg.hand_poses.left_pose.pos_xyz
    hand_pose.l_q = msg.hand_poses.left_pose.quat_xyzw
    hand_pose.r_p = msg.hand_poses.right_pose.pos_xyz
    hand_pose.r_q = msg.hand_poses.right_pose.quat_xyzw
    # print("hand pose cmd received")

def hand_result_callback(msg):
    global hand_pose_res
    hand_pose_res.l_p = msg.left_pose.pos_xyz
    hand_pose_res.l_q = msg.left_pose.quat_xyzw
    hand_pose_res.r_p = msg.right_pose.pos_xyz
    hand_pose_res.r_q = msg.right_pose.quat_xyzw
    # print("hand pose result received")

arm_joint_sub = rospy.Subscriber("/kuavo_arm_traj", JointState, kuavo_arm_traj_callback)
finger_joint_sub = rospy.Subscriber("/control_robot_hand_position", robotHandPosition, finger_joint_callback)
finger_joint_sub = rospy.Subscriber("/kuavo_head_body_orientation", headBodyPose, torso_pose_callback)
ik_cmd_sub = rospy.Subscriber("/ik/two_arm_hand_pose_cmd", twoArmHandPoseCmd, hand_cmd_callback)
ik_result_sub = rospy.Subscriber("/ik/result", twoArmHandPose, hand_result_callback)

marker_pub_cmd_left = rospy.Publisher("visualization_marker", Marker, queue_size=10)
marker_pub_cmd_right = rospy.Publisher("visualization_marker_right", Marker, queue_size=10)
marker_pub_res_left = rospy.Publisher("visualization_marker/result", Marker, queue_size=10)
marker_pub_res_right = rospy.Publisher("visualization_marker_right/result", Marker, queue_size=10)
marker_pub_workspace = rospy.Publisher("visualization_marker_workspace", Marker, queue_size=10)


if __name__ == '__main__':
    rospy.init_node('visulize_traj')
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    parser = argparse.ArgumentParser()
    parser.add_argument("--ee_type", "--end_effector_type", dest="end_effector_type", type=str, default="", help="End effector type, jodell or qiangnao.")
    args, unknown = parser.parse_known_args()
    end_effector_type = args.end_effector_type
    print(f"end effector type: {end_effector_type}")

    robot_version = 40
    if(rospy.has_param("robot_version")):
        robot_version = rospy.get_param("robot_version")

    kuavo_assests_path = get_package_path("kuavo_assets")
    model_path = kuavo_assests_path + "/models/biped_s" + str(robot_version)
    print(f"vis model path: {model_path}")

    robot_version = os.environ.get('ROBOT_VERSION', '40')
    model_config_file = kuavo_assests_path + f"/config/kuavo_v{robot_version}/kuavo.json"
    print(f"vis model config file: {model_config_file}")
    import json
    with open(model_config_file, 'r') as f:
        model_config = json.load(f)
    eef_visual_stl_files = model_config["eef_visual_stl_files"]
    quest3_arm_info_transformer = Quest3ArmInfoTransformer(model_path, eef_visual_stl_files=eef_visual_stl_files)
    # rospy.spin()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        quest3_arm_info_transformer.pub_whole_body_joint_state_msg(arm_joint_position,
                                                                finger_joint_position[0],
                                                                finger_joint_position[1],
                                                                torso_ypr, torso_height)
        marker = quest3_arm_info_transformer.construct_marker(hand_pose.l_p, hand_pose.l_q, 1.0, 0.0, 0.0, side="Left")
        marker_pub_cmd_left.publish(marker)
        marker_res = quest3_arm_info_transformer.construct_marker(hand_pose_res.l_p, hand_pose_res.l_q, 0.0, 1.0, 0.0, side="Left")
        marker_pub_res_left.publish(marker_res)
        marker = quest3_arm_info_transformer.construct_marker(hand_pose.r_p, hand_pose.r_q, 1.0, 0.0, 0.0, side="Right")
        marker_pub_cmd_right.publish(marker)
        marker_res = quest3_arm_info_transformer.construct_marker(hand_pose_res.r_p, hand_pose_res.r_q, 0.0, 1.0, 0.0, side="Right")
        marker_pub_res_right.publish(marker_res)
        # marker_workspace = quest3_arm_info_transformer.construct_point_marker(hand_pose_res.l_p, scale=0.1, alpha=0.6, color=[0.0, 1.0, 0.0])
        # marker_pub_workspace.publish(marker_workspace)
        rate.sleep()
