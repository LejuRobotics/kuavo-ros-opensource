#!/usr/bin/env python3
# coding: utf-8

import argparse
import subprocess
import sys
import os
import time
from common.logger import SDKLogger
import rospy
from kuavo_msgs.srv import changeArmCtrlMode

# 需要录制/回放的典型话题
RECORD_TOPICS = [
    "/ik/two_arm_hand_pose_cmd",
    "/mm/two_arm_hand_pose_cmd",
    "/humanoid_mpc_foot_pose_target_trajectories"
]

def record_bag(bag_path):
    cmd = ["rosbag", "record", "-O", bag_path] + RECORD_TOPICS
    SDKLogger.info(f"Recording bag to {bag_path} with topics: {RECORD_TOPICS}")
    SDKLogger.info(f"Command: {' '.join(cmd)}")
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        SDKLogger.info("Recording interrupted by user.")
    except Exception as e:
        SDKLogger.error(f"Error during recording: {e}")

def call_ctrl_services():
    try:
        rospy.wait_for_service("/change_arm_ctrl_mode", timeout=5)
        proxy = rospy.ServiceProxy("/change_arm_ctrl_mode", changeArmCtrlMode)
        proxy(1)
        SDKLogger.info("Called service /change_arm_ctrl_mode with arg 1.")
    except Exception as e:
        SDKLogger.error(f"Failed to call service /change_arm_ctrl_mode: {e}")

    # 2. /mobile_manipulator_mpc_control
    try:
        rospy.wait_for_service("/mobile_manipulator_mpc_control", timeout=5)
        proxy = rospy.ServiceProxy("/mobile_manipulator_mpc_control", changeArmCtrlMode)
        proxy(1)
        SDKLogger.info("Called service /mobile_manipulator_mpc_control with arg 1.")
    except Exception as e:
        SDKLogger.error(f"Failed to call service /mobile_manipulator_mpc_control: {e}")

    # 3. /enable_mm_wbc_arm_trajectory_control
    try:
        rospy.wait_for_service("/enable_mm_wbc_arm_trajectory_control", timeout=5)
        proxy = rospy.ServiceProxy("/enable_mm_wbc_arm_trajectory_control", changeArmCtrlMode)
        proxy(1)
        SDKLogger.info("Called service /enable_mm_wbc_arm_trajectory_control with arg 1.")
    except Exception as e:
        SDKLogger.error(f"Failed to call service /enable_mm_wbc_arm_trajectory_control: {e}")

def play_bag(bag_path, loop=False):
    rospy.init_node("robot_pico_recorder_play", anonymous=True)
    call_ctrl_services()
    if not os.path.exists(bag_path):
        SDKLogger.error(f"Bag file {bag_path} does not exist!")
        sys.exit(1)
    cmd = ["rosbag", "play", bag_path]
    SDKLogger.info(f"Playing bag {bag_path} {'in loop' if loop else ''}")
    SDKLogger.info(f"Command: {' '.join(cmd)}")
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        SDKLogger.info("Playback interrupted by user.")
    except Exception as e:
        SDKLogger.error(f"Error during playback: {e}")

def main():
    parser = argparse.ArgumentParser(description="Pico手臂/腿控bag录制与回放工具")
    subparsers = parser.add_subparsers(dest="mode", help="模式: record 或 play")

    # 录制子命令
    parser_record = subparsers.add_parser("record", help="录制bag")
    parser_record.add_argument("bag", type=str, help="保存的bag文件名")

    # 回放子命令
    parser_play = subparsers.add_parser("play", help="回放bag")
    parser_play.add_argument("bag", type=str, help="要回放的bag文件名")

    args = parser.parse_args()

    if args.mode == "record":
        record_bag(args.bag)
    elif args.mode == "play":
        play_bag(args.bag)
    else:
        parser.print_help()
        sys.exit(1)

if __name__ == "__main__":
    main()
