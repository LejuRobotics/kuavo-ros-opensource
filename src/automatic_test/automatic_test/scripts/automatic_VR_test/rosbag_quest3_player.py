#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rosbag
import sys
import os
import subprocess
from noitom_hi5_hand_udp_python.msg import PoseInfoList, JoySticks
import tf
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import time
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
class Quest3RosbagPlayer:
    def __init__(self):
        rospy.init_node('rosbag_quest3_player', anonymous=True)
        
        # 从命令行参数获取rosbag路径
        if len(sys.argv) < 2:
            rospy.logerr("No bag file path provided. Please provide the path as a command line argument.")
            sys.exit(1)
            
        self.bag_path = sys.argv[1]
        if not os.path.exists(self.bag_path):
            rospy.logerr(f"Bag file not found: {self.bag_path}")
            sys.exit(1)
        
        # 初始化发布器
        self.pose_pub = rospy.Publisher('/leju_quest_bone_poses', PoseInfoList, queue_size=10)
        self.joysticks_pub = rospy.Publisher('/quest_joystick_data', JoySticks, queue_size=10)
        self.hand_finger_tf_pub = rospy.Publisher('/quest_hand_finger_tf', TFMessage, queue_size=10)
        self.br = tf.TransformBroadcaster()
        
        # 骨骼名称列表
        self.bone_names = [
            "LeftArmUpper", "LeftArmLower", "RightArmUpper", "RightArmLower",
            "LeftHandPalm", "RightHandPalm", "LeftHandThumbMetacarpal",
            "LeftHandThumbProximal", "LeftHandThumbDistal", "LeftHandThumbTip",
            "LeftHandIndexTip", "LeftHandMiddleTip", "LeftHandRingTip",
            "LeftHandLittleTip", "RightHandThumbMetacarpal", "RightHandThumbProximal",
            "RightHandThumbDistal", "RightHandThumbTip", "RightHandIndexTip",
            "RightHandMiddleTip", "RightHandRingTip", "RightHandLittleTip",
            "Root", "Chest", "Neck", "Head"
        ]
        
        self.bone_name_to_index = {name: index for index, name in enumerate(self.bone_names)}
        self.index_to_bone_name = {index: name for index, name in enumerate(self.bone_names)}
        
        # 设置发布频率
        self.rate = rospy.Rate(100.0)  # 100Hz
        
        # 设置is_running标志
        self.is_running = True
        self.bag = None

    def convert_position_to_right_hand(self, left_hand_position):
        return {
            "x": 0 - left_hand_position["z"],
            "y": 0 - left_hand_position["x"],
            "z": left_hand_position["y"]
        }

    def convert_quaternion_to_right_hand(self, left_hand_quat):
        return (
            0 - left_hand_quat[2],
            0 - left_hand_quat[0],
            left_hand_quat[1],
            left_hand_quat[3]
        )

    def updateAFrame(self, frame_name, frame_position, frame_rotation_quat, time_now):
        self.br.sendTransform((frame_position["x"], frame_position["y"], frame_position["z"]), 
                              frame_rotation_quat, time_now, frame_name, "torso")

    def play_bag(self):
        try:

            rospy.loginfo("初始化bag文件播放完成")
            
            while not rospy.is_shutdown():
                # 使用rosbag play命令直接播放bag文件
                rospy.loginfo("开始直接播放bag文件...")
                
                
                try:
                    # 启动rosbag play进程，添加-q参数以消除打印信息
                    play_process = subprocess.Popen(["rosbag", "play", "-q", self.bag_path])

                    # 等待rosbag播放完成
                    play_process.wait()
                    
                    rospy.loginfo("bag文件播放完成")
                    rospy.sleep(5)
                except Exception as e:
                    rospy.logerr(f"播放bag文件时出错: {e}")
                    break
                
                if rospy.is_shutdown():
                    break
                break
        except Exception as e:
            rospy.logerr(f"Error playing bag file: {e}")
        finally:
            if self.bag is not None:
                self.bag.close()

if __name__ == "__main__":
    player = Quest3RosbagPlayer()
    player.play_bag()