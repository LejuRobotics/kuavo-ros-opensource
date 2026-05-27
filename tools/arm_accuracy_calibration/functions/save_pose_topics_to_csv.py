#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
订阅4个pose话题并保存为CSV文件
话题名: shoulder_pose, belly_pose, effector_pose, elbow_pose
"""

# ==================== 配置参数 ====================
MAX_FRAMES = 100000  # 最大保存帧数（可通过命令行参数覆盖）
OUTPUT_DIR = "pose_data"  # CSV文件保存目录
TIMEOUT_SECONDS = 5.0  # 如果5秒内没有收到任何数据，退出
# ==================================================

import rospy
import csv
import os
from datetime import datetime
from geometry_msgs.msg import PoseStamped
import threading
import queue

class PoseSubscriber:
    def __init__(self, max_frames=MAX_FRAMES):
        self.max_frames = max_frames
        self.lock = threading.Lock()
        self.first_message_time = None  # 记录第一条消息的时间
        
        # 话题名称
        self.topic_names = {
            'shoulder': 'shoulder_pose',
            'belly': 'belly_pose',
            'effector': 'effector_pose',
            'elbow': 'elbow_pose',
            'calimark': '/calimark_mocap_shoulder'
        }
        
        # 数据存储和每个话题的计数
        self.data = {name: [] for name in self.topic_names.keys()}
        self.frame_count = {name: 0 for name in self.topic_names.keys()}  # 每个话题分别计数
        
        # 创建订阅者（订阅 PoseStamped，使用消息自带时间戳）
        self.subscribers = {}
        for name, topic in self.topic_names.items():
            self.subscribers[name] = rospy.Subscriber(
                topic, 
                PoseStamped, 
                lambda msg, n=name: self.pose_callback(msg, n)
            )
            rospy.loginfo(f"Subscribed to: {topic}")
    
    def pose_callback(self, msg, name):
        with self.lock:
            if self.first_message_time is None:
                self.first_message_time = rospy.Time.now()
            
            # 检查该话题是否已达到最大帧数
            if self.frame_count[name] >= self.max_frames:
                return
            
            # 使用 PoseStamped 中的时间戳和位姿
            timestamp = msg.header.stamp.to_sec()
            position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            orientation = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
            
            self.data[name].append({
                'timestamp': timestamp,
                'position': position,
                'orientation': orientation
            })
            
            self.frame_count[name] += 1
            
            # 每100条消息打印一次日志（统计所有话题的总数）
            total_frames = sum(self.frame_count.values())
            if total_frames % 100 == 0:
                rospy.loginfo(f"Received {total_frames} total frames (shoulder:{self.frame_count['shoulder']}, belly:{self.frame_count['belly']}, effector:{self.frame_count['effector']}, elbow:{self.frame_count['elbow']})...")
    
    def save_to_csv(self, output_dir=OUTPUT_DIR):
        os.makedirs(output_dir, exist_ok=True)
        
        for name in self.topic_names.keys():
            filename = os.path.join(output_dir, f"{name}_pose.csv")
            
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # 写入表头
                writer.writerow([
                    'timestamp', 
                    'pos_x', 'pos_y', 'pos_z',
                    'ori_x', 'ori_y', 'ori_z', 'ori_w'
                ])
                
                # 写入数据
                for row in self.data[name]:
                    writer.writerow([
                        row['timestamp'],
                        row['position'][0], row['position'][1], row['position'][2],
                        row['orientation'][0], row['orientation'][1], 
                        row['orientation'][2], row['orientation'][3]
                    ])
            
            rospy.loginfo(f"Saved {len(self.data[name])} frames to {filename}")
    
    def get_frame_count(self):
        """返回所有话题的总帧数"""
        with self.lock:
            return sum(self.frame_count.values())
    
    def get_min_frame_count(self):
        """返回所有话题中的最小帧数（用于判断是否所有话题都达到max_frames）"""
        with self.lock:
            return min(self.frame_count.values()) if self.frame_count else 0
    
    def has_received_data(self):
        with self.lock:
            return self.first_message_time is not None
    
    def get_first_message_time(self):
        with self.lock:
            return self.first_message_time

def main():
    import sys
    
    # 参数设置（命令行参数可覆盖配置）
    max_frames = MAX_FRAMES
    if len(sys.argv) > 1:
        try:
            max_frames = int(sys.argv[1])
        except ValueError:
            rospy.logerr(f"Invalid max_frames value: {sys.argv[1]}, using default {MAX_FRAMES}")
    
    rospy.init_node('save_pose_topics_to_csv', anonymous=True)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("Pose Topics CSV Saver")
    rospy.loginfo("=" * 60)
    rospy.loginfo(f"Max frames: {max_frames}")
    rospy.loginfo(f"Topics: shoulder_pose, belly_pose, effector_pose, elbow_pose")
    rospy.loginfo("=" * 60)
    
    subscriber = PoseSubscriber(max_frames=max_frames)
    
    # 等待第一条消息，检查超时
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)  # 10Hz
    
    rospy.loginfo("Waiting for pose data...")
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        
        # 检查超时
        if elapsed >= TIMEOUT_SECONDS and not subscriber.has_received_data():
            rospy.logerr("=" * 60)
            rospy.logerr(f"TIMEOUT: No data received within {TIMEOUT_SECONDS} seconds!")
            rospy.logerr("Please check if the following topics are publishing:")
            for topic in subscriber.topic_names.values():
                rospy.logerr(f"  - {topic}")
            rospy.logerr("=" * 60)
            return
        
        # 检查是否所有话题都达到最大帧数
        min_frame_count = subscriber.get_min_frame_count()
        if min_frame_count >= max_frames:
            rospy.loginfo(f"All topics reached max frames ({max_frames}), saving data...")
            break
        
        rate.sleep()
    
    # 保存数据
    rospy.loginfo("Saving data to CSV files...")
    subscriber.save_to_csv()
    
    rospy.loginfo("Done!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
