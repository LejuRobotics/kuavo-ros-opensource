#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
订阅 /arm_calibration_matrices（robotBodyMatrices），将三个相邻齐次变换矩阵分文件保存为 CSV。
每个文件只保存一个相邻变换：T_joint2_in_joint1, T_joint3_in_joint2, T_joint4_in_joint3。
"""

# ==================== 配置参数 ====================
MAX_FRAMES = 100000  # 最大保存帧数（可通过命令行参数覆盖）
OUTPUT_DIR = "matrix_data"  # CSV 保存目录
TIMEOUT_SECONDS = 5.0  # 若超时未收到数据则退出
TOPIC_MATRICES = "/arm_calibration_matrices"
# 三个相邻变换对应的文件名（不含扩展名）
FILE_NAMES = ["T_joint2_in_joint1", "T_joint3_in_joint2", "T_joint4_in_joint3"]
# ==================================================

import rospy
import csv
import os
import threading

from kuavo_msgs.msg import robotBodyMatrices


class MatricesSubscriber:
    def __init__(self, max_frames=MAX_FRAMES):
        self.max_frames = max_frames
        self.lock = threading.Lock()
        self.first_message_time = None
        # 每个文件对应一组 (timestamp, 16 个矩阵元素)
        self.data = {name: [] for name in FILE_NAMES}
        self.frame_count = 0

        self.sub = rospy.Subscriber(
            TOPIC_MATRICES,
            robotBodyMatrices,
            self.callback,
            queue_size=10,
        )
        rospy.loginfo(f"Subscribed to: {TOPIC_MATRICES}")

    def callback(self, msg):
        with self.lock:
            if self.first_message_time is None:
                self.first_message_time = rospy.Time.now()
            if self.frame_count >= self.max_frames:
                return
            if msg.num_matrices < 3:
                return
            # 提取 3 个 4x4 矩阵（每个 16 个 float，行优先）
            timestamp = msg.header.stamp.to_sec()
            for i, name in enumerate(FILE_NAMES):
                if i >= msg.num_matrices:
                    break
                start = i * 16
                end = start + 16
                row = [timestamp] + list(msg.matrices_data[start:end])
                self.data[name].append(row)
            self.frame_count += 1
            if self.frame_count % 100 == 0:
                rospy.loginfo(f"Received {self.frame_count} frames...")

    def save_to_csv(self, output_dir=OUTPUT_DIR):
        os.makedirs(output_dir, exist_ok=True)
        header = ["timestamp"] + [f"m{r}{c}" for r in range(4) for c in range(4)]
        for name in FILE_NAMES:
            filename = os.path.join(output_dir, f"{name}.csv")
            with open(filename, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(self.data[name])
            rospy.loginfo(f"Saved {len(self.data[name])} rows to {filename}")

    def get_frame_count(self):
        with self.lock:
            return self.frame_count

    def has_received_data(self):
        with self.lock:
            return self.first_message_time is not None


def main():
    import sys
    import argparse

    parser = argparse.ArgumentParser(description="Save matrices to CSV files")
    parser.add_argument("--max_frames", type=int, default=MAX_FRAMES,
                        help=f"Maximum frames to save (default: {MAX_FRAMES})")
    parser.add_argument("--output_dir", type=str, default=OUTPUT_DIR,
                        help=f"Output directory for CSV files (default: {OUTPUT_DIR})")
    
    args = parser.parse_args()

    rospy.init_node("save_matrices_to_csv", anonymous=True)

    rospy.loginfo("=" * 60)
    rospy.loginfo("Matrices CSV Saver (adjacent transforms)")
    rospy.loginfo("=" * 60)
    rospy.loginfo(f"Topic: {TOPIC_MATRICES}")
    rospy.loginfo(f"Max frames: {args.max_frames}")
    rospy.loginfo(f"Output directory: {args.output_dir}")
    rospy.loginfo(f"Output files: {', '.join(f + '.csv' for f in FILE_NAMES)}")
    rospy.loginfo("=" * 60)

    subscriber = MatricesSubscriber(max_frames=args.max_frames)
    rate = rospy.Rate(10)
    start_time = rospy.Time.now()

    rospy.loginfo("Waiting for matrix data...")
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed >= TIMEOUT_SECONDS and not subscriber.has_received_data():
            rospy.logerr(f"TIMEOUT: No data from {TOPIC_MATRICES} within {TIMEOUT_SECONDS}s")
            return
        if subscriber.get_frame_count() >= args.max_frames:
            rospy.loginfo(f"Reached {args.max_frames} frames, saving...")
            break
        rate.sleep()

    rospy.loginfo("Saving CSV files...")
    subscriber.save_to_csv(output_dir=args.output_dir)
    rospy.loginfo("Done!")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
