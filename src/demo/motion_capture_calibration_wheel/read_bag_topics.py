#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospy
from utils.load_keyframes import load_keyframes
from utils.analysis_utils import (
    collect_keyframe_labels,
    compute_windows,
    extract_window_data,
    aggregate_window_errors,
)

target_topics = [
    '/r_hand_pose',
    '/l_hand_pose',
    '/car_pose',
    '/waist_pose',
    '/r_shoulder_pose',
    '/l_shoulder_pose', 
    '/head_pose',
    '/joint_cmd',
    '/sensors_data_raw',
]

def main():
    rospy.init_node("read_bag_extended_node", anonymous=True)

    bag_folder = 'src/demo/motion_capture_calibration_wheel/bag'
    flag_topic = '/keyframe_flag'
    offset_sec = 5.0
    window_sec = 1.0

    base_dir = "src/demo/motion_capture_calibration_wheel/output"
    os.makedirs(base_dir, exist_ok=True)

    # 1. 加载关键帧配置
    keyframes = load_keyframes()

    # 2. 收集关键帧标签
    bag_infos, labels = collect_keyframe_labels(bag_folder, flag_topic, keyframes)
    print(f"检测到标签数量: {len(labels)}")
    if not labels:
        print("无标签，退出。")
        return

    # 3. 计算窗口
    windows = compute_windows(bag_infos, labels, offset_sec, window_sec)
    print(f"有效窗口数量: {len(windows)}")
    if not windows:
        print("窗口为空，退出。")
        return
    for w in windows:
        print(f"窗口[{w['index']}]: bag={os.path.basename(w['bag_path'])} [{w['bag_start']:.3f},{w['bag_end']:.3f}]")

    # 4. 提取窗口数据
    results, refs, kf_indices = extract_window_data(windows, target_topics, keyframes)
    print(f"提取完成窗口数: {len(results)}")

    # 5. 分析聚合并保存结果
    summary_csv = aggregate_window_errors(results, refs, kf_indices, keyframes, base_dir)
    print(f"原始数据记录已生成: {summary_csv}")

if __name__ == '__main__':
    main()

