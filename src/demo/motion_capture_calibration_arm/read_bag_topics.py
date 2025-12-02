#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospy
from utils.analysis_utils import (
    load_keyframes_config,
    collect_keyframe_labels,
    compute_windows,
    extract_window_data,
    aggregate_window_errors,
)
from utils.load_keyframes import default_config_path

target_topics = [
    '/righthand_pose',
    '/lefthand_pose',
    '/base_pose',
    '/sensors_data_raw',
    '/mm_kuavo_arm_traj',
    '/kuavo_arm_traj',
    '/humanoid_mpc_target_arm',
    '/joint_cmd',
]

def main():
    rospy.init_node("read_bag_fk_node", anonymous=True)

    bag_folder = 'src/demo/motion_capture_calibration_arm/bag/IK_mpc_wbc_bag'
    flag_topic = '/keyframe_flag'
    offset_sec = 5.0
    window_sec = 1.0

    try:
        config_path = rospy.get_param('~config_path', default_config_path())
    except Exception:
        config_path = default_config_path()

    base_dir = "src/demo/motion_capture_calibration_arm/config"
    os.makedirs(base_dir, exist_ok=True)

    # 1. 加载关键帧配置
    keyframes = load_keyframes_config(config_path)

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
    print(f"汇总误差表生成: {summary_csv}")

if __name__ == '__main__':
    main()
