#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) Lejurobot 2025

from  kuavo_pose_calculator import KuavoPoseCalculator
import rosbag

def extract_camera_poses_from_bag(bag_path, urdf_path):
    """
    从rosbag中提取相机位姿数据
    
    Args:
        bag_path (str): rosbag文件路径
        urdf_path (str): URDF文件路径
    
    Returns:
        dict: 包含时间戳和各相机位姿的字典
    """
    # 初始化位姿计算器
    pose_calculator = KuavoPoseCalculator(urdf_path)
    
    # 存储结果
    camera_poses = {
        'timestamps': [],
        'head_camera_poses': [],
        'left_hand_camera_poses': [],
        'right_hand_camera_poses': []
    }
    
    # 打开rosbag
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/sensors_data_raw']):
            # 提取关节角度数据
            joint_q = msg.joint_data.joint_q
            # 提取各部分关节角度
            left_arm_q = joint_q[12:19]  # 左手关节 (索引12-18)
            right_arm_q = joint_q[19:26]  # 右手关节 (索引19-25)
            head_q = joint_q[-2:]  # 头部关节 (最后两个)
            
            try:
                # 计算各相机位姿
                head_camera_pose = pose_calculator.get_camera_pose(head_q)
                left_hand_camera_pose = pose_calculator.get_l_hand_camera_pose(left_arm_q)
                right_hand_camera_pose = pose_calculator.get_r_hand_camera_pose(right_arm_q)
                
                # Debug
                # print("joint_q:", joint_q)
                # print("left_arm_q:", left_arm_q)
                # print("right_arm_q:", right_arm_q)
                # print("head_q:", head_q)
                # print("head_camera_pose:", head_camera_pose)
                # print("left_hand_camera_pose:", left_hand_camera_pose)
                # print("right_hand_camera_pose:", right_hand_camera_pose)
                # return 

                # 存储结果
                camera_poses['timestamps'].append(t.to_sec())
                camera_poses['head_camera_poses'].append(head_camera_pose)
                camera_poses['left_hand_camera_poses'].append(left_hand_camera_pose)
                camera_poses['right_hand_camera_poses'].append(right_hand_camera_pose)
                
            except ValueError as e:
                print(f"跳过时间戳 {t.to_sec()} 的数据，原因: {e}")
                continue
    
    return camera_poses

if __name__ == "__main__":
    # 使用示例
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    bag_path = os.path.join(script_dir, "222.bag")
    urdf_path = os.path.join(script_dir, "../../src/kuavo_assets/models/biped_s45/urdf/biped_s45.urdf")
    
    camera_poses = extract_camera_poses_from_bag(bag_path, urdf_path)
    
    print(f"提取了 {len(camera_poses['timestamps'])} 个时间点的相机位姿数据")
    print(f"时间范围: {camera_poses['timestamps'][0]:.3f} - {camera_poses['timestamps'][-1]:.3f} 秒")

    # 将相机位姿数据保存为JSON文件
    import json
    import numpy as np
    # 转换RigidTransform对象为可序列化的格式
    def convert_rigid_transform_to_dict(rigid_transform):
        """将Drake的RigidTransform对象转换为字典格式"""
        rotation_matrix = rigid_transform.rotation().matrix()
        translation = rigid_transform.translation()
        
        return {
            'rotation_matrix': rotation_matrix.tolist(),
            'translation': translation.tolist()
        }
    
    # 转换所有相机位姿数据
    json_data = {
        'timestamps': camera_poses['timestamps'],
        'head_camera_poses': [convert_rigid_transform_to_dict(pose) for pose in camera_poses['head_camera_poses']],
        'left_hand_camera_poses': [convert_rigid_transform_to_dict(pose) for pose in camera_poses['left_hand_camera_poses']],
        'right_hand_camera_poses': [convert_rigid_transform_to_dict(pose) for pose in camera_poses['right_hand_camera_poses']]
    }
    
    # 保存到JSON文件
    output_file = "camera_poses.json"
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(json_data, f, indent=2, ensure_ascii=False)
    
    print(f"相机位姿数据已保存到: {output_file}")