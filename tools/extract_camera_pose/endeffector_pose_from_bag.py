#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) Lejurobot 2025

from kuavo_pose_calculator import KuavoPoseCalculator
from pydrake.math import RollPitchYaw
import rosbag
from geometry_msgs.msg import Pose, Point, Quaternion

def extract_pose_components(rigid_transform):
    """
    从RigidTransform中提取位置和四元数。
    
    Args:
        rigid_transform: RigidTransform对象
        
    Returns:
        dict: 包含位置和四元数的字典
    """
    # 获取位置 (x, y, z)
    position = rigid_transform.translation()
    
    # 获取旋转矩阵
    rotation_matrix = rigid_transform.rotation().matrix()
    
    # 从旋转矩阵转换为四元数 (w, x, y, z)
    # 使用RollPitchYaw来转换
    rpy = RollPitchYaw(rotation_matrix)
    quaternion = rpy.ToQuaternion()
    
    return {
        'position': position,
        'quaternion_wxyz': [quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()],
    }


def rigid_transform_to_pose_msg(rigid_transform):
    """
    将Drake的RigidTransform对象转换为geometry_msgs/Pose消息
    
    Args:
        rigid_transform: RigidTransform对象
        
    Returns:
        geometry_msgs/Pose: ROS Pose消息
    """
    pose_components = extract_pose_components(rigid_transform)
    
    pose_msg = Pose()
    
    # 设置位置
    pose_msg.position.x = pose_components['position'][0]
    pose_msg.position.y = pose_components['position'][1]
    pose_msg.position.z = pose_components['position'][2]
    
    # 设置四元数 (注意ROS使用xyzw顺序，而Drake返回的是wxyz)
    quat = pose_components['quaternion_wxyz']
    pose_msg.orientation.w = quat[0]
    pose_msg.orientation.x = quat[1]
    pose_msg.orientation.y = quat[2]
    pose_msg.orientation.z = quat[3]
    
    return pose_msg


def add_eef_pose_in_bag(bag_path, urdf_path, output_bag_path):
    """
    从rosbag中提取关节角度数据，并将末端执行器位姿写入新的bag文件
    
    Args:
        bag_path (str): 输入rosbag文件路径
        urdf_path (str): URDF文件路径
        output_bag_path (str): 输出rosbag文件路径
    
    Returns:
        dict: 包含时间戳和各相机位姿的字典
    """
    # 初始化位姿计算器
    pose_calculator = KuavoPoseCalculator(urdf_path)
    
    # 打开输入和输出rosbag
    with rosbag.Bag(bag_path, 'r') as input_bag, rosbag.Bag(output_bag_path, 'w') as output_bag:
        # 首先复制原始bag中的所有数据
        print("正在复制原始bag数据...")
        for topic, msg, t in input_bag.read_messages():
            output_bag.write(topic, msg, t)
        
        print("正在添加手部位姿数据...")
        # 重新读取原始bag，专门处理传感器数据并添加手部位姿
        for topic, msg, t in input_bag.read_messages(topics=['/sensors_data_raw']):
            # 提取关节角度数据
            joint_q = msg.joint_data.joint_q
            # 提取各部分关节角度
            left_arm_q = joint_q[12:19]  # 左手关节 (索引12-18)
            right_arm_q = joint_q[19:26]  # 右手关节 (索引19-25)
            
            try:
                # 计算各相机位姿
                left_eef_pose = pose_calculator.get_l_hand_camera_or_eef_pose("zarm_l7_end_effector", left_arm_q)
                right_eef_pose = pose_calculator.get_r_hand_camera_or_eef_pose("zarm_r7_end_effector", right_arm_q)
                
                # 将末端执行器位姿转换为ROS Pose消息
                left_eef_pose_msg = rigid_transform_to_pose_msg(left_eef_pose)
                right_eef_pose_msg = rigid_transform_to_pose_msg(right_eef_pose)
                
                # 写入末端执行器位姿到输出bag
                output_bag.write('/l_eef_pose', left_eef_pose_msg, t)
                output_bag.write('/r_eef_pose', right_eef_pose_msg, t)
                
            except ValueError as e:
                print(f"跳过时间戳 {t.to_sec()} 的数据，原因: {e}")
                continue
    

if __name__ == "__main__":
    # 使用示例
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    bag_path = os.path.join(script_dir, "SMCZRK_16_001_P4-203_20250808_084010.bag")
    urdf_path = os.path.join(script_dir, "../../src/kuavo_assets/models/biped_s49/urdf/biped_s49.urdf")
    output_bag_path = os.path.join(script_dir, "eef_pose.bag")
    
    add_eef_pose_in_bag(bag_path, urdf_path, output_bag_path)
    
    print(f"末端执行器位姿数据已写入新的bag文件: {output_bag_path}")

