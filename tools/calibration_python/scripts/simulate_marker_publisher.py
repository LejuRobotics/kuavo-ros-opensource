#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模拟发送seeker_markers话题的脚本

功能：
    模拟发送shoulder marker位置，用于验证transform_tag_to_base.py的计算流程
    假设base和动捕坐标系重合，验证计算出的base位置是否为[0,0,0]

使用方法:
    python3 simulate_marker_publisher.py
"""

import rospy
import numpy as np
from visualization_msgs.msg import Marker

# Shoulder marker相对位置（相对于marker1），单位：mm
SHOULDER_RELATIVE_TO_1_MM = {
    1: np.array([0.0, 0.0, 0.0]),
    2: np.array([70.0, 0.0, 60.0]),
    3: np.array([95.0, 0.0, -20.0]),
    4: np.array([70.0, -50.0, 0.0]),
}


def load_T_4x4_txt(path: str):
    """从文件加载4x4齐次变换矩阵"""
    try:
        arr = np.loadtxt(path, dtype=float)
        if arr.shape != (4, 4):
            print(f"错误：矩阵形状异常，期望(4,4)，实际{arr.shape}")
            return None
        return arr
    except Exception as e:
        print(f"错误：读取矩阵失败 {path}, {e}")
        return None


def compute_marker_positions_in_mocap():
    """
    计算marker在动捕坐标系下的位置
    
    假设：
        - base相对于动捕坐标系绕Z轴旋转90度（偏航角90度）
        - 根据base_link_to_shoulder_array.txt计算T_base_to_shoulder
        - T_mocap_to_shoulder = T_mocap_to_base @ T_base_to_shoulder
        - marker在shoulder坐标系下的相对位置（相对于marker1）
        - 转换到动捕坐标系：marker_pos_mocap = T_mocap_to_shoulder @ marker_pos_shoulder
    
    返回:
        dict: {marker_id: [x, y, z]} marker在动捕坐标系下的位置（单位：mm）
    """
    # 加载base到shoulder的变换矩阵
    config_path = '/root/kuavo_ws/tools/calibration_python/config/base_link_to_shoulder_array.txt'
    T_base_to_shoulder = load_T_4x4_txt(config_path)
    
    if T_base_to_shoulder is None:
        print("错误：无法加载base到shoulder的变换矩阵")
        return None
    
    # base相对于动捕坐标系绕Z轴旋转45度（偏航角45度）
    # 旋转矩阵：绕Z轴45度
    # [cos(45°)  -sin(45°)  0]   [√2/2  -√2/2  0]
    # [sin(45°)   cos(45°)  0] = [√2/2   √2/2  0]
    # [0          0         1]   [0      0     1]
    yaw_45_deg = np.pi / 4.0
    R_z_45 = np.array([
        [np.cos(yaw_45_deg), -np.sin(yaw_45_deg), 0],
        [np.sin(yaw_45_deg),  np.cos(yaw_45_deg), 0],
        [0,                    0,                   1]
    ])
    
    # 构建T_mocap_to_base（假设base原点在动捕坐标系下也是[0,0,0]）
    T_mocap_to_base = np.eye(4)
    T_mocap_to_base[:3, :3] = R_z_45
    T_mocap_to_base[:3, 3] = [0.0, 0.0, 0.0]  # base原点在动捕坐标系下为[0,0,0]
    
    # 计算shoulder在动捕坐标系下的变换矩阵
    # T_mocap_to_shoulder = T_mocap_to_base @ T_base_to_shoulder
    T_mocap_to_shoulder = T_mocap_to_base @ T_base_to_shoulder
    
    print("\n" + "="*60)
    print("模拟脚本调试信息（base相对动捕绕Z轴旋转45度）:")
    print("="*60)
    print(f"T_mocap_to_base旋转矩阵（绕Z轴45度）:")
    print(f"  [{T_mocap_to_base[0,0]:.6f}, {T_mocap_to_base[0,1]:.6f}, {T_mocap_to_base[0,2]:.6f}]")
    print(f"  [{T_mocap_to_base[1,0]:.6f}, {T_mocap_to_base[1,1]:.6f}, {T_mocap_to_base[1,2]:.6f}]")
    print(f"  [{T_mocap_to_base[2,0]:.6f}, {T_mocap_to_base[2,1]:.6f}, {T_mocap_to_base[2,2]:.6f}]")
    print(f"T_base_to_shoulder平移: [{T_base_to_shoulder[0,3]:.6f}, {T_base_to_shoulder[1,3]:.6f}, {T_base_to_shoulder[2,3]:.6f}] mm")
    print(f"T_mocap_to_shoulder平移: [{T_mocap_to_shoulder[0,3]:.6f}, {T_mocap_to_shoulder[1,3]:.6f}, {T_mocap_to_shoulder[2,3]:.6f}] mm")
    print("="*60 + "\n")
    
    # 计算每个marker在动捕坐标系下的位置
    marker_positions_mocap = {}
    
    for marker_id, rel_pos in SHOULDER_RELATIVE_TO_1_MM.items():
        # marker在shoulder坐标系下的位置（相对于marker1，齐次坐标）
        marker_pos_shoulder_homogeneous = np.array([rel_pos[0], rel_pos[1], rel_pos[2], 1.0])
        
        # 转换到动捕坐标系：marker_pos_mocap = T_mocap_to_shoulder @ marker_pos_shoulder
        marker_pos_mocap_homogeneous = T_mocap_to_shoulder @ marker_pos_shoulder_homogeneous
        marker_pos_mocap = marker_pos_mocap_homogeneous[:3]
        
        marker_positions_mocap[marker_id] = marker_pos_mocap
        
        print(f"Marker {marker_id}: shoulder相对位置=[{rel_pos[0]:.1f}, {rel_pos[1]:.1f}, {rel_pos[2]:.1f}] mm, "
              f"动捕位置=[{marker_pos_mocap[0]:.6f}, {marker_pos_mocap[1]:.6f}, {marker_pos_mocap[2]:.6f}] mm")
    
    print()
    
    return marker_positions_mocap


def create_marker_msg(marker_id, position, frame_id='shoulder'):
    """
    创建Marker消息
    
    参数:
        marker_id: marker编号
        position: [x, y, z] 位置（单位：mm）
        frame_id: 坐标系名称
        
    返回:
        Marker消息对象
    """
    msg = Marker()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.id = marker_id
    msg.type = Marker.SPHERE
    msg.action = Marker.ADD
    
    # 位置（单位：mm）
    msg.pose.position.x = float(position[0])
    msg.pose.position.y = float(position[1])
    msg.pose.position.z = float(position[2])
    
    # 姿态（单位矩阵，无旋转）
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    
    # 尺寸
    msg.scale.x = 10.0  # 10mm
    msg.scale.y = 10.0
    msg.scale.z = 10.0
    
    # 颜色（红色）
    msg.color.r = 1.0
    msg.color.g = 0.0
    msg.color.b = 0.0
    msg.color.a = 1.0
    
    return msg


def main():
    """主函数"""
    rospy.init_node('simulate_marker_publisher', anonymous=True)
    
    # 创建发布者
    marker_pub = rospy.Publisher('seeker_markers', Marker, queue_size=10)
    
    rospy.loginfo("开始模拟发送shoulder marker位置...")
    rospy.loginfo("假设：base相对动捕坐标系绕Z轴旋转45度（偏航角45度）")
    
    # 计算marker在动捕坐标系下的位置
    marker_positions = compute_marker_positions_in_mocap()
    
    if marker_positions is None:
        rospy.logerr("计算marker位置失败，退出")
        return
    
    # 设置发布频率（10Hz）
    rate = rospy.Rate(10.0)
    
    rospy.loginfo("开始发布marker消息...")
    
    while not rospy.is_shutdown():
        # 发布每个marker
        for marker_id, position in marker_positions.items():
            msg = create_marker_msg(marker_id, position, frame_id='shoulder')
            marker_pub.publish(msg)
        
        rate.sleep()


if __name__ == "__main__":
    main()
