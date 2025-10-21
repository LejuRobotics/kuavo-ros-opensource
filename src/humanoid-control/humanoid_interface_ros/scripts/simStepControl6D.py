#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories, footPose6D, footPose6DTargetTrajectories  # 导入自定义消息类型
import numpy as np
from utils.sat import RotatingRectangle
import numpy as np


def get_foot_pose_6d_traj_msg(time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_height_traj=None):
    """
    创建6D足部姿态目标轨迹消息
    
    Args:
        time_traj: 时间轨迹列表
        foot_idx_traj: 足部索引轨迹列表
        foot_traj_6d: 6D足部姿态轨迹列表，每个元素为[x, y, z, yaw, pitch, roll]
        torso_traj_6d: 6D躯干姿态轨迹列表，每个元素为[x, y, z, yaw, pitch, roll]
        swing_height_traj: 摆动高度轨迹列表（可选）
    
    Returns:
        footPose6DTargetTrajectories消息
    """
    num = len(time_traj)

    # 创建消息实例
    msg = footPose6DTargetTrajectories()
    msg.timeTrajectory = time_traj  # 设置时间轨迹
    msg.footIndexTrajectory = foot_idx_traj  # 设置脚索引
    msg.footPoseTrajectory = []  # 初始化脚姿态轨迹

    for i in range(num):
        # 创建6D脚姿态信息
        foot_pose_msg = footPose6D()
        foot_pose_msg.footPose6D = foot_traj_6d[i]  # 设置6D脚姿态 [x, y, z, yaw, pitch, roll]
        foot_pose_msg.torsoPose6D = torso_traj_6d[i]  # 设置6D躯干姿态 [x, y, z, yaw, pitch, roll]

        # 将脚姿态添加到消息中
        msg.footPoseTrajectory.append(foot_pose_msg)

    # 设置摆动高度轨迹（如果提供）
    if swing_height_traj is not None:
        msg.swingHeightTrajectory = swing_height_traj
    else:
        # 默认摆动高度
        msg.swingHeightTrajectory = [0.06] * num

    return msg


def create_simple_6d_trajectory():
    """
    创建简单的6D轨迹消息
    
    Args:
        foot_poses_6d: 足部6D姿态列表，每个元素为[x, y, z, yaw, pitch, roll]
        torso_poses_6d: 躯干6D姿态列表，每个元素为[x, y, z, yaw, pitch, roll]
        time_interval: 时间间隔
    
    Returns:
        footPose6DTargetTrajectories消息
    """
    time_traj = [0.5]
    foot_idx_traj = [0]  # 默认左脚
    foot_poses_6d = [[0.3, 0.1, 0.0, 0.0, -0.5, 0.0]]
    torso_poses_6d = [[0.15, 0.0, 0.0, 0.0, 0.1, 0.0]]
    
    return get_foot_pose_6d_traj_msg(time_traj, foot_idx_traj, foot_poses_6d, torso_poses_6d)


if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('foot_pose_publisher', anonymous=True)
    # 创建发布者，话题为 /humanoid_mpc_foot_pose_6d_target_trajectories (6D)
    pub_6d = rospy.Publisher('/humanoid_mpc_foot_pose_6d_target_trajectories', footPose6DTargetTrajectories, queue_size=10)

    # 等待一定时间以确保订阅者已经准备好
    rospy.sleep(1)

    # 直接填入6D姿态点进行测试
    # 格式：[x(m), y(m), z(m), yaw(rad), pitch(rad), roll(rad)]
    
    # 测试1：单个6D姿态点
    print("=== 测试1：单个6D姿态点 ===")
    
    msg_6d_single = create_simple_6d_trajectory()
    pub_6d.publish(msg_6d_single)
    print("单个6D姿态点已发布")
    
    rospy.sleep(2)
    
