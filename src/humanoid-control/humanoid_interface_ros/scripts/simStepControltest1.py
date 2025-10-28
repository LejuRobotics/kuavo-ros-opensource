#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories  # 导入自定义消息类型
import numpy as np
from utils.sat import RotatingRectangle


import numpy as np

def get_test_traj_msg():
    foot_traj = [[0.0, 0.4, 0.0, 0]]
    torso_traj = [[0.0, 0.2, -0.2, 0]]
    time_traj = [0.5]# 时间序列，递增
    foot_idx_traj = [0]

    # 创建消息实例
    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = []  # 设置时间轨迹
    msg.footIndexTrajectory = []         # 设置脚索引
    msg.footPoseTrajectory = []  # 初始化脚姿态轨迹
    num = len(foot_traj)

    for i in range(num):
        # 创建脚姿态信息
        msg.timeTrajectory.append(time_traj[i])
        msg.footIndexTrajectory.append(foot_idx_traj[i])
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]      # 设置脚姿态
        foot_pose_msg.torsoPose = torso_traj[i]    # 设置躯干姿态

        # 将脚姿态添加到消息中
        msg.footPoseTrajectory.append(foot_pose_msg)
    print(msg)
    return msg


if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('foot_pose_publisher', anonymous=True)
    # 创建发布者，话题为 /humanoid_mpc_foot_pose_target_trajectories
    pub = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)

    # 等待一定时间以确保订阅者已经准备好
    rospy.sleep(1)

    msg = get_test_traj_msg()

    pub.publish(msg)
