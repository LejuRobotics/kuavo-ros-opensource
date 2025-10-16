#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../scripts')))

import rospy
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories  # 导入自定义消息类型
from robot_tool import KuavoRobotTools
from tf.transformations import euler_from_quaternion
import numpy as np

def get_delta_com_z(step_length=0.4):
    if step_length <= 0.25:
        delta_com_z = 0.0
    elif 0.25 < step_length < 0.6:
        delta_com_z = (step_length - 0.2) * (0.1 - step_length)
    else:
        delta_com_z = -0.15
    return delta_com_z

def get_foot_pose():
    robot_tools = KuavoRobotTools()
    current_left_foot = robot_tools.get_tf_transform(
                    target_frame="base_link",
                    source_frame="leg_l6_link",
                    return_type="pose_quaternion"
                )
    current_right_foot = robot_tools.get_tf_transform(
        target_frame="base_link",
        source_frame="leg_r6_link",
        return_type="pose_quaternion"
    )
    _, _, yaw_l6 = euler_from_quaternion(current_left_foot.orientation)
    _, _, yaw_r6 = euler_from_quaternion(current_right_foot.orientation)

    current_left_foot = np.array([*current_left_foot.position, yaw_l6])
    current_right_foot = np.array([*current_right_foot.position, yaw_r6])

    return current_left_foot, current_right_foot

def get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj):
    """Get foot pose trajectory message."""
    num = len(time_traj)
    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = time_traj
    msg.footIndexTrajectory = [int(idx) for idx in foot_idx_traj]
    msg.footPoseTrajectory = []
    
    for i in range(num):
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]
        foot_pose_msg.torsoPose = torso_traj[i]

        msg.footPoseTrajectory.append(foot_pose_msg)

    return msg

def get_single_foot_msg(side, movement):
    time_traj = [0.4]
    foot_idx_traj = [side]
    foot_traj = []
    torso_traj = []
    current_left_foot, current_right_foot = get_foot_pose()
    if side == 0:
        current_swing_foot = current_left_foot.copy()
        current_support_foot = current_right_foot.copy()
    else:
        current_swing_foot = current_right_foot.copy()
        current_support_foot = current_left_foot.copy()

    print(f"current_swing_foot{current_swing_foot}")
    print(f"current_support_foot{current_support_foot}")
    movement_pose = np.asarray(movement)
    target_pose = current_swing_foot.copy() + movement_pose.copy()

    step_length_before = np.linalg.norm(current_swing_foot[:2] - current_support_foot[:2])
    step_length_after = np.linalg.norm(target_pose[:2] - current_support_foot[:2])
    torso_pose_before = (current_swing_foot.copy() + current_support_foot.copy()) / 2
    torso_pose_after = (target_pose.copy() + current_support_foot.copy()) / 2

    torso_height_before = get_delta_com_z(step_length_before)
    torso_height_after = get_delta_com_z(step_length_after)
    torso_height_offset = torso_height_after - torso_height_before
    torso_pose = torso_pose_after.copy() - torso_pose_before.copy()
    torso_pose[2] = torso_height_offset
    target_pose[2] = 0
    print(f"target_pose{target_pose}")
    foot_traj.append(target_pose)
    torso_traj.append(torso_pose)
    
    return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)

def get_max_step_msg(max_step_length):
    time_traj = [0.4, 0.8, 1.2, 1.6, 2.0, 2.4, 2.8, 3.2]  # 时间轨迹
    foot_idx_traj = [0, 0, 0, 0, 1, 1, 1, 1]  # 脚索引轨迹
    msg = footPoseTargetTrajectories()  # 创建消息实例
    msg.timeTrajectory = time_traj   # 设置时间轨迹
    msg.footIndexTrajectory = foot_idx_traj   # 设置脚索引
    msg.footPoseTrajectory = []         # 初始化脚姿态轨迹
    
    # ============== ACTIONS =============
    #   ↑_, ↓_, ←_, →_, _↑, _↓, _→, _←
    # ====================================
    delta_com_z = get_delta_com_z(max_step_length)
    # 设置脚姿态
    foot_traj = [[max_step_length, 0.1, 0.0, 0.0],             # l
                [0.0, 0.1, 0.0, 0.0],            # l
                [0.0, 0.1 + max_step_length, 0.0, 0.0],       # l
                [0.0, 0.1, 0.0, 0.0],       # l
                [max_step_length, -0.1, 0.0, 0.0],            # r
                [0.0, -0.1, 0.0, 0.0],           # r
                [0.0, -0.1 - max_step_length, 0.0, 0.0],       # r
                [0.0, -0.1, 0.0, 0.0],       # r
                ]
    
    # 设置躯干姿态
    torso_traj = [[max_step_length / 2, 0.0, delta_com_z, 0.0],    # l
                [0.0, 0.0, 0.0, 0.0],   # l
                [0.0, max_step_length / 2, delta_com_z, 0.0],    # l
                [0.0, 0.0, 0.0, 0.0],   # l
                [max_step_length / 2, 0.0, delta_com_z, 0.0],    # r
                [0.0, 0.0, 0.0, 0.0],   # r
                [0.0, -max_step_length / 2, delta_com_z, 0.0],    # r
                [0.0, 0.0, 0.0, 0.0],   # r
                ]  
    
    num = len(time_traj)
    for i in range(num):
        # 创建脚姿态信息
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]      # 设置脚姿态
        foot_pose_msg.torsoPose = torso_traj[i]    # 设置躯干姿态
        # 将脚姿态添加到消息中
        msg.footPoseTrajectory.append(foot_pose_msg)

    return msg

def test_max_step_length(pub, max_step_length):
    msg = get_max_step_msg(max_step_length)
    pub.publish(msg)

def test_workspace(pub):
    # ============== ACTIONS =============
    #       ↖_, ↘_, ↓_, ↑_, ←_， →_, 
    #       _↖, _↘, _↓, _↑, _→， _←
    # ====================================
    movements = [[0.4, 0.1, 0.0, 0.5],
                 [-0.4, 0.05, 0.0, -0.5],
                 [-0.2, 0.0, 0.0, 0.0],
                 [0.2, 0.0, 0.0, 0.0],
                 [0.0, 0.3, 0.0, 0.0],
                 [0.0, -0.2, 0.0, 0.0],
                 [0.4, -0.1, 0.0, -0.5],
                 [-0.4, 0.0, 0.0, 0.5],
                 [-0.2, 0.0, 0.0, 0.0],
                 [0.2, 0.0, 0.0, 0.0],
                 [0.0, -0.3, 0.0, 0.0],
                 [0.0, 0.2, 0.0, 0.0]
                ]
    num = len(movements)
    for i in range(num):
        side = 0 if i < (num / 2) else 1
        msg = get_single_foot_msg(side, movements[i])
        pub.publish(msg)
        print("=============================================")
        print(f"msg{msg}")
        print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        rospy.sleep(5)
        
    
if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('foot_pose_publisher', anonymous=True)
    # 创建发布者，话题为 /humanoid_mpc_foot_pose_target_trajectories
    pub = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)

    # 等待一定时间以确保订阅者已经准备好
    rospy.sleep(1)

    # 测试最大步长
    # test_max_step_length(pub, 0.35)

    # 测试工作空间
    test_workspace(pub)
