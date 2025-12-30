#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories  # 导入自定义消息类型
import numpy as np
from utils.sat import RotatingRectangle


import numpy as np

def euler_to_rotation_matrix(yaw, pitch, roll):
    # 计算各轴的旋转矩阵
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])

    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])

    # 按照 Yaw-Pitch-Roll 的顺序组合旋转矩阵
    R = np.dot(R_roll, np.dot(R_pitch, R_yaw))
    return R


def get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj):
    num = len(time_traj)

    # 创建消息实例
    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = time_traj  # 设置时间轨迹
    msg.footIndexTrajectory = foot_idx_traj         # 设置脚索引
    msg.footPoseTrajectory = []  # 初始化脚姿态轨迹

    for i in range(num):
        # 创建脚姿态信息
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]      # 设置脚姿态
        foot_pose_msg.torsoPose = torso_traj[i]    # 设置躯干姿态

        # 将脚姿态添加到消息中
        msg.footPoseTrajectory.append(foot_pose_msg)

    return msg

def generate_steps(torso_pos, torso_yaw, foot_bias):
    l_foot_bias = np.array([0, foot_bias, -torso_pos[2]])
    r_foot_bias = np.array([0, -foot_bias, -torso_pos[2]])
    R_z = np.array([
        [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
        [np.sin(torso_yaw), np.cos(torso_yaw), 0],
        [0, 0, 1]
    ])
    l_foot = torso_pos + R_z.dot(l_foot_bias)
    r_foot = torso_pos + R_z.dot(r_foot_bias)
    return l_foot, r_foot   

def get_multiple_steps_msg(body_poses, dt, is_left_first=True, collision_check=True):
    num_steps = 2*len(body_poses)
    time_traj = []
    foot_idx_traj = []
    foot_traj = []
    torso_traj = []
    l_foot_rect_last = RotatingRectangle(center=(0, 0.1), width=0.24, height=0.1, angle=0)
    r_foot_rect_last = RotatingRectangle(center=(0,-0.1), width=0.24, height=0.1, angle=0)
    torso_yaw_last = 0.0
    torso_pose_last = np.array([0, 0, 0, 0])
    for i in range(num_steps):
        time_traj.append(dt * (i+1))
        body_pose = body_poses[i//2]
        torso_pos = np.asarray(body_pose[:3])
        torso_yaw = np.radians(body_pose[3])
        # body_pose[3] = torso_yaw    
        l_foot, r_foot = generate_steps(torso_pos, torso_yaw, 0.1)
        l_foot = [*l_foot[:3], torso_yaw]
        r_foot = [*r_foot[:3], torso_yaw]

        if(i%2 == 0):        
            torso_pose = np.array([*body_pose[:3], torso_yaw])
            R_wl = euler_to_rotation_matrix(torso_pose_last[3], 0, 0)
            delta_pos = R_wl.T @ (torso_pose[:3] - torso_pose_last[:3])
            print("delta_pos:", delta_pos)
            if(torso_yaw > 0.0 or delta_pos[1] > 0.0):
                is_left_first = True
            else:
                is_left_first = False

        if(collision_check and i%2 == 0):
            l_foot_rect_next = RotatingRectangle(center=(l_foot[0],l_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
            r_foot_rect_next = RotatingRectangle(center=(r_foot[0],r_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
            l_collision = l_foot_rect_next.is_collision(r_foot_rect_last)
            r_collision = r_foot_rect_next.is_collision(l_foot_rect_last)
            if l_collision and r_collision:
                print("\033[91m[Error] Detect collision, Please adjust your body_poses input!!!\033[0m")
                break
            elif l_collision:
                print("\033[92m[Info] Left foot is in collision, switch to right foot\033[0m")
                is_left_first = False
            elif r_collision:
                print("\033[92m[Info] Right foot is in collision, switch to left foot\033[0m")
                is_left_first = True
            l_foot_rect_last = l_foot_rect_next
            r_foot_rect_last = r_foot_rect_next
        if(i%2 == 0):
            torso_traj.append((torso_pose_last + torso_pose)/2.0)
            if is_left_first:
                foot_idx_traj.append(0)
                foot_traj.append(l_foot)
            else:
                foot_idx_traj.append(1)
                foot_traj.append(r_foot)
        else:
            torso_traj.append(torso_pose)
            if is_left_first:
                foot_idx_traj.append(1)
                foot_traj.append(r_foot)
            else:
                foot_idx_traj.append(0)
                foot_traj.append(l_foot)
        # torso_traj.append([*body_pose[:3], torso_yaw])
        torso_pose_last = torso_traj[-1]
        torso_yaw_last = torso_yaw
    print("time_traj:", time_traj)
    print("foot_idx_traj:", foot_idx_traj)
    print("foot_traj:", foot_traj)
    print("torso_traj:", torso_traj)
    return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)


if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('foot_pose_publisher', anonymous=True)
    # 创建发布者，话题为 /humanoid_mpc_foot_pose_target_trajectories
    pub = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)

    # 等待一定时间以确保订阅者已经准备好
    rospy.sleep(1)

    is_left_first_default = True # 缺省左脚先行
    # 缺省开启碰撞检测，如果默认规划的步态顺序会导致碰撞，则会自动切换到另一侧的步态,如果设置为False则不会切换步态顺序
    # 注意：碰撞检测开启后，并且可能导致规划失败
    collision_check = True 
    # body_poses基于局部坐标系给定，每一个身体姿态对应两步到达
    dt = 0.4 #迈一步的时间间隔，腾空相和支撑相时间占比各dt/2
    # 一次完整的步态Mode序列为:[SS FS SS SF SS]或者[SS SF SS FS SS]
    # body_pose： [x(m), y(m), z(m), yaw(deg)]
    body_poses = [
        [0.1, 0.1, 0, 90],
        [0.1, 0.1, 0, 90],
        [0.1, 0.0, 0, 180],
        [0.1, 0.1, 0, 180],
        # [0.2, -0.1, 0, -30],
        # [0.3, 0.0, 0, -0],
        # [0.4, 0.0, 0, -30],
        # [0.5, 0.0, 0, 0],
    ]
    msg = get_multiple_steps_msg(body_poses, dt, is_left_first_default, collision_check)
    pub.publish(msg)
