#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
边走边快速转腰示例
该示例演示了如何让机器人在行走的同时进行腰部的快速转动
"""

import rospy
import math
from std_msgs.msg import Float64MultiArray
import time
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories, switchGaitByName  # 导入自定义消息类型
from sat import RotatingRectangle  # 导入用于碰撞检测的工具类
import numpy as np


# 腰部转速：1.57弧度
# 机器人 45 度


class SingleStepControl:
    def __init__(self):

        # 创建发布者，话题为 /humanoid_mpc_foot_pose_target_trajectories
        self.single_step_control_pub = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories',
                                                       footPoseTargetTrajectories, queue_size=10)
        # 等待一定时间以确保订阅者已经准备好
        rospy.sleep(1)

        self.is_left_first_default = True  # 缺省左脚先行
        self.collision_check = True  # 缺省开启碰撞检测

        self.dt = 0.3

        # 初始化body_poses_fast_go列表
        self.body_poses_fast_go = []
        self.body_poses_fast_go.append([0, 0.0, 0, 0])
        self.body_poses_fast_go.append([0, 0.0, 0, 0])

        # 从0.1到3.0 [0,0,0,0]
        for i in range(30):
            self.body_poses_fast_go.append([0.05 * i, 0.0, 0, 0])

        self.body_poses_stop = [
            [0.0, 0.0, 0, 0],
        ]

    def action(self, action):
        # 生成步态消息并发布
        msg = self.get_multiple_steps_msg(action, self.dt, self.is_left_first_default, self.collision_check)
        self.single_step_control_pub.publish(msg)

    def get_foot_pose_traj_msg(self, time_traj, foot_idx_traj, foot_traj, torso_traj):
        """
        创建并返回一个 footPoseTargetTrajectories 消息对象。

        参数：
        - time_traj: 时间轨迹列表
        - foot_idx_traj: 脚索引轨迹列表
        - foot_traj: 脚姿态轨迹列表
        - torso_traj: 躯干姿态轨迹列表

        返回：
        - footPoseTargetTrajectories 消息对象
        """
        num = len(time_traj)
        msg = footPoseTargetTrajectories()  # 创建消息实例
        msg.timeTrajectory = time_traj  # 设置时间轨迹
        msg.footIndexTrajectory = foot_idx_traj  # 设置脚索引
        msg.footPoseTrajectory = []  # 初始化脚姿态轨迹

        for i in range(num):
            foot_pose_msg = footPose()  # 创建脚姿态信息
            foot_pose_msg.footPose = foot_traj[i]  # 设置脚姿态
            foot_pose_msg.torsoPose = torso_traj[i]  # 设置躯干姿态
            msg.footPoseTrajectory.append(foot_pose_msg)  # 将脚姿态添加到消息中

        return msg

    def generate_steps(self, torso_pos, torso_yaw, foot_bias):
        """
        根据躯干位置和偏航角生成左右脚的位置。

        参数：
        - torso_pos: 躯干位置
        - torso_yaw: 躯干偏航角（弧度）
        - foot_bias: 脚偏移量

        返回：
        - 左脚和右脚的位置
        """
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

    def get_multiple_steps_msg(self, body_poses, dt, is_left_first=True, collision_check=True):
        """
        生成多步步态消息。

        参数：
        - body_poses: 身体姿态列表
        - dt: 时间间隔
        - is_left_first: 是否左脚先行
        - collision_check: 是否进行碰撞检测

        返回：
        - footPoseTargetTrajectories 消息对象
        """
        num_steps = 2 * len(body_poses)  # 每个身体姿态对应两步
        time_traj = []  # 时间轨迹
        foot_idx_traj = []  # 脚索引轨迹
        foot_traj = []  # 脚姿态轨迹
        torso_traj = []  # 躯干姿态轨迹

        # 初始化左右脚的旋转矩形用于碰撞检测
        l_foot_rect_last = RotatingRectangle(center=(0, 0.1), width=0.24, height=0.1, angle=0)
        r_foot_rect_last = RotatingRectangle(center=(0, -0.1), width=0.24, height=0.1, angle=0)

        is_left_support = not is_left_first  # 第一脚迈出
        for i, body_pose in enumerate(body_poses):
            time_traj.append(dt * (i + 1))  # 添加时间点
            torso_pos = np.asarray(body_pose[:3])  # 躯干位置
            torso_yaw = np.radians(body_pose[3])  # 躯干偏航角（转换为弧度）

            l_foot, r_foot = self.generate_steps(torso_pos, torso_yaw, 0.1)  # 生成左右脚位置
            l_foot = [*l_foot[:3], torso_yaw]  # 左脚姿态
            r_foot = [*r_foot[:3], torso_yaw]  # 右脚姿态

            if collision_check and i % 2 == 0:  # 每两步进行一次碰撞检测
                l_foot_rect_next = RotatingRectangle(center=(l_foot[0], l_foot[1]), width=0.24, height=0.1,
                                                     angle=torso_yaw)
                r_foot_rect_next = RotatingRectangle(center=(r_foot[0], r_foot[1]), width=0.24, height=0.1,
                                                     angle=torso_yaw)
                l_collision = l_foot_rect_next.is_collision(r_foot_rect_last)
                r_collision = r_foot_rect_next.is_collision(l_foot_rect_last)

                if l_collision and r_collision:
                    print("\033[91m[Error] Detect collision, Please adjust your body_poses input!!!\033[0m")
                    break
                elif l_collision:
                    print("\033[92m[Info] Left foot is in collision, switch to right foot\033[0m")
                    is_left_first = False
                else:
                    is_left_first = True

                l_foot_rect_last = l_foot_rect_next
                r_foot_rect_last = r_foot_rect_next

            if is_left_support:
                # 右脚迈步
                foot_idx_traj.append(1)
                foot_traj.append(r_foot)
            else:
                # 左脚迈步
                foot_idx_traj.append(0)
                foot_traj.append(l_foot)

            is_left_support = not is_left_support  # 交替
            torso_traj.append([*body_pose[:3], torso_yaw])  # 添加躯干姿态

        # 步态正常生成后，补一个双脚齐平
        final_body_pose = body_poses[-1]
        torso_pos = np.asarray(final_body_pose[:3])
        torso_yaw = np.radians(final_body_pose[3])
        l_foot, r_foot = self.generate_steps(torso_pos, torso_yaw, 0.1)
        l_foot = [*l_foot[:3], torso_yaw]
        r_foot = [*r_foot[:3], torso_yaw]

        # 左脚齐平
        foot_idx_traj.append(0)
        foot_traj.append(l_foot)
        torso_traj.append([*final_body_pose[:3], torso_yaw])
        time_traj.append(time_traj[-1] + dt)

        # 右脚齐平
        foot_idx_traj.append(1)
        foot_traj.append(r_foot)
        torso_traj.append([*final_body_pose[:3], torso_yaw])
        time_traj.append(time_traj[-1] + dt)

        print("time_traj:", time_traj)
        print("foot_idx_traj:", foot_idx_traj)
        print("foot_traj:", foot_traj)
        print("torso_traj:", torso_traj)

        return self.get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)


class WaistRotationWhileWalking:
    def __init__(self):

        self.single_step_control = SingleStepControl()
        # 创建腰部控制发布者
        self.waist_pub = rospy.Publisher('/robot_waist_motion_data', Float64MultiArray, queue_size=10)

        # 等待节点初始化
        rospy.sleep(1.0)

    def execute_demo(self):
        """执行边走边转腰演示"""
        rospy.loginfo("开始执行边走边快速转腰演示...")

        # 2. 开始行走并同时转腰
        rospy.loginfo("开始行走并同时转腰...")

        # 等待3秒后再开始转腰
        wait_time = 1.5
        waist_angle = 45

        # 腰部恢复为 0
        waist_msg = Float64MultiArray()
        waist_msg.data = [0]
        self.waist_pub.publish(waist_msg)

        # 等待3秒
        time.sleep(3)

        # 快速行走
        self.single_step_control.action(self.single_step_control.body_poses_fast_go)
        time.sleep(2)

        # ros频率
        rate = rospy.Rate(10)

        # 结束时间
        end_time = 1.6 * self.single_step_control.dt * len(self.single_step_control.body_poses_fast_go)

        last_waist_time = time.time()
        current_time = time.time()
        print("end_time:", end_time)
        while time.time() - current_time < end_time:

            # 每过 1.5 秒转一次腰
            if (time.time() - last_waist_time) >= wait_time:
                last_waist_time = time.time()

                waist_angle = -waist_angle
                # 发布腰部控制消息
                waist_msg.data = [waist_angle]
                self.waist_pub.publish(waist_msg)

            rate.sleep()

        # 重置腰部到中间位置
        waist_msg = Float64MultiArray()
        waist_msg.data = [0.0]
        self.waist_pub.publish(waist_msg)

        rospy.loginfo("演示完成！")


if __name__ == "__main__":
    try:
        # 初始化ROS节点
        rospy.init_node('waist_rotation_while_walking_demo', anonymous=False)

        # 创建演示实例并执行
        demo = WaistRotationWhileWalking()
        demo.execute_demo()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"演示过程中发生错误: {e}")
