#!/usr/bin/env python3
"""
增强版6D步态控制器
集成自动步态控制功能，保留斜坡功能，替换单步移动功能
"""

import rospy
import numpy as np
import sys
import time
from kuavo_msgs.msg import footPose6DTargetTrajectories, footPose6D
from std_srvs.srv import SetBool, SetBoolRequest

def get_user_input():
    """获取用户输入"""
    print("\n" + "="*60)
    print("增强版6D步态控制器 (集成自动步态)")
    print("="*60)
    print("请选择要执行的功能:")
    print("┌─" + "─"*56 + "─┐")
    print("│ 1. 上斜坡 (slope) - 6D轨迹控制                    │")
    print("│ 2. 下斜坡 (downslope) - 6D轨迹控制                │")
    print("│ 3. 上下斜坡 (up and downslope) - 6D轨迹控制                │")
    print("│ 4. 退出                                             │")
    print("└─" + "─"*56 + "─┘")
    
    while True:
        choice = input("\n请输入选择 (1-9): ").strip()
        
        if choice == '1':
            return 'slope'
        elif choice == '2':
            return 'downslope'
        elif choice == '3':
            return 'up_downslope'
        elif choice == '4':
            return 'quit'
        else:
            print("无效选择，请输入 1-4")

def set_pitch_limit(enable):
    """设置基座俯仰角限制"""
    rospy.wait_for_service('/humanoid/mpc/enable_base_pitch_limit')
    try:
        set_pitch_limit_service = rospy.ServiceProxy('/humanoid/mpc/enable_base_pitch_limit', SetBool)
        req = SetBoolRequest()
        req.data = enable
        resp = set_pitch_limit_service(req)
        if resp.success:
            rospy.loginfo(f"成功{'启用' if enable else '禁用'}pitch限制")
        else:
            rospy.logwarn(f"失败{'启用' if enable else '禁用'}pitch限制")
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

class EnhancedSlopeController:
    """增强版斜坡控制器，集成自动步态控制"""
    
    def __init__(self):
        # 全局高度管理
        self.swing_height = 0.06
        self.torso_height = 0.0
        
        # 轨迹发布器 (用于斜坡控制)
        self.traj_pub = rospy.Publisher('/humanoid_mpc_foot_pose_6d_target_trajectories', 
                                       footPose6DTargetTrajectories, queue_size=1)
        
        # 等待发布器准备就绪
        rospy.sleep(0.5)
    
    def simple_up_stairs(self, distance_x, stairs_angle, step_num, step_length):  # 输入x方向距离，坡度，除第一步和最后一步外，机器人需要进行的步数，机器人脚掌长度
        
        """完整的斜坡轨迹生成函数

        Args:
            distance_x: 斜坡x方向总长度 (0.9m)
            stairs_angle: 斜坡角度 (6度)
            step_num: 迈步次数 (8步)
            step_length: 机器人脚掌长度 (0.215m)
        """
        # 将角度转换为弧度
        angle_rad = np.radians(stairs_angle)

        # 计算斜坡高度
        slope_height = distance_x * np.tan(angle_rad)
        
        # 脚掌占斜坡的x方向长度和高度
        foot_length_x = step_length * np.cos(angle_rad)
        foot_length_z = step_length * np.sin(angle_rad)
        
        # 计算斜坡上每步的x和z增量
        step_x_increment = (distance_x - foot_length_x) / (step_num-2)
        step_z_increment = (slope_height - foot_length_z) / (step_num-2)
        
        # 两脚之间的宽度
        foot_width = 0.108536  # 脚宽度
        
        # 初始位置
        foot_x = 0.0
        foot_z = self.torso_height
        torso_x = 0.0
        torso_z = self.torso_height
        
        # 脚掌距离斜坡的初始距离
        foot_distance_initial = 0.02
        
        # 数据结构初始化
        time_traj = []
        foot_idx_traj = []  # 默认左脚
        foot_poses_6d = []
        torso_poses_6d = []
        
        # 迈步时间间隔
        swing_time = 0.5  # 基础时间间隔
        
        # 生成每一步的轨迹
        step_x_init = foot_x + foot_distance_initial + foot_length_x
        step_z_init = foot_z + foot_length_z
        torso_x_init = torso_x + step_x_init
        torso_z_init = torso_z + foot_length_z
        step_swing_time = swing_time
        for step in range(step_num+1):  # 生成第一步和上斜坡的步态
            # 左右脚交替
            foot_index = step % 2  # 0:左脚, 1:右脚
            if step == 0:
                # 第一步：准备姿势
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)  # 左脚
                foot_poses_6d.append([
                    step_x_init, foot_width, step_z_init, 0.0, -angle_rad, 0.0
                ])
                torso_poses_6d.append([
                    torso_x_init, 0.0, torso_z_init, 0.0, 0.0, 0.0
                ])
            elif step >= (step_num-1): # 最后两步回正
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)
                # y坐标：左脚在右侧(0.1)，右脚在左侧(-0.1)
                y_pos = foot_width if foot_index == 0 else -foot_width
                # 更新最后一步的位置
                step_x_last = step_x_init + step_x_increment * (step_num-2) + foot_length_x
                step_z_last = step_z_init + step_z_increment * (step_num-2) + foot_length_z
                torso_x_last = torso_x_init + step_x_increment * (step_num-2) + foot_length_x
                torso_z_last = torso_z_init + step_z_increment * (step_num-2) + foot_length_z
                # 足部姿态：保持水平
                foot_poses_6d.append([
                    step_x_last, y_pos, step_z_last, 0.0, 0.0, 0.0
                ])
                # 躯干姿态：保持水平，稍微调整高度
                torso_poses_6d.append([
                    torso_x_last, 0.0, torso_z_last, 0.0, 0.0, 0.0  # 躯干比脚高10cm
                ])
            else:
                # 正常迈步
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)

                # 更新下一步位置
                step_x_new = step_x_init + step_x_increment * step
                step_z_new = step_z_init + step_z_increment * step
                torso_x_new = torso_x_init + step_x_increment * step
                torso_z_new = torso_z_init + step_z_increment * step

                # y坐标：左脚在右侧(0.1)，右脚在左侧(-0.1)
                y_pos = foot_width if foot_index == 0 else -foot_width

                # 足部姿态：跟随斜坡角度
                foot_poses_6d.append([
                    step_x_new, y_pos, step_z_new, 0.0, -angle_rad, 0.0
                ])

                # 躯干姿态：保持水平，稍微调整高度
                torso_poses_6d.append([
                    torso_x_new, 0.0, torso_z_new, 0.0, 0.0, 0.0  # 躯干比脚高10cm
                ])
            step_swing_time = step_swing_time + swing_time
        
        # 更新全局躯干高度位姿
        self.torso_height = slope_height
                
        return self.get_foot_pose_6d_traj_msg(time_traj, foot_idx_traj, foot_poses_6d, torso_poses_6d)

    def simple_down_stairs(self, distance_x, stairs_angle, step_num, step_length):  # 输入x方向距离，坡度，除第一步和最后一步外，机器人需要进行的步数，机器人脚掌长度
        
        """完整的斜坡轨迹生成函数

        Args:
            distance_x: 斜坡x方向总长度 (0.9m)
            stairs_angle: 斜坡角度 (6度)
            step_num: 迈步次数 (8步)
            step_length: 机器人脚掌长度 (0.215m)
        """
        # 将角度转换为弧度
        angle_rad = np.radians(stairs_angle)

        # 计算斜坡高度
        slope_height = distance_x * np.tan(angle_rad)
        
        # 脚掌占斜坡的x方向长度和高度
        foot_length_x = step_length * np.cos(angle_rad)
        foot_length_z = step_length * np.sin(angle_rad)
        
        # 计算斜坡上每步的x和z增量
        step_x_increment = (distance_x - foot_length_x) / (step_num-2)
        step_z_increment = (slope_height - foot_length_z) / (step_num-2)
        
        # 两脚之间的宽度
        foot_width = 0.108536  # 脚宽度
        
        # 初始位置
        foot_x = 0.0
        foot_z = slope_height
        torso_x = 0.0
        torso_z = 0.0
        
        # 脚掌距离斜坡的初始距离
        foot_distance_initial = 0.02
        
        # 数据结构初始化
        time_traj = []
        foot_idx_traj = []  # 默认左脚
        foot_poses_6d = []
        torso_poses_6d = []
        
        # 迈步时间间隔
        swing_time = 0.5  # 基础时间间隔
        
        # 生成每一步的轨迹
        step_x_init = foot_x + foot_distance_initial + foot_length_x
        step_z_init = foot_z - foot_length_z
        torso_x_init = torso_x + step_x_init
        torso_z_init = torso_z - foot_length_z
        step_swing_time = swing_time
        for step in range(step_num+1):  # 生成第一步和上斜坡的步态
            # 左右脚交替
            foot_index = step % 2  # 0:左脚, 1:右脚
            if step == 0:
                # 第一步：准备姿势
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)  # 左脚
                foot_poses_6d.append([
                    step_x_init, foot_width, step_z_init, 0.0, angle_rad, 0.0
                ])
                torso_poses_6d.append([
                    torso_x_init, 0.0, torso_z, 0.0, 0.0, 0.0
                ])
            elif step >= (step_num-1): # 最后两步回正
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)
                # y坐标：左脚在右侧(0.1)，右脚在左侧(-0.1)
                y_pos = foot_width if foot_index == 0 else -foot_width
                # 更新最后一步的位置
                step_x_last = step_x_init + step_x_increment * (step_num-2) + foot_length_x
                step_z_last = step_z_init - step_z_increment * (step_num-2) - foot_length_z
                torso_x_last = torso_x_init + step_x_increment * (step_num-2) + foot_length_x
                torso_z_last = torso_z_init - step_z_increment * (step_num-2) - foot_length_z
                # 足部姿态：保持水平
                foot_poses_6d.append([
                    step_x_last, y_pos, step_z_last, 0.0, 0.0, 0.0
                ])
                # 躯干姿态：保持水平，稍微调整高度
                torso_poses_6d.append([
                    torso_x_last, 0.0, torso_z_last, 0.0, 0.0, 0.0  # 躯干比脚高10cm
                ])
                # 更新全局躯干高度位姿
                self.torso_height = torso_z_last
            else:
                # 正常迈步
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)

                # 更新下一步位置
                step_x_new = step_x_init + step_x_increment * step
                step_z_new = step_z_init - step_z_increment * step
                torso_x_new = torso_x_init + step_x_increment * step
                torso_z_new = torso_z_init - step_z_increment * step

                # y坐标：左脚在右侧(0.1)，右脚在左侧(-0.1)
                y_pos = foot_width if foot_index == 0 else -foot_width

                # 足部姿态：跟随斜坡角度
                foot_poses_6d.append([
                    step_x_new, y_pos, step_z_new, 0.0, angle_rad, 0.0
                ])

                # 躯干姿态：保持水平，稍微调整高度
                torso_poses_6d.append([
                    torso_x_new, 0.0, torso_z_new, 0.0, 0.0, 0.0  # 躯干比脚高10cm
                ])
            step_swing_time = step_swing_time + swing_time
        
        return self.get_foot_pose_6d_traj_msg(time_traj, foot_idx_traj, foot_poses_6d, torso_poses_6d)
    
    def simple_up_down_stairs(self, distance_x, stairs_angle, step_num, step_length):  # 输入x方向距离，坡度，除第一步和最后一步外，机器人需要进行的步数，机器人脚掌长度
        
        """完整的斜坡轨迹生成函数

        Args:
            distance_x: 斜坡x方向总长度 (0.9m)
            stairs_angle: 斜坡角度 (6度)
            step_num: 迈步次数 (8步)
            step_length: 机器人脚掌长度 (0.215m)
        """
        # 将角度转换为弧度
        angle_rad = np.radians(stairs_angle)

        # 计算斜坡高度
        slope_height = distance_x * np.tan(angle_rad)
        
        # 脚掌占斜坡的x方向长度和高度
        foot_length_x = step_length * np.cos(angle_rad)
        foot_length_z = step_length * np.sin(angle_rad)
        
        # 计算斜坡上每步的x和z增量
        step_x_increment = (distance_x - foot_length_x) / (step_num-2)
        step_z_increment = (slope_height - foot_length_z) / (step_num-2)
        
        # 两脚之间的宽度
        foot_width = 0.108536  # 脚宽度
        
        # 初始位置
        foot_x = 0.0
        foot_z = self.torso_height
        torso_x = 0.0
        torso_z = self.torso_height
        
        # 脚掌距离斜坡的初始距离
        foot_distance_initial = 0.02
        
        # 数据结构初始化
        time_traj = []
        foot_idx_traj = []  # 默认左脚
        foot_poses_6d = []
        torso_poses_6d = []
        
        # 迈步时间间隔
        swing_time = 0.5  # 基础时间间隔
        
        # 上斜坡轨迹
        step_x_init = foot_x + foot_distance_initial + foot_length_x
        step_z_init = foot_z + foot_length_z
        torso_x_init = torso_x + step_x_init
        torso_z_init = torso_z + foot_length_z
        step_swing_time = swing_time
        step_x_last = 0
        step_z_last = 0
        torso_x_last = 0
        torso_z_last = 0
        for step in range(step_num+1):  # 生成第一步和上斜坡的
            # 左右脚交替
            foot_index = step % 2  # 0:左脚, 1:右脚
            if step == 0:
                # 第一步：准备姿势
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)  # 左脚
                foot_poses_6d.append([
                    step_x_init, foot_width, step_z_init, 0.0, -angle_rad, 0.0
                ])
                torso_poses_6d.append([
                    torso_x_init, 0.0, torso_z_init, 0.0, 0.0, 0.0
                ])
            elif step >= (step_num-1): # 最后两步回正
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)
                # y坐标：左脚在右侧(0.1)，右脚在左侧(-0.1)
                y_pos = foot_width if foot_index == 0 else -foot_width
                # 更新最后一步的位置
                step_x_last = step_x_init + step_x_increment * (step_num-2) + foot_length_x
                step_z_last = step_z_init + step_z_increment * (step_num-2) + foot_length_z
                torso_x_last = torso_x_init + step_x_increment * (step_num-2) + foot_length_x
                torso_z_last = torso_z_init + step_z_increment * (step_num-2) + foot_length_z
                # 足部姿态：保持水平
                foot_poses_6d.append([
                    step_x_last, y_pos, step_z_last, 0.0, 0.0, 0.0
                ])
                # 躯干姿态：保持水平，稍微调整高度
                torso_poses_6d.append([
                    torso_x_last, 0.0, torso_z_last, 0.0, 0.0, 0.0  # 躯干比脚高10cm
                ])
            else:
                # 正常迈步
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)

                # 更新下一步位置
                step_x_new = step_x_init + step_x_increment * step
                step_z_new = step_z_init + step_z_increment * step
                torso_x_new = torso_x_init + step_x_increment * step
                torso_z_new = torso_z_init + step_z_increment * step

                # y坐标：左脚在右侧(0.1)，右脚在左侧(-0.1)
                y_pos = foot_width if foot_index == 0 else -foot_width

                # 足部姿态：跟随斜坡角度
                foot_poses_6d.append([
                    step_x_new, y_pos, step_z_new, 0.0, -angle_rad, 0.0
                ])

                # 躯干姿态：保持水平，稍微调整高度
                torso_poses_6d.append([
                    torso_x_new, 0.0, torso_z_new, 0.0, 0.0, 0.0  # 躯干比脚高10cm
                ])
            step_swing_time = step_swing_time + swing_time
        
        # 站立一会
        stance_time = 0.5;
        time_traj.append(step_swing_time + stance_time)
        foot_idx_traj.append(2)
        foot_poses_6d.append([step_x_last, 0.0, torso_z_last, 0.0,  0.0, 0.0])
        torso_poses_6d.append([step_x_last, 0.0, torso_z_last, 0.0, 0.0, 0.0])
        step_swing_time = step_swing_time + stance_time + swing_time
        
        # 下斜坡轨迹
        step_x_init = step_x_last + foot_length_x
        step_z_init = step_z_last - foot_length_z
        torso_x_init = torso_x_last + foot_length_x
        torso_z_init = torso_z_last - foot_length_z
        torso_z_offset = 0.02
        for step in range(step_num+1):  # 生成第一步和上斜坡的步态
            # 左右脚交替
            foot_index = step % 2  # 0:左脚, 1:右脚
            if step == 0:
                # 第一步：准备姿势
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)  # 左脚
                foot_poses_6d.append([
                    step_x_init, foot_width, step_z_init, 0.0, angle_rad, 0.0
                ])
                torso_poses_6d.append([
                    torso_x_init, 0.0, torso_z_init - torso_z_offset, 0.0, 0.0, 0.0
                ])
            elif step >= (step_num-1): # 最后两步回正
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)
                # y坐标：左脚在右侧(0.1)，右脚在左侧(-0.1)
                y_pos = foot_width if foot_index == 0 else -foot_width
                # 更新最后一步的位置
                step_x_last = step_x_init + step_x_increment * (step_num-2) + foot_length_x
                step_z_last = step_z_init - step_z_increment * (step_num-2) - foot_length_z
                torso_x_last = torso_x_init + step_x_increment * (step_num-2) + foot_length_x
                torso_z_last = torso_z_init - step_z_increment * (step_num-2) - foot_length_z
                # 足部姿态：保持水平
                foot_poses_6d.append([
                    step_x_last, y_pos, step_z_last, 0.0, 0.0, 0.0
                ])
                # 躯干姿态：保持水平，稍微调整高度
                torso_poses_6d.append([
                    torso_x_last, 0.0, torso_z_last - torso_z_offset, 0.0, 0.0, 0.0  # 躯干比脚高10cm
                ])
            else:
                # 正常迈步
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)

                # 更新下一步位置
                step_x_new = step_x_init + step_x_increment * step
                step_z_new = step_z_init - step_z_increment * step
                torso_x_new = torso_x_init + step_x_increment * step
                torso_z_new = torso_z_init - step_z_increment * step

                # y坐标：左脚在右侧(0.1)，右脚在左侧(-0.1)
                y_pos = foot_width if foot_index == 0 else -foot_width

                # 足部姿态：跟随斜坡角度
                foot_poses_6d.append([
                    step_x_new, y_pos, step_z_new, 0.0, angle_rad, 0.0
                ])

                # 躯干姿态：保持水平，稍微调整高度
                torso_poses_6d.append([
                    torso_x_new, 0.0, torso_z_new - torso_z_offset, 0.0, 0.0, 0.0  # 躯干比脚高10cm
                ])
            step_swing_time = step_swing_time + swing_time
        
        # 更新全局躯干高度位姿
        self.torso_height = slope_height
                
        return self.get_foot_pose_6d_traj_msg(time_traj, foot_idx_traj, foot_poses_6d, torso_poses_6d)
    
    def get_foot_pose_6d_traj_msg(self, time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_height_traj=None):
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
        
        self.torso_height = 0.0
        # 设置摆动高度轨迹（如果提供）
        if swing_height_traj is not None:
            msg.swingHeightTrajectory = swing_height_traj
        else:
            # 默认摆动高度
            msg.swingHeightTrajectory = [self.swing_height] * num

        return msg
    
    def publish_trajectory(self, msg):
        """发布轨迹消息"""
        if msg is not None:
            self.traj_pub.publish(msg)
            print("轨迹消息发布成功")
            return True
        else:
            print("错误: 轨迹消息为空")
            return False
        
def main():
    # 初始化ROS节点
    rospy.init_node('enhanced_slope_controller', anonymous=True)
    
    print("增强版6D步态控制器启动")
    print("正在连接ROS节点...")
    
    # 禁用俯仰角限制
    set_pitch_limit(False)
    # 创建增强控制器
    controller = EnhancedSlopeController()
    
    # ========== 所有参数统一设置 ==========
    
    # 斜坡参数
    SLOPE_X_DISTANCE = 0.9    # x方向长度
    UP_SLOPE_ANGLE = 6.0    #斜坡斜度，单位：度
    DOWN_SLOPE_ANGLE = 6.0
    
    # 机器人参数
    STEP_NUM = 6    # 迈步次数
    STEP_LENGTH = 0.215     # 机器人脚掌长度
    
    msg_6d_slope = None
    
    while not rospy.is_shutdown():
        try:
            # 获取用户选择
            command = get_user_input()
            
            if command == 'quit':
                print("退出程序...")
                break
                
            elif command == 'slope':
                msg_6d_slope = controller.simple_up_stairs(SLOPE_X_DISTANCE, UP_SLOPE_ANGLE, STEP_NUM, STEP_LENGTH)
                
            elif command == 'downslope':
                msg_6d_slope = controller.simple_down_stairs(SLOPE_X_DISTANCE, DOWN_SLOPE_ANGLE, STEP_NUM, STEP_LENGTH)
            
            elif command == 'up_downslope':
                msg_6d_slope = controller.simple_up_down_stairs(SLOPE_X_DISTANCE, DOWN_SLOPE_ANGLE, STEP_NUM, STEP_LENGTH)
            
            # 发布轨迹 (仅用于斜坡控制)
            controller.publish_trajectory(msg_6d_slope)
                
        except KeyboardInterrupt:
            print("\n用户中断执行")
            break
        except Exception as e:
            print(f"执行出错: {e}")
            import traceback
            traceback.print_exc()
            continue

if __name__ == '__main__':
    main() 
