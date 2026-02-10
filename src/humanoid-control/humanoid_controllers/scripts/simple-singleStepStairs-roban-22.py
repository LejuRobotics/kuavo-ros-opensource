#!/usr/bin/env python3
"""
增强版6D步态控制器
集成自动步态控制功能，保留斜坡功能，替换单步移动功能
"""

import rospy
import numpy as np
import sys
import time
import json
import os
from kuavo_msgs.msg import footPose6DTargetTrajectories, footPose6D, footPoses6D, AprilTagDetectionArray
from scipy.interpolate import CubicSpline, PchipInterpolator, Akima1DInterpolator, UnivariateSpline
from kuavo_msgs.srv import stairAlignmentSrv, stairAlignmentSrvRequest
from std_srvs.srv import SetBool, SetBoolRequest

def get_user_input():
    """获取用户输入"""
    print("\n" + "="*51)
    print("增强版6D步态控制器 (集成自动步态)")
    print("="*51)
    print("请选择要执行的功能:")
    print("┌─" + "─"*47 + "─┐")
    print("│ 1. 获取 AprilTag 偏置并对齐                     │")
    print("│ 2. 上楼梯 (stair)(实物使用) - 6D轨迹控制        │")
    print("│ 3. 上楼梯 (stair)(仿真使用) - 6D轨迹控制        │")
    print("│ 4. 退出                                         │")
    print("└─" + "─"*47 + "─┘")
    
    while True:
        choice = input("\n请输入选择 (1-4): ").strip()
        
        if choice == '1':
            return 'get_offset'
        elif choice == '2':
            return 'stair'
        elif choice == '3':
            return 'stair_sim'
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
        self.swing_height = 0.12
        self.torso_height = 0.0

        # 楼梯参数
        self.stair_height = 0.0
        self.stair_length = 0.0
        
        # 配置文件路径（保存在脚本同目录下）
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(script_dir, 'stair_alignment_config.json')
        
        # 对齐参数（默认值）
        self.default_offset_x = -0.80
        self.default_offset_y = -0.3
        self.default_offset_yaw = 0.0
        
        # 从配置文件加载参数
        self.input_tag_id = 0
        self.input_tag_offset_x = self.default_offset_x
        self.input_tag_offset_y = self.default_offset_y
        self.input_tag_offset_yaw = self.default_offset_yaw
        self.offset_set = False  # 标记是否已设置偏置
        
        # 加载配置
        self.load_config()
        
        # 轨迹发布器 (用于斜坡控制)
        self.traj_pub = rospy.Publisher('/humanoid_mpc_foot_pose_6d_target_trajectories', 
                                       footPose6DTargetTrajectories, queue_size=1)
        
        # 等待发布器准备就绪
        rospy.sleep(0.5)
    
    def plan_swing_phase(self, prev_foot_pose, next_foot_pose, num_points):
        """
        使用形状保持的三次样条插值规划腾空相的轨迹
        Args:
            prev_foot_pose: 上一个落点位置 [x, y, z, yaw]
            next_foot_pose: 下一个落点位置 [x, y, z, yaw]
            swing_height: 抬脚最大高度，默认0.2米
            plot: 是否绘制轨迹图，默认False
        Returns:
            additionalFootPoseTrajectory: 包含腾空相轨迹的footPoses消息
        """
        additionalFootPoseTrajectory = footPoses6D()

        # 创建时间序列
        t = np.linspace(0, 1, num_points)

        # 计算x和y方向的移动距离
        x_distance = next_foot_pose[0] - prev_foot_pose[0]
        y_distance = next_foot_pose[1] - prev_foot_pose[1]
        z_distance = next_foot_pose[2] - prev_foot_pose[2]

        # 创建控制点
        # 时间点：0, 0.2, 0.5, 1.0
        # 0.2时刻：x和y移动10%，z达到最高点
        # 0.5时刻：x和y移动50%，z保持最高点
        
        control_points = {
        't': [0, 0.25, 0.5, 0.75, 1.0],    # 按归一化来使用, 0, 20%, 60%, 1
        'x': [
            prev_foot_pose[0] + x_distance * 0.0,
            prev_foot_pose[0] + x_distance * 0.25,
            prev_foot_pose[0] + x_distance * 0.5,
            prev_foot_pose[0] + x_distance * 0.75,
            prev_foot_pose[0] + x_distance * 1.0,
        ],
        'z': [
            prev_foot_pose[2] + z_distance * 0.0,
            next_foot_pose[2] + self.swing_height * 0.5,
            next_foot_pose[2] + self.swing_height,
            next_foot_pose[2] + self.swing_height * 0.5,
            next_foot_pose[2],
        ]
        }

        # 为x、y和z创建形状保持的三次样条插值
        x_spline = PchipInterpolator(control_points['t'], control_points['x'])
        z_spline = PchipInterpolator(control_points['t'], control_points['z'])

        # 生成轨迹点
        for i in range(num_points):
            step_fp = footPose6D()
            x = float(x_spline(t[i]))
            z = float(z_spline(t[i]))
            
            step_fp.footPose6D = [x, next_foot_pose[1], z, next_foot_pose[3], 0.0, 0.0]
            additionalFootPoseTrajectory.data.append(step_fp)

        if len(additionalFootPoseTrajectory.data) >= 2:
            additionalFootPoseTrajectory.data.pop(0)  # 删除第一个点
            additionalFootPoseTrajectory.data.pop(-1)  # 删除最后一个点

        return additionalFootPoseTrajectory
    
    def plan_swing_phase_by_stair(self, prev_foot_pose, next_foot_pose, num_points):
        """
        使用形状保持的三次样条插值规划腾空相的轨迹
        Args:
            prev_foot_pose: 上一个落点位置 [x, y, z, yaw]
            next_foot_pose: 下一个落点位置 [x, y, z, yaw]
            swing_height: 抬脚最大高度，默认0.2米
            plot: 是否绘制轨迹图，默认False
        Returns:
            additionalFootPoseTrajectory_stair: 包含腾空相轨迹的footPoses消息
        """
        additionalFootPoseTrajectory_stair = footPoses6D()

        # 创建时间序列
        t = np.linspace(0, 1, num_points)

        # 根据落足点高度差计算台阶数量
        stairNum = int(np.round((next_foot_pose[2] - prev_foot_pose[2]) / self.stair_height))
        print("stairNum: ", stairNum)
        # 创建控制点
        # 时间点：0, 0.2, 0.5, 1.0
        # 0.2时刻：x和y移动10%，z达到最高点
        # 0.5时刻：x和y移动50%，z保持最高点

        # 一阶段最高点的scale
        phase1_height_scale = 1.6
        # 计算第二阶段起点和终点的z值
        phase2_start_z = prev_foot_pose[2] + self.stair_height * phase1_height_scale  # 第一阶段结束高度
        phase2_end_z = prev_foot_pose[2] + self.stair_height * stairNum * 1.0 + self.swing_height * 1.0  # 最高点
        phase2_num = 3      # 二阶段点数
        
        control_points = {
        't': [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],    # 按归一化来使用, 0, 20%, 60%, 1
        'x': [
            prev_foot_pose[0] + self.stair_length * stairNum * 0.0,
            prev_foot_pose[0] + self.stair_length * stairNum * 0.0,
            prev_foot_pose[0] + self.stair_length * stairNum * 0.0,
            prev_foot_pose[0] + self.stair_length * stairNum * 0.0,
            prev_foot_pose[0] + self.stair_length * stairNum * 0.0,
            prev_foot_pose[0] + self.stair_length * stairNum * 0.17,
            prev_foot_pose[0] + self.stair_length * stairNum * 0.33,
            prev_foot_pose[0] + self.stair_length * stairNum * 0.5,
            prev_foot_pose[0] + self.stair_length * stairNum * 0.67,
            prev_foot_pose[0] + self.stair_length * stairNum * 0.83,
            prev_foot_pose[0] + self.stair_length * stairNum * 1.0,
        ],
        'z': [
            # 第一阶段：前3个点上升到1节台阶高度
            prev_foot_pose[2] + self.stair_height * 0.0,                                # t=0.0: 起始高度
            prev_foot_pose[2] + self.stair_height * phase1_height_scale / 6,            # t=0.1: 半节台阶高度
            prev_foot_pose[2] + self.stair_height * phase1_height_scale / 2,        # t=0.2: 1节台阶高度
            prev_foot_pose[2] + self.stair_height * phase1_height_scale / 4 * 3,        # t=0.3: 1节台阶高度
            prev_foot_pose[2] + self.stair_height * phase1_height_scale,                # t=0.4: 1节台阶高度
            
            # 第二阶段：中间3个点使用线性插值
            # t=0.4: 插值参数 t=0.5
            phase2_end_z * 1/phase2_num + phase2_start_z * (1-1/phase2_num),    # t=0.4
            # t=0.5: 插值参数 t=0.75
            phase2_end_z * 2/phase2_num + phase2_start_z * (1-2/phase2_num),  # t=0.5
            # t=0.6: 插值参数 t=1.0 (最高点)
            phase2_end_z * 3/phase2_num,                          # t=0.6
            
            # 第三阶段：后3个点下降到台阶总高度
            prev_foot_pose[2] + self.stair_height * stairNum * 1.0 + self.swing_height * 0.5,   # t=0.8
            prev_foot_pose[2] + self.stair_height * stairNum * 1.0 + self.swing_height * 0.2,   # t=0.9
            prev_foot_pose[2] + self.stair_height * stairNum * 1.0,                        # t=1.0: 最终高度
        ]
        }

        # 为x、y和z创建形状保持的三次样条插值
        # x_spline = PchipInterpolator(control_points['t'], control_points['x'])
        # z_spline = PchipInterpolator(control_points['t'], control_points['z'])
        x_spline = UnivariateSpline(control_points['t'], control_points['x'], s=0.1, k=3)
        z_spline = UnivariateSpline(control_points['t'], control_points['z'], s=0.1, k=3)

        # 生成轨迹点
        for i in range(num_points):
            step_fp = footPose6D()
            x = float(x_spline(t[i]))
            z = float(z_spline(t[i]))
            
            step_fp.footPose6D = [x, next_foot_pose[1], z, next_foot_pose[3], 0.0, 0.0]
            additionalFootPoseTrajectory_stair.data.append(step_fp)

        if len(additionalFootPoseTrajectory_stair.data) >= 2:
            additionalFootPoseTrajectory_stair.data.pop(0)  # 删除第一个点
            additionalFootPoseTrajectory_stair.data.pop(-1)  # 删除最后一个点

        return additionalFootPoseTrajectory_stair
    
    def simple_up_stairs(self, stair_height, stair_length, stair_num, distance_initial):  # 输入x方向距离，坡度，除第一步和最后一步外，机器人需要进行的步数，机器人脚掌长度
        
        """完整的楼梯轨迹生成函数

        Args:
            stair_height: 单级楼梯高度
            stair_length: 单级楼梯长度（深度）
            stair_num: 楼梯级数（需要迈步的次数）
        """
        
        # 两脚之间的宽度
        foot_width = 0.12441237  # 脚宽度
        
        # 脚掌距离斜坡的初始距离
        foot_distance_initial = distance_initial
        
        # 脚掌参数（来自 URDF: l_foot_toe / l_foot_heel 相对于 leg_l6_link 的 x 坐标）
        foot_toe = 0.1510
        foot_heel = -0.0538
        # 脚掌几何中心相对于 foot_sole(ankle投影) 的 x 偏移
        foot_offset = (foot_toe + foot_heel) / 2  # = 0.0486
        
        # 数据结构初始化
        time_traj = []
        foot_idx_traj = []  # 默认左脚
        foot_poses_6d = []
        torso_poses_6d = []
        swing_trajectories_array = []
        
        # 迈步时间间隔
        swing_time = 0.8  # 基础时间间隔
        contact_time = 1.2
        
        # 除第一步外的步进距离
        step_x_increment = stair_length
        step_z_increment = stair_height
        
        # 躯干位置偏置
        torso_offset_x = -0.03

        torso_offset_z = 0.08  # 仅第一步和最后一步生效
        
        # 生成每一步的轨迹
        step_x_init = foot_distance_initial + foot_toe + stair_length/2 - foot_offset
        step_z_init = stair_height
        torso_x_init = step_x_init + torso_offset_x
        torso_z_init = stair_height - torso_offset_z
        step_swing_time = swing_time
        
        # 存储上一摆动相的足端高度，用于接触相
        last_step_x = step_x_init
        last_step_z = step_z_init
        
        last_step_x_left = last_step_x
        last_step_z_left = last_step_z
        last_step_x_right = 0
        last_step_z_right = 0

        # 存储当前躯干位置
        current_torso_x = 0
        current_torso_z = 0
        
        # 添加最后的并拢步
        total_phases = stair_num * 2 + 1  # 包括所有楼梯步和最后的并拢步
    
        for step in range(total_phases):  # 生成第一步和上斜坡的步态
            print("step_swing_time: ", step_swing_time)
            if step < stair_num * 2:    # 楼梯步态
                if step % 2 == 0:  # 摆动相（实际迈步）
                    step_index = step // 2
                    foot_index = step_index % 2  # 0:左脚, 1:右脚

                    if step_index == 0:
                        # 第一步：准备姿势
                        time_traj.append(step_swing_time)
                        foot_idx_traj.append(foot_index)
                        foot_poses_6d.append([
                            step_x_init, foot_width, step_z_init, 0.0, 0.0, 0.0
                        ])
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])
                        # y坐标：左脚在右侧，右脚在左侧
                        y_pos = foot_width if foot_index == 0 else -foot_width
                        prev_foot = [0.0, y_pos, 0.0, 0.0]
                        new_foot = [step_x_init, y_pos, step_z_init, 0.0]
                        FootPoseTrajectory = self.plan_swing_phase_by_stair(prev_foot, new_foot, 8)
                        swing_trajectories_array.append(FootPoseTrajectory)
                        current_torso_x += torso_x_init
                        current_torso_z += torso_z_init
                    else:
                        # 正常迈步
                        time_traj.append(step_swing_time)
                        foot_idx_traj.append(foot_index)

                        # 更新下一步位置
                        step_x_new = step_x_init + step_x_increment * step_index
                        step_z_new = step_z_init + step_z_increment * step_index

                        # y坐标：左脚在右侧，右脚在左侧
                        y_pos = foot_width if foot_index == 0 else -foot_width

                        # 足部姿态
                        foot_poses_6d.append([
                            step_x_new, y_pos, step_z_new, 0.0, 0.0, 0.0
                        ])

                        # 躯干姿态
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])

                        # 已知上一时间的迈步的 x 和 z 为 last_step_x, last_step_z，
                        # 当前时间的迈步的 x 和 z 为 step_x_new, step_z_new
                        if foot_index == 0:
                            last_step_x = last_step_x_left
                            last_step_z = last_step_z_left
                        else:
                            last_step_x = last_step_x_right
                            last_step_z = last_step_z_right

                        prev_foot = [last_step_x, y_pos, last_step_z, 0.0]
                        new_foot = [step_x_new, y_pos, step_z_new, 0.0]
                        FootPoseTrajectory = self.plan_swing_phase_by_stair(prev_foot, new_foot, 8)
                        swing_trajectories_array.append(FootPoseTrajectory)
                        current_torso_x = torso_x_init + step_x_increment * (step_index)
                        current_torso_z = torso_z_init + step_z_increment * (step_index)
                        
                        last_step_z = step_z_new
                        last_step_x = step_x_new

                        if foot_index == 0:
                            last_step_x_left = last_step_x
                            last_step_z_left = last_step_z
                        else:
                            last_step_x_right = last_step_x
                            last_step_z_right = last_step_z

                    step_swing_time += contact_time

                else:  # 接触相（双脚着地）
                    step_index = (step - 1) // 2
                    time_traj.append(step_swing_time)
                    foot_idx_traj.append(2)  # 接触相标志

                    # 保持当前位置不变
                    if step_index == 0:
                        foot_poses_6d.append([
                            step_x_init, 0.0, step_z_init, 0.0, 0.0, 0.0
                        ])
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])
                        swing_trajectories_array.append(footPoses6D())
                    else:
                        step_x_current = last_step_x
                        step_z_current = last_step_z

                        # 保持双脚着地状态
                        foot_poses_6d.append([
                            step_x_current, 0.0, step_z_current, 0.0, 0.0, 0.0
                        ])
                        torso_poses_6d.append([
                            current_torso_x, 0.0, current_torso_z, 0.0, 0.0, 0.0
                        ])
                        swing_trajectories_array.append(footPoses6D())
                    step_swing_time += swing_time
            else:
                step_index = step // 2
                foot_index = step_index % 2  # 0:左脚, 1:右脚
                # y坐标：左脚在右侧，右脚在左侧
                y_pos = foot_width if foot_index == 0 else -foot_width
                
                # 更新最后一步位置
                step_x_last = step_x_init + step_x_increment * (stair_num-1)
                step_z_last = step_z_init + step_z_increment * (stair_num-1)
                torso_x_last = torso_x_init + step_x_increment * (stair_num-1)
                torso_z_last = torso_z_init + step_z_increment * (stair_num-1) + torso_offset_z
                
                time_traj.append(step_swing_time)
                foot_idx_traj.append(foot_index)
                foot_poses_6d.append([
                    step_x_last, y_pos, step_z_last, 0.0, 0.0, 0.0
                ])
                torso_poses_6d.append([
                    torso_x_last, 0.0, torso_z_last, 0.0, 0.0, 0.0
                ])

                if foot_index == 0:
                    last_step_x = last_step_x_left
                    last_step_z = last_step_z_left
                else:
                    last_step_x = last_step_x_right
                    last_step_z = last_step_z_right
                prev_foot = [last_step_x, y_pos, last_step_z, 0.0]
                new_foot = [step_x_last, y_pos, step_z_last, 0.0]
                FootPoseTrajectory = self.plan_swing_phase_by_stair(prev_foot, new_foot, 8)
                swing_trajectories_array.append(FootPoseTrajectory)
                
        return self.get_foot_pose_6d_traj_msg(time_traj, foot_idx_traj, foot_poses_6d, torso_poses_6d, swing_trajectories_array)
    
    def get_foot_pose_6d_traj_msg(self, time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories=None):
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

            # 如果有腾空相轨迹，添加到消息中
            if swing_trajectories is not None:
                # 将swing_trajectories[i]中的轨迹点添加到swing_poses中
                if swing_trajectories[i] is not None:
                    msg.additionalFootPoseTrajectory.append(swing_trajectories[i])
                else:
                    msg.additionalFootPoseTrajectory.append(footPoses6D())
        
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
    
    def load_config(self):
        """从配置文件加载offset参数"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    
                self.input_tag_id = config.get('tag_id', 0)
                self.input_tag_offset_x = config.get('offset_x', self.default_offset_x)
                self.input_tag_offset_y = config.get('offset_y', self.default_offset_y)
                self.input_tag_offset_yaw = config.get('offset_yaw', self.default_offset_yaw)
                self.offset_set = config.get('offset_set', False)
                
                print("\n" + "="*50)
                print("✓ 已从配置文件加载 offset 参数:")
                print(f"  Tag ID: {self.input_tag_id}")
                print(f"  Offset X: {self.input_tag_offset_x:.3f}m")
                print(f"  Offset Y: {self.input_tag_offset_y:.3f}m")
                print(f"  Offset Yaw: {self.input_tag_offset_yaw:.3f}度")
                print("="*50)
            else:
                print("\n" + "="*50)
                print("未找到配置文件，使用默认 offset 参数:")
                print(f"  Offset X: {self.input_tag_offset_x:.3f}m")
                print(f"  Offset Y: {self.input_tag_offset_y:.3f}m")
                print(f"  Offset Yaw: {self.input_tag_offset_yaw:.3f}度")
                print("="*50)
        except Exception as e:
            print(f"加载配置文件失败: {e}")
            print("使用默认 offset 参数")
    
    def save_config(self):
        """保存offset参数到配置文件"""
        try:
            config = {
                'tag_id': self.input_tag_id,
                'offset_x': self.input_tag_offset_x,
                'offset_y': self.input_tag_offset_y,
                'offset_yaw': self.input_tag_offset_yaw,
                'offset_set': self.offset_set
            }
            
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=4, ensure_ascii=False)
            
            print(f"✓ 已保存 offset 参数到配置文件: {self.config_file}")
            return True
        except Exception as e:
            print(f"保存配置文件失败: {e}")
            return False
        
    def align_stair(self):
        """对齐楼梯"""
        rospy.wait_for_service('stair_alignment')
        try:
            align_stair_service = rospy.ServiceProxy('stair_alignment', stairAlignmentSrv)
            req = stairAlignmentSrvRequest()
            req.tag_id = self.input_tag_id
            req.offset_x = self.input_tag_offset_x
            req.offset_y = self.input_tag_offset_y
            # 将度转换为弧度 (服务需要弧度)
            req.offset_yaw = 0.0

            resp = align_stair_service(req)
            if resp.result:
                print(f"楼梯对齐成功: {resp.message}")
                return True
            else:
                print(f"楼梯对齐失败: {resp.message}")
                return False
        except rospy.ServiceException as e:
            print(f"楼梯对齐服务调用失败: {e}")
            return False

    def get_stair_offset(self, target_tag_id=None):
        """获取楼梯偏置，通过 /robot_tag_info 话题获取机器人当前的 x, y 位置并取反
        
        Args:
            target_tag_id: 目标tag id，如果指定则只查找该tag，直到找到为止
        """
        try:
            if target_tag_id is not None:
                # 持续尝试识别指定的tag，直到识别到为止
                print(f"正在持续识别目标 Tag ID: {target_tag_id}...")
                print("按 Ctrl+C 可取消识别")
                
                while not rospy.is_shutdown():
                    try:
                        # 等待从话题 "/robot_tag_info" 接收到 AprilTagDetectionArray 消息
                        msg = rospy.wait_for_message("/robot_tag_info", AprilTagDetectionArray, timeout=2)
                        
                        if not msg.detections:
                            print(".", end="", flush=True)  # 显示等待进度
                            continue
                        
                        # 查找目标tag
                        target_detection = None
                        for detection in msg.detections:
                            if detection.id[0] == target_tag_id:
                                target_detection = detection
                                break
                        
                        if target_detection is None:
                            print(".", end="", flush=True)  # 显示等待进度
                            continue
                        
                        # 找到目标tag
                        pos = target_detection.pose.pose.pose.position
                        offset_x = -pos.x
                        offset_y = -pos.y
                        
                        print(f"\n✓ 成功识别到 Tag ID: {target_tag_id}")
                        print(f"获取到机器人偏置: x={offset_x:.3f}m, y={offset_y:.3f}m")
                        return (target_tag_id, offset_x, offset_y)
                        
                    except rospy.ROSException:
                        print(".", end="", flush=True)  # 显示等待进度
                        continue
                        
            else:
                # 原有逻辑：显示所有检测到的tag供用户选择
                print("正在获取 AprilTag 信息...")
                msg = rospy.wait_for_message("/robot_tag_info", AprilTagDetectionArray, timeout=5)
                
                if not msg.detections:
                    print("未检测到任何 AprilTag")
                    return (None, 0.0, 0.0)
                
                # 显示所有检测到的 tag
                print("\n" + "="*50)
                print("检测到以下 AprilTag:")
                print("="*50)
                for i, detection in enumerate(msg.detections):
                    tag_id = detection.id[0]
                    pos = detection.pose.pose.pose.position
                    print(f"{i + 1}. Tag ID: {tag_id}, 位置: x={pos.x:.3f}m, y={pos.y:.3f}m, z={pos.z:.3f}m")
                print("="*50)
                
                # 让用户选择使用哪个 tag
                while True:
                    try:
                        choice = input(f"\n请选择要使用的 Tag (1-{len(msg.detections)}): ").strip()
                        choice_idx = int(choice) - 1
                        
                        if 0 <= choice_idx < len(msg.detections):
                            selected_detection = msg.detections[choice_idx]
                            selected_id = selected_detection.id[0]
                            pos = selected_detection.pose.pose.pose.position
                            
                            # 返回 tag_id, x, y 的取反值（作为偏置）
                            offset_x = -pos.x
                            offset_y = -pos.y
                            print(f"\n已选择 Tag ID: {selected_id}")
                            print(f"获取到机器人偏置: x={offset_x:.3f}m, y={offset_y:.3f}m")
                            return (selected_id, offset_x, offset_y)
                        else:
                            print(f"无效选择，请输入 1-{len(msg.detections)}")
                    except ValueError:
                        print("请输入有效的数字")
                    except KeyboardInterrupt:
                        print("\n用户取消操作")
                        return (None, 0.0, 0.0)
                        
        except KeyboardInterrupt:
            print("\n用户取消识别")
            return (None, 0.0, 0.0)
        except rospy.ROSException as e:
            print(f"\n获取楼梯偏置失败: {e}")
            return (None, 0.0, 0.0)
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
    
    # 楼梯参数
    STAIR_HEIGHT = 0.08
    STAIR_LENGTH = 0.25
    STAIR_NUM = 4

    controller.stair_height = STAIR_HEIGHT
    controller.stair_length = STAIR_LENGTH

    # 脚尖距离楼梯的距离
    DISTANCE_INITIAL = 0.01
    DISTANCE_INITIAL_SIM = 0.03
    
    msg_6d_slope = None
    
    while not rospy.is_shutdown():
        try:
            # 获取用户选择
            command = get_user_input()
            
            if command == 'quit':
                print("退出程序...")
                break
            
            elif command == 'get_offset':
                # 首先询问用户选择默认偏移还是识别偏移
                print("\n" + "="*50)
                print("请选择对齐方式:")
                print(f"1. 使用当前配置偏移 (offset = {controller.input_tag_offset_x:.3f}, {controller.input_tag_offset_y:.3f})")
                print("2. 使用识别偏移 (通过 AprilTag 识别)")
                print("="*50)
                
                offset_choice = input("请选择 (1-2): ").strip()
                
                if offset_choice == '1':
                    # 使用当前配置的偏移（已经从配置文件加载）
                    # 询问 tag_id
                    try:
                        tag_id_input = input(f"请输入 tag_id (用于对齐，当前: {controller.input_tag_id}): ").strip()
                        if tag_id_input:  # 如果输入了新的tag_id
                            tag_id = int(tag_id_input)
                            controller.input_tag_id = tag_id
                        print(f"✓ 已设置 tag_id: {controller.input_tag_id}")
                        print(f"✓ 已设置 offset: x={controller.input_tag_offset_x:.3f}m, y={controller.input_tag_offset_y:.3f}m, yaw={controller.input_tag_offset_yaw:.3f}度")
                        controller.offset_set = True
                        
                        # 保存到配置文件
                        controller.save_config()
                        
                        # 询问是否立即执行对齐
                        do_align = input("\n是否立即执行楼梯对齐？(y/n): ").strip().lower()
                        if do_align == 'y' or do_align == 'yes':
                            print("正在执行楼梯对齐...")
                            if controller.align_stair():
                                print("✓ 楼梯对齐完成")
                            else:
                                print("✗ 楼梯对齐失败")
                        else:
                            print("✓ offset 已保存，可在后续上楼梯时使用")
                    except ValueError:
                        print("无效输入，tag_id 保持当前值")
                        controller.input_tag_id = controller.input_tag_id
                
                elif offset_choice == '2':
                    # 使用识别偏移
                    print("\n请选择识别方式:")
                    print("1. 自动识别（显示检测到的所有 tag 供选择）")
                    print("2. 指定目标 tag id（持续识别直到识别到为止）")
                    
                    detect_choice = input("请选择 (1-2): ").strip()
                    
                    if detect_choice == '1':
                        # 自动识别模式
                        tag_id, offset_x, offset_y = controller.get_stair_offset()
                        
                        if tag_id is not None and (offset_x != 0.0 or offset_y != 0.0):
                            # 询问是否应用这些 offset
                            print("\n" + "="*50)
                            apply_offset = input(f"是否应用此 offset (x={offset_x:.3f}m, y={offset_y:.3f}m)？(y/n): ").strip().lower()
                            
                            if apply_offset == 'y' or apply_offset == 'yes':
                                controller.input_tag_id = tag_id
                                controller.input_tag_offset_x = offset_x
                                controller.input_tag_offset_y = offset_y
                                controller.input_tag_offset_yaw = 0.0
                                print(f"✓ 已设置 tag_id: {tag_id}")
                                print(f"✓ 已设置 offset: x={offset_x:.3f}m, y={offset_y:.3f}m, yaw=0.0度")
                                controller.offset_set = True
                                
                                # 保存到配置文件
                                controller.save_config()
                                
                                # 询问是否立即执行对齐
                                do_align = input("\n是否立即执行楼梯对齐？(y/n): ").strip().lower()
                                if do_align == 'y' or do_align == 'yes':
                                    print("正在执行楼梯对齐...")
                                    if controller.align_stair():
                                        print("✓ 楼梯对齐完成")
                                    else:
                                        print("✗ 楼梯对齐失败")
                                else:
                                    print("✓ offset 已保存，可在后续上楼梯时使用")
                            else:
                                print("已取消应用 offset")
                        else:
                            print("未获取到有效的 AprilTag offset")
                    
                    elif detect_choice == '2':
                        # 指定目标 tag id 模式
                        try:
                            target_tag_id = int(input("\n请输入目标 tag_id: ").strip())
                            
                            # 持续识别直到识别到目标tag
                            tag_id, offset_x, offset_y = controller.get_stair_offset(target_tag_id)
                            
                            if tag_id is not None:
                                # 自动应用识别到的offset
                                controller.input_tag_id = tag_id
                                controller.input_tag_offset_x = offset_x
                                controller.input_tag_offset_y = offset_y
                                controller.input_tag_offset_yaw = 0.0
                                print(f"✓ 已设置 tag_id: {tag_id}")
                                print(f"✓ 已设置 offset: x={offset_x:.3f}m, y={offset_y:.3f}m, yaw=0.0度")
                                controller.offset_set = True
                                
                                # 保存到配置文件
                                controller.save_config()
                                
                                # 询问是否立即执行对齐
                                do_align = input("\n是否立即执行楼梯对齐？(y/n): ").strip().lower()
                                if do_align == 'y' or do_align == 'yes':
                                    print("正在执行楼梯对齐...")
                                    if controller.align_stair():
                                        print("✓ 楼梯对齐完成")
                                    else:
                                        print("✗ 楼梯对齐失败")
                                else:
                                    print("✓ offset 已保存，可在后续上楼梯时使用")
                            else:
                                print("识别失败或已取消")
                        except ValueError:
                            print("无效输入，请输入有效的 tag_id")
                    else:
                        print("无效选择")
                else:
                    print("无效选择")
                
                print("="*50)
            
            elif command == 'stair' or command == 'stair_sim':
                # 显示当前 offset 状态
                if controller.offset_set:
                    print(f"\n当前已设置 offset: x={controller.input_tag_offset_x:.3f}m, "
                          f"y={controller.input_tag_offset_y:.3f}m, "
                          f"yaw={controller.input_tag_offset_yaw:.1f}度")
                else:
                    print("\n当前未设置 offset（使用默认值 0）")
                
                # 生成楼梯轨迹
                if command == 'stair':
                    msg_6d_slope = controller.simple_up_stairs(STAIR_HEIGHT, STAIR_LENGTH, STAIR_NUM, DISTANCE_INITIAL)
                else:  # stair_sim
                    msg_6d_slope = controller.simple_up_stairs(STAIR_HEIGHT, STAIR_LENGTH, STAIR_NUM, DISTANCE_INITIAL_SIM)
                
                # 发布轨迹
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
