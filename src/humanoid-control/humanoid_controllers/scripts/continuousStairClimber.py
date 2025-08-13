#!/usr/bin/env python3

import rospy
import numpy as np
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories, footPoses
import argparse
from std_srvs.srv import SetBool, SetBoolRequest
import time
from trajectory_interpolator import TrajectoryInterpolator

def parse_args():
    parser = argparse.ArgumentParser(description='Continuous stair climbing')
    parser.add_argument('--step_height', type=float, default=0.08, help='Step height (default: 0.08m)')
    parser.add_argument('--step_length', type=float, default=0.28, help='Step length (default: 0.28m)')
    parser.add_argument('--foot_width', type=float, default=0.108535, help='Foot width (default: 0.108535m)')
    parser.add_argument('--stand_height', type=float, default=0.0, help='Stand height offset (default: 0.0m)')
    parser.add_argument('--trajectory_method', type=str, default='trigonometric_quintic', 
                       choices=['trigonometric_quintic', 'spline'], 
                       help='Trajectory method: trigonometric_quintic (new) or spline (original)')
    parser.add_argument('--down_step_length', type=float, default=None, help='Down-stairs step length (default: use step_length)')
    return parser.parse_args()

STAND_HEIGHT = 0.0
DOWN_STEP_LENGTH = None

def set_pitch_limit(enable):
    """设置基座俯仰角限制"""
    print(f"设置pitch限制: {enable}")
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

class ContinuousStairClimber:
    def __init__(self):
        # 时间参数设置
        self.dt = 1  # 上下楼梯的步态周期
        self.ss_time = 0.6  # 上下楼梯的支撑迈步时间（较慢，确保稳定性）
        self.walk_dt = 0.6  # 前进/转弯的步态周期（更快，提高效率）
        self.walk_ss_time = 0.4  # 前进/转弯的支撑迈步时间（更快，提高效率）
        
        # 几何参数设置
        self.foot_width = 0.108535  # 脚宽度
        self.step_height = 0.08  # 台阶高度
        self.step_length = 0.28  # 上楼梯的台阶长度
        self.down_step_length = 0.28  # 下楼梯的迈步距离（独立参数）
        
        # 状态变量
        self.total_step = 0  # 总步数
        self.is_left_foot = False  # 当前是否为左脚
        self.trajectory_method = "trigonometric_quintic"  # 轨迹方法：trigonometric_quintic, spline
        self.last_action = None  # 最近一次执行的动作类型：up_stairs/down_stairs/forward/turn
        
        # 创建轨迹插值器
        self.interpolator = TrajectoryInterpolator()
        
        # 全局躯干和脚位置（世界坐标系）
        self.global_torso_pos = [0.0, 0.0, 0.0]  # 全局躯干位置
        self.global_left_foot = [0.0, self.foot_width, 0.0]  # 全局左脚位置
        self.global_right_foot = [0.0, -self.foot_width, 0.0]  # 全局右脚位置
        self.global_height = 0.0  # 全局高度
        
        # 全局基坐标系信息
        self.global_base_origin = [0.0, 0.0, 0.0]  # 全局基坐标系原点
        self.global_base_yaw = 0.0  # 全局基坐标系朝向
        
        # 保存当前状态
        self.current_torso_pos = [0.0, 0.0, 0.0]  # 当前躯干位置
        self.current_foot_pos = [0.0, 0.0, STAND_HEIGHT]  # 当前脚位置
        self.current_yaw = 0.0  # 当前朝向
        self.current_height = STAND_HEIGHT  # 当前高度
        self.prev_left_foot = [0.0, self.foot_width, self.current_height, 0.0]  # 前一次左脚位置
        self.prev_right_foot = [0.0, -self.foot_width, self.current_height, 0.0]  # 前一次右脚位置
        
        # 轨迹发布器
        self.traj_pub = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', 
                                       footPoseTargetTrajectories, queue_size=1)
        
        # 等待发布器准备就绪
        rospy.sleep(0.5)
        
    def generate_steps(self, torso_pos, torso_yaw, foot_height = 0):
        # 根据躯干位置计算落脚点
        l_foot_bias = np.array([0, self.foot_width, -torso_pos[2] + foot_height])
        r_foot_bias = np.array([0, -self.foot_width, -torso_pos[2] + foot_height])
        R_z = np.array([
            [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
            [np.sin(torso_yaw), np.cos(torso_yaw), 0],
            [0, 0, 1]
        ])
        l_foot = torso_pos + R_z.dot(l_foot_bias)
        r_foot = torso_pos + R_z.dot(r_foot_bias)
        return l_foot, r_foot
    
    def generate_steps_4d(self, torso_pos, torso_yaw, foot_height=0):
        """根据躯干位置计算落脚点（4D版本）"""
        l_foot_bias = np.array([0, self.foot_width, -torso_pos[2] + foot_height])
        r_foot_bias = np.array([0, -self.foot_width, -torso_pos[2] + foot_height])
        R_z = np.array([
            [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
            [np.sin(torso_yaw), np.cos(torso_yaw), 0],
            [0, 0, 1]
        ])
        l_foot = torso_pos + R_z.dot(l_foot_bias)
        r_foot = torso_pos + R_z.dot(r_foot_bias)
        return l_foot, r_foot   
    
    def update_current_state(self, time_traj, foot_idx_traj, foot_traj, torso_traj):
        """更新当前状态，基于最后一次轨迹，叠加更新全局坐标"""
        if len(torso_traj) > 0:
            # 获取最后一步结束时的状态（支撑相结束时的状态）
            final_torso_pos = torso_traj[-1][0:3].copy()
            final_yaw = float(torso_traj[-1][3])
            final_foot_pos = foot_traj[-1][0:3].copy()
            final_height = final_foot_pos[2]
            
            # 叠加更新全局躯干位置（世界坐标系）
            self.global_torso_pos = final_torso_pos.copy()
            
            # 仅当是上下楼梯动作时更新全局高度
            if self.last_action in ['up_stairs', 'down_stairs']:
                # 上楼：增加高度；下楼：降低高度
                if self.last_action == 'up_stairs':
                    # 上楼梯：final_height是局部高度，需要加上当前的global_height得到世界高度
                    self.global_height = final_height + self.global_height
                elif self.last_action == 'down_stairs':
                    # 下楼梯：final_height是局部高度，需要加上当前的global_height得到世界高度
                    self.global_height = final_height + self.global_height
            # 其他动作（前进/转身）不改变global_height
            
            self.global_base_yaw = final_yaw
            self.global_base_origin = final_torso_pos.copy()
            
            # 更新当前状态为新的基坐标系
            self.current_torso_pos = [0.0, 0.0, 0.0]
            self.current_yaw = 0.0
            self.current_foot_pos = [0.0, 0.0, final_height - final_torso_pos[2]]
            self.current_height = final_height - final_torso_pos[2]
            
            # 更新全局脚位置（世界坐标系）
            if len(foot_traj) >= 2:
                # 找到摆动相的脚位置
                for i in range(len(foot_idx_traj)):
                    if foot_idx_traj[i] in [0, 1]:
                        swing_foot_idx = foot_idx_traj[i]
                        swing_foot_pos = foot_traj[i][:3].copy()
                        if swing_foot_idx == 0:
                            self.global_left_foot = swing_foot_pos
                        elif swing_foot_idx == 1:
                            self.global_right_foot = swing_foot_pos
                        break
            
            # 更新前一次左右脚位置
            if len(foot_traj) >= 2:
                for i in range(len(foot_idx_traj)-1, -1, -1):
                    if foot_idx_traj[i] in [0, 1]:
                        last_swing_idx = i
                        if foot_idx_traj[last_swing_idx] == 0:
                            self.prev_left_foot = foot_traj[last_swing_idx].copy()
                            for j in range(last_swing_idx-1, -1, -1):
                                if foot_idx_traj[j] == 1:
                                    self.prev_right_foot = foot_traj[j].copy()
                                    break
                            else:
                                self.prev_right_foot = [0.0, -0.108535, 0.0, 0.0]
                        elif foot_idx_traj[last_swing_idx] == 1:
                            self.prev_right_foot = foot_traj[last_swing_idx].copy()
                            for j in range(last_swing_idx-1, -1, -1):
                                if foot_idx_traj[j] == 0:
                                    self.prev_left_foot = foot_traj[j].copy()
                                    break
                            else:
                                self.prev_left_foot = [0.0, 0.108535, 0.0, 0.0]
                        break

    def plan_up_stairs(self, num_steps=5, time_traj=None, foot_idx_traj=None, foot_traj=None, torso_traj=None, swing_trajectories=None):
        """连续上楼梯规划"""
        if time_traj is None:
            time_traj = []
        if foot_idx_traj is None:
            foot_idx_traj = []
        if foot_traj is None:
            foot_traj = []
        if torso_traj is None:
            torso_traj = []
        if swing_trajectories is None:
            swing_trajectories = []
            

        
        torso_yaw = 0.0
            
        # 获取最后一个轨迹点作为起始位置
        start_foot_pos_x = 0.0
        start_foot_pos_z = STAND_HEIGHT
        if len(torso_traj) > 0:
            current_torso_pos = np.array(torso_traj[-1][0:3])
            current_foot_pos = np.array(foot_traj[-1][0:3])
            start_foot_pos_x = current_foot_pos[0]
            torso_yaw = torso_traj[-1][3]
            start_foot_pos_z = current_foot_pos[2]
        else:
            current_torso_pos = np.array([0.0, 0.0, 0.0])
            current_foot_pos = np.array([0.0, 0.0, STAND_HEIGHT])

        # 初始位置
        torso_height_offset = 0.02  # 躯干高度偏移
        current_torso_pos[2] += torso_height_offset
        offset_x = [0.02, 0.03, 0.0, 0.0, 0.0]
        
        # 记录前一次的左右脚位置
        prev_left_foot = [start_foot_pos_x, 0.1, start_foot_pos_z, torso_yaw]
        prev_right_foot = [start_foot_pos_x, -0.1, start_foot_pos_z, torso_yaw]
        initial_index = len(foot_traj)
        
        # 为每一步生成落脚点
        for step in range(num_steps):
            # 更新时间
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + self.dt)
            
            # 左右脚交替
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)
            
            # 计算躯干位置
            if step == 0:
                current_foot_pos[0] = current_torso_pos[0] + self.step_length  # 脚掌相对躯干前移
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] = self.step_height + STAND_HEIGHT  # 脚掌高度
                current_torso_pos[0] += self.step_length/3
                
            elif step == num_steps - 1: # 最后一步
                current_torso_pos[0] = current_foot_pos[0] # 最后一步躯干x在双脚上方
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_torso_pos[2] += self.step_height 
            else:
                current_torso_pos[0] += self.step_length  # 向前移动
                current_torso_pos[2] += self.step_height  # 向上移动
            
                # 计算落脚点位置
                current_foot_pos[0] = current_torso_pos[0] + self.step_length/2  # 脚掌相对躯干前移
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] += self.step_height
                
            if step < len(offset_x) and not step == num_steps - 1:    # 脚掌偏移
                current_foot_pos[0] += offset_x[step]
                
            # 记录当前脚的位置
            current_foot = [*current_foot_pos, torso_yaw]
            
            # 生成腾空相轨迹
            if prev_left_foot is not None and prev_right_foot is not None:  # 从第二步开始生成腾空相轨迹
                prev_foot = prev_left_foot if self.is_left_foot else prev_right_foot
                swing_traj = self.plan_swing_phase(prev_foot, current_foot, swing_height=0.12, down_stairs=False, is_first_step=(step == 0 or step == num_steps - 1))
                swing_trajectories.append(swing_traj)
            else:
                swing_trajectories.append(None)
            
            # 更新前一次的脚位置
            if self.is_left_foot:
                prev_left_foot = current_foot
            else:
                prev_right_foot = current_foot
            
            # 添加轨迹点
            foot_traj.append(current_foot)
            torso_traj.append([*current_torso_pos, torso_yaw])
            
            last_torso_pose = torso_traj[-1].copy()
            last_foot_pose = foot_traj[-1].copy()
            
            # 添加支撑相
            if step != num_steps - 1:
                pass
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0] - self.step_length*0.0
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses())
            else: # 最后一步站立恢复站直
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0]
                last_torso_pose[2] = last_foot_pose[2] - STAND_HEIGHT
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses())
    
        # 处理旋转偏移量
        if initial_index > 0:
            init_torso_pos = torso_traj[initial_index-1]
            init_foot_pos = foot_traj[initial_index-1]
            for i in range(initial_index, len(foot_traj)):
                diff_yaw = torso_traj[i][3]
                R_z = np.array([
                    [np.cos(diff_yaw), -np.sin(diff_yaw), 0],
                    [np.sin(diff_yaw), np.cos(diff_yaw), 0],
                    [0, 0, 1]
                ])
                d_torso_pos = torso_traj[i][0:3] - init_torso_pos[0:3]
                torso_traj[i][0:2] = (R_z.dot(d_torso_pos) + init_torso_pos[0:3])[:2]   
                
                d_foot_pos = foot_traj[i][0:3] - init_torso_pos[0:3] # 计算相对于躯干位置的偏移量
                foot_traj[i][0:2] = (R_z.dot(d_foot_pos) + init_torso_pos[0:3])[:2]
                if swing_trajectories[i] is not None:# 旋转腾空相规划
                    for j in range(len(swing_trajectories[i].data)):
                        d_foot_pos = swing_trajectories[i].data[j].footPose[0:3] - init_torso_pos[0:3]
                        swing_trajectories[i].data[j].footPose[0:2] = (R_z.dot(d_foot_pos) + init_torso_pos[0:3])[:2]
                        
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories

    def plan_down_stairs(self, num_steps=5, time_traj=None, foot_idx_traj=None, foot_traj=None, torso_traj=None, swing_trajectories=None, stair_offset=0.02):
        """连续下楼梯规划"""
        if time_traj is None:
            time_traj = []
        if foot_idx_traj is None:
            foot_idx_traj = []
        if foot_traj is None:
            foot_traj = []
        if torso_traj is None:
            torso_traj = []
        if swing_trajectories is None:
            swing_trajectories = []
        
        torso_yaw = 0.0
            
        # 获取最后一个轨迹点作为起始位置
        start_foot_pos_x = 0.0
        start_foot_pos_z = STAND_HEIGHT
        if len(torso_traj) > 0:
            current_torso_pos = np.array(torso_traj[-1][0:3])
            current_foot_pos = np.array(foot_traj[-1][0:3])
            start_foot_pos_x = current_foot_pos[0]
            torso_yaw = torso_traj[-1][3]
            start_foot_pos_z = current_foot_pos[2]
        else:
            current_torso_pos = np.array([0.0, 0.0, 0.0])
            current_foot_pos = np.array([0.0, 0.0, STAND_HEIGHT])

        # 初始位置（下楼梯时从高处开始）
        torso_height_offset = -0.1  # 躯干高度偏移
        current_torso_pos[2] += torso_height_offset
        # 基础offset数组，后续会加上stair_offset
        base_offset_x = [-0.02, 0.00, -0.02, 0.0, 0.03]
        # 所有offset都加上离楼梯的偏置距离
        offset_x = [offset + stair_offset for offset in base_offset_x]
        
        # 记录前一次的左右脚位置
        prev_left_foot = [start_foot_pos_x, 0.1, start_foot_pos_z, torso_yaw]
        prev_right_foot = [start_foot_pos_x, -0.1, start_foot_pos_z, torso_yaw]
        initial_index = len(foot_traj)
        
        # 为每一步生成落脚点（下楼梯逻辑）
        for step in range(num_steps):
            # 更新时间
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + self.dt)
            
            # 左右脚交替
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)
            
            # 计算躯干位置（下楼梯逻辑）
            if step == 0:
                # 第一步：躯干稍微前移，脚掌下到第一个台阶
                current_foot_pos[0] = current_torso_pos[0] + self.down_step_length  # 脚掌相对躯干前移
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] = start_foot_pos_z - self.step_height  # 脚掌下降到第一个台阶
                current_torso_pos[0] += self.down_step_length/3
                
            elif step == num_steps - 1: # 最后一步
                # 最后一步：躯干移动到双脚上方，脚掌下到地面（发布时叠加global_height，因此此处设为 -global_height 以得到世界高度0）
                current_torso_pos[0] = current_foot_pos[0] # 最后一步躯干x在双脚上方
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] = STAND_HEIGHT - self.global_height  # 发布阶段加上global_height后=0
                current_torso_pos[2] -= self.step_height  # 躯干下降
            else:
                # 中间步骤：躯干前移并下降，脚掌下到下一个台阶
                current_torso_pos[0] += self.down_step_length  # 向前移动
                current_torso_pos[2] -= self.step_height  # 向下移动
            
                # 计算落脚点位置
                current_foot_pos[0] = current_torso_pos[0] + self.down_step_length/2  # 脚掌相对躯干前移
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] -= self.step_height  # 脚掌下降到下一个台阶
                
            if step < len(offset_x) and not step == num_steps - 1:    # 脚掌偏移
                current_foot_pos[0] += offset_x[step]
                
            # 记录当前脚的位置
            current_foot = [*current_foot_pos, torso_yaw]
            
            # 生成腾空相轨迹（下楼梯时使用down_stairs=True）
            if prev_left_foot is not None and prev_right_foot is not None:  # 从第二步开始生成腾空相轨迹
                prev_foot = prev_left_foot if self.is_left_foot else prev_right_foot
                swing_traj = self.plan_swing_phase(prev_foot, current_foot, swing_height=0.12, down_stairs=True, is_first_step=(step == 0 or step == num_steps - 1))
                swing_trajectories.append(swing_traj)
            else:
                swing_trajectories.append(None)
            
            # 更新前一次的脚位置
            if self.is_left_foot:
                prev_left_foot = current_foot
            else:
                prev_right_foot = current_foot
            
            # 添加轨迹点
            foot_traj.append(current_foot)
            torso_traj.append([*current_torso_pos, torso_yaw])
            
            last_torso_pose = torso_traj[-1].copy()
            last_foot_pose = foot_traj[-1].copy()
            
            # 添加支撑相
            if step != num_steps - 1:
                pass
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0] - self.down_step_length*0.0
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses())
            else: # 最后一步站立恢复站直
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0]
                last_torso_pose[2] = last_foot_pose[2] - STAND_HEIGHT  # 使用叠加计算，躯干高度 = 脚高度 - STAND_HEIGHT
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses())
    
        # 处理旋转偏移量
        if initial_index > 0:
            init_torso_pos = torso_traj[initial_index-1]
            init_foot_pos = foot_traj[initial_index-1]
            for i in range(initial_index, len(foot_traj)):
                diff_yaw = torso_traj[i][3]
                R_z = np.array([
                    [np.cos(diff_yaw), -np.sin(diff_yaw), 0],
                    [np.sin(diff_yaw), np.cos(diff_yaw), 0],
                    [0, 0, 1]
                ])
                d_torso_pos = torso_traj[i][0:3] - init_torso_pos[0:3]
                torso_traj[i][0:2] = (R_z.dot(d_torso_pos) + init_torso_pos[0:3])[:2]   
                
                d_foot_pos = foot_traj[i][0:3] - init_torso_pos[0:3] # 计算相对于躯干位置的偏移量
                foot_traj[i][0:2] = (R_z.dot(d_foot_pos) + init_torso_pos[0:3])[:2]
                if swing_trajectories[i] is not None:# 旋转腾空相规划
                    for j in range(len(swing_trajectories[i].data)):
                        d_foot_pos = swing_trajectories[i].data[j].footPose[0:3] - init_torso_pos[0:3]
                        swing_trajectories[i].data[j].footPose[0:2] = (R_z.dot(d_foot_pos) + init_torso_pos[0:3])[:2]
                        
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories
    
    def plan_swing_phase(self, prev_foot_pose, next_foot_pose, swing_height=0.10, plot=False, down_stairs=False, is_first_step = False, use_cycloid=True):
        """使用轨迹插值器规划腾空相的轨迹"""
        # 使用轨迹插值器进行插值
        return self.interpolator.interpolate_trajectory(
            prev_foot_pose=prev_foot_pose,
            next_foot_pose=next_foot_pose,
            swing_height=swing_height,
            method=self.trajectory_method,
            num_points=7,
            is_first_step=is_first_step,
            down_stairs=down_stairs,
            plot=plot
        )
    
    def plan_move_to_4d(self, dx=0.2, dy=0.0, dyaw=0.0, time_traj=None, foot_idx_traj=None, foot_traj=None, torso_traj=None, swing_trajectories=None, max_step_x=0.28, max_step_y=0.15, max_step_yaw=30.0):
        """
        规划4D移动到目标位置的轨迹（基于slope的6D版本改编）
        Args:
            dx: x方向目标位移
            dy: y方向目标位移
            dyaw: yaw方向目标角度(度)
            time_traj: 时间轨迹
            foot_idx_traj: 脚索引轨迹
            foot_traj: 脚4D姿态轨迹
            torso_traj: 躯干4D姿态轨迹
            swing_trajectories: 腾空相轨迹
            max_step_x: x方向最大步长
            max_step_y: y方向最大步长
            max_step_yaw: yaw方向最大步长(度)
        """
        # 初始化轨迹列表
        if time_traj is None:
            time_traj = []
        if foot_idx_traj is None:
            foot_idx_traj = []
        if foot_traj is None:
            foot_traj = []
        if torso_traj is None:
            torso_traj = []
        if swing_trajectories is None:
            swing_trajectories = []
        
        # 获取最后一个轨迹点作为起始位置
        if len(torso_traj) > 0:
            current_torso_pos = np.array(torso_traj[-1])
            current_foot_pos = np.array(foot_traj[-1])  # 获取完整的4D脚位置
            current_yaw = current_torso_pos[3]
            current_height = current_foot_pos[2]
            R_z = np.array([
                [np.cos(current_yaw), -np.sin(current_yaw), 0],
                [np.sin(current_yaw), np.cos(current_yaw), 0],
                [0, 0, 1]
            ])
            dx, dy, dz = R_z.dot(np.array([dx, dy, 0]))
        else:
            # 初始化为4D格式：[x, y, z, yaw]，使用相对高度（全局高度在publish时添加）
            current_height = STAND_HEIGHT
            current_torso_pos = np.array([0.0, 0.0, 0.0, 0.0])
            current_foot_pos = np.array([0.0, 0.0, current_height, 0.0])
            current_yaw = 0.0

        # 计算需要的步数
        num_steps_x = max(1, int(np.ceil(abs(dx) / max_step_x)))
        num_steps_y = max(1, int(np.ceil(abs(dy) / max_step_y)))
        num_steps_yaw = max(1, int(np.ceil(abs(dyaw) / max_step_yaw)))
        num_steps = max(num_steps_x, num_steps_y, num_steps_yaw)

        # 计算实际步长
        actual_step_x = dx / num_steps
        actual_step_y = dy / num_steps
        actual_step_yaw = dyaw / num_steps
        
        # 记录初始yaw角，用于计算目标yaw角
        initial_yaw = current_torso_pos[3]
        target_yaw = initial_yaw + np.radians(dyaw)
        
        # 设置起始脚（转身时优先选择左脚）
        if dyaw > 0:
            self.is_left_foot = True
        
        # 记录开始时的轨迹长度
        start_traj_len = len(foot_traj)
        num_steps += 1  # 第一步和最后一步是半步
        # 使用类变量中的时间参数
        walk_dt = self.walk_dt
        walk_ss_time = self.walk_ss_time



        for i in range(num_steps):
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + walk_dt)
            
            # 左右脚交替
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)
            
            # 计算当前步的目标yaw角（线性插值，只在有转向需求时更新）
            if abs(dyaw) > 0.1:  # 只有在需要显著转向时才更新yaw角
                progress = (i + 1) / num_steps
                current_torso_yaw = initial_yaw + progress * np.radians(dyaw)
            else:
                current_torso_yaw = initial_yaw  # 直线行走时保持yaw角不变
            
            # 更新躯干位置
            if i == 0:
                current_torso_pos[0] += actual_step_x/2
                current_torso_pos[1] += actual_step_y/2
                current_torso_pos[3] = current_torso_yaw
                # 根据当前yaw角度计算落脚点偏移
                desire_torso_pos = [current_torso_pos[0]+actual_step_x/2, current_torso_pos[1]+actual_step_y/2, current_torso_pos[2]]
                lf_foot, rf_foot = self.generate_steps_4d(desire_torso_pos, current_torso_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            elif i == num_steps - 1:
                current_torso_pos[0] += actual_step_x/2
                current_torso_pos[1] += actual_step_y/2
                current_torso_pos[3] = target_yaw  # 最后一步确保达到目标yaw角
                # 根据当前yaw角度计算落脚点偏移
                lf_foot, rf_foot = self.generate_steps_4d(current_torso_pos[:3], current_torso_pos[3], current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            else:
                current_torso_pos[0] += actual_step_x
                current_torso_pos[1] += actual_step_y
                current_torso_pos[3] = current_torso_yaw
                # 根据当前yaw角度计算落脚点偏移
                desire_torso_pos = [current_torso_pos[0]+actual_step_x/2, current_torso_pos[1]+actual_step_y/2, current_torso_pos[2]]
                lf_foot, rf_foot = self.generate_steps_4d(desire_torso_pos, current_torso_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            
            # 记录当前脚的位置（4D格式）
            current_foot = [current_foot_pos[0], current_foot_pos[1], current_foot_pos[2], current_torso_pos[3]]
            
            # 4D版本不使用腾空相轨迹（与slope保持一致）
            swing_trajectories.append(None)
            
            # 添加轨迹点
            foot_traj.append(current_foot)
            torso_traj.append([current_torso_pos[0], current_torso_pos[1], current_torso_pos[2], current_torso_pos[3]])

            # 添加支撑相
            time_traj.append(time_traj[-1] + walk_ss_time)
            foot_idx_traj.append(2)
            foot_traj.append(foot_traj[-1].copy())
            torso_traj.append(torso_traj[-1].copy())
            swing_trajectories.append(footPoses())  # 4D版本使用footPoses
        
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories
    
    def plan_forward_movement(self, distance=0.2):
        """规划前进/后退轨迹（使用plan_move_to_4d实现）"""
        # 使用plan_move_to_4d方法实现前进/后退，dx=distance, dy=0, dyaw=0
        # distance为正数表示前进，负数表示后退
        time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = self.plan_move_to_4d(
            dx=distance, dy=0.0, dyaw=0.0, 
            max_step_x=0.28, max_step_y=0.15, max_step_yaw=30.0
        )
        
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories
    
    def plan_turn_movement(self, turn_angle=180.0):
        """规划转身轨迹（使用plan_move_to_4d实现）"""
        # 使用plan_move_to_4d方法实现转身，dx=0, dy=0, dyaw=turn_angle（角度）
        time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = self.plan_move_to_4d(
            dx=0.0, dy=0.0, dyaw=turn_angle,
            max_step_x=0.28, max_step_y=0.15, max_step_yaw=30.0
        )
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories
    
    def get_trajectory_method_info(self):
        """获取当前轨迹方法的详细信息"""
        if self.trajectory_method == "trigonometric_quintic":
            return "三角函数+五次多项式 (正弦函数 + 五次多项式落地)"
        elif self.trajectory_method == "spline":
            return "三次样条插值 (形状保持插值)"
        else:
            return f"未知方法: {self.trajectory_method}"
    
    def execute_up_stairs(self, num_steps=5):
        """执行连续上楼梯"""
        try:
            self.last_action = 'up_stairs'
            # 计算并执行连续上楼梯
            time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = self.plan_up_stairs(
                num_steps=num_steps,
                time_traj=[], 
                foot_idx_traj=[], 
                foot_traj=[], 
                torso_traj=[], 
                swing_trajectories=[]
            )
            
            # 发布轨迹
            success = self.publish_trajectory(time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
            
            if success:
                # 更新全局状态
                self.update_current_state(time_traj, foot_idx_traj, foot_traj, torso_traj)
            
            return success
            
        except Exception as e:
            print(f"连续上楼梯执行失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def execute_down_stairs(self, num_steps=5, stair_offset=0.0):
        """执行连续下楼梯"""
        try:
            self.last_action = 'down_stairs'
            # 计算并执行连续下楼梯
            time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = self.plan_down_stairs(
                num_steps=num_steps,
                stair_offset=stair_offset,
                time_traj=[], 
                foot_idx_traj=[], 
                foot_traj=[], 
                torso_traj=[], 
                swing_trajectories=[]
            )
            
            # 发布轨迹
            success = self.publish_trajectory(time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
            
            if success:
                # 更新全局状态
                self.update_current_state(time_traj, foot_idx_traj, foot_traj, torso_traj)
            
            return success
            
        except Exception as e:
            print(f"连续下楼梯执行失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def execute_forward_movement(self, distance=0.2):
        """执行前进"""
        try:
            self.last_action = 'forward'
            # 计算并执行前进
            time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = self.plan_forward_movement(
                distance=distance
            )
            
            # 发布轨迹
            success = self.publish_trajectory(time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
            
            if success:
                # 更新全局状态
                self.update_current_state(time_traj, foot_idx_traj, foot_traj, torso_traj)
            
            return success
            
        except Exception as e:
            print(f"前进执行失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def execute_turn_movement(self, turn_angle=180.0):
        """执行转身"""
        try:
            self.last_action = 'turn'
            # 计算并执行转身
            time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = self.plan_turn_movement(
                turn_angle=turn_angle
            )
            
            # 发布轨迹
            success = self.publish_trajectory(time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
            
            if success:
                # 更新全局状态
                self.update_current_state(time_traj, foot_idx_traj, foot_traj, torso_traj)
            
            return success
            
        except Exception as e:
            print(f"转身执行失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def publish_trajectory(self, time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories=None):
        """发布4D轨迹"""
        try:
            # 等待发布器准备就绪
            rospy.sleep(0.1)
            
            msg = footPoseTargetTrajectories()
            msg.timeTrajectory = time_traj
            msg.footIndexTrajectory = foot_idx_traj
            msg.footPoseTrajectory = []
            msg.additionalFootPoseTrajectory = []
            msg.swingHeightTrajectory = []  # 初始化摆动高度轨迹
            
            # 创建完整的4D脚姿态轨迹
            print(f"当前全局高度: {self.global_height:.3f}m")
            for i in range(len(time_traj)):
                # 只给脚添加全局高度，躯干保持原高度
                foot_pose_4d_with_global_height = foot_traj[i].copy()
                foot_pose_4d_with_global_height[2] += self.global_height
                
                torso_pose_4d_with_global_height = torso_traj[i].copy()
                # 躯干不叠加全局高度，保持原来的相对高度
                
                foot_pose_msg = footPose()
                foot_pose_msg.footPose = foot_pose_4d_with_global_height  # 4D脚姿态 [x, y, z+global_height, yaw]
                foot_pose_msg.torsoPose = torso_pose_4d_with_global_height  # 4D躯干姿态 [x, y, z+global_height, yaw]
                msg.footPoseTrajectory.append(foot_pose_msg)
                
                # 打印发送的消息内容
                print(f"  消息点{i}: 脚4D={foot_pose_4d_with_global_height}, 躯干4D={torso_pose_4d_with_global_height}")
                
                # 处理腾空相轨迹（如果有的话）
                if swing_trajectories is not None and i < len(swing_trajectories):
                    swing_poses = footPoses()
                    if swing_trajectories[i] is not None:
                        # 对腾空相轨迹的每个点也添加全局高度
                        swing_with_global_height = footPoses()
                        swing_with_global_height.data = []
                        for swing_point in swing_trajectories[i].data:
                            # 手动复制footPose对象
                            swing_point_with_height = footPose()
                            swing_point_with_height.footPose = swing_point.footPose.copy()
                            swing_point_with_height.torsoPose = swing_point.torsoPose.copy()
                            swing_point_with_height.footPose[2] += self.global_height
                            swing_with_global_height.data.append(swing_point_with_height)
                        msg.additionalFootPoseTrajectory.append(swing_with_global_height)
                    else:
                        msg.additionalFootPoseTrajectory.append(footPoses())
                else:
                    msg.additionalFootPoseTrajectory.append(footPoses())  # 添加空的轨迹
            
            # 检查发布者状态
            if self.traj_pub.get_num_connections() == 0:
                print("警告: 没有订阅者连接到话题")
            
            self.traj_pub.publish(msg)
            
            # 等待消息发送完成
            rospy.sleep(0.2)
            
            return True
            
        except Exception as e:
            print(f"轨迹发布失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
def get_user_input():
    """获取用户输入"""
    print("\n=== 连续楼梯控制器 ===")
    print("请选择要执行的功能:")
    print("1. 连续上楼梯 (up_stairs)")
    print("2. 连续下楼梯 (down_stairs)")
    print("3. 前进/后退 (forward)")
    print("4. 转身 (turn)")
    print("5. 切换轨迹方法 (method)")
    print("6. 退出 (quit)")
    
    while True:
        choice = input("\n请输入选择 (1-6): ").strip()
        
        if choice == '1':
            return 'up_stairs'
        elif choice == '2':
            return 'down_stairs'
        elif choice == '3':
            return 'forward'
        elif choice == '4':
            return 'turn'
        elif choice == '5':
            return 'method'
        elif choice == '6':
            return 'quit'
        else:
            print("无效选择，请输入 1-6")

def main():
    # 初始化ROS节点
    rospy.init_node('continuous_stair_climber', anonymous=True)
    
    # 创建控制器
    climber = ContinuousStairClimber()
    
    while not rospy.is_shutdown():
        try:
            # 获取用户选择
            command = get_user_input()
            
            if command == 'quit':
                print("退出程序...")
                break
                
            elif command == 'method':
                print("当前轨迹方法:", climber.get_trajectory_method_info())
                print("请选择轨迹方法:")
                print("1. trigonometric_quintic (三角函数+五次多项式)")
                print("2. spline (三次样条插值)")
                
                try:
                    method_choice = input("请输入选择 (1-2): ").strip()
                    if method_choice == '1':
                        climber.trajectory_method = "trigonometric_quintic"
                        print("已切换到三角函数+五次多项式轨迹方法")
                    elif method_choice == '2':
                        climber.trajectory_method = "spline"
                        print("已切换到三次样条插值方法")
                    else:
                        print("无效选择，保持当前方法")
                except Exception as e:
                    print(f"切换失败: {e}")
                continue
            
            elif command == 'up_stairs':
                print(f"\n执行连续上楼梯")
                try:
                    num_steps_input = input("请输入迈步次数 (默认5步): ").strip()
                    if num_steps_input == "":
                        num_steps = 5
                    else:
                        num_steps = int(num_steps_input)
                    
                    if num_steps <= 0:
                        print("错误: 迈步次数必须为正数")
                        continue
                    
                    success = climber.execute_up_stairs(num_steps=num_steps)
                        
                except ValueError:
                    print("错误: 请输入有效的数字")
                    continue
                except Exception as e:
                    print(f"错误: {e}")
                    continue
            
            elif command == 'down_stairs':
                print(f"\n执行连续下楼梯")
                try:
                    # 设置离楼梯的偏置距离
                    offset_input = input("请输入离楼梯的偏置距离 (默认0.0m): ").strip()
                    if offset_input == "":
                        stair_offset = 0.0
                    else:
                        stair_offset = float(offset_input)
                    
                    num_steps_input = input("请输入迈步次数 (默认5步): ").strip()
                    if num_steps_input == "":
                        num_steps = 5
                    else:
                        num_steps = int(num_steps_input)
                    
                    if num_steps <= 0:
                        print("错误: 迈步次数必须为正数")
                        continue
                    
                    success = climber.execute_down_stairs(num_steps=num_steps, stair_offset=stair_offset)
                        
                except ValueError:
                    print("错误: 请输入有效的数字")
                    continue
                except Exception as e:
                    print(f"错误: {e}")
                    continue
            
            elif command == 'forward':
                print(f"\n执行前进/后退")
                try:
                    distance_input = input("请输入前进距离 (正数前进，负数后退，默认0.2m): ").strip()
                    if distance_input == "":
                        distance = 0.2
                    else:
                        distance = float(distance_input)
                    
                    # # 允许负距离（后退）
                    # if distance == 0:
                    #     print("错误: 距离不能为0")
                    #     continue
                    
                    # 显示移动方向
                    direction = "前进" if distance > 0 else "后退"
                    print(f"将执行{direction}，距离: {abs(distance):.2f}m")
                    
                    success = climber.execute_forward_movement(distance=distance)
                        
                except ValueError:
                    print("错误: 请输入有效的数字")
                    continue
                except Exception as e:
                    print(f"错误: {e}")
                    continue
            
            elif command == 'turn':
                print(f"\n执行转身")
                try:
                    angle_input = input("请输入转身角度(度，默认180): ").strip()
                    if angle_input == "":
                        angle = 180.0
                    else:
                        angle = float(angle_input)
                    
                    success = climber.execute_turn_movement(turn_angle=angle)
                        
                except ValueError:
                    print("错误: 请输入有效的数字")
                    continue
                except Exception as e:
                    print(f"错误: {e}")
                    continue
                
        except KeyboardInterrupt:
            print("\n用户中断执行")
            break
        except Exception as e:
            print(f"执行出错: {e}")
            import traceback
            traceback.print_exc()
            continue

if __name__ == '__main__':
    try:
        args = parse_args()
        STAND_HEIGHT = args.stand_height
        
        # 禁用俯仰角限制
        set_pitch_limit(False)
        
        # 进入主循环
        main()
        
    except rospy.ROSInterruptException:
        # 确保在发生异常时也重新启用俯仰角限制
        set_pitch_limit(True)
        pass
    except Exception as e:
        print(f"错误: {e}")
        set_pitch_limit(True) 