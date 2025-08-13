#!/usr/bin/env python3

import rospy
import numpy as np
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories, footPoses, footPose6D, footPose6DTargetTrajectories, footPoses6D
from scipy.interpolate import CubicSpline, PchipInterpolator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
from std_srvs.srv import SetBool, SetBoolRequest

def parse_args():
    parser = argparse.ArgumentParser(description='Stair climbing planner')
    parser.add_argument('--plot', action='store_true', help='Enable trajectory plotting')
    parser.add_argument('--initH', type=float, default=0.0, help='Stand height offset (default: 0.0)')
    parser.add_argument('--down_stairs', action='store_true', help='Down stairs')
    return parser.parse_args()

PLOT = False
STAND_HEIGHT = 0.0

def set_pitch_limit(enable):
    """
    设置基座俯仰角限制
    Args:
        enable: bool, True表示启用限制，False表示禁用限制
    """
    print(f"call set_pitch_limit:{enable}")
    rospy.wait_for_service('/humanoid/mpc/enable_base_pitch_limit')
    try:
        set_pitch_limit_service = rospy.ServiceProxy('/humanoid/mpc/enable_base_pitch_limit', SetBool)
        req = SetBoolRequest()
        req.data = enable
        resp = set_pitch_limit_service(req)
        if resp.success:
            rospy.loginfo(f"Successfully {'enabled' if enable else 'disabled'} pitch limit")
        else:
            rospy.logwarn(f"Failed to {'enable' if enable else 'disable'} pitch limit")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

class StairClimbingPlanner:
    def __init__(self):
        self.dt = 0.6  # 步态周期
        self.ss_time = 0.5
        self.foot_width = 0.10  # 宽
        self.step_height = 0.13  # 台阶高度
        self.step_length = 0.28  # 台阶长度,28,13
        self.total_step = 0  # 总步数
        self.is_left_foot = True
        self.foot_w = 0.12  # 半脚长
        
    def calculate_foot_tip_offset(self, pitch_angle):
        """
        计算脚尖掂起时，脚掌中心点相对于脚掌踩平时的xyz偏移量
        Args:
            pitch_angle: 脚尖掂起的角度（弧度，正值表示脚尖向上）
        Returns:
            [dx, dy, dz]: xyz方向的偏移量
        """
        # 假设脚掌绕脚尖旋转，脚尖位置固定
        # 脚掌中心点相对于脚尖的位置变化
        dx = self.foot_w * (1 - np.cos(pitch_angle))  # x方向偏移（向前）
        dy = 0.0  # y方向无偏移
        dz = self.foot_w * np.sin(pitch_angle)  # z方向偏移（向上）
        
        return [dx, dy, dz]
    
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
    
    def plan_move_to(self, dx=0.2, dy=0.0, dyaw=0.0, time_traj=None, foot_idx_traj=None, foot_traj=None, torso_traj=None, swing_trajectories=None, max_step_x=0.28, max_step_y=0.15, max_step_yaw=30.0):
        """
        规划移动到目标位置的轨迹
        Args:
            dx: x方向目标位移
            dy: y方向目标位移
            dyaw: yaw方向目标角度(度)
            time_traj: 时间轨迹
            foot_idx_traj: 脚索引轨迹
            foot_traj: 脚姿态轨迹
            torso_traj: 躯干姿态轨迹
            swing_trajectories: 腾空相轨迹
            max_step_x: x方向最大步长
            max_step_y: y方向最大步长
            max_step_yaw: yaw方向最大步长(度)
        """
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
        current_height = STAND_HEIGHT
        # 获取最后一个轨迹点作为起始位置
        if len(torso_traj) > 0:
            current_torso_pos = np.array(torso_traj[-1])
            current_foot_pos = np.array(foot_traj[-1][0:3])
            current_yaw = current_torso_pos[3]
            current_height = current_foot_pos[2]
            R_z = np.array([
                [np.cos(current_yaw), -np.sin(current_yaw), 0],
                [np.sin(current_yaw), np.cos(current_yaw), 0],
                [0, 0, 1]
            ])
            dx, dy, dz = R_z.dot(np.array([dx, dy, 0]))
            print("new dx, dy, dyaw", dx, dy, dyaw)
            
        else:
            current_torso_pos = np.array([0.0, 0.0, 0.0, 0.0])
            current_foot_pos = np.array([0.0, 0.0, STAND_HEIGHT])
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
        # is_left_foot = ((self.total_step - 1) % 2 == 0 or dyaw > 0)
        if dyaw > 0:
            self.is_left_foot = True
        # 记录开始时的轨迹长度
        start_traj_len = len(foot_traj)
        num_steps += 1 # 第一步和最后一步是半步
        walk_dt = 0.4
        walk_ss_time = 0.5

        for i in range(num_steps):
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + walk_dt)
            
            # 左右脚交替
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)
            # 更新躯干位置
            if i == 0:
                current_torso_pos[0] += actual_step_x/2
                current_torso_pos[1] += actual_step_y/2
                current_torso_pos[3] += np.radians(actual_step_yaw)
                # 根据当前yaw角度计算落脚点偏移
                current_yaw = current_torso_pos[3]
                desire_torso_pos = [current_torso_pos[0]+actual_step_x/2, current_torso_pos[1]+actual_step_y/2, current_torso_pos[2]]
                lf_foot, rf_foot = self.generate_steps(desire_torso_pos, current_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            # elif i == num_steps - 1 or (abs(dyaw)>0 and i == num_steps - 2):
            elif i == num_steps - 1 :
                current_torso_pos[0] += actual_step_x/2
                current_torso_pos[1] += actual_step_y/2
                # current_torso_pos[3] += np.radians(actual_step_yaw) if not (abs(dyaw)>0 and i == num_steps - 1) else 0
                current_torso_pos[3] += 0
                # 根据当前yaw角度计算落脚点偏移
                current_yaw = current_torso_pos[3]
                lf_foot, rf_foot = self.generate_steps(current_torso_pos[:3], current_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            else:
                current_torso_pos[0] += actual_step_x
                current_torso_pos[1] += actual_step_y
                current_torso_pos[3] += np.radians(actual_step_yaw)
                # 根据当前yaw角度计算落脚点偏移
                current_yaw = current_torso_pos[3]
                desire_torso_pos = [current_torso_pos[0]+actual_step_x/2, current_torso_pos[1]+actual_step_y/2, current_torso_pos[2]]
                lf_foot, rf_foot = self.generate_steps(desire_torso_pos, current_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            
            # 添加轨迹点
            foot_traj.append([current_foot_pos[0], current_foot_pos[1], current_foot_pos[2], current_torso_pos[3]])
            torso_traj.append(current_torso_pos.copy())
            swing_trajectories.append(footPoses6D())


            time_traj.append(time_traj[-1] + walk_ss_time)
            foot_idx_traj.append(2)
            foot_traj.append(foot_traj[-1].copy())
            torso_traj.append(torso_traj[-1].copy())
            swing_trajectories.append(footPoses6D())



            
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories

    def plan_move_to_world(self, dx=0.2, dy=0.0, dyaw=0.0, time_traj=None, foot_idx_traj=None, foot_traj=None, torso_traj=None, swing_trajectories=None, max_step_x=0.28, max_step_y=0.15, max_step_yaw=30.0, modify_current_x=0):
        """
        规划移动到目标位置的轨迹
        Args:
            dx: x方向目标位移
            dy: y方向目标位移
            dyaw: yaw方向目标角度(度)
            time_traj: 时间轨迹
            foot_idx_traj: 脚索引轨迹
            foot_traj: 脚姿态轨迹
            torso_traj: 躯干姿态轨迹
            swing_trajectories: 腾空相轨迹
            max_step_x: x方向最大步长
            max_step_y: y方向最大步长
            max_step_yaw: yaw方向最大步长(度)
        """
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
        current_height = STAND_HEIGHT
        # 获取最后一个轨迹点作为起始位置
        if len(torso_traj) > 0:
            current_torso_pos = np.array(torso_traj[-1])
            current_foot_pos = np.array(foot_traj[-1][0:3])
            current_yaw = current_torso_pos[3]
            current_height = current_foot_pos[2]
            R_z = np.array([
                [np.cos(current_yaw), -np.sin(current_yaw), 0],
                [np.sin(current_yaw), np.cos(current_yaw), 0],
                [0, 0, 1]
            ])
            dx, dy, dz = R_z.dot(np.array([dx, dy, 0]))
            print("new dx, dy, dyaw", dx, dy, dyaw)
            
        else:
            # current_torso_pos = np.array([0.0, 0.0, 0.0, 0.0])
            # current_foot_pos = np.array([0.0, 0.0, STAND_HEIGHT])
            # current_yaw = 0.0
            current_torso_pos = np.array([modify_current_x, 0.0, 0.0, 0.0])
            current_foot_pos = np.array([modify_current_x, 0.0, STAND_HEIGHT])
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
        # is_left_foot = ((self.total_step - 1) % 2 == 0 or dyaw > 0)
        if dyaw > 0:
            self.is_left_foot = True
        # 记录开始时的轨迹长度
        start_traj_len = len(foot_traj)
        num_steps += 1 # 第一步和最后一步是半步
        walk_dt = 0.4
        walk_ss_time = 0.5
        self.is_left_foot = False if actual_step_y >=0 else True  # 如果y方向步长为正，则左脚先落地，否则右脚先落地
        for i in range(num_steps):
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + walk_dt)
            
            # 左右脚交替
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)
            # 更新躯干位置
            if i == 0:
                is_first_step_left = self.is_left_foot
                current_torso_pos[0] += actual_step_x/2
                current_torso_pos[1] += actual_step_y/2
                current_torso_pos[3] += np.radians(actual_step_yaw)
                # 根据当前yaw角度计算落脚点偏移
                current_yaw = current_torso_pos[3]
                desire_torso_pos = [current_torso_pos[0]+actual_step_x/2, current_torso_pos[1]+(actual_step_y if num_steps!=2 else actual_step_y/2), current_torso_pos[2]]
                lf_foot, rf_foot = self.generate_steps(desire_torso_pos, current_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            # elif i == num_steps - 1 or (abs(dyaw)>0 and i == num_steps - 2):
            elif i == num_steps - 1 :
                current_torso_pos[0] += actual_step_x/2
                current_torso_pos[1] += actual_step_y/2
                # current_torso_pos[3] += np.radians(actual_step_yaw) if not (abs(dyaw)>0 and i == num_steps - 1) else 0
                current_torso_pos[3] += 0
                # 根据当前yaw角度计算落脚点偏移
                current_yaw = current_torso_pos[3]
                lf_foot, rf_foot = self.generate_steps(current_torso_pos[:3], current_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            else:
                current_torso_pos[0] += actual_step_x
                current_torso_pos[1] += actual_step_y
                current_torso_pos[3] += np.radians(actual_step_yaw)
                # 根据当前yaw角度计算落脚点偏移
                current_yaw = current_torso_pos[3]
                desire_torso_pos = [current_torso_pos[0]+actual_step_x/2, current_torso_pos[1]+(actual_step_y if is_first_step_left==self.is_left_foot and i!=num_steps - 2 else actual_step_y/2), current_torso_pos[2]]
                # desire_torso_pos = [current_torso_pos[0]+actual_step_x/2, current_torso_pos[1], current_torso_pos[2]]
                lf_foot, rf_foot = self.generate_steps(desire_torso_pos, current_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            
            # 添加轨迹点
            foot_traj.append([current_foot_pos[0], current_foot_pos[1], current_foot_pos[2], current_torso_pos[3]])
            torso_traj.append(current_torso_pos.copy())
            swing_trajectories.append(footPoses6D())


            time_traj.append(time_traj[-1] + walk_ss_time)
            foot_idx_traj.append(2)
            foot_traj.append(foot_traj[-1].copy())
            torso_traj.append(torso_traj[-1].copy())
            swing_trajectories.append(footPoses6D())



            
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories
    
    
    def plan_up_stairs(self, num_steps=5, time_traj=None, foot_idx_traj=None, foot_traj=None, torso_traj=None, swing_trajectories=None):
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
        torso_height_offset = -0.02  # 躯干高度偏移
        current_torso_pos[2] += torso_height_offset
        # current_foot_pos = np.array([0.0, 0.0, 0.0])
        offset_x = [0.0, 0.0, 0.0, 0.0, 0.0]
        first_step_offset = 0.35
        
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
                # current_torso_pos[0] += self.step_length/2  # 向前移动
                # current_torso_pos[2] += self.step_height/2  # 向上移动
                # current_foot_pos[0] = current_torso_pos[0] # 最后一步x不动
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
                swing_traj = self.plan_swing_phase(prev_foot, current_foot, swing_height=0.12, plot=PLOT, is_first_step=(step == 0 or step == num_steps - 1))
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
            # add SS 
            if step != num_steps - 1:
                pass
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0] - self.step_length*0.0
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses6D())
            else: # 最后一步站立恢复站直
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0]
                last_torso_pose[2] = last_foot_pose[2] - STAND_HEIGHT
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses6D())
    
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
                        d_foot_pos = swing_trajectories[i].data[j].footPose6D[0:3] - init_torso_pos[0:3]
                        swing_trajectories[i].data[j].footPose6D[0:2] = (R_z.dot(d_foot_pos) + init_torso_pos[0:3])[:2]
                        
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories

    def plan_up_stairs_world(self, num_steps=5, time_traj=None, foot_idx_traj=None, foot_traj=None, torso_traj=None, swing_trajectories=None, points=None, last_points=None, last_foot_traj=None, last_swing_trajectories=None):
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
        torso_height_offset = -0.02  # 躯干高度偏移
        current_torso_pos[2] += torso_height_offset
        # current_foot_pos = np.array([0.0, 0.0, 0.0])
        offset_x = [0.0, 0.0, 0.0, 0.0, 0.0]
        first_step_offset = 0.35
        
        # 记录前一次的左右脚位置
        prev_left_foot = [start_foot_pos_x, 0.1, start_foot_pos_z, torso_yaw]
        prev_right_foot = [start_foot_pos_x, -0.1, start_foot_pos_z, torso_yaw]
        if len(foot_traj) == 0:
            last2_foot_pos_x = points[0][0] - self.step_length*2
            last2_foot_pos_z = points[0][2] - self.step_height*2
            last_foot_pos_x = points[0][0] - self.step_length
            last_foot_pos_z = points[0][2] - self.step_height
            for i in range(len(last_foot_traj)):
                if abs(last_foot_traj[i][0] - points[0][0]) < 0.1 and abs(last_foot_traj[i][2] - points[0][2]) < 0.05:
                    if i >= 4:
                        last2_foot_pos_x = last_foot_traj[i-4][0]
                        last2_foot_pos_z = last_foot_traj[i-4][2]
                        last_foot_pos_x = last_foot_traj[i-2][0]
                        last_foot_pos_z = last_foot_traj[i-2][2]
                    else:
                        last2_foot_pos_x = last_swing_trajectories[i].data[0].footPose6D[0]
                        last2_foot_pos_z = last_swing_trajectories[i].data[0].footPose6D[2]
                        if i >= 2:
                            last_foot_pos_x = last_foot_traj[i-2][0]
                            last_foot_pos_z = last_foot_traj[i-2][2]
                        elif i+2 < len(last_swing_trajectories):
                            last_foot_pos_x = last_swing_trajectories[i+2].data[0].footPose6D[0]
                            last_foot_pos_z = last_swing_trajectories[i+2].data[0].footPose6D[2]
                    break

            if not self.is_left_foot:
                prev_left_foot = [last2_foot_pos_x, 0.1, last2_foot_pos_z, torso_yaw]
                prev_right_foot = [last_foot_pos_x, -0.1, last_foot_pos_z, torso_yaw]
            else:
                prev_left_foot = [last_foot_pos_x, 0.1, last_foot_pos_z, torso_yaw]
                prev_right_foot = [last2_foot_pos_x, -0.1, last2_foot_pos_z, torso_yaw]
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
                current_foot_pos[0] = points[step][0]
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] = points[step][2]
                
                # current_torso_pos[0] = points[step][0] - (points[step+1][0] - points[step][0])/2 if len(points) > 1 else points[step][0] - self.step_length/2
                # current_torso_pos[2] = points[step][2] - (points[step+1][2] - points[step][2])   if len(points) > 1 else points[step][2] - self.step_height 
                current_torso_pos[0] = (points[step][0] + last_foot_pos_x)/2 if len(foot_traj) == 0 else (points[step][0] + start_foot_pos_x)/2
                current_torso_pos[2] = last_foot_pos_z  if len(foot_traj) == 0 else start_foot_pos_z
                current_torso_pos[2] += torso_height_offset

            elif step == num_steps - 1: # 最后一步
                current_torso_pos[0] = current_foot_pos[0] # 最后一步躯干x在双脚上方
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_torso_pos[2] = points[step-1][2] + torso_height_offset

            else:
                current_torso_pos[0] = (points[step-1][0] + points[step][0]) / 2
                current_torso_pos[2] = points[step-1][2] + torso_height_offset

                current_foot_pos[0] = points[step][0]
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] = points[step][2]
                
            if step < len(offset_x) and not step == num_steps - 1:    # 脚掌偏移
                current_foot_pos[0] += offset_x[step]
                
            # 记录当前脚的位置
            current_foot = [*current_foot_pos, torso_yaw]
            
            # 生成腾空相轨迹
            if prev_left_foot is not None and prev_right_foot is not None:  # 从第二步开始生成腾空相轨迹
                prev_foot = prev_left_foot if self.is_left_foot else prev_right_foot
                # swing_traj = self.plan_swing_phase(prev_foot, current_foot, swing_height=0.12, plot=PLOT, is_first_step=(step == 0 or step == num_steps - 1))
                swing_traj = self.plan_swing_phase(prev_foot, current_foot, swing_height=0.12, plot=PLOT, is_first_step=(step == num_steps - 1))
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
            # add SS 
            if step != num_steps - 1:
                pass
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                # last_torso_pose[0] = last_foot_pose[0] - self.step_length*0.0
                last_torso_pose[0] = (last_foot_pose[0] + last_torso_pose[0])/2
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses6D())
            else: # 最后一步站立恢复站直
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0]
                last_torso_pose[2] = last_foot_pose[2] - STAND_HEIGHT
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses6D())
    
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
                        d_foot_pos = swing_trajectories[i].data[j].footPose6D[0:3] - init_torso_pos[0:3]
                        swing_trajectories[i].data[j].footPose6D[0:2] = (R_z.dot(d_foot_pos) + init_torso_pos[0:3])[:2]
                        
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories

    def plan_down_stairs(self, num_steps=5, time_traj=None, foot_idx_traj=None, foot_traj=None, torso_traj=None, swing_trajectories=None):
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
        self.dt = 0.6
        self.step_length = 0.27
        torso_yaw = 0.0
        start_foot_pos_x = 0.0
        start_foot_pos_z = STAND_HEIGHT 
        
        # 获取最后一个轨迹点作为起始位置
        if len(torso_traj) > 0:
            current_torso_pos = np.array(torso_traj[-1][0:3])
            current_foot_pos = np.array(foot_traj[-1][0:3])
            start_foot_pos_x = current_foot_pos[0]
            torso_yaw = torso_traj[-1][3]
            start_foot_pos_z = current_foot_pos[2]
            
        else:
            current_torso_pos = np.array([0.0, 0.0, 0.0])
            current_foot_pos = np.array([0.0, 0.0, STAND_HEIGHT])
            start_foot_pos_x = 0.0
        R_z = np.array([
            [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
            [np.sin(torso_yaw), np.cos(torso_yaw), 0],
            [0, 0, 1]
        ])
        # 初始位置
        torso_height_offset = -0.0  # 躯干高度偏移
        current_torso_pos[2] += torso_height_offset
        offset_x = [0.0, -0.0, -0.0, -0.0, -0.0]
        
        # 记录前一次的左右脚位置
        prev_left_foot = [start_foot_pos_x, 0.1, start_foot_pos_z, torso_yaw]
        prev_right_foot = [start_foot_pos_x, -0.1, start_foot_pos_z, torso_yaw]
        if len(foot_traj) > 0:
            if foot_idx_traj[-2] == 0:  # 最后一步是左脚
                prev_left_foot = foot_traj[-2]
                prev_right_foot = foot_traj[-4] if len(foot_traj) > 3 else None
            else:  # 最后一步是右脚
                prev_right_foot = foot_traj[-2]
                prev_left_foot = foot_traj[-4] if len(foot_traj) > 3 else None
        initial_index = len(foot_traj)
        print("prev_left_foot: ", prev_left_foot)
        print("prev_right_foot: ", prev_right_foot)        
        # 添加下蹲
        if len(time_traj) > 0:
            time_traj.append(time_traj[-1] + 1)
            foot_idx_traj.append(2)
            foot_traj.append(foot_traj[-1].copy())
            torso_traj.append(torso_traj[-1].copy())
            torso_traj[-1][2] = current_torso_pos[2]
            swing_trajectories.append(None)
        else:
            time_traj.append(1)
            foot_idx_traj.append(2)
            foot_traj.append([0,0,0,0])
            torso_traj.append([0,0,current_torso_pos[2],0])
            swing_trajectories.append(None)

        base_y = current_torso_pos[1]
        first_step_offset = -0.01
        # 为每一步生成落脚点
        for step in range(num_steps):
            # 更新时间
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + self.dt + (0.0 if step == 0 or step == num_steps - 1 else 0.0))
            
            # 左右脚交替
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)
            
            # 计算躯干位置
            if step == 0:
                # current_torso_pos[0] += self.step_length/2 + first_step_offset
                current_foot_pos[0] = current_torso_pos[0] + self.step_length + first_step_offset  # 脚掌相对躯干前移
                current_torso_pos[0] += self.step_length/2 + first_step_offset
                # current_torso_pos[0] = current_foot_pos[0] - 0.03 # 躯干落在前脚掌               
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] -= self.step_height  # 脚掌高度
                current_torso_pos[2] -= self.step_height-0.05 # 脚掌高度
            elif step == num_steps - 1: # 最后一步
                current_torso_pos[0] = current_foot_pos[0] # 最后一步躯干x在双脚上方
                # current_foot_pos[0] = current_torso_pos[0]  # 
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                # current_torso_pos[2] += self.step_height  # 脚掌高度
            else:
                current_torso_pos[0] += self.step_length  # 向前移动
                current_torso_pos[2] -= self.step_height  # 向下移动
            
                # 计算落脚点位置
                current_foot_pos[0] = current_foot_pos[0] + self.step_length # 脚掌相对躯干前移
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] -= self.step_height
                
            if step < len(offset_x) and not step == num_steps - 1:    # 脚掌偏移
                current_foot_pos[0] += offset_x[step]
                
            # 记录当前脚的位置
            current_foot = [*current_foot_pos, torso_yaw]
            
            # 生成腾空相轨迹
            if prev_left_foot is not None and prev_right_foot is not None:  # 从第二步开始生成腾空相轨迹
                prev_foot = prev_left_foot if self.is_left_foot else prev_right_foot
                swing_traj = self.plan_swing_phase(prev_foot, current_foot, swing_height=0.08, plot=PLOT, down_stairs=True, is_first_step=(step == 0 or step == num_steps - 1))
                swing_trajectories.append(swing_traj)
            else:
                swing_trajectories.append(None)
            
            # 更新前一次的脚位置
            if self.is_left_foot:
                prev_left_foot = current_foot
            else:
                prev_right_foot = current_foot
            
            y_bias = 0.02
            if self.is_left_foot:
                current_torso_pos[1] = base_y + y_bias
            else:
                current_torso_pos[1] = base_y - y_bias
            # 添加轨迹点
            # print("step: ", step, "foot: ", foot_idx_traj[-1])
            # print("current_foot: ", current_foot)
            # print("current_torso_pos", current_torso_pos)
            foot_traj.append(current_foot)
            torso_traj.append([*current_torso_pos, torso_yaw])
            
            last_torso_pose = torso_traj[-1].copy()
            last_foot_pose = foot_traj[-1].copy()
            # add ST 
            self.ss_time = 0.15
            self.st_time = 0.15
            if step != num_steps - 1:
                time_traj.append(time_traj[-1] + self.st_time)
                foot_idx_traj.append(3 if self.is_left_foot else 5)
                foot_traj.append(foot_traj[-1].copy())
                # last_torso_pose[0] = last_foot_pose[0]
                torso_traj.append(last_torso_pose.copy())
                swing_trajectories.append(footPoses6D())


                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(5 if self.is_left_foot else 3)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0]
                last_torso_pose[2] -= 0.05
                torso_traj.append(last_torso_pose.copy())
                swing_trajectories.append(footPoses6D())
                
                
            else: # 最后一步站立恢复站直
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0]
                last_torso_pose[2] = last_foot_pose[2] - STAND_HEIGHT
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses6D())
            # 恢复到中心位置
            if self.is_left_foot:
                current_torso_pos[1] = base_y - y_bias
            else:
                current_torso_pos[1] = base_y + y_bias
            # break
        
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
                        d_foot_pos = swing_trajectories[i].data[j].footPose6D[0:3] - init_torso_pos[0:3]
                        swing_trajectories[i].data[j].footPose6D[0:2] = (R_z.dot(d_foot_pos) + init_torso_pos[0:3])[:2]
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories
    
    def plan_down_stairs_step_by_step(self, num_steps=5, time_traj=None, foot_idx_traj=None, foot_traj=None, torso_traj=None, swing_trajectories=None):
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
        self.dt = 0.6
        self.step_length = 0.254
        torso_yaw = 0.0
        start_foot_pos_x = 0.0
        start_foot_pos_z = STAND_HEIGHT 
        
        # 获取最后一个轨迹点作为起始位置
        if len(torso_traj) > 0:
            current_torso_pos = np.array(torso_traj[-1][0:3])
            current_foot_pos = np.array(foot_traj[-1][0:3])
            start_foot_pos_x = current_foot_pos[0]
            torso_yaw = torso_traj[-1][3]
            start_foot_pos_z = current_foot_pos[2]
            
        else:
            current_torso_pos = np.array([0.0, 0.0, 0.0])
            current_foot_pos = np.array([0.0, 0.0, STAND_HEIGHT])
            start_foot_pos_x = 0.0
        R_z = np.array([
            [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
            [np.sin(torso_yaw), np.cos(torso_yaw), 0],
            [0, 0, 1]
        ])
        # 初始位置
        torso_height_offset = -0.0  # 躯干高度偏移
        current_torso_pos[2] += torso_height_offset
        offset_x = [0.0, -0.0, -0.0, -0.0, -0.0]
        
        # 记录前一次的左右脚位置
        prev_left_foot = [start_foot_pos_x, 0.1, start_foot_pos_z, torso_yaw]
        prev_right_foot = [start_foot_pos_x, -0.1, start_foot_pos_z, torso_yaw]
        if len(foot_traj) > 0:
            if foot_idx_traj[-2] == 0:  # 最后一步是左脚
                prev_left_foot = foot_traj[-2]
                prev_right_foot = foot_traj[-4] if len(foot_traj) > 3 else None
            else:  # 最后一步是右脚
                prev_right_foot = foot_traj[-2]
                prev_left_foot = foot_traj[-4] if len(foot_traj) > 3 else None
        initial_index = len(foot_traj)
        print("prev_left_foot: ", prev_left_foot)
        print("prev_right_foot: ", prev_right_foot)        
        # 添加下蹲
        if len(time_traj) > 0:
            time_traj.append(time_traj[-1] + 1)
            foot_idx_traj.append(2)
            foot_traj.append(foot_traj[-1].copy())
            torso_traj.append(torso_traj[-1].copy())
            torso_traj[-1][2] = current_torso_pos[2]
            swing_trajectories.append(None)
        else:
            time_traj.append(1)
            foot_idx_traj.append(2)
            foot_traj.append([0,0,0,0])
            torso_traj.append([0,0,current_torso_pos[2],0])
            swing_trajectories.append(None)

        first_step_offset = -0.01
        num_steps = (num_steps - 1)*2
        # 为每一步生成落脚点
        for step in range(num_steps):
            # 更新时间
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + self.dt )
            
            # 左右脚交替
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)
            
            # 计算躯干位置
            if step%2 == 0:
                # current_torso_pos[0] += self.step_length/2 + first_step_offset
                current_foot_pos[0] = current_torso_pos[0] + self.step_length  # 脚掌相对躯干前移
                current_torso_pos[0] += self.step_length/2
                # current_torso_pos[0] = current_foot_pos[0] - 0.03 # 躯干落在前脚掌               
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                current_foot_pos[2] -= self.step_height  # 脚掌高度
                current_torso_pos[2] -= self.step_height # 脚掌高度
            elif step == num_steps - 1: # 最后一步
                current_torso_pos[0] = current_foot_pos[0] # 最后一步躯干x在双脚上方
                # current_foot_pos[0] = current_torso_pos[0]  # 
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                # current_torso_pos[2] += self.step_height  # 脚掌高度
            else: # 对齐
                current_torso_pos[0] = current_foot_pos[0] # 向前移动
                current_foot_pos[1] = current_torso_pos[1] + self.foot_width if self.is_left_foot else -self.foot_width  # 左右偏移
                
            # 记录当前脚的位置
            current_foot = [*current_foot_pos, torso_yaw]
            
            # 生成腾空相轨迹
            if prev_left_foot is not None and prev_right_foot is not None:  # 从第二步开始生成腾空相轨迹
                prev_foot = prev_left_foot if self.is_left_foot else prev_right_foot
                swing_traj = self.plan_swing_phase(prev_foot, current_foot, swing_height=0.08, plot=PLOT, down_stairs=True, is_first_step=True, plan_Toe_Contact = step%2 == 0)
                swing_trajectories.append(swing_traj)
            else:
                swing_trajectories.append(None)
            
            # 更新前一次的脚位置
            if self.is_left_foot:
                prev_left_foot = current_foot
            else:
                prev_right_foot = current_foot
            

            foot_traj.append(current_foot)
            torso_traj.append([*current_torso_pos, torso_yaw])
            
            last_torso_pose = torso_traj[-1].copy()
            last_foot_pose = foot_traj[-1].copy()
            # add ST 
            self.ss_time = 1.0
            self.st_time = 0.15
            if step != num_steps - 1:


                if step%2 != 0:
                
                    time_traj.append(time_traj[-1] + self.ss_time)
                    foot_idx_traj.append(2)
                    foot_traj.append(foot_traj[-1].copy())
                    last_torso_pose[0] = last_foot_pose[0]
                    torso_traj.append(last_torso_pose.copy())
                    swing_trajectories.append(footPoses6D())
                    
                    time_traj.append(time_traj[-1] + self.st_time)
                    foot_idx_traj.append(5 if self.is_left_foot else 3)
                    foot_traj.append(foot_traj[-1].copy())
                    last_torso_pose[0] = last_foot_pose[0]
                    torso_traj.append(last_torso_pose.copy())
                    swing_trajectories.append(footPoses6D())
                else:
                    time_traj.append(time_traj[-1] + self.st_time)
                    foot_idx_traj.append(3 if self.is_left_foot else 5)
                    foot_traj.append(foot_traj[-1].copy())
                    last_torso_pose[0] = last_foot_pose[0]
                    torso_traj.append(last_torso_pose.copy())
                    swing_trajectories.append(footPoses6D())

                    # time_traj.append(time_traj[-1] + self.ss_time)
                    # foot_idx_traj.append(2)
                    # foot_traj.append(foot_traj[-1].copy())
                    # # last_torso_pose[0] = last_foot_pose[0]
                    # torso_traj.append(last_torso_pose.copy())
                    # swing_trajectories.append(footPoses6D())
                
                
            else: # 最后一步站立恢复站直
                time_traj.append(time_traj[-1] + self.ss_time)
                foot_idx_traj.append(2)
                foot_traj.append(foot_traj[-1].copy())
                last_torso_pose[0] = last_foot_pose[0]
                last_torso_pose[2] = last_foot_pose[2] - STAND_HEIGHT
                torso_traj.append(last_torso_pose)
                swing_trajectories.append(footPoses6D())
                
            
        
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
                        d_foot_pos = swing_trajectories[i].data[j].footPose6D[0:3] - init_torso_pos[0:3]
                        swing_trajectories[i].data[j].footPose6D[0:2] = (R_z.dot(d_foot_pos) + init_torso_pos[0:3])[:2]
        return time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories

    def plan_swing_phase(self, prev_foot_pose, next_foot_pose, swing_height=0.10, plot=False, down_stairs=False, is_first_step = False, plan_Toe_Contact = False):
        """
        使用形状保持的三次样条插值规划腾空相的轨迹
        Args:
            prev_foot_pose: 上一个落点位置 [x, y, z, yaw]
            next_foot_pose: 下一个落点位置 [x, y, z, yaw]
            swing_height: 抬脚最大高度，默认0.2米
            plot: 是否绘制轨迹图，默认False
        Returns:
            additionalFootPoseTrajectory: 包含腾空相轨迹的footPoses6D消息
        """
        additionalFootPoseTrajectory = footPoses6D()
        num_points = 7  # 轨迹点数量
        
        # 创建时间序列
        t = np.linspace(0, 1, num_points)
        
        # 计算x和y方向的移动距离
        x_distance = next_foot_pose[0] - prev_foot_pose[0]
        y_distance = next_foot_pose[1] - prev_foot_pose[1]
        
        # 计算基准高度（取两个落点中较高的点）
        base_height = max(prev_foot_pose[2], next_foot_pose[2])
        min_height = min(prev_foot_pose[2], next_foot_pose[2])
        
        # 创建控制点
        if not down_stairs:
            control_points = {
                't': [0, 0.2, 0.6, 1.0],
                'x': [
                    prev_foot_pose[0],
                    prev_foot_pose[0] + x_distance * 0.05,
                    prev_foot_pose[0] + x_distance * 0.6,
                    next_foot_pose[0]
                ],
                'y': [
                    prev_foot_pose[1],
                    prev_foot_pose[1] + y_distance * 0.05,
                    prev_foot_pose[1] + y_distance * 0.6,
                    next_foot_pose[1]
                ],
                'z': [
                    prev_foot_pose[2],
                    (base_height + swing_height*0.6) if is_first_step else (prev_foot_pose[2] + (base_height-min_height) * 0.5),
                    base_height + swing_height,
                    next_foot_pose[2]
                ],
                'yaw': [
                    prev_foot_pose[3],
                    prev_foot_pose[3] + (next_foot_pose[3] - prev_foot_pose[3]) * 0.2,
                    prev_foot_pose[3] + (next_foot_pose[3] - prev_foot_pose[3]) * 0.6,
                    next_foot_pose[3]
                ],
                'pitch': [0.0, 0.0, 0.0, 0.0],
                'roll': [0.0, 0.0, 0.0, 0.0],
            }
        else:
            if not is_first_step:
                pitch_sequence = [0.5,0.5,0.5,0.5,0.5] if (plan_Toe_Contact) else [0.5,0.5,0.0,0.0,0.00]
                first_step_offset = self.calculate_foot_tip_offset(pitch_sequence[0])
                last_step_offset = self.calculate_foot_tip_offset(pitch_sequence[-1])

                control_points = {
                    't': [0, 0.4, 0.5, 0.6, 1.0],
                    'x': [
                        prev_foot_pose[0] + first_step_offset[0],
                        prev_foot_pose[0] + x_distance * 0.5,
                        prev_foot_pose[0] + x_distance * 0.65,
                        prev_foot_pose[0] + x_distance * 0.8,
                        next_foot_pose[0] + last_step_offset[0]
                    ],
                    'y': [
                        prev_foot_pose[1] + first_step_offset[1],
                        prev_foot_pose[1] + y_distance * 0.5,
                        prev_foot_pose[1] + y_distance * 0.65,
                        prev_foot_pose[1] + y_distance * 0.8,
                        next_foot_pose[1] + last_step_offset[1]
                    ],
                    'z': [
                        prev_foot_pose[2] + first_step_offset[2],
                        base_height + swing_height,
                        base_height ,
                        (next_foot_pose[2] + (base_height-min_height) * 0.8),
                        next_foot_pose[2] + last_step_offset[2]
                    ],
                    'yaw': [
                        prev_foot_pose[3],
                        prev_foot_pose[3] + (next_foot_pose[3] - prev_foot_pose[3]) * 0.4,
                        prev_foot_pose[3] + (next_foot_pose[3] - prev_foot_pose[3]) * 0.5,
                        prev_foot_pose[3] + (next_foot_pose[3] - prev_foot_pose[3]) * 0.6,
                        next_foot_pose[3]
                    ],
                    'pitch': pitch_sequence,
                    'roll': [0,0,0,0,0],
                }
            else:
                pitch_sequence = [0.5,0.5,0.5,0.5] if (plan_Toe_Contact) else [0.5,0.5,0.5,0.0]
                first_step_offset = self.calculate_foot_tip_offset(pitch_sequence[0])
                last_step_offset = self.calculate_foot_tip_offset(pitch_sequence[-1])
                control_points = {
                    't': [0, 0.5, 0.6, 1.0],
                    'x': [
                        prev_foot_pose[0] + first_step_offset[0],
                        prev_foot_pose[0] + x_distance * 0.60,
                        prev_foot_pose[0] + x_distance * 0.95,
                        next_foot_pose[0] + last_step_offset[0]
                    ],
                    'y': [
                        prev_foot_pose[1] + first_step_offset[1],
                        prev_foot_pose[1] + y_distance * 0.6,
                        prev_foot_pose[1] + y_distance * 0.95,
                        next_foot_pose[1] + last_step_offset[1]
                    ],
                    'z': [
                        prev_foot_pose[2] + first_step_offset[2],
                        base_height + swing_height,
                        base_height + swing_height * 0.3,
                        next_foot_pose[2] + last_step_offset[2]
                    ],
                    'yaw': [
                        prev_foot_pose[3],
                        prev_foot_pose[3] + (next_foot_pose[3] - prev_foot_pose[3]) * 0.5,
                        prev_foot_pose[3] + (next_foot_pose[3] - prev_foot_pose[3]) * 0.6,
                        next_foot_pose[3]
                    ],
                    'pitch': pitch_sequence,
                    'roll': [0,0,0,0],
                }
        # 为x、y、z、yaw、pitch、roll创建形状保持的三次样条插值
        x_spline = PchipInterpolator(control_points['t'], control_points['x'])
        y_spline = PchipInterpolator(control_points['t'], control_points['y'])
        z_spline = PchipInterpolator(control_points['t'], control_points['z'])
        yaw_spline = PchipInterpolator(control_points['t'], control_points['yaw'])
        pitch_spline = PchipInterpolator(control_points['t'], control_points['pitch'])
        roll_spline = PchipInterpolator(control_points['t'], control_points['roll'])
        
        # 生成轨迹点
        trajectory_points = []
        for i in range(num_points):
            step_fp = footPose6D()
            x = float(x_spline(t[i]))
            y = float(y_spline(t[i]))
            z = float(z_spline(t[i]))
            yaw = float(yaw_spline(t[i]))
            pitch = float(pitch_spline(t[i]))
            roll = float(roll_spline(t[i]))
            
            # 如果是下楼梯且pitch不为0，计算脚掌中心点的偏移量
            # if down_stairs and abs(pitch) > 1e-6:
            #     offset = self.calculate_foot_tip_offset(pitch)
            #     x += offset[0]
            #     y += offset[1]
            #     z += offset[2]
            
            step_fp.footPose6D = [x, y, z, yaw, pitch, roll]
            additionalFootPoseTrajectory.data.append(step_fp)
            trajectory_points.append([x, y, z])
        
        # 如果需要绘图
        if plot:
            # 创建更密集的时间序列用于绘制平滑曲线
            t_dense = np.linspace(0, 1, 100)
            x_dense = x_spline(t_dense)
            y_dense = y_spline(t_dense)
            z_dense = z_spline(t_dense)
            
            # 创建3D图
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # 绘制轨迹
            ax.plot(x_dense, y_dense, z_dense, 'b-', label='trajectory')
            
            # 绘制控制点
            ax.scatter(control_points['x'], control_points['y'], control_points['z'], 
                      c='r', marker='o', label='control points')
            
            # 绘制实际轨迹点
            trajectory_points = np.array(trajectory_points)
            ax.scatter(trajectory_points[:, 0], trajectory_points[:, 1], trajectory_points[:, 2],
                      c='g', marker='^', label='trajectory points')
            
            # 设置图表属性
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title('foot pose trajectory')
            
            # 添加图例
            ax.legend()
            
            # 设置坐标轴比例相等
            max_range = np.array([
                max(x_dense) - min(x_dense),
                max(y_dense) - min(y_dense),
                max(z_dense) - min(z_dense)
            ]).max() / 2.0
            
            mid_x = (max(x_dense) + min(x_dense)) * 0.5
            mid_y = (max(y_dense) + min(y_dense)) * 0.5
            mid_z = (max(z_dense) + min(z_dense)) * 0.5
            
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
            
            # 显示图形
            plt.show()
            
        return additionalFootPoseTrajectory

def publish_foot_pose_traj(time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories=None):
    rospy.init_node('stair_climbing_planner', anonymous=True)
    pub = rospy.Publisher('/humanoid_mpc_foot_pose_6d_target_trajectories', 
                         footPose6DTargetTrajectories, queue_size=10)
    rospy.sleep(1)

    msg = footPose6DTargetTrajectories()
    msg.timeTrajectory = time_traj
    msg.footIndexTrajectory = foot_idx_traj
    msg.footPoseTrajectory = []
    msg.additionalFootPoseTrajectory = []

    for i in range(len(time_traj)):
        foot_pose_msg = footPose6D()
        # 将4D数据转换为6D格式 [x, y, z, yaw] -> [x, y, z, yaw, pitch, roll]
        foot_pose_msg.footPose6D = [*foot_traj[i], 0.0, 0.0]  # 添加pitch和roll为0
        foot_pose_msg.torsoPose6D = [*torso_traj[i], 0.0, 0.0]  # 添加pitch和roll为0
        msg.footPoseTrajectory.append(foot_pose_msg)
        
        # 如果有腾空相轨迹，添加到消息中
        if swing_trajectories is not None and i < len(swing_trajectories):
            if swing_trajectories[i] is not None:
                msg.additionalFootPoseTrajectory.append(swing_trajectories[i])
            else:
                msg.additionalFootPoseTrajectory.append(footPoses6D())
        else:
            msg.additionalFootPoseTrajectory.append(footPoses6D())  # 添加空的轨迹

    pub.publish(msg)
    rospy.sleep(1.5)

if __name__ == '__main__':
    try:
        args = parse_args()
        PLOT = args.plot
        STAND_HEIGHT = args.initH
        # 禁用俯仰角限制
        set_pitch_limit(False)
        
        planner = StairClimbingPlanner()
        time_traj=None
        foot_idx_traj=None
        foot_traj=None
        torso_traj=None
        swing_trajectories=None
        
        # time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = planner.plan_move_to(0.0,0,-180, time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
        # print("\nMove to down stairs plan done.")

        # if (time_traj is not None):
        #     for i,t in enumerate(time_traj):
        #         print(f"{i:2}:{t:3.2f} {foot_idx_traj[i]} {foot_traj[i]} {torso_traj[i]}")
        
        if not args.down_stairs:
            time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = planner.plan_up_stairs(5, time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
            print("Up stairs plan done.")
            if (time_traj is not None):
                for i,t in enumerate(time_traj):
                    print(f"{i:2}:{t:3.2f} {foot_idx_traj[i]} {foot_traj[i]} {torso_traj[i]}")
            
            time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = planner.plan_move_to(0.35,0,0, time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
            print("\nMove to down stairs plan done.")
            if (time_traj is not None):
                for i,t in enumerate(time_traj):
                    print(f"{i:2}:{t:3.2f} {foot_idx_traj[i]} {foot_traj[i]} {torso_traj[i]}")
            
        # time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = planner.plan_move_to(0.0,0,180, time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
        # print("\nMove to down stairs plan done.")
 
        
        # time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = planner.plan_move_to(0.16,0,0, time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
        # print("\nMove to down stairs plan done.")
        # if (time_traj is not None):
        #     for i,t in enumerate(time_traj):
        #         print(f"{i:2}:{t:3.2f} {foot_idx_traj[i]} {foot_traj[i]} {torso_traj[i]}")
        
        
        time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories = planner.plan_down_stairs_step_by_step(5, time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)
        print("\nDown stairs plan done.")
        if (time_traj is not None):
            for i,t in enumerate(time_traj):
                print(f"{i:2}:{t:3.2f} {foot_idx_traj[i]} {foot_traj[i]} {torso_traj[i]}")
        
        # 打印规划结果
        print("\nTime trajectory:", time_traj)
        if (time_traj is not None):
            for i,t in enumerate(time_traj):
                print(f"{i:2}:{t:3.2f} {foot_idx_traj[i]} {foot_traj[i]} {torso_traj[i]}")
        
        publish_foot_pose_traj(time_traj, foot_idx_traj, foot_traj, torso_traj, swing_trajectories)

    except rospy.ROSInterruptException:
        # 确保在发生异常时也重新启用俯仰角限制
        set_pitch_limit(True)
        pass
