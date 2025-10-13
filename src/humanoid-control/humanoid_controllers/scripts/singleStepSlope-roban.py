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

# 导入自动步态控制模块
try:
    from auto_gait_controller import AutoGaitController
except ImportError:
    print("警告: 无法导入 auto_gait_controller 模块")
    AutoGaitController = None

def get_user_input():
    """获取用户输入"""
    print("\n" + "="*60)
    print("增强版6D步态控制器 (集成自动步态)")
    print("="*60)
    print("请选择要执行的功能:")
    print("┌─" + "─"*56 + "─┐")
    print("│ 1. 上斜坡 (slope) - 6D轨迹控制                    │")
    print("│ 2. 下斜坡 (downslope) - 6D轨迹控制                │")
    print("│ 3. 转身 (turn) - 自动步态控制                     │")
    print("│ 4. 前走 (forward) - 自动步态控制                  │")
    print("│ 5. 横走 (strafe) - 自动步态控制                   │")
    print("│ 6. 预设1: 上斜坡→下斜坡                           │")
    print("│ 7. 预设2: 前走→上斜坡→下斜坡→前走                 │")
    print("│ 8. 退出                                             │")
    print("└─" + "─"*56 + "─┘")
    
    while True:
        choice = input("\n请输入选择 (1-8): ").strip()
        
        if choice == '1':
            return 'slope'
        elif choice == '2':
            return 'downslope'
        elif choice == '3':
            return 'turn'
        elif choice == '4':
            return 'forward'
        elif choice == '5':
            return 'strafe'
        elif choice == '6':
            return 'preset'
        elif choice == '7':
            return 'simple_preset'
        elif choice == '8':
            return 'quit'
        else:
            print("无效选择，请输入 1-8")

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
        # 保留原有斜坡控制功能
        self.foot_width = 0.108535  # 脚宽度
        
        # 步态时间控制参数
        self.walk_dt = 0.6  # 行走步态周期
        self.walk_ss_time = 0.2  # 行走支撑相时间
        self.slope_dt = 0.7  # 上/下斜坡步态周期
        self.slope_ss_time = 0.3  # 上/下斜坡支撑相时间
        
        # 步长限制参数
        self.max_step_x = 0.2  # 最大步长
        self.max_step_y = 0.15
        self.max_step_yaw = 30.0
        
        # 躯干偏置参数（参考continuousStairClimber-roban.py）
        self.torso_z_offset = 0.0  # 斜坡控制用的躯干高度偏移
        self.temp_x_offset = 0.002  # 前进方向临时偏移，每步叠加（参考continuousStairClimber-roban.py）
        
        # 状态变量
        self.total_step = 0
        self.is_left_foot = False
        
        # 全局高度管理
        self.global_height = 0.0
        self.last_torso_z_start = 0.0
        self.last_torso_z_end = 0.0
        
        # 轨迹发布器 (用于斜坡控制)
        self.traj_pub = rospy.Publisher('/humanoid_mpc_foot_pose_6d_target_trajectories', 
                                       footPose6DTargetTrajectories, queue_size=1)
        
        # 自动步态控制器 (新增)
        print("正在初始化自动步态控制器...")
        if AutoGaitController:
            try:
                self.auto_gait_controller = AutoGaitController()
                print("自动步态控制器初始化成功")
                rospy.loginfo("自动步态控制器初始化成功")
            except Exception as e:
                print(f"自动步态控制器初始化失败: {e}")
                self.auto_gait_controller = None
        else:
            self.auto_gait_controller = None
            print("自动步态控制器未初始化 - AutoGaitController类不可用")
            rospy.logwarn("自动步态控制器未初始化")
        
        # 等待发布器准备就绪
        rospy.sleep(0.5)
    
    def generate_steps_6d(self, torso_pos, torso_yaw, foot_height=0):
        """根据躯干位置计算落脚点（保留原有逻辑）"""
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
    
    def plan_move_to_6d(self, dx=0.2, dy=0.0, dyaw=0.0, dz=0.0, time_traj=None, foot_idx_traj=None, foot_traj_6d=None, torso_traj_6d=None, swing_trajectories=None, max_step_x=0.28, max_step_y=0.15, max_step_yaw=30.0):
        """规划6D移动到目标位置的轨迹（保留原有逻辑）"""
        # 初始化轨迹列表
        if time_traj is None:
            time_traj = []
        if foot_idx_traj is None:
            foot_idx_traj = []
        if foot_traj_6d is None:
            foot_traj_6d = []
        if torso_traj_6d is None:
            torso_traj_6d = []
        if swing_trajectories is None:
            swing_trajectories = []
        
        current_height = 0.0
        
        # 获取最后一个轨迹点作为起始位置
        if len(torso_traj_6d) > 0:
            current_torso_pos = np.array(torso_traj_6d[-1])
            current_foot_pos = np.array(foot_traj_6d[-1][0:3])
            current_yaw = current_torso_pos[3]
            current_height = current_foot_pos[2]
            R_z = np.array([
                [np.cos(current_yaw), -np.sin(current_yaw), 0],
                [np.sin(current_yaw), np.cos(current_yaw), 0],
                [0, 0, 1]
            ])
            rotated = R_z.dot(np.array([dx, dy, 0]))
            dx, dy = rotated[0], rotated[1]
        else:
            current_torso_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            current_foot_pos = np.array([0.0, 0.0, current_height])
            current_yaw = 0.0

        # 计算需要的步数
        num_steps_x = max(1, int(np.ceil(abs(dx) / max_step_x)))
        num_steps_y = max(1, int(np.ceil(abs(dy) / max_step_y)))
        num_steps_yaw = max(1, int(np.ceil(abs(dyaw) / max_step_yaw)))
        num_steps_z = max(1, int(np.ceil(abs(dz) / 0.05))) if abs(dz) > 0.001 else 0
        num_steps = max(num_steps_x, num_steps_y, num_steps_yaw, num_steps_z)

        # 计算实际步长
        actual_step_x = dx / num_steps
        actual_step_y = dy / num_steps
        actual_step_yaw = dyaw / num_steps
        actual_step_z = dz / num_steps if num_steps > 0 else 0.0
        
        initial_yaw = current_torso_pos[3]
        target_yaw = initial_yaw + np.radians(dyaw)
        
        if dyaw > 0:
            self.is_left_foot = True
        
        start_traj_len = len(foot_traj_6d)
        num_steps += 1
        walk_dt = self.walk_dt
        walk_ss_time = self.walk_ss_time

        print(f"规划6D移动: dx={dx:.3f}m, dy={dy:.3f}m, dyaw={dyaw:.1f}°, dz={dz:.3f}m, 步数={num_steps}")

        for i in range(num_steps):
            self.total_step += 1
            time_traj.append((time_traj[-1] if len(time_traj) > 0 else 0) + walk_dt)
            
            self.is_left_foot = not self.is_left_foot
            foot_idx_traj.append(0 if self.is_left_foot else 1)
            
            if abs(dyaw) > 0.1:
                progress = (i + 1) / num_steps
                current_torso_yaw = initial_yaw + progress * np.radians(dyaw)
            else:
                current_torso_yaw = initial_yaw
            
            if i == 0:
                current_torso_pos[0] += actual_step_x/2
                current_torso_pos[1] += actual_step_y/2
                current_torso_pos[2] += actual_step_z/2
                current_torso_pos[3] = current_torso_yaw
                desire_torso_pos = [current_torso_pos[0]+actual_step_x/2, current_torso_pos[1]+actual_step_y/2, current_torso_pos[2]]
                lf_foot, rf_foot = self.generate_steps_6d(desire_torso_pos, current_torso_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            elif i == num_steps - 1:
                current_torso_pos[0] += actual_step_x/2
                current_torso_pos[1] += actual_step_y/2
                current_torso_pos[2] += actual_step_z/2
                current_torso_pos[3] = target_yaw
                lf_foot, rf_foot = self.generate_steps_6d(current_torso_pos[:3], current_torso_pos[3], current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            else:
                current_torso_pos[0] += actual_step_x
                current_torso_pos[1] += actual_step_y
                current_torso_pos[2] += actual_step_z
                current_torso_pos[3] = current_torso_yaw
                desire_torso_pos = [current_torso_pos[0]+actual_step_x/2, current_torso_pos[1]+actual_step_y/2, current_torso_pos[2]]
                lf_foot, rf_foot = self.generate_steps_6d(desire_torso_pos, current_torso_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            
            # 叠加临时x方向偏置（每步都叠加，参考continuousStairClimber-roban.py）
            current_torso_pos[0] += self.temp_x_offset * (i + 1)
            
            foot_traj_6d.append([current_foot_pos[0], current_foot_pos[1], current_foot_pos[2], current_torso_pos[3], 0.0, 0.0])
            torso_6d = [current_torso_pos[0], current_torso_pos[1], current_torso_pos[2], current_torso_pos[3], 0.0, 0.0]
            torso_traj_6d.append(torso_6d)
            swing_trajectories.append(None)

            time_traj.append(time_traj[-1] + walk_ss_time)
            foot_idx_traj.append(2)
            foot_traj_6d.append(foot_traj_6d[-1].copy())
            torso_traj_6d.append(torso_traj_6d[-1].copy())
            swing_trajectories.append(None)
        
        return time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories
    
    def publish_trajectory(self, time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories=None):
        """发布6D轨迹（保留原有逻辑）"""
        try:
            rospy.sleep(0.1)
            
            if len(torso_traj_6d) > 0:
                self.last_torso_z_start = torso_traj_6d[0][2]
            
            msg = footPose6DTargetTrajectories()
            msg.timeTrajectory = time_traj
            msg.footIndexTrajectory = foot_idx_traj
            msg.footPoseTrajectory = []
            msg.swingHeightTrajectory = []
            
            for i in range(len(time_traj)):
                foot_pose_msg = footPose6D()
                
                foot_pose_6d_with_global = list(foot_traj_6d[i])
                foot_pose_6d_with_global[2] += self.global_height
                
                foot_pose_msg.footPose6D = foot_pose_6d_with_global
                foot_pose_msg.torsoPose6D = torso_traj_6d[i]
                msg.footPoseTrajectory.append(foot_pose_msg)
                msg.swingHeightTrajectory.append(0.06)
            
            if len(torso_traj_6d) > 0:
                self.last_torso_z_end = torso_traj_6d[-1][2]
                height_diff = self.last_torso_z_end - self.last_torso_z_start
                self.global_height += height_diff
            
            if self.traj_pub.get_num_connections() == 0:
                print("警告: 没有订阅者连接到话题")
            else:
                print(f"订阅者数量: {self.traj_pub.get_num_connections()}")
            
            self.traj_pub.publish(msg)
            rospy.sleep(0.2)
            print(f"轨迹发布成功，长度: {len(time_traj)}")
            
            return True
            
        except Exception as e:
            print(f"轨迹发布失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def plan_continuous_slope_movement(self, forward_distance=2.0, step_length=0.1, slope_angle=5.0, start_with_left_foot=None, first_step_opposite=False):
        """规划连续上斜坡/下斜坡轨迹（保留原有逻辑）
        
        Args:
            forward_distance: 前进距离
            step_length: 步长
            slope_angle: 坡度角度
            start_with_left_foot: 是否从左脚开始
            first_step_opposite: 第一步迈腿方向是否与上一步摆动相反（用于下楼梯）
        """
        is_downslope = slope_angle < 0
        action_name = "下斜坡" if is_downslope else "上斜坡"
        abs_slope_angle = abs(slope_angle)
        
        print(f"规划{action_name}: 距离={forward_distance}m, 步长={step_length}m, 坡度={abs_slope_angle}°")
        if first_step_opposite:
            print("启用第一步迈腿方向与上一步摆动相反")
        
        num_steps = max(1, int(np.ceil(forward_distance / step_length)))
        actual_step_length = forward_distance / num_steps
        
        time_traj = []
        foot_idx_traj = []
        foot_traj_6d = []
        torso_traj_6d = []
        swing_trajectories = []
        
        slope_rad = np.radians(abs_slope_angle)
        if is_downslope:
            slope_rad = -slope_rad
        
        current_torso_pos = [0.0, 0.0, 0.0]
        current_yaw = 0.0
        current_height = 0.0
        is_left_foot = start_with_left_foot if start_with_left_foot is not None else True
        
        # 如果启用第一步相反方向，则反转起始脚
        if first_step_opposite:
            is_left_foot = not is_left_foot
            print(f"第一步迈腿方向调整: {'左脚' if is_left_foot else '右脚'}")
        
        height_per_step = actual_step_length * np.tan(slope_rad)
        
        if is_downslope:
            self.torso_z_offset = -2*height_per_step
        else:
            self.torso_z_offset = -2*height_per_step
        
        print(f"斜坡躯干高度偏移: {self.torso_z_offset:.3f}m (类型: {'下斜坡' if is_downslope else '上斜坡'})")
        
        for i in range(num_steps):
            if (i == 0 or i == 1):
                first_step_length = min(actual_step_length * 1.2, 0.2)
                if is_downslope:
                    current_torso_pos[0] += actual_step_length * 0.5
                    current_torso_pos[2] += height_per_step
                    current_height += height_per_step
                    
                    if is_left_foot:
                        foot_pos = [current_torso_pos[0] + first_step_length , self.foot_width, current_height]
                    else:
                        foot_pos = [current_torso_pos[0] + first_step_length , -self.foot_width, current_height]
                else:
                    current_torso_pos[0] += actual_step_length / 2
                    current_torso_pos[2] += height_per_step
                    current_height += height_per_step
                    
                    if is_left_foot:
                        foot_pos = [current_torso_pos[0] + first_step_length, self.foot_width, current_height]
                    else:
                        foot_pos = [current_torso_pos[0] + first_step_length, -self.foot_width, current_height]
            else:
                if i == 2:
                    target_torso_x = actual_step_length * 3
                    current_torso_pos[0] = target_torso_x
                else:
                    current_torso_pos[0] += actual_step_length
                
                current_torso_pos[2] += height_per_step
                current_height += height_per_step
                
                if is_left_foot:
                    foot_pos = [current_torso_pos[0], self.foot_width, current_height]
                else:
                    foot_pos = [current_torso_pos[0], -self.foot_width, current_height]
            
            foot_pitch = -slope_rad
            
            if is_downslope:
                torso_pitch = 0.0*slope_rad
            else:
                torso_pitch = 0.6*slope_rad
            
            torso_pos_with_offset = [
                current_torso_pos[0],
                current_torso_pos[1],
                current_torso_pos[2] + self.torso_z_offset
            ]
            
            # 叠加临时x方向偏置（每步都叠加，参考continuousStairClimber-roban.py）
            torso_pos_with_offset[0] += self.temp_x_offset * (i + 1)
            
            time_traj.append((i + 1) * self.slope_dt)
            foot_idx_traj.append(0 if is_left_foot else 1)
            foot_traj_6d.append([foot_pos[0], foot_pos[1], foot_pos[2], current_yaw, foot_pitch, 0.0])
            torso_traj_6d.append([torso_pos_with_offset[0], torso_pos_with_offset[1], torso_pos_with_offset[2], current_yaw, torso_pitch, 0.0])
            swing_trajectories.append(None)
            
            time_traj.append((i + 1) * self.slope_dt + self.slope_ss_time)
            foot_idx_traj.append(2)
            foot_traj_6d.append(foot_traj_6d[-1].copy())
            torso_traj_6d.append(torso_traj_6d[-1].copy())
            swing_trajectories.append(None)
            
            is_left_foot = not is_left_foot
        
        print(f"{action_name}规划完成: 距离={current_torso_pos[0]:.3f}m, 高度={current_torso_pos[2]:.3f}m")
        
        recovery_distance = 0.15
        original_torso_z_offset = self.torso_z_offset
        self.torso_z_offset = 0.0
        
        last_swing_idx = -2
        while last_swing_idx >= -len(foot_idx_traj) and foot_idx_traj[last_swing_idx] == 2:
            last_swing_idx -= 1
        
        if last_swing_idx >= -len(foot_idx_traj):
            last_foot_idx = foot_idx_traj[last_swing_idx]
            last_foot_is_left = (last_foot_idx == 1)
            print(f"{action_name}最后摆动脚: {'左脚' if last_foot_is_left else '右脚'}")
        else:
            last_foot_is_left = is_left_foot
            print(f"无法检测摆动脚，使用当前状态: {'左脚' if last_foot_is_left else '右脚'}")
        
        if last_foot_is_left:
            self.is_left_foot = False
        else:
            self.is_left_foot = True
        
        dz_recover = -original_torso_z_offset
        recovery_time_traj, recovery_foot_idx_traj, recovery_foot_traj_6d, recovery_torso_traj_6d, recovery_swing_trajectories = self.plan_move_to_6d(
            dx=recovery_distance, dy=0.0, dyaw=0.0, dz=dz_recover,
            time_traj=time_traj, foot_idx_traj=foot_idx_traj, foot_traj_6d=foot_traj_6d, 
            torso_traj_6d=torso_traj_6d, swing_trajectories=swing_trajectories,
            max_step_x=self.max_step_x, max_step_y=self.max_step_y, max_step_yaw=self.max_step_yaw
        )
        
        print(f"恢复姿态完成: 距离={recovery_distance}m")
        
        return recovery_time_traj, recovery_foot_idx_traj, recovery_foot_traj_6d, recovery_torso_traj_6d, recovery_swing_trajectories
    
    def plan_turn_movement(self, turn_angle=90.0):
        """规划转身轨迹（使用plan_move_to_6d实现）"""
        # 判断转向方向
        if turn_angle > 0:
            direction = "向左"
        elif turn_angle < 0:
            direction = "向右"
        else:
            direction = "不转向"
        
        print(f"规划转身: {abs(turn_angle)}° {direction}")
        
        # 使用plan_move_to_6d方法实现转身，dx=0, dy=0, dyaw=turn_angle
        time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = self.plan_move_to_6d(
            dx=0.0, dy=0.0, dyaw=turn_angle, 
            max_step_x=self.max_step_x, max_step_y=self.max_step_y, max_step_yaw=self.max_step_yaw
        )
        
        return time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories
    
    def plan_forward_movement(self, dx=0.2, dy=0.0):
        """规划前/侧向行走轨迹（使用plan_move_to_6d实现；dx可为负表示后退，dy表示侧移）"""
        print(f"规划行走: dx={dx}m, dy={dy}m")
        
        # 使用plan_move_to_6d方法实现平面位移
        time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = self.plan_move_to_6d(
            dx=dx, dy=dy, dyaw=0.0, 
            max_step_x=self.max_step_x, max_step_y=self.max_step_y, max_step_yaw=self.max_step_yaw
        )
        
        return time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories
    
    # ========== 新增自动步态控制功能 ==========
    
    def execute_auto_gait_forward(self, distance=0.2):
        """使用自动步态执行前进"""
        print(f"execute_auto_gait_forward被调用，距离={distance}")
        print(f"auto_gait_controller状态: {self.auto_gait_controller}")
        
        if not self.auto_gait_controller:
            print("错误: 自动步态控制器未初始化")
            return False
        
        print(f"使用自动步态前进: 距离={distance}m")
        try:
            print("调用auto_gait_controller.execute_forward_movement...")
            self.auto_gait_controller.execute_forward_movement(distance=distance)
            print("execute_forward_movement调用完成")
            return True
        except Exception as e:
            print(f"自动步态前进失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def execute_auto_gait_strafe(self, distance=0.2):
        """使用自动步态执行横移"""
        if not self.auto_gait_controller:
            print("错误: 自动步态控制器未初始化")
            return False
        
        print(f"使用自动步态横移: 距离={distance}m")
        try:
            self.auto_gait_controller.execute_strafe_movement(distance=distance)
            return True
        except Exception as e:
            print(f"自动步态横移失败: {e}")
            return False
    
    def execute_auto_gait_turn(self, angle_degrees=90):
        """使用自动步态执行转身"""
        if not self.auto_gait_controller:
            print("错误: 自动步态控制器未初始化")
            return False
        
        print(f"使用自动步态转身: 角度={angle_degrees}°")
        try:
            self.auto_gait_controller.execute_turn_movement(angle_degrees=angle_degrees)
            return True
        except Exception as e:
            print(f"自动步态转身失败: {e}")
            return False
    
    def test_auto_gait_functions(self):
        """测试自动步态功能"""
        if not self.auto_gait_controller:
            print("错误: 自动步态控制器未初始化")
            return False
        
        print("=== 开始自动步态功能测试 ===")
        
        try:
            # 测试前进
            print("1. 测试前进 0.3m...")
            self.auto_gait_controller.execute_forward_movement(distance=0.3)
            rospy.sleep(1.0)
            
            # 测试横移
            print("2. 测试横移 0.2m...")
            self.auto_gait_controller.execute_strafe_movement(distance=0.2)
            rospy.sleep(1.0)
            
            # 测试转身
            print("3. 测试转身 90度...")
            self.auto_gait_controller.execute_turn_movement(angle_degrees=90)
            rospy.sleep(1.0)
            
            print("=== 自动步态功能测试完成 ===")
            return True
            
        except Exception as e:
            print(f"自动步态测试失败: {e}")
            return False

def main():
    # 初始化ROS节点
    rospy.init_node('enhanced_slope_controller', anonymous=True)
    
    print("增强版6D步态控制器启动")
    print("正在连接ROS节点...")
    
    # 创建增强控制器
    controller = EnhancedSlopeController()
    
    # 预设参数
    SLOPE_FORWARD_DISTANCE = 0.9
    SLOPE_STEP_LENGTH = 0.10
    UP_SLOPE_ANGLE = 9.0
    DOWN_SLOPE_ANGLE = 6.0
    TURN_ANGLE = -180.0
    PRESET_FIRST_FORWARD = 0.5
    PRESET_MID_FORWARD = 0.1
    PRESET_LAST_FORWARD = 0.5
    SECOND_LAST_FORWARD = 0.2
    LAST_FORWARD = 0.1
    
    print("控制器初始化完成")
    
    while not rospy.is_shutdown():
        try:
            # 获取用户选择
            command = get_user_input()
            
            if command == 'quit':
                print("退出程序...")
                break
                
            elif command == 'slope':
                print(f"执行上斜坡: 距离={SLOPE_FORWARD_DISTANCE}m, 步长={SLOPE_STEP_LENGTH}m, 坡度={UP_SLOPE_ANGLE}°")
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = controller.plan_continuous_slope_movement(
                    SLOPE_FORWARD_DISTANCE, SLOPE_STEP_LENGTH, UP_SLOPE_ANGLE)
                
            elif command == 'downslope':
                print(f"执行下斜坡: 距离={SLOPE_FORWARD_DISTANCE}m, 步长={SLOPE_STEP_LENGTH}m, 坡度={DOWN_SLOPE_ANGLE}°")
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = controller.plan_continuous_slope_movement(
                    SLOPE_FORWARD_DISTANCE, SLOPE_STEP_LENGTH, -DOWN_SLOPE_ANGLE)
                
            elif command == 'turn':
                # 使用6D轨迹控制转身
                while True:
                    try:
                        angle_input = input(f"请输入转身角度 (默认: {TURN_ANGLE}°): ").strip()
                        if angle_input == "":
                            turn_angle = TURN_ANGLE
                        else:
                            turn_angle = float(angle_input)
                        if -360 <= turn_angle <= 360:
                            break
                        else:
                            print("转身角度必须在-360°到360°之间")
                    except ValueError:
                        print("请输入有效的数字")
                
                if turn_angle > 0:
                    direction = "向左"
                elif turn_angle < 0:
                    direction = "向右"
                else:
                    direction = "不转向"
                
                print(f"使用6D轨迹转身: {abs(turn_angle)}° {direction}")
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = controller.plan_turn_movement(turn_angle)
                
            elif command == 'forward':
                # 使用自动步态控制前进
                while True:
                    try:
                        dx_input = input(f"请输入前进距离 (默认: 0.2m): ").strip()
                        dx = 0.2 if dx_input == "" else float(dx_input)
                        break
                    except ValueError:
                        print("请输入有效的数字")
                
                print(f"使用自动步态前进: dx={dx}m")
                success = controller.execute_auto_gait_forward(distance=dx)
                if success:
                    print("前进完成")
                else:
                    print("前进失败")
                continue
            
            elif command == 'strafe':
                # 使用6D轨迹控制横移
                while True:
                    try:
                        dy_input = input(f"请输入横移距离 (默认: 0.2m): ").strip()
                        dy = 0.2 if dy_input == "" else float(dy_input)
                        break
                    except ValueError:
                        print("请输入有效的数字")
                
                print(f"使用6D轨迹横移: dy={dy}m")
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = controller.plan_forward_movement(0.0, dy)
                
            elif command == 'auto_gait_test':
                # 测试自动步态功能
                controller.test_auto_gait_functions()
                continue
            
            elif command == 'preset':
                # 预设1: 混合使用斜坡控制和自动步态控制
                print("=== 执行预设1 (混合模式) ===")
                print("序列: 上斜坡(6D) → 下斜坡(6D)")
                
                # 上斜坡 (6D轨迹)
                print("1. 上斜坡...")
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = controller.plan_continuous_slope_movement(
                    SLOPE_FORWARD_DISTANCE, SLOPE_STEP_LENGTH, UP_SLOPE_ANGLE)
                success = controller.publish_trajectory(time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories)
                if not success:
                    print("上斜坡失败，终止预设")
                    continue
                
                execution_time = time_traj[-1] if time_traj else 0
                rospy.sleep(execution_time + 0.5)
                
                # 下斜坡 (6D轨迹)
                print("2. 下斜坡...")
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = controller.plan_continuous_slope_movement(
                    SLOPE_FORWARD_DISTANCE, SLOPE_STEP_LENGTH, -DOWN_SLOPE_ANGLE, first_step_opposite=True)
                success = controller.publish_trajectory(time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories)
                if not success:
                    print("下斜坡失败，终止预设")
                    continue
                
                execution_time = time_traj[-1] if time_traj else 0
                rospy.sleep(execution_time + 0.5)
                
                print("=== 预设1执行完成 ===")
                continue
            
            elif command == 'simple_preset':
                # 预设2: 混合使用自动步态控制和斜坡控制
                print("=== 执行预设2 (混合模式) ===")
                print("序列: 前走(自动步态) → 上斜坡(6D) → 下斜坡(6D) → 前走(自动步态)")
                
                # 前走 (自动步态)
                print("1. 前走...")
                success = controller.execute_auto_gait_forward(distance=PRESET_FIRST_FORWARD)
                if not success:
                    print("前走失败，终止预设")
                    continue
                
                # 上斜坡 (6D轨迹)
                print("2. 上斜坡...")
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = controller.plan_continuous_slope_movement(
                    SLOPE_FORWARD_DISTANCE, SLOPE_STEP_LENGTH, UP_SLOPE_ANGLE)
                success = controller.publish_trajectory(time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories)
                if not success:
                    print("上斜坡失败，终止预设")
                    continue
                
                execution_time = time_traj[-1] if time_traj else 0
                rospy.sleep(execution_time + 0.5)
                
                # 下斜坡 (6D轨迹)
                print("3. 下斜坡...")
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = controller.plan_continuous_slope_movement(
                    SLOPE_FORWARD_DISTANCE, SLOPE_STEP_LENGTH, -DOWN_SLOPE_ANGLE, first_step_opposite=True)
                success = controller.publish_trajectory(time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories)
                if not success:
                    print("下斜坡失败，终止预设")
                    continue
                
                execution_time = time_traj[-1] if time_traj else 0
                rospy.sleep(execution_time + 0.5)
                
                # 等待位置信息稳定，确保自动步态控制器能正确获取当前位置
                print("等待位置信息稳定...")
                rospy.sleep(1.0)
                
                # 前走 (自动步态)
                print("4. 前走...")
                success = controller.execute_auto_gait_forward(distance=PRESET_LAST_FORWARD)
                if not success:
                    print("前走失败，终止预设")
                    continue
                
                print("=== 预设2执行完成 ===")
                continue
            
            # 发布轨迹 (仅用于斜坡控制)
            success = controller.publish_trajectory(time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories)
            
            if success:
                print(f"轨迹发布成功，总时间: {time_traj[-1]:.1f}s")
            else:
                print("轨迹发布失败")
                
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