#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from kuavo_msgs.msg import footPose6DTargetTrajectories, footPose6D
from std_srvs.srv import SetBool, SetBoolRequest

def get_user_input():
    """获取用户输入"""
    print("\n" + "="*50)
    print("简化版6D步态控制器")
    print("="*50)
    print("请选择要执行的功能:")
    print("┌─" + "─"*46 + "─┐")
    print("│ 1. 上斜坡 (slope)                    │")
    print("│ 2. 下斜坡 (downslope)                │")
    print("│ 3. 转身 (turn)                        │")
    print("│ 4. 前走 (forward)                    │")
    print("│ 5. 横走 (strafe)                     │")
    print("│ 6. 预设1: 上斜坡→前走→下斜坡→前走→转身→前走 │")
    print("│ 7. 预设2: 前走→上斜坡→前走→下斜坡→前走     │")
    print("│ 8. 退出 (quit)                        │")
    print("└─" + "─"*46 + "─┘")
    
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

class SimpleSlopeController:
    def __init__(self):
        # 预设参数
        self.foot_width = 0.108535  # 脚宽度
        
        # 步态时间控制参数
        self.walk_dt = 0.6  # 行走步态周期28
        self.walk_ss_time = 0.2  # 行走支撑相时间
        self.slope_dt = 0.7  # 上/下斜坡步态周期（摆动相基准）
        self.slope_ss_time = 0.3  # 上/下斜坡支撑相时间（接触相基准）

        
        # 步长限制参数
        self.max_step_x = 0.2  # 最大步长
        self.max_step_y = 0.15
        self.max_step_yaw = 30.0
        
        # 躯干偏置参数（仅Z方向，仅用于上斜坡）
        self.torso_z_offset = 0.0  # 躯干Z方向偏置，将在上斜坡时动态计算
        
        # 状态变量
        self.total_step = 0  # 总步数
        self.is_left_foot = False  # 当前摆动脚
        
        # 全局高度管理
        self.global_height = 0.0  # 全局高度累积
        self.last_torso_z_start = 0.0  # 上次规划开始时的torso Z高度
        self.last_torso_z_end = 0.0  # 上次规划结束时的torso Z高度
        
        # 轨迹发布器
        self.traj_pub = rospy.Publisher('/humanoid_mpc_foot_pose_6d_target_trajectories', 
                                       footPose6DTargetTrajectories, queue_size=1)
        
        # 等待发布器准备就绪
        rospy.sleep(0.5)
    
    def generate_steps_6d(self, torso_pos, torso_yaw, foot_height=0):
        """根据躯干位置计算落脚点（完全复刻原始逻辑）"""
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
        """
        规划6D移动到目标位置的轨迹（完全复刻原始逻辑）
        Args:
            dx: x方向目标位移
            dy: y方向目标位移
            dyaw: yaw方向目标角度(度)
            dz: z方向目标位移（用于高度恢复）
            time_traj: 时间轨迹
            foot_idx_traj: 脚索引轨迹
            foot_traj_6d: 脚6D姿态轨迹
            torso_traj_6d: 躯干6D姿态轨迹
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
        if foot_traj_6d is None:
            foot_traj_6d = []
        if torso_traj_6d is None:
            torso_traj_6d = []
        if swing_trajectories is None:
            swing_trajectories = []
        
        current_height = 0.0  # 简化版本使用0.0作为站立高度
        
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
            dx, dy = rotated[0], rotated[1]  # 保持dz不变，用于高度恢复
        else:
            # 初始化为6D格式：[x, y, z, yaw, pitch, roll]
            current_torso_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            current_foot_pos = np.array([0.0, 0.0, current_height])
            current_yaw = 0.0

        # 计算需要的步数
        num_steps_x = max(1, int(np.ceil(abs(dx) / max_step_x)))
        num_steps_y = max(1, int(np.ceil(abs(dy) / max_step_y)))
        num_steps_yaw = max(1, int(np.ceil(abs(dyaw) / max_step_yaw)))
        num_steps_z = max(1, int(np.ceil(abs(dz) / 0.05))) if abs(dz) > 0.001 else 0  # 高度步数，每步最大0.05m
        num_steps = max(num_steps_x, num_steps_y, num_steps_yaw, num_steps_z)

        # 计算实际步长
        actual_step_x = dx / num_steps
        actual_step_y = dy / num_steps
        actual_step_yaw = dyaw / num_steps
        actual_step_z = dz / num_steps if num_steps > 0 else 0.0
        
        # 记录初始yaw角，用于计算目标yaw角
        initial_yaw = current_torso_pos[3]
        target_yaw = initial_yaw + np.radians(dyaw)
        
        # 设置起始脚（转身时优先选择左脚）
        if dyaw > 0:
            self.is_left_foot = True
        
        # 记录开始时的轨迹长度
        start_traj_len = len(foot_traj_6d)
        num_steps += 1  # 第一步和最后一步是半步
        # 使用类变量中的时间参数
        walk_dt = self.walk_dt
        walk_ss_time = self.walk_ss_time

        print(f"规划6D移动: dx={dx:.3f}m, dy={dy:.3f}m, dyaw={dyaw:.1f}°, dz={dz:.3f}m, 步数={num_steps}")

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
                current_torso_pos[2] += actual_step_z/2  # 添加高度变化
                current_torso_pos[3] = current_torso_yaw
                # 根据当前yaw角度计算落脚点偏移
                desire_torso_pos = [current_torso_pos[0]+actual_step_x/2, current_torso_pos[1]+actual_step_y/2, current_torso_pos[2]]
                lf_foot, rf_foot = self.generate_steps_6d(desire_torso_pos, current_torso_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            elif i == num_steps - 1:
                current_torso_pos[0] += actual_step_x/2
                current_torso_pos[1] += actual_step_y/2
                current_torso_pos[2] += actual_step_z/2  # 添加高度变化
                current_torso_pos[3] = target_yaw  # 最后一步确保达到目标yaw角
                # 根据当前yaw角度计算落脚点偏移
                lf_foot, rf_foot = self.generate_steps_6d(current_torso_pos[:3], current_torso_pos[3], current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            else:
                current_torso_pos[0] += actual_step_x
                current_torso_pos[1] += actual_step_y
                current_torso_pos[2] += actual_step_z  # 添加高度变化
                current_torso_pos[3] = current_torso_yaw
                # 根据当前yaw角度计算落脚点偏移
                desire_torso_pos = [current_torso_pos[0]+actual_step_x/2, current_torso_pos[1]+actual_step_y/2, current_torso_pos[2]]
                lf_foot, rf_foot = self.generate_steps_6d(desire_torso_pos, current_torso_yaw, current_height)
                current_foot_pos = lf_foot if self.is_left_foot else rf_foot
            
            # 添加轨迹点（6D格式：[x, y, z, yaw, pitch, roll]）
            foot_traj_6d.append([current_foot_pos[0], current_foot_pos[1], current_foot_pos[2], current_torso_pos[3], 0.0, 0.0])
            # 确保躯干轨迹是6D格式：[x, y, z, yaw, pitch, roll]
            torso_6d = [current_torso_pos[0], current_torso_pos[1], current_torso_pos[2], current_torso_pos[3], 0.0, 0.0]
            torso_traj_6d.append(torso_6d)
            swing_trajectories.append(None)  # 6D版本暂时不使用腾空相轨迹

            # 添加支撑相
            time_traj.append(time_traj[-1] + walk_ss_time)
            foot_idx_traj.append(2)
            foot_traj_6d.append(foot_traj_6d[-1].copy())
            torso_traj_6d.append(torso_traj_6d[-1].copy())
            swing_trajectories.append(None)  # 6D版本不使用footPoses
        
        return time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories
        
    def publish_trajectory(self, time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories=None):
        """发布6D轨迹"""
        try:
            # 等待发布器准备就绪
            rospy.sleep(0.1)
            
            # 记录规划开始时的torso Z高度
            if len(torso_traj_6d) > 0:
                self.last_torso_z_start = torso_traj_6d[0][2]
            
            msg = footPose6DTargetTrajectories()
            msg.timeTrajectory = time_traj
            msg.footIndexTrajectory = foot_idx_traj
            msg.footPoseTrajectory = []
            msg.swingHeightTrajectory = []
            
            # 创建完整的6D脚姿态轨迹
            for i in range(len(time_traj)):
                foot_pose_msg = footPose6D()
                
                # 添加全局高度到脚部轨迹（躯干不需要）
                foot_pose_6d_with_global = list(foot_traj_6d[i])
                foot_pose_6d_with_global[2] += self.global_height  # 添加全局高度到Z坐标
                
                foot_pose_msg.footPose6D = foot_pose_6d_with_global  # 6D脚姿态 [x, y, z, yaw, pitch, roll]
                foot_pose_msg.torsoPose6D = torso_traj_6d[i]  # 6D躯干姿态 [x, y, z, yaw, pitch, roll]
                msg.footPoseTrajectory.append(foot_pose_msg)
                msg.swingHeightTrajectory.append(0.06)  # 默认摆动高度
            
            # 记录规划结束时的torso Z高度
            if len(torso_traj_6d) > 0:
                self.last_torso_z_end = torso_traj_6d[-1][2]
                
                # 统一策略：根据躯干高度差累计更新全局高度（与上斜坡一致）
                height_diff = self.last_torso_z_end - self.last_torso_z_start
                self.global_height += height_diff
            
            # 检查发布者状态
            if self.traj_pub.get_num_connections() == 0:
                print("警告: 没有订阅者连接到话题")
            else:
                print(f"订阅者数量: {self.traj_pub.get_num_connections()}")
            
            self.traj_pub.publish(msg)
            
            # 等待消息发送完成
            rospy.sleep(0.2)
            print(f"轨迹发布成功，长度: {len(time_traj)}")
            
            return True
            
        except Exception as e:
            print(f"轨迹发布失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def plan_continuous_slope_movement(self, forward_distance=2.0, step_length=0.1, slope_angle=5.0, start_with_left_foot=None):
        """规划连续上斜坡/下斜坡轨迹"""
        # 判断是上斜坡还是下斜坡
        is_downslope = slope_angle < 0
        action_name = "下斜坡" if is_downslope else "上斜坡"
        abs_slope_angle = abs(slope_angle)
        
        print(f"规划{action_name}: 距离={forward_distance}m, 步长={step_length}m, 坡度={abs_slope_angle}°")
        
        # 根据前进距离和步长自动计算步数
        num_steps = max(1, int(np.ceil(forward_distance / step_length)))
        
        # 重新计算实际步长，确保总距离准确
        actual_step_length = forward_distance / num_steps
        
        # 初始化轨迹
        time_traj = []
        foot_idx_traj = []
        foot_traj_6d = []
        torso_traj_6d = []
        swing_trajectories = []
        
        # 将坡度角度转换为弧度
        slope_rad = np.radians(abs_slope_angle)
        if is_downslope:
            slope_rad = -slope_rad  # 下斜坡时坡度为负
        
        # 初始状态
        current_torso_pos = [0.0, 0.0, 0.0]  # 躯干位置
        current_yaw = 0.0  # 朝向
        current_height = 0.0  # 当前高度
        # 如果指定了起始脚，使用指定值；否则默认从左脚开始
        is_left_foot = start_with_left_foot if start_with_left_foot is not None else True
        
        # 计算每步的高度增量
        height_per_step = actual_step_length * np.tan(slope_rad)
        
        # 动态计算Z方向偏移：根据上下斜坡方向调整
        if is_downslope:
            # 下斜坡时降低躯干，但不要降太多，避免重心不稳
            self.torso_z_offset = -2*height_per_step  # 减少偏移量
        else:
            # 上斜坡时拉高躯干
            self.torso_z_offset = -2*height_per_step
        
        for i in range(num_steps):
            # 更新躯干位置
            # 前几步特殊处理
            if (i == 0 or i == 1):
                # 前两步特殊处理：脚迈出更远距离，躯干移动较小
                first_step_length = min(actual_step_length * 1.2, 0.2)  # 迈步距离更大，但不超过0.2m
                if is_downslope:
                    # 下坡时前几步躯干移动更保守，避免重心不稳
                    current_torso_pos[0] += actual_step_length * 0.5  # 躯干移动更小
                    current_torso_pos[2] += height_per_step
                    current_height += height_per_step
                    
                    # 脚位置：相对于躯干前移，但不要太多
                    if is_left_foot:
                        foot_pos = [current_torso_pos[0] + first_step_length , self.foot_width, current_height]
                    else:
                        foot_pos = [current_torso_pos[0] + first_step_length , -self.foot_width, current_height]
                else:
                    # 上坡时保持原有逻辑
                    current_torso_pos[0] += actual_step_length / 2  # 躯干移动较小
                    current_torso_pos[2] += height_per_step
                    current_height += height_per_step
                    
                    # 脚位置：相对于躯干前移更远
                    if is_left_foot:
                        foot_pos = [current_torso_pos[0] + first_step_length, self.foot_width, current_height]
                    else:
                        foot_pos = [current_torso_pos[0] + first_step_length, -self.foot_width, current_height]
                    
            else:
                # 后续步骤：需要考虑到前两步已经移动了更大的距离
                # 前两步累计移动了 actual_step_length/3 * 2 = actual_step_length * 2/3
                # 但脚迈出了 actual_step_length * 1.5 * 2 = actual_step_length * 3
                # 所以后续步骤需要基于脚的实际位置来规划
                
                # 计算前两步脚的实际位置
                if i == 2:
                    # 第三步：基于前两步脚的实际位置
                    # 前两步脚的位置是 actual_step_length * 1.5 * 2 = actual_step_length * 3
                    # 躯干位置是 actual_step_length * 2/3
                    # 所以第三步躯干应该移动到 actual_step_length * 3 的位置
                    target_torso_x = actual_step_length * 3
                    current_torso_pos[0] = target_torso_x
                else:
                    # 第四步及以后：正常叠加
                    current_torso_pos[0] += actual_step_length
                
                current_torso_pos[2] += height_per_step
                current_height += height_per_step
                
                # 正常步骤的脚位置
                if is_left_foot:
                    foot_pos = [current_torso_pos[0], self.foot_width, current_height]
                else:
                    foot_pos = [current_torso_pos[0], -self.foot_width, current_height]
            
            # 计算脚部pitch角度（适应坡度）
            foot_pitch = -slope_rad
            
            # 计算torso的pitch角度（适应坡度）
            if is_downslope:
                # 下坡时躯干应该后倾，保持重心稳定,实物测试后倾会倒，暂时不调节躯干pitch
                torso_pitch = 0.0*slope_rad  
            else:
                # 上坡时躯干前倾，适应坡度
                torso_pitch = 0.6*slope_rad
            
            # 下斜坡最后一步不再硬编码归零，保持与上斜坡一致的高度累计策略
            
            # 应用躯干Z方向偏置（取消最后两步特殊恢复，统一在预设末尾的move_to恢复）
            torso_pos_with_offset = [
                current_torso_pos[0],
                current_torso_pos[1],
                current_torso_pos[2] + self.torso_z_offset
            ]
            
            # 添加摆动相
            time_traj.append((i + 1) * self.slope_dt)
            foot_idx_traj.append(0 if is_left_foot else 1)
            foot_traj_6d.append([foot_pos[0], foot_pos[1], foot_pos[2], current_yaw, foot_pitch, 0.0])
            torso_traj_6d.append([torso_pos_with_offset[0], torso_pos_with_offset[1], torso_pos_with_offset[2], current_yaw, torso_pitch, 0.0])
            swing_trajectories.append(None)
            
            # 添加支撑相
            time_traj.append((i + 1) * self.slope_dt + self.slope_ss_time)
            foot_idx_traj.append(2)
            foot_traj_6d.append(foot_traj_6d[-1].copy())
            # 支撑相时torso保持相同的pitch角度和偏置
            torso_traj_6d.append(torso_traj_6d[-1].copy())
            swing_trajectories.append(None)
            
            # 切换脚
            is_left_foot = not is_left_foot
        
        print(f"{action_name}规划完成: 距离={current_torso_pos[0]:.3f}m, 高度={current_torso_pos[2]:.3f}m")
        
        # 上斜坡/下斜坡完成后，添加一个move_to来恢复躯干姿态
        recovery_distance = 0.15  # 恢复距离，仿照stairClimbPlanner-roban.py
        
        # 恢复Z方向偏移为0
        original_torso_z_offset = self.torso_z_offset
        self.torso_z_offset = 0.0
        
        # 检测上斜坡/下斜坡最后一步是左脚还是右脚
        # 需要找到最后一个摆动相的脚索引（不是支撑相的索引2）
        last_swing_idx = -2  # 倒数第二个是最后一个摆动相
        while last_swing_idx >= -len(foot_idx_traj) and foot_idx_traj[last_swing_idx] == 2:
            last_swing_idx -= 1
        
        if last_swing_idx >= -len(foot_idx_traj):
            last_foot_idx = foot_idx_traj[last_swing_idx]
            last_foot_is_left = (last_foot_idx == 1)  # 0表示左脚，1表示右脚，2表示支撑相
            print(f"{action_name}最后摆动脚: {'左脚' if last_foot_is_left else '右脚'}")
        else:
            # 如果找不到摆动相，使用is_left_foot变量的状态
            last_foot_is_left = is_left_foot
            print(f"无法检测摆动脚，使用当前状态: {'左脚' if last_foot_is_left else '右脚'}")
        
        # 设置恢复move_to的起始脚为另一只脚
        if last_foot_is_left:
            # 最后一步是左脚，恢复move_to从右脚开始
            self.is_left_foot = False
        else:
            # 最后一步是右脚，恢复move_to从左脚开始
            self.is_left_foot = True
        
        # 使用plan_move_to_6d添加恢复轨迹（仅补偿躯干Z偏置）
        dz_recover = -original_torso_z_offset  # 只补偿offset，不改变累计高度
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

    def execute_simple_preset_sequence(self, forward_distance, step_length, up_slope_angle, down_slope_angle, mid_forward=0.1, first_forward=0.1, last_forward=0.1):
        """执行预设2序列：前走first_forward -> 上斜坡 -> 前走mid_forward -> 下斜坡 -> 前走last_forward"""
        print(f"=== 执行预设2 ===")
        print(f"序列: 前走{first_forward}m -> 上斜坡({up_slope_angle}°) -> 前走{mid_forward}m -> 下斜坡({down_slope_angle}°) -> 前走{last_forward}m")
        
        try:
            # 保存前一步的轨迹信息，用于脚检测
            prev_foot_idx_traj = None
            
            # 定义动作序列（包含恢复move_to的额外等待时间）
            actions = [
                (f"前走{first_forward}m", lambda: self.plan_forward_movement(first_forward), 0.2),
                ("上斜坡", lambda: self.plan_continuous_slope_movement(forward_distance, step_length, up_slope_angle), 1.0),  # 额外等待3秒包含恢复move_to
                (f"前走{mid_forward}m", lambda: self.plan_forward_movement(mid_forward), 0.2),
                ("下斜坡", lambda: self.plan_continuous_slope_movement(forward_distance, step_length, -down_slope_angle), 0.5),  # 额外等待3秒包含恢复move_to
                (f"前走{last_forward}m", lambda: self.plan_forward_movement(last_forward), 0.0),
            ]
            
            # 执行每个动作
            for i, (action_name, action_func, extra_wait) in enumerate(actions, 1):
                print(f"\n--- 步骤{i}: {action_name} ---")
                
                # 特殊处理：如果是上斜坡且前面有前走，检测前走的最后摆动相
                if action_name == "上斜坡" and prev_foot_idx_traj is not None:
                    last_swing_idx = -2  # 倒数第二个是最后一个摆动相
                    while last_swing_idx >= -len(prev_foot_idx_traj) and prev_foot_idx_traj[last_swing_idx] == 2:
                        last_swing_idx -= 1
                    
                    if last_swing_idx >= -len(prev_foot_idx_traj):
                        last_foot_idx = prev_foot_idx_traj[last_swing_idx]
                        last_foot_is_left = (last_foot_idx == 0)  # 0表示左脚，1表示右脚，2表示支撑相
                        print(f"前走最后摆动脚: {'左脚' if last_foot_is_left else '右脚'}")
                        
                        # 上斜坡的起始脚应该和前走最后一步的摆动脚相反
                        upslope_start_left = not last_foot_is_left
                        print(f"上斜坡起始脚: {'左脚' if upslope_start_left else '右脚'}")
                        
                        # 重新定义上斜坡的lambda函数，包含起始脚参数
                        action_func = lambda: self.plan_continuous_slope_movement(forward_distance, step_length, up_slope_angle, start_with_left_foot=upslope_start_left)
                    else:
                        print("无法检测摆动脚，使用默认左脚")
                
                # 特殊处理：如果是下斜坡且前面有动作，检测前一个动作的最后摆动相
                if action_name == "下斜坡" and prev_foot_idx_traj is not None:
                    last_swing_idx = -2  # 倒数第二个是最后一个摆动相
                    while last_swing_idx >= -len(prev_foot_idx_traj) and prev_foot_idx_traj[last_swing_idx] == 2:
                        last_swing_idx -= 1
                    
                    if last_swing_idx >= -len(prev_foot_idx_traj):
                        last_foot_idx = prev_foot_idx_traj[last_swing_idx]
                        last_foot_is_left = (last_foot_idx == 0)  # 0表示左脚，1表示右脚，2表示支撑相
                        print(f"前一个动作最后摆动脚: {'左脚' if last_foot_is_left else '右脚'}")
                        
                        # 下斜坡的起始脚应该和前一个动作最后一步的摆动脚相反
                        downslope_start_left = not last_foot_is_left
                        print(f"下斜坡起始脚: {'左脚' if downslope_start_left else '右脚'}")
                        
                        # 重新定义下斜坡的lambda函数，包含起始脚参数
                        action_func = lambda: self.plan_continuous_slope_movement(forward_distance, step_length, -down_slope_angle, start_with_left_foot=downslope_start_left)
                    else:
                        print("无法检测摆动脚，使用默认左脚")
                
                # 规划轨迹
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = action_func()
                
                # 保存当前轨迹信息，供下一步使用
                if "前走" in action_name or "斜坡" in action_name:
                    prev_foot_idx_traj = foot_idx_traj
                
                # 发布轨迹
                success = self.publish_trajectory(time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories)
                if not success:
                    print(f"{action_name}轨迹发布失败，终止简单预设序列")
                    return False
                print(f"{action_name}完成")
                
                # 等待轨迹执行完成
                execution_time = time_traj[-1] if time_traj else 0
                print(f"等待执行完成: {execution_time:.1f}秒")
                rospy.sleep(execution_time)
                
                # 额外等待时间（用于包含恢复move_to等）
                if extra_wait > 0:
                    print(f"额外等待: {extra_wait}秒")
                    rospy.sleep(extra_wait)
            
            print(f"\n=== 预设2执行完成 ===")
            return True
            
        except Exception as e:
            print(f"预设2执行出错: {e}")
            import traceback
            traceback.print_exc()
            return False

    def execute_preset_sequence(self, forward_distance, step_length, up_slope_angle, turn_angle, second_last_forward=0.2, last_forward=0.2, down_slope_angle=None, mid_forward=0.1):
        """执行预设1序列：上斜坡 -> 前走mid_forward -> 下斜坡 -> 前走Xm -> 转身180° -> 前走Ym"""
        # 如果没有指定下坡角度，使用上坡角度
        if down_slope_angle is None:
            down_slope_angle = up_slope_angle
        
        print(f"=== 执行预设1 ===")
        print(f"序列: 上斜坡({up_slope_angle}°) -> 前走{mid_forward}m -> 下斜坡({down_slope_angle}°) -> 前走{second_last_forward}m -> 转身{turn_angle}° -> 前走{last_forward}m")
        
        try:
            # 保存前一步的轨迹信息，用于脚检测
            prev_foot_idx_traj = None
            
            # 定义动作序列（包含恢复move_to的额外等待时间）
            actions = [
                ("上斜坡", lambda: self.plan_continuous_slope_movement(forward_distance, step_length, up_slope_angle), 1.0),  # 额外等待3秒包含恢复move_to
                (f"前走{mid_forward}m", lambda: self.plan_forward_movement(mid_forward), 0.2),
                ("下斜坡", lambda: self.plan_continuous_slope_movement(forward_distance, step_length, -down_slope_angle), 0.5),  # 额外等待3秒包含恢复move_to
                (f"前走{second_last_forward}m", lambda: self.plan_forward_movement(second_last_forward), 0.0),
                ("转身", lambda: self.plan_turn_movement(turn_angle), 0.0),
                (f"前走{last_forward}m", lambda: self.plan_forward_movement(last_forward), 0.0)
            ]
            
            # 执行每个动作
            for i, (action_name, action_func, extra_wait) in enumerate(actions, 1):
                print(f"\n--- 步骤{i}: {action_name} ---")
                
                # 特殊处理：如果是上斜坡且前面有前走，检测前走的最后摆动相
                if action_name == "上斜坡" and prev_foot_idx_traj is not None:
                    last_swing_idx = -2  # 倒数第二个是最后一个摆动相
                    while last_swing_idx >= -len(prev_foot_idx_traj) and prev_foot_idx_traj[last_swing_idx] == 2:
                        last_swing_idx -= 1
                    
                    if last_swing_idx >= -len(prev_foot_idx_traj):
                        last_foot_idx = prev_foot_idx_traj[last_swing_idx]
                        last_foot_is_left = (last_foot_idx == 0)  # 0表示左脚，1表示右脚，2表示支撑相
                        print(f"前走最后摆动脚: {'左脚' if last_foot_is_left else '右脚'}")
                        
                        # 上斜坡的起始脚应该和前走最后一步的摆动脚相反
                        upslope_start_left = not last_foot_is_left
                        print(f"上斜坡起始脚: {'左脚' if upslope_start_left else '右脚'}")
                        
                        # 重新定义上斜坡的lambda函数，包含起始脚参数
                        action_func = lambda: self.plan_continuous_slope_movement(forward_distance, step_length, up_slope_angle, start_with_left_foot=upslope_start_left)
                    else:
                        print("无法检测摆动脚，使用默认左脚")
                
                # 特殊处理：如果是下斜坡且前面有动作，检测前一个动作的最后摆动相
                if action_name == "下斜坡" and prev_foot_idx_traj is not None:
                    last_swing_idx = -2  # 倒数第二个是最后一个摆动相
                    while last_swing_idx >= -len(prev_foot_idx_traj) and prev_foot_idx_traj[last_swing_idx] == 2:
                        last_swing_idx -= 1
                    
                    if last_swing_idx >= -len(prev_foot_idx_traj):
                        last_foot_idx = prev_foot_idx_traj[last_swing_idx]
                        last_foot_is_left = (last_foot_idx == 0)  # 0表示左脚，1表示右脚，2表示支撑相
                        print(f"前一个动作最后摆动脚: {'左脚' if last_foot_is_left else '右脚'}")
                        
                        # 下斜坡的起始脚应该和前一个动作最后一步的摆动脚相反
                        downslope_start_left = not last_foot_is_left
                        print(f"下斜坡起始脚: {'左脚' if downslope_start_left else '右脚'}")
                        
                        # 重新定义下斜坡的lambda函数，包含起始脚参数
                        action_func = lambda: self.plan_continuous_slope_movement(forward_distance, step_length, -down_slope_angle, start_with_left_foot=downslope_start_left)
                    else:
                        print("无法检测摆动脚，使用默认左脚")
                
                # 规划轨迹
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = action_func()
                
                # 保存当前轨迹信息，供下一步使用
                if "前走" in action_name or "斜坡" in action_name:
                    prev_foot_idx_traj = foot_idx_traj
                
                # 发布轨迹
                success = self.publish_trajectory(time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories)
                if not success:
                    print(f"{action_name}轨迹发布失败，终止预设序列")
                    return False
                print(f"{action_name}完成")
                
                # 等待轨迹执行完成
                execution_time = time_traj[-1] if time_traj else 0
                print(f"等待执行完成: {execution_time:.1f}秒")
                rospy.sleep(execution_time)
                
                # 额外等待时间（用于包含恢复move_to等）
                if extra_wait > 0:
                    print(f"额外等待: {extra_wait}秒")
                    rospy.sleep(extra_wait)
            
            print(f"\n=== 预设1执行完成 ===")
            return True
            
        except Exception as e:
            print(f"预设1执行出错: {e}")
            import traceback
            traceback.print_exc()
            return False

def main():
    # 初始化ROS节点
    rospy.init_node('simple_slope_controller', anonymous=True)
    
    print("简化版6D步态控制器启动")
    print("正在连接ROS节点...")
    
    # 创建控制器
    controller = SimpleSlopeController()
    
    # 预设参数（在主函数中设定）
    SLOPE_FORWARD_DISTANCE = 0.9  # 上斜坡前进距离
    SLOPE_STEP_LENGTH = 0.10  # 上斜坡每步长度
    UP_SLOPE_ANGLE = 9.0      # 上斜坡角度（默认9度）
    DOWN_SLOPE_ANGLE = 6.0    # 下斜坡角度（默认6度）下斜坡坡度设定大了实物会跺脚
    TURN_ANGLE = -180.0        # 转身角度
    PRESET_FIRST_FORWARD = 0.3  # 预设第一段前走距离（可修改）
    PRESET_MID_FORWARD = 0.1   # 预设中间前走距离（可修改）
    PRESET_LAST_FORWARD = 0.3  # 预设最后一段前走距离（可修改）
    SECOND_LAST_FORWARD = 0.2  # 倒数第二段前走距离
    LAST_FORWARD = 0.1        # 最后一段前走距离
    
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
                    SLOPE_FORWARD_DISTANCE, SLOPE_STEP_LENGTH, -DOWN_SLOPE_ANGLE) # 下斜坡时坡度为负
                
            elif command == 'turn':
                # 获取转身参数
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
                
                # 判断转向方向
                if turn_angle > 0:
                    direction = "向左"
                elif turn_angle < 0:
                    direction = "向右"
                else:
                    direction = "不转向"
                
                print(f"执行转身: {abs(turn_angle)}° {direction}")
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = controller.plan_turn_movement(turn_angle)
                
            elif command == 'forward':
                # 获取仅前后位移 dx（dx可为负表示后退）
                while True:
                    try:
                        dx_input = input(f"请输入前后位移dx (默认: 0.2m，可负值后退): ").strip()
                        dx = 0.2 if dx_input == "" else float(dx_input)
                        break
                    except ValueError:
                        print("请输入有效的数字")
                
                print(f"执行前走: dx={dx}m")
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = controller.plan_forward_movement(dx, 0.0)
            
            elif command == 'strafe':
                # 获取侧向位移 dy（左正右负）
                while True:
                    try:
                        dy_input = input(f"请输入侧向位移dy (默认: 0.0m，左正右负): ").strip()
                        dy = 0.0 if dy_input == "" else float(dy_input)
                        break
                    except ValueError:
                        print("请输入有效的数字")
                
                print(f"执行横走: dy={dy}m")
                time_traj, foot_idx_traj, foot_traj_6d, torso_traj_6d, swing_trajectories = controller.plan_forward_movement(0.0, dy) # 横走时dx为0
            
            elif command == 'preset':
                # 获取坡度选择
                print(f"当前预设坡度: 上坡{UP_SLOPE_ANGLE}°, 下坡{DOWN_SLOPE_ANGLE}°")
                while True:
                    try:
                        slope_choice = input("是否使用自定义坡度? (y/n, 默认n): ").strip().lower()
                        if slope_choice in ['', 'n', 'no']:
                            up_angle = UP_SLOPE_ANGLE
                            down_angle = DOWN_SLOPE_ANGLE
                            break
                        elif slope_choice in ['y', 'yes']:
                            up_input = input(f"请输入上坡角度 (默认{UP_SLOPE_ANGLE}°): ").strip()
                            up_angle = float(up_input) if up_input else UP_SLOPE_ANGLE
                            down_input = input(f"请输入下坡角度 (默认{DOWN_SLOPE_ANGLE}°): ").strip()
                            down_angle = float(down_input) if down_input else DOWN_SLOPE_ANGLE
                            break
                        else:
                            print("请输入 y 或 n")
                    except ValueError:
                        print("请输入有效的数字")
                
                print(f"执行预设1: 上斜坡({up_angle}°) → 前走{PRESET_MID_FORWARD}m → 下斜坡({down_angle}°) → 前走{SECOND_LAST_FORWARD}m → 转身{TURN_ANGLE}° → 前走{LAST_FORWARD}m")
                controller.execute_preset_sequence(SLOPE_FORWARD_DISTANCE, SLOPE_STEP_LENGTH, up_angle, TURN_ANGLE, SECOND_LAST_FORWARD, LAST_FORWARD, down_angle, PRESET_MID_FORWARD)
                continue  # 跳过后续的轨迹发布
            
            elif command == 'simple_preset':
                # 获取坡度选择
                print(f"当前预设坡度: 上坡{UP_SLOPE_ANGLE}°, 下坡{DOWN_SLOPE_ANGLE}°")
                while True:
                    try:
                        slope_choice = input("是否使用自定义坡度? (y/n, 默认n): ").strip().lower()
                        if slope_choice in ['', 'n', 'no']:
                            up_angle = UP_SLOPE_ANGLE
                            down_angle = DOWN_SLOPE_ANGLE
                            break
                        elif slope_choice in ['y', 'yes']:
                            up_input = input(f"请输入上坡角度 (默认{UP_SLOPE_ANGLE}°): ").strip()
                            up_angle = float(up_input) if up_input else UP_SLOPE_ANGLE
                            down_input = input(f"请输入下坡角度 (默认{DOWN_SLOPE_ANGLE}°): ").strip()
                            down_angle = float(down_input) if down_input else DOWN_SLOPE_ANGLE
                            break
                        else:
                            print("请输入 y 或 n")
                    except ValueError:
                        print("请输入有效的数字")
                
                print(f"执行预设2: 前走{PRESET_FIRST_FORWARD}m → 上斜坡({up_angle}°) → 前走{PRESET_MID_FORWARD}m → 下斜坡({down_angle}°) → 前走{PRESET_LAST_FORWARD}m")
                controller.execute_simple_preset_sequence(SLOPE_FORWARD_DISTANCE, SLOPE_STEP_LENGTH, up_angle, down_angle, PRESET_MID_FORWARD, PRESET_FIRST_FORWARD, PRESET_LAST_FORWARD)
                continue  # 跳过后续的轨迹发布
            
            # 发布轨迹
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