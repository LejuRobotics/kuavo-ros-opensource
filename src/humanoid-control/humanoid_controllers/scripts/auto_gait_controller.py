#!/usr/bin/env python3
"""
自动步态控制模块
用于将自动步态控制功能集成到 singleStepSlope-roban.py 中
支持基于状态的平滑停止
"""

import rospy
import numpy as np
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray
from ocs2_msgs.msg import mpc_observation

class AutoGaitController:
    """自动步态控制器"""
    
    def __init__(self):
        # 初始化ROS节点（如果还没有初始化）
        try:
            rospy.init_node('auto_gait_controller_pd', anonymous=True)
        except rospy.exceptions.ROSException:
            pass  # 节点已经初始化
        
        # 发布器
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 订阅器
        self.observation_sub = rospy.Subscriber('/humanoid_mpc_observation', mpc_observation, self.observation_callback)
        
        # 状态变量
        self.current_state = None
        self.is_moving = False
        self.movement_start_time = 0.0
        self.movement_duration = 0.0
        self.last_stable_time = 0.0
        self.has_movement_command = True
        self.reached_target = False
        
        # 目标位置
        self.initial_position_x = 0.0
        self.initial_position_y = 0.0
        self.initial_yaw = 0.0
        self.target_position_x = 0.0
        self.target_position_y = 0.0
        self.target_yaw = 0.0
        
        # 控制参数
        self.max_linear_velocity = 0.2  # m/s
        self.max_angular_velocity = 0.3  # rad/s
        self.dead_zone = 0.05  # 死区
        self.position_tolerance = 0.05   # m (位置容差，调整为更精确)
        self.angular_tolerance = 0.1    # rad (角度容差，约5.7度，调整为更精确)
        self.control_rate = 50  # Hz
        
        # PI控制器参数
        self.kp_position = 1.0  # 位置比例增益
        self.ki_position = 0.2  # 位置积分增益
        self.kp_yaw = 1.5       # 偏航角比例增益
        self.ki_yaw = 0.3       # 偏航角积分增益
        
        # 积分项限制
        self.max_integral_position = 0.5  # 位置积分项最大值
        self.max_integral_yaw = 0.5       # 角度积分项最大值
        
        # 速度限制
        self.max_cmd_linear_velocity = 0.15  # m/s (降低最大命令速度)
        self.max_cmd_angular_velocity = 0.25  # rad/s (降低最大命令角速度)
        
        # 步态按钮映射
        self.joy_button_map = {
            "BUTTON_STANCE": 0,  # A按钮
            "BUTTON_WALK": 3,    # Y按钮
            "BUTTON_TROT": 1     # B按钮
        }
        
        # 等待MPC observation数据
        rospy.loginfo("等待MPC observation数据...")
        while self.current_state is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("MPC observation数据已就绪")
    
    def observation_callback(self, msg):
        """MPC observation 回调函数"""
        self.current_state = msg.state.value if msg.state.value else []
        # 添加调试信息
        if len(self.current_state) >= 6:
            rospy.logdebug(f"State vector: [{self.current_state[0]:.6f}, {self.current_state[1]:.6f}, {self.current_state[2]:.6f}, {self.current_state[3]:.6f}, {self.current_state[4]:.6f}, {self.current_state[5]:.6f}]")
    
    def feet_callback(self, msg):
        """双脚位置回调函数"""
        self.feet_pos_measured = msg.data
        
    def get_current_position(self):
        """获取当前位置"""
        if len(self.current_state) >= 12:  # 确保至少有12个元素 (6位base速度 + 6位base位姿)
            # 根据状态向量结构：6位base速度(0-5) + 6位base位姿(6-11)
            pos_x = self.current_state[6] if len(self.current_state) > 6 else 0.0
            pos_y = self.current_state[7] if len(self.current_state) > 7 else 0.0
            pos_z = self.current_state[8] if len(self.current_state) > 8 else 0.0
            roll = self.current_state[9] if len(self.current_state) > 9 else 0.0
            pitch = self.current_state[10] if len(self.current_state) > 10 else 0.0
            yaw = self.current_state[11] if len(self.current_state) > 11 else 0.0
            return pos_x, pos_y, pos_z, yaw
        return 0.0, 0.0, 0.0, 0.0
    
    def get_current_velocity(self):
        """获取当前速度"""
        if len(self.current_state) >= 6:  # 确保至少有6个元素 (6位base速度)
            # 根据状态向量结构：6位base速度(0-5)
            linear_vel_x = self.current_state[0] if len(self.current_state) > 0 else 0.0
            linear_vel_y = self.current_state[1] if len(self.current_state) > 1 else 0.0
            linear_vel_z = self.current_state[2] if len(self.current_state) > 2 else 0.0
            angular_vel_roll = self.current_state[3] if len(self.current_state) > 3 else 0.0
            angular_vel_pitch = self.current_state[4] if len(self.current_state) > 4 else 0.0
            angular_vel_yaw = self.current_state[5] if len(self.current_state) > 5 else 0.0
            return linear_vel_x, linear_vel_y, angular_vel_yaw
        return 0.0, 0.0, 0.0
    
    def is_target_reached(self):
        """检查是否达到目标位置和角度"""
        if len(self.current_state) < 12:
            return False
            
        current_pos_x, current_pos_y, current_pos_z, current_yaw = self.get_current_position()
        
        # 计算位置误差
        pos_error_x = abs(current_pos_x - self.target_position_x)
        pos_error_y = abs(current_pos_y - self.target_position_y)
        pos_error = (pos_error_x**2 + pos_error_y**2)**0.5  # 欧几里得距离
        
        # 计算角度误差
        yaw_error = abs(current_yaw - self.target_yaw)
        # 处理角度环绕问题
        if yaw_error > np.pi:
            yaw_error = 2 * np.pi - yaw_error
        
        # 添加调试信息
        rospy.loginfo(f"当前位置: x={current_pos_x:.3f}, y={current_pos_y:.3f}, yaw={current_yaw:.3f}rad ({np.degrees(current_yaw):.1f}°)")
        rospy.loginfo(f"目标位置: x={self.target_position_x:.3f}, y={self.target_position_y:.3f}, yaw={self.target_yaw:.3f}rad ({np.degrees(self.target_yaw):.1f}°)")
        rospy.loginfo(f"位置误差: {pos_error:.3f}m, 角度误差: {yaw_error:.3f}rad ({np.degrees(yaw_error):.1f}°)")
        
        # 检查是否达到目标
        position_reached = pos_error < self.position_tolerance
        yaw_reached = yaw_error < self.angular_tolerance
        
        if position_reached and yaw_reached:
            rospy.loginfo(f"目标已达成: 位置误差={pos_error:.3f}m, 角度误差={np.degrees(yaw_error):.1f}°")
            return True
            
        return False
    
    def check_feet_contact_pos(self):
        """检查双脚接触位置 (参考C++代码的checkFeetContactPos)"""
        if self.current_mode != 11:  # SS mode (stance)
            return True  # 非stance模式下不需要检查
        
        if self.feet_pos_measured is None or len(self.feet_pos_measured) < 24:
            return True  # 没有双脚数据时默认通过
        
        # 计算左右脚的平均位置 (参考C++代码)
        lf_pos_x = 0.0
        rf_pos_x = 0.0
        
        # 左脚位置 (前4个点)
        for i in range(4):
            if i * 3 + 0 < len(self.feet_pos_measured):
                lf_pos_x += self.feet_pos_measured[i * 3 + 0] / 4.0
        
        # 右脚位置 (后4个点)
        for i in range(4):
            if (i + 4) * 3 + 0 < len(self.feet_pos_measured):
                rf_pos_x += self.feet_pos_measured[(i + 4) * 3 + 0] / 4.0
        
        # 检查双脚x坐标差是否小于阈值
        feet_diff = abs(lf_pos_x - rf_pos_x)
        is_parallel = feet_diff < self.feet_threshold
        
        if is_parallel:
            rospy.loginfo(f"双脚平行: 左脚x={lf_pos_x:.3f}, 右脚x={rf_pos_x:.3f}, 差值={feet_diff:.3f}m")
        else:
            rospy.logdebug(f"双脚不平: 左脚x={lf_pos_x:.3f}, 右脚x={rf_pos_x:.3f}, 差值={feet_diff:.3f}m")
        
        return is_parallel
    
    def is_velocity_stable(self):
        """检查速度是否稳定（接近零）"""
        linear_vel_x, linear_vel_y, angular_vel_z = self.get_current_velocity()
        
        linear_stable = abs(linear_vel_x) < self.velocity_threshold and abs(linear_vel_y) < self.velocity_threshold
        angular_stable = abs(angular_vel_z) < self.velocity_threshold
        
        return linear_stable and angular_stable
    
    def should_stop_movement(self):
        """判断是否应该停止移动 - 基于observation位置"""
        if not self.is_moving:
            return False
            
        current_time = rospy.Time.now().to_sec()
        
        # 检查是否有移动命令
        if self.has_movement_command:
            # 有移动命令时，继续移动
            return False
        
        # 没有移动命令时，检查位置是否达到目标
        if (self.current_state is not None and 
            len(self.current_state) >= 6 and
            self.is_target_reached() and 
            not self.reached_target):
            
            rospy.loginfo("位置已达成目标，停止移动")
            self.reached_target = True
            return True
        
        # 安全检查：检查是否超过最大移动时间
        if self.movement_duration > 0 and current_time - self.movement_start_time > self.movement_duration:
            rospy.logwarn("超过最大移动时间，强制停止")
            return True
            
        return False
    
    def switch_gait(self, gait_type):
        """切换步态类型"""
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 11
        
        if gait_type == "stance":
            joy_msg.buttons[self.joy_button_map["BUTTON_STANCE"]] = 1
        elif gait_type == "walk":
            joy_msg.buttons[self.joy_button_map["BUTTON_WALK"]] = 1
        elif gait_type == "trot":
            joy_msg.buttons[self.joy_button_map["BUTTON_TROT"]] = 1
        
        self.joy_pub.publish(joy_msg)
        rospy.sleep(0.2)  # 等待步态切换
        rospy.loginfo(f"切换到 {gait_type} 步态")
    
    def send_velocity_command(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """发送速度命令"""
        # 计算速度命令 (参考C++代码的逻辑)
        # 将目标速度转换为-1到1的输入值，然后乘以速度限制
        input_x = linear_x / self.max_linear_velocity if self.max_linear_velocity > 0 else 0.0
        input_y = linear_y / self.max_linear_velocity if self.max_linear_velocity > 0 else 0.0
        input_yaw = angular_z / self.max_angular_velocity if self.max_angular_velocity > 0 else 0.0
        
        # 限制输入值在-1到1之间
        input_x = np.clip(input_x, -1.0, 1.0)
        input_y = np.clip(input_y, -1.0, 1.0)
        input_yaw = np.clip(input_yaw, -1.0, 1.0)
        
        # 应用死区检查
        if abs(input_x) < self.dead_zone:
            input_x = 0.0
        if abs(input_y) < self.dead_zone:
            input_y = 0.0
        if abs(input_yaw) < self.dead_zone:
            input_yaw = 0.0
        
        # 计算最终速度命令
        cmd_linear_x = input_x * self.max_linear_velocity
        cmd_linear_y = input_y * self.max_linear_velocity
        cmd_angular_z = input_yaw * self.max_angular_velocity
        
        # 减少日志输出频率，避免阻塞
        # rospy.loginfo(f"速度命令: linear_x={cmd_linear_x:.3f}, linear_y={cmd_linear_y:.3f}, angular_z={cmd_angular_z:.3f}")
        
        # 发布速度命令
        cmd_vel = Twist()
        cmd_vel.linear.x = cmd_linear_x
        cmd_vel.linear.y = cmd_linear_y
        cmd_vel.angular.z = cmd_angular_z
        
        self.cmd_vel_pub.publish(cmd_vel)
    
    def stop_movement(self):
        """停止移动"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        self.is_moving = False
        self.has_movement_command = False  # 清除移动命令标志
        self.last_stable_time = 0.0
        self.reached_target = False  # 重置目标达成标志
        rospy.loginfo("停止移动")
        
        # 切换到stance模式确保稳定
        rospy.sleep(0.3)  # 等待停止完成
        self.switch_gait("stance")
        rospy.loginfo("已切换到stance模式")
    
    def execute_auto_gait_movement_with_state_monitoring(self, linear_x, linear_y, angular_z, max_duration, target_distance_x=0.0, target_distance_y=0.0, target_angle=0.0):
        """执行自动步态移动 - 简化版本，只使用时间控制"""
        rospy.loginfo(f"开始执行自动步态移动，参数: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}, max_duration={max_duration}")
        
        # 1. 切换到walk步态
        self.switch_gait("walk")
        rospy.sleep(0.2)  # 等待步态切换
        
        # 2. 初始化移动状态
        self.is_moving = True
        self.movement_start_time = rospy.Time.now().to_sec()
        self.movement_duration = max_duration
        
        rospy.loginfo(f"移动状态初始化完成: is_moving={self.is_moving}")
        
        # 3. 开始移动
        rospy.loginfo(f"开始自动步态移动: 速度x={linear_x:.3f}m/s, y={linear_y:.3f}m/s, z={angular_z:.3f}rad/s, 持续时间={max_duration:.1f}s")
        
        # 移动监控循环
        rate = rospy.Rate(20)  # 20Hz检查频率，减少系统负载
        start_time = rospy.Time.now().to_sec()
        command_count = 0
        
        while self.is_moving and not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            elapsed_time = current_time - start_time
            command_count += 1
            
            # 检查是否超时
            if elapsed_time >= max_duration:
                rospy.loginfo(f"移动时间达到 {max_duration:.1f}s，停止移动 (已发送 {command_count} 个命令)")
                break
            
            # 发送速度命令
            self.send_velocity_command(linear_x, linear_y, angular_z)
            
            # 每50个命令输出一次状态
            if command_count % 50 == 0:
                rospy.loginfo(f"移动中... 已执行 {elapsed_time:.1f}s, 剩余 {max_duration - elapsed_time:.1f}s")
            
            rate.sleep()
        
        # 4. 停止移动并切换到stance
        rospy.loginfo(f"停止移动，总共发送了 {command_count} 个命令")
        self.send_velocity_command(0.0, 0.0, 0.0)
        self.is_moving = False
        
        # 5. 切换到stance步态
        rospy.sleep(0.5)  # 等待速度命令生效
        self.switch_gait("stance")
        rospy.sleep(0.2)
        
        rospy.loginfo("自动步态移动完成，已切换到stance模式")
    
    def smooth_stop(self):
        """平滑停止"""
        # 逐渐减小速度直到停止
        rate = rospy.Rate(self.control_rate)
        deceleration_steps = 10
        
        for i in range(deceleration_steps):
            # 计算当前应该的速度
            factor = (deceleration_steps - i) / deceleration_steps
            
            # 使用计算后的速度命令进行减速
            cmd_vel = Twist()
            cmd_vel.linear.x = self.target_linear_x * factor
            cmd_vel.linear.y = self.target_linear_y * factor
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.z = self.target_angular_z * factor
            
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()
        
        # 最终停止
        self.stop_movement()
        
        # 切换到stance模式确保机器人稳定
        rospy.sleep(0.5)  # 等待停止完成
        self.switch_gait("stance")
        rospy.loginfo("已切换到stance模式确保稳定")
    
    def execute_auto_gait_movement(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0, duration=2.0):
        """执行自动步态移动（基于时间的旧版本，保留兼容性）"""
        # 1. 切换到walk步态
        self.switch_gait("walk")
        
        # 2. 限制速度范围
        linear_x = np.clip(linear_x, -self.max_linear_velocity, self.max_linear_velocity)
        linear_y = np.clip(linear_y, -self.max_linear_velocity, self.max_linear_velocity)
        linear_z = np.clip(linear_z, -self.max_linear_velocity, self.max_linear_velocity)
        angular_z = np.clip(angular_z, -self.max_angular_velocity, self.max_angular_velocity)
        
        # 3. 发布速度命令
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.linear.y = linear_y
        cmd_vel.linear.z = linear_z
        cmd_vel.angular.z = angular_z
        
        # 4. 持续发布指定时间
        start_time = rospy.Time.now()
        rate = rospy.Rate(self.control_rate)
        
        rospy.loginfo(f"开始自动步态移动: linear_x={linear_x:.2f}, linear_y={linear_y:.2f}, angular_z={angular_z:.2f}, 持续时间={duration:.1f}s")
        
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()
        
        # 5. 停止移动
        self.stop_movement()
        rospy.loginfo("自动步态移动完成")
    
    def wait_for_position_sync(self, timeout=2.0):
        """等待位置信息同步，使用rospy.wait_for_message提高效率"""
        try:
            rospy.loginfo("等待位置信息同步...")
            # 使用wait_for_message等待最新的observation消息
            msg = rospy.wait_for_message('/humanoid_mpc_observation', mpc_observation, timeout=timeout)
            if msg.state.value and len(msg.state.value) >= 12:
                current_pos_x, current_pos_y, current_pos_z, current_yaw = self.get_current_position()
                rospy.loginfo(f"位置同步完成: x={current_pos_x:.3f}, y={current_pos_y:.3f}, yaw={np.degrees(current_yaw):.1f}°")
                return True
            else:
                rospy.logwarn("位置信息不完整")
                return False
        except rospy.ROSException as e:
            rospy.logwarn(f"等待位置信息超时: {e}")
            return False
    
    def execute_forward_movement(self, distance):
        """执行前进移动 - 简化版本"""
        if not self.is_moving:
            rospy.loginfo(f"前进目标: 距离={distance:.3f}m")
            
            # 使用高效的位置同步方法
            if not self.wait_for_position_sync(timeout=2.0):
                rospy.logwarn("位置同步失败，继续执行")
            
            # 计算线速度和执行时间
            linear_velocity = 0.2  # m/s
            duration = abs(distance) / linear_velocity
            
            # 执行移动
            self.execute_auto_gait_movement_with_state_monitoring(
                linear_x=linear_velocity if distance > 0 else -linear_velocity, 
                linear_y=0.0, 
                angular_z=0.0,
                max_duration=duration
            )
        else:
            rospy.logwarn("机器人正在移动中，无法执行新的前进命令")
    
    def execute_strafe_movement(self, distance):
        """执行横移移动 - 简化版本"""
        if not self.is_moving:
            rospy.loginfo(f"横移目标: 距离={distance:.3f}m")
            
            # 计算线速度和执行时间
            linear_velocity = 0.15  # m/s (横移速度稍慢)
            duration = abs(distance) / linear_velocity
            
            # 执行移动
            self.execute_auto_gait_movement_with_state_monitoring(
                linear_x=0.0, 
                linear_y=linear_velocity if distance > 0 else -linear_velocity, 
                angular_z=0.0,
                max_duration=duration
            )
        else:
            rospy.logwarn("机器人正在移动中，无法执行新的横移命令")
    
    def execute_turn_movement(self, angle_degrees):
        """执行转身移动 - 简化版本"""
        if not self.is_moving:
            rospy.loginfo(f"转身目标: 角度={angle_degrees:.1f}°")
            
            # 计算角速度和执行时间
            angular_velocity = 0.5  # rad/s
            angle_radians = np.radians(abs(angle_degrees))
            duration = angle_radians / angular_velocity
            
            # 执行移动
            self.execute_auto_gait_movement_with_state_monitoring(
                linear_x=0.0, 
                linear_y=0.0, 
                angular_z=angular_velocity if angle_degrees > 0 else -angular_velocity,
                max_duration=duration
            )
        else:
            rospy.logwarn("机器人正在移动中，无法执行新的转身命令")
    
    def execute_complex_movement(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=2.0, use_state_monitoring=True):
        """执行复合移动（前进+横移+转向）"""
        if use_state_monitoring:
            self.execute_auto_gait_movement_with_state_monitoring(linear_x=linear_x, linear_y=linear_y, angular_z=angular_z, max_duration=duration)
        else:
            self.execute_auto_gait_movement(linear_x=linear_x, linear_y=linear_y, angular_z=angular_z, duration=duration)

if __name__ == '__main__':
    # 测试代码
    rospy.init_node('auto_gait_controller_test')
    
    controller = AutoGaitController()
    
    try:
        # 等待MPC observation数据
        rospy.loginfo("等待MPC observation数据...")
        while controller.current_state is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        if controller.current_state is not None:
            rospy.loginfo("MPC observation数据已就绪")
            
            # 测试基于状态监控的前进
            print("测试基于状态监控的前进 0.5m...")
            controller.execute_forward_movement(distance=0.5)
            rospy.sleep(1.0)
            
            # 测试基于状态监控的横移
            print("测试基于状态监控的横移 0.3m...")
            controller.execute_strafe_movement(distance=0.3)
            rospy.sleep(1.0)
            
            # 测试基于状态监控的转身
            print("测试基于状态监控的转身 90度...")
            controller.execute_turn_movement(angle_degrees=90)
            rospy.sleep(1.0)
            
            print("测试完成")
        else:
            print("未能获取MPC observation数据")
        
    except KeyboardInterrupt:
        print("测试被用户中断")
        controller.stop_movement() 