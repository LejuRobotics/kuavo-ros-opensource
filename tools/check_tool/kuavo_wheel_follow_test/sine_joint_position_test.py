#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
正弦波关节位置控制测试脚本
基于 S60JointController 进行选择关节的正弦位置控制测试

重要说明：
- 本脚本中所有角度相关的参数（振幅、相位、位置、速度等）均使用角度制（度），而非弧度制
- 传感器数据（弧度）会在接收时自动转换为度
- 发送给硬件的命令（弧度）会在发送时自动从度转换为弧度
- 用户输入的所有角度参数均为度
"""

import rospy
import numpy as np
import time
import signal
import sys
import argparse
from geometry_msgs.msg import Pose, Point, Quaternion
from kuavo_msgs.msg import jointCmd, sensorsData
from kuavo_msgs.srv import jointMoveTo
from std_msgs.msg import Header
import math

# 全局控制变量
running = True

def signal_handler(signum, frame):
    global running
    print(f"\n接收到信号 {signum}，正在退出...")
    running = False
    rospy.signal_shutdown("接收到退出信号")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

class SineJointPositionTester:
    def __init__(self):
        rospy.init_node('sine_joint_position_tester', anonymous=True)
        
        # 关节配置
        self.num_joints = 20  # 总关节数
        self.max_test_joints = 20  # 可测试的关节数
        
        # 状态变量
        self.current_positions = [0.0] * self.num_joints
        self.base_positions = [0.0] * self.num_joints  # 初始位置
        
        # 每个关节的目标位置（度）- 在开始正弦波运动前先移动到此位置
        # 关节ID从1开始，这里使用字典存储，键为关节ID（1-20）
        self.joint_target_positions = {
            1: 10.0,   # 关节1的目标位置（度）
            2: -10.0,  # 关节2的目标位置（度）
            3: 1.0,    # 关节3的目标位置（度）
            4: 0.0,    # 关节4的目标位置（度）
            5: 0.0,    # 关节5的目标位置（度）
            6: 0.0,    # 关节6的目标位置（度）
            7: 0.0,    # 关节7的目标位置（度）
            8: -50.0,  # 关节8的目标位置（度）
            9: 0.0,    # 关节9的目标位置（度）
            10: 0.0,   # 关节10的目标位置（度）
            11: 0.0,   # 关节11的目标位置（度）
            12: 0.0,   # 关节12的目标位置（度）
            13: 0.0,   # 关节13的目标位置（度）
            14: 0.0,   # 关节14的目标位置（度）
            15: -50.0, # 关节15的目标位置（度）
            16: 0.0,   # 关节16的目标位置（度）
            17: 0.0,   # 关节17的目标位置（度）
            18: 0.0,   # 关节18的目标位置（度）
            19: 0.0,   # 关节19的目标位置（度）
            20: 0.0,   # 关节20的目标位置（度）
        }
        
        # 每个关节移动到目标位置的速度（度/秒）- 在移动到目标位置时使用
        # 关节ID从1开始，这里使用字典存储，键为关节ID（1-20）
        self.joint_move_speeds = {
            1: 2.0,   # 关节1的移动速度（度/秒）
            2: 2.0,   # 关节2的移动速度（度/秒）
            3: 2.0,   # 关节3的移动速度（度/秒）
            4: 2.0,   # 关节4的移动速度（度/秒）
            5: 2.0,   # 关节5的移动速度（度/秒）
            6: 2.0,   # 关节6的移动速度（度/秒）
            7: 2.0,   # 关节7的移动速度（度/秒）
            8: 2.0,   # 关节8的移动速度（度/秒）
            9: 2.0,   # 关节9的移动速度（度/秒）
            10: 2.0,  # 关节10的移动速度（度/秒）
            11: 2.0,  # 关节11的移动速度（度/秒）
            12: 2.0,  # 关节12的移动速度（度/秒）
            13: 2.0,  # 关节13的移动速度（度/秒）
            14: 2.0,  # 关节14的移动速度（度/秒）
            15: 2.0,  # 关节15的移动速度（度/秒）
            16: 2.0,  # 关节16的移动速度（度/秒）
            17: 2.0,  # 关节17的移动速度（度/秒）
            18: 2.0,  # 关节18的移动速度（度/秒）
            19: 2.0,  # 关节19的移动速度（度/秒）
            20: 2.0,  # 关节20的移动速度（度/秒）
        }
        
        # 正弦波参数 - 单关节测试
        self.test_joint_id = None     # 被测关节ID（用户输入，从1开始）
        self.test_joint_index = None  # 被测关节索引（内部使用，从0开始）
        self.sine_amplitude = 0.0     # 振幅（度）
        self.sine_frequency = 0.0     # 频率（Hz）
        self.sine_phase = 0.0         # 相位（度）
        
        # 测试参数
        self.test_duration = None  # 测试持续时间（秒），None表示持续运行直到退出
        self.control_frequency = 500.0  # 控制频率（Hz）
        self.move_to_target_timeout = 5.0  # 移动到目标位置的超时时间（秒）
        self.move_to_target_threshold = 0.05  # 到达目标位置的阈值（度）
        
        # ROS接口设置
        self._setup_ros_interface()
        
        # 等待连接
        self.wait_for_connections()
        
        rospy.loginfo("正弦波关节位置测试器初始化完成")
    
    def _setup_ros_interface(self):
        """设置ROS发布者和订阅者"""
        # 发布者 - 与S60JointController保持一致
        self.joint_cmd_pub = rospy.Publisher('/joint_cmd', jointCmd, queue_size=10)
        self.joint_ref_cmd_pub = rospy.Publisher('/ref_joint_cmd', jointCmd, queue_size=10)
        
        # 订阅者
        self.sensors_sub = rospy.Subscriber('/sensors_data_raw', sensorsData, self.sensors_callback)
        
        # ROS服务客户端 - 用于调用jointMoveTo服务
        self.joint_move_to_service = None
        try:
            rospy.wait_for_service('/hardware/joint_move_to', timeout=2.0)
            self.joint_move_to_service = rospy.ServiceProxy('/hardware/joint_move_to', jointMoveTo)
            rospy.loginfo("jointMoveTo服务连接成功")
        except rospy.ROSException:
            rospy.logwarn("jointMoveTo服务不可用，将使用位置控制方式移动关节")
    
    def wait_for_connections(self):
        """等待ROS连接"""
        rate = rospy.Rate(10)
        start_time = time.time()
        timeout = 10.0
        
        rospy.loginfo("等待ROS连接...")
        while not rospy.is_shutdown() and running:
            joint_subscribers = self.joint_cmd_pub.get_num_connections()
            sensor_publishers = self.sensors_sub.get_num_connections()
            
            rospy.loginfo(f"连接状态: joint_cmd订阅者={joint_subscribers}, sensors发布者={sensor_publishers}")
            
            if sensor_publishers > 0:
                rospy.loginfo("传感器数据连接已建立")
                break
            
            if time.time() - start_time > timeout:
                rospy.logwarn("连接超时，继续运行...")
                break
                
            rate.sleep()
    
    def sensors_callback(self, msg):
        """
        传感器数据回调函数
        注意：传感器数据（sensorsData.joint_q）的单位是弧度，这里自动转换为度存储
        以便脚本内部统一使用角度制（度）
        """
        # 传感器数据返回的是弧度，转换为度存储
        self.current_positions = [np.degrees(q) for q in msg.joint_data.joint_q]
    
    def set_joint_sine_params(self, joint_id, amplitude, frequency, phase=0.0):
        """
        设置单关节正弦波参数
        Args:
            joint_id: 关节ID（从1开始，1-20）
            amplitude: 振幅（度）
            frequency: 频率（Hz）
            phase: 相位（度）
        """
        if joint_id < 1 or joint_id > self.max_test_joints:
            rospy.logwarn(f"关节ID超出范围，只支持1-{self.max_test_joints}，实际：{joint_id}")
            return False
        
        self.test_joint_id = joint_id
        self.test_joint_index = joint_id - 1  # 转换为数组索引（从0开始）
        self.sine_amplitude = amplitude  # 存储为度
        self.sine_frequency = frequency
        self.sine_phase = phase  # 存储为度
        
        rospy.loginfo(f"设置关节[{joint_id}]: 振幅={amplitude:.3f}°, 频率={frequency:.2f}Hz, 相位={phase:.3f}°")
        return True
    
    def set_base_positions(self):
        """
        设置基准位置，使用当前位置
        注意：基准位置以度为单位（已从传感器数据的弧度转换）
        """
        if len(self.current_positions) == self.num_joints:
            self.base_positions = self.current_positions.copy()
            rospy.loginfo("使用当前位置作为基准位置")
        else:
            rospy.logwarn("当前位置数据不完整，使用零位置作为基准")
            self.base_positions = [0.0] * self.num_joints
        
        rospy.loginfo(f"基准位置（度）: {[f'{p:.3f}' for p in self.base_positions[:]]}...")
        return True
    
    def move_joint_to_target_position_via_service(self, joint_id, target_position_deg):
        """
        使用线性插值在5秒内将指定关节移动到目标位置
        Args:
            joint_id: 关节ID（从1开始）
            target_position_deg: 目标位置（度）
        Returns:
            bool: 是否成功
        """
        if joint_id < 1 or joint_id > self.max_test_joints:
            rospy.logwarn(f"关节ID超出范围: {joint_id}")
            return False
        
        joint_index = joint_id - 1
        
        # 获取起始位置
        if joint_index >= len(self.current_positions):
            rospy.logwarn(f"无法获取关节[{joint_id}]的当前位置")
            return False
        
        start_pos = self.current_positions[joint_index]
        distance = abs(target_position_deg - start_pos)
        
        rospy.loginfo(f"移动关节[{joint_id}]到目标位置: {target_position_deg:.3f}度 (起始: {start_pos:.3f}度, 距离: {distance:.3f}度)")
        rospy.loginfo(f"使用线性插值，5秒内移动到目标位置")
        
        # 插值时间：5秒
        interpolation_duration = 5.0
        
        # 创建目标位置数组（其他关节保持当前位置）
        target_positions = self.current_positions.copy()
        
        # 线性插值移动
        rate = rospy.Rate(self.control_frequency)
        start_time = time.time()
        
        rospy.loginfo(f"开始5秒线性插值移动关节[{joint_id}]...")
        
        while not rospy.is_shutdown() and running:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # 如果超过5秒，直接设置为目标位置并退出
            if elapsed_time >= interpolation_duration:
                target_positions = self.current_positions.copy()
                target_positions[joint_index] = target_position_deg
                self.send_joint_cmd(target_positions)
                rospy.loginfo(f"5秒插值完成，关节[{joint_id}]已到达目标位置")
                break
            
            # 计算插值比例 (0.0 到 1.0)
            t = elapsed_time / interpolation_duration
            t = min(t, 1.0)  # 确保不超过1.0
            
            # 线性插值：current = start + (target - start) * t
            interpolated_pos = start_pos + (target_position_deg - start_pos) * t
            
            # 更新目标位置数组（其他关节保持当前位置）
            target_positions = self.current_positions.copy()
            target_positions[joint_index] = interpolated_pos
            
            # 发送位置命令（与正弦波控制方式一致）
            self.send_joint_cmd(target_positions)
            
            # 每1秒打印一次进度
            if int(elapsed_time * 10) % 10 == 0:  # 每0.1秒检查一次，但只在整秒打印
                rospy.loginfo(f"[{elapsed_time:.1f}s] 关节[{joint_id}]: 插值位置={interpolated_pos:.3f}度, 目标={target_position_deg:.3f}度, 进度={t*100:.1f}%")
            
            rate.sleep()
        
        rospy.loginfo(f"关节[{joint_id}]插值移动完成")
        
        return True
    
    def move_joint_to_target_position(self, joint_id, target_position_deg):
        """
        将指定关节移动到目标位置
        Args:
            joint_id: 关节ID（从1开始）
            target_position_deg: 目标位置（度）
        Returns:
            bool: 是否成功到达目标位置
        """
        if joint_id < 1 or joint_id > self.max_test_joints:
            rospy.logwarn(f"关节ID超出范围: {joint_id}")
            return False
        
        joint_index = joint_id - 1
        
        # 获取该关节的移动速度
        move_speed_deg_per_sec = self.joint_move_speeds.get(joint_id, 10.0)  # 默认10度/秒
        rospy.loginfo(f"移动关节[{joint_id}]到目标位置: {target_position_deg:.3f}度, 速度: {move_speed_deg_per_sec:.2f}度/秒")
        
        # 创建目标位置数组（其他关节保持当前位置）
        target_positions = self.current_positions.copy()
        target_positions[joint_index] = target_position_deg
        
        # 创建目标速度数组（只有目标关节有速度，其他为0）
        target_velocities = [0.0] * self.num_joints
        # 计算速度方向：如果目标位置大于当前位置，速度为正，否则为负
        if joint_index < len(self.current_positions):
            current_pos = self.current_positions[joint_index]
            if target_position_deg > current_pos:
                target_velocities[joint_index] = move_speed_deg_per_sec
            elif target_position_deg < current_pos:
                target_velocities[joint_index] = -move_speed_deg_per_sec
            # 如果已经到达，速度为0
        
        # 发送位置命令
        rate = rospy.Rate(self.control_frequency)
        start_time = time.time()
        last_error = float('inf')
        
        while not rospy.is_shutdown() and running:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # 检查超时
            if elapsed_time > self.move_to_target_timeout:
                rospy.logwarn(f"移动到目标位置超时（{self.move_to_target_timeout}秒）")
                break
            
            # 动态更新速度方向（根据当前位置和目标位置）
            if joint_index < len(self.current_positions):
                current_pos = self.current_positions[joint_index]
                error = target_position_deg - current_pos
                if abs(error) > 0.01:  # 如果还有误差
                    if error > 0:
                        target_velocities[joint_index] = move_speed_deg_per_sec
                    else:
                        target_velocities[joint_index] = -move_speed_deg_per_sec
                else:
                    target_velocities[joint_index] = 0.0  # 接近目标时速度设为0
            
            # 发送命令（带速度）
            self.send_joint_cmd(target_positions, target_velocities)
            
            # 检查是否到达目标位置
            if joint_index < len(self.current_positions):
                current_pos = self.current_positions[joint_index]
                error = abs(current_pos - target_position_deg)
                
                # 如果误差小于阈值，认为已到达
                if error < self.move_to_target_threshold:
                    rospy.loginfo(f"关节[{joint_id}]已到达目标位置: 当前={current_pos:.3f}度, 目标={target_position_deg:.3f}度, 误差={error:.3f}度")
                    return True
                
                # 如果误差不再减小，可能卡住了
                if error >= last_error - 0.001:  # 误差基本不变
                    if elapsed_time > 1.0:  # 至少运动1秒后检查
                        rospy.logwarn(f"关节[{joint_id}]可能已到达（误差不再减小）: 当前={current_pos:.3f}度, 目标={target_position_deg:.3f}度, 误差={error:.3f}度")
                        return True
                
                last_error = error
            
            rate.sleep()
        
        # 最终检查
        if joint_index < len(self.current_positions):
            current_pos = self.current_positions[joint_index]
            error = abs(current_pos - target_position_deg)
            rospy.loginfo(f"关节[{joint_id}]移动完成: 当前={current_pos:.3f}度, 目标={target_position_deg:.3f}度, 误差={error:.3f}度")
            return error < self.move_to_target_threshold * 2  # 放宽阈值
        
        return False
    
    def compute_sine_positions(self, t):
        """
        计算当前时刻的正弦波位置
        Args:
            t: 时间（秒）
        Returns:
            list: 目标关节位置（度）
        """
        # 使用当前位置作为基础，只修改被测关节
        target_positions = self.base_positions.copy()
        
        if self.test_joint_index is not None:
            # 计算正弦波位置：base + amplitude * sin(2π * frequency * t + phase)
            # 注意：振幅和相位存储为度，sin函数需要弧度，所以需要转换
            # 公式：offset = amplitude * sin(2π * frequency * t + phase_rad)
            # 振幅保持为度（不转换），相位转换为弧度用于sin函数
            phase_rad = np.radians(self.sine_phase)  # 相位转换为弧度
            # 振幅是度，sin函数返回-1到1之间的值，所以结果单位是度
            sine_offset_deg = self.sine_amplitude * np.sin(2 * np.pi * self.sine_frequency * t + phase_rad)
            target_positions[self.test_joint_index] = self.base_positions[self.test_joint_index] + sine_offset_deg
        
        return target_positions
    
    def create_joint_cmd_msg(self, target_positions, target_velocities=None):
        """
        创建关节命令消息 - 基于S60JointController的实现
        Args:
            target_positions: 目标关节位置列表（度）
            target_velocities: 目标关节速度列表（度/秒），如果为None则使用0
        Returns:
            jointCmd: 关节命令消息
        """
        msg = jointCmd()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        
        # 初始化所有数组
        msg.joint_q = [0.0] * self.num_joints
        msg.joint_v = [0.0] * self.num_joints
        msg.tau = [0.0] * self.num_joints
        msg.tau_max = [100.0] * self.num_joints
        msg.tau_ratio = [1.0] * self.num_joints
        msg.joint_kp = [0.0] * self.num_joints
        msg.joint_kd = [0.0] * self.num_joints
        msg.control_modes = [2] * self.num_joints 
        
        # 设置目标位置和速度
        for i in range(self.num_joints):
            # 注意：脚本内部统一使用度，但jointCmd消息需要弧度
            # 因此发送前需要将度转换为弧度
            msg.joint_q[i] = np.radians(target_positions[i])  # 度 -> 弧度
            msg.control_modes[i] = 2  # 位置控制模式
            
            # 如果提供了速度，则使用提供的速度，否则使用0
            # 速度也需要从度/秒转换为弧度/秒
            if target_velocities is not None and i < len(target_velocities):
                msg.joint_v[i] = np.radians(target_velocities[i])  # 度/秒 -> 弧度/秒
            else:
                msg.joint_v[i] = 0.0
            
            msg.joint_kp[i] = 0
            msg.joint_kd[i] = 0
        
        return msg
    
    def send_joint_cmd(self, target_positions, target_velocities=None):
        """
        发送关节命令
        Args:
            target_positions: 目标关节位置列表（度）
            target_velocities: 目标关节速度列表（度/秒），如果为None则使用0
        """
        msg = self.create_joint_cmd_msg(target_positions, target_velocities)
        self.joint_cmd_pub.publish(msg)
        self.joint_ref_cmd_pub.publish(msg)
    
    def run_sine_test(self):
        """运行正弦波测试"""
        if self.test_joint_index is None:
            rospy.logwarn("没有设置测试关节，退出测试")
            return
        
        rospy.loginfo("========== 开始正弦波关节位置测试 ==========")
        if self.test_duration is not None:
            rospy.loginfo(f"测试持续时间: {self.test_duration}秒")
        else:
            rospy.loginfo("测试持续时间: 持续运行（按Ctrl+C退出）")
        rospy.loginfo(f"控制频率: {self.control_frequency}Hz")
        rospy.loginfo(f"测试关节: [{self.test_joint_id}]")
        
        # 显示关节参数
        rospy.loginfo(f"  关节[{self.test_joint_id}]: 振幅={self.sine_amplitude:.3f}°, "
                     f"频率={self.sine_frequency:.2f}Hz, 相位={self.sine_phase:.3f}°")
        
        # 记录总开始时间（用于计算第6秒）
        total_start_time = time.time()
        
        # 第一步：将选定关节移动到预设的目标位置（5秒内线性插值）
        if self.test_joint_id in self.joint_target_positions:
            target_pos_deg = self.joint_target_positions[self.test_joint_id]
            rospy.loginfo(f"========== 步骤1: 移动关节[{self.test_joint_id}]到预设目标位置（5秒插值） ==========")
            rospy.loginfo(f"预设目标位置: {target_pos_deg:.3f}度")
            
            # 使用线性插值在5秒内移动到目标位置
            success = self.move_joint_to_target_position_via_service(self.test_joint_id, target_pos_deg)
            if success:
                rospy.loginfo(f"关节[{self.test_joint_id}]已成功移动到预设目标位置")
            else:
                rospy.logwarn(f"关节[{self.test_joint_id}]移动到预设目标位置可能未完全到达，继续执行正弦波测试")
            
            # 更新基准位置为当前到达的位置
            if self.test_joint_index < len(self.current_positions):
                self.base_positions = self.current_positions.copy()
                rospy.loginfo(f"更新基准位置: 关节[{self.test_joint_id}] = {self.base_positions[self.test_joint_index]:.3f}度")
        else:
            rospy.logwarn(f"关节[{self.test_joint_id}]没有配置预设目标位置，使用当前位置作为基准")
            self.base_positions = self.current_positions.copy()
        
        # 等待到第6秒开始正弦波运动
        current_time = time.time()
        elapsed_since_start = current_time - total_start_time
        wait_until_6s = 6.0 - elapsed_since_start
        
        if wait_until_6s > 0:
            rospy.loginfo(f"等待到第6秒开始正弦波运动（还需等待 {wait_until_6s:.2f} 秒）...")
            rospy.sleep(wait_until_6s)
        else:
            rospy.logwarn(f"已经超过6秒，立即开始正弦波运动")
        
        # 第二步：开始正弦波运动（从第6秒开始）
        rospy.loginfo(f"========== 步骤2: 开始正弦波运动（从第6秒开始） ==========")
        
        rate = rospy.Rate(self.control_frequency)
        sine_start_time = time.time()  # 正弦波开始时间
        
        while not rospy.is_shutdown() and running:
            current_time = time.time()
            # 正弦波运动的时间从第6秒开始计算（即从sine_start_time开始）
            elapsed_time = current_time - sine_start_time
            
            # 检查是否超时（如果设置了持续时间）
            if self.test_duration is not None and elapsed_time >= self.test_duration:
                rospy.loginfo("测试完成")
                break
            
            # 计算目标位置（正弦波运动）
            target_positions = self.compute_sine_positions(elapsed_time)
            
            # 发送命令
            self.send_joint_cmd(target_positions)
            
            # 打印活动关节的状态（每秒一次）
            if int(elapsed_time * 10) % 10 == 0:  # 每0.1秒打印一次，但只在整秒显示
                self.print_joint_status(elapsed_time, target_positions)
            
            rate.sleep()
        
        rospy.loginfo("========== 正弦波测试结束 ==========")
    
    def print_joint_status(self, t, target_positions):
        """打印关节状态信息"""
        if self.test_joint_index is not None:
            target = target_positions[self.test_joint_index]
            current = self.current_positions[self.test_joint_index] if self.test_joint_index < len(self.current_positions) else 0.0
            error = abs(target - current)
            status_str = f"[{t:.1f}s] 关节[{self.test_joint_id}]: 目标={target:.3f}, 当前={current:.3f}, 误差={error:.3f}"
            rospy.loginfo(status_str)
    


def main():
    """
    主函数
    注意：所有角度参数（振幅、相位）的单位都是度（°），不是弧度（rad）
    """
    try:
        # 命令行参数解析
        parser = argparse.ArgumentParser(
            description='正弦波关节位置控制测试（单关节测试）\n'
                       '重要：所有角度参数（振幅、相位）的单位都是度（°），不是弧度（rad）',
            formatter_class=argparse.RawDescriptionHelpFormatter
        )
        parser.add_argument('--joint_index', type=int, required=True,
                          help='关节ID（从1开始，1-20）')
        parser.add_argument('--amplitude', type=float, required=True,
                          help='正弦波振幅（度）')
        parser.add_argument('--sine_freq', type=float, required=True,
                          help='正弦波频率（Hz）')
        parser.add_argument('--phase', type=float, default=0.0,
                          help='正弦波相位（度），默认为0')
        parser.add_argument('--duration', type=float, default=None,
                          help='测试持续时间（秒），不指定则持续运行直到手动退出')
        parser.add_argument('--frequency', type=float, default=500.0,
                          help='控制频率（Hz）')
        
        args = parser.parse_args()
        
        # 创建测试器
        tester = SineJointPositionTester()
        tester.test_duration = args.duration
        tester.control_frequency = args.frequency
        
        # 等待传感器数据
        rospy.sleep(1.0)
        
        # 设置基准位置
        tester.set_base_positions()  # 使用当前位置作为基准
        
        # 配置测试
        if args.joint_index < 1 or args.joint_index > tester.max_test_joints:
            rospy.logerr(f"关节ID超出范围，只支持1-{tester.max_test_joints}，实际：{args.joint_index}")
            return
        
        rospy.loginfo(f"单关节测试: 关节[{args.joint_index}]")
        rospy.loginfo(f"参数: 振幅={args.amplitude:.3f}°, 频率={args.sine_freq:.2f}Hz, 相位={args.phase:.3f}°")
        
        tester.set_joint_sine_params(args.joint_index, args.amplitude, args.sine_freq, args.phase)
        
        # 运行测试
        tester.run_sine_test()
        
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        global running
        running = False
        rospy.loginfo("测试程序退出")

if __name__ == '__main__':
    main()