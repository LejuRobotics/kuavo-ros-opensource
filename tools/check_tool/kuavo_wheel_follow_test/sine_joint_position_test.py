#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
正弦波关节位置控制测试脚本
基于 S60JointController 进行选择关节的正弦位置控制测试
"""

import rospy
import numpy as np
import time
import signal
import sys
import argparse
from geometry_msgs.msg import Pose, Point, Quaternion
from kuavo_msgs.msg import jointCmd, sensorsData
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
        
        # 正弦波参数 - 单关节测试
        self.test_joint_index = None  # 被测关节索引
        self.sine_amplitude = 0.0     # 振幅
        self.sine_frequency = 0.0     # 频率
        self.sine_phase = 0.0         # 相位
        
        # 测试参数
        self.test_duration = 3.0  # 测试持续时间（秒）
        self.control_frequency = 500.0  # 控制频率（Hz）
        
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
        """传感器数据回调函数"""
        self.current_positions = list(msg.joint_data.joint_q)
    
    def set_joint_sine_params(self, joint_index, amplitude, frequency, phase=0.0):
        """
        设置单关节正弦波参数
        Args:
            joint_index: 关节序号
            amplitude: 振幅（弧度）
            frequency: 频率（Hz）
            phase: 相位（弧度）
        """
        if joint_index < 0 or joint_index >= self.max_test_joints:
            rospy.logwarn(f"关节序号超出范围，只支持0-{self.max_test_joints-1}，实际：{joint_index}")
            return False
        
        self.test_joint_index = joint_index
        self.sine_amplitude = amplitude
        self.sine_frequency = frequency
        self.sine_phase = phase
        
        rospy.loginfo(f"设置关节[{joint_index}]: 振幅={amplitude:.3f}, 频率={frequency:.2f}Hz, 相位={phase:.3f}")
        return True
    
    def set_base_positions(self):
        """
        设置基准位置, 使用当前位置
        """
        if len(self.current_positions) == self.num_joints:
            self.base_positions = self.current_positions.copy()
            rospy.loginfo("使用当前位置作为基准位置")
        else:
            rospy.logwarn("当前位置数据不完整，使用零位置作为基准")
            self.base_positions = [0.0] * self.num_joints
        
        rospy.loginfo(f"基准位置: {[f'{p:.3f}' for p in self.base_positions[:]]}...")
        return True
    
    def compute_sine_positions(self, t):
        """
        计算当前时刻的正弦波位置
        Args:
            t: 时间（秒）
        Returns:
            list: 目标关节位置
        """
        # 使用当前位置作为基础，只修改被测关节
        target_positions = self.base_positions.copy()
        
        if self.test_joint_index is not None:
            # 计算正弦波位置：base + amplitude * sin(2π * frequency * t + phase)
            sine_offset = self.sine_amplitude * np.sin(2 * np.pi * self.sine_frequency * t + self.sine_phase)
            target_positions[self.test_joint_index] = self.base_positions[self.test_joint_index] + sine_offset
        
        return target_positions
    
    def create_joint_cmd_msg(self, target_positions):
        """
        创建关节命令消息 - 基于S60JointController的实现
        Args:
            target_positions: 目标关节位置列表
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
        
        # 设置目标位置（转换为度数）
        # 只设置被测关节的位置，其他关节保持当前位置
        for i in range(self.num_joints):
            msg.joint_q[i] = target_positions[i]
            msg.control_modes[i] = 2  # 位置控制模式
            
            msg.joint_kp[i] = 0
            msg.joint_kd[i] = 0
        
        return msg
    
    def send_joint_cmd(self, target_positions):
        """发送关节命令"""
        msg = self.create_joint_cmd_msg(target_positions)
        self.joint_cmd_pub.publish(msg)
        self.joint_ref_cmd_pub.publish(msg)
    
    def run_sine_test(self):
        """运行正弦波测试"""
        if self.test_joint_index is None:
            rospy.logwarn("没有设置测试关节，退出测试")
            return
        
        rospy.loginfo("========== 开始正弦波关节位置测试 ==========")
        rospy.loginfo(f"测试持续时间: {self.test_duration}秒")
        rospy.loginfo(f"控制频率: {self.control_frequency}Hz")
        rospy.loginfo(f"测试关节: [{self.test_joint_index}]")
        
        # 显示关节参数
        rospy.loginfo(f"  关节[{self.test_joint_index}]: 振幅={self.sine_amplitude:.3f}rad({np.degrees(self.sine_amplitude):.1f}°), "
                     f"频率={self.sine_frequency:.2f}Hz, 相位={self.sine_phase:.3f}rad({np.degrees(self.sine_phase):.1f}°)")
        
        rate = rospy.Rate(self.control_frequency)
        start_time = time.time()

        self.base_positions = self.current_positions.copy()
        
        while not rospy.is_shutdown() and running:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # 检查是否超时
            if elapsed_time >= self.test_duration:
                rospy.loginfo("测试完成")
                break
            
            # 计算目标位置
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
            status_str = f"[{t:.1f}s] 关节[{self.test_joint_index}]: 目标={target:.3f}, 当前={current:.3f}, 误差={error:.3f}"
            rospy.loginfo(status_str)
    


def main():
    try:
        # 命令行参数解析
        parser = argparse.ArgumentParser(description='正弦波关节位置控制测试（单关节测试）')
        parser.add_argument('--joint_index', type=int, required=True,
                          help='关节序号')
        parser.add_argument('--amplitude', type=float, required=True,
                          help='正弦波振幅（弧度）')
        parser.add_argument('--sine_freq', type=float, required=True,
                          help='正弦波频率（Hz）')
        parser.add_argument('--duration', type=float, default=3.0,
                          help='测试持续时间（秒）')
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
        if args.joint_index < 0 or args.joint_index >= tester.max_test_joints:
            rospy.logerr(f"关节序号超出范围，只支持0-{tester.max_test_joints-1}，实际：{args.joint_index}")
            return
        
        rospy.loginfo(f"单关节测试: 关节[{args.joint_index}]")
        rospy.loginfo(f"参数: 振幅={args.amplitude:.3f}rad({np.degrees(args.amplitude):.1f}°), 频率={args.sine_freq:.2f}Hz")
        
        tester.set_joint_sine_params(args.joint_index, args.amplitude, args.sine_freq)
        
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