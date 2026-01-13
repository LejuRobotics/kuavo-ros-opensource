#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
正弦波关节力矩控制测试脚本
基于 S60JointController 对选择关节进行正弦力矩控制测试
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

class SineJointTorqueTester:
    def __init__(self):
        rospy.init_node('sine_joint_torque_tester', anonymous=True)
        
        # 关节配置
        self.num_joints = 20  # 总关节数
        self.max_test_joints = 20  # 可测试的关节数
        
        # 状态变量
        self.current_positions = [0.0] * self.num_joints
        self.current_torques = [0.0] * self.num_joints
        
        # 正弦波参数 - 单关节测试
        self.test_joint_index = None  # 被测关节索引
        self.sine_amplitude = 0.0     # 振幅（Nm）
        self.sine_frequency = 0.0     # 频率（Hz）
        self.sine_phase = 0.0         # 相位（rad）
        
        # 测试参数
        self.test_duration = 3.0  # 测试持续时间（秒）
        self.control_frequency = 500.0  # 控制频率（Hz）
        
        # ROS接口设置
        self._setup_ros_interface()
        
        # 等待连接
        self.wait_for_connections()
        
        rospy.loginfo("正弦波关节力矩测试器初始化完成")
    
    def _setup_ros_interface(self):
        """设置ROS发布者和订阅者"""
        # 发布者 - 与S60JointController保持一致
        self.joint_cmd_pub = rospy.Publisher('/joint_cmd', jointCmd, queue_size=10)
        
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
        self.current_torques = list(msg.joint_data.joint_torque)
    
    def set_joint_sine_params(self, joint_index, amplitude, frequency, phase=0.0):
        """
        设置单关节正弦波参数
        Args:
            joint_index: 关节序号
            amplitude: 振幅（Nm）
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
        
        rospy.loginfo(f"设置关节[{joint_index}]: 力矩振幅={amplitude:.3f}Nm, 频率={frequency:.2f}Hz, 相位={phase:.3f}rad")
        return True
    
    def compute_sine_torque(self, t):
        """
        计算当前时刻的正弦波力矩
        Args:
            t: 时间（秒）
        Returns:
            list: 目标关节力矩
        """
        target_torque = [0.0] * self.num_joints
        
        if self.test_joint_index is not None:
            # tau = amplitude * sin(2π f t + phase)
            sine_tau = self.sine_amplitude * np.sin(2 * np.pi * self.sine_frequency * t + self.sine_phase)
            target_torque[self.test_joint_index] = float(sine_tau)
        
        return target_torque
    
    def create_joint_cmd_msg(self, target_tau, current_q):
        """
        创建关节命令消息（力矩控制）
        Args:
            target_tau: 目标关节力矩列表
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
        
        for i in range(self.num_joints):
            if i == self.test_joint_index:
                # 被测关节：使用正弦波目标力矩
                msg.tau[i] = target_tau[i]
                msg.control_modes[i] = 2  # 力矩控制模式
            else:
                # 非测试关节：保持当前位置（位置控制）
                if i < len(current_q):
                    msg.joint_q[i] = current_q[i]
            
            msg.joint_kp[i] = 0
            msg.joint_kd[i] = 0
        
        return msg
    
    def send_joint_cmd(self, target_tau, current_q):
        """发送关节命令"""
        msg = self.create_joint_cmd_msg(target_tau, current_q)
        self.joint_cmd_pub.publish(msg)
    
    def run_sine_test(self):
        """运行正弦波力矩测试"""
        if self.test_joint_index is None:
            rospy.logwarn("没有设置测试关节，退出测试")
            return
        
        rospy.loginfo("========== 开始正弦波关节力矩测试 ==========")
        rospy.loginfo(f"测试持续时间: {self.test_duration}秒")
        rospy.loginfo(f"控制频率: {self.control_frequency}Hz")
        rospy.loginfo(f"测试关节: [{self.test_joint_index}]")
        
        # 显示关节参数
        rospy.loginfo(f"  关节[{self.test_joint_index}]: 力矩振幅={self.sine_amplitude:.3f}Nm, 频率={self.sine_frequency:.2f}Hz, 相位={self.sine_phase:.3f}rad")
        
        rate = rospy.Rate(self.control_frequency)
        start_time = time.time()

        target_q =  self.current_positions.copy()
        
        while not rospy.is_shutdown() and running:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # 检查是否超时
            if elapsed_time >= self.test_duration:
                rospy.loginfo("测试完成")
                break
            
            # 计算目标力矩
            target_tau = self.compute_sine_torque(elapsed_time)
            
            # 发送命令
            self.send_joint_cmd(target_tau, target_q)
            
            # 打印状态（每0.1秒打印一次）
            if int(elapsed_time * 10) % 10 == 0:
                self.print_joint_status(elapsed_time, target_tau)
            
            rate.sleep()
        
        rospy.loginfo("========== 正弦波测试结束 ==========")
    
    def print_joint_status(self, t, target_tau):
        """打印关节状态信息"""
        if self.test_joint_index is not None:
            target = target_tau[self.test_joint_index]
            measured = self.current_torques[self.test_joint_index] if self.test_joint_index < len(self.current_torques) else 0.0
            error = abs(target - measured)
            status_str = f"[{t:.1f}s] 关节[{self.test_joint_index}]: 目标力矩={target:.3f}Nm, 实测力矩={measured:.3f}Nm, 误差={error:.3f}"
            rospy.loginfo(status_str)
    


def main():
    try:
        # 命令行参数解析
        parser = argparse.ArgumentParser(description='正弦波关节力矩控制测试（单关节测试）')
        parser.add_argument('--joint_index', type=int, required=True,
                          help='关节序号')
        parser.add_argument('--amplitude', type=float, required=True,
                          help='正弦波力矩振幅（Nm）')
        parser.add_argument('--sine_freq', type=float, required=True,
                          help='正弦波频率（Hz）')
        parser.add_argument('--duration', type=float, default=3.0,
                          help='测试持续时间（秒）')
        parser.add_argument('--frequency', type=float, default=500.0,
                          help='控制频率（Hz）')
        
        args = parser.parse_args()
        
        # 创建测试器
        tester = SineJointTorqueTester()
        tester.test_duration = args.duration
        tester.control_frequency = args.frequency
        
        # 等待传感器数据
        rospy.sleep(1.0)
        
        # 配置测试
        if args.joint_index < 4 or args.joint_index >= tester.max_test_joints:
            rospy.logerr(f"关节序号超出范围，只支持4-{tester.max_test_joints-1}，实际：{args.joint_index}")
            return
        
        rospy.loginfo(f"单关节测试: 关节[{args.joint_index}]")
        rospy.loginfo(f"参数: 力矩振幅={args.amplitude:.3f}Nm, 频率={args.sine_freq:.2f}Hz")
        
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