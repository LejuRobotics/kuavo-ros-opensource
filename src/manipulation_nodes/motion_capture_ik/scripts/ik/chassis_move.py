#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化的底盘移动测试脚本 - 输入目标姿态，自动计算差值并逐步下发
"""

import rospy
import numpy as np
import time
import signal
import sys
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

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

class ChassisController:
    def __init__(self, init_node=True):
        if init_node:
            rospy.init_node('chassis_controller', anonymous=True)
        
        # 设置发布者和订阅者
        self.chassis_pose_pub = rospy.Publisher('/chassic_pose', Pose, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 当前姿态
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.195  # 固定高度
        self.current_yaw = 0.0
        
        # 步长设置
        self.pos_step = 0.01  # 位置步长1cm
        self.yaw_step = np.radians(1.0)  # 角度步长1度
        
        # 等待连接（只在独立运行时执行）
        if init_node:
            self.wait_for_connections()
    
    def wait_for_connections(self):
        try:
            rate = rospy.Rate(10)
            start_time = time.time()
            timeout = 5.0
            
            rospy.loginfo("等待ROS连接...")
            while not rospy.is_shutdown():
                try:
                    sensor_publishers = self.sensors_sub.get_num_connections()
                    rospy.loginfo(f"连接状态: sensor_data_raw发布者={sensor_publishers}")
                    
                    if sensor_publishers > 0:
                        rospy.loginfo("传感器数据连接已建立，可以开始底盘控制")
                        break
                    
                    if time.time() - start_time > timeout:
                        rospy.logwarn("连接超时，继续运行...")
                        break
                        
                    rate.sleep()
                except Exception as e:
                    rospy.logwarn(f"等待连接时出错: {e}")
                    break
        except Exception as e:
            rospy.logwarn(f"wait_for_connections方法出错: {e}")
    
    def send_chassis_pose_quaternion(self, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
        """发送四元数姿态"""
        pose = Pose()
        pose.position = Point(x, y, z)
        pose.orientation.w = qw
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        
        self.chassis_pose_pub.publish(pose)
        # rospy.loginfo(f"发送姿态: 位置({x:.3f}, {y:.3f}, {z:.3f}), 偏航角({np.degrees(np.arctan2(2*(qw*qz), 1-2*(qz*qz))):.2f}°)")
    
    def send_cmd_vel(self, linear_x, linear_y, angular_z):
        """发送速度控制命令"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = 0.0  # 不使用
        twist.angular.x = 0.0  # 不使用
        twist.angular.y = 0.0  # 不使用
        twist.angular.z = angular_z
        
        self.cmd_vel_pub.publish(twist)
        # rospy.loginfo(f"发送速度命令: linear_x={linear_x:.3f}, linear_y={linear_y:.3f}, angular_z={angular_z:.3f}")

    def move_chassis(self, target_x, target_y, target_yaw_degrees, duration=2.0):
        """
        移动底盘到指定位置和姿态（使用位置控制）
        Args:
            target_x: 目标x坐标 (米)
            target_y: 目标y坐标 (米)
            target_yaw_degrees: 目标偏航角 (度)
            duration: 移动持续时间 (秒)
        Returns:
            bool: 是否成功完成移动
        """
        target_yaw = np.radians(target_yaw_degrees)
        target_z = self.current_z  # 保持当前高度不变
        
        rospy.loginfo(f"移动底盘到: 位置({target_x:.3f}, {target_y:.3f}), 偏航角({target_yaw_degrees:.2f}°)")
        
        # 计算差值
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dyaw = target_yaw - self.current_yaw
        
        # 标准化角度差值到[-π, π]
        while dyaw > np.pi:
            dyaw -= 2 * np.pi
        while dyaw < -np.pi:
            dyaw += 2 * np.pi
        
        # 计算步数
        pos_steps = max(abs(dx), abs(dy)) / self.pos_step
        yaw_steps = abs(dyaw) / self.yaw_step
        total_steps = max(int(pos_steps), int(yaw_steps), 1)
        
        # 根据duration调整频率
        frequency = total_steps / duration
        rate = rospy.Rate(frequency)
        
        rospy.loginfo(f"移动步数: {total_steps}, 频率: {frequency:.1f}Hz")
        
        # 逐步移动
        for step in range(total_steps):
            if rospy.is_shutdown():
                return False
            
            # 计算当前步的位置
            progress = (step + 1) / total_steps
            current_x = self.current_x + dx * progress
            current_y = self.current_y + dy * progress
            current_yaw = self.current_yaw + dyaw * progress
            
            # 转换为四元数
            qw = np.cos(current_yaw / 2)
            qz = np.sin(current_yaw / 2)
            
            # 发送姿态
            self.send_chassis_pose_quaternion(current_x, current_y, target_z, qw, 0.0, 0.0, qz)
            rate.sleep()
        
        # 更新当前位置
        self.current_x = target_x
        self.current_y = target_y
        self.current_yaw = target_yaw
        
        rospy.loginfo("底盘移动完成！")
        return True
    
    def move_chassis_velocity(self, target_x, target_y, target_yaw_degrees, duration=10.0):
        """
        使用速度控制移动底盘到指定位置和姿态（带平滑加减速）
        Args:
            target_x: 目标x坐标 (米)
            target_y: 目标y坐标 (米)
            target_yaw_degrees: 目标偏航角 (度)
            duration: 移动持续时间 (秒)
        Returns:
            bool: 是否成功完成移动
        """
        target_yaw = np.radians(target_yaw_degrees)
        
        rospy.loginfo(f"使用速度控制移动底盘到: 位置({target_x:.3f}, {target_y:.3f}), 偏航角({target_yaw_degrees:.2f}°)")
        
        # 速度控制参数
        max_linear_vel = 0.3  # 最大线速度 0.5 m/s
        max_angular_vel = 1.0  # 最大角速度 1.0 rad/s
        pos_tolerance = 0.01   # 位置误差容忍度 1cm
        yaw_tolerance = np.radians(1.0)  # 角度误差容忍度 1度
        accel_time = 0.5       # 加速时间 0.5秒
        decel_time = 0.5       # 减速时间 0.5秒
        
        # 计算总距离和角度
        total_distance = np.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
        total_angle = abs(target_yaw - self.current_yaw)
        while total_angle > np.pi:
            total_angle = 2 * np.pi - total_angle
        
        # 计算所需的最大速度
        if total_distance > 0:
            # 考虑加减速时间的有效距离
            effective_distance = total_distance - 0.5 * max_linear_vel * (accel_time + decel_time)
            if effective_distance > 0:
                required_vel = min(max_linear_vel, effective_distance / (duration - accel_time - decel_time))
            else:
                required_vel = np.sqrt(2 * total_distance / (accel_time + decel_time))
        else:
            required_vel = 0.0
        
        if total_angle > 0:
            required_ang_vel = min(max_angular_vel, total_angle / (duration - accel_time - decel_time))
        else:
            required_ang_vel = 0.0
        
        rospy.loginfo(f"总距离: {total_distance:.3f}m, 总角度: {np.degrees(total_angle):.2f}°")
        rospy.loginfo(f"计算的最大速度: 线速度={required_vel:.3f}m/s, 角速度={required_ang_vel:.3f}rad/s")
        
        # 计算梯形速度曲线的时间点
        if total_distance > 0:
            # 计算加速、匀速、减速的时间
            accel_distance = 0.5 * required_vel * accel_time
            decel_distance = 0.5 * required_vel * decel_time
            cruise_distance = total_distance - accel_distance - decel_distance
            
            if cruise_distance > 0:
                # 有匀速段
                cruise_time = cruise_distance / required_vel
                t1 = accel_time  # 加速结束时间
                t2 = t1 + cruise_time  # 匀速结束时间
                t3 = t2 + decel_time  # 减速结束时间
            else:
                # 没有匀速段，只有加减速
                # 重新计算最大速度
                required_vel = np.sqrt(2 * total_distance / (accel_time + decel_time))
                t1 = accel_time * (accel_time / (accel_time + decel_time))
                t2 = t1
                t3 = duration
        else:
            t1 = t2 = t3 = 0
        
        if total_angle > 0:
            # 角度运动的时间点
            ang_accel_time = 0.5 * required_ang_vel * accel_time
            ang_decel_time = 0.5 * required_ang_vel * decel_time
            ang_cruise_angle = total_angle - ang_accel_time - ang_decel_time
            
            if ang_cruise_angle > 0:
                ang_cruise_time = ang_cruise_angle / required_ang_vel
                at1 = accel_time
                at2 = at1 + ang_cruise_time
                at3 = at2 + decel_time
            else:
                required_ang_vel = np.sqrt(2 * total_angle / (accel_time + decel_time))
                at1 = accel_time * (accel_time / (accel_time + decel_time))
                at2 = at1
                at3 = duration
        else:
            at1 = at2 = at3 = 0
        
        rospy.loginfo(f"梯形曲线时间点: t1={t1:.2f}s, t2={t2:.2f}s, t3={t3:.2f}s")
        rospy.loginfo(f"角度梯形曲线时间点: at1={at1:.2f}s, at2={at2:.2f}s, at3={at3:.2f}s")
        
        # 持续发送速度命令直到到达目标位置
        rate = rospy.Rate(100)  # 10Hz 发送频率
        start_time = rospy.Time.now()
        timeout = max(duration * 2, 10.0)  # 超时时间
        
        # 当前速度（用于平滑控制）
        current_vel_x = 0.0
        current_vel_y = 0.0
        current_ang_vel = 0.0
        
        while not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            
            # 检查超时
            if elapsed_time > timeout:
                rospy.logwarn("移动超时，停止移动")
                break
            
            # 计算当前位置到目标位置的误差
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            dyaw = target_yaw - self.current_yaw
            
            # 标准化角度差值到[-π, π]
            while dyaw > np.pi:
                dyaw -= 2 * np.pi
            while dyaw < -np.pi:
                dyaw += 2 * np.pi
            
            # 检查是否到达目标位置
            pos_error = np.sqrt(dx**2 + dy**2)
            if pos_error < pos_tolerance and abs(dyaw) < yaw_tolerance:
                rospy.loginfo(f"已到达目标位置，误差: 位置={pos_error:.3f}m, 角度={np.degrees(abs(dyaw)):.2f}°")
                break
            
            # 计算梯形速度曲线
            if elapsed_time <= t1:
                # 加速段
                vel_scale = elapsed_time / t1 if t1 > 0 else 0
            elif elapsed_time <= t2:
                # 匀速段
                vel_scale = 1.0
            elif elapsed_time <= t3:
                # 减速段
                vel_scale = (t3 - elapsed_time) / (t3 - t2) if (t3 - t2) > 0 else 0
            else:
                # 停止
                vel_scale = 0.0
            
            # 计算角度速度曲线
            if elapsed_time <= at1:
                # 角度加速段
                ang_vel_scale = elapsed_time / at1 if at1 > 0 else 0
            elif elapsed_time <= at2:
                # 角度匀速段
                ang_vel_scale = 1.0
            elif elapsed_time <= at3:
                # 角度减速段
                ang_vel_scale = (at3 - elapsed_time) / (at3 - at2) if (at3 - at2) > 0 else 0
            else:
                # 角度停止
                ang_vel_scale = 0.0
            
            # 计算目标速度方向
            if pos_error > 0:
                target_vel_x = (dx / pos_error) * required_vel * vel_scale
                target_vel_y = (dy / pos_error) * required_vel * vel_scale
            else:
                target_vel_x = 0.0
                target_vel_y = 0.0
            
            target_ang_vel = np.sign(dyaw) * required_ang_vel * ang_vel_scale
            
            # 平滑速度控制（简单的低通滤波）
            alpha = 0.3  # 平滑系数
            current_vel_x = alpha * target_vel_x + (1 - alpha) * current_vel_x
            current_vel_y = alpha * target_vel_y + (1 - alpha) * current_vel_y
            current_ang_vel = alpha * target_ang_vel + (1 - alpha) * current_ang_vel
            
            # 发送速度命令
            self.send_cmd_vel(current_vel_x, current_vel_y, current_ang_vel)
            
            # 更新当前位置（基于速度和时间步长）
            dt = 0.1  # 时间步长 0.1秒
            self.current_x += current_vel_x * dt
            self.current_y += current_vel_y * dt
            self.current_yaw += current_ang_vel * dt
            
            # 标准化角度
            while self.current_yaw > np.pi:
                self.current_yaw -= 2 * np.pi
            while self.current_yaw < -np.pi:
                self.current_yaw += 2 * np.pi
            
            # 定期打印状态
            if elapsed_time % 1.0 < 0.1:  # 大约每秒打印一次
                rospy.loginfo(f"移动中: 位置误差={pos_error:.3f}m, 角度误差={np.degrees(abs(dyaw)):.2f}°")
                rospy.loginfo(f"速度比例: 位置={vel_scale:.2f}, 角度={ang_vel_scale:.2f}")
                rospy.loginfo(f"当前速度: ({current_vel_x:.3f}, {current_vel_y:.3f}, {current_ang_vel:.3f})")
                rospy.loginfo(f"当前位置: ({self.current_x:.3f}, {self.current_y:.3f}), 偏航角: {np.degrees(self.current_yaw):.2f}°")
            
            rate.sleep()
        
        # 平滑停止（逐渐减速到零）
        for i in range(10):  # 1秒内逐渐停止
            slowdown = 1.0 - i / 10.0
            self.send_cmd_vel(current_vel_x * slowdown, current_vel_y * slowdown, current_ang_vel * slowdown)
            rospy.sleep(0.1)
        
        # 完全停止
        self.send_cmd_vel(0.0, 0.0, 0.0)
        
        # 更新当前位置
        self.current_x = target_x
        self.current_y = target_y
        self.current_yaw = target_yaw
        
        rospy.loginfo("底盘移动完成！")
        return True
    
    def stop_chassis(self):
        """停止底盘移动"""
        self.send_cmd_vel(0.0, 0.0, 0.0)
        rospy.loginfo("底盘已停止")
    
    def move_to_target(self, target_x, target_y, target_z, target_yaw_degrees):
        """移动到目标姿态，自动计算差值并逐步下发（保持原有接口）"""
        target_yaw = np.radians(target_yaw_degrees)
        
        rospy.loginfo(f"目标姿态: 位置({target_x:.3f}, {target_y:.3f}, {target_z:.3f}), 偏航角({target_yaw_degrees:.2f}°)")
        rospy.loginfo(f"当前位置: 位置({self.current_x:.3f}, {self.current_y:.3f}, {self.current_z:.3f}), 偏航角({np.degrees(self.current_yaw):.2f}°)")
        
        # 计算差值
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dz = target_z - self.current_z
        dyaw = target_yaw - self.current_yaw
        
        # 标准化角度差值到[-π, π]
        while dyaw > np.pi:
            dyaw -= 2 * np.pi
        while dyaw < -np.pi:
            dyaw += 2 * np.pi
        
        rospy.loginfo(f"需要移动: 位置差值({dx:.3f}, {dy:.3f}, {dz:.3f}), 角度差值({np.degrees(dyaw):.2f}°)")
        
        # 计算步数
        pos_steps = max(abs(dx), abs(dy), abs(dz)) / self.pos_step
        yaw_steps = abs(dyaw) / self.yaw_step
        total_steps = max(int(pos_steps), int(yaw_steps), 1)
        
        rospy.loginfo(f"总步数: {total_steps}")
        
        # 逐步移动
        rate = rospy.Rate(5)  # 5Hz
        
        for step in range(total_steps):
            if not running or rospy.is_shutdown():
                break
            
            # 计算当前步的位置
            progress = (step + 1) / total_steps
            current_x = self.current_x + dx * progress
            current_y = self.current_y + dy * progress
            current_z = self.current_z + dz * progress
            current_yaw = self.current_yaw + dyaw * progress
            
            # 转换为四元数
            qw = np.cos(current_yaw / 2)
            qz = np.sin(current_yaw / 2)
            
            # 发送姿态
            self.send_chassis_pose_quaternion(current_x, current_y, current_z, qw, 0.0, 0.0, qz)
            
            rospy.loginfo(f"第{step+1}/{total_steps}步: 位置({current_x:.3f}, {current_y:.3f}, {current_z:.3f}), 偏航角({np.degrees(current_yaw):.2f}°)")
            rate.sleep()
        
        # 更新当前位置
        self.current_x = target_x
        self.current_y = target_y
        self.current_z = target_z
        self.current_yaw = target_yaw
        
        rospy.loginfo("移动完成！")
    
    def get_current_pose(self):
        """获取当前底盘位姿"""
        return {
            'x': self.current_x,
            'y': self.current_y,
            'z': self.current_z,
            'yaw_degrees': np.degrees(self.current_yaw)
        }
    
    def reset_to_origin(self):
        """重置到原点"""
        return self.move_chassis(0.0, 0.0, 0.0)
    
    def simple_chassis_control(self):
        """简化的底盘控制 - 输入目标姿态"""
        rospy.loginfo("=== 简化底盘控制 ===")
        rospy.loginfo("选择控制方式:")
        rospy.loginfo("1. 位置控制 (position control)")
        rospy.loginfo("2. 速度控制 (velocity control)")
        
        while True:
            try:
                choice = input("请选择控制方式 (1/2): ").strip()
                if choice in ['1', '2']:
                    break
                else:
                    rospy.logwarn("请输入 1 或 2")
            except KeyboardInterrupt:
                return
        
        control_mode = "position" if choice == '1' else "velocity"
        rospy.loginfo(f"已选择: {control_mode} 控制")
        
        rospy.loginfo("输入格式: x y yaw_degrees")
        rospy.loginfo("例如: 0.1 0.05 45")
        rospy.loginfo("输入 'q' 退出")
        rospy.loginfo("输入 'reset' 重置到原点")
        rospy.loginfo("输入 'status' 查看当前位置")
        rospy.loginfo("输入 'stop' 停止移动")
        
        while not rospy.is_shutdown() and running:
            try:
                user_input = input("请输入目标姿态: ").strip()
                
                if user_input.lower() == 'q':
                    break
                
                if user_input.lower() == 'reset':
                    if control_mode == "position":
                        self.reset_to_origin()
                    else:
                        self.move_chassis_velocity(0.0, 0.0, 0.0)
                    continue
                
                if user_input.lower() == 'status':
                    pose = self.get_current_pose()
                    rospy.loginfo(f"当前位置: ({pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f}), 偏航角: {pose['yaw_degrees']:.2f}°")
                    continue
                
                if user_input.lower() == 'stop':
                    if control_mode == "velocity":
                        self.stop_chassis()
                    continue
                
                # 解析输入
                parts = user_input.split()
                if len(parts) != 3:
                    rospy.logwarn("输入格式错误，请输入: x y yaw_degrees")
                    continue
                
                x = float(parts[0])
                y = float(parts[1])
                yaw_degrees = float(parts[2])
                
                # 根据控制方式移动到目标姿态
                if control_mode == "position":
                    self.move_chassis(x, y, yaw_degrees)
                else:
                    self.move_chassis_velocity(x, y, yaw_degrees)
                
            except ValueError:
                rospy.logwarn("输入格式错误，请输入数字")
            except KeyboardInterrupt:
                break
        
        rospy.loginfo("=== 简化底盘控制结束 ===")

def main():
    try:
        controller = ChassisController()
        controller.simple_chassis_control()
            
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        running = False
        rospy.loginfo("程序退出")

if __name__ == '__main__':
    main() 