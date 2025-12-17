#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from kuavo_msgs.msg import twoArmHandPoseCmd, twoArmHandPose, armHandPose
from geometry_msgs.msg import Pose, Point, Quaternion
import time

class ArmMotionTester:
    def __init__(self):
        rospy.init_node('arm_motion_tester', anonymous=True)
        
        # 发布器
        self.pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
        
        # 等待发布器准备就绪
        rospy.sleep(1.0)
        
        print("手臂运动测试器已初始化")
        print("可用的测试命令:")
        print("1. test_basic_motion() - 基础运动测试")
        print("2. test_circular_motion() - 圆形运动测试")
        print("3. test_parallel_motion() - 平行运动测试")
        print("4. test_individual_arm() - 单臂运动测试")
        print("5. test_waist_integration() - 腰部集成测试")
    
    def create_hand_pose(self, x, y, z, qx=0, qy=0, qz=0, qw=1):
        """创建手部姿态"""
        pose = armHandPose()
        pose.pos_xyz = [x, y, z]
        pose.quat_xyzw = [qx, qy, qz, qw]
        pose.elbow_pos_xyz = [0, 0, 0]  # 肘部位置，暂时设为0
        pose.joint_angles = [0] * 7  # 关节角度，暂时设为0
        return pose
    
    def create_two_arm_command(self, left_pose, right_pose, frame=2):
        """创建双臂命令"""
        cmd = twoArmHandPoseCmd()
        cmd.hand_poses.left_pose = left_pose
        cmd.hand_poses.right_pose = right_pose
        cmd.use_custom_ik_param = False
        cmd.joint_angles_as_q0 = False
        cmd.frame = frame  # 2表示local frame
        return cmd
    
    def publish_command(self, cmd, duration=2.0):
        """发布命令并等待"""
        print(f"发布命令: 左手位置({cmd.hand_poses.left_pose.pos_xyz}), 右手位置({cmd.hand_poses.right_pose.pos_xyz})")
        self.pub.publish(cmd)
        rospy.sleep(duration)
    
    def test_basic_motion(self):
        """基础运动测试"""
        print("\n=== 基础运动测试 ===")
        
        # 初始位置
        left_pose = self.create_hand_pose(0.3, 0.2, 0.5)
        right_pose = self.create_hand_pose(0.3, -0.2, 0.5)
        cmd = self.create_two_arm_command(left_pose, right_pose)
        self.publish_command(cmd, 3.0)
        
        # 向上移动
        left_pose = self.create_hand_pose(0.3, 0.2, 0.7)
        right_pose = self.create_hand_pose(0.3, -0.2, 0.7)
        cmd = self.create_two_arm_command(left_pose, right_pose)
        self.publish_command(cmd, 3.0)
        
        # 向前移动
        left_pose = self.create_hand_pose(0.5, 0.2, 0.7)
        right_pose = self.create_hand_pose(0.5, -0.2, 0.7)
        cmd = self.create_two_arm_command(left_pose, right_pose)
        self.publish_command(cmd, 3.0)
        
        # 回到初始位置
        left_pose = self.create_hand_pose(0.3, 0.2, 0.5)
        right_pose = self.create_hand_pose(0.3, -0.2, 0.5)
        cmd = self.create_two_arm_command(left_pose, right_pose)
        self.publish_command(cmd, 3.0)
        
        print("基础运动测试完成")
    
    def test_circular_motion(self):
        """圆形运动测试"""
        print("\n=== 圆形运动测试 ===")
        
        center_x, center_y, center_z = 0.4, 0.0, 0.6
        radius = 0.1
        steps = 20
        
        for i in range(steps):
            angle = 2 * math.pi * i / steps
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            left_pose = self.create_hand_pose(x, y + 0.2, center_z)
            right_pose = self.create_hand_pose(x, y - 0.2, center_z)
            cmd = self.create_two_arm_command(left_pose, right_pose)
            self.publish_command(cmd, 0.5)
        
        print("圆形运动测试完成")
    
    def test_parallel_motion(self):
        """平行运动测试"""
        print("\n=== 平行运动测试 ===")
        
        # 水平移动
        for x in np.linspace(0.3, 0.6, 10):
            left_pose = self.create_hand_pose(x, 0.2, 0.6)
            right_pose = self.create_hand_pose(x, -0.2, 0.6)
            cmd = self.create_two_arm_command(left_pose, right_pose)
            self.publish_command(cmd, 0.3)
        
        # 垂直移动
        for z in np.linspace(0.6, 0.8, 10):
            left_pose = self.create_hand_pose(0.6, 0.2, z)
            right_pose = self.create_hand_pose(0.6, -0.2, z)
            cmd = self.create_two_arm_command(left_pose, right_pose)
            self.publish_command(cmd, 0.3)
        
        # 回到初始位置
        left_pose = self.create_hand_pose(0.3, 0.2, 0.6)
        right_pose = self.create_hand_pose(0.3, -0.2, 0.6)
        cmd = self.create_two_arm_command(left_pose, right_pose)
        self.publish_command(cmd, 2.0)
        
        print("平行运动测试完成")
    
    def test_individual_arm(self):
        """单臂运动测试"""
        print("\n=== 单臂运动测试 ===")
        
        # 只移动左手
        left_pose = self.create_hand_pose(0.4, 0.3, 0.6)
        right_pose = self.create_hand_pose(0.3, -0.2, 0.5)  # 右手保持不动
        cmd = self.create_two_arm_command(left_pose, right_pose)
        self.publish_command(cmd, 3.0)
        
        # 只移动右手
        left_pose = self.create_hand_pose(0.3, 0.2, 0.5)  # 左手保持不动
        right_pose = self.create_hand_pose(0.4, -0.3, 0.6)
        cmd = self.create_two_arm_command(left_pose, right_pose)
        self.publish_command(cmd, 3.0)
        
        # 回到初始位置
        left_pose = self.create_hand_pose(0.3, 0.2, 0.5)
        right_pose = self.create_hand_pose(0.3, -0.2, 0.5)
        cmd = self.create_two_arm_command(left_pose, right_pose)
        self.publish_command(cmd, 2.0)
        
        print("单臂运动测试完成")
    
    def test_waist_integration(self):
        """腰部集成测试"""
        print("\n=== 腰部集成测试 ===")
        
        # 测试不同坐标系下的运动
        frames = [2, 1, 3]  # local frame, world frame, VR frame
        frame_names = ["Local Frame", "World Frame", "VR Frame"]
        
        for i, frame in enumerate(frames):
            print(f"测试 {frame_names[i]}")
            
            left_pose = self.create_hand_pose(0.4, 0.2, 0.6)
            right_pose = self.create_hand_pose(0.4, -0.2, 0.6)
            cmd = self.create_two_arm_command(left_pose, right_pose, frame)
            self.publish_command(cmd, 2.0)
            
            left_pose = self.create_hand_pose(0.4, 0.2, 0.8)
            right_pose = self.create_hand_pose(0.4, -0.2, 0.8)
            cmd = self.create_two_arm_command(left_pose, right_pose, frame)
            self.publish_command(cmd, 2.0)
        
        print("腰部集成测试完成")
    
    def run_all_tests(self):
        """运行所有测试"""
        print("开始运行所有测试...")
        
        try:
            self.test_basic_motion()
            rospy.sleep(2.0)
            
            self.test_circular_motion()
            rospy.sleep(2.0)
            
            self.test_parallel_motion()
            rospy.sleep(2.0)
            
            self.test_individual_arm()
            rospy.sleep(2.0)
            
            self.test_waist_integration()
            
            print("\n所有测试完成！")
            
        except KeyboardInterrupt:
            print("\n测试被用户中断")
        except Exception as e:
            print(f"测试过程中出现错误: {e}")

def main():
    tester = ArmMotionTester()
    
    print("请选择测试模式:")
    print("1. 基础运动测试")
    print("2. 圆形运动测试")
    print("3. 平行运动测试")
    print("4. 单臂运动测试")
    print("5. 腰部集成测试")
    print("6. 运行所有测试")
    print("0. 退出")
    
    while not rospy.is_shutdown():
        try:
            choice = input("\n请输入选择 (0-6): ").strip()
            
            if choice == '0':
                print("退出测试")
                break
            elif choice == '1':
                tester.test_basic_motion()
            elif choice == '2':
                tester.test_circular_motion()
            elif choice == '3':
                tester.test_parallel_motion()
            elif choice == '4':
                tester.test_individual_arm()
            elif choice == '5':
                tester.test_waist_integration()
            elif choice == '6':
                tester.run_all_tests()
            else:
                print("无效选择，请重新输入")
                
        except KeyboardInterrupt:
            print("\n退出测试")
            break
        except Exception as e:
            print(f"输入错误: {e}")

if __name__ == '__main__':
    main()
