#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gripper_controller import GripperController
import time

def test_basic_control(controller):
    """测试基本的夹爪控制功能"""
    rospy.loginfo("=== Test 1: Basic Control ===")
    
    # 测试张开夹爪
    rospy.loginfo("Opening grippers...")
    controller.open_grippers()
    time.sleep(2)
    
    # 测试闭合夹爪
    rospy.loginfo("Closing grippers...")
    controller.close_grippers()
    time.sleep(2)
    
    # 测试张开夹爪
    rospy.loginfo("Opening grippers again...")
    controller.open_grippers()
    time.sleep(2)

def test_individual_control(controller):
    """测试单独控制左右夹爪"""
    rospy.loginfo("=== Test 2: Individual Control ===")
    
    # 先张开所有夹爪
    controller.open_grippers()
    time.sleep(1)
    
    # 只闭合右夹爪
    rospy.loginfo("Closing right gripper only...")
    controller.control_right_gripper(255.0)
    time.sleep(2)
    
    # 只闭合左夹爪
    rospy.loginfo("Closing left gripper only...")
    controller.control_left_gripper(255.0)
    time.sleep(2)
    
    # 只张开右夹爪
    rospy.loginfo("Opening right gripper only...")
    controller.control_right_gripper(0.0)
    time.sleep(2)
    
    # 张开所有夹爪
    rospy.loginfo("Opening all grippers...")
    controller.open_grippers()
    time.sleep(1)

def test_partial_positions(controller):
    """测试部分开合位置"""
    rospy.loginfo("=== Test 3: Partial Positions ===")
    
    # 测试不同的开合程度
    positions = [
        (64.0, 64.0, "25% closed"),
        (128.0, 128.0, "50% closed"),
        (192.0, 192.0, "75% closed"),
        (255.0, 255.0, "100% closed"),
        (0.0, 0.0, "Fully open")
    ]
    
    for left, right, description in positions:
        rospy.loginfo(f"Setting position: {description}")
        controller.set_gripper_position(left, right)
        time.sleep(2)

def test_asymmetric_control(controller):
    """测试非对称控制"""
    rospy.loginfo("=== Test 4: Asymmetric Control ===")
    
    # 左右夹爪不同位置
    test_cases = [
        (0.0, 255.0, "Left open, Right closed"),
        (255.0, 0.0, "Left closed, Right open"),
        (128.0, 64.0, "Left 50%, Right 25%"),
        (64.0, 192.0, "Left 25%, Right 75%")
    ]
    
    for left, right, description in test_cases:
        rospy.loginfo(f"Setting: {description}")
        controller.set_gripper_position(left, right)
        time.sleep(2)
    
    # 恢复到张开状态
    controller.open_grippers()
    time.sleep(1)

def test_status_monitoring(controller):
    """测试状态监控"""
    rospy.loginfo("=== Test 5: Status Monitoring ===")
    
    # 设置目标位置并监控
    controller.set_gripper_position(128.0, 128.0)
    
    for i in range(10):
        cmd_left, cmd_right = controller.get_current_commands()
        pos_left, pos_right = controller.get_current_positions()
        
        rospy.loginfo(f"Commands: L={cmd_left:.1f}, R={cmd_right:.1f} | "
                     f"Positions: L={pos_left:.3f}, R={pos_right:.3f}")
        time.sleep(0.5)
    
    controller.open_grippers()
    time.sleep(1)

def test_wait_for_position(controller):
    """测试等待位置到达功能"""
    rospy.loginfo("=== Test 6: Wait for Position ===")
    
    # 设置目标并等待
    controller.set_gripper_position(200.0, 200.0)
    rospy.loginfo("Waiting for grippers to reach target position...")
    
    # 注意: 这里需要将命令值(0-255)转换为位置值(0-0.8)来等待
    # 假设线性映射: position = command / 255.0 * 0.8
    target_position = 200.0 / 255.0 * 0.8
    
    success = controller.wait_for_position(
        target_left=target_position,
        target_right=target_position,
        tolerance=0.05,
        timeout=5.0
    )
    
    if success:
        rospy.loginfo("Grippers reached target position!")
    else:
        rospy.logwarn("Timeout waiting for target position")
    
    controller.open_grippers()
    time.sleep(1)

def run_all_tests():
    """运行所有测试"""
    rospy.init_node('gripper_test_node', anonymous=True)
    
    rospy.loginfo("Initializing Gripper Controller...")
    controller = GripperController(publish_frequency=100.0)
    
    # 等待初始化
    rospy.sleep(1.0)
    
    try:
        # 运行各项测试
        test_basic_control(controller)
        rospy.sleep(1.0)
        
        test_individual_control(controller)
        rospy.sleep(1.0)
        
        test_partial_positions(controller)
        rospy.sleep(1.0)
        
        test_asymmetric_control(controller)
        rospy.sleep(1.0)
        
        test_status_monitoring(controller)
        rospy.sleep(1.0)
        
        test_wait_for_position(controller)
        
        rospy.loginfo("=== All tests completed ===")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted by user")
    except Exception as e:
        rospy.logerr(f"Test failed with error: {e}")
    finally:
        controller.stop()
        rospy.loginfo("Gripper Controller stopped")

if __name__ == '__main__':
    run_all_tests()
