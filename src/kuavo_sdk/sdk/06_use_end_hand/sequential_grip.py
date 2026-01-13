#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
依次握紧手指控制脚本
从小拇指开始依次握紧到大拇指
"""

import rospy
from kuavo_msgs.msg import robotHandPosition
import time


def publish_controlEndHand(hand_traj):
    """
    控制机器人的手部动作。

    参数:
    hand_traj (list): 包含左手和右手位置的列表，前6个元素为左手，后6个元素为右手。

    返回:
    bool: 服务调用结果，成功返回True，失败返回False。
    """
    try:
        # 创建Publisher对象
        pub = rospy.Publisher('control_robot_hand_position', robotHandPosition, queue_size=10)
        rospy.sleep(0.5)  # 确保Publisher注册
        # 创建消息对象
        msg = robotHandPosition()
        # 设置左手和右手的位置
        msg.left_hand_position = hand_traj[0:6]
        msg.right_hand_position = hand_traj[6:]

        rate = rospy.Rate(10)  # 10Hz
        while pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("等待订阅者连接...")
            rate.sleep()

        # 发布消息
        pub.publish(msg)
        rospy.loginfo(f"发布手部位置: 左手={msg.left_hand_position}, 右手={msg.right_hand_position}")
        return True

    except rospy.ServiceException as e:
        # 记录错误日志
        rospy.logerr(f"controlEndHand 服务调用失败: {e}")
        return False


def sequential_grip(hand='right', grip_value=100, delay=0.5):
    """
    依次握紧手指，从小拇指开始到大拇指
    
    参数:
    hand (str): 控制哪只手，'left'表示左手，'right'表示右手，'both'表示双手
    grip_value (float): 握紧的目标值（0-100），0表示张开，100表示完全握紧
    delay (float): 每个手指动作之间的延迟时间（秒）
    
    手指索引说明:
    - 索引 0, 1: 大拇指的两个自由度
    - 索引 2: 食指
    - 索引 3: 中指
    - 索引 4: 无名指
    - 索引 5: 小拇指
    
    握紧顺序: 小拇指(5) -> 无名指(4) -> 中指(3) -> 食指(2) -> 大拇指(1,0)
    """
    
    # 初始位置（全部张开）
    left_hand = [0, 0, 0, 0, 0, 0]
    right_hand = [0, 0, 0, 0, 0, 0]
    
    # 定义握紧顺序：从小拇指到大拇指
    finger_sequence = [5, 4, 3, 2, 1, 0]  # 小拇指 -> 无名指 -> 中指 -> 食指 -> 大拇指(两个自由度)
    finger_names = ['小拇指', '无名指', '中指', '食指', '大拇指第二关节', '大拇指第一关节']
    
    rospy.loginfo(f"开始依次握紧{hand}手，从小拇指到大拇指...")
    rospy.loginfo(f"握紧目标值: {grip_value}, 每个手指延迟: {delay}秒")
    
    # 依次握紧每个手指
    for i, finger_index in enumerate(finger_sequence):
        if hand == 'left' or hand == 'both':
            left_hand[finger_index] = grip_value
        
        if hand == 'right' or hand == 'both':
            right_hand[finger_index] = grip_value
        
        # 组合手部轨迹
        hand_traj = left_hand + right_hand
        
        # 发布控制命令
        rospy.loginfo(f"步骤 {i+1}/{len(finger_sequence)}: 握紧 {finger_names[i]} (索引{finger_index})")
        publish_controlEndHand(hand_traj)
        
        # 延迟
        rospy.sleep(delay)
    
    rospy.loginfo("依次握紧完成！")


def sequential_release(hand='right', delay=0.5):
    """
    依次松开手指，从大拇指开始到小拇指（与握紧相反的顺序）
    
    参数:
    hand (str): 控制哪只手，'left'表示左手，'right'表示右手，'both'表示双手
    delay (float): 每个手指动作之间的延迟时间（秒）
    """
    
    # 初始位置（全部握紧）
    grip_value = 100
    left_hand = [grip_value] * 6
    right_hand = [grip_value] * 6
    
    # 定义松开顺序：从大拇指到小拇指
    finger_sequence = [0, 1, 2, 3, 4, 5]
    finger_names = ['大拇指第一关节', '大拇指第二关节', '食指', '中指', '无名指', '小拇指']
    
    rospy.loginfo(f"开始依次松开{hand}手，从大拇指到小拇指...")
    
    # 依次松开每个手指
    for i, finger_index in enumerate(finger_sequence):
        if hand == 'left' or hand == 'both':
            left_hand[finger_index] = 0
        
        if hand == 'right' or hand == 'both':
            right_hand[finger_index] = 0
        
        # 组合手部轨迹
        hand_traj = left_hand + right_hand
        
        # 发布控制命令
        rospy.loginfo(f"步骤 {i+1}/{len(finger_sequence)}: 松开 {finger_names[i]} (索引{finger_index})")
        publish_controlEndHand(hand_traj)
        
        # 延迟
        rospy.sleep(delay)
    
    rospy.loginfo("依次松开完成！")


def main():
    # 初始化ROS节点
    rospy.init_node('sequential_grip_controller')
    
    # 等待节点准备好
    rospy.sleep(1)
    
    # 示例1: 右手依次握紧（从小拇指到大拇指）
    rospy.loginfo("=" * 50)
    rospy.loginfo("示例1: 右手依次握紧")
    rospy.loginfo("=" * 50)
    sequential_grip(hand='right', grip_value=80, delay=0.8)
    
    # 等待一段时间
    rospy.sleep(2)
    
    # 示例2: 右手依次松开（从大拇指到小拇指）
    rospy.loginfo("=" * 50)
    rospy.loginfo("示例2: 右手依次松开")
    rospy.loginfo("=" * 50)
    sequential_release(hand='right', delay=0.8)
    
    # 等待一段时间
    rospy.sleep(2)
    
    # 示例3: 双手同时依次握紧
    rospy.loginfo("=" * 50)
    rospy.loginfo("示例3: 双手同时依次握紧")
    rospy.loginfo("=" * 50)
    sequential_grip(hand='both', grip_value=80, delay=0.8)
    
    rospy.loginfo("所有动作完成！")


if __name__ == "__main__":
    main()
