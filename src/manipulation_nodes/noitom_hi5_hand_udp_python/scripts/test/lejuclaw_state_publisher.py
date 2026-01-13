#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
发布lejuclaw状态话题的脚本
发布 /leju_claw_state 话题，提供二指夹爪的状态数据
"""

import rospy
import numpy as np
from kuavo_msgs.msg import lejuClawState, endEffectorData
from std_msgs.msg import Header

class LejuClawStatePublisher:
    def __init__(self):
        rospy.init_node('leju_claw_state_publisher', anonymous=True)

        # 发布器
        self.claw_state_pub = rospy.Publisher('/leju_claw_state', lejuClawState, queue_size=10)

        # 模拟参数
        self.left_target_pos = 50.0  # 左夹爪目标位置
        self.right_target_pos = 50.0  # 右夹爪目标位置
        self.left_current_pos = 10.0  # 左夹爪当前位置
        self.right_current_pos = 10.0  # 右夹爪当前位置
        self.left_state = 2  # 左夹爪状态 (2: Reached)
        self.right_state = 2  # 右夹爪状态 (2: Reached)

        # 发布频率
        self.publish_rate = rospy.Rate(10)  # 10Hz

        rospy.loginfo("lejuclaw状态发布器已启动")
        rospy.loginfo("发布话题: /leju_claw_state")
        rospy.loginfo("发布频率: 10Hz")

    def update_claw_positions(self):
        """更新夹爪位置，模拟运动过程"""
        # 简单的位置更新模拟
        if abs(self.left_current_pos - self.left_target_pos) > 0.1:
            self.left_current_pos += np.sign(self.left_target_pos - self.left_current_pos) * 2.0
            self.left_state = 1  # Moving
        else:
            self.left_current_pos = self.left_target_pos
            self.left_state = 2  # Reached

        if abs(self.right_current_pos - self.right_target_pos) > 0.1:
            self.right_current_pos += np.sign(self.right_target_pos - self.right_current_pos) * 2.0
            self.right_state = 1  # Moving
        else:
            self.right_current_pos = self.right_target_pos
            self.right_state = 2  # Reached

    def calculate_velocity(self, current_pos, target_pos):
        """计算速度"""
        if abs(current_pos - target_pos) > 0.1:
            return np.sign(target_pos - current_pos) * 20.0  # 20.0 units/s
        return 0.0

    def calculate_effort(self):
        """计算模拟电流值"""
        # 简单的模拟：运动时电流较大，静止时电流较小
        left_effort = 0.5 if self.left_state == 1 else 0.1
        right_effort = 0.5 if self.right_state == 1 else 0.1

        # 添加一些随机噪声
        left_effort += np.random.normal(0, 0.05)
        right_effort += np.random.normal(0, 0.05)

        return left_effort, right_effort

    def create_claw_state_msg(self):
        """创建夹爪状态消息"""
        msg = lejuClawState()

        # 设置状态
        msg.state = [self.left_state, self.right_state]

        # 设置末端执行器数据
        msg.data = endEffectorData()
        msg.data.name = ["left_claw", "right_claw"]
        msg.data.position = [self.left_current_pos, self.right_current_pos]
        msg.data.velocity = [
            self.calculate_velocity(self.left_current_pos, self.left_target_pos),
            self.calculate_velocity(self.right_current_pos, self.right_target_pos)
        ]

        left_effort, right_effort = self.calculate_effort()
        msg.data.effort = [left_effort, right_effort]

        return msg

    def simulate_random_commands(self):
        """模拟随机控制指令"""
        if np.random.random() < 0.05:  # 5%概率改变目标位置
            self.left_target_pos = np.random.uniform(0, 100)
            self.right_target_pos = np.random.uniform(0, 100)
            rospy.loginfo(f"新的目标位置 - 左夹爪: {self.left_target_pos:.1f}, 右夹爪: {self.right_target_pos:.1f}")

    def run(self):
        """主循环"""
        rospy.loginfo("开始发布lejuclaw状态数据...")

        count = 0
        while not rospy.is_shutdown():
            # 更新位置
            self.update_claw_positions()

            # 模拟随机指令
            self.simulate_random_commands()

            # 创建并发布消息
            msg = self.create_claw_state_msg()
            self.claw_state_pub.publish(msg)

            # 每100次循环输出一次状态信息
            if count % 100 == 0:
                rospy.loginfo(f"发布状态 - 左: pos={self.left_current_pos:.1f}, state={self.left_state}, "
                            f"右: pos={self.right_current_pos:.1f}, state={self.right_state}")

            count += 1
            self.publish_rate.sleep()

def main():
    try:
        publisher = LejuClawStatePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("lejuclaw状态发布器已停止")
    except Exception as e:
        rospy.logerr(f"运行出错: {e}")

if __name__ == '__main__':
    main()