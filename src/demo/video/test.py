#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from kuavo_msgs.srv import ExecuteArmAction, changeArmCtrlMode
from kuavo_msgs.msg import robotWaistControl


# from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot


class RobotForwardWalk:
    def __init__(self):
        # 初始化节点
        rospy.init_node('forward_walk_node', anonymous=True)


        # 创建发布者，发布到/cmd_vel话题
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.waist_pub = rospy.Publisher('/robot_waist_motion_data', robotWaistControl, queue_size=10)

        # 设置发布频率
        self.rate = rospy.Rate(10)  # 10Hz

        self.waist_msg = robotWaistControl()
        self.waist_msg.header.stamp = rospy.Time.now()
        self.waist_msg.data.data = [0]

    def walk_forward(self, linear_speed=0.0, angular_speed=0.0, duration=0.0, waist_angle=0):
        """
        控制机器人向前行走

        Args:
            linear_speed: 线速度 (m/s)，限制在0-0.3范围内
            angular_speed: 角速度 (rad/s)，限制在-0.15到0.15范围内
            duration: 持续时间 (seconds)
        """
        # 限制线速度在0-0.3范围内
        linear_speed = max(-0.3, min(linear_speed, 0.3))
        
        # 限制角速度在-0.15到0.15范围内
        angular_speed = max(-0.4, min(angular_speed, 0.4))

        # 创建Twist消息
        cmd_vel_msg = Twist()

        # 设置线速度，x方向为正表示向前
        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0

        # 设置角速度
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = angular_speed

        # 计算持续时间
        start_time = rospy.Time.now().to_sec()
        current_time = rospy.Time.now().to_sec()

        # rospy.loginfo(f"开始向前行走，速度: {linear_speed} m/s")

        # 持续发布速度指令
        while (current_time - start_time) < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.turn_waist(waist_angle)
            self.rate.sleep()
            current_time = rospy.Time.now().to_sec()

        # # 停止机器人
        # self.stop_robot()
        # rospy.loginfo("行走结束")

    def stop_robot(self):
        """停止机器人运动"""
        # 发送多次全零指令确保机器人完全停止
        cmd_vel_msg = Twist()
        for i in range(10):  # 发送10次全零指令
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.rate.sleep()

        rospy.loginfo("机器人已完全停止")

    # 转腰
    def turn_waist(self, waist_angle=0):
        self.waist_msg.data.data = [waist_angle]
        self.waist_pub.publish(self.waist_msg)




if __name__ == '__main__':

    try:

        # 创建机器人行走控制器实例
        robot_walker = RobotForwardWalk()


        # 等待连接建立
        rospy.sleep(1.0)
        robot_walker.turn_waist(0)
        rospy.sleep(3.0)


        # 直线行走
        rospy.loginfo(f"开始直线行走")
        robot_walker.walk_forward(linear_speed=0.3,angular_speed=0.0, duration=5)
        

        # 击拳
        rospy.loginfo(f"开始击拳")
        robot_walker.stop_robot()
        rospy.sleep(3.0)
        rospy.loginfo(f"开始转弯")
        robot_walker.walk_forward(linear_speed=0.15,angular_speed=0.4, duration=4)

        
        # # 转身敬礼
        # rospy.loginfo(f"开始转腰打招呼")
        robot_walker.walk_forward(linear_speed=0.3, duration=3.0)
        robot_walker.stop_robot()
        robot_walker.turn_waist(30)
        rospy.sleep(3.0)
        robot_walker.walk_forward(linear_speed=0.3, duration=3.0)
        robot_walker.stop_robot()


    except rospy.ROSInterruptException:
        pass