#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import uuid
from pytrees_actions.srv import GetRobotInfo, GetRobotInfoResponse
from kuavo_humanoid_sdk.kuavo import KuavoRobot

# 这是一个ROS服务端，用于提供机器人信息
class RobotInfoService:
    def __init__(self, node_name):
        # 初始化节点
        rospy.init_node(node_name)
        # 创建服务
        self.service = rospy.Service('/dispatch/get_robot_info', GetRobotInfo, self.handle_robot_info_request)
        rospy.loginfo(f"{node_name} 已启动，等待服务请求...")
        # 启动循环以等待请求
        rospy.spin()

    def get_mac_address(self):
        mac_num = hex(uuid.getnode()).replace('0x', '').upper()
        mac_address = ':'.join(mac_num[i:i+2] for i in range(0, 12, 2))
        return mac_address

    def handle_robot_info_request(self, request):
        # 处理服务请求
        response = GetRobotInfoResponse()
        response.mac_address=self.get_mac_address()
        response.end_effector_type="clamp"
        response.robot_type="humanoid"
        response.version="4Pro"
        return response

if __name__ == '__main__':
    service = RobotInfoService('robot_info_service_node')
