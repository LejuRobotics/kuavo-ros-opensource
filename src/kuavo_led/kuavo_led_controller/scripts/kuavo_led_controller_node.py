#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from kuavo_led_controller.srv import SetLEDMode, SetLEDModeResponse, SetLEDMode_free, SetLEDMode_freeResponse
from std_srvs.srv import Trigger, TriggerResponse
import os
try:
    import serial
except ImportError:
    import subprocess
    import sys
    print("正在安装pyserial库...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
    import serial
import time
import sys

class LEDController:
    def __init__(self, port='/dev/ttyLED0', baudrate=115200):
        """
        初始化LED控制器
        :param port: 串口设备路径
        :param baudrate: 波特率
        """
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"成功连接到设备: {port}")
        except serial.SerialException as e:
            print(f"无法连接到设备: {e}")
            print("请检查 UDEV 规则以及硬件设备是否正常！！")
            sys.exit(1)

    def calculate_checksum(self, data):
        """
        计算校验和
        :param data: 数据列表
        :return: 校验和
        """
        return (~sum(data)) & 0xFF

    def set_led_mode(self, mode, colors):
        """
        设置LED灯的模式和颜色
        :param mode: 模式 (0:常亮, 1:呼吸, 2:快闪, 3:律动)
        :param colors: 颜色列表，每个颜色为(R,G,B)元组
        """
        # 构建数据包
        packet = [0xFF, 0xFF, 0x00, 0x22, 0x02, 0x02, mode]
        
        # 添加颜色数据
        for r, g, b in colors:
            packet.extend([r, g, b])
        
        # 计算校验和
        checksum = self.calculate_checksum(packet[2:])
        packet.append(checksum)
        
        # 发送数据
        self.ser.write(bytes(packet))
        # print(f"发送数据: {[hex(x) for x in packet]}")

    def close(self):
        """关闭串口连接"""
        self.ser.close()

    def deinit(self):
        self.set_led_mode(0x00, [(0, 0, 0)] * 10)
        self.close()

class LEDControllerNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('led_controller_node', anonymous=True)

        # 创建LED控制器实例
        self.led_controller = LEDController()
        
        # 创建ROS服务
        self.led_service = rospy.Service('control_led', SetLEDMode, self.handle_led_control)
        self.led_service_free = rospy.Service('control_led_free', SetLEDMode_free, self.handle_led_control_free)
        self.stop_led_service = rospy.Service('close_led',Trigger,self.handle_close_led)
        rospy.loginfo("LED控制服务已启动，等待请求...")
    
    def handle_close_led(self,req):
        colors = [(0,0,0),(0,0,0),(0,0,0),
                  (0,0,0),(0,0,0),(0,0,0),
                  (0,0,0),(0,0,0),(0,0,0),
                  (0,0,0)]
        self.led_controller.set_led_mode(0,colors)
        return TriggerResponse(success=True,message="success")
    
    def handle_led_control(self, req):
        """处理LED控制服务请求"""
        response = SetLEDModeResponse()
        colors = [
            req.color1, 
            req.color2, 
            req.color3,
            req.color4,
            req.color5,
            req.color6,
            req.color7,
            req.color8,
            req.color9,
            req.color10
        ]
        self.led_controller.set_led_mode(req.mode, colors)
        response.success = True
        # rospy.loginfo("LED控制成功")

        return response
    def handle_led_control_free(self, req):
        """处理LED控制服务请求"""
        response = SetLEDMode_freeResponse()
        colors = []
        for i in req.colors:
            colors.append((i.r,i.g,i.b))
        self.led_controller.set_led_mode(req.mode, colors)
        response.success = True
        return response

if __name__ == '__main__':
    try:
        node = LEDControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
