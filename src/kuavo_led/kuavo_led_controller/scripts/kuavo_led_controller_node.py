#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
from kuavo_msgs.srv import SetLEDMode, SetLEDModeResponse, SetLEDMode_free, SetLEDMode_freeResponse
from kuavo_msgs.srv import GetBatteryInfo, GetBatteryInfoResponse
from std_srvs.srv import Trigger, TriggerResponse
import os
import sys

# 添加 controller 目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'controller'))

from hardware.serial_port import SerialPort
from hardware.battery_query import BatteryQueryCache

class LEDController:
    def __init__(self, port='/dev/ttyLED0', baudrate=115200):
        """
        初始化LED控制器
        :param port: 串口设备路径
        :param baudrate: 波特率
        """
        self.ser = SerialPort()

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
        self.ser.send_data(bytes(packet))
        # print(f"发送数据: {[hex(x) for x in packet]}")

    def close(self):
        """关闭LED连接"""
        self.set_led_mode(0x00, [(0, 0, 0)] * 10)

    def deinit(self):
        self.set_led_mode(0x00, [(0, 0, 0)] * 10)
        self.close()

class LEDControllerNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('led_controller_node', anonymous=True)

        # 创建LED控制器实例
        self.led_controller = LEDController()

        # 获取共享串口单例引用
        self.serial_port = SerialPort()
        # 操作锁，确保 LED 和电池查询互斥使用串口
        self._op_lock = threading.Lock()
        # 后台电池查询缓存，使用 try_lock 不阻塞 LED 操作
        self._battery_cache = BatteryQueryCache(self.serial_port, self._op_lock)

        # 创建ROS服务
        self.led_service = rospy.Service('control_led', SetLEDMode, self.handle_led_control)
        self.led_service_free = rospy.Service('control_led_free', SetLEDMode_free, self.handle_led_control_free)
        self.stop_led_service = rospy.Service('close_led',Trigger,self.handle_close_led)

        # 电池查询内部服务，供 battery_info_node 通过 ROS service 调用
        try:
            self._battery_service = rospy.Service(
                '_query_battery_hw',
                GetBatteryInfo,
                self.handle_query_battery_hw
            )
        except rospy.ServiceException as e:
            rospy.logwarn(f"无法注册 _query_battery_hw 服务（另一 LED 节点已注册）: {e}")

        rospy.loginfo("LED控制服务已启动，等待请求...")
        rospy.on_shutdown(self._shutdown)

    def _shutdown(self):
        """节点退出时清理后台线程"""
        self._battery_cache.stop()
    
    def handle_close_led(self,req):
        colors = [(0,0,0),(0,0,0),(0,0,0),
                  (0,0,0),(0,0,0),(0,0,0),
                  (0,0,0),(0,0,0),(0,0,0),
                  (0,0,0)]
        with self._op_lock:
            self.serial_port.clear_buffer()
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
        with self._op_lock:
            self.serial_port.clear_buffer()
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
        with self._op_lock:
            self.serial_port.clear_buffer()
            self.led_controller.set_led_mode(req.mode, colors)
        response.success = True
        return response

    def handle_query_battery_hw(self, req):
        """返回后台缓存的最新电池数据（瞬时返回，不阻塞 LED）"""
        try:
            battery_info, age = self._battery_cache.get(req.battery_id)

            if battery_info is None:
                if age < 0:
                    msg = f"No cached data for battery {req.battery_id} yet"
                else:
                    msg = f"Battery {req.battery_id} data stale ({age:.0f}s old, may be disconnected)"
                return GetBatteryInfoResponse(
                    success=False,
                    message=msg
                )

            return GetBatteryInfoResponse(
                battery_id=req.battery_id,
                voltage=battery_info['voltage'],
                current=battery_info['current'],
                remaining_capacity=battery_info['remaining_capacity'],
                full_capacity=battery_info['full_capacity'],
                percentage=battery_info['percentage'],
                cycle_count=battery_info['cycle_count'],
                protection_flags=battery_info['protection_flags'],
                temperatures=battery_info['temperatures'],
                success=True,
                message=f"Battery {req.battery_id} (cached)"
            )

        except Exception as e:
            rospy.logerr(f"电池查询失败: {e}")
            return GetBatteryInfoResponse(
                success=False,
                message=f"Service error: {str(e)}"
            )

if __name__ == '__main__':
    try:
        node = LEDControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
