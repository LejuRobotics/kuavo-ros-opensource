#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED Strip ROS 服务节点
提供对 LEDStrip 类的 set_mode_and_color 和 close 方法的 ROS 服务接口
"""

import rospy
import threading
from kuavo_msgs.srv import SetLEDMode_free, SetLEDMode_freeResponse
from kuavo_msgs.srv import GetBatteryInfo, GetBatteryInfoResponse
from std_srvs.srv import Trigger, TriggerResponse
import sys
import os

# 添加 controller 目录到路径
# 当前脚本路径: scripts/led_strip_service.py
# controller 路径: src/controller/led/led_strip.py
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'controller'))

from led.led_strip import LEDStrip, LEDMode
from hardware.serial_port import SerialPort
from hardware.battery_query import BatteryQueryCache


class LEDStripServiceNode:
    """LED Strip ROS 服务节点类"""
    
    def __init__(self):
        """初始化 ROS 节点和服务"""
        # 初始化 ROS 节点
        rospy.init_node('led_strip_service_node', anonymous=False)
        
        # 创建 LEDStrip 实例
        self.led_strip = LEDStrip()

        # 获取共享串口单例引用
        self.serial_port = SerialPort()
        # 操作锁，确保 LED 和电池查询互斥使用串口
        self._op_lock = threading.Lock()
        # 后台电池查询缓存，使用 try_lock 不阻塞 LED 操作
        self._battery_cache = BatteryQueryCache(self.serial_port, self._op_lock)

        # 创建 ROS 服务
        self.set_mode_service = rospy.Service(
            'led_strip_set_mode_and_color',
            SetLEDMode_free,
            self.handle_set_mode_and_color
        )
        
        self.close_service = rospy.Service(
            'led_strip_close',
            Trigger,
            self.handle_close
        )

        # 电池查询内部服务，供 battery_info_node 通过 ROS service 调用
        try:
            self._battery_service = rospy.Service(
                '_query_battery_hw',
                GetBatteryInfo,
                self.handle_query_battery_hw
            )
        except rospy.ServiceException as e:
            rospy.logwarn(f"无法注册 _query_battery_hw 服务（另一 LED 节点已注册）: {e}")

        rospy.loginfo("LED Strip 服务已启动")
        rospy.loginfo("可用服务:")
        rospy.loginfo("  - /led_strip_set_mode_and_color (SetLEDMode_free)")
        rospy.loginfo("  - /led_strip_close (Trigger)")
        rospy.on_shutdown(self.cleanup)

    def handle_set_mode_and_color(self, req):
        """
        处理设置模式和颜色的服务请求
        
        Args:
            req: SetLEDMode_free 请求
            
        Returns:
            SetLEDMode_freeResponse
        """
        response = SetLEDMode_freeResponse()
        
        try:
            # 解析模式
            mode = LEDMode(req.mode)
            
            # 解析颜色列表
            colors = []
            for color_msg in req.colors:
                colors.append((color_msg.r, color_msg.g, color_msg.b))
            
            # 检查颜色数量是否为 24 个
            if len(colors) != LEDStrip.LED_COUNT:
                response.success = False
                rospy.logerr(f"颜色数量错误: 期望 {LEDStrip.LED_COUNT}, 实际 {len(colors)}")
                return response
            
            # 调用 set_mode_and_color 方法
            with self._op_lock:
                self.serial_port.clear_buffer()
                success = self.led_strip.set_mode_and_color(mode, colors)
            
            response.success = success
            
            if success:
                rospy.loginfo(f"LED 设置成功: 模式={mode.name}, 灯数={len(colors)}")
            else:
                rospy.logwarn("LED 设置失败")
                
        except ValueError as e:
            response.success = False
            rospy.logerr(f"无效的模式值: {req.mode}")
        except Exception as e:
            response.success = False
            rospy.logerr(f"设置 LED 时发生错误: {e}")
        
        return response
    
    def handle_close(self, req):
        """
        处理关闭 LED 的服务请求
        
        Args:
            req: Trigger 请求
            
        Returns:
            TriggerResponse
        """
        try:
            # 调用 close 方法关闭所有 LED
            with self._op_lock:
                self.serial_port.clear_buffer()
                success = self.led_strip.close()
            
            if success:
                rospy.loginfo("LED 已关闭")
                return TriggerResponse(success=True, message="LED closed successfully")
            else:
                rospy.logwarn("关闭 LED 失败")
                return TriggerResponse(success=False, message="Failed to close LED")
                
        except Exception as e:
            rospy.logerr(f"关闭 LED 时发生错误: {e}")
            return TriggerResponse(success=False, message=f"Error: {str(e)}")

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

    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("正在关闭 LED Strip 服务节点...")
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        try:
            self._battery_cache.stop()
            self.led_strip.close()
            rospy.loginfo("LED Strip 已关闭")
        except Exception as e:
            rospy.logerr(f"清理时发生错误: {e}")


if __name__ == '__main__':
    try:
        node = LEDStripServiceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点启动失败: {e}")
        sys.exit(1)
