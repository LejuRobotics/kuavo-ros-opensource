#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED Strip ROS 服务节点
提供对 LEDStrip 类的 set_mode_and_color 和 close 方法的 ROS 服务接口
"""

import rospy
from kuavo_msgs.srv import SetLEDMode_free, SetLEDMode_freeResponse
from std_srvs.srv import Trigger, TriggerResponse
import sys
import os

# 添加 controller 目录到路径
# 当前脚本路径: scripts/led_strip_service.py
# controller 路径: src/controller/led/led_strip.py
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'controller'))

from led.led_strip import LEDStrip, LEDMode


class LEDStripServiceNode:
    """LED Strip ROS 服务节点类"""
    
    def __init__(self):
        """初始化 ROS 节点和服务"""
        # 初始化 ROS 节点
        rospy.init_node('led_strip_service_node', anonymous=False)
        
        # 创建 LEDStrip 实例
        self.led_strip = LEDStrip()
        
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
        
        rospy.loginfo("LED Strip 服务已启动")
        rospy.loginfo("可用服务:")
        rospy.loginfo("  - /led_strip_set_mode_and_color (SetLEDMode_free)")
        rospy.loginfo("  - /led_strip_close (Trigger)")

        # 读取 led_for_state 开关（由 set_led_mode.launch 的 <param> 传入）：
        # 控制 Ctrl+C 打断 launch 时是否亮红灯做失能指示。
        #   true（默认）= 亮红灯常亮（利用硬件锁存）
        #   false        = 恢复默认关灯行为，不做失能指示
        self.led_for_state_enabled = rospy.get_param('~led_for_state', True)

        # 节点关闭时的灯状态由 _on_shutdown 负责，受 led_for_state 开关控制。
        rospy.on_shutdown(self._on_shutdown)

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
    
    def _on_shutdown(self):
        """节点关闭时的灯状态，受 led_for_state 开关控制。

        - led_for_state=True（默认）：亮红灯常亮做失能/打断指示。LED 硬件
          锁存最后状态，红灯持续亮到下次 launch 启动切回正常颜色。
        - led_for_state=False：恢复默认关灯行为（熄灭），不做失能指示。
        """
        try:
            if self.led_for_state_enabled:
                red = [(255, 0, 0)] * LEDStrip.LED_COUNT
                self.led_strip.set_mode_and_color(LEDMode.CONSTANT, red)
                rospy.loginfo("[LED Strip] 节点关闭，已设置红灯常亮（失能指示）")
            else:
                self.led_strip.close()
                rospy.loginfo("[LED Strip] led_for_state=False，节点关闭，已熄灭 LED")
        except Exception as e:
            rospy.logerr(f"[LED Strip] 关闭时设置 LED 失败: {e}")

    def run(self):
        """运行节点。关闭时的灯状态由 rospy.on_shutdown(_on_shutdown) 负责。"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LEDStripServiceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点启动失败: {e}")
        sys.exit(1)
