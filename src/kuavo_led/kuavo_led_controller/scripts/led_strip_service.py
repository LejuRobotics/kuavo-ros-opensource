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
from kuavo_msgs.srv import GetPowerBoardStatus, GetPowerBoardStatusResponse
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
from kuavo_msgs.msg import PowerBoardStatus


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

        # 电源板系统状态查询内部服务（0x01），供 battery_info_node 在 ros_service
        # 模式下获取 /power_board_status 数据，避免其直开串口与 LED 冲突。
        try:
            self._power_board_service = rospy.Service(
                '_query_power_board_status',
                GetPowerBoardStatus,
                self.handle_query_power_board_status
            )
        except rospy.ServiceException as e:
            rospy.logwarn(f"无法注册 _query_power_board_status 服务（另一 LED 节点已注册）: {e}")

        rospy.loginfo("LED Strip 服务已启动")
        rospy.loginfo("可用服务:")
        rospy.loginfo("  - /led_strip_set_mode_and_color (SetLEDMode_free)")
        rospy.loginfo("  - /led_strip_close (Trigger)")
        rospy.loginfo("  - /_query_battery_hw (GetBatteryInfo)")
        rospy.loginfo("  - /_query_power_board_status (GetPowerBoardStatus)")

        # 读取 led_for_state 开关（由 set_led_mode.launch 的 <param> 传入）：
        # 控制 Ctrl+C 打断 launch 时是否亮红灯做失能指示。
        #   true（默认）= 亮红灯常亮（利用硬件锁存）
        #   false        = 恢复默认关灯行为，不做失能指示
        self.led_for_state_enabled = rospy.get_param('~led_for_state', True)

        # 节点关闭时的全部善后（设灯 + 停后台线程）统一由 _on_shutdown 负责，
        # 只注册一个 on_shutdown 回调，避免多回调间因执行顺序/覆盖导致红灯
        # 锁存状态被后续 close() 抹掉。
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
    
    def _on_shutdown(self):
        """节点关闭善后（单入口）：先设灯(try)→后停后台线程(finally)。
        led_for_state=True 亮红灯常亮(硬件锁存做失能指示)；False 熄灭。
        先设灯后停线程，避免设灯时与后台线程争串口；finally 保证线程必停。
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
        finally:
            try:
                self._battery_cache.stop()
                rospy.loginfo("[LED Strip] 电池查询后台线程已停止")
            except Exception as e:
                rospy.logerr(f"[LED Strip] 清理电池查询线程失败: {e}")


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

    def handle_query_power_board_status(self, req):
        """返回缓存的电源板系统状态(0x01)，瞬时返回不阻塞 LED。"""
        try:
            status, age = self._battery_cache.get_system_status()

            if status is None:
                if age < 0:
                    msg = "No cached power board status yet"
                else:
                    msg = f"Power board status stale ({age:.0f}s old, may be disconnected)"
                return GetPowerBoardStatusResponse(
                    success=False,
                    message=msg
                )

            # 将缓存字典填充到 PowerBoardStatus 消息（字段一一对应）
            pb_msg = PowerBoardStatus()
            pb_msg.timestamp = rospy.Time.now()
            pb_msg.status_byte1 = status['status_byte1']
            pb_msg.status_byte2 = status['status_byte2']
            pb_msg.status_byte3 = status['status_byte3']
            pb_msg.ntc_temperature = status['ntc_temperature']
            pb_msg.charge_voltage = status['charge_voltage']
            pb_msg.bat1_voltage = status['bat1_voltage']
            pb_msg.bat2_voltage = status['bat2_voltage']
            # Param1 拆解
            pb_msg.stop_int = status['stop_int']
            pb_msg.rf_int = status['rf_int']
            pb_msg.board_is_wheel = status['board_is_wheel']
            pb_msg.ideal_diode_fail = status['ideal_diode_fail']
            pb_msg.cur_ov_protection = status['cur_ov_protection']
            pb_msg.bat1_comm_ok = status['bat1_comm_ok']
            pb_msg.bat2_comm_ok = status['bat2_comm_ok']
            # Param2 拆解
            pb_msg.bat1_exists = status['bat1_exists']
            pb_msg.bat2_exists = status['bat2_exists']
            pb_msg.bat1_low_power = status['bat1_low_power']
            pb_msg.bat2_low_power = status['bat2_low_power']
            pb_msg.charging = status['charging']
            pb_msg.fail_12v = status['fail_12v']
            pb_msg.fail_19v = status['fail_19v']
            pb_msg.fail_24v = status['fail_24v']
            # Param3 拆解
            pb_msg.arm_en = status['arm_en']
            pb_msg.leg_en = status['leg_en']
            pb_msg.out_19v_en = status['out_19v_en']
            pb_msg.out1_12v_en = status['out1_12v_en']
            pb_msg.out2_12v_en = status['out2_12v_en']
            pb_msg.out3_12v_en = status['out3_12v_en']
            pb_msg.out_24v_en = status['out_24v_en']

            return GetPowerBoardStatusResponse(
                status=pb_msg,
                success=True,
                message="Power board status (cached)"
            )

        except Exception as e:
            rospy.logerr(f"电源板系统状态查询失败: {e}")
            return GetPowerBoardStatusResponse(
                success=False,
                message=f"Service error: {str(e)}"
            )

    def run(self):
        """运行节点。关闭时的善后由 rospy.on_shutdown(_on_shutdown) 负责。"""
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
