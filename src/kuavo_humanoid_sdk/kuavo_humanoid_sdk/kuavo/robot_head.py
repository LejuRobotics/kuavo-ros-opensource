#!/usr/bin/env python3
# coding: utf-8
import math
import asyncio
import threading
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.kuavo.core.sdk_deprecated import sdk_deprecated
from kuavo_humanoid_sdk.common.logger import SDKLogger
import json

@sdk_deprecated(reason="接口废弃", version="1.2.2", replacement="KuavoRobot", remove_date="2026-06-30")
class KuavoRobotHead:
    """机器人头部控制类
    
    .. warning:: 
        此类已过期废弃，将在 2026-06-30 移除。
        请使用 KuavoRobot 类替代。
    """
    def __init__(self):
        self._kuavo_core = KuavoRobotCore()

    def _send_log(self, message: str):
        """发送日志到8889端口的辅助方法"""
        self._kuavo_core.logger.send_log(message)
    
    @sdk_deprecated(reason="接口废弃", version="1.2.2", replacement="KuavoRobot.control_head", remove_date="2026-06-30")
    def control_head(self, yaw: float, pitch: float) -> bool:
        """控制机器人的头部关节运动。
        
        .. warning:: 
            此接口已过期废弃，将在 2026-06-30 移除。
            请使用 KuavoRobot.control_head() 替代。
        
        Args:
            yaw (float): 头部的偏航角,单位弧度,范围[-1.396, 1.396](-80到80度)。
            pitch (float): 头部的俯仰角,单位弧度,范围[-0.436, 0.436](-25到25度)。
            
        Returns:
            bool: 如果头部控制成功返回True,否则返回False。
        """
        # 发送开始控制头部的日志
        self._send_log(f"开始控制头部运动: yaw={yaw:.3f}, pitch={pitch:.3f}")
        
        limited_yaw = yaw
        limited_pitch = pitch
        
        # 原有的代码逻辑保持不变
        # Check yaw limits (-80 to 80 degrees)
        if yaw < -math.pi*4/9 or yaw > math.pi*4/9:  # -80 to 80 degrees in radians
            SDKLogger.warn(f"[Robot] yaw {yaw} exceeds limit [-{math.pi*4/9:.3f}, {math.pi*4/9:.3f}] radians (-80 to 80 degrees), will be limited")
            limited_yaw = min(math.pi*4/9, max(-math.pi*4/9, yaw))
            self._send_log(f"yaw值超限，已限制为: {limited_yaw:.3f}")
            
        # Check pitch limits (-25 to 25 degrees)
        if pitch < -math.pi/7.2 - 0.001 or pitch > math.pi/7.2 + 0.001:  # -25 to 25 degrees in radians
            SDKLogger.warn(f"[Robot] pitch {pitch} exceeds limit [-{math.pi/7.2:.3f}, {math.pi/7.2:.3f}] radians (-25 to 25 degrees), will be limited")
            limited_pitch = min(math.pi/7.2, max(-math.pi/7.2, pitch))
            self._send_log(f"pitch值超限，已限制为: {limited_pitch:.3f}")
        
        # 执行头部控制
        result = self._kuavo_core.control_robot_head(yaw=limited_yaw, pitch=limited_pitch)
        
        # 发送执行结果日志
        self._send_log(f"头部控制完成: yaw={limited_yaw:.3f}, pitch={limited_pitch:.3f}, 结果={'成功' if result else '失败'}")
        
        return result
    @sdk_deprecated(reason="接口废弃", version="1.2.2", replacement="KuavoRobot.enable_head_tracking", remove_date="2026-06-30")
    def enable_head_tracking(self, target_id: int)->bool:
        """启用头部跟踪功能，在机器人运动过程中，头部将始终追踪指定的 Apriltag ID

        .. warning:: 
            此接口已过期废弃，将在 2026-06-30 移除。
            请使用 KuavoRobot.enable_head_tracking() 替代。

        Args:
            target_id (int): 目标ID。

        Returns:
            bool: 如果启用成功返回True，否则返回False。
        """
        return self._kuavo_core.enable_head_tracking(target_id)
    
    @sdk_deprecated(reason="接口废弃", version="1.2.2", replacement="KuavoRobot.disable_head_tracking", remove_date="2026-06-30")
    def disable_head_tracking(self)->bool:
        """禁用头部跟踪功能。

        .. warning:: 
            此接口已过期废弃，将在 2026-06-30 移除。
            请使用 KuavoRobot.disable_head_tracking() 替代。

        Returns:
            bool: 如果禁用成功返回True，否则返回False。
        """
        return self._kuavo_core.disable_head_tracking()

