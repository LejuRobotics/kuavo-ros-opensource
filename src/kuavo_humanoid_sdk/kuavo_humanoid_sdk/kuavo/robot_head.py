#!/usr/bin/env python3
# coding: utf-8
import math
import asyncio
import threading
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.common.logger import SDKLogger
import json

class KuavoRobotHead:
    """机器人头部控制类"""
    def __init__(self):
        self._kuavo_core = KuavoRobotCore()

    def _send_log(self, message: str):
        """发送日志到8889端口的辅助方法"""
        self._kuavo_core.logger.send_log(message)
            
        
    def control_head(self, yaw: float, pitch: float) -> bool:
        """
        Control the head of the robot.
        Args:
            yaw (float): The yaw angle of the head in radians, range [-1.396, 1.396] (-80 to 80 degrees).
            pitch (float): The pitch angle of the head in radians, range [-0.436, 0.436] (-25 to 25 degrees).
        Returns:
            bool: True if the head is controlled successfully, False otherwise.
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
    def enable_head_tracking(self, target_id: int)->bool:
        """启用头部跟踪功能，在机器人运动过程中，头部将始终追踪指定的 Apriltag ID

        Args:
            target_id (int): 目标ID。

        Returns:
            bool: 如果启用成功返回True，否则返回False。
        """
        return self._kuavo_core.enable_head_tracking(target_id)
    
    def disable_head_tracking(self)->bool:
        """禁用头部跟踪功能。

        Returns:
            bool: 如果禁用成功返回True，否则返回False。
        """
        return self._kuavo_core.disable_head_tracking()

