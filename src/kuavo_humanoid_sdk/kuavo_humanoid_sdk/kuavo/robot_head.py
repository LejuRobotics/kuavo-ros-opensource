#!/usr/bin/env python3
# coding: utf-8
import math
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.common.logger import SDKLogger

class KuavoRobotHead:
    """机器人头部控制类"""
    def __init__(self):
        self._kuavo_core = KuavoRobotCore()
    
    def control_head(self, yaw: float, pitch: float)->bool:
        """控制机器人头部。

        Args:
            yaw (float): 头部偏航角，单位为弧度，范围 [-1.396, 1.396] (-80度 到 80度)。
            pitch (float): 头部俯仰角，单位为弧度，范围 [-0.436, 0.436] (-25度 到 25度)。

        Returns:
            bool: 如果控制成功返回True，否则返回False。
        """
        # Check yaw limits (-80 to 80 degrees)
        if yaw < -math.pi*4/9 or yaw > math.pi*4/9:  # -80 to 80 degrees in radians
            SDKLogger.warn(f"[Robot] yaw {yaw} exceeds limit [-{math.pi*4/9:.3f}, {math.pi*4/9:.3f}] radians (-80 to 80 degrees), will be limited")
        limited_yaw = min(math.pi*4/9, max(-math.pi*4/9, yaw))

        # Check pitch limits (-25 to 25 degrees)
        if pitch < -math.pi/7.2 - 0.001 or pitch > math.pi/7.2 + 0.001:  # -25 to 25 degrees in radians
            SDKLogger.warn(f"[Robot] pitch {pitch} exceeds limit [-{math.pi/7.2:.3f}, {math.pi/7.2:.3f}] radians (-25 to 25 degrees), will be limited")
        limited_pitch = min(math.pi/7.2, max(-math.pi/7.2, pitch))
        return self._kuavo_core.control_robot_head(yaw=limited_yaw, pitch=limited_pitch)

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