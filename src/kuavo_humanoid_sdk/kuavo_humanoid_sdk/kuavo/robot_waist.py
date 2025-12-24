#!/usr/bin/env python3
# coding: utf-8
import math

from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.common.logger import SDKLogger
import json

class KuavoRobotWaist:
    """机器人腰部控制类"""
    def __init__(self):
        self._kuavo_core = KuavoRobotCore()

    def _send_log(self, message: str):
        """发送日志到8889端口的辅助方法"""
        self._kuavo_core.logger.send_log(message)
            
        
    def control_waist(self, target_pos: list) -> bool:
        # 发送开始控制头部的日志
        self._send_log(f"开始控制腰部运动: yaw={target_pos[0]:.3f}")
        
        limited_yaw = target_pos[0]
        # 第一个腰关节为yaw
        if target_pos[0] < -180 or target_pos[0] > 180:  # -120 to 120 degrees in radians
            SDKLogger.warn(f"[Robot] yaw {target_pos[0]} exceeds limit [-{180:.3f}, {180:.3f}] radians (-180 to 180 degrees), will be limited")
            limited_yaw = min(180, max(-180, target_pos[0]))
            self._send_log(f"yaw值超限，已限制为: {limited_yaw:.3f}")
        
        # 执行腰部控制
        data = list(target_pos)
        result = self._kuavo_core.control_robot_waist(data)
        
        # 发送执行结果日志
        self._send_log(f"腰部控制完成: yaw={limited_yaw:.3f}, 结果={'成功' if result else '失败'}")
        
        return result

    def getCurrentWaistPos(self) -> list:
        """获取当前腰部姿态"""
        result, data = self._kuavo_core.get_robot_state()
