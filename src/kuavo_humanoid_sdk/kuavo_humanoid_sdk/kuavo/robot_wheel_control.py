#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轮臂机器人控制类

提供轮臂机器人的专用控制接口，包括手臂模式切换等。
模仿 KuavoRobotWaist 结构，通过 KuavoRobotCore 调用底层能力。
"""

from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.interfaces.data_types import KuavoArmCtrlMode


class KuavoRobotWheelControl:
    """轮臂机器人控制类"""

    def __init__(self):
        self._kuavo_core = KuavoRobotCore()

    def set_arm_ctrl_mode(self, mode) -> bool:
        """
        设置轮臂手臂控制模式。

        Args:
            mode: 控制模式，可为 int 或 KuavoArmCtrlMode 枚举
                - 0: 保持当前位置控制 (ArmFixed，回零时使用)
                - 1: 重置手臂到初始目标位置 (AutoSwing)
                - 2: 使用外部控制器 (ExternalControl)

        Returns:
            bool: 设置成功返回 True，否则返回 False
        """
        if not isinstance(mode, KuavoArmCtrlMode):
            mode = KuavoArmCtrlMode(int(mode))
        return self._kuavo_core.change_robot_arm_ctrl_mode(mode)

    def reset_and_set_external(self) -> bool:
        """
        轮臂手臂回零后切回外部模式。

        依次执行：先设为 ArmFixed(0) 触发回零，再设为 ExternalControl(2) 以便接收外部轨迹控制。

        Returns:
            bool: 两次切换均成功返回 True
        """
        ok_reset = self.set_arm_ctrl_mode(KuavoArmCtrlMode.AutoSwing)
        ok_external = self.set_arm_ctrl_mode(KuavoArmCtrlMode.ExternalControl)
        return ok_reset and ok_external
