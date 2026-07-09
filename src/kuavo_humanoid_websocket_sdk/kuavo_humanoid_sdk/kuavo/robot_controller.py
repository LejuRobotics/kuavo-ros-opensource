#!/usr/bin/env python3
# coding: utf-8
"""
Kuavo 机器人控制器管理模块 (WebSocket)
"""

from typing import Optional

from kuavo_humanoid_sdk.kuavo.core.ros.controller import (
    Controller,
    ControllerListInfo,
    ControllerResult,
    SwitchControllerResult,
)


class KuavoRobotController:
    """Kuavo 机器人控制器管理类"""

    def __init__(self):
        self._controller = Controller()

    def get_current_controller_name(self) -> Optional[str]:
        return self._controller.get_current_controller_name()

    def get_controller_list(self) -> Optional[ControllerListInfo]:
        return self._controller.get_controller_list()

    def switch_controller(self, controller_name: str) -> ControllerResult:
        return self._controller.switch_controller(controller_name)

    def switch_to_next_controller(self) -> SwitchControllerResult:
        return self._controller.switch_to_next_controller()

    def switch_to_previous_controller(self) -> SwitchControllerResult:
        return self._controller.switch_to_previous_controller()

    def switch_to_vmp_controller(self) -> ControllerResult:
        return self._controller.switch_to_vmp_controller()

    def switch_to_dance_controller(self, data: str = "") -> ControllerResult:
        return self._controller.switch_to_dance_controller(data)
