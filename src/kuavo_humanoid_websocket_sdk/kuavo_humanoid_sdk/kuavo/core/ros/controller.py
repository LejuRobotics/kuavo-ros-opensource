#!/usr/bin/env python3
# coding: utf-8
"""
机器人控制器管理模块 (WebSocket)

通过 WebSocket (roslibpy) 调用 ROS 服务，提供机器人控制器的查询、切换等功能。
"""

import copy
from dataclasses import dataclass
from typing import Optional

import roslibpy

from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK
from kuavo_humanoid_sdk.kuavo.core.ros.param import is_wheel_arm_robot_from_client


@dataclass
class ControllerListInfo:
    """控制器列表信息"""
    controller_names: list
    count: int
    current_controller: str
    success: bool
    message: str


@dataclass
class ControllerResult:
    """控制器操作结果"""
    success: bool
    message: str


@dataclass
class SwitchControllerResult:
    """控制器切换结果"""
    success: bool
    message: str
    current_controller: str
    target_controller: str
    current_index: int
    target_index: int


class Controller:
    """机器人控制器管理类 (WebSocket)，用于获取和查询机器人控制器信息。"""

    def __init__(self):
        self._current_controller_name: Optional[str] = None
        self._websocket = WebSocketKuavoSDK()

    def _is_wheel_arm_robot(self) -> bool:
        return is_wheel_arm_robot_from_client(self._websocket.client)

    def get_current_controller_name(self) -> Optional[str]:
        if self._current_controller_name is not None:
            return copy.deepcopy(self._current_controller_name)
        controller_list_info = self.get_controller_list()
        if controller_list_info is not None:
            self._current_controller_name = controller_list_info.current_controller
        return copy.deepcopy(self._current_controller_name)

    def get_controller_list(self) -> Optional[ControllerListInfo]:
        if self._is_wheel_arm_robot():
            return ControllerListInfo(
                controller_names=["mpc"],
                count=1,
                current_controller="mpc",
                success=True,
                message="wheel-arm: stub controller list (stack has no /humanoid_controller/get_controller_list)",
            )

        service_name = '/humanoid_controller/get_controller_list'
        try:
            service = roslibpy.Service(self._websocket.client, service_name, 'kuavo_msgs/getControllerList')
            response = service.call({})
            if not response.get('success', False):
                SDKLogger.error(f"Failed to get controller list: {response.get('message', '')}")
                return None
            info = ControllerListInfo(
                controller_names=response.get('controller_names', []),
                count=response.get('count', 0),
                current_controller=response.get('current_controller', ''),
                success=response.get('success', False),
                message=response.get('message', ''),
            )
            self._current_controller_name = info.current_controller
            return info
        except Exception as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
            return None

    def switch_controller(self, controller_name: str) -> ControllerResult:
        service_name = '/humanoid_controller/switch_controller'
        try:
            service = roslibpy.Service(self._websocket.client, service_name, 'kuavo_msgs/switchController')
            response = service.call({'controller_name': controller_name})
            if not response.get('success', False):
                SDKLogger.error(f"Failed to switch controller: {response.get('message', '')}")
            return ControllerResult(
                success=response.get('success', False),
                message=response.get('message', ''),
            )
        except Exception as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
            return ControllerResult(success=False, message="Service call failed")

    def switch_to_next_controller(self) -> SwitchControllerResult:
        service_name = '/humanoid_controller/switch_to_next_controller'
        try:
            service = roslibpy.Service(self._websocket.client, service_name, 'kuavo_msgs/switchToNextController')
            response = service.call({})
            if not response.get('success', False):
                SDKLogger.error(f"Failed to switch to next controller: {response.get('message', '')}")
            return SwitchControllerResult(
                success=response.get('success', False),
                message=response.get('message', ''),
                current_controller=response.get('current_controller', ''),
                target_controller=response.get('next_controller', ''),
                current_index=response.get('current_index', -1),
                target_index=response.get('next_index', -1),
            )
        except Exception as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
            return SwitchControllerResult(
                success=False, message="Service call failed",
                current_controller="", target_controller="",
                current_index=-1, target_index=-1,
            )

    def switch_to_previous_controller(self) -> SwitchControllerResult:
        service_name = '/humanoid_controller/switch_to_previous_controller'
        try:
            service = roslibpy.Service(self._websocket.client, service_name, 'kuavo_msgs/switchToNextController')
            response = service.call({})
            if not response.get('success', False):
                SDKLogger.error(f"Failed to switch to previous controller: {response.get('message', '')}")
            return SwitchControllerResult(
                success=response.get('success', False),
                message=response.get('message', ''),
                current_controller=response.get('current_controller', ''),
                target_controller=response.get('next_controller', ''),
                current_index=response.get('current_index', -1),
                target_index=response.get('next_index', -1),
            )
        except Exception as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
            return SwitchControllerResult(
                success=False, message="Service call failed",
                current_controller="", target_controller="",
                current_index=-1, target_index=-1,
            )

    def switch_to_vmp_controller(self) -> ControllerResult:
        service_name = '/humanoid_controller/switch_to_vmp_controller'
        try:
            service = roslibpy.Service(self._websocket.client, service_name, 'std_srvs/Trigger')
            response = service.call({})
            if not response.get('success', False):
                SDKLogger.error(f"Failed to switch to VMP controller: {response.get('message', '')}")
            return ControllerResult(
                success=response.get('success', False),
                message=response.get('message', ''),
            )
        except Exception as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
            return ControllerResult(success=False, message="Service call failed")

    def switch_to_dance_controller(self, data: str = "") -> ControllerResult:
        service_name = '/humanoid_controller/switch_to_dance_controller'
        try:
            service = roslibpy.Service(self._websocket.client, service_name, 'kuavo_msgs/SetString')
            response = service.call({'data': data})
            if not response.get('success', False):
                SDKLogger.error(f"Failed to switch to Dance controller: {response.get('message', '')}")
            return ControllerResult(
                success=response.get('success', False),
                message=response.get('message', ''),
            )
        except Exception as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
            return ControllerResult(success=False, message="Service call failed")
