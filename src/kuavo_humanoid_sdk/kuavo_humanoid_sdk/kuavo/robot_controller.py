#!/usr/bin/env python3
# coding: utf-8
"""
Kuavo 机器人控制器管理模块

提供机器人控制器的高级管理接口，用于获取和切换机器人控制器。
"""

from typing import Optional
from kuavo_humanoid_sdk.interfaces.Controller import (
    ControllerListInfo,
    ControllerResult,
    SwitchControllerResult
)
from kuavo_humanoid_sdk.kuavo.core.ros.controller import Controller


class KuavoRobotController:
    """Kuavo 机器人控制器管理类，用于获取和查询机器人控制器信息。"""

    def __init__(self):
        """初始化机器人控制器管理类。"""
        self._controller = Controller()

    def get_current_controller_name(self) -> Optional[str]:
        """获取当前控制器名称。

        返回内部定时查询存储的当前控制器名称。

        Returns:
            Optional[str]: 当前控制器名称，如果尚未查询到则返回 None
        """
        return self._controller.get_current_controller_name()

    def get_controller_list(self) -> Optional[ControllerListInfo]:
        """获取控制器列表信息。

        Returns:
            ControllerListInfo: 控制器列表信息，包含以下字段：
                - controller_names (list): 可用控制器名称列表
                - count (int): 控制器数量
                - current_controller (str): 当前控制器名称
                - success (bool): 获取是否成功
                - message (str): 返回消息
            None: 如果服务调用失败则返回 None
        """
        return self._controller.get_controller_list()

    def switch_controller(self, controller_name: str) -> ControllerResult:
        """切换到指定的控制器。

        Args:
            controller_name (str): 要切换到的控制器名称

        Returns:
            ControllerResult: 切换结果，包含：
                - success (bool): 切换是否成功
                - message (str): 返回消息
        """
        return self._controller.switch_controller(controller_name)

    def switch_to_next_controller(self) -> SwitchControllerResult:
        """切换到下一个控制器。

        按顺序切换到列表中的下一个控制器（循环切换）。

        Returns:
            SwitchControllerResult: 切换结果，包含：
                - success (bool): 切换是否成功
                - message (str): 返回消息
                - current_controller (str): 当前控制器名称
                - target_controller (str): 下一个控制器名称
                - current_index (int): 当前控制器索引
                - target_index (int): 下一个控制器索引
        """
        return self._controller.switch_to_next_controller()

    def switch_to_previous_controller(self) -> SwitchControllerResult:
        """切换到上一个控制器。

        按顺序切换到列表中的上一个控制器（循环切换）。

        Returns:
            SwitchControllerResult: 切换结果，包含：
                - success (bool): 切换是否成功
                - message (str): 返回消息
                - current_controller (str): 当前控制器名称
                - target_controller (str): 上一个控制器名称
                - current_index (int): 当前控制器索引
                - target_index (int): 上一个控制器索引
        """
        return self._controller.switch_to_previous_controller()

    def switch_to_vmp_controller(self) -> ControllerResult:
        """切换到 VMP 控制器。

        VMP (Variational Motion Primitive) 控制器是一种基于运动原语的强化学习控制器，
        用于执行预定义的复杂动作轨迹（如武术动作、舞蹈等）。
        它通过 VAE 编码器将参考运动轨迹编码为潜在空间表示，
        结合实时观测数据，由策略网络生成关节控制命令。

        支持的机器人型号：
            - 4PRO 机器人（机器人类型 45、46）

        Returns:
            ControllerResult: 操作结果，包含：
                - success (bool): 操作是否成功
                - message (str): 返回消息
        """
        return self._controller.switch_to_vmp_controller()

    def switch_to_dance_controller(self, data: str = "") -> ControllerResult:
        """切换到舞蹈控制器；``data`` 同节点侧 SetString（空=首项，``#0``/``#1``/名称）。"""
        return self._controller.switch_to_dance_controller(data)
