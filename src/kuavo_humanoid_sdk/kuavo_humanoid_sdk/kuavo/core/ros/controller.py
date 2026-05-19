#!/usr/bin/env python3
# coding: utf-8
"""
机器人控制器管理模块

提供机器人控制器的查询、切换等功能。
"""

import copy
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from kuavo_msgs.srv import SetString, SetStringRequest
from kuavo_humanoid_sdk.interfaces.Controller import (
    ControllerListInfo,
    ControllerResult,
    SwitchControllerResult
)
from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import (
    getControllerList, getControllerListRequest,
    switchController, switchControllerRequest,
    switchToNextController, switchToNextControllerRequest,
)
from kuavo_humanoid_sdk.common.logger import SDKLogger
from typing import Optional


class Controller:
    """机器人控制器管理类，用于获取和查询机器人控制器信息。"""

    def __init__(self):
        """初始化控制器管理类。"""
        self._current_controller_name: Optional[str] = None
        self._query_timer: Optional[rospy.Timer] = None
        self._start_query_timer()

    def _start_query_timer(self):
        """启动定时查询当前控制器名称的定时器。"""
        # 立即执行一次查询
        self._query_current_controller_callback(None)
        # 然后启动定时器，每秒查询一次
        query_interval = 1.0  # 每秒查询一次
        self._query_timer = rospy.Timer(rospy.Duration(query_interval), self._query_current_controller_callback)

    def _query_current_controller_callback(self, event):
        """定时查询当前控制器名称的回调函数。"""
        controller_list_info = self.get_controller_list()
        if controller_list_info is not None:
            self._current_controller_name = controller_list_info.current_controller

    def get_current_controller_name(self) -> Optional[str]:
        """获取当前控制器名称。

        返回内部定时查询存储的当前控制器名称。

        Returns:
            Optional[str]: 当前控制器名称，如果尚未查询到则返回 None
        """
        return copy.deepcopy(self._current_controller_name)

    def get_controller_list(self) -> Optional[ControllerListInfo]:
        """获取控制器列表信息。

        调用/humanoid_controller/get_controller_list 服务获取当前可用的控制器列表，
        包括控制器名称、数量、当前控制器等信息。

        Returns:
            ControllerListInfo: 控制器列表信息，包含以下字段：
                - controller_names (list): 可用控制器名称列表
                - count (int): 控制器数量
                - current_controller (str): 当前控制器名称
                - success (bool): 获取是否成功
                - message (str): 返回消息
            None: 如果服务调用失败则返回 None
        """
        service_name = '/humanoid_controller/get_controller_list'
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            get_controller_list_srv = rospy.ServiceProxy(service_name, getControllerList)

            req = getControllerListRequest()
            resp = get_controller_list_srv(req)

            if not resp.success:
                SDKLogger.error(f"Failed to get controller list: {resp.message}")
                return None

            return ControllerListInfo(
                controller_names=resp.controller_names,
                count=resp.count,
                current_controller=resp.current_controller,
                success=resp.success,
                message=resp.message
            )
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to get controller list: {e}")
        return None

    def switch_controller(self, controller_name: str) -> ControllerResult:
        """切换到指定的控制器。

        Args:
            controller_name (str): 要切换到的控制器名称

        Returns:
            ControllerResult: 切换结果，包含：
                - success (bool): 切换是否成功
                - message (str): 返回消息
        """
        service_name = '/humanoid_controller/switch_controller'
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            switch_controller_srv = rospy.ServiceProxy(service_name, switchController)

            req = switchControllerRequest()
            req.controller_name = controller_name

            resp = switch_controller_srv(req)

            if not resp.success:
                SDKLogger.error(f"Failed to switch controller: {resp.message}")

            return ControllerResult(
                success=resp.success,
                message=resp.message
            )
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to switch controller: {e}")
        return ControllerResult(success=False, message="Service call failed")

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
        service_name = '/humanoid_controller/switch_to_next_controller'
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            switch_to_next_srv = rospy.ServiceProxy(service_name, switchToNextController)

            req = switchToNextControllerRequest()
            resp = switch_to_next_srv(req)

            if not resp.success:
                SDKLogger.error(f"Failed to switch to next controller: {resp.message}")

            return SwitchControllerResult(
                success=resp.success,
                message=resp.message,
                current_controller=resp.current_controller,
                target_controller=resp.next_controller,
                current_index=resp.current_index,
                target_index=resp.next_index
            )
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to switch to next controller: {e}")
        return SwitchControllerResult(
            success=False,
            message="Service call failed",
            current_controller="",
            target_controller="",
            current_index=-1,
            target_index=-1
        )

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
        service_name = '/humanoid_controller/switch_to_previous_controller'
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            switch_to_previous_srv = rospy.ServiceProxy(service_name, switchToNextController)

            req = switchToNextControllerRequest()
            resp = switch_to_previous_srv(req)

            if not resp.success:
                SDKLogger.error(f"Failed to switch to previous controller: {resp.message}")

            return SwitchControllerResult(
                success=resp.success,
                message=resp.message,
                current_controller=resp.current_controller,
                target_controller=resp.next_controller,  # 实际是上一个控制器
                current_index=resp.current_index,
                target_index=resp.next_index  # 实际是上一个索引
            )
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to switch to previous controller: {e}")
        return SwitchControllerResult(
            success=False,
            message="Service call failed",
            current_controller="",
            target_controller="",
            current_index=-1,
            target_index=-1
        )

    def switch_to_vmp_controller(self) -> ControllerResult:
        """切换到 VMP 控制器。

        VMP (Variational Motion Primitive) 控制器是一种基于运动原语的强化学习控制器，
        用于执行预定义的复杂动作轨迹（如武术动作、舞蹈等）。
        它通过 VAE 编码器将参考运动轨迹编码为潜在空间表示，
        结合实时观测数据，由策略网络生成关节控制命令。

        Returns:
            ControllerResult: 操作结果，包含：
                - success (bool): 操作是否成功
                - message (str): 返回消息
        """
        service_name = '/humanoid_controller/switch_to_vmp_controller'
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            switch_to_vmp_srv = rospy.ServiceProxy(service_name, Trigger)

            req = TriggerRequest()
            resp = switch_to_vmp_srv(req)

            if not resp.success:
                SDKLogger.error(f"Failed to switch to VMP controller: {resp.message}")

            return ControllerResult(
                success=resp.success,
                message=resp.message
            )
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to switch to VMP controller: {e}")
        return ControllerResult(success=False, message="Service call failed")

    def switch_to_dance_controller(self, data: str = "") -> ControllerResult:
        """切换到舞蹈控制器（kuavo_msgs/SetString，与节点内 switchDanceControllerByStringCallback 一致）。

        - data 为空：切换到配置中舞蹈列表的第一项
        - data 为 ``#0``、``#1``：按在 get_dance_controller_list 中的下标切换
        - 否则：按已注册的舞蹈控制器名称切换

        Args:
            data: 请求中 ``data`` 字段，默认 ``""`` 表示第一个舞蹈。

        Returns:
            ControllerResult: 操作结果
        """
        service_name = '/humanoid_controller/switch_to_dance_controller'
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            switch_to_dance_srv = rospy.ServiceProxy(service_name, SetString)
            req = SetStringRequest()
            req.data = data
            resp = switch_to_dance_srv(req)

            if not resp.success:
                SDKLogger.error(f"Failed to switch to Dance controller: {resp.message}")

            return ControllerResult(
                success=resp.success,
                message=resp.message
            )
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call to {service_name} failed: {e}")
        except rospy.ROSException as e:
            SDKLogger.error(f"Failed to connect to service {service_name}: {e}")
        except Exception as e:
            SDKLogger.error(f"Failed to switch to Dance controller: {e}")
        return ControllerResult(success=False, message="Service call failed")
