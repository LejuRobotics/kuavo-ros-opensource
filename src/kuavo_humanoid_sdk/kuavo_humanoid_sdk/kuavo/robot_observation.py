#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoJointCommand, KuavoTwist)
from kuavo_humanoid_sdk.kuavo.core.ros.observation import KuavoRobotObservationCore

class KuavoRobotObservation:
    """用于访问机器人观测数据的类。
    
    该类提供了一个高级接口来访问机器人的观测数据，包括关节命令、速度命令和姿态命令。
    """
    
    def __init__(self, robot_type: str = "kuavo"):
        """初始化机器人观测接口
        
        Args:
            robot_type (str): 机器人类型。默认为"kuavo"
        """
        self._obs_core = KuavoRobotObservationCore()

    @property
    def joint_command(self) -> KuavoJointCommand:
        """获取当前关节控制命令
        
        Returns:
            KuavoJointCommand: 包含所有机器人关节的位置、速度和力矩命令的对象。
        """
        return self._obs_core.joint_command

    @property
    def cmd_vel(self) -> KuavoTwist:
        """获取当前 cmd_vel 速度控制命令
        
        Returns:
            KuavoTwist: 包含线速度(m/s)和角速度(rad/s)命令的对象。
        """
        return self._obs_core.cmd_vel

    @property
    def cmd_pose(self) -> KuavoTwist:
        """获取当前 cmd_pose 姿态控制命令
        
        Returns:
            KuavoTwist: 包含线性姿态命令(m)和角度姿态命令(rad)的对象。
        """
        return self._obs_core.cmd_pose

    @property    
    def arm_position_command(self) -> list:
        """获取手臂关节的位置控制命令
        
        Returns:
            list: 手臂关节(索引12-25)的位置命令，单位为弧度。
        """
        return self._obs_core.arm_position_command

    @property
    def head_position_command(self) -> list:
        """获取头部关节的位置控制命令
        
        Returns:
            list: 头部关节(索引26-27)的位置命令，单位为弧度。
        """
        return self._obs_core.head_position_command