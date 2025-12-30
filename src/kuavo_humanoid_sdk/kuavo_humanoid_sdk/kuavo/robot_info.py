#!/usr/bin/env python3
# coding: utf-8
from typing import Tuple
from kuavo_humanoid_sdk.interfaces.robot_info import RobotInfoBase
from kuavo_humanoid_sdk.kuavo.core.ros.param import RosParameter, make_robot_param

class KuavoRobotInfo(RobotInfoBase):
    def __init__(self, robot_type: str = "kuavo"):
        super().__init__(robot_type=robot_type)
        
        # Load robot parameters from ROS parameter server
        kuavo_ros_param = make_robot_param()
        self._ros_param = RosParameter()
            
        self._robot_version = kuavo_ros_param['robot_version']
        self._end_effector_type = kuavo_ros_param['end_effector_type']
        self._arm_joint_dof = kuavo_ros_param['arm_dof']
        self._joint_dof = kuavo_ros_param['arm_dof'] + kuavo_ros_param['leg_dof'] + kuavo_ros_param['head_dof']
        self._joint_names = kuavo_ros_param['joint_names']
        self._end_frames_names = kuavo_ros_param['end_frames_names']
        self._head_joint_dof = kuavo_ros_param['head_dof']
        self._head_joint_names = self._joint_names[-2:]
        self._arm_joint_names = self._joint_names[12:self._arm_joint_dof + 12]
    @property
    def robot_version(self) -> str:
        """返回 Kuavo 机器人的版本。

        Returns:
            str: 机器人版本号，例如 "42"、"45" 等。
        """
        return self._robot_version

    @property
    def end_effector_type(self) -> str:
        """返回 Kuavo 机器人末端执行器的类型。

        Returns:
            str: 末端执行器类型，其中：
                - ``qiangnao`` 表示普通灵巧手
                - ``lejuclaw`` 表示乐聚二指夹爪
                - ``qiangnao_touch`` 表示触觉灵巧手
                - ...
        """
        return self._end_effector_type

    @property
    def joint_names(self) -> list:
        """返回 Kuavo 机器人所有关节的名称。

        Returns:
            list: 包含所有关节名称的列表。
        """
        return self._joint_names

    @property
    def joint_dof(self) -> int:
        """返回 Kuavo 机器人的总关节数。

        Returns:
            int: 总关节数，例如 28。
        """
        return self._joint_dof

    @property
    def arm_joint_dof(self) -> int:
        """返回 Kuavo 机器人双臂的关节数。

        Returns:
            int: 双臂的关节数，例如 14。 
        """
        return self._arm_joint_dof

    @property
    def arm_joint_names(self) -> list:
        """返回 Kuavo 机器人双臂关节的名称。

        Returns:
            list: 包含双臂关节名称的列表。
        """
        return self._arm_joint_names

    @property
    def head_joint_dof(self) -> int:
        """返回 Kuavo 机器人头部的关节数。

        Returns:
            int: 头部的关节数，例如 2。
        """
        return self._head_joint_dof

    @property
    def head_joint_names(self) -> list:
        """返回 Kuavo 机器人头部关节的名称。

        Returns:
            list: 包含头部关节名称的列表。
        """
        return self._head_joint_names

    @property
    def eef_frame_names(self) -> Tuple[str, str]:
        """返回 Kuavo 机器人末端执行器坐标系的名称。

        Returns:
            Tuple[str, str]: 包含末端执行器坐标系名称的元组，其中：\n
                - 第一个元素是左手坐标系名称\n
                - 第二个元素是右手坐标系名称\n
                例如 ("zarm_l7_link", "zarm_r7_link") \n
        """
        return self._end_frames_names[1], self._end_frames_names[2]
    
    def __str__(self) -> str:
        return f"KuavoRobotInfo(robot_type={self.robot_type}, robot_version={self.robot_version}, end_effector_type={self.end_effector_type}, joint_names={self.joint_names}, joint_dof={self.joint_dof}, arm_joint_dof={self.arm_joint_dof})"