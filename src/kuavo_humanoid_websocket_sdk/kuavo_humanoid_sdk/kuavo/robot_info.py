#!/usr/bin/env python3
# coding: utf-8
from typing import Tuple
from kuavo_humanoid_sdk.interfaces.robot_info import RobotInfoBase
from kuavo_humanoid_sdk.kuavo.core.ros.param import RosParamWebsocket, make_robot_param

class KuavoRobotInfo(RobotInfoBase):
    def __init__(self, robot_type: str = "kuavo"):
        super().__init__(robot_type=robot_type)
        
        # Load robot parameters from ROS parameter server
        kuavo_ros_param = make_robot_param()
        self._ros_param = RosParamWebsocket()
            
        self._robot_version = kuavo_ros_param['robot_version']
        self._robot_version_major = (int(self._robot_version) // 10) % 10
        self._end_effector_type = kuavo_ros_param['end_effector_type']
        self._arm_joint_dof = kuavo_ros_param['arm_dof']
        self._leg_joint_dof = kuavo_ros_param['leg_dof']
        self._waist_joint_dof = kuavo_ros_param.get('waist_dof', 0)  # 腰部关节自由度，默认为0
        self._joint_dof = kuavo_ros_param['arm_dof'] + kuavo_ros_param['leg_dof'] + kuavo_ros_param['head_dof'] + self._waist_joint_dof
        self._joint_names = kuavo_ros_param['joint_names']
        self._end_frames_names = kuavo_ros_param['end_frames_names']
        self._head_joint_dof = kuavo_ros_param['head_dof']
        

        # 动态计算关节索引，避免硬编码
        self._head_joint_names = self._joint_names[-self._head_joint_dof:]
        
        if self._waist_joint_dof > 0:
            self._waist_joint_names = self._joint_names[self._leg_joint_dof:self._leg_joint_dof + self._waist_joint_dof]
        else:
            self._waist_joint_names = []
        
        # 根据版本计算手臂关节索引
        if self._robot_version_major == 6:
            wheel_joint_dof = 4  
            arm_start_idx = wheel_joint_dof
            self._arm_joint_names = self._joint_names[arm_start_idx:arm_start_idx + self._arm_joint_dof]
        else:
            # 手臂关节起始索引需要考虑腰部关节的偏移
            arm_start_idx = self._leg_joint_dof + self._waist_joint_dof
            self._arm_joint_names = self._joint_names[arm_start_idx:arm_start_idx + self._arm_joint_dof]
        self._init_stand_height = kuavo_ros_param['init_stand_height']
    @property
    def robot_version(self) -> str:
        """Return the version of the robot.

        Returns:
            str: The robot version, e.g. "42", "43"...
        """
        return self._robot_version

    @property
    def end_effector_type(self) -> str:
        """Return the type of the end effector.

        Returns:
            str: The end effector type, where:
                - "qiangnao" means "dexteroushand"
                - "lejuclaw" means "lejuclaw"
                - "qiangnao_touch" means "touchdexteroushand"
                - ...
        """
        return self._end_effector_type

    @property
    def joint_names(self) -> list:
        """Return the names of all joints in the robot.

        Returns:
            list: A list containing the names of all robot joints.
        """
        return self._joint_names

    @property
    def joint_dof(self) -> int:
        """Return the total number of joints in the robot.

        Returns:
            int: Total number of joints, e.g. 28
        """
        return self._joint_dof

    @property
    def arm_joint_dof(self) -> int:
        """Return the number of joints in the double-arm.

        Returns:
            int: Number of joints in double-arm, e.g. 14
        """
        return self._arm_joint_dof

    @property
    def arm_joint_names(self) -> list:
        """Return the names of joints in the double-arm.

        Returns:
            list: A list containing the names of joints in the double-arm.
        """
        return self._arm_joint_names

    @property
    def head_joint_dof(self) -> int:
        """Return the number of joints in the head.

        Returns:
            int: Number of joints in head, e.g. 2
        """
        return self._head_joint_dof

    @property
    def waist_joint_dof(self) -> int:
        """Return the number of joints in the waist.

        Returns:
            int: Number of joints in waist, e.g. 0 or 1
        """
        return self._waist_joint_dof

    @property
    def waist_joint_names(self) -> list:
        """Return the names of joints in the waist.

        Returns:
            list: A list containing the names of joints in the waist.
                Note: For websocket SDK, joint_names() does not include waist joints,
                so this will always return an empty list.
        """
        return self._waist_joint_names

    @property
    def head_joint_names(self) -> list:
        """Return the names of joints in the head.

        Returns:
            list: A list containing the names of joints in the head.
        """
        return self._head_joint_names

    @property
    def eef_frame_names(self) -> Tuple[str, str]:
        """Returns the names of the end effector frames.

        Returns:
            Tuple[str, str]:
                A tuple containing the end effector frame names, where:
                - First element is the left hand frame name
                - Second element is the right hand frame name
                e.g. ("zarm_l7_link", "zarm_r7_link")
        """
        return self._end_frames_names[1], self._end_frames_names[2]
    
    @property
    def init_stand_height(self) -> float:
        """Return the height of the robot's center of mass.
        """
        return self._com_height
    
    def __str__(self) -> str:
        return f"KuavoRobotInfo(robot_type={self.robot_type}, robot_version={self.robot_version}, end_effector_type={self.end_effector_type}, joint_names={self.joint_names}, joint_dof={self.joint_dof}, arm_joint_dof={self.arm_joint_dof}, init_stand_height={self.init_stand_height})"