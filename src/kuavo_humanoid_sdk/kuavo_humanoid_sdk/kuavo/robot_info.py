#!/usr/bin/env python3
# coding: utf-8
from typing import Tuple
import math
import xml.etree.ElementTree as ET
from kuavo_humanoid_sdk.interfaces.robot_info import RobotInfoBase
from kuavo_humanoid_sdk.kuavo.core.ros.param import RosParameter, make_robot_param
from kuavo_humanoid_sdk.common.logger import SDKLogger

class KuavoRobotInfo(RobotInfoBase):
    def __init__(self, robot_type: str = "kuavo"):
        super().__init__(robot_type=robot_type)
        
        # Load robot parameters from ROS parameter server
        kuavo_ros_param = make_robot_param()
        self._ros_param = RosParameter()
            
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
        
        # Extract waist joint names (waist joints are after leg joints and before arm joints)
        if self._waist_joint_dof > 0:
            self._waist_joint_names = self._joint_names[self._leg_joint_dof:self._leg_joint_dof + self._waist_joint_dof]
        else:
            self._waist_joint_names = []
        
        # 根据版本计算手臂关节索引
        if self._robot_version_major == 6:
            print("\033[35m当前为轮臂模型\033[0m")
            # 版本6（轮臂模型）：折叠臂关节（4个） + 手臂关节
            wheel_joint_dof = 4  # 轮臂模型折叠臂关节
            arm_start_idx = wheel_joint_dof
            self._arm_joint_names = self._joint_names[arm_start_idx:arm_start_idx + self._arm_joint_dof]
        else:
            print("\033[35m当前为双足模型\033[0m")
            arm_start_idx = self._leg_joint_dof + self._waist_joint_dof
            self._arm_joint_names = self._joint_names[arm_start_idx:arm_start_idx + self._arm_joint_dof]
        self._init_stand_height = kuavo_ros_param['init_stand_height']

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
    def waist_joint_dof(self) -> int:
        """返回 Kuavo 机器人腰部的关节数。

        Returns:
            int: 腰部的关节数，例如 0 或 1。
        """
        return self._waist_joint_dof

    @property
    def waist_joint_names(self) -> list:
        """返回 Kuavo 机器人腰部关节的名称。

        Returns:
            list: 包含腰部关节名称的列表。
        """
        return self._waist_joint_names

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

    @property
    def init_stand_height(self) -> float:
        """返回 Kuavo 机器人初始化站立时的质心高度。

        Returns:
            float: 初始化站立时的质心高度
        """
        return self._init_stand_height
    
    def get_arm_joint_limits(self) -> Tuple[list, list]:
        """获取手臂关节的角度限制范围（基于URDF文件中的真实物理限制）。
        
        返回每个关节的最小值和最大值（单位：弧度）。
        这些限制从URDF文件中解析得到，基于机器人的真实物理结构，用于防止电机堵转。
        
        Returns:
            Tuple[list, list]: (arm_min, arm_max) 
                - arm_min: 每个关节的最小角度值列表（弧度）
                - arm_max: 每个关节的最大角度值列表（弧度）
                
        Note:
            这些限制从URDF文件的 <joint><limit> 标签中解析得到，针对每个关节的真实角度范围进行了定义。
            动态从URDF中提取所有手臂相关的link和关节，兼容不同机器人版本。
        """
        try:
            # 获取URDF内容
            robot_desc = self._ros_param.humanoid_description()
            if robot_desc is None:
                SDKLogger.warn("Failed to get URDF description, using default limits")
                return self._get_default_arm_joint_limits()
            
            # 解析URDF XML
            root = ET.fromstring(robot_desc)
            
            # 获取关节限制
            arm_min = []
            arm_max = []
            
            # 直接使用 arm_joint_names 从 URDF 中查找对应的关节
            for joint_name in self._arm_joint_names:
                # 直接通过关节名称查找
                joint_elem = root.find(f".//joint[@name='{joint_name}']")
                
                if joint_elem is not None:
                    limit_elem = joint_elem.find("limit")
                    if limit_elem is not None:
                        lower = float(limit_elem.get("lower", str(-math.pi)))
                        upper = float(limit_elem.get("upper", str(math.pi)))
                        arm_min.append(lower)
                        arm_max.append(upper)
                    else:
                        SDKLogger.warn(f"Joint {joint_name} has no limit tag, using default [-π, π]")
                        arm_min.append(-math.pi)
                        arm_max.append(math.pi)
                else:
                    SDKLogger.warn(f"Joint {joint_name} not found in URDF, using default [-π, π]")
                    arm_min.append(-math.pi)
                    arm_max.append(math.pi)
            
            if len(arm_min) != self._arm_joint_dof or len(arm_max) != self._arm_joint_dof:
                SDKLogger.warn(f"Failed to parse all joint limits from URDF (got {len(arm_min)}/{self._arm_joint_dof}), using default limits")
                return self._get_default_arm_joint_limits()
            
            return arm_min, arm_max
            
        except Exception as e:
            SDKLogger.error(f"Error parsing joint limits from URDF: {e}, using default limits")
            return self._get_default_arm_joint_limits()
    
    def _get_default_arm_joint_limits(self) -> Tuple[list, list]:
        """获取默认的手臂关节限制（当无法从URDF解析时使用）。
        
        Returns:
            Tuple[list, list]: (arm_min, arm_max) 默认限制值
        """
        # 默认使用较大的范围作为后备方案
        arm_min = [-math.pi] * self._arm_joint_dof
        arm_max = [math.pi] * self._arm_joint_dof
        return arm_min, arm_max
    
    def __str__(self) -> str:
        return (
            f"KuavoRobotInfo("
            f"robot_type={self.robot_type}, "
            f"robot_version={self.robot_version}, "
            f"end_effector_type={self.end_effector_type}, "
            f"joint_names={self.joint_names}, "
            f"joint_dof={self.joint_dof}, "
            f"arm_joint_dof={self.arm_joint_dof}, "
            f"init_stand_height={self.init_stand_height}"
            f")"
        )