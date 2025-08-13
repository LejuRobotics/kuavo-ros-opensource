#!/usr/bin/env python3
# coding: utf-8
import math
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.interfaces.data_types import (AprilTagData)
from kuavo_humanoid_sdk.kuavo.core.ros.vision import KuavoRobotVisionCore

class KuavoRobotVision:
    """Kuavo机器人视觉系统接口。
    
    提供从不同坐标系获取AprilTag检测数据的接口。
    """
    
    def __init__(self, robot_type: str = "kuavo"):
        """初始化视觉系统。
        
        Args:
            robot_type (str, optional): 机器人类型标识符。默认为"kuavo"
        """
        if not hasattr(self, '_initialized'):
            self._vision_core = KuavoRobotVisionCore()

    def get_data_by_id(self, target_id: int, data_source: str = "base") -> dict:
        """获取指定ID的AprilTag检测数据。
        
        Args:
            target_id (int): 要检索的AprilTag ID
            data_source (str, optional): 数据源坐标系。可以是"base"、"camera"或"odom"。默认为"base"。
                
        Returns:
            dict: 包含位置、方向和元数据的检测数据
        """
        return self._vision_core._get_data_by_id(target_id, data_source)
    
    def get_data_by_id_from_camera(self, target_id: int) -> dict:
        """从相机坐标系获取AprilTag数据。
        
        Args:
            target_id (int): 要检索的AprilTag ID
            
        Returns:
            dict: 包含位置、方向和元数据的检测数据。参见 :meth:`get_data_by_id` 的返回格式说明。
        """
        return self._vision_core._get_data_by_id(target_id, "camera")
    
    def get_data_by_id_from_base(self, target_id: int) -> dict:
        """从基座坐标系获取AprilTag数据。
        
        Args:
            target_id (int): 要检索的AprilTag ID
            
        Returns:
            dict: 包含位置、方向和元数据的检测数据。参见 :meth:`get_data_by_id` 的返回格式说明。
        """
        return self._vision_core._get_data_by_id(target_id, "base")
    
    def get_data_by_id_from_odom(self, target_id: int) -> dict:
        """从里程计坐标系获取AprilTag数据。
        
        Args:
            target_id (int): 要检索的AprilTag ID
            
        Returns:
            dict: 包含位置、方向和元数据的检测数据。参见 :meth:`get_data_by_id` 的返回格式说明。
        """
        return self._vision_core._get_data_by_id(target_id, "odom")
    
    @property
    def apriltag_data_from_camera(self) -> AprilTagData:
        """AprilTagData: 相机坐标系中检测到的所有AprilTag（属性）"""
        return self._vision_core.apriltag_data_from_camera
    
    @property
    def apriltag_data_from_base(self) -> AprilTagData:
        """AprilTagData: 基座坐标系中检测到的所有AprilTag（属性）"""
        return self._vision_core.apriltag_data_from_base
    
    @property
    def apriltag_data_from_odom(self) -> AprilTagData:
        """AprilTagData: 里程计坐标系中检测到的所有AprilTag（属性）"""
        return self._vision_core.apriltag_data_from_odom
