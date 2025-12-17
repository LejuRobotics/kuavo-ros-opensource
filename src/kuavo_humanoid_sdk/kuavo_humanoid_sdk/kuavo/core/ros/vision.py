#! /usr/bin/env python3
# coding: utf-8

import copy
import time
from collections import deque
from typing import Tuple, Optional

from kuavo_humanoid_sdk.kuavo.core.ros.param import make_robot_param, EndEffectorType
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.interfaces.data_types import (AprilTagData, AprilTagDetection)

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, PoseStamped


class KuavoRobotVisionCore:
    """Handles vision-related data processing for Kuavo humanoid robot.
    
    Attributes:
        tf_buffer (tf2_ros.Buffer): TF2 transform buffer
        tf_listener (tf2_ros.TransformListener): TF2 transform listener
        tf_broadcaster (tf2_ros.TransformBroadcaster): TF2 transform broadcaster
    """
    
    def __init__(self):
        """Initializes vision system components including TF and AprilTag subscribers."""
        if not hasattr(self, '_initialized'):


            # FIRST: Initialize data structures before creating subscribers
            """ data """
            self._apriltag_data_from_camera = AprilTagData(
                id = [],
                size = [],
                pose = []
            )

            self._apriltag_data_from_base = AprilTagData(
                id = [],
                size = [],
                pose = []
            )

            self._apriltag_data_from_odom = AprilTagData(
                id = [],
                size = [],
                pose = []
            )
            
            # THEN: Create subscribers after all data structures are initialized
            self._apriltag_data_camera_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._apriltag_data_callback_camera)
            self._apriltag_data_base_sub = rospy.Subscriber('/robot_tag_info', AprilTagDetectionArray, self._apriltag_data_callback_base)
            self._apriltag_data_odom_sub = rospy.Subscriber('/robot_tag_info_odom', AprilTagDetectionArray, self._apriltag_data_callback_odom)
            
            # Mark as initialized
            self._initialized = True

    def _apriltag_data_callback_camera(self, data):
        """Callback for processing AprilTag detections from camera.
        
        Args:
            data (AprilTagDetectionArray): Raw detection data from camera
        """
        # 清空之前的数据
        self._apriltag_data_from_camera.id = []
        self._apriltag_data_from_camera.size = []
        self._apriltag_data_from_camera.pose = []
        
        # 处理每个标签检测
        for detection in data.detections:
            # 添加ID
            for id in detection.id:
                self._apriltag_data_from_camera.id.append(id)
            
            # 添加尺寸 (从size数组中获取)
            if detection.size and len(detection.size) > 0:
                self._apriltag_data_from_camera.size.append(detection.size[0])
            else:
                self._apriltag_data_from_camera.size.append(0.0)
            
            # 添加姿态
            self._apriltag_data_from_camera.pose.append(AprilTagDetection(
                position=detection.pose.pose.pose.position,
                orientation=detection.pose.pose.pose.orientation
            ))
        # # debug
        # rospy.loginfo("Apriltag data from camera: %s", self._apriltag_data_from_camera)

    def _apriltag_data_callback_base(self, data):
        """Callback for processing AprilTag detections from base link.
        
        Args:
            data (AprilTagDetectionArray): Raw detection data from base frame
        """
        # 清空之前的数据
        self._apriltag_data_from_base.id = []
        self._apriltag_data_from_base.size = []
        self._apriltag_data_from_base.pose = []
        
        # 处理每个标签检测
        for detection in data.detections:
            # 添加ID
            for id in detection.id:
                self._apriltag_data_from_base.id.append(id)
            
            # 添加尺寸 (从size数组中获取)
            if detection.size and len(detection.size) > 0:
                self._apriltag_data_from_base.size.append(detection.size[0])
            else:
                self._apriltag_data_from_base.size.append(0.0)
            
            # 添加姿态
            self._apriltag_data_from_base.pose.append(AprilTagDetection(
                position=detection.pose.pose.pose.position,
                orientation=detection.pose.pose.pose.orientation
            ))

        # # debug
        # rospy.loginfo("Apriltag data from base: %s", self._apriltag_data_from_base)

    def _apriltag_data_callback_odom(self, data):
        """Callback for processing AprilTag detections from odom.
        
        Args:
            data (AprilTagDetectionArray): Raw detection data from odom frame
        """
        # 清空之前的数据
        self._apriltag_data_from_odom.id = []
        self._apriltag_data_from_odom.size = []
        self._apriltag_data_from_odom.pose = []
        
        # 处理每个标签检测
        for detection in data.detections:
            # 添加ID
            for id in detection.id:
                self._apriltag_data_from_odom.id.append(id)
            
            # 添加尺寸 (从size数组中获取)
            if detection.size and len(detection.size) > 0:
                self._apriltag_data_from_odom.size.append(detection.size[0])
            else:
                self._apriltag_data_from_odom.size.append(0.0)

            # 添加姿态
            self._apriltag_data_from_odom.pose.append(AprilTagDetection(
                position=detection.pose.pose.pose.position,
                orientation=detection.pose.pose.pose.orientation
            ))

        # # debug
        # rospy.loginfo("Apriltag data from odom: %s", self._apriltag_data_from_odom)
        
    @property 
    def apriltag_data_from_camera(self) -> AprilTagData:
        """AprilTag detection data in camera coordinate frame.
        
        Returns:
            AprilTagData: Contains lists of tag IDs, sizes and poses
        """
        return self._apriltag_data_from_camera
    
    @property
    def apriltag_data_from_base(self) -> AprilTagData:
        """AprilTag detection data in base_link coordinate frame.
        
        Returns:
            AprilTagData: Contains lists of tag IDs, sizes and poses
        """
        return self._apriltag_data_from_base
    
    @property
    def apriltag_data_from_odom(self) -> AprilTagData:
        """AprilTag detection data in odom coordinate frame.
        
        Returns:
            AprilTagData: Contains lists of tag IDs, sizes and transformed poses
        """
        return self._apriltag_data_from_odom

    def _get_data_by_id(self, target_id: int, data_source: str = "base") -> Optional[dict]:
        """Retrieves AprilTag data by specific ID from selected source.
        
        Args:
            target_id (int): AprilTag ID to search for
            data_source (str): Data source selector, valid options: 
                "camera", "base", "odom"
        
        Returns:
            Optional[dict]: Dictionary containing 'sizes' and 'poses' lists if found,
                None if no matching data
        
        Raises:
            ValueError: If invalid data_source is specified
        """
        data_map = {
            "camera": self._apriltag_data_from_camera,
            "base": self._apriltag_data_from_base,
            "odom": self._apriltag_data_from_odom
        }
        
        if data_source not in data_map:
            # SDKLogger.error(f"Invalid data source: {data_source}, must be one of {list(data_map.keys())}")
            return None
        
        data = data_map[data_source]
        
        # 查找所有匹配的索引
        indices = [i for i, tag_id in enumerate(data.id) if tag_id == target_id]
        
        if not indices:
            # SDKLogger.debug(f"No data found for tag ID {target_id} in {data_source} source")
            return None
        
        return {
            "sizes": [data.size[i] for i in indices],
            "poses": [data.pose[i] for i in indices]
        }

# if __name__ == "__main__":

#     kuavo_robot_vision_core = KuavoRobotVisionCore()
#     time.sleep(5)
#     print("apriltag_data_from_camera:")
#     print(kuavo_robot_vision_core.apriltag_data_from_camera)
#     print("apriltag_data_from_base:")
#     print(kuavo_robot_vision_core.apriltag_data_from_base)
#     print("apriltag_data_from_odom:")
#     print(kuavo_robot_vision_core.apriltag_data_from_odom)
#     rospy.spin()