#! /usr/bin/env python3
# coding: utf-8
import rospy
import copy
import time
import numpy as np
import tf2_ros
import tf.transformations as tf_trans
from typing import Union
from common.data_types import (PoseQuaternion, HomogeneousMatrix)
from common.logger import SDKLogger

class KuavoRobotToolsCore:
    """Provides core ROS tools for Kuavo humanoid robot transformations.
    
    Attributes:
        tf_buffer (tf2_ros.Buffer): TF2 transform buffer for storing transforms
        tf_listener (tf2_ros.TransformListener): TF2 transform listener
    """
    
    def __init__(self):
        """Initializes TF2 buffer and listener for coordinate transformations."""
        if not hasattr(self, '_initialized'):
            # 初始化TF2相关组件
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _get_tf_tree_transform(self, target_frame: str, source_frame: str, 
                     time=rospy.Time(0), timeout=1.0,
                     return_type: str = "pose_quaternion") -> Union[PoseQuaternion, HomogeneousMatrix, None]:
        """Gets transform between coordinate frames.
        
        Args:
            target_frame (str): Target coordinate frame name
            source_frame (str): Source coordinate frame name
            time (rospy.Time, optional): Time of transform. Defaults to latest.
            timeout (float, optional): Wait timeout in seconds. Defaults to 1.0.
            return_type (str, optional): Return data format. Options: 
                "pose_quaternion", "homogeneous". Defaults to "pose_quaternion".
        
        Returns:
            Union[PoseQuaternion, HomogeneousMatrix, None]: Requested transform data
                or None if failed
        
        Raises:
            tf2_ros.LookupException: If transform is not available
            tf2_ros.ConnectivityException: If transform connectivity issue
            tf2_ros.ExtrapolationException: If transform time is out of range
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time,
                rospy.Duration(timeout)
            )
            return self._parse_transform(transform.transform, return_type)
        except Exception as e:
            SDKLogger.error(f"Transform error: {str(e)}")
            return None

    def _parse_transform(self, transform, return_type: str) -> Union[PoseQuaternion, HomogeneousMatrix, None]:
        """Parses transform data to specified format.
        
        Args:
            transform (geometry_msgs/Transform): Input transform data
            return_type (str): Output format type. Valid values: 
                "pose_quaternion", "homogeneous"
        
        Returns:
            Union[PoseQuaternion, HomogeneousMatrix, None]: Parsed transform data
                in requested format, or None if invalid input
        
        Note:
            Falls back to pose_quaternion format if invalid return_type specified
        """
        if return_type == "pose_quaternion":
            return PoseQuaternion(
                position=(transform.translation.x, 
                         transform.translation.y,
                         transform.translation.z),
                orientation=(transform.rotation.x,
                            transform.rotation.y,
                            transform.rotation.z,
                            transform.rotation.w)
            )
        elif return_type == "homogeneous":
            return HomogeneousMatrix(
                matrix=self._transform_to_homogeneous(transform)
            )
        else:
            SDKLogger.warn(f"Invalid return_type: {return_type}, using default(pose_quaternion)")
            return self._parse_transform(transform, "pose_quaternion")

    def _transform_to_homogeneous(self, transform) -> np.ndarray:
        """Converts geometry_msgs/Transform to homogeneous matrix.
        
        Args:
            transform (geometry_msgs/Transform): Input transform message
        
        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix (numpy.float32)
        
        Example:
            >>> matrix = _transform_to_homogeneous(transform_msg)
            >>> print(matrix.shape)
            (4, 4)
        """
        # 四元数转旋转矩阵
        rotation = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        ]
        rot_matrix = tf_trans.quaternion_matrix(rotation)

        # 设置平移分量
        translation = [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z
        ]
        rot_matrix[:3, 3] = translation
        
        return rot_matrix.astype(np.float32)  # 确保矩阵数据类型一致

    def get_link_pose(self, link_name: str, target_frame: str = "base_link") -> Union[PoseQuaternion, None]:
        """获取指定关节链接的位姿（核心实现）
        
        Args:
            link_name (str): 关节链接名称
            target_frame (str): 目标坐标系
            
        Returns:
            PoseQuaternion: 包含位置和姿态的四元数表示
        """
        return self._get_tf_tree_transform(
            target_frame,
            link_name,
            return_type="pose_quaternion"
        )

# if __name__ == "__main__":
#     robot_tools = KuavoRobotToolsCore()
#     time.sleep(0.1)
#     # 获取位姿信息
#     pose = robot_tools._get_tf_tree_transform("odom", "base_link", return_type="pose_quaternion")
#     print(f"Position: {pose.position}")
#     print(f"Orientation: {pose.orientation}")

#     # 获取齐次矩阵
#     homogeneous = robot_tools._get_tf_tree_transform("odom", "base_link", return_type="homogeneous")
#     print(f"Transformation matrix:\n{homogeneous.matrix}")

#     # 矩阵运算示例
#     transform_matrix = homogeneous.matrix
#     inverse_matrix = np.linalg.inv(transform_matrix)  # 求逆变换
#     print(f"Inverse matrix:\n{inverse_matrix}")