# -*- coding: utf-8 -*-
"""
ROS位姿消息转换工具：将变换矩阵和位姿数据转换为ROS消息
"""
import numpy as np

try:
    import rospy
    from visualization_msgs.msg import Marker
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    Marker = None

from .transform_utils import rotation_matrix_to_quaternion


def transform_matrix_to_marker(T: np.ndarray, frame_id: str = "shoulder_frame", 
                               marker_id: int = 0, marker_type: int = Marker.SPHERE,
                               scale: float = 0.02, color: tuple = (1.0, 0.0, 0.0, 1.0),
                               namespace: str = "tool_pose", lifetime: float = 0.0) -> Marker:
    """
    将4x4变换矩阵直接转换为Marker消息

    参数:
        T: 4x4 齐次变换矩阵，位置单位 mm
        frame_id: 坐标系ID
        marker_id: Marker ID
        marker_type: Marker类型（如 Marker.SPHERE, Marker.ARROW等）
        scale: Marker尺寸（米）
        color: RGBA颜色 (r, g, b, a)，范围0-1
        namespace: Marker命名空间
        lifetime: Marker保留时间（秒），0表示永久保留

    返回:
        Marker 消息
    """
    if not ROS_AVAILABLE:
        raise ImportError("rospy is not available. Cannot create Marker message.")
    
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    
    # 提取位置（单位：mm -> m）
    marker.pose.position.x = T[0, 3] / 1000.0
    marker.pose.position.y = T[1, 3] / 1000.0
    marker.pose.position.z = T[2, 3] / 1000.0
    
    # 提取旋转并转换为四元数
    R = T[:3, :3]
    quat = rotation_matrix_to_quaternion(R)
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    
    # 尺寸
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    
    # 颜色
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    
    # 保留时间（0表示永久保留）
    marker.lifetime = rospy.Duration(lifetime)
    
    return marker


def pose_to_marker(pos_mm: np.ndarray, quat: np.ndarray, frame_id: str = "shoulder_frame",
                   marker_id: int = 0, marker_type: int = Marker.SPHERE,
                   scale: float = 0.02, color: tuple = (1.0, 0.0, 0.0, 1.0),
                   namespace: str = "tool_pose", lifetime: float = 0.0) -> Marker:
    """
    将位置和四元数直接转换为Marker消息用于可视化

    参数:
        pos_mm: 位置数组 [x, y, z]，单位 mm
        quat: 四元数数组 [x, y, z, w]
        frame_id: 坐标系ID
        marker_id: Marker ID
        marker_type: Marker类型（如 Marker.SPHERE, Marker.ARROW等）
        scale: Marker尺寸（米）
        color: RGBA颜色 (r, g, b, a)，范围0-1
        namespace: Marker命名空间
        lifetime: Marker保留时间（秒），0表示永久保留

    返回:
        Marker 消息
    """
    if not ROS_AVAILABLE:
        raise ImportError("rospy is not available. Cannot create Marker message.")
    
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    
    # 位置（单位：mm -> m）
    marker.pose.position.x = pos_mm[0] / 1000.0
    marker.pose.position.y = pos_mm[1] / 1000.0
    marker.pose.position.z = pos_mm[2] / 1000.0
    
    # 四元数
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    
    # 尺寸
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    
    # 颜色
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    
    # 保留时间（0表示永久保留）
    marker.lifetime = rospy.Duration(lifetime)
    
    return marker


def calculate_position_distance(pos1_m: np.ndarray, pos2_m: np.ndarray) -> float:
    """
    计算两个位置之间的欧几里得距离（模长）

    参数:
        pos1_m: 第一个位置 [x, y, z]，单位 m
        pos2_m: 第二个位置 [x, y, z]，单位 m

    返回:
        位置距离（单位：m）
    """
    distance = np.linalg.norm(pos2_m - pos1_m)
    return distance
