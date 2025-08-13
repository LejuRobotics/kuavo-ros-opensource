#! /usr/bin/env python3
# coding: utf-8

import copy
import time
import numpy as np
from typing import Tuple, Union
from kuavo_humanoid_sdk.common.logger import SDKLogger
from transforms3d import euler, quaternions
import roslibpy
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK
from kuavo_humanoid_sdk.interfaces.data_types import (PoseQuaternion, HomogeneousMatrix)
from kuavo_humanoid_sdk.kuavo.core.ros.param import make_robot_param, EndEffectorType


class KuavoRobotToolsCoreWebsocket:
    """Provides core ROS tools for Kuavo humanoid robot transformations.
    
    Attributes:
        tf_service (roslibpy.Service): Service proxy for tf2_web_republisher
        _transform_cache (dict): Cache for storing recent transforms
    """
    
    def __init__(self):
        """Initializes TF2 web republisher service proxy."""
        if not hasattr(self, '_initialized'):
            try:
                # Initialize WebSocket connection
                self.websocket = WebSocketKuavoSDK()
                
                # Initialize TF2 web republisher service
                self.tf_service = roslibpy.Service(
                    self.websocket.client,
                    '/republish_tfs',
                    'kuavo_msgs/RepublishTFs'
                )
                self._transform_cache = {}
                self._initialized = True
            except Exception as e:
                SDKLogger.error(f"Failed to initialize kuavo_tf2_web_republisher: {str(e)}")
                SDKLogger.error(f"kuavo_tf2_web_republisher 节点未运行")
                SDKLogger.error("请运行 `cd <kuavo_ros_application> && source devel/setup.bash && rosrun kuavo_tf2_web_republisher kuavo_tf2_web_republisher` 启动 kuavo_tf2_web_republisher 节点")
                raise

    def _get_tf_tree_transform(self, target_frame: str, source_frame: str, 
                     time_=0.0, timeout=5.0,
                     return_type: str = "pose_quaternion") -> Union[PoseQuaternion, HomogeneousMatrix, None]:
        """Gets transform between coordinate frames using tf2_web_republisher.
        
        Args:
            target_frame (str): Target coordinate frame name
            source_frame (str): Source coordinate frame name
            time (float, optional): Time of transform. Defaults to 0.0.
            timeout (float, optional): Wait timeout in seconds. Defaults to 5.0.
            return_type (str, optional): Return data format. Options: 
                "pose_quaternion", "homogeneous". Defaults to "pose_quaternion".
        
        Returns:
            Union[PoseQuaternion, HomogeneousMatrix, None]: Requested transform data
                or None if failed
        """
        try:
                       
            # 调用服务
            request = {
                'source_frames': [source_frame],
                'target_frame': target_frame,
                'angular_thres': 0.01,
                'trans_thres': 0.01,
                'rate': 10.0,
                'timeout': {'secs': int(timeout), 'nsecs': int((timeout % 1) * 1e9)}
            }
            
            response = self.tf_service.call(request)
            if response['status'] == -1:
                SDKLogger.error(f"{source_frame} or {target_frame} not exist")
                return None
            
            # 检查话题是否发布
            topic_list = self.websocket.client.get_topics()
            if response['topic_name'] not in topic_list:
                SDKLogger.error(f"Topic {response['topic_name']} not published")
                return None
                
            # 创建订阅者
            transform_received = False
            transform_data = None
            
            def transform_callback(msg):
                nonlocal transform_received, transform_data
                transform_received = True
                transform_data = msg
                
            sub = roslibpy.Topic(self.websocket.client, response['topic_name'], 'kuavo_msgs/TFArray')
            sub.subscribe(transform_callback)
            
            # 等待接收数据
            start_time = time.time()
            while not transform_received and (time.time() - start_time) < timeout:
                time.sleep(0.1)
                
            # 取消订阅
            sub.unsubscribe()
            
            if not transform_received:
                SDKLogger.error("No transform data received")
                return None
                
            # 从TFArray中获取对应的变换
            for tf_msg in transform_data['transforms']:
                if tf_msg['header']['frame_id'] == target_frame and tf_msg['child_frame_id'] == source_frame:
                    return self._parse_transform(tf_msg['transform'], return_type)
                    
            SDKLogger.error(f"No matching transform found in TFArray")
            return None
            
        except Exception as e:
            SDKLogger.error(f"Transform error: {str(e)}")
            return None

    def _parse_transform(self, transform, return_type: str) -> Union[PoseQuaternion, HomogeneousMatrix, None]:
        """Parses transform data to specified format.
        
        Args:
            transform (dict): Input transform data
            return_type (str): Output format type. Valid values: 
                "pose_quaternion", "homogeneous"
        
        Returns:
            Union[PoseQuaternion, HomogeneousMatrix, None]: Parsed transform data
                in requested format, or None if invalid input
        """
        if return_type == "pose_quaternion":
            return PoseQuaternion(
                position=(transform['translation']['x'], 
                         transform['translation']['y'],
                         transform['translation']['z']),
                orientation=(transform['rotation']['x'],
                            transform['rotation']['y'],
                            transform['rotation']['z'],
                            transform['rotation']['w'])
            )
        elif return_type == "homogeneous":
            return HomogeneousMatrix(
                matrix=self._transform_to_homogeneous(transform)
            )
        else:
            SDKLogger.warn(f"Invalid return_type: {return_type}, using default(pose_quaternion)")
            return self._parse_transform(transform, "pose_quaternion")

    def _transform_to_homogeneous(self, transform) -> np.ndarray:
        """Converts transform dict to homogeneous matrix.
        
        Args:
            transform (dict): Input transform message
        
        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix (numpy.float32)
        """
        # 四元数转旋转矩阵
        rotation = [
            transform['rotation']['x'],
            transform['rotation']['y'],
            transform['rotation']['z'],
            transform['rotation']['w']
        ]
        
        # Convert quaternion to rotation matrix using transforms3d
        rot_matrix = np.eye(4)
        rot_matrix[:3,:3] = quaternions.quat2mat([rotation[3], rotation[0], rotation[1], rotation[2]])

        # 设置平移分量
        translation = [
            transform['translation']['x'],
            transform['translation']['y'],
            transform['translation']['z']
        ]
        rot_matrix[:3, 3] = translation
        
        return rot_matrix.astype(np.float32)  # 确保矩阵数据类型一致

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