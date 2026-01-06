#! /usr/bin/env python3
# coding: utf-8

import copy
import time
import numpy as np
from typing import Tuple, Union
from kuavo_humanoid_sdk.common.logger import SDKLogger
import rospy
from kuavo_humanoid_sdk.msg.kuavo_msgs.srv import RepublishTFs
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import TFArray
import tf.transformations as tf_trans
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import sensorsData, lejuClawState, gaitTimeName, dexhandTouchState
from geometry_msgs.msg import TransformStamped
from kuavo_humanoid_sdk.interfaces.data_types import (PoseQuaternion, HomogeneousMatrix)
from kuavo_humanoid_sdk.kuavo.core.ros.param import make_robot_param, EndEffectorType



class KuavoRobotToolsCore:
    """Provides core ROS tools for Kuavo humanoid robot transformations.
    
    Attributes:
        tf_service (rospy.ServiceProxy): Service proxy for tf2_web_republisher
        _transform_cache (dict): Cache for storing recent transforms
    """
    
    def __init__(self):
        """Initializes TF2 web republisher service proxy."""
        if not hasattr(self, '_initialized'):
            try:
                # 初始化TF2 web republisher服务
                rospy.wait_for_service('/republish_tfs', timeout=5.0)
                self.tf_service = rospy.ServiceProxy('/republish_tfs', RepublishTFs)
                self._transform_cache = {}
                self._initialized = True
            except Exception as e:
                SDKLogger.error(f"Failed to initialize kuavo_tf2_web_republisher: {str(e)}")
                SDKLogger.error(f"kuavo_tf2_web_republisher 节点未运行")
                SDKLogger.error("请运行 `cd <kuavo_ros_application> && source devel/setup.bash && rosrun kuavo_tf2_web_republisher kuavo_tf2_web_republisher` 启动 kuavo_tf2_web_republisher 节点")
                raise

    def _get_tf_tree_transform(self, target_frame: str, source_frame: str, 
                     time=0.0, timeout=5.0,
                     return_type: str = "pose_quaternion") -> Union[PoseQuaternion, HomogeneousMatrix, None]:
        """Gets transform between coordinate frames using tf2_web_republisher.
        
        Args:
            target_frame (str): Target coordinate frame name
            source_frame (str): Source coordinate frame name
            time (rospy.Time, optional): Time of transform. Defaults to latest.
            timeout (float, optional): Wait timeout in seconds. Defaults to 5.0.
            return_type (str, optional): Return data format. Options: 
                "pose_quaternion", "homogeneous". Defaults to "pose_quaternion".
        
        Returns:
            Union[PoseQuaternion, HomogeneousMatrix, None]: Requested transform data
                or None if failed
        """
        try:

            # #####################################
            # 坐标系转换映射
            # 仅在轮臂 (robot_type == 1) 时启用映射；双足等其他形态保持原坐标系不变
            robot_type = rospy.get_param("/robot_type", 0)
            is_wheel_arm = (robot_type == 1)

            frame_mapping = {}
            if is_wheel_arm:
                frame_mapping = {
                    "base_link": "waist_yaw_link",      # 人形中的 base_link 对应轮臂的 waist_yaw_link,获取轮臂躯干位置时使用 waist_yaw_link 坐标系
                    "odom": "base_link",                # 轮臂获取底盘位置时使用 base_link 坐标系
                }

                # 转换坐标系名称
                if target_frame in frame_mapping:
                    mapped_target_frame = frame_mapping[target_frame]
                    SDKLogger.debug(f"🔵 目标坐标系映射: {target_frame} -> {mapped_target_frame}")
                    target_frame = mapped_target_frame

                if source_frame in frame_mapping:
                    mapped_source_frame = frame_mapping[source_frame]
                    SDKLogger.debug(f"🟢 源坐标系映射: {source_frame} -> {mapped_source_frame}")
                    source_frame = mapped_source_frame
            
            # #######################################
            # 调用服务
            response = self.tf_service(
                source_frames=[source_frame],
                target_frame=target_frame,
                angular_thres=0.01,  # 角度阈值
                trans_thres=0.01,    # 平移阈值
                rate=10.0,           # 更新频率
                timeout=rospy.Duration(timeout)
            )

            if response.status == -1:
                SDKLogger.error(f"{source_frame} or {target_frame} not exist")
                return None
            
            # 检查话题是否发布
            published_topics = rospy.get_published_topics()
            if not any(topic_tuple[0] == response.topic_name for topic_tuple in published_topics):
                SDKLogger.error(f"Topic {response.topic_name} not published")
                return None
                
            # 创建订阅者
            transform_received = False
            transform_data = None
            
            def transform_callback(msg):
                nonlocal transform_received, transform_data
                transform_received = True
                transform_data = msg
                
            sub = rospy.Subscriber(response.topic_name, TFArray, transform_callback)
            
            # 等待接收数据
            start_time = rospy.Time.now()
            while not transform_received or (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.sleep(0.1)
                
            # 取消订阅
            sub.unregister()
                        
            if not transform_received:
                SDKLogger.error("No transform data received")
                return None
                
            # 从TFArray中获取对应的变换
            for tf_msg in transform_data.transforms:
                if tf_msg.header.frame_id == target_frame and tf_msg.child_frame_id == source_frame:
                    return self._parse_transform(tf_msg.transform, return_type)
                    
            SDKLogger.error(f"No matching transform found in TFArray")
            return None
            
        except rospy.ServiceException as e:
            SDKLogger.error(f"Service call failed: {str(e)}")
            return None
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