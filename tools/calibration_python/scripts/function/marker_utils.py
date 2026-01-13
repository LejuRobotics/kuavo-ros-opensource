#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Marker可视化工具函数：
- 初始化marker发布器和transformer
- 发布位姿marker到rviz
- 支持自定义topic名称和颜色
- 支持自定义marker命名空间（namespace）
"""

import os
import rospy
import rospkg
import json
from visualization_msgs.msg import Marker

# 默认颜色常量
DEFAULT_COLOR_RED = [1.0, 0.0, 0.0, 0.9]
DEFAULT_COLOR_BLUE = [0.0, 0.0, 1.0, 0.9]
DEFAULT_COLOR_GREEN = [0.0, 1.0, 0.0, 0.9]
DEFAULT_COLOR_CYAN = [0.0, 1.0, 1.0, 0.9]
DEFAULT_COLOR_YELLOW = [1.0, 1.0, 0.0, 0.9]
DEFAULT_COLOR_MAGENTA = [1.0, 0.0, 1.0, 0.9]
DEFAULT_COLOR_WHITE = [1.0, 1.0, 1.0, 0.9]

# 默认marker命名空间（当外部未提供namespace时使用）
DEFAULT_MARKER_NAMESPACE = "arm_marker"


def publish_marker(quest3_arm_info_transformer, marker_pub, pos, quat, arm_side, 
                   rgba=None, marker_id=None, point_idx=None, namespace=None):
    """
    发布marker到rviz
    
    Args:
        quest3_arm_info_transformer: Quest3ArmInfoTransformer 实例
        marker_pub: marker发布器
        pos: 位置 (3,)
        quat: 四元数 (4,) [x, y, z, w]
        arm_side: 手臂侧，"left" 或 "right"
        rgba: 颜色 [r, g, b, a]，如果为None则使用默认红色
        marker_id: marker_id，如果为None则根据arm_side自动设置（left=0, right=1）
        point_idx: 点索引（用于日志，不影响marker_id）
        namespace: marker的命名空间（ns字段），用于在Rviz中组织和区分marker。
                   如果为None，则使用默认命名空间 DEFAULT_MARKER_NAMESPACE
    """
    if quest3_arm_info_transformer is None or marker_pub is None:
        return
    
    try:
        side_str = "Left" if arm_side == "left" else "Right"
        
        # 设置颜色，默认使用红色
        if rgba is None:
            rgba = DEFAULT_COLOR_RED
        
        # 设置marker_id，默认根据arm_side设置
        if marker_id is None:
            marker_id = 0 if arm_side == "left" else 1
        
        marker = quest3_arm_info_transformer.construct_marker(
            pos, quat, rgba=rgba, side=side_str, marker_id=marker_id
        )
        
        if marker is not None:
            # 设置marker的命名空间：优先使用外部传入的，否则使用默认值
            marker.ns = namespace if namespace is not None else DEFAULT_MARKER_NAMESPACE
            
            marker.lifetime = rospy.Duration()  # 永不过期
            marker.header.stamp = rospy.Time.now()
            marker_pub.publish(marker)
            
            # 连续发布几次确保rviz能接收到，每次更新时间戳
            for _ in range(3):
                marker.header.stamp = rospy.Time.now()
                marker_pub.publish(marker)
                rospy.sleep(0.02)
    except Exception as e:
        rospy.logwarn(f"发布marker时出错: {e}")


def create_marker_publisher(topic_name, queue_size=10):
    """
    创建单个marker发布器
    
    Args:
        topic_name: ROS topic名称
        queue_size: 队列大小，默认10
    
    Returns:
        rospy.Publisher: marker发布器，如果失败返回None
    """
    try:
        publisher = rospy.Publisher(topic_name, Marker, queue_size=queue_size)
        rospy.sleep(0.1)  # 等待发布器注册
        return publisher
    except Exception as e:
        rospy.logwarn(f"创建marker发布器失败 (topic: {topic_name}): {e}")
        return None


def init_quest3_transformer(Quest3ArmInfoTransformer):
    """
    初始化Quest3ArmInfoTransformer
    
    Args:
        Quest3ArmInfoTransformer: Quest3ArmInfoTransformer 类（可能为None）
    
    Returns:
        Quest3ArmInfoTransformer实例，如果失败返回None
    """
    if Quest3ArmInfoTransformer is None:
        return None
    
    try:
        # 获取模型路径和配置
        rospack = rospkg.RosPack()
        kuavo_assets_path = rospack.get_path("kuavo_assets")
        robot_version = os.environ.get('ROBOT_VERSION', '40')
        if rospy.has_param("robot_version"):
            robot_version = rospy.get_param("robot_version")
        
        model_path = kuavo_assets_path + "/models/biped_s" + str(robot_version)
        model_config_file = kuavo_assets_path + f"/config/kuavo_v{robot_version}/kuavo.json"
        
        with open(model_config_file, 'r') as f:
            model_config = json.load(f)
        eef_visual_stl_files = model_config["eef_visual_stl_files"]
        
        transformer = Quest3ArmInfoTransformer(model_path, eef_visual_stl_files=eef_visual_stl_files)
        rospy.loginfo("Quest3ArmInfoTransformer初始化成功")
        return transformer
    except Exception as e:
        rospy.logwarn(f"Quest3ArmInfoTransformer初始化失败: {e}")
        return None


def init_marker_publishers(Quest3ArmInfoTransformer, topic_names=None):
    """
    初始化marker发布器和transformer
    
    Args:
        Quest3ArmInfoTransformer: Quest3ArmInfoTransformer 类（可能为None）
        topic_names: 可选的topic名称字典，格式如下：
            {
                "target_left": "/custom_target_marker_left",  # 默认: "/arm_marker_left"
                "target_right": "/custom_target_marker_right",  # 默认: "/arm_marker_right"
                "result_param1_left": "/custom_result_param1_marker_left",  # 默认: "/arm_result_param1_marker_left"
                "result_param1_right": "/custom_result_param1_marker_right",  # 默认: "/arm_result_param1_marker_right"
                "result_param2_left": "/custom_result_param2_marker_left",  # 默认: "/arm_result_param2_marker_left"
                "result_param2_right": "/custom_result_param2_marker_right",  # 默认: "/arm_result_param2_marker_right"
            }
            如果某个key不存在，则使用默认值
    
    Returns:
        tuple: (quest3_arm_info_transformer, 
                marker_pub_target_left, marker_pub_target_right,
                marker_pub_result_param1_left, marker_pub_result_param1_right,
                marker_pub_result_param2_left, marker_pub_result_param2_right)
                如果初始化失败，返回 (None, None, None, None, None, None, None)
    """
    # 默认topic名称
    default_topic_names = {
        "target_left": "/arm_marker_left",
        "target_right": "/arm_marker_right",
        "result_param1_left": "/arm_result_param1_marker_left",
        "result_param1_right": "/arm_result_param1_marker_right",
        "result_param2_left": "/arm_result_param2_marker_left",
        "result_param2_right": "/arm_result_param2_marker_right",
    }
    
    # 合并用户提供的topic名称和默认值
    if topic_names is None:
        topic_names = {}
    final_topic_names = {**default_topic_names, **topic_names}
    
    quest3_arm_info_transformer = None
    marker_pub_target_left = None
    marker_pub_target_right = None
    marker_pub_result_param1_left = None
    marker_pub_result_param1_right = None
    marker_pub_result_param2_left = None
    marker_pub_result_param2_right = None
    
    if Quest3ArmInfoTransformer is not None:
        try:
            quest3_arm_info_transformer = init_quest3_transformer(Quest3ArmInfoTransformer)
            
            if quest3_arm_info_transformer is not None:
                # 创建marker发布器
                marker_pub_target_left = create_marker_publisher(final_topic_names["target_left"])
                marker_pub_target_right = create_marker_publisher(final_topic_names["target_right"])
                marker_pub_result_param1_left = create_marker_publisher(final_topic_names["result_param1_left"])
                marker_pub_result_param1_right = create_marker_publisher(final_topic_names["result_param1_right"])
                marker_pub_result_param2_left = create_marker_publisher(final_topic_names["result_param2_left"])
                marker_pub_result_param2_right = create_marker_publisher(final_topic_names["result_param2_right"])
                
                COLOR_PURPLE = "\033[95m"
                COLOR_RESET = "\033[0m"
                rospy.loginfo(f"{COLOR_PURPLE}Marker发布器初始化完成{COLOR_RESET}")
                rospy.loginfo(f"   - 左手臂话题: {final_topic_names['target_left']}")
                rospy.loginfo(f"   - 右手臂话题: {final_topic_names['target_right']}")
                rospy.loginfo(f"   - param1左手臂话题: {final_topic_names['result_param1_left']}")
                rospy.loginfo(f"   - param1右手臂话题: {final_topic_names['result_param1_right']}")
                rospy.loginfo(f"   - param2左手臂话题: {final_topic_names['result_param2_left']}")
                rospy.loginfo(f"   - param2右手臂话题: {final_topic_names['result_param2_right']}")
                rospy.sleep(0.5)  # 等待发布器注册
        except Exception as e:
            rospy.logwarn(f"Marker发布器初始化失败: {e}")
            rospy.logwarn("   继续测试，但不显示可视化marker")
    else:
        rospy.logwarn("Quest3ArmInfoTransformer不可用，跳过marker初始化")
    
    return (quest3_arm_info_transformer, marker_pub_target_left, marker_pub_target_right,
            marker_pub_result_param1_left, marker_pub_result_param1_right,
            marker_pub_result_param2_left, marker_pub_result_param2_right)


def publish_trajectory_markers_both_arms(
    quest3_arm_info_transformer,
    marker_pub_target_left, marker_pub_target_right,
    marker_pub_result_left, marker_pub_result_right,
    expected_pos_left, expected_pos_right,
    expected_quat_left, expected_quat_right,
    result_pos_left=None, result_pos_right=None,
    result_quat_left=None, result_quat_right=None,
    target_color=None, result_color=None,
    point_idx=None, target_namespace=None, result_namespace=None
):
    """
    发布双臂轨迹marker（简化版，只负责发布，不包含业务逻辑）
    
    Args:
        quest3_arm_info_transformer: Quest3ArmInfoTransformer 实例
        marker_pub_target_left: 左手期望位姿marker发布器
        marker_pub_target_right: 右手期望位姿marker发布器
        marker_pub_result_left: 左手实际位姿marker发布器（可选）
        marker_pub_result_right: 右手实际位姿marker发布器（可选）
        expected_pos_left: 左手期望位置（米）
        expected_pos_right: 右手期望位置（米）
        expected_quat_left: 左手期望四元数 [x, y, z, w]
        expected_quat_right: 右手期望四元数 [x, y, z, w]
        result_pos_left: 左手实际位置（米，可选）
        result_pos_right: 右手实际位置（米，可选）
        result_quat_left: 左手实际四元数（可选）
        result_quat_right: 右手实际四元数（可选）
        target_color: 期望位姿颜色，默认红色
        result_color: 实际位姿颜色，默认蓝色
        point_idx: 点索引（用于日志）
        target_namespace: 期望位姿marker的命名空间，如果为None则使用默认值
        result_namespace: 实际位姿marker的命名空间，如果为None则使用默认值
    """
    if quest3_arm_info_transformer is None:
        return
    
    try:
        # 发布期望位姿
        if marker_pub_target_left is not None and expected_pos_left is not None:
            publish_marker(quest3_arm_info_transformer, marker_pub_target_left,
                         expected_pos_left, expected_quat_left, "left", 
                         rgba=target_color, point_idx=point_idx, namespace=target_namespace)
        if marker_pub_target_right is not None and expected_pos_right is not None:
            publish_marker(quest3_arm_info_transformer, marker_pub_target_right,
                         expected_pos_right, expected_quat_right, "right",
                         rgba=target_color, point_idx=point_idx, namespace=target_namespace)
        
        # 发布实际位姿（如果提供）
        if marker_pub_result_left is not None and result_pos_left is not None:
            publish_marker(quest3_arm_info_transformer, marker_pub_result_left,
                         result_pos_left, result_quat_left, "left",
                         rgba=result_color, point_idx=point_idx, namespace=result_namespace)
        if marker_pub_result_right is not None and result_pos_right is not None:
            publish_marker(quest3_arm_info_transformer, marker_pub_result_right,
                         result_pos_right, result_quat_right, "right",
                         rgba=result_color, point_idx=point_idx, namespace=result_namespace)
    except Exception as e:
        rospy.logwarn(f"发布轨迹marker时出错: {e}")


def publish_trajectory_markers_single_arm(
    quest3_arm_info_transformer,
    marker_pub_target, marker_pub_result,
    expected_pos, expected_quat,
    arm_side,
    result_pos=None, result_quat=None,
    target_color=None, result_color=None,
    point_idx=None, target_namespace=None, result_namespace=None
):
    """
    发布单臂轨迹marker（简化版，只负责发布，不包含业务逻辑）
    
    Args:
        quest3_arm_info_transformer: Quest3ArmInfoTransformer 实例
        marker_pub_target: 期望位姿marker发布器
        marker_pub_result: 实际位姿marker发布器（可选）
        expected_pos: 期望位置（米）
        expected_quat: 期望四元数 [x, y, z, w]
        arm_side: 手臂侧，"left" 或 "right"
        result_pos: 实际位置（米，可选）
        result_quat: 实际四元数（可选）
        target_color: 期望位姿颜色，默认红色
        result_color: 实际位姿颜色，默认蓝色
        point_idx: 点索引（用于日志）
        target_namespace: 期望位姿marker的命名空间，如果为None则使用默认值
        result_namespace: 实际位姿marker的命名空间，如果为None则使用默认值
    """
    if quest3_arm_info_transformer is None or expected_pos is None:
        return
    
    try:
        # 发布期望位姿
        if marker_pub_target is not None:
            publish_marker(quest3_arm_info_transformer, marker_pub_target,
                         expected_pos, expected_quat, arm_side,
                         rgba=target_color, point_idx=point_idx, namespace=target_namespace)
        
        # 发布实际位姿（如果提供）
        if marker_pub_result is not None and result_pos is not None:
            publish_marker(quest3_arm_info_transformer, marker_pub_result,
                         result_pos, result_quat, arm_side,
                         rgba=result_color, point_idx=point_idx, namespace=result_namespace)
    except Exception as e:
        rospy.logwarn(f"发布轨迹marker时出错: {e}")
