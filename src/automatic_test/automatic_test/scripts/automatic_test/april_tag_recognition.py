#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import tf
import numpy as np
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, TriggerRequest, SetBool, SetBoolRequest
from gazebo_msgs.srv import GetModelState
from kuavo_msgs.msg import robotHeadMotionData, AprilTagDetectionArray
from .apriltag_position_controller import AprilTagPositionController

class AprilTagRecognitionInterface:
    """
    AprilTag识别接口类
    
    提供以下功能：
    1. 机器人停止运动
    2. 机器人低头动作（低头20度，识别完成后抬头20度）
    3. AprilTag识别（订阅/robot_tag_info话题）
    4. 识别结果处理
    5. 坐标系转换和位置偏差评估
    6. 轨迹终点订阅和AprilTag/灯光移动
    """
    
    def __init__(self):
        # AprilTag识别数据存储
        self.april_tag_detections = []  # 存储检测到的标签数据
        self.detection_count = 0        # 检测到的帧数
        self.last_detection_time = None # 最后一次检测时间
        
        # 轨迹终点相关
        self.trajectory_endpoint = (0.0, 0.0)  # 存储轨迹终点坐标 (x, y)
        self.endpoint_x = 0.0
        self.endpoint_y = 0.0
        
        # 参数配置
        base_ns = '/april_tag_recognition'
        self.recognition_timeout = rospy.get_param(f'{base_ns}/recognition_timeout', 10.0)  # 识别超时时间
        self.head_down_duration = rospy.get_param(f'{base_ns}/head_down_duration', 3.0)    # 低头持续时间
        self.recognition_retries = rospy.get_param(f'{base_ns}/recognition_retries', 3)    # 识别重试次数
        
        # 头部控制参数
        self.head_down_pitch = rospy.get_param(f'{base_ns}/head_down_pitch', 20)  # 低头角度（度）
        self.head_up_pitch = rospy.get_param(f'{base_ns}/head_up_pitch', 0)      # 抬头角度（度）
        self.head_motion_timeout = rospy.get_param(f'{base_ns}/head_motion_timeout', 3.0)  # 头部动作超时时间
        
        # AprilTag识别参数
        self.required_frames = rospy.get_param(f'{base_ns}/required_frames', 5)    # 需要的帧数
        self.frame_timeout = rospy.get_param(f'{base_ns}/frame_timeout', 1.0)     # 帧获取超时时间（秒）
        
        # 标签真值位置（在odom坐标系下）
        self.tag_ground_truth = rospy.get_param(f'{base_ns}/tag_ground_truth', {})
        
        # 位置评估参数
        self.max_position_error = rospy.get_param(f'{base_ns}/position_evaluation/max_position_error', 0.1)
        self.min_detection_frames = rospy.get_param(f'{base_ns}/position_evaluation/min_detection_frames', 3)
        self.position_std_threshold = rospy.get_param(f'{base_ns}/position_evaluation/position_std_threshold', 0.05)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_pose_pub = rospy.Publisher('/cmd_pose', Twist, queue_size=10)
        self.head_motion_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
        self.april_tag_sub = rospy.Subscriber('/robot_tag_info', AprilTagDetectionArray, self.april_tag_callback)
        self.realtime_path_sub = rospy.Subscriber('trace_path/path', Path, self.groundtruth_path_callback)
        
        self.tf_listener = tf.TransformListener()

        self.apriltag_controller = AprilTagPositionController()
        
        rospy.loginfo("AprilTag识别接口初始化完成")
    
    def _get_detection_position(self, detection):
        """
        从检测结果中获取位置信息的辅助方法
        
        Args:
            detection: AprilTag检测结果对象
            
        Returns:
            tuple: (x, y, z) 位置坐标，如果获取失败返回 None
        """
        try:
            # 检查是否是 PoseWithCovarianceStamped 类型
            if hasattr(detection.pose, 'pose') and hasattr(detection.pose.pose, 'pose'):
                if hasattr(detection.pose.pose.pose, 'position'):
                    pos = detection.pose.pose.pose.position
                    return (pos.x, pos.y, pos.z)
            
            # 检查是否是 Pose 类型
            elif hasattr(detection.pose, 'position'):
                pos = detection.pose.position
                return (pos.x, pos.y, pos.z)
            
            # 尝试其他可能的结构
            elif hasattr(detection.pose, 'x') and hasattr(detection.pose, 'y') and hasattr(detection.pose, 'z'):
                return (detection.pose.x, detection.pose.y, detection.pose.z)
            
            else:
                rospy.logwarn(f"无法获取位置信息，pose类型: {type(detection.pose)}")
                return None
                
        except Exception as e:
            rospy.logerr(f"获取位置信息时出错: {e}")
            return None
    
    def april_tag_callback(self, msg):
        """
        AprilTag识别数据回调函数
        
        Args:
            msg (AprilTagDetectionArray): AprilTag检测数据
        """
        try:
            current_time = rospy.Time.now()
            
            # 更新检测计数和时间
            self.detection_count += 1
            self.last_detection_time = current_time
            
            # 存储检测数据
            detection_data = {
                'timestamp': current_time,
                'detections': msg.detections,
                'frame_count': self.detection_count
            }
            self.april_tag_detections.append(detection_data)
            
            # rospy.loginfo(f"接收到AprilTag数据帧 {self.detection_count}: 检测到 {len(msg.detections)} 个标签")
            
            # 记录每个标签的详细信息
            for i, detection in enumerate(msg.detections):
                position = self._get_detection_position(detection)
                if position:
                    x, y, z = position
                    # rospy.loginfo(f"  标签 {i+1}: ID={detection.id}, 位置=({x:.3f}, {y:.3f}, {z:.3f})")
                else:
                    rospy.logwarn(f"  标签 {i+1}: ID={detection.id}, 无法获取位置信息")
                
        except Exception as e:
            rospy.logerr(f"处理AprilTag数据时发生错误: {e}")
    
    def groundtruth_path_callback(self, msg):
        """实时轨迹回调函数，用于获取机器人当前位置"""
        try:
            if len(msg.poses) > 0:
                last_pose = msg.poses[-1]
                # print(f"last_pose: {last_pose}")
                # 直接设置轨迹终点坐标 (x, y)
                self.trajectory_endpoint = (last_pose.pose.position.x, last_pose.pose.position.y)
                # rospy.loginfo(f"✅ 接收到轨迹数据，包含 {len(msg.poses)} 个点，帧ID: {msg.header.frame_id}")
                # rospy.loginfo(f"   最新轨迹点: x={self.trajectory_endpoint[0]:.3f}, y={self.trajectory_endpoint[1]:.3f}")
                # rospy.loginfo(f"   轨迹终点已设置: {self.trajectory_endpoint}")
            else:
                rospy.logwarn("接收到空轨迹数据")
        except Exception as e:
            rospy.logwarn(f"处理轨迹数据时出错: {e}")
    
    def move_apriltags_and_light(self):
        """
        根据轨迹终点移动所有AprilTag和点光源
        
        Returns:
            bool: 是否成功移动
        """
        try:
            if isinstance(self.trajectory_endpoint, (list, tuple)) and len(self.trajectory_endpoint) >= 2:
                self.endpoint_x, self.endpoint_y = self.trajectory_endpoint[0], self.trajectory_endpoint[1]
            else:
                rospy.logerr(f"❌ 轨迹终点格式错误: {self.trajectory_endpoint}")
                return False
                
            rospy.loginfo(f"🚀 开始根据轨迹终点 ({self.endpoint_x:.3f}, {self.endpoint_y:.3f}) 移动AprilTag和灯光...")
            
            # 检查是否有标签真值数据
            if not self.tag_ground_truth:
                rospy.logwarn("❌ 没有标签真值数据，无法移动AprilTag")
                rospy.logwarn(f"当前标签真值数据: {self.tag_ground_truth}")
                return False
            
            rospy.loginfo(f"📋 标签真值数据: {self.tag_ground_truth}")
            
            # 移动AprilTag
            rospy.loginfo(f"  移动AprilTag到终点位置: ({self.endpoint_x:.3f}, {self.endpoint_y:.3f}, 0.0005)")
            tag_success = self.apriltag_controller.move_apriltag(self.endpoint_x, self.endpoint_y, 0.0005)
            
            # 移动点光源（移动到终点位置）
            rospy.loginfo(f"  移动点光源到终点位置: ({self.endpoint_x:.3f}, {self.endpoint_y:.3f}, 1.0)")
            light_success = self.apriltag_controller.move_point_light(self.endpoint_x, self.endpoint_y, 1.0)
            
            rospy.loginfo(f"🎯 AprilTag和灯光移动完成:")
            rospy.loginfo(f"  成功移动标签: {'✅ 成功' if tag_success else '❌ 失败'}")
            rospy.loginfo(f"  点光源移动: {'✅ 成功' if light_success else '❌ 失败'}")
            
            overall_success = tag_success and light_success
            if overall_success:
                rospy.loginfo("✅ 所有AprilTag和灯光移动成功")
            else:
                rospy.logwarn("⚠️ 部分AprilTag或灯光移动失败")
            
            return overall_success
            
        except Exception as e:
            rospy.logerr(f"移动AprilTag和灯光时发生错误: {e}")
            return False
    
    def transform_to_odom_frame(self, tag_id, stamp=None):
        """
        直接从TF树查询 tag_x 在 odom 下的位姿。
        Args:
            tag_id (int): 标签ID
            stamp (rospy.Time|None): 查询时间（可使用帧时间戳）；None则使用最新
        Returns:
            tuple(np.array, np.array) | (None, None): 平移(x,y,z), 四元数(x,y,z,w)
        """
        try:
            child = f'tag_{tag_id}'
            if stamp is None:
                stamp = rospy.Time(0)
            self.tf_listener.waitForTransform('odom', child, stamp, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('odom', child, stamp)
            return np.array(trans), np.array(rot)
        except Exception as e:
            rospy.logwarn(f"查询TF失败(odom->{child}): {e}")
            return None, None
    
    def get_robot_position_in_tag_frame(self, tag_id, stamp=None):
        """
        获取机器人在tag_x坐标系下的位置
        
        Args:
            tag_id (int): 标签ID
            stamp (rospy.Time|None): 查询时间（可使用帧时间戳）；None则使用最新
            
        Returns:
            tuple(np.array, np.array) | (None, None): 平移(x,y,z), 四元数(x,y,z,w)
        """
        try:
            tag_frame = f'tag_{tag_id}'
            if stamp is None:
                stamp = rospy.Time(0)
            
            # 查询机器人在tag_x坐标系下的位置
            self.tf_listener.waitForTransform(tag_frame, 'base_link', stamp, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(tag_frame, 'base_link', stamp)
            return np.array(trans), np.array(rot)
            
        except Exception as e:
            rospy.logwarn(f"查询TF失败({tag_frame}->base_link): {e}")
            return None, None
    
    def evaluate_position_accuracy(self, tag_id, detection_frames, ground_truth):
        """
        评估标签位置精度（只比较x和y轴数据）
        
        Args:
            tag_id (int): 标签ID
            detection_frames (list): 检测帧数据列表
            ground_truth (list): 真值位置 [x, y]（只保留x和y）
            
        Returns:
            dict: 评估结果
        """
        if not detection_frames:
            return {'valid': False, 'error': '没有检测数据'}
        
        try:

            # 调试函数：输出tag_x在odom的位置和base_link在odom的位置
            # self._debug_position_comparison(tag_id)
            
            # 提取位置数据（只保留x和y）
            positions = []
            
            # 仅在该帧确实检测到该id时，按该帧时间戳查询TF
            for frame_data in detection_frames:
                has_id = any(self._normalize_tag_id(d.id) == tag_id for d in frame_data['detections'])
                if not has_id:
                    continue
                    
                stamp = frame_data.get('timestamp', rospy.Time(0))
                # 获取机器人在tag_x坐标系下的位置
                robot_trans, robot_rot = self.get_robot_position_in_tag_frame(tag_id, stamp)
                
                if robot_trans is not None:
                    # 只保留x和y坐标
                    positions.append(robot_trans[:2])  # 只取前两个元素 [x, y]
            
            if not positions:
                return {'valid': False, 'error': f'标签 {tag_id} 没有有效的位置数据'}
            
            # 计算统计信息（只针对x和y）
            positions = np.array(positions)  # shape: (n_frames, 2)
            # 稳健评估：按到中位数的距离进行90%内点裁剪
            median_xy = np.median(positions, axis=0)
            distances = np.linalg.norm(positions - median_xy, axis=1)
            threshold = np.quantile(distances, 0.9)  # 保留最接近的90%
            inlier_mask = distances <= threshold
            robust_positions = positions[inlier_mask]
            if robust_positions.shape[0] == 0:
                return {'valid': False, 'error': f'标签 {tag_id} 稳健裁剪后无有效样本'}
            avg_position = np.mean(robust_positions, axis=0)  # [avg_x, avg_y]
            position_std = np.std(robust_positions, axis=0)   # [std_x, std_y]
            
            # 计算与真值的偏差（只比较x和y）
            ground_truth_array = np.array(ground_truth[:2])  # 只取前两个元素 [x, y]
            position_error = np.linalg.norm(avg_position - ground_truth_array)
            rospy.loginfo(f"[评估] tag_{tag_id} 真值(x,y)={ground_truth_array.tolist()} 平均(x,y)={avg_position.tolist()} 误差={position_error:.3f} 使用样本={robust_positions.shape[0]}")
            
            # 评估结果
            evaluation_result = {
                'valid': True,
                'tag_id': tag_id,
                'detection_frames': len(detection_frames),
                'used_frames': int(robust_positions.shape[0]),
                'avg_position': avg_position.tolist(),  # [avg_x, avg_y]
                'position_std': position_std.tolist(),  # [std_x, std_y]
                'ground_truth': ground_truth[:2],      # [x, y]
                'position_error': float(position_error),
                'position_std_max': float(np.max(position_std)),
                'meets_criteria': {
                    'position_error': position_error <= self.max_position_error,
                    'detection_frames': positions.shape[0] >= self.min_detection_frames,
                    'position_std': np.max(position_std) <= self.position_std_threshold
                }
            }
            
            # 判断是否满足所有评估标准
            evaluation_result['overall_pass'] = all(evaluation_result['meets_criteria'].values())
            
            return evaluation_result
            
        except Exception as e:
            rospy.logerr(f"评估标签 {tag_id} 位置精度时发生错误: {e}")
            return {'valid': False, 'error': str(e)}
    
    def _debug_position_comparison(self, tag_id):
        """
        调试函数：输出tag_x在odom的位置和base_link在odom的位置，并计算误差
        
        Args:
            tag_id (int): 标签ID
        """
        try:
            rospy.loginfo(f"🔍 [调试] 开始位置对比分析 - tag_{tag_id}")
            
            # 1. 获取tag_x在odom坐标系下的位置
            tag_trans, tag_rot = self.transform_to_odom_frame(tag_id)
            if tag_trans is not None:
                tag_x_odom = tag_trans[0]
                tag_y_odom = tag_trans[1]
                tag_z_odom = tag_trans[2]
                rospy.loginfo(f"📍 [调试] tag_{tag_id} 在odom坐标系下的位置:")
                rospy.loginfo(f"   x: {tag_x_odom:.6f}")
                rospy.loginfo(f"   y: {tag_y_odom:.6f}")
                rospy.loginfo(f"   z: {tag_z_odom:.6f}")
            else:
                rospy.logwarn(f"❌ [调试] 无法获取tag_{tag_id}在odom坐标系下的位置")
                return
            
            # 2. 获取base_link在odom坐标系下的位置
            try:
                self.tf_listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                (base_trans, base_rot) = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
                base_x_odom = base_trans[0]
                base_y_odom = base_trans[1]
                base_z_odom = base_trans[2]
                
                rospy.loginfo(f"🤖 [调试] base_link在odom坐标系下的位置:")
                rospy.loginfo(f"   x: {base_x_odom:.6f}")
                rospy.loginfo(f"   y: {base_y_odom:.6f}")
                rospy.loginfo(f"   z: {base_z_odom:.6f}")
                
            except Exception as e:
                rospy.logwarn(f"❌ [调试] 无法获取base_link在odom坐标系下的位置: {e}")
                return
            
            # 3. 计算两个位置之间的误差（只考虑x和y）
            position_error_2d = np.sqrt((tag_x_odom - base_x_odom)**2 + (tag_y_odom - base_y_odom)**2)
            position_error_3d = np.sqrt((tag_x_odom - base_x_odom)**2 + (tag_y_odom - base_y_odom)**2 + (tag_z_odom - base_z_odom)**2)
            
            rospy.loginfo(f"📏 [调试] 位置误差分析:")
            rospy.loginfo(f"   tag_{tag_id} <-> base_link 2D误差: {position_error_2d:.6f}")
            rospy.loginfo(f"   tag_{tag_id} <-> base_link 3D误差: {position_error_3d:.6f}")
            rospy.loginfo(f"   X轴误差: {abs(tag_x_odom - base_x_odom):.6f}")
            rospy.loginfo(f"   Y轴误差: {abs(tag_y_odom - base_y_odom):.6f}")
            rospy.loginfo(f"   Z轴误差: {abs(tag_z_odom - base_z_odom):.6f}")
            
            # 4. 显示相对位置关系
            rospy.loginfo(f"🎯 [调试] 相对位置关系:")
            if tag_x_odom > base_x_odom:
                rospy.loginfo(f"   tag_{tag_id} 在base_link的右侧 (+{tag_x_odom - base_x_odom:.6f})")
            else:
                rospy.loginfo(f"   tag_{tag_id} 在base_link的左侧 ({tag_x_odom - base_x_odom:.6f})")
                
            if tag_y_odom > base_y_odom:
                rospy.loginfo(f"   tag_{tag_id} 在base_link的前方 (+{tag_y_odom - base_y_odom:.6f})")
            else:
                rospy.loginfo(f"   tag_{tag_id} 在base_link的后方 ({tag_y_odom - base_y_odom:.6f})")
                
            if tag_z_odom > base_z_odom:
                rospy.loginfo(f"   tag_{tag_id} 在base_link的上方 (+{tag_z_odom - base_z_odom:.6f})")
            else:
                rospy.loginfo(f"   tag_{tag_id} 在base_link的下方 ({tag_z_odom - base_z_odom:.6f})")
            
            rospy.loginfo(f"✅ [调试] 位置对比分析完成")
            
        except Exception as e:
            rospy.logerr(f"❌ [调试] 位置对比分析时发生错误: {e}")
    
    def evaluate_all_tags(self):
        """
        评估所有检测到的标签的位置精度
        
        Returns:
            dict: 所有标签的评估结果
        """
        if not self.april_tag_detections:
            rospy.logwarn("没有AprilTag检测数据可供评估")
            return None
        
        try:
            evaluation_results = {}
            overall_pass = True
            
            # 获取所有检测到的标签ID
            detected_tag_ids = set()
            for frame_data in self.april_tag_detections:
                for detection in frame_data['detections']:
                    detected_tag_ids.add(self._normalize_tag_id(detection.id))
            
            rospy.loginfo(f"检测到的标签ID: {detected_tag_ids}")
            
            # 评估每个标签
            for tag_id_int in detected_tag_ids:
                # 将整数ID转换为字符串，以匹配params.yaml中的键类型
                tag_id_str = str(tag_id_int)
                if tag_id_str in self.tag_ground_truth:
                    # 评估位置精度
                    result = self.evaluate_position_accuracy(
                        tag_id_int, 
                        self.april_tag_detections, 
                        self.tag_ground_truth[tag_id_str]
                    )
                    
                    if result['valid']:
                        evaluation_results[tag_id_int] = result
                        if not result['overall_pass']:
                            overall_pass = False
                    else:
                        rospy.logwarn(f"标签 {tag_id_int} 评估失败: {result['error']}")
                        overall_pass = False
                else:
                    rospy.logwarn(f"标签 {tag_id_int} 没有真值数据，跳过评估")
            
            # 总体评估结果
            final_result = {
                'overall_pass': overall_pass,
                'total_tags': len(detected_tag_ids),
                'evaluated_tags': len(evaluation_results),
                'tag_evaluations': evaluation_results,
                'evaluation_criteria': {
                    'max_position_error': self.max_position_error,
                    'min_detection_frames': self.min_detection_frames,
                    'position_std_threshold': self.position_std_threshold
                }
            }
            return final_result
            
        except Exception as e:
            rospy.logerr(f"评估所有标签时发生错误: {e}")
            return None
    
    def reset_detection_data(self):
        """重置检测数据"""
        self.april_tag_detections = []
        self.detection_count = 0
        self.last_detection_time = None
        rospy.loginfo("AprilTag检测数据已重置")
    
    def wait_for_detection_frames(self, timeout=None):
        """
        等待获取指定数量的检测帧
        
        Args:
            timeout (float): 超时时间，None使用默认值
            
        Returns:
            bool: 是否成功获取足够的帧数
        """
        if timeout is None:
            timeout = self.frame_timeout
            
        rospy.loginfo(f"开始等待AprilTag检测帧，需要 {self.required_frames} 帧，超时时间 {timeout} 秒")
        
        # 重置检测数据
        self.reset_detection_data()
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz检查频率
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            # 检查是否超时
            if (current_time - start_time).to_sec() > timeout:
                rospy.logwarn(f"获取AprilTag检测帧超时，当前帧数: {self.detection_count}/{self.required_frames}")
                return False
            
            # 检查是否获取到足够的帧数
            if self.detection_count >= self.required_frames:
                rospy.loginfo(f"成功获取 {self.detection_count} 帧AprilTag检测数据")
                return True
            
            # 检查最后一帧是否过期（如果超过0.5秒没有新数据，认为数据流中断）
            if (self.last_detection_time is not None and 
                (current_time - self.last_detection_time).to_sec() > 0.5):
                rospy.logwarn("AprilTag数据流中断，等待新数据...")
            
            rate.sleep()
        
        return False
    
    def analyze_detection_data(self):
        """
        分析检测到的AprilTag数据
        
        Returns:
            dict: 分析结果
        """
        if not self.april_tag_detections:
            rospy.logwarn("没有AprilTag检测数据可供分析")
            return None
        
        try:
            analysis_result = {
                'total_frames': len(self.april_tag_detections),
                'detection_summary': {},
                'tag_statistics': {},
                'position_consistency': {}
            }
            
            # 统计每个标签的出现次数和位置信息
            tag_positions = {}  # {tag_id: [(x, y, z, frame_count), ...]}
            
            for frame_data in self.april_tag_detections:
                frame_count = frame_data['frame_count']
                for detection in frame_data['detections']:
                    tag_id = detection.id
                    
                    # 使用辅助方法获取位置信息
                    position = self._get_detection_position(detection)
                    if position is None:
                        rospy.logwarn(f"标签 {tag_id} 无法获取位置信息，跳过")
                        continue
                    
                    x, y, z = position
                    if tag_id not in tag_positions:
                        tag_positions[tag_id] = []
                    
                    tag_positions[tag_id].append((x, y, z, frame_count))
            
            # 分析每个标签的统计信息
            for tag_id, positions in tag_positions.items():
                detection_count = len(positions)
                avg_x = sum(pos[0] for pos in positions) / detection_count
                avg_y = sum(pos[1] for pos in positions) / detection_count
                avg_z = sum(pos[2] for pos in positions) / detection_count
                
                # 计算位置一致性（标准差）
                std_x = (sum((pos[0] - avg_x) ** 2 for pos in positions) / detection_count) ** 0.5
                std_y = (sum((pos[1] - avg_y) ** 2 for pos in positions) / detection_count) ** 0.5
                std_z = (sum((pos[2] - avg_z) ** 2 for pos in positions) / detection_count) ** 0.5
                
                analysis_result['tag_statistics'][tag_id] = {
                    'detection_count': detection_count,
                    'average_position': (avg_x, avg_y, avg_z),
                    'position_std': (std_x, std_y, std_z),
                    'detection_frames': [pos[3] for pos in positions]
                }
                
                # 评估位置一致性
                position_consistency = 'good' if max(std_x, std_y, std_z) < 0.05 else 'poor'
                analysis_result['position_consistency'][tag_id] = position_consistency
            
            # 总体检测摘要
            analysis_result['detection_summary'] = {
                'total_tags_detected': len(tag_positions),
                'most_frequent_tag': max(tag_positions.keys(), key=lambda k: len(tag_positions[k])) if tag_positions else None,
                'detection_quality': 'good' if len(tag_positions) > 0 else 'poor'
            }
            
            rospy.loginfo(f"AprilTag数据分析完成:")
            rospy.loginfo(f"  总帧数: {analysis_result['total_frames']}")
            rospy.loginfo(f"  检测到的标签数: {analysis_result['detection_summary']['total_tags_detected']}")
            rospy.loginfo(f"  最频繁标签: {analysis_result['detection_summary']['most_frequent_tag']}")
            
            return analysis_result
            
        except Exception as e:
            rospy.logerr(f"分析AprilTag检测数据时发生错误: {e}")
            return None
    
    def stop_robot_motion(self):
        """
        停止机器人运动
        
        Returns:
            bool: 是否成功停止
        """
        try:
            rospy.loginfo("停止机器人运动...")
            
            # 发布零速度命令
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0
            stop_cmd.linear.z = 0.0
            stop_cmd.angular.x = 0.0
            stop_cmd.angular.y = 0.0
            stop_cmd.angular.z = 0.0
            
            # 连续发布几次确保停止
            for i in range(5):
                self.cmd_vel_pub.publish(stop_cmd)
                rospy.sleep(0.1)
            
            rospy.loginfo("机器人运动已停止")
            return True
            
        except Exception as e:
            rospy.logerr(f"停止机器人运动失败: {e}")
            return False
    
    def control_head_motion(self, yaw_increment=0, pitch_increment=0, timeout=None):
        """
        控制机器人头部运动
        
        Args:
            yaw_increment (int): Yaw轴增量（度）
            pitch_increment (int): Pitch轴增量（度）
            timeout (float): 动作超时时间，None使用默认值
            
        Returns:
            bool: 是否成功执行头部动作
        """
        if timeout is None:
            timeout = self.head_motion_timeout
            
        try:
            rospy.loginfo(f"执行头部动作: Yaw={yaw_increment}°, Pitch={pitch_increment}°")
            
            # 创建头部控制消息
            head_msg = robotHeadMotionData()
            head_msg.joint_data = [yaw_increment, pitch_increment]  # [yaw, pitch]
            
            # 发布头部控制命令
            self.head_motion_pub.publish(head_msg)
            
            # 等待头部动作完成
            rospy.sleep(timeout)
            
            rospy.loginfo(f"头部动作完成: Yaw={yaw_increment}°, Pitch={pitch_increment}°")
            return True
            
        except Exception as e:
            rospy.logerr(f"执行头部动作失败: {e}")
            return False
    
    def head_down_action(self):
        """
        执行机器人低头动作（低头20度）
        
        Returns:
            bool: 是否成功执行低头动作
        """
        try:
            rospy.loginfo("执行机器人低头动作...")
            
            # 低头20度（pitch轴负向）
            success = self.control_head_motion(
                yaw_increment=0, 
                pitch_increment=self.head_down_pitch,
                timeout=self.head_down_duration
            )
            
            if success:
                rospy.loginfo("机器人低头动作完成")
            else:
                rospy.logerr("机器人低头动作失败")
            
            return success
            
        except Exception as e:
            rospy.logerr(f"执行低头动作失败: {e}")
            return False
    
    def head_up_action(self):
        """
        执行机器人抬头动作（抬头20度回到原位）
        
        Returns:
            bool: 是否成功执行抬头动作
        """
        try:
            rospy.loginfo("执行机器人抬头动作...")
            
            # 抬头20度（pitch轴正向，回到原位）
            success = self.control_head_motion(
                yaw_increment=0, 
                pitch_increment=self.head_up_pitch,
                timeout=self.head_motion_timeout
            )
            
            if success:
                rospy.loginfo("机器人抬头动作完成")
            else:
                rospy.logerr("机器人抬头动作失败")
            
            return success
            
        except Exception as e:
            rospy.logerr(f"执行抬头动作失败: {e}")
            return False
    
    def detect_april_tag(self, timeout=None):
        """
        执行AprilTag识别（订阅/robot_tag_info话题）
        
        Args:
            timeout (float): 识别超时时间，None使用默认值
            
        Returns:
            dict: 识别结果和分析数据
        """
        if timeout is None:
            timeout = self.recognition_timeout
            
        try:
            rospy.loginfo("开始AprilTag识别...")
            
            # 等待获取指定数量的检测帧
            if not self.wait_for_detection_frames(timeout):
                rospy.logwarn("获取AprilTag检测帧超时")
                return None
            
            # 分析检测数据
            analysis_result = self.analyze_detection_data()
            
            if analysis_result and analysis_result['detection_summary']['total_tags_detected'] > 0:
                rospy.loginfo(f"AprilTag识别成功，检测到 {analysis_result['detection_summary']['total_tags_detected']} 个标签")
                return analysis_result
            else:
                rospy.logwarn("AprilTag识别失败，未检测到有效标签")
                return None
            
        except Exception as e:
            rospy.logerr(f"AprilTag识别过程中发生错误: {e}")
            return None
    
    def execute_recognition_workflow(self):
        """
        执行完整的AprilTag识别工作流程
        
        Returns:
            bool: 识别是否成功
        """
        
        try:
            # 步骤1: 停止机器人运动
            if not self.stop_robot_motion():
                rospy.logerr("停止机器人运动失败，终止识别流程")
                return False
            
            # 步骤2: 根据轨迹终点移动AprilTag和灯光
            rospy.loginfo("🚀 开始移动AprilTag和灯光...")
            
            if not self.move_apriltags_and_light():
                rospy.logerr("移动AprilTag和灯光失败，终止识别流程")
                return False
            
            # 等待移动完成
            rospy.sleep(1.0)
            rospy.loginfo("✅ AprilTag和灯光移动完成，开始识别流程")
            
            # 步骤3: 执行低头动作（低头20度）
            if not self.head_down_action():
                rospy.logerr("执行低头动作失败，终止识别流程")
                return False
            
            # 步骤4: 执行AprilTag识别（带重试）
            recognition_success = False
            for attempt in range(self.recognition_retries):
                rospy.loginfo(f"AprilTag识别尝试 {attempt + 1}/{self.recognition_retries}")
                
                result = self.detect_april_tag()
                if result and result['detection_summary']['total_tags_detected'] > 0:
                    recognition_success = True
                    rospy.loginfo(f"识别成功，检测到 {result['detection_summary']['total_tags_detected']} 个标签")
                    break
                else:
                    rospy.logwarn(f"识别尝试 {attempt + 1} 失败")
                    if attempt < self.recognition_retries - 1:
                        rospy.sleep(1.0)  # 重试前等待
            
            # 步骤5: 执行位置精度评估
            evaluation_result = None
            if recognition_success:
                rospy.loginfo("开始位置精度评估...")
                evaluation_result = self.evaluate_all_tags()
                
                if evaluation_result:
                    rospy.loginfo(f"位置精度评估完成，总体结果: {'✅ 通过' if evaluation_result['overall_pass'] else '❌ 失败'}")
                else:
                    rospy.logwarn("位置精度评估失败")
            else:
                rospy.logwarn("AprilTag识别失败，跳过位置精度评估")
            
            # 步骤6: 执行抬头动作（抬头20度回到原位）
            rospy.loginfo("识别流程结束，准备抬头回到原位...")
            if not self.head_up_action():
                rospy.logwarn("执行抬头动作失败，但识别流程已完成")
            
            if recognition_success:
                rospy.loginfo("AprilTag识别工作流程执行成功")
                # 如果有评估结果，返回评估结果；否则返回识别成功
                if evaluation_result:
                    return evaluation_result['overall_pass']
                else:
                    return True
            else:
                rospy.logerr(f"AprilTag识别失败，已重试 {self.recognition_retries} 次")
                return False
            
        except Exception as e:
            rospy.logerr(f"执行AprilTag识别工作流程时发生错误: {e}")
            # 即使出错也要尝试抬头回到原位
            try:
                rospy.loginfo("尝试抬头回到原位...")
                self.head_up_action()
            except:
                rospy.logwarn("抬头动作失败")
            return False

    def _normalize_tag_id(self, raw_id):
        """将检测到的标签ID归一化为整数。
        支持 (id,) 元组、列表或直接的整数/字符串。
        """
        try:
            # 处理 tuple/list: 取第一个元素
            if hasattr(raw_id, '__iter__') and not isinstance(raw_id, (str, bytes)):
                raw_id = list(raw_id)[0]
            return int(raw_id)
        except Exception:
            try:
                return int(str(raw_id))
            except Exception:
                return raw_id

    def evaluate_end_position_with_gazebo(self, model_name: str = 'biped_s45') -> bool:
        """使用 /gazebo/get_model_state 评估轨迹终点精度（不依赖AprilTag）。

        与类内保存的终点 (endpoint_x, endpoint_y) 比较 Gazebo 世界系下模型位置。

        Args:
            model_name (str): Gazebo 模型名，默认 'biped_s45'。

        Returns:
            bool: 误差<=self.max_position_error 返回 True，否则 False；异常时返回 False。
        """
        try:
            # 确保已有终点

            if isinstance(self.trajectory_endpoint, (list, tuple)) and len(self.trajectory_endpoint) >= 2:
                self.endpoint_x, self.endpoint_y = self.trajectory_endpoint[0], self.trajectory_endpoint[1]
            else:
                rospy.logerr(f"❌ 轨迹终点格式错误: {self.trajectory_endpoint}")
                return False

            # 调用服务
            rospy.wait_for_service('/gazebo/get_model_state', timeout=3.0)
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = get_model_state(model_name=model_name)

            if not resp.success:
                rospy.logwarn(f"/gazebo/get_model_state 调用失败（{model_name}）")
                return False

            gx = resp.pose.position.x
            gy = resp.pose.position.y
            dx = gx - float(self.endpoint_x)
            dy = gy - float(self.endpoint_y)
            dist = (dx * dx + dy * dy) ** 0.5

            if dist > self.max_position_error:
                rospy.logerr(
                    f"[Gazebo终点评估] 模型({model_name}) 实际(x={gx:.3f}, y={gy:.3f}) vs 期望(x={self.endpoint_x:.3f}, y={self.endpoint_y:.3f}), "
                    f"误差={dist:.3f} m, 阈值={self.max_position_error:.3f} m")
            else:
                rospy.loginfo(
                    f"[Gazebo终点评估] 模型({model_name}) 实际(x={gx:.3f}, y={gy:.3f}) vs 期望(x={self.endpoint_x:.3f}, y={self.endpoint_y:.3f}), "
                    f"误差={dist:.3f} m, 阈值={self.max_position_error:.3f} m")

            return dist <= self.max_position_error

        except Exception as e:
            rospy.logerr(f"Gazebo 终点评估异常: {e}")
            return False


