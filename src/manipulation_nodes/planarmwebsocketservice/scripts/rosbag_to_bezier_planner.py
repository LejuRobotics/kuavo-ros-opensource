#!/usr/bin/env python3

import rospy
import rosbag
import math
import numpy as np
import json
from datetime import datetime
from humanoid_plan_arm_trajectory.msg import planArmState
from humanoid_plan_arm_trajectory.srv import planArmTrajectoryBezierCurve, planArmTrajectoryBezierCurveRequest
from humanoid_plan_arm_trajectory.msg import bezierCurveCubicPoint, jointBezierTrajectory
from trajectory_msgs.msg import JointTrajectory
from scipy import interpolate
import os


def get_robot_type_for_dekstop_which_only_include_classify_robot_version(robot_version: int) -> str:
    """
    根据 robot_version 获取 robotType

    映射规则：
    - 4代标准版本: '41'
    - 4pro短手: '42'
    - 4pro长手: '45' (45 <= version < 50)
    - 5代标准版本: '52' (50 <= version < 60)
    - 鲁班2: '11' (10 <= version < 20)
    """
    if 45 <= robot_version < 50:
        return '45'
    elif 50 <= robot_version < 60:
        return '52'
    elif 10 <= robot_version < 20:
        return '11'
    else:
        return str(robot_version)  # 41, 42 精确匹配


class RosbagToBezierPlanner:
    """
    Rosbag到贝塞尔曲线轨迹规划器
    
    该类从rosbag文件中提取机器人手臂关节数据，使用贝塞尔曲线规划器生成平滑轨迹。
    支持手臂和手部数据的独立处理，并可选择性地进行自定义采样。
    
    主要功能：
    1. 从rosbag加载关节数据（手臂和手部）
    2. 数据预处理和降采样
    3. 生成贝塞尔曲线控制点
    4. 调用C++规划器生成平滑轨迹
    5. 可选的自定义采样处理
    6. 保存结果为TACT文件格式
    """
    def __init__(self):
        # 检查是否已经有节点初始化，避免重复初始化
        if rospy.get_node_uri() is None:
            rospy.init_node('rosbag_to_bezier_planner')
        # 读取robot_version参数
        self.robot_version = int(os.environ.get("ROBOT_VERSION", "45"))
        rospy.loginfo(f"Robot version: {self.robot_version}")

        # 确定 robotType
        self.robot_type = get_robot_type_for_dekstop_which_only_include_classify_robot_version(self.robot_version)
        rospy.loginfo(f"Robot type: {self.robot_type}")

        # 判断机器人类型
        self.is_roban = (self.robot_version // 10) == 1  # 鲁班系列（版本号以1开头）
        self.has_waist = (self.robot_version // 10) >= 5  # V5系列（版本号以5开头）
        rospy.loginfo(f"Robot type: {'ROBAN' if self.is_roban else 'KUAVO'}, has_waist: {self.has_waist}")

        # 手臂关节名称
        if self.is_roban:
            # 鲁班：8个手臂关节（左4右4），索引13-20
            self.arm_joint_names = [
                "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
                "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
            ]
        else:
            # KUAVO：14个手臂关节（左右各7个）
            self.arm_joint_names = [
                "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
                "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
                "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
                "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
            ]

        # 构建控制关节名称列表
        if self.is_roban:
            # 鲁班：23个关节（8手臂+12手指+2头部+1腰部）
            self.control_joint_names = [
                # 手臂 8 个（索引 0-7）
                "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
                "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
                # 手指 12 个（索引 8-19）
                "l_thumb1", "l_thumb2", "l_index1", "l_middle1", "l_ring1", "l_pinky1",
                "r_thumb1", "r_thumb2", "r_index1", "r_middle1", "r_ring1", "r_pinky1",
                # 头部 2 个（索引 20-21）
                "head_yaw", "head_pitch",
                # 腰部 1 个（索引 22）
                "waist_joint",
            ]
        else:
            # KUAVO：28或29个关节（14手臂+12手指+2头部+可选1腰部）
            self.control_joint_names = [
                "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
                "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
                "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
                "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
                "l_thumb1", "l_thumb2", "l_index1", "l_middle1", "l_ring1", "l_pinky1",
                "r_thumb1", "r_thumb2", "r_index1", "r_middle1", "r_ring1", "r_pinky1",
                "head_yaw", "head_pitch",
            ]
            # 如果有腰部，在最后添加腰部关节
            if self.has_waist:
                self.control_joint_names.append("waist_joint")
                rospy.loginfo("Added waist_joint to control_joint_names")

        # 关节在sensors_data_raw中的索引
        # ROBAN（v10-19）: 腿部0-11, 腰部12, 手臂13-20（8个）, 头部21-22（2个）
        # KUAVO v50+: 腰部在索引12，手臂在索引13-26（14个），头部在索引27-28（2个）
        # KUAVO v40-: 手臂在索引12-25（14个），头部在索引26-27（2个）
        if self.is_roban:
            # ROBAN: 需要从 sensors_data_raw 提取手臂、头部、腰部（手指从/dexhand/state）
            self.arm_joint_indices = list(range(13, 21))  # 13-20，手臂8个
            self.head_joint_indices = [21, 22]  # 21-22，头部2个
            self.waist_joint_index = 12  # 12，腰部1个
        elif self.has_waist:
            # KUAVO v50+: 手臂关节索引为13-28（14个手臂+2个头部）
            self.arm_joint_indices = list(range(13, 29))  # 13-28，其中13-26是手臂，27-28是头部
            self.head_joint_indices = None
            self.waist_joint_index = 12  # 腰部关节索引
        else:
            # KUAVO v40-: 手臂关节索引为12-27（14个手臂+2个头部）
            self.arm_joint_indices = list(range(12, 28))  # 12-27，其中12-25是手臂，26-27是头部
            self.head_joint_indices = None
            self.waist_joint_index = None

        # 数据存储
        self.joint_data = []  # 手臂关节数据（包含头部数据，可能包含腰部数据）
        self.hand_sensors_data = []  # 手部传感器数据
        self.arm_timestamp_data = []  # 手臂数据时间戳
        self.hand_timestamp_data = []  # 手部数据时间戳
        self.waist_data = []  # 腰部关节数据（v50+）

        # 贝塞尔曲线数据
        self.bezier_control_points = []  # 当前处理的控制点
        self.generated_trajectory = []  # 当前生成的轨迹
        self.generated_trajectory_arm = []  # 手臂生成的轨迹
        self.generated_trajectory_hand = []  # 手部生成的轨迹
        self.arm_keyframe_counter = 0  # 手臂轨迹的keyframe_counter

        # ROS发布器和订阅器
        self.trajectory_sub = None
        self.trajectory_state_sub = None

    def load_rosbag_data(self, bag_file_path):
        """
        从rosbag文件中加载关节数据
        
        从以下话题加载数据：
        - /sensors_data_raw: 手臂关节数据
          * v40-: 索引12-27（其中12-25为手臂14个，26-27为头部2个）
          * v50+: 索引12-28（其中12为腰部1个，13-26为手臂14个，27-28为头部2个）
        - /dexhand/state: 手部传感器数据
        
        Args:
            bag_file_path: rosbag文件路径
            
        Returns:
            bool: 是否成功加载数据
        """
        sensors_topic_name = "/sensors_data_raw"
        hand_topic_name = "/dexhand/state"
        rospy.loginfo(f"Loading rosbag data from: {bag_file_path}")
        rospy.loginfo(f"Topic: {sensors_topic_name}")
        rospy.loginfo(f"Topic: {hand_topic_name}")

        try:
            bag = rosbag.Bag(bag_file_path)
            global_start_time = None  # 使用统一的全局起始时间

            for topic, msg, t in bag.read_messages(topics=[sensors_topic_name,hand_topic_name]):
                # 使用两个话题中最早的时间作为全局起始时间
                if global_start_time is None:
                    global_start_time = msg.header.stamp.to_sec()
                else:
                    global_start_time = min(global_start_time, msg.header.stamp.to_sec())

            # 重新读取bag，使用统一的起始时间
            bag.close()
            bag = rosbag.Bag(bag_file_path)
            rospy.loginfo(f"Global start time: {global_start_time:.3f}s")

            for topic, msg, t in bag.read_messages(topics=[sensors_topic_name,hand_topic_name]):
                if topic == sensors_topic_name:
                    # 计算相对时间（使用全局起始时间）
                    relative_time = msg.header.stamp.to_sec() - global_start_time

                    # 提取手臂关节数据
                    arm_positions = []
                    head_positions = []  # 头部数据
                    waist_position = 0.0

                    if self.is_roban:
                        # ROBAN: 提取腰部、手臂、头部数据
                        # 提取腰部数据（索引12）
                        if self.waist_joint_index is not None and self.waist_joint_index < len(msg.joint_data.joint_q):
                            waist_position = msg.joint_data.joint_q[self.waist_joint_index]

                        # 提取手臂数据（索引13-20，8个）
                        for idx in self.arm_joint_indices:
                            if idx < len(msg.joint_data.joint_q):
                                arm_positions.append(msg.joint_data.joint_q[idx])
                            else:
                                arm_positions.append(0.0)

                        # 提取头部数据（索引21-22，2个）
                        for idx in self.head_joint_indices:
                            if idx < len(msg.joint_data.joint_q):
                                head_positions.append(msg.joint_data.joint_q[idx])
                            else:
                                head_positions.append(0.0)

                        # 确保 arm_positions 有 8 个元素
                        expected_arm_count = 8
                        if len(arm_positions) < expected_arm_count:
                            arm_positions.extend([0.0] * (expected_arm_count - len(arm_positions)))
                        elif len(arm_positions) > expected_arm_count:
                            arm_positions = arm_positions[:expected_arm_count]

                        # 确保 head_positions 有 2 个元素
                        expected_head_count = 2
                        if len(head_positions) < expected_head_count:
                            head_positions.extend([0.0] * (expected_head_count - len(head_positions)))
                        elif len(head_positions) > expected_head_count:
                            head_positions = head_positions[:expected_head_count]

                        # 组合控制关节数据：手臂(8) + 手指(12) + 头部(2) + 腰部(1) = 23
                        control_positions = arm_positions + [0.0] * 12 + head_positions + [waist_position]
                    else:
                        # KUAVO v40-/v50+: 提取腰部、手臂、头部数据
                        # 提取腰部数据（v50+）
                        if self.has_waist and self.waist_joint_index is not None:
                            if self.waist_joint_index < len(msg.joint_data.joint_q):
                                waist_position = msg.joint_data.joint_q[self.waist_joint_index]

                        # 提取手臂和头部关节数据
                        # v50+: 腰部在索引12，手臂在索引13-26（14个），头部在索引27-28（2个）
                        # v40-: 手臂在索引12-25（14个），头部在索引26-27（2个）
                        for idx in self.arm_joint_indices:
                            if idx < len(msg.joint_data.joint_q):
                                if self.has_waist:
                                    # v50+: 头部索引从27开始（27-28）
                                    if idx >= 27:
                                        head_positions.append(msg.joint_data.joint_q[idx])
                                    else:
                                        arm_positions.append(msg.joint_data.joint_q[idx])
                                else:
                                    # v40-: 头部索引从26开始（26-27）
                                    if idx >= 26:
                                        head_positions.append(msg.joint_data.joint_q[idx])
                                    else:
                                        arm_positions.append(msg.joint_data.joint_q[idx])
                            else:
                                arm_positions.append(0.0)

                        # 确保 head_positions 是列表
                        head_positions = list(head_positions)

                        # 确保 arm_positions 有 14 个元素（左右各 7 个手臂关节）
                        expected_arm_count = 14
                        if len(arm_positions) < expected_arm_count:
                            rospy.logwarn(f"arm_positions has {len(arm_positions)} elements, padding to {expected_arm_count}")
                            arm_positions.extend([0.0] * (expected_arm_count - len(arm_positions)))
                        elif len(arm_positions) > expected_arm_count:
                            rospy.logwarn(f"arm_positions has {len(arm_positions)} elements, truncating to {expected_arm_count}")
                            arm_positions = arm_positions[:expected_arm_count]

                        # 确保 head_positions 有 2 个元素（头部关节）
                        expected_head_count = 2
                        if len(head_positions) < expected_head_count:
                            rospy.logwarn(f"head_positions has {len(head_positions)} elements, padding to {expected_head_count}")
                            head_positions.extend([0.0] * (expected_head_count - len(head_positions)))
                        elif len(head_positions) > expected_head_count:
                            rospy.logwarn(f"head_positions has {len(head_positions)} elements, truncating to {expected_head_count}")
                            head_positions = head_positions[:expected_head_count]

                        # 组合控制关节数据：手臂(14) + 手指(12) + 头部(2) [+ 腰部(1)]
                        control_positions = arm_positions + [0.0] * 12 + head_positions
                        if self.has_waist:
                            control_positions.append(waist_position)  # 在最后添加腰部数据
                    
                    # 验证最终长度
                    expected_total = len(self.control_joint_names)
                    if len(control_positions) != expected_total:
                        rospy.logerr(f"control_positions length mismatch: expected {expected_total}, got {len(control_positions)}")
                        rospy.logerr(f"  arm_positions: {len(arm_positions)}, hand_positions: {len(hand_positions)}, has_waist: {self.has_waist}")
                        # 填充或截断以匹配期望长度
                        if len(control_positions) < expected_total:
                            control_positions.extend([0.0] * (expected_total - len(control_positions)))
                        else:
                            control_positions = control_positions[:expected_total]
                    
                    self.joint_data.append(control_positions)
                    self.arm_timestamp_data.append(relative_time)
                elif topic == hand_topic_name:
                    # 计算相对时间（使用全局起始时间）
                    relative_time = msg.header.stamp.to_sec() - global_start_time
                    # 确保 msg.position 是列表（可能是元组）
                    hand_position_list = list(msg.position) if hasattr(msg.position, '__iter__') else []
                    
                    # 确保 hand_position_list 有 12 个元素（手指关节）
                    expected_finger_count = 12
                    if len(hand_position_list) < expected_finger_count:
                        rospy.logwarn(f"hand_position_list has {len(hand_position_list)} elements, padding to {expected_finger_count}")
                        hand_position_list.extend([0.0] * (expected_finger_count - len(hand_position_list)))
                    elif len(hand_position_list) > expected_finger_count:
                        rospy.logwarn(f"hand_position_list has {len(hand_position_list)} elements, truncating to {expected_finger_count}")
                        hand_position_list = hand_position_list[:expected_finger_count]
                    
                    # 组合控制关节数据：根据机器人类型构建
                    if self.is_roban:
                        # ROBAN: 手臂(8) + 手指(12) + 头部(2) + 腰部(1) = 23
                        control_positions = [0.0] * 8 + hand_position_list + [0.0] * 2 + [0.0]
                    else:
                        # KUAVO: 手臂(14) + 手指(12) + 头部(2) [+ 腰部(1)]
                        control_positions = [0.0] * 14 + hand_position_list + [0.0] * 2
                        if self.has_waist:
                            control_positions.append(0.0)  # 手部数据中没有腰部，填充0
                    
                    # 验证最终长度
                    expected_total = len(self.control_joint_names)
                    if len(control_positions) != expected_total:
                        rospy.logerr(f"hand control_positions length mismatch: expected {expected_total}, got {len(control_positions)}")
                        # 填充或截断以匹配期望长度
                        if len(control_positions) < expected_total:
                            control_positions.extend([0.0] * (expected_total - len(control_positions)))
                        else:
                            control_positions = control_positions[:expected_total]
                    
                    self.hand_sensors_data.append(control_positions)
                    self.hand_timestamp_data.append(relative_time)

            bag.close()
            
            # 对齐两个话题的数据长度
            sensors_count = len(self.joint_data)
            hand_count = len(self.hand_sensors_data)
            
            rospy.loginfo(f"Loaded {sensors_count} data points from {sensors_topic_name}")
            rospy.loginfo(f"Loaded {hand_count} data points from {hand_topic_name}")

            # 打印时间范围信息，帮助调试时间对齐问题
            if self.arm_timestamp_data:
                rospy.loginfo(f"  {sensors_topic_name} time range: {self.arm_timestamp_data[0]:.3f}s - {self.arm_timestamp_data[-1]:.3f}s")
            if self.hand_timestamp_data:
                rospy.loginfo(f"  {hand_topic_name} time range: {self.hand_timestamp_data[0]:.3f}s - {self.hand_timestamp_data[-1]:.3f}s")
            
            # 如果长度不一致，用较短话题的最后一个值填充
            if sensors_count != hand_count:
                if sensors_count < hand_count:
                    # sensors_data_raw 较短，需要填充
                    rospy.logwarn(f"Length mismatch: {sensors_topic_name} has {sensors_count} points, "
                                f"{hand_topic_name} has {hand_count} points. "
                                f"Padding {sensors_topic_name} with last value.")
                    last_joint_data = self.joint_data[-1] if self.joint_data else None
                    last_arm_timestamp = self.arm_timestamp_data[-1] if self.arm_timestamp_data else None
                    
                    if last_joint_data is not None and last_arm_timestamp is not None:
                        # 计算时间间隔（使用最后一个时间戳和倒数第二个时间戳的差值，或使用默认值）
                        if len(self.arm_timestamp_data) > 1:
                            time_interval = self.arm_timestamp_data[-1] - self.arm_timestamp_data[-2]
                        else:
                            time_interval = 0.01  # 默认10ms
                        
                        # 填充到与 hand_count 一致
                        for i in range(sensors_count, hand_count):
                            self.joint_data.append(list(last_joint_data))  # 复制最后一个值
                            self.arm_timestamp_data.append(last_arm_timestamp + time_interval * (i - sensors_count + 1))
                        
                        rospy.loginfo(f"Padded {sensors_topic_name} to {len(self.joint_data)} points")
                else:
                    # hand_sensors_data 较短，需要填充
                    rospy.logwarn(f"Length mismatch: {hand_topic_name} has {hand_count} points, "
                                f"{sensors_topic_name} has {sensors_count} points. "
                                f"Padding {hand_topic_name} with last value.")
                    last_hand_data = self.hand_sensors_data[-1] if self.hand_sensors_data else None
                    last_hand_timestamp = self.hand_timestamp_data[-1] if self.hand_timestamp_data else None
                    
                    if last_hand_data is not None and last_hand_timestamp is not None:
                        # 计算时间间隔
                        if len(self.hand_timestamp_data) > 1:
                            time_interval = self.hand_timestamp_data[-1] - self.hand_timestamp_data[-2]
                        else:
                            time_interval = 0.01  # 默认10ms
                        
                        # 填充到与 sensors_count 一致
                        for i in range(hand_count, sensors_count):
                            self.hand_sensors_data.append(list(last_hand_data))  # 复制最后一个值
                            self.hand_timestamp_data.append(last_hand_timestamp + time_interval * (i - hand_count + 1))
                        
                        rospy.loginfo(f"Padded {hand_topic_name} to {len(self.hand_sensors_data)} points")
                
                rospy.loginfo(f"After alignment: {sensors_topic_name}={len(self.joint_data)}, "
                            f"{hand_topic_name}={len(self.hand_sensors_data)}")
            
            return True

        except Exception as e:
            rospy.logerr(f"Error loading rosbag: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return False

    def preprocess_data(self, sampling_rate=0.01, control_data=None, timestamp_data=None):
        """
        预处理数据：降采样和滤波
        
        Args:
            sampling_rate: 采样率（秒）
            control_data: 要处理的控制数据列表
            timestamp_data: 对应的时间戳列表
            
        Returns:
            tuple: (处理后的控制数据, 处理后的时间戳数据) 或 (None, None) 如果失败
        """
        rospy.loginfo("Preprocessing data...")

        if not control_data or not timestamp_data:
            rospy.logerr("No data to preprocess")
            return None, None

        # 转换为numpy数组
        joint_array = np.array(control_data)
        timestamp_array = np.array(timestamp_data)

        # 降采样 - 每sampling_rate秒取一个点
        target_times = np.arange(timestamp_array[0], timestamp_array[-1], sampling_rate)

        # 对每个关节进行插值
        processed_data = []
        for joint_idx in range(joint_array.shape[1]):
            joint_positions = joint_array[:, joint_idx]

            # 使用三次样条插值
            spline = interpolate.CubicSpline(timestamp_array, joint_positions)
            processed_positions = spline(target_times)
            processed_data.append(processed_positions)

        # 转换为列表格式
        processed_control_data = np.array(processed_data).T.tolist()
        processed_timestamp_data = target_times.tolist()

        rospy.loginfo(f"Preprocessed to {len(processed_control_data)} data points")
        return processed_control_data, processed_timestamp_data

    def calculate_bezier_control_points(self, p0_time, p0_value, p3_time, p3_value, 
                                         prev_value=None, next_value=None, 
                                         prev_time=None, next_time=None,
                                         segment_idx=0, total_segments=1,
                                         smoothing_factor=0.3):
        """
        计算贝塞尔曲线的左右控制点（P1和P2）
        
        使用标准三次贝塞尔曲线格式：P0, P1, P2, P3
        - P0: 起始点 (p0_time, p0_value)
        - P1: 左控制点 (left_control_point)
        - P2: 右控制点 (right_control_point)
        - P3: 结束点 (p3_time, p3_value)
        
        Args:
            p0_time: 起始点时间
            p0_value: 起始点位置值
            p3_time: 结束点时间
            p3_value: 结束点位置值
            prev_value: 前一个点的位置值（用于计算后向斜率，中间段和最后段需要）
            next_value: 下一个点的位置值（用于计算前向斜率，第一段和中间段需要）
            prev_time: 前一个点的时间（用于计算后向斜率）
            next_time: 下一个点的时间（用于计算前向斜率）
            segment_idx: 当前段的索引（0表示第一段，total_segments-1表示最后一段）
            total_segments: 总段数
            smoothing_factor: 平滑因子，控制控制点的时间位置（默认0.3，即30%和70%处）
            
        Returns:
            dict: 包含以下键的字典
                - 'left_control_point': [p1_time, p1_value] - 左控制点（P1）
                - 'right_control_point': [p2_time, p2_value] - 右控制点（P2）
        """
        # 计算控制点时间位置（所有情况相同）
        segment_duration = p3_time - p0_time
        p1_time = p0_time + segment_duration * smoothing_factor
        p2_time = p3_time - segment_duration * smoothing_factor
        
        # 根据段的位置计算斜率
        is_first_segment = (segment_idx == 0)
        is_last_segment = (segment_idx == total_segments - 1)
        
        if is_first_segment:
            # 第一段：使用前向切线
            if next_value is not None and next_time is not None:
                # 使用下一个点计算前向斜率
                forward_slope = (next_value - p0_value) / (next_time - p0_time)
            else:
                # 如果没有下一个点，使用当前段的斜率
                forward_slope = (p3_value - p0_value) / segment_duration if segment_duration > 0 else 0.0
            
            # P1: 在P0基础上沿前向切线延伸
            p1_value = p0_value + forward_slope * (p1_time - p0_time)
            # P2: 在P3基础上沿前向切线延伸（反向）
            p2_value = p3_value - forward_slope * (p3_time - p2_time)
            
        elif is_last_segment:
            # 最后一段：使用后向切线
            if prev_value is not None and prev_time is not None:
                # 使用前一个点计算后向斜率
                backward_slope = (p0_value - prev_value) / (p0_time - prev_time)
            else:
                # 如果没有前一个点，使用当前段的斜率
                backward_slope = (p3_value - p0_value) / segment_duration if segment_duration > 0 else 0.0
            
            # P1: 在P0基础上沿后向切线延伸
            p1_value = p0_value + backward_slope * (p1_time - p0_time)
            # P2: 在P3基础上沿后向切线延伸（反向）
            p2_value = p3_value - backward_slope * (p3_time - p2_time)
            
        else:
            # 中间段：使用前后切线的平均值
            forward_slope = 0.0
            backward_slope = 0.0
            
            # 计算前向斜率
            if next_value is not None and next_time is not None:
                forward_slope = (next_value - p0_value) / (next_time - p0_time)
            else:
                forward_slope = (p3_value - p0_value) / segment_duration if segment_duration > 0 else 0.0
            
            # 计算后向斜率
            if prev_value is not None and prev_time is not None:
                backward_slope = (p0_value - prev_value) / (p0_time - prev_time)
            else:
                backward_slope = (p3_value - p0_value) / segment_duration if segment_duration > 0 else 0.0
            
            # 使用平均斜率
            avg_slope = (forward_slope + backward_slope) * 0.5
            
            # P1: 在P0基础上沿平均斜率延伸
            p1_value = p0_value + avg_slope * (p1_time - p0_time)
            # P2: 在P3基础上沿平均斜率延伸（反向）
            p2_value = p3_value - avg_slope * (p3_time - p2_time)
        
        return {
            'left_control_point': [p1_time, p1_value],
            'right_control_point': [p2_time, p2_value]
        }

    def generate_bezier_control_points(self, smoothing_factor=0.3, control_data=None, timestamp_data=None, has_finger_data=False):
        """
        从原始数据生成贝塞尔曲线控制点 - 与C++实现保持一致
        使用标准三次贝塞尔曲线格式：P0, P1, P2, P3
        
        Args:
            smoothing_factor: 平滑因子
            control_data: 控制数据列表
            timestamp_data: 时间戳列表
            has_finger_data: 是否有手指数据（dexhand/state话题是否有数据）
            
        Returns:
            bool: 是否成功生成控制点
        """
        rospy.loginfo("Generating Bezier control points (C++ compatible)...")

        if not control_data or len(control_data) < 2:
            rospy.logerr("Insufficient data for Bezier curve generation")
            return False

        if not timestamp_data or len(timestamp_data) != len(control_data):
            rospy.logerr("Timestamp data mismatch")
            return False

        # 检查 control_data 中每个点的长度是否一致且等于期望的关节数
        expected_joint_count = len(self.control_joint_names)
        if len(control_data) > 0:
            first_point_length = len(control_data[0])
            if first_point_length != expected_joint_count:
                rospy.logerr(f"Control data dimension mismatch: expected {expected_joint_count} joints, "
                           f"but got {first_point_length} in first data point")
                return False
            
            # 检查所有点的长度是否一致
            for i, point in enumerate(control_data):
                if len(point) != expected_joint_count:
                    rospy.logerr(f"Control data dimension mismatch at index {i}: "
                               f"expected {expected_joint_count} joints, but got {len(point)}")
                    rospy.logerr(f"  First point length: {len(control_data[0])}, "
                               f"point[{i}] length: {len(point)}")
                    return False

        self.bezier_control_points = []

        for joint_idx in range(len(self.control_joint_names)):
            # 检查是否为手指关节（根据机器人类型判断索引范围）
            if self.is_roban:
                is_finger_joint = 8 <= joint_idx < 20   # ROBAN: 手指索引8-19
            else:
                is_finger_joint = 14 <= joint_idx < 26  # KUAVO: 手指索引14-25
            
            # 如果没有手指数据且当前是手指关节，生成全0的控制点
            # 注意：腰部关节的数据来自sensors_data_raw话题，已经包含在control_data中，应该正常处理
            if is_finger_joint and not has_finger_data:
                # 为手指关节生成全0的控制点序列
                # 当dexhand/state话题为空时，所有手指关节保持为0
                curve_points = []
                total_segments = len(timestamp_data) - 1
                for i in range(total_segments):
                    p0_time = timestamp_data[i]
                    p3_time = timestamp_data[i + 1]
                    # 使用统一的函数接口计算控制点（位置值设为0）
                    control_points = self.calculate_bezier_control_points(
                        p0_time=p0_time,
                        p0_value=0.0,
                        p3_time=p3_time,
                        p3_value=0.0,
                        prev_value=None,
                        next_value=None,
                        prev_time=None,
                        next_time=None,
                        segment_idx=i,
                        total_segments=total_segments,
                        smoothing_factor=smoothing_factor
                    )
                    # 所有控制点的位置值都设置为0
                    curve_segment = {
                        'end_point': [p3_time, 0.0],
                        'left_control_point': [control_points['left_control_point'][0], 0.0],
                        'right_control_point': [control_points['right_control_point'][0], 0.0]
                    }
                    curve_points.append(curve_segment)
                self.bezier_control_points.append(curve_points)
                rospy.loginfo(f"Generated zero control points for finger joint {joint_idx} (no dexhand/state data)")
                continue
            
            # 正常处理：手臂、头部、腰部关节，或有手指数据时的手指关节
            # 安全检查：确保所有点都有足够的元素
            try:
                joint_positions = [point[joint_idx] for point in control_data]
            except IndexError as e:
                rospy.logerr(f"IndexError when accessing joint {joint_idx} ({self.control_joint_names[joint_idx]}): {e}")
                # 检查第一个点的长度
                if len(control_data) > 0:
                    rospy.logerr(f"First data point has {len(control_data[0])} elements, but trying to access index {joint_idx}")
                    rospy.logerr(f"Expected {len(self.control_joint_names)} joints, but data has {len(control_data[0])} elements")
                    # 找出所有长度不一致的点
                    for i, point in enumerate(control_data):
                        if len(point) != len(control_data[0]):
                            rospy.logerr(f"  Point[{i}] has {len(point)} elements (expected {len(control_data[0])})")
                return False

            # 如果是手指关节（索引14-25），将百分比（0-100）转换为弧度（0-1.7453）
            # C++ 贝塞尔规划器期望手指数据是弧度格式
            if is_finger_joint and has_finger_data:
                finger_max_radians = 1.7453
                joint_positions = [(pos / 100.0) * finger_max_radians for pos in joint_positions]

            joint_times = timestamp_data

            curve_points = []
            total_segments = len(joint_times) - 1

            for i in range(total_segments):
                # 当前段：P0 -> P3
                p0_time = joint_times[i]
                p3_time = joint_times[i + 1]
                p0_value = joint_positions[i]
                p3_value = joint_positions[i + 1]

                # 准备前后点的数据（用于计算斜率）
                prev_value = joint_positions[i - 1] if i > 0 else None
                prev_time = joint_times[i - 1] if i > 0 else None
                next_value = joint_positions[i + 2] if i + 2 < len(joint_positions) else None
                next_time = joint_times[i + 2] if i + 2 < len(joint_times) else None

                # 使用统一的函数接口计算控制点
                control_points = self.calculate_bezier_control_points(
                    p0_time=p0_time,
                    p0_value=p0_value,
                    p3_time=p3_time,
                    p3_value=p3_value,
                    prev_value=prev_value,
                    next_value=next_value,
                    prev_time=prev_time,
                    next_time=next_time,
                    segment_idx=i,
                    total_segments=total_segments,
                    smoothing_factor=smoothing_factor
                )

                # 创建贝塞尔曲线段 - 与C++格式一致
                curve_segment = {
                    'end_point': [p3_time, p3_value],
                    'left_control_point': control_points['left_control_point'],
                    'right_control_point': control_points['right_control_point']
                }
                curve_points.append(curve_segment)

            self.bezier_control_points.append(curve_points)

        rospy.loginfo(f"Generated {len(self.bezier_control_points)} joint control point sequences")
        return True

    def create_bezier_request(self, start_time=0.0, save_tact_file=False):
        """
        创建贝塞尔曲线规划请求 - 与C++实现保持一致
        
        Args:
            start_time: 起始时间
            save_tact_file: 如果提供文件路径，将贝塞尔控制点保存为TACT文件
        """
        req = planArmTrajectoryBezierCurveRequest()

        for joint_control_points in self.bezier_control_points:
            msg = jointBezierTrajectory()

            # 按照C++期望的格式组织控制点
            # C++期望格式：[P0, P1, P2, P3] 其中P1和P2是控制点
            for curve_point in joint_control_points:
                point = bezierCurveCubicPoint()

                # 设置终点 (P3)
                point.end_point = curve_point['end_point']

                # 设置控制点 (P1 和 P2)
                point.left_control_point = curve_point['left_control_point']
                point.right_control_point = curve_point['right_control_point']

                msg.bezier_curve_points.append(point)

            req.multi_joint_bezier_trajectory.append(msg)

        req.start_frame_time = start_time
        # 从bezier_control_points中获取最后一个时间点
        if self.bezier_control_points and len(self.bezier_control_points) > 0:
            last_joint_points = self.bezier_control_points[0]
            if last_joint_points:
                req.end_frame_time = last_joint_points[-1]['end_point'][0]
            else:
                req.end_frame_time = 10.0
        else:
            req.end_frame_time = 10.0
        # 使用所有28个关节名称（包括手臂、手指、头部）
        req.joint_names = self.control_joint_names

        # 如果指定了保存路径，将贝塞尔控制点保存为TACT文件
        if save_tact_file:
            # 如果 save_tact_file 是布尔值 True，生成默认文件路径
            if isinstance(save_tact_file, bool):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                output_dir = getattr(self, 'output_dir', './bezier_results')
                if not os.path.exists(output_dir):
                    os.makedirs(output_dir)
                save_tact_file = os.path.join(output_dir, f'bezier_request_{timestamp}.tact')
            try:
                self._save_bezier_control_points_to_tact(save_tact_file, start_time)
            except Exception as e:
                rospy.logerr(f"Failed to save Bezier control points to TACT file: {e}")
                import traceback
                traceback.print_exc()

        return req

    def setup_trajectory_listeners(self):
        """
        设置轨迹监听器来记录生成的曲线点
        
        订阅以下话题：
        - /bezier/arm_traj: 规划器生成的轨迹点
        - /bezier/arm_traj_state: 轨迹执行状态
        
        注意：/bezier/arm_traj 发布频率为1000Hz，keyframe转换为10ms单位（乘以0.1）
        """
        self.generated_trajectory = []
        self.planning_complete = False
        self.keyframe_counter = 0

        def trajectory_callback(msg):
            start_time = msg.header.stamp.to_sec()
            # 根据机器人类型确定手指索引范围
            if self.is_roban:
                finger_start_idx = 8   # ROBAN: 手指索引8-19
                finger_end_idx = 20
            else:
                finger_start_idx = 14  # KUAVO: 手指索引14-25
                finger_end_idx = 26
            # 手指关节的弧度范围：0~100 映射到 0~1.7453 弧度
            finger_max_radians = 1.7453

            for i, point in enumerate(msg.points):
                t = start_time + point.time_from_start.to_sec()
                positions = list(point.positions)

                # 将手指关节（索引14-25）从弧度映射回0~100范围
                for finger_idx in range(finger_start_idx, min(finger_end_idx, len(positions))):
                    if positions[finger_idx] is not None:
                        # 反向映射：radians -> 0~100
                        # 如果弧度值在合理范围内（0~1.7453），映射回0~100
                        if 0 <= positions[finger_idx] <= finger_max_radians:
                            positions[finger_idx] = (positions[finger_idx] / finger_max_radians) * 100.0
                        elif positions[finger_idx] < 0:
                            # 如果小于0，映射为0
                            positions[finger_idx] = 0.0
                        elif positions[finger_idx] > finger_max_radians:
                            # 如果大于最大值，映射为100
                            positions[finger_idx] = 100.0

                trajectory_point = {
                    'time': t,
                    'positions': positions,
                    'velocities': list(point.velocities) if point.velocities else [0.0] * len(self.control_joint_names),
                    'accelerations': list(point.accelerations) if point.accelerations else [0.0] * len(self.control_joint_names),
                    'keyframe': self.keyframe_counter * 0.1  # 转换为10ms单位（1000Hz -> 10ms）
                }
                self.generated_trajectory.append(trajectory_point)
            self.keyframe_counter += 1
        def trajectory_state_callback(msg):
            """处理轨迹状态信息"""
            if hasattr(msg, 'progress'):
                # rospy.loginfo(f"Trajectory progress: {msg.progress}ms, finished: {msg.is_finished}")
                if msg.is_finished:
                    self.planning_complete = True
                    rospy.loginfo(f"Planning complete! Generated {len(self.generated_trajectory)} trajectory points")

        self.trajectory_sub = rospy.Subscriber(
            '/bezier/arm_traj',
            JointTrajectory,
            trajectory_callback,
            queue_size=1,
            tcp_nodelay=True
        )

        self.trajectory_state_sub = rospy.Subscriber(
            '/bezier/arm_traj_state',
            planArmState,
            trajectory_state_callback,
            queue_size=1,
            tcp_nodelay=True
        )

    def call_bezier_planner(self, req):
        """
        调用贝塞尔曲线规划服务
        """
        rospy.loginfo("Calling Bezier curve planner...")

        service_name = '/bezier/plan_arm_trajectory'

        try:
            rospy.wait_for_service(service_name, timeout=5.0)
            plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
            res = plan_service(req)

            if res.success:
                rospy.loginfo("Bezier curve planning successful!")
                return True
            else:
                rospy.logerr("Bezier curve planning failed")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        except rospy.ROSException as e:
            rospy.logerr(f"Service not available: {e}")
            return False

    def save_to_tact_file(self, output_file, total_duration=None, trajectory_data=None):
        """
        将数据保存为tact文件格式

        Args:
            output_file: 输出文件路径
            total_duration: 总持续时间
            trajectory_data: 轨迹数据，如果为None则使用原始控制点数据
        """
        if trajectory_data:
            rospy.loginfo(f"Saving sampling results to TACT file: {output_file}")
            return self._save_trajectory_to_tact(output_file, trajectory_data, total_duration)
        else:
            rospy.loginfo(f"Saving Bezier control points to TACT file (C++ compatible): {output_file}")
            rospy.logwarn("Saving control points to TACT is not yet implemented")
            return False

    def _save_trajectory_to_tact(self, output_file, trajectory_data, total_duration=None):
        """
        将采样后的轨迹数据保存为tact文件格式

        Args:
            output_file: 输出文件路径
            trajectory_data: 轨迹数据列表
            total_duration: 总持续时间
        """
        if not trajectory_data:
            rospy.logerr("No trajectory data to save")
            return False

        # 按时间排序轨迹数据
        sorted_trajectory = sorted(trajectory_data, key=lambda x: x['time'])
        
        # 计算原始轨迹的 finish 和 first 值（基于原始轨迹，不受去重影响）
        original_finish = 0
        original_first = 0
        if sorted_trajectory:
            # 计算原始 finish（基于最后一个点的时间或keyframe）
            last_point = sorted_trajectory[-1]
            if 'keyframe' in last_point:
                if isinstance(last_point['keyframe'], float):
                    original_finish = int(last_point['keyframe'])  # 秒转10ms单位
                else:
                    original_finish = int(last_point['keyframe'])
            else:
                original_finish = int(last_point['time'] * 100)
            
            # 计算原始 first（基于第一个点的时间或keyframe）
            first_point = sorted_trajectory[0]
            if 'keyframe' in first_point:
                if isinstance(first_point['keyframe'], float):
                    original_first = int(first_point['keyframe'] )  # 秒转10ms单位
                else:
                    original_first = int(first_point['keyframe'])
            else:
                original_first = int(first_point['time'] * 100)
        
        # 去重：如果 keyframe 已存在，则跳过该点（不使用）
        unique_trajectory = []
        seen_keyframes = set()
        skipped_count = 0
        
        for point in sorted_trajectory:
            # 计算keyframe（10ms单位）
            if 'keyframe' in point:
                # 如果point中有keyframe字段，检查其单位
                # 根据第370行的设置，keyframe = counter * 0.1，单位是秒
                # 需要转换为10ms单位：秒 * 100 = 10ms单位
                if isinstance(point['keyframe'], float):
                    keyframe = int(point['keyframe'])  # 秒转10ms单位
                else:
                    # 如果是整数，假设已经是10ms单位
                    keyframe = int(point['keyframe'])
            else:
                # 否则从时间计算（时间单位是秒，转换为10ms单位）
                keyframe = int(point['time'])
            
            # 如果 keyframe 已存在，跳过该点（不使用）
            if keyframe in seen_keyframes:
                skipped_count += 1
                continue
            
            # 使用该点，记录 keyframe
            seen_keyframes.add(keyframe)
            # 更新point的keyframe（保存为10ms单位）
            point['keyframe'] = keyframe
            unique_trajectory.append(point)
        

        # 创建tact文件结构
        # finish 和 first 使用原始值，不受去重影响
        tact_data = {
            "frames": [],
            "musics": [],
            "finish": original_finish,
            "first": original_first,
            "version": "rosbag_to_act_frames",
            "robotType": self.robot_type
        }

        # 为每个轨迹点创建frame
        for frame_idx, point in enumerate(unique_trajectory):
            # 计算关节数量：ROBAN为23个，KUAVO v40-为28个，v50+为29个（包含腰部）
            if self.is_roban:
                num_joints = 23  # 鲁班：8手臂+12手指+2头部+1腰部
            elif self.has_waist:
                num_joints = 29  # KUAVO v50+：28个+1个腰部
            else:
                num_joints = 28  # KUAVO v40-：28个
            # 创建servo数据
            servos = [0] * num_joints

            # 填充所有关节数据
            # ROBAN: 手臂8个 + 手指12个 + 头部2个 + 腰部1个 = 23个
            # KUAVO v40-: 手臂14个 + 手指12个 + 头部2个 = 28个
            # KUAVO v50+: 手臂14个 + 手指12个 + 头部2个 + 腰部1个 = 29个
            positions = point.get('positions', [])
            # 根据机器人类型确定手指索引范围
            if self.is_roban:
                finger_start_idx = 8   # ROBAN: 手指索引8-19
                finger_end_idx = 20
            else:
                finger_start_idx = 14  # KUAVO: 手指索引14-25
                finger_end_idx = 26

            for joint_idx in range(num_joints):  # 扩展到所有关节
                if joint_idx < len(positions):
                    # 手指关节来自/dexhand/state，已经是0-100的百分比值，不需要转换
                    # 手臂和头部关节来自/sensors_data_raw，是弧度值，需要转换为角度
                    if finger_start_idx <= joint_idx < finger_end_idx:
                        # 手指关节：直接使用原值（已经是0-100范围）
                        servos[joint_idx] = int(round(positions[joint_idx]))
                    else:
                        # 手臂和头部关节：将弧度转换为角度
                        angle_deg = math.degrees(positions[joint_idx])
                        servos[joint_idx] = int(round(angle_deg))

            # 使用去重后的唯一keyframe
            keyframe = point['keyframe']

            # 创建属性数据（为所有关节计算控制点）
            attributes = {}
            # 计算关节数量：ROBAN为23个，KUAVO v40-为28个，v50+为29个
            if self.is_roban:
                num_joints = 23  # 鲁班：8手臂+12手指+2头部+1腰部
            elif self.has_waist:
                num_joints = 29  # KUAVO v50+：28个+1个腰部
            else:
                num_joints = 28  # KUAVO v40-：28个
            for servo_idx in range(1, num_joints + 1):  # 伺服电机索引从1开始
                joint_idx = servo_idx - 1  # 转换为0-based索引
                # 为所有关节（手臂、手指、头部、腰部）计算控制点
                if joint_idx < len(positions):
                    cp_data = self._calculate_control_points_for_trajectory(unique_trajectory, frame_idx, joint_idx)
                    attributes[str(servo_idx)] = {
                        "CP": cp_data,
                        "CPType": ["AUTO", "AUTO"],
                        "select": False
                    }
                else:
                    # 如果数据不足，使用默认控制点
                    attributes[str(servo_idx)] = {
                        "CP": [[0, 0], [21, 0]],
                        "CPType": ["AUTO", "AUTO"],
                        "select": False
                    }

            frame_data = {
                "servos": servos,
                "keyframe": keyframe,
                "attribute": attributes
            }

            tact_data["frames"].append(frame_data)

        # 保存为JSON文件
        with open(output_file, 'w') as f:
            json.dump(tact_data, f, indent=2)

        rospy.loginfo(f"TACT file saved successfully: {output_file}")
        return True

    def _save_bezier_control_points_to_tact(self, output_file, start_time=0.0):
        """
        将贝塞尔控制点保存为TACT文件格式
        
        Args:
            output_file: 输出文件路径
            start_time: 起始时间（秒）
        """
        if not self.bezier_control_points or len(self.bezier_control_points) == 0:
            rospy.logerr("No Bezier control points to save")
            return False
        
        rospy.loginfo(f"Saving Bezier control points to TACT file: {output_file}")
        
        # 收集所有唯一的时间点（从所有关节的end_point中）
        all_times = set()
        for joint_control_points in self.bezier_control_points:
            for curve_point in joint_control_points:
                end_time = curve_point['end_point'][0]
                all_times.add(end_time)
        
        # 添加起始时间点（第一个P0点）
        if len(self.bezier_control_points) > 0 and len(self.bezier_control_points[0]) > 0:
            # 第一个关节的第一个曲线段的起始点时间
            first_segment = self.bezier_control_points[0][0]
            # P0时间 = left_control_point时间 - (end_point时间 - left_control_point时间) / smoothing_factor
            # 简化：使用第一个left_control_point的时间作为参考
            # 实际上，我们需要从控制点反推P0
            # 为了简化，我们使用start_time或第一个end_point之前的时间
            if start_time > 0:
                all_times.add(start_time)
            else:
                # 估算第一个P0时间
                first_end_time = first_segment['end_point'][0]
                first_left_time = first_segment['left_control_point'][0]
                estimated_p0_time = first_left_time - (first_end_time - first_left_time) * 2
                all_times.add(max(0.0, estimated_p0_time))
        
        sorted_times = sorted(all_times)
        
        if len(sorted_times) == 0:
            rospy.logerr("No time points found in control points")
            return False
        
        # 计算finish和first（转换为10ms单位）
        first_time = sorted_times[0]
        finish_time = sorted_times[-1]
        first_keyframe = int(first_time * 100)  # 秒转10ms单位
        finish_keyframe = int(finish_time * 100)  # 秒转10ms单位
        
        # 创建TACT文件结构
        tact_data = {
            "frames": [],
            "musics": [],
            "finish": finish_keyframe,
            "first": first_keyframe,
            "version": "rosbag_to_act",
            "robotType": self.robot_type
        }
        
        # 为每个时间点创建frame
        for time_idx, current_time in enumerate(sorted_times):
            keyframe = int(current_time * 100)  # 秒转10ms单位

            # 计算关节数量：ROBAN为23个，KUAVO v40-为28个，v50+为29个
            if self.is_roban:
                num_joints = 23  # 鲁班：8手臂+12手指+2头部+1腰部
            elif self.has_waist:
                num_joints = 29  # KUAVO v50+：28个+1个腰部
            else:
                num_joints = 28  # KUAVO v40-：28个
            # 创建servo数据
            servos = [0] * num_joints
            
            # 创建属性数据
            attributes = {}
            
            # 对每个关节计算在该时间点的位置和控制点
            for joint_idx in range(len(self.bezier_control_points)):
                joint_control_points = self.bezier_control_points[joint_idx]
                
                # 找到包含当前时间点的曲线段
                current_position = 0.0
                left_cp = [0, 0]  # [时间偏移(10ms), 角度偏移(度)]
                right_cp = [0, 0]
                
                # 查找当前时间点所在的曲线段
                for seg_idx, curve_point in enumerate(joint_control_points):
                    end_time = curve_point['end_point'][0]
                    
                    if current_time <= end_time or seg_idx == len(joint_control_points) - 1:
                        # 找到对应的曲线段
                        # 计算P0时间（前一个end_point，或start_time）
                        if seg_idx == 0:
                            p0_time = start_time if start_time > 0 else (end_time - (end_time - curve_point['left_control_point'][0]) * 2)
                        else:
                            p0_time = joint_control_points[seg_idx - 1]['end_point'][0]
                        
                        p0_value = 0.0
                        if seg_idx > 0:
                            p0_value = joint_control_points[seg_idx - 1]['end_point'][1]
                        
                        p3_time = end_time
                        p3_value = curve_point['end_point'][1]
                        p1_time, p1_value = curve_point['left_control_point']
                        p2_time, p2_value = curve_point['right_control_point']
                        
                        # 使用贝塞尔曲线公式计算当前位置
                        if p3_time > p0_time:
                            t = (current_time - p0_time) / (p3_time - p0_time)
                            t = max(0.0, min(1.0, t))  # 限制在[0,1]范围
                            
                            # 三次贝塞尔曲线公式: B(t) = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3
                            current_position = (
                                (1-t)**3 * p0_value +
                                3 * (1-t)**2 * t * p1_value +
                                3 * (1-t) * t**2 * p2_value +
                                t**3 * p3_value
                            )
                        else:
                            current_position = p0_value if current_time <= p0_time else p3_value
                        
                        # 计算控制点（转换为TACT格式：相对于当前帧的偏移）
                        # 手指关节已经是0-100范围，不需要弧度转换
                        # 其他关节是弧度值，需要转换为度
                        # 根据机器人类型确定手指索引范围
                        if self.is_roban:
                            finger_start_idx = 8   # ROBAN: 手指索引8-19
                            finger_end_idx = 20
                        else:
                            finger_start_idx = 14  # KUAVO: 手指索引14-25
                            finger_end_idx = 26
                        is_finger_joint = finger_start_idx <= joint_idx < finger_end_idx
                        
                        # 左控制点：从当前时间到P1的时间偏移和角度偏移
                        left_time_offset = int((p1_time - current_time) * 100)  # 秒转10ms单位
                        if is_finger_joint:
                            # 手指关节：直接使用差值（已经是0-100范围）
                            left_angle_offset = int(round(p1_value - current_position))
                        else:
                            # 其他关节：弧度转角度
                            left_angle_offset = int(round(math.degrees(p1_value - current_position)))
                        left_cp = [left_time_offset, left_angle_offset]
                        
                        # 右控制点：从当前时间到P2的时间偏移和角度偏移
                        right_time_offset = int((p2_time - current_time) * 100)  # 秒转10ms单位
                        if is_finger_joint:
                            # 手指关节：直接使用差值（已经是0-100范围）
                            right_angle_offset = int(round(p2_value - current_position))
                        else:
                            # 其他关节：弧度转角度
                            right_angle_offset = int(round(math.degrees(p2_value - current_position)))
                        right_cp = [right_time_offset, right_angle_offset]
                        
                        break
                
                # 手指关节已经是0-100范围，不需要弧度转换
                # 其他关节是弧度值，需要转换为角度
                # 根据机器人类型确定手指索引范围
                if self.is_roban:
                    finger_start_idx = 8   # ROBAN: 手指索引8-19
                    finger_end_idx = 20
                else:
                    finger_start_idx = 14  # KUAVO: 手指索引14-25
                    finger_end_idx = 26
                is_finger_joint = finger_start_idx <= joint_idx < finger_end_idx
                
                if is_finger_joint:
                    # 手指关节：直接使用原值（已经是0-100范围）
                    servos[joint_idx] = int(round(current_position))
                else:
                    # 其他关节：将弧度转换为角度
                    angle_deg = int(round(math.degrees(current_position)))
                    servos[joint_idx] = angle_deg
                
                # 设置属性
                attributes[str(joint_idx + 1)] = {
                    "CP": [left_cp, right_cp],
                    "CPType": ["AUTO", "AUTO"],
                    "select": False
                }
            
            # 创建frame数据
            frame_data = {
                "servos": servos,
                "keyframe": keyframe,
                "attribute": attributes
            }
            
            tact_data["frames"].append(frame_data)
        
        # 去重keyframe
        unique_frames = []
        seen_keyframes = set()
        for frame in tact_data["frames"]:
            keyframe = frame["keyframe"]
            if keyframe not in seen_keyframes:
                seen_keyframes.add(keyframe)
                unique_frames.append(frame)
            else:
                rospy.logwarn(f"Skipping duplicate keyframe: {keyframe}")
        
        tact_data["frames"] = unique_frames
        
        # 保存为JSON文件
        with open(output_file, 'w') as f:
            json.dump(tact_data, f, indent=2)
        
        rospy.loginfo(f"Bezier control points saved to TACT file: {output_file} ({len(unique_frames)} frames)")
        return True

    def _calculate_control_points_for_trajectory(self, trajectory, frame_idx, joint_idx, 
                                                  cp_time_left=3.0, cp_time_right=7.0,
                                                  cp_velocity_scale=10.0, cp_static_threshold=0.01):
        """
        为轨迹数据计算控制点（CP - Control Points）
        
        控制点的作用：
        1. 控制关节在两个关键帧之间的插值曲线形状
        2. 左控制点（left CP）：影响从当前帧开始的运动趋势
        3. 右控制点（right CP）：影响到达下一帧的运动趋势
        4. 控制点的时间值：相对于当前帧的时间偏移（单位：10ms）
        5. 控制点的位置值：相对于当前帧位置的偏移量（单位：角度）
        
        控制点格式：[[left_time, left_value], [right_time, right_value]]
        - left_time: 左控制点时间偏移（通常3.0，即30ms）
        - left_value: 左控制点位置偏移（根据速度计算）
        - right_time: 右控制点时间偏移（通常7.0，即70ms）
        - right_value: 右控制点位置偏移（根据速度计算）
        
        调整参数说明：
        - cp_time_left/right: 控制点的时间偏移，影响控制点的作用范围
          * 增大 → 控制点影响范围更大，曲线更平滑但可能过度
          * 减小 → 控制点影响范围更小，曲线更直接但可能不够平滑
        - cp_velocity_scale: 速度到控制点值的缩放因子
          * 增大 → 控制点值更大，曲线变化更明显
          * 减小 → 控制点值更小，曲线变化更平缓
        - cp_static_threshold: 判断静止的速度阈值
          * 增大 → 更多点被视为静止，使用默认控制点
          * 减小 → 更少点被视为静止，更多点使用动态控制点

        Args:
            trajectory: 轨迹数据列表
            frame_idx: 当前帧索引
            joint_idx: 关节索引
            cp_time_left: 左控制点时间偏移（默认3.0，即30ms）
            cp_time_right: 右控制点时间偏移（默认7.0，即70ms）
            cp_velocity_scale: 速度到控制点值的缩放因子（默认10.0）
            cp_static_threshold: 判断静止的速度阈值（默认0.01 rad/s）

        Returns:
            控制点数据 [[left_time, left_value], [right_time, right_value]]
        """
        num_frames = len(trajectory)

        if num_frames <= 1:
            return [[0, 0], [21, 0]]

        if frame_idx == num_frames - 1:
            # 最后一帧：左控制点为负（表示减速停止），右控制点为0
            return [[-21.9, 0], [0, 0]]
        elif frame_idx == 0:
            # 第一帧：使用小的正控制点（平滑启动）
            return [[cp_time_left, 0], [cp_time_right, 0]]
        else:
            # 中间帧：根据运动趋势计算控制点
            current_point = trajectory[frame_idx]
            prev_point = trajectory[frame_idx - 1]
            next_point = trajectory[frame_idx + 1] if frame_idx + 1 < num_frames else current_point

            positions = current_point.get('positions', [])
            prev_positions = prev_point.get('positions', [])
            next_positions = next_point.get('positions', [])

            if joint_idx >= len(positions):
                return [[0, 0], [21, 0]]

            current_pos = positions[joint_idx]
            prev_pos = prev_positions[joint_idx] if joint_idx < len(prev_positions) else current_pos
            next_pos = next_positions[joint_idx] if joint_idx < len(next_positions) else current_pos

            # 计算时间差
            time_diff = next_point['time'] - current_point['time']
            if time_diff <= 0:
                return [[cp_time_left, 0], [cp_time_right, 0]]

            # 计算速度趋势
            velocity = (next_pos - current_pos) / time_diff
            prev_velocity = (current_pos - prev_pos) / (current_point['time'] - prev_point['time'])

            # 根据趋势调整控制点
            if abs(velocity) < cp_static_threshold:  # 接近静止
                # 静止状态：使用默认控制点（不产生额外偏移）
                cp_data = [[cp_time_left, 0], [cp_time_right, 0]]
            else:
                # 运动状态：根据运动方向和速度设置控制点
                direction = 1 if velocity > 0 else -1
                # 限制控制点大小，避免过度
                magnitude = min(abs(velocity) * cp_velocity_scale, 10.0)

                # 左控制点：影响当前帧的运动趋势（权重较小）
                left_value = direction * magnitude * 0.3
                # 右控制点：影响到达下一帧的运动趋势（权重较大）
                right_value = direction * magnitude * 0.7

                cp_data = [[cp_time_left, left_value], [cp_time_right, right_value]]

            return cp_data

    def filter_zero_velocity_points(self, trajectory, velocity_threshold=1e-3):
        """
        按关节分别过滤速度为0的点（保留第一个和最后一个点）
        
        工作原理：
        1. 对14个手臂关节分别独立处理
        2. 每个关节判断哪些时间点需要保留（有运动的时间点）
        3. 合并所有关节需要的点，确保不遗漏任何关节的运动
        4. 始终保留第一个和最后一个点，确保轨迹完整性
        
        Args:
            trajectory: 轨迹数据列表，每个元素包含 'time', 'positions', 'velocities' 等字段
            velocity_threshold: 速度阈值（rad/s），小于此值视为静止
            
        Returns:
            过滤后的轨迹数据列表
        """
        if not trajectory or len(trajectory) <= 2:
            return trajectory

        # 步骤1: 初始化需要保留的时间点集合（使用set自动去重）
        required_time_indices = set()
        
        # 步骤2: 始终保留第一个和最后一个点（确保轨迹边界完整）
        required_time_indices.add(0)
        required_time_indices.add(len(trajectory) - 1)

        # 步骤3: 对每个关节分别处理
        # ROBAN: 23个关节 (0-22)，KUAVO: 28或29个关节
        # 获取轨迹中实际有多少个关节数据
        if trajectory:
            num_joints = len(trajectory[0].get('positions', []))
        else:
            num_joints = 14

        for joint_idx in range(num_joints):
            joint_required_indices = self._filter_joint_zero_velocity_points(
                trajectory, joint_idx, velocity_threshold
            )
            # 合并该关节需要的所有时间点
            required_time_indices.update(joint_required_indices)
        
        # 步骤4: 按时间索引排序并提取对应的轨迹点
        sorted_indices = sorted(required_time_indices)
        filtered = [trajectory[i] for i in sorted_indices]

        # 步骤5: 输出过滤统计信息
        removed_count = len(trajectory) - len(filtered)
        if removed_count > 0:
            reduction_ratio = (removed_count / len(trajectory)) * 100
            rospy.loginfo(f"Filtered trajectory: {len(trajectory)} -> {len(filtered)} points "
                         f"(removed {removed_count} zero-velocity points, {reduction_ratio:.1f}% reduction)")
        return filtered

    def _filter_joint_zero_velocity_points(self, trajectory, joint_idx, velocity_threshold=1e-3):
        """
        为单个关节判断需要保留的时间点
        
        判断逻辑：
        1. 优先使用轨迹点中的速度信息（velocities字段）
        2. 如果没有速度信息，通过位置变化和时间差计算速度
        3. 如果速度大于阈值，标记该时间点需要保留
        
        Args:
            trajectory: 轨迹数据列表
            joint_idx: 关节索引（0-13，对应14个手臂关节）
            velocity_threshold: 速度阈值（rad/s）
            
        Returns:
            需要保留的时间点索引集合
        """
        required_indices = set()
        
        # 始终保留第一个和最后一个点
        required_indices.add(0)
        required_indices.add(len(trajectory) - 1)
        
        # 遍历中间的所有点（跳过第一个和最后一个）
        for i in range(1, len(trajectory) - 1):
            point = trajectory[i]
            velocities = point.get('velocities', [])
            
            has_motion = False
            
            # 方法1: 优先使用轨迹点中的速度信息
            if velocities and len(velocities) > joint_idx:
                velocity = velocities[joint_idx]
                if abs(velocity) > velocity_threshold:
                    has_motion = True
            else:
                # 方法2: 如果没有速度信息，通过位置变化计算速度
                prev_point = trajectory[i - 1]
                prev_positions = prev_point.get('positions', [])
                current_positions = point.get('positions', [])
                prev_time = prev_point.get('time', 0)
                current_time = point.get('time', 0)
                
                # 检查数据有效性
                if (current_time > prev_time and 
                    joint_idx < len(current_positions) and 
                    joint_idx < len(prev_positions)):
                    dt = current_time - prev_time
                    if dt > 0:
                        # 计算速度：位置变化 / 时间差
                        position_change = abs(current_positions[joint_idx] - prev_positions[joint_idx])
                        velocity = position_change / dt
                        if velocity > velocity_threshold:
                            has_motion = True
            
            # 如果该关节在此时间点有运动，标记需要保留
            if has_motion:
                required_indices.add(i)
        
        return required_indices

    def filter_small_position_change_points(self, trajectory, position_threshold=1e-3):
        """
        按关节分别过滤“位置变化很小”的点（保留第一个和最后一个点）

        ⚠ 注意：
        - 这里虽然参数名还是 position_threshold，但实际上作为“位置差异阈值”使用
        - 即：当某关节在相邻保留点之间的位置变化量 > 该阈值 时，才保留中间点
        - 这样可以过滤掉小抖动 / 噪声，但保留慢速的大范围运动

        工作原理：
        1. 对 14 个手臂关节分别独立处理（索引 0~13）
        2. 每个关节判断：与该关节上一次“保留点”的位置差是否超过阈值
        3. 合并所有关节需要保留的时间点（取并集）
        4. 始终保留第一个和最后一个点，确保轨迹完整性

        Args:
            trajectory: 轨迹数据列表，每个元素包含 'time', 'positions', 'velocities' 等字段
            velocity_threshold: 实际上作为“位置差异阈值”，单位与 positions 一致（rad 或 deg）

        Returns:
            过滤后的轨迹数据列表
        """
        if not trajectory or len(trajectory) <= 2:
            return trajectory


        # 步骤1: 初始化需要保留的时间点集合（使用 set 自动去重）
        required_time_indices = set()

        # 步骤2: 始终保留第一个和最后一个点（确保轨迹边界完整）
        required_time_indices.add(0)
        required_time_indices.add(len(trajectory) - 1)

        # 步骤3: 对每个关节分别处理
        # ROBAN: 23个关节 (0-22)，KUAVO: 28或29个关节
        # 获取轨迹中实际有多少个关节数据
        if trajectory:
            num_joints = len(trajectory[0].get('positions', []))
        else:
            num_joints = 14

        for joint_idx in range(num_joints):
            joint_required_indices = self._filter_joint_small_position_change_points(
                trajectory, joint_idx, position_threshold
            )
            # 合并该关节需要的所有时间点
            required_time_indices.update(joint_required_indices)

        # 步骤4: 按时间索引排序并提取对应的轨迹点
        sorted_indices = sorted(required_time_indices)
        filtered = [trajectory[i] for i in sorted_indices]

        # 步骤5: 输出过滤统计信息
        removed_count = len(trajectory) - len(filtered)
        if removed_count > 0:
            reduction_ratio = (removed_count / len(trajectory)) * 100
            rospy.loginfo(
                f"Filtered trajectory (by position): {len(trajectory)} -> {len(filtered)} points "
                f"(removed {removed_count} points, {reduction_ratio:.1f}% reduction, "
                f"position_threshold={position_threshold})"
            )
        return filtered

    def _filter_joint_small_position_change_points(self, trajectory, joint_idx, position_threshold=1e-3):
        """
        为单个关节判断需要保留的时间点（按位置差异判断，而不是速度）

        判断逻辑：
        1. 不再看速度，只看该关节位置相对于“上一次保留点”的变化量
        2. 如果 |pos[i] - pos[last_kept_idx]| > position_threshold，则保留该时间点 i
        3. 始终保留第一个和最后一个点

        Args:
            trajectory: 轨迹数据列表
            joint_idx: 关节索引（0-13，对应 14 个手臂关节）
            position_threshold: 实际上作为“位置差异阈值”，单位与 positions 一致

        Returns:
            需要保留的时间点索引集合（set[int]）
        """
        n = len(trajectory)
        if n <= 2:
            return {0, max(0, n - 1)}


        required_indices = set()

        # 始终保留第一个和最后一个点
        required_indices.add(0)
        required_indices.add(n - 1)

        # 获取第一个点的该关节位置，作为初始 last_kept
        first_positions = trajectory[0].get('positions', [])
        if joint_idx >= len(first_positions):
            # 该关节在轨迹中没有数据，直接只保留首尾
            return required_indices

        last_kept_idx = 0
        last_kept_pos = first_positions[joint_idx]

        # 遍历中间的所有点（跳过第一个和最后一个）
        for i in range(1, n - 1):
            point = trajectory[i]
            positions = point.get('positions', [])

            # 检查是否有该关节的数据
            if joint_idx >= len(positions):
                continue

            current_pos = positions[joint_idx]
            pos_diff = abs(current_pos - last_kept_pos)

            # 如果该关节从上次保留点到现在已经走了“足够大”的距离 → 保留该点
            if pos_diff > position_threshold:
                required_indices.add(i)
                last_kept_idx = i
                last_kept_pos = current_pos

        return required_indices


    def process_planned_trajectory(self, sampling_rate=None, generated_trajectory=None, filter_small_position_change=True):
        """
        处理规划后的轨迹，可选地进行重新采样和过滤速度为0的点
        
        Args:
            sampling_rate: 自定义采样率（秒），None表示不重新采样
            generated_trajectory: 生成的轨迹数据列表
            filter_zero_velocity: 是否过滤速度为0的点
            
        Returns:
            处理后的轨迹数据列表，如果输入为空则返回None
        """
        if not generated_trajectory:
            return None

        # 按时间排序
        sorted_traj = sorted(generated_trajectory, key=lambda x: x['time'])

        if filter_small_position_change:
            sorted_traj = self.filter_small_position_change_points(sorted_traj, position_threshold=0.005)
        

        start_time = sorted_traj[0]['time'] if sorted_traj else 0.0

        # 如果指定了采样率，进行重新采样
        if sampling_rate is not None:
            processed = []
            next_sample_time = start_time
            for point in sorted_traj:
                if point['time'] >= next_sample_time:
                    new_point = point.copy()
                    processed.append(new_point)
                    next_sample_time += sampling_rate
            return processed
        else:
            return sorted_traj

    def save_results(self, output_dir='./bezier_results', save_tact=True, custom_sampling_rate=0.5, save_sampling_tact=True):
        """
        保存结果到文件

        Args:
            output_dir: 输出目录
            save_tact: 是否保存tact文件
            custom_sampling_rate: 自定义采样率（秒）
            save_sampling_tact: 是否保存采样结果为tact文件
        """
        rospy.loginfo(f"Saving results to {output_dir}...")

        # 创建输出目录
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # 保存为tact文件
        if save_tact:
            # 合并手臂和手部轨迹（包含手指和头部数据）
            merged_trajectory = None
            if self.generated_trajectory_arm:
                merged_trajectory = self.generated_trajectory_arm
            
            # 1. 如果使用了自定义采样，保存采样结果的TACT文件
            if save_sampling_tact and custom_sampling_rate is not None and merged_trajectory:
                processed_trajectory = self.process_planned_trajectory(
                    custom_sampling_rate, 
                    merged_trajectory,
                    filter_small_position_change=True
                )
                if processed_trajectory:
                    # 使用手臂轨迹的keyframe_counter计算finish
                    original_keyframe_counter = self.keyframe_counter
                    self.keyframe_counter = self.arm_keyframe_counter
                    
                    sampling_tact_file = os.path.join(output_dir, f'sampling_trajectory_{timestamp}.tact')
                    self.save_to_tact_file(sampling_tact_file, trajectory_data=processed_trajectory)
                    rospy.loginfo(f"Sampling results saved as TACT file: {sampling_tact_file}")
                    
                    # 恢复keyframe_counter
                    self.keyframe_counter = original_keyframe_counter
                else:
                    rospy.logwarn("Failed to process trajectory for sampling TACT file")

        rospy.loginfo(f"Results saved successfully to {output_dir}")

    def _process_trajectory_for_data(self, control_data, timestamp_data, sampling_rate, smoothing_factor, data_name="data"):
        """
        处理单个数据源的轨迹生成流程（手臂或手部）
        
        Args:
            control_data: 控制数据列表
            timestamp_data: 时间戳列表
            sampling_rate: 预处理采样率
            smoothing_factor: 平滑因子
            data_name: 数据名称（用于日志）
            
        Returns:
            bool: 是否成功
        """
        rospy.loginfo(f"Processing {data_name} trajectory...")
        
        # 预处理数据（降采样）
        processed_data, processed_timestamps = self.preprocess_data(sampling_rate, control_data, timestamp_data)
        if processed_data is None:
            rospy.logerr(f"Failed to preprocess {data_name} data")
            return False

        # 生成贝塞尔控制点
        # 检查是否有手指数据（dexhand/state话题是否有数据）
        # 如果是处理手部数据，说明有手指数据；如果是处理手臂数据，需要检查hand_sensors_data
        if data_name == "hand":
            has_finger_data = True  # 处理手部数据时，肯定有手指数据
        else:
            has_finger_data = len(self.hand_sensors_data) > 0  # 处理手臂数据时，检查是否有手指数据
        
        if not self.generate_bezier_control_points(smoothing_factor, processed_data, processed_timestamps, has_finger_data):
            rospy.logerr(f"Failed to generate Bezier control points for {data_name}")
            return False

        # 重置轨迹监听器状态
        self.generated_trajectory = []
        self.planning_complete = False
        self.keyframe_counter = 0

        # 设置轨迹监听器
        self.setup_trajectory_listeners()
        rospy.sleep(1.0)  # 等待订阅器建立

        # 调用C++贝塞尔曲线规划器
        req = self.create_bezier_request()
        if not self.call_bezier_planner(req):
            rospy.logerr(f"Failed to call Bezier planner for {data_name}")
            return False

        # 等待轨迹生成完成
        rospy.loginfo(f"Waiting for C++ planner to generate {data_name} trajectory...")
        max_wait_time = processed_timestamps[-1] + 10.0 if processed_timestamps else 10.0
        wait_start = rospy.Time.now()

        while (rospy.Time.now() - wait_start).to_sec() < max_wait_time:
            if self.planning_complete:
                rospy.loginfo(f"{data_name.capitalize()} trajectory generation completed!")
                break
            rospy.sleep(0.1)
        else:
            rospy.logwarn(f"{data_name.capitalize()} trajectory generation timed out, using available data")

        return True

    def run(self, bag_file_path, sampling_rate=0.1, smoothing_factor=0.3, output_dir='./bezier_results',
           save_tact=True, custom_sampling_rate=0.5, get_raw_trajectory=False, save_sampling_tact=True):
        """
        运行完整的处理流程：原始数据 → C++规划器 → 规划曲线 → 自定义采样

        Args:
            bag_file_path: rosbag文件路径
            sampling_rate: 数据预处理采样率（秒）
            smoothing_factor: 贝塞尔曲线平滑因子
            output_dir: 输出目录
            save_tact: 是否保存tact文件
            custom_sampling_rate: 自定义轨迹采样率（None表示使用规划器原始采样）
            get_raw_trajectory: 是否只获取原始规划轨迹而不进行额外处理
            save_sampling_tact: 是否保存采样结果为tact文件

        Returns:
            bool: 处理是否成功
        """
        rospy.loginfo("Starting rosbag to Bezier planner processing...")
        rospy.loginfo("Workflow: Raw Data → C++ Planner → Planned Curve → Custom Sampling")

        # 保存输出目录到实例变量，供后续使用
        self.output_dir = output_dir

        # 步骤1: 加载rosbag原始数据
        if not self.load_rosbag_data(bag_file_path):
            rospy.logerr("Failed to load rosbag data")
            return False

        # 步骤1.5: 合并手部数据到手臂数据中（按时间戳对齐）
        # 如果有手部数据，将手指部分合并到joint_data中
        if len(self.hand_sensors_data) > 0 and len(self.joint_data) > 0:
            # 根据机器人类型确定手指索引范围
            if self.is_roban:
                finger_start_idx = 8   # ROBAN: 手指索引8-19
                finger_end_idx = 20
            else:
                finger_start_idx = 14  # KUAVO: 手指索引14-25
                finger_end_idx = 26

            merged_count = 0
            hand_idx = 0  # 手部数据的当前索引

            for arm_idx in range(len(self.joint_data)):
                arm_time = self.arm_timestamp_data[arm_idx]

                # 找到时间最接近的手部数据点
                # 向前搜索，直到找到时间戳大于或等于arm_time的手部数据
                while hand_idx < len(self.hand_timestamp_data) - 1:
                    current_hand_time = self.hand_timestamp_data[hand_idx]
                    next_hand_time = self.hand_timestamp_data[hand_idx + 1]

                    # 如果下一个手部时间点更接近arm_time，则移动到下一个
                    if abs(next_hand_time - arm_time) < abs(current_hand_time - arm_time):
                        hand_idx += 1
                    else:
                        break

                # 检查时间差是否在合理范围内（例如100ms）
                time_diff = abs(self.hand_timestamp_data[hand_idx] - arm_time)
                if time_diff > 0.1:  # 时间差超过100ms，跳过
                    continue

                # 检查数据长度
                if len(self.joint_data[arm_idx]) >= finger_end_idx and len(self.hand_sensors_data[hand_idx]) >= finger_end_idx:
                    # 将手部数据中的手指部分（索引14-25）复制到joint_data对应位置
                    for finger_idx in range(finger_start_idx, finger_end_idx):
                        self.joint_data[arm_idx][finger_idx] = self.hand_sensors_data[hand_idx][finger_idx]
                    merged_count += 1

            if merged_count > 0:
                rospy.loginfo(f"Merged finger data by timestamp alignment: {merged_count}/{len(self.joint_data)} points")
            else:
                rospy.logwarn("Hand sensors data exists but could not be merged (timestamp mismatch or data length mismatch)")
        elif len(self.hand_sensors_data) > 0:
            rospy.logwarn("Hand sensors data exists but no joint_data to merge into")

        # 步骤2: 处理手臂轨迹
        if len(self.joint_data) > 0:
            if not self._process_trajectory_for_data(
                self.joint_data, self.arm_timestamp_data, 
                sampling_rate, smoothing_factor, "arm"
            ):
                rospy.logerr("Failed to process arm trajectory")
                return False
            self.generated_trajectory_arm = self.generated_trajectory.copy()
            self.arm_keyframe_counter = self.keyframe_counter  # 保存手臂轨迹的keyframe_counter
            
        else:
            rospy.logwarn("No arm joint data found in rosbag")

        # 步骤3: 保存结果
        self.save_results(
            output_dir, 
            save_tact, 
            custom_sampling_rate if not get_raw_trajectory else None, 
            save_sampling_tact
        )

        rospy.loginfo("Processing completed successfully!")
        return True

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Convert rosbag data to Bezier curve trajectories using C++ planner')
    parser.add_argument('bag_file', help='Path to the rosbag file')
    parser.add_argument('--sampling_rate', type=float, default=0.1,
                       help='Data preprocessing sampling rate in seconds (default: 0.1)')
    parser.add_argument('--smoothing_factor', type=float, default=0.3,
                       help='Smoothing factor for control points (default: 0.3)')
    parser.add_argument('--output_dir', default='./bezier_results',
                       help='Output directory for results (default: ./bezier_results)')
    parser.add_argument('--no_tact', action='store_true',
                       help='Disable TACT file generation')
    parser.add_argument('--custom_sampling', type=float, default=None,
                       help='Custom sampling rate for the planned trajectory in seconds (default: None)')
    parser.add_argument('--get_raw_trajectory', action='store_true',
                       help='Get raw trajectory from C++ planner without additional processing')
    parser.add_argument('--no_sampling_tact', action='store_true',
                       help='Disable saving sampling results as TACT file')

    args = parser.parse_args()

    # 创建处理器实例
    processor = RosbagToBezierPlanner()

    # 运行处理流程
    try:
        success = processor.run(
            args.bag_file,
            args.sampling_rate,
            args.smoothing_factor,
            args.output_dir,
            not args.no_tact,
            args.custom_sampling,
            args.get_raw_trajectory,
            not args.no_sampling_tact
        )
        if success:
            rospy.loginfo("Processing completed successfully!")
            if processor.generated_trajectory:
                rospy.loginfo(f"Generated {len(processor.generated_trajectory)} trajectory points")
        else:
            rospy.logerr("Processing failed!")
    except KeyboardInterrupt:
        rospy.loginfo("Processing interrupted by user")
    except Exception as e:
        rospy.logerr(f"Processing failed with error: {e}")

if __name__ == "__main__":
    main()
