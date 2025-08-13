"""
Utility functions and classes for Pico VR device integration with ROS.

Movement Detection Modes:
1. Threshold-based detection (default): Real-time detection based on position and angle thresholds
2. Complete action detection: Detects complete "ground-lift-ground" actions for more accurate human motion following

Usage Examples:

# Enable complete action detection
transformer = KuavoPicoInfoTransformer(...)
transformer.enable_complete_action_detection()

# Or set detection mode manually
transformer.set_detection_mode('complete_action')

# Configure complete action parameters
transformer.set_complete_action_parameters({
    'lift_threshold': 0.05,      # 5cm lift threshold
    'ground_threshold': 0.02,    # 2cm ground threshold
    'min_action_duration': 0.3,  # 0.3s minimum duration
    'max_action_duration': 2.0,  # 2.0s maximum duration
    'min_horizontal_movement': 0.05  # 5cm minimum horizontal movement
})

# Switch back to threshold detection
transformer.enable_threshold_detection()

# Get detection information
info = transformer.get_detection_info()
print(f"Current mode: {info['current_mode']}")
"""

import os
import socket
import json
import time
import math
import rospy
import numpy as np
import quaternion
import threading
import queue
import concurrent.futures
from dataclasses import dataclass
from functools import lru_cache, partial
from .sat_utils import RotatingRectangle
from robot_tool import KuavoRobotTools
from tf.transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    euler_from_quaternion,
    quaternion_from_euler,
    translation_matrix
)
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Float32MultiArray, Bool
from kuavo_msgs.msg import (
    picoPoseInfo, picoPoseInfoList,
    footPoseTargetTrajectories, footPose
)
from ocs2_msgs.msg import mpc_observation, mpc_flattened_controller
from noitom_hi5_hand_udp_python.msg import JoySticks # TODO: 从noitom_hi5_hand_udp_python导入是临时的，等待该消息迁移到 kuavo_msgs
from enum import IntEnum
from common.logger import SDKLogger
from pydrake.all import (
    RotationMatrix,
    Quaternion as DrakeQuaternion,
    RollPitchYaw,
)

from .net_utils import get_localip_and_broadcast_ips

# Module-level constants for bone names and indices
BODY_TRACKER_ROLES = [
    "Pelvis", "LEFT_HIP", "RIGHT_HIP", "SPINE1", "LEFT_KNEE", "RIGHT_KNEE",
    "SPINE2", "LEFT_ANKLE", "RIGHT_ANKLE", "SPINE3", "LEFT_FOOT", "RIGHT_FOOT",
    "NECK", "LEFT_COLLAR", "RIGHT_COLLAR", "HEAD", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HAND", "RIGHT_HAND"
]

INDEX_TO_BONE_NAME = {index: name for index, name in enumerate(BODY_TRACKER_ROLES)}
BONE_NAME_TO_INDEX = {name: index for index, name in enumerate(BODY_TRACKER_ROLES)}

LEFT_ARM_IDXS = [BODY_TRACKER_ROLES.index(name) for name in [
    "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST", "LEFT_HAND"
]]

RIGHT_ARM_IDXS = [BODY_TRACKER_ROLES.index(name) for name in [
    "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST", "RIGHT_HAND"
]]

ALIGN_TO_ROBOT_URDF_JOINTS = [
    "LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_ELBOW", "RIGHT_ELBOW",
    "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HAND", "RIGHT_HAND"
]

CONTROL_MODES = ["WholeBody", "UpperBody", "LowerBody"]

class RobotInfoBroadcaster:
    """Broadcast robot information over network."""
    
    def __init__(self):
        self.robot_name = os.getenv("ROBOT_NAME", "KUAVO")
        # self.robot_ip = get_wifi_ip()
        # self.broadcast_ip = f"{self.robot_ip.rsplit('.', 1)[0]}.255"
        self.ip_pairs = get_localip_and_broadcast_ips()
        self.broadcast_port = 8443
        self.is_paused = False  # 添加暂停标志
        self.broadcast_condition = threading.Condition()
        self.eef_type = rospy.get_param('end_effector_type', None)
        self.robot_info = {
            "data": {
                "robot_name": self.robot_name,
                "robot_ip": '',
                "eef_type": self.eef_type,
            }
        }

    #     self.start_test_pause_thread()

    # def start_test_pause_thread(self):
    #     def test_pause():
    #         time.sleep(15)
    #         self.pause_broadcast()
        
    #     test_thread = threading.Thread(target=test_pause, daemon=True)
    #     test_thread.start()
    #     SDKLogger.info("Started test thread - will pause broadcast in 30 seconds")

    
    def pause_broadcast(self):
        """暂停广播."""
        with self.broadcast_condition:
            self.is_paused = True
            self.broadcast_condition.notify_all()
        SDKLogger.info("Robot info broadcast paused")
    
    def resume_broadcast(self):
        """恢复广播."""
        with self.broadcast_condition:
            self.is_paused = False
            self.broadcast_condition.notify_all()
        SDKLogger.info("Robot info broadcast resumed")
    

    def broadcast(self):
        """Broadcast robot information."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            for local_ip, broadcast_ip in self.ip_pairs:
                SDKLogger.info(f"Broadcast IP: {local_ip}:{self.broadcast_port}")            
            while True:
                try:
                    # 使用条件信号等待广播恢复或超时
                    with self.broadcast_condition:
                        # 如果暂停了，等待恢复信号
                        while self.is_paused:
                            self.broadcast_condition.wait()
                        
                        # 广播机器人信息
                        for local_ip, broadcast_ip in self.ip_pairs:
                            self.robot_info["data"]["robot_ip"] = local_ip
                            message = json.dumps(self.robot_info).encode("utf-8")
                            sock.sendto(message, (broadcast_ip, self.broadcast_port))
                            SDKLogger.debug(f"Broadcasting to {broadcast_ip}:{self.broadcast_port}")
                    time.sleep(1)
                except Exception as e:
                    SDKLogger.error(f"Error broadcasting robot info: {e}")
                    break
        finally:
            sock.close()


@dataclass
class HeadBodyPose:
    """Data class for head and body pose information."""
    head_pitch: float = 0.0
    head_yaw: float = 0.0
    body_yaw: float = 0.0
    body_x: float = 0.0
    body_y: float = 0.0
    body_pitch: float = 6 * np.pi / 180.0
    body_height: float = 0.74
    body_roll: float = 0.0


class KuavoPicoInfoTransformer():
    """Class for transforming Pico VR device information to robot coordinates."""
    
    def __init__(self, model_path: str, tf_broadcaster=None, bone_frame_publisher=None):
        """Initialize the KuavoPicoInfoTransformer."""
        self.model_path = model_path
        
        # Store external dependencies
        self.tf_broadcaster = tf_broadcaster
        self.bone_frame_publisher = bone_frame_publisher
        
        # Initialize bone names and indices
        self._init_bone_names()
        
        # Initialize gesture detection variables
        self._init_gesture_vars()
        
        # Initialize poses and matrices
        self._init_poses_and_matrices()
        
        # Initialize arm poses
        self._init_arm_poses()
        
        # # Initialize shoulder rotation matrices
        # self._init_shoulder_matrices()
        
        # Initialize publishers
        self._init_publishers()

        # Initialize arm lengths
        self._init_arm_lengths()

        # Initialize control mode
        self._init_control_mode()

        # Initialize torso control mode
        self._init_torso_control_mode()

        self._init_movement_detector()

        self._init_predifined_actions()

        self.tools = KuavoRobotTools()

    def _init_predifined_actions(self) -> None:
        """Initialize predefined actions."""
        self.predifined_actions = {
            "forward_step_left": {
                "type": "single",
                "foot_type": 0,  # Left foot
                "movement": [0.5, 0.0, 0.0, 0.0, 0.4]  # [x, y, z, yaw]
            },
            # "backward_step_left": {
            #     "type": "single", 
            #     "foot_type": 0,
            #     "movement": [-0.4, 0.0, 0.0, 0.0, 0.4]
            # },
            "forward_step_right": {
                "type": "single",
                "foot_type": 1,  
                "movement": [0.5, 0.0, 0.0, 0.0, 0.4]  # [x, y, z, yaw]
            },
            # "backward_step_right": {
            #     "type": "single", 
            #     "foot_type": 1,
            #     "movement": [-0.4, 0.0, 0.0, 0.0, 0.4]
            # },
            # "left_step": {
            #     "type": "single",
            #     "foot_type": 0, 
            #     "movement": [0.0, 0.6, 0.0, 0.0, 0.4]
            # },
            # "left_step_back": {
            #     "type": "single",
            #     "foot_type": 0, 
            #     "movement": [0.0, -0.3, 0.0, 0.0, 0.4]  
            # },
            # "right_step": {
            #     "type": "single",
            #     "foot_type": 1,  # Right foot
            #     "movement": [0.0, -0.6, 0.0, 0.0, 0.4]
            # },
            # "right_step_back": {
            #     "type": "single",
            #     "foot_type": 1,
            #     "movement": [0.0, 0.3, 0.0, 0.0, 0.4]
            # },
            # "turn_left": {
            #     "type": "single",
            #     "foot_type": 0,
            #     "movement": [0.0, 0.3, 0.0, 90.0, 0.4]  # 30 degrees left
            # },
            # "turn_left_back": {
            #     "type": "single",
            #     "foot_type": 0,
            #     "movement": [0.0, 0.3, 0.0, -30.0, 0.4]  # 30 degrees left
            # },
            # "turn_right": {
            #     "type": "single", 
            #     "foot_type": 1,
            #     "movement": [0.0, 0.3, 0.0, -30.0, 0.4]  # 30 degrees right
            # },
            # "turn_right_back": {
            #     "type": "single",
            #     "foot_type": 1,
            #     "movement": [0.0, 0.3, 0.0, 30.0, 0.4]  # 30 degrees right
            # }
        }

    def _init_bone_names(self) -> None:
        """Initialize bone names and their indices."""
        self.bone_names = BODY_TRACKER_ROLES
        
        self.left_arm_idxs = LEFT_ARM_IDXS
        
        self.right_arm_idxs = RIGHT_ARM_IDXS
        
        self.bone_name_to_index = BONE_NAME_TO_INDEX
        self.index_to_bone_name = INDEX_TO_BONE_NAME

    def _init_gesture_vars(self) -> None:
        """Initialize gesture detection variables."""
        self.is_running = True
        self.left_finger_joints = None
        self.right_finger_joints = None

    def _init_poses_and_matrices(self) -> None:
        """Initialize poses and transformation matrices."""
        self.head_body_pose = Twist()
        self.cached_matrices = np.tile(np.eye(4), (len(self.bone_names), 1, 1))
        self.T_spine3_to_base = translation_matrix([0, 0, 0.34])  # base_link is 0.34m below SPINE3
        

    def _init_arm_poses(self) -> None:
        """Initialize arm pose variables."""
        self.left_hand_pose = None
        self.right_hand_pose = None
        self.left_elbow_pos = None
        self.right_elbow_pos = None

    def _init_publishers(self) -> None:
        """Initialize ROS publishers."""
        self.shoulder_angle_puber = rospy.Publisher('/pico_debug/shoulder_angle', Float32MultiArray, queue_size=10)
        self.chest_axis_puber = rospy.Publisher('/pico_debug/chest_axis', Float32MultiArray, queue_size=10)
        self.head_body_pose_puber = rospy.Publisher('/cmd_pose', Twist, queue_size=10)
        self.vis_pub = True
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.marker_pub_right = rospy.Publisher("visualization_marker_right", Marker, queue_size=10)
        self.marker_pub_elbow = rospy.Publisher("visualization_marker/elbow", Marker, queue_size=10)
        self.marker_pub_elbow_right = rospy.Publisher("visualization_marker_right/elbow", Marker, queue_size=10)
        self.marker_pub_shoulder = rospy.Publisher("visualization_marker/shoulder", Marker, queue_size=10)
        self.marker_pub_shoulder_right = rospy.Publisher("visualization_marker_right/shoulder", Marker, queue_size=10)
        self.marker_pub_shoulder_pico = rospy.Publisher("visualization_marker/shoulder_pico", Marker, queue_size=10)
        self.marker_pub_shoulder_pico_right = rospy.Publisher("visualization_marker_right/shoulder_pico", Marker, queue_size=10)
        self.marker_pub_chest = rospy.Publisher("visualization_marker_chest", Marker, queue_size=10)
        self.foot_pose_pub = rospy.Publisher("foot_pose", Float32MultiArray, queue_size=10)

    def _init_arm_lengths(self) -> None:
        """Initialize arm length variables."""
        self.upper_arm_length = rospy.get_param("/pico/upper_arm_length")/100.0
        self.lower_arm_length = rospy.get_param("/pico/lower_arm_length")/100.0
        self.shoulder_width = rospy.get_param("/pico/shoulder_width")/100.0
        
        self.human_upper_arm_length = 0.3
        self.human_lower_arm_length = 0.3
        self.human_shoulder_width = 0.4
        
        self.measure_arm_length = True
        self.arm_length_num = 30
        self.left_upper_arm_lengths = []
        self.left_lower_arm_lengths = []
        self.right_upper_arm_lengths = []
        self.right_lower_arm_lengths = []
        self.avg_left_upper_arm_length = None
        self.avg_left_lower_arm_length = None
        self.avg_right_upper_arm_length = None
        self.avg_right_lower_arm_length = None

    def _init_control_mode(self) -> None:
        """Initialize control mode."""
        self.control_mode = CONTROL_MODES[0]

    def set_control_mode(self, mode: str) -> None:
        """Set control mode."""
        if mode in CONTROL_MODES:
            self.control_mode = mode
        else:
            SDKLogger.warning(f"Invalid control mode: {mode}")
    
    def _init_torso_control_mode(self) -> None:
        """Initialize torso control mode."""
        self.control_torso_mode = False

    def set_control_torso_mode(self, mode: bool) -> None:
        """Set torso control mode."""
        self.control_torso_mode = mode

    def stop_arm_length_measurement(self) -> None:
        """Stop arm length measurement and switch to using average values."""
        if self.measure_arm_length:
            # Calculate final averages if we have any data
            if len(self.left_upper_arm_lengths) > 0:
                self.avg_left_upper_arm_length = sum(self.left_upper_arm_lengths) / len(self.left_upper_arm_lengths)
                self.avg_left_lower_arm_length = sum(self.left_lower_arm_lengths) / len(self.left_lower_arm_lengths)
            if len(self.right_upper_arm_lengths) > 0:
                self.avg_right_upper_arm_length = sum(self.right_upper_arm_lengths) / len(self.right_upper_arm_lengths)
                self.avg_right_lower_arm_length = sum(self.right_lower_arm_lengths) / len(self.right_lower_arm_lengths)
            
            self.measure_arm_length = False
            SDKLogger.info(f"Arm length measurement completed. Left upper: {self.avg_left_upper_arm_length:.3f}, lower: {self.avg_left_lower_arm_length:.3f}, "
                          f"Right upper: {self.avg_right_upper_arm_length:.3f}, lower: {self.avg_right_lower_arm_length:.3f}")

    def start_arm_length_measurement(self) -> None:
        """Start arm length measurement mode."""
        self.measure_arm_length = True
        # Clear previous measurements
        self.left_upper_arm_lengths.clear()
        self.left_lower_arm_lengths.clear()
        self.right_upper_arm_lengths.clear()
        self.right_lower_arm_lengths.clear()
        SDKLogger.info("Started arm length measurement mode")

    def _init_movement_detector(self) -> None:
        # Simple movement detection based on initial state
        self.movement_detector = {
            'horizontal_threshold': 0.2,    # 10cm position threshold
            'vertical_threshold': 0.01,    # 5cm vertical threshold
            'angle_threshold': 20.0,        # 20 degrees angle threshold
            'step_length': 0.2,            # 10cm step length
            'initial_left_foot_pose': [0.0, 0.1, 0.0, 0.0],
            'initial_right_foot_pose': [0.0, -0.1, 0.0, 0.0],
            'initial_body_pose': None,      # Initial body pose reference
            
            # 新增：完整动作检测器参数
            'detection_mode': 'complete_action',  # 'threshold' 或 'complete_action'
            'complete_action': {
                'lift_threshold': 0.03,     # 5cm 抬起阈值，认为脚离开地面
                'ground_threshold': 0.01,   # 2cm 地面阈值，认为脚接触地面
                'min_action_duration': 0.3, # 最小动作时长 0.3秒
                'max_action_duration': 10.0, # 最大动作时长 2.0秒
                'action_buffer_size': 50,   # 动作缓冲区大小
                'min_horizontal_movement': 0.05,  # 最小水平移动距离 5cm
                
                # 新增：自适应阈值调整参数
                'adaptive_threshold_enabled': True,  # 是否启用自适应阈值
                'calibration_samples': 50,  # 校准样本数量
                'adaptive_lift_offset': 0.02,  # 相对于基准高度的抬起偏移量
                'adaptive_ground_offset': 0.005,  # 相对于基准高度的地面偏移量
            },
            
            # 新增：并行检测配置参数
            'parallel_detection': {
                'enabled': True,           # 是否启用并行检测
                'timeout_ms': 50,          # 检测超时时间（毫秒）
                'max_workers': 2,          # 最大工作线程数
                'precise_timing': True,    # 是否使用精确时间戳
                'debug_mode': False,       # 调试模式
            }
        }
        
        # 初始化完整动作检测器状态
        self.complete_action_detector = {
            'left': {
                'state': 'ground',  # 'ground', 'lifted', 'action_complete'
                'lift_time': None,
                'ground_time': None,
                'start_pose': None,
                'end_pose': None,
                'pose_history': [],
                'action_duration': 0.0,
                # 新增：自适应校准相关
                'calibration_samples': [],  # 校准样本
                'base_height': None,  # 基准高度
                'adaptive_lift_threshold': None,  # 自适应抬起阈值
                'adaptive_ground_threshold': None,  # 自适应地面阈值
                'calibration_complete': False,
                # 是否使用实际脚部数据
                'use_real_foot_data': True,
            },
            'right': {
                'state': 'ground',
                'lift_time': None,
                'ground_time': None,
                'start_pose': None,
                'end_pose': None,
                'pose_history': [],
                'action_duration': 0.0,
                # 新增：自适应校准相关
                'calibration_samples': [],
                'base_height': None,
                'adaptive_lift_threshold': None,
                'adaptive_ground_threshold': None,
                'calibration_complete': False,
                # 是否使用实际脚部数据
                'use_real_foot_data': True,
            }
        }
        
        # Initialize trajectory queue and thread management
        self.queue_lock = threading.Lock()
        self.trajectory_queue = queue.Queue(maxsize=10)  # Limit queue size to prevent memory issues
        self.movement_queue = queue.Queue(maxsize=1)  # Limit queue size to prevent memory issues
        self.active_queue = None
        self.is_running = True
        self.trajectory_executor_thread = None
        self.trajectory_executor_started = False  # 新增标志
        self.mode_number = {
            'FF': 0, 'FH': 1, 'FT': 2, 'FS': 3, 'HF': 4, 'HH': 5, 'HT': 6, 'HS': 7, 
            'TF': 8, 'TH': 9, 'TT': 10, 'TS': 11, 'SF': 12, 'SH': 13, 'ST': 14, 'SS': 15
        }
        self.mpc_mode_lock = threading.Lock()
        self.mpc_mode = None
        self.mpc_obs_sub = rospy.Subscriber('/humanoid_mpc_observation', mpc_observation, self.mpc_observation_callback)
        
        # Initialize foot controller instance
        self.foot_controller = KuavoSingleFootController(self)
        
        # self._start_trajectory_executor()  # 不再这里直接启动

        # Initialize parallel foot detector
        self.parallel_detector = ParallelFootDetector(self)

    def set_movement_detector(self, movement_detector: dict) -> None:
        """Set movement detector."""
        self.movement_detector = movement_detector

    def set_detection_mode(self, mode: str) -> None:
        """Set detection mode.
        
        Args:
            mode: 'threshold' for real-time threshold-based detection
                  'complete_action' for ground-lift-ground complete action detection
        """
        if mode in ['threshold', 'complete_action']:
            self.movement_detector['detection_mode'] = mode
            SDKLogger.info(f"Detection mode set to: {mode}")
        else:
            SDKLogger.warning(f"Invalid detection mode: {mode}. Using 'threshold' mode.")
            self.movement_detector['detection_mode'] = 'threshold'

    def get_detection_mode(self) -> str:
        """Get current detection mode."""
        return self.movement_detector.get('detection_mode', 'threshold')

    def reset_complete_action_detector(self, pose_type: str = None) -> None:
        """Reset complete action detector state.
        
        Args:
            pose_type: 'left', 'right', 'body', or None for all
        """
        if pose_type is None:
            # Reset all
            for key in self.complete_action_detector:
                self.complete_action_detector[key] = {
                    'state': 'ground',
                    'lift_time': None,
                    'ground_time': None,
                    'start_pose': None,
                    'end_pose': None,
                    'pose_history': [],
                    'action_duration': 0.0,
                    # 新增：自适应校准相关
                    'calibration_samples': [],
                    'base_height': None,
                    'adaptive_lift_threshold': None,
                    'adaptive_ground_threshold': None,
                    'calibration_complete': False,
                    # 是否使用实际脚部数据
                    'use_real_foot_data': False,
                }
            SDKLogger.info("Reset all complete action detectors")
        elif pose_type in self.complete_action_detector:
            # Reset specific detector
            self.complete_action_detector[pose_type] = {
                'state': 'ground',
                'lift_time': None,
                'ground_time': None,
                'start_pose': None,
                'end_pose': None,
                'pose_history': [],
                'action_duration': 0.0,
                # 新增：自适应校准相关
                'calibration_samples': [],
                'base_height': None,
                'adaptive_lift_threshold': None,
                'adaptive_ground_threshold': None,
                'calibration_complete': False,
            }
            SDKLogger.info(f"Reset complete action detector for {pose_type}")
        else:
            SDKLogger.warning(f"Invalid pose_type: {pose_type}")

    def _start_trajectory_executor(self) -> None:
        """Start the trajectory executor thread."""
        self.trajectory_executor_thread = threading.Thread(
            target=self._trajectory_executor_worker, 
            daemon=True,
            name="TrajectoryExecutor"
        )
        self.trajectory_executor_thread.start()
        SDKLogger.info("Trajectory executor thread started")
        SDKLogger.info(f"Max size of movement queue: {self.movement_queue.maxsize}")

    def _trajectory_executor_worker(self) -> None:
        """Worker thread that executes trajectory messages from the queue."""
        last_mode = ModeNumber.FF
        ss_command_issued = False
        while self.is_running:
            try:
                with self.mpc_mode_lock:
                    new_mode = self.mpc_mode

                if new_mode == ModeNumber.SS:
                    if not ss_command_issued:
                        if self.complete_action_detector['left']['use_real_foot_data']:
                            movement = self.movement_queue.get()
                            SDKLogger.info(f"movement: {movement}, time: {time.time()}")
                            foot_pose_traj_msg = self.foot_controller.get_single_foot_trajectory_msg(
                                movement['foot_type'], 
                                [movement['movement_vector']], 
                                0.4, 
                                True
                            )
                            # time.sleep(0.4)
                        else:
                            foot_pose_traj_msg = self.trajectory_queue.get()
                            SDKLogger.info(f"foot_pose_traj_msg: {foot_pose_traj_msg}, time: {time.time()}")
                        if foot_pose_traj_msg is not None:
                            self.bone_frame_publisher.publish_foot_pose_trajectory(foot_pose_traj_msg)
                            # time.sleep(0.4)
                            ss_command_issued = True
                        else:
                            SDKLogger.warning("Movement is illegal !!!")
                    last_mode = new_mode
                else:
                    ss_command_issued = False
                    if new_mode.value != last_mode.value:
                        if self.foot_controller.is_execution_window(last_mode, new_mode):
                            if self.complete_action_detector['left']['use_real_foot_data']:
                                movement = self.movement_queue.get()
                                SDKLogger.info(f"movement: {movement}, time: {time.time()}")
                                foot_pose_traj_msg = self.foot_controller.get_single_foot_trajectory_msg(
                                    movement['foot_type'], 
                                    [movement['movement_vector']], 
                                    0.4, 
                                    True
                                )
                                time.sleep(0.4)
                            else:
                                foot_pose_traj_msg = self.trajectory_queue.get()
                                SDKLogger.info(f"foot_pose_traj_msg: {foot_pose_traj_msg}, time: {time.time()}")
                            self.bone_frame_publisher.publish_foot_pose_trajectory(foot_pose_traj_msg)
                            last_mode = ModeNumber(new_mode.value)
                            time.sleep(0.4)
            except queue.Empty:
                # No message in queue, continue waiting
                continue
            except Exception as e:
                SDKLogger.error(f"Error in trajectory executor thread: {e}")
                continue

    def stop_trajectory_executor(self) -> None:
        """Stop the trajectory executor thread."""
        self.is_running = False
        if hasattr(self, 'trajectory_executor_thread') and self.trajectory_executor_thread and self.trajectory_executor_thread.is_alive():
            self.trajectory_executor_thread.join(timeout=1.0)
            SDKLogger.info("Trajectory executor thread stopped")
    
    def update_active_queue(self):
        """Update the active queue based on the current mode."""
        if self.complete_action_detector['left']['use_real_foot_data']:
            self.active_queue = self.movement_queue
        else:
            self.active_queue = self.trajectory_queue

    def add_trajectory_to_queue(self, foot_pose_traj_msg) -> bool:
        """Add trajectory message to the execution queue."""
        with self.queue_lock:
            self.update_active_queue()
            was_empty = self.active_queue.qsize() == 0
            try:
                self.active_queue.put_nowait(foot_pose_traj_msg)
                if was_empty and not self.trajectory_executor_started:
                    self.start_trajectory_executor()
                return True
            except queue.Full:
                SDKLogger.warning("Trajectory queue is full, dropping trajectory")
                return False

    def publish_foot_pose(self, robot_urdf_matrices):
        """Publish foot pose message."""
        left_foot_pose = self.compute_foot_pose(robot_urdf_matrices, 'left')
        right_foot_pose = self.compute_foot_pose(robot_urdf_matrices, 'right')
        foot_pose_msg = Float32MultiArray()
        foot_pose_msg.data = [
            left_foot_pose[0],
            left_foot_pose[1],
            left_foot_pose[2],
            right_foot_pose[0],
            right_foot_pose[1],
            right_foot_pose[2]
        ]
        self.foot_pose_pub.publish(foot_pose_msg)

    def test_predefined_action(self, action_name: str = "forward_step") -> bool:
        """Test predefined action sequences.
        
        Args:
            action_name: Name of the predefined action
                - "forward_step": Single forward step
                - "backward_step": Single backward step  
                - "left_step": Single left step
                - "right_step": Single right step
                - "turn_left": Turn left in place
                - "turn_right": Turn right in place
                - "walk_forward": Multiple forward steps
                - "walk_backward": Multiple backward steps
                - "sidestep_left": Multiple left steps
                - "sidestep_right": Multiple right steps
        
        Returns:
            bool: True if action was successfully added to queue
        """
        if action_name not in self.predifined_actions:
            SDKLogger.error(f"Unknown action: {action_name}")
            SDKLogger.info(f"Available actions: {list(self.predifined_actions.keys())}")
            return False
        
        action = self.predifined_actions[action_name]
        SDKLogger.info(f"Testing predefined action: {action_name}")
        
        if action["type"] == "single":
            # Single foot movement
            foot_pose_traj_msg = self.foot_controller.get_single_foot_trajectory_msg(
                action["foot_type"], 
                [action["movement"]], 
                0.4, 
                True
            )
            self.add_trajectory_to_queue(foot_pose_traj_msg)

        return True

    def get_available_test_actions(self) -> list:
        """Get list of available predefined test actions."""
        actions = []
        for key in self.predifined_actions:
            actions.append(key)
        return actions

    def cleanup(self) -> None:
        """Cleanup resources and stop threads."""
        try:
            self.stop_trajectory_executor()
        except Exception as e:
            SDKLogger.warning(f"Error stopping trajectory executor: {e}")
        
        # 关闭并行检测器
        try:
            if hasattr(self, 'parallel_detector'):
                self.parallel_detector.shutdown()
        except Exception as e:
            SDKLogger.warning(f"Error shutting down parallel detector: {e}")
        
        # Clear the queue
        try:
            if hasattr(self, 'trajectory_queue'):
                while not self.trajectory_queue.empty():
                    try:
                        self.trajectory_queue.get_nowait()
                    except queue.Empty:
                        break
        except Exception as e:
            SDKLogger.warning(f"Error clearing trajectory queue: {e}")
            
        # SDKLogger.info("KuavoPicoInfoTransformer cleanup completed")

    def __del__(self):
        """Destructor to ensure cleanup."""
        try:
            self.cleanup()
        except Exception as e:
            # Don't log during garbage collection as it might cause issues
            pass

    @staticmethod
    def homogeneous_matrix_roll(angle_degrees: float) -> np.ndarray:
        """Generate homogeneous transformation matrix for rotation around X axis."""
        angle_radians = np.radians(angle_degrees)
        c = np.cos(angle_radians)
        s = np.sin(angle_radians)
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])

    def get_hand_pose(self, side):
        if side == "Left":
            return self.left_hand_pose, self.left_elbow_pos
        elif side == "Right":
            return self.right_hand_pose, self.right_elbow_pos
        else:
            print("Invalid side: {}".format(side))
            return None, None
        
    def transform_matrix_to_ros(self, matrix):
        """Transform Unity coordinate system matrices to robot URDF coordinate system."""
        # Unity to ROS transformation
        T_unity_2_ros = np.array([[0, 0, -1, 0],
                                 [-1, 0, 0, 0],
                                 [0, 1, 0, 0],
                                 [0, 0, 0, 1]])
        
        T_ros_2_robot_urdf = np.array([[0, -1, 0, 0],
                                      [0, 0, 1, 0],
                                      [-1, 0, 0, 0],
                                      [0, 0, 0, 1]])
        
        # Arm correction matrices
        T_LEFT_ARM_CORRECTION = self.homogeneous_matrix_roll(90)
        T_RIGHT_ARM_CORRECTION = self.homogeneous_matrix_roll(-90)
        
        ros_matrices = np.matmul(T_unity_2_ros, matrix)
        ros_matrices = np.matmul(ros_matrices, T_ros_2_robot_urdf)

        # Reverse x position
        ros_matrices[:,0,3] = -ros_matrices[:,0,3]

        # Reverse y and z rotation
        ros_matrices[:, :3, :3] = np.array([
            quaternion_matrix(
                quaternion_from_euler(
                    *(np.array(euler_from_quaternion(quaternion_from_matrix(matrix))) * np.array([1, -1, -1]))
                )
            )[:3, :3] 
            for matrix in ros_matrices
        ])

        # Align to robot urdf
        for idx in self.left_arm_idxs:
            ros_matrices[idx] = ros_matrices[idx] @ T_LEFT_ARM_CORRECTION
        for idx in self.right_arm_idxs:
            ros_matrices[idx] = ros_matrices[idx] @ T_RIGHT_ARM_CORRECTION
        return ros_matrices

    def parse_joy_data(self, data_str):
        """Parse JSON format joy data."""
        try:
            data = json.loads(data_str)
            if 'controllers' in data:
                return data['controllers']
            else:
                return None
        except Exception as e:
            SDKLogger.error(f"Error parsing JSON data: {e}")
            return None

    def parse_transform_data(self, data_str):
        """Parse JSON format transform data."""
        try:
            data = json.loads(data_str)
            transforms = []
            
            for i, pose in enumerate(data['poses']):
                transform = {
                    'role': i,
                    'position': np.array(pose[0:3]),  
                    'rotation': np.array(pose[3:7])   
                }
                transforms.append(transform)
            return transforms
        except Exception as e:
            SDKLogger.error(f"Error parsing JSON data: {e}")
            SDKLogger.error(f"data: {data}")
            return None
        
    def parse_transform_data_to_matrix(self, data_str):
        """Parse JSON format transform data and return a [len(body_tracker_role), 4, 4] matrix."""
        transforms = self.parse_transform_data(data_str)
        if not transforms:
            return None
        
        for i, transform in enumerate(transforms):
            self.cached_matrices[i][:3, :3] = quaternion_matrix(transform['rotation'])[:3, :3]
            self.cached_matrices[i][:3, 3] = transform['position']

        return self.cached_matrices
    
    def get_robot_urdf_matrix(self, data_str):
        """Get the robot URDF matrix from the data string."""
        matrices = self.parse_transform_data_to_matrix(data_str)
        if matrices is None or matrices.size == 0:
            SDKLogger.warning("Received invalid matrices data")
            return
        robot_urdf_matrices = self.transform_matrix_to_ros(matrices)
        current_time = rospy.Time.now()
        return robot_urdf_matrices, current_time

    def scale_arm_positions(self, shoulder_pos, elbow_pos, hand_pos, human_shoulder_pos, side):
        """Scale the arm positions from human scale to robot scale."""
        human_upper_arm_length = math.sqrt((elbow_pos[0] - shoulder_pos[0])**2 + (elbow_pos[1] - shoulder_pos[1])**2 + (elbow_pos[2] - shoulder_pos[2])**2)
        human_lower_arm_length = math.sqrt((hand_pos[0] - elbow_pos[0])**2 + (hand_pos[1] - elbow_pos[1])**2 + (hand_pos[2] - elbow_pos[2])**2)

        if self.measure_arm_length:
            if side == "Left":
                self.left_upper_arm_lengths.append(human_upper_arm_length)
                self.left_lower_arm_lengths.append(human_lower_arm_length)
                if len(self.left_upper_arm_lengths) > self.arm_length_num:
                    self.left_upper_arm_lengths.pop(0)
                    self.left_lower_arm_lengths.pop(0)
                # Calculate average when we have enough data
                if len(self.left_upper_arm_lengths) >= self.arm_length_num:
                    self.avg_left_upper_arm_length = sum(self.left_upper_arm_lengths) / len(self.left_upper_arm_lengths)
                    self.avg_left_lower_arm_length = sum(self.left_lower_arm_lengths) / len(self.left_lower_arm_lengths)
            else:
                self.right_upper_arm_lengths.append(human_upper_arm_length)
                self.right_lower_arm_lengths.append(human_lower_arm_length)
                if len(self.right_upper_arm_lengths) > self.arm_length_num:
                    self.right_upper_arm_lengths.pop(0)
                    self.right_lower_arm_lengths.pop(0)
                # Calculate average when we have enough data
                if len(self.right_upper_arm_lengths) >= self.arm_length_num:
                    self.avg_right_upper_arm_length = sum(self.right_upper_arm_lengths) / len(self.right_upper_arm_lengths)
                    self.avg_right_lower_arm_length = sum(self.right_lower_arm_lengths) / len(self.right_lower_arm_lengths)
            radi1 = self.upper_arm_length/human_upper_arm_length
            radi2 = self.lower_arm_length/human_lower_arm_length 
        else:
            if side == "Left":
                radi1 = self.upper_arm_length/self.avg_left_upper_arm_length
                radi2 = (self.lower_arm_length + self.upper_arm_length)/(self.avg_left_lower_arm_length + self.avg_left_upper_arm_length)
            else:
                radi1 = self.upper_arm_length/self.avg_right_upper_arm_length
                radi2 = (self.lower_arm_length + self.upper_arm_length)/(self.avg_right_lower_arm_length + self.avg_right_upper_arm_length)
        
        scaled_elbow_pos = np.zeros(3)
        scaled_hand_pos = np.zeros(3)
        for i in range(3):
            scaled_elbow_pos[i] = shoulder_pos[i] + radi1 * (elbow_pos[i] - human_shoulder_pos[i])
            scaled_hand_pos[i] = scaled_elbow_pos[i] + radi2 * (hand_pos[i] - elbow_pos[i])

        return scaled_elbow_pos, scaled_hand_pos
    
    def check_if_vr_error(self):
        """
        Check if the VR system is error.
        """
        if self.left_hand_pose is None:
            return False
        left_hand_pos = self.left_hand_pose[0]
        right_hand_pos = self.right_hand_pose[0]
        error = True
        error &= (-0.1 < left_hand_pos[0] < 0.15)
        error &= (-0.05 < left_hand_pos[1] < 0.05)
        error &= (0.15 < left_hand_pos[2] < 0.4)
        error &= (-0.1 < right_hand_pos[0] < 0.15)
        error &= (-0.05 < right_hand_pos[1] < 0.05)
        error &= (0.15 < right_hand_pos[2] < 0.4)
        return error

    def read_msg_hand(self, msg):
        """Read hand pose data from message without additional transformations."""
        if not msg.poses:
            return

        left_hand_idx = self.bone_name_to_index["LEFT_HAND"]
        left_hand_pose = msg.poses[left_hand_idx]
        self.left_hand_pose = [
            [left_hand_pose.position.x, left_hand_pose.position.y, left_hand_pose.position.z],
            [left_hand_pose.orientation.x, left_hand_pose.orientation.y, 
             left_hand_pose.orientation.z, left_hand_pose.orientation.w]
        ]

        right_hand_idx = self.bone_name_to_index["RIGHT_HAND"]
        right_hand_pose = msg.poses[right_hand_idx]
        self.right_hand_pose = [
            [right_hand_pose.position.x, right_hand_pose.position.y, right_hand_pose.position.z],
            [right_hand_pose.orientation.x, right_hand_pose.orientation.y, 
             right_hand_pose.orientation.z, right_hand_pose.orientation.w]
        ]

        left_elbow_idx = self.bone_name_to_index["LEFT_ELBOW"]
        left_elbow_pose = msg.poses[left_elbow_idx]
        self.left_elbow_pos = [
            left_elbow_pose.position.x,
            left_elbow_pose.position.y,
            left_elbow_pose.position.z
        ]

        right_elbow_idx = self.bone_name_to_index["RIGHT_ELBOW"]
        right_elbow_pose = msg.poses[right_elbow_idx]
        self.right_elbow_pos = [
            right_elbow_pose.position.x,
            right_elbow_pose.position.y,
            right_elbow_pose.position.z
        ]
        
        left_shoulder_idx = self.bone_name_to_index["LEFT_SHOULDER"]
        left_shoulder_pose = msg.poses[left_shoulder_idx]
        self.left_shoulder_pos = [
            left_shoulder_pose.position.x,
            left_shoulder_pose.position.y,
            left_shoulder_pose.position.z
        ]
        
        right_shoulder_idx = self.bone_name_to_index["RIGHT_SHOULDER"]
        right_shoulder_pose = msg.poses[right_shoulder_idx]
        self.right_shoulder_pos = [
            right_shoulder_pose.position.x,
            right_shoulder_pose.position.y,
            right_shoulder_pose.position.z
        ]

        self.left_elbow_pos, self.left_hand_pos = self.scale_arm_positions(self.left_shoulder_pos, self.left_elbow_pos, self.left_hand_pose[0], self.left_shoulder_pos, "Left")
        self.right_elbow_pos, self.right_hand_pos = self.scale_arm_positions(self.right_shoulder_pos, self.right_elbow_pos, self.right_hand_pose[0], self.right_shoulder_pos, "Right")
        
        self.is_running = True
        self.vr_error = False
        
        if self.vis_pub:
            left_marker = self.construct_point_marker(self.left_hand_pose[0], 0.08, 0.9, color=[1, 0, 0])
            left_elbow_marker = self.construct_point_marker(self.left_elbow_pos, 0.1, color=[0, 1, 0])
            left_shoulder_marker = self.construct_point_marker(self.left_shoulder_pos, 0.1, color=[0, 0, 1])
            self.marker_pub.publish(left_marker)
            self.marker_pub_elbow.publish(left_elbow_marker)
            self.marker_pub_shoulder.publish(left_shoulder_marker)

            right_marker = self.construct_point_marker(self.right_hand_pose[0], 0.08, 0.9, color=[1, 0, 0])
            right_elbow_marker = self.construct_point_marker(self.right_elbow_pos, 0.1, color=[0, 1, 0])
            right_shoulder_marker = self.construct_point_marker(self.right_shoulder_pos, 0.1, color=[0, 0, 1])
            self.marker_pub_right.publish(right_marker)
            self.marker_pub_elbow_right.publish(right_elbow_marker)
            self.marker_pub_shoulder_right.publish(right_shoulder_marker)

    def construct_point_marker(self, point, scale=0.05, alpha=0.3, color=[0, 0, 1]):
        if len(point) != 3:
            print("Invalid pos, cannot construct marker")
            return None
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = alpha
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.w = 1.0
        return marker 

    
    @staticmethod
    def euler_to_rotation_matrix(yaw, pitch, roll):
        """Convert euler angles to rotation matrix."""
        R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                            [0, 1, 0],
                            [-np.sin(pitch), 0, np.cos(pitch)]])

        R_roll = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])

        R = np.dot(R_roll, np.dot(R_pitch, R_yaw))
        return R
    
    def get_multiple_steps_msg(self, body_poses, dt, is_left_first=True, collision_check=True):
        num_steps = 2*len(body_poses)
        time_traj = []
        foot_idx_traj = []
        foot_traj = []
        torso_traj = []
        l_foot_rect_last = RotatingRectangle(center=(0, 0.1), width=0.24, height=0.1, angle=0)
        r_foot_rect_last = RotatingRectangle(center=(0,-0.1), width=0.24, height=0.1, angle=0)
        torso_yaw_last = 0.0
        torso_pose_last = np.array([0, 0, 0, 0])
        for i in range(num_steps):
            time_traj.append(dt * (i+1))
            body_pose = body_poses[i//2]
            # body_pose now contains [position, yaw_deg]
            torso_pos = np.asarray(body_pose[:3]) 
            torso_yaw = np.radians(body_pose[3])  
            l_foot, r_foot = self.generate_steps(torso_pos, torso_yaw, 0.1)
            l_foot = [*l_foot[:3], torso_yaw]
            r_foot = [*r_foot[:3], torso_yaw]

            if(i%2 == 0):        
                torso_pose = np.array([*torso_pos, torso_yaw])
                R_wl = self.euler_to_rotation_matrix(torso_pose_last[3], 0, 0)
                delta_pos = R_wl.T @ (torso_pose[:3] - torso_pose_last[:3])
                SDKLogger.debug(f"delta_pos: {delta_pos}")
                if(torso_yaw > 0.0 or delta_pos[1] > 0.0):
                    is_left_first = True
                else:
                    is_left_first = False
            if(collision_check and i%2 == 0):
                l_foot_rect_next = RotatingRectangle(center=(l_foot[0],l_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
                r_foot_rect_next = RotatingRectangle(center=(r_foot[0],r_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
                l_collision = l_foot_rect_next.is_collision(r_foot_rect_last)
                r_collision = r_foot_rect_next.is_collision(l_foot_rect_last)
                if l_collision and r_collision:
                    SDKLogger.error("Detect collision, Please adjust your body_poses input!!!")
                    break
                elif l_collision:
                    SDKLogger.warning("Left foot is in collision, switch to right foot")
                    is_left_first = False
                elif r_collision:
                    SDKLogger.warning("Right foot is in collision, switch to left foot")
                    is_left_first = True
                l_foot_rect_last = l_foot_rect_next
                r_foot_rect_last = r_foot_rect_next
            if(i%2 == 0):
                torso_traj.append((torso_pose_last + torso_pose)/2.0)
                if is_left_first:
                    foot_idx_traj.append(0)
                    foot_traj.append(l_foot)
                else:
                    foot_idx_traj.append(1)
                    foot_traj.append(r_foot)
            else:
                torso_traj.append(torso_pose)
                if is_left_first:
                    foot_idx_traj.append(1)
                    foot_traj.append(r_foot)
                else:
                    foot_idx_traj.append(0)
                    foot_traj.append(l_foot)
            torso_pose_last = torso_traj[-1]
            torso_yaw_last = torso_yaw
        SDKLogger.debug(f"time_traj: {time_traj}")
        SDKLogger.debug(f"foot_idx_traj: {foot_idx_traj}")
        SDKLogger.debug(f"foot_traj: {foot_traj}")
        SDKLogger.debug(f"torso_traj: {torso_traj}")
        return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)
    
    def detect_significant_movement(self, current_pose, pose_type='left'):
        """Detect significant movement based on current detection mode.
        
        Args:
            current_pose: Current pose [x, y, z, yaw]
            pose_type: 'left', 'right', or 'body'
            
        Returns:
            tuple: (detected, movement_vector) for threshold mode
                   (detected, movement_vector, action_duration) for complete_action mode
        """
        detection_mode = self.get_detection_mode()
        
        if detection_mode == 'complete_action':
            # 使用完整动作检测
            return self.detect_complete_action(current_pose, pose_type)
        else:
            # 使用阈值检测（原有方式）
            return self._detect_threshold_movement(current_pose, pose_type)
    
    def _detect_threshold_movement(self, current_pose, pose_type='left'):
        """Original threshold-based movement detection."""
        # Get initial pose reference
        if pose_type == 'left':
            initial_pose = self.movement_detector['initial_left_foot_pose']
        elif pose_type == 'right':
            initial_pose = self.movement_detector['initial_right_foot_pose']
        elif pose_type == 'body':
            initial_pose = self.movement_detector['initial_body_pose']
        else:
            return False, None
        
        # If no initial pose set, set it and return False
        if initial_pose is None:
            if pose_type == 'left':
                self.movement_detector['initial_left_foot_pose'] = current_pose
                print(f"\033[92mInitial left foot pose set to: {current_pose}\033[0m")
            elif pose_type == 'right':
                self.movement_detector['initial_right_foot_pose'] = current_pose
                print(f"\033[92mInitial right foot pose set to: {current_pose}\033[0m")
            elif pose_type == 'body':
                self.movement_detector['initial_body_pose'] = current_pose
                print(f"\033[92mInitial body pose set to: {current_pose}\033[0m")
            return False, None
        
        # Calculate position and angle differences
        pos_diff_horizontal = [
            current_pose[0] - initial_pose[0],
            current_pose[1] - initial_pose[1]
        ]

        pos_diff_vertical = current_pose[2] - initial_pose[2]
        # print(f"\033[92mCurrent {pose_type} vertical diff is: {pos_diff_vertical}\033[0m")

        # Calculate yaw difference (handle angle wrapping)
        yaw_diff = current_pose[3] - initial_pose[3]
        while yaw_diff > 180:
            yaw_diff -= 360
        while yaw_diff < -180:
            yaw_diff += 360
        
        # Calculate magnitudes
        # pos_magnitude = np.sqrt(pos_diff[0]**2 + pos_diff[1]**2 + pos_diff[2]**2)
        pos_magnitude = np.sqrt(pos_diff_horizontal[0]**2 + pos_diff_horizontal[1]**2)
        
        if pos_magnitude >= self.movement_detector['horizontal_threshold']:
            movement_vector = [pos_diff_horizontal[0], pos_diff_horizontal[1], 0.0, yaw_diff]
            SDKLogger.debug(f"Significant movement detected for {pose_type}: pos={pos_magnitude:.3f}m, angle={movement_vector[3]:.1f}°")

            # Update initial pose to current pose for next detection
            if pose_type == 'left':
                self.movement_detector['initial_left_foot_pose'] = current_pose
            elif pose_type == 'right':
                self.movement_detector['initial_right_foot_pose'] = current_pose
            elif pose_type == 'body':
                self.movement_detector['initial_body_pose'] = current_pose
            
            movement_vector_time = [*movement_vector, 0.15]

            return True, movement_vector_time
        
        return False, None
    
    @staticmethod
    def generate_steps(torso_pos, torso_yaw, foot_bias):
        l_foot_bias = np.array([0, foot_bias, -torso_pos[2]])
        r_foot_bias = np.array([0, -foot_bias, -torso_pos[2]])
        R_z = np.array([
            [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
            [np.sin(torso_yaw), np.cos(torso_yaw), 0],
            [0, 0, 1]
        ])
        l_foot = torso_pos + R_z.dot(l_foot_bias)
        r_foot = torso_pos + R_z.dot(r_foot_bias)
        return l_foot, r_foot
    
    def get_pose_from_matrix(self, matrix):
        """Get pose from matrix."""
        pos = matrix[:3, 3]
        quat = quaternion_from_matrix(matrix)
        return pos, quat
    
    def publish_tf_transforms(self, robot_urdf_matrices, current_time):
        """Publish TF transforms for all bones"""
        for i, matrix in enumerate(robot_urdf_matrices):
            try:
                pos, quat = self.get_pose_from_matrix(matrix)
            except Exception as e:
                SDKLogger.error(f"Error getting pose from matrix: {e}")
                continue
            
            self.tf_broadcaster.sendTransform(
                pos,
                quat,
                current_time,
                BODY_TRACKER_ROLES[i],
                "world"
            )

    def get_local_pose(self, joint_matrix, spine3_matrix):
        """
        Get joint pose relative to robot base_link
        Args:
            joint_matrix: 4x4 transformation matrix of the joint
            spine3_matrix: 4x4 transformation matrix of SPINE3
        Returns:
            pose: picoPoseInfo message with pose relative to base_link
        """
        try:
            # Calculate transform from joint to base_link
            # First transform joint to SPINE3 frame, then to base_link
            T_joint_to_spine3 = np.linalg.inv(spine3_matrix) @ joint_matrix
            T_joint_to_base = self.T_spine3_to_base @ T_joint_to_spine3
            
            # Extract position and orientation
            pos = T_joint_to_base[:3, 3]
            quat = quaternion_from_matrix(T_joint_to_base)
            
            # Create picoPoseInfo message
            pose = picoPoseInfo()
            pose.position = Point(*pos)
            pose.orientation = Quaternion(*quat)
            
            return pose
        except Exception as e:
            SDKLogger.error(f"Error getting local pose: {e}")
            return None
        
    def publish_local_poses(self, robot_urdf_matrices, current_time):
        """Publish local poses relative to base_link"""
        self.compute_head_body_pose(robot_urdf_matrices)

        # Get SPINE3 matrix for base_link calculations
        spine3_idx = BODY_TRACKER_ROLES.index('SPINE3')
        spine3_matrix = robot_urdf_matrices[spine3_idx]
        
        pose_info_list = picoPoseInfoList()
        pose_info_list.timestamp_ms = int(current_time.to_sec() * 1000)  # Convert Time to milliseconds
        pose_info_list.is_high_confidence = True
        pose_info_list.is_hand_tracking = True
        
        for i, matrix in enumerate(robot_urdf_matrices):
            try:
                # Create pose info for each bone
                local_pose = self.get_local_pose(matrix, spine3_matrix)
                if local_pose is not None:
                    pose_info_list.poses.append(local_pose)
            except Exception as e:
                SDKLogger.error(f"Error getting local pose for bone {i}: {e}")
                continue
        
        self.bone_frame_publisher.publish_pose_info_list(pose_info_list)

    def publish_pico_joys(self, joy_data):
        """Publish pico joys"""
        joy_msg = JoySticks()
        try:   
            def process_joy_data(joy_data, side)->dict:
                if side in joy_data:
                    data = joy_data[side]
                    primary = data.get('primaryButton', False)
                    secondary = data.get('secondaryButton', False)
                    grip_btn = data.get('gripButton', False)
                    trigger_btn = data.get('triggerButton', False)
                    grip_val = data.get('gripValue', 0.0)
                    trigger_val = data.get('triggerValue', 0.0)
                    thumbstick = data.get('thumbstick', [0.0, 0.0])
                    return  {
                        "buttons":  [
                            True if primary == 1 else False,    # X/A
                            True if secondary == 1 else False,  # Y/B
                            True if grip_btn == 1 else False,   # Grip
                            True if trigger_btn == 1 else False # Trigger
                        ],
                        "axes": [
                            float(thumbstick[0]), # thumbstick_x
                            float(thumbstick[1]), # thumbstick_y
                            float(grip_val),    # grip_value
                            float(trigger_val)  # trigger_value
                        ]
                    }
                else:
                    SDKLogger.error(f"Process Joy data lost {side} in json!")
                    return  {
                        "buttons":  [False]*4,
                        "axes": [0.0]*4
                    }
            left_joy = process_joy_data(joy_data, 'left')
            right_joy = process_joy_data(joy_data, 'right')
            joy_msg.left_x = left_joy["axes"][0]
            joy_msg.left_y = left_joy["axes"][1]
            joy_msg.left_grip = left_joy["axes"][2]
            joy_msg.left_trigger = left_joy["axes"][3]
            joy_msg.left_first_button_pressed = bool(left_joy["buttons"][0])
            joy_msg.left_second_button_pressed = bool(left_joy["buttons"][1])
            joy_msg.left_first_button_touched = False  # 默认值
            joy_msg.left_second_button_touched = False  # 默认值
            
            joy_msg.right_x = right_joy["axes"][0]
            joy_msg.right_y = right_joy["axes"][1]
            joy_msg.right_trigger = right_joy["axes"][3]
            joy_msg.right_grip = right_joy["axes"][2]
            joy_msg.right_first_button_pressed = bool(right_joy["buttons"][0])
            joy_msg.right_second_button_pressed = bool(right_joy["buttons"][1])
            joy_msg.right_first_button_touched = False  # 默认值
            joy_msg.right_second_button_touched = False  # 默认值
            # print(joy_msg)
        except Exception as e:
            SDKLogger.error(f"Error processing joy data: {e}")
            return None
        self.bone_frame_publisher.publish_pico_joys(joy_msg)

    def process_body_pose_for_stepping(self, robot_urdf_matrices):
        """Process body pose for step generation"""
        spine3_idx = BODY_TRACKER_ROLES.index('SPINE3')
        spine3_matrix = robot_urdf_matrices[spine3_idx]
        
        # Get SPINE3 pose for body pose calculation
        try:
            pos, quat = self.get_pose_from_matrix(spine3_matrix)
            _, _, yaw = euler_from_quaternion(quat)
            current_body_pose = [pos[0], pos[1], pos[2]-0.34, np.degrees(yaw)]
        except Exception as e:
            SDKLogger.error(f"Error getting SPINE3 pose: {e}")
            return
        
        # Check if movement is significant enough for a step
        detection_result = self.detect_significant_movement(current_body_pose, 'body')
        
        # 处理不同检测模式的返回值
        if len(detection_result) == 2:
            # 阈值检测模式
            significant, movement_vector = detection_result
            action_duration = None
        elif len(detection_result) == 3:
            # 完整动作检测模式（旧版本）
            significant, movement_vector, action_duration = detection_result
        elif len(detection_result) == 4:
            # 完整动作检测模式（新版本，包含current_time）
            significant, movement_vector, action_duration, current_time = detection_result
        else:
            SDKLogger.error(f"Unexpected detection result format: {detection_result}")
            return
        
        if significant and movement_vector is not None:
            if action_duration is not None:
                SDKLogger.info(f"Generating step with movement vector: {movement_vector}, duration: {action_duration:.3f}s")
            else:
                SDKLogger.info(f"Generating step with movement vector: {movement_vector}")
            
            foot_pose_traj_msg = self.get_multiple_steps_msg([movement_vector], 0.15, True, True)
            # Add trajectory to queue instead of direct publishing
            if not self.add_trajectory_to_queue(foot_pose_traj_msg):
                SDKLogger.warning("Failed to add body trajectory to queue")
        elif not significant:
            # SDKLogger.debug("Movement too small, continuing monitoring...")
            pass

    def compute_foot_pose(self, robot_urdf_matrices, side):
        """Compute foot pose"""
        foot_idx = BODY_TRACKER_ROLES.index(f"{side.upper()}_FOOT")
        foot_matrix = robot_urdf_matrices[foot_idx]
        pos, quat = self.get_pose_from_matrix(foot_matrix)
        roll, pitch, yaw = euler_from_quaternion(quat)
        # SDKLogger.info(f"{side} foot yaw: {np.degrees(yaw)}")
        return [pos[0], pos[1], pos[2], np.degrees(yaw)]
    
    def rpy_to_matrix(self, rpy):
        R = RollPitchYaw(rpy).ToRotationMatrix()
        return R.matrix()
    
    def matrix_to_quaternion(self, mat):
        mat = np.asarray(mat, dtype=np.float64)
        quat = DrakeQuaternion(mat)
        quat_vec = [quat.x(), quat.y(), quat.z(), quat.w()]
        return quat_vec
    
    def matrix_to_axis_angle(self, R):
        quat = np.asarray(self.matrix_to_quaternion(R))
        # rotvec = np.zeros(3)
        angle = 2.0 * np.arctan2(np.linalg.norm(quat[:3]), quat[3])
        if angle <= 1e-3:  # small angle Taylor series expansion
            angle2 = angle * angle
            scale = 2.0 + angle2 / 12.0 + 7.0 * angle2 * angle2 / 2880.0
        else:  # large angle
            scale = angle / np.sin(angle / 2)
        rotvec = quat[:3] * scale
        return rotvec, angle

    def quaternion_to_matrix(self, quat):
        q1 = np.quaternion(quat[3], quat[0], quat[1], quat[2])
        q1 = q1.normalized()
        R = RotationMatrix(DrakeQuaternion(q1.w, q1.x, q1.y, q1.z))
        return R.matrix()
    
    def axis_angle_to_matrix(self, rotvec):
        rotvec = np.asarray(rotvec, dtype=float)
        angle = np.linalg.norm(rotvec)

        if angle < 1e-3:   # small angle Taylor series expansion
            angle2 = angle * angle
            scale = 0.5 - angle2 / 48 + angle2 * angle2 / 3840
        else:
            scale = np.sin(angle / 2) / angle

        normalized_rotvec = rotvec * scale

        quat = np.zeros(4)
        quat[0:3] = normalized_rotvec
        quat[3] = np.cos(angle / 2)

        R = self.quaternion_to_matrix(quat)
        return R
    
    def matrix_to_rpy(self, mat):
        return RollPitchYaw(mat).vector()
    
    def compute_head_body_pose(self, robot_urdf_matrices):
        """Compute head body pose"""
        init_R_wC = self.rpy_to_matrix([np.pi/2, 0, np.pi/2]) # rotation matrix from world to chest
        chest_idx = self.bone_name_to_index["SPINE3"]
        T_wChest = robot_urdf_matrices[chest_idx]
        chest_pos, chest_quat = self.get_pose_from_matrix(T_wChest)
        axis, angle = self.matrix_to_axis_angle(init_R_wC.T @ T_wChest[:3, :3])
        chest_axis_agl = [0, 0, axis[1]]
        self.head_body_pose.angular.z = axis[1]
        R_wChest_rm_yaw = self.axis_angle_to_matrix(chest_axis_agl).T @ T_wChest[:3, :3]
        self.head_body_pose.angular.y = self.matrix_to_rpy(init_R_wC.T @ R_wChest_rm_yaw)[0]
        self.head_body_pose.linear.x = chest_pos[0]
        self.head_body_pose.linear.y = chest_pos[1]
        self.head_body_pose.linear.z = chest_pos[2]

    def pub_head_body_pose_msg(self, head_body_pose: HeadBodyPose):
        """Publish head body pose message."""
        msg = Twist()
        msg.angular.y = head_body_pose.angular.y
        msg.angular.x = 0.0 
        msg.angular.z = 0.0
        pitch_ratio = 0.8
        msg.angular.y = max(3*np.pi/180.0, min(pitch_ratio*head_body_pose.angular.y, 40*np.pi/180.0))
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = max(-0.4, min(head_body_pose.linear.z - 1.24, 0.1))
        self.head_body_pose_puber.publish(msg)

    def process_foot_pose_for_stepping(self, robot_urdf_matrices, side):
        """Process foot pose for stepping"""
        # Convert side string to foot_type integer
        foot_type = 0 if side.lower() == 'left' else 1
        
        try:
            current_foot_pose = self.compute_foot_pose(robot_urdf_matrices, side)
        except Exception as e:
            SDKLogger.error(f"Error getting {side}_foot pose: {e}")
            return
        
        # Check if movement is significant enough for a step
        significant, movement_vector_time = self.detect_significant_movement(current_foot_pose, side.lower())
        
        if significant:
            foot_pose_traj_msg = self.foot_controller.get_single_foot_trajectory_msg(foot_type, [movement_vector_time], 0.4, True)
            if not self.add_trajectory_to_queue(foot_pose_traj_msg):
                SDKLogger.warning(f"Failed to add {side} foot trajectory to queue")
        elif not significant:
            # SDKLogger.debug(f"{side} foot movement too small, continuing monitoring...")
            pass

    def test_foot_controller(self):
        """Test foot controller"""
        actions = self.get_available_test_actions()
        for action in actions:
            SDKLogger.info(f"Testing action: {action}")
            self.test_predefined_action(action)

    def process_foot_poses_parallel(self, robot_urdf_matrices):
        """并行处理左右脚姿态检测（推荐使用）
        
        Args:
            robot_urdf_matrices: 机器人URDF矩阵
            
        Returns:
            dict: 检测结果 {'left': (detected, movement_vector), 'right': (detected, movement_vector)}
        """
        try:
            results = self.parallel_detector.detect_parallel(robot_urdf_matrices)
            
            # # 记录检测结果
            # for side, (detected, movement_vector) in results.items():
            #     if detected:
            #         SDKLogger.info(f"Parallel detection: {side} foot movement detected")
            #     else:
            #         SDKLogger.debug(f"Parallel detection: {side} foot no significant movement")
            
            return results
            
        except Exception as e:
            SDKLogger.error(f"Error in parallel foot detection: {e}")
            return {'left': (False, None), 'right': (False, None)}

    def enable_parallel_detection(self):
        """启用并行检测模式"""
        SDKLogger.info("Parallel foot detection enabled")
        return True

    def disable_parallel_detection(self):
        """禁用并行检测模式"""
        SDKLogger.info("Parallel foot detection disabled")
        return True

    def get_parallel_detection_status(self):
        """获取并行检测状态"""
        return {
            'enabled': self.parallel_detector.is_running,
            'executor_active': not self.parallel_detector.executor._shutdown,
            'active_futures': len([f for f in self.parallel_detector.detection_futures.values() if not f.done()])
        }

    def set_parallel_detection_config(self, config: dict):
        """设置并行检测配置
        
        Args:
            config: 配置字典，包含以下可选参数：
                - enabled: bool - 是否启用并行检测
                - timeout_ms: int - 检测超时时间（毫秒）
                - max_workers: int - 最大工作线程数
                - precise_timing: bool - 是否使用精确时间戳
                - debug_mode: bool - 调试模式
        """
        current_config = self.movement_detector.get('parallel_detection', {})
        current_config.update(config)
        self.movement_detector['parallel_detection'] = current_config
        
        # 更新并行检测器配置
        if hasattr(self, 'parallel_detector'):
            if 'timeout_ms' in config:
                self.parallel_detector.timeout = config['timeout_ms'] / 1000.0
            if 'debug_mode' in config:
                self.parallel_detector.debug_mode = config['debug_mode']
        
        SDKLogger.info(f"Parallel detection config updated: {config}")

    def get_parallel_detection_config(self):
        """获取并行检测配置"""
        return self.movement_detector.get('parallel_detection', {})

    def reset_parallel_detector(self):
        """重置并行检测器"""
        if hasattr(self, 'parallel_detector'):
            self.parallel_detector.shutdown()
            self.parallel_detector = ParallelFootDetector(self)
            SDKLogger.info("Parallel detector reset completed")

    def get_parallel_detection_performance(self):
        """获取并行检测性能统计"""
        if hasattr(self, 'parallel_detector'):
            return self.parallel_detector.get_performance_stats()
        return None

    def reset_parallel_detection_performance(self):
        """重置并行检测性能统计"""
        if hasattr(self, 'parallel_detector'):
            self.parallel_detector.reset_performance_stats()
            return True
        return False

    def detect_complete_action(self, current_pose, pose_type='left'):
        """Detect complete action based on ground-lift-ground process.
        
        Args:
            current_pose: Current pose [x, y, z, yaw]
            pose_type: 'left', 'right'
            
        Returns:
            tuple: (detected, movement_vector, action_duration, current_time)
        """
        if pose_type not in self.complete_action_detector:
            SDKLogger.error(f"Invalid pose_type: {pose_type}")
            return False, None, 0.0, None
        
        detector = self.complete_action_detector[pose_type]
        current_time = time.time()
        
        # 添加当前姿态到历史记录
        detector['pose_history'].append({
            'time': current_time,
            'pose': current_pose
        })
        
        # 限制历史记录大小
        buffer_size = self.movement_detector['complete_action']['action_buffer_size']
        if len(detector['pose_history']) > buffer_size:
            detector['pose_history'] = detector['pose_history'][-buffer_size:]
        
        # 获取当前高度（z坐标）
        current_height = current_pose[2]
        
        # 获取阈值（自适应或固定）
        lift_threshold, ground_threshold = self._get_adaptive_thresholds(pose_type)
        
        # 状态机逻辑
        if detector['state'] == 'ground':
            # 检测是否抬起
            if current_height > lift_threshold:
                detector['state'] = 'lifted'
                detector['lift_time'] = current_time
                detector['start_pose'] = current_pose
                SDKLogger.debug(f"{pose_type} lifted at height {current_height:.3f}m (threshold: {lift_threshold:.3f}m)")
        
        elif detector['state'] == 'lifted':
            # 检测是否回到地面
            if current_height <= ground_threshold:
                detector['state'] = 'action_complete'
                detector['ground_time'] = current_time
                detector['end_pose'] = current_pose
                
                # 计算动作时长
                if detector['lift_time'] is not None:
                    detector['action_duration'] = current_time - detector['lift_time']

                SDKLogger.debug(f"{pose_type} action completed, duration: {detector['action_duration']:.3f}s")
        
        elif detector['state'] == 'action_complete':
            # 检查动作是否有效
            if self._validate_complete_action(detector, pose_type):
                # 计算移动向量
                movement_vector = self._calculate_movement_vector(detector)
                action_duration = detector['action_duration']
                movement_vector_time = [*movement_vector, action_duration]
                
                # 重置检测器状态
                self._reset_detector_state(detector)

                return True, movement_vector_time
            else:
                # 动作无效，重置状态
                SDKLogger.debug(f"Invalid action for {pose_type}, resetting")
                self._reset_detector_state(detector)
        
        return False, None
    
    def _validate_complete_action(self, detector, pose_type):
        """Validate if the complete action is valid."""
        config = self.movement_detector['complete_action']
        
        # 检查动作时长
        if (detector['action_duration'] < config['min_action_duration'] or 
            detector['action_duration'] > config['max_action_duration']):
            SDKLogger.debug(f"{pose_type} action duration {detector['action_duration']:.3f}s out of range "
                          f"[{config['min_action_duration']:.3f}, {config['max_action_duration']:.3f}]")
            return False
        
        # 检查是否有足够的水平移动
        if detector['start_pose'] is not None and detector['end_pose'] is not None:
            horizontal_distance = np.sqrt(
                (detector['end_pose'][0] - detector['start_pose'][0])**2 +
                (detector['end_pose'][1] - detector['start_pose'][1])**2
            )
            
            if horizontal_distance < config['min_horizontal_movement']:
                SDKLogger.debug(f"{pose_type} horizontal movement {horizontal_distance:.3f}m below threshold "
                              f"{config['min_horizontal_movement']:.3f}m")
                return False
        
        return True
    
    def _calculate_movement_vector(self, detector):
        """Calculate movement vector from start to end pose."""
        if detector['start_pose'] is None or detector['end_pose'] is None:
            return [0.0, 0.0, 0.0, 0.0]
        
        # 计算位置差异
        pos_diff = [
            (detector['end_pose'][0] - detector['start_pose'][0]),
            (detector['end_pose'][1] - detector['start_pose'][1]),
            0.0
        ]

        # 计算角度差异
        yaw_diff = detector['end_pose'][3] - detector['start_pose'][3]
        while yaw_diff > 180:
            yaw_diff -= 360
        while yaw_diff < -180:
            yaw_diff += 360
        
        yaw_diff = 0.0 if -20 < yaw_diff < 20 else yaw_diff
        pos_diff[0] = 0.0 if -0.05 < pos_diff[0] < 0.05 else pos_diff[0]
        pos_diff[1] = 0.0 if -0.05 < pos_diff[1] < 0.05 else pos_diff[1]
        print(f"修正后的yaw:{yaw_diff}")

        return [pos_diff[0], pos_diff[1], pos_diff[2], yaw_diff]
    
    def _reset_detector_state(self, detector):
        """Reset detector state for next action."""
        detector['state'] = 'ground'
        detector['lift_time'] = None
        detector['ground_time'] = None
        detector['start_pose'] = None
        detector['end_pose'] = None
        detector['pose_history'] = []
        detector['action_duration'] = 0.0
    
    def should_perform_calibration(self, pose_type):
        """Check if calibration should be performed."""
        config = self.movement_detector['complete_action']
        detector = self.complete_action_detector[pose_type]
        
        # 如果自适应阈值未启用，跳过校准
        if not config.get('adaptive_threshold_enabled', True):
            return False
        
        # 检查该特定pose_type的校准是否已完成
        if detector.get('calibration_complete', False):
            return False
        
        # 确保calibration_samples字段存在
        if 'calibration_samples' not in detector:
            detector['calibration_samples'] = []
        
        # 如果样本数量不足，继续收集
        if len(detector['calibration_samples']) < config.get('calibration_samples', 50):
            return True
        
        return False
    
    def perform_calibration(self, current_height, pose_type):
        """Perform calibration for adaptive thresholds."""
        detector = self.complete_action_detector[pose_type]
        config = self.movement_detector['complete_action']
        
        # 确保calibration_samples字段存在
        if 'calibration_samples' not in detector:
            detector['calibration_samples'] = []
        
        # 添加当前高度到校准样本
        detector['calibration_samples'].append(current_height)
        
        # 检查是否收集了足够的样本
        if len(detector['calibration_samples']) >= config.get('calibration_samples', 50):
            # 计算基准高度（使用中位数，更稳定）
            base_height = np.median(detector['calibration_samples'])
            detector['base_height'] = base_height
            
            # 计算自适应阈值
            lift_offset = config.get('adaptive_lift_offset', 0.02)
            ground_offset = config.get('adaptive_ground_offset', 0.005)
            
            detector['adaptive_lift_threshold'] = base_height + lift_offset
            detector['adaptive_ground_threshold'] = base_height + ground_offset
            
            # 标记该特定pose_type的校准完成
            detector['calibration_complete'] = True
            
            SDKLogger.info(f"Calibration completed for {pose_type}: "
                         f"base_height={base_height:.3f}m, "
                         f"lift_threshold={detector['adaptive_lift_threshold']:.3f}m, "
                         f"ground_threshold={detector['adaptive_ground_threshold']:.3f}m")
        else:
            # 显示校准进度
            progress = len(detector['calibration_samples']) / config.get('calibration_samples', 50) * 100
            if len(detector['calibration_samples']) % 10 == 0:  # 每10个样本显示一次进度
                SDKLogger.info(f"Calibrating {pose_type}: {progress:.1f}% complete "
                             f"({len(detector['calibration_samples'])}/{config.get('calibration_samples', 50)})")
    
    def _get_adaptive_thresholds(self, pose_type):
        """Get adaptive thresholds for the given pose type."""
        config = self.movement_detector['complete_action']
        detector = self.complete_action_detector[pose_type]
        
        # 使用自适应阈值
        if detector['adaptive_lift_threshold'] is not None and detector['adaptive_ground_threshold'] is not None:
            return detector['adaptive_lift_threshold'], detector['adaptive_ground_threshold']
        
        # 如果自适应阈值未设置，回退到固定阈值
        return config['lift_threshold'], config['ground_threshold']
    
    def reset_adaptive_calibration(self, pose_type: str = None):
        """Reset adaptive calibration for specified pose type or all.
        
        Args:
            pose_type: 'left', 'right', 'body', or None for all
        """
        config = self.movement_detector['complete_action']
        
        if pose_type is None:
            # Reset all
            for key in self.complete_action_detector:
                detector = self.complete_action_detector[key]
                detector['calibration_samples'] = []
                detector['base_height'] = None
                detector['adaptive_lift_threshold'] = None
                detector['adaptive_ground_threshold'] = None
                detector['calibration_complete'] = False
            SDKLogger.info("Reset adaptive calibration for all pose types")
        elif pose_type in self.complete_action_detector:
            # Reset specific detector
            detector = self.complete_action_detector[pose_type]
            detector['calibration_samples'] = []
            detector['base_height'] = None
            detector['adaptive_lift_threshold'] = None
            detector['adaptive_ground_threshold'] = None
            detector['calibration_complete'] = False
            SDKLogger.info(f"Reset adaptive calibration for {pose_type}")
        else:
            SDKLogger.warning(f"Invalid pose_type: {pose_type}")
    
    def get_adaptive_calibration_status(self, pose_type: str = None):
        """Get adaptive calibration status.
        
        Args:
            pose_type: 'left', 'right', 'body', or None for all
            
        Returns:
            dict: Calibration status information
        """
        config = self.movement_detector['complete_action']
        
        if pose_type is None:
            # Return status for all
            status = {
                'adaptive_threshold_enabled': config.get('adaptive_threshold_enabled', True),
                'calibration_samples': config.get('calibration_samples', 50),
                'adaptive_lift_offset': config.get('adaptive_lift_offset', 0.02),
                'adaptive_ground_offset': config.get('adaptive_ground_offset', 0.005),
                'detectors': {}
            }
            
            for key in self.complete_action_detector:
                detector = self.complete_action_detector[key]
                # 确保所有必需的字段都存在
                calibration_samples = detector.get('calibration_samples', [])
                base_height = detector.get('base_height', None)
                adaptive_lift_threshold = detector.get('adaptive_lift_threshold', None)
                adaptive_ground_threshold = detector.get('adaptive_ground_threshold', None)
                calibration_complete = detector.get('calibration_complete', False)
                
                status['detectors'][key] = {
                    'samples_collected': len(calibration_samples),
                    'base_height': base_height,
                    'adaptive_lift_threshold': adaptive_lift_threshold,
                    'adaptive_ground_threshold': adaptive_ground_threshold,
                    'calibration_complete': calibration_complete
                }
            
            return status
        elif pose_type in self.complete_action_detector:
            # Return status for specific detector
            detector = self.complete_action_detector[pose_type]
            # 确保所有必需的字段都存在
            calibration_samples = detector.get('calibration_samples', [])
            base_height = detector.get('base_height', None)
            adaptive_lift_threshold = detector.get('adaptive_lift_threshold', None)
            adaptive_ground_threshold = detector.get('adaptive_ground_threshold', None)
            calibration_complete = detector.get('calibration_complete', False)
            
            return {
                'adaptive_threshold_enabled': config.get('adaptive_threshold_enabled', True),
                'calibration_complete': calibration_complete,
                'samples_collected': len(calibration_samples),
                'base_height': base_height,
                'adaptive_lift_threshold': adaptive_lift_threshold,
                'adaptive_ground_threshold': adaptive_ground_threshold
            }
        else:
            SDKLogger.warning(f"Invalid pose_type: {pose_type}")
            return None
    
    def update_movement_detector(self, pose_type, movement_vector):
        """Update movement detector for next action."""
        if pose_type == 0:
            # Convert to numpy arrays for proper addition, then back to list
            initial_pose = np.array(self.movement_detector['initial_left_foot_pose'])
            movement_array = np.array(movement_vector.copy()[:4])
            print(f"movement_array:{movement_array}")
            updated_pose = initial_pose + movement_array
            self.movement_detector['initial_left_foot_pose'] = updated_pose.tolist()
            SDKLogger.info(f"Updated left foot pose: {updated_pose}")
        elif pose_type == 1:
            # Convert to numpy arrays for proper addition, then back to list
            initial_pose = np.array(self.movement_detector['initial_right_foot_pose'])
            movement_array = np.array(movement_vector.copy()[:4])
            updated_pose = initial_pose + movement_array
            self.movement_detector['initial_right_foot_pose'] = updated_pose.tolist()
            SDKLogger.info(f"Updated right foot pose: {updated_pose}")
            
    def get_complete_action_status(self, pose_type: str = None):
        """Get current status of complete action detectors.
        
        Args:
            pose_type: 'left', 'right', 'body', or None for all
            
        Returns:
            dict: Status information
        """
        if pose_type is None:
            return self.complete_action_detector
        elif pose_type in self.complete_action_detector:
            return self.complete_action_detector[pose_type]
        else:
            SDKLogger.warning(f"Invalid pose_type: {pose_type}")
            return None

    def set_complete_action_parameters(self, parameters: dict):
        """Set parameters for complete action detection.
        
        Args:
            parameters: Dictionary with parameter names and values
                - lift_threshold: Height threshold for lift detection
                - ground_threshold: Height threshold for ground contact
                - min_action_duration: Minimum valid action duration
                - max_action_duration: Maximum valid action duration
                - action_buffer_size: Size of pose history buffer
                - min_horizontal_movement: Minimum horizontal movement distance
                - adaptive_threshold_enabled: Enable/disable adaptive threshold
                - calibration_samples: Number of samples for calibration
                - adaptive_lift_offset: Offset from base height for lift threshold
                - adaptive_ground_offset: Offset from base height for ground threshold
        """
        config = self.movement_detector['complete_action']
        for key, value in parameters.items():
            if key in config:
                config[key] = value
                SDKLogger.info(f"Set {key} to {value}")
            else:
                SDKLogger.warning(f"Unknown parameter: {key}")
        
        # 如果修改了自适应阈值相关参数，可能需要重新校准
        adaptive_params = ['adaptive_threshold_enabled', 'calibration_samples', 
                          'adaptive_lift_offset', 'adaptive_ground_offset']
        if any(param in parameters for param in adaptive_params):
            SDKLogger.info("Adaptive threshold parameters changed. Consider resetting calibration if needed.")

    def get_complete_action_parameters(self):
        """Get current parameters for complete action detection."""
        if 'complete_action' in self.movement_detector:
            return self.movement_detector['complete_action']
        else:
            return {}

    def enable_complete_action_detection(self):
        """Enable complete action detection mode."""
        self.set_detection_mode('complete_action')
        self.reset_complete_action_detector()
        SDKLogger.info("Complete action detection enabled")

    def enable_threshold_detection(self):
        """Enable threshold-based detection mode."""
        self.set_detection_mode('threshold')
        SDKLogger.info("Threshold-based detection enabled")

    def enable_adaptive_threshold(self):
        """Enable adaptive threshold detection."""
        config = self.movement_detector['complete_action']
        config['adaptive_threshold_enabled'] = True
        SDKLogger.info("Adaptive threshold detection enabled")

    def disable_adaptive_threshold(self):
        """Disable adaptive threshold detection."""
        config = self.movement_detector['complete_action']
        config['adaptive_threshold_enabled'] = False
        SDKLogger.info("Adaptive threshold detection disabled")

    def force_recalibration(self, pose_type: str = None):
        """Force recalibration of adaptive thresholds.
        
        Args:
            pose_type: 'left', 'right', 'body', or None for all
        """
        self.reset_adaptive_calibration(pose_type)
        SDKLogger.info(f"Forced recalibration for {pose_type if pose_type else 'all pose types'}")

    def get_current_thresholds(self, pose_type: str = None):
        """Get current thresholds being used for detection.
        
        Args:
            pose_type: 'left', 'right', 'body', or None for all
            
        Returns:
            dict: Current threshold values
        """
        if pose_type is None:
            # Return thresholds for all pose types
            thresholds = {}
            for key in self.complete_action_detector:
                lift_threshold, ground_threshold = self._get_adaptive_thresholds(key)
                thresholds[key] = {
                    'lift_threshold': lift_threshold,
                    'ground_threshold': ground_threshold
                }
            return thresholds
        elif pose_type in self.complete_action_detector:
            # Return thresholds for specific pose type
            lift_threshold, ground_threshold = self._get_adaptive_thresholds(pose_type)
            return {
                'lift_threshold': lift_threshold,
                'ground_threshold': ground_threshold
            }
        else:
            SDKLogger.warning(f"Invalid pose_type: {pose_type}")
            return None

    def get_detection_info(self):
        """Get comprehensive detection information."""
        current_mode = self.get_detection_mode()
        
        info = {
            'current_mode': current_mode,
            'threshold_parameters': {
                'horizontal_threshold': self.movement_detector['horizontal_threshold'],
                'vertical_threshold': self.movement_detector['vertical_threshold'],
                'angle_threshold': self.movement_detector['angle_threshold']
            }
        }
        
        # Only include complete action parameters if in complete action mode
        if current_mode == 'complete_action':
            info['complete_action_parameters'] = self.get_complete_action_parameters()
            info['detector_status'] = self.get_complete_action_status()
            info['adaptive_calibration_status'] = self.get_adaptive_calibration_status()
        
        return info
    
    def reset_foot_controller_state(self):
        """Reset foot controller state to prevent torso height issues."""
        # 重置脚位置状态
        self.foot_controller.reset_foot_positions()
        
        # 重置移动检测器的初始脚位置
        self.movement_detector['initial_left_foot_pose'] = None
        self.movement_detector['initial_right_foot_pose'] = None
        self.movement_detector['initial_body_pose'] = None
        
        # 重置完整动作检测器
        self.reset_complete_action_detector()
        
        SDKLogger.info("Foot controller state reset completed")

    def mpc_observation_callback(self, msg):
        """MPC观测回调函数"""
        with self.mpc_mode_lock:
            self.mpc_mode = ModeNumber(msg.mode)
            self.mpc_mode_received = True

    def start_trajectory_executor(self):
        """只有队列有消息时才启动执行线程"""
        while self.active_queue.qsize() == 0 and self.is_running:
            SDKLogger.info("Waiting for trajectory queue to have messages before starting executor thread...")
            time.sleep(0.01)
        if not self.trajectory_executor_started:
            self._start_trajectory_executor()
            self.trajectory_executor_started = True

def get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj):
    """Get foot pose trajectory message."""
    num = len(time_traj)
    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = time_traj
    msg.footIndexTrajectory = [int(idx) for idx in foot_idx_traj]
    msg.footPoseTrajectory = []
    
    for i in range(num):
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]
        foot_pose_msg.torsoPose = torso_traj[i]

        msg.footPoseTrajectory.append(foot_pose_msg)

    return msg

class ModeNumber(IntEnum):
    FF = 0
    FH = 1
    FT = 2
    FS = 3
    HF = 4
    HH = 5
    HT = 6
    HS = 7
    TF = 8
    TH = 9
    TT = 10
    TS = 11
    SF = 12
    SH = 13
    ST = 14
    SS = 15

class KuavoSingleFootController:
    """Single foot controller class."""
    
    def __init__(self, pico_info_transformer):
        """Initialize the single foot controller."""
        self.robot_tools = KuavoRobotTools()
        self.pico_info_transformer = pico_info_transformer

        self.default_foot_spacing = 0.25
        self.safe_foot_spacing = 0.65
        self.safe_back_foot_spacing = 0.3

        self.current_left_foot = np.array([0.0, 0.1, 0.0, 0.0])
        self.current_right_foot = np.array([0.0, -0.1, 0.0, 0.0])
        self.current_torso = np.array([0.0, 0.0, 0.0, 0.0])

        self.mpc_plan_target = [0.0, 0.0, 0.0, 0.0]
        self.mpc_policy_sub = rospy.Subscriber('/humanoid_mpc_policy', mpc_flattened_controller, self.mpc_policy_callback)
        
    def set_current_state(self, left_foot, right_foot, torso):
        """Set current state."""
        self.current_left_foot = np.array(left_foot)
        self.current_right_foot = np.array(right_foot)
        self.current_torso = np.array(torso)
    
    def reset_foot_positions(self):
        """Reset foot positions to default state."""
        self.current_left_foot = np.array([0.0, 0.1, 0.0, 0.0])
        self.current_right_foot = np.array([0.0, -0.1, 0.0, 0.0])
        self.current_torso = np.array([0.0, 0.0, 0.0, 0.0])
        SDKLogger.info("Foot positions reset to default state")
    
    def mpc_policy_callback(self, msg):
        """MPC策略回调函数"""
        state_trajs = msg.planTargetTrajectories.stateTrajectory
        if state_trajs:
            state_traj = state_trajs[-1]
            self.mpc_plan_target[:3] = state_traj.value[6:9]
            # SDKLogger.info(f"MPC plan target: {self.mpc_plan_target}")

    def get_delta_com_z(self, step_length):
        if step_length <= 0.25:
            delta_com_z = 0.0
        elif 0.25 < step_length <= 0.6:
            delta_com_z = (step_length - 0.2) * (0.1 - step_length)
        else:
            delta_com_z = -0.2
        return delta_com_z
    
    def process_movement_pose(self, movement_pose):
        """处理运动轨迹"""

        step_length = np.sqrt(movement_pose[0]**2 + movement_pose[1]**2)

        # max_step_length = 0.2 if movement_pose[0] < 0.0 and not stride2stance else 0.6
        max_step_length = 0.6
        scale = 1.0 if step_length < max_step_length else max_step_length / step_length

        movement_pose[0] = 0.0 if -0.05 < movement_pose[0] < 0.05 else movement_pose[0]
        movement_pose[1] = 0.0 if -0.05 < movement_pose[1] < 0.05 else movement_pose[1]
        movement_pose[3] = 0.0 if -20 < movement_pose[3] < 20 else movement_pose[3]

        movement_pose[0] *= scale
        movement_pose[1] *= scale

        return movement_pose
    
    def get_single_foot_trajectory_msg(self, foot_type, movements, dt=0.4, collision_check=True):
        """Generate single foot control trajectory message."""
        time_traj = []
        foot_idx_traj = []
        foot_traj = []
        torso_traj = []
        
        if self.pico_info_transformer.complete_action_detector['left']['use_real_foot_data']:
            current_left_foot_base = self.robot_tools.get_tf_transform(
                target_frame="odom",
                source_frame="leg_l6_link",
                return_type="pose_quaternion"
            )
            current_right_foot_base = self.robot_tools.get_tf_transform(
                target_frame="odom",
                source_frame="leg_r6_link",
                return_type="pose_quaternion"
            )
            _, _, yaw_l6 = euler_from_quaternion(current_left_foot_base.orientation)
            _, _, yaw_r6 = euler_from_quaternion(current_right_foot_base.orientation)

            SDKLogger.info(f"current_left_foot_base: {current_left_foot_base.position}, yaw_l6: {yaw_l6}")
            SDKLogger.info(f"current_right_foot_base: {current_right_foot_base.position}, yaw_r6: {yaw_r6}")
            SDKLogger.info(f"mpc_plan_target: {self.mpc_plan_target}")

            self.current_left_foot = np.array([*current_left_foot_base.position, yaw_l6]) - self.mpc_plan_target
            self.current_right_foot = np.array([*current_right_foot_base.position, yaw_r6]) - self.mpc_plan_target
        else:
            self.current_left_foot = np.array(self.pico_info_transformer.movement_detector['initial_left_foot_pose'])
            self.current_right_foot = np.array(self.pico_info_transformer.movement_detector['initial_right_foot_pose'])

        SDKLogger.info(f"current_left_foot: {self.current_left_foot}, time: {time.time()}")
        SDKLogger.info(f"current_right_foot: {self.current_right_foot}, time: {time.time()}")

        current_support_foot = self.current_right_foot.copy() if foot_type == 0 else self.current_left_foot.copy()
        current_swing_foot = self.current_left_foot.copy() if foot_type == 0 else self.current_right_foot.copy()
        
        for i, movement in enumerate(movements):
            movement_pos = np.asarray(movement[:3])
            movement_yaw = np.radians(movement[3])
            movement_pose = np.array([*movement_pos, movement_yaw])
            movement_time = min(movement[4], dt) * (i + 1) if len(movement) > 4 else dt * (i + 1)
            time_traj.append(movement_time)
            foot_idx_traj.append(foot_type)
            
            target_pose = current_swing_foot.copy() + movement_pose.copy()
            foot_pose = movement_pose.copy()

            step_length_before = np.linalg.norm(current_swing_foot[:2] - current_support_foot[:2])
            step_length_after = np.linalg.norm(target_pose[:2] - current_support_foot[:2])

            SDKLogger.info(f"foot_pose_before:{foot_pose}")
            foot_pose = self.process_movement_pose(foot_pose)
            SDKLogger.info(f"foot_pose_after:{foot_pose}")

            target_pose_safe = current_swing_foot.copy() + foot_pose.copy()
            step_length_safe = np.linalg.norm(target_pose_safe[:2] - current_support_foot[:2])

            if step_length_safe > self.safe_foot_spacing:
                return None
            
            target_pose_safe[0] = -0.25 if target_pose_safe[0] < -0.25 else target_pose_safe[0]
            
            if foot_type == 0:
                target_pose_safe[1] = current_support_foot[1] + 0.2 if target_pose_safe[1] < current_support_foot[1] + 0.2 else target_pose_safe[1]
            else:
                target_pose_safe[1] = current_support_foot[1] - 0.2 if target_pose_safe[1] > current_support_foot[1] - 0.2 else target_pose_safe[1]

            print(f"target_pose_safe:{target_pose_safe}")

            torso_pose_before = (current_swing_foot.copy() + current_support_foot.copy()) / 2
            torso_pose_after = (target_pose_safe.copy() + current_support_foot.copy()) / 2
            torso_pose = torso_pose_after.copy() - torso_pose_before.copy()

            torso_height_before = self.get_delta_com_z(step_length_before)
            torso_height_after = self.get_delta_com_z(step_length_safe)
            torso_height_offset = torso_height_after - torso_height_before
            torso_pose[2] = torso_height_offset

            target_pose_safe[2] = 0.0

            print(f"step_length_before:{step_length_before}")
            print(f"step_length_after:{step_length_after}")
            print(f"step_length_safe:{step_length_safe}")
            print(f"torso_height_before:{torso_height_before}")
            print(f"torso_height_after:{torso_height_after}")
            print(f"torso_height_offset:{torso_height_offset}")

            print(f"use_real_foot_data: {self.pico_info_transformer.complete_action_detector['left']['use_real_foot_data']}")
            if not self.pico_info_transformer.complete_action_detector['left']['use_real_foot_data']:
                self.pico_info_transformer.update_movement_detector(foot_type, foot_pose)

            foot_traj.append(target_pose_safe)
            torso_traj.append(torso_pose)
            print(f"foot_traj:{foot_traj}")
            print(f"torso_traj:{torso_traj}")

        return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)
    
    def get_dual_foot_trajectory_msg(self, left_trajectory, right_trajectory, 
                                   dt=0.4, swing_height=0.1, mode='alternate'):
        """Generate dual foot control trajectory message."""
        if mode == 'alternate':
            return self._generate_alternate_trajectory(left_trajectory, right_trajectory, dt, swing_height)
        elif mode == 'sync':
            return self._generate_sync_trajectory(left_trajectory, right_trajectory, dt, swing_height)
        else:
            raise ValueError("Mode must be 'alternate' or 'sync'")
    
    def _generate_alternate_trajectory(self, left_traj, right_traj, dt, swing_height):
        """Generate alternate trajectory."""
        time_traj = []
        foot_idx_traj = []
        foot_traj = []
        torso_traj = []
        
        max_steps = max(len(left_traj), len(right_traj))
        
        for i in range(max_steps):
            if i < len(left_traj):
                target = left_traj[i]
                target_time = target[4] if len(target) > 4 else dt * (2*i + 1)
                time_traj.append(target_time)
                foot_idx_traj.append(0)
                
                torso_pos = (self.current_right_foot[:3] + np.array(target[:3])) / 2.0
                torso_yaw = (self.current_right_foot[3] + target[3]) / 2.0
                torso_traj.append([*torso_pos, torso_yaw])
                
                swing_traj = self._generate_swing_trajectory(
                    self.current_left_foot, target[:4], swing_height, i
                )
                foot_traj.append(swing_traj)
                self.current_left_foot = np.array(target[:4])
            
            if i < len(right_traj):
                target = right_traj[i]
                target_time = target[4] if len(target) > 4 else dt * (2*i + 2)
                time_traj.append(target_time)
                foot_idx_traj.append(1)
                
                torso_pos = (self.current_left_foot[:3] + np.array(target[:3])) / 2.0
                torso_yaw = (self.current_left_foot[3] + target[3]) / 2.0
                torso_traj.append([*torso_pos, torso_yaw])
                
                swing_traj = self._generate_swing_trajectory(
                    self.current_right_foot, target[:4], swing_height, i
                )
                foot_traj.append(swing_traj)
                self.current_right_foot = np.array(target[:4])
        
        return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)
    
    def _generate_sync_trajectory(self, left_traj, right_traj, dt, swing_height):
        """Generate synchronous trajectory."""
        time_traj = []
        foot_idx_traj = []
        foot_traj = []
        torso_traj = []
        
        max_steps = max(len(left_traj), len(right_traj))
        
        for i in range(max_steps):
            if i < len(left_traj):
                target = left_traj[i]
                target_time = target[4] if len(target) > 4 else dt * (i + 1)
                time_traj.append(target_time)
                foot_idx_traj.append(0)
                
                swing_traj = self._generate_swing_trajectory(
                    self.current_left_foot, target[:4], swing_height, i
                )
                foot_traj.append(swing_traj)
                self.current_left_foot = np.array(target[:4])
            
            if i < len(right_traj):
                target = right_traj[i]
                target_time = target[4] if len(target) > 4 else dt * (i + 1)
                time_traj.append(target_time)
                foot_idx_traj.append(1)
                
                swing_traj = self._generate_swing_trajectory(
                    self.current_right_foot, target[:4], swing_height, i
                )
                foot_traj.append(swing_traj)
                self.current_right_foot = np.array(target[:4])
            
            torso_pos = (self.current_left_foot[:3] + self.current_right_foot[:3]) / 2.0
            torso_yaw = (self.current_left_foot[3] + self.current_right_foot[3]) / 2.0
            
            if i < len(left_traj):
                torso_traj.append([*torso_pos, torso_yaw])
            if i < len(right_traj):
                torso_traj.append([*torso_pos, torso_yaw])
        
        return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)

    def is_execution_window(self, last: ModeNumber, current: ModeNumber):
        """判断是否从F变为H/T/S"""
        left_just_landed = (last.value & 0b1100) == 0 and (current.value & 0b1100) != 0
        right_just_landed = (last.value & 0b0011) == 0 and (current.value & 0b0011) != 0
        return left_just_landed or right_just_landed

class TimeStampManager:
    """精确时间戳管理器，确保并行检测中的时间戳一致性"""
    
    def __init__(self):
        self.base_time = time.time()  # 使用Unix时间戳作为基准
        self.call_count = 0
        self.lock = threading.Lock()
    
    def get_precise_timestamp(self):
        """获取精确时间戳，确保并行调用中的时间顺序"""
        with self.lock:
            self.call_count += 1
            # 使用0.1ms间隔确保时间戳唯一性，但保持Unix时间戳格式
            return self.base_time + (self.call_count * 0.0001)
    
    def reset_base_time(self):
        """重置基准时间"""
        with self.lock:
            self.base_time = time.time()  # 使用Unix时间戳
            self.call_count = 0
    
    def get_duration(self, start_time, end_time):
        """计算精确的持续时间"""
        return end_time - start_time


class ParallelFootDetector:
    """并行脚部检测器，提供高性能的左右脚同时检测"""
    
    def __init__(self, transformer):
        self.transformer = transformer
        
        # 从配置中获取参数
        config = transformer.movement_detector.get('parallel_detection', {})
        max_workers = config.get('max_workers', 2)
        timeout_ms = config.get('timeout_ms', 50)
        self.debug_mode = config.get('debug_mode', False)
        
        self.executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=max_workers, 
            thread_name_prefix="FootDetector"
        )
        self.time_manager = TimeStampManager()
        self.detection_futures = {}
        self.detection_lock = threading.Lock()
        self.is_running = True
        self.timeout = timeout_ms / 1000.0  # 转换为秒
        
        # 性能监控
        self.performance_stats = {
            'total_detections': 0,
            'successful_detections': 0,
            'timeout_count': 0,
            'error_count': 0,
            'avg_detection_time': 0.0,
            'last_detection_time': 0.0,
            'start_time': time.time()  # 使用Unix时间戳
        }
        
        SDKLogger.info(f"ParallelFootDetector initialized with {max_workers} workers, {timeout_ms}ms timeout")
    
    def detect_parallel(self, robot_urdf_matrices):
        """并行检测左右脚动作
        
        Args:
            robot_urdf_matrices: 机器人URDF矩阵
            
        Returns:
            dict: {'left': (detected, movement_vector), 'right': (detected, movement_vector)}
        """
        if not self.is_running:
            return {'left': (False, None), 'right': (False, None)}
        
        for side in ['left', 'right']:
            current_foot_pose = self.transformer.compute_foot_pose(robot_urdf_matrices, side)
            if self.transformer.should_perform_calibration(side):
                self.transformer.perform_calibration(current_foot_pose[2], side)

        # 记录开始时间
        start_time = time.perf_counter()
        
        # 获取基准时间戳
        base_time = self.time_manager.get_precise_timestamp()
        
        # 准备检测任务 - 使用相同的时间戳，避免时间偏移导致的问题
        left_task = partial(
            self._detect_single_foot_isolated, 
            robot_urdf_matrices, 'left', base_time
        )
        right_task = partial(
            self._detect_single_foot_isolated, 
            robot_urdf_matrices, 'right', base_time  # 移除0.1ms偏移
        )
        
        # 提交并行任务
        with self.detection_lock:
            # 取消之前的任务（如果还在运行）
            for future in self.detection_futures.values():
                if not future.done():
                    future.cancel()
            
            # 提交新任务
            self.detection_futures['left'] = self.executor.submit(left_task)
            self.detection_futures['right'] = self.executor.submit(right_task)
        
        # 收集结果
        results = {}
        timeout_count = 0
        error_count = 0
        
        for side, future in self.detection_futures.items():
            try:
                result = future.result(timeout=self.timeout)  # 使用配置的超时时间
                results[side] = result
                
                if self.debug_mode:
                    SDKLogger.debug(f"{side} foot detection completed: {result[0]}")
                    
            except concurrent.futures.TimeoutError:
                SDKLogger.warning(f"{side} foot detection timeout after {self.timeout*1000:.1f}ms")
                results[side] = (False, None)
                timeout_count += 1
            except Exception as e:
                SDKLogger.error(f"Error in {side} foot detection: {e}")
                results[side] = (False, None)
                error_count += 1
        
        # 更新性能统计
        detection_time = time.perf_counter() - start_time
        self._update_performance_stats(detection_time, timeout_count, error_count, results)
        
        if self.debug_mode:
            SDKLogger.debug(f"Parallel detection results: {results}")
        
        return results
    
    def _detect_single_foot_isolated(self, robot_urdf_matrices, side, base_time):
        """隔离的单脚检测逻辑，在线程池中执行
        
        Args:
            robot_urdf_matrices: 机器人URDF矩阵
            side: 'left' 或 'right'
            base_time: 基准时间戳
            
        Returns:
            tuple: (detected, movement_vector)
        """
        try:
            # 计算脚部姿态
            current_foot_pose = self.transformer.compute_foot_pose(robot_urdf_matrices, side)
        except Exception as e:
            SDKLogger.error(f"Error getting {side}_foot pose: {e}")
            return False, None
        
        # 使用精确时间戳进行检测
        detection_result = self._detect_with_precise_time(current_foot_pose, side, base_time)
        
        if detection_result[0]:  # 如果检测到动作
            foot_type = 0 if side.lower() == 'left' else 1
            if self.transformer.complete_action_detector['left']['use_real_foot_data']:
                # 将foot_type和movement_vector一起放入movement_queue
                movement_item = {'foot_type': foot_type, 'movement_vector': detection_result[1]}
                success = self.transformer.add_trajectory_to_queue(movement_item)
                SDKLogger.info(f"add movement time: {time.time()}")
            else:
                # 使用transformer的控制器实例生成轨迹
                foot_pose_traj_msg = self.transformer.foot_controller.get_single_foot_trajectory_msg(
                    foot_type, [detection_result[1]], 0.4, True
                )
                success = self.transformer.add_trajectory_to_queue(foot_pose_traj_msg)
            if not success:
                SDKLogger.warning(f"Failed to add {side} foot trajectory to queue")
            
            return True, detection_result[1]
        
        return False, None
    
    def _detect_with_precise_time(self, pose, side, base_time):
        """使用精确时间戳进行动作检测
        
        Args:
            pose: 脚部姿态 [x, y, z, yaw]
            side: 'left' 或 'right'
            base_time: 基准时间戳（Unix时间戳格式）
            
        Returns:
            tuple: (detected, movement_vector)
        """
        # 不替换全局时间函数，而是直接调用检测方法
        # 这样可以避免影响其他代码的时间计算
        
        # 直接调用检测方法，不使用时间偏移
        detection_result = self.transformer.detect_significant_movement(pose, side.lower())
        return detection_result
    
    def shutdown(self):
        """关闭并行检测器"""
        self.is_running = False
        
        # 取消所有正在运行的任务
        with self.detection_lock:
            for future in self.detection_futures.values():
                if not future.done():
                    future.cancel()
        
        # 关闭线程池
        self.executor.shutdown(wait=True)
        # SDKLogger.info("ParallelFootDetector shutdown completed")
    
    def _update_performance_stats(self, detection_time, timeout_count, error_count, results):
        """更新性能统计"""
        stats = self.performance_stats
        stats['total_detections'] += 1
        stats['timeout_count'] += timeout_count
        stats['error_count'] += error_count
        stats['last_detection_time'] = detection_time
        
        # 计算平均检测时间
        if stats['total_detections'] == 1:
            stats['avg_detection_time'] = detection_time
        else:
            stats['avg_detection_time'] = (
                (stats['avg_detection_time'] * (stats['total_detections'] - 1) + detection_time) 
                / stats['total_detections']
            )
        
        # 统计成功检测次数
        successful = sum(1 for result in results.values() if result[0])
        stats['successful_detections'] += successful
    
    def get_performance_stats(self):
        """获取性能统计信息"""
        stats = self.performance_stats.copy()
        uptime = time.time() - stats['start_time']
        stats['uptime_seconds'] = uptime
        stats['detection_rate'] = stats['total_detections'] / max(uptime, 1.0)  # 检测频率
        stats['success_rate'] = (
            stats['successful_detections'] / max(stats['total_detections'], 1) * 100
        )  # 成功率百分比
        return stats
    
    def reset_performance_stats(self):
        """重置性能统计"""
        self.performance_stats = {
            'total_detections': 0,
            'successful_detections': 0,
            'timeout_count': 0,
            'error_count': 0,
            'avg_detection_time': 0.0,
            'last_detection_time': 0.0,
            'start_time': time.time()  # 使用Unix时间戳
        }
        SDKLogger.info("Performance stats reset")