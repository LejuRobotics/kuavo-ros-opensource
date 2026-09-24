import math
import os
import sys
import threading
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from drake_trans import *
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList, JoySticks
from kuavo_msgs.msg import ControllerSwitchEvent, headBodyPose
import rospy
import time
import tf
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from geometry_msgs.msg import TransformStamped,Twist
from tf2_msgs.msg import TFMessage
from constanst import gesture_corresponding_hand_position

# Array of bone names
bone_names = [
    "LeftArmUpper",
    "LeftArmLower",
    "RightArmUpper",
    "RightArmLower",
    "LeftHandPalm",
    "RightHandPalm",
    "LeftHandThumbMetacarpal",
    "LeftHandThumbProximal",
    "LeftHandThumbDistal",
    "LeftHandThumbTip",
    "LeftHandIndexTip",
    "LeftHandMiddleTip",
    "LeftHandRingTip",
    "LeftHandLittleTip",
    "RightHandThumbMetacarpal",
    "RightHandThumbProximal",
    "RightHandThumbDistal",
    "RightHandThumbTip",
    "RightHandIndexTip",
    "RightHandMiddleTip",
    "RightHandRingTip",
    "RightHandLittleTip",
    "Root",  # Added new bone
    "Chest"  # Added new bone
]

FINGER_PARTS = [
    "HandPalm",
    "HandThumbMetacarpal",
    "HandThumbProximal",
    "HandThumbDistal",
    "HandThumbTip",
    "HandIndexTip",
    "HandMiddleTip",
    "HandRingTip",
    "HandLittleTip"
]


# Create bone_name_to_index dictionary
bone_name_to_index = {name: index for index, name in enumerate(bone_names)}

# List of bone IDs in order
fullBodyBoneIds_leju_arms = [bone_name_to_index[name] for name in bone_names]

# Create a reverse mapping from index to bone name
index_to_bone_name = {index: bone_names[index] for index in range(len(bone_names))}

bias_chest_to_base_link = [0.0, 0, 0.42]

class HeadBodyPose:
    head_pitch = 0.0
    head_yaw = 0.0
    body_yaw = 0.0
    body_x = 0.0
    body_y = 0.0
    body_roll = 0.0
    body_pitch = 6 * np.pi / 180.0
    body_height = 0.74


class Quest3ArmInfoTransformer:
    def __init__(self, model_path, vis_pub=True, predict_gesture=False,
                 eef_visual_stl_files=None, hand_reference_mode="thumb_index",
                 debug_pub=False, debug_publish_hz=10.0):
        self.predict_gesture = predict_gesture
        self.model_path = model_path
        self.vis_pub = vis_pub
        self.hand_reference_mode = hand_reference_mode  # "fingertips", "middle_finger", "thumb_index"
        self.debug_pub = bool(debug_pub)
        self.debug_publish_hz = max(0.0, float(debug_publish_hz))
        self._debug_publish_period = (
            1.0 / self.debug_publish_hz if self.debug_publish_hz > 0.0 else 0.0
        )
        self._last_debug_publish_monotonic = float("-inf")
        self._finger_state_lock = threading.RLock()
        self.left_finger_joints = None # down dom to 6 dof
        self.right_finger_joints = None
        self.left_hand_pose = None # pos + quaternion
        self.right_hand_pose = None
        self.left_elbow_pos = None
        self.right_elbow_pos = None
        self.pose_info_list = None # PoseInfoList message
        self.timestamp_ms = 0.0
        self.is_high_confidence = True
        self.is_hand_tracking = False
        self.ok_gesture_counts = 0
        self.shot_gesture_counts = 0
        self.is_runing = False     
        self.control_torso = False   
        self.init_R_wC = rpy_to_matrix([np.pi/2, 0, np.pi/2]) # rotation matrix from world to chest
        self.init_R_wLS = None # rotation matrix from world to left shoulder
        self.init_R_wRS = None # rotation matrix from world to right shoulder
        self.left_shoulder_rpy_in_robot = [0.0, 0.0, 0.0] # expressed in robot frame
        self.right_shoulder_rpy_in_robot = [0.0, 0.0, 0.0]
        self.chest_axis_agl = [0.0, 0.0, 0.0]  # in robot frame
        self.upper_arm_length = 22.0
        self.lower_arm_length = 19.0
        self.base_shoulder_x_bias = 0.0
        self.base_shoulder_y_bias = 0.15
        self.base_shoulder_z_bias = 0.42
        self.shoulder_width = 0.15
        self.head_body_pose = HeadBodyPose()
        # Add new variables for arm length measurement
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
        # 添加上臂向量角速度限制相关变量
        self.last_left_upper_arm_vec = None
        self.last_right_upper_arm_vec = None
        self.last_left_arm_timestamp = None
        self.last_right_arm_timestamp = None
        self.max_shoulder_angular_velocity = 4.0  # rad/s, 当前控制器配置的肩关节最大角速度
        # 添加小臂向量角速度限制相关变量
        self.last_left_lower_arm_vec = None
        self.last_right_lower_arm_vec = None
        self.max_elbow_angular_velocity = 4.0  # rad/s, 当前控制器配置的肘关节最大角速度
        self.enable_elbow_angular_velocity_limit = False
        self._motion_limit_profiles = {}
        self._active_motion_limit_profile = {}
        self._motion_limit_profile_name = None
        self.controller_switch_event_sub = None
        # 添加上臂向量空间约束参数
        self.upper_arm_cone_angle_forward = 140.0 * np.pi / 180.0  # 前方锥角（相对于前方x轴）
        # Marker publishers are debugging-only. Avoid registering them in the
        # production path, otherwise a regex rosbag subscriber makes every VR
        # callback construct and serialize eight extra messages.
        self.marker_pub = None
        self.marker_pub_right = None
        self.marker_pub_elbow = None
        self.marker_pub_elbow_right = None
        self.marker_pub_shoulder = None
        self.marker_pub_human_array_left = None
        self.marker_pub_human_array_right = None
        self.marker_pub_shoulder_right = None
        self.marker_pub_shoulder_quest3 = None
        self.marker_pub_shoulder_quest3_right = None
        self.marker_pub_chest = None
        self.marker_pub_upper_arm_constraint_left = None
        self.marker_pub_upper_arm_constraint_right = None
        if self.vis_pub:
            self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
            self.marker_pub_right = rospy.Publisher("visualization_marker_right", Marker, queue_size=10)
            self.marker_pub_elbow = rospy.Publisher("visualization_marker/elbow", Marker, queue_size=10)
            self.marker_pub_elbow_right = rospy.Publisher("visualization_marker_right/elbow", Marker, queue_size=10)
            self.marker_pub_shoulder = rospy.Publisher("visualization_marker/shoulder", Marker, queue_size=10)
            self.marker_pub_human_array_left = rospy.Publisher("visualization_marker/human_array_left", MarkerArray, queue_size=10)
            self.marker_pub_human_array_right = rospy.Publisher("visualization_marker/human_array_right", MarkerArray, queue_size=10)
            self.marker_pub_shoulder_right = rospy.Publisher("visualization_marker_right/shoulder", Marker, queue_size=10)
            self.marker_pub_shoulder_quest3 = rospy.Publisher("visualization_marker/shoulder_quest3", Marker, queue_size=10)
            self.marker_pub_shoulder_quest3_right = rospy.Publisher("visualization_marker_right/shoulder_quest3", Marker, queue_size=10)
            self.marker_pub_chest = rospy.Publisher("visualization_marker_chest", Marker, queue_size=10)
            self.marker_pub_upper_arm_constraint_left = rospy.Publisher("visualization_marker/upper_arm_constraint_left", Marker, queue_size=10)
            self.marker_pub_upper_arm_constraint_right = rospy.Publisher("visualization_marker/upper_arm_constraint_right", Marker, queue_size=10)
        self.joint_state_puber = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.shoulder_angle_puber = None
        self.chest_axis_puber = None
        self.hand_thumb_pub = None
        if self.debug_pub:
            self.shoulder_angle_puber = rospy.Publisher(
                '/quest3_debug/shoulder_angle', Float32MultiArray, queue_size=10
            )
            self.chest_axis_puber = rospy.Publisher(
                '/quest3_debug/chest_axis', Float32MultiArray, queue_size=10
            )
            self.hand_thumb_pub = rospy.Publisher(
                '/hand_thumb', Float32MultiArray, queue_size=10
            )
        self.head_body_pose_puber = rospy.Publisher('/kuavo_head_body_orientation_data', headBodyPose, queue_size=10)
        self.head_body_pose_control_puber = rospy.Publisher('/kuavo_head_body_orientation', headBodyPose, queue_size=10)    


        self.left_joystick = None
        self.right_joystick = None
        if eef_visual_stl_files is not None:
            self.eef_visual_stl_files = eef_visual_stl_files
        if self.predict_gesture:
            from hand_gesture_predictor import HandGesturePredictor
            hand_gesture_model_path = os.path.join(current_dir, "hand_gesture_model.onnx")
            self.hand_gesture_predictor = HandGesturePredictor(hand_gesture_model_path)
        self.listener = tf.TransformListener()
        if rospy.has_param("/quest3/upper_arm_length"):
            self.upper_arm_length = rospy.get_param("/quest3/upper_arm_length")
            print(f"get rosparams upper_arm_length: {self.upper_arm_length}")
        else:
            print("/quest3/upper_arm_length not found")
        if rospy.has_param("/quest3/lower_arm_length"):
            self.lower_arm_length = rospy.get_param("/quest3/lower_arm_length")
            print(f"get rosparams lower_arm_length: {self.lower_arm_length}")
        else:
            print("/quest3/lower_arm_length not found")
        if rospy.has_param("/quest3/base_height_offset"):
            bias_chest_to_base_link[2] = rospy.get_param("/quest3/base_height_offset")
            print(f"get rosparams base_height_offset: {bias_chest_to_base_link[2]}")
        else:
            print("/quest3/base_height_offset not found")
        if rospy.has_param("/quest3/base_chest_offset_x"):
            bias_chest_to_base_link[0] = rospy.get_param("/quest3/base_chest_offset_x")
            print(f"get rosparams base_chest_offset_x: {bias_chest_to_base_link[0]}")
        else:
            print("/quest3/base_chest_offset_x not found")
        if rospy.has_param("/quest3/shoulder_width"):
            self.shoulder_width = rospy.get_param("/quest3/shoulder_width")
            print(f"get rosparams shoulder_width: {self.shoulder_width}")
        else:
            print("/quest3/shoulder_width not found")
        if rospy.has_param("/quest3/max_shoulder_angular_velocity"):
            self.max_shoulder_angular_velocity = rospy.get_param("/quest3/max_shoulder_angular_velocity")
            print(f"get rosparams max_shoulder_angular_velocity: {self.max_shoulder_angular_velocity}")
        else:
            print("/quest3/max_shoulder_angular_velocity not found, using default 4.0 rad/s")
        if rospy.has_param("/quest3/max_elbow_angular_velocity"):
            self.max_elbow_angular_velocity = rospy.get_param("/quest3/max_elbow_angular_velocity")
            print(f"get rosparams max_elbow_angular_velocity: {self.max_elbow_angular_velocity}")
        else:
            print("/quest3/max_elbow_angular_velocity not found, using default 4.0 rad/s")
        self._load_motion_limit_profiles()
        self.controller_switch_event_sub = rospy.Subscriber(
            "/humanoid_controller/controller_switch_event",
            ControllerSwitchEvent,
            self._controller_switch_event_callback,
            queue_size=1,
        )
        if rospy.has_param("/quest3/upper_arm_cone_angle_forward"):
            self.upper_arm_cone_angle_forward = rospy.get_param("/quest3/upper_arm_cone_angle_forward") * np.pi / 180.0
            print(f"get rosparams upper_arm_cone_angle_forward: {self.upper_arm_cone_angle_forward * 180.0 / np.pi} degrees")
        if rospy.has_param("/quest3/upper_arm_cone_angle_side"):
            self.upper_arm_cone_angle_side = rospy.get_param("/quest3/upper_arm_cone_angle_side") * np.pi / 180.0
            print(f"get rosparams upper_arm_cone_angle_side: {self.upper_arm_cone_angle_side * 180.0 / np.pi} degrees")
        if rospy.has_param("/quest3/upper_arm_cone_angle_back"):
            self.upper_arm_cone_angle_back = rospy.get_param("/quest3/upper_arm_cone_angle_back") * np.pi / 180.0
            print(f"get rosparams upper_arm_cone_angle_back: {self.upper_arm_cone_angle_back * 180.0 / np.pi} degrees")
    
    def _load_motion_limit_profiles(self):
        """Load MoRE-specific limits without changing other controllers' legacy settings."""
        self._motion_limit_profiles = {
            "base": {
                # 非 MoRE 保持 MR 父版本行为：肩部沿用原有全局参数，肘部不限速。
                "max_shoulder_angular_velocity": self.max_shoulder_angular_velocity,
                "max_elbow_angular_velocity": self.max_elbow_angular_velocity,
                "enable_elbow_angular_velocity_limit": False,
            },
            "more": {
                "max_shoulder_angular_velocity": rospy.get_param(
                    "/quest3/more_max_shoulder_angular_velocity", 2.4
                ),
                "max_elbow_angular_velocity": rospy.get_param(
                    "/quest3/more_max_elbow_angular_velocity", 2.4
                ),
                "enable_elbow_angular_velocity_limit": rospy.get_param(
                    "/quest3/more_enable_elbow_angular_velocity_limit", True
                ),
            },
        }
        self._apply_motion_limit_profile("base")

    def _apply_motion_limit_profile(self, profile_name):
        if profile_name == self._motion_limit_profile_name:
            return
        profile = self._motion_limit_profiles[profile_name]
        # Replacing the profile reference lets read_msg take one consistent snapshot
        # even if the controller-switch callback runs concurrently.
        self._active_motion_limit_profile = profile
        self._motion_limit_profile_name = profile_name
        self.max_shoulder_angular_velocity = profile["max_shoulder_angular_velocity"]
        self.max_elbow_angular_velocity = profile["max_elbow_angular_velocity"]
        self.enable_elbow_angular_velocity_limit = profile["enable_elbow_angular_velocity_limit"]
        rospy.loginfo(
            "Quest3 arm rate-limit profile=%s: shoulder=%.3f rad/s, elbow=%s (%.3f rad/s)",
            profile_name,
            self.max_shoulder_angular_velocity,
            "enabled" if self.enable_elbow_angular_velocity_limit else "disabled",
            self.max_elbow_angular_velocity,
        )

    def _controller_switch_event_callback(self, msg):
        profile_name = "more" if msg.to_controller == "more_controller" else "base"
        self._apply_motion_limit_profile(profile_name)

    def constrain_upper_arm_vector(self, upper_arm_vec, side):
        """
        限制上臂向量在允许的锥形空间内（以肩膀自然姿态为中心的锥）
        Args:
            upper_arm_vec: 上臂向量（从肩膀指向手肘，在机器人base_link坐标系中）
            side: "Left" or "Right"
        Returns:
            受约束后的上臂向量
        """
        vec_len = np.linalg.norm(upper_arm_vec)
        if vec_len < 1e-6:
            return upper_arm_vec
        
        # 归一化方向向量
        direction = upper_arm_vec / vec_len
        
        # 定义肩膀自然姿态的参考方向（略微向前和向外）
        # 左臂：向前、向左、略向下  [1.0, 0.3, -0.1]
        # 右臂：向前、向右、略向下  [1.0, -0.3, -0.1]
        if side == "Left":
            reference_vec = np.array([1.0, 0.3, -0.1])
        else:
            reference_vec = np.array([1.0, -0.3, -0.1])
        reference_vec = reference_vec / np.linalg.norm(reference_vec)
        
        # 计算当前方向与参考方向的夹角
        angle = np.arccos(np.clip(np.dot(direction, reference_vec), -1.0, 1.0))
        
        if angle > self.upper_arm_cone_angle_forward / 2:
            # 投影回锥面
            axis = np.cross(reference_vec, direction)
            axis_norm = np.linalg.norm(axis)
            if axis_norm > 1e-6:
                axis = axis / axis_norm
                # 旋转到锥面边界
                target_angle = self.upper_arm_cone_angle_forward / 2
                constrained_direction = reference_vec * np.cos(target_angle) + \
                                      np.cross(axis, reference_vec) * np.sin(target_angle) + \
                                      axis * np.dot(axis, reference_vec) * (1 - np.cos(target_angle))
                # 返回保持原长度的约束向量
                return constrained_direction * vec_len
        
        # 未超出约束，返回原向量
        return upper_arm_vec
    
    def visualize_upper_arm_constraint(self, shoulder_pos, side):
        """
        可视化上臂的锥形约束空间（以肩膀自然姿态为中心）
        Args:
            shoulder_pos: 肩膀位置
            side: "Left" or "Right"
        """
        if not self.vis_pub:
            return
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"upper_arm_constraint_{side}"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # 线宽
        marker.color.a = 0.3
        marker.color.r = 1.0 if side == "Left" else 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0 if side == "Left" else 1.0
        
        # 定义锥轴方向（与约束函数中的参考方向一致）
        if side == "Left":
            cone_axis = np.array([1.0, 0.3, -0.1])
        else:
            cone_axis = np.array([1.0, -0.3, -0.1])
        cone_axis = cone_axis / np.linalg.norm(cone_axis)
        
        # 绘制锥形约束空间（高度减小到0.15米，约为上臂长度的一半）
        self._add_cone_to_marker(marker, shoulder_pos, cone_axis, 
                                 self.upper_arm_cone_angle_forward, 0.1)
        
        if side == "Left":
            self.marker_pub_upper_arm_constraint_left.publish(marker)
        else:
            self.marker_pub_upper_arm_constraint_right.publish(marker)
    
    def _add_cone_to_marker(self, marker, apex, direction, cone_angle, height):
        """
        向marker添加锥形的线框
        Args:
            marker: Marker对象
            apex: 锥顶位置
            direction: 锥轴方向（归一化）
            cone_angle: 锥角（全角）
            height: 锥的高度
        """
        direction = direction / np.linalg.norm(direction)
        
        # 计算锥底半径
        radius = height * np.tan(cone_angle / 2)
        
        # 计算锥底圆心
        base_center = apex + direction * height
        
        # 构建垂直于锥轴的两个正交向量
        if abs(direction[2]) < 0.9:
            perpendicular1 = np.cross(direction, np.array([0, 0, 1]))
        else:
            perpendicular1 = np.cross(direction, np.array([0, 1, 0]))
        perpendicular1 = perpendicular1 / np.linalg.norm(perpendicular1)
        perpendicular2 = np.cross(direction, perpendicular1)
        perpendicular2 = perpendicular2 / np.linalg.norm(perpendicular2)
        
        # 在锥底圆上生成点
        num_points = 16
        base_points = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            point = base_center + radius * (np.cos(angle) * perpendicular1 + 
                                           np.sin(angle) * perpendicular2)
            base_points.append(point)
        
        # 添加从锥顶到锥底的线
        for point in base_points:
            # 从锥顶到锥底边缘
            p1 = Pose().position
            p1.x, p1.y, p1.z = apex
            marker.points.append(p1)
            p2 = Pose().position
            p2.x, p2.y, p2.z = point
            marker.points.append(p2)
        
        # 添加锥底圆周的线
        for i in range(num_points):
            p1 = Pose().position
            p1.x, p1.y, p1.z = base_points[i]
            marker.points.append(p1)
            p2 = Pose().position
            p2.x, p2.y, p2.z = base_points[(i + 1) % num_points]
            marker.points.append(p2)
    
    def limit_arm_vector_rotation(self, current_vec, last_vec, dt, max_angular_velocity=None):
        """
        限制上臂向量的旋转角速度
        Args:
            current_vec: 当前上臂向量
            last_vec: 上一帧上臂向量
            dt: 时间步长 (s)
        Returns:
            限制后的上臂向量
        """
        if last_vec is None or dt <= 0:
            return current_vec
        
        # 归一化向量
        len_current = np.linalg.norm(current_vec)
        len_last = np.linalg.norm(last_vec)
        
        if len_current < 1e-6 or len_last < 1e-6:
            return current_vec
        
        v1 = last_vec / len_last
        v2 = current_vec / len_current
        
        # 计算夹角
        cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle = np.arccos(cos_angle)
        
        # 计算最大允许角度
        if max_angular_velocity is None:
            max_angular_velocity = self.max_shoulder_angular_velocity
        max_angle = max_angular_velocity * dt
        
        if angle > max_angle:
            # 使用球面线性插值 (SLERP) 限制旋转
            ratio = max_angle / angle
            
            # 计算旋转轴
            axis = np.cross(v1, v2)
            axis_norm = np.linalg.norm(axis)
            
            if axis_norm > 1e-6:
                axis = axis / axis_norm
                # 使用 Rodrigues 旋转公式
                limited_dir = v1 * np.cos(max_angle) + \
                             np.cross(axis, v1) * np.sin(max_angle) + \
                             axis * np.dot(axis, v1) * (1 - np.cos(max_angle))
                # 保持原始长度
                return limited_dir * len_current
            else:
                # 向量几乎平行，直接返回
                return current_vec
        
        return current_vec
    
    def limit_arm_vector_rotation_elbow(self, current_vec, last_vec, dt, max_angular_velocity=None):
        """
        限制小臂向量的旋转角速度（肘关节）
        Args:
            current_vec: 当前小臂向量
            last_vec: 上一帧小臂向量
            dt: 时间步长 (s)
        Returns:
            限制后的小臂向量
        """
        if last_vec is None or dt <= 0:
            return current_vec
        
        # 归一化向量
        len_current = np.linalg.norm(current_vec)
        len_last = np.linalg.norm(last_vec)
        
        if len_current < 1e-6 or len_last < 1e-6:
            return current_vec
        
        v1 = last_vec / len_last
        v2 = current_vec / len_current
        
        # 计算夹角
        cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle = np.arccos(cos_angle)
        
        # 计算最大允许角度（使用肘关节的最大角速度）
        if max_angular_velocity is None:
            max_angular_velocity = self.max_elbow_angular_velocity
        max_angle = max_angular_velocity * dt
        
        if angle > max_angle:
            # 使用球面线性插值 (SLERP) 限制旋转
            ratio = max_angle / angle
            
            # 计算旋转轴
            axis = np.cross(v1, v2)
            axis_norm = np.linalg.norm(axis)
            
            if axis_norm > 1e-6:
                axis = axis / axis_norm
                # 使用 Rodrigues 旋转公式
                limited_dir = v1 * np.cos(max_angle) + \
                             np.cross(axis, v1) * np.sin(max_angle) + \
                             axis * np.dot(axis, v1) * (1 - np.cos(max_angle))
                # 保持原始长度
                return limited_dir * len_current
            else:
                # 向量几乎平行，直接返回
                return current_vec
        
        return current_vec
              
    def read_joySticks_msg(self, msg):
        """Compatibility entry point; the absolute IK path passes snapshots instead."""
        with self._finger_state_lock:
            self.left_joystick = [msg.left_trigger, msg.left_grip]
            self.right_joystick = [msg.right_trigger, msg.right_grip]

    def _claim_debug_publish_slot(self):
        """Return true at most ``debug_publish_hz`` times per second."""
        if not (self.debug_pub or self.vis_pub):
            return False
        now = time.monotonic()
        if (self._debug_publish_period > 0.0
                and now - self._last_debug_publish_monotonic < self._debug_publish_period):
            return False
        self._last_debug_publish_monotonic = now
        return True

    @staticmethod
    def _validate_pose_message(msg):
        if len(msg.poses) < len(bone_names):
            return False
        # 镜像 C++ validateInput：关键 4 骨骼位姿须为有限值。
        for index in (bone_name_to_index["LeftHandPalm"],
                      bone_name_to_index["LeftArmLower"],
                      bone_name_to_index["RightHandPalm"],
                      bone_name_to_index["RightArmLower"]):
            pose = msg.poses[index]
            if not (
                    math.isfinite(pose.position.x)
                    and math.isfinite(pose.position.y)
                    and math.isfinite(pose.position.z)
                    and math.isfinite(pose.orientation.x)
                    and math.isfinite(pose.orientation.y)
                    and math.isfinite(pose.orientation.z)
                    and math.isfinite(pose.orientation.w)):
                return False
        return True

    def read_msg(self, msg, collect_timing=False, joystick_snapshot=None,
                 defer_finger_processing=False):
        """
        Read the PoseInfoList message and extract the left and right finger joint angles.

        Args:
            msg: VR bone message.
            collect_timing: Return a fine-grained timing breakdown for diagnostics.
                This is disabled by default so normal VR paths do not pay for the
                additional perf_counter calls.
            defer_finger_processing: Leave finger/gesture work to an external
                latest-frame worker. Defaults to false for compatibility with
                the incremental and visualization nodes that share this class.
        """
        timing_start = time.perf_counter() if collect_timing else None
        if joystick_snapshot is None:
            with self._finger_state_lock:
                if self.left_joystick is not None and self.right_joystick is not None:
                    joystick_snapshot = (
                        tuple(self.left_joystick), tuple(self.right_joystick)
                    )
        if joystick_snapshot is None:
            rospy.logwarn_throttle(1.0, "No joystick message received yet")
            return False
        if not self._validate_pose_message(msg):
            return False

        self.pose_info_list = msg.poses
        self.timestamp_ms = msg.timestamp_ms
        self.is_high_confidence = msg.is_high_confidence
        self.is_hand_tracking = msg.is_hand_tracking
        with self._finger_state_lock:
            self.left_joystick = list(joystick_snapshot[0])
            self.right_joystick = list(joystick_snapshot[1])
        validation_done = time.perf_counter() if collect_timing else None

        if defer_finger_processing:
            fingers_done = validation_done
        else:
            if self.is_hand_tracking:
                self.compute_finger_joints("Left", pose_info_list=msg.poses)
                self.compute_finger_joints("Right", pose_info_list=msg.poses)
            else:
                self.compute_finger_joints_joy("Left", pose_info_list=msg.poses)
                self.compute_finger_joints_joy("Right", pose_info_list=msg.poses)
            fingers_done = time.perf_counter() if collect_timing else None

        arm_timings = self.compute_arm_poses(
            msg.poses, collect_timing=collect_timing
        )
        arm_done = time.perf_counter() if collect_timing else None
        with self._finger_state_lock:
            if not defer_finger_processing:
                self._update_gesture_state_locked()
            is_running = bool(self.is_runing)
        if is_running:
            self.pub_head_body_pose_msg(self.head_body_pose)

        if collect_timing:
            timing_done = time.perf_counter()
            result = {
                "input_validation_ms": (validation_done - timing_start) * 1000.0,
                "finger_compute_ms": (
                    float("nan") if defer_finger_processing
                    else (fingers_done - validation_done) * 1000.0
                ),
                "gesture_publish_ms": (timing_done - arm_done) * 1000.0,
                "read_msg_total_ms": (timing_done - timing_start) * 1000.0,
            }
            result.update(arm_timings)
            return result
        return True

    def process_finger_frame(self, msg, joystick_snapshot):
        """Compute both hands from an immutable latest-frame input.

        This method is intended for the lower-rate finger worker. It never
        changes the arm pose input used by ``compute_arm_poses``.
        """
        if joystick_snapshot is None or not self._validate_pose_message(msg):
            return None
        poses = msg.poses
        if msg.is_hand_tracking:
            left = self.compute_finger_joints(
                "Left", pose_info_list=poses, update_state=False
            )
            right = self.compute_finger_joints(
                "Right", pose_info_list=poses, update_state=False
            )
        else:
            left = self.compute_finger_joints_joy(
                "Left", pose_info_list=poses, update_state=False
            )
            right = self.compute_finger_joints_joy(
                "Right", pose_info_list=poses, update_state=False
            )
        if left is None or right is None:
            return None

        with self._finger_state_lock:
            self.left_finger_joints = list(left)
            self.right_finger_joints = list(right)
            self.left_joystick = list(joystick_snapshot[0])
            self.right_joystick = list(joystick_snapshot[1])
            self._update_gesture_state_locked()
            return {
                "left_finger_joints": list(self.left_finger_joints),
                "right_finger_joints": list(self.right_finger_joints),
                "is_running": bool(self.is_runing),
                "is_hand_tracking": bool(msg.is_hand_tracking),
            }

    def _update_gesture_state_locked(self):
        max_counts = 50
        if self.is_runing_gesture():
            self.ok_gesture_counts += 1
            self.shot_gesture_counts = 0
            if self.ok_gesture_counts >= max_counts:
                self.is_runing = True
        elif self.is_stop_gesture():
            self.shot_gesture_counts += 1
            self.ok_gesture_counts = 0
            if self.shot_gesture_counts >= max_counts:
                self.is_runing = False

    def get_finger_state(self):
        with self._finger_state_lock:
            return {
                "left_finger_joints": (
                    None if self.left_finger_joints is None
                    else list(self.left_finger_joints)
                ),
                "right_finger_joints": (
                    None if self.right_finger_joints is None
                    else list(self.right_finger_joints)
                ),
                "is_running": bool(self.is_runing),
            }


    def get_relative_finger_poses(self, side):
        """
        Get relative finger poses from TF and convert to array format directly
        Args:
            side: 'Left' or 'Right', indicating hand side
        Returns:
            frame_poses: numpy array of length 63 (9 transforms * (3 position + 4 quaternion))
        """
        try:
            frame_poses = np.zeros(63)
            parent_frame = side + "HandPalm"
            
            for i, part in enumerate(FINGER_PARTS):
                try:
                    lookup_start = time.time()
                    child_frame = side + part
                    position, rotation = self.listener.lookupTransform(
                        parent_frame, child_frame, rospy.Time(0)
                    )
                    
                    # Mirror position for right hand
                    if side == "Right":
                        position = [-p for p in position]
                    
                    # Directly fill the array with position and rotation
                    start_idx = i * 7
                    frame_poses[start_idx:start_idx+3] = position
                    frame_poses[start_idx+3:start_idx+7] = rotation
                    
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr(f"TF lookup failed for {child_frame}: {e}")
                    continue
            
            return frame_poses
            
        except Exception as e:
            rospy.logerr(f"Error in get_relative_finger_poses: {e}")
            return np.zeros(63)

    def get_hand_transform(self, side, pose_info_list=None):
        """
        根据配置的hand_reference_mode获取手部变换矩阵
        """
        poses = self.pose_info_list if pose_info_list is None else pose_info_list
        if poses is None:
            return None
            
        if self.hand_reference_mode == "palm":
            return self.pose_info2_transform(poses[bone_name_to_index[side + "HandPalm"]])
        elif self.hand_reference_mode == "fingertips":
            T_hand = self.compute_hand_center_from_fingertips(side, poses)
            if T_hand is None:
                T_hand = self.pose_info2_transform(poses[bone_name_to_index[side + "HandPalm"]])
            return T_hand
        elif self.hand_reference_mode == "middle_finger":
            T_hand = self.compute_hand_center_from_middle_finger(side, poses)
            if T_hand is None:
                T_hand = self.pose_info2_transform(poses[bone_name_to_index[side + "HandPalm"]])
            return T_hand
        elif self.hand_reference_mode == "thumb_index":
            T_hand = self.compute_hand_center_from_thumb_index(side, poses)
            if T_hand is None:
                T_hand = self.pose_info2_transform(poses[bone_name_to_index[side + "HandPalm"]])
            return T_hand
        else:
            # 默认使用手掌
            return self.pose_info2_transform(poses[bone_name_to_index[side + "HandPalm"]])

    def compute_hand_center_from_middle_finger(self, side, pose_info_list=None):
        """
        使用中指尖作为手的参考点
        """
        poses = self.pose_info_list if pose_info_list is None else pose_info_list
        if poses is None:
            return None
            
        try:
            # 获取中指尖的位置
            middle_finger_pose = poses[bone_name_to_index[side + "HandMiddleTip"]]
            middle_finger_pos = [middle_finger_pose.position.x, middle_finger_pose.position.y, middle_finger_pose.position.z]
            
            # 使用手掌的方向
            hand_palm_pose = poses[bone_name_to_index[side + "HandPalm"]]
            hand_orientation = hand_palm_pose.orientation
            
            # 构建变换矩阵
            T_hand_middle = np.eye(4)
            T_hand_middle[:3, 3] = middle_finger_pos
            quat = [hand_orientation.x, hand_orientation.y, hand_orientation.z, hand_orientation.w]
            T_hand_middle[:3, :3] = quaternion_to_matrix(quat)
            
            return T_hand_middle
            
        except (KeyError, IndexError):
            return None

    def compute_hand_center_from_thumb_index(self, side, pose_info_list=None):
        """
        使用拇指和食指中点作为手的参考点
        """
        poses = self.pose_info_list if pose_info_list is None else pose_info_list
        if poses is None:
            return None
            
        try:
            # 获取拇指尖和食指尖的位置
            thumb_pose = poses[bone_name_to_index[side + "HandThumbTip"]]
            index_pose = poses[bone_name_to_index[side + "HandIndexTip"]]
            
            thumb_pos = np.array([thumb_pose.position.x, thumb_pose.position.y, thumb_pose.position.z])
            index_pos = np.array([index_pose.position.x, index_pose.position.y, index_pose.position.z])
            
            # 计算中点
            center_pos = (thumb_pos + index_pos) / 2.0
            
            # 使用手掌的方向
            hand_palm_pose = poses[bone_name_to_index[side + "HandPalm"]]
            hand_orientation = hand_palm_pose.orientation
            
            # 构建变换矩阵
            T_hand_center = np.eye(4)
            T_hand_center[:3, 3] = center_pos
            quat = [hand_orientation.x, hand_orientation.y, hand_orientation.z, hand_orientation.w]
            T_hand_center[:3, :3] = quaternion_to_matrix(quat)
            
            return T_hand_center
            
        except (KeyError, IndexError):
            return None

    def compute_hand_center_from_fingertips(self, side, pose_info_list=None):
        """
        计算手指尖的中心位置作为手的参考点
        """
        poses = self.pose_info_list if pose_info_list is None else pose_info_list
        if poses is None:
            return None
            
        # 获取所有手指尖的位置
        finger_tips = ["HandThumbTip", "HandIndexTip", "HandMiddleTip", "HandRingTip", "HandLittleTip"]
        finger_positions = []
        
        for finger_tip in finger_tips:
            try:
                pose_info = poses[bone_name_to_index[side + finger_tip]]
                finger_positions.append([pose_info.position.x, pose_info.position.y, pose_info.position.z])
            except (KeyError, IndexError):
                continue
        
        if len(finger_positions) < 3:  # 至少需要3个手指尖
            return None
            
        # 计算手指尖的中心位置
        center_pos = np.mean(finger_positions, axis=0)
        
        # 使用手掌的方向
        hand_palm_pose = poses[bone_name_to_index[side + "HandPalm"]]
        hand_orientation = hand_palm_pose.orientation
        
        # 构建变换矩阵
        T_hand_center = np.eye(4)
        T_hand_center[:3, 3] = center_pos
        quat = [hand_orientation.x, hand_orientation.y, hand_orientation.z, hand_orientation.w]
        T_hand_center[:3, :3] = quaternion_to_matrix(quat)
        
        return T_hand_center

    def compute_finger_joints(self, side, pose_info_list=None, update_state=True):
        poses = self.pose_info_list if pose_info_list is None else pose_info_list
        if self.predict_gesture:
            gesture, confidence, inference_time = self.hand_gesture_predictor.predict(self.get_relative_finger_poses(side))
            if gesture in gesture_corresponding_hand_position:
                hand_position = gesture_corresponding_hand_position[gesture]
                finger_joints = [1.7*hand_position[i]/100.0 for i in range(6)]
                if update_state:
                    with self._finger_state_lock:
                        if side == "Left":
                            self.left_finger_joints = finger_joints
                        elif side == "Right":
                            self.right_finger_joints = finger_joints
                return finger_joints
        """
        y轴从手腕指向中指指尖, x轴从拇指位置指向无名指方向
        """
        if poses is None:
            return None
        
        # 根据配置的hand_reference_mode获取手部变换矩阵
        T_hand = self.get_hand_transform(side, poses)
        
        T_finger_thumb_tip = self.pose_info2_transform(poses[bone_name_to_index[side + "HandThumbTip"]])
        finger_index_tip_ori = poses[bone_name_to_index[side + "HandIndexTip"]].orientation
        finger_middle_tip_ori = poses[bone_name_to_index[side + "HandMiddleTip"]].orientation
        finger_ring_tip_ori = poses[bone_name_to_index[side + "HandRingTip"]].orientation
        finger_little_tip_ori = poses[bone_name_to_index[side + "HandLittleTip"]].orientation
        R0_inv = T_hand[:3, :3].T
        finger_index_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_index_tip_ori))
        finger_middle_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_middle_tip_ori))
        finger_ring_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_ring_tip_ori))
        finger_little_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_little_tip_ori))
        # print(f"index: {finger_index_tip_rpy[2]}, middle: {finger_middle_tip_rpy[2]}")
        # finger_joints = [finger_thumb_metacarpal_rpy[2], finger_thumb_proximal_rpy[2], finger_thumb_distal_rpy[2], finger_thumb_tip_rpy[2], 

        p_hf_w = T_finger_thumb_tip[:3, 3] - T_hand[:3, 3]
        p_hf_h = R0_inv @ p_hf_w
        x, z = 0.0, 0.0
        if side == "Left":
            # x:[-0.075, 0]->[open, close]->[0, 100]
            # z:[0.044, 0.08]->[close, open]->[100, 0]
            x_min, x_max = -0.07, -0.01
            z_min, z_max = 0.02, 0.08
            x = min(x_max, max(x_min, p_hf_h[0]))
            z = min(z_max, max(z_min, p_hf_h[2]))
            x = 100.0 * (x - x_min) / (x_max - x_min)
            z = 100.0 * (z - z_min) / (z_max - z_min)
            # print(f"x: {x}")
            
        if side == "Right":
            # x:[0, 0.075]->[close, open]->[100, 0]
            # z:[-0.08, -0.044]->[open, close]
            x_min, x_max = -0.01, 0.06
            z_min, z_max = -0.08, -0.05
            x = min(x_max, max(x_min, p_hf_h[0]))
            z = min(z_max, max(z_min, p_hf_h[2]))
            x = 100 - 100.0 * (x - x_min) / (x_max - x_min)
            z = 100 - 100.0 * (z - z_min) / (z_max - z_min)

        finger_joints = [1.7*x/100.0, 1.7*z/100.0, # TODO: fix thumb finger joints
                finger_index_tip_rpy[0], finger_middle_tip_rpy[0], finger_ring_tip_rpy[0], finger_little_tip_rpy[0]]
        for i in range(len(finger_joints)):
            finger_joints[i] = self.limit_finger_angle(finger_joints[i], back_agl=np.pi/2.0)
        if update_state:
            with self._finger_state_lock:
                if side == "Left":
                    self.left_finger_joints = finger_joints
                elif side == "Right":
                    self.right_finger_joints = finger_joints
        if side == "Left" and self.hand_thumb_pub is not None:
            thumb_msg = Float32MultiArray()
            thumb_msg.data = p_hf_h
            self.hand_thumb_pub.publish(thumb_msg)
        return finger_joints

    def compute_finger_joints_joy(self, side, pose_info_list=None, update_state=True):
        """
        手柄版本的计算手指关节角度
        """
        poses = self.pose_info_list if pose_info_list is None else pose_info_list
        if poses is None:
            return None
        hand_ori = poses[bone_name_to_index[side + "HandPalm"]].orientation
        finger_thumb_metacarpal_ori = poses[bone_name_to_index[side + "HandThumbMetacarpal"]].orientation
        finger_thumb_proximal_ori = poses[bone_name_to_index[side + "HandThumbProximal"]].orientation
        finger_thumb_distal_ori = poses[bone_name_to_index[side + "HandThumbDistal"]].orientation
        finger_thumb_tip_ori = poses[bone_name_to_index[side + "HandThumbTip"]].orientation
        finger_index_tip_ori = poses[bone_name_to_index[side + "HandIndexTip"]].orientation
        finger_middle_tip_ori = poses[bone_name_to_index[side + "HandMiddleTip"]].orientation
        finger_ring_tip_ori = poses[bone_name_to_index[side + "HandRingTip"]].orientation
        finger_little_tip_ori = poses[bone_name_to_index[side + "HandLittleTip"]].orientation
        R0_inv = self.quaternion_msg_to_matrix(hand_ori).T
        finger_thumb_metacarpal_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_metacarpal_ori))
        finger_thumb_proximal_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_proximal_ori))
        finger_thumb_distal_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_distal_ori))
        finger_thumb_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_tip_ori))
        finger_index_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_index_tip_ori))
        finger_middle_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_middle_tip_ori))
        finger_ring_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_ring_tip_ori))
        finger_little_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_little_tip_ori))
        # print(f"index: {finger_index_tip_rpy[2]}, middle: {finger_middle_tip_rpy[2]}")
        # finger_joints = [finger_thumb_metacarpal_rpy[2], finger_thumb_proximal_rpy[2], finger_thumb_distal_rpy[2], finger_thumb_tip_rpy[2], 

        finger_joints = [0.0 for _ in range(6)]
        finger_joints[0] = self.limit_finger_angle(finger_index_tip_rpy[0])
        finger_joints[1] = 1.7
        finger_joints[2] = self.limit_finger_angle(finger_index_tip_rpy[0])
        finger_joints[3] = 0.0 if finger_middle_tip_rpy[0] > 0.0 else 1.7
        finger_joints[4] = 0.0 if finger_ring_tip_rpy[0] > 0.0 else 1.7
        finger_joints[5] = 0.0 if finger_little_tip_rpy[0] > 0.0 else 1.7

        if update_state:
            with self._finger_state_lock:
                if side == "Left":
                    self.left_finger_joints = finger_joints
                elif side == "Right":
                    self.right_finger_joints = finger_joints
        return finger_joints

    def get_finger_joints(self, side):
        with self._finger_state_lock:
            if side == "Left":
                return self.left_finger_joints
            elif side == "Right":
                return self.right_finger_joints
            else:
                print("Invalid side: {}".format(side))
                return None

    @staticmethod
    def limit_finger_angle(angle, min_angle=0.0, max_angle=1.7, back_agl=1/4*np.pi):
        """
        finger joint angle might in the range of [-3.14, 3.14]
        Limit the finger joint angles to within the range [0.0, 1.7].
        back_agl: VR中手指可能存在反向的角度, 这里设置一个阈值, 超过这个阈值, 才认为是反向的, 否则认为是0
        """
        res = angle
        if res < 0.0:
            res += 2.0 * np.pi
        res = np.fmod(res, 2.0 * np.pi)
        if res < min_angle:
            res = min_angle
        if res > max_angle:
            if res < 2*np.pi - back_agl:
                res = max_angle
            else:  # res > 3/2*np.pi and res <= 2*np.pi
                res = 0.0
        return res

    def is_runing_gesture(self):
        """
        ok-ok:->Runing
        """
        is_runing = True
        is_runing &= self.ok_gesture_check(self.left_finger_joints)
        is_runing &= self.ok_gesture_check(self.right_finger_joints)

        # 兼容手柄的情况
        is_joyRun = True
        is_joyRun &= self.joy_ok_gesture_check(self.left_joystick)
        is_joyRun &= self.joy_ok_gesture_check(self.right_joystick)

        # Calculate average arm lengths when OK gesture is detected
        if (is_runing or is_joyRun) and self.measure_arm_length:
            if len(self.left_upper_arm_lengths) > 0:
                self.avg_left_upper_arm_length = sum(self.left_upper_arm_lengths) / len(self.left_upper_arm_lengths)
                self.avg_left_lower_arm_length = sum(self.left_lower_arm_lengths) / len(self.left_lower_arm_lengths)
            if len(self.right_upper_arm_lengths) > 0:
                self.avg_right_upper_arm_length = sum(self.right_upper_arm_lengths) / len(self.right_upper_arm_lengths)
                self.avg_right_lower_arm_length = sum(self.right_lower_arm_lengths) / len(self.right_lower_arm_lengths)
            self.measure_arm_length = False
            print(f"Arm length measurement completed. Left upper: {self.avg_left_upper_arm_length:.2f}, lower: {self.avg_left_lower_arm_length:.2f}, "
                  f"Right upper: {self.avg_right_upper_arm_length:.2f}, lower: {self.avg_right_lower_arm_length:.2f}")

        return is_runing or is_joyRun

    def is_stop_gesture(self):
        """
        shot-shot:->Stop
        """
        is_stop = True
        is_stop &= self.shot_gesture_check(self.left_finger_joints)
        is_stop &= self.shot_gesture_check(self.right_finger_joints)

        # 兼容手柄的情况
        is_joyStop = True
        is_joyStop &= self.joy_shot_gesture_check(self.left_joystick)
        is_joyStop &= self.joy_shot_gesture_check(self.right_joystick)

        if is_stop or is_joyStop:
            self.measure_arm_length = True

        return is_stop or is_joyStop

    @staticmethod
    def ok_gesture_check(finger_pos, min_agl=0.5, max_agl=1.0):
        """
        finger_pos: list of 6 float, [thumb1, thumb2, index, middle, ring, pinky]
        """
        is_ok = True
        is_ok &= (finger_pos[2] > max_agl)
        is_ok &= (finger_pos[3] < min_agl)
        is_ok &= (finger_pos[4] < min_agl)
        return is_ok    

    @staticmethod
    def shot_gesture_check(finger_pos, min_agl=0.5, max_agl=1.0):
        """
        finger_pos: list of 6 float, [thumb1, thumb2, index, middle, ring, pinky]
        """
        is_shot = True
        is_shot &= (finger_pos[2] < min_agl)
        is_shot &= (finger_pos[3] > max_agl)
        is_shot &= (finger_pos[4] > max_agl)
        return is_shot
    
    @staticmethod
    def joy_shot_gesture_check(stick_pos, min_agl=0.5, max_agl=0.8):
        """
        stick_pos : [trigger, grip]
        """
        is_shot = True
        is_shot &= (stick_pos[0] < min_agl)
        is_shot &= (stick_pos[1] > max_agl)
        return is_shot    

    @staticmethod
    def joy_ok_gesture_check(stick_pos):
        """
        stick_pos : [trigger, grip]
        """
        is_ok = True
        is_ok &= (stick_pos[0] > 0.5)
        return is_ok


    @staticmethod
    def quaternion_msg_to_matrix(ori):
        """
        Convert a quaternion to roll-pitch-yaw angles.
        """
        quat = [ori.x, ori.y, ori.z, ori.w]
        mat = quaternion_to_matrix(quat)
        return mat

    @staticmethod
    def quaternion_msg_to_RPY(ori):
        """
        Convert a quaternion to roll-pitch-yaw angles.
        """
        quat = [ori.x, ori.y, ori.z, ori.w]
        rpy = quaternion_to_RPY(quat)
        return rpy

    def get_hand_pose(self, side):
        if side == "Left":
            return self.left_hand_pose, self.left_elbow_pos
        elif side == "Right":
            return self.right_hand_pose, self.right_elbow_pos
        else:
            print("Invalid side: {}".format(side))
            return None, None

    def compute_shoudler_pose(self, R_wS, side):
        """
        R_wS: 3x3 rotation matrix from world to <side> shoulder
        """
        if side == "Left":
            R_01 = self.init_R_wLS.T @ R_wS
            shoulder_rpy = matrix_to_rpy(R_01)
            # align to urdf model, the shoulder angle is negative
            self.left_shoulder_rpy_in_robot[0] = shoulder_rpy[2]
            self.left_shoulder_rpy_in_robot[1] = -shoulder_rpy[0]
            self.left_shoulder_rpy_in_robot[2] = -shoulder_rpy[1]
        elif side == "Right":
            R_01 = self.init_R_wRS.T @ R_wS
            shoulder_rpy = matrix_to_rpy(R_01)
            # align to urdf model, the shoulder angle is positive
            self.right_shoulder_rpy_in_robot[0] = -shoulder_rpy[2]
            self.right_shoulder_rpy_in_robot[1] = -shoulder_rpy[0]
            self.right_shoulder_rpy_in_robot[2] = shoulder_rpy[1]

    def scale_arm_positions(self, shoulder_pos, elbow_pos, hand_pos, human_shoulder_pos, chest_pos, side, adapt_width_gamma):
        """
        Scale the arm positions from human scale to robot scale
        Args:
            shoulder_pos: robot shoulder position
            elbow_pos: human elbow position (VR captured)
            hand_pos: human hand position (VR captured)
            human_shoulder_pos: human shoulder position
            side: "Left" or "Right"
            adapt_width_gamma: adaptation factor for shoulder width adjustment (larger when hand is more inside)
        Returns:
            scaled_elbow_pos: scaled elbow position
            scaled_hand_pos: scaled hand position
        """
        human_upper_arm_length = math.sqrt((elbow_pos[0] - human_shoulder_pos[0])**2 + (elbow_pos[1] - human_shoulder_pos[1])**2 + (elbow_pos[2] - human_shoulder_pos[2])**2)
        human_lower_arm_length = math.sqrt((hand_pos[0] - elbow_pos[0])**2 + (hand_pos[1] - elbow_pos[1])**2 + (hand_pos[2] - elbow_pos[2])**2)

        # Collect arm length data if measurement is active
        if self.measure_arm_length:
            if side == "Left":
                self.left_upper_arm_lengths.append(human_upper_arm_length)
                self.left_lower_arm_lengths.append(human_lower_arm_length)
                if len(self.left_upper_arm_lengths) > self.arm_length_num:
                    self.left_upper_arm_lengths.pop(0)
                    self.left_lower_arm_lengths.pop(0)
            else:
                self.right_upper_arm_lengths.append(human_upper_arm_length)
                self.right_lower_arm_lengths.append(human_lower_arm_length)
                if len(self.right_upper_arm_lengths) > self.arm_length_num:
                    self.right_upper_arm_lengths.pop(0)
                    self.right_lower_arm_lengths.pop(0) 
            # Use the latest measurements for scaling
            radi1 = self.upper_arm_length/human_upper_arm_length
            radi2 = (self.lower_arm_length + self.upper_arm_length)/(human_lower_arm_length + human_upper_arm_length)
        else:
            # Use average measurements for scaling
            if side == "Left":
                radi1 = self.upper_arm_length/human_upper_arm_length
                # radi1 = self.upper_arm_length/self.left_upper_arm_lengths[-1]
                # radi2 = (self.lower_arm_length + self.upper_arm_length)/(self.avg_left_lower_arm_length + self.avg_left_upper_arm_length)
                radi2 = self.lower_arm_length/human_lower_arm_length
                # radi2 = (self.lower_arm_length + self.upper_arm_length)/(self.left_lower_arm_lengths[-1] + self.left_upper_arm_lengths[-1])
            else:
                radi1 = self.upper_arm_length/human_upper_arm_length
                # radi1 = self.upper_arm_length/self.right_upper_arm_lengths[-1]
                # radi2 = (self.lower_arm_length + self.upper_arm_length)/(self.avg_right_lower_arm_length + self.avg_right_upper_arm_length)
                # radi2 = (self.lower_arm_length + self.upper_arm_length)/(self.right_lower_arm_lengths[-1] + self.right_upper_arm_lengths[-1])
                radi2 = self.lower_arm_length/human_lower_arm_length
        
        # Calculate upper arm vector (from shoulder to elbow)
        upper_arm_vec = np.array([elbow_pos[i] - human_shoulder_pos[i] for i in range(3)])
        lower_arm_vec = np.array([hand_pos[i] - elbow_pos[i] for i in range(3)])  
        
        # 限制上臂向量的旋转角速度（根据adapt_width_gamma平滑调整限制强度）
        current_timestamp = rospy.Time.now().to_sec()
        
        # 计算速度限制因子：adapt_width_gamma越大，限制越宽松
        # adapt_width_gamma范围[0, 0.3]，映射到速度限制倍数[1.0, 5.0]
        # 即手越过胸部时允许的速度是正常情况的5倍
        speed_limit_multiplier = 1.0 
        # + (adapt_width_gamma / 0.3) * 2.0  # 1.0 -> 5.0
        motion_limit_profile = self._active_motion_limit_profile
        max_shoulder_angular_velocity = motion_limit_profile["max_shoulder_angular_velocity"]
        max_elbow_angular_velocity = motion_limit_profile["max_elbow_angular_velocity"]
        enable_elbow_angular_velocity_limit = motion_limit_profile["enable_elbow_angular_velocity_limit"]
        
        if side == "Left":
            if self.last_left_arm_timestamp is not None:
                dt = current_timestamp - self.last_left_arm_timestamp
                dt = max(0.001, min(dt, 0.1))  # 限制dt在合理范围内
            else:
                dt = 0.01  # 默认100Hz
            
            upper_arm_vec = self.limit_arm_vector_rotation(
                upper_arm_vec,
                self.last_left_upper_arm_vec,
                dt,
                max_shoulder_angular_velocity * speed_limit_multiplier,
            )
            
            self.last_left_upper_arm_vec = upper_arm_vec.copy()
            self.last_left_arm_timestamp = current_timestamp
        else:  # Right
            if self.last_right_arm_timestamp is not None:
                dt = current_timestamp - self.last_right_arm_timestamp
                dt = max(0.001, min(dt, 0.1))  # 限制dt在合理范围内
            else:
                dt = 0.01  # 默认100Hz
            
            upper_arm_vec = self.limit_arm_vector_rotation(
                upper_arm_vec,
                self.last_right_upper_arm_vec,
                dt,
                max_shoulder_angular_velocity * speed_limit_multiplier,
            )
            
            self.last_right_upper_arm_vec = upper_arm_vec.copy()
            self.last_right_arm_timestamp = current_timestamp
        
        # 限制小臂向量的旋转角速度（肘关节）
        if side == "Left":
            if enable_elbow_angular_velocity_limit:
                lower_arm_vec = self.limit_arm_vector_rotation_elbow(
                    lower_arm_vec,
                    self.last_left_lower_arm_vec,
                    dt,
                    max_elbow_angular_velocity * speed_limit_multiplier,
                )
            self.last_left_lower_arm_vec = lower_arm_vec.copy()
        else:  # Right
            if enable_elbow_angular_velocity_limit:
                lower_arm_vec = self.limit_arm_vector_rotation_elbow(
                    lower_arm_vec,
                    self.last_right_lower_arm_vec,
                    dt,
                    max_elbow_angular_velocity * speed_limit_multiplier,
                )
            self.last_right_lower_arm_vec = lower_arm_vec.copy()
        
        # Apply rotation to upper arm vector based on adapt_width_gamma
        # adapt_width_gamma ranges from 0 (hand far outside) to 0.3 (hand deep inside)
        # Use it to determine rotation angle for arm inward adjustment
        if adapt_width_gamma > 0.0:
            # Maximum rotation angle when adapt_width_gamma is at max (0.3)
            max_rotation_angle_rad_x = hand_pos[0]-chest_pos.position.x
            max_rotation_angle_rad_z = hand_pos[2]-chest_pos.position.z

            max_rotation_angle_rad_x = max(0.25, abs(max_rotation_angle_rad_x))  # Prevent division by zero
            max_rotation_angle_rad_z = max(0.05, max_rotation_angle_rad_z)  # Prevent division by zero           
           # print("✅max_rotation_angle_rad:", 30/max_rotation_angle_rad)
            max_rotation_angle_rad_x = math.sqrt(max_rotation_angle_rad_x)
            max_rotation_angle_rad_z = max_rotation_angle_rad_z*max_rotation_angle_rad_z
            max_rotation_angle = 60.0/max_rotation_angle_rad_x * np.pi / 180.0  # 25 degrees
            
            max_rotation_angle = max_rotation_angle*max_rotation_angle_rad_z
            
            # Normalize adapt_width_gamma to [0, 1] range (0.3 -> 1.0)
            normalized_gamma = min(1.0, adapt_width_gamma / 0.3)
            # Apply non-linear scaling for smoother transition
            rotation_ratio = normalized_gamma ** 1  # Power of 1.5 for moderate non-linearity
            rotation_angle = max_rotation_angle * rotation_ratio
            
            # Rotate upper arm vector around z-axis (vertical axis)
            # Left arm rotates clockwise (negative), right arm rotates counter-clockwise (positive)
            if side == "Left":
                theta = -rotation_angle  # Rotate inward (clockwise viewed from top)
            else:
                theta = rotation_angle   # Rotate inward (counter-clockwise viewed from top)
            
            # Rotation matrix around z-axis
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            rotation_matrix = np.array([
                [cos_theta, -sin_theta, 0],
                [sin_theta, cos_theta, 0],
                [0, 0, 1]
            ])
            
            # Apply rotation to upper arm vector
            upper_arm_vec = rotation_matrix @ upper_arm_vec
        
        upper_arm_vec = self.constrain_upper_arm_vector(upper_arm_vec, side)

        scaled_elbow_pos = shoulder_pos + radi1 * upper_arm_vec
        scaled_hand_pos = scaled_elbow_pos + radi2 * lower_arm_vec

        return scaled_elbow_pos, scaled_hand_pos

    @staticmethod
    def _pose_position(pose):
        return np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=float
        )

    @staticmethod
    def _pose_rotation_matrix(pose):
        return quaternion_to_matrix([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])

    def _build_chest_frame_context(self, pose_info_list):
        """Compute the per-frame chest frame and yaw compensation once."""
        chest_pose = pose_info_list[bone_name_to_index["Chest"]]
        chest_position = self._pose_position(chest_pose)
        R_w_chest = self._pose_rotation_matrix(chest_pose)
        R_initial_chest = self.init_R_wC.T @ R_w_chest
        chest_axis, _ = matrix_to_axis_angle(R_initial_chest)
        chest_rpy = matrix_to_rpy(R_initial_chest)

        self.chest_axis_agl = np.array([0.0, 0.0, chest_axis[1]])
        compensation_rotation = axis_angle_to_matrix(self.chest_axis_agl).T
        R_w_chest_without_yaw = compensation_rotation @ R_w_chest

        self.head_body_pose.body_yaw = chest_axis[1]
        self.head_body_pose.body_pitch = matrix_to_rpy(
            self.init_R_wC.T @ R_w_chest_without_yaw
        )[0]
        self.head_body_pose.body_roll = chest_rpy[2]
        self.head_body_pose.body_x = chest_position[0]
        self.head_body_pose.body_y = chest_position[1]
        self.head_body_pose.body_height = chest_position[2]

        return {
            "pose_info_list": pose_info_list,
            "chest_pose": chest_pose,
            "chest_position": chest_position,
            "chest_axis": chest_axis,
            "compensation_rotation": compensation_rotation,
        }

    def compute_arm_poses(self, pose_info_list=None, collect_timing=False):
        """Compute left and right arm targets from one shared frame context."""
        poses = self.pose_info_list if pose_info_list is None else pose_info_list
        if poses is None:
            return {} if collect_timing else False

        pair_start = time.perf_counter() if collect_timing else None
        publish_debug = self._claim_debug_publish_slot()
        # Gesture processing may finalize arm-length calibration. Hold its
        # short state lock across the pair so left/right never use different
        # calibration modes within one frame.
        with self._finger_state_lock:
            frame_context = self._build_chest_frame_context(poses)
            chest_done = time.perf_counter() if collect_timing else None
            self.compute_hand_pose("Left", frame_context, publish_debug)
            left_done = time.perf_counter() if collect_timing else None
            self.compute_hand_pose("Right", frame_context, publish_debug)
            right_done = time.perf_counter() if collect_timing else None
        self._publish_frame_debug(frame_context, publish_debug)
        debug_done = time.perf_counter() if collect_timing else None

        if collect_timing:
            return {
                "chest_context_compute_ms": (chest_done - pair_start) * 1000.0,
                "left_hand_compute_ms": (left_done - chest_done) * 1000.0,
                "right_hand_compute_ms": (right_done - left_done) * 1000.0,
                "arm_debug_publish_ms": (debug_done - right_done) * 1000.0,
                "arm_pair_compute_ms": (debug_done - pair_start) * 1000.0,
            }
        return True

    def _publish_frame_debug(self, frame_context, publish_debug):
        if not publish_debug:
            return
        if self.shoulder_angle_puber is not None:
            msg = Float32MultiArray()
            msg.data = np.append(
                self.left_shoulder_rpy_in_robot,
                self.right_shoulder_rpy_in_robot,
            )
            self.shoulder_angle_puber.publish(msg)
        if self.chest_axis_puber is not None:
            msg = Float32MultiArray()
            msg.data = frame_context["chest_axis"]
            self.chest_axis_puber.publish(msg)
        if self.vis_pub and self.marker_pub_chest is not None:
            chest_marker = self.construct_point_marker(
                frame_context["chest_position"], 0.1, 0.8
            )
            self.marker_pub_chest.publish(chest_marker)

    def compute_hand_pose(self, side, frame_context=None, publish_debug=None):
        if frame_context is None:
            if self.pose_info_list is None:
                return
            frame_context = self._build_chest_frame_context(self.pose_info_list)
            if publish_debug is None:
                publish_debug = self._claim_debug_publish_slot()
        if publish_debug is None:
            publish_debug = False

        poses = frame_context["pose_info_list"]
        chest_pose = frame_context["chest_pose"]
        chest_position = frame_context["chest_position"]
        compensation_rotation = frame_context["compensation_rotation"]
        elbow_pose = poses[bone_name_to_index[side + "ArmLower"]]
        shoulder_pose = poses[bone_name_to_index[side + "ArmUpper"]]
        hand_pose = poses[bone_name_to_index[side + "HandPalm"]]

        vr_quat = [hand_pose.orientation.x, hand_pose.orientation.y, hand_pose.orientation.z, hand_pose.orientation.w]
        hand_quat_in_w = vr_quat2robot_quat(vr_quat, side, 15*np.pi/180.0 if not self.is_hand_tracking else 0.0) # [x, y, z, w]

        hand_mat_cH = compensation_rotation @ quaternion_to_matrix(hand_quat_in_w)
        hand_quat = matrix_to_quaternion(hand_mat_cH)
        R_wS = self._pose_rotation_matrix(shoulder_pose)
        if side == "Left" and self.init_R_wLS is None:
            self.init_R_wLS = R_wS
        if side == "Right" and self.init_R_wRS is None:
            self.init_R_wRS = R_wS
        self.compute_shoudler_pose(R_wS, side)

        # Only positions are needed below; avoid constructing 4x4 transforms
        # and converting their rotations to RPY.
        hand_pos = compensation_rotation @ (
            self._pose_position(hand_pose) - chest_position
        )
        hand_pos[0] += bias_chest_to_base_link[0]
        hand_pos[1] += bias_chest_to_base_link[1]
        hand_pos[2] += bias_chest_to_base_link[2]
        hand_pos[0] = 0.1 if hand_pos[0] < 0.1 else hand_pos[0]
        
        elbow_pos = compensation_rotation @ (
            self._pose_position(elbow_pose) - chest_position
        )
        elbow_pos[0] += bias_chest_to_base_link[0]
        elbow_pos[2] += bias_chest_to_base_link[2]

        shoulder_pos = compensation_rotation @ (
            self._pose_position(shoulder_pose) - chest_position
        )
        shoulder_pos[0] = bias_chest_to_base_link[0]
        shoulder_pos[2] = bias_chest_to_base_link[2]

        # print(f"{side} HandPalm y position: ✅", hand_pos[1])
        y_distance = abs(hand_pos[1])
        # print(f"{side} HandPalm y distance from chest: ✅ {y_distance:.2f} m")
        # 判断是处理哪只手
        if side == "Left" and hand_pos[1]< 0:
            overchest_b = True
            # print(f"Left hand over chest: ✅ {overchest_b}")
        elif side == "Right" and hand_pos[1] > 0:
            overchest_b = True
            # print(f"Right hand over chest: ✅ {overchest_b}")
        else:
            overchest_b = False   
        

        human_shoulder_pos = list(shoulder_pos[:])
       
        if(y_distance <= self.shoulder_width - 0.1) and (not overchest_b):
            adapt_width_gamma = 1*(self.shoulder_width - 0.1 - y_distance)
        elif(y_distance <= self.shoulder_width - 0.1) and overchest_b:
            adapt_width_gamma = 1*(self.shoulder_width - 0.1 + y_distance)
        elif(y_distance > self.shoulder_width - 0.1) and overchest_b:
            adapt_width_gamma = 1*(self.shoulder_width - 0.1 + y_distance)
        else:
            adapt_width_gamma = 0.0
        
        adapt_width_gamma = min(adapt_width_gamma, 0.3)  # 限制最大适应宽度为0.1m

        if (side == "Right"):
            shoulder_pos[1] = -self.shoulder_width
            # print("✅Right adapt_width_gamma",adapt_width_gamma)
            # +adapt_width_gamma
        elif (side == "Left"):
            shoulder_pos[1] = self.shoulder_width
            # print("✅Left adapt_width_gamma",adapt_width_gamma)

            # -adapt_width_gamma
        # human_hand_pos = hand_pos[:].copy()
        # human_elbow_pos = elbow_pos[:].copy()
        # human_hand_pos = hand_pos[:].copy()
        # Scale arm positions with rotation-based inward adjustment
        # Pass adapt_width_gamma which already considers y_distance and overchest_b
        
        elbow_pos, hand_pos = self.scale_arm_positions(shoulder_pos, elbow_pos, hand_pos, human_shoulder_pos, chest_pose, side, adapt_width_gamma)
        

        if self.vis_pub and publish_debug:
            marker = self.construct_point_marker(hand_pos, 0.08, 0.9, color=[1, 0, 0])
            elbow_marker = self.construct_point_marker(elbow_pos, 0.1, color=[0, 1, 0])
            shoulder_marker = self.construct_point_marker(shoulder_pos, 0.1, color=[0, 0, 1])
            # 可视化上臂约束空间
            # self.visualize_upper_arm_constraint(shoulder_pos, side)
            # human_shoulder_marker = self.construct_point_marker(human_shoulder_pos, 0.02, 0.8, color=[1, 1, 1])
            # human_elbow_marker = self.construct_point_marker(human_elbow_pos, 0.02, 0.8, color=[1, 1, 1])
            # human_hand_marker = self.construct_point_marker(human_hand_pos, 0.02, 0.8, color=[1, 1, 1])
            # huaman_marker = self.construct_point_marker(human_hand_pos, 0.02, 0.8, color=[1, 1, 1])
            if side == "Left":
                self.marker_pub.publish(marker)
                self.marker_pub_elbow.publish(elbow_marker)
                self.marker_pub_shoulder.publish(shoulder_marker)
                # # 只保留一次性发布human三个部位的MarkerArray
                # human_array = MarkerArray()
                # human_shoulder_marker.id = 0
                # human_elbow_marker.id = 1
                # human_hand_marker.id = 2
                # human_array.markers.append(human_shoulder_marker)
                # human_array.markers.append(human_elbow_marker)
                # human_array.markers.append(human_hand_marker)
                # self.marker_pub_human_array_left.publish(human_array)
                # self.marker_pub_human.publish(huaman_marker)
            else:
                self.marker_pub_right.publish(marker)
                self.marker_pub_elbow_right.publish(elbow_marker)
                self.marker_pub_shoulder_right.publish(shoulder_marker)
                # # 只保留一次性发布human三个部位的MarkerArray
                # human_array = MarkerArray()
                # human_shoulder_marker.id = 0
                # human_elbow_marker.id = 1
                # human_hand_marker.id = 2
                # human_array.markers.append(human_shoulder_marker)
                # human_array.markers.append(human_elbow_marker)
                # human_array.markers.append(human_hand_marker)
                # self.marker_pub_human_array_right.publish(human_array)
        if side == "Left":
            self.left_hand_pose = (hand_pos, hand_quat)
            self.left_elbow_pos = elbow_pos
        else:
            self.right_hand_pose = (hand_pos, hand_quat)
            self.right_elbow_pos = elbow_pos

    def check_if_vr_error(self):
        """
        Check if the VR system is error.
        """
        if self.left_hand_pose is None or self.right_hand_pose is None:
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

    @staticmethod
    def pose_info2_transform(pose_info):
        """
        Convert a PoseInfo message to a 4x4 transformation matrix.
        """
        pos = [pose_info.position.x, pose_info.position.y, pose_info.position.z]
        quat = [pose_info.orientation.x, pose_info.orientation.y, pose_info.orientation.z, pose_info.orientation.w]
        rpy = quaternion_to_RPY(quat)
        T = pos_rpy_to_transform(pos, rpy)
        return T
    
    def construct_marker(self, arm_pose_p, arm_pose_q, rgba, side, marker_id):
        if len(arm_pose_q) != 4 or len(arm_pose_p)!= 3:
            print("Invalid arm pose, cannot construct marker")
            return None
        marker = Marker()
        marker.id = marker_id
        marker.header.frame_id = (
            "base_link"  # set frame_id according to the actual situation
        )
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        if side == "Left":
            marker.mesh_resource = (
                "file://"
                + self.model_path +
                "/meshes/" + self.eef_visual_stl_files[0]
            )
        elif side == "Right":
            marker.mesh_resource = (
                "file://"
                + self.model_path
                + "/meshes/" + self.eef_visual_stl_files[1]
            )
        elif side == "Torso":
            marker.mesh_resource = (
                "file://"
                + self.model_path
                + "/meshes/base_link.STL"
            )
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = rgba[3]
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = arm_pose_p[0]
        marker.pose.position.y = arm_pose_p[1]
        marker.pose.position.z = arm_pose_p[2]
        marker.pose.orientation = Quaternion(*arm_pose_q)
        return marker

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

    def pub_whole_body_joint_state_msg(self, arm_joint_angles, 
                                       left_finger_joints=None, right_finger_joints=None,
                                       torso_ypr=None, torso_height=None):
        """
        Build a JointState message for the whole body joints.
        """
        if len(arm_joint_angles) != 14:
            print("Invalid arm joint angles, cannot construct joint state message")
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [
            # "leg_l1_joint",
            # "leg_l2_joint",
            # "leg_l3_joint",
            # "leg_l4_joint",
            # "leg_l5_joint",
            # "leg_l6_joint",
            # "leg_r1_joint",
            # "leg_r2_joint",
            # "leg_r3_joint",
            # "leg_r4_joint",
            # "leg_r5_joint",
            # "leg_r6_joint",
            "vj_z",
            "vj_yaw",
            "vj_pitch",
            "vj_roll",
            "zarm_l1_joint",
            "zarm_l2_joint",
            "zarm_l3_joint",
            "zarm_l4_joint",
            "zarm_l5_joint",
            "zarm_l6_joint",
            "zarm_l7_joint",
            "zarm_r1_joint",
            "zarm_r2_joint",
            "zarm_r3_joint",
            "zarm_r4_joint",
            "zarm_r5_joint",
            "zarm_r6_joint",
            "zarm_r7_joint",
            "l_thumb_proximal_yaw_joint",
            "l_thumb_distal_pitch_joint",
            "l_index_proximal_finger_joint",
            "l_index_distal_finger_joint",
            "l_middle_proximal_finger_joint",
            "l_middle_distal_finger_joint",
            "l_ring_proximal_finger_joint",
            "l_ring_distal_finger_joint",
            "l_pinky_proximal_finger_joint",
            "l_pinky_distal_finger_joint",
            "r_thumb_proximal_yaw_joint",
            "r_thumb_distal_pitch_joint",
            "r_index_proximal_finger_joint",
            "r_index_distal_finger_joint",
            "r_middle_proximal_finger_joint",
            "r_middle_distal_finger_joint",
            "r_ring_proximal_finger_joint",
            "r_ring_distal_finger_joint",
            "r_pinky_proximal_finger_joint",
            "r_pinky_distal_finger_joint"
            ]
        # print(f"Dim: {len(msg.name)}")
        for i in range(len(msg.name)):
            msg.position.append(0.0)
        msg.position[0] = torso_height
        if torso_ypr is not None:
            for i in range(3):
                msg.position[1+i] = torso_ypr[i]
        vj_n = 4
        for i in range(14):  # arm
            msg.position[vj_n + i] = arm_joint_angles[i]
        map = [0, 1, 2, 4, 6, 8]  # 6
        if left_finger_joints is not None and right_finger_joints is not None:
            for i in range(6):  # left hand finger
                msg.position[vj_n + 14 + map[i]] = left_finger_joints[i]
            for i in range(6):  # right hand finger
                msg.position[vj_n + 24 + map[i]] = right_finger_joints[i]
        # deal right hand thumb
        msg.position[vj_n + 24 + 1] *= -1
        self.joint_state_puber.publish(msg)

    def pub_head_body_pose_msg(self, head_body_pose: HeadBodyPose):
        msg = headBodyPose()
        msg.head_pitch = head_body_pose.head_pitch
        msg.head_yaw = head_body_pose.head_yaw
        msg.body_yaw = head_body_pose.body_yaw
        msg.body_roll = head_body_pose.body_roll
        pitch_ratio = 0.8
        msg.body_pitch = max(3*np.pi/180.0, min(pitch_ratio*head_body_pose.body_pitch, 40*np.pi/180.0))

        msg.body_x = head_body_pose.body_x
        msg.body_y = head_body_pose.body_y
        # msg.body_height = max(-0.4, min(head_body_pose.body_height + 0.3, 0.2))
        msg.body_height = head_body_pose.body_height
        # self.head_body_pose_puber.publish(msg)
        # 打印发送body pose的信息
        # print(f"✅Publish head and body pose: {msg.body_yaw}, {msg.body_pitch}, {msg.body_height}")  
        if self.control_torso:
            self.head_body_pose_puber.publish(msg)
