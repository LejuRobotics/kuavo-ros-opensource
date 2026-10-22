import math
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from lightweight_transforms import *
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList, JoySticks
from kuavo_msgs.msg import ControllerSwitchEvent, headBodyPose
import rospy
import time
from tools import runtime_timing_probe as runtime_probe
from tools.noncontrol_publish_process import publisher as diagnostic_publisher
from tools.arm_marker_process import MarkerProcess, TOPICS as ARM_MARKER_TOPICS
import tf
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from geometry_msgs.msg import TransformStamped,Twist
from tf2_msgs.msg import TFMessage
from constanst import gesture_corresponding_hand_position
from pose_velocity_kalman import SkeletonCV, mapped_rates
# to dev
hand_thumb_pub = rospy.Publisher('/hand_thumb', Float32MultiArray, queue_size=10)

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
    def __init__(self, model_path, vis_pub=True, predict_gesture=False, eef_visual_stl_files=None, hand_reference_mode="thumb_index"):
        self.predict_gesture = predict_gesture
        self.model_path = model_path
        self.vis_pub = vis_pub
        self.hand_reference_mode = hand_reference_mode  # "fingertips", "middle_finger", "thumb_index"
        self.left_finger_joints = None # down dom to 6 dof
        self.right_finger_joints = None
        self.left_hand_pose = None # pos + quaternion
        self.right_hand_pose = None
        self.left_elbow_pos = None
        self.right_elbow_pos = None
        # Task07 unified TCP target.  This is intentionally separate from
        # left/right_hand_pose: the legacy pose stores l6 position together
        # with l7 attitude and must remain unchanged until takeover.
        self.left_tcp_target = None
        self.right_tcp_target = None
        self.left_tcp_elbow_hint = None
        self.right_tcp_elbow_hint = None
        self.tcp_mapping_diagnostics = {"Left": None, "Right": None}
        self.pose_info_list = None # PoseInfoList message
        self.timestamp_ms = 0.0
        self.skeleton_filter_enabled = bool(rospy.get_param("~vr_kalman_enabled", True)) and bool(rospy.get_param("~enable_velocity_ik_takeover", False))
        self.skeleton_filter = SkeletonCV(
            rospy.get_param("~vr_kalman_position_sigma", .001),
            rospy.get_param("~vr_kalman_position_q", .2),
            rospy.get_param("~vr_kalman_rotation_sigma", .003),
            rospy.get_param("~vr_kalman_rotation_q", 1.8),
            hand_q_r=rospy.get_param("~vr_kalman_hand_rotation_q", .45))
        self.skeleton_contract = None
        self.estimated_bones = {}
        self.tcp_estimated_rates = {}
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
        # Visual ROS connections live outside the input callback's GIL domain.
        self._marker_async_enabled = bool(self.vis_pub and rospy.get_param("~arm_marker_async_enabled", True))
        self._marker_process = None
        if self._marker_async_enabled:
            try:
                self._marker_process = MarkerProcess(
                    {name: rospy.resolve_name(topic) for name, topic in ARM_MARKER_TOPICS.items()},
                    float(rospy.get_param("~arm_marker_publish_rate", 30.0)))
                rospy.on_shutdown(self._marker_process.close)
            except Exception as exc:
                # Fail visual-only: never fall back to slow synchronous ROS work.
                rospy.logwarn("Arm marker process unavailable; control unchanged: %s", exc)
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10) if not self._marker_async_enabled else None
        # self.marker_pub_human = rospy.Publisher("visualization_marker/result", Marker, queue_size=10)
        self.marker_pub_right = rospy.Publisher("visualization_marker_right", Marker, queue_size=10) if not self._marker_async_enabled else None
        # self.marker_pub_human_right = rospy.Publisher("visualization_marker_human_right", Marker, queue_size=10)
        self.marker_pub_elbow = rospy.Publisher("visualization_marker/elbow", Marker, queue_size=10) if not self._marker_async_enabled else None
        self.marker_pub_elbow_right = rospy.Publisher("visualization_marker_right/elbow", Marker, queue_size=10) if not self._marker_async_enabled else None
        self.marker_pub_shoulder = rospy.Publisher("visualization_marker/shoulder", Marker, queue_size=10) if not self._marker_async_enabled else None

        # 新增：一次性发布human三个部位的MarkerArray
        self.marker_pub_human_array_left = diagnostic_publisher("visualization_marker/human_array_left", MarkerArray, queue_size=10)
        self.marker_pub_human_array_right = diagnostic_publisher("visualization_marker/human_array_right", MarkerArray, queue_size=10)
        # self.marker_pub_shoulder_human = rospy.Publisher("visualization_marker/shoulder_human", Marker, queue_size=10)
        self.marker_pub_shoulder_right = rospy.Publisher("visualization_marker_right/shoulder", Marker, queue_size=10) if not self._marker_async_enabled else None
        # self.marker_pub_shoulder_human_right = rospy.Publisher("visualization_marker_right/shoulder_human", Marker, queue_size=10)
        self.marker_pub_shoulder_quest3 = diagnostic_publisher("visualization_marker/shoulder_quest3", Marker, queue_size=10)
        self.marker_pub_shoulder_quest3_right = diagnostic_publisher("visualization_marker_right/shoulder_quest3", Marker, queue_size=10)
        self.marker_pub_chest = rospy.Publisher("visualization_marker_chest", Marker, queue_size=10) if not self._marker_async_enabled else None
        self.marker_pub_upper_arm_constraint_left = diagnostic_publisher("visualization_marker/upper_arm_constraint_left", Marker, queue_size=10)
        self.marker_pub_upper_arm_constraint_right = diagnostic_publisher("visualization_marker/upper_arm_constraint_right", Marker, queue_size=10)
        self.joint_state_puber = diagnostic_publisher('/joint_states', JointState, queue_size=10)
        self.shoulder_angle_puber = diagnostic_publisher('/quest3_debug/shoulder_angle', Float32MultiArray, queue_size=10)
        self.chest_axis_puber = diagnostic_publisher('/quest3_debug/chest_axis', Float32MultiArray, queue_size=10)
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
        # Controller retargeting reads PoseInfoList directly. TF is only used
        # by the optional finger-gesture predictor; avoid its background load.
        self.listener = tf.TransformListener() if self.predict_gesture else None
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
        # This is a retargeting length, not an invariant l4->TCP transform.
        # Velocity takeover uses an explicit geometry configuration; it is
        # NOT calibrated by a q0 l4->EEF distance. Keep its provenance separate
        # from lower_arm_length in the legacy position-IK path.
        self.tcp_forearm_retarget_length = float(rospy.get_param(
            "/quest3/teleop_tcp/forearm_retarget_length", self.lower_arm_length
        ))
        self.tcp_controller_rpy = {
            "Left": np.asarray(rospy.get_param(
                "/quest3/teleop_tcp/left_controller_rpy", [-15.0 * np.pi / 180.0, 0.0, 0.0]
            ), dtype=float),
            "Right": np.asarray(rospy.get_param(
                "/quest3/teleop_tcp/right_controller_rpy", [15.0 * np.pi / 180.0, 0.0, 0.0]
            ), dtype=float),
        }
        self.tcp_hand_tracking_rpy = {
            "Left": np.asarray(rospy.get_param(
                "/quest3/teleop_tcp/left_hand_tracking_rpy", [0.0, 0.0, 0.0]
            ), dtype=float),
            "Right": np.asarray(rospy.get_param(
                "/quest3/teleop_tcp/right_hand_tracking_rpy", [0.0, 0.0, 0.0]
            ), dtype=float),
        }
        # Backward-compatible shared offset.  The previous implementation
        # expressed it in the final TCP axes (R_tcp * offset).  It is converted
        # below to a translation in each source frame so the new full SE(3)
        # adapter reproduces the old output exactly when no new parameter is set.
        self.tcp_palm_offset = {
            "Left": np.asarray(rospy.get_param(
                "/quest3/teleop_tcp/left_palm_offset", [0.0, 0.0, 0.0]
            ), dtype=float),
            "Right": np.asarray(rospy.get_param(
                "/quest3/teleop_tcp/right_palm_offset", [0.0, 0.0, 0.0]
            ), dtype=float),
        }
        self.tcp_source_to_tcp = {"controller": {}, "hand_tracking": {}}
        for side in ("Left", "Right"):
            if self.tcp_controller_rpy[side].shape != (3,):
                raise ValueError(f"/quest3/teleop_tcp/{side.lower()}_controller_rpy must have 3 values")
            if self.tcp_hand_tracking_rpy[side].shape != (3,):
                raise ValueError(f"/quest3/teleop_tcp/{side.lower()}_hand_tracking_rpy must have 3 values")
            if self.tcp_palm_offset[side].shape != (3,):
                raise ValueError(f"/quest3/teleop_tcp/{side.lower()}_palm_offset must have 3 values")
            for source_key, source_param, calibration_rpy in (
                    ("controller", "controller", self.tcp_controller_rpy[side]),
                    ("hand_tracking", "hand_tracking", self.tcp_hand_tracking_rpy[side])):
                R_source_to_tcp = rpy_to_matrix(calibration_rpy)
                legacy_equivalent_translation = R_source_to_tcp @ self.tcp_palm_offset[side]
                translation = np.asarray(rospy.get_param(
                    f"/quest3/teleop_tcp/{side.lower()}_{source_param}_translation",
                    legacy_equivalent_translation.tolist(),
                ), dtype=float)
                if translation.shape != (3,) or not np.isfinite(translation).all():
                    raise ValueError(
                        f"/quest3/teleop_tcp/{side.lower()}_{source_param}_translation "
                        "must have 3 finite values"
                    )
                if not np.isfinite(R_source_to_tcp).all():
                    raise ValueError(
                        f"/quest3/teleop_tcp/{side.lower()}_{source_param}_rpy "
                        "must have 3 finite values"
                    )
                T_source_to_tcp = np.eye(4)
                T_source_to_tcp[:3, :3] = R_source_to_tcp
                T_source_to_tcp[:3, 3] = translation
                self.tcp_source_to_tcp[source_key][side] = T_source_to_tcp
                rospy.loginfo(
                    "[Quest3 TCP] %s %s->teleop_tcp: xyz=[%.5f %.5f %.5f]m, "
                    "rpy=[%.5f %.5f %.5f]rad",
                    side,
                    source_key,
                    translation[0], translation[1], translation[2],
                    calibration_rpy[0], calibration_rpy[1], calibration_rpy[2],
                )
        from target_geometry_contract import LEGACY, WRIST, validate_wrist_contract
        self.tcp_geometry_binding = rospy.get_param("/quest3/teleop_tcp/geometry_binding", {})
        self.tcp_geometry_mode = self.tcp_geometry_binding.get("operation_mode", LEGACY)
        self.tcp_wrist_contract = None
        if self.tcp_geometry_mode == WRIST:
            if rospy.get_param("/real", True):
                raise ValueError("WRIST_CONSISTENT_6D is isolated-simulation only")
            self.tcp_wrist_contract = validate_wrist_contract(
                rospy.get_param("~velocity_wrist_contract", {}),
                self.tcp_geometry_binding["ik_model"]["model_sha256"],
                real_source=not rospy.get_param("~velocity_geometry_source_is_synthetic", False))
            for sides in self.tcp_wrist_contract["sources"].values():
                for side, contract in sides.items():
                    actual = self.tcp_geometry_binding["ik_model"]["arms"][side]["joint7_to_tcp"]
                    if not np.allclose(contract["joint7_to_tcp"], actual, atol=1e-10, rtol=0):
                        raise ValueError("Wrist contract uses a different robot tool transform")
        elif self.tcp_geometry_mode != LEGACY:
            raise ValueError("Unsupported target geometry mode")
        inventory = dict(self.tcp_geometry_binding)
        inventory["source_to_tcp"] = {source: {side: T.tolist() for side, T in sides.items()}
                                       for source, sides in self.tcp_source_to_tcp.items()}
        inventory["source_to_tcp_applied"] = self.tcp_geometry_mode == LEGACY
        inventory["wrist_contract"] = self.tcp_wrist_contract or {}
        rospy.set_param("/quest3/teleop_tcp/resolved_contract", inventory)
        rospy.loginfo(
            "[Quest3 TCP] full chest-relative retarget enabled in parallel: "
            "upper=%.4fm, forearm-map=%.4fm",
            self.upper_arm_length,
            self.tcp_forearm_retarget_length,
        )
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
        self.left_joystick = [msg.left_trigger, msg.left_grip]
        self.right_joystick = [msg.right_trigger, msg.right_grip]

    def read_msg(self, msg, defer_visuals=False):
        """
        Read the PoseInfoList message and extract the left and right finger joint angles.
        """
        self._defer_arm_markers = bool(defer_visuals)
        self._pending_arm_markers = []
        self._marker_wall_ns = 0
        self._marker_cpu_ns = 0
        # A rejected frame must never make the parallel TCP contract look
        # current.  Raw metadata remains available to the callback for debug.
        self.left_tcp_target = None
        self.right_tcp_target = None
        self.left_tcp_elbow_hint = None
        self.right_tcp_elbow_hint = None
        self.tcp_mapping_diagnostics = {"Left": None, "Right": None}
        self.estimated_bones = {}
        self.tcp_estimated_rates = {}
        if self.left_joystick is None:
            print("No joystick message received yet")
            return False
        # poses 为空或不足时跳过该帧，避免带载等空 poses 命令包导致按固定骨骼索引越界 (IndexError)
        # 注：len < len(bone_names) 已涵盖 empty（0 < 24），等价 C++ :313 empty() + :318 size()<24
        if len(msg.poses) < len(bone_names):
            return False
        # 完整相对位姿依赖 Chest、肩、肘、手；拒绝非有限值和无效四元数。
        for _idx in (bone_name_to_index["LeftHandPalm"], bone_name_to_index["LeftArmLower"],
                     bone_name_to_index["RightHandPalm"], bone_name_to_index["RightArmLower"],
                     bone_name_to_index["Chest"], bone_name_to_index["LeftArmUpper"],
                     bone_name_to_index["RightArmUpper"]):
            _p = msg.poses[_idx]
            if not (math.isfinite(_p.position.x) and math.isfinite(_p.position.y) and math.isfinite(_p.position.z)
                    and math.isfinite(_p.orientation.x) and math.isfinite(_p.orientation.y)
                    and math.isfinite(_p.orientation.z) and math.isfinite(_p.orientation.w)):
                return False
            if sum(v * v for v in (_p.orientation.x, _p.orientation.y,
                                    _p.orientation.z, _p.orientation.w)) < 1.0e-12:
                return False
        self.pose_info_list = msg.poses
        self.timestamp_ms = msg.timestamp_ms
        self.is_high_confidence = msg.is_high_confidence
        self.is_hand_tracking = msg.is_hand_tracking
        # Confidence is diagnostic only; valid poses keep one continuous filter history.
        runtime_probe.point(100)
        if self.skeleton_filter_enabled:
            try:
                transforms = {i: self.pose_info2_transform(msg.poses[i])
                              for i in SkeletonCV.indices if i != 22}
                # Root is optional diagnostic data, never a new dependency
                # for the chest-relative arm/body control contract.
                root = msg.poses[22]
                root_values = [root.position.x, root.position.y, root.position.z,
                               root.orientation.x, root.orientation.y,
                               root.orientation.z, root.orientation.w]
                transforms[22] = (self.pose_info2_transform(root)
                                  if np.isfinite(root_values).all() and
                                  np.linalg.norm(root_values[3:]) > .5 else np.eye(4))
                runtime_probe.point(101)
                self.estimated_bones = self.skeleton_filter.update(
                    transforms, float(msg.timestamp_ms)*.001,
                    (self.skeleton_contract, bool(msg.is_hand_tracking)))
            except (ValueError, np.linalg.LinAlgError):
                self.skeleton_filter.reset()
                return False
        runtime_probe.point(102)
        # print("is_high_confidence: {}, is_hand_tracking: {}".format(self.is_high_confidence, self.is_hand_tracking))
        if self.is_hand_tracking:
            self.compute_finger_joints("Left")
            self.compute_finger_joints("Right")
        else:
            self.compute_finger_joints_joy("Left")
            self.compute_finger_joints_joy("Right")
        runtime_probe.point(104)
        self._chest_axis_published = False
        self._chest_marker_published = False
        self.compute_hand_pose("Left")
        runtime_probe.point(105)
        self.compute_hand_pose("Right")
        runtime_probe.point(106)
        # gesture counts
        max_counts = 50
        if(self.is_runing_gesture()):
            self.ok_gesture_counts += 1
            self.shot_gesture_counts = 0
            if(self.ok_gesture_counts >= max_counts):
                self.is_runing = True
        elif(self.is_stop_gesture()):
            self.shot_gesture_counts += 1
            self.ok_gesture_counts = 0
            if(self.shot_gesture_counts >= max_counts):
                self.is_runing = False
        runtime_probe.point(107)
        if self.is_runing:
            self.pub_head_body_pose_msg(self.head_body_pose)
        runtime_probe.point(109)
        return True


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

    def get_hand_transform(self, side):
        """
        根据配置的hand_reference_mode获取手部变换矩阵
        """
        if self.pose_info_list is None:
            return None

        if self.hand_reference_mode == "palm":
            return self.pose_info2_transform(self.pose_info_list[bone_name_to_index[side + "HandPalm"]])
        elif self.hand_reference_mode == "fingertips":
            T_hand = self.compute_hand_center_from_fingertips(side)
            if T_hand is None:
                T_hand = self.pose_info2_transform(self.pose_info_list[bone_name_to_index[side + "HandPalm"]])
            return T_hand
        elif self.hand_reference_mode == "middle_finger":
            T_hand = self.compute_hand_center_from_middle_finger(side)
            if T_hand is None:
                T_hand = self.pose_info2_transform(self.pose_info_list[bone_name_to_index[side + "HandPalm"]])
            return T_hand
        elif self.hand_reference_mode == "thumb_index":
            T_hand = self.compute_hand_center_from_thumb_index(side)
            if T_hand is None:
                T_hand = self.pose_info2_transform(self.pose_info_list[bone_name_to_index[side + "HandPalm"]])
            return T_hand
        else:
            # 默认使用手掌
            return self.pose_info2_transform(self.pose_info_list[bone_name_to_index[side + "HandPalm"]])

    def compute_hand_center_from_middle_finger(self, side):
        """
        使用中指尖作为手的参考点
        """
        if self.pose_info_list is None:
            return None

        try:
            # 获取中指尖的位置
            middle_finger_pose = self.pose_info_list[bone_name_to_index[side + "HandMiddleTip"]]
            middle_finger_pos = [middle_finger_pose.position.x, middle_finger_pose.position.y, middle_finger_pose.position.z]

            # 使用手掌的方向
            hand_palm_pose = self.pose_info_list[bone_name_to_index[side + "HandPalm"]]
            hand_orientation = hand_palm_pose.orientation

            # 构建变换矩阵
            T_hand_middle = np.eye(4)
            T_hand_middle[:3, 3] = middle_finger_pos
            quat = [hand_orientation.x, hand_orientation.y, hand_orientation.z, hand_orientation.w]
            T_hand_middle[:3, :3] = quaternion_to_matrix(quat)

            return T_hand_middle

        except (KeyError, IndexError):
            return None

    def compute_hand_center_from_thumb_index(self, side):
        """
        使用拇指和食指中点作为手的参考点
        """
        if self.pose_info_list is None:
            return None

        try:
            # 获取拇指尖和食指尖的位置
            thumb_pose = self.pose_info_list[bone_name_to_index[side + "HandThumbTip"]]
            index_pose = self.pose_info_list[bone_name_to_index[side + "HandIndexTip"]]

            thumb_pos = np.array([thumb_pose.position.x, thumb_pose.position.y, thumb_pose.position.z])
            index_pos = np.array([index_pose.position.x, index_pose.position.y, index_pose.position.z])

            # 计算中点
            center_pos = (thumb_pos + index_pos) / 2.0

            # 使用手掌的方向
            hand_palm_pose = self.pose_info_list[bone_name_to_index[side + "HandPalm"]]
            hand_orientation = hand_palm_pose.orientation

            # 构建变换矩阵
            T_hand_center = np.eye(4)
            T_hand_center[:3, 3] = center_pos
            quat = [hand_orientation.x, hand_orientation.y, hand_orientation.z, hand_orientation.w]
            T_hand_center[:3, :3] = quaternion_to_matrix(quat)

            return T_hand_center

        except (KeyError, IndexError):
            return None

    def compute_hand_center_from_fingertips(self, side):
        """
        计算手指尖的中心位置作为手的参考点
        """
        if self.pose_info_list is None:
            return None

        # 获取所有手指尖的位置
        finger_tips = ["HandThumbTip", "HandIndexTip", "HandMiddleTip", "HandRingTip", "HandLittleTip"]
        finger_positions = []

        for finger_tip in finger_tips:
            try:
                pose_info = self.pose_info_list[bone_name_to_index[side + finger_tip]]
                finger_positions.append([pose_info.position.x, pose_info.position.y, pose_info.position.z])
            except (KeyError, IndexError):
                continue

        if len(finger_positions) < 3:  # 至少需要3个手指尖
            return None

        # 计算手指尖的中心位置
        center_pos = np.mean(finger_positions, axis=0)

        # 使用手掌的方向
        hand_palm_pose = self.pose_info_list[bone_name_to_index[side + "HandPalm"]]
        hand_orientation = hand_palm_pose.orientation

        # 构建变换矩阵
        T_hand_center = np.eye(4)
        T_hand_center[:3, 3] = center_pos
        quat = [hand_orientation.x, hand_orientation.y, hand_orientation.z, hand_orientation.w]
        T_hand_center[:3, :3] = quaternion_to_matrix(quat)

        return T_hand_center

    def compute_finger_joints(self, side):
        if self.predict_gesture:
            gesture, confidence, inference_time = self.hand_gesture_predictor.predict(self.get_relative_finger_poses(side))
            if gesture in gesture_corresponding_hand_position:
                hand_position = gesture_corresponding_hand_position[gesture]
                finger_joints = [1.7*hand_position[i]/100.0 for i in range(6)]
                if side == "Left":
                    self.left_finger_joints = finger_joints
                elif side == "Right":
                    self.right_finger_joints = finger_joints
                return finger_joints
        """
        y轴从手腕指向中指指尖, x轴从拇指位置指向无名指方向
        """
        if self.pose_info_list is None:
            return None

        # 根据配置的hand_reference_mode获取手部变换矩阵
        T_hand = self.get_hand_transform(side)

        T_finger_thumb_tip = self.pose_info2_transform(self.pose_info_list[bone_name_to_index[side + "HandThumbTip"]])
        finger_index_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandIndexTip"]].orientation
        finger_middle_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandMiddleTip"]].orientation
        finger_ring_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandRingTip"]].orientation
        finger_little_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandLittleTip"]].orientation
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
        if side == "Left":
            self.left_finger_joints = finger_joints
            thumb_msg = Float32MultiArray()
            thumb_msg.data = p_hf_h
            hand_thumb_pub.publish(thumb_msg)
        elif side == "Right":
            self.right_finger_joints = finger_joints
        return finger_joints

    def compute_finger_joints_joy(self, side):
        """
        手柄版本的计算手指关节角度
        """
        if self.pose_info_list is None:
            return None
        hand_ori = self.pose_info_list[bone_name_to_index[side + "HandPalm"]].orientation
        finger_thumb_metacarpal_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbMetacarpal"]].orientation
        finger_thumb_proximal_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbProximal"]].orientation
        finger_thumb_distal_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbDistal"]].orientation
        finger_thumb_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbTip"]].orientation
        finger_index_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandIndexTip"]].orientation
        finger_middle_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandMiddleTip"]].orientation
        finger_ring_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandRingTip"]].orientation
        finger_little_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandLittleTip"]].orientation
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

        if side == "Left":
            self.left_finger_joints = finger_joints
        elif side == "Right":
            self.right_finger_joints = finger_joints
        return finger_joints

    def get_finger_joints(self, side):
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
        # Controller virtual finger poses are not optical gesture authority.
        # Their tiny relative roll may cross the display encoding sign bit.
        is_runing = bool(self.is_hand_tracking)
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
        is_stop = bool(self.is_hand_tracking)
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

    def get_tcp_target(self, side):
        """Return the parallel, single-frame TCP pose and optional elbow hint."""
        if side == "Left":
            return self.left_tcp_target, self.left_tcp_elbow_hint
        if side == "Right":
            return self.right_tcp_target, self.right_tcp_elbow_hint
        print("Invalid side: {}".format(side))
        return None, None

    def get_tcp_mapping_diagnostics(self, side):
        """Return source-frame and source->TCP details for the current frame."""
        return self.tcp_mapping_diagnostics.get(side)

    def _active_source_to_tcp(self, side):
        source_key = "hand_tracking" if self.is_hand_tracking else "controller"
        return source_key, self.tcp_source_to_tcp[source_key][side]

    @staticmethod
    def _unit_vector(vector):
        vector = np.asarray(vector, dtype=float)
        norm = np.linalg.norm(vector)
        if not np.isfinite(norm) or norm < 1.0e-6:
            return None
        return vector / norm

    def _compute_unified_tcp_target(self, side, T_wChest, T_wShoulder,
                                    T_wElbow, T_wPalm, vr_quat, palm_robot_rotation=None):
        """Build one rigid TCP target without reusing the legacy l6 target.

        Human bone directions are first expressed in a complete chest-relative
        frame.  Only then are they anchored at the robot shoulder.  The elbow
        point is a redundancy hint; TCP position and attitude form the primary
        6-D task and therefore share exactly the same base_link semantics.
        """
        transforms = (T_wChest, T_wShoulder, T_wElbow, T_wPalm)
        if not all(np.asarray(T).shape == (4, 4) and np.isfinite(T).all() for T in transforms):
            return None, None

        # At the initial/reference chest attitude this rotation is identity.
        # Subsequent chest roll, pitch and yaw are all removed, unlike the
        # legacy yaw-only compensation.
        R_reference_world_from_chest = self.init_R_wC
        R_world_from_chest = T_wChest[:3, :3]
        R_remove_chest_motion = R_reference_world_from_chest @ R_world_from_chest.T
        chest_position = T_wChest[:3, 3]

        shoulder_relative = R_remove_chest_motion @ (T_wShoulder[:3, 3] - chest_position)
        elbow_relative = R_remove_chest_motion @ (T_wElbow[:3, 3] - chest_position)
        palm_relative = R_remove_chest_motion @ (T_wPalm[:3, 3] - chest_position)
        upper_direction = self._unit_vector(elbow_relative - shoulder_relative)
        forearm_direction = self._unit_vector(palm_relative - elbow_relative)
        if upper_direction is None or forearm_direction is None:
            return None, None

        shoulder_anchor = np.array([
            bias_chest_to_base_link[0],
            self.shoulder_width if side == "Left" else -self.shoulder_width,
            bias_chest_to_base_link[2],
        ], dtype=float)
        elbow_hint = shoulder_anchor + self.upper_arm_length * upper_direction
        mapped_source_position = (
            elbow_hint + self.tcp_forearm_retarget_length * forearm_direction
        )

        # Keep the proven Quest->robot axis mapping, but express the former
        # hard-coded +/-15 degree wrist bias as an explicit Palm->TCP
        # calibration.  Controller and optical hand tracking have independent
        # calibrations because their Palm semantics differ.
        if palm_robot_rotation is None:
            palm_robot_rotation = quaternion_to_matrix(vr_quat2robot_quat(vr_quat, side, 0.0))
        R_mapped_source = R_remove_chest_motion @ palm_robot_rotation
        source_key, T_source_to_tcp = self._active_source_to_tcp(side)
        R_source_to_tcp = T_source_to_tcp[:3, :3]
        source_to_tcp_translation = T_source_to_tcp[:3, 3]
        R_tcp = R_mapped_source @ R_source_to_tcp
        tcp_position = (
            mapped_source_position
            + R_mapped_source @ source_to_tcp_translation
        )
        # Opt-in, fail-closed wrist mapping. Legacy output is bit-for-bit
        # unchanged unless a complete source/model contract was admitted.
        mapped_joint7 = None
        if getattr(self, "tcp_geometry_mode", "LEGACY_PALM_6D") == "WRIST_CONSISTENT_6D":
            from target_geometry_contract import map_wrist_pose
            contract = self.tcp_wrist_contract["sources"].get(source_key, {}).get(side)
            if contract is None:
                return None, None
            X_source = np.eye(4)
            X_source[:3, :3] = palm_robot_rotation
            X_source[:3, 3] = T_wPalm[:3, 3]
            try:
                X_tcp, elbow_hint, mapped_joint7 = map_wrist_pose(
                    T_wChest, T_wShoulder[:3, 3], T_wElbow[:3, 3],
                    X_source, self.tcp_wrist_contract["robot_task_from_chest_axes"],
                    shoulder_anchor, self.upper_arm_length, contract)
            except ValueError:
                return None, None
            tcp_position, R_tcp = X_tcp[:3, 3], X_tcp[:3, :3]
        mapped_source_quaternion = np.asarray(
            matrix_to_quaternion(R_mapped_source), dtype=float
        )
        tcp_quaternion = np.asarray(matrix_to_quaternion(R_tcp), dtype=float)
        if not (np.isfinite(tcp_position).all() and np.isfinite(elbow_hint).all()
                and np.isfinite(mapped_source_position).all()
                and np.isfinite(mapped_source_quaternion).all()
                and np.isfinite(tcp_quaternion).all()):
            return None, None
        self.tcp_mapping_diagnostics[side] = {
            "source_key": source_key,
            "geometry_mode": getattr(self, "tcp_geometry_mode", "LEGACY_PALM_6D"),
            "mapped_joint7": mapped_joint7,
            "source_to_tcp_applied": mapped_joint7 is None,
            "mapped_source_position": mapped_source_position.copy(),
            "mapped_source_quaternion": mapped_source_quaternion.copy(),
            "tcp_position": tcp_position.copy(),
            "tcp_quaternion": tcp_quaternion.copy(),
            "elbow_hint": elbow_hint.copy(),
            "upper_direction": upper_direction.copy(),
            "forearm_direction": forearm_direction.copy(),
            "source_to_tcp": T_source_to_tcp.copy(),
        }
        return (tcp_position, tcp_quaternion), elbow_hint

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
        msg = Float32MultiArray()
        msg.data = np.append(self.left_shoulder_rpy_in_robot, self.right_shoulder_rpy_in_robot)
        self.shoulder_angle_puber.publish(msg)

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

    def compute_hand_pose(self, side):
        chest_pose = self.pose_info_list[bone_name_to_index["Chest"]]
        #打印chest对应的index
        # print("Chest index: ✅", bone_name_to_index["Chest"])
        elbow_pose = self.pose_info_list[bone_name_to_index[side + "ArmLower"]]
        #打印elbow对应的index
        # print(f"{side} ArmLower index: ✅", bone_name_to_index[side + "ArmLower"])
        shoulder_pose = self.pose_info_list[bone_name_to_index[side + "ArmUpper"]]
        #打印shoulder对应的index
        # print(f"{side} ArmUpper index: ✅", bone_name_to_index[side + "ArmUpper"])
        hand_pose = self.pose_info_list[bone_name_to_index[side + "HandPalm"]]
        #打印hand对应的index
        # print(f"{side} HandPalm index: ✅", bone_name_to_index[side + "HandPalm"])
        #打印hand对应的位置
        # print(f"{side} HandPalm y position: ✅", hand_pose.position.y)

        vr_quat = [hand_pose.orientation.x, hand_pose.orientation.y, hand_pose.orientation.z, hand_pose.orientation.w]
        # The legacy q-only path now consumes the same source->TCP rotational
        # adapter as the unified TCP path.  Default calibration reproduces the
        # historical controller +/-15 degree and hand-tracking 0 degree output.
        palm_robot_quat = vr_quat2robot_quat(vr_quat, side, 0.0)
        palm_robot_rotation = quaternion_to_matrix(palm_robot_quat)
        _, T_source_to_tcp = self._active_source_to_tcp(side)
        hand_quat_in_w = matrix_to_quaternion(
            palm_robot_rotation @ T_source_to_tcp[:3, :3]
        )

        T_wChest = self.pose_info2_transform(chest_pose)
        T_wElbow = self.pose_info2_transform(elbow_pose)
        T_wHand = self.pose_info2_transform(hand_pose)
        T_wS = self.pose_info2_transform(shoulder_pose)

        tcp_chest, tcp_shoulder, tcp_elbow, tcp_palm = T_wChest, T_wS, T_wElbow, T_wHand
        tcp_rotation = palm_robot_rotation
        if self.estimated_bones:
            ids = (0, 1, 4) if side == "Left" else (2, 3, 5)
            c, s, e, h = [self.estimated_bones[i] for i in (23,)+ids]
            tcp_chest, tcp_shoulder, tcp_elbow, tcp_palm = c[0], s[0], e[0], h[0]
            tcp_rotation = quaternion_to_matrix(vr_quat2robot_quat(
                matrix_to_quaternion(tcp_palm[:3, :3]), side, 0.0))
        tcp_target, tcp_elbow_hint = self._compute_unified_tcp_target(
            side, tcp_chest, tcp_shoulder, tcp_elbow, tcp_palm, vr_quat, tcp_rotation)
        if self.estimated_bones and tcp_target is not None:
            source_key, tool = self._active_source_to_tcp(side)
            axes, lower, offset = self.init_R_wC, self.tcp_forearm_retarget_length, tool[:3, 3]
            wrist_offset = wrist_rotation = None
            if getattr(self, "tcp_geometry_mode", "LEGACY_PALM_6D") == "WRIST_CONSISTENT_6D":
                contract = self.tcp_wrist_contract["sources"][source_key][side]
                X = np.asarray(contract["source_to_wrist"])
                wrist_offset = X[:3, 3]
                wrist_rotation = X[:3, :3]@np.asarray(contract["wrist_to_joint7_rotation"])
                axes = np.asarray(self.tcp_wrist_contract["robot_task_from_chest_axes"])
                lower = contract["forearm_to_joint7_m"]
                offset = np.asarray(contract["joint7_to_tcp"])[:3, 3]
            self.tcp_estimated_rates[side] = mapped_rates(
                c, s, e, h, axes, self.upper_arm_length, lower, tcp_rotation,
                offset, wrist_offset, wrist_rotation)
        if side == "Left":
            self.left_tcp_target = tcp_target
            self.left_tcp_elbow_hint = tcp_elbow_hint
        else:
            self.right_tcp_target = tcp_target
            self.right_tcp_elbow_hint = tcp_elbow_hint

        axis, angle = matrix_to_axis_angle(self.init_R_wC.T @ T_wChest[:3, :3])
        chest_rpy = matrix_to_rpy(self.init_R_wC.T @ T_wChest[:3, :3])
        self.chest_axis_agl = [0, 0, axis[1]]
        self.head_body_pose.body_yaw = axis[1]
        R_wChest_rm_yaw = axis_angle_to_matrix(self.chest_axis_agl).T @ T_wChest[:3, :3]
        self.head_body_pose.body_pitch = matrix_to_rpy(self.init_R_wC.T @ R_wChest_rm_yaw)[0]
        self.head_body_pose.body_roll = chest_rpy[2]  # Extract roll from chest RPY
        self.head_body_pose.body_x = chest_pose.position.x
        self.head_body_pose.body_y = chest_pose.position.y
        self.head_body_pose.body_height = chest_pose.position.z

        # 输出body_roll
        # print(f"⚠️body_roll: {self.head_body_pose.body_roll}")

        hand_mat_cH = axis_angle_to_matrix(self.chest_axis_agl).T @ quaternion_to_matrix(hand_quat_in_w)
        hand_quat = matrix_to_quaternion(hand_mat_cH)
        if not getattr(self, "_chest_axis_published", False):
            chest_axis_msg = Float32MultiArray()
            chest_axis_msg.data = axis
            self.chest_axis_puber.publish(chest_axis_msg)
            self._chest_axis_published = True
        if side == "Left" and self.init_R_wLS is None:
            self.init_R_wLS = T_wS[:3, :3]
        if side == "Right" and self.init_R_wRS is None:
            self.init_R_wRS = T_wS[:3, :3]
        self.compute_shoudler_pose(T_wS[:3, :3], side)

        T_ChestElbow = T_wElbow
        T_ChestHand = T_wHand

        hand_pos = T_ChestHand[:3, 3].copy()
        hand_pos[0] -= chest_pose.position.x
        hand_pos[1] -= chest_pose.position.y
        hand_pos[2] -= chest_pose.position.z
        hand_pos = axis_angle_to_matrix(self.chest_axis_agl).T @ hand_pos
        hand_pos[0] += bias_chest_to_base_link[0]
        hand_pos[1] += bias_chest_to_base_link[1]
        hand_pos[2] += bias_chest_to_base_link[2]
        hand_pos[0] = 0.1 if hand_pos[0] < 0.1 else hand_pos[0]

        elbow_pos = T_ChestElbow[:3, 3].copy()
        elbow_pos[0] -= chest_pose.position.x
        elbow_pos[1] -= chest_pose.position.y
        elbow_pos[2] -= chest_pose.position.z
        elbow_pos = axis_angle_to_matrix(self.chest_axis_agl).T @ elbow_pos
        elbow_pos[0] += bias_chest_to_base_link[0]
        elbow_pos[2] += bias_chest_to_base_link[2]

        shoulder_pos = T_wS[:3, 3].copy()
        shoulder_pos[0] -= chest_pose.position.x
        shoulder_pos[1] -= chest_pose.position.y
        shoulder_pos[2] -= chest_pose.position.z
        shoulder_pos = axis_angle_to_matrix(self.chest_axis_agl).T @ shoulder_pos
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


        if self.vis_pub:
            if self._defer_arm_markers:
                # At most two immutable jobs per raw callback. Keep every
                # marker, but publish the atomic TCP contract first.
                self._pending_arm_markers.append((
                    side, hand_pos.copy(), elbow_pos.copy(),
                    shoulder_pos.copy(), chest_pose))
            else:
                self._publish_arm_markers(side, hand_pos, elbow_pos, shoulder_pos, chest_pose)

        if side == "Left":
            self.left_hand_pose = (hand_pos, hand_quat)
            self.left_elbow_pos = elbow_pos
        else:
            self.right_hand_pose = (hand_pos, hand_quat)
            self.right_elbow_pos = elbow_pos

    def flush_pending_arm_markers(self):
        pending = getattr(self, "_pending_arm_markers", [])
        self._pending_arm_markers = []
        if self._marker_async_enabled:
            self._submit_arm_markers(pending)
        else:
            for values in pending:
                self._publish_arm_markers(*values)

    def _submit_arm_markers(self, jobs):
        if not jobs:
            return
        wall_start = time.monotonic_ns()
        cpu_start = time.thread_time_ns()
        try:
            if self._marker_process is not None:
                self._marker_process.submit(jobs)
        finally:
            # Same diagnostic fields now measure IPC handoff, not worker publish.
            self._marker_wall_ns += time.monotonic_ns() - wall_start
            self._marker_cpu_ns += time.thread_time_ns() - cpu_start

    def _publish_arm_markers(self, side, hand_pos, elbow_pos, shoulder_pos, chest_pose):
        if self._marker_async_enabled:
            return self._submit_arm_markers([(side, hand_pos, elbow_pos, shoulder_pos, chest_pose)])
        wall_start = time.monotonic_ns()
        cpu_start = time.thread_time_ns()
        try:
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
            if not getattr(self, "_chest_marker_published", False):
                chest_pos = [chest_pose.position.x, chest_pose.position.y, chest_pose.position.z]
                chest_marker = self.construct_point_marker(chest_pos, 0.1, 0.8)
                self.marker_pub_chest.publish(chest_marker)
                self._chest_marker_published = True

        finally:
            self._marker_wall_ns += time.monotonic_ns() - wall_start
            self._marker_cpu_ns += time.thread_time_ns() - cpu_start

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
        T = np.eye(4)
        T[:3, :3] = quaternion_to_matrix(quat)
        T[:3, 3] = pos
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
