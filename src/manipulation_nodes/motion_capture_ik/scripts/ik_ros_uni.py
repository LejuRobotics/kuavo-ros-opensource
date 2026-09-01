#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import signal
import rospy
import argparse
from std_msgs.msg import (
    Float32,
    Float32MultiArray,
    Float64MultiArray,
    Int32,
    Bool,
    Float64,
    UInt64,
    MultiArrayDimension,
)
from sensor_msgs.msg import JointState
from handcontrollerdemorosnode.msg import armPoseWithTimeStamp
from kuavo_msgs.msg import robotHandPosition
from kuavo_msgs.srv import controlLejuClaw, controlLejuClawRequest
from kuavo_msgs.msg import lejuClawCommand
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
import time
import math
import sys
import struct
import threading
import ctypes
from dataclasses import dataclass
from tools.drake_trans import *

from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeKuavo
from kuavo_msgs.msg import sensorsData
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolRequest, SetBoolResponse

import numpy as np
from ik.diff_ik import DiffIK, interpolate_pose
from ik.torso_ik import ArmIk

import rospy
from noitom_hi5_hand_udp_python.msg import handRotationEular
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList
from kuavo_msgs.msg import JoySticks
from tools.quest3_utils import Quest3ArmInfoTransformer
from kuavo_msgs.msg import (
    Float32MultiArrayStamped,
    ikSolveError,
    handPose,
    robotArmQVVD,
    armHandPose,
    twoArmHandPose,
    twoArmHandPoseCmd,
)
from ocs2_msgs.msg import mpc_observation

from tools.utils import get_package_path, ArmIdx, IkTypeIdx, rotation_matrix_diff_in_angle_axis, limit_value
from tools.drake_trans import rpy_to_matrix
from tools.kalman_filter import KalmanFilter3D
import os


# 定义调度策略常量
SCHED_OTHER = 0
SCHED_FIFO = 1
SCHED_RR = 2
num_arm_joints_var = 14

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

# 定义sched_param结构体
class sched_param(ctypes.Structure):
    _fields_ = [('sched_priority', ctypes.c_int)]

# 加载pthread库
pthread = ctypes.CDLL('libpthread.so.0')

# 定义pthread_setschedparam函数
pthread_setschedparam = pthread.pthread_setschedparam
pthread_setschedparam.argtypes = [ctypes.c_ulong, ctypes.c_int, ctypes.POINTER(sched_param)]
pthread_setschedparam.restype = ctypes.c_int

def set_thread_priority(thread, policy, priority):
    try:
        param = sched_param()
        param.sched_priority = priority
        thread_id = thread.ident
        ret = pthread_setschedparam(thread_id, policy, ctypes.byref(param))
        if ret != 0:
            rospy.logerr(
                "Failed to set thread priority: tid=%s policy=%s priority=%s error=%s. "
                "Try to run with permission to set real-time scheduling.",
                thread_id, policy, priority, ret,
            )
            return False
        rospy.loginfo(
            "IK thread priority configured: tid=%s policy=%s priority=%s",
            thread_id, policy, priority,
        )
        return True
    except Exception as e:
        rospy.logerr(f"Failed to set thread priority: {e}")
        return False
    
QIANGNAO = "qiangnao"
JODELL = "jodell"
LEJUCLAW = "lejuclaw"
QIANGNAO_TOUCH = "qiangnao_touch"
REVO2 = "revo2"
LINKER_HAND = "linker_hand"

control_finger_type = 0
control_torso = 0


@dataclass(frozen=True)
class BoneInputFrame:
    """Bone message and the timing metadata captured at callback entry."""

    sequence: int
    message: object
    receive_time: float
    receive_monotonic: float
    source_timestamp_ms: int


@dataclass(frozen=True)
class ProcessedBoneFrame:
    """Immutable same-frame target snapshot consumed by the IK thread."""

    sequence: int
    receive_time: float
    source_timestamp_ms: int
    transform_done_time: float
    transform_done_monotonic: float
    left_pose: object
    right_pose: object
    left_elbow_pos: object
    right_elbow_pos: object
    left_finger_joints: object
    right_finger_joints: object
    left_shoulder_rpy: object
    right_shoulder_rpy: object
    is_running: bool
    is_hand_tracking: bool
    vr_error: bool


@dataclass(frozen=True)
class IkSolutionFrame:
    """Latest valid IK result consumed by the fixed-rate publisher."""

    target_generation: int
    processed_bone: object
    desired_arm_q: object
    shoulder_velocity_limit: float
    ready_time: float
    ready_monotonic: float


# /vr_absolute/transform_stage_latency_ms 的 data 顺序。字段名写入
# Float32MultiArray.layout.dim[0].label，使 bag 本身可以解释各列含义。
TRANSFORM_STAGE_DIAGNOSTIC_FIELDS = (
    "callback_to_worker_ms",
    "worker_housekeeping_ms",
    "transformer_lock_wait_ms",
    "input_validation_ms",
    "finger_compute_ms",
    "chest_context_compute_ms",
    "left_hand_compute_ms",
    "right_hand_compute_ms",
    "arm_debug_publish_ms",
    "arm_pair_compute_ms",
    "gesture_publish_ms",
    "snapshot_build_ms",
    "target_lock_wait_ms",
    "target_update_ms",
    "worker_total_ms",
    "callback_to_transform_done_ms",
    "read_msg_total_ms",
)


def calculate_ik_stage_latencies_ms(
        processed_bone, solve_start_monotonic, solve_done_monotonic,
        ik_done_monotonic):
    """返回同一骨骼帧在IK阶段的等待/准备、求解和后处理耗时。"""
    return (
        (solve_start_monotonic - processed_bone.transform_done_monotonic) * 1000.0,
        (solve_done_monotonic - solve_start_monotonic) * 1000.0,
        (ik_done_monotonic - solve_done_monotonic) * 1000.0,
    )


def calculate_first_publish_latencies_ms(solution, publish_time,
                                         publish_monotonic):
    """Return solution wait and same-frame published output latencies."""
    solution_to_publish_ms = max(
        0.0, (publish_monotonic - solution.ready_monotonic) * 1000.0
    )
    if solution.processed_bone is None:
        return solution_to_publish_ms, None, None
    processed_bone = solution.processed_bone
    callback_to_publish_ms = max(
        0.0, (publish_time - processed_bone.receive_time) * 1000.0
    )
    source_to_publish_ms = None
    if processed_bone.source_timestamp_ms > 0:
        source_to_publish_ms = max(
            0.0,
            (publish_time - processed_bone.source_timestamp_ms / 1000.0)
            * 1000.0,
        )
    return solution_to_publish_ms, callback_to_publish_ms, source_to_publish_ms

class IkRos:
    def __init__(self, ik, ctrl_arm_idx=ArmIdx.LEFT, q_limit=None, publish_err=True, use_original_pose=False, end_effector_type="", send_srv=True, predict_gesture=False, hand_reference_mode="thumb_index", use_two_stage_ik=False):
        self.__start_time = None
        self.__timestamp = None
        self.__ctrl_arm_idx = ctrl_arm_idx
        self.__q_lb = np.array(q_limit[0], dtype=float) if q_limit is not None else None
        self.__q_ub = np.array(q_limit[1], dtype=float) if q_limit is not None else None
        self.__publish_err = publish_err
        self.__use_original_pose = use_original_pose
        self.arm_ik = ik
        self.controller_dt = 0.01  # 10ms
        self.__target_pose = (None, None)  # tuple(pos, quat), quat(x, y, z, w)
        self.__target_pose_right = (None, None)  # tuple(pos, quat), quat(x, y, z, w)
        self.__left_elbow_pos = None # agument ik problem
        self.__right_elbow_pos = None

        self.__recieved_new_target_pose = False
        self.__target_pose = (None, None)
        self.__current_pose = (None, None)
        self.__current_pose_right = (None, None)
        self.__joint_states = None  # left and right arm joint states
        self.__last_mc_time = None
        self.__time_cost_mc_glove = 0.0
        self.joySticks_data = None
        self.hand_finger_data = None
        self.__as_mc_ik = True  # 默认作为遥操作的IK，精度较低，速度要求较高
        self.__send_srv = send_srv
        self.__freeze_finger = False
        self.__button_y_last = False
        self.__frozen_left_hand_position = [0 for i in range(6)]
        self.__frozen_right_hand_position = [0 for i in range(6)]
        self.__robot_walking_status = False
        self.__arm_control_mode = 0
        self.__frozen_claw_pos = [0.0, 0.0]
        # IK 主循环已接管夹爪发布时为 True；准备姿态等待阶段为 False
        self.__ik_claw_publish_active = False
        self.__arm_dof = num_arm_joints_var
        self.__single_arm_dof = self.__arm_dof//2
        self.trigger_reset_mode = False

        # Thread lifecycle and latest-frame handoff. The subscriber callback only
        # replaces _pending_bone_frame; all expensive transforms run elsewhere.
        self.stop_event = threading.Event()
        self._bone_condition = threading.Condition()
        self._pending_bone_frame = None
        self._bone_input_seq = 0
        self._bone_overwrite_count = 0
        self._last_reported_bone_overwrite_count = 0
        self._target_lock = threading.Lock()
        # Coordinate conversion notifies this condition after atomically
        # committing a new target. IK solving is event driven; it no longer
        # polls the target from a fixed-rate loop.
        self._ik_condition = threading.Condition(self._target_lock)
        self._ik_target_generation = 0
        self._ik_target_ready_time = 0.0
        self._ik_target_ready_monotonic = 0.0
        # The fixed-rate publisher reads one immutable latest solution. The
        # last published command is kept separately so velocity limiting still
        # advances at controller_dt even when IK input arrives at a lower rate.
        self._ik_solution_lock = threading.Lock()
        self._latest_ik_solution = None
        self._last_published_arm_q = None
        self._ik_stale_solution_drop_count = 0
        self._ik_publish_timeout_count = 0
        self.ik_solution_timeout_s = max(
            0.05, float(rospy.get_param("~ik_solution_timeout_s", 0.2))
        )
        # Joystick callbacks publish an immutable snapshot. They never mutate
        # the arm transformer, so a joystick burst cannot block bone transforms.
        self._joystick_lock = threading.Lock()
        self._joystick_snapshot = None
        # Finger work has its own latest-frame mailbox and lower-rate worker.
        self._finger_condition = threading.Condition()
        self._pending_finger_input = None
        self._latest_processed_bone = None
        self._vr_is_running = False
        self._vr_is_hand_tracking = False
        self._vr_error = False
        
        # 添加两阶段IK控制参数
        self.__use_two_stage_ik = use_two_stage_ik  # 从构造函数参数获取
        
        # 允许通过ROS参数覆盖
        if rospy.has_param('~use_two_stage_ik'):
            self.__use_two_stage_ik = rospy.get_param('~use_two_stage_ik')
            rospy.loginfo(f"[IkRos] 通过ROS参数覆盖两阶段IK模式: {self.__use_two_stage_ik}")
        
        if self.__use_two_stage_ik:
            rospy.loginfo("[IkRos] 启用两阶段IK模式")
        else:
            rospy.loginfo("[IkRos] 使用标准IK模式")

        # 检查是否是半身模式
        self.only_half_up_body = False
        if rospy.has_param('/only_half_up_body'):
            self.only_half_up_body = rospy.get_param('/only_half_up_body')

        if rospy.has_param('/robot_type'):
            self.robot_type = rospy.get_param('/robot_type')
            if self.robot_type == 1:
                self.only_half_up_body = False
                print("[IkRos] 机器人类型为轮臂")
            else:
                print("[IkRos] 机器人类型为双足")
                if self.only_half_up_body:
                     print("✅采用用半身模式")
        else:
            self.robot_type = 0

        # 轮臂：用 /mobile_manipulator_mpc_observation 的 time（MPC 内部时间）判断是否已运行足够久（#3009）
        self._wheel_mpc_obs_time = None
        self._wheel_mm_mpc_min_internal_time = rospy.get_param("~wheel_mm_mpc_min_internal_time", 3.0)
        self._wheel_mm_mpc_observation_topic = rospy.get_param(
            "~wheel_mm_mpc_observation_topic", "/mobile_manipulator_mpc_observation"
        )

        self.use_arm_collision = rospy.get_param('~use_arm_collision', False)
        # self.hand_pub_timer = rospy.Timer(rospy.Duration(0.001), self.hand_finger_data_process)


        model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
        self.enable_vr_visualization = rospy.get_param("~enable_vr_visualization", False)
        self.enable_vr_latency_diagnostics = rospy.get_param(
            "~enable_vr_latency_diagnostics", False
        )
        self.enable_vr_transform_diagnostics = (
            self.enable_vr_latency_diagnostics
            and rospy.get_param("~enable_vr_transform_diagnostics", False)
        )
        self.enable_vr_transform_debug_topics = rospy.get_param(
            "~enable_vr_transform_debug_topics", False
        )
        self.vr_transform_debug_publish_hz = max(
            0.0, float(rospy.get_param("~vr_transform_debug_publish_hz", 10.0))
        )
        self.finger_processing_hz = max(
            1.0, float(rospy.get_param("~finger_processing_hz", 30.0))
        )
        self.quest3_arm_info_transformer = Quest3ArmInfoTransformer(
            model_path,
            vis_pub=self.enable_vr_visualization,
            predict_gesture=predict_gesture,
            hand_reference_mode=hand_reference_mode,
            debug_pub=self.enable_vr_transform_debug_topics,
            debug_publish_hz=self.vr_transform_debug_publish_hz,
        )
        rospy.loginfo(
            "[IkRos] VR visualization publishers: %s",
            "enabled" if self.enable_vr_visualization else "disabled",
        )
        rospy.loginfo(
            "[IkRos] VR latency diagnostics: %s",
            "enabled" if self.enable_vr_latency_diagnostics else "disabled",
        )
        rospy.loginfo(
            "[IkRos] VR transform diagnostics: %s",
            "enabled" if self.enable_vr_transform_diagnostics else "disabled",
        )
        rospy.loginfo(
            "[IkRos] VR transform debug topics: %s (%.1f Hz), finger worker: %.1f Hz",
            "enabled" if self.enable_vr_transform_debug_topics else "disabled",
            self.vr_transform_debug_publish_hz,
            self.finger_processing_hz,
        )
        self.quest3_arm_info_transformer.control_torso = control_torso
        initial_state = np.array([0, 0, 0, 0, 0, 0])  # 初始状态 [x, y, z, vx, vy, vz]
        initial_covariance = np.eye(6)  # 初始协方差矩阵
        process_noise = np.eye(6) * 0.001  # 过程噪声协方差矩阵
        measurement_noise = np.eye(3) * 1.1  # 测量噪声协方差矩阵

        initial_state[0:3] = self.arm_ik.left_hand_pose(self.arm_ik.q0())[0]
        self.kf_left = KalmanFilter3D(initial_state, initial_covariance, process_noise, measurement_noise,dt = 1)
        initial_state[0:3] = self.arm_ik.right_hand_pose(self.arm_ik.q0())[0]
        self.kf_right = KalmanFilter3D(initial_state, initial_covariance, process_noise, measurement_noise,dt = 1)
        self.external_q0 = None
        
        # 设置ArmIk实例的两阶段IK模式
        if hasattr(self.arm_ik, 'set_use_two_stage_ik'):
            self.arm_ik.set_use_two_stage_ik(self.__use_two_stage_ik)
            rospy.loginfo(f"[IkRos] ArmIk实例两阶段IK模式设置为: {self.__use_two_stage_ik}")
        
        self.arm_mode_changing = False
        # 检测到碰撞后，由外部控制手臂
        self.collision_check_control = False
        self.sensor_data_raw = None
        self.maxSpeed = rospy.get_param("/arm_move_spd_half_up_body", 0.21)
        self.threshold_arm_diff_half_up_body = rospy.get_param("/threshold_arm_diff_half_up_body", 0.2)
        self._interp_time_last = rospy.Time.now().to_sec()
        
        # 半身模式下退出mode2时保持手臂位置的变量
        self.frozen_arm_state = None  # 保存退出mode2时的手臂状态
        self.hold_arm_timer = None  # 保持手臂位置的定时器
        self.optimized_state = None  # 存储MPC优化后的状态


        if self.use_arm_collision:
            self.pub = rospy.Publisher("/arm_collision/kuavo_arm_traj", JointState, queue_size=2)
        else:
            self.pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=2)
        self.pub_origin_joint = rospy.Publisher("/kuavo_arm_traj_origin", Float32MultiArray, queue_size=10)
        self.pub_filtered_joint = rospy.Publisher("/kuavo_arm_traj_filtered", Float32MultiArray, queue_size=10)
        self.pub_real_arm_hand_pose = rospy.Publisher("/drake_ik/real_arm_hand_pose", twoArmHandPose, queue_size=10)
        self.pub_time_cost = rospy.Publisher(
            "/drake_ik/time_cost/ik", Float32, queue_size=10
        )
        self.pub_ik_solve_error = rospy.Publisher(
            "/drake_ik/ik_solve_error", ikSolveError, queue_size=10
        )
        self.control_robot_hand_position_pub = rospy.Publisher(
            "control_robot_hand_position", robotHandPosition, queue_size=10
        )
        self.pub_ik_solved_eef_pose = rospy.Publisher(
            "/drake_ik/eef_pose", twoArmHandPose, queue_size=10
        )
        # 带时间戳：/drake_ik/input_pos
        self.pub_ik_input_pos = rospy.Publisher(
            "/drake_ik/input_pos", Float32MultiArrayStamped, queue_size=10
        )
        self.pub_q0_tmp = rospy.Publisher(
            "/drake_ik/q0_tmp", Float32MultiArray, queue_size=10
        )
        self.leju_claw_command_pub = rospy.Publisher(
            "leju_claw_command", lejuClawCommand, queue_size=10
        )
        
        if self.robot_type == 1:
            # 添加发布/mm/two_arm_hand_pose_cmd话题的发布器
            self.pub_mm_two_arm_hand_pose_cmd = rospy.Publisher(
                "/mm/two_arm_hand_pose_cmd", twoArmHandPoseCmd, queue_size=10
            )
        
        # 添加可视化marker发布器
        self.ik_visualization_pub = rospy.Publisher(
            "/ik_visualization_markers", MarkerArray, queue_size=10
        )

        # ── 延迟测量发布器 ──
        # 默认不向ROS master注册这些诊断话题；只有显式开启统一开关时
        # 才创建真实发布器，避免常规运行时的序列化、传输和录包开销。
        def latency_publisher(topic, msg_type, queue_size=10):
            if not self.enable_vr_latency_diagnostics:
                return None
            return rospy.Publisher(topic, msg_type, queue_size=queue_size)

        # VR数据处理节点 → 绝对式IK节点 的通信延迟
        self.comm_latency_pub = latency_publisher(
            "/vr_absolute/comm_latency_ms", Float64, queue_size=10
        )
        # 绝对式IK处理延迟（骨骼接收 → IK求解完成）
        self.arm_traj_latency_pub = latency_publisher(
            "/vr_absolute/arm_traj_latency_ms", Float64, queue_size=10
        )
        # 骨骼回调入口 → 坐标转换完成（包含最新帧邮箱等待）
        self.transform_pipeline_latency_pub = latency_publisher(
            "/vr_absolute/transform_pipeline_latency_ms", Float64, queue_size=10
        )
        # 单帧坐标转换本身的处理耗时
        self.transform_processing_latency_pub = latency_publisher(
            "/vr_absolute/transform_processing_latency_ms", Float64, queue_size=10
        )
        # Finger work is asynchronous and excluded from the arm transform path.
        self.finger_processing_latency_pub = latency_publisher(
            "/vr_absolute/finger_processing_latency_ms", Float64, queue_size=10
        )
        # VR发布时刻 → 同一帧IK求解完成
        self.end_to_end_latency_pub = latency_publisher(
            "/vr_absolute/end_to_end_latency_ms", Float64, queue_size=10
        )
        # 坐标转换完成 → computeIK开始（含IK线程等待与求解前准备）
        self.ik_wait_latency_pub = latency_publisher(
            "/vr_absolute/ik_wait_latency_ms", Float64, queue_size=10
        )
        # computeIK调用本身的求解耗时
        self.ik_solve_latency_pub = latency_publisher(
            "/vr_absolute/ik_solve_latency_ms", Float64, queue_size=10
        )
        # computeIK返回 → 本轮轨迹后处理完成
        self.ik_postprocess_latency_pub = latency_publisher(
            "/vr_absolute/ik_postprocess_latency_ms", Float64, queue_size=10
        )
        # 坐标转换完成 → 目标快照提交并通知IK线程
        self.ik_target_commit_latency_pub = latency_publisher(
            "/vr_absolute/ik_target_commit_latency_ms", Float64, queue_size=10
        )
        # 目标提交通知 → IK线程取得该目标（包含线程调度和锁等待）
        self.ik_thread_wakeup_latency_pub = latency_publisher(
            "/vr_absolute/ik_thread_wakeup_latency_ms", Float64, queue_size=10
        )
        # IK线程持锁复制同一目标快照
        self.ik_target_snapshot_latency_pub = latency_publisher(
            "/vr_absolute/ik_target_snapshot_latency_ms", Float64, queue_size=10
        )
        # 求解前双臂正运动学
        self.ik_fk_latency_pub = latency_publisher(
            "/vr_absolute/ik_fk_latency_ms", Float64, queue_size=10
        )
        # FK完成 → computeIK开始（滤波、姿态转换、初值与约束准备）
        self.ik_input_prepare_latency_pub = latency_publisher(
            "/vr_absolute/ik_input_prepare_latency_ms", Float64, queue_size=10
        )
        # 目标提交 → 最新有效IK解写入发布邮箱
        self.ik_solution_ready_latency_pub = latency_publisher(
            "/vr_absolute/ik_solution_ready_latency_ms", Float64, queue_size=10
        )
        # 最新有效解 → 第一次100Hz轨迹发布
        self.ik_solution_to_publish_latency_pub = latency_publisher(
            "/vr_absolute/ik_solution_to_publish_latency_ms", Float64, queue_size=10
        )
        # 单次100Hz发布线程内的限速、消息构造与发布耗时
        self.ik_publish_execution_latency_pub = latency_publisher(
            "/vr_absolute/ik_publish_execution_latency_ms", Float64, queue_size=10
        )
        # 实际轨迹发布周期间隔，用于观察100Hz抖动
        self.ik_publish_period_pub = latency_publisher(
            "/vr_absolute/ik_publish_period_ms", Float64, queue_size=10
        )
        # IK回调接收/VR节点发布 → 同一解首次轨迹发布
        self.published_arm_traj_latency_pub = latency_publisher(
            "/vr_absolute/published_arm_traj_latency_ms", Float64, queue_size=10
        )
        self.published_end_to_end_latency_pub = latency_publisher(
            "/vr_absolute/published_end_to_end_latency_ms", Float64, queue_size=10
        )
        self.ik_stale_solution_drop_pub = latency_publisher(
            "/vr_absolute/ik_stale_solution_dropped_frames",
            UInt64,
            queue_size=10,
        )
        self.ik_publish_timeout_pub = latency_publisher(
            "/vr_absolute/ik_publish_timeout_events", UInt64, queue_size=10
        )
        # 最新帧邮箱覆盖的累计帧数
        self.bone_overwrite_count_pub = latency_publisher(
            "/vr_absolute/mailbox_overwritten_frames", UInt64, queue_size=10
        )
        self.transform_stage_latency_pub = None
        if self.enable_vr_transform_diagnostics:
            self.transform_stage_latency_pub = rospy.Publisher(
                "/vr_absolute/transform_stage_latency_ms",
                Float32MultiArrayStamped,
                queue_size=50,
            )

        try:
            end_effector_mapping = {
                QIANGNAO: QIANGNAO,
                JODELL: JODELL,
                LEJUCLAW: LEJUCLAW,
                QIANGNAO_TOUCH:QIANGNAO_TOUCH,
                REVO2: REVO2,
                LINKER_HAND: LINKER_HAND
            }
            if end_effector_type in end_effector_mapping:
                self.end_effector_type = end_effector_mapping[end_effector_type]
            else:
                self.end_effector_type = QIANGNAO
        except Exception as e:
            print(f"get end effector type error: {e}, use default qiangnao")
            self.end_effector_type = QIANGNAO
        print(f"\033[93m--------------------------------------------------\033[0m")        
        print(f"\033[93m- End effector type: {self.end_effector_type} \033[0m")
        print(f"\033[93m--------------------------------------------------\033[0m")        

        # All callback-visible state must be initialized before subscriptions
        # are registered. This also removes the startup race seen in rosout.
        self.initial_q_first = None
        self.__need_reset_ik_guess = False
        self.__first_change_arm_mode = True

        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)
        rospy.on_shutdown(self._stop_threads)

        self.bone_processing_thread = threading.Thread(
            target=self._bone_processing_loop,
            name="vr-bone-latest-frame",
        )
        self.bone_processing_thread.start()

        self.finger_processing_thread = threading.Thread(
            target=self._finger_processing_loop,
            name="vr-finger-latest-frame",
        )
        self.finger_processing_thread.start()

        self._initialize_ros_interfaces()

        self.ik_publish_thread = threading.Thread(
            target=self.ik_fixed_rate_publish_thread,
            name="vr-absolute-ik-publisher",
        )
        self.ik_publish_thread.start()
        # Output cadence is safety-critical; keep it above the event-driven
        # solver when real-time scheduling permission is available.
        set_thread_priority(self.ik_publish_thread, int(SCHED_FIFO), 51)

        self.hand_command_publish_thread = threading.Thread(
            target=self.hand_fixed_rate_publish_thread,
            name="vr-absolute-hand-publisher",
        )
        self.hand_command_publish_thread.start()

        self.ik_thread = threading.Thread(
            target=self.ik_controller_thread,
            name="vr-absolute-ik-solver",
        )
        self.ik_thread.start()
        set_thread_priority(self.ik_thread, int(SCHED_FIFO), 50)

        self.run()
        self.ik_thread.join()
        self.ik_publish_thread.join()
        self.hand_command_publish_thread.join()
        self.bone_processing_thread.join()
        self.finger_processing_thread.join()

    def _initialize_ros_interfaces(self):
        """Register callbacks only after every callback dependency exists."""
        self.joint_sub = rospy.Subscriber(
            "/robot_arm_q_v_tau", robotArmQVVD, self.kuavo_joint_states_callback, queue_size=10
        )
        self.quest_bone_poses_sub = rospy.Subscriber(
            "/leju_quest_bone_poses",
            PoseInfoList,
            self.quest_bone_poses_callback,
            queue_size=1,
            # Read a possible burst in one pass, then rospy keeps only the newest message.
            buff_size=1024 * 1024,
        )
        self.joySticks_sub = rospy.Subscriber(
            "/quest_joystick_data", JoySticks, self.joySticks_data_callback, queue_size=3
        )
        self.ik_cmd_sub = rospy.Subscriber(
            "/ik/two_arm_hand_pose_cmd", twoArmHandPoseCmd,
            self.two_arm_hand_pose_target_callback, queue_size=10
        )
        self.sensor_data_raw_sub = rospy.Subscriber(
            "/sensors_data_raw", sensorsData, self.sensor_data_raw_callback, queue_size=1
        )
        self.optimized_state_sub = rospy.Subscriber(
            "/humanoid_controller/optimizedState_wbc_mrt_origin",
            Float64MultiArray,
            self.optimized_state_callback,
            queue_size=10,
        )

        self._wheel_mpc_obs_sub = None
        if self.robot_type == 1:
            self._wheel_mpc_obs_sub = rospy.Subscriber(
                self._wheel_mm_mpc_observation_topic,
                mpc_observation,
                self.wheel_mpc_observation_callback,
                queue_size=10,
            )

        self.stop_robot_sub = rospy.Subscriber(
            "/stop_robot", Bool, self.stop_robot_callback, queue_size=1
        )
        self.robot_walking_status_sub = rospy.Subscriber(
            "/robot_walking_status", Bool, self.robot_walking_status_callback, queue_size=1
        )
        self.arm_mode_sub = rospy.Subscriber(
            "/quest3/triger_arm_mode", Int32, self.arm_mode_callback
        )
        self.arm_control_mode_sub = rospy.Subscriber(
            "/humanoid/mpc/arm_control_mode",
            Float64MultiArray,
            self.arm_control_mode_callback,
        )
        self.arm_mode_service = rospy.Service(
            "/quest3/set_arm_mode_changing", Trigger, self.set_arm_mode_changing_callback
        )
        self.set_two_stage_ik_service = rospy.Service(
            "/quest3/set_two_stage_ik", SetBool, self.set_two_stage_ik_callback
        )
   
    def run(self):
        while rospy.is_shutdown() is False:
            rospy.spin()

    def _stop_threads(self):
        self.stop_event.set()
        with self._bone_condition:
            self._bone_condition.notify_all()
        with self._finger_condition:
            self._finger_condition.notify_all()
        with self._ik_condition:
            self._ik_condition.notify_all()

    def shutdown(self, signal, frame):
        rospy.loginfo("Shutting down...")
        self._stop_threads()
        rospy.signal_shutdown("Shutdown")
        rospy.loginfo("Shutdown complete.")

    @staticmethod
    def drake_pose_to_tuple(drake_pose):
        quat = drake_pose.rotation().ToQuaternion()
        quat_vec = np.array([quat.x(), quat.y(), quat.z(), quat.w()])
        return (drake_pose.translation(), quat_vec)

    def get_two_arm_pose(self, q_arm):
        # q0 = self.arm_ik.q0()
        # q0[7:] = q_arm
        left_hand_pose = self.arm_ik.left_hand_pose(q_arm)
        right_hand_pose = self.arm_ik.right_hand_pose(q_arm)
        # print(f"left_hand_pose: {left_hand_pose}")
        quat_left = rpy_to_quaternion(left_hand_pose[1][0], left_hand_pose[1][1], left_hand_pose[1][2])
        quat_right = rpy_to_quaternion(right_hand_pose[1][0], right_hand_pose[1][1], right_hand_pose[1][2])
        return (left_hand_pose[0], quat_left), (right_hand_pose[0], quat_right)

    def current_pose(self):
        return self.__current_pose

    def limit_angle(self, q):
        if self.__q_lb is not None and self.__q_ub is not None:
            q_limited = np.zeros(self.__arm_dof)
            for i in range(self.__arm_dof):
                q_limited[i] = max(self.__q_lb[i], min(q[i], self.__q_ub[i]))
            return q_limited
        else:
            return q

    def limit_angle_by_velocity(self, q_last, q_now, vel_limit=50.0, shoulder_vel_limit=600.0):
        """
        limit the angle change by velocity, default 50 deg/s
        对左右手臂的第一个关节(肩膀俯仰)进行120度限制
        """
        size = len(q_now)
        q_limited = q_now.copy()
        agl_limit = self.controller_dt * vel_limit * np.pi / 180.0  # deg/s to rad/s
        
        # 120度限制转换为弧度
        angle_limit_120_deg = self.controller_dt * shoulder_vel_limit * np.pi / 180.0  # 约2.09弧度
        
        for i in range(size):
            # 速度限制
            q_limited[i] = max(q_last[i] - agl_limit, min(q_now[i], q_last[i] + agl_limit))
            
            # 对左右手臂的第一个关节进行120度限制
            if i == 0:  # 左臂第一个关节 (l_arm_pitch)
                q_limited[i] = max(q_last[i]-angle_limit_120_deg, min(q_now[i], q_last[i] + angle_limit_120_deg))
            elif i == self.__single_arm_dof:  # 右臂第一个关节 (r_arm_pitch)，索引7
                q_limited[i] = max(q_last[i]-angle_limit_120_deg, min(q_now[i], q_last[i] + angle_limit_120_deg))
                
        return q_limited

    @staticmethod
    def change_arm_ctrl_mode(mode: int):
        service_name = "/change_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name)
            changeHandTrackingMode_srv = rospy.ServiceProxy(
                service_name, changeArmCtrlMode
            )
            changeHandTrackingMode_srv(mode)
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")

    @staticmethod
    def wheel_change_arm_ctrl_mode(mode: int):
        service_name = "/wheel_arm_change_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name)
            changeHandTrackingMode_srv = rospy.ServiceProxy(
                service_name, changeArmCtrlMode
            )
            changeHandTrackingMode_srv(mode)
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")

    @staticmethod
    def change_arm_ctrl_mode4kuavo(mode: bool):
        service_name = "/change_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name)
            changeHandTrackingMode_srv = rospy.ServiceProxy(
                service_name, changeArmCtrlModeKuavo
            )
            changeHandTrackingMode_srv(mode)
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")

    @staticmethod
    def control_lejuclaw(pos:list):
        # print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> control_lejuclaw: {pos}")
        service_name = "/control_robot_leju_claw"
        try:
            rospy.wait_for_service("/control_robot_leju_claw", timeout=1)
            control_lejucalw_srv = rospy.ServiceProxy(
                service_name, controlLejuClaw
            )
            req = controlLejuClawRequest()
            req.data.name = ['left_claw', 'right_claw']
            req.data.position = pos
            req.data.velocity = [90, 90]
            # print(f">>>>>>>>>>>>>>>> control_lejucalw_srv: {req}")
            control_lejucalw_srv(req)
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
        except Exception as e:
            rospy.logerr(f"Error: {e}")   

    def pub_leju_claw_command(self, pos:list):
        msg = lejuClawCommand()
        msg.header.stamp = rospy.Time.now()
        msg.data.name = ['left_claw', 'right_claw']
        msg.data.position = pos
        msg.data.velocity = [90, 90]
        self.leju_claw_command_pub.publish(msg)

    def pub_solved_arm_eef_pose(self, q_robot, current_pose, current_pose_right):
        msg = twoArmHandPose()
        msg.header.frame_id = "torso"
        msg.header.stamp = rospy.Time.now()
        msg.left_pose.pos_xyz = current_pose[0]
        msg.left_pose.quat_xyzw = current_pose[1]
        msg.left_pose.joint_angles = q_robot[-self.__arm_dof:-self.__single_arm_dof]
        msg.right_pose.pos_xyz = current_pose_right[0]
        msg.right_pose.quat_xyzw = current_pose_right[1]
        msg.right_pose.joint_angles = q_robot[-self.__single_arm_dof:]
        self.pub_ik_solved_eef_pose.publish(msg)

    def _clear_latest_ik_solution(self, clear_published_command=False):
        with self._ik_solution_lock:
            self._latest_ik_solution = None
            if clear_published_command:
                self._last_published_arm_q = None

    def _wait_for_latest_ik_target(self, last_generation):
        """Wait for and atomically copy the newest committed IK target."""
        with self._ik_condition:
            has_target = self._ik_condition.wait_for(
                lambda: (
                    self._ik_target_generation != last_generation
                    or self.stop_event.is_set()
                    or rospy.is_shutdown()
                ),
                timeout=0.1,
            )
            if self.stop_event.is_set() or rospy.is_shutdown():
                return None
            if not has_target or self._ik_target_generation == last_generation:
                return ()

            snapshot_start = time.perf_counter()
            target_generation = self._ik_target_generation
            target_ready_time = self._ik_target_ready_time
            target_ready_monotonic = self._ik_target_ready_monotonic
            target_pose = self._copy_pose(self.__target_pose)
            target_pose_right = self._copy_pose(self.__target_pose_right)
            left_elbow_snapshot = self._copy_optional_array(self.__left_elbow_pos)
            right_elbow_snapshot = self._copy_optional_array(self.__right_elbow_pos)
            processed_bone = self._latest_processed_bone
            vr_error = self._vr_error
            snapshot_done = time.perf_counter()

        return {
            "generation": target_generation,
            "ready_time": target_ready_time,
            "ready_monotonic": target_ready_monotonic,
            "snapshot_start_monotonic": snapshot_start,
            "snapshot_done_monotonic": snapshot_done,
            "target_pose": target_pose,
            "target_pose_right": target_pose_right,
            "left_elbow": left_elbow_snapshot,
            "right_elbow": right_elbow_snapshot,
            "processed_bone": processed_bone,
            "vr_error": vr_error,
        }

    def ik_fixed_rate_publish_thread(self):
        """Publish the newest safe IK solution at the controller's fixed rate."""
        rate = rospy.Rate(1.0 / self.controller_dt)
        last_publish_monotonic = None
        first_published_generation = -1
        timeout_active = False

        while not self.stop_event.is_set() and not rospy.is_shutdown():
            try:
                with self._ik_solution_lock:
                    solution = self._latest_ik_solution
                    last_command = (
                        None if self._last_published_arm_q is None
                        else np.asarray(self._last_published_arm_q).copy()
                    )

                now_monotonic = time.perf_counter()
                solution_is_fresh = solution is not None
                if (solution_is_fresh and solution.processed_bone is not None
                        and now_monotonic - solution.ready_monotonic
                        > self.ik_solution_timeout_s):
                    solution_is_fresh = False
                    if not timeout_active:
                        timeout_active = True
                        self._ik_publish_timeout_count += 1
                        if self.enable_vr_latency_diagnostics:
                            self.ik_publish_timeout_pub.publish(
                                UInt64(self._ik_publish_timeout_count)
                            )
                    rospy.logwarn_throttle(
                        1.0,
                        "Latest absolute VR IK solution is older than %.0f ms; "
                        "trajectory publishing is paused.",
                        self.ik_solution_timeout_s * 1000.0,
                    )
                elif solution_is_fresh:
                    timeout_active = False

                if solution_is_fresh:
                    desired_arm_q = np.asarray(solution.desired_arm_q).copy()
                    if not self.arm_mode_changing and last_command is not None:
                        command_arm_q = self.limit_angle_by_velocity(
                            last_command,
                            desired_arm_q,
                            vel_limit=720,
                            shoulder_vel_limit=solution.shoulder_velocity_limit,
                        )
                    else:
                        command_arm_q = desired_arm_q

                    publish_start = (
                        time.perf_counter()
                        if self.enable_vr_latency_diagnostics else None
                    )
                    filtered_msg = Float32MultiArray()
                    filtered_msg.data = command_arm_q * 180.0 / np.pi
                    self.pub_filtered_joint.publish(filtered_msg)
                    published = self.publish_joint_states(
                        q_now=command_arm_q, q_last=last_command
                    )
                    publish_done = (
                        time.perf_counter()
                        if self.enable_vr_latency_diagnostics else None
                    )

                    if published:
                        with self._ik_solution_lock:
                            self._last_published_arm_q = command_arm_q.copy()
                        if self.enable_vr_latency_diagnostics:
                            publish_time = time.time()
                            self.ik_publish_execution_latency_pub.publish(
                                Float64((publish_done - publish_start) * 1000.0)
                            )
                            if last_publish_monotonic is not None:
                                self.ik_publish_period_pub.publish(
                                    Float64(
                                        (publish_done - last_publish_monotonic)
                                        * 1000.0
                                    )
                                )
                            last_publish_monotonic = publish_done

                            if solution.target_generation != first_published_generation:
                                first_published_generation = solution.target_generation
                                (solution_to_publish_ms,
                                 callback_to_publish_ms,
                                 source_to_publish_ms) = (
                                    calculate_first_publish_latencies_ms(
                                        solution, publish_time, publish_done
                                    )
                                )
                                self.ik_solution_to_publish_latency_pub.publish(
                                    Float64(solution_to_publish_ms)
                                )
                                if callback_to_publish_ms is not None:
                                    self.published_arm_traj_latency_pub.publish(
                                        Float64(callback_to_publish_ms)
                                    )
                                if source_to_publish_ms is not None:
                                    self.published_end_to_end_latency_pub.publish(
                                        Float64(source_to_publish_ms)
                                    )

            except Exception as exc:
                rospy.logerr_throttle(
                    1.0, "Fixed-rate absolute IK publishing failed: %s", exc
                )

            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                return

    def hand_fixed_rate_publish_thread(self):
        """Preserve 100 Hz hand commands without delaying arm publication."""
        rate = rospy.Rate(1.0 / self.controller_dt)
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            if self.__ik_claw_publish_active:
                try:
                    self.hand_finger_data_process(0)
                except Exception as exc:
                    rospy.logerr_throttle(
                        1.0, "Fixed-rate hand publishing failed: %s", exc
                    )
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                return

    def ik_controller_thread(self):
        rate = rospy.Rate(1 / self.controller_dt)
        traj_X_G = None
        traj_X_G_right = None
        # print("[ik]:waiting for first joint states")
        # while self.__joint_states is None:
        #     rate.sleep()
        # print("[ik]:first joint states recieved.")
        print("[ik]: Waiting for first eef target pose")
        while not self.__recieved_new_target_pose:
            if self.stop_event.is_set():
                print("[ik]: Stop event is set, exit.")
                return
            rate.sleep()
        print("[ik]: First eef target pose recieved.")
        if self.__as_mc_ik:
            print("[ik]: Waiting for OK-guesture(hold on for 1-2 seconds) to start teleoperation...")
            while True:
                with self._target_lock:
                    vr_is_running = self._vr_is_running
                    vr_error = self._vr_error
                if vr_is_running:
                    break
                self.hand_finger_data_process(0)
                if self.stop_event.is_set():
                    print("[ik]: Stop event is set, exit.")
                    return
                if vr_error:
                    sys.stdout.write("\r\033[91mDetected VR ERROR!!! Please restart VR app in quest3 or check the battery level of the joystick!!!\033[0m")
                rate.sleep()
            print("[ik]: OK-guesture recieved!!!")

        if self.__send_srv:
            if self.robot_type == 1:
                rospy.loginfo(
                    "[IkRos] 轮臂: 等待 %s 中 MPC time > %.1fs 后再切换手臂模式（#3009）...",
                    self._wheel_mm_mpc_observation_topic,
                    self._wheel_mm_mpc_min_internal_time,
                )
                while not self.stop_event.is_set() and not rospy.is_shutdown():
                    if self._wheel_mpc_stable_for_mm_cmd():
                        rospy.loginfo("[IkRos] 轮臂: MPC 内部时间已满足，发送手臂控制模式")
                        break
                    rate.sleep()
                if self.stop_event.is_set() or rospy.is_shutdown():
                    print("[ik]: Stop or shutdown before MPC ready, exit.")
                    return
            print("[ik]: Send start service signal to robot, wait for response.")
            self.change_arm_ctrl_mode(2)
            if self.robot_type == 1:
                self.wheel_change_arm_ctrl_mode(2)
            # self.change_arm_ctrl_mode4kuavo(True)
            print("\033[92m[ik]: Recied start signal response, Start teleoperation.\033[0m")

        self.arm_mode_changing = True
        if self.__as_mc_ik:
            print("[ik]: If you want to stop teleoperation, please make a Shot-guesture(hold on for 1-2 seconds).")
        q_last = self.arm_ik.q0() if self.external_q0 is None else self.external_q0
        pre_q_first = q_last.copy()
        # q_last[-14:] = self.__joint_states  # two arm joint states
        # q_last[7:14] = [0.1084,  0.0478 , 0.1954 ,-0.0801 , 0.1966 ,-0.5861 , 0.0755]
        q_now = q_last
        t_ctrl = 0.0
        print(f"IK Type: {self.arm_ik.type()}")
        run_count, fail_count = 0, 0
        sum_time_cost = 0.0
        arm_q_filtered = [0.0] * self.__arm_dof
        is_runing = False
        self.__ik_claw_publish_active = True
        last_target_generation = 0
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            ik_solve_start_monotonic = None
            ik_solve_done_monotonic = None
            solution = None
            target_state = self._wait_for_latest_ik_target(
                last_target_generation
            )
            if target_state is None:
                return
            if not target_state:
                continue
            last_target_generation = target_state["generation"]

            target_ready_monotonic = target_state["ready_monotonic"]
            if (self.enable_vr_latency_diagnostics
                    and target_ready_monotonic > 0.0):
                self.ik_thread_wakeup_latency_pub.publish(
                    Float64(
                        max(
                            0.0,
                            (target_state["snapshot_start_monotonic"]
                             - target_ready_monotonic) * 1000.0,
                        )
                    )
                )
            if self.enable_vr_latency_diagnostics:
                self.ik_target_snapshot_latency_pub.publish(
                    Float64(
                        (target_state["snapshot_done_monotonic"]
                         - target_state["snapshot_start_monotonic"]) * 1000.0
                    )
                )

            target_pose = target_state["target_pose"]
            target_pose_right = target_state["target_pose_right"]
            left_elbow_snapshot = target_state["left_elbow"]
            right_elbow_snapshot = target_state["right_elbow"]
            processed_bone = target_state["processed_bone"]
            vr_error = target_state["vr_error"]

            # 检测是否需要重置IK初始猜测（模式切换时）
            if self.__need_reset_ik_guess:
                # 重置q_last和arm_ik内部的last_solution
                # 将q_last设置为全0，避免内翻
                q0_ref = self.arm_ik.q0() if self.external_q0 is None else self.external_q0
                q_last = np.zeros(len(q0_ref))
                pre_q_first = q_last.copy()
                q_now = q_last.copy()
                # 重置IK求解器内部的last_solution为默认初始状态
                if hasattr(self.arm_ik, 'reset_last_solution'):
                    self.arm_ik.reset_last_solution(self.arm_ik.q0())
                
                # 重置arm_q_filtered为当前q_last的手臂部分
                arm_q_filtered = q_last[-self.__arm_dof:].copy()
                
                self.__need_reset_ik_guess = False

            if self.trigger_reset_mode:
                with self._ik_condition:
                    self.__target_pose = (None, None)
                    self.__target_pose_right = (None, None)
                    self._latest_processed_bone = None
                self.__current_pose = (None, None)
                self.__current_pose_right = (None, None)
                q_last = pre_q_first.copy()
                q_now = q_last.copy()
                self._clear_latest_ik_solution(clear_published_command=True)
                self.trigger_reset_mode = False
                continue

            if self.__as_mc_ik and vr_error:
                self._clear_latest_ik_solution()
                rospy.logerr_throttle(
                    1.0,
                    "Detected VR ERROR; absolute IK publishing is paused.",
                )
                continue

            # Keep the IK initial state close to the command actually emitted
            # by the 100 Hz publisher, not merely the last raw solver result.
            with self._ik_solution_lock:
                last_published_arm_q = (
                    None if self._last_published_arm_q is None
                    else np.asarray(self._last_published_arm_q).copy()
                )
            if last_published_arm_q is not None:
                q_last[-self.__arm_dof:] = last_published_arm_q

            fk_start_monotonic = time.perf_counter()
            self.__current_pose, self.__current_pose_right = (
                self.get_two_arm_pose(q_last)
            )
            fk_done_monotonic = time.perf_counter()
            if self.enable_vr_latency_diagnostics:
                self.ik_fk_latency_pub.publish(
                    Float64((fk_done_monotonic - fk_start_monotonic) * 1000.0)
                )
            self.pub_solved_arm_eef_pose(
                q_last, self.__current_pose, self.__current_pose_right
            )

            is_runing_last = is_runing
            is_runing = True
            
            if(not is_runing_last and is_runing):
                self.arm_mode_changing = True
            if target_pose[0] is None or target_pose_right[0] is None or \
                self.__current_pose[0] is None or self.__current_pose_right[0] is None:
                continue
            if self.arm_ik.type().name() == IkTypeIdx.TorsoIK.name():
                l_hand_pose, l_hand_RPY, l_hand_quat = None, None, None
                r_hand_pose, r_hand_RPY, r_hand_quat = None, None, None
                l_elbow_pos, r_elbow_pos = None, None
                left_shoulder_rpy_in_robot, right_shoulder_rpy_in_robot = None, None
                if target_pose[0] is not None and (self.__ctrl_arm_idx == ArmIdx.BOTH
                                                          or self.__ctrl_arm_idx == ArmIdx.LEFT):
                    l_hand_pose, l_hand_quat = target_pose
                    l_hand_pose = self.kf_left.filter(l_hand_pose)
                    l_hand_RPY = quaternion_to_RPY(l_hand_quat)
                    l_elbow_pos = left_elbow_snapshot
                    # if l_elbow_pos is not None:
                    #     # print(f"l_elbow_pos: {l_elbow_pos}")
                    #     l_elbow_pos[0] = -0.3 if l_elbow_pos[0] < -0.3 else l_elbow_pos[0]
                    if processed_bone is not None:
                        left_shoulder_rpy_in_robot = processed_bone.left_shoulder_rpy
                if target_pose_right[0] is not None and (self.__ctrl_arm_idx == ArmIdx.BOTH
                                                                or self.__ctrl_arm_idx == ArmIdx.RIGHT):
                    r_hand_pose, r_hand_quat = target_pose_right
                    r_hand_pose = self.kf_right.filter(r_hand_pose)
                    r_hand_RPY = quaternion_to_RPY(r_hand_quat)
                    r_elbow_pos = right_elbow_snapshot
                    # if r_elbow_pos is not None:
                    #     r_elbow_pos[0] = -0.3 if r_elbow_pos[0] < -0.3 else r_elbow_pos[0]
                    if processed_bone is not None:
                        right_shoulder_rpy_in_robot = processed_bone.right_shoulder_rpy

                ik_input_data = []
                if l_hand_pose is not None and l_hand_quat is not None:
                    ik_input_data.extend(np.asarray(l_hand_pose).flatten().tolist())
                    ik_input_data.extend(np.asarray(l_hand_quat).flatten().tolist())
                else:
                    ik_input_data.extend([np.nan] * 7)
                if r_hand_pose is not None and r_hand_quat is not None:
                    ik_input_data.extend(np.asarray(r_hand_pose).flatten().tolist())
                    ik_input_data.extend(np.asarray(r_hand_quat).flatten().tolist())
                else:
                    ik_input_data.extend([np.nan] * 7)
                arr = np.asarray(ik_input_data, dtype=np.float32)

                input_pos_msg = Float32MultiArrayStamped()
                input_pos_msg.header.stamp = rospy.Time.now()
                input_pos_msg.data.data = arr.tolist()
                self.pub_ik_input_pos.publish(input_pos_msg)
                time_0 = time.time()
                # 通过限制初值，避免迭代到不可解的区域
                q0_tmp = q_last.copy()
                threashold = -3.0
                q0_tmp[-self.__arm_dof] += 0.5 if q0_tmp[-self.__arm_dof] < threashold else 0.0
                q0_tmp[-self.__single_arm_dof] += 0.5 if q0_tmp[-self.__single_arm_dof] < threashold else 0.0
                # 限制左臂和右臂的特定关节角度在 [-0.1, 0.1] 范围内
                q0_tmp[-self.__arm_dof + 2] = limit_value(q0_tmp[-self.__arm_dof + 2], -0.1, 0.1)
                q0_tmp[-self.__single_arm_dof + 2] = limit_value(q0_tmp[-self.__single_arm_dof + 2], -0.1, 0.1)
                # 针对roban的调整（当arm_dof为8时）
                if self.__arm_dof == 8:
                    q0_tmp[-self.__single_arm_dof] = limit_value(q0_tmp[-self.__single_arm_dof], -float('inf'), 0.0)
                    q0_tmp[0] = limit_value(q0_tmp[0], -float('inf'), 0.0)

                # 限制肘部位置，避免动作幅度过大导致肩膀翻转
                if self.__arm_dof == 8:
                    left_shoulder_pos = self.get_shoulder_position(q0_tmp, "left")
                    right_shoulder_pos = self.get_shoulder_position(q0_tmp, "right")
                    if l_elbow_pos[2]>0.1:
                        if l_elbow_pos[0] < (left_shoulder_pos[0]+0.1):
                            l_elbow_pos[0] = left_shoulder_pos[0]+0.1
                    if r_elbow_pos[2]>0.1:
                        if r_elbow_pos[0] < (right_shoulder_pos[0]+0.1):
                            r_elbow_pos[0] = right_shoulder_pos[0]+0.1

                
                # q0_tmp[-self.__single_arm_dof + 3] = -0.5
                # q0_tmp[-self.__arm_dof+3] = -0.5
                
                # # 计算肘部角度并设置到q0_tmp中
                # # 获取肩膀位置
                # left_shoulder_pos = self.get_shoulder_position(q0_tmp, "left")
                # right_shoulder_pos = self.get_shoulder_position(q0_tmp, "right")
                
                # # 计算左臂肘部角度
                # if left_shoulder_pos is not None and l_elbow_pos is not None and l_hand_pose is not None:
                #     left_elbow_angle = self.calculate_elbow_angle(left_shoulder_pos, l_elbow_pos, l_hand_pose)
                #     if left_elbow_angle is not None:
                #         # 左臂肘部关节通常是第4个关节（索引3）
                #         elbow_joint_idx = 3
                #         q0_tmp[-self.__arm_dof + elbow_joint_idx] = -left_elbow_angle
                #         print(f"左臂肘部角度: {left_elbow_angle * 180.0 / np.pi:.2f}°")
                
                # # 计算右臂肘部角度
                # if right_shoulder_pos is not None and r_elbow_pos is not None and r_hand_pose is not None:
                #     right_elbow_angle = self.calculate_elbow_angle(right_shoulder_pos, r_elbow_pos, r_hand_pose)
                #     if right_elbow_angle is not None:
                #         # 右臂肘部关节通常是第4个关节（索引3）
                #         elbow_joint_idx = 3
                #         q0_tmp[-self.__single_arm_dof + elbow_joint_idx] = -right_elbow_angle
                #         print(f"右臂肘部角度: {right_elbow_angle * 180.0 / np.pi:.2f}°")
                
                # # 发布q0_tmp
                # q0_tmp_msg = Float32MultiArray()
                # q0_tmp_msg.data = q0_tmp * 180.0 / np.pi  # 转换为角度
                # self.pub_q0_tmp.publish(q0_tmp_msg)
                
                
                ik_solve_start_monotonic = time.perf_counter()
                if self.enable_vr_latency_diagnostics:
                    self.ik_input_prepare_latency_pub.publish(
                        Float64(
                            (ik_solve_start_monotonic - fk_done_monotonic) * 1000.0
                        )
                    )
                q_now = self.arm_ik.computeIK(
                    q0_tmp, l_hand_pose, r_hand_pose, l_hand_RPY, r_hand_RPY, l_elbow_pos, r_elbow_pos, left_shoulder_rpy_in_robot, right_shoulder_rpy_in_robot
                )
                ik_solve_done_monotonic = time.perf_counter()
                time_cost = time.time() - time_0
                if time_cost >= 0.010:
                    rospy.logwarn_throttle(
                        1.0,
                        "IK solve time exceeded 10 ms: %.2f ms",
                        1e3 * time_cost,
                    )
                self.pub_time_cost.publish(Float32(1e3 * time_cost))
                if q_now is not None:
                    msg = Float32MultiArray()
                    msg.data = q_now[-self.__arm_dof:] * 180.0 / np.pi
                    self.pub_origin_joint.publish(msg)
                    
                    arm_q_filtered = self.limit_angle(q_now[-self.__arm_dof:])
                    
                    # 判断手臂往上抬的高度，如果高度超过肩部则限制速度
                    # 检查左右手相对于（肩膀处）的高度差，取较大的那个
                    hand_heights = []
                    shoulder_base_offset = 0.0  # 肩膀下0.0米作为基准点

                    # 计算左手相对于肩部的高度差
                    left_shoulder_pos = self.get_shoulder_position(q0_tmp, "left")
                    shoulder_base_z = left_shoulder_pos[2] - shoulder_base_offset
                    hand_height_relative = l_hand_pose[2] - shoulder_base_z
                    hand_heights.append(hand_height_relative)
                    # 计算右手相对于肩部的高度差
                    right_shoulder_pos = self.get_shoulder_position(q0_tmp, "right")
                    shoulder_base_z = right_shoulder_pos[2] - shoulder_base_offset
                    hand_height_relative = r_hand_pose[2] - shoulder_base_z
                    hand_heights.append(hand_height_relative)
                    
                    # 如果手部高度大于基准点（肩膀下0.2米处），使用较小的速度限制（30 deg/s），否则使用正常速度（120 deg/s）
                    if hand_heights and max(hand_heights) > 0:
                        # 手臂往上抬的高度超过基准点，使用较小的速度限制
                        shoulder_vel_limit = 20.0  # deg/s
                    else:
                        # 正常情况，使用正常速度限制
                        shoulder_vel_limit = 120.0  # deg/s
                    # print(f"shoulder_vel_limit: {shoulder_vel_limit}")
                    # 关节速度限制已移到100 Hz发布线程，使限速步长仍严格
                    # 对应controller_dt，不受VR输入频率影响。
                    q_last[:self.__single_arm_dof] = q_now[:self.__single_arm_dof]
                    q_last[-self.__arm_dof:] = arm_q_filtered

                    solution_ready_time = time.time()
                    solution_ready_monotonic = time.perf_counter()
                    solution = IkSolutionFrame(
                        target_generation=last_target_generation,
                        processed_bone=processed_bone,
                        desired_arm_q=np.asarray(arm_q_filtered).copy(),
                        shoulder_velocity_limit=float(shoulder_vel_limit),
                        ready_time=solution_ready_time,
                        ready_monotonic=solution_ready_monotonic,
                    )
                    # A mode reset or a newer target may arrive while the
                    # solver is running. Never publish the superseded result.
                    with self._ik_condition:
                        solution_is_current = (
                            not self.trigger_reset_mode
                            and self._ik_target_generation
                            == last_target_generation
                        )
                        if solution_is_current:
                            with self._ik_solution_lock:
                                self._latest_ik_solution = solution
                        else:
                            self._ik_stale_solution_drop_count += 1
                            if self.enable_vr_latency_diagnostics:
                                self.ik_stale_solution_drop_pub.publish(
                                    UInt64(self._ik_stale_solution_drop_count)
                                )
                            solution = None
                else:
                    fail_count += 1
                    # print(f"""\nq_last:{q_last}\n l_hand_pose:{l_hand_pose}\n 
                    #       r_hand_pose:{r_hand_pose}\n 
                    #       l_hand_RPY:{l_hand_RPY}\n 
                    #       r_hand_RPY:{r_hand_RPY}\n 
                    #       l_elbow_pos:{l_elbow_pos}\n 
                    #       r_elbow_pos:{r_elbow_pos}\n 
                    #       left_shoulder_rpy_in_robot:{left_shoulder_rpy_in_robot}\n 
                    #       right_shoulder_rpy_in_robot:{right_shoulder_rpy_in_robot}\n""")

                # Same-frame measurements: this event-driven IK round uses one
                # immutable target and one immutable solution snapshot.
                if (self.enable_vr_latency_diagnostics
                        and solution is not None):
                    ik_done_time = solution.ready_time
                    ik_done_monotonic = solution.ready_monotonic
                    if (ik_solve_start_monotonic is not None
                            and ik_solve_done_monotonic is not None):
                        self.ik_solve_latency_pub.publish(
                            Float64(
                                (ik_solve_done_monotonic
                                 - ik_solve_start_monotonic) * 1000.0
                            )
                        )
                        self.ik_postprocess_latency_pub.publish(
                            Float64(
                                (ik_done_monotonic
                                 - ik_solve_done_monotonic) * 1000.0
                            )
                        )
                    if target_ready_monotonic > 0.0:
                        self.ik_solution_ready_latency_pub.publish(
                            Float64(
                                (ik_done_monotonic - target_ready_monotonic)
                                * 1000.0
                            )
                        )

                    if processed_bone is not None:
                        ik_wait_ms, _, _ = calculate_ik_stage_latencies_ms(
                            processed_bone,
                            ik_solve_start_monotonic,
                            ik_solve_done_monotonic,
                            ik_done_monotonic,
                        )
                        self.ik_wait_latency_pub.publish(Float64(ik_wait_ms))
                        arm_traj_latency_ms = (
                            ik_done_time - processed_bone.receive_time
                        ) * 1000.0
                        self.arm_traj_latency_pub.publish(
                            Float64(arm_traj_latency_ms)
                        )
                        if processed_bone.source_timestamp_ms > 0:
                            end_to_end_latency_ms = (
                                ik_done_time
                                - processed_bone.source_timestamp_ms / 1000.0
                            ) * 1000.0
                            self.end_to_end_latency_pub.publish(
                                Float64(end_to_end_latency_ms)
                            )

                run_count += 1
                success_rate = 100 * (1.0 - fail_count / float(run_count))
                sum_time_cost += time_cost
                if run_count % 10 == 0:
                    sys.stdout.write(
                        "\rStatus1: {}, IK success rate: {:.1f}%, avg time-cost: {:.1f} ms, is target far?: {}".format(
                            "RUNING" if is_runing else "STOPED", success_rate, 1e3 * sum_time_cost/run_count, self.judge_target_is_far()
                        )
                    )

            if self.__publish_err and q_now is not None:
                msg_pose_err = ikSolveError()
                msg_pose_err.ik_type = self.arm_ik.type().name()
                pose_left = self.arm_ik.left_hand_pose(q_now)
                pose_right = self.arm_ik.right_hand_pose(q_now)
                if target_pose[0] is not None:
                    pose_left_des = (target_pose[0], quaternion_to_RPY(target_pose[1]))
                    msg_pose_err.left_pose_error = self.generate_ik_solve_error_msg(pose_res=pose_left, pose_des=pose_left_des)
                if target_pose_right[0] is not None:
                    pose_right_des = (target_pose_right[0], quaternion_to_RPY(target_pose_right[1]))
                    msg_pose_err.right_pose_error = self.generate_ik_solve_error_msg(pose_res=pose_right, pose_des=pose_right_des)
                self.pub_ik_solve_error.publish(msg_pose_err)

    def publish_joint_states(self, q_now, q_last):
        arm_agl_limited = self.limit_angle(q_now[-self.__arm_dof:])
        msg = JointState()
        msg.name = ["arm_joint_" + str(i) for i in range(1, self.__arm_dof+1)]
        msg.header.stamp = rospy.Time.now()
        
        if self.only_half_up_body and self.optimized_state is None and self.sensor_data_raw is None:
            print(f"[ik_ros_uni]: optimized_state is None")
            return False

        if self.only_half_up_body and self.arm_mode_changing:
            # 获取当前关节角度（从MPC优化后的状态中提取手臂部分，索引24:38）
            arm_current_state = None
            if self.optimized_state is not None:
                arm_current_state = np.array(self.optimized_state[24:38]).copy()
            else:
                arm_current_state = np.array(self.sensor_data_raw.joint_data.joint_q[-16:-2]).copy()
            
            # 计算状态差
            delta_state = np.array(arm_agl_limited) - np.array(arm_current_state)
            total_distance = np.linalg.norm(delta_state)
            
            # 如果距离太小，直接使用目标状态
            if total_distance < self.threshold_arm_diff_half_up_body:
                arm_agl_interpolated = arm_agl_limited
                self.arm_mode_changing = False
            else:
                max_move = self.maxSpeed
            
                scale = np.clip(max_move / total_distance, 0, 1)
                arm_agl_interpolated = arm_current_state + delta_state * scale
            
            msg.position = 180.0 / np.pi * np.array(arm_agl_interpolated)
        else:
            # 非插值模式下直接使用目标状态
            msg.position = 180.0 / np.pi * np.array(arm_agl_limited)
        
        # 只有在没有hold_arm_timer激活时才发布（避免与保持位置定时器冲突）
        if self.hold_arm_timer is None:
            self.pub.publish(msg)
            return True
        return False

    def kuavo_joint_states_callback(self, joint_states_msg):
        # 手臂状态正解
        self.__joint_states = np.array(joint_states_msg.q)
        q_drake = np.zeros(7+self.__arm_dof)
        q_drake[0] = 1.0
        q_drake[7:] = self.__joint_states
        left_hand_pose = self.arm_ik.left_hand_pose(q_drake)
        right_hand_pose = self.arm_ik.right_hand_pose(q_drake)
        arm_hand_pose_msg = twoArmHandPose()
        arm_hand_pose_msg.header.frame_id = "torso"
        arm_hand_pose_msg.header.stamp = rospy.Time.now()
        arm_hand_pose_msg.left_pose.pos_xyz = left_hand_pose[0]
        r, p, y = left_hand_pose[1]
        arm_hand_pose_msg.left_pose.quat_xyzw = rpy_to_quaternion(r, p, y)
        arm_hand_pose_msg.left_pose.joint_angles = self.__joint_states[:self.__single_arm_dof]
        arm_hand_pose_msg.right_pose.pos_xyz = right_hand_pose[0]
        r, p, y = right_hand_pose[1]
        arm_hand_pose_msg.right_pose.quat_xyzw = rpy_to_quaternion(r, p, y)
        arm_hand_pose_msg.right_pose.joint_angles = self.__joint_states[-self.__single_arm_dof:]
        self.pub_real_arm_hand_pose.publish(arm_hand_pose_msg)
        # print(f"received joint_states: {self.__joint_states}")

    def quest_bone_poses_callback(self, quest_bone_poses_msg):
        """Keep the TCPROS receive callback short and always retain the newest frame."""
        recv_time = time.time()
        recv_monotonic = time.perf_counter()
        vr_ts_ms = int(quest_bone_poses_msg.timestamp_ms)

        with self._bone_condition:
            self._bone_input_seq += 1
            frame = BoneInputFrame(
                sequence=self._bone_input_seq,
                message=quest_bone_poses_msg,
                receive_time=recv_time,
                receive_monotonic=recv_monotonic,
                source_timestamp_ms=vr_ts_ms,
            )
            overwritten = self._pending_bone_frame is not None
            if overwritten:
                self._bone_overwrite_count += 1
            self._pending_bone_frame = frame
            self._bone_condition.notify()

        # 测量通信延迟：VR节点发布时刻(timestamp_ms) → IK节点回调接收时刻
        if self.enable_vr_latency_diagnostics and vr_ts_ms > 0:
            comm_latency_ms = (recv_time - vr_ts_ms / 1000.0) * 1000.0
            self.comm_latency_pub.publish(Float64(comm_latency_ms))

    def _publish_transform_stage_diagnostics(self, frame, timings):
        """Publish one self-describing, same-frame transform timing sample."""
        if self.transform_stage_latency_pub is None:
            return
        msg = Float32MultiArrayStamped()
        msg.header.seq = int(frame.sequence) & 0xFFFFFFFF
        msg.header.stamp = rospy.Time.from_sec(frame.receive_time)
        msg.header.frame_id = "vr_absolute_transform_stage_v1"
        dimension = MultiArrayDimension()
        dimension.label = ",".join(TRANSFORM_STAGE_DIAGNOSTIC_FIELDS)
        dimension.size = len(TRANSFORM_STAGE_DIAGNOSTIC_FIELDS)
        dimension.stride = len(TRANSFORM_STAGE_DIAGNOSTIC_FIELDS)
        msg.data.layout.dim = [dimension]
        msg.data.layout.data_offset = 0
        msg.data.data = [
            float(timings.get(name, float("nan")))
            for name in TRANSFORM_STAGE_DIAGNOSTIC_FIELDS
        ]
        self.transform_stage_latency_pub.publish(msg)

    @staticmethod
    def _copy_pose(pose):
        if pose is None or pose[0] is None or pose[1] is None:
            return (None, None)
        return (np.asarray(pose[0]).copy(), np.asarray(pose[1]).copy())

    @staticmethod
    def _copy_optional_array(value):
        return None if value is None else np.asarray(value).copy()

    def _take_latest_bone_frame(self):
        with self._bone_condition:
            self._bone_condition.wait_for(
                lambda: self._pending_bone_frame is not None or self.stop_event.is_set(),
                timeout=0.1,
            )
            if self.stop_event.is_set():
                return None
            frame = self._pending_bone_frame
            self._pending_bone_frame = None
            return frame

    def _get_joystick_snapshot(self):
        with self._joystick_lock:
            return self._joystick_snapshot

    def _queue_latest_finger_input(self, frame, joystick_snapshot):
        with self._finger_condition:
            self._pending_finger_input = (frame, joystick_snapshot)
            self._finger_condition.notify()

    def _take_latest_finger_input(self):
        with self._finger_condition:
            self._finger_condition.wait_for(
                lambda: self._pending_finger_input is not None
                or self.stop_event.is_set(),
                timeout=0.1,
            )
            if self.stop_event.is_set():
                return None
            value = self._pending_finger_input
            self._pending_finger_input = None
            return value

    def _finger_processing_loop(self):
        """Process only the newest finger frame at a bounded lower rate."""
        min_period = 1.0 / self.finger_processing_hz
        last_started = 0.0
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            pending = self._take_latest_finger_input()
            if pending is None:
                continue

            remaining = min_period - (time.monotonic() - last_started)
            if remaining > 0.0:
                with self._finger_condition:
                    self._finger_condition.wait(timeout=remaining)
                    if self.stop_event.is_set():
                        return
                    if self._pending_finger_input is not None:
                        pending = self._pending_finger_input
                        self._pending_finger_input = None

            frame, joystick_snapshot = pending
            process_start = time.perf_counter()
            try:
                result = self.quest3_arm_info_transformer.process_finger_frame(
                    frame.message, joystick_snapshot
                )
                if result is None:
                    continue
                with self._target_lock:
                    self.hand_finger_data = [
                        result["left_finger_joints"],
                        result["right_finger_joints"],
                    ]
                    self._vr_is_running = result["is_running"]
                if self.enable_vr_latency_diagnostics:
                    self.finger_processing_latency_pub.publish(
                        Float64((time.perf_counter() - process_start) * 1000.0)
                    )
            except Exception as exc:
                rospy.logerr_throttle(
                    1.0, "Failed to process latest VR finger frame: %s", exc
                )
            finally:
                last_started = time.monotonic()

    def _bone_processing_loop(self):
        """Transform only the newest pending bone frame and publish an atomic snapshot."""
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            frame = self._take_latest_bone_frame()
            if frame is None:
                continue
            worker_start = time.perf_counter()

            with self._bone_condition:
                overwrite_count = self._bone_overwrite_count
            if (self.enable_vr_latency_diagnostics
                    and overwrite_count
                    != self._last_reported_bone_overwrite_count):
                self._last_reported_bone_overwrite_count = overwrite_count
                self.bone_overwrite_count_pub.publish(UInt64(overwrite_count))

            processing_start = time.perf_counter()
            try:
                joystick_snapshot = self._get_joystick_snapshot()
                transformer_lock_request = time.perf_counter()
                # The transformer now has a single arm writer. Joystick and
                # finger workers exchange immutable snapshots, so no shared
                # Transformer lock is required on this critical path.
                transformer_lock_acquired = transformer_lock_request
                read_msg_timings = self.quest3_arm_info_transformer.read_msg(
                    frame.message,
                    collect_timing=self.transform_stage_latency_pub is not None,
                    joystick_snapshot=joystick_snapshot,
                    defer_finger_processing=True,
                )
                if read_msg_timings is False:
                    continue
                read_msg_done = time.perf_counter()
                left_pose, left_elbow_pos = (
                    self.quest3_arm_info_transformer.get_hand_pose("Left")
                )
                right_pose, right_elbow_pos = (
                    self.quest3_arm_info_transformer.get_hand_pose("Right")
                )
                finger_state = self.quest3_arm_info_transformer.get_finger_state()
                transform_done_time = time.time()
                transform_done_monotonic = time.perf_counter()
                snapshot = ProcessedBoneFrame(
                    sequence=frame.sequence,
                    receive_time=frame.receive_time,
                    source_timestamp_ms=frame.source_timestamp_ms,
                    transform_done_time=transform_done_time,
                    transform_done_monotonic=transform_done_monotonic,
                    left_pose=self._copy_pose(left_pose),
                    right_pose=self._copy_pose(right_pose),
                    left_elbow_pos=self._copy_optional_array(left_elbow_pos),
                    right_elbow_pos=self._copy_optional_array(right_elbow_pos),
                    left_finger_joints=finger_state["left_finger_joints"],
                    right_finger_joints=finger_state["right_finger_joints"],
                    left_shoulder_rpy=np.asarray(
                        self.quest3_arm_info_transformer.left_shoulder_rpy_in_robot
                    ).copy(),
                    right_shoulder_rpy=np.asarray(
                        self.quest3_arm_info_transformer.right_shoulder_rpy_in_robot
                    ).copy(),
                    is_running=finger_state["is_running"],
                    is_hand_tracking=bool(
                        self.quest3_arm_info_transformer.is_hand_tracking
                    ),
                    vr_error=bool(
                        self.quest3_arm_info_transformer.check_if_vr_error()
                    ),
                )
                snapshot_done = time.perf_counter()
                self._queue_latest_finger_input(frame, joystick_snapshot)

                target_lock_request = time.perf_counter()
                with self._ik_condition:
                    target_lock_acquired = time.perf_counter()
                    self._latest_processed_bone = snapshot
                    self.__target_pose = snapshot.left_pose
                    self.__target_pose_right = snapshot.right_pose
                    self.__left_elbow_pos = snapshot.left_elbow_pos
                    self.__right_elbow_pos = snapshot.right_elbow_pos
                    # Finger data and gesture state have a single writer: the
                    # asynchronous finger worker. Do not overwrite a newer
                    # result with the snapshot observed by this arm frame.
                    self._vr_is_hand_tracking = snapshot.is_hand_tracking
                    self._vr_error = snapshot.vr_error
                    if snapshot.left_pose[0] is not None and snapshot.right_pose[0] is not None:
                        self.__recieved_new_target_pose = True
                    target_ready_time = time.time()
                    target_ready_monotonic = time.perf_counter()
                    self._ik_target_generation += 1
                    self._ik_target_ready_time = target_ready_time
                    self._ik_target_ready_monotonic = target_ready_monotonic
                    self._ik_condition.notify()
                target_update_done = time.perf_counter()

                if self.enable_vr_latency_diagnostics:
                    processing_ms = (
                        target_update_done - processing_start
                    ) * 1000.0
                    pipeline_ms = (
                        snapshot.transform_done_time - frame.receive_time
                    ) * 1000.0
                    self.transform_processing_latency_pub.publish(
                        Float64(processing_ms)
                    )
                    self.transform_pipeline_latency_pub.publish(
                        Float64(pipeline_ms)
                    )
                    self.ik_target_commit_latency_pub.publish(
                        Float64(
                            max(
                                0.0,
                                (target_ready_monotonic
                                 - snapshot.transform_done_monotonic) * 1000.0,
                            )
                        )
                    )
                self._publish_wheel_target_from_snapshot(snapshot)
                if self.transform_stage_latency_pub is not None:
                    read_msg_timings = read_msg_timings or {}
                    stage_timings = {
                        "callback_to_worker_ms": (
                            worker_start - frame.receive_monotonic
                        ) * 1000.0,
                        "worker_housekeeping_ms": (
                            transformer_lock_request - worker_start
                        ) * 1000.0,
                        "transformer_lock_wait_ms": (
                            transformer_lock_acquired - transformer_lock_request
                        ) * 1000.0,
                        "snapshot_build_ms": (
                            snapshot_done - read_msg_done
                        ) * 1000.0,
                        "target_lock_wait_ms": (
                            target_lock_acquired - target_lock_request
                        ) * 1000.0,
                        "target_update_ms": (
                            target_update_done - target_lock_acquired
                        ) * 1000.0,
                        "worker_total_ms": (
                            target_update_done - worker_start
                        ) * 1000.0,
                        "callback_to_transform_done_ms": (
                            transform_done_monotonic - frame.receive_monotonic
                        ) * 1000.0,
                    }
                    stage_timings.update(read_msg_timings)
                    self._publish_transform_stage_diagnostics(frame, stage_timings)
            except Exception as exc:
                rospy.logerr_throttle(1.0, "Failed to process latest VR bone frame: %s", exc)

    def _publish_wheel_target_from_snapshot(self, snapshot):
        # 轮臂需 mpc_observation.time > 阈值，避免 #3009。
        if (self.robot_type == 1 and snapshot.is_running
                and self._wheel_mpc_stable_for_mm_cmd()
                and snapshot.left_pose[0] is not None
                and snapshot.right_pose[0] is not None):
            eef_pose_msg = twoArmHandPoseCmd()
            eef_pose_msg.frame = 3
            eef_pose_msg.hand_poses.left_pose.pos_xyz = snapshot.left_pose[0]
            eef_pose_msg.hand_poses.left_pose.quat_xyzw = snapshot.left_pose[1]
            eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = (
                snapshot.left_elbow_pos if snapshot.left_elbow_pos is not None else [0.0, 0.0, 0.0]
            )
            eef_pose_msg.hand_poses.right_pose.pos_xyz = snapshot.right_pose[0]
            eef_pose_msg.hand_poses.right_pose.quat_xyzw = snapshot.right_pose[1]
            eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = (
                snapshot.right_elbow_pos if snapshot.right_elbow_pos is not None else [0.0, 0.0, 0.0]
            )
            eef_pose_msg.timestamp_ms = int(snapshot.receive_time * 1000)
            self.pub_mm_two_arm_hand_pose_cmd.publish(eef_pose_msg)

    def two_arm_hand_pose_target_callback(self, msg_ori):
        msg = msg_ori.hand_poses
        if msg_ori.use_custom_ik_param:
            self.external_q0 = list(msg.left_pose.joint_angles) + list(msg.right_pose.joint_angles)
        target_pose = (np.asarray(msg.left_pose.pos_xyz), np.asarray(msg.left_pose.quat_xyzw))
        target_pose_right = (np.asarray(msg.right_pose.pos_xyz), np.asarray(msg.right_pose.quat_xyzw))
        if(abs(msg.left_pose.elbow_pos_xyz[0]) <= 1e-5 
            and abs(msg.left_pose.elbow_pos_xyz[1]) <= 1e-5 
            and abs(msg.left_pose.elbow_pos_xyz[2]) <= 1e-5):  # 都为0，则不控制elbow
            left_elbow_pos = None
        else:
            left_elbow_pos = np.array(msg.left_pose.elbow_pos_xyz)
        if(abs(msg.right_pose.elbow_pos_xyz[0]) <= 1e-5 
            and abs(msg.right_pose.elbow_pos_xyz[1]) <= 1e-5 
            and abs(msg.right_pose.elbow_pos_xyz[2]) <= 1e-5):
            right_elbow_pos = None
        else:
            right_elbow_pos = np.array(msg.right_pose.elbow_pos_xyz)

        if self.__as_mc_ik:
            self.__as_mc_ik = False
            self.arm_ik.set_as_mc_ik(self.__as_mc_ik)
        with self._ik_condition:
            self.__target_pose = target_pose
            self.__target_pose_right = target_pose_right
            self.__left_elbow_pos = left_elbow_pos
            self.__right_elbow_pos = right_elbow_pos
            self._latest_processed_bone = None
            self.__recieved_new_target_pose = True
            self._ik_target_generation += 1
            self._ik_target_ready_time = time.time()
            self._ik_target_ready_monotonic = time.perf_counter()
            self._ik_condition.notify()

    def joySticks_data_callback(self, msg):
        joystick_snapshot = (
            (float(msg.left_trigger), float(msg.left_grip)),
            (float(msg.right_trigger), float(msg.right_grip)),
        )
        with self._joystick_lock:
            self._joystick_snapshot = joystick_snapshot
        with self._target_lock:
            is_hand_tracking = self._vr_is_hand_tracking
        self.joySticks_data = msg
        # 准备姿态下 IK 线程可能卡在等待首帧姿态/MPC 就绪，此时由回调补发夹爪指令。
        # 遥操主循环启动后改由 hand_finger_data_process 单路发布，避免双路重复下发。
        # qa: https://www.lejuhub.com/highlydynamic/kuavodevlab/-/issues/3505
        if (self.end_effector_type == LEJUCLAW
                and not is_hand_tracking
                and not self.__ik_claw_publish_active):
            self.pub_robot_end_hand(joyStick_data=msg)

    @staticmethod
    def vector3_to_milliseconds(x, y, z):
        # Reconstruct the bytes from the Vector3 components
        # x, y, z are assumed to be float but should contain integer values for this purpose
        byte0 = int(x) & 0xFF
        byte1 = int(y) >> 8 & 0xFF
        byte2 = int(y) & 0xFF
        byte3 = int(z) & 0xFF

        # Reverse the order of the bytes
        reversed_bytes = [byte3, byte2, byte1, byte0]

        # Pack the reversed bytes back into a single integer
        # '<' denotes little-endian byte order, 'I' denotes an unsigned int
        milliseconds_int = struct.unpack("<I", bytes(reversed_bytes))[0]
        return milliseconds_int
    
    def get_shoulder_position(self, q, side="left"):
        """
        获取肩膀位置
        Args:
            q: 关节角度
            side: "left" 或 "right"
        Returns:
            np.array: 肩膀位置 [x, y, z]
        """
        try:
            self.arm_ik._ArmIk__plant.SetPositions(self.arm_ik._ArmIk__plant_context, q)
            if side.lower() == "left":
                shoulder_frame = self.arm_ik._ArmIk__plant.GetFrameByName(self.arm_ik.shoulder_frame_names[0])
            else:
                shoulder_frame = self.arm_ik._ArmIk__plant.GetFrameByName(self.arm_ik.shoulder_frame_names[1])
            
            shoulder_pose = shoulder_frame.CalcPoseInWorld(self.arm_ik._ArmIk__plant_context)
            return shoulder_pose.translation()
        except Exception as e:
            print(f"获取肩膀位置失败: {e}")
            return None

    def calculate_elbow_angle(self, shoulder_pos, elbow_pos, hand_pos):
        """
        计算肘部关节的夹角
        Args:
            shoulder_pos: 肩膀位置 [x, y, z]
            elbow_pos: 肘部位置 [x, y, z]
            hand_pos: 手部位置 [x, y, z]
        Returns:
            float: 肘部关节角度（弧度）
        """
        if shoulder_pos is None or elbow_pos is None or hand_pos is None:
            return None
            
        # 计算向量
        vec_shoulder_to_elbow = np.array(elbow_pos) - np.array(shoulder_pos)
        vec_elbow_to_hand = np.array(hand_pos) - np.array(elbow_pos)
        
        # 计算向量长度
        len_shoulder_elbow = np.linalg.norm(vec_shoulder_to_elbow)
        len_elbow_hand = np.linalg.norm(vec_elbow_to_hand)
        
        # 避免除零错误
        if len_shoulder_elbow < 1e-6 or len_elbow_hand < 1e-6:
            return None
            
        # 计算夹角（弧度）
        cos_angle = np.dot(vec_shoulder_to_elbow, vec_elbow_to_hand) / (len_shoulder_elbow * len_elbow_hand)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)  # 限制在[-1, 1]范围内
        angle = np.arccos(cos_angle)
        
        return angle

    def generate_ik_solve_error_msg(self, pose_res, pose_des):
        rad2deg = 180.0/np.pi      
        hand_pose_err = handPose()
        pos_res, rpy_res = pose_res
        pos_des, rpy_des = pose_des
        hand_pose_err.x = pos_res[0] - pos_des[0]
        hand_pose_err.y = pos_res[1] - pos_des[1]
        hand_pose_err.z = pos_res[2] - pos_des[2]
        rpy_err = compute_rpy_error(rpy_res, rpy_des)
        hand_pose_err.roll = rad2deg * rpy_err[0]
        hand_pose_err.pitch = rad2deg * rpy_err[1]
        hand_pose_err.yaw = rad2deg * rpy_err[2]
        return hand_pose_err


    def control_to_pos(self, diff_ik: DiffIK, q_now, traj, ctrl_arm_idx=ArmIdx.LEFT, dt=0.01, pub_joint=None):
        """
        ctrl_arm_idx: Note: ONLY support ArmIdx.LEFT or ArmIdx.RIGHT
        """
        if ctrl_arm_idx.name() == ArmIdx.BOTH.name():
            print(f"\033[91mControl both arms is not supported.\033[0m")
            return None
        t_duration = traj.get_segment_times()[1] - traj.get_segment_times()[0]
        t_sim = traj.get_segment_times()[0]
        q0 = q_now
        last_q = q_now
        # diff_ik.start_recording()
        traj_V_G = traj.MakeDerivative() if traj is not None else None

        t_play = 0.0
        last_norm = 100.0
        last_v = np.zeros(self.__single_arm_dof)
        while t_sim < t_duration:
            pose = arm_ik.left_hand_pose(last_q) if ctrl_arm_idx.name() == ArmIdx.LEFT.name() else arm_ik.right_hand_pose(last_q)
            with self._target_lock:
                target_pose = (
                    self.__target_pose
                    if ctrl_arm_idx.name() == ArmIdx.LEFT.name()
                    else self.__target_pose_right
                )
            pos_target, quat_target = target_pose

            is_close, norm = self.check_if_close(pose, (pos_target, quaternion_to_RPY(quat_target)))
            if is_close:
                break

            t_sim += dt
            time_0 = time.time()
            V_G_vec = np.array(traj_V_G.value(t_sim))[:, 0] if traj is not None else None

            # 仅控制单臂
            v_max = 1.0
            vd_max = 5.0
            v0 = np.zeros(self.__single_arm_dof)
            if ctrl_arm_idx.name() == ArmIdx.LEFT.name():
                v0 = diff_ik.solve_left_hand(last_q, last_v, V_G_vec, dt, v_max, vd_max)
                q0[self.__single_arm_dof:self.__arm_dof] += dt*v0
            if ctrl_arm_idx.name() == ArmIdx.RIGHT.name():
                v0 = diff_ik.solve_right_hand(last_q, last_v, V_G_vec, dt, v_max, vd_max)
                q0[-self.__single_arm_dof:] += dt*v0
            time_cost = time.time() - time_0
            # animate trajectory
            diff_ik.visualize_animation([last_q, q0], t_play, dt)
            last_q = q0
            last_v = v0
            t_play = t_play + dt
            # if self.pub is not None:
            #     self.publish_joint_states(q_now=q_now, q_last=None)
            if last_norm < norm:
                break
            last_norm = norm
        return last_q

    def check_if_close(self, pose_res, pose_des, threshold_pos=0.002, threshold_theta=3.0*np.pi/180.0):
        """
        check if the pose_res is close to pose_des
        位置误差: 默认0.002m
        姿态误差: 默认3.0deg
        Returns:
            bool: True if the pose_res is close to pose_des, False otherwise
            float: the norm of the position error
        """
        norm = np.linalg.norm(pose_res[0] - pose_des[0])
        # print(f"pose_res: {pose_res}")
        # print(f"pose_res: {pose_des}")
        mat_res, mat_des = rpy_to_matrix(pose_res[1]), rpy_to_matrix(pose_des[1])

        delta_theta, _ = rotation_matrix_diff_in_angle_axis(mat_res, mat_des)
        if norm > threshold_pos:
            return False, norm
        if delta_theta > threshold_theta:
            return False, norm
        return True, norm

    def judge_target_is_far(self, threshold=0.4):
        """
        If target is far, return True, else return False.
        """
        with self._target_lock:
            target_pose = self.__target_pose
            target_pose_right = self.__target_pose_right
        if target_pose[0] is None or target_pose_right[0] is None:
            return False
        if self.__current_pose[0] is None or self.__current_pose_right[0] is None:
            return False
        pos_left, _ = self.__current_pose
        pos_right, _ = self.__current_pose_right
        pos_target_left, _ = target_pose
        pos_target_right, _ = target_pose_right
        dist_left = np.linalg.norm(pos_left - pos_target_left)
        dist_right = np.linalg.norm(pos_right - pos_target_right)
        if dist_left > threshold and self.__ctrl_arm_idx.name() != ArmIdx.RIGHT.name():
            # print(f"\033[91mLeft arm is far from target, distance: {dist_left:.3f} m.\033[0m")
            return True
        if dist_right > threshold and self.__ctrl_arm_idx.name() != ArmIdx.LEFT.name():
            # print(f"\033[91mRight arm is far from target, distance: {dist_right:.3f} m.\033[0m")
            return True
        return False

    @staticmethod
    def isJoyPushed(joySticks_data):
        pushed = False
        pushed |= (joySticks_data.left_trigger > 0.0)
        pushed |= (joySticks_data.right_trigger > 0.0)
        pushed |= (joySticks_data.left_grip > 0.0)
        pushed |= (joySticks_data.right_grip > 0.0)
        pushed |= joySticks_data.left_first_button_touched
        pushed |= joySticks_data.right_first_button_touched
        return pushed

    def hand_finger_data_process(self, event):
        # if(self.isJoyPushed(self.joySticks_data)):
        with self._target_lock:
            is_hand_tracking = self._vr_is_hand_tracking
            hand_finger_data = self.hand_finger_data
        if not is_hand_tracking:
            # print(f"\033[91mJoystick is pushed, stop control.\033[0m")
            self.pub_robot_end_hand(joyStick_data=self.joySticks_data)            
        else:
            # print(f"\033[91mJoystick is not pushed, continue control.\033[0m")
            self.pub_robot_end_hand(hand_finger_data=hand_finger_data)


    def pub_robot_end_hand(self, joyStick_data=None, hand_finger_data = None):
        # hand tracking 时判断保护
        if hand_finger_data is not None and len(hand_finger_data) < 2:
            return
        global control_finger_type
        left_hand_position = [0 for i in range(6)]
        right_hand_position = [0 for i in range(6)]
        robot_hand_position = robotHandPosition()
        robot_hand_position.header.stamp = rospy.Time.now()
        if self.end_effector_type == QIANGNAO or self.end_effector_type == QIANGNAO_TOUCH or self.end_effector_type == REVO2 or self.end_effector_type == LINKER_HAND:
            if joyStick_data is not None:
                if joyStick_data.left_second_button_pressed and self.__button_y_last is False:
                    print(f"\033[91mButton Y is pressed.\033[0m")
                    self.__freeze_finger = not self.__freeze_finger
                self.__button_y_last = joyStick_data.left_second_button_pressed
                if self.__freeze_finger is True:
                    # print(f"\033[91mFinger is frozen.\033[0m")
                    # Use frozen values
                    left_hand_position = self.__frozen_left_hand_position.copy()
                    right_hand_position = self.__frozen_right_hand_position.copy()
                else:
                    # Calculate new values and store them for potential freezing
                    for i in range(6):
                        idx = 6 if (control_finger_type == 0) else 2
                        if i <= idx:
                            left_hand_position[i] = int(100.0 * joyStick_data.left_trigger)
                            right_hand_position[i] = int(100.0 * joyStick_data.right_trigger)
                        else:
                            left_hand_position[i] = int(100.0 * joyStick_data.left_grip)
                            right_hand_position[i] = int(100.0 * joyStick_data.right_grip)
                        left_hand_position[i] = limit_value(left_hand_position[i], 0, 100)
                        right_hand_position[i] = limit_value(right_hand_position[i], 0, 100)
                    left_hand_position[1] = 100 if joyStick_data.left_first_button_touched else 0
                    right_hand_position[1] = 100 if joyStick_data.right_first_button_touched else 0

                    if joyStick_data.left_first_button_touched and joyStick_data.right_first_button_pressed:
                        for i in range(0, 6):
                            left_hand_position[i] = 100 
                        left_hand_position[2] = 0
                    if joyStick_data.left_first_button_touched and joyStick_data.right_second_button_pressed:
                        for i in range(0, 6):
                            right_hand_position[i] = 100 
                        right_hand_position[2] = 0
                    
                    if self.end_effector_type == LINKER_HAND and self.__robot_walking_status:
                        left_hand_position[0] = left_hand_position[0] if joyStick_data.left_first_button_touched else 100
                        right_hand_position[0] = right_hand_position[0] if joyStick_data.right_first_button_touched else 100

                    # Store current values for freezing
                    self.__frozen_left_hand_position = left_hand_position.copy()
                    self.__frozen_right_hand_position = right_hand_position.copy()
                # print(f"left_hand_position[1]: {left_hand_position[1]}, right_hand_position[1]: {right_hand_position[1]}\n")
            elif hand_finger_data is not None:
                if self.__freeze_finger is True:
                    # Use frozen values
                    left_hand_position = self.__frozen_left_hand_position.copy()
                    right_hand_position = self.__frozen_right_hand_position.copy()
                else:
                    # Calculate new values and store them for potential freezing
                    left_qpos = hand_finger_data[0]
                    right_qpos = hand_finger_data[1]
                    for i in range(6):
                        left_hand_position[i] = int(100.0 * left_qpos[i]/1.70)
                        right_hand_position[i] = int(100.0 * right_qpos[i]/1.70)
                        left_hand_position[i] = limit_value(left_hand_position[i], 0, 100)
                        right_hand_position[i] = limit_value(right_hand_position[i], 0, 100)
                    # Store current values for freezing
                    self.__frozen_left_hand_position = left_hand_position.copy()
                    self.__frozen_right_hand_position = right_hand_position.copy()
            
            robot_hand_position.left_hand_position = left_hand_position
            robot_hand_position.right_hand_position = right_hand_position
            self.control_robot_hand_position_pub.publish(robot_hand_position)
        elif self.end_effector_type == LEJUCLAW:
            if joyStick_data is not None:
                if joyStick_data.left_second_button_pressed and self.__button_y_last is False:
                    print(f"\033[91mButton Y is pressed.\033[0m")
                    self.__freeze_finger = not self.__freeze_finger
                self.__button_y_last = joyStick_data.left_second_button_pressed
                if self.__freeze_finger is True:
                    # Use frozen values
                    self.pub_leju_claw_command(self.__frozen_claw_pos)
                else:
                    # Calculate new values and store them for potential freezing
                    pos = [0.0] * 2
                    pos[0] = int(100.0 * joyStick_data.left_trigger)
                    pos[1] = int(100.0 * joyStick_data.right_trigger)
                    pos[0] = limit_value(pos[0], 0, 100)
                    pos[1] = limit_value(pos[1], 0, 100)
                    # Store current values for freezing
                    self.__frozen_claw_pos = pos.copy()
                    self.pub_leju_claw_command(pos)
            elif hand_finger_data is not None:
                if self.__freeze_finger is True:
                    # Use frozen values
                    self.pub_leju_claw_command(self.__frozen_claw_pos)
                else:
                    # Calculate new values and store them for potential freezing
                    left_qpos = hand_finger_data[0]
                    right_qpos = hand_finger_data[1]
                    left_claw_pos = limit_value(int(100.0 * left_qpos[2] / 1.70), 0, 100)
                    right_claw_pos = limit_value(int(100.0 * right_qpos[2] / 1.70), 0, 100)
                    pos = [left_claw_pos, right_claw_pos]
                    # Store current values for freezing
                    self.__frozen_claw_pos = pos.copy()
                    self.pub_leju_claw_command(pos)
                    # print(f"left_claw_pos: {left_claw_pos}, right_claw_pos: {right_claw_pos}")
            else:
                return

    # 添加手臂模式回调函数
    def arm_mode_callback(self, msg):
        new_mode = msg.data
        if new_mode == 0:  # 当模式不是2时
            # 重置所有姿态
            print(f"\033[91m[IK]Reset arm mode.\033[0m")
            self.trigger_reset_mode = True
            with self._ik_condition:
                self._ik_target_generation += 1
                self._ik_target_ready_time = time.time()
                self._ik_target_ready_monotonic = time.perf_counter()
                self._ik_condition.notify()
            self._clear_latest_ik_solution(clear_published_command=True)
            self.arm_mode_changing = False
            self.collision_check_control = False
            
            # 半身模式下，保存当前手臂状态并启动定时器持续发布
            if self.only_half_up_body and self.optimized_state is not None:
                self.frozen_arm_state = np.array(self.optimized_state[24:38]).copy()
                # 停止旧定时器（如果存在）
                if self.hold_arm_timer is not None:
                    self.hold_arm_timer.shutdown()
                # 启动新定时器，以50Hz频率发布保持位置
                self.hold_arm_timer = rospy.Timer(rospy.Duration(0.02), self.hold_arm_position_callback)
                print(f"\033[93m[IK]Half body mode: Started holding arm position.\033[0m")
        elif new_mode == 1:
            self.arm_mode_changing = True
            # 重置所有姿态
            print(f"\033[91m[IK]Reset arm mode.\033[0m")
            self.trigger_reset_mode = True
            with self._ik_condition:
                self._ik_target_generation += 1
                self._ik_target_ready_time = time.time()
                self._ik_target_ready_monotonic = time.perf_counter()
                self._ik_condition.notify()
            self._clear_latest_ik_solution(clear_published_command=True)
            self.collision_check_control = False
            
            # 半身模式下，保存当前手臂状态并启动定时器持续发布
            if self.only_half_up_body and self.optimized_state is not None:
                # self.frozen_arm_state = np.array(self.optimized_state[24:38]).copy()
                self.frozen_arm_state = np.zeros(14)
                # 停止旧定时器（如果存在）
                if self.hold_arm_timer is not None:
                    self.hold_arm_timer.shutdown()
                # 启动新定时器，以50Hz频率发布保持位置
                self.hold_arm_timer = rospy.Timer(rospy.Duration(0.02), self.hold_arm_position_callback)
                print(f"\033[93m[IK]Half body mode: Started holding arm position.\033[0m")


        elif new_mode == 2:
            print(f"\033[91m[IK]Arm mode changing.\033[0m")
            self.arm_mode_changing = True
            
            # 进入mode2时停止保持位置的定时器
            if self.only_half_up_body and self.hold_arm_timer is not None:
                self.hold_arm_timer.shutdown()
                self.hold_arm_timer = None
                self.frozen_arm_state = None
                print(f"\033[93m[IK]Half body mode: Stopped holding arm position.\033[0m")
    
    def arm_control_mode_callback(self, msg):
        """监听手臂控制模式变化，检测切换模式时重置IK初始猜测"""
        if len(msg.data) >= 2:
            current_mode = int(msg.data[0])  # 当前模式
            new_mode = int(msg.data[1])      # 新模式
            self.__arm_control_mode = current_mode
            

            # 检测模式切换：当data[0] != data[1]时表示正在切换，重置IK初始猜测。且只在模式切换刚开始时重置IK初始猜测。
            if current_mode != new_mode and self.__first_change_arm_mode:
                self.__first_change_arm_mode = False
                self.__need_reset_ik_guess = True
                self.arm_mode_changing = True
            elif current_mode == new_mode and not self.__first_change_arm_mode:
                # 模式切换完成，关闭arm_mode_changing标志
                if not self.only_half_up_body:
                    self.arm_mode_changing = False
                    self.__first_change_arm_mode = True

    def sensor_data_raw_callback(self, msg):
        self.sensor_data_raw = msg
    
    def wheel_mpc_observation_callback(self, msg):
        """轮臂 MPC 观测：time 为 MPC 模块内部时间，从 0 递增"""
        self._wheel_mpc_obs_time = float(msg.time)

    def _wheel_mpc_stable_for_mm_cmd(self):
        """轮臂：仅当 mobile_manipulator_mpc_observation.time 大于阈值时才允许发 mm / 切手臂模式（kuavodevlab#3009）。"""
        if self.robot_type != 1:
            return True
        if self._wheel_mpc_obs_time is None:
            return False
        return self._wheel_mpc_obs_time > self._wheel_mm_mpc_min_internal_time

    def optimized_state_callback(self, msg):
        """接收MPC优化后的状态数据"""
        self.optimized_state = np.array(msg.data)
    
    def hold_arm_position_callback(self, event):
        """定时器回调：持续发布冻结的手臂位置，使用插值平滑过渡"""
        if self.frozen_arm_state is None or self.optimized_state is None:
            return
        
        # 获取当前关节角度（从MPC优化后的状态中提取手臂部分，索引24:38）
        arm_current_state = np.array(self.optimized_state[24:38]).copy()
        
        # 计算状态差
        delta_state = self.frozen_arm_state - arm_current_state
        total_distance = np.linalg.norm(delta_state)
        
        # 如果距离太小，直接使用目标状态
        if total_distance < self.threshold_arm_diff_half_up_body:
            arm_agl_interpolated = self.frozen_arm_state
        else:
            # 使用插值平滑过渡
            max_move = self.maxSpeed
            scale = np.clip(max_move / total_distance, 0, 1)
            arm_agl_interpolated = arm_current_state + delta_state * scale
        
        msg = JointState()
        msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]
        msg.header.stamp = rospy.Time.now()
        msg.position = 180.0 / np.pi * arm_agl_interpolated
        self.pub.publish(msg)

    def stop_robot_callback(self, msg):
        """停止机器人信号回调函数"""
        if msg.data:  # 当收到True信号时退出程序
            rospy.loginfo("[IkRos] 收到停止机器人信号，正在退出程序...")
            self._stop_threads()
            rospy.signal_shutdown("Received stop signal")  # 触发ROS节点关闭

    def robot_walking_status_callback(self, msg):
        self.__robot_walking_status = msg.data

    def set_arm_mode_changing_callback(self, req):
        """服务回调函数，设置arm_mode_changing为True"""

        self.arm_mode_changing = True
        
        if self.only_half_up_body:
            # 发送当前手臂的关节状态到kuavo_arm_traj来清空mpc节点话题接收队列
            # 防止半身手臂切换时刻mpc执行旧的kuavo_arm_tarj
            if self.optimized_state is None and self.sensor_data_raw is None:
                print(f"[ik_ros_uni]: optimized_state and sensor_data_raw are None")
                return
            else:
                rate = rospy.Rate(1 / self.controller_dt)
                arm_current_state = None
                if self.optimized_state is not None:
                    arm_current_state = np.array(self.optimized_state[24:38]).copy()
                else:
                    arm_current_state = np.array(self.sensor_data_raw.joint_data.joint_q[-16:-2]).copy()
                msg = JointState()
                msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]
                msg.header.stamp = rospy.Time.now()
                msg.position = 180.0 / np.pi * np.array(arm_current_state)
                for i in range(5):  # 减少发送次数从20到5，避免过长卡顿
                    self.pub.publish(msg)
                    rate.sleep()

        response = TriggerResponse()
        response.success = True
        response.message = "Arm mode changing set to True"
        return response
    
    def set_two_stage_ik_callback(self, req):
        """服务回调函数，设置两阶段IK模式"""
        self.__use_two_stage_ik = req.data
        
        # 设置ArmIk实例的两阶段IK模式
        if hasattr(self.arm_ik, 'set_use_two_stage_ik'):
            self.arm_ik.set_use_two_stage_ik(self.__use_two_stage_ik)
            rospy.loginfo(f"[IkRos] 两阶段IK模式设置为: {self.__use_two_stage_ik}")
        else:
            rospy.logwarn("[IkRos] ArmIk实例不支持两阶段IK模式")
        
        response = SetBoolResponse()
        response.success = True
        response.message = f"Two-stage IK mode set to {self.__use_two_stage_ik}"
        return response
    
    def collision_control_complete(self, req):
        """服务回调函数，设置collision_check_control状态"""
        self.collision_check_control = req.data
        if not req.data:
            self.arm_mode_changing = True

        response = SetBoolResponse()
        response.success = True
        response.message = "Collision check control set to " + str(self.collision_check_control)
        return response

if __name__ == "__main__":
    rospy.init_node("diff_ik_node", anonymous=True)

    np.set_printoptions(linewidth=240)
    np.set_printoptions(threshold=2000)
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)

    meshcat = None
    version = 4
    ik_type_idx = IkTypeIdx.TorsoIK
    ctrl_arm_idx = ArmIdx.LEFT  # 默认只控制左臂
    eef_z_bias = -0.0  # 末端坐标系的z轴偏移量
    parser = argparse.ArgumentParser()
    parser.add_argument("--ctrl_arm_idx", type=int, default=0, help="Control left or right arm, 0 for left, 1 for right.2 for both.")
    parser.add_argument("--ik_type_idx", type=int, default=0, help="Ik type, 0 for TorsoIK, 1 for DiffIK.")
    parser.add_argument("--ee_type", "--end_effector_type", dest="end_effector_type", type=str, default="", help="End effector type, jodell , qiangnao or lejuclaw.")
    parser.add_argument("--send_srv", type=int, default=1, help="Send arm control service, True or False.")
    parser.add_argument("--control_finger_type", type=int, default=0, help="0: control all fingers by upper-gripper. 1: control thumb and index fingers by upper-gripper, control other fingers by lower-gripper.")
    parser.add_argument("--control_torso", type=str2bool, default=0, help="0: do NOT control, 1: control torso.")
    parser.add_argument("--predict_gesture", type=str2bool, default=False, help="Use Neural Network to predict hand gesture, True or False.")
    parser.add_argument("--eef_z_bias", type=float, default=-0.0, help="End effector z-axis bias distance.")
    parser.add_argument("--hand_reference_mode", type=str, default="thumb_index", help="Hand reference mode: fingertips, middle_finger, or thumb_index.")
    parser.add_argument("--use_two_stage_ik", type=str2bool, default=False, help="Use two-stage IK solver for better wrist control.")
    args, unknown = parser.parse_known_args()
    
    ctrl_arm_idx = ArmIdx(args.ctrl_arm_idx)
    ik_type_idx = IkTypeIdx(args.ik_type_idx)
    send_srv = args.send_srv
    control_finger_type = args.control_finger_type
    control_torso = args.control_torso
    predict_gesture = args.predict_gesture
    hand_reference_mode = args.hand_reference_mode
    use_two_stage_ik = args.use_two_stage_ik

    print(f"\033[92mControl {ctrl_arm_idx.name()} arms.\033[0m")
    print(f"\033[92mIk type: {ik_type_idx.name()}\033[0m")
    print(f"\033[92mControl_torso: {control_torso}\033[0m")
    print(f"\033[92mUse two-stage IK: {use_two_stage_ik}\033[0m")
    
    current_pkg_path = get_package_path("motion_capture_ik")
    kuavo_assests_path = get_package_path("kuavo_assets")
    robot_version = os.environ.get('ROBOT_VERSION', '40')

    # Handle version 15 special case: use version 14 assets
    if robot_version == '15':
        robot_version = '14'

    model_file = kuavo_assests_path + f"/models/biped_s{robot_version}/urdf/drake/biped_v3_arm.urdf"
    model_config_file = kuavo_assests_path + f"/config/kuavo_v{robot_version}/kuavo.json"
    # model_file = current_pkg_path + "/models/biped_gen4.0/urdf/biped_v3_arm.urdf"
    
    assert os.path.exists(model_file), f"Model file {model_file} does not exist."
    assert os.path.exists(model_config_file), f"Model config file {model_config_file} does not exist."
    
    # end_frames_name = ["torso", "l_hand_roll", "r_hand_roll", "l_forearm_pitch", "r_forearm_pitch"]
    import json
    with open(model_config_file, 'r') as f:
        model_config = json.load(f)
    end_frames_name = model_config["end_frames_name_ik"]
    shoulder_frame_names = model_config["shoulder_frame_names"]
    upper_arm_length = model_config["upper_arm_length"]
    lower_arm_length = model_config["lower_arm_length"]
    num_arm_joints_var = model_config["NUM_ARM_JOINT"]
    eef_z_bias = model_config.get("eef_z_offset", 0.0)
    base_chest_offset_x = model_config.get("base_chest_offset_x", 0.0)
    hand_ref_length = model_config.get("hand_ref_length", 0.193)
    if use_two_stage_ik and robot_version != "13": # 使用两阶段IK时，需要减去手腕参考长度
        lower_arm_length -= hand_ref_length 
        print(f"using two-stage IK, adjust lower_arm_length: {lower_arm_length} m")
    
    # ee_type
    end_effector_type=""
    try:
        if rospy.has_param("/end_effector_type"):
            end_effector_type = rospy.get_param("/end_effector_type")
            print(f"\033[92mend_effector_type from rosparm: {end_effector_type}\033[0m")
        else:
            end_effector_type = model_config.get("EndEffectorType", ["qiangnao", "qiangnao"])[0]
            print(f"\033[92mend_effector_type from model_config: {end_effector_type}\033[0m")
    except Exception as e:
        print(e)
        
    print(f"num_arm_joints_var: {num_arm_joints_var}")
    print(f"upper_arm_length: {upper_arm_length}, lower_arm_length: {lower_arm_length}")

    rospy.set_param("/quest3/shoulder_width", model_config.get("shoulder_width", 0.15))
    rospy.set_param("/quest3/base_height_offset", model_config.get("base_height_offset", 0.23))
    rospy.set_param("/quest3/base_chest_offset_x", base_chest_offset_x)
    rospy.set_param("/quest3/upper_arm_length", upper_arm_length)
    rospy.set_param("/quest3/lower_arm_length", lower_arm_length)
    
    print(f"shoulder_width: {model_config.get('shoulder_width', 0.15)}")
    print(f"Model file: {model_file}")
    print(f"Model config file: {model_config_file}")
    print(f"shoulder_frame_names: {shoulder_frame_names}")
    print(f"End effector z-axis bias distance: {eef_z_bias} m.")
    print(f"End frames names: {end_frames_name}")
    print(f"Send srv?: {send_srv}")
    print(f"Control finger type: {control_finger_type}")
    print(f"Predict gesture?: {predict_gesture}")
    arm_ik = None


    arm_min = np.array([-3.14, -0.70, -1.57, -1.57, -1.57, -1.57, -1.57, -3.14, -2.09, -1.57, -1.57, -1.57, -1.57, -1.57], dtype=float)
    arm_max = np.array([0.520, 2.09, 1.570, 0.000, 1.570, 1.570, 1.570, 0.7, 1.000, 1.570, 0.000, 1.570, 1.570, 1.570], dtype=float)
    q_limit = None
    if ik_type_idx == IkTypeIdx.DiffIK:        
        arm_ik = DiffIK(
            model_file, 
            end_frames_name,
            arm_idx=ctrl_arm_idx, 
            q_limit=q_limit, 
            meshcat=meshcat,
            eef_z_bias=eef_z_bias,
            shoulder_frame_names=shoulder_frame_names
            )
    solver_tol_default = 9.0e-3
    iterations_limit_default = 100
    if robot_version == "13":
        solver_tol_default = 9.0e-6
        iterations_limit_default = 2000
    if ik_type_idx == IkTypeIdx.TorsoIK:
        arm_ik = ArmIk(
            model_file,
            end_frames_name,
            meshcat,
            constraint_tol=9e-3,
            solver_tol=solver_tol_default,
            iterations_limit=iterations_limit_default,
            eef_z_bias=eef_z_bias,
            ctrl_arm_idx=ctrl_arm_idx,
            as_mc_ik=True,
            shoulder_frame_names=shoulder_frame_names

        )
        arm_ik.init_state(0.0, 0.0)
        
    print("\n" + "*"*10 + "IK ARM INFO" + "*"*10)
    arm_length_left, arm_length_right = arm_ik.get_arm_length()
    p_bS = arm_ik.get_two_frame_dis_vec(shoulder_frame_names[0], end_frames_name[0])
    upper_arm_length = arm_ik.get_two_frame_dis(shoulder_frame_names[0], end_frames_name[3])
    lower_arm_length = arm_ik.get_two_frame_dis(end_frames_name[3], end_frames_name[1])
    shoulder_width_vec = arm_ik.get_two_frame_dis_vec(shoulder_frame_names[0], shoulder_frame_names[1])
    
    
    # shoulder_width = shoulder_width_vec[1]/2
    print(f"upper_arm_length: {upper_arm_length:.3f} cm, lower_arm_length: {lower_arm_length:.3f} cm")
    print(f"shoulder_width: {shoulder_width_vec[1]/2} m")
    print(f"bias_chest_to_base_link: {p_bS} m")
    # rospy.set_param("/quest3/base_shoulder_x_bias", float(p_bS[0]))
    # rospy.set_param("/quest3/base_shoulder_y_bias", float(p_bS[1]))
    # rospy.set_param("/quest3/base_shoulder_z_bias", float(p_bS[2]))
    # rospy.set_param("/quest3/upper_arm_length", float(upper_arm_length))
    # rospy.set_param("/quest3/lower_arm_length", float(lower_arm_length))
    # rospy.set_param("/quest3/shoulder_width", float(shoulder_width))
    # print(f"\033[92mLeft Arm Length: {arm_length_left:.3f} m, Right Arm Length:{arm_length_right:.3f} m.\033[0m")
    print("*"*10 + "IK ARM INFO END" + "*"*10 + "\n")

    ik_ros = IkRos(arm_ik, ctrl_arm_idx=ctrl_arm_idx, q_limit=q_limit, end_effector_type=end_effector_type, send_srv=send_srv, predict_gesture=predict_gesture, hand_reference_mode=hand_reference_mode, use_two_stage_ik=use_two_stage_ik)
