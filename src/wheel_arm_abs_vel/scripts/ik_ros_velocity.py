#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import legacy_dependencies

import signal
from tools import runtime_timing_probe as runtime_probe
import rospy
import argparse
import argparse
from enum import IntEnum
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray, Int32, Bool, Float64
from sensor_msgs.msg import JointState
from handcontrollerdemorosnode.msg import armPoseWithTimeStamp
from kuavo_msgs.msg import robotHandPosition
from kuavo_msgs.srv import controlLejuClaw, controlLejuClawRequest
from kuavo_msgs.msg import lejuClawCommand
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3, Twist
from tools.wheel_base_command import WheelBaseCommandMapper
from absolute_end_effector import checked, resolve
import time
import math
import sys
import struct
import threading
import ctypes
from tools.lightweight_transforms import *

from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeResponse, changeArmCtrlModeKuavo
from kuavo_msgs.msg import sensorsData
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolRequest, SetBoolResponse

import numpy as np
import threading

# import matplotlib.pyplot as plt

# from scipy.spatial.transform import Rotation as R

import rospy
from noitom_hi5_hand_udp_python.msg import handRotationEular
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList
from wheel_arm_abs_vel.msg import (
    AbsoluteIkCommitDebug,
    QuestTcpMappingDebug,
    TeleopTcpTarget,
    VelocityIkShadow,
    VelocityIkMapperStatus,
    WholeBodyIkReference,
)
from geometry_msgs.msg import Vector3Stamped
from kuavo_msgs.msg import JoySticks
from tools.quest3_utils_velocity import Quest3ArmInfoTransformer
from tools.torso_controls import TorsoControls
from tools.mapper_input_scheduling import MapperInputScheduler
from tools.sg100_vr_gesture import Sg100VrGestureClient
from tools.noncontrol_publish_process import publisher as diagnostic_publisher
from tools.absolute_ik_safety import AbsoluteIkExecutionSafety
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
from tools.lightweight_transforms import rpy_to_matrix
from tools.kalman_filter import KalmanFilter3D
import os

sys.path.append(os.path.join(get_package_path("noitom_hi5_hand_udp_python"), "scripts"))
from quest_input_buffer import SourceClockGate

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
            rospy.logerr(f"Failed to set thread priority! try to run as root.")
    except Exception as e:
        rospy.logerr(f"Failed to set thread priority: {e}")

QIANGNAO = "qiangnao"
JODELL = "jodell"
LEJUCLAW = "lejuclaw"
QIANGNAO_TOUCH = "qiangnao_touch"
REVO2 = "revo2"
LINKER_HAND = "linker_hand"


class AbsoluteEnableState(IntEnum):
    """Absolute VR takeover lifecycle for the wheeled-arm Python IK path."""

    WAITING = 0
    CAPTURED = 1
    SWITCHING = 2
    ACTIVE = 3
    WAIT_RELEASE = 4
    PAUSED = 5
    RESUME_PENDING = 6
    RETURNING_TO_PREP = 7
    PREPARED = 8


class AbsoluteEnableReason(IntEnum):
    NONE = 0
    ENABLE_EDGE = 1
    POSITION_ERROR = 2
    ORIENTATION_ERROR = 3
    VR_TARGET_INVALID = 4
    VR_TARGET_STALE = 5
    JOINT_STATE_INVALID = 6
    JOINT_STATE_STALE = 7
    MODE_SERVICE_FAILED = 8
    MODE_TIMEOUT = 9
    STOP_GESTURE = 10
    VR_STREAM_LOST = 11
    EXTERNAL_MODE_CHANGE = 12
    MODE_READY = 13
    MPC_NOT_READY = 14
    PAUSE_REQUEST = 15
    RESUME_REQUEST = 16
    TRACKING_SOURCE_CHANGED = 17
    TRACKING_SOURCE_MISMATCH = 18
    TRACKING_INPUT_DISCONTINUITY = 19
    IK_OUTPUT_DISCONTINUITY = 20
    PREPARED = 21


class TrackingSourceMode(IntEnum):
    AUTO = 0
    CONTROLLER = 1
    HAND = 2

control_finger_type = 0
control_torso = 0

class IkRos:
    def __init__(self, ik, ctrl_arm_idx=ArmIdx.LEFT, q_limit=None, publish_err=True, use_original_pose=False, end_effector_type="", send_srv=True, predict_gesture=False, hand_reference_mode="thumb_index", use_two_stage_ik=False, wheel_joystick_config=None):
        # Sequence counters and callback handles must exist before any ROS
        # subscription can dispatch.  The Quest subscriber is intentionally
        # created only after the complete object has been initialized below.
        self._mm_target_seq = 0
        self._tcp_target_seq = 0
        self.quest_bone_poses_sub = None
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

        # 添加两阶段IK控制参数
        self.__use_two_stage_ik = use_two_stage_ik  # 从构造函数参数获取
        self.enable_velocity_ik_takeover = bool(rospy.get_param(
            "~enable_velocity_ik_takeover", False
        ))
        self.enable_position_ik_status_log = bool(rospy.get_param(
            "~enable_position_ik_status_log", False
        ))

        # 允许通过ROS参数覆盖
        if rospy.has_param('~use_two_stage_ik'):
            self.__use_two_stage_ik = rospy.get_param('~use_two_stage_ik')
            rospy.loginfo(f"[IkRos] 通过ROS参数覆盖两阶段IK模式: {self.__use_two_stage_ik}")

        if self.enable_velocity_ik_takeover:
            rospy.loginfo(
                "[IkRos] Velocity-IK takeover requested; the legacy position-IK "
                "solver and two-stage retry path will not run"
            )
        elif self.__use_two_stage_ik:
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

        # Controller grip and optical Hand Tracking are different input
        # contracts.  A control session must never hot-switch between them.
        tracking_source_mode_name = str(
            rospy.get_param("~tracking_source_mode", "controller")
        ).strip().lower()
        tracking_source_aliases = {
            "auto": TrackingSourceMode.AUTO,
            "controller": TrackingSourceMode.CONTROLLER,
            "hand": TrackingSourceMode.HAND,
            "hand_tracking": TrackingSourceMode.HAND,
        }
        if tracking_source_mode_name not in tracking_source_aliases:
            raise ValueError(
                "~tracking_source_mode must be one of auto|controller|hand, got {!r}".format(
                    tracking_source_mode_name
                )
            )
        self.tracking_source_mode = tracking_source_aliases[tracking_source_mode_name]
        self.tracking_source_mode_name = (
            "hand"
            if self.tracking_source_mode == TrackingSourceMode.HAND
            else tracking_source_mode_name
        )
        self._tracking_session_source = (
            TeleopTcpTarget.SOURCE_QUEST_CONTROLLER
            if self.tracking_source_mode == TrackingSourceMode.CONTROLLER
            else TeleopTcpTarget.SOURCE_QUEST_HAND_TRACKING
            if self.tracking_source_mode == TrackingSourceMode.HAND
            else TeleopTcpTarget.SOURCE_UNKNOWN
        )
        self._tracking_last_observed_source = TeleopTcpTarget.SOURCE_UNKNOWN
        self._tracking_last_accepted_source = TeleopTcpTarget.SOURCE_UNKNOWN
        self._tracking_control_running = False
        self._tracking_source_hold_latched = False
        self._tracking_rejected_source_count = 0
        self._tracking_source_hold_count = 0
        self._tracking_hard_discontinuity_count = 0
        self._tracking_matching_source_streak = 0
        self._tracking_safety_previous_tcp = None
        self._tracking_safety_previous_source = TeleopTcpTarget.SOURCE_UNKNOWN
        self._tracking_source_interrupted = False
        self._absolute_ik_discontinuity_count = 0
        self._absolute_ik_consecutive_failure_count = 0
        # Root/Chest are an independent, session-relative body-intent channel.
        # They never shape or rate-limit the direct hand target.  A coherent
        # tracking-space jump advances only this epoch so the robot-side body
        # reference can re-anchor without freezing responsive arm tracking.
        self._body_coordinate_epoch = 0
        self._body_previous_sample = None
        self._body_jump_threshold = float(rospy.get_param(
            "~body_coordinate_jump_threshold", 0.25
        ))
        self._body_jump_coherence_tolerance = float(rospy.get_param(
            "~body_coordinate_jump_coherence_tolerance", 0.06
        ))

        # Rich Quest source/frame diagnostics are opt-in and are recorded by
        # the Task07 focused bag.  They do not participate in target control.
        self.enable_tracking_mapping_debug = rospy.get_param(
            "~publish_tracking_mapping_debug", False
        )
        self._tracking_debug_seq = 0
        self._tracking_debug_previous_source = QuestTcpMappingDebug.SOURCE_UNKNOWN
        self._tracking_debug_previous_timestamp_ms = None
        self._tracking_debug_previous_receive_time = None
        self._tracking_debug_previous_raw = None
        self._tracking_debug_previous_mapped = None
        self._tracking_debug_last_unique_raw_time = None
        self._tracking_debug_last_unique_mapped_time = None
        self._tracking_debug_counts = {
            "received": 0,
            "controller": 0,
            "hand_tracking": 0,
            "source_switch": 0,
            "low_confidence": 0,
            "malformed": 0,
            "raw_duplicate": 0,
            "mapped_duplicate": 0,
            "target_published": 0,
            "rejected_source": 0,
            "source_hold": 0,
            "hard_discontinuity": 0,
        }

        # 模式过渡状态只描述控制器生命周期；是否启用旧 IK 关节限速是独立策略。
        # 默认关闭限速，保持步骤 1 修改前的绝对式遥操作输出行为。
        self.wheel_arm_mode_transitioning = False
        self.enable_ik_velocity_limit = rospy.get_param("~enable_ik_velocity_limit", False)
        self.enable_ik_velocity_limit_shadow_publish = rospy.get_param(
            "/vr_ik/enable_ik_velocity_limit_shadow_publish", False
        )
        rospy.loginfo(
            "[IkRos] IK joint velocity limiter: %s; shadow diagnostics: %s; "
            "velocity IK takeover: %s",
            "enabled" if self.enable_ik_velocity_limit else "disabled",
            "enabled" if self.enable_ik_velocity_limit_shadow_publish else "disabled",
            "enabled" if self.enable_velocity_ik_takeover else "disabled",
        )

        # 轮臂绝对式启用对齐状态机。回调只写缓存/事件，IK 线程负责检查、服务调用和状态推进。
        self.enable_absolute_target_alignment = (
            self.robot_type == 1
            and rospy.get_param("~enable_absolute_target_alignment", False)
        )
        if self.enable_velocity_ik_takeover and not self.enable_absolute_target_alignment:
            raise ValueError(
                "velocity IK takeover requires the wheel-arm absolute alignment state machine"
            )
        # In takeover mode Python remains the Quest/gesture/mode supervisor, but
        # it must never solve or publish a joint-position IK trajectory.  The
        # C++ velocity IK is then the only /kuavo_arm_traj producer in mode 2.
        self.position_ik_control_enabled = not self.enable_velocity_ik_takeover
        # 默认允许操作者从任意自然手位启动。严格绝对式只禁止会话偏置，
        # 首目标差距检查作为可选安全策略保留，不再作为正常启用的前置条件。
        self.enable_absolute_start_gap_check = rospy.get_param(
            "~enable_absolute_start_gap_check", False
        )
        self.absolute_enable_max_position_error = rospy.get_param(
            "~absolute_enable_max_position_error", 0.30
        )
        self.absolute_enable_max_orientation_error = rospy.get_param(
            "~absolute_enable_max_orientation_error", 0.35
        )
        self.absolute_enable_max_vr_age = rospy.get_param(
            "~absolute_enable_max_vr_age", 0.25
        )
        self.absolute_enable_max_joint_state_age = rospy.get_param(
            "~absolute_enable_max_joint_state_age", 0.10
        )
        self.absolute_enable_mode_timeout = rospy.get_param(
            "~absolute_enable_mode_timeout", 3.0
        )
        self.absolute_enable_service_timeout = rospy.get_param(
            "~absolute_enable_service_timeout", 2.0
        )
        self.absolute_enable_exit_mode = int(rospy.get_param("~absolute_enable_exit_mode", 0))
        self.enable_absolute_discontinuity_guard = rospy.get_param(
            "~enable_absolute_discontinuity_guard", True
        )
        self.absolute_max_tcp_position_step = float(rospy.get_param(
            "~absolute_max_tcp_position_step", 0.15
        ))
        self.absolute_max_tcp_orientation_step = float(rospy.get_param(
            "~absolute_max_tcp_orientation_step", math.radians(60.0)
        ))
        self.absolute_max_joint_step = float(rospy.get_param(
            "~absolute_max_joint_step", 0.25
        ))
        self.absolute_ik_trust_region_step = max(0.0, float(rospy.get_param(
            "~absolute_ik_trust_region_step", 0.25
        )))
        self.absolute_hard_recovery_successes = max(
            1, int(rospy.get_param("~absolute_hard_recovery_successes", 3))
        )
        self.absolute_hard_target_max_step = max(0.0, float(rospy.get_param(
            "~absolute_hard_target_max_step", 0.25
        )))
        self.absolute_hard_recovery_max_offset = max(0.0, float(rospy.get_param(
            "~absolute_hard_recovery_max_offset", 0.25
        )))
        self.absolute_hard_recovery_stability_step = max(
            0.0, float(rospy.get_param(
                "~absolute_hard_recovery_stability_step", 0.05
            ))
        )
        self.absolute_execution_max_joint_velocity = max(
            0.0, float(rospy.get_param(
                "~absolute_execution_max_joint_velocity", 1.5
            ))
        )
        self.absolute_execution_max_joint_acceleration = max(
            0.0, float(rospy.get_param(
                "~absolute_execution_max_joint_acceleration", 6.0
            ))
        )
        self.absolute_execution_min_joint_limit_margin = max(
            0.0, float(rospy.get_param(
                "~absolute_execution_min_joint_limit_margin", 0.03
            ))
        )
        self.whole_body_shadow_isolate_position_ik = bool(rospy.get_param(
            "~whole_body_shadow_isolate_position_ik", False
        ))
        self._absolute_execution_safety = AbsoluteIkExecutionSafety(
            self.__arm_dof,
            self.absolute_hard_recovery_successes,
            self.absolute_hard_target_max_step,
            self.absolute_hard_recovery_max_offset,
            self.absolute_hard_recovery_stability_step,
            self.absolute_execution_max_joint_velocity,
            self.absolute_execution_max_joint_acceleration,
            joint_lower=self.__q_lb,
            joint_upper=self.__q_ub,
            min_joint_limit_margin=self.absolute_execution_min_joint_limit_margin,
        )
        self.tracking_rearm_stable_frames = max(
            1, int(rospy.get_param("~tracking_rearm_stable_frames", 3))
        )

        self._fixed_operation_mutex = threading.Lock()
        self._fixed_operation_block = False
        self._fixed_operation_error = False
        self._fixed_prepare_bootstrap = False
        self._fixed_arm_hold = False
        self._fixed_arm_operation_seq = 0
        self._fixed_pending_operation = None
        self._fixed_consumer_status = None
        self._fixed_owner_status = None
        self._fixed_control_enabled = None
        self._fixed_stop_epoch = 0
        self._fixed_operation_phase = 0  # idle, dependencies, acquisition, prepare, complete, failed
        self._fixed_dependency_timeout = float(rospy.get_param('~fixed_start_dependency_timeout', 12.0))
        self._fixed_acquisition_timeout = float(rospy.get_param('~fixed_start_acquisition_timeout', 5.0))
        self._fixed_prepare_timeout = float(rospy.get_param('~fixed_start_prepare_timeout', 20.0))
        for timeout in (self._fixed_dependency_timeout, self._fixed_acquisition_timeout,
                        self._fixed_prepare_timeout):
            if not np.isfinite(timeout) or not 0.1 <= timeout <= 30.0:
                raise ValueError('fixed start timeouts must be finite and between 0.1 and 30 seconds')
        self._absolute_lock = threading.RLock()
        self._absolute_state = AbsoluteEnableState.WAITING
        self._absolute_reason = AbsoluteEnableReason.NONE
        self._absolute_session_id = 0
        self._absolute_generation = 0
        # Mapped absolute targets are frozen in the base pose of the current
        # generation.  They must not be labelled as the live base_link once
        # whole-body IK is allowed to observe a moving base.
        self.velocity_ik_session_frame = str(
            rospy.get_param("~velocity_ik_session_frame", "ik_session_base")
        ).strip()
        if not self.velocity_ik_session_frame:
            raise ValueError("~velocity_ik_session_frame must not be empty")
        self._absolute_gesture_running = False
        self._absolute_stop_pending = False
        self._absolute_pause_pending = False
        self._absolute_external_mode_reset_pending = False
        self._absolute_latest_target = None
        self._absolute_pending_capture = None
        self._absolute_frozen_target = None
        self._absolute_latest_measured_q = None
        self._absolute_latest_measured_q_time = None
        self._absolute_latest_measured_q_source = "none"
        self._absolute_mode_status = (0, 0, False)
        self._absolute_mode_status_time = None
        self._absolute_mode_request_time = None
        self._absolute_switch_start_time = None
        self._absolute_resume_request_time = None
        self._absolute_resume_after_prepare = False
        self._absolute_prepare_complete = False
        self._joystick_receive_monotonic = None
        self._absolute_fault_hold_pending = False
        self._absolute_fault_hold_reason = AbsoluteEnableReason.NONE
        self._absolute_alignment_errors = [float("nan")] * 4
        self._absolute_capture_ages = [float("inf"), float("inf")]
        rospy.loginfo(
            "[IkRos] Absolute enable alignment: %s; start gap check: %s; "
            "pos<=%.3fm, ori<=%.3frad, "
            "VR age<=%.3fs, q age<=%.3fs, mode timeout=%.2fs",
            "enabled" if self.enable_absolute_target_alignment else "disabled",
            "enabled" if self.enable_absolute_start_gap_check else "disabled",
            self.absolute_enable_max_position_error,
            self.absolute_enable_max_orientation_error,
            self.absolute_enable_max_vr_age,
            self.absolute_enable_max_joint_state_age,
            self.absolute_enable_mode_timeout,
        )
        rospy.loginfo(
            "[IkRos] Catastrophic discontinuity guard: %s; TCP step<=%.3fm/%.3frad, "
            "joint step<=%.3frad, IK trust region=%.3frad, rearm stable frames=%d",
            "enabled" if self.enable_absolute_discontinuity_guard else "disabled",
            self.absolute_max_tcp_position_step,
            self.absolute_max_tcp_orientation_step,
            self.absolute_max_joint_step,
            self.absolute_ik_trust_region_step,
            self.tracking_rearm_stable_frames,
        )
        rospy.loginfo(
            "[IkRos] Hard-IK execution safety: soft-pose commit=disabled, "
            "target step<=%.3frad, recovery=%d frames within %.3frad "
            "(stability %.3frad), "
            "joint margin>=%.3frad, velocity<=%.2frad/s, acceleration<=%.2frad/s^2; "
            "whole-body shadow position-IK isolation=%s",
            self.absolute_hard_target_max_step,
            self.absolute_hard_recovery_successes,
            self.absolute_hard_recovery_max_offset,
            self.absolute_hard_recovery_stability_step,
            self.absolute_execution_min_joint_limit_margin,
            self.absolute_execution_max_joint_velocity,
            self.absolute_execution_max_joint_acceleration,
            "enabled" if self.whole_body_shadow_isolate_position_ik else "disabled",
        )
        rospy.loginfo(
            "[IkRos] Tracking source mode: %s; runtime source hot-switching is disabled",
            self.tracking_source_mode_name,
        )

        # 轮臂：用 /mobile_manipulator_mpc_observation 的 time（MPC 内部时间）判断是否已运行足够久（#3009）
        self._wheel_mpc_obs_time = None
        self._wheel_mpc_obs_receive_monotonic = None
        self._wheel_mm_mpc_min_internal_time = rospy.get_param("~wheel_mm_mpc_min_internal_time", 3.0)
        self._wheel_mm_mpc_observation_topic = rospy.get_param(
            "~wheel_mm_mpc_observation_topic", "/mobile_manipulator_mpc_observation"
        )

        self.use_arm_collision = rospy.get_param('~use_arm_collision', False)
        # Opt-in for wheel Quest/Pico launches; other users retain legacy gestures.
        self.dexhand_front_trigger = rospy.get_param("~dexhand_front_trigger", False)
        self.sg100_enabled = (rospy.get_param("~sg100_enabled", False)
                              and end_effector_type == "heiman")
        if self.sg100_enabled:
            self.sg100_gesture_client = Sg100VrGestureClient()
            self.sg100_trigger_pub = rospy.Publisher(
                "/sg100/mapper_triggers", Vector3Stamped, queue_size=1, tcp_nodelay=True)

        # Callbacks may run immediately after Subscriber construction.  Resolve
        # the end-effector type before registering any callback that reads it.
        self.end_effector_type = checked(end_effector_type)
        print("\033[93m--------------------------------------------------\033[0m")
        print(
            "\033[93m- End effector type: {} \033[0m".format(
                self.end_effector_type
            )
        )
        print("\033[93m--------------------------------------------------\033[0m")
        # 添加服务
        self.arm_mode_service = rospy.Service('/quest3/set_arm_mode_changing', Trigger, self.set_arm_mode_changing_callback)

        # self.hand_pub_timer = rospy.Timer(rospy.Duration(0.001), self.hand_finger_data_process)

        # 该服务为其他位置IK配置保留；速度IK接管时会明确拒绝切换。
        self.set_two_stage_ik_service = rospy.Service('/quest3/set_two_stage_ik', SetBool, self.set_two_stage_ik_callback)


        model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
        self.quest3_arm_info_transformer = Quest3ArmInfoTransformer(model_path, predict_gesture=predict_gesture, hand_reference_mode=hand_reference_mode)
        self.quest3_arm_info_transformer.control_torso = control_torso
        self.kf_left = None
        self.kf_right = None
        if self.position_ik_control_enabled:
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
        if (self.position_ik_control_enabled
                and hasattr(self.arm_ik, 'set_use_two_stage_ik')):
            self.arm_ik.set_use_two_stage_ik(self.__use_two_stage_ik)
            rospy.loginfo(f"[IkRos] ArmIk实例两阶段IK模式设置为: {self.__use_two_stage_ik}")

        # TO-DO(matthew): subscribe to joint states
        # update joint states
        self.joint_sub = rospy.Subscriber(
            "/robot_arm_q_v_tau", robotArmQVVD, self.kuavo_joint_states_callback, queue_size=1, tcp_nodelay=True
        )
        # Pico uses long X/A for thumb touch only. Keep raw arm/FSM input unchanged.
        self._torso_controls = TorsoControls(rospy.get_param('/vr_input_source', 'quest'))
        self._torso_state_pub = rospy.Publisher('/quest3/torso_control_enabled', Bool, queue_size=1, latch=True)
        self._torso_last_published = None
        self._publish_torso_control_state()
        self._pico_thumb_input = None
        if rospy.get_param('/vr_input_source', 'quest') == 'pico':
            from pico_input.thumb_buttons import ThumbButtons
            self._pico_thumb_input = ThumbButtons()
        self.joySticks_sub = rospy.Subscriber(
            "/quest_joystick_data",
            JoySticks,
            self.joySticks_data_callback,
            queue_size=32, tcp_nodelay=True
        )

        self.ik_cmd_sub = None
        if self.position_ik_control_enabled:
            self.ik_cmd_sub = rospy.Subscriber(
                "/ik/two_arm_hand_pose_cmd", twoArmHandPoseCmd,
                self.two_arm_hand_pose_target_callback, queue_size=10
            )

        self.sensor_data_raw_sub = rospy.Subscriber(
            "/sensors_data_raw", sensorsData, self.sensor_data_raw_callback, queue_size=1
        )

        # 订阅MPC优化后的状态，用于半身模式的插值和保持
        self.optimized_state_sub = rospy.Subscriber(
            "/humanoid_controller/optimizedState_wbc_mrt_origin", Float64MultiArray, self.optimized_state_callback, queue_size=10
        )

        # 轮臂：订阅 MPC observation，用 msg.time（MPC 内部时间）判断是否可发 mm / 切手臂模式（#3009）
        self._wheel_mpc_obs_sub = None
        if self.robot_type == 1:
            self._wheel_mpc_obs_sub = rospy.Subscriber(
                self._wheel_mm_mpc_observation_topic,
                mpc_observation,
                self.wheel_mpc_observation_callback,
                queue_size=10,
            )

        # 订阅停止机器人信号
        self.stop_robot_sub = rospy.Subscriber(
            "/stop_robot", Bool, self.stop_robot_callback, queue_size=1
        )

        self.robot_walking_status_sub = rospy.Subscriber(
            "/robot_walking_status", Bool, self.robot_walking_status_callback, queue_size=1
        )

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


        self.pub = None
        if self.position_ik_control_enabled and self.use_arm_collision:
            self.pub = rospy.Publisher("/arm_collision/kuavo_arm_traj", JointState, queue_size=2)
        elif self.position_ik_control_enabled:
            self.pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=2)
        self.pub_origin_joint = None
        self.pub_filtered_joint = None
        if self.position_ik_control_enabled:
            self.pub_origin_joint = rospy.Publisher(
                "/kuavo_arm_traj_origin", Float32MultiArray, queue_size=10
            )
            self.pub_filtered_joint = rospy.Publisher(
                "/kuavo_arm_traj_filtered", Float32MultiArray, queue_size=10
            )
        self.pub_velocity_limit_shadow = None
        self.pub_velocity_limit_status = None
        if (self.position_ik_control_enabled
                and self.enable_ik_velocity_limit_shadow_publish):
            self.pub_velocity_limit_shadow = rospy.Publisher(
                "/ik_debug/arm_traj_velocity_limit_shadow", Float32MultiArray, queue_size=10
            )
            self.pub_velocity_limit_status = rospy.Publisher(
                "/ik_debug/arm_traj_velocity_limit_status", Float32MultiArray, queue_size=10
            )
        self.pub_absolute_enable_status = None
        self.pub_absolute_enable_captured_q = None
        self.pub_absolute_ik_commit_debug = None
        self.pub_quest_tcp_mapping_debug = None
        if self.robot_type == 1 and self.enable_tracking_mapping_debug:
            self.pub_quest_tcp_mapping_debug = diagnostic_publisher(
                "/ik_debug/quest_tcp_mapping",
                QuestTcpMappingDebug,
                queue_size=50,
            )
        if self.enable_absolute_target_alignment:
            self.pub_absolute_enable_status = rospy.Publisher(
                "/ik_debug/absolute_enable/status",
                Float32MultiArrayStamped,
                queue_size=10,
                latch=True,
            )
            self.pub_absolute_enable_captured_q = rospy.Publisher(
                "/ik_debug/absolute_enable/captured_q",
                JointState,
                queue_size=1,
                latch=True,
            )
            if self.position_ik_control_enabled and self.enable_tracking_mapping_debug:
                self.pub_absolute_ik_commit_debug = rospy.Publisher(
                    "/ik_debug/absolute_ik_commit",
                    AbsoluteIkCommitDebug,
                    queue_size=50,
                )
        self.pub_real_arm_hand_pose = rospy.Publisher("/drake_ik/real_arm_hand_pose", twoArmHandPose, queue_size=10)
        self.pub_time_cost = None
        self.pub_ik_solve_error = None
        self.control_robot_hand_position_pub = rospy.Publisher(
            "control_robot_hand_position", robotHandPosition, queue_size=10
        )
        self.pub_ik_solved_eef_pose = None
        self.pub_ik_input_pos = None
        self.pub_q0_tmp = None
        if self.position_ik_control_enabled:
            self.pub_time_cost = rospy.Publisher(
                "/drake_ik/time_cost/ik", Float32, queue_size=10
            )
            self.pub_ik_solve_error = rospy.Publisher(
                "/drake_ik/ik_solve_error", ikSolveError, queue_size=10
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
            self.pub_mm_two_arm_hand_pose_cmd = None
            if self.position_ik_control_enabled:
                self.pub_mm_two_arm_hand_pose_cmd = rospy.Publisher(
                    "/mm/two_arm_hand_pose_cmd", twoArmHandPoseCmd, queue_size=10
                )
            # Unified TCP target consumed by the 500 Hz velocity IK.
            self.pub_velocity_timing = diagnostic_publisher("/whole_body_velocity_ik/diagnostics/mapper_timing", Float64MultiArray, queue_size=32)
            self.pub_mm_teleop_tcp_target = rospy.Publisher(
                "/mm/teleop_tcp_target", TeleopTcpTarget, queue_size=1, tcp_nodelay=True
            )

        # 添加可视化marker发布器
        self.ik_visualization_pub = diagnostic_publisher(
            "/ik_visualization_markers", MarkerArray, queue_size=10
        )

        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)
        self.stop_event = threading.Event()

        # 保存初始关节角度
        self.initial_q_first = None

        # 订阅手臂模式topic
        self._runtime_probe = runtime_probe.Recorder()
        self.arm_mode_sub = rospy.Subscriber('/quest3/triger_arm_mode', Int32, self.arm_mode_callback)

        # 订阅手臂控制模式变化话题，用于检测退出复位模式
        self.arm_control_mode_sub = rospy.Subscriber(
            "/humanoid/mpc/arm_control_mode",
            Float64MultiArray,
            self.arm_control_mode_callback
        )
        self.wheel_arm_control_mode_status_sub = None
        self.whole_body_velocity_ik_status_sub = None
        if self.robot_type == 1:
            self.wheel_arm_control_mode_status_sub = rospy.Subscriber(
                "/humanoid_wheel/arm_control_mode_status",
                Float64MultiArray,
                self.wheel_arm_control_mode_status_callback,
            )
            if self.enable_velocity_ik_takeover:
                # Keep lifecycle checks unchanged while avoiding full 21-D
                # diagnostic deserialization in the input-mapping process.
                compact_status = bool(rospy.get_param("~use_compact_ik_status", False))
                self.whole_body_velocity_ik_status_sub = rospy.Subscriber(
                    "/whole_body_velocity_ik/diagnostics/mapper_status" if compact_status
                    else "/whole_body_velocity_ik/diagnostics/reference",
                    VelocityIkMapperStatus if compact_status else VelocityIkShadow,
                    self.whole_body_velocity_ik_status_callback,
                    queue_size=10,
                )
                self._fixed_owner_sub = rospy.Subscriber(
                    '/whole_body_velocity_ik/reference/execution_owner', WholeBodyIkReference,
                    self._fixed_owner_callback, queue_size=1, tcp_nodelay=True)
                self._fixed_enable_sub = rospy.Subscriber('/enable_control_state', Bool,
                    self._fixed_enable_callback, queue_size=10)
        self.__need_reset_ik_guess = False  # 标志是否需要重置IK初始猜测
        self.__first_change_arm_mode = True  # 标志是否为第一次切换手臂模式

        # All publishers/state are now ready.  Creating this high-rate
        # subscriber earlier caused startup callbacks to observe missing
        # _tcp_target_seq/publisher members.
        self._input_clock_gate = SourceClockGate(
            float(rospy.get_param("~input_max_relative_age", 0.12)))
        self._last_fresh_source_receive = time.monotonic()
        # Scheduling belongs to the TCPROS receive thread, not ik_thread.
        scheduling = bool(rospy.get_param("~mapper_input_scheduling", False))
        self._mapper_input_scheduler = MapperInputScheduler(
            rospy.get_param("~mapper_input_cpus", "") if scheduling else "",
            rospy.get_param("~mapper_input_nice", -5) if scheduling else None)
        self.quest_bone_poses_sub = rospy.Subscriber(
            "/leju_quest_bone_poses", PoseInfoList,
            self._scheduled_quest_bone_poses_callback, queue_size=1, tcp_nodelay=True
        )

        # 所有状态、发布器和订阅器就绪后再启动IK线程，避免启动期读到半初始化状态。
        self._publish_absolute_enable_status()
        self._fixed_operation_state_pub = rospy.Publisher(
            '/quest3/pinocchio_arm_operation_state', Float64MultiArray, queue_size=1, latch=True)
        self._fixed_startup_pub = rospy.Publisher(
            '/whole_body_velocity_ik/diagnostics/startup_operation_v1',
            Float64MultiArray, queue_size=1, latch=True)
        self._fixed_operation_service = rospy.Service(
            '/quest3/pinocchio_arm_operation', changeArmCtrlMode, self.fixed_arm_operation_callback)
        self._publish_fixed_operation_state()
        self._wheel_base_pub = None
        if self.robot_type == 1 and self.enable_velocity_ik_takeover:
            self._wheel_base_mapper = WheelBaseCommandMapper(
                wheel_joystick_config,
                [rospy.get_param('/vr_cmd_vel/' + key, default) for key, default in
                 (('linear_scale_x', 0.4), ('linear_scale_y', 0.4), ('angular_scale_z', 0.25))])
            self._wheel_base_pub = rospy.Publisher(
                '/whole_body_velocity_ik/base_cmd_vel', Twist, queue_size=1)
        self.ik_thread = threading.Thread(target=self.ik_controller_thread)
        self.ik_thread.start()
        if not (self.enable_velocity_ik_takeover and os.environ.get("KUAVO_RUNTIME_PARTITION_JSON")):
            set_thread_priority(self.ik_thread, int(SCHED_FIFO), 50)
        # In the fixed chain this is the service/hand supervisor, not IK.
        # Keep it ordinary-policy on background CPUs; IK is the C++ tick.

        self.run()
        self.ik_thread.join()

    def run(self):
        while rospy.is_shutdown() is False:
            rospy.spin()

    def shutdown(self, signal, frame):
        rospy.loginfo("Shutting down...")
        self.stop_event.set()
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

    def _configured_tracking_source(self):
        if self.tracking_source_mode == TrackingSourceMode.CONTROLLER:
            return TeleopTcpTarget.SOURCE_QUEST_CONTROLLER
        if self.tracking_source_mode == TrackingSourceMode.HAND:
            return TeleopTcpTarget.SOURCE_QUEST_HAND_TRACKING
        return TeleopTcpTarget.SOURCE_UNKNOWN

    def _reset_tracking_session_source_locked(self):
        configured_source = self._configured_tracking_source()
        self._tracking_session_source = configured_source

    def _bump_absolute_generation_locked(self):
        # A subsystem pause never survives a stopped/faulted/new session.
        self._fixed_arm_hold = False
        self._absolute_generation = (self._absolute_generation + 1) & 0xFFFFFFFFFFFFFFFF
        return self._absolute_generation

    def _tracking_can_rearm_locked(self):
        expected_source = self._configured_tracking_source()
        if expected_source == TeleopTcpTarget.SOURCE_UNKNOWN:
            expected_source = self._tracking_session_source
        return (
            not self._absolute_fault_hold_pending
            and self._tracking_matching_source_streak >= self.tracking_rearm_stable_frames
            and (
                expected_source == TeleopTcpTarget.SOURCE_UNKNOWN
                or self._tracking_last_observed_source == expected_source
            )
        )

    def _clear_tracking_fault_locked(self):
        """Acknowledge a fault without manufacturing a new enable edge."""
        self._bump_absolute_generation_locked()
        self._tracking_source_hold_latched = False
        self._tracking_control_running = False
        self._tracking_safety_previous_tcp = None
        self._tracking_safety_previous_source = TeleopTcpTarget.SOURCE_UNKNOWN
        self._tracking_source_interrupted = False
        self._absolute_latest_target = None
        self._absolute_pending_capture = None
        self._absolute_frozen_target = None
        self._absolute_stop_pending = False
        self._absolute_pause_pending = False
        self._absolute_external_mode_reset_pending = False
        self._absolute_mode_request_time = None
        self._absolute_switch_start_time = None
        self._absolute_resume_request_time = None
        self._absolute_resume_after_prepare = False
        self._absolute_prepare_complete = False
        self._absolute_fault_hold_reason = AbsoluteEnableReason.NONE
        self._reset_tracking_session_source_locked()

    def _latch_tracking_fault_locked(self, reason, request_hold):
        """Invalidate all in-flight IK work and make mode-0 hold an IK-thread event."""
        if hasattr(self, '_torso_controls'):
            self._torso_controls.reset()
        if self._tracking_source_hold_latched:
            return False
        self._bump_absolute_generation_locked()
        self._tracking_source_hold_latched = True
        self._tracking_source_hold_count += 1
        self._tracking_control_running = False
        self._absolute_state = AbsoluteEnableState.WAIT_RELEASE
        self._absolute_reason = reason
        self._absolute_latest_target = None
        self._absolute_pending_capture = None
        self._absolute_frozen_target = None
        self._absolute_stop_pending = False
        self._absolute_pause_pending = False
        self._absolute_external_mode_reset_pending = False
        self._absolute_mode_request_time = None
        self._absolute_switch_start_time = None
        self._absolute_resume_request_time = None
        if request_hold:
            self._absolute_fault_hold_pending = True
            self._absolute_fault_hold_reason = reason
        return True

    def _tracking_source_policy(self, source, teleop_running):
        """Keep the configured source contract without cancelling an active session."""
        with self._absolute_lock:
            previous_source = self._tracking_last_observed_source
            source_changed = (
                previous_source != TeleopTcpTarget.SOURCE_UNKNOWN
                and source != previous_source
            )
            self._tracking_last_observed_source = source

            expected_source = self._configured_tracking_source()
            if (expected_source == TeleopTcpTarget.SOURCE_UNKNOWN
                    and self._tracking_session_source != TeleopTcpTarget.SOURCE_UNKNOWN):
                expected_source = self._tracking_session_source
            source_matches_mode = (
                expected_source == TeleopTcpTarget.SOURCE_UNKNOWN
                or source == expected_source
            )
            if source_matches_mode:
                self._tracking_matching_source_streak += 1
            else:
                self._tracking_matching_source_streak = 0

            was_interrupted = self._tracking_source_interrupted
            self._tracking_source_interrupted = not source_matches_mode
            source_hold_triggered = False
            source_hold_released = was_interrupted and source_matches_mode
            source_allowed = source_matches_mode
            if not source_allowed:
                self._tracking_rejected_source_count += 1
            session_source = self._tracking_session_source
            source_hold_latched = self._tracking_source_hold_latched
            fault_hold_pending = self._absolute_fault_hold_pending
            generation = self._absolute_generation
            rearm_stable_count = self._tracking_matching_source_streak

        if not source_matches_mode:
            rospy.logwarn_throttle(
                1.0,
                "[IkRos][TrackingSource] Ignoring source %d while mode %s is locked; "
                "holding the last valid reference and waiting for automatic recovery",
                source,
                self.tracking_source_mode_name,
            )
        elif source_hold_released:
            rospy.loginfo(
                "[IkRos][TrackingSource] Configured source returned; resuming the existing session"
            )

        return {
            "source_changed": source_changed,
            "source_matches_mode": source_matches_mode,
            "source_allowed": source_allowed,
            "source_hold_latched": False,
            "source_hold_triggered": source_hold_triggered,
            "source_hold_released": source_hold_released,
            "session_source": session_source,
            "hard_discontinuity_detected": False,
            "hard_position_step": 0.0,
            "hard_orientation_step": 0.0,
            "fault_hold_pending": fault_hold_pending,
            "generation": generation,
            "rearm_stable_count": rearm_stable_count,
            "reference_governed": False,
            "governor_alpha": 1.0,
            "governor_position_error": 0.0,
            "governor_orientation_error": 0.0,
            "governor_position_step": 0.0,
            "governor_orientation_step": 0.0,
            "governed_left_tcp": None,
            "governed_right_tcp": None,
            "governed_left_tcp_elbow": None,
            "governed_right_tcp_elbow": None,
            # Retained in the debug schema for old bag readers.  The
            # Cartesian governor has been removed, so this is always zero.
            "governor_activation_count": 0,
        }

    def _tracking_input_safety_policy(
            self, source, teleop_running, message_accepted, target_available,
            left_tcp_target, right_tcp_target, left_elbow_target,
            right_elbow_target, source_policy):
        """Record exceptional TCP/elbow steps without shaping valid motion.

        Physical feasibility belongs to the robot-side dynamic IK.  A finite
        high-speed controller sample is therefore diagnostic information, not
        a reason to replace the latest target with an interpolated trajectory.
        Coordinate/source discontinuities are handled by the session contract.
        """
        result = dict(source_policy)
        current = None
        if message_accepted and target_available:
            try:
                current = {
                    "left": (
                        np.asarray(left_tcp_target[0], dtype=float).reshape(3),
                        np.asarray(left_tcp_target[1], dtype=float).reshape(4),
                    ),
                    "right": (
                        np.asarray(right_tcp_target[0], dtype=float).reshape(3),
                        np.asarray(right_tcp_target[1], dtype=float).reshape(4),
                    ),
                    "left_elbow": np.asarray(
                        left_elbow_target, dtype=float
                    ).reshape(3),
                    "right_elbow": np.asarray(
                        right_elbow_target, dtype=float
                    ).reshape(3),
                }
                if not all(
                        np.all(np.isfinite(value))
                        for pose in (current["left"], current["right"])
                        for value in pose):
                    current = None
                elif not all(
                        np.all(np.isfinite(current[key]))
                        for key in ("left_elbow", "right_elbow")):
                    current = None
            except (TypeError, ValueError):
                current = None

        hard_position_step = 0.0
        hard_orientation_step = 0.0
        hard_discontinuity = False
        with self._absolute_lock:
            session_active = self._absolute_state in (
                AbsoluteEnableState.CAPTURED,
                AbsoluteEnableState.SWITCHING,
                AbsoluteEnableState.ACTIVE,
            ) or self._tracking_control_running
            previous = self._tracking_safety_previous_tcp
            same_source = source == self._tracking_safety_previous_source
            if current is not None and previous is not None and same_source:
                deltas = [
                    self._tracking_pose_delta(current[side], previous[side])
                    for side in ("left", "right")
                ]
                values = [value for pair in deltas for value in pair]
                if all(np.isfinite(value) for value in values):
                    hard_position_step = max(pair[0] for pair in deltas)
                    elbow_steps = [
                        float(np.linalg.norm(
                            current[side + "_elbow"] -
                            previous[side + "_elbow"]
                        ))
                        for side in ("left", "right")
                    ]
                    hard_position_step = max(
                        hard_position_step, max(elbow_steps)
                    )
                    hard_orientation_step = max(pair[1] for pair in deltas)
                    unique_frame = (
                        hard_position_step > 1.0e-9
                        or hard_orientation_step > 1.0e-7
                    )
                    hard_discontinuity = (
                        self.enable_absolute_discontinuity_guard
                        and session_active
                        and teleop_running
                        and result["source_allowed"]
                        and unique_frame
                        and (
                            hard_position_step > self.absolute_max_tcp_position_step
                            or hard_orientation_step > self.absolute_max_tcp_orientation_step
                        )
                    )

            if hard_discontinuity:
                self._tracking_hard_discontinuity_count += 1
            if (current is not None
                    and message_accepted
                    and result["source_allowed"]):
                self._tracking_safety_previous_tcp = {
                    "left": (
                        current["left"][0].copy(), current["left"][1].copy()
                    ),
                    "right": (
                        current["right"][0].copy(), current["right"][1].copy()
                    ),
                    "left_elbow": current["left_elbow"].copy(),
                    "right_elbow": current["right_elbow"].copy(),
                }
                self._tracking_safety_previous_source = source

            result["hard_discontinuity_detected"] = hard_discontinuity
            result["hard_position_step"] = hard_position_step
            result["hard_orientation_step"] = hard_orientation_step
            result["fault_hold_pending"] = self._absolute_fault_hold_pending
            result["generation"] = self._absolute_generation
            result["rearm_stable_count"] = self._tracking_matching_source_streak

        if hard_discontinuity:
            rospy.logwarn(
                "[IkRos][TrackingSafety] Exceptional TCP/elbow step detected: "
                "position %.3fm (limit %.3fm), orientation %.3frad (limit %.3frad). "
                "Recorded for diagnosis; the finite target remains direct and robot-side "
                "joint constraints own motion safety.",
                hard_position_step,
                self.absolute_max_tcp_position_step,
                hard_orientation_step,
                self.absolute_max_tcp_orientation_step,
            )
        return result

    @staticmethod
    def _tracking_pose_arrays(pose):
        position = np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=float
        )
        quaternion = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=float,
        )
        quaternion_norm = np.linalg.norm(quaternion)
        if np.isfinite(quaternion_norm) and quaternion_norm > 1.0e-12:
            quaternion /= quaternion_norm
        return position, quaternion

    @staticmethod
    def _tracking_quaternion_delta(left, right):
        if not (np.all(np.isfinite(left)) and np.all(np.isfinite(right))):
            return float("nan")
        left_norm = np.linalg.norm(left)
        right_norm = np.linalg.norm(right)
        if left_norm < 1.0e-12 or right_norm < 1.0e-12:
            if left_norm < 1.0e-12 and right_norm < 1.0e-12:
                return 0.0
            return float("nan")
        dot = abs(float(np.dot(left / left_norm, right / right_norm)))
        return 2.0 * math.acos(np.clip(dot, -1.0, 1.0))

    @classmethod
    def _tracking_pose_delta(cls, current, previous):
        if current is None or previous is None:
            return -1.0, -1.0
        current_position, current_quaternion = current
        previous_position, previous_quaternion = previous
        if not (
            np.all(np.isfinite(current_position))
            and np.all(np.isfinite(previous_position))
        ):
            return float("nan"), float("nan")
        return (
            float(np.linalg.norm(current_position - previous_position)),
            cls._tracking_quaternion_delta(current_quaternion, previous_quaternion),
        )

    @staticmethod
    def _fill_debug_pose(destination, value):
        if value is None:
            return
        position, quaternion = value
        destination.position.x = float(position[0])
        destination.position.y = float(position[1])
        destination.position.z = float(position[2])
        destination.orientation.x = float(quaternion[0])
        destination.orientation.y = float(quaternion[1])
        destination.orientation.z = float(quaternion[2])
        destination.orientation.w = float(quaternion[3])

    @staticmethod
    def _fill_debug_point(destination, value):
        if value is None:
            return
        destination.x = float(value[0])
        destination.y = float(value[1])
        destination.z = float(value[2])

    @staticmethod
    def _fill_debug_transform(destination, transform):
        if transform is None:
            return
        quaternion = np.asarray(
            matrix_to_quaternion(transform[:3, :3]), dtype=float
        )
        destination.translation.x = float(transform[0, 3])
        destination.translation.y = float(transform[1, 3])
        destination.translation.z = float(transform[2, 3])
        destination.rotation.x = float(quaternion[0])
        destination.rotation.y = float(quaternion[1])
        destination.rotation.z = float(quaternion[2])
        destination.rotation.w = float(quaternion[3])

    @staticmethod
    def _tracking_unique_dt(last_unique, receive_time, timestamp_ms):
        if last_unique is None:
            return 0.0
        last_receive_time, last_timestamp_ms = last_unique
        if timestamp_ms > 0 and last_timestamp_ms is not None and timestamp_ms > last_timestamp_ms:
            return (timestamp_ms - last_timestamp_ms) * 1.0e-3
        return max(0.0, receive_time - last_receive_time)

    def _publish_quest_tcp_mapping_debug(
            self, quest_msg, message_accepted, target_available, target_published,
            source_policy):
        """Record raw-source, mapping and the control-side source contract."""
        if self.pub_quest_tcp_mapping_debug is None:
            return

        receive_time = time.monotonic()
        timestamp_ms = int(quest_msg.timestamp_ms)
        source = (
            QuestTcpMappingDebug.SOURCE_QUEST_HAND_TRACKING
            if quest_msg.is_hand_tracking
            else QuestTcpMappingDebug.SOURCE_QUEST_CONTROLLER
        )
        previous_source = self._tracking_debug_previous_source
        source_changed = (
            previous_source != QuestTcpMappingDebug.SOURCE_UNKNOWN
            and source != previous_source
        )

        counts = self._tracking_debug_counts
        counts["received"] += 1
        if source == QuestTcpMappingDebug.SOURCE_QUEST_HAND_TRACKING:
            counts["hand_tracking"] += 1
        else:
            counts["controller"] += 1
        if source_changed:
            counts["source_switch"] += 1
        if not quest_msg.is_high_confidence:
            counts["low_confidence"] += 1

        raw = None
        if len(quest_msg.poses) > 0:
            raw = [self._tracking_pose_arrays(pose) for pose in quest_msg.poses]
        previous_raw = self._tracking_debug_previous_raw
        max_raw_position_delta = -1.0
        max_raw_orientation_delta = -1.0
        raw_payload_duplicate = False
        if raw is not None and previous_raw is not None and len(raw) == len(previous_raw):
            raw_deltas = [
                self._tracking_pose_delta(current, previous)
                for current, previous in zip(raw, previous_raw)
            ]
            raw_position_deltas = np.asarray([value[0] for value in raw_deltas], dtype=float)
            raw_orientation_deltas = np.asarray([value[1] for value in raw_deltas], dtype=float)
            if np.all(np.isfinite(raw_position_deltas)) and np.all(np.isfinite(raw_orientation_deltas)):
                max_raw_position_delta = float(np.max(raw_position_deltas))
                max_raw_orientation_delta = float(np.max(raw_orientation_deltas))
                raw_payload_duplicate = (
                    max_raw_position_delta <= 1.0e-12
                    and max_raw_orientation_delta <= 1.0e-12
                )
        if raw_payload_duplicate:
            counts["raw_duplicate"] += 1

        left_mapping = self.quest3_arm_info_transformer.get_tcp_mapping_diagnostics("Left")
        right_mapping = self.quest3_arm_info_transformer.get_tcp_mapping_diagnostics("Right")
        mapped = None
        if left_mapping is not None and right_mapping is not None:
            mapped = {
                "left_source": (
                    left_mapping["mapped_source_position"],
                    left_mapping["mapped_source_quaternion"],
                ),
                "right_source": (
                    right_mapping["mapped_source_position"],
                    right_mapping["mapped_source_quaternion"],
                ),
                "left_tcp": (
                    left_mapping["tcp_position"], left_mapping["tcp_quaternion"]
                ),
                "right_tcp": (
                    right_mapping["tcp_position"], right_mapping["tcp_quaternion"]
                ),
            }
        previous_mapped = self._tracking_debug_previous_mapped
        mapped_target_duplicate = False
        if mapped is not None and previous_mapped is not None:
            left_tcp_delta = self._tracking_pose_delta(
                mapped["left_tcp"], previous_mapped["left_tcp"]
            )
            right_tcp_delta = self._tracking_pose_delta(
                mapped["right_tcp"], previous_mapped["right_tcp"]
            )
            if all(np.isfinite(value) for value in left_tcp_delta + right_tcp_delta):
                mapped_target_duplicate = (
                    max(left_tcp_delta[0], right_tcp_delta[0]) <= 1.0e-12
                    and max(left_tcp_delta[1], right_tcp_delta[1]) <= 1.0e-12
                )
        if mapped_target_duplicate:
            counts["mapped_duplicate"] += 1
        if not source_policy["source_allowed"]:
            counts["rejected_source"] += 1
        if source_policy["source_hold_triggered"]:
            counts["source_hold"] += 1
        if source_policy["hard_discontinuity_detected"]:
            counts["hard_discontinuity"] += 1

        is_malformed = not message_accepted or not target_available or mapped is None
        if is_malformed:
            counts["malformed"] += 1
            decision = QuestTcpMappingDebug.REJECTED_MALFORMED
        elif source_policy["source_hold_latched"]:
            decision = QuestTcpMappingDebug.REJECTED_SOURCE_HOLD
        elif not source_policy["source_matches_mode"]:
            decision = QuestTcpMappingDebug.REJECTED_SOURCE_MODE
        elif not self.quest3_arm_info_transformer.is_runing:
            decision = QuestTcpMappingDebug.OBSERVED_TELEOP_INACTIVE
        elif target_published:
            if source_policy["reference_governed"]:
                decision = QuestTcpMappingDebug.PUBLISHED_GOVERNED
            elif previous_mapped is None:
                decision = QuestTcpMappingDebug.PUBLISHED_INITIAL
            elif mapped_target_duplicate:
                decision = QuestTcpMappingDebug.PUBLISHED_DUPLICATE
            else:
                decision = QuestTcpMappingDebug.PUBLISHED_UNIQUE
        else:
            decision = QuestTcpMappingDebug.REJECTED_MALFORMED
        if target_published:
            counts["target_published"] += 1

        message = QuestTcpMappingDebug()
        message.header.seq = self._tracking_debug_seq
        self._tracking_debug_seq = (self._tracking_debug_seq + 1) & 0xFFFFFFFF
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "base_link"
        if timestamp_ms > 0:
            message.source_stamp = rospy.Time.from_sec(timestamp_ms * 1.0e-3)
        message.source_timestamp_ms = timestamp_ms
        message.source = source
        message.previous_source = previous_source
        message.decision = decision
        message.configured_source_mode = int(self.tracking_source_mode)
        with self._absolute_lock:
            message.session_source = self._tracking_session_source
        message.source_changed = source_changed
        message.source_matches_mode = source_policy["source_matches_mode"]
        message.source_allowed = source_policy["source_allowed"]
        message.source_hold_latched = source_policy["source_hold_latched"]
        message.source_hold_triggered = source_policy["source_hold_triggered"]
        message.hard_discontinuity_detected = source_policy["hard_discontinuity_detected"]
        message.fault_hold_pending = source_policy["fault_hold_pending"]
        message.reference_governed = source_policy["reference_governed"]
        message.is_high_confidence = bool(quest_msg.is_high_confidence)
        message.message_accepted = bool(message_accepted)
        message.teleop_running = bool(self.quest3_arm_info_transformer.is_runing)
        message.target_available = bool(target_available)
        message.target_published = bool(target_published)
        message.source_stamp_changed = (
            self._tracking_debug_previous_timestamp_ms is None
            or timestamp_ms != self._tracking_debug_previous_timestamp_ms
        )
        message.raw_payload_duplicate = raw_payload_duplicate
        message.mapped_target_duplicate = mapped_target_duplicate
        message.pose_count = len(quest_msg.poses)
        if self._tracking_debug_previous_receive_time is not None:
            message.receive_dt = max(
                0.0, receive_time - self._tracking_debug_previous_receive_time
            )
        if self._tracking_debug_previous_timestamp_ms is not None:
            message.source_stamp_dt = (
                timestamp_ms - self._tracking_debug_previous_timestamp_ms
            ) * 1.0e-3

        raw_is_unique = raw is not None and not raw_payload_duplicate
        mapped_is_unique = mapped is not None and not mapped_target_duplicate
        if raw_is_unique:
            message.unique_raw_sample_dt = self._tracking_unique_dt(
                self._tracking_debug_last_unique_raw_time, receive_time, timestamp_ms
            )
            self._tracking_debug_last_unique_raw_time = (receive_time, timestamp_ms)
        if mapped_is_unique:
            message.unique_mapped_sample_dt = self._tracking_unique_dt(
                self._tracking_debug_last_unique_mapped_time, receive_time, timestamp_ms
            )
            self._tracking_debug_last_unique_mapped_time = (receive_time, timestamp_ms)
        if self._tracking_debug_last_unique_raw_time is not None:
            message.last_unique_raw_age = max(
                0.0, receive_time - self._tracking_debug_last_unique_raw_time[0]
            )
        if self._tracking_debug_last_unique_mapped_time is not None:
            message.last_unique_mapped_age = max(
                0.0, receive_time - self._tracking_debug_last_unique_mapped_time[0]
            )
        message.hard_position_step_limit = self.absolute_max_tcp_position_step
        message.hard_orientation_step_limit = self.absolute_max_tcp_orientation_step
        message.hard_position_step = source_policy["hard_position_step"]
        message.hard_orientation_step = source_policy["hard_orientation_step"]
        message.governor_alpha = source_policy["governor_alpha"]
        message.governor_position_error = source_policy["governor_position_error"]
        message.governor_orientation_error = source_policy["governor_orientation_error"]
        message.governor_position_step = source_policy["governor_position_step"]
        message.governor_orientation_step = source_policy["governor_orientation_step"]

        raw_indices = {
            "raw_left_upper_arm_pose": 0,
            "raw_left_lower_arm_pose": 1,
            "raw_right_upper_arm_pose": 2,
            "raw_right_lower_arm_pose": 3,
            "raw_left_palm_pose": 4,
            "raw_right_palm_pose": 5,
            "raw_chest_pose": 23,
        }
        if raw is not None:
            for field_name, index in raw_indices.items():
                if index < len(raw):
                    self._fill_debug_pose(getattr(message, field_name), raw[index])

        if mapped is not None:
            self._fill_debug_pose(message.left_mapped_source_pose, mapped["left_source"])
            self._fill_debug_pose(message.right_mapped_source_pose, mapped["right_source"])
            self._fill_debug_pose(message.left_tcp_pose, mapped["left_tcp"])
            self._fill_debug_pose(message.right_tcp_pose, mapped["right_tcp"])
            self._fill_debug_point(message.left_elbow_hint, left_mapping["elbow_hint"])
            self._fill_debug_point(message.right_elbow_hint, right_mapping["elbow_hint"])
            self._fill_debug_point(
                message.left_upper_arm_direction, left_mapping["upper_direction"]
            )
            self._fill_debug_point(
                message.left_forearm_direction, left_mapping["forearm_direction"]
            )
            self._fill_debug_point(
                message.right_upper_arm_direction, right_mapping["upper_direction"]
            )
            self._fill_debug_point(
                message.right_forearm_direction, right_mapping["forearm_direction"]
            )
            self._fill_debug_transform(
                message.left_source_to_tcp, left_mapping["source_to_tcp"]
            )
            self._fill_debug_transform(
                message.right_source_to_tcp, right_mapping["source_to_tcp"]
            )
        self._fill_debug_pose(
            message.left_governed_tcp_pose, source_policy["governed_left_tcp"]
        )
        self._fill_debug_pose(
            message.right_governed_tcp_pose, source_policy["governed_right_tcp"]
        )

        def write_delta(prefix, current, previous):
            position_delta, orientation_delta = self._tracking_pose_delta(current, previous)
            setattr(message, prefix + "_position_delta", position_delta)
            setattr(message, prefix + "_orientation_delta", orientation_delta)

        message.max_raw_position_delta = max_raw_position_delta
        message.max_raw_orientation_delta = max_raw_orientation_delta
        write_delta(
            "chest",
            raw[23] if raw is not None and len(raw) > 23 else None,
            previous_raw[23] if previous_raw is not None and len(previous_raw) > 23 else None,
        )
        write_delta(
            "left_raw_palm",
            raw[4] if raw is not None and len(raw) > 4 else None,
            previous_raw[4] if previous_raw is not None and len(previous_raw) > 4 else None,
        )
        write_delta(
            "right_raw_palm",
            raw[5] if raw is not None and len(raw) > 5 else None,
            previous_raw[5] if previous_raw is not None and len(previous_raw) > 5 else None,
        )
        for prefix, key in (
                ("left_mapped", "left_source"),
                ("right_mapped", "right_source"),
                ("left_tcp", "left_tcp"),
                ("right_tcp", "right_tcp")):
            write_delta(
                prefix,
                mapped[key] if mapped is not None else None,
                previous_mapped[key] if previous_mapped is not None else None,
            )

        message.received_count = counts["received"]
        message.controller_count = counts["controller"]
        message.hand_tracking_count = counts["hand_tracking"]
        message.source_switch_count = counts["source_switch"]
        message.low_confidence_count = counts["low_confidence"]
        message.malformed_count = counts["malformed"]
        message.raw_duplicate_count = counts["raw_duplicate"]
        message.mapped_duplicate_count = counts["mapped_duplicate"]
        message.target_published_count = counts["target_published"]
        message.rejected_source_count = counts["rejected_source"]
        message.source_hold_count = counts["source_hold"]
        message.hard_discontinuity_count = counts["hard_discontinuity"]
        message.governor_activation_count = source_policy["governor_activation_count"]
        message.absolute_generation = source_policy["generation"]
        message.rearm_stable_count = source_policy["rearm_stable_count"]
        self.pub_quest_tcp_mapping_debug.publish(message)

        if source_changed:
            rospy.logwarn(
                "[Quest3 TCP] source switch %d->%d: confidence=%s, raw_palm_delta="
                "L %.4fm/%.3frad R %.4fm/%.3frad, tcp_delta="
                "L %.4fm/%.3frad R %.4fm/%.3frad",
                previous_source,
                source,
                "high" if quest_msg.is_high_confidence else "low",
                message.left_raw_palm_position_delta,
                message.left_raw_palm_orientation_delta,
                message.right_raw_palm_position_delta,
                message.right_raw_palm_orientation_delta,
                message.left_tcp_position_delta,
                message.left_tcp_orientation_delta,
                message.right_tcp_position_delta,
                message.right_tcp_orientation_delta,
            )

        self._tracking_debug_previous_source = source
        self._tracking_debug_previous_timestamp_ms = timestamp_ms
        self._tracking_debug_previous_receive_time = receive_time
        if raw is not None:
            self._tracking_debug_previous_raw = raw
        if mapped is not None:
            self._tracking_debug_previous_mapped = mapped

    @staticmethod
    def _absolute_vector(value, size):
        if value is None:
            return None
        vector = np.asarray(value, dtype=float).reshape(-1)
        if vector.size != size or not np.all(np.isfinite(vector)):
            return None
        return vector.copy()

    @staticmethod
    def _clone_absolute_target(target):
        if target is None:
            return None
        return {
            "left_pos": target["left_pos"].copy(),
            "left_quat": target["left_quat"].copy(),
            "right_pos": target["right_pos"].copy(),
            "right_quat": target["right_quat"].copy(),
            "left_elbow": target["left_elbow"].copy(),
            "right_elbow": target["right_elbow"].copy(),
            "left_shoulder_rpy": target["left_shoulder_rpy"].copy(),
            "right_shoulder_rpy": target["right_shoulder_rpy"].copy(),
            "source_stamp": target["source_stamp"],
            "source_timestamp_ms": target["source_timestamp_ms"],
            "source": target["source"],
            "receive_time": target["receive_time"],
        }

    def _build_absolute_target_snapshot(self, quest_msg, message_accepted):
        """Copy one complete, valid mapped Quest target without changing mapping semantics."""
        if not message_accepted:
            return None
        if self.quest3_arm_info_transformer.check_if_vr_error():
            return None

        left_pos = self._absolute_vector(self.__target_pose[0], 3)
        left_quat = self._absolute_vector(self.__target_pose[1], 4)
        right_pos = self._absolute_vector(self.__target_pose_right[0], 3)
        right_quat = self._absolute_vector(self.__target_pose_right[1], 4)
        left_elbow = self._absolute_vector(self.__left_elbow_pos, 3)
        right_elbow = self._absolute_vector(self.__right_elbow_pos, 3)
        left_shoulder_rpy = self._absolute_vector(
            self.quest3_arm_info_transformer.left_shoulder_rpy_in_robot, 3
        )
        right_shoulder_rpy = self._absolute_vector(
            self.quest3_arm_info_transformer.right_shoulder_rpy_in_robot, 3
        )
        values = (
            left_pos,
            left_quat,
            right_pos,
            right_quat,
            left_elbow,
            right_elbow,
            left_shoulder_rpy,
            right_shoulder_rpy,
        )
        if any(value is None for value in values):
            return None

        left_quat_norm = np.linalg.norm(left_quat)
        right_quat_norm = np.linalg.norm(right_quat)
        if left_quat_norm < 0.5 or right_quat_norm < 0.5:
            return None
        left_quat /= left_quat_norm
        right_quat /= right_quat_norm

        if quest_msg.timestamp_ms > 0:
            source_stamp = rospy.Time.from_sec(quest_msg.timestamp_ms * 1.0e-3)
        else:
            source_stamp = rospy.Time.now()
        return {
            "left_pos": left_pos,
            "left_quat": left_quat,
            "right_pos": right_pos,
            "right_quat": right_quat,
            "left_elbow": left_elbow,
            "right_elbow": right_elbow,
            "left_shoulder_rpy": left_shoulder_rpy,
            "right_shoulder_rpy": right_shoulder_rpy,
            "source_stamp": source_stamp,
            "source_timestamp_ms": float(quest_msg.timestamp_ms),
            "source": (
                TeleopTcpTarget.SOURCE_QUEST_HAND_TRACKING
                if quest_msg.is_hand_tracking
                else TeleopTcpTarget.SOURCE_QUEST_CONTROLLER
            ),
            "receive_time": time.monotonic(),
        }

    def _update_absolute_measured_q(self, q, source):
        if not self.enable_absolute_target_alignment:
            return
        measured_q = self._absolute_vector(q, self.__arm_dof)
        if measured_q is None:
            return
        with self._absolute_lock:
            self._absolute_latest_measured_q = measured_q
            self._absolute_latest_measured_q_time = time.monotonic()
            self._absolute_latest_measured_q_source = source

    def _queue_absolute_capture_locked(self, reason, rejection_state):
        """Snapshot the latest target and measured joints while holding _absolute_lock."""
        generation = self._bump_absolute_generation_locked()
        captured_target = self._clone_absolute_target(self._absolute_latest_target)
        if captured_target is not None:
            if self.tracking_source_mode == TrackingSourceMode.AUTO:
                self._tracking_session_source = captured_target["source"]
            else:
                self._tracking_session_source = self._configured_tracking_source()
        self._absolute_session_id += 1
        self._absolute_pending_capture = {
            "target": captured_target,
            "q": (
                None
                if self._absolute_latest_measured_q is None
                else self._absolute_latest_measured_q.copy()
            ),
            "q_time": self._absolute_latest_measured_q_time,
            "q_source": self._absolute_latest_measured_q_source,
            "capture_time": time.monotonic(),
            "rejection_state": rejection_state,
            "reason": reason,
            "generation": generation,
        }
        self._absolute_frozen_target = None
        self._absolute_state = AbsoluteEnableState.CAPTURED
        self._absolute_reason = reason
        self._absolute_alignment_errors = [float("nan")] * 4
        self._absolute_capture_ages = [float("inf"), float("inf")]
        self._absolute_resume_request_time = None
        self._absolute_resume_after_prepare = False
        self._absolute_prepare_complete = False

    def _handle_absolute_target_and_gesture(self, target, gesture_running):
        """Cache targets and enqueue lifecycle events; never call services in a ROS callback."""
        if not self.enable_absolute_target_alignment:
            return None

        publish_target = None
        publish_status = False
        # Copy the immutable complete frame before acquiring the lifecycle lock.
        # State/epoch checks and publication selection remain in the transaction.
        latest_target = self._clone_absolute_target(target) if target is not None else None
        with self._absolute_lock:
            if latest_target is not None:
                self._absolute_latest_target = latest_target

            previous_running = self._absolute_gesture_running
            self._absolute_gesture_running = bool(gesture_running)
            if self._fixed_operation_block or self._fixed_prepare_bootstrap:
                # Do not swallow receiver/head/finger callbacks; only suppress
                # gesture-driven arm transactions while fixed or preparing.
                return None
            if gesture_running and not previous_running:
                if self._absolute_state in (
                        AbsoluteEnableState.RETURNING_TO_PREP,
                        AbsoluteEnableState.PREPARED):
                    # X+A, not a trigger re-press, authorizes resumption from
                    # the explicit ready-pose lifecycle.
                    return None
                # 启用沿只采样最近一帧完整有效目标；当前手势帧无效时保留此前的
                # 有效缓存，随后由统一的新鲜度检查决定是否允许接管。
                if self.enable_velocity_ik_takeover:
                    self._fixed_operation_block = True
                    self._queue_fixed_request_locked(2, 'dependencies')
                else:
                    self._queue_absolute_capture_locked(
                        AbsoluteEnableReason.ENABLE_EDGE,
                        AbsoluteEnableState.WAIT_RELEASE,
                    )
                publish_status = True
                rospy.loginfo(
                    "[IkRos][AbsoluteEnable] Session %d captured on enable edge",
                    self._absolute_session_id,
                )
            elif not gesture_running and previous_running:
                if self._absolute_state in (
                    AbsoluteEnableState.RETURNING_TO_PREP,
                    AbsoluteEnableState.PREPARED,
                ):
                    # X+A ready-pose return is a self-contained safety
                    # trajectory.  Releasing the controller trigger must not
                    # revoke its 21-D authority halfway through the motion.
                    self._tracking_control_running = False
                else:
                    self._bump_absolute_generation_locked()
                if self._absolute_state in (
                    AbsoluteEnableState.SWITCHING,
                    AbsoluteEnableState.ACTIVE,
                ):
                    self._absolute_stop_pending = True
                elif self._absolute_state not in (
                    AbsoluteEnableState.RETURNING_TO_PREP,
                    AbsoluteEnableState.PREPARED,
                ):
                    self._absolute_pending_capture = None
                    self._absolute_frozen_target = None
                    self._absolute_state = AbsoluteEnableState.WAITING
                    self._absolute_reason = AbsoluteEnableReason.NONE
                    self._absolute_pause_pending = False
                    self._absolute_resume_request_time = None
                    self._tracking_control_running = False
                    self._reset_tracking_session_source_locked()
                    publish_status = True

            if (self._absolute_state == AbsoluteEnableState.SWITCHING
                    and self._absolute_gesture_running
                    and not self._absolute_stop_pending):
                publish_target = self._clone_absolute_target(self._absolute_frozen_target)
            elif (self._absolute_state == AbsoluteEnableState.ACTIVE
                    and self._absolute_gesture_running
                    and not self._absolute_stop_pending
                    and target is not None):
                publish_target = self._clone_absolute_target(target)

        if publish_status:
            self._publish_absolute_enable_status()
        return publish_target

    def _compose_absolute_q_seed(self, measured_arm_q):
        if self.enable_velocity_ik_takeover:
            # The C++ 21-D solver captures and aligns against its own measured
            # full-body state. Python needs only an atomic copy of the 14 arm
            # measurements for the lifecycle transaction; constructing a
            # legacy position-IK model solely to manufacture a q0 prefix kept
            # the real Quest mapper offline for tens of seconds.
            return np.asarray(measured_arm_q, dtype=float).copy()
        q_template = self.arm_ik.q0() if self.external_q0 is None else self.external_q0
        q_seed = np.asarray(q_template, dtype=float).copy()
        if q_seed.size < self.__arm_dof:
            return None
        q_seed[-self.__arm_dof:] = measured_arm_q
        return q_seed

    def _reset_absolute_position_filters(self, target):
        """Start each absolute session from its captured target, never old filter history."""
        for position_filter, position in (
            (self.kf_left, target["left_pos"]),
            (self.kf_right, target["right_pos"]),
        ):
            state = np.zeros_like(position_filter.state, dtype=float)
            state[:3] = position
            position_filter.state = state
            position_filter.covariance = np.eye(position_filter.covariance.shape[0])

    @staticmethod
    def _quaternion_angular_distance(quat_a, quat_b):
        quat_a = np.asarray(quat_a, dtype=float)
        quat_b = np.asarray(quat_b, dtype=float)
        norm_a = np.linalg.norm(quat_a)
        norm_b = np.linalg.norm(quat_b)
        if norm_a < 1.0e-9 or norm_b < 1.0e-9:
            return float("inf")
        dot = abs(float(np.dot(quat_a / norm_a, quat_b / norm_b)))
        return 2.0 * math.acos(np.clip(dot, -1.0, 1.0))

    def _calculate_absolute_alignment_errors(self, q_seed, target):
        """Compare the measured robot state using the current mixed two-stage target contract."""
        plant = self.arm_ik._ArmIk__plant
        context = plant.CreateDefaultContext()
        plant.SetPositions(context, q_seed)
        base_frame = plant.GetFrameByName(self.arm_ik._ArmIk__base_link_name)

        position_frame_names = (
            ("zarm_l6_link", "zarm_r6_link")
            if self.__use_two_stage_ik
            else (self.arm_ik._ArmIk__left_eef_name, self.arm_ik._ArmIk__right_eef_name)
        )
        orientation_frame_names = (
            self.arm_ik._ArmIk__left_eef_name,
            self.arm_ik._ArmIk__right_eef_name,
        )
        target_positions = (target["left_pos"], target["right_pos"])
        target_quaternions = (target["left_quat"], target["right_quat"])

        errors = []
        for arm_index in range(2):
            position_pose = plant.GetFrameByName(position_frame_names[arm_index]).CalcPose(
                context, base_frame
            )
            orientation_pose = plant.GetFrameByName(orientation_frame_names[arm_index]).CalcPose(
                context, base_frame
            )
            measured_quat = orientation_pose.rotation().ToQuaternion()
            measured_quat_xyzw = np.array(
                [measured_quat.x(), measured_quat.y(), measured_quat.z(), measured_quat.w()]
            )
            errors.append(
                (
                    float(np.linalg.norm(position_pose.translation() - target_positions[arm_index])),
                    self._quaternion_angular_distance(
                        measured_quat_xyzw, target_quaternions[arm_index]
                    ),
                )
            )
        return errors

    def _arm_is_controlled(self, arm_index):
        if self.__ctrl_arm_idx == ArmIdx.BOTH:
            return True
        if arm_index == 0:
            return self.__ctrl_arm_idx == ArmIdx.LEFT
        return self.__ctrl_arm_idx == ArmIdx.RIGHT

    def _publish_absolute_enable_status(self):
        self._publish_fixed_operation_state()
        if self.pub_absolute_enable_status is None:
            return
        with self._absolute_lock:
            current_mode, target_mode, transitioning = self._absolute_mode_status
            left_pos, right_pos, left_ori, right_ori = self._absolute_alignment_errors
            vr_age, q_age = self._absolute_capture_ages
            # v2 appends the tracking-source contract to the original v1 layout.
            msg_data = [
                float(self._absolute_state),
                float(self._absolute_reason),
                float(self._absolute_session_id),
                float(left_pos),
                float(right_pos),
                float(left_ori),
                float(right_ori),
                float(vr_age * 1000.0),
                float(q_age * 1000.0),
                float(current_mode),
                float(target_mode),
                float(transitioning),
                float(self._absolute_state == AbsoluteEnableState.ACTIVE),
                float(self.tracking_source_mode),
                float(self._tracking_session_source),
                float(self._tracking_source_hold_latched),
                float(self._tracking_rejected_source_count),
                float(self._tracking_source_hold_count),
                float(self._absolute_generation),
                float(self._absolute_fault_hold_pending),
                float(self._tracking_matching_source_streak),
                float(self._tracking_hard_discontinuity_count),
                float(self._tracking_source_interrupted),
                0.0,  # retired Cartesian governor active
                0.0,  # retired Cartesian governor activation count
                float(self._absolute_ik_discontinuity_count),
                float(self._absolute_ik_consecutive_failure_count),
                float(self._absolute_resume_after_prepare),
                float(self._absolute_prepare_complete),
            ]
        msg = Float32MultiArrayStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "absolute_enable_v4"
        msg.data.data = msg_data
        self.pub_absolute_enable_status.publish(msg)

    def _publish_absolute_captured_q(self, measured_arm_q):
        if self.pub_absolute_enable_captured_q is None:
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "measured_radians"
        msg.name = ["arm_joint_" + str(i) for i in range(1, self.__arm_dof + 1)]
        msg.position = measured_arm_q.tolist()
        self.pub_absolute_enable_captured_q.publish(msg)

    def _publish_mm_absolute_target(self, target):
        if (self.robot_type != 1 or target is None
                or self.pub_mm_two_arm_hand_pose_cmd is None):
            return
        eef_pose_msg = twoArmHandPoseCmd()
        eef_pose_msg.frame = 3
        eef_pose_msg.hand_poses.header.stamp = target["source_stamp"]
        eef_pose_msg.hand_poses.header.seq = self._mm_target_seq
        self._mm_target_seq = (self._mm_target_seq + 1) & 0xFFFFFFFF
        eef_pose_msg.hand_poses.header.frame_id = "base_link"
        eef_pose_msg.hand_poses.left_pose.pos_xyz = target["left_pos"]
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = target["left_quat"]
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = target["left_elbow"]
        eef_pose_msg.hand_poses.right_pose.pos_xyz = target["right_pos"]
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = target["right_quat"]
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = target["right_elbow"]
        self.pub_mm_two_arm_hand_pose_cmd.publish(eef_pose_msg)


    def _fixed_operation_mode(self):
        with self._absolute_lock:
            if self._fixed_operation_error:
                return -1  # explicit fault/pending hold, not a fake completed mode
            if self._fixed_pending_operation is not None:
                return (2 if self._fixed_pending_operation['resume'] else
                        self._fixed_pending_operation['mode'])  # intent, NOT completion
            if self._fixed_prepare_bootstrap or self._absolute_state in (
                    AbsoluteEnableState.RETURNING_TO_PREP, AbsoluteEnableState.PREPARED):
                return 1
            return (2 if self._absolute_state == AbsoluteEnableState.ACTIVE
                    and not getattr(self, '_fixed_arm_hold', False) else 0)

    def _publish_fixed_operation_state(self):
        if hasattr(self, '_torso_controls'):
            self._publish_torso_control_state()
        self._publish_wheel_base_command()
        if hasattr(self, '_fixed_operation_state_pub'):
            mode = self._fixed_operation_mode()
            self._fixed_operation_state_pub.publish(Float64MultiArray(data=[float(mode), float(mode)]))
        if hasattr(self, '_fixed_startup_pub'):
            with self._absolute_lock:
                p = self._fixed_pending_operation
                self._fixed_startup_pub.publish(Float64MultiArray(data=[
                    1., float(self._fixed_operation_phase),
                    float(p['mode'] if p else -1),
                    max(0., p['deadline'] - time.monotonic()) if p else 0.,
                    float(self._absolute_session_id), float(self._absolute_generation)]))

    def fixed_arm_operation_callback(self, req):
        """Operation adapter: real capture/consumer handshake, never a second writer."""
        if not self._fixed_operation_mutex.acquire(False):
            return changeArmCtrlModeResponse(False, self._fixed_operation_mode(), "Operation is still pending")
        try:
            return self._fixed_arm_operation(req.control_mode)
        finally:
            self._publish_fixed_operation_state()
            self._fixed_operation_mutex.release()

    def _fixed_capture_input_error_locked(self):
        """Explicit button authority does not replace fresh pose/sensor input."""
        now = time.monotonic()
        target = self._absolute_latest_target
        target_age = float("inf") if target is None else now - target["receive_time"]
        joint_age = (float("inf") if self._absolute_latest_measured_q is None
                     or self._absolute_latest_measured_q_time is None
                     else now - self._absolute_latest_measured_q_time)
        joystick_age = (float("inf") if self._joystick_receive_monotonic is None
                        else now - self._joystick_receive_monotonic)
        if (not 0.0 <= target_age <= self.absolute_enable_max_vr_age
                or not 0.0 <= joint_age <= self.absolute_enable_max_joint_state_age
                or not 0.0 <= joystick_age <= self.absolute_enable_max_vr_age):
            return ("Fresh Quest input and measured joints are required "
                    "(target={:.1f}ms, joints={:.1f}ms, joystick={:.1f}ms)".format(
                        target_age * 1000., joint_age * 1000., joystick_age * 1000.))
        return None

    def _authorize_fixed_follow_locked(self):
        # mode=2 is an explicit operator follow request (X+A or X+B), not a
        # pose-confidence inference. Use the SAME latch as gesture control so
        # subsequent real stop gestures still revoke it normally.
        if hasattr(self, '_torso_controls'):
            self._torso_controls.request_follow()
        transformer = self.quest3_arm_info_transformer
        transformer.ok_gesture_counts = 0
        transformer.shot_gesture_counts = 0
        transformer.is_runing = True
        self._absolute_gesture_running = True
        self._absolute_stop_pending = False
        self._absolute_pause_pending = False
        self._absolute_external_mode_reset_pending = False

    def _interrupt_fixed_prepare_locked(self):
        """Recapture a live PREPARE transaction; solver retains owned q/dq/ddq."""
        now = time.monotonic()
        p = self._fixed_pending_operation
        error = self._fixed_capture_input_error_locked()
        if (error or self._fixed_control_enabled is not True
                or self._absolute_mode_status != (2, 2, False)
                or not self._fixed_consumer_ack_locked(now)
                or self._absolute_stop_pending or self._absolute_fault_hold_pending
                or self._tracking_source_hold_latched or self._absolute_external_mode_reset_pending
                or self.quest3_arm_info_transformer.is_stop_gesture()
                or (p is not None and (p['generation'] != self._absolute_generation
                    or p['stop_epoch'] != self._fixed_stop_epoch
                    or p['epoch'] != self._body_coordinate_epoch
                    or p['source'] != self._configured_tracking_source()
                    or now >= p['deadline']))):
            reason = error or 'stop, stale execution owner, disabled control or changed transaction'
            rospy.logwarn('[FixedStart] PREPARE interruption rejected: %s', reason)
            return False, reason
        self._authorize_fixed_follow_locked()
        self._fixed_prepare_bootstrap = False
        self._fixed_operation_block = False
        self._queue_absolute_capture_locked(AbsoluteEnableReason.RESUME_REQUEST,
                                            AbsoluteEnableState.WAIT_RELEASE)
        self._queue_fixed_request_locked(2, 'acquisition')
        self._fixed_pending_operation['deadline'] = now + self._fixed_acquisition_timeout
        self._fixed_operation_phase = 2
        rospy.loginfo('[FixedStart] PREPARE interrupted; recapturing follow generation %d',
                      self._absolute_generation)
        return True, 'PREPARE interrupted; continuous reference retained while follow reacquires'

    def _fixed_arm_operation(self, mode):
        def result(ok, message):
            return changeArmCtrlModeResponse(ok, self._fixed_operation_mode(), message)
        if not (self.enable_absolute_target_alignment and self.enable_velocity_ik_takeover):
            return result(False, "Fixed Pinocchio operation requires the formal pipeline")
        if mode not in (0, 1, 2):
            return result(False, "Invalid operation mode")
        with self._absolute_lock:
            held = getattr(self, '_fixed_arm_hold', False)
            torso = getattr(self, '_torso_controls', None)
            # Keep the existing execution owner while pausing only the arms,
            # whether or not torso control has been enabled. A real stop/fault still ends the complete session.
            if mode in (0, 2) and (held or (mode == 0 and self._absolute_state == AbsoluteEnableState.ACTIVE)):
                now = time.monotonic()
                ready = (self._torso_input_ready_locked(now)
                         and not self.quest3_arm_info_transformer.is_stop_gesture()
                         and self._fixed_capture_input_error_locked() is None
                         and self._fixed_consumer_ack_locked(now))
                if ready:
                    requested_hold = mode == 0
                    if requested_hold != held:
                        self._fixed_arm_operation_seq = getattr(self, '_fixed_arm_operation_seq', 0) + 1
                        self._fixed_arm_hold = requested_hold
                    return result(True, "Arm-only brake/hold requested; torso session retained"
                                  if requested_hold else "Arm follow resumed; torso session retained")
                if held and mode == 2:
                    return result(False, "Arm resume requires fresh input and a healthy execution owner; arms remain held")
                # A pause request with unavailable dependencies falls back to
                # the existing whole-session hold below, never silent motion.
            self._fixed_arm_hold = False
            if hasattr(self, '_torso_controls'):
                if mode == 2:
                    self._torso_controls.wait_for_follow()
                else:
                    self._torso_controls.reset()
        if mode == 0:
            # End the input generation before the existing stopped-owner/hold
            # handshake. Partial service failures remain faults, never success.
            with self._absolute_lock:
                self._fixed_pending_operation = None
                self._fixed_operation_phase = 0
                self._fixed_operation_block = True
                self._fixed_prepare_bootstrap = False
            self._finish_absolute_session(AbsoluteEnableReason.EXTERNAL_MODE_CHANGE, False,
                                          "User requested fixed hold")
            ok = self._request_absolute_hold_mode()
            if not ok:
                with self._absolute_lock:
                    self._absolute_fault_hold_pending = True
                    self._fixed_operation_error = True
                    self._absolute_fault_hold_reason = AbsoluteEnableReason.MODE_SERVICE_FAILED
                return result(False, "Partial hold failure; safety hold remains pending")
            return result(True, "Fixed hold accepted by RM and consumer")
        with self._absolute_lock:
            state = self._absolute_state
            if self._absolute_fault_hold_pending:
                return result(False, "Consumer fault hold is still pending")
            if self._tracking_source_hold_latched:
                # Preserve explicit source-fault rearm, including stability gates.
                if mode == 2:
                    self.arm_mode_callback(Int32(data=2))
                    if not self._tracking_source_hold_latched:
                        self._fixed_operation_block = False
                    return result(not self._tracking_source_hold_latched,
                                  "Source rearm evaluated; a fresh allowed frame is still required")
                if not self._tracking_can_rearm_locked():
                    return result(False, "Source is not stable enough to prepare")
                self._clear_tracking_fault_locked()
            if self.quest3_arm_info_transformer.is_stop_gesture():
                return result(False, "Release the stop gesture before requesting an operation")
            if self._fixed_pending_operation is not None:
                pending = self._fixed_pending_operation
                if pending['mode'] == 1:
                    if mode == 2:
                        error = self._fixed_capture_input_error_locked()
                        if error:
                            return result(False, error)
                        if state == AbsoluteEnableState.RETURNING_TO_PREP:
                            return result(*self._interrupt_fixed_prepare_locked())
                        # PREPARE has not acquired execution yet: change intent,
                        # keeping the original bounded dependency/acquisition wait.
                        pending.update(mode=2, resume=False)
                        self._fixed_prepare_bootstrap = False
                        self._fixed_operation_block = False
                        self._authorize_fixed_follow_locked()
                        return result(True, "Follow replaces pending PREPARE before execution")
                    pending['resume'] = False
                    self._absolute_resume_after_prepare = False
                    return result(True, "PREPARE retained; queued follow cancelled")
                if mode == 1 and pending['mode'] == 2:
                    # A third X+A cancels in-flight follow acquisition. Do not
                    # silently swallow it merely because the owner ack is late.
                    pending.update(mode=1, resume=False)
                    self._fixed_operation_block = True
                    if state == AbsoluteEnableState.ACTIVE:
                        self._pause_absolute_session()
                        pending.update(phase='prepare', generation=self._absolute_generation)
                    else:
                        self._fixed_prepare_bootstrap = True
                    return result(True, "Follow acquisition replaced by PREPARE")
                return result(True, "Request already pending; deadline unchanged (not completed)")
            if mode == 1 and self._absolute_mode_status == (2, 2, False) and state in (AbsoluteEnableState.ACTIVE, AbsoluteEnableState.PAUSED,
                                      AbsoluteEnableState.RETURNING_TO_PREP, AbsoluteEnableState.PREPARED):
                self._fixed_operation_block = True
                if state not in (AbsoluteEnableState.RETURNING_TO_PREP, AbsoluteEnableState.PREPARED):
                    self._pause_absolute_session()
                if state != AbsoluteEnableState.PREPARED:
                    self._queue_fixed_request_locked(1, 'prepare')
                return result(True, "Pinocchio PREPARE requested; not yet a completion acknowledgement")
            # A button request requires live operator transport. Missing robot
            # feedback/MPC readiness is instead a bounded dependency wait.
            age = (float('inf') if self._joystick_receive_monotonic is None
                   else time.monotonic() - self._joystick_receive_monotonic)
            if not 0 <= age <= self.absolute_enable_max_vr_age:
                return result(False, "Fresh joystick input is required")
            if mode == 2 and state == AbsoluteEnableState.RETURNING_TO_PREP:
                return result(*self._interrupt_fixed_prepare_locked())
            self._fixed_operation_error = False
            self._fixed_operation_block = True
            self._queue_fixed_request_locked(mode, 'dependencies')
        return result(True, "Request accepted; waiting for readiness and real execution acknowledgement")

    def _queue_fixed_request_locked(self, mode, phase):
        timeout = self._fixed_dependency_timeout if phase == 'dependencies' else self._fixed_prepare_timeout
        self._fixed_pending_operation = dict(mode=mode, phase=phase, resume=False,
            deadline=time.monotonic()+timeout, generation=self._absolute_generation,
            epoch=self._body_coordinate_epoch, source=self._configured_tracking_source(),
            stop_epoch=self._fixed_stop_epoch)
        self._fixed_operation_phase = 1 if phase == 'dependencies' else 3

    def _fixed_enable_callback(self, msg):
        with self._absolute_lock:
            if not msg.data and (self._fixed_control_enabled is True
                                 or self._fixed_pending_operation is not None):
                self._fixed_stop_epoch += 1
            self._fixed_control_enabled = bool(msg.data)
        self._publish_wheel_base_command()

    def _fixed_owner_callback(self, msg):
        now = time.monotonic()
        # Reject delayed/replayed owner messages using the same host monotonic
        # domain as the C++ handshake. This does not grant motion authority.
        age = now - msg.generated_monotonic_ns * 1e-9
        if not 0 <= age <= .1:
            return
        with self._absolute_lock:
            self._fixed_owner_status = (now, msg)

    def _fixed_consumer_ack_locked(self, now):
        if self._fixed_consumer_status is None or self._fixed_owner_status is None:
            return False
        for received, msg in (self._fixed_consumer_status, self._fixed_owner_status):
            if (not 0 <= now-received <= .15 or msg.session_id != self._absolute_session_id
                    or msg.generation != self._absolute_generation
                    or not msg.valid or not msg.takeover_active):
                return False
        owner = self._fixed_owner_status[1]
        return (owner.authority_state == WholeBodyIkReference.AUTHORITY_ACTIVE
                and not owner.safety_braking and bool(owner.execution_owner_instance)
                and owner.producer_lease_deadline_ns > now*1e9)

    def _advance_fixed_request(self):
        """Nonblocking progress in the existing IK thread; service accepts intent only."""
        failure = None
        with self._absolute_lock:
            # Callback timestamps and this snapshot must share the lock.
            # Sampling before acquiring it can make a fresh callback appear
            # to come from the future and spuriously cancel PREPARE.
            now = time.monotonic()
            p = self._fixed_pending_operation
            if p is None:
                return
            if (p['generation'] != self._absolute_generation
                    or p['stop_epoch'] != self._fixed_stop_epoch
                    or p['epoch'] != self._body_coordinate_epoch
                    or p['source'] != self._configured_tracking_source()
                    or self._absolute_fault_hold_pending or self._tracking_source_hold_latched
                    or self._absolute_stop_pending or self._absolute_external_mode_reset_pending
                    or self.quest3_arm_info_transformer.is_stop_gesture()):
                failure = 'Pending operation cancelled by stop/fault/session/source change'
            elif now >= p['deadline']:
                failure = 'Operation timed out in ' + p['phase'] + '; press X+A again after checking readiness'
            elif (self._joystick_receive_monotonic is None
                    or not 0 <= now-self._joystick_receive_monotonic <= .5):
                age = (float('inf') if self._joystick_receive_monotonic is None
                       else now-self._joystick_receive_monotonic)
                failure = ('Pending operation cancelled: joystick %s (age=%.6fs)' %
                           ('timestamp invalid' if age < 0 else 'stream lost', age))
            elif (self._absolute_latest_target is not None
                    and not 0 <= now-self._absolute_latest_target['receive_time'] <= .5):
                age = now-self._absolute_latest_target['receive_time']
                failure = ('Pending operation cancelled: pose %s (age=%.6fs)' %
                           ('timestamp invalid' if age < 0 else 'stream lost', age))
            elif p['phase'] == 'dependencies':
                error = self._fixed_capture_input_error_locked()
                if error or self._fixed_control_enabled is not True or not self._wheel_mpc_stable_for_mm_cmd():
                    rospy.loginfo_throttle(1., '[FixedStart] waiting: %s', error or
                        ('controller not enabled' if self._fixed_control_enabled is not True else 'MPC internal time not ready'))
                    self._publish_fixed_operation_state()
                    return
                if p['mode'] == 2 or p['resume']:
                    self._authorize_fixed_follow_locked()
                self._fixed_prepare_bootstrap = p['mode'] == 1
                self._fixed_operation_block = p['mode'] == 1
                self._queue_absolute_capture_locked(AbsoluteEnableReason.RESUME_REQUEST,
                                                    AbsoluteEnableState.WAIT_RELEASE)
                self._absolute_resume_after_prepare = p['resume']
                p.update(phase='acquisition', generation=self._absolute_generation,
                         deadline=now+self._fixed_acquisition_timeout)
                self._fixed_operation_phase = 2
            elif self._fixed_consumer_ack_locked(now):
                if p['mode'] == 2 and self._absolute_state == AbsoluteEnableState.ACTIVE:
                    self._fixed_pending_operation = None
                    self._fixed_operation_phase = 4
                    rospy.loginfo('[FixedStart] Follow active: valid IK reference and execution lease confirmed')
                elif self._absolute_state == AbsoluteEnableState.PREPARED:
                    self._fixed_pending_operation = None
                    self._fixed_operation_phase = 4
                    rospy.loginfo('[FixedStart] PREPARE completed with execution acknowledgement')
                elif p['phase'] == 'acquisition' and p['mode'] == 1:
                    p.update(phase='prepare', deadline=now+self._fixed_prepare_timeout)
                    self._fixed_operation_phase = 3
                    rospy.loginfo('[FixedStart] Execution acquired; PREPARE is now running')
            if failure:
                self._fixed_pending_operation = None
                self._fixed_operation_phase = 5
                self._fixed_operation_block = True
                self._fixed_prepare_bootstrap = False
                self._absolute_resume_after_prepare = False
        if failure:
            self._finish_absolute_session(AbsoluteEnableReason.MODE_TIMEOUT, True, failure)
        self._publish_fixed_operation_state()

    def _call_absolute_mode_service(self, service_name, mode):
        try:
            rospy.wait_for_service(service_name, timeout=self.absolute_enable_service_timeout)
            client = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            response = client(mode)
            if not response.result:
                rospy.logerr(
                    "[IkRos][AbsoluteEnable] %s rejected mode %d: %s",
                    service_name,
                    mode,
                    response.message,
                )
                return False
            return True
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logerr(
                "[IkRos][AbsoluteEnable] Failed to call %s(mode=%d): %s",
                service_name,
                mode,
                error,
            )
            return False

    def _request_absolute_external_mode(self, expected_generation):
        # 先准备MPC/RM控制面，最后请求WBC物理接管。
        with self._absolute_lock:
            transaction_valid = (
                expected_generation == self._absolute_generation
                and self._absolute_state == AbsoluteEnableState.SWITCHING
                and not self._tracking_source_hold_latched
            )
        if not transaction_valid:
            return False
        if not self._call_absolute_mode_service("/wheel_arm_change_arm_ctrl_mode", 2):
            return False
        with self._absolute_lock:
            transaction_valid = (
                expected_generation == self._absolute_generation
                and self._absolute_state == AbsoluteEnableState.SWITCHING
                and not self._tracking_source_hold_latched
            )
        if not transaction_valid:
            self._call_absolute_mode_service(
                "/wheel_arm_change_arm_ctrl_mode", self.absolute_enable_exit_mode
            )
            return False
        with self._absolute_lock:
            self._absolute_mode_request_time = time.monotonic()
        if not self._call_absolute_mode_service("/change_arm_ctrl_mode", 2):
            self._call_absolute_mode_service(
                "/wheel_arm_change_arm_ctrl_mode", self.absolute_enable_exit_mode
            )
            return False
        with self._absolute_lock:
            transaction_valid = (
                expected_generation == self._absolute_generation
                and self._absolute_state == AbsoluteEnableState.SWITCHING
                and not self._tracking_source_hold_latched
            )
        if not transaction_valid:
            self._request_absolute_hold_mode()
            return False
        return True

    def _request_absolute_hold_mode(self):
        wbc_ok = self._call_absolute_mode_service(
            "/change_arm_ctrl_mode", self.absolute_enable_exit_mode
        )
        rm_ok = self._call_absolute_mode_service(
            "/wheel_arm_change_arm_ctrl_mode", self.absolute_enable_exit_mode
        )
        return wbc_ok and rm_ok

    def _publish_absolute_ik_commit_debug(
            self, decision, target, solve_generation, current_generation,
            previous_q, candidate_q, max_joint_step=0.0, max_joint_index=0,
            joint_limit_count=0, committed=False, state=None,
            source_hold_latched=None, fault_hold_pending=None,
            trust_region_retry_used=False, relaxed_retry_used=False,
            pre_retry_max_joint_step=0.0, pre_retry_max_joint_index=0,
            command_held=False, shadow_isolation_active=False,
            command_rate_limited=False, trusted_max_joint_offset=0.0,
            min_joint_limit_margin=float("inf")):
        if self.pub_absolute_ik_commit_debug is None:
            return
        msg = AbsoluteIkCommitDebug()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "arm_joint_radians"
        if target is not None:
            msg.source_stamp = target["source_stamp"]
            msg.source_timestamp_ms = int(target["source_timestamp_ms"])
            msg.source = int(target["source"])
        else:
            msg.source = TeleopTcpTarget.SOURCE_UNKNOWN
        with self._absolute_lock:
            msg.session_id = self._absolute_session_id
            if state is None:
                state = self._absolute_state
            if source_hold_latched is None:
                source_hold_latched = self._tracking_source_hold_latched
            if fault_hold_pending is None:
                fault_hold_pending = self._absolute_fault_hold_pending
            discontinuity_count = self._absolute_ik_discontinuity_count
            consecutive_failure_count = self._absolute_ik_consecutive_failure_count
        msg.solve_generation = int(solve_generation or 0)
        msg.current_generation = int(current_generation)
        msg.absolute_state = int(state)
        msg.decision = int(decision)
        msg.committed = bool(committed)
        msg.command_held = bool(command_held)
        msg.controls_wbc = bool(
            (committed or command_held) and not self.enable_velocity_ik_takeover
        )
        msg.source_hold_latched = bool(source_hold_latched)
        msg.fault_hold_pending = bool(fault_hold_pending)
        msg.trust_region_retry_used = bool(trust_region_retry_used)
        msg.relaxed_retry_used = bool(relaxed_retry_used)
        msg.hard_local_hold_active = bool(
            self._absolute_execution_safety.hold_active
        )
        msg.shadow_isolation_active = bool(shadow_isolation_active)
        msg.command_rate_limited = bool(command_rate_limited)
        msg.hard_recovery_success_count = int(
            self._absolute_execution_safety.recovery_success_count
        )
        msg.hard_recovery_required = int(
            self.absolute_hard_recovery_successes
        )
        msg.trusted_max_joint_offset = float(trusted_max_joint_offset)
        msg.min_joint_limit_margin = float(min_joint_limit_margin)
        msg.max_joint_step = float(max_joint_step)
        msg.max_joint_index = int(max_joint_index)
        msg.joint_limit_count = int(joint_limit_count)
        msg.joint_step_limit = self.absolute_max_joint_step
        msg.trust_region_step = self.absolute_ik_trust_region_step
        msg.pre_retry_max_joint_step = float(pre_retry_max_joint_step)
        msg.pre_retry_max_joint_index = int(pre_retry_max_joint_index)
        msg.discontinuity_count = discontinuity_count
        msg.consecutive_failure_count = consecutive_failure_count
        if previous_q is not None:
            msg.previous_q_rad = np.asarray(previous_q, dtype=float).reshape(-1).tolist()
        if candidate_q is not None:
            msg.candidate_q_rad = np.asarray(candidate_q, dtype=float).reshape(-1).tolist()
        self.pub_absolute_ik_commit_debug.publish(msg)

    def _absolute_joint_limit_count(self, candidate_q):
        if self.__q_lb is None or self.__q_ub is None:
            return 0
        candidate = np.asarray(candidate_q, dtype=float).reshape(-1)
        lower = np.asarray(self.__q_lb, dtype=float).reshape(-1)
        upper = np.asarray(self.__q_ub, dtype=float).reshape(-1)
        if candidate.size != lower.size or candidate.size != upper.size:
            return 0
        return int(np.count_nonzero(
            (np.abs(candidate - lower) <= 1.0e-5)
            | (np.abs(candidate - upper) <= 1.0e-5)
        ))

    def _publish_absolute_position_hold(
            self, generation, target, q_last, decision, candidate_q=None,
            trust_region_retry_used=False,
            pre_retry_max_joint_step=0.0,
            pre_retry_max_joint_index=0,
            shadow_isolation_active=False,
            trusted_max_joint_offset=0.0,
            min_joint_limit_margin=float("inf"),
            count_failure=True):
        """Refresh the last safe q command without accepting a new IK branch."""
        held_q = np.asarray(q_last[-self.__arm_dof:], dtype=float).copy()
        filtered_msg = Float32MultiArray()
        filtered_msg.data = held_q * 180.0 / np.pi
        self.pub_filtered_joint.publish(filtered_msg)
        self.publish_joint_states(q_now=held_q, q_last=q_last)

        candidate = self._absolute_vector(candidate_q, self.__arm_dof)
        joint_steps = (
            np.abs(candidate - held_q)
            if candidate is not None else np.array([], dtype=float)
        )
        max_joint_index = int(np.argmax(joint_steps)) if joint_steps.size else 0
        max_joint_step = (
            float(joint_steps[max_joint_index]) if joint_steps.size else 0.0
        )
        joint_limit_count = (
            self._absolute_joint_limit_count(candidate)
            if candidate is not None else 0
        )
        with self._absolute_lock:
            if count_failure:
                self._absolute_ik_consecutive_failure_count += 1
            current_generation = self._absolute_generation
            state = self._absolute_state
            source_hold_latched = self._tracking_source_hold_latched
            fault_hold_pending = self._absolute_fault_hold_pending
        self._publish_absolute_ik_commit_debug(
            decision,
            target,
            generation,
            current_generation,
            held_q,
            candidate,
            max_joint_step=max_joint_step,
            max_joint_index=max_joint_index,
            joint_limit_count=joint_limit_count,
            committed=False,
            state=state,
            source_hold_latched=source_hold_latched,
            fault_hold_pending=fault_hold_pending,
            trust_region_retry_used=trust_region_retry_used,
            relaxed_retry_used=False,
            pre_retry_max_joint_step=pre_retry_max_joint_step,
            pre_retry_max_joint_index=pre_retry_max_joint_index,
            command_held=True,
            shadow_isolation_active=shadow_isolation_active,
            trusted_max_joint_offset=trusted_max_joint_offset,
            min_joint_limit_margin=min_joint_limit_margin,
        )

    def _commit_absolute_preload(self, generation, target, q_seed):
        arm_q = np.asarray(q_seed[-self.__arm_dof:], dtype=float).copy()
        committed = False
        with self._absolute_lock:
            current_generation = self._absolute_generation
            state = self._absolute_state
            valid = (
                generation == current_generation
                and state == AbsoluteEnableState.SWITCHING
                and not self._tracking_source_hold_latched
                and not self._absolute_fault_hold_pending
            )
            if valid:
                # The operational velocity-IK path synchronizes its reference
                # directly from measured q/dq while the contract is ARMED.  A
                # Python q-only preload would be redundant and would re-create
                # a second joint-command path.  Legacy position IK keeps its
                # original preload behavior when takeover is explicitly off.
                if self.position_ik_control_enabled:
                    self.publish_joint_states(q_now=q_seed, q_last=q_seed)
                    self._publish_mm_absolute_target(target)
                committed = True
            source_hold_latched = self._tracking_source_hold_latched
            fault_hold_pending = self._absolute_fault_hold_pending
        decision = (
            AbsoluteIkCommitDebug.PRELOAD_COMMITTED
            if committed
            else AbsoluteIkCommitDebug.REJECTED_STALE_GENERATION
            if generation != current_generation
            else AbsoluteIkCommitDebug.REJECTED_INACTIVE_STATE
        )
        self._publish_absolute_ik_commit_debug(
            decision, target, generation, current_generation,
            arm_q, arm_q, committed=committed, state=state,
            source_hold_latched=source_hold_latched,
            fault_hold_pending=fault_hold_pending,
        )
        return committed

    def _commit_absolute_ik_result(
            self, generation, target, q_last, candidate_q,
            trust_region_retry_used=False, relaxed_retry_used=False,
            pre_retry_max_joint_step=0.0, pre_retry_max_joint_index=0,
            command_rate_limited=False, trusted_max_joint_offset=0.0,
            min_joint_limit_margin=float("inf")):
        previous_arm_q = np.asarray(q_last[-self.__arm_dof:], dtype=float).copy()
        try:
            candidate_arm_q = np.asarray(candidate_q, dtype=float).reshape(-1).copy()
        except (TypeError, ValueError):
            candidate_arm_q = np.array([], dtype=float)
        candidate_valid = (
            candidate_arm_q.size == previous_arm_q.size
            and np.all(np.isfinite(candidate_arm_q))
        )
        joint_steps = (
            np.abs(candidate_arm_q - previous_arm_q)
            if candidate_valid else np.array([], dtype=float)
        )
        max_joint_index = int(np.argmax(joint_steps)) if joint_steps.size else 0
        max_joint_step = float(joint_steps[max_joint_index]) if joint_steps.size else 0.0
        joint_limit_count = (
            self._absolute_joint_limit_count(candidate_arm_q) if candidate_valid else 0
        )
        committed = False
        rejected_discontinuity = False
        with self._absolute_lock:
            current_generation = self._absolute_generation
            state = self._absolute_state
            if generation != current_generation:
                decision = AbsoluteIkCommitDebug.REJECTED_STALE_GENERATION
            elif (state not in (AbsoluteEnableState.SWITCHING, AbsoluteEnableState.ACTIVE)
                    or self._tracking_source_hold_latched
                    or self._absolute_fault_hold_pending
                    or not self._absolute_gesture_running):
                decision = AbsoluteIkCommitDebug.REJECTED_INACTIVE_STATE
            elif not candidate_valid:
                decision = AbsoluteIkCommitDebug.REJECTED_INVALID_CANDIDATE
                self._absolute_ik_consecutive_failure_count += 1
            elif (self.enable_absolute_discontinuity_guard
                    and state == AbsoluteEnableState.ACTIVE
                    and max_joint_step > self.absolute_max_joint_step):
                decision = AbsoluteIkCommitDebug.REJECTED_JOINT_DISCONTINUITY
                self._absolute_ik_discontinuity_count += 1
                self._absolute_ik_consecutive_failure_count += 1
                rejected_discontinuity = True
            else:
                decision = AbsoluteIkCommitDebug.COMMITTED
                filtered_msg = Float32MultiArray()
                filtered_msg.data = candidate_arm_q * 180.0 / np.pi
                self.pub_filtered_joint.publish(filtered_msg)
                self.publish_joint_states(q_now=candidate_arm_q, q_last=q_last)
                committed = True
                self._absolute_ik_consecutive_failure_count = 0
            source_hold_latched = self._tracking_source_hold_latched
            fault_hold_pending = self._absolute_fault_hold_pending

        self._publish_absolute_ik_commit_debug(
            decision, target, generation, current_generation,
            previous_arm_q, candidate_arm_q,
            max_joint_step=max_joint_step,
            max_joint_index=max_joint_index,
            joint_limit_count=joint_limit_count,
            committed=committed,
            state=state,
            source_hold_latched=source_hold_latched,
            fault_hold_pending=fault_hold_pending,
            trust_region_retry_used=trust_region_retry_used,
            relaxed_retry_used=relaxed_retry_used,
            pre_retry_max_joint_step=pre_retry_max_joint_step,
            pre_retry_max_joint_index=pre_retry_max_joint_index,
            command_rate_limited=command_rate_limited,
            trusted_max_joint_offset=trusted_max_joint_offset,
            min_joint_limit_margin=min_joint_limit_margin,
        )
        if rejected_discontinuity:
            rospy.logwarn(
                "[IkRos][TrackingSafety] IK joint discontinuity rejected: joint %d "
                "changed %.3frad (limit %.3frad); keeping the previous command and "
                "retrying locally without leaving mode 2",
                max_joint_index + 1,
                max_joint_step,
                self.absolute_max_joint_step,
            )
            self._publish_absolute_enable_status()
        elif decision == AbsoluteIkCommitDebug.REJECTED_INVALID_CANDIDATE:
            rospy.logwarn_throttle(
                1.0,
                "[IkRos][TrackingSafety] Invalid IK candidate ignored; keeping the "
                "previous command and retrying without leaving mode 2",
            )
        return committed

    def _publish_absolute_ik_solve_failed(
            self, generation, target, q_last, trust_region_retry_used=False,
            relaxed_retry_used=False, pre_retry_max_joint_step=0.0,
            pre_retry_max_joint_index=0):
        with self._absolute_lock:
            self._absolute_ik_consecutive_failure_count += 1
            current_generation = self._absolute_generation
            state = self._absolute_state
            source_hold_latched = self._tracking_source_hold_latched
            fault_hold_pending = self._absolute_fault_hold_pending
        self._publish_absolute_ik_commit_debug(
            AbsoluteIkCommitDebug.SOLVE_FAILED,
            target,
            generation,
            current_generation,
            q_last[-self.__arm_dof:],
            None,
            state=state,
            source_hold_latched=source_hold_latched,
            fault_hold_pending=fault_hold_pending,
            trust_region_retry_used=trust_region_retry_used,
            relaxed_retry_used=relaxed_retry_used,
            pre_retry_max_joint_step=pre_retry_max_joint_step,
            pre_retry_max_joint_index=pre_retry_max_joint_index,
        )

    def _reject_absolute_capture(self, reason, message, expected_generation=None):
        with self._absolute_lock:
            if (expected_generation is not None
                    and (expected_generation != self._absolute_generation
                         or self._absolute_state != AbsoluteEnableState.CAPTURED)):
                rospy.logwarn(
                    "[IkRos][AbsoluteEnable] Ignored stale capture rejection from generation %d; current=%d",
                    expected_generation,
                    self._absolute_generation,
                )
                return
            self._bump_absolute_generation_locked()
            rejection_state = AbsoluteEnableState.WAIT_RELEASE
            if self._absolute_pending_capture is not None:
                rejection_state = self._absolute_pending_capture.get(
                    "rejection_state", AbsoluteEnableState.WAIT_RELEASE
                )
            self._absolute_state = (
                rejection_state
                if self._absolute_gesture_running
                else AbsoluteEnableState.WAITING
            )
            self._absolute_reason = reason
            self._absolute_pending_capture = None
            self._absolute_frozen_target = None
            self._absolute_resume_request_time = None
        rospy.logwarn("[IkRos][AbsoluteEnable] %s", message)
        self._publish_absolute_enable_status()

    def _pause_absolute_session(self):
        """Start the velocity-IK-owned return to the ready arm posture."""
        source_fault_cleared = False
        with self._absolute_lock:
            if hasattr(self, '_torso_controls'):
                self._torso_controls.reset()
            self._bump_absolute_generation_locked()
            if (self._tracking_source_hold_latched
                    and self._tracking_can_rearm_locked()):
                self._clear_tracking_fault_locked()
                source_fault_cleared = True
            self._absolute_state = AbsoluteEnableState.RETURNING_TO_PREP
            self._absolute_reason = AbsoluteEnableReason.PAUSE_REQUEST
            self._absolute_pending_capture = None
            self._absolute_frozen_target = None
            self._absolute_stop_pending = False
            self._absolute_pause_pending = False
            self._absolute_external_mode_reset_pending = False
            self._absolute_mode_request_time = None
            self._absolute_switch_start_time = None
            self._absolute_resume_request_time = None
            self._absolute_resume_after_prepare = False
            self._absolute_prepare_complete = False
        # The 21-D ReferenceManager remains the sole mode-2 owner.  No WBC
        # mode transition is expected while the ready-pose trajectory runs.
        self.arm_mode_changing = False
        rospy.loginfo(
            "[IkRos][AbsoluteEnable] X+A started whole-body velocity-IK ready-pose return for session %d%s",
            self._absolute_session_id,
            "; source fault acknowledged" if source_fault_cleared else "",
        )
        self._publish_absolute_enable_status()

    def _finish_absolute_session(self, reason, request_hold, message):
        with self._absolute_lock:
            if hasattr(self, '_torso_controls'):
                self._torso_controls.reset()
            if self._fixed_pending_operation is not None:
                self._fixed_pending_operation = None
                self._fixed_operation_phase = 5
                self._fixed_operation_block = True
                self._fixed_prepare_bootstrap = False
            self._bump_absolute_generation_locked()
            self._absolute_state = (
                AbsoluteEnableState.WAIT_RELEASE
                if self._absolute_gesture_running
                else AbsoluteEnableState.WAITING
            )
            self._absolute_reason = reason
            self._absolute_pending_capture = None
            self._absolute_frozen_target = None
            self._absolute_stop_pending = False
            self._absolute_pause_pending = False
            self._absolute_external_mode_reset_pending = False
            self._absolute_mode_request_time = None
            self._absolute_switch_start_time = None
            self._absolute_resume_request_time = None
            self._absolute_resume_after_prepare = False
            self._absolute_prepare_complete = False
            self._tracking_control_running = False
            self._tracking_source_interrupted = False
            if not self._absolute_gesture_running:
                self._reset_tracking_session_source_locked()
        self.arm_mode_changing = False
        rospy.logwarn("[IkRos][AbsoluteEnable] %s", message)
        if request_hold and self.__send_srv:
            if not self._request_absolute_hold_mode():
                with self._absolute_lock:
                    self._absolute_fault_hold_pending = True
                    self._fixed_operation_error = True
                    self._absolute_fault_hold_reason = AbsoluteEnableReason.MODE_SERVICE_FAILED
        self._publish_absolute_enable_status()

    def _begin_absolute_session(self):
        now = time.monotonic()
        with self._absolute_lock:
            capture = self._absolute_pending_capture
            gesture_running = self._absolute_gesture_running or self._fixed_prepare_bootstrap
            capture_state = self._absolute_state
            capture_generation = (
                None if capture is None else capture.get("generation")
            )
        if capture is None and capture_state != AbsoluteEnableState.CAPTURED:
            return None, None, None
        if capture is None or not gesture_running:
            self._reject_absolute_capture(
                AbsoluteEnableReason.VR_TARGET_INVALID,
                "Enable capture disappeared or gesture was released",
                expected_generation=capture_generation,
            )
            return None, None, None
        if not self._wheel_mpc_stable_for_mm_cmd():
            self._reject_absolute_capture(
                AbsoluteEnableReason.MPC_NOT_READY,
                "Rejected enable: wheel MPC internal time is not ready; release and retry",
                expected_generation=capture_generation,
            )
            return None, None, None

        target = capture["target"]
        measured_q = capture["q"]
        target_age = float("inf") if target is None else now - target["receive_time"]
        q_age = float("inf") if capture["q_time"] is None else now - capture["q_time"]
        with self._absolute_lock:
            self._absolute_capture_ages = [target_age, q_age]

        if target is None:
            self._reject_absolute_capture(
                AbsoluteEnableReason.VR_TARGET_INVALID,
                "Rejected enable: the triggering Quest frame was not a complete valid target",
                expected_generation=capture_generation,
            )
            return None, None, None
        if target_age > self.absolute_enable_max_vr_age:
            self._reject_absolute_capture(
                AbsoluteEnableReason.VR_TARGET_STALE,
                "Rejected enable: captured VR target is {:.1f} ms old".format(target_age * 1000.0),
                expected_generation=capture_generation,
            )
            return None, None, None
        if measured_q is None:
            self._reject_absolute_capture(
                AbsoluteEnableReason.JOINT_STATE_INVALID,
                "Rejected enable: no complete measured 14-DoF arm state",
                expected_generation=capture_generation,
            )
            return None, None, None
        if q_age > self.absolute_enable_max_joint_state_age:
            self._reject_absolute_capture(
                AbsoluteEnableReason.JOINT_STATE_STALE,
                "Rejected enable: captured arm measurement is {:.1f} ms old".format(q_age * 1000.0),
                expected_generation=capture_generation,
            )
            return None, None, None

        q_seed = self._compose_absolute_q_seed(measured_q)
        if q_seed is None or not np.all(np.isfinite(q_seed)):
            self._reject_absolute_capture(
                AbsoluteEnableReason.JOINT_STATE_INVALID,
                "Rejected enable: measured arm state cannot initialize the IK model",
                expected_generation=capture_generation,
            )
            return None, None, None

        if self.enable_velocity_ik_takeover:
            # Alignment is owned by the 21-D solver and captured from measured
            # FK in the same q/dq domain. The Python-side ArmIk comparison was
            # only a legacy start-gap diagnostic and is deliberately absent
            # from the takeover process.
            alignment_errors = [(0.0, 0.0), (0.0, 0.0)]
        else:
            try:
                alignment_errors = self._calculate_absolute_alignment_errors(
                    q_seed, target
                )
            except Exception as error:
                self._reject_absolute_capture(
                    AbsoluteEnableReason.JOINT_STATE_INVALID,
                    "Rejected enable: measured-state FK failed: {}".format(error),
                    expected_generation=capture_generation,
                )
                return None, None, None

        left_pos, left_ori = alignment_errors[0]
        right_pos, right_ori = alignment_errors[1]
        with self._absolute_lock:
            if (capture_generation != self._absolute_generation
                    or self._absolute_state != AbsoluteEnableState.CAPTURED
                    or self._absolute_pending_capture is not capture):
                return None, None, None
            self._absolute_alignment_errors = [left_pos, right_pos, left_ori, right_ori]

        position_failed = any(
            self._arm_is_controlled(index)
            and alignment_errors[index][0] > self.absolute_enable_max_position_error
            for index in range(2)
        )
        if self.enable_absolute_start_gap_check and position_failed:
            self._reject_absolute_capture(
                AbsoluteEnableReason.POSITION_ERROR,
                "Rejected enable: wrist-center position error L={:.3f}m R={:.3f}m, limit={:.3f}m".format(
                    left_pos, right_pos, self.absolute_enable_max_position_error
                ),
                expected_generation=capture_generation,
            )
            return None, None, None

        orientation_failed = any(
            self._arm_is_controlled(index)
            and alignment_errors[index][1] > self.absolute_enable_max_orientation_error
            for index in range(2)
        )
        if self.enable_absolute_start_gap_check and orientation_failed:
            self._reject_absolute_capture(
                AbsoluteEnableReason.ORIENTATION_ERROR,
                "Rejected enable: EEF orientation error L={:.3f}rad R={:.3f}rad, limit={:.3f}rad".format(
                    left_ori, right_ori, self.absolute_enable_max_orientation_error
                ),
                expected_generation=capture_generation,
            )
            return None, None, None

        frozen_target = self._clone_absolute_target(target)
        with self._absolute_lock:
            if (capture_generation != self._absolute_generation
                    or self._absolute_state != AbsoluteEnableState.CAPTURED
                    or self._absolute_pending_capture is not capture
                    or not (self._absolute_gesture_running or self._fixed_prepare_bootstrap)
                    or self._tracking_source_hold_latched
                    or self._absolute_fault_hold_pending):
                return None, None, None
            if self.position_ik_control_enabled:
                self._reset_absolute_position_filters(frozen_target)
            if (self.position_ik_control_enabled
                    and hasattr(self.arm_ik, "reset_last_solution")):
                self.arm_ik.reset_last_solution(q_seed)
            self._absolute_frozen_target = frozen_target
            self._absolute_state = AbsoluteEnableState.SWITCHING
            self._absolute_reason = capture.get(
                "reason", AbsoluteEnableReason.ENABLE_EDGE
            )
            # 先完成测量状态锚定，下一周期再请求 mode 2。
            self._absolute_switch_start_time = None
            self._absolute_mode_request_time = None
            self._absolute_pending_capture = None
        self.arm_mode_changing = True
        self._publish_absolute_captured_q(measured_q)
        if not self._commit_absolute_preload(
                capture_generation, frozen_target, q_seed):
            return None, None, None
        self._publish_absolute_enable_status()

        rospy.loginfo(
            "[IkRos][AbsoluteEnable] Session %d accepted; measured-state anchor "
            "prepared before mode request (%s)",
            self._absolute_session_id,
            "velocity IK" if self.enable_velocity_ik_takeover else "position IK",
        )
        return (
            frozen_target,
            None if self.enable_velocity_ik_takeover else q_seed,
            capture_generation,
        )

    def _absolute_control_cycle(self):
        """Return (control_target, optional_q_seed, transactional generation)."""
        if not self.enable_absolute_target_alignment:
            return None, None, None

        self._advance_fixed_request()

        with self._absolute_lock:
            state = self._absolute_state
            stop_pending = self._absolute_stop_pending
            pause_pending = self._absolute_pause_pending
            external_reset_pending = self._absolute_external_mode_reset_pending
            fault_hold_pending = self._absolute_fault_hold_pending
            fault_hold_reason = self._absolute_fault_hold_reason
            generation = self._absolute_generation
        if fault_hold_pending:
            self.arm_mode_changing = False
            hold_ok = (not self.__send_srv) or self._request_absolute_hold_mode()
            with self._absolute_lock:
                # Keep the latch pending while the blocking service calls are in
                # flight.  Otherwise a concurrent X+A callback could authorize
                # a new session before mode 0 has actually taken control.  A
                # failed hold request remains pending and is retried by the IK
                # thread; safety faults must not silently fall back to the last
                # (possibly corrupted) mode-2 reference.
                if hold_ok:
                    self._absolute_fault_hold_pending = False
                    self._fixed_operation_error = False
                    self._absolute_fault_hold_reason = AbsoluteEnableReason.NONE
            rospy.logwarn(
                "[IkRos][AbsoluteEnable] Safety fault %d cancelled generation %d; "
                "mode-0 measured hold request %s",
                int(fault_hold_reason),
                generation,
                "succeeded" if hold_ok else "failed",
            )
            self._publish_absolute_enable_status()
            return None, None, None
        if stop_pending:
            self._finish_absolute_session(
                AbsoluteEnableReason.STOP_GESTURE,
                request_hold=not self.enable_velocity_ik_takeover,
                message=(
                    "Stop gesture received; whole-body velocity IK is braking to hold"
                    if self.enable_velocity_ik_takeover
                    else "Stop gesture received; returning arm control to hold mode"
                ),
            )
            return None, None, None
        if pause_pending:
            self._pause_absolute_session()
            return None, None, None
        if external_reset_pending:
            self._finish_absolute_session(
                AbsoluteEnableReason.EXTERNAL_MODE_CHANGE,
                request_hold=False,
                message="External arm-mode change ended the absolute VR session",
            )
            return None, None, None
        if state == AbsoluteEnableState.RESUME_PENDING:
            now = time.monotonic()
            publish_status = False
            resume_ready = False
            resume_session = None
            with self._absolute_lock:
                current_mode, target_mode, transitioning = self._absolute_mode_status
                resume_request_time = self._absolute_resume_request_time
                if not self._absolute_gesture_running:
                    self._bump_absolute_generation_locked()
                    self._absolute_state = AbsoluteEnableState.WAITING
                    self._absolute_reason = AbsoluteEnableReason.NONE
                    self._absolute_resume_request_time = None
                    publish_status = True
                elif (resume_request_time is None
                        or now - resume_request_time > self.absolute_enable_mode_timeout):
                    self._bump_absolute_generation_locked()
                    self._absolute_state = AbsoluteEnableState.PAUSED
                    self._absolute_reason = AbsoluteEnableReason.MODE_TIMEOUT
                    self._absolute_resume_request_time = None
                    publish_status = True
                elif (current_mode, target_mode, transitioning) == (1, 1, False):
                    self._queue_absolute_capture_locked(
                        AbsoluteEnableReason.RESUME_REQUEST,
                        AbsoluteEnableState.PAUSED,
                    )
                    resume_ready = True
                    resume_session = self._absolute_session_id
                    publish_status = True
            if resume_ready:
                rospy.loginfo(
                    "[IkRos][AbsoluteEnable] Session %d captured after pause mode became ready",
                    resume_session,
                )
            elif publish_status:
                rospy.logwarn(
                    "[IkRos][AbsoluteEnable] X+A resume was cancelled or timed out before mode 1 became ready"
                )
            if publish_status:
                self._publish_absolute_enable_status()
            return None, None, None
        if state == AbsoluteEnableState.CAPTURED:
            return self._begin_absolute_session()
        if state in (
            AbsoluteEnableState.WAITING,
            AbsoluteEnableState.WAIT_RELEASE,
            AbsoluteEnableState.PAUSED,
            AbsoluteEnableState.RETURNING_TO_PREP,
            AbsoluteEnableState.PREPARED,
        ):
            return None, None, None

        now = time.monotonic()
        with self._absolute_lock:
            latest_target = self._clone_absolute_target(self._absolute_latest_target)
            frozen_target = self._clone_absolute_target(self._absolute_frozen_target)
            current_mode, target_mode, transitioning = self._absolute_mode_status
            mode_status_time = self._absolute_mode_status_time
            mode_request_time = self._absolute_mode_request_time
            switch_start_time = self._absolute_switch_start_time
            generation = self._absolute_generation
        latest_target_age = (
            float("inf") if latest_target is None else now - latest_target["receive_time"]
        )
        if latest_target_age > self.absolute_enable_max_vr_age:
            # Runtime input loss is recoverable: keep mode 2 and the last committed
            # reference.  A fresh configured-source frame will replace it without
            # requiring another trigger/X+A sequence.
            latest_target = latest_target if latest_target is not None else frozen_target
            rospy.logwarn_throttle(
                1.0,
                "[IkRos][AbsoluteEnable] Valid VR target is %.1fms old; "
                "holding the last reference and waiting for automatic recovery",
                latest_target_age * 1000.0,
            )
            if state == AbsoluteEnableState.ACTIVE:
                return latest_target, None, generation

        if state == AbsoluteEnableState.SWITCHING:
            if mode_request_time is None:
                with self._absolute_lock:
                    self._absolute_switch_start_time = time.monotonic()
                if not self.__send_srv or not self._request_absolute_external_mode(generation):
                    with self._absolute_lock:
                        transaction_stale = generation != self._absolute_generation
                    if transaction_stale:
                        return None, None, None
                    self._finish_absolute_session(
                        AbsoluteEnableReason.MODE_SERVICE_FAILED,
                        request_hold=self.__send_srv,
                        message="Mode-2 request failed; frozen target was not released",
                    )
                    return None, None, None
                rospy.loginfo(
                    "[IkRos][AbsoluteEnable] Session %d mode 2 requested; target remains frozen",
                    self._absolute_session_id,
                )
                return frozen_target, None, generation

            with self._absolute_lock:
                switch_start_time = self._absolute_switch_start_time
            if switch_start_time is None or now - switch_start_time > self.absolute_enable_mode_timeout:
                self._finish_absolute_session(
                    AbsoluteEnableReason.MODE_TIMEOUT,
                    request_hold=True,
                    message="Timed out waiting for /humanoid_wheel/arm_control_mode_status=[2,2,0]",
                )
                return None, None, None

            status_after_request = (
                mode_request_time is not None
                and mode_status_time is not None
                and mode_status_time >= mode_request_time
            )
            if status_after_request and (current_mode, target_mode, transitioning) == (2, 2, False):
                with self._absolute_lock:
                    if (generation != self._absolute_generation
                            or self._absolute_state != AbsoluteEnableState.SWITCHING
                            or self._tracking_source_hold_latched):
                        return None, None, None
                    self._absolute_state = (AbsoluteEnableState.RETURNING_TO_PREP
                                            if self._fixed_prepare_bootstrap else AbsoluteEnableState.ACTIVE)
                    self._absolute_reason = AbsoluteEnableReason.MODE_READY
                    self._fixed_prepare_bootstrap = False
                self.arm_mode_changing = False
                rospy.loginfo(
                    "[IkRos][AbsoluteEnable] Session %d mode ready; releasing live VR target",
                    self._absolute_session_id,
                )
                self._publish_absolute_enable_status()
                return latest_target, None, generation
            if status_after_request and target_mode != 2:
                self._finish_absolute_session(
                    AbsoluteEnableReason.EXTERNAL_MODE_CHANGE,
                    request_hold=False,
                    message="Mode target changed away from 2 during frozen takeover",
                )
                return None, None, None
            return frozen_target, None, generation

        if state == AbsoluteEnableState.ACTIVE:
            if (current_mode, target_mode, transitioning) != (2, 2, False):
                self._finish_absolute_session(
                    AbsoluteEnableReason.EXTERNAL_MODE_CHANGE,
                    request_hold=False,
                    message="Arm mode left [2,2,0]; live VR target was blocked",
                )
                return None, None, None
            return latest_target, None, generation
        return None, None, None

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
        Limit per-cycle joint changes using velocity limits in deg/s.
        The first joint of each arm uses the separate shoulder velocity limit.
        """
        size = len(q_now)
        q_limited = q_now.copy()
        agl_limit = self.controller_dt * vel_limit * np.pi / 180.0  # deg/s to rad/s

        shoulder_step_limit = self.controller_dt * shoulder_vel_limit * np.pi / 180.0

        for i in range(size):
            # 速度限制
            q_limited[i] = max(q_last[i] - agl_limit, min(q_now[i], q_last[i] + agl_limit))

            if i == 0:  # 左臂第一个关节 (l_arm_pitch)
                q_limited[i] = max(q_last[i]-shoulder_step_limit, min(q_now[i], q_last[i] + shoulder_step_limit))
            elif i == self.__single_arm_dof:  # 右臂第一个关节 (r_arm_pitch)，索引7
                q_limited[i] = max(q_last[i]-shoulder_step_limit, min(q_now[i], q_last[i] + shoulder_step_limit))

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

    def _run_velocity_ik_takeover_supervisor(self, rate):
        """Run Quest/session supervision without executing position IK."""
        self.__ik_claw_publish_active = True
        rospy.loginfo(
            "[IkRos] Velocity-IK-only chain active: Python position solve, "
            "two-stage retry, q-only filtering and q-only publishing are disabled"
        )
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            self.hand_finger_data_process(0)
            # This preserves capture, mode-2 handshake, pause/stop, source lock
            # and automatic input recovery.  The returned target/q seed belong
            # to the legacy solver and are deliberately not consumed here.
            self._absolute_control_cycle()
            rate.sleep()

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
        if self.enable_absolute_target_alignment:
            print(
                "[ik]: Absolute enable alignment is active; waiting for a fresh aligned OK-gesture capture."
            )
        elif self.__as_mc_ik:
            print("[ik]: Waiting for OK-guesture(hold on for 1-2 seconds) to start teleoperation...")
            while not self.quest3_arm_info_transformer.is_runing:
                self.hand_finger_data_process(0)
                if self.stop_event.is_set():
                    print("[ik]: Stop event is set, exit.")
                    return
                if(self.quest3_arm_info_transformer.check_if_vr_error()):
                    sys.stdout.write("\r\033[91mDetected VR ERROR!!! Please restart VR app in quest3 or check the battery level of the joystick!!!\033[0m")
                rate.sleep()
            print("[ik]: OK-guesture recieved!!!")

        if self.__send_srv and not self.enable_absolute_target_alignment:
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

        self.arm_mode_changing = not self.enable_absolute_target_alignment
        if self.enable_velocity_ik_takeover:
            self._run_velocity_ik_takeover_supervisor(rate)
            return
        if self.__as_mc_ik:
            print("[ik]: If you want to stop teleoperation, please make a Shot-guesture(hold on for 1-2 seconds).")
        q_last = np.asarray(
            self.arm_ik.q0() if self.external_q0 is None else self.external_q0,
            dtype=float,
        ).copy()
        self._absolute_execution_safety.reset(q_last[-self.__arm_dof:])
        pre_q_first = q_last.copy()
        velocity_limit_shadow_last = np.array(q_last[-self.__arm_dof:]).copy()
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
        while not rospy.is_shutdown():
            # 检测是否需要重置IK初始猜测（模式切换时）
            if self.__need_reset_ik_guess:
                if self.enable_absolute_target_alignment:
                    # 绝对启用状态机只允许使用捕获时测量状态初始化；禁止零位重置。
                    self.__need_reset_ik_guess = False
                else:
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
                    velocity_limit_shadow_last = np.array(arm_q_filtered).copy()
                    self._absolute_execution_safety.reset(arm_q_filtered)

                    self.__need_reset_ik_guess = False
            self.hand_finger_data_process(0)

            control_target = None
            solve_generation = None
            if self.enable_absolute_target_alignment:
                control_target, captured_q_seed, solve_generation = (
                    self._absolute_control_cycle()
                )
                if captured_q_seed is not None:
                    q_last = captured_q_seed.copy()
                    pre_q_first = captured_q_seed.copy()
                    q_now = captured_q_seed.copy()
                    arm_q_filtered = captured_q_seed[-self.__arm_dof:].copy()
                    velocity_limit_shadow_last = arm_q_filtered.copy()
                    self._absolute_execution_safety.reset(arm_q_filtered)
                is_runing_last = is_runing
                is_runing = control_target is not None
            else:
                # Legacy path still obeys the same source-hold latch.  Keeping
                # the last q reference is safer than continuously resolving a
                # stale target after a source handoff.
                is_runing_last = is_runing
                with self._absolute_lock:
                    is_runing = not self._tracking_source_hold_latched
            self.__current_pose, self.__current_pose_right = self.get_two_arm_pose(q_last)
            self.pub_solved_arm_eef_pose(q_last, self.__current_pose, self.__current_pose_right)
            if self.trigger_reset_mode:
                if not self.enable_absolute_target_alignment:
                    self.__target_pose = (None, None)
                    self.__current_pose = (None, None)
                    self.__target_pose_right = (None, None)
                    self.__current_pose_right = (None, None)
                    q_last = pre_q_first.copy()
                    velocity_limit_shadow_last = np.array(q_last[-self.__arm_dof:]).copy()
                self.trigger_reset_mode = False

            if(not self.enable_absolute_target_alignment and self.__as_mc_ik
                    and self.quest3_arm_info_transformer.check_if_vr_error()):
                rate.sleep()
                print("\033[91mDetected VR ERROR!!! Please restart VR app in quest3 or check the battery level of the joystick!!!\.\033[0m")
                continue
            elif(self.__as_mc_ik and (not is_runing)):
                rate.sleep()
                if not self.enable_absolute_target_alignment:
                    sys.stdout.write("\rStatus0: {}, is target far?: {}".format("RUNING" if is_runing else "STOPED", self.judge_target_is_far()))
                continue

            if(not is_runing_last and is_runing):
                self.arm_mode_changing = True
            if self.enable_absolute_target_alignment:
                if control_target is None:
                    rate.sleep()
                    continue
                if self.whole_body_shadow_isolate_position_ik:
                    held_arm_q = np.asarray(
                        q_last[-self.__arm_dof:], dtype=float
                    ).copy()
                    self._absolute_execution_safety.reset(held_arm_q)
                    self._publish_absolute_position_hold(
                        solve_generation,
                        control_target,
                        q_last,
                        AbsoluteIkCommitDebug.SHADOW_ISOLATION_HOLD,
                        candidate_q=held_arm_q,
                        shadow_isolation_active=True,
                        count_failure=False,
                    )
                    q_now = q_last.copy()
                    arm_q_filtered = held_arm_q
                    velocity_limit_shadow_last = held_arm_q.copy()
                    rospy.loginfo_throttle(
                        5.0,
                        "[IkRos][WholeBodyShadow] Position IK is holding the "
                        "captured arm command; Quest TCP targets are routed only "
                        "to the whole-body shadow solver",
                    )
                    rate.sleep()
                    continue
                target_pose_left = (control_target["left_pos"], control_target["left_quat"])
                target_pose_right = (control_target["right_pos"], control_target["right_quat"])
                target_left_elbow = control_target["left_elbow"]
                target_right_elbow = control_target["right_elbow"]
                target_left_shoulder_rpy = control_target["left_shoulder_rpy"]
                target_right_shoulder_rpy = control_target["right_shoulder_rpy"]
            else:
                target_pose_left = self.__target_pose
                target_pose_right = self.__target_pose_right
                target_left_elbow = self.__left_elbow_pos
                target_right_elbow = self.__right_elbow_pos
                target_left_shoulder_rpy = self.quest3_arm_info_transformer.left_shoulder_rpy_in_robot
                target_right_shoulder_rpy = self.quest3_arm_info_transformer.right_shoulder_rpy_in_robot

            if target_pose_left[0] is None or target_pose_right[0] is None or \
                self.__current_pose[0] is None or self.__current_pose_right[0] is None:
                rate.sleep()
                continue
            if self.arm_ik.type().name() == IkTypeIdx.TorsoIK.name():
                l_hand_pose, l_hand_RPY, l_hand_quat = None, None, None
                r_hand_pose, r_hand_RPY, r_hand_quat = None, None, None
                l_elbow_pos, r_elbow_pos = None, None
                left_shoulder_rpy_in_robot, right_shoulder_rpy_in_robot = None, None
                if target_pose_left[0] is not None and (self.__ctrl_arm_idx == ArmIdx.BOTH
                                                          or self.__ctrl_arm_idx == ArmIdx.LEFT):
                    l_hand_pose, l_hand_quat = target_pose_left
                    l_hand_pose = self.kf_left.filter(l_hand_pose)
                    l_hand_RPY = quaternion_to_RPY(l_hand_quat)
                    l_elbow_pos = target_left_elbow
                    # if l_elbow_pos is not None:
                    #     # print(f"l_elbow_pos: {l_elbow_pos}")
                    #     l_elbow_pos[0] = -0.3 if l_elbow_pos[0] < -0.3 else l_elbow_pos[0]
                    left_shoulder_rpy_in_robot = target_left_shoulder_rpy
                if target_pose_right[0] is not None and (self.__ctrl_arm_idx == ArmIdx.BOTH
                                                                or self.__ctrl_arm_idx == ArmIdx.RIGHT):
                    r_hand_pose, r_hand_quat = target_pose_right
                    r_hand_pose = self.kf_right.filter(r_hand_pose)
                    r_hand_RPY = quaternion_to_RPY(r_hand_quat)
                    r_elbow_pos = target_right_elbow
                    # if r_elbow_pos is not None:
                    #     r_elbow_pos[0] = -0.3 if r_elbow_pos[0] < -0.3 else r_elbow_pos[0]
                    right_shoulder_rpy_in_robot = target_right_shoulder_rpy

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


                active_joint_step_limit = None
                trusted_hard_target = None
                trusted_hard_reference = None
                if self.enable_absolute_target_alignment:
                    trusted_hard_target = (
                        self._absolute_execution_safety.trusted_target
                    )
                    with self._absolute_lock:
                        if (self._absolute_state == AbsoluteEnableState.ACTIVE
                                and self.absolute_ik_trust_region_step > 0.0
                                and trusted_hard_target is not None):
                            active_joint_step_limit = self.absolute_ik_trust_region_step
                            trusted_hard_reference = q_last.copy()
                            trusted_hard_reference[-self.__arm_dof:] = (
                                trusted_hard_target
                            )
                trust_region_retry_used = False
                relaxed_retry_used = False
                pre_retry_max_joint_step = 0.0
                pre_retry_max_joint_index = 0
                q_now = arm_ik.computeIK(
                    q0_tmp, l_hand_pose, r_hand_pose, l_hand_RPY, r_hand_RPY,
                    l_elbow_pos, r_elbow_pos, left_shoulder_rpy_in_robot,
                    right_shoulder_rpy_in_robot,
                    update_last_solution=not self.enable_absolute_target_alignment,
                )
                local_retry_needed = q_now is None
                if q_now is not None and active_joint_step_limit is not None:
                    candidate = np.asarray(q_now[-self.__arm_dof:], dtype=float).reshape(-1)
                    previous = np.asarray(
                        trusted_hard_target, dtype=float
                    ).reshape(-1)
                    if (candidate.size != previous.size
                            or not np.all(np.isfinite(candidate))):
                        local_retry_needed = True
                        pre_retry_max_joint_step = float("inf")
                    else:
                        pre_retry_steps = np.abs(candidate - previous)
                        if pre_retry_steps.size:
                            pre_retry_max_joint_index = int(np.argmax(pre_retry_steps))
                            pre_retry_max_joint_step = float(
                                pre_retry_steps[pre_retry_max_joint_index]
                            )
                        local_retry_needed = (
                            pre_retry_max_joint_step > active_joint_step_limit
                        )
                if local_retry_needed and active_joint_step_limit is not None:
                    trust_region_retry_used = True
                    q_now = arm_ik.computeIK(
                        q0_tmp, l_hand_pose, r_hand_pose, l_hand_RPY, r_hand_RPY,
                        l_elbow_pos, r_elbow_pos, left_shoulder_rpy_in_robot,
                        right_shoulder_rpy_in_robot,
                        joint_step_limit=active_joint_step_limit,
                        joint_step_reference=trusted_hard_reference,
                        update_last_solution=False,
                    )
                    if q_now is not None:
                        rospy.logwarn_throttle(
                            1.0,
                            "[IkRos][TrackingSafety] IK candidate failed or left the local branch "
                            "(pre-retry max step %.3frad); bounded retry succeeded",
                            pre_retry_max_joint_step,
                        )
                # A soft hand-pose result is not an executable fallback.  The
                # 2026-09-02 incident showed that a sequence of individually
                # bounded soft results can walk to an unrelated joint branch.
                # A failed hard local solve therefore enters an explicit hold.
                time_cost = time.time() - time_0
                if time_cost >= 10.0:
                    print(f"\033[91m The time-cost of ik is {1e3*time_cost:.2f} ms !!!\033[0m")
                self.pub_time_cost.publish(Float32(1e3 * time_cost))
                if q_now is None and self.enable_absolute_target_alignment:
                    self._absolute_execution_safety.enter_hold(
                        q_last[-self.__arm_dof:]
                    )
                    self._publish_absolute_position_hold(
                        solve_generation,
                        control_target,
                        q_last,
                        AbsoluteIkCommitDebug.HARD_LOCAL_HOLD,
                        candidate_q=None,
                        trust_region_retry_used=trust_region_retry_used,
                        pre_retry_max_joint_step=pre_retry_max_joint_step,
                        pre_retry_max_joint_index=pre_retry_max_joint_index,
                    )
                    rospy.logerr_throttle(
                        1.0,
                        "[IkRos][TrackingSafety] Hard local IK is infeasible; "
                        "holding the last trusted arm command. Return both hands "
                        "toward the last feasible pose to recover automatically.",
                    )
                    fail_count += 1
                    run_count += 1
                    sum_time_cost += time_cost
                    q_now = q_last.copy()
                    rate.sleep()
                    continue
                if q_now is not None:
                    msg = Float32MultiArray()
                    msg.data = q_now[-self.__arm_dof:] * 180.0 / np.pi
                    self.pub_origin_joint.publish(msg)

                    arm_q_filtered = self.limit_angle(q_now[-self.__arm_dof:])

                    hard_candidate_decision = None
                    if self.enable_absolute_target_alignment:
                        hard_candidate_decision = (
                            self._absolute_execution_safety.observe_hard_candidate(
                                arm_q_filtered,
                                q_last[-self.__arm_dof:],
                            )
                        )
                        if not hard_candidate_decision.accepted:
                            if hard_candidate_decision.reason == "joint_limit_margin":
                                hold_decision = (
                                    AbsoluteIkCommitDebug.REJECTED_JOINT_MARGIN
                                )
                            elif hard_candidate_decision.reason == "invalid_candidate":
                                hold_decision = (
                                    AbsoluteIkCommitDebug.REJECTED_INVALID_CANDIDATE
                                )
                            elif hard_candidate_decision.reason == "target_discontinuity":
                                hold_decision = (
                                    AbsoluteIkCommitDebug.REJECTED_HARD_TARGET_DISCONTINUITY
                                )
                            else:
                                hold_decision = (
                                    AbsoluteIkCommitDebug.RECOVERY_VALIDATING
                                )
                            self._publish_absolute_position_hold(
                                solve_generation,
                                control_target,
                                q_last,
                                hold_decision,
                                candidate_q=arm_q_filtered,
                                trust_region_retry_used=trust_region_retry_used,
                                pre_retry_max_joint_step=pre_retry_max_joint_step,
                                pre_retry_max_joint_index=pre_retry_max_joint_index,
                                trusted_max_joint_offset=(
                                    hard_candidate_decision.max_trusted_offset
                                ),
                                min_joint_limit_margin=(
                                    hard_candidate_decision.min_joint_limit_margin
                                ),
                            )
                            rospy.logwarn_throttle(
                                1.0,
                                "[IkRos][TrackingSafety] Holding the last trusted "
                                "arm command while hard-IK recovery is validated "
                                "(%s, %d/%d, trusted offset %.3frad, margin %.3frad)",
                                hard_candidate_decision.reason,
                                hard_candidate_decision.recovery_success_count,
                                self.absolute_hard_recovery_successes,
                                hard_candidate_decision.max_trusted_offset,
                                hard_candidate_decision.min_joint_limit_margin,
                            )
                            fail_count += 1
                            run_count += 1
                            sum_time_cost += time_cost
                            q_now = q_last.copy()
                            rate.sleep()
                            continue
                        if hard_candidate_decision.reason == "recovered":
                            rospy.loginfo(
                                "[IkRos][TrackingSafety] Hard local IK recovered "
                                "after %d stable candidates; resuming through the "
                                "execution rate limiter",
                                hard_candidate_decision.recovery_success_count,
                            )

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

                    # 保留旧限速策略用于显式开关和 shadow 对照；默认不影响控制输出。
                    if hand_heights and max(hand_heights) > 0:
                        shoulder_vel_limit = 20.0  # deg/s
                    else:
                        shoulder_vel_limit = 120.0  # deg/s

                    limiter_applied = False
                    arm_q_unlimited = arm_q_filtered.copy()
                    arm_q_velocity_limit_shadow = arm_q_unlimited.copy()
                    if self.enable_ik_velocity_limit or self.enable_ik_velocity_limit_shadow_publish:
                        if self.arm_mode_changing:
                            # 模式过渡期间旧策略不会介入；同步 shadow 初值，等待完成后独立推进。
                            velocity_limit_shadow_last = arm_q_unlimited.copy()
                        else:
                            arm_q_velocity_limit_shadow = self.limit_angle_by_velocity(
                                velocity_limit_shadow_last,
                                arm_q_unlimited,
                                vel_limit=720,
                                shoulder_vel_limit=shoulder_vel_limit,
                            )
                            velocity_limit_shadow_last = arm_q_velocity_limit_shadow.copy()

                    # 模式过渡状态与限速策略解耦：只有显式开启且不在过渡中才改变输出。
                    if self.enable_ik_velocity_limit and not self.arm_mode_changing:
                        arm_q_filtered = arm_q_velocity_limit_shadow
                        limiter_applied = True

                    command_rate_limited = False
                    execution_unlimited = np.asarray(
                        arm_q_filtered, dtype=float
                    ).copy()
                    if self.enable_absolute_target_alignment:
                        arm_q_filtered = (
                            self._absolute_execution_safety.limit_accepted_candidate(
                                execution_unlimited,
                                self.controller_dt,
                            )
                        )
                        command_rate_limited = bool(np.max(np.abs(
                            arm_q_filtered - execution_unlimited
                        )) > 1.0e-9)

                    if self.pub_velocity_limit_shadow is not None:
                        shadow_msg = Float32MultiArray()
                        shadow_msg.data = arm_q_velocity_limit_shadow * 180.0 / np.pi
                        self.pub_velocity_limit_shadow.publish(shadow_msg)

                        status_msg = Float32MultiArray()
                        status_msg.data = [
                            float(self.enable_ik_velocity_limit),
                            float(self.arm_mode_changing),
                            float(self.wheel_arm_mode_transitioning),
                            float(limiter_applied),
                            float(shoulder_vel_limit),
                            float(hand_heights[0]),
                            float(hand_heights[1]),
                            float(np.max(np.abs(arm_q_velocity_limit_shadow - arm_q_unlimited)) * 180.0 / np.pi),
                            float(self.enable_absolute_target_alignment),
                            float(command_rate_limited),
                            float(self.absolute_execution_max_joint_velocity),
                            float(self.absolute_execution_max_joint_acceleration),
                            float(np.max(np.abs(
                                arm_q_filtered - execution_unlimited
                            )) * 180.0 / np.pi),
                        ]
                        self.pub_velocity_limit_status.publish(status_msg)

                    if self.enable_absolute_target_alignment:
                        committed = self._commit_absolute_ik_result(
                            solve_generation,
                            control_target,
                            q_last,
                            arm_q_filtered,
                            trust_region_retry_used=trust_region_retry_used,
                            relaxed_retry_used=relaxed_retry_used,
                            pre_retry_max_joint_step=pre_retry_max_joint_step,
                            pre_retry_max_joint_index=pre_retry_max_joint_index,
                            command_rate_limited=command_rate_limited,
                            trusted_max_joint_offset=(
                                hard_candidate_decision.max_trusted_offset
                                if hard_candidate_decision is not None else 0.0
                            ),
                            min_joint_limit_margin=(
                                hard_candidate_decision.min_joint_limit_margin
                                if hard_candidate_decision is not None
                                else float("inf")
                            ),
                        )
                        if committed:
                            # Solver branch continuity follows the accepted
                            # hard-IK target, while q_last below follows the
                            # independently rate-limited executed command.
                            committed_solution = np.asarray(q_now, dtype=float).copy()
                            if hasattr(self.arm_ik, "commit_solution"):
                                self.arm_ik.commit_solution(committed_solution)
                            # self.quest3_arm_info_transformer.pub_whole_body_joint_state_msg(arm_q_filtered, map_finger=self.__as_mc_ik)
                            q_last[:self.__single_arm_dof] = q_now[:self.__single_arm_dof]
                            q_last[-self.__arm_dof:] = arm_q_filtered
                        else:
                            self._absolute_execution_safety.reset(
                                q_last[-self.__arm_dof:]
                            )
                            q_now = q_last.copy()
                    else:
                        msg.data = arm_q_filtered * 180.0 / np.pi
                        self.pub_filtered_joint.publish(msg)
                        self.publish_joint_states(q_now=arm_q_filtered, q_last=q_last)
                        q_last[:self.__single_arm_dof] = q_now[:self.__single_arm_dof]
                        q_last[-self.__arm_dof:] = arm_q_filtered
                else:
                    fail_count += 1
                    if self.enable_absolute_target_alignment:
                        self._publish_absolute_ik_solve_failed(
                            solve_generation, control_target, q_last,
                            trust_region_retry_used=trust_region_retry_used,
                            relaxed_retry_used=relaxed_retry_used,
                            pre_retry_max_joint_step=pre_retry_max_joint_step,
                            pre_retry_max_joint_index=pre_retry_max_joint_index,
                        )
                    # print(f"""\nq_last:{q_last}\n l_hand_pose:{l_hand_pose}\n
                    #       r_hand_pose:{r_hand_pose}\n
                    #       l_hand_RPY:{l_hand_RPY}\n
                    #       r_hand_RPY:{r_hand_RPY}\n
                    #       l_elbow_pos:{l_elbow_pos}\n
                    #       r_elbow_pos:{r_elbow_pos}\n
                    #       left_shoulder_rpy_in_robot:{left_shoulder_rpy_in_robot}\n
                    #       right_shoulder_rpy_in_robot:{right_shoulder_rpy_in_robot}\n""")

                run_count += 1
                success_rate = 100 * (1.0 - fail_count / float(run_count))
                sum_time_cost += time_cost
                if self.enable_position_ik_status_log and run_count % 10 == 0:
                    sys.stdout.write(
                        "\r[PositionIK] {}, success rate: {:.1f}%, avg time-cost: {:.1f} ms, is target far?: {}".format(
                            "RUNING" if is_runing else "STOPED", success_rate, 1e3 * sum_time_cost/run_count, self.judge_target_is_far()
                        )
                    )

            if self.__publish_err and q_now is not None:
                msg_pose_err = ikSolveError()
                msg_pose_err.ik_type = self.arm_ik.type().name()
                pose_left = arm_ik.left_hand_pose(q_now)
                pose_right = arm_ik.right_hand_pose(q_now)
                if target_pose_left[0] is not None:
                    pose_left_des = (target_pose_left[0], quaternion_to_RPY(target_pose_left[1]))
                    msg_pose_err.left_pose_error = self.generate_ik_solve_error_msg(pose_res=pose_left, pose_des=pose_left_des)
                if target_pose_right[0] is not None:
                    pose_right_des = (target_pose_right[0], quaternion_to_RPY(target_pose_right[1]))
                    msg_pose_err.right_pose_error = self.generate_ik_solve_error_msg(pose_res=pose_right, pose_des=pose_right_des)
                self.pub_ik_solve_error.publish(msg_pose_err)
            rate.sleep()

    def publish_joint_states(self, q_now, q_last):
        if self.pub is None:
            rospy.logerr_throttle(
                2.0,
                "[IkRos] Suppressed a legacy position-IK joint command while "
                "velocity IK has exclusive authority",
            )
            return
        arm_agl_limited = self.limit_angle(q_now[-self.__arm_dof:])
        msg = JointState()
        msg.name = ["arm_joint_" + str(i) for i in range(1, self.__arm_dof+1)]
        msg.header.stamp = rospy.Time.now()

        if self.only_half_up_body and self.optimized_state is None and self.sensor_data_raw is None:
            print(f"[ik_ros_uni]: optimized_state is None")
            return

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

    def kuavo_joint_states_callback(self, joint_states_msg):
        # 手臂状态正解
        self.__joint_states = np.array(joint_states_msg.q)
        self._update_absolute_measured_q(self.__joint_states, "/robot_arm_q_v_tau")
        if self.enable_velocity_ik_takeover:
            # The legacy FK publication is not part of the 21-D reference
            # contract. Skipping it allows takeover mode to run without ever
            # constructing ArmIk; C++ diagnostics expose measured FK instead.
            return
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

    @staticmethod
    def _finite_body_pose(pose_info):
        if pose_info is None:
            return False
        values = (
            pose_info.position.x, pose_info.position.y, pose_info.position.z,
            pose_info.orientation.x, pose_info.orientation.y,
            pose_info.orientation.z, pose_info.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return False
        quat_norm = math.sqrt(sum(float(value) ** 2 for value in values[3:]))
        return quat_norm > 0.5

    @staticmethod
    def _copy_body_pose(destination, source):
        destination.position.x = float(source.position.x)
        destination.position.y = float(source.position.y)
        destination.position.z = float(source.position.z)
        destination.orientation.x = float(source.orientation.x)
        destination.orientation.y = float(source.orientation.y)
        destination.orientation.z = float(source.orientation.z)
        destination.orientation.w = float(source.orientation.w)

    def _populate_whole_body_sample(self, tcp_msg, quest_msg, source,
                                    session_id, generation):
        """Attach same-packet Root/Chest body intent without shaping hands."""
        tcp_msg.body_tracking_frame = "quest_tracking_robot_axes"
        tcp_msg.body_high_confidence = bool(quest_msg.is_high_confidence)
        # Fixed ABI from monitor_quest3: Root=22, Chest=23, Neck=24,
        # Head=25; upper-arm origins are the available shoulder landmarks.
        indices = {
            "root": 22,
            "chest": 23,
            "head": 25,
            "left_shoulder": 0,
            "right_shoulder": 2,
        }
        samples = {}
        for name, index in indices.items():
            sample = quest_msg.poses[index] if len(quest_msg.poses) > index else None
            samples[name] = sample if self._finite_body_pose(sample) else None

        body_valid = (
            samples["chest"] is not None
            and (not self.quest3_arm_info_transformer.skeleton_filter_enabled
                 or bool(self.quest3_arm_info_transformer.estimated_bones))
        )
        # Fast coherent human motion is NOT evidence of tracking recentering.
        # Root is optional diagnostic data, not a torso validity dependency.

        tcp_msg.body_coordinate_epoch = int(self._body_coordinate_epoch)
        fields = (
            ("root", tcp_msg.root_pose, "root_valid"),
            ("chest", tcp_msg.chest_pose, "chest_valid"),
            ("head", tcp_msg.head_pose, "head_valid"),
            ("left_shoulder", tcp_msg.left_shoulder_pose,
             "left_shoulder_valid"),
            ("right_shoulder", tcp_msg.right_shoulder_pose,
             "right_shoulder_valid"),
        )
        for name, destination, valid_field in fields:
            valid = body_valid and samples[name] is not None
            setattr(tcp_msg, valid_field, bool(valid))
            if valid:
                self._copy_body_pose(destination, samples[name])
                estimator = self.quest3_arm_info_transformer
                state = estimator.estimated_bones.get(indices[name])
                if state is not None:
                    T = state[0]
                    destination.position.x, destination.position.y, destination.position.z = T[:3, 3]
                    q = matrix_to_quaternion(T[:3, :3])
                    (destination.orientation.x, destination.orientation.y,
                     destination.orientation.z, destination.orientation.w) = q

        estimator = self.quest3_arm_info_transformer
        if (body_valid and estimator.estimated_bones and
                all(side in estimator.tcp_estimated_rates for side in ("Left", "Right"))):
            def vector(out, value):
                out.x, out.y, out.z = map(float, value)
            for side, twist, elbow in (
                    ("Left", tcp_msg.left_tcp_twist, tcp_msg.left_elbow_velocity),
                    ("Right", tcp_msg.right_tcp_twist, tcp_msg.right_elbow_velocity)):
                linear, angular, hint = estimator.tcp_estimated_rates[side]
                vector(twist.linear, linear); vector(twist.angular, angular); vector(elbow, hint)
            vector(tcp_msg.chest_twist.linear, estimator.estimated_bones[23][1])
            vector(tcp_msg.chest_twist.angular, estimator.estimated_bones[23][2])
            vector(tcp_msg.root_velocity, estimator.estimated_bones[22][1])
            tcp_msg.estimated_twists_valid = True

    @runtime_probe.callback
    def _scheduled_quest_bone_poses_callback(self, msg):
        try:
            status = self._mapper_input_scheduler.apply_current_thread()
            if status is not None and status["errors"]:
                rospy.logwarn("Mapper input scheduling: %s", status["errors"])
        except Exception as exc:
            # Scheduling failure must never consume an input frame.
            rospy.logwarn("Mapper input scheduling unavailable: %s", exc)
        runtime_probe.point(1)
        return self.quest_bone_poses_callback(msg)

    def quest_bone_poses_callback(self, quest_bone_poses_msg):
        runtime_probe.point(2)
        mapper_callback_cpu_ns = time.thread_time_ns()
        received_monotonic = time.monotonic()
        received_ros = rospy.Time.now()
        stamp = float(quest_bone_poses_msg.timestamp_ms) * 0.001
        gate = self._input_clock_gate
        if (gate.latest_source is not None and stamp < gate.latest_source
                and received_monotonic - self._last_fresh_source_receive > 0.5):
            # Sustained device-clock restart, never inferred from pose speed.
            gate.reset()
            self._body_coordinate_epoch += 1
        if not gate.accept(quest_bone_poses_msg.timestamp_ms, received_monotonic):
            rospy.logwarn_throttle(1.0, "[IkRos] discarded delayed/reordered raw Quest sample")
            return
        self._last_fresh_source_receive = received_monotonic
        mapper_start_ns = time.monotonic_ns()
        mapper_transform_cpu_ns = time.thread_time_ns()
        runtime_probe.point(10)
        with self._absolute_lock:
            runtime_probe.point(11)
            self.quest3_arm_info_transformer.skeleton_contract = (
                self._absolute_session_id, self._absolute_generation,
                self._body_coordinate_epoch)
        runtime_probe.point(30)
        read_accepted = self.quest3_arm_info_transformer.read_msg(
            quest_bone_poses_msg, defer_visuals=self.enable_velocity_ik_takeover)
        runtime_probe.point(31)
        mapper_transform_end_ns = time.monotonic_ns()
        mapper_transform_end_cpu_ns = time.thread_time_ns()
        message_accepted = (
            bool(read_accepted)
            and self.quest3_arm_info_transformer.pose_info_list is quest_bone_poses_msg.poses
        )
        source = (
            TeleopTcpTarget.SOURCE_QUEST_HAND_TRACKING
            if quest_bone_poses_msg.is_hand_tracking
            else TeleopTcpTarget.SOURCE_QUEST_CONTROLLER
        )
        source_policy = self._tracking_source_policy(
            source, self.quest3_arm_info_transformer.is_runing
        )

        left_pose, left_elbow_pos = self.quest3_arm_info_transformer.get_hand_pose("Left")
        right_pose, right_elbow_pos = self.quest3_arm_info_transformer.get_hand_pose("Right")
        left_finger_joints = self.quest3_arm_info_transformer.get_finger_joints("Left")
        right_finger_joints = self.quest3_arm_info_transformer.get_finger_joints("Right")
        left_tcp_target, left_tcp_elbow_hint = (
            self.quest3_arm_info_transformer.get_tcp_target("Left")
        )
        right_tcp_target, right_tcp_elbow_hint = (
            self.quest3_arm_info_transformer.get_tcp_target("Right")
        )
        mapper_targets_end_ns = time.monotonic_ns()
        runtime_probe.point(32)
        target_available = (
            left_tcp_target is not None
            and right_tcp_target is not None
            and (self.enable_velocity_ik_takeover or (
                left_tcp_elbow_hint is not None and right_tcp_elbow_hint is not None))
        )
        source_policy = self._tracking_input_safety_policy(
            source,
            self.quest3_arm_info_transformer.is_runing,
            message_accepted,
            target_available,
            left_tcp_target,
            right_tcp_target,
            left_tcp_elbow_hint,
            right_tcp_elbow_hint,
            source_policy,
        )

        # This is the common control gate.  Diagnostics may inspect every raw
        # frame. Confidence is diagnostic only; malformed or disallowed-source
        # frames must not overwrite the shared target consumed by q-only IK.
        control_frame_accepted = (
            message_accepted
            and source_policy["source_allowed"]
        )
        if control_frame_accepted:
            self.__target_pose = left_pose
            self.__left_elbow_pos = left_elbow_pos
            self.__target_pose_right = right_pose
            self.__right_elbow_pos = right_elbow_pos
            self.hand_finger_data = [left_finger_joints, right_finger_joints]
            runtime_probe.point(12)
            with self._absolute_lock:
                runtime_probe.point(13)
                self._tracking_last_accepted_source = source
                self._tracking_control_running = bool(
                    self.quest3_arm_info_transformer.is_runing
                )
            if (not self.__recieved_new_target_pose
                    and self.__target_pose is not None
                    and self.__target_pose_right is not None
                    and self.__target_pose[0] is not None
                    and self.__target_pose_right[0] is not None):
                self.__recieved_new_target_pose = True

        absolute_target = None
        effective_left_tcp_target = left_tcp_target
        effective_right_tcp_target = right_tcp_target
        effective_left_tcp_elbow_hint = left_tcp_elbow_hint
        effective_right_tcp_elbow_hint = right_tcp_elbow_hint
        if self.enable_absolute_target_alignment and control_frame_accepted:
            raw_absolute_target = self._build_absolute_target_snapshot(
                quest_bone_poses_msg, control_frame_accepted
            )
            # Skeleton estimates are already mapped coherently. Session
            # alignment remains one rigid transform; continuous task-reference
            # shaping and physical feasibility belong to the C++ control tick.
            absolute_target = raw_absolute_target
            source_policy["governed_left_tcp"] = left_tcp_target
            source_policy["governed_right_tcp"] = right_tcp_target
            source_policy["governed_left_tcp_elbow"] = left_tcp_elbow_hint
            source_policy["governed_right_tcp_elbow"] = right_tcp_elbow_hint

        # Publish the unified TCP contract independently from MPC.  It carries
        # the absolute-session transaction identity so velocity IK can mirror
        # measured state while ARMED, own the complete ACTIVE/HOLD lifecycle,
        # and reject work from an older generation.
        tcp_frame_is_current = (
            message_accepted
            and quest_bone_poses_msg.timestamp_ms > 0
            and self.quest3_arm_info_transformer.timestamp_ms == quest_bone_poses_msg.timestamp_ms
        )
        tcp_target_published = False
        tcp_contract_should_publish = (
            self.robot_type == 1
            and tcp_frame_is_current
            and target_available
        )
        if tcp_contract_should_publish:
            tcp_msg = TeleopTcpTarget()
            tcp_msg.header.seq = self._tcp_target_seq
            self._tcp_target_seq = (self._tcp_target_seq + 1) & 0xFFFFFFFF
            tcp_msg.header.stamp = rospy.Time.from_sec(
                quest_bone_poses_msg.timestamp_ms * 1.0e-3
            )
            tcp_msg.header.frame_id = (
                "quest_chest_robot_axes" if self.enable_velocity_ik_takeover
                else self.velocity_ik_session_frame)
            tcp_msg.left_controlled_frame = "left_teleop_tcp"
            tcp_msg.right_controlled_frame = "right_teleop_tcp"

            tcp_msg.left_tcp_pose.position.x = float(effective_left_tcp_target[0][0])
            tcp_msg.left_tcp_pose.position.y = float(effective_left_tcp_target[0][1])
            tcp_msg.left_tcp_pose.position.z = float(effective_left_tcp_target[0][2])
            tcp_msg.left_tcp_pose.orientation.x = float(effective_left_tcp_target[1][0])
            tcp_msg.left_tcp_pose.orientation.y = float(effective_left_tcp_target[1][1])
            tcp_msg.left_tcp_pose.orientation.z = float(effective_left_tcp_target[1][2])
            tcp_msg.left_tcp_pose.orientation.w = float(effective_left_tcp_target[1][3])
            tcp_msg.right_tcp_pose.position.x = float(effective_right_tcp_target[0][0])
            tcp_msg.right_tcp_pose.position.y = float(effective_right_tcp_target[0][1])
            tcp_msg.right_tcp_pose.position.z = float(effective_right_tcp_target[0][2])
            tcp_msg.right_tcp_pose.orientation.x = float(effective_right_tcp_target[1][0])
            tcp_msg.right_tcp_pose.orientation.y = float(effective_right_tcp_target[1][1])
            tcp_msg.right_tcp_pose.orientation.z = float(effective_right_tcp_target[1][2])
            tcp_msg.right_tcp_pose.orientation.w = float(effective_right_tcp_target[1][3])

            tcp_msg.left_elbow_hint_valid = effective_left_tcp_elbow_hint is not None
            tcp_msg.right_elbow_hint_valid = effective_right_tcp_elbow_hint is not None
            if effective_left_tcp_elbow_hint is not None:
                tcp_msg.left_elbow_hint.x = float(effective_left_tcp_elbow_hint[0])
                tcp_msg.left_elbow_hint.y = float(effective_left_tcp_elbow_hint[1])
                tcp_msg.left_elbow_hint.z = float(effective_left_tcp_elbow_hint[2])
            if effective_right_tcp_elbow_hint is not None:
                tcp_msg.right_elbow_hint.x = float(effective_right_tcp_elbow_hint[0])
                tcp_msg.right_elbow_hint.y = float(effective_right_tcp_elbow_hint[1])
                tcp_msg.right_elbow_hint.z = float(effective_right_tcp_elbow_hint[2])
            runtime_probe.point(14)
            with self._absolute_lock:
                runtime_probe.point(15)
                absolute_state = self._absolute_state
                tcp_msg.session_id = int(self._absolute_session_id)
                tcp_msg.generation = int(self._absolute_generation)
                tcp_msg.absolute_state = int(absolute_state)
                if not self.enable_absolute_target_alignment:
                    contract_state = TeleopTcpTarget.CONTROL_ACTIVE
                elif absolute_state == AbsoluteEnableState.SWITCHING:
                    contract_state = TeleopTcpTarget.CONTROL_ARMED
                elif absolute_state == AbsoluteEnableState.ACTIVE:
                    contract_state = (
                        TeleopTcpTarget.CONTROL_ACTIVE
                        if ((source_policy["source_allowed"] or self.enable_velocity_ik_takeover)
                            and not self._tracking_source_hold_latched
                            and not self._absolute_fault_hold_pending
                            and self.quest3_arm_info_transformer.is_runing)
                        else TeleopTcpTarget.CONTROL_HOLD
                    )
                elif absolute_state in (
                        AbsoluteEnableState.RETURNING_TO_PREP,
                        AbsoluteEnableState.PREPARED):
                    contract_state = TeleopTcpTarget.CONTROL_PREPARE
                else:
                    contract_state = TeleopTcpTarget.CONTROL_INACTIVE
                tcp_msg.control_state = int(contract_state)
                now = time.monotonic()
                self._fill_subsystem_permissions_locked(tcp_msg, now)
                # ARMED is a valid, deliberately frozen reference contract.
                # The velocity IK uses it to capture measured FK and construct
                # the per-session bumpless target alignment before ACTIVE is
                # allowed to command motion.  Marking ARMED invalid made the
                # manual Quest path skip that capture while the deterministic
                # headless driver exercised it correctly.
                absolute_tcp_control_valid = (
                    not self.enable_absolute_target_alignment
                    or contract_state in (
                        TeleopTcpTarget.CONTROL_ARMED,
                        TeleopTcpTarget.CONTROL_ACTIVE,
                        TeleopTcpTarget.CONTROL_PREPARE,
                    )
                )
            tcp_control_valid = (
                control_frame_accepted
                and (self.quest3_arm_info_transformer.is_runing
                     or self._fixed_prepare_bootstrap
                     or contract_state == TeleopTcpTarget.CONTROL_PREPARE)
                and absolute_tcp_control_valid
            )
            # Pose validity stays truthful. A previously accepted fixed PREPARE
            # may continue on fresh lifecycle traffic while pose validity
            # or pose source changes. It cannot bootstrap from an invalid pose.
            runtime_probe.point(16)
            with self._absolute_lock:
                runtime_probe.point(17)
                joystick_age = (
                    float("inf") if self._joystick_receive_monotonic is None
                    else time.monotonic() - self._joystick_receive_monotonic)
                fixed_prepare_control_valid = (
                    self.enable_velocity_ik_takeover
                    and contract_state == TeleopTcpTarget.CONTROL_PREPARE
                    and self._absolute_state in (
                        AbsoluteEnableState.RETURNING_TO_PREP,
                        AbsoluteEnableState.PREPARED)
                    and tcp_msg.session_id == self._absolute_session_id
                    and tcp_msg.generation == self._absolute_generation
                    and 0.0 <= joystick_age <= self.absolute_enable_max_vr_age
                    and not self._absolute_stop_pending
                    and not self._absolute_external_mode_reset_pending
                    and not self._absolute_fault_hold_pending)
            tcp_msg.left_valid = tcp_control_valid
            tcp_msg.right_valid = tcp_control_valid
            tcp_msg.control_valid = (
                fixed_prepare_control_valid
                if contract_state == TeleopTcpTarget.CONTROL_PREPARE
                else tcp_control_valid)
            if self.enable_velocity_ik_takeover and contract_state != TeleopTcpTarget.CONTROL_PREPARE:
                # Pose validity is NOT operator permission. Keep invalid
                # observations truthful, but let the robot's timed cache handle
                # a brief occlusion without manufacturing a CONTROL_HOLD edge.
                tcp_msg.control_valid = bool(absolute_tcp_control_valid and (
                    self.quest3_arm_info_transformer.is_runing or
                    contract_state == TeleopTcpTarget.CONTROL_ARMED))
            tcp_msg.source = source
            mapper_contract_end_ns = time.monotonic_ns()
            runtime_probe.identity(tcp_msg.header.seq,tcp_msg.session_id,tcp_msg.generation)
            runtime_probe.point(40)
            self._populate_whole_body_sample(
                tcp_msg, quest_bone_poses_msg, source,
                tcp_msg.session_id, tcp_msg.generation,
            )
            tcp_msg.mapper_receive_stamp = received_ros
            tcp_msg.mapper_publish_stamp = rospy.Time.now()
            tcp_msg.mapper_processing_time = time.monotonic() - received_monotonic
            tcp_msg.source_relative_age = gate.relative_age
            if tcp_msg.mapper_processing_time + gate.relative_age <= gate.max_age:
                runtime_probe.point(50)
                mapper_publish_begin_ns = time.monotonic_ns()
                # Critical publish precedes deferred chest/shoulder observations,
                # auxiliary head/body output, markers and all timing diagnostics.
                self.pub_mm_teleop_tcp_target.publish(tcp_msg)
                mapper_publish_end_ns = time.monotonic_ns()
                runtime_probe.point(51)
                mapper_publish_end_cpu_ns = time.thread_time_ns()
                runtime_probe.point(52)
                self.quest3_arm_info_transformer.flush_pending_arm_markers()
                runtime_probe.point(53)
                self.pub_velocity_timing.publish(Float64MultiArray(data=[2.0,float(tcp_msg.header.seq),float(tcp_msg.session_id),float(tcp_msg.generation),tcp_msg.header.stamp.to_sec(),received_monotonic*1e9,float(mapper_publish_end_ns),float(quest_bone_poses_msg.timestamp_ms),float(mapper_start_ns),float(mapper_transform_end_ns),float(mapper_targets_end_ns),float(mapper_contract_end_ns),float(mapper_publish_begin_ns),float(tcp_msg.control_state),float(tcp_msg.control_valid),float(mapper_callback_cpu_ns),float(mapper_transform_cpu_ns),float(mapper_transform_end_cpu_ns),float(mapper_publish_end_cpu_ns),float(self.quest3_arm_info_transformer._marker_wall_ns),float(self.quest3_arm_info_transformer._marker_cpu_ns)]))
                tcp_target_published = tcp_control_valid

        # Rejected/inactive TCP frames still keep their visual diagnostics.
        self.quest3_arm_info_transformer.flush_pending_arm_markers()

        if self.enable_absolute_target_alignment and source_policy["source_allowed"]:
            publish_target = self._handle_absolute_target_and_gesture(
                absolute_target, self.quest3_arm_info_transformer.is_runing
            )
            if publish_target is not None and self._wheel_mpc_stable_for_mm_cmd():
                self._publish_mm_absolute_target(publish_target)
        # Legacy path: preserve immediate publishing behavior when the new state machine is disabled.
        elif (self.robot_type == 1 and self.quest3_arm_info_transformer.is_runing
                and not self.enable_absolute_target_alignment
                and control_frame_accepted
                and self._wheel_mpc_stable_for_mm_cmd()
                and self.__target_pose is not None and self.__target_pose_right is not None):
            eef_pose_msg = twoArmHandPoseCmd()
            eef_pose_msg.frame = 3
            # Preserve the Quest device timestamp for causal target-twist
            # estimation. Its epoch need not match ROS time; only adjacent
            # device-time differences are used by the velocity IK shadow.
            if quest_bone_poses_msg.timestamp_ms > 0:
                eef_pose_msg.hand_poses.header.stamp = rospy.Time.from_sec(
                    quest_bone_poses_msg.timestamp_ms * 1.0e-3
                )
            else:
                eef_pose_msg.hand_poses.header.stamp = rospy.Time.now()
            eef_pose_msg.hand_poses.header.seq = self._mm_target_seq
            self._mm_target_seq = (self._mm_target_seq + 1) & 0xFFFFFFFF
            eef_pose_msg.hand_poses.header.frame_id = "base_link"
            # self.__target_pose 是 (hand_pos, hand_quat) 元组
            eef_pose_msg.hand_poses.left_pose.pos_xyz = self.__target_pose[0]  # hand_pos [x, y, z]
            eef_pose_msg.hand_poses.left_pose.quat_xyzw = self.__target_pose[1]  # hand_quat [x, y, z, w]
            eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = self.__left_elbow_pos if self.__left_elbow_pos is not None else [0.0, 0.0, 0.0]

            # self.__target_pose_right 是 (hand_pos, hand_quat) 元组
            eef_pose_msg.hand_poses.right_pose.pos_xyz = self.__target_pose_right[0]  # hand_pos [x, y, z]
            eef_pose_msg.hand_poses.right_pose.quat_xyzw = self.__target_pose_right[1]  # hand_quat [x, y, z, w]
            eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = self.__right_elbow_pos if self.__right_elbow_pos is not None else [0.0, 0.0, 0.0]

            self.pub_mm_two_arm_hand_pose_cmd.publish(eef_pose_msg)

        runtime_probe.point(60)
        self._publish_quest_tcp_mapping_debug(
            quest_bone_poses_msg,
            message_accepted,
            target_available,
            tcp_target_published,
            source_policy,
        )
        runtime_probe.point(61)
        if source_policy["source_hold_triggered"] or source_policy["source_hold_released"]:
            self._publish_absolute_enable_status()

        # self.pub_robot_end_hand(left_finger_joints, right_finger_joints)

    def two_arm_hand_pose_target_callback(self, msg_ori):
        msg = msg_ori.hand_poses
        if msg_ori.use_custom_ik_param:
            self.external_q0 = list(msg.left_pose.joint_angles) + list(msg.right_pose.joint_angles)
        self.__target_pose = (msg.left_pose.pos_xyz, msg.left_pose.quat_xyzw)
        self.__target_pose_right = (msg.right_pose.pos_xyz, msg.right_pose.quat_xyzw)
        if(abs(msg.left_pose.elbow_pos_xyz[0]) <= 1e-5
            and abs(msg.left_pose.elbow_pos_xyz[1]) <= 1e-5
            and abs(msg.left_pose.elbow_pos_xyz[2]) <= 1e-5):  # 都为0，则不控制elbow
            self.__left_elbow_pos = None
        else:
            self.__left_elbow_pos = np.array(msg.left_pose.elbow_pos_xyz)
        if(abs(msg.right_pose.elbow_pos_xyz[0]) <= 1e-5
            and abs(msg.right_pose.elbow_pos_xyz[1]) <= 1e-5
            and abs(msg.right_pose.elbow_pos_xyz[2]) <= 1e-5):
            self.__right_elbow_pos = None
        else:
            self.__right_elbow_pos = np.array(msg.right_pose.elbow_pos_xyz)

        if self.__recieved_new_target_pose == False:
            self.__recieved_new_target_pose = True
        if self.__as_mc_ik:
            self.__as_mc_ik = False
            self.arm_ik.set_as_mc_ik(self.__as_mc_ik)

    def _publish_wheel_base_command(self, msg=None):
        if getattr(self, '_wheel_base_pub', None) is None:
            return
        with self._absolute_lock:
            now = time.monotonic()
            active = self._base_input_ready_locked(now)
            if msg is None:
                if active:
                    return  # Never replay cached joystick input as a new command.
                self._wheel_base_mapper.reset()
                velocity = (0.0, 0.0, 0.0)
            else:
                velocity = self._wheel_base_mapper.sample(msg, active, now)
            reason = self._wheel_base_mapper.reason if active else self._base_gate_reason
            if reason != getattr(self, '_base_last_logged_reason', None):
                rospy.loginfo('[BaseGate][Mapper] %s', reason)
                self._base_last_logged_reason = reason
            command = Twist()
            command.linear.x, command.linear.y, command.angular.z = velocity
            self._wheel_base_pub.publish(command)

    def _base_input_ready_locked(self, now):
        # Independent wheel channel: neither arm lifecycle nor body freshness
        # participates. Global control/stop and live robot feedback still do.
        observed = getattr(self, '_wheel_mpc_obs_receive_monotonic', None)
        measured = self._absolute_latest_measured_q_time
        reason = 'READY'
        if self._fixed_control_enabled is not True or self.stop_event.is_set():
            reason = 'GLOBAL_STOP'
        elif (not self._wheel_mpc_stable_for_mm_cmd() or observed is None
                or not 0 <= now-observed <= .2 or measured is None
                or not 0 <= now-measured <= self.absolute_enable_max_joint_state_age):
            reason = 'NOT_READY'
        elif (self._joystick_receive_monotonic is None
                or not 0 <= now-self._joystick_receive_monotonic < .2):
            reason = 'JOY_STALE'
        self._base_gate_reason = reason
        return reason == 'READY'

    def _torso_input_ready_locked(self, now):
        target = self._absolute_latest_target
        return (self.enable_velocity_ik_takeover
                and self._absolute_state == AbsoluteEnableState.ACTIVE
                and self._fixed_control_enabled is True
                and self._absolute_mode_status == (2, 2, False)
                and self._fixed_pending_operation is None
                and not self._fixed_operation_error and not self._fixed_operation_block
                and not self._absolute_stop_pending and not self._absolute_fault_hold_pending
                and not self._tracking_source_hold_latched
                and not self._absolute_external_mode_reset_pending
                and target is not None and 0 <= now-target['receive_time'] <= self.absolute_enable_max_vr_age)

    def _fill_subsystem_permissions_locked(self, msg, now):
        msg.torso_control_enabled = self._torso_controls.authorized(
            now, self._torso_input_ready_locked(now))
        msg.arm_hold = (self._absolute_state == AbsoluteEnableState.ACTIVE
                        and getattr(self, '_fixed_arm_hold', False))
        msg.arm_operation_seq = getattr(self, '_fixed_arm_operation_seq', 0)

    def _publish_torso_control_state(self):
        with self._absolute_lock:
            now = time.monotonic()
            enabled = self._torso_controls.authorized(now, self._torso_input_ready_locked(now))
            if enabled == self._torso_last_published:
                return
            self._torso_last_published = enabled
        self._torso_state_pub.publish(Bool(data=enabled))
        rospy.loginfo('[AbsoluteTorso] torso control %s', 'enabled' if enabled else 'disabled (brake and hold)')

    def _consume_torso_end_effector_edges(self, hand_msg):
        # Pico Y is resolved on release. Consume this event in the input callback
        # so a slower hand timer cannot miss it, then suppress duplicate handling.
        if self._torso_controls.source != 'pico':
            return
        if (hand_msg.left_second_button_pressed and not self.sg100_enabled
                and self.end_effector_type in (QIANGNAO, QIANGNAO_TOUCH, REVO2, LINKER_HAND, LEJUCLAW)):
            self.__freeze_finger = not self.__freeze_finger
        hand_msg.left_second_button_pressed = False

    def _controller_start_request(self, msg):
        """Latch an entire double-trigger press, including staggered first frames."""
        if not self.enable_velocity_ik_takeover:
            return
        down = (math.isfinite(msg.left_trigger) and math.isfinite(msg.right_trigger)
                and msg.left_trigger > .5 and msg.right_trigger > .5)
        with self._absolute_lock:
            edge = down and not getattr(self, '_controller_start_down', False)
            self._controller_start_down = down
            # Active triggers retain their existing end-effector meaning.
            eligible = (self._absolute_state in (AbsoluteEnableState.WAITING,
                        AbsoluteEnableState.WAIT_RELEASE, AbsoluteEnableState.PREPARED)
                        and self._fixed_pending_operation is None)
        if edge and eligible:
            if not self._fixed_operation_mutex.acquire(False):
                # Retry a still-held press on the next fresh joy packet, never
                # lose the first edge to a simultaneous service operation.
                with self._absolute_lock:
                    self._controller_start_down = False
                return
            try:
                response = self._fixed_arm_operation(2)
                rospy.loginfo('[FixedStart] double front trigger: %s', response.message)
            finally:
                self._publish_fixed_operation_state()
                self._fixed_operation_mutex.release()

    def joySticks_data_callback(self, msg):
        with self._absolute_lock:
            self._joystick_receive_monotonic = time.monotonic()
        self.quest3_arm_info_transformer.read_joySticks_msg(msg)
        self._controller_start_request(msg)
        hand_msg = msg
        if getattr(self, '_pico_thumb_input', None) is not None:
            hand_msg = self._pico_thumb_input.update(
                msg, time.monotonic(), enabled=self._wheel_mpc_obs_time is not None)
        if self.enable_velocity_ik_takeover:
            with self._absolute_lock:
                now = time.monotonic()
                hand_msg = self._torso_controls.update(
                    msg, hand_msg, now, self._torso_input_ready_locked(now))
            self._consume_torso_end_effector_edges(hand_msg)
            self._publish_torso_control_state()
        self.joySticks_data = hand_msg
        self._publish_wheel_base_command(msg)
        if self._uses_front_trigger_dexhand():
            self._publish_front_trigger_dexhand(hand_msg)
        # 准备姿态下 IK 线程可能卡在等待首帧姿态/MPC 就绪，此时由回调补发夹爪指令。
        # 遥操主循环启动后改由 hand_finger_data_process 单路发布，避免双路重复下发。
        # qa: https://www.lejuhub.com/highlydynamic/kuavodevlab/-/issues/3505
        if (not self.sg100_enabled and self.end_effector_type == LEJUCLAW
                and not self.quest3_arm_info_transformer.is_hand_tracking
                and not self.__ik_claw_publish_active):
            self.pub_robot_end_hand(joyStick_data=hand_msg)

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
            pos_target, quat_target = self.__target_pose if ctrl_arm_idx.name() == ArmIdx.LEFT.name() else self.__target_pose_right

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
        if self.__target_pose[0] is None or self.__target_pose_right[0] is None:
            return False
        if self.__current_pose[0] is None or self.__current_pose_right[0] is None:
            return False
        pos_left, _ = self.__current_pose
        pos_right, _ = self.__current_pose_right
        pos_target_left, _ = self.__target_pose
        pos_target_right, _ = self.__target_pose_right
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

    def _uses_front_trigger_dexhand(self):
        return self.sg100_enabled or self.dexhand_front_trigger and self.end_effector_type in (
            QIANGNAO, QIANGNAO_TOUCH, REVO2, LINKER_HAND)

    def _publish_front_trigger_dexhand(self, msg):
        if self.sg100_enabled:
            self._publish_sg100_triggers(msg)
            return
        # One command per new joystick packet; never replay cached input from
        # the IK supervisor. No command is sent during construction or shutdown.
        if (not hasattr(self, "control_robot_hand_position_pub")
                or not hasattr(self, "stop_event") or self.stop_event.is_set()
                or self.quest3_arm_info_transformer.is_hand_tracking):
            return
        with self._absolute_lock:
            if self._tracking_source_hold_latched:
                return
        if self.end_effector_type in (QIANGNAO, QIANGNAO_TOUCH):
            # Match dev ik_ros_uni.py: channel 1 is the X/A thumb touch,
            # with Y-edge finger freeze and the original combo gestures.
            # Keep one publication per new controller packet.
            values = (msg.left_trigger, msg.right_trigger, msg.left_grip, msg.right_grip)
            if not all(math.isfinite(value) for value in values):
                rospy.logwarn_throttle(2.0, "Ignoring non-finite dexhand controller input")
                return
            with self._absolute_lock:
                self.pub_robot_end_hand(joyStick_data=msg)
            return
        values = (msg.left_trigger, msg.right_trigger)
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(2.0, "Ignoring non-finite dexhand trigger input")
            return
        positions = [int(100.0 * min(1.0, max(0.0, value))) for value in values]
        command = robotHandPosition()
        command.header.stamp = rospy.Time.now()
        command.left_hand_position = [positions[0]] * 6
        command.right_hand_position = [positions[1]] * 6
        self.control_robot_hand_position_pub.publish(command)

    def _publish_sg100_triggers(self, msg):
        if not hasattr(self, "sg100_trigger_pub"):
            return
        command = Vector3Stamped()
        command.header.stamp = rospy.Time.now()
        with self._absolute_lock:
            allowed = (hasattr(self, "stop_event") and not self.stop_event.is_set()
                       and not self._tracking_source_hold_latched
                       and not self.quest3_arm_info_transformer.is_hand_tracking
                       # Joystick packets can arrive before the first bone frame.
                       # Revoke hand input until both gesture inputs exist.
                       and self.quest3_arm_info_transformer.left_finger_joints is not None
                       and self.quest3_arm_info_transformer.right_finger_joints is not None
                       and not self.quest3_arm_info_transformer.is_stop_gesture())
        values = (msg.left_trigger, msg.right_trigger)
        if allowed and all(math.isfinite(value) for value in values):
            command.vector.x, command.vector.y = [min(1.0, max(0.0, v)) for v in values]
            command.vector.z = 1.0
        # Publish revocation as well, so an explicit stop need not await timeout.
        self.sg100_trigger_pub.publish(command)
        if hasattr(self, "sg100_gesture_client"):
            torso_modifier = getattr(getattr(self, '_torso_controls', None), 'quest_consumed', False)
            self.sg100_gesture_client.update(msg, command.vector.z == 1.0 and not torso_modifier)

    def hand_finger_data_process(self, event):
        if self._uses_front_trigger_dexhand():
            # Qiangnao controller packets use the callback. Valid optical
            # hand tracking still uses dev's six-angle conversion below.
            if (self.sg100_enabled
                    or self.end_effector_type not in (QIANGNAO, QIANGNAO_TOUCH)
                    or not self.quest3_arm_info_transformer.is_hand_tracking):
                return
            if self._tracking_last_accepted_source != TeleopTcpTarget.SOURCE_QUEST_HAND_TRACKING:
                return
        # if(self.isJoyPushed(self.joySticks_data)):
        with self._absolute_lock:
            accepted_source = self._tracking_last_accepted_source
            source_hold_latched = self._tracking_source_hold_latched
        if source_hold_latched:
            return
        if accepted_source != TeleopTcpTarget.SOURCE_QUEST_HAND_TRACKING:
            # print(f"\033[91mJoystick is pushed, stop control.\033[0m")
            self.pub_robot_end_hand(joyStick_data=self.joySticks_data)
        else:
            # print(f"\033[91mJoystick is not pushed, continue control.\033[0m")
            self.pub_robot_end_hand(hand_finger_data=self.hand_finger_data)


    def pub_robot_end_hand(self, joyStick_data=None, hand_finger_data = None):
        if self.sg100_enabled:
            return
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

                    if (getattr(self, '_pico_thumb_input', None) is None
                            and joyStick_data.left_first_button_touched and joyStick_data.right_first_button_pressed):
                        for i in range(0, 6):
                            left_hand_position[i] = 100
                        left_hand_position[2] = 0
                    if (getattr(self, '_pico_thumb_input', None) is None
                            and joyStick_data.left_first_button_touched and joyStick_data.right_second_button_pressed):
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
        publish_absolute_status = False
        fault_rearm_authorized = False
        fault_rearm_blocked = False
        if self.enable_absolute_target_alignment:
            with self._absolute_lock:
                state = self._absolute_state
                if new_mode == 1:
                    # In velocity takeover this is a lifecycle intent, not a
                    # request to hand authority back to legacy mode 1.
                    if state in (
                        AbsoluteEnableState.CAPTURED,
                        AbsoluteEnableState.SWITCHING,
                        AbsoluteEnableState.ACTIVE,
                        AbsoluteEnableState.WAIT_RELEASE,
                        AbsoluteEnableState.RESUME_PENDING,
                    ):
                        self._absolute_pause_pending = True
                elif new_mode == 2:
                    # 第二次 X+A 只发送恢复意图，mode-2 服务由绝对目标状态机
                    # 在重新捕获、预装冻结目标后统一发起。
                    if self._tracking_source_hold_latched:
                        if (state in (
                                AbsoluteEnableState.WAIT_RELEASE,
                                AbsoluteEnableState.PAUSED,
                            ) and self._tracking_can_rearm_locked()):
                            self._clear_tracking_fault_locked()
                            # X+A 是显式重新授权；把内部手势置 false，让下一条
                            # 已允许来源帧形成新的启用沿并原子捕获当前目标/实测 q。
                            self._absolute_gesture_running = False
                            self._absolute_state = AbsoluteEnableState.WAITING
                            self._absolute_reason = AbsoluteEnableReason.RESUME_REQUEST
                            self._absolute_pause_pending = False
                            self._absolute_external_mode_reset_pending = False
                            fault_rearm_authorized = True
                            publish_absolute_status = True
                        else:
                            fault_rearm_blocked = True
                    elif state == AbsoluteEnableState.RETURNING_TO_PREP:
                        self._interrupt_fixed_prepare_locked()
                        publish_absolute_status = True
                    elif state == AbsoluteEnableState.PREPARED:
                        self._queue_absolute_capture_locked(
                            AbsoluteEnableReason.RESUME_REQUEST,
                            AbsoluteEnableState.PREPARED,
                        )
                        publish_absolute_status = True
                    elif (self._absolute_gesture_running
                            and (state == AbsoluteEnableState.PAUSED
                                 or self._absolute_pause_pending)):
                        self._absolute_pause_pending = False
                        self._absolute_external_mode_reset_pending = False
                        if self.enable_velocity_ik_takeover:
                            # Mode 2 remains owned while the formal 21-D
                            # ReferenceManager holds. Re-enable by atomically
                            # recapturing the current target/state; do not wait
                            # for a legacy mode-1 transition.
                            self._queue_absolute_capture_locked(
                                AbsoluteEnableReason.RESUME_REQUEST,
                                AbsoluteEnableState.PAUSED,
                            )
                        else:
                            self._absolute_state = AbsoluteEnableState.RESUME_PENDING
                            self._absolute_reason = AbsoluteEnableReason.RESUME_REQUEST
                            self._absolute_resume_request_time = time.monotonic()
                        publish_absolute_status = True
                elif state in (
                    AbsoluteEnableState.CAPTURED,
                    AbsoluteEnableState.SWITCHING,
                    AbsoluteEnableState.ACTIVE,
                    AbsoluteEnableState.RESUME_PENDING,
                    AbsoluteEnableState.RETURNING_TO_PREP,
                    AbsoluteEnableState.PREPARED,
                ):
                    self._absolute_external_mode_reset_pending = True
        if publish_absolute_status:
            if fault_rearm_authorized:
                rospy.loginfo(
                    "[IkRos][AbsoluteEnable] X+A acknowledged the source/input fault; "
                    "the next stable configured-source frame will create a fresh session"
                )
            else:
                if state == AbsoluteEnableState.RETURNING_TO_PREP:
                    rospy.loginfo(
                        "[IkRos][AbsoluteEnable] X+A resume queued until the velocity-IK ready pose settles"
                    )
                else:
                    rospy.loginfo(
                        "[IkRos][AbsoluteEnable] X+A resume requested; %s",
                        ("recaptured under retained mode 2"
                         if self.enable_velocity_ik_takeover
                         else "waiting for mode 1 before recapture")
                    )
            self._publish_absolute_enable_status()
        elif fault_rearm_blocked:
            rospy.logwarn(
                "[IkRos][AbsoluteEnable] X+A rearm rejected: configured source is not yet "
                "stable for %d frames or mode-0 fault hold is still pending",
                self.tracking_rearm_stable_frames,
            )
        if new_mode == 0:  # 当模式不是2时
            # 重置所有姿态
            print(f"\033[91m[IK]Reset arm mode.\033[0m")
            self.trigger_reset_mode = True
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
                if not self.enable_absolute_target_alignment:
                    self.__need_reset_ik_guess = True
                self.arm_mode_changing = True
            elif current_mode == new_mode and not self.__first_change_arm_mode:
                # 模式切换完成，关闭arm_mode_changing标志
                if not self.only_half_up_body:
                    self.arm_mode_changing = False
                    self.__first_change_arm_mode = True

    def wheel_arm_control_mode_status_callback(self, msg):
        """轮臂WBC模式状态：[current_mode, target_mode, transitioning]。"""
        if len(msg.data) < 3:
            rospy.logwarn_throttle(
                2.0,
                "[IkRos] Invalid /humanoid_wheel/arm_control_mode_status; expected 3 values",
            )
            return

        current_mode = int(msg.data[0])
        target_mode = int(msg.data[1])
        transitioning = bool(round(msg.data[2]))
        self.__arm_control_mode = current_mode
        self.wheel_arm_mode_transitioning = transitioning
        if self.enable_absolute_target_alignment:
            with self._absolute_lock:
                self._absolute_mode_status = (current_mode, target_mode, transitioning)
                self._absolute_mode_status_time = time.monotonic()
            self._publish_absolute_enable_status()

        if transitioning:
            self.arm_mode_changing = True
            return

        if current_mode == target_mode and not self.only_half_up_body:
            if self.arm_mode_changing:
                rospy.loginfo(
                    "[IkRos] Wheel arm mode transition complete: mode=%d; "
                    "IK velocity limiter is %s",
                    current_mode,
                    "enabled" if self.enable_ik_velocity_limit else "disabled",
                )
            self.arm_mode_changing = False

    def whole_body_velocity_ik_status_callback(self, msg):
        """Complete the X+A ready-pose transaction using 21-D feedback."""
        publish_status = False
        resume_session = None
        prepared_session = None
        with self._absolute_lock:
            self._fixed_consumer_status = (time.monotonic(), msg)
            if (msg.session_id != self._absolute_session_id
                    or msg.generation != self._absolute_generation
                    or msg.control_state != TeleopTcpTarget.CONTROL_PREPARE):
                return
            if (self._absolute_state == AbsoluteEnableState.RETURNING_TO_PREP
                    and msg.preparation_complete and self._fixed_consumer_ack_locked(time.monotonic())):
                self._absolute_prepare_complete = True
                self._absolute_state = AbsoluteEnableState.PREPARED
                self._absolute_reason = AbsoluteEnableReason.PREPARED
                prepared_session = self._absolute_session_id
                publish_status = True
                if self._absolute_resume_after_prepare:
                    pending = self._fixed_pending_operation
                    error = self._fixed_capture_input_error_locked()
                    if (error or self._fixed_control_enabled is not True
                            or self._absolute_stop_pending or self._absolute_fault_hold_pending
                            or self._tracking_source_hold_latched
                            or self._absolute_external_mode_reset_pending
                            or self.quest3_arm_info_transformer.is_stop_gesture()
                            or pending is None
                            or pending['generation'] != self._absolute_generation
                            or pending['stop_epoch'] != self._fixed_stop_epoch
                            or pending['epoch'] != self._body_coordinate_epoch
                            or pending['source'] != self._configured_tracking_source()
                            or time.monotonic() >= pending['deadline']):
                        self._absolute_resume_after_prepare = False
                        if pending is not None:
                            pending['resume'] = False
                        rospy.logwarn('[FixedStart] Queued follow cancelled at PREPARE completion: %s',
                                      error or 'stop, fault, disabled control or changed request')
                    else:
                        self._authorize_fixed_follow_locked()
                        self._fixed_operation_block = False
                if self._absolute_resume_after_prepare:
                    self._queue_absolute_capture_locked(
                        AbsoluteEnableReason.RESUME_REQUEST,
                        AbsoluteEnableState.PREPARED,
                    )
                    resume_session = self._absolute_session_id
                    if self._fixed_pending_operation is not None:
                        self._fixed_pending_operation.update(mode=2, phase='acquisition', resume=False,
                            generation=self._absolute_generation,
                            deadline=time.monotonic()+self._fixed_acquisition_timeout)
                        self._fixed_operation_phase = 2
        if prepared_session is not None:
            rospy.loginfo(
                "[IkRos][AbsoluteEnable] Session %d reached the whole-body velocity-IK ready pose",
                prepared_session,
            )
        if resume_session is not None:
            rospy.loginfo(
                "[IkRos][AbsoluteEnable] Deferred X+A resume accepted as session %d after ready-pose completion",
                resume_session,
            )
        if publish_status:
            self._publish_absolute_enable_status()

    def sensor_data_raw_callback(self, msg):
        self.sensor_data_raw = msg
        if self.enable_absolute_target_alignment:
            joint_q = np.asarray(msg.joint_data.joint_q, dtype=float)
            # 轮臂传感器布局末尾为14个手臂关节和2个轮关节。
            if joint_q.size >= self.__arm_dof + 2:
                self._update_absolute_measured_q(
                    joint_q[-(self.__arm_dof + 2):-2], "/sensors_data_raw"
                )

    def wheel_mpc_observation_callback(self, msg):
        """轮臂 MPC 观测：time 为 MPC 模块内部时间，从 0 递增"""
        self._wheel_mpc_obs_time = float(msg.time)
        self._wheel_mpc_obs_receive_monotonic = time.monotonic()

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
        if (self.pub is None or self.frozen_arm_state is None
                or self.optimized_state is None):
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
            self.stop_event.set()  # 设置停止事件
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
        if not self.position_ik_control_enabled:
            response = SetBoolResponse()
            response.success = False
            response.message = (
                "Position IK is disabled because velocity IK has exclusive authority"
            )
            rospy.logwarn("[IkRos] %s", response.message)
            return response
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
    velocity_ik_takeover_requested = bool(rospy.get_param(
        "~enable_velocity_ik_takeover", False
    ))

    if not velocity_ik_takeover_requested:
        raise RuntimeError("This candidate requires the Pinocchio formal reference pipeline")

    print(f"\033[92mControl {ctrl_arm_idx.name()} arms.\033[0m")
    print(f"\033[92mIk type: {ik_type_idx.name()}\033[0m")
    print(f"\033[92mControl_torso: {control_torso}\033[0m")
    if velocity_ik_takeover_requested:
        print("\033[92mPosition IK solve: disabled (velocity IK has exclusive authority)\033[0m")
    else:
        print(f"\033[92mUse two-stage IK: {use_two_stage_ik}\033[0m")

    current_pkg_path = get_package_path("motion_capture_ik")
    kuavo_assests_path = get_package_path("kuavo_assets")
    robot_version = os.environ.get('ROBOT_VERSION', '40')

    from tools.full_model_contract import full_model_relative_path, require_full_model
    full_model_relative_path(robot_version)  # Reject unsupported versions before config lookup.
    if not velocity_ik_takeover_requested:
        raise ValueError("Absolute Mapper requires velocity IK takeover")
    model_config_file = kuavo_assests_path + f"/config/kuavo_v{robot_version}/kuavo.json"

    # end_frames_name = ["torso", "l_hand_roll", "r_hand_roll", "l_forearm_pitch", "r_forearm_pitch"]
    import json
    with open(model_config_file, 'r') as f:
        model_config = json.load(f)
    overlay_path = os.path.join(get_package_path("wheel_arm_abs_vel"),
                                "config", f"model_s{robot_version}.json")
    with open(overlay_path, "r") as overlay_file:
        overlay = json.load(overlay_file)
    if any(not key.startswith("velocity_ik_") for key in overlay):
        raise ValueError("Non-absolute key in model contract")
    model_config.update(overlay)
    model_file = str(require_full_model(robot_version, kuavo_assests_path, overlay))
    if int(rospy.get_param("/robot_version", int(robot_version))) != int(robot_version):
        raise ValueError("Absolute Mapper/controller robot_version mismatch")
    if rospy.has_param("/urdfFile"):
        require_full_model(robot_version, kuavo_assests_path,
                           {"velocity_ik_urdf": rospy.get_param("/urdfFile")})
    rospy.set_param("~resolved_model_urdf", model_file)
    end_frames_name = model_config["end_frames_name_ik"]
    shoulder_frame_names = model_config["shoulder_frame_names"]
    upper_arm_length = model_config["upper_arm_length"]
    lower_arm_length = model_config["lower_arm_length"]
    num_arm_joints_var = model_config["NUM_ARM_JOINT"]
    eef_z_bias = model_config.get("eef_z_offset", 0.0)
    base_chest_offset_x = model_config.get("base_chest_offset_x", 0.0)
    hand_ref_length = model_config.get("hand_ref_length", 0.193)
    neutral_robot_geometry = None
    if velocity_ik_takeover_requested:
        if "velocity_ik_urdf" in model_config:
            from tools.target_geometry_contract import neutral_retarget_geometry
            neutral_robot_geometry = neutral_retarget_geometry(
                model_file, end_frames_name[0], shoulder_frame_names, end_frames_name[3:5])
            upper_arm_length = neutral_robot_geometry["upper_arm_length"]
            base_chest_offset_x = neutral_robot_geometry["base_chest_offset_x"]
    if not velocity_ik_takeover_requested and use_two_stage_ik and robot_version != "13": # 使用两阶段IK时，需要减去手腕参考长度
        lower_arm_length -= hand_ref_length
        print(f"using two-stage IK, adjust lower_arm_length: {lower_arm_length} m")

    # Use the same kuavo.json as dev's controller/joystick chain, with an
    # intentional CLI override. Do not let an old ROS master parameter silently
    # disagree with the launch-time SG100 process selection.
    end_effector_type = resolve(model_config, args.end_effector_type)
    rospy.set_param("~resolved_end_effector_type", end_effector_type)
    rospy.loginfo("Absolute end effector: %s (config=%s, override=%s)",
                  end_effector_type, model_config_file, args.end_effector_type)

    print(f"num_arm_joints_var: {num_arm_joints_var}")
    print(f"upper_arm_length: {upper_arm_length}, lower_arm_length: {lower_arm_length}")

    rospy.set_param("/quest3/shoulder_width", (neutral_robot_geometry or model_config).get("shoulder_width", 0.15))
    rospy.set_param("/quest3/base_height_offset", (neutral_robot_geometry or model_config).get("base_height_offset", 0.23))
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
    q_limit = None
    if velocity_ik_takeover_requested:
        # Quest retargeting needs link lengths, not a second IK solver. The
        # complete 21-D Pinocchio model/FK lives in the C++ velocity-IK process;
        # constructing legacy ArmIk here delayed raw-input availability and
        # created an unnecessary second kinematic owner.
        from tools.velocity_tcp_geometry import resolve_velocity_tcp_geometry
        velocity_geometry = resolve_velocity_tcp_geometry(
            model_config, robot_version,
            rospy.get_param("~velocity_tcp_forearm_length", None),
            rospy.get_param("~velocity_tcp_geometry_id", ""),
        )
        rospy.set_param("/quest3/teleop_tcp/forearm_retarget_length", velocity_geometry["forearm_retarget_length"])
        from tools.target_geometry_contract import model_geometry, LEGACY, WRIST
        # Same velocity-only model selection as the Pinocchio node.
        actual_ik_model = model_file
        if neutral_robot_geometry is not None:
            velocity_geometry["neutral_robot_geometry"] = neutral_robot_geometry
        tcp_transforms = {}
        for side in ("Left", "Right"):
            T = np.eye(4)
            xyz = np.asarray(rospy.get_param(
                f"/absolute_velocity_ik/{side.lower()}_tcp_translation", [0., 0., 0.]), dtype=float)
            rpy = np.asarray(rospy.get_param(
                f"/absolute_velocity_ik/{side.lower()}_tcp_rpy", [0., 0., 0.]), dtype=float)
            if xyz.shape != (3,) or rpy.shape != (3,):
                raise ValueError("Invalid Pinocchio TCP configuration")
            T[:3, :3] = rpy_to_matrix(rpy)
            T[:3, 3] = xyz
            T[2, 3] += model_config.get("eef_z_offset", 0.)
            tcp_transforms[side] = T
        velocity_geometry["ik_model"] = model_geometry(
            actual_ik_model, end_frames_name[1:3], tcp_transforms)
        velocity_geometry["operation_mode"] = rospy.get_param(
            "~velocity_target_geometry_mode", LEGACY)
        if velocity_geometry["operation_mode"] not in (LEGACY, WRIST):
            raise ValueError("Unknown velocity target geometry mode")
        velocity_geometry["source_origin_status"] = {
            "controller": "UNVERIFIED_GRIP_OR_AIM_PALM_LABEL",
            "hand_tracking": "HAND_PALM_LABEL_NOT_ANATOMICAL_WRIST_CALIBRATION"}
        velocity_geometry["robot_elbow_frames"] = end_frames_name[3:5]
        velocity_geometry["source_elbow_points"] = ["LeftArmLower", "RightArmLower"]
        velocity_geometry["fixed_axes"] = "quest_chest_robot_axes"
        velocity_geometry["units"] = {"length": "m", "angle": "rad"}
        rospy.set_param("/quest3/teleop_tcp/geometry_binding", velocity_geometry)
        print("\n" + "*"*10 + "VELOCITY IK MAPPER INFO" + "*"*10)
        print(
            "Legacy position IK model: disabled; retarget lengths from kuavo.json"
        )
        print(
            f"upper_arm_length: {upper_arm_length:.3f} m, "
            f"forearm_retarget_length: "
            f"{rospy.get_param('/quest3/teleop_tcp/forearm_retarget_length'):.3f} m"
        )
    else:
        # The legacy position-IK implementation remains available, but its
        # Drake modules are loaded only for that mode.  Velocity takeover must
        # register the Quest input contract immediately and has its own C++
        # Pinocchio model/solver.
        from ik.diff_ik import DiffIK
        from ik.torso_ik import ArmIk

        arm_min = np.array([-3.14, -0.70, -1.57, -1.57, -1.57, -1.57, -1.57, -3.14, -2.09, -1.57, -1.57, -1.57, -1.57, -1.57], dtype=float)
        arm_max = np.array([0.520, 2.09, 1.570, 0.000, 1.570, 1.570, 1.570, 0.7, 1.000, 1.570, 0.000, 1.570, 1.570, 1.570], dtype=float)
        if ik_type_idx == IkTypeIdx.DiffIK:
            arm_ik = DiffIK(
                model_file,
                end_frames_name,
                arm_idx=ctrl_arm_idx,
                q_limit=q_limit,
                meshcat=meshcat,
                eef_z_bias=eef_z_bias,
                shoulder_frame_names=shoulder_frame_names,
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
                shoulder_frame_names=shoulder_frame_names,
            )
            arm_ik.init_state(0.0, 0.0)

        print("\n" + "*"*10 + "IK ARM INFO" + "*"*10)
        arm_length_left, arm_length_right = arm_ik.get_arm_length()
        p_bS = arm_ik.get_two_frame_dis_vec(
            shoulder_frame_names[0], end_frames_name[0]
        )
        upper_arm_length = arm_ik.get_two_frame_dis(
            shoulder_frame_names[0], end_frames_name[3]
        )
        lower_arm_length = arm_ik.get_two_frame_dis(
            end_frames_name[3], end_frames_name[1]
        )
        shoulder_width_vec = arm_ik.get_two_frame_dis_vec(
            shoulder_frame_names[0], shoulder_frame_names[1]
        )
        print(
            f"upper_arm_length: {upper_arm_length:.3f} m, "
            f"lower_arm_length: {lower_arm_length:.3f} m"
        )
        # The unified TCP retarget uses the actual l4->EEF distance at q0.
        rospy.set_param(
            "/quest3/teleop_tcp/forearm_retarget_length",
            float(lower_arm_length),
        )
        print(f"shoulder_width: {shoulder_width_vec[1]/2} m")
        print(f"bias_chest_to_base_link: {p_bS} m")
    # rospy.set_param("/quest3/base_shoulder_x_bias", float(p_bS[0]))
    # rospy.set_param("/quest3/base_shoulder_y_bias", float(p_bS[1]))
    # rospy.set_param("/quest3/base_shoulder_z_bias", float(p_bS[2]))
    # rospy.set_param("/quest3/upper_arm_length", float(upper_arm_length))
    # rospy.set_param("/quest3/lower_arm_length", float(lower_arm_length))
    # rospy.set_param("/quest3/shoulder_width", float(shoulder_width))
    # print(f"\033[92mLeft Arm Length: {arm_length_left:.3f} m, Right Arm Length:{arm_length_right:.3f} m.\033[0m")
    print("*"*10 + "IK/MAPPER INFO END" + "*"*10 + "\n")

    ik_ros = IkRos(arm_ik, ctrl_arm_idx=ctrl_arm_idx, q_limit=q_limit, end_effector_type=end_effector_type, send_srv=send_srv, predict_gesture=predict_gesture, hand_reference_mode=hand_reference_mode, use_two_stage_ik=use_two_stage_ik, wheel_joystick_config=model_config.get("joystick_filter"))
