#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立的头部控制节点。

head_control_mode:
  fixed       默认。不发布头部指令，把头留给其它控制（例如 tact）
  vr_follow   跟随 Quest 骨骼 Chest 相对 Head 的姿态。由 VR 节点调用服务切入；
              Chest→Head 超过 3 秒没有新 TF 时自动回到 fixed，不用缓存姿态继续发布
  track_hand  跟踪手，目标由 head_track_target 决定
head_track_target（仅 track_hand）:
  both_hands_midpoint  双手连线，hand_bias 决定默认偏左还是偏右，再按近期速度偏向动得更勤的手
  left_hand / right_hand  固定跟踪指定手
  active_hand  跟踪正在移动的手
平滑、限幅和换手参数从 config.json 读取；launch 传入的模式优先。
跟踪目标位于头部后方（相对 x <= 0 m）时保持当前头角；
目标回到前方至少 0.03 m 后恢复，避免 atan2 在 +/-180 度处左右跳变。

轮臂 zhead_1_joint 的父 link 是 waist_yaw_link，头部指令也是相对这个坐标系的
关节角。手和头都在 waist_yaw_link 下取值，腰转动时不会把腰的偏航重复加到头上。
"""

import json
import math
import os

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from kuavo_msgs.msg import headCtrlMode
from kuavo_msgs.srv import SetHeadControlMode, SetHeadControlModeResponse
from kuavo_ros_interfaces.msg import robotHeadMotionData
from std_msgs.msg import Empty

from head_control_manager import HeadControlManager, HeadControlMode


_DEFAULT_CONFIG = {
    "joint_limits": {"yaw": [-85, 85], "pitch": [-25, 25]},
    "smoothing_factor": 0.5,
    "active_hand_threshold": 0.015,
    "switch_ratio": 1.5,
    "switch_confirm_sec": 0.15,
    "release_sec": 0.40,
    "max_yaw_rate_deg_s": 100.0,
    "max_pitch_rate_deg_s": 50.0,
    "fixed_main_hand": "right",
}

_TRACK_MODES = {
    "fixed": HeadControlMode.FIXED,
    "auto_track_active": HeadControlMode.AUTO_TRACK_ACTIVE,
    "fixed_main_hand": HeadControlMode.FIXED_MAIN_HAND,
    "vr_follow": HeadControlMode.VR_FOLLOW,
}
_PUBLIC_MODES = {"vr_follow", "track_hand", "fixed"}
_PUBLIC_TARGETS = {"both_hands_midpoint", "left_hand", "right_hand", "active_hand"}
# mode=track_hand 时，服务的 fixed_hand 字段用来指定注视目标
_FIXED_HAND_TO_TARGET = {
    "left": "left_hand",
    "right": "right_hand",
    "left_hand": "left_hand",
    "right_hand": "right_hand",
    "both_hands_midpoint": "both_hands_midpoint",
    "active_hand": "active_hand",
    "auto_track_active": "active_hand",
    "fixed_left_hand": "left_hand",
    "fixed_right_hand": "right_hand",
}
# 服务和旧调用里，这些名字直接表示 track_hand 的目标
_TARGET_FROM_MODE = {
    "both_hands_midpoint": "both_hands_midpoint",
    "active_hand": "active_hand",
    "auto_track_active": "active_hand",
    "auto_track": "active_hand",
    "left_hand": "left_hand",
    "right_hand": "right_hand",
    "fixed_left_hand": "left_hand",
    "fixed_right_hand": "right_hand",
}

TRACK_ACTIVE_HAND = "active_hand"
TRACK_BOTH_HANDS_MIDPOINT = "both_hands_midpoint"
# Chest→Head 超过该时间没有新的 TF 发布，vr_follow 回到 fixed。
# lookupTransform(Time(0)) 在发布端退出后仍会返回缓冲区里的最后一帧。
DEFAULT_VR_TF_TIMEOUT_SEC = 3.0
# 单帧速度超过该值视为 TF 尖峰，不计入活动量
_MAX_HAND_SPEED_M_S = 2.0
# 两边活动量都低于该值时保持中点，避免静止噪声把头拉开
_ACTIVITY_FLOOR = 0.03


class _HandActivity:
    """一只手的平滑速度，以及会慢慢忘掉的活动量。"""

    def __init__(self):
        self.last_pos = None
        self.last_t = None
        self.speed = 0.0
        self.activity = 0.0

    def update(self, pos, now, hold_sec):
        if pos is None:
            self.last_pos = None
            self.last_t = None
            self.speed = 0.0
            self.activity = 0.0
            return 0.0
        if self.last_pos is None or self.last_t is None:
            self.last_pos = list(pos)
            self.last_t = now
            return self.activity
        dt = min(0.1, max(1e-3, now - self.last_t))
        dist = math.sqrt(sum((a - b) * (a - b) for a, b in zip(pos, self.last_pos)))
        inst = min(_MAX_HAND_SPEED_M_S, dist / dt)
        # 50 Hz 下约 3 帧跟上真实速度，单次抖动拉不动权重
        self.speed = 0.6 * self.speed + 0.4 * inst
        self.activity = self.activity * math.exp(-dt / hold_sec) + self.speed
        self.last_pos = list(pos)
        self.last_t = now
        return self.activity


def chest_head_tf_is_fresh(now_sec, stamp_sec, timeout_sec):
    """TF 时间戳够新才算 VR 正在发布。过期缓存不算。"""
    if stamp_sec is None or timeout_sec < 0.0:
        return False
    age = now_sec - stamp_sec
    return 0.0 <= age <= timeout_sec


def _load_head_config(config_path):
    config = dict(_DEFAULT_CONFIG)
    config["joint_limits"] = dict(_DEFAULT_CONFIG["joint_limits"])
    if not config_path or not os.path.exists(config_path):
        rospy.logwarn("Head track config not found at %s, using built-in defaults", config_path)
        return config
    with open(config_path, "r") as config_file:
        loaded = json.load(config_file)
    head_control = loaded.get("head_control", {})
    if isinstance(head_control, dict):
        for key, value in head_control.items():
            if key.startswith("_"):
                continue
            config[key] = value
    motion_range = loaded.get("head_motion_range", {})
    if isinstance(motion_range, dict):
        config["head_motion_range"] = motion_range
    return config


class HeadTrackHandNode:
    def __init__(self):
        rospy.init_node("head_control_node")

        default_config = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")
        config = _load_head_config(rospy.get_param("~config_path", default_config))

        mode_name = str(rospy.get_param("~head_control_mode", "")).strip().lower()
        if not mode_name:
            mode_name = str(config.get("head_control_mode", "fixed")).strip().lower()
        self._vr_tf_timeout = float(rospy.get_param("~vr_tf_timeout", DEFAULT_VR_TF_TIMEOUT_SEC))
        self._vr_tf_missing_since = None
        # 默认 fixed 不抢 /robot_head_motion_data。服务主动切到 fixed 时再按原逻辑发正前方。
        self._release_head_command = mode_name in ("", "fixed")
        track_target = str(rospy.get_param("~head_track_target", "")).strip().lower()
        if not track_target:
            track_target = str(config.get("head_track_target", TRACK_BOTH_HANDS_MIDPOINT)).strip().lower()
        self.head_control_mode = "vr_follow"
        self.head_track_target = TRACK_BOTH_HANDS_MIDPOINT
        self.track_target = TRACK_ACTIVE_HAND
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.base_frame = rospy.get_param("~base_frame", "waist_yaw_link")
        self.left_hand_frame = rospy.get_param("~left_hand_frame", "zarm_l7_end_effector")
        self.right_hand_frame = rospy.get_param("~right_hand_frame", "zarm_r7_end_effector")
        self.head_frames = rospy.get_param(
            "~head_frames", ["head_camera_base", "camera_base", "zhead_2_link"]
        )

        self.manager = HeadControlManager()
        # Resume only 3 cm in front of the head; avoid rear/front chatter.
        self.manager.hold_rear_targets = True
        self.manager.rear_target_held = False
        joint_limits = config.get("joint_limits", {})
        self.manager.set_joint_limits(
            joint_limits.get("yaw", [-85, 85]),
            joint_limits.get("pitch", [-25, 25]),
        )
        self.manager.set_smoothing_factor(config.get("smoothing_factor", 0.5))
        self.manager.set_active_hand_threshold(config.get("active_hand_threshold", 0.015))
        self.manager.set_switch_policy(
            config.get("switch_ratio", 1.5),
            config.get("switch_confirm_sec", 0.15),
            config.get("release_sec", 0.40),
        )
        self.manager.set_slew_rates(
            config.get("max_yaw_rate_deg_s", 100.0),
            config.get("max_pitch_rate_deg_s", 50.0),
        )
        self.hand_bias = float(rospy.get_param(
            "~hand_bias", config.get("hand_bias", 0.5)
        ))
        self.hand_bias = max(0.0, min(1.0, self.hand_bias))
        self.velocity_bias = float(rospy.get_param(
            "~velocity_bias", config.get("velocity_bias", 0.6)
        ))
        self.velocity_bias = max(0.0, min(1.0, self.velocity_bias))
        self.velocity_bias_hold_sec = float(rospy.get_param(
            "~velocity_bias_hold_sec", config.get("velocity_bias_hold_sec", 0.8)
        ))
        self.velocity_bias_hold_sec = max(0.05, self.velocity_bias_hold_sec)
        self._left_activity = _HandActivity()
        self._right_activity = _HandActivity()
        motion_range = config.get("head_motion_range", {})
        self.vr_yaw_range = motion_range.get("yaw", [-90, 90])
        self.vr_pitch_range = motion_range.get("pitch", [-20, 30])
        self.vr_chest_frame = rospy.get_param("~vr_chest_frame", "Chest")
        self.vr_head_frame = rospy.get_param("~vr_head_frame", "Head")
        self._published_mode = None
        self._apply_public(mode_name, track_target)

        self.listener = tf.TransformListener()
        self.head_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)
        self.head_pose_pub = rospy.Publisher("/robot_head_pose", PoseStamped, queue_size=10)
        self.mode_pub = rospy.Publisher("/quest3/head_control_mode", headCtrlMode, queue_size=10, latch=True)
        self.fixed_intent_pub = rospy.Publisher(
            "/quest3/head_fixed_forward_user_intent", Empty, queue_size=1, latch=False
        )
        self.mode_service = rospy.Service(
            "~set_head_control_mode",
            SetHeadControlMode,
            self._handle_set_mode,
        )
        rospy.set_param("~active", True)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            "head_control_node started. head_control_mode=%s head_track_target=%s rate=%.1f Hz "
            "hand_bias=%.2f velocity_bias=%.2f hold=%.2fs vr_tf_timeout=%.2fs. "
            "This node is the only publisher of /robot_head_motion_data.",
            self.head_control_mode,
            self.head_track_target,
            self.rate_hz,
            self.hand_bias,
            self.velocity_bias,
            self.velocity_bias_hold_sec,
            self._vr_tf_timeout,
        )

    def _apply_mode(self, mode_name, fixed_hand):
        mode = _TRACK_MODES.get(mode_name)
        if mode is None:
            rospy.logwarn("Invalid head track mode '%s', fallback to vr_follow", mode_name)
            mode = HeadControlMode.VR_FOLLOW
            mode_name = "vr_follow"
        if fixed_hand not in ("left", "right"):
            fixed_hand = "right"
        self.manager.set_mode(mode, fixed_hand)
        return mode_name, fixed_hand

    def _normalize_public(self, mode_name, target_name, fixed_hand):
        mode_name = str(mode_name or "").strip().lower()
        target_name = str(target_name or "").strip().lower()
        fixed_hand = str(fixed_hand or "").strip().lower()
        if mode_name in _TARGET_FROM_MODE:
            target_name = _TARGET_FROM_MODE[mode_name]
            mode_name = "track_hand"
        elif mode_name in ("fixed_main_hand", "fixed_main"):
            mode_name = "track_hand"
            if fixed_hand == "left":
                target_name = "left_hand"
            elif fixed_hand == "right":
                target_name = "right_hand"
        if mode_name == "track_hand" and fixed_hand in _FIXED_HAND_TO_TARGET:
            target_name = _FIXED_HAND_TO_TARGET[fixed_hand]
        if mode_name not in _PUBLIC_MODES:
            rospy.logwarn("Invalid head_control_mode '%s', fallback to vr_follow", mode_name)
            mode_name = "vr_follow"
        if target_name not in _PUBLIC_TARGETS:
            rospy.logwarn(
                "Invalid head_track_target '%s', fallback to both_hands_midpoint", target_name
            )
            target_name = TRACK_BOTH_HANDS_MIDPOINT
        return mode_name, target_name

    def _apply_public(self, mode_name, target_name, fixed_hand=""):
        mode_name, target_name = self._normalize_public(mode_name, target_name, fixed_hand)
        self.head_control_mode = mode_name
        self.head_track_target = target_name
        rospy.set_param("~head_control_mode", mode_name)
        rospy.set_param("~head_track_target", target_name)
        if mode_name == "fixed":
            self._apply_mode("fixed", "right")
            self.track_target = TRACK_ACTIVE_HAND
        elif mode_name == "track_hand" and target_name == "left_hand":
            self._apply_mode("fixed_main_hand", "left")
            self.track_target = "left_hand"
        elif mode_name == "track_hand" and target_name == "right_hand":
            self._apply_mode("fixed_main_hand", "right")
            self.track_target = "right_hand"
        elif mode_name == "track_hand" and target_name == TRACK_ACTIVE_HAND:
            self._apply_mode("auto_track_active", "right")
            self.track_target = TRACK_ACTIVE_HAND
        elif mode_name == "track_hand":
            self._apply_mode("auto_track_active", "right")
            self.track_target = TRACK_BOTH_HANDS_MIDPOINT
        else:
            self._apply_mode("vr_follow", "right")
            self.track_target = TRACK_ACTIVE_HAND
        rospy.loginfo(
            "Head control set to head_control_mode=%s head_track_target=%s",
            self.head_control_mode,
            self.head_track_target,
        )
        return self.head_control_mode, self.head_track_target

    def _handle_set_mode(self, req):
        response = SetHeadControlModeResponse()
        mode_name = req.mode.strip().lower() if req.mode else ""
        fixed_hand = req.fixed_hand.strip().lower() if req.fixed_hand else ""
        known = (
            mode_name in _PUBLIC_MODES
            or mode_name in _TARGET_FROM_MODE
            or mode_name in ("fixed_main_hand", "fixed_main")
        )
        if not known:
            response.success = False
            response.message = (
                "Invalid mode: %s. Valid modes are: vr_follow, fixed, track_hand, "
                "both_hands_midpoint, left_hand, right_hand, active_hand"
                % req.mode
            )
            response.current_mode = self._effective_mode_name()
            return response
        if mode_name in ("fixed_main_hand", "fixed_main") and fixed_hand not in ("left", "right"):
            response.success = False
            response.message = "For 'fixed_main_hand' mode, fixed_hand must be 'left' or 'right'"
            response.current_mode = self._effective_mode_name()
            return response
        if mode_name == "track_hand" and fixed_hand and fixed_hand not in _FIXED_HAND_TO_TARGET:
            response.success = False
            response.message = (
                "mode 为 track_hand 时，fixed_hand 只能是 both_hands_midpoint、left_hand、"
                "right_hand、active_hand；留空则保持当前注视目标"
            )
            response.current_mode = self._effective_mode_name()
            return response
        mode_name, _target_name = self._apply_public(mode_name, self.head_track_target, fixed_hand)
        self._vr_tf_missing_since = None
        self._release_head_command = False
        if mode_name == "fixed":
            self.fixed_intent_pub.publish(Empty())
        response.success = True
        response.current_mode = self._effective_mode_name()
        response.message = "Head control mode set to: %s" % response.current_mode
        self._publish_head_mode()
        return response

    def _lookup_xyz(self, frame):
        try:
            trans, _ = self.listener.lookupTransform(self.base_frame, frame, rospy.Time(0))
            return list(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def _lookup_head(self):
        for frame in self.head_frames:
            position = self._lookup_xyz(frame)
            if position is not None:
                return position
        return None

    def _update_both_hands_midpoint(self, left_pos, right_pos, head_pos):
        """注视左右手连线的中点。缺一只手时保持当前头角。"""
        if left_pos is None or right_pos is None or head_pos is None:
            rospy.logwarn_throttle(
                2.0, "both_hands_midpoint needs both hand TFs, holding the current head command"
            )
            return
        gaze = self._velocity_biased_point(left_pos, right_pos)
        target_yaw, target_pitch = self.manager.calculate_head_pose_from_hand(gaze, head_pos)
        if target_yaw is None or target_pitch is None:
            return
        self.manager.smooth_update(target_yaw, target_pitch)

    def _static_hand_point(self, left_pos, right_pos):
        """hand_bias 为右手权重：0 是左手，0.5 是中点，1 是右手。"""
        bias = self.hand_bias
        return [(1.0 - bias) * left + bias * right for left, right in zip(left_pos, right_pos)]

    def _velocity_biased_point(self, left_pos, right_pos):
        """默认按 hand_bias 落在两手之间，再按活动量偏移。活动量是平滑速度的泄漏积分。"""
        base = self._static_hand_point(left_pos, right_pos)
        if self.velocity_bias <= 0.0:
            return base
        now = rospy.Time.now().to_sec()
        left_activity = self._left_activity.update(left_pos, now, self.velocity_bias_hold_sec)
        right_activity = self._right_activity.update(right_pos, now, self.velocity_bias_hold_sec)
        total = left_activity + right_activity
        if total < _ACTIVITY_FLOOR:
            return base
        weighted = [
            (left_activity * left + right_activity * right) / total
            for left, right in zip(left_pos, right_pos)
        ]
        bias = self.velocity_bias
        return [(1.0 - bias) * point + bias * active for point, active in zip(base, weighted)]

    def _effective_mode_name(self):
        if self.manager.mode == HeadControlMode.VR_FOLLOW:
            return "vr_follow"
        if self.manager.mode == HeadControlMode.FIXED:
            return "fixed"
        if self.head_track_target == "left_hand":
            return "left_hand"
        if self.head_track_target == "right_hand":
            return "right_hand"
        if self.head_track_target == TRACK_BOTH_HANDS_MIDPOINT:
            return TRACK_BOTH_HANDS_MIDPOINT
        return TRACK_ACTIVE_HAND

    def _publish_head_mode(self):
        name = self._effective_mode_name()
        if name != self._published_mode:
            rospy.set_param("/head_control_mode", name)
            self._published_mode = name
        self.manager.publish_head_control_mode(
            self.mode_pub, name, self.manager.fixed_main_hand
        )

    @staticmethod
    def _normalize_degree_in_180(degree):
        if degree > 180:
            degree -= 180
        elif degree < -180:
            degree += 180
        return degree

    def _vr_chest_head_is_fresh(self):
        """Chest→Head 是否仍在发布。缓冲区里的最后一帧不算。"""
        try:
            latest = self.listener.getLatestCommonTime(self.vr_chest_frame, self.vr_head_frame)
        except Exception:
            return False
        if latest is None:
            return False
        try:
            stamp_sec = latest.to_sec()
        except AttributeError:
            stamp_sec = float(latest)
        return chest_head_tf_is_fresh(rospy.Time.now().to_sec(), stamp_sec, self._vr_tf_timeout)

    def _vr_follow_timed_out(self):
        """vr_follow 下 Chest→Head 持续缺失达到超时，则回到 fixed 且不再发头部指令。"""
        if self._vr_chest_head_is_fresh():
            self._vr_tf_missing_since = None
            return False
        now = rospy.Time.now().to_sec()
        if self._vr_tf_missing_since is None:
            self._vr_tf_missing_since = now
            return False
        if now - self._vr_tf_missing_since < self._vr_tf_timeout:
            return False
        self._vr_tf_missing_since = None
        self._release_head_command = True
        self._apply_public("fixed", self.head_track_target)
        rospy.logwarn(
            "Chest->Head TF missing for %.1fs, head control switches to fixed and stops publishing",
            self._vr_tf_timeout,
        )
        return True

    def _update_vr_follow(self):
        """头显随动：Quest 骨骼 Chest 到 Head 的姿态直接作为头部关节角。"""
        try:
            _trans, rot = self.listener.lookupTransform(
                self.vr_chest_frame, self.vr_head_frame, rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(
                2.0, "vr_follow waiting for TF %s -> %s", self.vr_chest_frame, self.vr_head_frame
            )
            return
        rpy_deg = [math.degrees(angle) for angle in tf.transformations.euler_from_quaternion(rot)]
        pitch = self._normalize_degree_in_180(round(rpy_deg[0], 2))
        yaw = self._normalize_degree_in_180(round(rpy_deg[1], 2))
        pitch = max(self.vr_pitch_range[0], min(self.vr_pitch_range[1], pitch))
        yaw = max(self.vr_yaw_range[0], min(self.vr_yaw_range[1], yaw))
        command = robotHeadMotionData()
        command.joint_data = [yaw, pitch]
        self.head_pub.publish(command)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.vr_chest_frame
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        self.head_pose_pub.publish(pose)
        self.manager.target_yaw = yaw
        self.manager.target_pitch = pitch
        self.manager.current_yaw = yaw
        self.manager.current_pitch = pitch

    def _on_shutdown(self):
        try:
            rospy.set_param("~active", False)
        except Exception:
            pass

    def spin(self):
        rate = rospy.Rate(self.rate_hz if self.rate_hz > 0.0 else 50.0)
        missing_tf_logged = False
        while not rospy.is_shutdown():
            if self.manager.mode == HeadControlMode.FIXED and self._release_head_command:
                self._publish_head_mode()
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
                continue
            if self.manager.mode == HeadControlMode.VR_FOLLOW:
                if self._vr_follow_timed_out():
                    self._publish_head_mode()
                    try:
                        rate.sleep()
                    except rospy.ROSInterruptException:
                        break
                    continue
                if not self._vr_chest_head_is_fresh():
                    rospy.logwarn_throttle(
                        2.0, "vr_follow waiting for fresh Chest -> Head TF, not publishing the cached pose"
                    )
                    self._publish_head_mode()
                    try:
                        rate.sleep()
                    except rospy.ROSInterruptException:
                        break
                    continue
                self._update_vr_follow()
                self._publish_head_mode()
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
                continue
            head_pos = self._lookup_head()
            left_pos = self._lookup_xyz(self.left_hand_frame)
            right_pos = self._lookup_xyz(self.right_hand_frame)
            if head_pos is None or (left_pos is None and right_pos is None):
                if not missing_tf_logged:
                    rospy.logwarn(
                        "Waiting for TF %s -> head %s and hands %s / %s",
                        self.base_frame,
                        self.head_frames,
                        self.left_hand_frame,
                        self.right_hand_frame,
                    )
                    missing_tf_logged = True
            else:
                if missing_tf_logged:
                    rospy.loginfo("Head track TF is available")
                    missing_tf_logged = False
                if (
                    self.track_target == TRACK_BOTH_HANDS_MIDPOINT
                    and self.manager.mode == HeadControlMode.AUTO_TRACK_ACTIVE
                ):
                    self._update_both_hands_midpoint(left_pos, right_pos, head_pos)
                else:
                    self.manager.update(
                        left_hand_tcp_pos=left_pos,
                        right_hand_tcp_pos=right_pos,
                        head_pos=head_pos,
                    )
                # 主动手还没动之前不发 0 度，避免一启动就把头拉回正前方
                if self.manager.last_smooth_time is not None:
                    self.manager.publish_head_command(self.head_pub)
            self._publish_head_mode()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


if __name__ == "__main__":
    try:
        HeadTrackHandNode().spin()
    except rospy.ROSInterruptException:
        pass
