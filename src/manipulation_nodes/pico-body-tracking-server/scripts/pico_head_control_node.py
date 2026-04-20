#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PICO 头部控制节点。

支持四种模式：
1. fixed
2. auto_track_active
3. fixed_main_hand
4. vr_follow
"""

import math
from typing import Dict, Optional, Tuple

import numpy as np
import rospy
import tf.transformations as tft
from std_msgs.msg import String
from kuavo_msgs.msg import robotBodyMatrices, robotHeadMotionData
from kuavo_msgs.srv import SetHeadControlMode, SetHeadControlModeResponse


class HeadControlMode:
    FIXED = "fixed"
    AUTO_TRACK_ACTIVE = "auto_track_active"
    FIXED_MAIN_HAND = "fixed_main_hand"
    VR_FOLLOW = "vr_follow"

    ALL = {FIXED, AUTO_TRACK_ACTIVE, FIXED_MAIN_HAND, VR_FOLLOW}


class PicoHeadControlNode:
    def __init__(self):
        rospy.init_node("pico_head_control_node")

        self.mode = rospy.get_param("~mode", HeadControlMode.VR_FOLLOW)
        if self.mode not in HeadControlMode.ALL:
            rospy.logwarn("Invalid head mode '%s', fallback to vr_follow", self.mode)
            self.mode = HeadControlMode.VR_FOLLOW

        self.fixed_main_hand = rospy.get_param("~fixed_main_hand", "right").lower()
        if self.fixed_main_hand not in ("left", "right"):
            self.fixed_main_hand = "right"

        self.smoothing_factor = self._clamp(rospy.get_param("~smoothing_factor", 0.2), 0.0, 1.0)
        self.active_hand_threshold = rospy.get_param("~active_hand_threshold", 0.01)
        self.yaw_limit = rospy.get_param("~yaw_limit", [-80.0, 80.0])
        self.pitch_limit = rospy.get_param("~pitch_limit", [-25.0, 25.0])

        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.target_yaw = 0.0
        self.target_pitch = 0.0

        self.last_left_hand_pos = None
        self.last_right_hand_pos = None
        self.last_mode = self.mode
        self.mode_changed = True

        # 兼容旧链路：当外部头控开启时，PICO主节点会关闭内部头部发布
        rospy.set_param("/pico/use_external_head_control", True)
        rospy.on_shutdown(self._on_shutdown)

        self.head_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)
        rospy.Subscriber("/robot_body_matrices", robotBodyMatrices, self._on_robot_body_matrices)
        rospy.Subscriber("/pico/head_control_mode", String, self._on_mode)
        self.head_control_mode_service = rospy.Service(
            "/pico/set_head_control_mode",
            SetHeadControlMode,
            self._handle_set_head_control_mode,
        )

        rospy.loginfo(
            "PicoHeadControlNode started. mode=%s, fixed_main_hand=%s",
            self.mode,
            self.fixed_main_hand,
        )
        rospy.loginfo("Head control mode service started at /pico/set_head_control_mode")

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))

    def _on_mode(self, msg: String):
        mode = msg.data.strip().lower()
        if mode in HeadControlMode.ALL and mode != self.mode:
            self.last_mode = self.mode
            self.mode = mode
            self.mode_changed = True
            rospy.loginfo("Head mode switched: %s -> %s", self.last_mode, self.mode)
        elif mode not in HeadControlMode.ALL:
            rospy.logwarn("Ignore invalid head mode: %s", mode)

    def _handle_set_head_control_mode(self, req):
        response = SetHeadControlModeResponse()
        mode = req.mode.strip().lower()
        fixed_hand = req.fixed_hand.strip().lower()

        if mode not in HeadControlMode.ALL:
            response.success = False
            response.message = (
                f"Invalid mode: {req.mode}. Valid modes are: "
                "fixed, auto_track_active, fixed_main_hand, vr_follow"
            )
            response.current_mode = self.mode
            return response

        if mode == HeadControlMode.FIXED_MAIN_HAND:
            if fixed_hand not in ("left", "right"):
                response.success = False
                response.message = (
                    "For 'fixed_main_hand' mode, fixed_hand must be 'left' or 'right'"
                )
                response.current_mode = self.mode
                return response
            self.fixed_main_hand = fixed_hand
        else:
            self.fixed_main_hand = "right"

        if mode != self.mode:
            self.last_mode = self.mode
            self.mode = mode
            self.mode_changed = True
            rospy.loginfo("Head mode switched via service: %s -> %s", self.last_mode, self.mode)

        response.success = True
        response.message = (
            f"Head control mode set to: {mode}"
            + (f", fixed_hand: {self.fixed_main_hand}" if mode == HeadControlMode.FIXED_MAIN_HAND else "")
        )
        response.current_mode = self.mode
        return response

    def _parse_matrices(self, msg: robotBodyMatrices) -> Dict[str, np.ndarray]:
        matrices = {}
        if msg.num_matrices == 0:
            return matrices
        for idx, name in enumerate(msg.body_parts):
            start = idx * 16
            end = start + 16
            if end > len(msg.matrices_data):
                break
            matrix = np.array(msg.matrices_data[start:end], dtype=np.float64).reshape(4, 4)
            matrices[name] = matrix
        return matrices

    def _extract_xyz(self, matrix: np.ndarray) -> np.ndarray:
        return matrix[:3, 3]

    def _calculate_head_pose_from_hand(self, hand_pos: np.ndarray, head_pos: np.ndarray) -> Tuple[float, float]:
        dx, dy, dz = hand_pos - head_pos
        yaw = math.degrees(math.atan2(dy, dx))
        distance_xy = math.sqrt(dx * dx + dy * dy)
        pitch = -math.degrees(math.atan2(dz, distance_xy))
        yaw = self._clamp(yaw, float(self.yaw_limit[0]), float(self.yaw_limit[1]))
        pitch = self._clamp(pitch, float(self.pitch_limit[0]), float(self.pitch_limit[1]))
        return yaw, pitch

    def _calculate_vr_follow_pose(self, pelvis_mat: np.ndarray, head_mat: np.ndarray) -> Tuple[float, float]:
        rel_mat = np.matmul(np.linalg.inv(pelvis_mat), head_mat)
        roll, pitch, yaw = tft.euler_from_matrix(rel_mat)
        del roll
        yaw_deg = self._clamp(math.degrees(yaw), float(self.yaw_limit[0]), float(self.yaw_limit[1]))
        pitch_deg = self._clamp(math.degrees(pitch), float(self.pitch_limit[0]), float(self.pitch_limit[1]))
        return yaw_deg, pitch_deg

    def _detect_active_hand(
        self, left_pos: Optional[np.ndarray], right_pos: Optional[np.ndarray]
    ) -> Optional[str]:
        left_movement = 0.0
        right_movement = 0.0
        left_moving = False
        right_moving = False

        if left_pos is not None and self.last_left_hand_pos is not None:
            left_movement = float(np.linalg.norm(left_pos - self.last_left_hand_pos))
            left_moving = left_movement > self.active_hand_threshold
        if right_pos is not None and self.last_right_hand_pos is not None:
            right_movement = float(np.linalg.norm(right_pos - self.last_right_hand_pos))
            right_moving = right_movement > self.active_hand_threshold

        self.last_left_hand_pos = left_pos
        self.last_right_hand_pos = right_pos

        if left_moving and not right_moving:
            return "left"
        if right_moving and not left_moving:
            return "right"
        if left_moving and right_moving:
            return "left" if left_movement > right_movement else "right"
        return None

    def _smooth_update(self, target_yaw: float, target_pitch: float):
        if self.mode_changed:
            self.current_yaw = target_yaw
            self.current_pitch = target_pitch
            self.mode_changed = False

        self.current_yaw = (1.0 - self.smoothing_factor) * self.current_yaw + self.smoothing_factor * target_yaw
        self.current_pitch = (1.0 - self.smoothing_factor) * self.current_pitch + self.smoothing_factor * target_pitch
        self.target_yaw = self.current_yaw
        self.target_pitch = self.current_pitch

    def _publish(self):
        msg = robotHeadMotionData()
        msg.joint_data = [self.target_yaw, self.target_pitch]
        self.head_pub.publish(msg)

    def _on_robot_body_matrices(self, msg: robotBodyMatrices):
        matrices = self._parse_matrices(msg)
        if not matrices:
            return

        pelvis_mat = matrices.get("Pelvis")
        head_mat = matrices.get("HEAD")
        left_hand_mat = matrices.get("LEFT_HAND")
        right_hand_mat = matrices.get("RIGHT_HAND")

        if pelvis_mat is None or head_mat is None:
            return

        head_pos = self._extract_xyz(head_mat)
        left_hand_pos = self._extract_xyz(left_hand_mat) if left_hand_mat is not None else None
        right_hand_pos = self._extract_xyz(right_hand_mat) if right_hand_mat is not None else None

        target_yaw = self.target_yaw
        target_pitch = self.target_pitch

        if self.mode == HeadControlMode.FIXED:
            target_yaw, target_pitch = 0.0, 0.0
        elif self.mode == HeadControlMode.AUTO_TRACK_ACTIVE:
            active_hand = self._detect_active_hand(left_hand_pos, right_hand_pos)
            if active_hand == "left" and left_hand_pos is not None:
                target_yaw, target_pitch = self._calculate_head_pose_from_hand(left_hand_pos, head_pos)
            elif active_hand == "right" and right_hand_pos is not None:
                target_yaw, target_pitch = self._calculate_head_pose_from_hand(right_hand_pos, head_pos)
        elif self.mode == HeadControlMode.FIXED_MAIN_HAND:
            if self.fixed_main_hand == "left" and left_hand_pos is not None:
                target_yaw, target_pitch = self._calculate_head_pose_from_hand(left_hand_pos, head_pos)
            elif self.fixed_main_hand == "right" and right_hand_pos is not None:
                target_yaw, target_pitch = self._calculate_head_pose_from_hand(right_hand_pos, head_pos)
        elif self.mode == HeadControlMode.VR_FOLLOW:
            target_yaw, target_pitch = self._calculate_vr_follow_pose(pelvis_mat, head_mat)

        self._smooth_update(target_yaw, target_pitch)
        self._publish()

    def spin(self):
        rospy.spin()

    def _on_shutdown(self):
        # 退出时恢复旧链路，避免下次仅启动主节点时头控被禁用
        try:
            rospy.set_param("/pico/use_external_head_control", False)
        except Exception:
            pass


if __name__ == "__main__":
    try:
        node = PicoHeadControlNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
