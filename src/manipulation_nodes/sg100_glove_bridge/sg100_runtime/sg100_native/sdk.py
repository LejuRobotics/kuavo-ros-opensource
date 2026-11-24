"""Stateful SG100 solver API with left and right hand models."""

import time
from pathlib import Path

import numpy as np

from .config import (
    load_calibration,
    load_config_with_path,
    make_solver_config,
    select_input_profile,
)
from .filter import TargetStateFilter, output_guard
from .ik import SG100IK, default_fingers
from .quaternion import normalize_quat
from .transforms import (
    mcp_abduction_angles,
    module_to_tip,
    to_robot_frame_quaternions,
    transform_orientations,
    transform_positions,
    transform_sensor_axes,
)

FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]
JOINT_ORDER = [
    "thumb_j1",
    "thumb_j2",
    "thumb_j3",
    "index_j1",
    "index_j2",
    "index_j3",
    "middle_j1",
    "middle_j2",
    "little_j1",
    "little_j2",
    "little_j3",
]


class SG100SDK:
    def __init__(self, config_path=None, *, hand=None, clock=None):
        if hand is not None and hand not in ("right", "left"):
            raise ValueError("hand must be 'right' or 'left'")
        self._clock = clock or time.time
        if config_path is None:
            side = hand or "right"
            name = f"sg100_{side}_{12 if side == 'left' else 2}_v1_1_1.ini"
            config_path = Path(__file__).with_name("model") / name
        self.config_path, self.config = load_config_with_path(config_path)
        info = self.config.get("sdk", {})
        self.hand = info.get("side", hand or "right")
        if self.hand not in ("right", "left") or (hand is not None and self.hand != hand):
            raise ValueError("INI side does not match the selected hand")
        if any(
            info.get(key, expected) != expected
            for key, expected in [
                ("official_name", "SG100"),
                ("side", self.hand),
                ("version", "1_1_1"),
            ]
        ):
            raise ValueError("INI product, side or version mismatch")
        if self.config.get("input", {}).get("finger_order", FINGER_ORDER) != FINGER_ORDER:
            raise ValueError("unsupported input finger order")
        for section, key, expected in [
            ("input", "position_length", 15),
            ("input", "orientation_length", 20),
            ("output", "joint_count", 11),
            ("output", "joint_order", JOINT_ORDER),
        ]:
            if self.config.get(section, {}).get(key, expected) != expected:
                raise ValueError("INI interface mismatch: " + section + "." + key)
        urdf = Path(self.config.get("model", {}).get("urdf_path", f"{self.hand}_sg100.urdf"))
        self.urdf_path = urdf if urdf.is_absolute() else self.config_path.parent / urdf
        self.in_tf = load_calibration(
            select_input_profile(self.config.get("input_transform", {})), self.config_path.parent
        )
        self.finger_order = list(self.config.get("input", {}).get("finger_order", FINGER_ORDER))
        self.sg_indices = np.asarray(
            self.config.get("sg100_input_indices", [0, 1, 2, 3]), dtype=int
        )
        self.sg_finger_names = [self.finger_order[i] for i in self.sg_indices]
        self.sg_finger_names[-1] = "little"
        self.use_orientation = bool(self.in_tf.get("use_orientation", True))
        self.mcp_aa_direct_enabled = bool(
            self.config.get("solver", {}).get("mcp_aa_direct", {}).get("enabled", False)
        )
        self.mcp_aa_anchor = normalize_quat(
            np.asarray(
                self.config.get(
                    "mcp_aa_anchor_wxyz", [0.7071067811865476, 0.0, 0.0, -0.7071067811865476]
                ),
                dtype=float,
            )
        )
        self.reset()

    def reset(self):
        self.ik = SG100IK(
            urdf_path=self.urdf_path,
            fingers=default_fingers(self.hand),
            cfg=make_solver_config(self.config),
        )
        self._output_q_indices = np.asarray([self.ik.qidx(jn) for jn in JOINT_ORDER], dtype=int)
        self.target_filter = TargetStateFilter(self.config.get("target_state_filter", {}))
        self.q_prev = self.ik.make_neutral()
        self.quat_prev = None
        self.quat_ref = None
        self.t0 = self._clock()

    def solve(self, position, orientation):
        position = np.asarray(position, dtype=float).reshape(-1)
        orientation = np.asarray(orientation, dtype=float).reshape(-1)
        if position.size != 15 or orientation.size != 20:
            raise ValueError("expected 15 position and 20 orientation values")
        if not np.isfinite(position).all() or not np.isfinite(orientation).all():
            raise ValueError("input must contain finite values")
        t_now = self._clock() - self.t0
        pos = position.reshape(5, 3).copy()
        quat = orientation.reshape(5, 4).copy()
        pos += np.asarray(self.in_tf.get("raw_xyz_offset_m", [0.0, 0.0, 0.0]), dtype=float).reshape(
            3
        )
        pos, quat = transform_sensor_axes(pos, quat, self.in_tf)
        anchor_quat = normalize_quat(
            np.asarray(
                self.config.get("orientation_anchor_quat_wxyz", [1.0, 0.0, 0.0, 0.0]), dtype=float
            )
        )
        quat, self.quat_prev, self.quat_ref = transform_orientations(
            quat,
            self.quat_prev,
            self.quat_ref,
            self.in_tf,
            anchor_quat=anchor_quat,
            finger_order=self.finger_order,
        )
        pos, quat = module_to_tip(pos, quat, self.in_tf.get("module_to_tip", {}), self.finger_order)
        pos = transform_positions(pos, self.in_tf, self.finger_order)
        if self.in_tf.get("negate_x_before_ik", False):
            pos = pos.copy()
            pos[:, 0] *= -1.0
        reach = float(self.in_tf.get("max_reach_m", 0.5))
        valid_mask = np.linalg.norm(pos, axis=1) < reach
        pos, target_filter_state = self.target_filter.update(pos, t_now, valid_mask=valid_mask)
        pos = pos[self.sg_indices]
        quat = quat[self.sg_indices]
        valid_mask = valid_mask[self.sg_indices]
        q_init = self.q_prev.copy()
        if self.ik.last_position_ik_q is not None:
            for finger in self.ik.fingers:
                if finger.name == "thumb":
                    q_init[finger.active_q_idx] = self.ik.last_position_ik_q[finger.active_q_idx]
        tempx = pos[:, 0].copy()
        pos[:, 0] = pos[:, 1]
        pos[:, 1] = -tempx
        quat = to_robot_frame_quaternions(quat)
        mcp_aa_angles = None
        if self.mcp_aa_direct_enabled:
            mcp_aa_angles = mcp_abduction_angles(quat, self.mcp_aa_anchor, self.sg_finger_names)
        q_sol = self.ik.solve(
            pos,
            quat_wxyz=quat if self.use_orientation else None,
            q_init=q_init,
            valid_mask=valid_mask,
            mcp_aa_angles=mcp_aa_angles,
            mcp_aa_target_quat_wxyz=quat if self.use_orientation else None,
        )
        q_sol = output_guard(
            q_sol, self.q_prev, target_filter_state, self.config.get("q_output_guard", {})
        )
        if self.config.get("little_force_zero", False):
            q_sol = q_sol.copy()
            little = next((f for f in self.ik.fingers if f.name == "little"), None)
            if little is not None:
                for name in little.joints:
                    q_sol[self.ik.qidx(name)] = 0.0
        self.q_prev = q_sol
        return np.asarray(q_sol[self._output_q_indices], dtype=float).copy()
