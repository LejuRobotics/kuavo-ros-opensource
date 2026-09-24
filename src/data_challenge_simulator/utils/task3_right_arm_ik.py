#!/usr/bin/env python3
"""Online right-arm pose IK for Task 3."""

from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares, minimize
from scipy.spatial.transform import Rotation

from utils.hand_postures import internal_expansion_pose


@dataclass(frozen=True)
class RightArmIKResult:
    joints: tuple
    target_compact_finger_center: tuple


class Task3RightArmIK:
    """Solve an exact hand-base pose from live base, object, and arm state."""

    def __init__(self, scene_path):
        self.model = mujoco.MjModel.from_xml_path(str(Path(scene_path)))
        self.data = mujoco.MjData(self.model)
        self.base_body = self._required_id(
            mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_joint = int(self.model.body_jntadr[self.base_body])
        self.base_qpos = int(self.model.jnt_qposadr[base_joint])
        self.hand_body = self._required_id(
            mujoco.mjtObj.mjOBJ_BODY, "r_hand_base")
        joint_ids = np.asarray([
            self._required_id(
                mujoco.mjtObj.mjOBJ_JOINT,
                "zarm_r{}_joint".format(index))
            for index in range(1, 8)
        ], dtype=int)
        self.arm_qpos = np.asarray([
            self.model.jnt_qposadr[joint_id]
            for joint_id in joint_ids
        ], dtype=int)
        self.lower = self.model.jnt_range[joint_ids, 0].copy()
        self.upper = self.model.jnt_range[joint_ids, 1].copy()
        self.compact_finger_center_in_hand = (
            self._measure_compact_finger_center_in_hand())
        ring_height_id = self._required_id(
            mujoco.mjtObj.mjOBJ_NUMERIC, "task3_ring_height")
        self.ring_height = float(
            self.model.numeric_data[
                self.model.numeric_adr[ring_height_id]])

    def _required_id(self, object_type, name):
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise RuntimeError("model object is missing: {}".format(name))
        return int(object_id)

    def _measure_compact_finger_center_in_hand(self):
        self.data.qpos[:] = self.model.qpos0
        for name, value in internal_expansion_pose(0.0, "r").items():
            joint_id = self._required_id(
                mujoco.mjtObj.mjOBJ_JOINT, name)
            address = int(self.model.jnt_qposadr[joint_id])
            self.data.qpos[address] = float(value)
        mujoco.mj_forward(self.model, self.data)
        hand_position = self.data.xpos[self.hand_body]
        hand_rotation = self.data.xmat[self.hand_body].reshape(3, 3)
        finger_centers = []
        for finger in ("index", "middle", "little"):
            geom_id = self._required_id(
                mujoco.mjtObj.mjOBJ_GEOM,
                "r_{}_fingertip_collision".format(finger))
            finger_centers.append(self.data.geom_xpos[geom_id].copy())
        center_world = np.mean(finger_centers, axis=0)
        return hand_rotation.T @ (center_world - hand_position)

    def _pose(self, base_pose, right_joints):
        base_x, base_y, base_yaw = (float(value) for value in base_pose)
        self.data.qpos[:] = self.model.qpos0
        address = self.base_qpos
        self.data.qpos[address:address + 3] = (base_x, base_y, 0.0)
        self.data.qpos[address + 3:address + 7] = (
            math.cos(0.5 * base_yaw), 0.0, 0.0,
            math.sin(0.5 * base_yaw))
        self.data.qpos[self.arm_qpos] = right_joints
        mujoco.mj_forward(self.model, self.data)
        return (
            self.data.xpos[self.hand_body].copy(),
            Rotation.from_matrix(
                self.data.xmat[self.hand_body].reshape(3, 3).copy()),
        )

    def solve_above_ring(
            self, base_pose, ring_position, measured_arm,
            above_ring_clearance_m):
        """Keep the initialized palm orientation and center it above the ring."""
        measured = np.asarray(measured_arm, dtype=float)
        if measured.shape != (14,) or not np.all(np.isfinite(measured)):
            raise ValueError("measured_arm must contain 14 finite joints")
        seed = np.clip(measured[7:14], self.lower, self.upper)
        _, target_rotation = self._pose(base_pose, seed)
        ring = np.asarray(ring_position, dtype=float)
        target_finger_center = np.asarray((
            ring[0],
            ring[1],
            ring[2] + self.ring_height + float(above_ring_clearance_m),
        ))

        def equality(joints):
            hand_position, rotation = self._pose(base_pose, joints)
            finger_center = (
                hand_position +
                rotation.apply(self.compact_finger_center_in_hand))
            return np.concatenate((
                finger_center - target_finger_center,
                (target_rotation.inv() * rotation).as_rotvec(),
            ))

        def warm_start_residual(joints):
            residual = equality(joints)
            return np.concatenate((
                20.0 * residual[:3],
                residual[3:],
                0.001 * (joints - seed),
            ))

        warm_start = least_squares(
            warm_start_residual, seed,
            bounds=(self.lower, self.upper),
            max_nfev=2000, xtol=1e-11, ftol=1e-11, gtol=1e-11)
        joint_midpoint = 0.5 * (self.lower + self.upper)
        joint_span = self.upper - self.lower

        def objective(joints):
            seed_delta = joints - seed
            centered = (joints - joint_midpoint) / joint_span
            return float(
                0.5 * np.dot(seed_delta, seed_delta) +
                5.0 * np.dot(centered, centered))

        result = minimize(
            objective,
            warm_start.x,
            method="SLSQP",
            bounds=list(zip(self.lower, self.upper)),
            constraints=({"type": "eq", "fun": equality},),
            options={"ftol": 1e-12, "maxiter": 500, "disp": False})
        if not result.success:
            return None
        return RightArmIKResult(
            joints=tuple(float(value) for value in result.x),
            target_compact_finger_center=tuple(
                float(value) for value in target_finger_center),
        )
