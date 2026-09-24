#!/usr/bin/env python3
"""Task 3 second IK for the accepted object-relative insertion pose."""

from dataclasses import dataclass
import math

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from utils.hand_postures import internal_expansion_pose
from utils.task3_right_arm_ik import Task3RightArmIK


FINGERS = (
    ("index", "r_index_link3"),
    ("middle", "r_middle_link2"),
    ("little", "r_little_link3"),
)


@dataclass(frozen=True)
class Task3SecondIKResult:
    """Exact-pose IK target and its compact-hand geometry."""

    right_joints: tuple
    right_waypoints: tuple
    hand_position_m: tuple
    hand_quat_xyzw: tuple
    fingertip_positions_m: tuple
    fingertip_radii_m: tuple
    fingertip_depths_m: tuple
    nearest_joint_limit_margin_rad: float
    target_hand_position_m: tuple
    target_hand_quat_xyzw: tuple


class Task3SecondIKPlanner:
    """Solve the accepted ring-relative second IK and verify its path."""

    TARGET_HAND_OFFSET_FROM_RING_M = np.asarray(
        (-0.098195, -0.001697, 0.210822), dtype=float)
    TARGET_HAND_QUAT_XYZW = np.asarray(
        (0.50000168, 0.50000255, 0.49999842, 0.49999734), dtype=float)
    # Same pattern as Task 1's GRASP_TRACKING_BIAS_WORLD: this is the
    # measured actual-minus-command steady offset from the uncompensated
    # Task 3 second IK.  Subtract it from the desired position before IK.
    TRACKING_POSITION_BIAS_WORLD_M = np.asarray(
        (-0.000109, -0.001916, 0.011914), dtype=float)
    # measured orientation = commanded orientation * tracking rotation
    TRACKING_ROTATION_QUAT_XYZW = np.asarray(
        (-0.0802403100, -0.0073438200, 0.0423728100, 0.9958490300),
        dtype=float,
    )
    MAX_POSITION_ERROR_M = 5e-4
    MAX_ROTATION_ERROR_RAD = math.radians(0.2)
    MAXIMUM_PATH_STEP_RAD = 0.025

    def __init__(self, scene_path, ring_name="task3_hollow_cylinder"):
        self.ik = Task3RightArmIK(scene_path)
        self.model = self.ik.model
        self.data = self.ik.data
        self.compact_pose = internal_expansion_pose(0.0, "r")
        self.compact_addresses = {
            name: int(self.model.jnt_qposadr[self._required_id(
                mujoco.mjtObj.mjOBJ_JOINT, name)])
            for name in self.compact_pose
        }
        self.ring_name = str(ring_name)
        self.ring_body = self._required_id(
            mujoco.mjtObj.mjOBJ_BODY, self.ring_name)
        ring_joint = int(self.model.body_jntadr[self.ring_body])
        self.ring_qpos = int(self.model.jnt_qposadr[ring_joint])
        geom_prefix = (
            "task3_ring_segment_" if
            self.ring_name == "task3_hollow_cylinder" else
            self.ring_name + "_segment_")
        self.ring_geom_ids = {
            geom_id for geom_id in range(self.model.ngeom)
            if (mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
                ).startswith(geom_prefix)
        }
        self.inner_radius = self._numeric("task3_inner_radius")
        self.ring_height = self._numeric("task3_ring_height")
        self.target_rotation = Rotation.from_quat(
            self.TARGET_HAND_QUAT_XYZW)
        self.tracking_rotation = Rotation.from_quat(
            self.TRACKING_ROTATION_QUAT_XYZW)

    def _required_id(self, object_type, name):
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise RuntimeError("model object is missing: {}".format(name))
        return int(object_id)

    def _numeric(self, name):
        object_id = self._required_id(mujoco.mjtObj.mjOBJ_NUMERIC, name)
        return float(self.model.numeric_data[self.model.numeric_adr[object_id]])

    def _set_configuration(self, base_pose, right_joints, ring_position):
        self.ik._pose(base_pose, np.asarray(right_joints, dtype=float))
        for name, value in self.compact_pose.items():
            self.data.qpos[self.compact_addresses[name]] = float(value)
        address = self.ring_qpos
        self.data.qpos[address:address + 3] = ring_position
        self.data.qpos[address + 3:address + 7] = (1.0, 0.0, 0.0, 0.0)
        mujoco.mj_forward(self.model, self.data)

    def _tip_geometry(self):
        points = []
        for finger, distal_body_name in FINGERS:
            geom_id = self._required_id(
                mujoco.mjtObj.mjOBJ_GEOM,
                "r_{}_fingertip_collision".format(finger))
            body_id = self._required_id(
                mujoco.mjtObj.mjOBJ_BODY, distal_body_name)
            center = self.data.geom_xpos[geom_id].copy()
            direction = center - self.data.xpos[body_id]
            direction /= np.linalg.norm(direction)
            rotation = self.data.geom_xmat[geom_id].reshape(3, 3)
            support = float(np.sum(
                self.model.geom_size[geom_id] *
                np.abs(rotation.T @ direction)))
            points.append(center + support * direction)
        return np.asarray(points)

    def _ring_contacts(self):
        contacts = []
        for contact in self.data.contact:
            if contact.geom1 in self.ring_geom_ids:
                other = contact.geom2
            elif contact.geom2 in self.ring_geom_ids:
                other = contact.geom1
            else:
                continue
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, other) or ""
            if name.startswith("r_"):
                contacts.append(name)
        return contacts

    def _direct_path_is_free(
            self, base_pose, ring_position, start, target):
        distance = np.linalg.norm(target - start)
        sample_count = max(
            1, int(math.ceil(distance / self.MAXIMUM_PATH_STEP_RAD)))
        for ratio in np.linspace(0.0, 1.0, sample_count + 1):
            joints = (1.0 - ratio) * start + ratio * target
            self._set_configuration(base_pose, joints, ring_position)
            if self._ring_contacts():
                return False
        return True

    def reference_pose(self, ring_position):
        """Return the user-approved final hand pose for the current ring."""
        ring = np.asarray(ring_position, dtype=float)
        if ring.shape != (3,) or not np.all(np.isfinite(ring)):
            raise ValueError("ring_position must contain three finite values")
        return (
            ring + self.TARGET_HAND_OFFSET_FROM_RING_M,
            self.target_rotation,
        )

    def feedforward_command_pose(self, ring_position):
        """Return the one-shot Task-1-style compensated IK request."""
        desired_position, desired_rotation = self.reference_pose(ring_position)
        return (
            desired_position - self.TRACKING_POSITION_BIAS_WORLD_M,
            desired_rotation * self.tracking_rotation.inv(),
        )

    def measured_pose(self, base_pose, ring_position, right_joints):
        """Evaluate the actual compact-hand pose without solving IK."""
        self._set_configuration(base_pose, right_joints, ring_position)
        hand_position = self.data.xpos[self.ik.hand_body].copy()
        hand_rotation = Rotation.from_matrix(
            self.data.xmat[self.ik.hand_body].reshape(3, 3).copy())
        return hand_position, hand_rotation

    def solve_target(
            self, base_pose, ring_position, measured_arm,
            target_hand_position, target_hand_rotation,
            validate_insertion=True):
        """Solve one exact target pose from the latest measured arm."""
        measured = np.asarray(measured_arm, dtype=float)
        if measured.shape != (14,) or not np.all(np.isfinite(measured)):
            raise ValueError("measured_arm must contain 14 finite joints")
        ring = np.asarray(ring_position, dtype=float)
        if ring.shape != (3,) or not np.all(np.isfinite(ring)):
            raise ValueError("ring_position must contain three finite values")
        target_hand_position = np.asarray(target_hand_position, dtype=float)
        if (target_hand_position.shape != (3,) or
                not np.all(np.isfinite(target_hand_position))):
            raise ValueError(
                "target_hand_position must contain three finite values")
        if not isinstance(target_hand_rotation, Rotation):
            raise TypeError("target_hand_rotation must be a Rotation")
        seed = np.clip(measured[7:14], self.ik.lower, self.ik.upper)

        def residual(joints):
            position, rotation = self.ik._pose(base_pose, joints)
            return np.concatenate((
                30.0 * (position - target_hand_position),
                (target_hand_rotation.inv() * rotation).as_rotvec(),
                0.001 * (joints - seed),
            ))

        solved = least_squares(
            residual, seed, bounds=(self.ik.lower, self.ik.upper),
            max_nfev=5000, xtol=1e-12, ftol=1e-12, gtol=1e-12)
        joints = np.asarray(solved.x, dtype=float)
        hand_position, hand_rotation = self.ik._pose(base_pose, joints)
        position_error = np.linalg.norm(
            hand_position - target_hand_position)
        rotation_error = (
            target_hand_rotation.inv() * hand_rotation).magnitude()
        if (position_error > self.MAX_POSITION_ERROR_M or
                rotation_error > self.MAX_ROTATION_ERROR_RAD):
            return None
        if (validate_insertion and not self._direct_path_is_free(
                base_pose, ring, seed, joints)):
            return None

        self._set_configuration(base_pose, joints, ring)
        tips = self._tip_geometry()
        radii = np.linalg.norm(tips[:, :2] - ring[:2], axis=1)
        depths = ring[2] + self.ring_height - tips[:, 2]
        if (validate_insertion and (
                np.any(radii >= self.inner_radius) or
                np.any(depths <= 0.0) or self._ring_contacts())):
            return None
        margin = np.minimum(
            joints - self.ik.lower, self.ik.upper - joints)
        return Task3SecondIKResult(
            right_joints=tuple(float(value) for value in joints),
            right_waypoints=(
                tuple(float(value) for value in seed),
                tuple(float(value) for value in joints),
            ),
            hand_position_m=tuple(float(value) for value in hand_position),
            hand_quat_xyzw=tuple(
                float(value) for value in hand_rotation.as_quat()),
            fingertip_positions_m=tuple(
                tuple(float(value) for value in tip) for tip in tips),
            fingertip_radii_m=tuple(float(value) for value in radii),
            fingertip_depths_m=tuple(float(value) for value in depths),
            nearest_joint_limit_margin_rad=float(np.min(margin)),
            target_hand_position_m=tuple(
                float(value) for value in target_hand_position),
            target_hand_quat_xyzw=tuple(
                float(value) for value in target_hand_rotation.as_quat()),
        )

    def solve(self, base_pose, ring_position, measured_arm):
        """Solve the uncompensated user-approved ring-relative pose."""
        target_position, target_rotation = self.reference_pose(ring_position)
        return self.solve_target(
            base_pose, ring_position, measured_arm,
            target_position, target_rotation)

    def solve_feedforward(self, base_pose, ring_position, measured_arm):
        """Solve one biased IK request, matching Task 1 execution semantics."""
        command_position, command_rotation = self.feedforward_command_pose(
            ring_position)
        return self.solve_target(
            base_pose, ring_position, measured_arm,
            command_position, command_rotation,
            validate_insertion=False,
        )
