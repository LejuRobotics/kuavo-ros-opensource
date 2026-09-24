#!/usr/bin/env python3
"""Live Task 3 descent planning with free wrist pose and tip-only goals."""

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
class Task3DescentAssessment:
    """One measured pre-descent error snapshot in ring coordinates."""

    fingertip_positions_m: tuple
    fingertip_xy_errors_m: tuple
    fingertip_heights_above_top_m: tuple
    center_xy_correction_m: tuple
    vertical_correction_m: float


@dataclass(frozen=True)
class Task3DescentResult:
    right_joints: tuple
    right_waypoints: tuple
    fingertip_radii_m: tuple
    fingertip_depths_m: tuple
    maximum_path_penetration_m: float
    maximum_path_contact: str
    pitch_deg: float
    roll_deg: float
    hand_xy_offset_m: tuple
    measured_error: Task3DescentAssessment
    hand_translation_correction_m: tuple


class Task3DescentPlanner:
    """Place the three compact fingertip fronts inside the open bore."""

    # These are search seeds, not required wrist targets.  The returned joint
    # state is accepted from its actual fingertip geometry, not pose error.
    CANDIDATES = (
        (32.0, -12.0, -0.006, -0.004),
        (32.0, -12.0, -0.002, -0.006),
        (28.0, -20.0, -0.002, -0.006),
        (32.0, -16.0, -0.006, -0.004),
        (24.0, -28.0, -0.004, -0.004),
        (28.0, -24.0, 0.000, -0.006),
    )
    INSERTION_MARGIN_M = 0.001

    def __init__(self, scene_path):
        self.ik = Task3RightArmIK(scene_path)
        self.model = self.ik.model
        self.data = self.ik.data
        self.compact_pose = internal_expansion_pose(0.0, "r")
        self.compact_addresses = {
            name: self.model.jnt_qposadr[self._required_id(
                mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in self.compact_pose
        }
        self.ring_body = self._required_id(
            mujoco.mjtObj.mjOBJ_BODY, "task3_hollow_cylinder")
        ring_joint = int(self.model.body_jntadr[self.ring_body])
        self.ring_qpos = int(self.model.jnt_qposadr[ring_joint])
        self.ring_geom_ids = {
            geom_id for geom_id in range(self.model.ngeom)
            if (mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
                ).startswith("task3_ring_segment_")
        }
        self.inner_radius = self._numeric("task3_inner_radius")
        self.ring_height = self._numeric("task3_ring_height")

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
                contacts.append((name, max(0.0, -float(contact.dist))))
        return contacts

    def _metrics(self, ring_position):
        tips = self._tip_geometry()
        ring = np.asarray(ring_position, dtype=float)
        top = ring[2] + self.ring_height
        radii = np.linalg.norm(tips[:, :2] - ring[:2], axis=1)
        depths = top - tips[:, 2]
        return tips, radii, depths

    @classmethod
    def assessment_from_tips(cls, tips, ring_position, ring_height):
        """Describe the single measured error used by the descent command.

        Positive XY values point from each measured fingertip front toward the
        bore center.  Positive height values mean that a front is still above
        the ring's upper plane.  The Z correction puts the highest front the
        existing 1 mm insertion margin below that plane.
        """
        tips = np.asarray(tips, dtype=float)
        ring = np.asarray(ring_position, dtype=float)
        if tips.shape != (len(FINGERS), 3):
            raise ValueError("tips must contain three finite XYZ positions")
        if ring.shape != (3,) or not (
                np.all(np.isfinite(tips)) and np.all(np.isfinite(ring))):
            raise ValueError("tips and ring_position must be finite")
        xy_errors = ring[:2] - tips[:, :2]
        heights = tips[:, 2] - (ring[2] + float(ring_height))
        center_xy_correction = np.mean(xy_errors, axis=0)
        vertical_correction = -(
            float(np.max(heights)) + cls.INSERTION_MARGIN_M)
        return Task3DescentAssessment(
            fingertip_positions_m=tuple(
                tuple(float(value) for value in point) for point in tips),
            fingertip_xy_errors_m=tuple(
                tuple(float(value) for value in error)
                for error in xy_errors),
            fingertip_heights_above_top_m=tuple(
                float(value) for value in heights),
            center_xy_correction_m=tuple(
                float(value) for value in center_xy_correction),
            vertical_correction_m=float(vertical_correction),
        )

    def assess(self, base_pose, ring_position, measured_arm):
        """Take the sole live geometry snapshot used before descent."""
        measured = np.asarray(measured_arm, dtype=float)
        if measured.shape != (14,) or not np.all(np.isfinite(measured)):
            raise ValueError("measured_arm must contain 14 finite joints")
        seed = np.clip(measured[7:14], self.ik.lower, self.ik.upper)
        self._set_configuration(base_pose, seed, ring_position)
        tips, _, _ = self._metrics(ring_position)
        return self.assessment_from_tips(
            tips, ring_position, self.ring_height)

    def _solve_candidate(
            self, base_pose, ring_position, seed, candidate, assessment):
        pitch_deg, roll_deg, offset_x, offset_y = candidate
        self._set_configuration(base_pose, seed, ring_position)
        hand_position = self.data.xpos[self.ik.hand_body].copy()
        hand_rotation = self.data.xmat[
            self.ik.hand_body].reshape(3, 3).copy()
        tips_world = np.asarray(assessment.fingertip_positions_m, dtype=float)
        tips_hand = (
            hand_rotation.T @ (tips_world - hand_position).T).T

        target_rotation = (
            Rotation.from_euler(
                "xy", (roll_deg, pitch_deg), degrees=True).as_matrix() @
            hand_rotation)
        tip_offsets = (target_rotation @ tips_hand.T).T
        current_tip_offsets = tips_world - hand_position
        pose_change_compensation = np.asarray((
            *(np.mean(current_tip_offsets[:, :2], axis=0) -
              np.mean(tip_offsets[:, :2], axis=0)),
            np.max(current_tip_offsets[:, 2]) -
            np.max(tip_offsets[:, 2]),
        ))
        measured_correction = np.asarray((
            *assessment.center_xy_correction_m,
            assessment.vertical_correction_m,
        ))
        hand_translation_correction = (
            measured_correction + pose_change_compensation +
            np.asarray((offset_x, offset_y, 0.0)))
        target_position = hand_position + hand_translation_correction
        target_rotation_object = Rotation.from_matrix(target_rotation)

        def residual(joints):
            position, rotation = self.ik._pose(base_pose, joints)
            return np.concatenate((
                30.0 * (position - target_position),
                (target_rotation_object.inv() * rotation).as_rotvec(),
                0.001 * (joints - seed),
            ))

        solved = least_squares(
            residual, seed, bounds=(self.ik.lower, self.ik.upper),
            max_nfev=5000, xtol=1e-12, ftol=1e-12, gtol=1e-12)
        joints = solved.x
        self._set_configuration(base_pose, joints, ring_position)
        _, radii, depths = self._metrics(ring_position)
        if (np.any(radii >= self.inner_radius) or
                np.any(depths <= 0.0) or self._ring_contacts()):
            return None

        maximum_penetration = 0.0
        maximum_contact = "none"
        for ratio in np.linspace(0.0, 1.0, 181):
            interpolated = (1.0 - ratio) * seed + ratio * joints
            self._set_configuration(
                base_pose, interpolated, ring_position)
            for name, penetration in self._ring_contacts():
                if penetration > maximum_penetration:
                    maximum_penetration = penetration
                    maximum_contact = name
        return Task3DescentResult(
            right_joints=tuple(float(value) for value in joints),
            right_waypoints=(),
            fingertip_radii_m=tuple(float(value) for value in radii),
            fingertip_depths_m=tuple(float(value) for value in depths),
            maximum_path_penetration_m=float(maximum_penetration),
            maximum_path_contact=maximum_contact,
            pitch_deg=float(pitch_deg),
            roll_deg=float(roll_deg),
            hand_xy_offset_m=(float(offset_x), float(offset_y)),
            measured_error=assessment,
            hand_translation_correction_m=tuple(
                float(value) for value in hand_translation_correction),
        )

    def _configuration_is_free(
            self, base_pose, ring_position, right_joints):
        self._set_configuration(base_pose, right_joints, ring_position)
        return not self._ring_contacts()

    def _edge_is_free(
            self, base_pose, ring_position, start, target,
            maximum_step=0.025):
        start = np.asarray(start, dtype=float)
        target = np.asarray(target, dtype=float)
        sample_count = max(
            1, int(math.ceil(np.linalg.norm(target - start) / maximum_step)))
        return all(
            self._configuration_is_free(
                base_pose, ring_position,
                (1.0 - ratio) * start + ratio * target)
            for ratio in np.linspace(0.0, 1.0, sample_count + 1)
        )

    @staticmethod
    def _branch(nodes, parents, index):
        branch = []
        while index >= 0:
            branch.append(nodes[index])
            index = parents[index]
        return branch[::-1]

    def _rrt_path(self, base_pose, ring_position, start, goal):
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        if self._edge_is_free(base_pose, ring_position, start, goal):
            return [start, goal]

        random = np.random.default_rng(20260915)
        lower = np.maximum(
            self.ik.lower, np.minimum(start, goal) - 0.65)
        upper = np.minimum(
            self.ik.upper, np.maximum(start, goal) + 0.65)
        tree_a, parent_a = [start], [-1]
        tree_b, parent_b = [goal], [-1]
        swapped = False
        meeting = None
        for _ in range(20000):
            sample = (
                goal if random.random() < 0.12
                else random.uniform(lower, upper))
            nearest_a = int(np.argmin([
                np.linalg.norm(node - sample) for node in tree_a]))
            direction = sample - tree_a[nearest_a]
            candidate = tree_a[nearest_a] + direction * min(
                1.0, 0.10 / max(np.linalg.norm(direction), 1e-12))
            if not self._edge_is_free(
                    base_pose, ring_position,
                    tree_a[nearest_a], candidate):
                tree_a, tree_b = tree_b, tree_a
                parent_a, parent_b = parent_b, parent_a
                swapped = not swapped
                continue
            tree_a.append(candidate)
            parent_a.append(nearest_a)
            new_a = len(tree_a) - 1
            while True:
                nearest_b = int(np.argmin([
                    np.linalg.norm(node - tree_a[new_a])
                    for node in tree_b]))
                direction = tree_a[new_a] - tree_b[nearest_b]
                candidate_b = tree_b[nearest_b] + direction * min(
                    1.0, 0.10 / max(np.linalg.norm(direction), 1e-12))
                if not self._edge_is_free(
                        base_pose, ring_position,
                        tree_b[nearest_b], candidate_b):
                    break
                tree_b.append(candidate_b)
                parent_b.append(nearest_b)
                new_b = len(tree_b) - 1
                if np.linalg.norm(tree_b[new_b] - tree_a[new_a]) < 1e-8:
                    meeting = (new_a, new_b)
                    break
            if meeting is not None:
                break
            tree_a, tree_b = tree_b, tree_a
            parent_a, parent_b = parent_b, parent_a
            swapped = not swapped
        if meeting is None:
            return None

        path_a = self._branch(tree_a, parent_a, meeting[0])
        path_b = self._branch(tree_b, parent_b, meeting[1])
        path = (
            path_a + path_b[-2::-1] if not swapped
            else path_b + path_a[-2::-1])
        shortened = [path[0]]
        index = 0
        while index < len(path) - 1:
            next_index = len(path) - 1
            while (next_index > index + 1 and
                   not self._edge_is_free(
                       base_pose, ring_position,
                       path[index], path[next_index])):
                next_index -= 1
            shortened.append(path[next_index])
            index = next_index
        return shortened

    def solve(self, base_pose, ring_position, measured_arm, assessment=None):
        measured = np.asarray(measured_arm, dtype=float)
        if measured.shape != (14,) or not np.all(np.isfinite(measured)):
            raise ValueError("measured_arm must contain 14 finite joints")
        seed = np.clip(measured[7:14], self.ik.lower, self.ik.upper)
        if assessment is None:
            assessment = self.assess(
                base_pose, ring_position, measured_arm)
        if not isinstance(assessment, Task3DescentAssessment):
            raise TypeError("assessment must be a Task3DescentAssessment")
        results = []
        for candidate in self.CANDIDATES:
            result = self._solve_candidate(
                base_pose, ring_position, seed, candidate, assessment)
            if result is not None:
                results.append(result)
        results.sort(key=lambda result: (
            result.maximum_path_penetration_m,
            np.linalg.norm(np.asarray(result.right_joints) - seed)))
        for result in results:
            path = self._rrt_path(
                base_pose, ring_position, seed,
                np.asarray(result.right_joints))
            if path is None:
                continue
            return Task3DescentResult(
                right_joints=result.right_joints,
                right_waypoints=tuple(
                    tuple(float(value) for value in waypoint)
                    for waypoint in path),
                fingertip_radii_m=result.fingertip_radii_m,
                fingertip_depths_m=result.fingertip_depths_m,
                maximum_path_penetration_m=0.0,
                maximum_path_contact="none",
                pitch_deg=result.pitch_deg,
                roll_deg=result.roll_deg,
                hand_xy_offset_m=result.hand_xy_offset_m,
                measured_error=result.measured_error,
                hand_translation_correction_m=(
                    result.hand_translation_correction_m),
            )
        return None
