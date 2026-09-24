#!/usr/bin/env python3
"""Live-pose docking selection and bimanual IK planning for Task 2."""

from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares, minimize
from scipy.spatial.transform import Rotation

from utils.hand_postures import box_gripper_pose


BOX_NAMES = ("box_1", "box_2")
FINGERS = ("thumb", "index", "middle", "little")


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_xyzw(quaternion):
    return Rotation.from_quat(quaternion).as_euler("xyz")[2]


@dataclass(frozen=True)
class DockingTarget:
    box_name: str
    x: float
    y: float
    yaw: float
    cost: float
    already_comfortable: bool


@dataclass(frozen=True)
class ArmIKResult:
    joints: tuple
    r7_position: tuple
    r7_rotation: tuple
    position_error_m: float
    orientation_error_deg: float


@dataclass(frozen=True)
class PickPlan:
    box_name: str
    above_joints: tuple
    contact_joints: tuple
    lift_waypoints: tuple


def select_front_docking_target(base_pose, box_poses, config, docking_bases=None):
    """Choose a box using chassis motion only; no arm IK participates.

    The choice always comes from the live poses, because a box already moved
    to the destination conveyor must not be selected again.  ``docking_bases``
    maps a box name to the saved layout's docking base pose; when it is given
    the returned coordinates come from there, so the chassis drives to the
    pose the offline generator verified while the IK below still solves
    against the live box poses.
    """
    base_x, base_y, base_yaw = (float(value) for value in base_pose)
    standoff = float(config["front_standoff_m"])
    longitudinal_tolerance = float(
        config["comfortable_longitudinal_tolerance_m"])
    yaw_tolerance = math.radians(
        float(config["comfortable_yaw_tolerance_deg"]))
    yaw_cost_weight = float(config["yaw_cost_weight_m_per_rad"])
    if standoff <= 0.0:
        raise ValueError("front_standoff_m must be positive")

    targets = []
    for box_name in BOX_NAMES:
        position, quaternion = box_poses[box_name]
        box_rotation = Rotation.from_quat(quaternion).as_matrix()
        box_yaw = yaw_from_xyzw(quaternion)
        live_position = (
            np.asarray(position, dtype=float) -
            standoff * box_rotation[:, 0]
        )
        delta_world = np.array(
            (base_x - live_position[0], base_y - live_position[1]))
        delta_box = box_rotation[:2, :2].T @ delta_world
        yaw_error = normalize_angle(base_yaw - box_yaw)
        comfortable = (
            abs(delta_box[0]) <= longitudinal_tolerance and
            abs(yaw_error) <= yaw_tolerance
        )
        cost = float(
            np.linalg.norm(delta_world) +
            yaw_cost_weight * abs(yaw_error))
        if docking_bases is None:
            target_x, target_y = live_position[0], live_position[1]
            target_yaw = box_yaw
        else:
            saved = docking_bases[box_name]
            target_x, target_y, target_yaw = (
                float(saved[0]), float(saved[1]), float(saved[2]))
        targets.append(DockingTarget(
            box_name=box_name,
            x=float(target_x),
            y=float(target_y),
            yaw=float(target_yaw),
            cost=cost,
            already_comfortable=bool(comfortable),
        ))

    comfortable_targets = [
        target for target in targets if target.already_comfortable]
    return min(
        comfortable_targets if comfortable_targets else targets,
        key=lambda target: (target.cost, target.box_name),
    )


class Task2PickPlanner:
    """Solve the complete pre-motion Task 2 IK sequence."""

    def __init__(self, scene_path=None, ik_config=None):
        if scene_path is None:
            package_dir = Path(__file__).resolve().parents[1]
            scene_path = package_dir / (
                "models/biped_s400062/xml/task2.xml")
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        self.config = dict(ik_config or {})
        self.base_body_id = self._required_id(
            mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_joint_id = int(self.model.body_jntadr[self.base_body_id])
        self.base_qpos_address = int(self.model.jnt_qposadr[base_joint_id])

        self.arm_qpos_addresses = {}
        self.arm_lower = {}
        self.arm_upper = {}
        self.r7_body_ids = {}
        self.hand_body_ids = {}
        self.hand_qpos_addresses = {}
        self.hand_geom_ids = {}
        self.pad_geom_ids = {}
        for side in ("l", "r"):
            joint_ids = np.asarray([
                self._required_id(
                    mujoco.mjtObj.mjOBJ_JOINT,
                    "zarm_{}{}_joint".format(side, index))
                for index in range(1, 8)
            ], dtype=int)
            self.arm_qpos_addresses[side] = np.asarray([
                self.model.jnt_qposadr[joint_id]
                for joint_id in joint_ids
            ], dtype=int)
            self.arm_lower[side] = (
                self.model.jnt_range[joint_ids, 0].copy() + 1e-5)
            self.arm_upper[side] = (
                self.model.jnt_range[joint_ids, 1].copy() - 1e-5)
            self.r7_body_ids[side] = self._required_id(
                mujoco.mjtObj.mjOBJ_BODY,
                "zarm_{}7_link".format(side))
            self.hand_qpos_addresses[side] = {
                name: int(self.model.jnt_qposadr[self._required_id(
                    mujoco.mjtObj.mjOBJ_JOINT, name)])
                for name in box_gripper_pose(0.0, side)
            }
            hand_body_id = self._required_id(
                mujoco.mjtObj.mjOBJ_BODY, "{}_hand_base".format(side))
            self.hand_body_ids[side] = hand_body_id
            self.hand_geom_ids[side] = self._descendant_geom_ids(hand_body_id)
            self.pad_geom_ids[side] = {
                self._required_id(
                    mujoco.mjtObj.mjOBJ_GEOM,
                    "{}_{}_fingertip_collision".format(side, finger))
                for finger in FINGERS
            }

        self.box_body_ids = {
            name: self._required_id(mujoco.mjtObj.mjOBJ_BODY, name)
            for name in BOX_NAMES
        }
        self.box_qpos_addresses = {}
        self.box_geom_ids = {}
        self.assigned_wall_geom_ids = {}
        for name, body_id in self.box_body_ids.items():
            joint_id = int(self.model.body_jntadr[body_id])
            self.box_qpos_addresses[name] = int(
                self.model.jnt_qposadr[joint_id])
            self.box_geom_ids[name] = self._descendant_geom_ids(body_id)
            self.assigned_wall_geom_ids[name] = {
                "l": self._required_id(
                    mujoco.mjtObj.mjOBJ_GEOM, "{}_left_wall".format(name)),
                "r": self._required_id(
                    mujoco.mjtObj.mjOBJ_GEOM, "{}_right_wall".format(name)),
            }

        table_ids = [
            self._required_id(mujoco.mjtObj.mjOBJ_BODY, name)
            for name in ("work_table", "destination_conveyor")
        ]
        self.static_obstacle_geom_ids = set()
        for body_id in table_ids:
            self.static_obstacle_geom_ids.update(
                self._descendant_geom_ids(body_id))
        self.robot_geom_ids = self._descendant_geom_ids(self.base_body_id)
        self._fixed_jaw_tcp_in_r7 = {
            side: self._measure_fixed_jaw_tcp_in_r7(side)
            for side in ("l", "r")
        }
        self._hand_base_origin_in_r7 = {
            side: self._measure_body_origin_in_r7(
                side, self.hand_body_ids[side])
            for side in ("l", "r")
        }

    def _required_id(self, object_type, name):
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise RuntimeError("model object is missing: {}".format(name))
        return int(object_id)

    def _descendant_geom_ids(self, root_body_id):
        body_ids = {int(root_body_id)}
        for body_id in range(1, self.model.nbody):
            if int(self.model.body_parentid[body_id]) in body_ids:
                body_ids.add(body_id)
        return {
            geom_id for geom_id in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom_id]) in body_ids
        }

    def _geom_separation(self, geom_a, geom_b, report_limit):
        """Return a conservative separation, avoiding MuJoCo 3.2 OBB bugs."""
        box_type = int(mujoco.mjtGeom.mjGEOM_BOX)
        if (int(self.model.geom_type[geom_a]) != box_type or
                int(self.model.geom_type[geom_b]) != box_type):
            return float(mujoco.mj_geomDistance(
                self.model, self.data, geom_a, geom_b,
                report_limit, None))

        centers = (self.data.geom_xpos[geom_a], self.data.geom_xpos[geom_b])
        rotations = (
            self.data.geom_xmat[geom_a].reshape(3, 3),
            self.data.geom_xmat[geom_b].reshape(3, 3))
        half_sizes = (
            self.model.geom_size[geom_a], self.model.geom_size[geom_b])
        delta = centers[1] - centers[0]
        axes = [rotations[index][:, axis]
                for index in (0, 1) for axis in range(3)]
        for axis_a in range(3):
            for axis_b in range(3):
                cross = np.cross(
                    rotations[0][:, axis_a], rotations[1][:, axis_b])
                norm = np.linalg.norm(cross)
                if norm > 1e-10:
                    axes.append(cross / norm)
        maximum_axis_separation = -float("inf")
        for axis in axes:
            center_distance = abs(float(np.dot(delta, axis)))
            projected_radii = sum(
                float(np.sum(
                    half_sizes[index] *
                    np.abs(rotations[index].T @ axis)))
                for index in (0, 1))
            maximum_axis_separation = max(
                maximum_axis_separation,
                center_distance - projected_radii)
        return min(float(report_limit), maximum_axis_separation)

    def _collision_threshold(self, geom_a, geom_b, configured_margin):
        # For two oriented boxes the SAT value is a conservative axis
        # separation, not the exact Euclidean gap near diagonal edges.  Zero
        # still separates overlap from non-overlap exactly; applying the full
        # Euclidean margin to this lower bound would reject valid edge paths.
        box_type = int(mujoco.mjtGeom.mjGEOM_BOX)
        if (int(self.model.geom_type[geom_a]) == box_type and
                int(self.model.geom_type[geom_b]) == box_type):
            return 0.0
        return float(configured_margin)

    def _set_configuration(
            self, base_pose, box_poses, arm_joints, closure=None,
            box_shape_blend=1.0):
        self.data.qpos[:] = self.model.qpos0
        base_x, base_y, base_yaw = (float(value) for value in base_pose)
        address = self.base_qpos_address
        self.data.qpos[address:address + 3] = (base_x, base_y, 0.0)
        self.data.qpos[address + 3:address + 7] = (
            math.cos(0.5 * base_yaw), 0.0, 0.0,
            math.sin(0.5 * base_yaw))
        for name, (position, quaternion) in box_poses.items():
            address = self.box_qpos_addresses[name]
            self.data.qpos[address:address + 3] = position
            qx, qy, qz, qw = quaternion
            self.data.qpos[address + 3:address + 7] = (qw, qx, qy, qz)
        for side in ("l", "r"):
            self.data.qpos[self.arm_qpos_addresses[side]] = arm_joints[side]
            if closure is not None:
                for name, value in box_gripper_pose(closure, side).items():
                    address = self.hand_qpos_addresses[side][name]
                    normal_value = float(self.model.qpos0[address])
                    self.data.qpos[address] = (
                        (1.0 - box_shape_blend) * normal_value +
                        box_shape_blend * value)
        mujoco.mj_forward(self.model, self.data)

    def _measure_fixed_jaw_tcp_in_r7(self, side):
        """Return the midpoint of the fixed middle/little fingertip pads."""
        arms = {"l": np.zeros(7), "r": np.zeros(7)}
        boxes = {
            name: (
                self.model.body_pos[body_id].copy(),
                (0.0, 0.0, 0.0, 1.0),
            )
            for name, body_id in self.box_body_ids.items()
        }
        self._set_configuration(
            (0.0, 0.0, 0.0), boxes, arms, 0.0,
            box_shape_blend=1.0)
        body_id = self.r7_body_ids[side]
        rotation = self.data.xmat[body_id].reshape(3, 3)
        position = self.data.xpos[body_id]
        points = []
        for finger in ("middle", "little"):
            geom_id = self._required_id(
                mujoco.mjtObj.mjOBJ_GEOM,
                "{}_{}_fingertip_collision".format(side, finger))
            points.append(
                rotation.T @ (self.data.geom_xpos[geom_id] - position))
        return np.mean(points, axis=0)

    def _measure_body_origin_in_r7(self, side, body_id):
        self.data.qpos[:] = self.model.qpos0
        mujoco.mj_forward(self.model, self.data)
        r7_body_id = self.r7_body_ids[side]
        rotation = self.data.xmat[r7_body_id].reshape(3, 3)
        return rotation.T @ (
            self.data.xpos[body_id] - self.data.xpos[r7_body_id])

    @staticmethod
    def _forward_finger_orientation(side, forward_world, up_world):
        # In the r7 frame the hand extends along -Z.  Both hands therefore
        # share the same -Z direction when their fingers point forward.  The
        # opposite Y choices retain the mirrored left/right palm roll without
        # turning either hand toward the other side of the box.
        z_axis = -np.asarray(forward_world, dtype=float)
        z_axis /= np.linalg.norm(z_axis)
        up_axis = np.asarray(up_world, dtype=float)
        up_axis /= np.linalg.norm(up_axis)
        y_axis = up_axis if side == "l" else -up_axis
        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        return np.column_stack((x_axis, y_axis, z_axis))

    def _solve_arm(
            self, side, target_position, target_rotation, seed,
            base_pose, box_poses, point_in_r7, alternate_seeds=()):
        target_position = np.asarray(target_position, dtype=float)
        target_rotation_object = Rotation.from_matrix(target_rotation)
        seed = np.clip(
            np.asarray(seed, dtype=float),
            self.arm_lower[side], self.arm_upper[side])
        other_side = "r" if side == "l" else "l"
        fixed_other = np.zeros(7)

        def forward(joints):
            arms = {side: joints, other_side: fixed_other}
            self._set_configuration(
                base_pose, box_poses, arms, 0.0,
                box_shape_blend=1.0)
            body_id = self.r7_body_ids[side]
            rotation = self.data.xmat[body_id].reshape(3, 3).copy()
            r7_position = self.data.xpos[body_id].copy()
            point_position = r7_position + rotation @ point_in_r7
            return point_position, r7_position, rotation

        def residual(joints):
            point_position, _, rotation = forward(joints)
            orientation_error = (
                target_rotation_object.inv() *
                Rotation.from_matrix(rotation)).as_rotvec()
            return np.concatenate((
                float(self.config["position_weight"]) *
                (point_position - target_position),
                float(self.config["orientation_weight"]) *
                orientation_error,
                float(self.config["joint_regularization"]) *
                (joints - seed),
            ))

        candidates = []
        starts = (seed,) + tuple(
            np.clip(np.asarray(value, dtype=float),
                    self.arm_lower[side], self.arm_upper[side])
            for value in alternate_seeds)
        for start in starts:
            result = least_squares(
                residual, start,
                bounds=(self.arm_lower[side], self.arm_upper[side]),
                max_nfev=int(self.config["maximum_function_evaluations"]),
                xtol=1e-9, ftol=1e-9, gtol=1e-9)
            point_position, r7_position, rotation = forward(result.x)
            position_error = float(np.linalg.norm(
                point_position - target_position))
            orientation_error_rad = float(np.linalg.norm(
                (target_rotation_object.inv() *
                 Rotation.from_matrix(rotation)).as_rotvec()))
            candidates.append((
                position_error + orientation_error_rad,
                result.x.copy(), point_position, r7_position, rotation,
                position_error, math.degrees(orientation_error_rad),
            ))
        (_, joints, _, r7_position, rotation,
         position_error, orientation_error) = min(
            candidates, key=lambda value: value[0])
        if position_error > float(self.config["maximum_position_error_m"]):
            raise RuntimeError(
                "{} arm IK candidate rejected: position_error={:.4f} m "
                "(limit {:.4f} m), orientation_error={:.1f} deg".format(
                    side, position_error,
                    float(self.config["maximum_position_error_m"]),
                    orientation_error))
        return ArmIKResult(
            joints=tuple(float(value) for value in joints),
            r7_position=tuple(float(value) for value in r7_position),
            r7_rotation=tuple(float(value) for value in rotation.ravel()),
            position_error_m=position_error,
            orientation_error_deg=orientation_error,
        )

    def solve_palm_down_ready(
            self, base_pose, arm_seed, outward_angle_deg):
        """Keep both hand origins in place while matching grasp orientation."""
        arm_seed = np.asarray(arm_seed, dtype=float)
        if arm_seed.shape != (14,) or not np.all(np.isfinite(arm_seed)):
            raise ValueError("arm_seed must contain 14 finite joints")
        arms = {"l": arm_seed[:7], "r": arm_seed[7:]}
        self._set_configuration(base_pose, {}, arms, closure=0.0)
        target_positions = {
            side: self.data.xpos[self.hand_body_ids[side]].copy()
            for side in ("l", "r")
        }
        base_yaw = float(base_pose[2])
        forward = np.array((math.cos(base_yaw), math.sin(base_yaw), 0.0))
        up = np.array((0.0, 0.0, 1.0))
        solved = []
        for side, sign in (("l", 1.0), ("r", -1.0)):
            orientation = self._forward_finger_orientation(
                side, forward, up)
            orientation = (
                Rotation.from_rotvec(
                    sign * math.radians(float(outward_angle_deg)) * up
                ).as_matrix() @ orientation)
            result = self._solve_arm(
                side, target_positions[side], orientation,
                arms[side], base_pose, {},
                self._hand_base_origin_in_r7[side])
            solved.extend(result.joints)
        return tuple(float(value) for value in solved)

    def _solve_bimanual_lift_target(
            self, contact_joints, base_pose, box_poses, selected_box,
            target_hand_poses):
        """Solve the confirmed terminal poses of both hand_base bodies."""
        contact_joints = {
            side: np.asarray(contact_joints[side], dtype=float)
            for side in ("l", "r")
        }
        reference = np.concatenate((
            contact_joints["l"], contact_joints["r"]))
        bounds = list(zip(
            np.concatenate((self.arm_lower["l"], self.arm_lower["r"])),
            np.concatenate((self.arm_upper["l"], self.arm_upper["r"]))))

        box_position, box_quaternion = box_poses[selected_box]
        box_position = np.asarray(box_position, dtype=float)
        box_rotation = Rotation.from_quat(box_quaternion).as_matrix()
        target_positions = {}
        target_rotations = {}
        for side in ("l", "r"):
            try:
                target = target_hand_poses[side]
                position_in_box = np.asarray(
                    target["position_m"], dtype=float)
                orientation_xyzw = np.asarray(
                    target["orientation_xyzw"], dtype=float)
            except (KeyError, TypeError) as error:
                raise ValueError(
                    "lift target {} must define position_m and "
                    "orientation_xyzw".format(side)) from error
            if (position_in_box.shape != (3,) or
                    not np.all(np.isfinite(position_in_box))):
                raise ValueError(
                    "lift target {} position_m must contain 3 finite "
                    "values".format(side))
            if (orientation_xyzw.shape != (4,) or
                    not np.all(np.isfinite(orientation_xyzw)) or
                    np.linalg.norm(orientation_xyzw) <= 1e-12):
                raise ValueError(
                    "lift target {} orientation_xyzw must contain a "
                    "finite non-zero quaternion".format(side))
            orientation_in_box = Rotation.from_quat(
                orientation_xyzw).as_matrix()
            target_positions[side] = (
                box_position + box_rotation @ position_in_box)
            target_rotations[side] = Rotation.from_matrix(
                box_rotation @ orientation_in_box)

        def pose(joints):
            arms = {"l": joints[:7], "r": joints[7:]}
            self._set_configuration(
                base_pose, box_poses, arms,
                closure=0.0)
            positions = {
                side: self.data.xpos[self.hand_body_ids[side]].copy()
                for side in ("l", "r")
            }
            rotations = {
                side: (
                    Rotation.from_matrix(
                        self.data.xmat[self.hand_body_ids[side]].reshape(
                            3, 3).copy()))
                for side in ("l", "r")
            }
            return positions, rotations

        def terminal_pose_equality(joints):
            positions, rotations = pose(joints)
            residuals = []
            for side in ("l", "r"):
                residuals.extend(positions[side] - target_positions[side])
                residuals.extend((
                    target_rotations[side].inv() *
                    rotations[side]).as_rotvec())
            return np.asarray(residuals, dtype=float)

        def objective(joints):
            delta = joints - reference
            return 0.5 * float(delta @ delta)

        result = minimize(
            objective, reference, method="SLSQP", bounds=bounds,
            constraints=({"type": "eq", "fun": terminal_pose_equality},),
            options={"ftol": 1e-12, "maxiter": 400, "disp": False})
        residual = terminal_pose_equality(result.x).reshape(2, 6)
        maximum_position_error = float(np.max(
            np.linalg.norm(residual[:, :3], axis=1)))
        maximum_orientation_error = float(np.max(
            np.linalg.norm(residual[:, 3:], axis=1)))
        if (not result.success or maximum_position_error > 1e-5 or
                maximum_orientation_error > 1e-4):
            raise RuntimeError(
                "bimanual terminal-pose IK failed: success={}, "
                "position_error={:.9f} m, orientation_error={:.6f} deg, "
                "message={}".format(
                    result.success, maximum_position_error,
                    math.degrees(maximum_orientation_error), result.message))
        return np.asarray(result.x, dtype=float)

    def solve_lift_path(
            self, contact_joints, base_pose, box_poses, selected_box,
            grasp_config):
        """Solve one terminal-pose IK and interpolate to it in joint space."""
        contact_joints = np.asarray(contact_joints, dtype=float)
        if (contact_joints.shape != (14,) or
                not np.all(np.isfinite(contact_joints))):
            raise ValueError(
                "contact_joints must contain 14 finite joints")
        waypoint_count = int(
            grasp_config.get("lift_waypoint_count", 9))
        if waypoint_count < 2:
            raise ValueError("lift_waypoint_count must be at least 2")
        terminal_joints = self._solve_bimanual_lift_target(
            {
                "l": contact_joints[:7],
                "r": contact_joints[7:],
            },
            base_pose, box_poses, selected_box,
            grasp_config["lift_target_hand_poses_in_grasp_box"])
        return tuple(
            tuple(float(value) for value in (
                contact_joints +
                float(index) / float(waypoint_count - 1) *
                (terminal_joints - contact_joints)))
            for index in range(waypoint_count))

    def _minimum_path_clearance(
            self, base_pose, box_poses, start, end, closure_start,
            closure_end, selected_box, allow_selected_hand_contact,
            ignore_selected_box=False, shape_blend_start=1.0,
            shape_blend_end=1.0):
        samples = int(self.config["path_samples_per_segment"])
        report_limit = float(self.config["collision_report_limit_m"])
        margin = float(self.config["collision_margin_m"])
        minimum = report_limit
        obstacles = set(self.static_obstacle_geom_ids)
        for box_name in BOX_NAMES:
            if not (ignore_selected_box and box_name == selected_box):
                obstacles.update(self.box_geom_ids[box_name])
        allowed = set()
        if allow_selected_hand_contact:
            for side in ("l", "r"):
                for robot_geom in self.hand_geom_ids[side]:
                    allowed.add((
                        robot_geom,
                        self.assigned_wall_geom_ids[selected_box][side],
                    ))

        for index in range(samples + 1):
            ratio = index / float(samples)
            arms = {
                side: ((1.0 - ratio) * np.asarray(start[side]) +
                       ratio * np.asarray(end[side]))
                for side in ("l", "r")
            }
            closure = (
                (1.0 - ratio) * closure_start + ratio * closure_end)
            shape_blend = (
                (1.0 - ratio) * shape_blend_start +
                ratio * shape_blend_end)
            self._set_configuration(
                base_pose, box_poses, arms, closure,
                box_shape_blend=shape_blend)
            for robot_geom in self.robot_geom_ids:
                for obstacle_geom in obstacles:
                    if (robot_geom, obstacle_geom) in allowed:
                        continue
                    distance = self._geom_separation(
                        robot_geom, obstacle_geom, report_limit)
                    minimum = min(minimum, distance)
                    if distance < self._collision_threshold(
                            robot_geom, obstacle_geom, margin):
                        raise RuntimeError(
                            "planned path collision: {} vs {}, "
                            "distance={:.4f} m".format(
                                mujoco.mj_id2name(
                                    self.model, mujoco.mjtObj.mjOBJ_GEOM,
                                    robot_geom) or "geom_{}".format(robot_geom),
                                mujoco.mj_id2name(
                                    self.model, mujoco.mjtObj.mjOBJ_GEOM,
                                    obstacle_geom) or "geom_{}".format(obstacle_geom),
                                distance))
        return minimum

    def plan(
            self, selected_box, base_pose, box_poses, arm_seed, grasp_config,
            execution_arm_start=None):
        if selected_box not in BOX_NAMES:
            raise ValueError("unsupported selected box: {}".format(selected_box))
        arm_seed = np.asarray(arm_seed, dtype=float)
        if arm_seed.shape != (14,) or not np.all(np.isfinite(arm_seed)):
            raise ValueError("arm_seed must contain 14 finite joints")
        seeds = {"l": arm_seed[:7], "r": arm_seed[7:]}
        if execution_arm_start is None:
            execution_arm_start = arm_seed
        execution_arm_start = np.asarray(execution_arm_start, dtype=float)
        if (execution_arm_start.shape != (14,) or
                not np.all(np.isfinite(execution_arm_start))):
            raise ValueError(
                "execution_arm_start must contain 14 finite joints")
        execution_start = {
            "l": execution_arm_start[:7],
            "r": execution_arm_start[7:],
        }
        box_position, box_quaternion = box_poses[selected_box]
        box_rotation = Rotation.from_quat(box_quaternion).as_matrix()
        box_forward = box_rotation[:, 0]
        box_up = box_rotation[:, 2]
        contact_z = float(grasp_config["contact_z_local_m"])
        tangent = float(grasp_config["tangent_offset_m"])
        fixed_jaw_y = float(grasp_config["fixed_jaw_y_local_m"])
        outward_angle = math.radians(
            float(grasp_config["outward_angle_deg"]))
        above_height = float(grasp_config["above_height_m"])
        above = {}
        contact = {}
        for side, sign in (("l", 1.0), ("r", -1.0)):
            grasp_tcp = self._fixed_jaw_tcp_in_r7[side]
            local_target = np.array((
                tangent,
                sign * fixed_jaw_y,
                contact_z,
            ))
            contact_target = (
                np.asarray(box_position, dtype=float) +
                box_rotation @ local_target)
            orientation = self._forward_finger_orientation(
                side, box_forward, box_up)
            orientation = (
                Rotation.from_rotvec(
                    sign * outward_angle * box_up).as_matrix() @
                orientation)
            above[side] = self._solve_arm(
                side, contact_target + above_height * box_up,
                orientation, seeds[side], base_pose, box_poses,
                grasp_tcp)
            contact[side] = self._solve_arm(
                side, contact_target, orientation, above[side].joints,
                base_pose, box_poses, grasp_tcp,
                alternate_seeds=(seeds[side],))

        above_group = {side: above[side].joints for side in ("l", "r")}
        contact_group = {side: contact[side].joints for side in ("l", "r")}
        lift_waypoints = self.solve_lift_path(
            tuple(contact_group["l"] + contact_group["r"]),
            base_pose, box_poses, selected_box, grasp_config)
        return PickPlan(
            box_name=selected_box,
            above_joints=tuple(above_group["l"] + above_group["r"]),
            contact_joints=tuple(contact_group["l"] + contact_group["r"]),
            lift_waypoints=lift_waypoints,
        )
