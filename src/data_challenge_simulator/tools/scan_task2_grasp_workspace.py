#!/usr/bin/env python3
"""Offline base/dual-arm grasp-workspace scan for the Task 2 boxes.

The scan keeps the task policy separate from scene legality:

* both identical boxes are evaluated in their own body frames;
* both opposing X-face and Y-face grasp families are considered;
* the base pose is sampled before solving the two arm targets;
* the dedicated whole-box posture supplies the fingertip contact plane;
* all robot geoms are checked against the table and both boxes, even though
  most visual robot geoms are disabled in the runtime contact model.

This tool does not choose the final Task 2 grasp or modify the MJCF.  It emits
feasible candidates for a later trajectory/controller decision.
"""

import argparse
from collections import Counter
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PACKAGE_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.hand_postures import box_gripper_pose


DEFAULT_SCENE = (
    SRC_DIR / "data_challenge_simulator/models/biped_s400062/xml/task2.xml"
)
FINGERS = ("thumb", "index", "middle", "little")
READY = {
    "l": np.array((-0.9, 0.265, -1.0, -0.8, -0.58, 0.4, 0.35)),
    "r": np.array((-0.9, -0.265, 1.0, -0.8, 0.58, -0.4, 0.35)),
}
BOX_HALF_EXTENT = {"x": 0.32, "y": 0.22}
BOX_WALL_CENTER_Z = 0.0675


def parse_csv_floats(value):
    values = []
    for item in value.split(","):
        item = item.strip()
        if item:
            values.append(float(item))
    if not values:
        raise argparse.ArgumentTypeError("expected at least one number")
    return values


def parse_csv_choices(value, allowed):
    values = [item.strip() for item in value.split(",") if item.strip()]
    invalid = [item for item in values if item not in allowed]
    if not values or invalid:
        raise argparse.ArgumentTypeError(
            "expected comma-separated values from {}; invalid={}".format(
                sorted(allowed), invalid))
    return values


def object_name(model, object_type, object_id):
    name = mujoco.mj_id2name(model, object_type, object_id)
    return name if name is not None else "<unnamed:{}>".format(object_id)


@dataclass
class ArmSolution:
    joints: list
    position_error_m: float
    orientation_error_deg: float
    hand_roll_deg: float
    cost: float


@dataclass
class GraspCandidate:
    box: str
    grasp_mode: str
    face_axis: str
    assignment: str
    base_x: float
    base_y: float
    base_yaw_deg: float
    tangent_offset_m: float
    contact_z_local_m: float
    pregrasp_closure: float
    grasp_closure: float
    left_pregrasp: ArmSolution
    right_pregrasp: ArmSolution
    left_grasp: ArmSolution
    right_grasp: ArmSolution
    left_target_pad_contacts: int
    right_target_pad_contacts: int
    minimum_forbidden_clearance_m: float

    def score(self):
        solutions = (
            self.left_pregrasp,
            self.right_pregrasp,
            self.left_grasp,
            self.right_grasp,
        )
        position_term = sum(item.position_error_m for item in solutions)
        orientation_term = sum(
            item.orientation_error_deg for item in solutions) / 180.0
        posture_term = 0.02 * sum(
            np.linalg.norm(np.asarray(item.joints) - READY[side])
            for side, item in (
                ("l", self.left_grasp), ("r", self.right_grasp)))
        return float(position_term + orientation_term + posture_term)


class Scene2GraspWorkspace:
    def __init__(self, args):
        self.args = args
        self.scene_path = Path(args.scene).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)

        self.base_body_id = self.required_id(
            mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if self.model.body_jntnum[self.base_body_id] != 1:
            raise RuntimeError("base_link must have exactly one joint")
        self.base_joint_id = self.model.body_jntadr[self.base_body_id]
        if self.model.jnt_type[self.base_joint_id] != mujoco.mjtJoint.mjJNT_FREE:
            raise RuntimeError("base_link must use a free joint")
        self.base_qpos_address = self.model.jnt_qposadr[self.base_joint_id]

        self.arm_joint_ids = {}
        self.arm_qpos_addresses = {}
        self.arm_lower = {}
        self.arm_upper = {}
        self.r7_body_ids = {}
        self.hand_joint_ids = {}
        self.hand_qpos_addresses = {}
        self.pad_geom_ids = {}
        self.fingertip_geom_ids = {}
        self.hand_geom_ids = {}
        for side in ("l", "r"):
            joint_ids = np.array([
                self.required_id(
                    mujoco.mjtObj.mjOBJ_JOINT,
                    "zarm_{}{}_joint".format(side, index))
                for index in range(1, 8)
            ])
            self.arm_joint_ids[side] = joint_ids
            self.arm_qpos_addresses[side] = np.array([
                self.model.jnt_qposadr[joint_id]
                for joint_id in joint_ids
            ])
            self.arm_lower[side] = (
                self.model.jnt_range[joint_ids, 0].copy() + 1e-5)
            self.arm_upper[side] = (
                self.model.jnt_range[joint_ids, 1].copy() - 1e-5)
            self.r7_body_ids[side] = self.required_id(
                mujoco.mjtObj.mjOBJ_BODY,
                "zarm_{}7_link".format(side))
            hand_joint_ids = {}
            hand_qpos_addresses = {}
            for joint_name in box_gripper_pose(0.0, side):
                joint_id = self.required_id(
                    mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                hand_joint_ids[joint_name] = joint_id
                hand_qpos_addresses[joint_name] = (
                    self.model.jnt_qposadr[joint_id])
            self.hand_joint_ids[side] = hand_joint_ids
            self.hand_qpos_addresses[side] = hand_qpos_addresses
            self.pad_geom_ids[side] = {
                self.required_id(
                    mujoco.mjtObj.mjOBJ_GEOM,
                    "{}_{}_fingertip_collision".format(side, finger))
                for finger in FINGERS
            }
            fingertip_body_ids = {
                self.model.geom_bodyid[geom_id]
                for geom_id in self.pad_geom_ids[side]
            }
            self.fingertip_geom_ids[side] = {
                geom_id for geom_id in range(self.model.ngeom)
                if self.model.geom_bodyid[geom_id] in fingertip_body_ids
            }
            hand_body_id = self.required_id(
                mujoco.mjtObj.mjOBJ_BODY,
                "{}_hand_base".format(side))
            self.hand_geom_ids[side] = self.descendant_geom_ids(hand_body_id)

        self.box_body_ids = {
            box_name: self.required_id(mujoco.mjtObj.mjOBJ_BODY, box_name)
            for box_name in ("box_1", "box_2")
        }
        self.box_geom_ids = {
            box_name: self.descendant_geom_ids(body_id)
            for box_name, body_id in self.box_body_ids.items()
        }
        table_body_id = self.required_id(
            mujoco.mjtObj.mjOBJ_BODY, "work_table")
        self.table_geom_ids = self.descendant_geom_ids(table_body_id)
        self.robot_geom_ids = self.descendant_geom_ids(self.base_body_id)
        self.obstacle_geom_ids = set(self.table_geom_ids)
        for geom_ids in self.box_geom_ids.values():
            self.obstacle_geom_ids.update(geom_ids)

        self.grasp_tcp_cache = {}
        self.arm_solve_count = 0
        self.rejection_counts = Counter()
        self.rejection_examples = {}

    def required_id(self, object_type, name):
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise RuntimeError("model object is missing: {}".format(name))
        return object_id

    def descendant_geom_ids(self, root_body_id):
        body_ids = {root_body_id}
        changed = True
        while changed:
            changed = False
            for body_id in range(1, self.model.nbody):
                if (body_id not in body_ids and
                        self.model.body_parentid[body_id] in body_ids):
                    body_ids.add(body_id)
                    changed = True
        return {
            geom_id for geom_id in range(self.model.ngeom)
            if self.model.geom_bodyid[geom_id] in body_ids
        }

    def reset_configuration(self, base_pose, arm_joints=None, closure=None):
        self.data.qpos[:] = self.model.qpos0
        base_x, base_y, base_yaw = base_pose
        address = self.base_qpos_address
        self.data.qpos[address] = base_x
        self.data.qpos[address + 1] = base_y
        self.data.qpos[address + 3:address + 7] = (
            math.cos(0.5 * base_yaw), 0.0, 0.0,
            math.sin(0.5 * base_yaw))
        if arm_joints:
            for side, joints in arm_joints.items():
                self.data.qpos[self.arm_qpos_addresses[side]] = joints
        if closure is not None:
            for side in ("l", "r"):
                for name, value in box_gripper_pose(closure, side).items():
                    self.data.qpos[
                        self.hand_qpos_addresses[side][name]] = value
        mujoco.mj_forward(self.model, self.data)

    def centered_grasp_tcp_in_r7(self, side, closure):
        key = (side, float(closure))
        if key in self.grasp_tcp_cache:
            return self.grasp_tcp_cache[key].copy()
        self.reset_configuration((0.0, 0.0, 0.0), closure=closure)
        r7_body_id = self.r7_body_ids[side]
        r7_rotation = self.data.xmat[r7_body_id].reshape(3, 3)
        r7_position = self.data.xpos[r7_body_id]
        points = [
            r7_rotation.T @ (
                self.data.geom_xpos[geom_id] - r7_position)
            for geom_id in self.pad_geom_ids[side]
        ]
        centroid = np.mean(points, axis=0)
        centroid[0] = 0.0
        self.grasp_tcp_cache[key] = centroid
        return centroid.copy()

    def solve_arm(self, side, target_world, orientation_world, hand_roll_deg,
                  base_pose, closure, starts):
        point_in_r7 = self.centered_grasp_tcp_in_r7(side, closure)
        target_world = np.asarray(target_world, dtype=float)
        target_rotation = Rotation.from_matrix(orientation_world)
        lower = self.arm_lower[side]
        upper = self.arm_upper[side]
        qpos_addresses = self.arm_qpos_addresses[side]
        r7_body_id = self.r7_body_ids[side]

        def forward(joints):
            self.reset_configuration(
                base_pose, {side: joints}, closure=closure)
            rotation = self.data.xmat[r7_body_id].reshape(3, 3).copy()
            position = (
                self.data.xpos[r7_body_id].copy() +
                rotation @ point_in_r7)
            return position, rotation

        regularization_seed = np.clip(
            np.asarray(starts[0], dtype=float), lower, upper)

        def residual(joints):
            position, rotation = forward(joints)
            orientation_error = (
                target_rotation.inv() *
                Rotation.from_matrix(rotation)).as_rotvec()
            return np.concatenate((
                self.args.position_weight * (position - target_world),
                self.args.orientation_weight * orientation_error,
                self.args.joint_regularization * (
                    joints - regularization_seed),
            ))

        candidates = []
        for start in starts:
            result = least_squares(
                residual,
                np.clip(np.asarray(start, dtype=float), lower, upper),
                bounds=(lower, upper),
                max_nfev=self.args.max_nfev,
                xtol=1e-9,
                ftol=1e-9,
                gtol=1e-9,
            )
            position, rotation = forward(result.x)
            position_error = float(np.linalg.norm(position - target_world))
            orientation_error = float(np.linalg.norm(
                (target_rotation.inv() *
                 Rotation.from_matrix(rotation)).as_rotvec()))
            candidates.append((
                position_error + orientation_error,
                ArmSolution(
                    joints=result.x.tolist(),
                    position_error_m=position_error,
                    orientation_error_deg=math.degrees(orientation_error),
                    hand_roll_deg=float(hand_roll_deg),
                    cost=float(result.cost),
                ),
            ))
            self.arm_solve_count += 1
        return min(candidates, key=lambda item: item[0])[1]

    def solve_pregrasp_with_rolls(self, side, target_world, outward_world,
                                  base_orientation, base_pose, closure,
                                  roll_candidates):
        candidates = []
        for roll_deg in roll_candidates:
            roll_rotation = Rotation.from_rotvec(
                np.asarray(outward_world) * math.radians(roll_deg)).as_matrix()
            orientation = roll_rotation @ base_orientation
            solution = self.solve_arm(
                side, target_world, orientation, roll_deg, base_pose,
                closure, (READY[side],))
            score = (
                solution.position_error_m /
                max(self.args.max_position_error, 1e-9) +
                solution.orientation_error_deg /
                max(self.args.max_orientation_error_deg, 1e-9))
            candidates.append((score, orientation, solution))
        return min(candidates, key=lambda item: item[0])[1:]

    def box_pose(self, box_name):
        self.reset_configuration((0.0, 0.0, 0.0), closure=0.0)
        body_id = self.box_body_ids[box_name]
        return (
            self.data.xpos[body_id].copy(),
            self.data.xmat[body_id].reshape(3, 3).copy(),
        )

    @staticmethod
    def hand_orientation(side, outward_world, box_up_world):
        outward_world = np.asarray(outward_world, dtype=float)
        outward_world /= np.linalg.norm(outward_world)
        z_axis = np.asarray(box_up_world, dtype=float)
        z_axis /= np.linalg.norm(z_axis)
        # Left fingertips extend along local -Y; right fingertips along +Y.
        y_axis = outward_world if side == "l" else -outward_world
        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        z_axis = np.cross(x_axis, y_axis)
        return np.column_stack((x_axis, y_axis, z_axis))

    @staticmethod
    def rim_orientation(side, forward_world, box_up_world):
        # The hand extends along r7 local -Z.  Keep that direction parallel to
        # the box forward axis for both hands; only the palm roll is mirrored.
        z_axis = -np.asarray(forward_world, dtype=float)
        z_axis /= np.linalg.norm(z_axis)
        up_axis = np.asarray(box_up_world, dtype=float)
        up_axis /= np.linalg.norm(up_axis)
        y_axis = up_axis if side == "l" else -up_axis
        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        return np.column_stack((x_axis, y_axis, z_axis))

    @staticmethod
    def face_wall_name(box_name, axis, sign):
        suffix = {
            ("x", -1): "front_wall",
            ("x", 1): "back_wall",
            ("y", -1): "right_wall",
            ("y", 1): "left_wall",
        }[(axis, sign)]
        return "{}_{}".format(box_name, suffix)

    def face_target(self, box_name, axis, sign, tangent_offset, local_z,
                    outward_offset):
        box_position, box_rotation = self.box_pose(box_name)
        local_position = np.array((0.0, 0.0, local_z))
        normal_local = np.zeros(3)
        axis_index = 0 if axis == "x" else 1
        tangent_index = 1 - axis_index
        local_position[axis_index] = sign * BOX_HALF_EXTENT[axis]
        local_position[tangent_index] = tangent_offset
        normal_local[axis_index] = sign
        outward_world = box_rotation @ normal_local
        surface_world = box_position + box_rotation @ local_position
        target_world = surface_world + outward_offset * outward_world
        orientation_world = self.hand_orientation(
            "l", outward_world, box_rotation[:, 2])
        return target_world, outward_world, orientation_world

    def target_for_side(self, grasp_mode, side, box_name, axis, sign,
                        tangent_offset, local_z, approach_offset):
        if grasp_mode == "surface":
            target, outward, _ = self.face_target(
                box_name, axis, sign, tangent_offset, local_z,
                approach_offset)
            _, box_rotation = self.box_pose(box_name)
            orientation = self.hand_orientation(
                side, outward, box_rotation[:, 2])
            return target, outward, orientation

        box_position, box_rotation = self.box_pose(box_name)
        axis_index = 0 if axis == "x" else 1
        tangent_index = 1 - axis_index
        local_position = np.array((0.0, 0.0, local_z))
        local_position[axis_index] = sign * (
            BOX_HALF_EXTENT[axis] - 0.01)
        local_position[tangent_index] = tangent_offset
        normal_local = np.zeros(3)
        normal_local[axis_index] = sign
        outward = box_rotation @ normal_local
        box_up = box_rotation[:, 2]
        target = (
            box_position + box_rotation @ local_position +
            approach_offset * box_up)
        orientation = self.rim_orientation(
            side, box_rotation[:, 0], box_up)
        return target, outward, orientation

    def geom_distance(self, geom_a, geom_b, distance_limit):
        return float(mujoco.mj_geomDistance(
            self.model, self.data, geom_a, geom_b,
            distance_limit, None))

    def collision_report(self, allowed_pairs, distance_limit):
        collisions = []
        minimum = distance_limit
        for robot_geom in self.robot_geom_ids:
            for obstacle_geom in self.obstacle_geom_ids:
                pair = (robot_geom, obstacle_geom)
                distance = self.geom_distance(
                    robot_geom, obstacle_geom, distance_limit)
                if pair in allowed_pairs:
                    continue
                if distance < minimum:
                    minimum = distance
                if distance < self.args.collision_margin:
                    collisions.append((
                        object_name(
                            self.model, mujoco.mjtObj.mjOBJ_GEOM,
                            robot_geom),
                        object_name(
                            self.model, mujoco.mjtObj.mjOBJ_GEOM,
                            obstacle_geom),
                        distance,
                    ))
        return collisions, minimum

    def target_contact_count(self, side, wall_geom_id):
        return sum(
            self.geom_distance(
                pad_geom_id, wall_geom_id,
                self.args.clearance_report_limit) <=
            self.args.contact_tolerance
            for pad_geom_id in self.pad_geom_ids[side]
        )

    def validate_path(self, grasp_mode, base_pose, pregrasp, grasp,
                      wall_geom_ids):
        allowed_pairs = {
            (pad_geom_id, wall_geom_ids[side])
            for side in ("l", "r")
            # Each distal link has an unnamed visual mesh overlapping its
            # explicit fingertip collision box.  Allow both representations
            # against the assigned wall, but count contact using only the
            # named fingertip collision boxes below.
            for pad_geom_id in self.fingertip_geom_ids[side]
        }
        if grasp_mode == "rim":
            allowed_pairs = {
                (hand_geom_id, wall_geom_ids[side])
                for side in ("l", "r")
                for hand_geom_id in self.hand_geom_ids[side]
            }
        minimum_clearance = self.args.clearance_report_limit
        for step in range(self.args.path_steps + 1):
            ratio = step / float(self.args.path_steps)
            arm_joints = {
                side: (
                    (1.0 - ratio) * np.asarray(pregrasp[side].joints) +
                    ratio * np.asarray(grasp[side].joints))
                for side in ("l", "r")
            }
            closure = (
                (1.0 - ratio) * self.args.pregrasp_closure +
                ratio * self.args.grasp_closure)
            self.reset_configuration(base_pose, arm_joints, closure)
            step_allowed = allowed_pairs if step else set()
            collisions, clearance = self.collision_report(
                step_allowed, self.args.clearance_report_limit)
            minimum_clearance = min(minimum_clearance, clearance)
            if collisions:
                return False, minimum_clearance, collisions[:5], {"l": 0, "r": 0}

        contacts = {
            side: self.target_contact_count(side, wall_geom_ids[side])
            for side in ("l", "r")
        }
        valid_contacts = all(
            count >= self.args.min_target_pad_contacts
            for count in contacts.values())
        return valid_contacts, minimum_clearance, [], contacts

    def scan_candidate(self, grasp_mode, box_name, axis, assignment_signs,
                       base_pose, tangent_offset, local_z):
        assignment_name = "l{:+d}_r{:+d}".format(
            assignment_signs["l"], assignment_signs["r"])
        wall_geom_ids = {
            side: self.required_id(
                mujoco.mjtObj.mjOBJ_GEOM,
                self.face_wall_name(box_name, axis, sign))
            for side, sign in assignment_signs.items()
        }

        pregrasp = {}
        grasp = {}
        for side in ("l", "r"):
            sign = assignment_signs[side]
            pre_target, outward, base_orientation = self.target_for_side(
                grasp_mode, side, box_name, axis, sign, tangent_offset, local_z,
                self.args.approach_clearance)
            orientation, pregrasp[side] = self.solve_pregrasp_with_rolls(
                side, pre_target, outward, base_orientation, base_pose,
                self.args.pregrasp_closure,
                (self.args.rim_rolls_deg if grasp_mode == "rim"
                 else self.args.hand_rolls_deg))
            if (pregrasp[side].position_error_m >
                    self.args.max_position_error or
                    pregrasp[side].orientation_error_deg >
                    self.args.max_orientation_error_deg):
                self.record_rejection(
                    "{}_pregrasp_ik".format(side),
                    "position={:.4f}m orientation={:.1f}deg".format(
                        pregrasp[side].position_error_m,
                        pregrasp[side].orientation_error_deg))
                return None

            final_offset = (
                -self.args.contact_depth if grasp_mode == "surface" else 0.0)
            grasp_target, _, _ = self.target_for_side(
                grasp_mode, side, box_name, axis, sign, tangent_offset,
                local_z, final_offset)
            grasp[side] = self.solve_arm(
                side, grasp_target, orientation,
                pregrasp[side].hand_roll_deg, base_pose,
                self.args.grasp_closure,
                (np.asarray(pregrasp[side].joints), READY[side]))
            if (grasp[side].position_error_m >
                    self.args.max_position_error or
                    grasp[side].orientation_error_deg >
                    self.args.max_orientation_error_deg):
                self.record_rejection(
                    "{}_grasp_ik".format(side),
                    "position={:.4f}m orientation={:.1f}deg".format(
                        grasp[side].position_error_m,
                        grasp[side].orientation_error_deg))
                return None

        valid, minimum_clearance, collisions, contacts = self.validate_path(
            grasp_mode, base_pose, pregrasp, grasp, wall_geom_ids)
        if not valid:
            if collisions:
                first = collisions[0]
                self.record_rejection(
                    "path_collision",
                    "{} vs {} distance={:.4f}m".format(*first))
            else:
                self.record_rejection(
                    "target_contacts",
                    "left={} right={} required={}".format(
                        contacts["l"], contacts["r"],
                        self.args.min_target_pad_contacts))
            return None
        return GraspCandidate(
            box=box_name,
            grasp_mode=grasp_mode,
            face_axis=axis,
            assignment=assignment_name,
            base_x=float(base_pose[0]),
            base_y=float(base_pose[1]),
            base_yaw_deg=math.degrees(base_pose[2]),
            tangent_offset_m=float(tangent_offset),
            contact_z_local_m=float(local_z),
            pregrasp_closure=self.args.pregrasp_closure,
            grasp_closure=self.args.grasp_closure,
            left_pregrasp=pregrasp["l"],
            right_pregrasp=pregrasp["r"],
            left_grasp=grasp["l"],
            right_grasp=grasp["r"],
            left_target_pad_contacts=contacts["l"],
            right_target_pad_contacts=contacts["r"],
            minimum_forbidden_clearance_m=float(minimum_clearance),
        )

    def record_rejection(self, reason, detail):
        self.rejection_counts[reason] += 1
        self.rejection_examples.setdefault(reason, detail)

    def scan(self):
        candidates = []
        attempted = 0
        for box_name in self.args.boxes:
            box_position, _ = self.box_pose(box_name)
            for base_x_offset in self.args.base_x_offsets:
                for base_y_offset in self.args.base_y_offsets:
                    for yaw_deg in self.args.base_yaws_deg:
                        base_pose = (
                            float(box_position[0] + base_x_offset),
                            float(box_position[1] + base_y_offset),
                            math.radians(yaw_deg),
                        )
                        for grasp_mode in self.args.grasp_modes:
                            for axis in self.args.face_axes:
                                for left_sign in (-1, 1):
                                    assignment = {
                                        "l": left_sign,
                                        "r": -left_sign,
                                    }
                                    for tangent_offset in self.args.tangent_offsets:
                                        for local_z in self.args.contact_z_local:
                                            attempted += 1
                                            if (self.args.max_candidates and
                                                    attempted > self.args.max_candidates):
                                                return attempted - 1, candidates
                                            candidate = self.scan_candidate(
                                                grasp_mode, box_name, axis,
                                                assignment, base_pose,
                                                tangent_offset, local_z)
                                            if candidate is not None:
                                                candidates.append(candidate)
                                            if attempted % self.args.progress_every == 0:
                                                print(
                                                    "progress attempted={} feasible={} arm_solves={}".format(
                                                        attempted, len(candidates),
                                                        self.arm_solve_count),
                                                    file=sys.stderr)
        return attempted, candidates


def build_parser():
    parser = argparse.ArgumentParser(
        description="Scan Task 2 box grasp candidates offline")
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument(
        "--boxes", type=lambda value: parse_csv_choices(
            value, {"box_1", "box_2"}),
        default=["box_1", "box_2"])
    parser.add_argument(
        "--face-axes", type=lambda value: parse_csv_choices(
            value, {"x", "y"}),
        default=["x", "y"])
    parser.add_argument(
        "--grasp-modes", type=lambda value: parse_csv_choices(
            value, {"surface", "rim"}),
        default=["surface", "rim"])
    parser.add_argument(
        "--base-x-offsets", type=parse_csv_floats,
        default=[-0.85, -0.75, -0.65],
        help="Base X minus box X, metres")
    parser.add_argument(
        "--base-y-offsets", type=parse_csv_floats,
        default=[-0.15, 0.0, 0.15],
        help="Base Y minus box Y, metres")
    parser.add_argument(
        "--base-yaws-deg", type=parse_csv_floats, default=[0.0])
    parser.add_argument(
        "--tangent-offsets", type=parse_csv_floats, default=[0.0])
    parser.add_argument(
        "--contact-z-local", type=parse_csv_floats,
        default=[0.10],
        help="Contact height in the box body frame")
    parser.add_argument(
        "--hand-rolls-deg", type=parse_csv_floats,
        default=[-60.0, 0.0, 60.0],
        help="Surface-mode wrist rolls about each box-face normal")
    parser.add_argument(
        "--rim-rolls-deg", type=parse_csv_floats, default=[0.0],
        help="Rim-pinch wrist rolls about each wall normal")
    parser.add_argument("--pregrasp-closure", type=float, default=0.0)
    parser.add_argument("--grasp-closure", type=float, default=1.0)
    parser.add_argument("--approach-clearance", type=float, default=0.10)
    parser.add_argument("--contact-depth", type=float, default=0.001)
    parser.add_argument("--path-steps", type=int, default=4)
    parser.add_argument(
        "--min-target-pad-contacts", type=int, default=1,
        help="Required named fingertip contacts per hand")
    parser.add_argument("--contact-tolerance", type=float, default=0.002)
    parser.add_argument("--collision-margin", type=float, default=0.001)
    parser.add_argument("--clearance-report-limit", type=float, default=0.10)
    parser.add_argument("--max-position-error", type=float, default=0.025)
    parser.add_argument("--max-orientation-error-deg", type=float, default=3.0)
    parser.add_argument("--position-weight", type=float, default=100.0)
    parser.add_argument("--orientation-weight", type=float, default=20.0)
    parser.add_argument("--joint-regularization", type=float, default=0.03)
    parser.add_argument("--max-nfev", type=int, default=350)
    parser.add_argument(
        "--max-candidates", type=int, default=0,
        help="Stop after this many attempted candidates; 0 scans all")
    parser.add_argument("--progress-every", type=int, default=10)
    parser.add_argument("--top", type=int, default=10)
    parser.add_argument("--output", type=Path)
    return parser


def validate_args(parser, args):
    for name in ("pregrasp_closure", "grasp_closure"):
        value = getattr(args, name)
        if not 0.0 <= value <= 1.0:
            parser.error("--{} must be in [0, 1]".format(
                name.replace("_", "-")))
    if args.grasp_closure < args.pregrasp_closure:
        parser.error("--grasp-closure must be >= --pregrasp-closure")
    if args.path_steps < 1:
        parser.error("--path-steps must be >= 1")
    if not 0 <= args.min_target_pad_contacts <= len(FINGERS):
        parser.error("--min-target-pad-contacts must be in [0, 4]")
    if args.progress_every < 1:
        parser.error("--progress-every must be >= 1")


def main():
    parser = build_parser()
    args = parser.parse_args()
    validate_args(parser, args)

    workspace = Scene2GraspWorkspace(args)
    attempted, candidates = workspace.scan()
    candidates.sort(key=lambda item: item.score())

    print("scene={}".format(workspace.scene_path))
    print("attempted={} feasible={} arm_solves={}".format(
        attempted, len(candidates), workspace.arm_solve_count))
    for reason, count in workspace.rejection_counts.most_common():
        print("rejected {}={} example={}".format(
            reason, count, workspace.rejection_examples[reason]))
    for index, candidate in enumerate(candidates[:args.top], 1):
        print(
            "#{:02d} box={} mode={} axis={} assignment={} "
            "base=({:.3f},{:.3f},{:.1f}deg) tangent={:.3f} z={:.4f} "
            "roll=({:.1f},{:.1f}) contacts=({},{}) score={:.5f}".format(
                index, candidate.box, candidate.grasp_mode,
                candidate.face_axis,
                candidate.assignment, candidate.base_x, candidate.base_y,
                candidate.base_yaw_deg, candidate.tangent_offset_m,
                candidate.contact_z_local_m,
                candidate.left_grasp.hand_roll_deg,
                candidate.right_grasp.hand_roll_deg,
                candidate.left_target_pad_contacts,
                candidate.right_target_pad_contacts,
                candidate.score()))

    if args.output:
        payload = {
            "scene": str(workspace.scene_path),
            "attempted": attempted,
            "feasible": len(candidates),
            "parameters": {
                key: value for key, value in vars(args).items()
                if key != "output"
            },
            "candidates": [
                dict(asdict(candidate), score=candidate.score())
                for candidate in candidates
            ],
        }
        payload["parameters"]["scene"] = str(args.scene)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
        print("output={}".format(args.output.resolve()))

    return 0 if candidates else 2


if __name__ == "__main__":
    sys.exit(main())
