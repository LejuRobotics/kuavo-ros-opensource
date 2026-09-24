#!/usr/bin/env python3
"""Scene-aware shoulder-only arm-lift auditing for Task 2."""

from dataclasses import dataclass
import math

import mujoco
import numpy as np

ARM_JOINT_NAMES = tuple(
    "zarm_{}{}_joint".format(side, index)
    for side in ("l", "r") for index in range(1, 8)
)
OBSTACLE_BODY_NAMES = (
    "work_table", "destination_conveyor", "box_1", "box_2")


@dataclass(frozen=True)
class InitializationDecision:
    retreat_m: float
    minimum_clearance_m: float

    @property
    def direct(self):
        return self.retreat_m == 0.0


class Task2InitializationPlanner:
    """Audit initialization paths against every Task 2 task object."""

    def __init__(self, scene_path, collision_margin_m=0.015,
                 report_limit_m=0.20):
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        self.collision_margin_m = float(collision_margin_m)
        self.report_limit_m = float(report_limit_m)
        if self.collision_margin_m < 0.0:
            raise ValueError("collision_margin_m must be non-negative")
        if self.report_limit_m <= self.collision_margin_m:
            raise ValueError(
                "report_limit_m must be greater than collision_margin_m")

        self.base_body_id = self._required_id(
            mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_joint_id = int(self.model.body_jntadr[self.base_body_id])
        if (self.model.body_jntnum[self.base_body_id] != 1 or
                self.model.jnt_type[base_joint_id] !=
                mujoco.mjtJoint.mjJNT_FREE):
            raise RuntimeError("base_link must own one free joint")
        self.base_qpos_address = int(self.model.jnt_qposadr[base_joint_id])

        self.arm_qpos_addresses = np.asarray([
            self.model.jnt_qposadr[
                self._required_id(mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in ARM_JOINT_NAMES
        ], dtype=int)
        self.robot_geom_ids = self._descendant_geom_ids(self.base_body_id)
        self.obstacle_geom_ids = set()
        self.object_qpos_addresses = {}
        for body_name in OBSTACLE_BODY_NAMES:
            body_id = self._required_id(mujoco.mjtObj.mjOBJ_BODY, body_name)
            self.obstacle_geom_ids.update(
                self._descendant_geom_ids(body_id))
            if body_name.startswith("box_"):
                joint_id = int(self.model.body_jntadr[body_id])
                if (self.model.body_jntnum[body_id] != 1 or
                        self.model.jnt_type[joint_id] !=
                        mujoco.mjtJoint.mjJNT_FREE):
                    raise RuntimeError(
                        "{} must own one free joint".format(body_name))
                self.object_qpos_addresses[body_name] = int(
                    self.model.jnt_qposadr[joint_id])

    def _required_id(self, object_type, name):
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise RuntimeError("model object is missing: {}".format(name))
        return int(object_id)

    def _descendant_geom_ids(self, root_body_id):
        body_ids = {int(root_body_id)}
        for body_id in range(1, self.model.nbody):
            parent = int(self.model.body_parentid[body_id])
            if parent in body_ids:
                body_ids.add(body_id)
        return {
            geom_id for geom_id in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom_id]) in body_ids
        }

    @staticmethod
    def _validate_arm_vector(values, label):
        vector = np.asarray(values, dtype=float)
        if vector.shape != (14,) or not np.all(np.isfinite(vector)):
            raise ValueError("{} must contain 14 finite joints".format(label))
        return vector

    def _set_configuration(self, base_pose, arm_joints, object_poses=None):
        base_x, base_y, base_yaw = (float(value) for value in base_pose)
        self.data.qpos[:] = self.model.qpos0
        address = self.base_qpos_address
        self.data.qpos[address:address + 3] = (base_x, base_y, 0.0)
        self.data.qpos[address + 3:address + 7] = (
            math.cos(0.5 * base_yaw), 0.0, 0.0,
            math.sin(0.5 * base_yaw))
        for name, pose in (object_poses or {}).items():
            if name not in self.object_qpos_addresses:
                raise ValueError("unsupported movable object: {}".format(name))
            position, quaternion_xyzw = pose
            if len(position) != 3 or len(quaternion_xyzw) != 4:
                raise ValueError("invalid pose for {}".format(name))
            object_address = self.object_qpos_addresses[name]
            self.data.qpos[object_address:object_address + 3] = position
            qx, qy, qz, qw = quaternion_xyzw
            self.data.qpos[object_address + 3:object_address + 7] = (
                qw, qx, qy, qz)
        self.data.qpos[self.arm_qpos_addresses] = arm_joints
        mujoco.mj_forward(self.model, self.data)

    def configuration_clearance(
            self, base_pose, arm_joints, object_poses=None):
        arm_joints = self._validate_arm_vector(arm_joints, "arm_joints")
        self._set_configuration(base_pose, arm_joints, object_poses)
        minimum = self.report_limit_m
        for robot_geom_id in self.robot_geom_ids:
            for obstacle_geom_id in self.obstacle_geom_ids:
                distance = float(mujoco.mj_geomDistance(
                    self.model, self.data, robot_geom_id, obstacle_geom_id,
                    self.report_limit_m, None))
                minimum = min(minimum, distance)
        return minimum

    def path_clearance(self, base_pose, measured_arm, waypoints,
                       samples_per_segment=40, object_poses=None):
        if samples_per_segment < 1:
            raise ValueError("samples_per_segment must be positive")
        start = self._validate_arm_vector(measured_arm, "measured_arm")
        minimum = self.report_limit_m
        for waypoint_index, waypoint in enumerate(waypoints):
            target = self._validate_arm_vector(
                waypoint, "waypoint[{}]".format(waypoint_index))
            for sample in range(samples_per_segment + 1):
                ratio = sample / float(samples_per_segment)
                joints = (1.0 - ratio) * start + ratio * target
                minimum = min(
                    minimum,
                    self.configuration_clearance(
                        base_pose, joints, object_poses))
                if minimum < self.collision_margin_m:
                    return minimum
            start = target
        return minimum

    @staticmethod
    def backward_pose(base_pose, retreat_m):
        base_x, base_y, base_yaw = (float(value) for value in base_pose)
        return (
            base_x - retreat_m * math.cos(base_yaw),
            base_y - retreat_m * math.sin(base_yaw),
            base_yaw,
        )

    def decide(self, base_pose, measured_arm, waypoints,
               retreat_step_m=0.05, maximum_retreat_m=0.90,
               samples_per_segment=40, object_poses=None):
        if retreat_step_m <= 0.0 or maximum_retreat_m < 0.0:
            raise ValueError("invalid retreat search range")
        candidate_count = int(
            math.floor(maximum_retreat_m / retreat_step_m + 1e-9)) + 1
        tested = []
        for index in range(candidate_count):
            retreat_m = index * retreat_step_m
            candidate_pose = self.backward_pose(base_pose, retreat_m)
            clearance = self.path_clearance(
                candidate_pose, measured_arm, waypoints,
                samples_per_segment=samples_per_segment,
                object_poses=object_poses)
            tested.append((retreat_m, clearance))
            if clearance >= self.collision_margin_m:
                return InitializationDecision(retreat_m, clearance)
        details = ", ".join(
            "{:.2f}m:{:.3f}m".format(distance, clearance)
            for distance, clearance in tested)
        raise RuntimeError(
            "no collision-free initialization pose; tested {}".format(
                details))
