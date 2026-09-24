#!/usr/bin/env python3
"""Collision auditing primitives for the Task 3 right-arm initialization."""

import math

import mujoco
import numpy as np


ARM_JOINT_NAMES = tuple(
    "zarm_{}{}_joint".format(side, index)
    for side in ("l", "r") for index in range(1, 8)
)
OBSTACLE_BODY_NAMES = (
    "task3_source_table",
    "task3_destination_table",
    "task3_hollow_cylinder",
)


class Task3InitializationPlanner:
    """Audit base and right-arm initialization paths in the Task 3 scene."""

    def __init__(self, scene_path, collision_margin_m=0.015,
                 report_limit_m=0.05):
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
        self.base_qpos_address = int(
            self.model.jnt_qposadr[base_joint_id])
        self.arm_qpos_addresses = np.asarray([
            self.model.jnt_qposadr[
                self._required_id(mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in ARM_JOINT_NAMES
        ], dtype=int)

        robot_body_ids = self._descendant_body_ids(self.base_body_id)
        self.robot_geom_ids = self._collidable_geom_ids(robot_body_ids)
        obstacle_body_ids = set()
        for name in OBSTACLE_BODY_NAMES:
            obstacle_body_ids.update(self._descendant_body_ids(
                self._required_id(mujoco.mjtObj.mjOBJ_BODY, name)))
        self.obstacle_geom_ids = self._collidable_geom_ids(
            obstacle_body_ids)

    def _required_id(self, object_type, name):
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise RuntimeError("model object is missing: {}".format(name))
        return int(object_id)

    def _descendant_body_ids(self, root_body_id):
        body_ids = {int(root_body_id)}
        for body_id in range(1, self.model.nbody):
            if int(self.model.body_parentid[body_id]) in body_ids:
                body_ids.add(body_id)
        return body_ids

    def _collidable_geom_ids(self, body_ids):
        return tuple(
            geom_id for geom_id in range(self.model.ngeom)
            if (int(self.model.geom_bodyid[geom_id]) in body_ids and
                self.model.geom_contype[geom_id] != 0)
        )

    @staticmethod
    def _validate_arm_vector(values, label):
        vector = np.asarray(values, dtype=float)
        if vector.shape != (14,) or not np.all(np.isfinite(vector)):
            raise ValueError("{} must contain 14 finite joints".format(label))
        return vector

    def _set_configuration(self, base_pose, arm_joints):
        base_x, base_y, base_yaw = (float(value) for value in base_pose)
        self.data.qpos[:] = self.model.qpos0
        address = self.base_qpos_address
        self.data.qpos[address:address + 3] = (base_x, base_y, 0.0)
        self.data.qpos[address + 3:address + 7] = (
            math.cos(0.5 * base_yaw), 0.0, 0.0,
            math.sin(0.5 * base_yaw))
        self.data.qpos[self.arm_qpos_addresses] = arm_joints
        mujoco.mj_forward(self.model, self.data)

    def configuration_clearance(self, base_pose, arm_joints):
        arm_joints = self._validate_arm_vector(arm_joints, "arm_joints")
        self._set_configuration(base_pose, arm_joints)
        minimum = self.report_limit_m
        for robot_geom_id in self.robot_geom_ids:
            for obstacle_geom_id in self.obstacle_geom_ids:
                center_bound = (
                    np.linalg.norm(
                        self.data.geom_xpos[robot_geom_id] -
                        self.data.geom_xpos[obstacle_geom_id]) -
                    self.model.geom_rbound[robot_geom_id] -
                    self.model.geom_rbound[obstacle_geom_id]
                )
                if center_bound >= minimum:
                    continue
                minimum = min(minimum, float(mujoco.mj_geomDistance(
                    self.model, self.data, robot_geom_id, obstacle_geom_id,
                    minimum, None)))
        return minimum

    def arm_path_clearance(self, base_pose, start_arm, target_arm,
                           samples=80):
        if samples < 1:
            raise ValueError("samples must be positive")
        start = self._validate_arm_vector(start_arm, "start_arm")
        target = self._validate_arm_vector(target_arm, "target_arm")
        return min(
            self.configuration_clearance(
                base_pose, (1.0 - ratio) * start + ratio * target)
            for ratio in np.linspace(0.0, 1.0, samples + 1)
        )

    def base_path_clearance(self, start_pose, target_pose, arm_joints,
                            samples=80):
        if samples < 1:
            raise ValueError("samples must be positive")
        start = np.asarray(start_pose, dtype=float)
        target = np.asarray(target_pose, dtype=float)
        if start.shape != (3,) or target.shape != (3,):
            raise ValueError("base poses must contain x, y, and yaw")
        arm = self._validate_arm_vector(arm_joints, "arm_joints")
        return min(
            self.configuration_clearance(
                (1.0 - ratio) * start + ratio * target, arm)
            for ratio in np.linspace(0.0, 1.0, samples + 1)
        )

    @staticmethod
    def backward_pose(base_pose, retreat_m):
        base_x, base_y, base_yaw = (float(value) for value in base_pose)
        retreat_m = float(retreat_m)
        return (
            base_x - retreat_m * math.cos(base_yaw),
            base_y - retreat_m * math.sin(base_yaw),
            base_yaw,
        )


def right_ready_target(measured_arm, right_arm_ready):
    """Copy measured arms and replace only the seven right-arm joints."""
    target = np.asarray(measured_arm, dtype=float).copy()
    ready = np.asarray(right_arm_ready, dtype=float)
    if target.shape != (14,) or ready.shape != (7,):
        raise ValueError("expected 14 measured and 7 right ready joints")
    target[7:14] = ready
    return target

