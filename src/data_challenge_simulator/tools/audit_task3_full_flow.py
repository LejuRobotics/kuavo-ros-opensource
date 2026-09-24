#!/usr/bin/env python3
"""Offline gate for the fixed-pose Task 3 full-flow entry."""

import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PACKAGE_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.hand_postures import internal_expansion_pose


SCENE = SRC_DIR / (
    "data_challenge_simulator/models/biped_s400062/xml/"
    "task3.xml")
CONFIG = PACKAGE_DIR / "config/task3_transfer.json"
ACTIVE_FINGERS = ("index", "middle", "little")


def required_id(model, object_type, name):
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise RuntimeError("missing model object: {}".format(name))
    return int(object_id)


def descendants(model, root_body):
    result = {int(root_body)}
    for body_id in range(1, model.nbody):
        if int(model.body_parentid[body_id]) in result:
            result.add(body_id)
    return result


class FullFlowAudit:
    def __init__(self, scene, config):
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.config = config
        self.source_height = float(config["source_table_height_m"])
        self.base_body = required_id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.base_qpos = int(self.model.jnt_qposadr[
            self.model.body_jntadr[self.base_body]])
        self.ring_body = required_id(
            self.model, mujoco.mjtObj.mjOBJ_BODY,
            "task3_hollow_cylinder")
        self.ring_qpos = int(self.model.jnt_qposadr[
            self.model.body_jntadr[self.ring_body]])
        self.right_arm_qpos = np.asarray([
            self.model.jnt_qposadr[required_id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT,
                "zarm_r{}_joint".format(index))]
            for index in range(1, 8)
        ], dtype=int)
        self.right_arm_joint_ids = np.asarray([
            required_id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT,
                "zarm_r{}_joint".format(index))
            for index in range(1, 8)
        ], dtype=int)
        self.hand_qpos = {
            name: int(self.model.jnt_qposadr[required_id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name)])
            for name in internal_expansion_pose(0.0, "r")
        }
        self.pad_ids = {
            finger: required_id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM,
                "r_{}_fingertip_collision".format(finger))
            for finger in ACTIVE_FINGERS
        }
        robot_bodies = descendants(self.model, self.base_body)
        self.robot_geoms = {
            geom for geom in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom]) in robot_bodies
        }
        obstacle_bodies = set()
        for name in (
                "task3_source_table", "task3_destination_table",
                "task3_hollow_cylinder"):
            obstacle_bodies.update(descendants(
                self.model, required_id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, name)))
        self.obstacle_geoms = {
            geom for geom in range(self.model.ngeom)
            if (int(self.model.geom_bodyid[geom]) in obstacle_bodies and
                self.model.geom_contype[geom])
        }
        self.ring_geoms = {
            geom for geom in self.obstacle_geoms
            if int(self.model.geom_bodyid[geom]) == self.ring_body
        }
        self.r7_body = required_id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "zarm_r7_link")

    def set_configuration(
            self, right_arm, expansion, base_y=0.0,
            ring_y=0.0, ring_z=None):
        if ring_z is None:
            ring_z = self.source_height
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[self.base_qpos + 1] = float(base_y)
        self.data.qpos[self.ring_qpos:self.ring_qpos + 3] = (
            0.62, float(ring_y), float(ring_z))
        self.data.qpos[self.right_arm_qpos] = np.asarray(
            right_arm, dtype=float)
        for name, value in internal_expansion_pose(expansion, "r").items():
            self.data.qpos[self.hand_qpos[name]] = value
        mujoco.mj_forward(self.model, self.data)

    def nearby_fingers(self):
        inner = 0.033
        tolerance = 0.004
        height = 0.060
        maximum_gap = 0.001
        ring_rotation = self.data.xmat[self.ring_body].reshape(3, 3)
        ring_position = self.data.xpos[self.ring_body]
        ring_axis = ring_rotation[:, 2]
        nearby = []
        for finger, geom_id in self.pad_ids.items():
            local = ring_rotation.T @ (
                self.data.geom_xpos[geom_id] - ring_position)
            radius = math.hypot(local[0], local[1])
            if radius < 1e-9 or radius > inner + tolerance:
                continue
            radial_world = ring_rotation @ np.array(
                (local[0] / radius, local[1] / radius, 0.0))
            geom_rotation = self.data.geom_xmat[geom_id].reshape(3, 3)
            size = self.model.geom_size[geom_id]
            radial_extent = float(np.sum(
                size * np.abs(geom_rotation.T @ radial_world)))
            axial_extent = float(np.sum(
                size * np.abs(geom_rotation.T @ ring_axis)))
            gap = inner - radius - radial_extent
            if (gap < maximum_gap and
                    local[2] + axial_extent >= tolerance and
                    local[2] - axial_extent <= height - tolerance):
                nearby.append((finger, gap))
        return nearby

    def forbidden_contacts(self, allow_active_ring=False):
        result = []
        active_pads = set(self.pad_ids.values())
        for contact in self.data.contact:
            pair = {int(contact.geom1), int(contact.geom2)}
            robot = pair & self.robot_geoms
            obstacle = pair & self.obstacle_geoms
            if not robot or not obstacle:
                continue
            if (allow_active_ring and robot <= active_pads and
                    obstacle <= self.ring_geoms):
                continue
            result.append((
                mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
                mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
                float(contact.dist),
            ))
        return result

    def audit_segment(
            self, start, end, expansion, start_height, end_height,
            start_y=0.0, end_y=0.0, allow_active_ring=False):
        for ratio in np.linspace(0.0, 1.0, 81):
            joints = (
                (1.0 - ratio) * np.asarray(start) +
                ratio * np.asarray(end))
            height = (1.0 - ratio) * start_height + ratio * end_height
            y = (1.0 - ratio) * start_y + ratio * end_y
            self.set_configuration(
                joints, expansion, base_y=y, ring_y=y,
                ring_z=self.source_height + height)
            contacts = self.forbidden_contacts(allow_active_ring)
            if contacts:
                raise RuntimeError(
                    "forbidden planned contact at ratio {:.3f}: {}".format(
                        ratio, contacts[0]))

    def run(self):
        waypoints = self.config["vertical_waypoints"]
        heights = [float(item["height_m"]) for item in waypoints]
        joints = [np.asarray(item["right_arm_rad"], dtype=float)
                  for item in waypoints]
        lower = self.model.jnt_range[self.right_arm_joint_ids, 0]
        upper = self.model.jnt_range[self.right_arm_joint_ids, 1]
        for joint_vector in joints + [np.asarray(
                self.config["place_right_arm_rad"], dtype=float)]:
            if np.any(joint_vector <= lower) or np.any(joint_vector >= upper):
                raise RuntimeError("configured Task 3 joint lies at/outside limit")

        compact = float(self.config["compact_expansion"])
        grasp = float(self.config["grasp_expansion"])
        self.set_configuration(joints[0], compact)
        if self.forbidden_contacts() or self.nearby_fingers():
            raise RuntimeError("compact contact pose is not insertion-clear")
        self.set_configuration(joints[0], grasp)
        nearby = self.nearby_fingers()
        if len(nearby) < 2:
            raise RuntimeError("grasp pose has fewer than two inner-wall pads")
        if self.forbidden_contacts(allow_active_ring=True):
            raise RuntimeError("grasp pose has a non-pad collision")

        above_index = heights.index(0.100)
        ready = np.asarray((
            -0.9, -0.265, 1.0, -0.8, 0.58, -0.4, 0.35))
        self.audit_segment(
            ready, joints[above_index], compact, 0.0, 0.0)
        for index in range(above_index, 0, -1):
            self.audit_segment(
                joints[index], joints[index - 1], compact, 0.0, 0.0)
        for index in range(len(joints) - 1):
            self.audit_segment(
                joints[index], joints[index + 1], grasp,
                heights[index], heights[index + 1],
                allow_active_ring=True)
        self.audit_segment(
            joints[-1], joints[-1], grasp,
            heights[-1], heights[-1], start_y=0.0, end_y=0.85,
            allow_active_ring=True)
        place = np.asarray(self.config["place_right_arm_rad"], dtype=float)
        self.audit_segment(
            joints[-1], place, grasp, heights[-1],
            float(self.config["place_height_m"]),
            start_y=0.85, end_y=0.85, allow_active_ring=True)
        self.set_configuration(
            place, compact, base_y=0.85, ring_y=0.85,
            ring_z=(self.source_height +
                    float(self.config["place_height_m"])))
        release_nearby = self.nearby_fingers()
        if len(release_nearby) > 1:
            raise RuntimeError("compact release leaves more than one pad nearby")
        return nearby, release_nearby


def main():
    with CONFIG.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    nearby, release_nearby = FullFlowAudit(SCENE, config).run()
    print("scene={}".format(SCENE))
    print("config={}".format(CONFIG))
    print("grasp_nearby={}".format([
        (name, round(1000.0 * gap, 3)) for name, gap in nearby]))
    print("release_nearby={}".format([
        (name, round(1000.0 * gap, 3))
        for name, gap in release_nearby]))
    print("task3_full_flow_offline_gate=PASS")


if __name__ == "__main__":
    main()
