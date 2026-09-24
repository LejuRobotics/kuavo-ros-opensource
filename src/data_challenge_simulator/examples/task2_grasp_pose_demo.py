#!/usr/bin/env python3
"""Interactively tune the Task 2 bimanual box-grasp pose without ROS."""

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.task2_pick_planner import Task2PickPlanner


DEFAULT_CONFIG = PACKAGE_DIR / "config/task2_pick.json"
FINGERS = ("thumb", "index", "middle", "little")
MOVING_FINGERS = ("thumb", "index")
FIXED_FINGERS = ("middle", "little")
BOX_INNER_HALF_WIDTH_M = 0.20
BOX_OUTER_HALF_WIDTH_M = 0.22


def initial_box_poses(planner):
    poses = {}
    for name, address in planner.box_qpos_addresses.items():
        qpos = planner.model.qpos0[address:address + 7]
        poses[name] = (
            tuple(float(value) for value in qpos[:3]),
            tuple(float(value) for value in (
                qpos[4], qpos[5], qpos[6], qpos[3])),
        )
    return poses


def fixed_jaw_tcp_in_r7(planner, box_poses, side):
    zero_arms = {"l": np.zeros(7), "r": np.zeros(7)}
    planner._set_configuration(
        (0.0, 0.0, 0.0), box_poses, zero_arms, closure=0.0)
    body_id = planner.r7_body_ids[side]
    rotation = planner.data.xmat[body_id].reshape(3, 3)
    origin = planner.data.xpos[body_id]
    points = []
    for finger in FIXED_FINGERS:
        geom_id = planner._required_id(
            mujoco.mjtObj.mjOBJ_GEOM,
            "{}_{}_fingertip_collision".format(side, finger))
        points.append(
            rotation.T @ (planner.data.geom_xpos[geom_id] - origin))
    return np.mean(points, axis=0)


class GraspPoseDemo:
    def __init__(self, args):
        with args.config.open("r", encoding="utf-8") as stream:
            config = json.load(stream)
        self.grasp_config = config["grasp"]
        self.planner = Task2PickPlanner(ik_config=config["ik"])
        self.reference_box_poses = initial_box_poses(self.planner)
        self.box_poses = dict(self.reference_box_poses)
        self.box_name = args.box
        self.box_position = np.asarray(
            self.box_poses[self.box_name][0], dtype=float)
        self.box_rotation = Rotation.from_quat(
            self.box_poses[self.box_name][1]).as_matrix()
        self.outward_angle_deg = float(
            self.grasp_config["outward_angle_deg"]
            if args.outward_angle_deg is None else args.outward_angle_deg)
        self.forward_tilt_deg = float(args.forward_tilt_deg)
        self.lateral_tilt_deg = float(args.lateral_tilt_deg)
        self.contact_z_m = float(
            self.grasp_config["contact_z_local_m"]
            if args.contact_z_m is None else args.contact_z_m)
        self.fixed_jaw_y_m = float(
            self.grasp_config["fixed_jaw_y_local_m"]
            if args.fixed_jaw_y_m is None else args.fixed_jaw_y_m)
        self.tangent_offset_m = float(
            self.grasp_config["tangent_offset_m"]
            if args.tangent_offset_m is None else args.tangent_offset_m)
        self.closure = float(
            self.grasp_config["grasp_closure"]
            if args.closure is None else args.closure)
        self.standoff_m = float(
            config["docking"]["front_standoff_m"]
            if args.standoff_m is None else args.standoff_m)
        self.pending_keys = []
        self.key_lock = threading.Lock()
        self.arms = None
        self.contact_arms = None
        self.contact_rotations = None
        self.lift_path = None
        self.lift_index = int(args.lift_waypoint)
        self.hand_body_ids = {
            side: self.planner._required_id(
                mujoco.mjtObj.mjOBJ_BODY, "{}_hand_base".format(side))
            for side in ("l", "r")
        }
        self.fixed_tcp = {
            side: fixed_jaw_tcp_in_r7(
                self.planner, self.reference_box_poses, side)
            for side in ("l", "r")
        }
        self._show_fingertip_geoms()
        self.solve()

    def _show_fingertip_geoms(self):
        colors = {
            "thumb": (1.0, 0.15, 0.10, 0.9),
            "index": (1.0, 0.55, 0.05, 0.9),
            "middle": (0.10, 0.65, 1.0, 0.9),
            "little": (0.10, 1.0, 0.45, 0.9),
        }
        for side in ("l", "r"):
            for finger in FINGERS:
                geom_id = self.planner._required_id(
                    mujoco.mjtObj.mjOBJ_GEOM,
                    "{}_{}_fingertip_collision".format(side, finger))
                self.planner.model.geom_rgba[geom_id] = colors[finger]

    def solve(self):
        forward = self.box_rotation[:, 0]
        lateral = self.box_rotation[:, 1]
        up = self.box_rotation[:, 2]
        base_position = self.box_position - self.standoff_m * forward
        base_pose = (
            float(base_position[0]), float(base_position[1]),
            math.atan2(forward[1], forward[0]))
        seeds = self.arms or {
            "l": (-1.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "r": (-1.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        }
        solved = {}
        for side, sign in (("l", 1.0), ("r", -1.0)):
            orientation = self.planner._forward_finger_orientation(
                side, forward, up)
            outward_rotation = Rotation.from_rotvec(
                sign * math.radians(self.outward_angle_deg) * up).as_matrix()
            forward_tilt = Rotation.from_rotvec(
                math.radians(self.forward_tilt_deg) * forward).as_matrix()
            lateral_tilt = Rotation.from_rotvec(
                math.radians(self.lateral_tilt_deg) * lateral).as_matrix()
            orientation = (
                lateral_tilt @ forward_tilt @ outward_rotation @ orientation)
            local_target = np.array((
                self.tangent_offset_m,
                sign * self.fixed_jaw_y_m,
                self.contact_z_m))
            target = self.box_position + self.box_rotation @ local_target
            result = self.planner._solve_arm(
                side, target, orientation, seeds[side], base_pose,
                self.reference_box_poses, self.fixed_tcp[side])
            solved[side] = result.joints
        self.contact_arms = solved
        self.contact_rotations = {}
        self.planner._set_configuration(
            base_pose, self.reference_box_poses, self.contact_arms,
            closure=self.closure)
        for side in ("l", "r"):
            body_id = self.planner.r7_body_ids[side]
            self.contact_rotations[side] = Rotation.from_matrix(
                self.planner.data.xmat[body_id].reshape(3, 3).copy())
        waypoint_count = int(self.grasp_config["lift_waypoint_count"])
        self.lift_path = self.planner.solve_lift_path(
            tuple(self.contact_arms["l"] + self.contact_arms["r"]),
            base_pose, self.reference_box_poses, self.box_name,
            self.grasp_config)
        self.lift_index = min(max(0, self.lift_index), waypoint_count - 1)
        self._show_lift_waypoint(self.lift_index)

    def _hand_midpoint(self):
        return 0.5 * sum(
            (self.planner.data.xpos[body_id].copy()
             for body_id in self.hand_body_ids.values()),
            np.zeros(3))

    def _show_lift_waypoint(self, index):
        self.lift_index = min(max(0, int(index)), len(self.lift_path) - 1)
        waypoint = self.lift_path[self.lift_index]
        self.arms = {"l": waypoint[:7], "r": waypoint[7:]}
        forward = self.box_rotation[:, 0]
        base_position = self.box_position - self.standoff_m * forward
        base_pose = (
            float(base_position[0]), float(base_position[1]),
            math.atan2(forward[1], forward[0]))

        contact_arms = self.contact_arms
        self.planner._set_configuration(
            base_pose, self.reference_box_poses, contact_arms,
            closure=self.closure)
        contact_midpoint = self._hand_midpoint()
        object_offset = self.box_position - contact_midpoint

        self.planner._set_configuration(
            base_pose, self.reference_box_poses, self.arms,
            closure=self.closure)
        lifted_midpoint = self._hand_midpoint()
        lifted_position = lifted_midpoint + object_offset
        self.box_poses = dict(self.reference_box_poses)
        self.box_poses[self.box_name] = (
            tuple(float(value) for value in lifted_position),
            self.reference_box_poses[self.box_name][1])
        self.planner._set_configuration(
            base_pose, self.box_poses, self.arms, closure=self.closure)
        self.print_state()

    def set_closure(self, value):
        self.closure = min(1.0, max(0.0, float(value)))
        forward = self.box_rotation[:, 0]
        base_position = self.box_position - self.standoff_m * forward
        base_pose = (
            float(base_position[0]), float(base_position[1]),
            math.atan2(forward[1], forward[0]))
        self.planner._set_configuration(
            base_pose, self.box_poses, self.arms, closure=self.closure)
        self.print_state()

    def _projected_radius(self, geom_id, world_axis):
        rotation = self.planner.data.geom_xmat[geom_id].reshape(3, 3)
        size = self.planner.model.geom_size[geom_id]
        return float(np.sum(size * np.abs(rotation.T @ world_axis)))

    def print_state(self):
        print(
            "\nwaypoint={}/{}  outward={:.1f} deg  "
            "forward-tilt={:+.1f} deg  lateral-tilt={:+.1f} deg  "
            "z={:.3f} m  fixed_y={:.3f} m  tangent={:+.3f} m  "
            "closure={:.2f}  standoff={:.3f} m".format(
                self.lift_index, len(self.lift_path) - 1,
                self.outward_angle_deg, self.forward_tilt_deg,
                self.lateral_tilt_deg, self.contact_z_m,
                self.fixed_jaw_y_m, self.tangent_offset_m,
                self.closure, self.standoff_m))
        lateral = self.box_rotation[:, 1]
        up = self.box_rotation[:, 2]
        displayed_box_position = np.asarray(
            self.box_poses[self.box_name][0], dtype=float)
        for side, sign in (("l", 1.0), ("r", -1.0)):
            points = {}
            moving_gaps = []
            fixed_gaps = []
            lowest = float("inf")
            for finger in FINGERS:
                geom_id = self.planner._required_id(
                    mujoco.mjtObj.mjOBJ_GEOM,
                    "{}_{}_fingertip_collision".format(side, finger))
                local = self.box_rotation.T @ (
                    self.planner.data.geom_xpos[geom_id] -
                    displayed_box_position)
                points[finger] = local
                lateral_radius = self._projected_radius(geom_id, lateral)
                vertical_radius = self._projected_radius(geom_id, up)
                lowest = min(lowest, float(local[2] - vertical_radius))
                if finger in MOVING_FINGERS:
                    moving_gaps.append(
                        BOX_INNER_HALF_WIDTH_M -
                        sign * float(local[1]) - lateral_radius)
                else:
                    fixed_gaps.append(
                        sign * float(local[1]) - lateral_radius -
                        BOX_OUTER_HALF_WIDTH_M)
            line = points["little"] - points["middle"]
            line_angle = math.degrees(math.atan2(line[1], line[0]))
            body_id = self.planner.r7_body_ids[side]
            current_rotation = Rotation.from_matrix(
                self.planner.data.xmat[body_id].reshape(3, 3))
            orientation_delta = math.degrees(float(np.linalg.norm(
                (self.contact_rotations[side].inv() *
                 current_rotation).as_rotvec())))
            print(
                "{}: orientation-delta={:.6f} deg  "
                "fixed-line={:+.2f} deg  moving-wall-gap={:+.1f} mm  "
                "fixed-wall-gap={:+.1f} mm  lowest-pad={:+.1f} mm".format(
                    side, orientation_delta, line_angle,
                    1000.0 * min(moving_gaps),
                    1000.0 * min(fixed_gaps), 1000.0 * lowest))

    def key_callback(self, keycode):
        try:
            key = chr(keycode).lower()
        except (TypeError, ValueError):
            return
        with self.key_lock:
            self.pending_keys.append(key)

    def process_keys(self):
        with self.key_lock:
            keys = self.pending_keys[:]
            self.pending_keys.clear()
        for key in keys:
            if key == "q":
                self.set_closure(self.closure - 0.05)
            elif key == "e":
                self.set_closure(self.closure + 0.05)
            elif key == "a":
                self.outward_angle_deg -= 1.0
                self.solve()
            elif key == "d":
                self.outward_angle_deg += 1.0
                self.solve()
            elif key == "s":
                self.contact_z_m -= 0.005
                self.solve()
            elif key == "w":
                self.contact_z_m += 0.005
                self.solve()
            elif key == "j":
                self.fixed_jaw_y_m -= 0.002
                self.solve()
            elif key == "l":
                self.fixed_jaw_y_m += 0.002
                self.solve()
            elif key == "f":
                self.standoff_m -= 0.01
                self.solve()
            elif key == "r":
                self.standoff_m += 0.01
                self.solve()
            elif key == "c":
                self.tangent_offset_m -= 0.01
                self.solve()
            elif key == "v":
                self.tangent_offset_m += 0.01
                self.solve()
            elif key == "u":
                self._show_lift_waypoint(self.lift_index + 1)
            elif key == "i":
                self._show_lift_waypoint(self.lift_index - 1)
            elif key == "t":
                target = 0 if self.lift_index else len(self.lift_path) - 1
                self._show_lift_waypoint(target)
            elif key == "z":
                self.forward_tilt_deg -= 1.0
                self.solve()
            elif key == "x":
                self.forward_tilt_deg += 1.0
                self.solve()
            elif key == "n":
                self.lateral_tilt_deg -= 1.0
                self.solve()
            elif key == "m":
                self.lateral_tilt_deg += 1.0
                self.solve()


def main():
    parser = argparse.ArgumentParser(
        description="Tune the Task 2 grasp pose without ROS initialization")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--box", choices=("box_1", "box_2"), default="box_2")
    parser.add_argument("--outward-angle-deg", type=float)
    parser.add_argument("--forward-tilt-deg", type=float, default=0.0)
    parser.add_argument("--lateral-tilt-deg", type=float, default=0.0)
    parser.add_argument("--contact-z-m", type=float)
    parser.add_argument("--fixed-jaw-y-m", type=float)
    parser.add_argument("--tangent-offset-m", type=float)
    parser.add_argument("--closure", type=float)
    parser.add_argument("--standoff-m", type=float)
    parser.add_argument("--lift-waypoint", type=int, default=0)
    parser.add_argument("--no-viewer", action="store_true")
    args = parser.parse_args()
    if args.closure is not None and not 0.0 <= args.closure <= 1.0:
        parser.error("closure must be in [0, 1]")
    if args.lift_waypoint < 0:
        parser.error("lift-waypoint must be non-negative")

    demo = GraspPoseDemo(args)
    if args.no_viewer:
        return

    print(
        "controls: Q/E closure  A/D outward angle  S/W height  "
        "J/L fixed jaw Y  F/R standoff  C/V tangent  "
        "I/U lift waypoint  T contact/final lift  "
        "Z/X forward tilt  N/M lateral tilt")
    from mujoco import viewer as mujoco_viewer
    with mujoco_viewer.launch_passive(
            demo.planner.model, demo.planner.data,
            key_callback=demo.key_callback) as viewer:
        viewer.cam.lookat[:] = demo.box_position
        viewer.cam.distance = 1.35
        viewer.cam.azimuth = 145.0
        viewer.cam.elevation = -25.0
        while viewer.is_running():
            demo.process_keys()
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
