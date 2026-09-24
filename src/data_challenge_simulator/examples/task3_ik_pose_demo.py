#!/usr/bin/env python3
"""Inspect Task 3 hand-shape and arm-pose targets without ROS."""

import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


CURRENT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = CURRENT_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.task3_descent import FINGERS, Task3DescentPlanner
from utils.task3_right_arm_ik import Task3RightArmIK

INITIALIZATION_CONFIG = PACKAGE_DIR / "config/task3_initialization.json"
TRANSFER_CONFIG = PACKAGE_DIR / "config/task3_transfer.json"
DEFAULT_SCENE = PACKAGE_DIR / (
    "models/biped_s400062/xml/task3.xml")
STAGES = (
    "above_open", "above_compact", "insertion_parallel",
    "insertion_tilted")


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


class Task3IKPoseDemo:
    """Solve and display Task 3 target poses without a controller in the loop."""

    def __init__(self, args):
        initialization = load_json(args.initialization_config)
        transfer = load_json(args.transfer_config)
        self.base_pose = tuple(float(value) for value in args.base_pose)
        self.planner = Task3DescentPlanner(args.scene)
        self.model = self.planner.model
        self.data = self.planner.data
        self.ring_position = self._initial_ring_position()
        ready_right = tuple(
            float(value) for value in initialization["right_arm_ready_rad"])
        nominal_arm = (0.0,) * 7 + ready_right

        above_solver = Task3RightArmIK(args.scene)
        above = above_solver.solve_above_ring(
            self.base_pose,
            self.ring_position,
            nominal_arm,
            float(transfer["above_ring_clearance_m"]),
        )
        if above is None:
            raise RuntimeError("nominal Task 3 above-ring IK is not solvable")
        self.above_right = tuple(above.joints)

        above_arm = (0.0,) * 7 + self.above_right
        assessment = self.planner.assess(
            self.base_pose, self.ring_position, above_arm)
        insertion = self.planner.solve(
            self.base_pose,
            self.ring_position,
            above_arm,
            assessment=assessment,
        )
        if insertion is None:
            raise RuntimeError(
                "no collision-free Task 3 insertion IK follows the "
                "nominal above-ring pose")
        self.insertion = insertion
        self.insertion_right = tuple(insertion.right_joints)
        _, self.above_rotation = self.planner.ik._pose(
            self.base_pose, self.above_right)
        self.parallel_right, self.parallel_waypoints = (
            self._solve_parallel_insertion())
        self.stage = args.stage
        self.pending_keys = []
        self.key_lock = threading.Lock()
        self._highlight_geometry()
        self.show(self.stage)

    def _initial_ring_position(self):
        body_id = self.planner._required_id(
            mujoco.mjtObj.mjOBJ_BODY, "task3_hollow_cylinder")
        joint_id = int(self.model.body_jntadr[body_id])
        address = int(self.model.jnt_qposadr[joint_id])
        return tuple(
            float(value) for value in self.model.qpos0[address:address + 3])

    def _highlight_geometry(self):
        colors = {
            "index": (1.0, 0.20, 0.10, 1.0),
            "middle": (1.0, 0.75, 0.05, 1.0),
            "little": (0.15, 1.0, 0.35, 1.0),
        }
        for finger, _ in FINGERS:
            geom_id = self.planner._required_id(
                mujoco.mjtObj.mjOBJ_GEOM,
                "r_{}_fingertip_collision".format(finger),
            )
            self.model.geom_rgba[geom_id] = colors[finger]

    def _solve_parallel_insertion(self):
        """Keep the accepted above-ring palm direction at insertion depth."""
        self.planner._set_configuration(
            self.base_pose, self.above_right, self.ring_position)
        hand_body = self.planner.ik.hand_body
        hand_position = self.data.xpos[hand_body].copy()
        hand_rotation = self.data.xmat[hand_body].reshape(3, 3).copy()
        tips_hand = (
            hand_rotation.T @
            (self.planner._tip_geometry() - hand_position).T).T

        target_rotation = self.above_rotation
        tip_offsets = target_rotation.apply(tips_hand)
        ring = np.asarray(self.ring_position, dtype=float)
        target_position = np.asarray((
            *(ring[:2] - np.mean(tip_offsets[:, :2], axis=0)),
            ring[2] + self.planner.ring_height
            - self.planner.INSERTION_MARGIN_M
            - np.max(tip_offsets[:, 2]),
        ))
        seed = np.asarray(self.above_right, dtype=float)

        def residual(joints):
            position, rotation = self.planner.ik._pose(
                self.base_pose, joints)
            return np.concatenate((
                30.0 * (position - target_position),
                (target_rotation.inv() * rotation).as_rotvec(),
                0.001 * (joints - seed),
            ))

        solved = least_squares(
            residual, seed,
            bounds=(self.planner.ik.lower, self.planner.ik.upper),
            max_nfev=5000, xtol=1e-12, ftol=1e-12, gtol=1e-12)
        joints = np.asarray(solved.x, dtype=float)
        position, rotation = self.planner.ik._pose(self.base_pose, joints)
        position_error = np.linalg.norm(position - target_position)
        rotation_error = (
            target_rotation.inv() * rotation).magnitude()
        self.planner._set_configuration(
            self.base_pose, joints, self.ring_position)
        _, radii, depths = self.planner._metrics(self.ring_position)
        if (position_error > 5e-4 or rotation_error > math.radians(0.2) or
                np.any(radii >= self.planner.inner_radius) or
                np.any(depths <= 0.0) or self.planner._ring_contacts()):
            raise RuntimeError(
                "near-parallel insertion target is not geometrically reachable")
        path = self.planner._rrt_path(
            self.base_pose, self.ring_position, seed, joints)
        if path is None:
            raise RuntimeError(
                "near-parallel insertion target has no collision-free path")
        return (
            tuple(float(value) for value in joints),
            tuple(tuple(float(value) for value in waypoint)
                  for waypoint in path),
        )

    def _contact_names(self):
        contacts = []
        for contact in self.data.contact:
            names = []
            for geom_id in (contact.geom1, contact.geom2):
                names.append(
                    mujoco.mj_id2name(
                        self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                    or "geom_{}".format(geom_id))
            if any(name.startswith("r_") for name in names):
                contacts.append(tuple(names))
        return contacts

    def _set_stage_configuration(self, stage):
        if stage == "insertion_tilted":
            right = self.insertion_right
        elif stage == "insertion_parallel":
            right = self.parallel_right
        else:
            right = self.above_right
        self.planner._set_configuration(
            self.base_pose, right, self.ring_position)
        if stage == "above_open":
            for address in self.planner.compact_addresses.values():
                self.data.qpos[address] = self.model.qpos0[address]
            mujoco.mj_forward(self.model, self.data)
        return right

    def _finger_axis_tilts_deg(self):
        tilts = []
        for finger, distal_body_name in FINGERS:
            geom_id = self.planner._required_id(
                mujoco.mjtObj.mjOBJ_GEOM,
                "r_{}_fingertip_collision".format(finger))
            body_id = self.planner._required_id(
                mujoco.mjtObj.mjOBJ_BODY, distal_body_name)
            axis = self.data.geom_xpos[geom_id] - self.data.xpos[body_id]
            axis /= np.linalg.norm(axis)
            tilts.append(math.degrees(math.acos(np.clip(
                abs(float(axis[2])), 0.0, 1.0))))
        return np.asarray(tilts)

    def show(self, stage):
        if stage not in STAGES:
            raise ValueError("unknown Task 3 IK demo stage: {}".format(stage))
        self.stage = stage
        right = self._set_stage_configuration(stage)
        tips, radii, depths = self.planner._metrics(self.ring_position)
        hand_body = self.planner.ik.hand_body
        hand_position = self.data.xpos[hand_body].copy()
        hand_rotation = Rotation.from_matrix(
            self.data.xmat[hand_body].reshape(3, 3).copy())
        wrist_change_deg = math.degrees(
            (self.above_rotation.inv() * hand_rotation).magnitude())
        margin = np.minimum(
            np.asarray(right) - self.planner.ik.lower,
            self.planner.ik.upper - np.asarray(right),
        )
        print("\nstage={}".format(stage))
        print("right_arm_rad={}".format(
            np.round(np.asarray(right), 6).tolist()))
        print("hand_base_world_m={}".format(
            np.round(hand_position, 6).tolist()))
        print("hand_base_quat_xyzw={}".format(
            np.round(hand_rotation.as_quat(), 8).tolist()))
        print("hand_base_rotation_matrix={}".format(
            np.round(hand_rotation.as_matrix(), 8).tolist()))
        print("fingertip_fronts_world_m={}".format(
            np.round(tips, 6).tolist()))
        print("tip_radii_mm={}".format(
            np.round(1000.0 * radii, 3).tolist()))
        print("tip_depths_below_top_mm={}".format(
            np.round(1000.0 * depths, 3).tolist()))
        print("finger_axis_tilts_from_vertical_deg={}".format(
            np.round(self._finger_axis_tilts_deg(), 3).tolist()))
        print("wrist_rotation_from_above_deg={:.3f}".format(
            wrist_change_deg))
        print("nearest_joint_limit_margin_deg={:.3f}".format(
            math.degrees(float(np.min(margin)))))
        print("right_arm_contacts={}".format(
            self._contact_names() or "none"))
        if stage in ("insertion_parallel", "insertion_tilted"):
            print("joint_path_waypoints={}".format(
                len(self.parallel_waypoints) if stage == "insertion_parallel"
                else len(self.insertion.right_waypoints)))
            print("nominal_insertion_geometry={}".format(
                "PASS" if (
                    np.all(radii < self.planner.inner_radius) and
                    np.all(depths > 0.0) and
                    not self._contact_names()) else "FAIL"))

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
            if key == "t":
                index = (STAGES.index(self.stage) + 1) % len(STAGES)
                self.show(STAGES[index])
            elif key in ("1", "2", "3", "4"):
                self.show(STAGES[int(key) - 1])


def main():
    parser = argparse.ArgumentParser(
        description="Display Task 3 IK targets without ROS or controllers")
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument(
        "--initialization-config", type=Path, default=INITIALIZATION_CONFIG)
    parser.add_argument("--transfer-config", type=Path, default=TRANSFER_CONFIG)
    parser.add_argument(
        "--base-pose", type=float, nargs=3, metavar=("X", "Y", "YAW"),
        default=(0.0, 0.0, 0.0))
    parser.add_argument(
        "--stage", choices=STAGES, default="insertion_parallel")
    parser.add_argument("--no-viewer", action="store_true")
    args = parser.parse_args()

    demo = Task3IKPoseDemo(args)
    if args.no_viewer:
        return

    print("controls: T cycle stages; 1 above-open; 2 above-compact; "
          "3 insertion-parallel; 4 insertion-tilted")
    from mujoco import viewer as mujoco_viewer
    with mujoco_viewer.launch_passive(
            demo.model, demo.data,
            key_callback=demo.key_callback) as viewer:
        viewer.cam.lookat[:] = demo.ring_position
        viewer.cam.distance = 0.82
        viewer.cam.azimuth = 145.0
        viewer.cam.elevation = -22.0
        while viewer.is_running():
            demo.process_keys()
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
