#!/usr/bin/env python3
"""Manually generate an offline-verified Task 1 V2 layout catalogue."""

import argparse
import contextlib
import io
import json
import math
from pathlib import Path
import random
import secrets
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R


PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.hand_postures import single_gripper_targets
from utils.scene1_v2_right_arm_ik import Scene1V2RightArmIK


DEFAULT_TEMPLATE = PACKAGE_DIR / "config/task1_v2_layout_seeds.json"
# User X is the transverse-conveyor direction (world -Y); user Y is robot
# forward (world X).  The default open thumb requires a rearward spawn, while
# Task1 initialization rotates l_thumb_j1 to 1.75 rad before advancing to the
# independently sampled working pose.  The final working range retains the
# accepted +0.015 m forward cap.
OBJECT_Z = 0.685
MAX_POSITION_ERROR = 0.001
MAX_SOURCE_ORIENTATION_ERROR_DEG = 8.0
SOURCE_WALL_TOP_Z = 0.740
LIFT_CLEARANCE = 0.010
OBJECT_HALF_HEIGHT = 0.020
TARGET_BIN_CENTER_XY = (0.3075, -0.528)
DROP_GRASP_CENTER_Z = 0.820
DROP_CENTER_OFFSETS = (
    (-0.060, -0.012),
    (0.000, 0.022),
    (0.060, -0.012),
)
GRASP_TRACKING_BIASES_WORLD = (
    np.array((0.002, 0.014, 0.010)),
    np.array((0.002, 0.010, 0.010)),
    np.array((0.002, 0.005, 0.010)),
)
RIGHT_ARM_READY = np.array((-0.9, -0.265, 1.0, -0.8, 0.58, -0.4, 0.35))
RIGHT_ARM_STAGING = np.array((
    -0.569927, -0.301512, 0.658253, -0.488704,
    0.698827, -0.653404, -0.289423,
))
GRASP_YAW_ADJUSTMENT_RAD = math.radians(14.0)
GRASP_LIFT_SCALE = 1.65
FIXED_GRASP_LIFTS_RIGHT_RAD = (
    np.array((0.210334, -0.338870, -0.036279, -0.850745,
              -0.089775, 0.299690, 0.193743)),
    np.array((0.189635, -0.314196, -0.020128, -0.875594,
              -0.022019, 0.340597, 0.248531)),
    np.array((0.210334, -0.338870, -0.036279, -0.850745,
              -0.089775, 0.299690, 0.193743)),
)
OPEN_HAND = np.array(single_gripper_targets("r", 0.0))
CLOSED_HAND = np.array(single_gripper_targets("r", 1.0))


class OfflineLayoutGenerator:
    def __init__(self, generation):
        self.generation = generation
        self.scene_ik = Scene1V2RightArmIK()
        self.scene_ik.ignored_task_contact_geoms.update({
            "block_3_visual", "block_3_geom",
        })
        self.orientation = self.scene_ik.eef_quaternion_world_with_yaw(
            RIGHT_ARM_READY, GRASP_YAW_ADJUSTMENT_RAD)
        self.initial_right = self.scene_ik.model.qpos0[
            self.scene_ik.qpos_addresses].copy()
        self.initial_hand = self.scene_ik.model.qpos0[
            self.scene_ik.hand_qpos_addresses].copy()

    def sample(self, seed):
        rng = random.Random(int(seed))
        initial_base = (
            rng.uniform(*self.generation["robot_initial_x_range"]),
            rng.uniform(*self.generation["robot_initial_y_range"]),
            0.0,
        )
        profiles = self.generation["object_region_profiles"]
        profile = profiles[rng.randrange(len(profiles))]
        positions = tuple(
            (
                rng.uniform(*region["x"]),
                rng.uniform(*region["y"]),
                OBJECT_Z,
            )
            for region in profile["regions"]
        )
        cylinders = tuple({
            "position": position,
            "yaw": rng.uniform(-math.pi, math.pi),
        } for position in positions)
        minimum_transverse = float(
            self.generation["minimum_object_transverse_separation"])
        for first in range(len(cylinders)):
            for second in range(first + 1, len(cylinders)):
                # User X points to the robot's right (world -Y).  Keep a small
                # separation on that transverse axis; user Y/world X only has
                # to remain inside its reachable sampling range.
                if abs(
                        cylinders[first]["position"][1]
                        - cylinders[second]["position"][1]
                ) < minimum_transverse:
                    return None
        task_base = tuple(
            float(value) for value in self.generation["direct_task_base"])
        return initial_base, task_base, cylinders, profile["name"]

    def solve(self, target, seed):
        with contextlib.redirect_stdout(io.StringIO()):
            solution = np.asarray(self.scene_ik.solve(
                target,
                self.orientation,
                seed,
                point_in_r7=self.scene_ik.CLOSED_GRASP_CENTER_IN_R7,
            ))
        achieved = np.asarray(
            self.scene_ik.closed_grasp_center_world(solution))
        target_rotation = R.from_quat(self.orientation)
        achieved_rotation = R.from_matrix(
            self.scene_ik.eef_rotation_world(solution))
        orientation_error_deg = math.degrees(np.linalg.norm(
            (target_rotation.inv() * achieved_rotation).as_rotvec()))
        return (
            solution,
            float(np.linalg.norm(achieved - target)),
            orientation_error_deg,
        )

    def path_clear(self, start, target, hand):
        for phase in np.linspace(0.0, 1.0, 21):
            joints = (1.0 - phase) * start + phase * target
            if self.scene_ik.task_obstacle_contacts(joints, hand):
                return False
        return True

    def base_path_clear(self, start, target, arm, hand):
        for phase in np.linspace(0.0, 1.0, 21):
            base = tuple(
                start[index] + phase * (target[index] - start[index])
                for index in range(3)
            )
            self.scene_ik.set_base_translation_world(base)
            if self.scene_ik.task_obstacle_contacts(arm, hand):
                return False
        return True

    def initialization_clear(self, initial_base, task_base):
        safe_base = (task_base[0] - 0.20, task_base[1], task_base[2])
        return (
            self.base_path_clear(
                initial_base, safe_base, self.initial_right,
                self.initial_hand)
            and self.base_path_clear(
                safe_base, task_base, RIGHT_ARM_READY, OPEN_HAND)
        )

    def source_plan(self, base, cylinder, index):
        self.scene_ik.set_base_translation_world(base)
        grasp_target = (
            np.asarray(cylinder["position"])
            - GRASP_TRACKING_BIASES_WORLD[index])
        grasp, error, orientation_error_deg = self.solve(
            grasp_target, RIGHT_ARM_STAGING)
        if (error > MAX_POSITION_ERROR or
                orientation_error_deg > MAX_SOURCE_ORIENTATION_ERROR_DEG or
                not self.path_clear(
                    RIGHT_ARM_STAGING, grasp, OPEN_HAND)):
            return None
        lifted = (
            grasp + GRASP_LIFT_SCALE * FIXED_GRASP_LIFTS_RIGHT_RAD[index])
        if np.any(lifted < self.scene_ik.lower) or np.any(
                lifted > self.scene_ik.upper):
            return None
        grasp_z = self.scene_ik.closed_grasp_center_world(grasp)[2]
        lifted_z = self.scene_ik.closed_grasp_center_world(lifted)[2]
        object_bottom = (
            cylinder["position"][2] + lifted_z - grasp_z
            - OBJECT_HALF_HEIGHT)
        if object_bottom < SOURCE_WALL_TOP_Z + LIFT_CLEARANCE:
            return None
        if not self.path_clear(grasp, lifted, CLOSED_HAND):
            return None
        return grasp, lifted, error, orientation_error_deg

    def drop_clear(self, base, lifted, index):
        self.scene_ik.set_base_translation_world(base)
        drop_target = np.array((
            TARGET_BIN_CENTER_XY[0] + DROP_CENTER_OFFSETS[index][0],
            TARGET_BIN_CENTER_XY[1] + DROP_CENTER_OFFSETS[index][1],
            DROP_GRASP_CENTER_Z,
        ))
        drop, error, _ = self.solve(drop_target, lifted)
        if (error > MAX_POSITION_ERROR or
                not self.path_clear(lifted, drop, CLOSED_HAND)):
            return None
        return error

    def blue_plan_at_base(self, task_base, cylinder, blue_base):
        if not self.base_path_clear(
                task_base, blue_base, RIGHT_ARM_STAGING, OPEN_HAND):
            return None
        source = self.source_plan(blue_base, cylinder, 2)
        if source is None:
            return None
        _, lifted, error, orientation_error_deg = source
        if not self.base_path_clear(
                blue_base, task_base, lifted, CLOSED_HAND):
            return None
        drop_error = self.drop_clear(task_base, lifted, 2)
        if drop_error is None:
            return None
        return {
            "base": blue_base,
            "source_error": error,
            "source_orientation_error_deg": orientation_error_deg,
            "drop_error": drop_error,
        }

    def blue_plan_at_shift(self, task_base, cylinder, shift):
        blue_base = (
            task_base[0], task_base[1] + shift, task_base[2])
        return self.blue_plan_at_base(task_base, cylinder, blue_base)

    def layout_plan(self, task_base, cylinders):
        grasp_bases = [task_base, task_base, None]
        maximum_error = 0.0
        maximum_orientation_error_deg = 0.0

        for index in (0, 1):
            source = self.source_plan(task_base, cylinders[index], index)
            if source is None:
                return None
            _, lifted, error, orientation_error_deg = source
            drop_error = self.drop_clear(task_base, lifted, index)
            if drop_error is None:
                return None
            maximum_error = max(maximum_error, error, drop_error)
            maximum_orientation_error_deg = max(
                maximum_orientation_error_deg, orientation_error_deg)

        # Blue samples are intentionally outside the direct-grasp set.
        if self.source_plan(task_base, cylinders[2], 2) is not None:
            return None

        shift_min, shift_max = self.generation["blue_left_shift_range"]
        shift_step = float(self.generation["blue_left_shift_step"])
        shift_margin = float(self.generation["blue_left_shift_margin"])
        shift_count = int(round((shift_max - shift_min) / shift_step)) + 1
        minimum_feasible_shift = None
        for step in range(shift_count):
            shift = shift_min + step * shift_step
            if self.blue_plan_at_shift(task_base, cylinders[2], shift):
                minimum_feasible_shift = shift
                break

        if minimum_feasible_shift is None:
            return None

        # Do not command the first just-feasible grid point. Move farther
        # left to absorb chassis undershoot and execution/model mismatch, then
        # validate the final commanded pose through the same complete plan.
        commanded_shift = minimum_feasible_shift + shift_margin
        blue_plan = self.blue_plan_at_shift(
            task_base, cylinders[2], commanded_shift)
        if blue_plan is None:
            return None
        grasp_bases[2] = blue_plan["base"]
        maximum_error = max(
            maximum_error,
            blue_plan["source_error"],
            blue_plan["drop_error"],
        )
        maximum_orientation_error_deg = max(
            maximum_orientation_error_deg,
            blue_plan["source_orientation_error_deg"],
        )
        return {
            "grasp_bases": grasp_bases,
            "blue_minimum_feasible_shift_m": minimum_feasible_shift,
            "blue_shift_margin_m": shift_margin,
            "maximum_error": maximum_error,
            "maximum_orientation_error_deg": (
                maximum_orientation_error_deg),
        }

    def generate(self, seed):
        sampled = self.sample(seed)
        if sampled is None:
            return None
        initial_base, task_base, cylinders, profile_name = sampled
        if not self.initialization_clear(initial_base, task_base):
            return None
        plan = self.layout_plan(task_base, cylinders)
        if plan is None:
            return None

        return {
            "seed": int(seed),
            "initial_base": list(initial_base),
            "task_base": list(task_base),
            "profile": profile_name,
            "grasp_bases": [list(base) for base in plan["grasp_bases"]],
            "blue_minimum_feasible_shift_m": (
                plan["blue_minimum_feasible_shift_m"]),
            "blue_shift_margin_m": plan["blue_shift_margin_m"],
            "cylinders": [{
                "position": list(item["position"]),
                "yaw": item["yaw"],
            } for item in cylinders],
            "offline_maximum_ik_error_m": plan["maximum_error"],
            "offline_maximum_source_orientation_error_deg": (
                plan["maximum_orientation_error_deg"]),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--start-seed", type=int, default=1)
    parser.add_argument(
        "--random-seeds", action="store_true",
        help="draw an independent random seed for every layout attempt")
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("--count must be positive")
    if args.output.resolve() == args.template.resolve():
        parser.error("--output must differ from --template")

    with args.template.open("r", encoding="utf-8") as stream:
        document = json.load(stream)
    generator = OfflineLayoutGenerator(document["generation"])
    layouts = []
    seed = args.start_seed
    attempted_seeds = set()
    while len(layouts) < args.count:
        if args.random_seeds:
            seed = secrets.randbelow(900000000) + 100000000
            if seed in attempted_seeds:
                continue
            attempted_seeds.add(seed)
        layout = generator.generate(seed)
        if layout is not None:
            layouts.append(layout)
            print("accepted seed {} ({}/{})".format(
                seed, len(layouts), args.count))
        else:
            print("rejected seed {}".format(seed))
        if not args.random_seeds:
            seed += 1

    output = {
        "version": document["version"],
        "description": document["description"],
        "generation": document["generation"],
        "layouts": layouts,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2, sort_keys=False)
        stream.write("\n")
    print("wrote {} layouts to {}".format(
        len(layouts), args.output.resolve()))


if __name__ == "__main__":
    main()
