#!/usr/bin/env python3
"""Offline audit for Task 2 direct-versus-retreat initialization."""

import argparse
import json
from pathlib import Path
import sys


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PACKAGE_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.task2_initialization import Task2InitializationPlanner


DEFAULT_SCENE = SRC_DIR / (
    "data_challenge_simulator/models/biped_s400062/xml/task2.xml")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--base-x", type=float, default=0.0)
    parser.add_argument("--base-y", type=float, default=0.0)
    parser.add_argument("--base-yaw-deg", type=float, default=0.0)
    parser.add_argument("--collision-margin", type=float, default=0.015)
    args = parser.parse_args()

    with args.config.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    measured = config.get("measured_arm_joints_rad", [0.0] * 14)
    retreat_m = float(config["safe_retreat_m"])
    base_pose = (
        args.base_x, args.base_y,
        args.base_yaw_deg * 3.141592653589793 / 180.0)

    planner = Task2InitializationPlanner(
        args.scene, collision_margin_m=args.collision_margin,
        report_limit_m=float(config["collision_report_limit_m"]))
    safe_pose = planner.backward_pose(base_pose, retreat_m)
    lifted = list(float(value) for value in measured)
    lifted[0] = float(config["shoulder_pitch_rad"])
    lifted[7] = float(config["shoulder_pitch_rad"])
    lift_path_clearance = planner.path_clearance(
        safe_pose, measured, [lifted],
        samples_per_segment=int(config["path_samples"]))
    return_clearance = planner.configuration_clearance(base_pose, lifted)
    print(json.dumps({
        "safe": (
            lift_path_clearance >= args.collision_margin
            and return_clearance >= args.collision_margin),
        "retreat_m": retreat_m,
        "shoulder_pitch_rad": float(config["shoulder_pitch_rad"]),
        "lifted_arm_joints_rad": lifted,
        "lift_path_clearance_m": lift_path_clearance,
        "lifted_at_return_clearance_m": return_clearance,
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
