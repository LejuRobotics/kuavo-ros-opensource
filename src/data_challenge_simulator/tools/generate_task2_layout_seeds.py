#!/usr/bin/env python3
"""Manually generate an offline-verified Task 2 layout catalogue.

The catalogue carries both boxes' world poses and their front docking base
poses.  Docking is the box centre pulled back by the configured front
standoff, so it is a constant offset of the box pose and is stored rather
than recomputed at runtime.

The generator only samples and records.  The X and Y ranges in the template
are deliberately small policy constants, well inside the physical limits, so
no sample is expected to be rejected; the attempt loop is kept so the
structure matches Task 1 V2 and Task 3 and so that widening the ranges later
is a template-only change.
"""

import argparse
import json
from pathlib import Path
import random
import secrets
import sys

import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.task2_pick_planner import BOX_NAMES, Task2PickPlanner


DEFAULT_TEMPLATE = PACKAGE_DIR / "config/task2_layout_seeds.json"
DEFAULT_PICK_CONFIG = PACKAGE_DIR / "config/task2_pick.json"
DEFAULT_SCENE = (
    PACKAGE_DIR / "models/biped_s400062/xml/task2.xml"
)
# box_1 sits on the table's -Y edge and has no room to move in Y.
FIXED_BOX_1_Y = -0.42


class OfflineLayoutGenerator:
    def __init__(self, generation, scene_path, grasp_config, ik_config):
        self.generation = generation
        self.standoff = float(generation["docking_front_standoff_m"])
        self.box_z = float(generation["box_z"])
        self.scene_path = Path(scene_path)
        self.planner = Task2PickPlanner(
            scene_path=self.scene_path, ik_config=ik_config)
        self.grasp_config = grasp_config
        self.arm_seed = np.concatenate([
            self.planner.model.qpos0[
                self.planner.arm_qpos_addresses[side]]
            for side in ("l", "r")
        ]).tolist()

    def _sample_boxes(self, rng):
        x_range = self.generation["box_x_range"]
        y_range = self.generation["box_2_y_range"]
        box_1_x = rng.uniform(*x_range)
        box_2_x = rng.uniform(*x_range)
        box_2_y = rng.uniform(*y_range)
        return (
            ("box_1", (box_1_x, FIXED_BOX_1_Y, self.box_z)),
            ("box_2", (box_2_x, box_2_y, self.box_z)),
        )

    def _verify_grasp(self, selected, positions):
        """Return the position error of the solved grasp, or None."""
        box_poses = {
            name: (position, (0.0, 0.0, 0.0, 1.0))
            for name, position in positions
        }
        box_x, box_y, _ = dict(positions)[selected]
        docking_base = (box_x - self.standoff, box_y, 0.0)
        try:
            self.planner.plan(
                selected, docking_base, box_poses, self.arm_seed,
                self.grasp_config)
        except RuntimeError as error:
            print("  {} grasp rejected: {}".format(selected, error))
            return None
        return docking_base

    def generate(self, seed):
        rng = random.Random(int(seed))
        initial_base = (
            rng.uniform(*self.generation["robot_initial_x_range"]),
            rng.uniform(*self.generation["robot_initial_y_range"]),
            0.0,
        )
        attempts = int(self.generation["maximum_sample_attempts"])
        for _ in range(attempts):
            sampled = self._sample_boxes(rng)
            boxes = []
            for name, position in sampled:
                docking_base = self._verify_grasp(name, sampled)
                if docking_base is None:
                    break
                boxes.append({
                    "name": name,
                    "position": list(position),
                    "yaw": 0.0,
                    "docking_base": list(docking_base),
                })
            if len(boxes) == len(BOX_NAMES):
                return {
                    "seed": int(seed),
                    "initial_base": list(initial_base),
                    "boxes": boxes,
                }
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument(
        "--pick-config", type=Path, default=DEFAULT_PICK_CONFIG)
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
    with args.pick_config.open("r", encoding="utf-8") as stream:
        pick_config = json.load(stream)
    generator = OfflineLayoutGenerator(
        document["generation"], args.scene.resolve(),
        pick_config["grasp"], pick_config["ik"])

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
        if layout is None:
            print("rejected seed {}".format(seed))
        else:
            layouts.append(layout)
            print("accepted seed {} ({}/{})".format(
                seed, len(layouts), args.count))
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
