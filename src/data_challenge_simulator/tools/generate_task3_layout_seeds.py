#!/usr/bin/env python3
"""Manually generate an offline-verified Task 3 layout catalogue."""

import argparse
import json
import math
from pathlib import Path
import random
import secrets
import sys


PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from utils.task3_randomization import RING_NAMES
from utils.task3_second_ik import Task3SecondIKPlanner


DEFAULT_TEMPLATE = PACKAGE_DIR / "config/task3_layout_seeds.json"
DEFAULT_SCENE = (
    PACKAGE_DIR / "models/biped_s400062/xml/task3.xml"
)
DEFAULT_INITIALIZATION = PACKAGE_DIR / "config/task3_initialization.json"


class OfflineLayoutGenerator:
    def __init__(self, generation, scene_path, initialized_right_arm):
        self.generation = generation
        self.scene_path = Path(scene_path)
        self.measured_arm = tuple([0.0] * 7 + list(initialized_right_arm))

    def _sample_positions(self, rng):
        positions = tuple((
            rng.uniform(*self.generation["source_x_range"]),
            rng.uniform(*self.generation["source_y_range"]),
            float(self.generation["source_z"]),
        ) for _ in RING_NAMES)
        minimum = float(self.generation["minimum_ring_separation_m"])
        for first in range(len(positions)):
            for second in range(first + 1, len(positions)):
                if math.dist(
                        positions[first][:2],
                        positions[second][:2]) < minimum:
                    return None
        return positions

    def _verify_docking(self, name, position):
        preferred = self.generation["preferred_ring_in_base_xy"]
        docking = (
            position[0] - preferred[0],
            position[1] - preferred[1],
            0.0,
        )
        planner = Task3SecondIKPlanner(self.scene_path, ring_name=name)
        result = planner.solve_feedforward(
            docking, position, self.measured_arm)
        minimum_margin = float(
            self.generation["minimum_docked_joint_margin_rad"])
        if (result is None or
                result.nearest_joint_limit_margin_rad < minimum_margin):
            return None
        return docking, float(result.nearest_joint_limit_margin_rad)

    def generate(self, seed):
        rng = random.Random(int(seed))
        initial_base = (
            rng.uniform(*self.generation["robot_initial_x_range"]),
            rng.uniform(*self.generation["robot_initial_y_range"]),
            0.0,
        )
        attempts = int(self.generation["maximum_sample_attempts"])
        destinations = self.generation["destination_slots"]
        for _ in range(attempts):
            positions = self._sample_positions(rng)
            if positions is None:
                continue
            rings = []
            for name, position, destination in zip(
                    RING_NAMES, positions, destinations):
                verified = self._verify_docking(name, position)
                if verified is None:
                    break
                docking, margin = verified
                rings.append({
                    "name": name,
                    "position": list(position),
                    "docking_base": list(docking),
                    "destination_xy": list(destination),
                    "joint_margin_rad": margin,
                })
            if len(rings) == len(RING_NAMES):
                return {
                    "seed": int(seed),
                    "initial_base": list(initial_base),
                    "rings": rings,
                }
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument(
        "--initialization", type=Path, default=DEFAULT_INITIALIZATION)
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
    with args.initialization.open("r", encoding="utf-8") as stream:
        initialized_right_arm = json.load(stream)["right_arm_ready_rad"]
    generator = OfflineLayoutGenerator(
        document["generation"], args.scene.resolve(), initialized_right_arm)

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
