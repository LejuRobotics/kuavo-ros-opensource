"""Deterministic Task 1 sampling shared by task, collect and model."""
from dataclasses import dataclass
from functools import lru_cache
import contextlib
import io
import json
import math
from pathlib import Path
import random

PACKAGE_DIR = Path(__file__).resolve().parents[1]
OBJECT_COUNT = 3
FIXED_TASK_BASE = (0.015, 0.0, 0.0)
DROP_CENTER_OFFSETS = ((-0.045, -0.015), (0.0, 0.015), (0.045, -0.015))

@dataclass(frozen=True)
class CylinderSpawn:
    position: tuple
    yaw: float

@dataclass(frozen=True)
class RandomizationPlan:
    seed: int
    initial_base: tuple
    task_base: tuple
    cylinders: tuple
    target_position: tuple

    @property
    def catalogue_seed(self):
        return self.seed

    @property
    def grasp_bases(self):
        return (self.task_base,) * OBJECT_COUNT

    @property
    def profile(self):
        return "seeded_fixed_base"

    @property
    def requires_base_motion(self):
        return False

@lru_cache(maxsize=1)
def _reach_solver():
    from utils.scene1_v2_right_arm_ik import Scene1V2RightArmIK
    return Scene1V2RightArmIK(PACKAGE_DIR / "models/biped_s400062/xml/task1.xml")

def _reachable(positions, target, base):
    import numpy as np
    from scipy.spatial.transform import Rotation
    ik = _reach_solver()
    ik.set_base_translation_world(base)
    import mujoco
    ik.model.body_pos[ik.model.body("target_bin").id] = target
    for index, position in enumerate(positions, 1):
        joint = ik.model.joint("cylinder_{}_freejoint".format(index)).id
        address = ik.model.jnt_qposadr[joint]
        ik.model.qpos0[address:address+3] = position
    # Reject candidates whose existing preparation curve crosses a fixture.
    # Keep the accepted motion/control; choose a collision-clear reachable spawn.
    path = ik.preparation_trajectory([0.0] * 14, positions[0], points=400)
    for joints in path:
        ik._forward(joints[7:14])
        mujoco.mj_forward(ik.model, ik.data)
        for contact in ik.data.contact:
            right = [ik.model.body(int(ik.model.geom_bodyid[int(g)])).name
                     .startswith(("r_", "zarm_r")) for g in (contact.geom1, contact.geom2)]
            if sum(right) == 1:
                return False
    orientation = (Rotation.from_euler("y", 15, degrees=True)
        * Rotation.from_quat(ik.eef_quaternion_world_with_yaw(
            ik.READY, math.radians(-15)))).as_quat()
    lower = ik.lower.copy()
    lower[1] = -1.6
    targets = []
    for position in positions:
        targets.extend(((position[0] - .015, position[1] - .010, position[2] - .010),
                        (position[0] - .015, position[1] - .010, .85)))
    targets.extend((target[0] + dx, target[1] + dy, .81)
                   for dx, dy in DROP_CENTER_OFFSETS)
    for point in targets:
        q = ik.solve(point, orientation, ik.READY,
                     point_in_r7=ik.CLOSED_GRASP_CENTER_IN_R7,
                     joint_bounds=(lower, ik.upper))
        if np.linalg.norm(np.array(ik.closed_grasp_center_world(q)) - point) > .003:
            return False
    return True

class Task1V2RandomizationPlanner:
    def plan(self, seed):
        cfg = json.loads((PACKAGE_DIR / "config/task1_randomization.json").read_text())
        rng = random.Random(int(seed))
        base = tuple(cfg["initial_base"])
        # Draw the target independently of reachability rejection, retaining
        # the requested full +/-10 cm target distribution.
        target = (.3075, -.528 + rng.uniform(*cfg["target_y_offset_range_m"]), .50)
        for _ in range(100):
            positions = tuple((rng.uniform(*cfg["object_x_range"]),
                               rng.uniform(*yr), cfg["object_z"])
                              for yr in cfg["object_y_ranges"])
            if any(math.dist(a[:2], b[:2]) < cfg["minimum_object_separation_m"]
                   for i, a in enumerate(positions) for b in positions[i+1:]):
                continue
            with contextlib.redirect_stdout(io.StringIO()):
                reachable = _reachable(positions, target, base)
            if not reachable:
                continue
            cylinders = tuple(CylinderSpawn(p, rng.uniform(-math.pi, math.pi))
                              for p in positions)
            return RandomizationPlan(int(seed), base, base, cylinders, target)
        raise RuntimeError("Task 1 cannot generate a reachable scene for seed {}".format(seed))
