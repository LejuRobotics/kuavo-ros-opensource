"""Deterministic Task 3 sampling shared by task, collect and model."""
from dataclasses import dataclass
from functools import lru_cache
import json
import math
from pathlib import Path
import random

PACKAGE_DIR = Path(__file__).resolve().parents[1]
RING_NAMES = ("task3_hollow_cylinder", "task3_hollow_cylinder_2", "task3_hollow_cylinder_3")
MIN_RING_SEPARATION_M = .160
MIN_DOCKED_JOINT_MARGIN_RAD = .250

@dataclass(frozen=True)
class Task3RingSpawn:
    name: str
    position: tuple
    docking_base: tuple
    destination_xy: tuple

@dataclass(frozen=True)
class Task3RandomizationPlan:
    seed: int
    initial_base: tuple
    rings: tuple
    destination_position: tuple

    @property
    def catalogue_seed(self):
        return self.seed

@lru_cache(maxsize=1)
def _docking_model():
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(
        PACKAGE_DIR / "models/biped_s400062/xml/task3.xml"))
    # Leave clearance for the 3 cm chassis stopping tolerance.
    for gid in range(model.ngeom):
        name = model.geom(gid).name
        if name.startswith(("task3_source_", "task3_destination_")):
            model.geom_size[gid, :2] += .035
    return model


def docking_is_clear(base, destination_position):
    """Reject wheel/base collisions at a proposed source pickup stop."""
    import mujoco
    model = _docking_model()
    model.body_pos[model.body("task3_destination_table").id] = destination_position
    data = mujoco.MjData(model)
    data.qpos[:2] = base[:2]
    for yaw in (0.0, math.pi / 2):
        for wheel in ("LF", "RF", "LB", "RB"):
            data.qpos[model.jnt_qposadr[model.joint(wheel+"_wheel_yaw_joint").id]] = yaw
        mujoco.mj_forward(model, data)
        for contact in data.contact:
            names = [model.body(int(model.geom_bodyid[g])).name
                     for g in (contact.geom1, contact.geom2)]
            if (any(n in ("task3_source_table", "task3_destination_table") for n in names)
                    and any(n == "base_link" or "wheel" in n for n in names)):
                return False
    return True


class Task3RandomizationPlanner:
    def plan(self, seed):
        cfg = json.loads((PACKAGE_DIR / "config/task3_randomization.json").read_text())
        rng = random.Random(int(seed))
        dy = rng.uniform(*cfg["destination_y_offset_range_m"])
        for _ in range(1000):
            positions = tuple((rng.uniform(*cfg["source_x_range"]),
                               rng.uniform(*cfg["source_y_range"]),
                               cfg["source_z"]) for _ in RING_NAMES)
            if any(math.dist(a[:2], b[:2]) < cfg["minimum_ring_separation_m"]
                   for i, a in enumerate(positions) for b in positions[i+1:]):
                continue
            preferred = cfg["preferred_ring_in_base_xy"]
            rings = tuple(Task3RingSpawn(name, p,
                (p[0]-preferred[0], p[1]-preferred[1], 0.0),
                (slot[0], slot[1]+dy))
                for name, p, slot in zip(RING_NAMES, positions, cfg["destination_slots"]))
            if not all(docking_is_clear(r.docking_base, (.62, .85+dy, 0.0))
                       for r in rings):
                continue
            return Task3RandomizationPlan(int(seed), tuple(cfg["initial_base"]),
                                          rings, (.62, .85+dy, 0.0))
        raise RuntimeError("Task 3 cannot generate separated rings for seed {}".format(seed))
