"""Load precomputed Task 3 layouts without runtime feasibility search."""

from dataclasses import dataclass
import json
import os
from pathlib import Path


RING_NAMES = (
    "task3_hollow_cylinder",
    "task3_hollow_cylinder_2",
    "task3_hollow_cylinder_3",
)
MIN_RING_SEPARATION_M = 0.160
MIN_DOCKED_JOINT_MARGIN_RAD = 0.250
DEFAULT_LAYOUT_FILE = (
    Path(__file__).resolve().parents[1]
    / "config/task3_layout_seeds.json"
)
LAYOUT_FILE_ENV = "TASK3_LAYOUT_FILE"


@dataclass(frozen=True)
class Task3RingSpawn:
    name: str
    position: tuple
    docking_base: tuple
    destination_xy: tuple
    joint_margin_rad: float


@dataclass(frozen=True)
class Task3RandomizationPlan:
    seed: int
    catalogue_seed: int
    initial_base: tuple
    rings: tuple


class Task3RandomizationPlanner:
    """Select a saved layout; never run IK or sampling at runtime."""

    def __init__(self, layout_file=None):
        if layout_file is None:
            layout_file = os.environ.get(LAYOUT_FILE_ENV, DEFAULT_LAYOUT_FILE)
        self.layout_file = Path(layout_file)
        with self.layout_file.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        self.layouts = tuple(document["layouts"])
        if not self.layouts:
            raise ValueError("Task 3 layout seed file contains no layouts")
        for layout in self.layouts:
            names = tuple(ring["name"] for ring in layout["rings"])
            if names != RING_NAMES:
                raise ValueError(
                    "Task 3 layout {} must contain the three ordered rings"
                    .format(layout.get("seed")))

    def plan(self, seed):
        requested_seed = int(seed)
        layout = self.layouts[(requested_seed - 1) % len(self.layouts)]
        return Task3RandomizationPlan(
            seed=requested_seed,
            catalogue_seed=int(layout["seed"]),
            initial_base=tuple(layout["initial_base"]),
            rings=tuple(
                Task3RingSpawn(
                    name=ring["name"],
                    position=tuple(ring["position"]),
                    docking_base=tuple(ring["docking_base"]),
                    destination_xy=tuple(ring["destination_xy"]),
                    joint_margin_rad=float(ring["joint_margin_rad"]),
                )
                for ring in layout["rings"]
            ),
        )
