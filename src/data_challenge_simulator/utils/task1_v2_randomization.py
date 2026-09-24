"""Load precomputed Scene 1 V2 layouts without runtime feasibility search."""

from dataclasses import dataclass
import json
import os
from pathlib import Path


OBJECT_COUNT = 3
MIN_OBJECT_TRANSVERSE_SEPARATION = 0.007
DROP_CENTER_OFFSETS = (
    (-0.060, -0.012),
    (0.000, 0.022),
    (0.060, -0.012),
)
DEFAULT_LAYOUT_FILE = (
    Path(__file__).resolve().parents[1]
    / "config/task1_v2_layout_seeds.json"
)
LAYOUT_FILE_ENV = "TASK1_V2_LAYOUT_FILE"


@dataclass(frozen=True)
class CylinderSpawn:
    position: tuple
    yaw: float


@dataclass(frozen=True)
class RandomizationPlan:
    seed: int
    catalogue_seed: int
    initial_base: tuple
    task_base: tuple
    cylinders: tuple

    @property
    def requires_base_motion(self):
        return any(
            abs(start - target) > 1e-6
            for start, target in zip(self.initial_base, self.task_base))


class Task1V2RandomizationPlanner:
    """Select a saved layout; never run IK or collision checks at runtime."""

    def __init__(self, layout_file=None):
        if layout_file is None:
            layout_file = os.environ.get(LAYOUT_FILE_ENV, DEFAULT_LAYOUT_FILE)
        self.layout_file = Path(layout_file)
        with self.layout_file.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        self.layouts = tuple(document["layouts"])
        if not self.layouts:
            raise ValueError("Task 1 layout seed file contains no layouts")
        for layout in self.layouts:
            if len(layout["cylinders"]) != OBJECT_COUNT:
                raise ValueError(
                    "Task 1 layout {} must contain three cylinders".format(
                        layout.get("seed")))

    def plan(self, seed):
        requested_seed = int(seed)
        index = (requested_seed - 1) % len(self.layouts)
        layout = self.layouts[index]
        return RandomizationPlan(
            seed=requested_seed,
            catalogue_seed=int(layout["seed"]),
            initial_base=tuple(layout["initial_base"]),
            task_base=tuple(layout["task_base"]),
            cylinders=tuple(
                CylinderSpawn(
                    position=tuple(cylinder["position"]),
                    yaw=float(cylinder["yaw"]),
                )
                for cylinder in layout["cylinders"]
            ),
        )
