"""Load precomputed Task 2 layouts without runtime feasibility search."""

from dataclasses import dataclass
import json
import os
from pathlib import Path


BOX_NAMES = ("box_1", "box_2")
DEFAULT_LAYOUT_FILE = (
    Path(__file__).resolve().parents[1]
    / "config/task2_layout_seeds.json"
)
LAYOUT_FILE_ENV = "TASK2_LAYOUT_FILE"


@dataclass(frozen=True)
class Task2BoxSpawn:
    name: str
    position: tuple
    yaw: float
    docking_base: tuple


@dataclass(frozen=True)
class Task2RandomizationPlan:
    seed: int
    catalogue_seed: int
    initial_base: tuple
    boxes: tuple


class Task2RandomizationPlanner:
    """Select a saved layout; never sample or run IK at runtime."""

    def __init__(self, layout_file=None):
        if layout_file is None:
            layout_file = os.environ.get(LAYOUT_FILE_ENV, DEFAULT_LAYOUT_FILE)
        self.layout_file = Path(layout_file)
        with self.layout_file.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        self.layouts = tuple(document["layouts"])
        if not self.layouts:
            raise ValueError("Task 2 layout seed file contains no layouts")
        for layout in self.layouts:
            names = tuple(box["name"] for box in layout["boxes"])
            if names != BOX_NAMES:
                raise ValueError(
                    "Task 2 layout {} must contain the two ordered boxes"
                    .format(layout.get("seed")))

    def plan(self, seed):
        requested_seed = int(seed)
        layout = self.layouts[(requested_seed - 1) % len(self.layouts)]
        return Task2RandomizationPlan(
            seed=requested_seed,
            catalogue_seed=int(layout["seed"]),
            initial_base=tuple(layout["initial_base"]),
            boxes=tuple(
                Task2BoxSpawn(
                    name=box["name"],
                    position=tuple(box["position"]),
                    yaw=float(box["yaw"]),
                    docking_base=tuple(box["docking_base"]),
                )
                for box in layout["boxes"]
            ),
        )
