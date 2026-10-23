"""One seeded Task 2 scene generator shared by task, collect and model."""

from contextlib import contextmanager
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import random
import tempfile
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = PACKAGE_DIR / "models/biped_s400062/xml/task2.xml"
CONFIG_FILE = PACKAGE_DIR / "config/task2_randomization.json"
BOX_NAMES = ("box_1", "box_2")


def settings():
    return json.loads(CONFIG_FILE.read_text())


def seed_for_round(round_id):
    if int(round_id) < 1:
        raise ValueError("Round must start at 1")
    return int(settings()["seed_base"]) + int(round_id)


def active_scene(task_id=2):
    default = DEFAULT_SCENE.with_name("task{}.xml".format(task_id))
    return Path(os.environ.get("TASK{}_SCENE_FILE".format(task_id), str(default)))


def vector(element, attribute):
    return tuple(float(x) for x in element.get(attribute, "0 0 0").split())


@dataclass(frozen=True)
class Task2BoxSpawn:
    name: str
    position: tuple
    yaw: float
    docking_base: tuple


@dataclass(frozen=True)
class Task2RandomizationPlan:
    seed: int
    initial_base: tuple
    boxes: tuple
    conveyor_position: tuple
    conveyor_x_offset: float
    conveyor_bounds: tuple
    minimum_box_gap: float
    lighting: dict


class Task2RandomizationPlanner:
    """Sample within the actual tabletop; never consult the legacy catalogue."""

    def plan(self, seed):
        cfg = settings()
        rng = random.Random(int(seed))
        root = ET.parse(DEFAULT_SCENE).getroot()
        table = root.find(".//body[@name='work_table']")
        top = table.find("geom[@name='table_top']")
        center = tuple(a + b for a, b in zip(vector(table, "pos"), vector(top, "pos")))
        size = vector(top, "size")
        half_boxes = [vector(root.find(".//geom[@name='%s_floor']" % name), "size")
                      for name in BOX_NAMES]
        if half_boxes[0] != half_boxes[1]:
            raise ValueError("Task 2 generation expects equal box footprints")
        hx, hy, _ = half_boxes[0]
        side_margin = float(cfg["box_side_margin_m"])
        gap = float(cfg["minimum_box_gap_m"])
        inset = float(cfg["box_front_inset_m"])
        low, high = center[1] - size[1] + hy + side_margin, center[1] + size[1] - hy - side_margin
        slack = high - low - 2 * hy - gap
        if min(side_margin, inset) < 0 or gap <= 0 or slack < 0 or 2 * hx + inset > 2 * size[0]:
            raise ValueError("Task 2 boxes and grasp clearance do not fit the tabletop")
        bounds = cfg["conveyor_x_offset_range_m"]
        if len(bounds) != 2 or bounds[0] > bounds[1]:
            raise ValueError("Invalid conveyor X bounds")
        dx = rng.uniform(*bounds)
        shifts = sorted((rng.uniform(0, slack), rng.uniform(0, slack)))
        ys = (low + shifts[0], low + 2 * hy + gap + shifts[1])
        x, z = center[0] - size[0] + hx + inset, center[2] + size[2]
        standoff = json.loads((PACKAGE_DIR / "config/task2_pick.json").read_text())["docking"]["front_standoff_m"]
        boxes = tuple(Task2BoxSpawn(name, (x, y, z), 0.0, (x - standoff, y, 0.0))
                      for name, y in zip(BOX_NAMES, ys))
        conveyor = root.find(".//body[@name='destination_conveyor']")
        pos = vector(conveyor, "pos")
        belt = conveyor.find("geom[@name='destination_conveyor_belt']")
        bp, bs = vector(belt, "pos"), vector(belt, "size")
        # The existing belt stays at +90 degrees yaw; only world X translates.
        position = (pos[0] + dx, pos[1], pos[2])
        cx, cy, cz = position[0] - bp[1], position[1] + bp[0], position[2] + bp[2]
        belt_bounds = (cx - bs[1], cx + bs[1], cy - bs[0], cy + bs[0], cz + bs[2])
        # A separate deterministic stream keeps adding light draws from changing
        # the physical layout for an existing seed.
        lighting = lighting_for_seed(seed)
        return Task2RandomizationPlan(int(seed), tuple(cfg["initial_base"]), boxes,
                                     position, dx, belt_bounds, gap, lighting)


def lighting_for_seed(seed):
    cfg = settings()["lighting"]
    rng = random.Random("task2-lighting:{}".format(int(seed)))
    color = rng.choice(tuple(cfg["headlight_colors"]))
    return {"headlight_color": color,
            "headlight_diffuse": tuple(cfg["headlight_colors"][color]),
            "headlight_ambient": tuple(cfg["headlight_ambient"])}


@contextmanager
def task_scene(task_id, seed):
    """Keep a per-round XML beside its source so relative assets stay valid."""
    default_scene = DEFAULT_SCENE.with_name("task{}.xml".format(task_id))
    tree = ET.parse(default_scene)
    root = tree.getroot()
    if task_id == 1:
        from utils.task1_v2_randomization import Task1V2RandomizationPlanner
        plan = Task1V2RandomizationPlanner().plan(seed)
        root.find(".//body[@name='target_bin']").set("pos", " ".join(map(str, plan.target_position)))
        for index, spawn in enumerate(plan.cylinders, 1):
            body = root.find(".//body[@name='cylinder_{}']".format(index))
            body.set("pos", " ".join(map(str, spawn.position)))
            body.set("quat", "{} 0 0 {}".format(math.cos(spawn.yaw/2), math.sin(spawn.yaw/2)))
    elif task_id == 3:
        from utils.task3_randomization import Task3RandomizationPlanner
        plan = Task3RandomizationPlanner().plan(seed)
        root.find(".//body[@name='task3_destination_table']").set(
            "pos", " ".join(map(str, plan.destination_position)))
        for ring in plan.rings:
            root.find(".//body[@name='{}']".format(ring.name)).set("pos", " ".join(map(str, ring.position)))
    else:
        plan = Task2RandomizationPlanner().plan(seed)
    lighting = lighting_for_seed(seed)
    if task_id == 2:
        root.find(".//body[@name='destination_conveyor']").set(
            "pos", " ".join(map(str, plan.conveyor_position)))
        for box in plan.boxes:
            root.find(".//body[@name='%s']" % box.name).set("pos", " ".join(map(str, box.position)))
    # Keep the ICRA scene light fixed. Only the camera headlight color varies,
    # using bright pastels and neutral ambient fill to preserve box visibility.
    headlight = root.find("visual/headlight")
    headlight.attrib.update(active="1", specular="0 0 0")
    headlight.set("diffuse", " ".join(map(str, lighting["headlight_diffuse"])))
    headlight.set("ambient", " ".join(map(str, lighting["headlight_ambient"])))
    for light in root.findall(".//worldbody//light"):
        light.attrib.update(pos="0 0 7", dir="0 0 -1", directional="true",
                            diffuse="0.7 0.7 0.7", ambient="0 0 0", specular="0.3 0.3 0.3")
    # Preserve these generated lights instead of applying the old C++ ranges.
    custom = root.find("custom")
    if custom is None:
        custom = ET.SubElement(root, "custom")
    ET.SubElement(custom, "numeric", name="task2_scene_seed" if task_id == 2 else "task_lighting_seed", data=str(int(seed)))
    descriptor, filename = tempfile.mkstemp(prefix=".task{}-round-".format(task_id), suffix=".xml", dir=default_scene.parent)
    scene_env = "TASK{}_SCENE_FILE".format(task_id)
    previous = os.environ.get(scene_env)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            os.fchmod(stream.fileno(), 0o644)
            tree.write(stream, encoding="utf-8", xml_declaration=True)
        os.environ[scene_env] = filename
        print("Task{} generated scene: ".format(task_id) + json.dumps(
            dict(asdict(plan), lighting=lighting), sort_keys=True), flush=True)
        yield filename
    finally:
        if previous is None:
            os.environ.pop(scene_env, None)
        else:
            os.environ[scene_env] = previous
        Path(filename).unlink(missing_ok=True)
