"""Regression checks for Task 1's collision-free post-release settle."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
MUJOCO_NODE = PACKAGE.parent / "mujoco/src/mujoco_node.cc"
SCENE = PACKAGE / "models/biped_s400062/xml/task1.xml"


def test_task1_release_disables_object_collision_for_vertical_fall():
    source = MUJOCO_NODE.read_text(encoding="utf-8")

    assert 'm->geom_contype[geom_id] = 0;' in source
    assert 'm->geom_conaffinity[geom_id] = 0;' in source
    assert "state.release_xy_world[0]" in source
    assert "legacyObjectLowestWorldZ" in source
    assert "legacyObjectOverTargetFloor" in source


def test_task1_latch_disables_collision_and_release_uses_open_command():
    source = MUJOCO_NODE.read_text(encoding="utf-8")
    latch = source[source.index(
        "if (grasp_latch_complete || legacy_contact_complete)"):]
    latch_end = latch.index("continue;")
    latch = latch[:latch_end]

    held = source[source.index("if (state.held)"):]
    held_end = held.index("const Eigen::Matrix3d hand_rotation")
    held = held[:held_end]

    assert "state.held = true;" in latch
    assert latch.index("state.held = true;") < latch.index(
        "m->geom_contype[geom_id] = 0;")
    assert "m->geom_conaffinity[geom_id] = 0;" in latch
    assert "if (opening_command)" in held
    assert "contacting_fingertips.size() < 2" not in held


def test_task1_target_floor_settle_has_configurable_fast_upright_speed():
    source = MUJOCO_NODE.read_text(encoding="utf-8")
    scene = SCENE.read_text(encoding="utf-8")

    assert 'numericScalarOrDefault("task1_target_settle_angular_speed"' in source
    assert "settleLegacyObjectOnTargetFloor" in source
    assert "maximum_step / angle" in source
    assert "legacyTargetFloorHeight(state) - legacyObjectLowestWorldZ(state)" in source
    assert '<numeric name="task1_target_settle_angular_speed" data="6.283185307"/>' in scene


def test_source_bin_walls_block_task1_cylinders():
    root = ET.parse(SCENE).getroot()
    geoms = {geom.get("name"): geom for geom in root.findall(".//geom")}

    for suffix in ("front", "back", "left", "right"):
        wall = geoms[f"source_bin_{suffix}_wall"]
        assert int(wall.get("conaffinity")) & 4

    for index in range(1, 4):
        cylinder = geoms[f"block_{index}_geom"]
        assert int(cylinder.get("contype")) & 4
