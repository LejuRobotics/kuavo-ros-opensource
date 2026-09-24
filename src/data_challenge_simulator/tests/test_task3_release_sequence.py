"""Regression checks for Task 3's post-release control flow."""

import json
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
GRASP_SCRIPT = PACKAGE / "examples/task3_grasp.py"
GRASP_CONFIG = PACKAGE / "config/task3_grasp.json"
MUJOCO_NODE = PACKAGE.parent / "mujoco/src/mujoco_node.cc"
SCENE = (
    PACKAGE / "models/biped_s400062/xml/task3.xml"
)


def test_release_does_not_gate_progress_on_ring_height():
    source = GRASP_SCRIPT.read_text(encoding="utf-8")
    release = source[source.index("lambda: not feedback.held"):]

    assert "wait_for_ring_height" not in source
    assert "position[2]" not in release
    assert "release acknowledged" in release


def test_grasp_config_has_no_post_release_height_gate():
    config = json.loads(GRASP_CONFIG.read_text(encoding="utf-8"))

    assert "destination_table_height_m" not in config
    assert "settle_height_tolerance_m" not in config
    assert "settle_timeout_s" not in config


def test_collision_free_table_settle_has_configurable_fast_upright_speed():
    source = MUJOCO_NODE.read_text(encoding="utf-8")
    scene = SCENE.read_text(encoding="utf-8")

    assert 'numericScalarOrDefault(\n      "task3_table_settle_angular_speed"' in source
    assert "Eigen::Quaterniond::FromTwoVectors" in source
    assert "maximum_step / angle" in source
    assert "destination_table_height - internalObjectLowestWorldZ(state)" in source
    assert '<numeric name="task3_table_settle_angular_speed" data="6.283185307"/>' in scene
