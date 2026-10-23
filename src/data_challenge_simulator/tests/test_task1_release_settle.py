"""Regression checks for Task 1's physical grasp and release."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
MUJOCO_NODE = PACKAGE.parent / "mujoco/src/mujoco_node.cc"
SCENE = PACKAGE / "models/biped_s400062/xml/task1.xml"


def test_grasp_feedback_cannot_move_objects_or_disable_collisions():
    source = MUJOCO_NODE.read_text(encoding="utf-8")
    body = source.split("bool updateInternalContactFollowers()", 1)[1].split(
        "bool updateContactFollowers", 1)[0]
    assert "mj_contactForce" in body
    assert "destination_supported" in source
    for mutation in ("d->qpos[", "d->qvel[", "m->geom_contype[",
                     "m->geom_conaffinity[", "d->qfrc_applied["):
        assert mutation not in body
    assert "settleLegacyObjectOnTargetFloor" not in source
    assert "legacyObjectLowestWorldZ" not in source


def test_cylinders_collide_with_static_surfaces_and_hand_shells():
    root = ET.parse(SCENE).getroot()
    geoms = {geom.get("name"): geom for geom in root.findall(".//geom")}
    for index in range(1, 4):
        cylinder = geoms[f"block_{index}_geom"]
        assert int(cylinder.get("conaffinity")) & 1
        assert int(cylinder.get("conaffinity")) & 8
    assert int(geoms["target_bin_floor"].get("conaffinity")) & 4


def test_source_bin_walls_block_task1_cylinders():
    root = ET.parse(SCENE).getroot()
    geoms = {geom.get("name"): geom for geom in root.findall(".//geom")}

    for suffix in ("front", "back", "left", "right"):
        wall = geoms[f"source_bin_{suffix}_wall"]
        assert int(wall.get("conaffinity")) & 4

    for index in range(1, 4):
        cylinder = geoms[f"block_{index}_geom"]
        assert int(cylinder.get("contype")) & 4


def test_partial_release_uses_the_public_gripper_command_units():
    import ast
    source = ast.parse((PACKAGE / "examples/task1.py").read_text())
    release = next(node for node in ast.walk(source)
                   if isinstance(node, ast.With)
                   and any(isinstance(child, ast.Constant)
                           and child.value == "{}.gripper_release"
                           for child in ast.walk(node.items[0].context_expr)))
    command = release.body[0].value.args[0]
    value = eval(compile(ast.Expression(command), "<release>", "eval"),
                 {"__builtins__": {}})
    assert abs(value / 255.0 - 0.65) < 1e-9


def test_release_opens_hand_without_waiting_for_rest_or_support():
    import ast
    from contextlib import nullcontext
    from types import SimpleNamespace
    tree = ast.parse((PACKAGE / "examples/task1.py").read_text())
    release = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                   and node.name == "release_cylinder")
    # No ROS feedback or time.sleep is available: release must finish once
    # the hand opens, leaving target-region validation to the caller.
    namespace = {}
    exec(compile(ast.Module(body=[release], type_ignores=[]), "<release>", "exec"), namespace)
    commands = []
    gripper = SimpleNamespace(control_right_gripper=lambda command, duration:
                              commands.append((command, duration)))
    timing = SimpleNamespace(segment=lambda label: nullcontext())
    namespace["release_cylinder"](gripper, "cylinder_1", timing)
    assert commands == [(0.65 * 255, 0.5), (0, 0.3)]
