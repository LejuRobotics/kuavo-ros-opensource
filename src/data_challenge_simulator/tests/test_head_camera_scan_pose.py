"""Keep all three formal entries on the same physical head-camera pose."""

import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE.parent
ROBOT_XMLS = (
    PACKAGE / "models/biped_s400062/xml/biped_s400062_icra.xml",
    WORKSPACE_SRC / "kuavo_assets/models/biped_s400062/xml/"
    "biped_s400062_icra.xml",
)
TASK_ENTRIES = (
    PACKAGE / "examples/task1.py",
    PACKAGE / "examples/task2.py",
    PACKAGE / "examples/task3.py",
)


def _literal_assignments(path):
    values = {}
    for node in ast.parse(path.read_text(encoding="utf-8")).body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)):
            try:
                values[node.targets[0].id] = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                if (isinstance(node.value, ast.Call)
                        and isinstance(node.value.func, ast.Attribute)
                        and node.value.func.attr == "radians"):
                    values[node.targets[0].id] = math.radians(
                        ast.literal_eval(node.value.args[0]))
    return values


def test_head_cameras_have_the_same_20_degree_mount_in_both_robot_models():
    for path in ROBOT_XMLS:
        camera = ET.parse(path).find(".//camera[@name='cam_h']")
        assert camera is not None, path
        assert camera.attrib["pos"] == "0.14 0 0.047", path
        assert camera.attrib["xyaxes"] == \
            "0 -1 0 0.342020 0 0.939693", path
        assert camera.attrib["fovy"] == "90", path


def test_formal_task_entries_command_the_same_20_degree_head_pose():
    for path in TASK_ENTRIES:
        source = path.read_text(encoding="utf-8")
        values = _literal_assignments(path)
        assert math.isclose(
            values["HEAD_SCAN_PITCH_RAD"], math.radians(20.0),
            abs_tol=1e-12), path
        assert "robot.control_head(yaw=0.0, pitch=HEAD_SCAN_PITCH_RAD)" \
            in source, path


def test_model_entry_uses_the_matching_upstream_head_pose():
    entry = PACKAGE / "scripts/model_entry.py"
    values = _literal_assignments(entry)
    assert math.isclose(
        values["UPSTREAM_HEAD_INIT"][1], math.radians(12.0),
        abs_tol=5e-4)
