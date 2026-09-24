"""Keep S400062 robot parameters and canonical task entries aligned."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE.parent
REPOSITORY = WORKSPACE_SRC.parent
ASSET_MODEL = WORKSPACE_SRC / "kuavo_assets/models/biped_s400062"
URDF = ASSET_MODEL / "urdf/biped_s400062.urdf"
MJCF_MODELS = (
    ASSET_MODEL / "xml/biped_s400062_icra.xml",
    PACKAGE / "models/biped_s400062/xml/biped_s400062_icra.xml",
)
FORMAL_SCENES = (
    PACKAGE / "models/biped_s400062/xml/task1.xml",
    PACKAGE / "models/biped_s400062/xml/task2.xml",
    PACKAGE / "models/biped_s400062/xml/task3.xml",
)


def _literal_assignment(path, name):
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name:
                    return ast.literal_eval(node.value)
    raise AssertionError(f"missing assignment {name} in {path}")


def _numbers(value):
    return tuple(float(item) for item in value.split())


def test_arm_fifth_joint_damping_matches_urdf():
    urdf_root = ET.parse(URDF).getroot()

    for side in ("l", "r"):
        name = f"zarm_{side}5_joint"
        urdf_joint = urdf_root.find(f"./joint[@name='{name}']")
        assert urdf_joint is not None
        expected = float(urdf_joint.find("dynamics").attrib["damping"])

        for model_path in MJCF_MODELS:
            joint = ET.parse(model_path).find(f".//joint[@name='{name}']")
            assert joint is not None, model_path
            assert float(joint.attrib["damping"]) == expected, model_path


def test_left_thumb_collision_position_is_shared():
    positions = []
    for model_path in MJCF_MODELS:
        geom = ET.parse(model_path).find(
            ".//geom[@name='l_thumb_fingertip_collision']")
        assert geom is not None, model_path
        positions.append(_numbers(geom.attrib["pos"]))

    assert positions == [(-0.010, 0.046, 0.014)] * len(MJCF_MODELS)


def test_right_fingertip_collision_sizes_are_shared():
    expected_sizes = {
        "r_thumb_fingertip_collision": (0.009, 0.008, 0.006),
        "r_index_fingertip_collision": (0.008, 0.008, 0.006),
    }

    for name, expected in expected_sizes.items():
        for model_path in MJCF_MODELS:
            geom = ET.parse(model_path).find(f".//geom[@name='{name}']")
            assert geom is not None, model_path
            assert _numbers(geom.attrib["size"]) == expected, model_path


def test_formal_task_scenes_share_the_canonical_robot_model():
    for scene_path in FORMAL_SCENES:
        include = ET.parse(scene_path).find("include")
        assert include is not None, scene_path
        assert include.attrib["file"] == "biped_s400062_icra.xml", scene_path

    legacy_xml = ASSET_MODEL / "xml"
    for scene_name in (
            "task1.xml", "task2.xml", "task3.xml",
            "scene_two_boxes.xml", "scene_three_internal_cylinder.xml"):
        assert not (legacy_xml / scene_name).exists()


def test_run_scene_task_collect_and_model_entries_use_canonical_names():
    helper = PACKAGE / "examples/helperfunc.py"
    model_entry = PACKAGE / "scripts/model_entry.py"
    task_scripts = {
        1: "task1.py",
        2: "task2.py",
        3: "task3.py",
    }
    simulator_launches = {
        1: "load_kuavo_mujoco_sim1.launch",
        2: "load_kuavo_mujoco_sim2.launch",
        3: "load_kuavo_mujoco_sim3.launch",
    }
    model_launches = {
        1: "load_kuavo_mujoco_model1.launch",
        2: "load_kuavo_mujoco_model2.launch",
        3: "load_kuavo_mujoco_model3.launch",
    }

    assert _literal_assignment(helper, "TASK_SCRIPTS") == task_scripts
    assert _literal_assignment(
        helper, "SIMULATOR_LAUNCH_FILES") == simulator_launches
    assert _literal_assignment(
        model_entry, "MODEL_LAUNCH_FILES") == model_launches

    shared_wrapper = (REPOSITORY / "run-scene1.sh").read_text()
    assert 'helper_args=(--task-id "${TASK_ID}"' in shared_wrapper
    assert 'run_model "${2:-anonymous}"' in shared_wrapper
    assert 'bash "${TASK_ID}" "${model_name}"' in shared_wrapper
    for task_id in (1, 2, 3):
        wrapper = REPOSITORY / f"run-scene{task_id}.sh"
        assert wrapper.exists()
        if task_id > 1:
            assert f"--internal-task-id {task_id}" in wrapper.read_text()

        assert (PACKAGE / "examples" / task_scripts[task_id]).exists()
        simulator_launch = PACKAGE / "launch" / simulator_launches[task_id]
        simulator_source = simulator_launch.read_text()
        assert f"/xml/task{task_id}.xml" in simulator_source

        model_launch = PACKAGE / "launch" / model_launches[task_id]
        model_source = model_launch.read_text()
        assert simulator_launches[task_id] in model_source
