"""Static contracts for the explicitly approved Task 2 speed changes."""

import json
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
INITIALIZATION_CONFIG = PACKAGE / "config/task2_initialization.json"
PICK_CONFIG = PACKAGE / "config/task2_pick.json"
TASK = PACKAGE / "examples/task2.py"
PICK = PACKAGE / "examples/task2_pick.py"
MODEL_ENTRY = PACKAGE / "scripts/model_entry.py"


def load(path):
    return json.loads(path.read_text())


def test_task2_initialization_finishes_near_the_above_ik_height_quickly():
    config = load(INITIALIZATION_CONFIG)
    assert config["shoulder_pitch_rad"] == -1.0
    assert config["lift_trajectory_points"] == 80
    assert config["ready_trajectory_points"] == 60
    assert config["open_loop_linear_speed_mps"] == 0.16


def test_task2_uses_reduced_ik_budget_and_relaxed_tolerance():
    ik = load(PICK_CONFIG)["ik"]
    assert ik["maximum_function_evaluations"] == 120
    assert ik["maximum_position_error_m"] == 0.03
    assert ik["solver_tolerance"] == 1e-5


def test_task2_entries_read_the_faster_chassis_config():
    config = load(PICK_CONFIG)["chassis"]
    assert config == {
        "linear_speed_mps": 0.20,
        "angular_speed_radps": 0.60,
        "minimum_linear_speed_mps": 0.08,
        "minimum_angular_speed_radps": 0.08,
    }
    task_source = TASK.read_text()
    assert 'pick_config["chassis"]["linear_speed_mps"]' in task_source
    assert 'pick_config["chassis"]["angular_speed_radps"]' in task_source
    assert 'initialization_config["open_loop_linear_speed_mps"]' in task_source
    model_source = MODEL_ENTRY.read_text()
    assert "linear_speed=None" in model_source
    assert "        config)" in model_source


def test_final_release_ends_at_conveyor_contact_without_return_route():
    source = PICK.read_text()
    release = source[source.index("if is_first_box:", source.index(
        '"Task2 placing %s on destination conveyor"')):
        source.index('rospy.loginfo("Task2 returning to the source start")')]
    assert release.count('"post_release_ready"') == 1
    assert release.index('"post_release_ready"') < release.index("else:")
    assert "final box contacted conveyor" in release
    contact_wait = source.index("wait_for_conveyor_contact(", source.index(
        '"Task2 placing %s on destination conveyor"'))
    final_branch = source.index("if not is_first_box:", contact_wait)
    assert contact_wait < final_branch
    assert release.index("if not is_first_box:") < release.index(
        "\n                return\n")
    return_route = source[source.index(
        'rospy.loginfo("Task2 returning to the source start")'):
        source.index('"TASK2 BOX PLACED AND RETURNED: box=%s"')]
    assert "translate_relative(" not in return_route


def test_task2_arm_execution_uses_shorter_trajectories():
    config = load(PICK_CONFIG)
    grasp = config["grasp"]
    assert grasp["above_trajectory_points"] == 50
    assert grasp["descent_trajectory_points"] == 60
    assert grasp["lift_trajectory_points"] == 80
    assert grasp["gripper_duration_s"] == 0.8
    assert grasp["settle_time_s"] == 0.6
    transport = config["transport"]
    assert transport["release_ready_trajectory_points"] == 60
    assert transport["place_contact_max_z_m"] == 0.605
    assert transport["place_contact_timeout_s"] == 2.0
