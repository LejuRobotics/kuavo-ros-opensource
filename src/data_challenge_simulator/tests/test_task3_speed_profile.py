import json
import math
from pathlib import Path

import numpy as np

from utils.task3_second_ik import Task3SecondIKPlanner


PACKAGE = Path(__file__).resolve().parents[1]
SCENE = PACKAGE / "models/biped_s400062/xml/task3.xml"


def _config(name):
    return json.loads((PACKAGE / "config" / name).read_text())


def test_ready_pose_is_just_above_the_common_first_ik_endpoint():
    initialization = _config("task3_initialization.json")
    ready_right = np.asarray(initialization["right_arm_ready_rad"])
    planner = Task3SecondIKPlanner(SCENE)
    result = planner.solve_feedforward(
        base_pose=(-0.1270768596, 0.0006738904, 0.0),
        ring_position=(0.5229231404, -0.2743261096, 0.65),
        measured_arm=np.concatenate((np.zeros(7), ready_right)),
    )

    assert result is not None
    ready_position, _ = planner.ik._pose((0.0, 0.0, 0.0), ready_right)
    endpoint_position = np.asarray(result.hand_position_m) - np.asarray(
        (-0.1270768596, 0.0006738904, 0.0))
    assert 0.025 < ready_position[2] - endpoint_position[2] < 0.040
    assert np.max(np.abs(ready_right - result.right_joints)) < 0.080


def test_task3_fast_profile_keeps_insertion_timing_unchanged():
    initialization = _config("task3_initialization.json")
    transfer = _config("task3_transfer.json")
    grasp = _config("task3_grasp.json")

    assert initialization["ready_trajectory_points"] == 80
    assert transfer["approach_trajectory_points"] == 60
    assert transfer["approach_settle_seconds"] == 0.3
    assert transfer["hand_motion_duration_s"] == 0.25
    assert grasp["finger_expansion_step"] == 0.01
    assert grasp["finger_command_period_s"] == 0.01
    assert grasp["lift_trajectory_points"] == 100
    assert grasp["lift_settle_seconds"] == 0.3
    assert grasp["loaded_base_linear_speed_mps"] == 0.15
    assert grasp["hand_motion_duration_s"] == 0.25

    # The contact-sensitive 27 mm insertion retains its accepted trajectory.
    assert grasp["descent_distance_m"] == 0.027
    assert grasp["descent_trajectory_points"] == 80
    assert grasp["descent_settle_seconds"] == 0.8

    full_search_seconds = (
        math.ceil(1.0 / grasp["finger_expansion_step"])
        * grasp["finger_command_period_s"])
    assert full_search_seconds <= 1.0
