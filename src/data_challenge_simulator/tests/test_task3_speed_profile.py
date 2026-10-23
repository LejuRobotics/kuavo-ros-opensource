import json
import math
from pathlib import Path

import numpy as np
import mujoco

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


def test_grasp_search_is_bounded_and_inserts_within_ring_height():
    grasp = _config("task3_grasp.json")
    planner = Task3SecondIKPlanner(SCENE)
    assert 0 < grasp["descent_distance_m"] < planner.ring_height
    assert grasp["descent_trajectory_points"] > 1
    assert grasp["lift_trajectory_points"] > 1
    assert grasp["descent_settle_seconds"] >= 0
    assert grasp["loaded_base_linear_speed_mps"] > 0
    full_search_seconds = (
        math.ceil(1.0 / grasp["finger_expansion_step"])
        * grasp["finger_command_period_s"])
    assert full_search_seconds < grasp["latch_timeout_s"]


def test_compact_hand_has_clearance_inside_ring_at_grasp_depth():
    planner = Task3SecondIKPlanner(SCENE)
    ring = np.asarray((0.65, -0.275, 0.65))
    position, rotation = planner.reference_pose(ring)
    position[2] -= _config("task3_grasp.json")["descent_distance_m"]
    seed = np.asarray([0.0] * 7 + [-0.6, -0.36, 1.0, -0.8, 0.77, -0.33, 0.36])
    result = planner.solve_target((0, 0, 0), ring, seed, position, rotation,
                                  validate_clearance=False)
    assert result is not None
    planner._set_configuration((0, 0, 0), result.right_joints, ring)
    assert not planner._ring_contacts()
    tips = planner._tip_geometry() - ring
    assert np.all(tips[:, 2] > 0)
    assert np.all(tips[:, 2] < planner.ring_height)


def test_fast_pose_matches_full_mujoco_forward():
    planner = Task3SecondIKPlanner(SCENE)
    for base, joints in [((0, 0, 0), [-.6, -.36, 1, -.8, .77, -.33, .36]),
                         ((.1, .3, .2), [-.5, -.6, .8, -1.5, .6, .3, .6])]:
        position, rotation = planner.ik._pose(base, np.asarray(joints))
        mujoco.mj_forward(planner.model, planner.data)
        np.testing.assert_allclose(position, planner.data.xpos[planner.ik.hand_body], atol=1e-12)
        np.testing.assert_allclose(rotation.as_matrix(),
                                   planner.data.xmat[planner.ik.hand_body].reshape(3, 3), atol=1e-12)


def test_placement_slots_fit_table_and_fill_far_to_near():
    slots = np.asarray(_config("task3_randomization.json")["destination_slots"])
    planner = Task3SecondIKPlanner(SCENE)
    mujoco.mj_forward(planner.model, planner.data)
    top = mujoco.mj_name2id(planner.model, mujoco.mjtObj.mjOBJ_GEOM,
                          "task3_destination_table_top")
    center = planner.data.geom_xpos[top, :2]
    half_size = planner.model.geom_size[top, :2]
    radius = _config("task3_transfer.json")["ring_outer_radius_m"]
    assert np.all(np.abs(slots - center) + radius < half_size)
    assert np.all(np.diff(slots[:, 1]) < -2 * radius)
