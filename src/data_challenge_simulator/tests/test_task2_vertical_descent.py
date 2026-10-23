"""Check hand clearance and the actual Cartesian path between descent commands."""
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from utils.task2_pick_planner import Task2PickPlanner


PACKAGE = Path(__file__).resolve().parents[1]
SEED = [-0.802, 0.024, -0.694, -0.156, -0.910, 0.661, 0.284,
        -0.802, -0.024, 0.693, -0.156, 0.917, -0.661, 0.287]
BOXES = {"box_1": ((0.755, -0.42, 0.6), (0, 0, 0, 1)),
         "box_2": ((0.755, 0.12, 0.6), (0, 0, 0, 1))}


def configuration():
    pick = json.loads((PACKAGE / "config/task2_pick.json").read_text())
    init = json.loads((PACKAGE / "config/task2_initialization.json").read_text())
    return pick, init


@pytest.mark.parametrize("arm_seed", [SEED, [-1.0, 0, 0, 0, 0, 0, 0] * 2])
def test_ready_hands_clear_the_box_rim_and_align_with_the_grasp(arm_seed):
    pick, init = configuration()
    planner = Task2PickPlanner(ik_config=pick["ik"])
    base = (0.075, 0.12, 0)
    ready = planner.solve_palm_down_ready(
        base, arm_seed, pick["grasp"]["outward_angle_deg"],
        hand_position_base_m=init["ready_hand_position_base_m"])
    planner._set_configuration(base, BOXES, {"l": ready[:7], "r": ready[7:]}, 0)
    rim = max(
        planner.data.geom_xpos[g][2] +
        abs(planner.data.geom_xmat[g].reshape(3, 3)[2]) @ planner.model.geom_size[g]
        for g in planner.box_geom_ids["box_1"])
    for side, sign in (("l", 1), ("r", -1)):
        target = np.array(init["ready_hand_position_base_m"])
        target[1] *= sign
        target[:2] += base[:2]
        assert np.linalg.norm(
            planner.data.xpos[planner.hand_body_ids[side]] - target) < 0.005
        for geom in planner.hand_geom_ids[side]:
            if not (planner.model.geom_contype[geom] or
                    planner.model.geom_conaffinity[geom]):
                continue
            size = planner.model.geom_size[geom]
            rotation = planner.data.geom_xmat[geom].reshape(3, 3)
            kind = planner.model.geom_type[geom]
            if kind == mujoco.mjtGeom.mjGEOM_BOX:
                extent = abs(rotation[2]) @ size
            elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
                extent = size[0] + abs(rotation[2, 2]) * size[1]
            elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
                extent = size[0]
            elif kind == mujoco.mjtGeom.mjGEOM_MESH:
                mesh = planner.model.geom_dataid[geom]
                first = planner.model.mesh_vertadr[mesh]
                vertices = planner.model.mesh_vert[
                    first:first + planner.model.mesh_vertnum[mesh]]
                extent = -min(vertices @ rotation[2])
            else:
                raise AssertionError("unhandled hand collision geometry")
            assert planner.data.geom_xpos[geom][2] - extent >= rim + 0.05


@pytest.mark.parametrize("box", ["box_1", "box_2"])
def test_descent_has_no_backward_arc_or_second_raise(box):
    pick, init = configuration()
    planner = Task2PickPlanner(ik_config=pick["ik"])
    base = (0.075, BOXES[box][0][1], 0)
    ready = planner.solve_palm_down_ready(
        base, SEED, pick["grasp"]["outward_angle_deg"],
        hand_position_base_m=init["ready_hand_position_base_m"])
    plan = planner.plan(box, base, BOXES, ready, pick["grasp"])
    samples = []
    for start, end in zip(plan.descent_waypoints, plan.descent_waypoints[1:]):
        for alpha in np.linspace(0, 1, 6):
            q = (1 - alpha) * np.array(start) + alpha * np.array(end)
            planner._set_configuration(base, BOXES, {"l": q[:7], "r": q[7:]}, 0)
            samples.append([
                planner.data.xpos[planner.hand_body_ids[side]].copy()
                for side in ("l", "r")])
    samples = np.asarray(samples)
    assert np.max(np.linalg.norm(samples[:, :, :2] - samples[0, :, :2], axis=2)) < .003
    assert np.max(np.diff(samples[:, :, 2], axis=0)) < 1e-5
    assert np.allclose(
        samples[0, :, 2] - samples[-1, :, 2],
        pick["grasp"]["above_height_m"], atol=1e-4)
    assert np.allclose(plan.above_joints, plan.descent_waypoints[0])
    assert np.allclose(plan.contact_joints, plan.descent_waypoints[-1])
