"""Check the current stationary Task1 preparation against real MJCF collisions."""

import ast
import math
from pathlib import Path

import mujoco
import numpy as np

from utils.scene1_v2_right_arm_ik import Scene1V2RightArmIK

PACKAGE = Path(__file__).resolve().parents[1]


def motion_symbols():
    tree = ast.parse((PACKAGE / "examples/task1.py").read_text())
    names = {"_smootherstep", "_right_only_smooth_trajectory", "_clearance_point"}
    nodes = [node for node in tree.body
             if isinstance(node, ast.FunctionDef) and node.name in names]
    values = {"math": math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), "<motion>", "exec"), values)
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id == "RIGHT_ARM_SIDE_READY_RAD"
                for target in node.targets):
            values["ready"] = ast.literal_eval(node.value)
    return values


def test_preparation_reaches_above_first_object_without_touching_fixtures():
    from utils.task1_v2_randomization import Task1V2RandomizationPlanner
    ik = Scene1V2RightArmIK()
    ik.set_base_translation_world((0.015, 0.0, 0.0))
    for seed in range(1, 6):
        obj = Task1V2RandomizationPlanner().plan(seed).cylinders[0].position
        path = ik.preparation_trajectory([0.0] * 14, obj)
        for point in path:
            ik._forward(point[7:14])
            mujoco.mj_forward(ik.model, ik.data)
            for contact in ik.data.contact:
                names = [ik.model.geom(int(g)).name
                         for g in (contact.geom1, contact.geom2)]
                right = [ik.model.body(int(ik.model.geom_bodyid[int(g)])).name
                         .startswith(("r_", "zarm_r"))
                         for g in (contact.geom1, contact.geom2)]
                assert sum(right) != 1, "preparation collides: {}".format(names)
        assert np.allclose(ik.closed_grasp_center_world(path[-1][7:14]),
                           [obj[0] - 0.015, obj[1] - 0.010, 0.85], atol=0.001)
        speed = np.diff(np.asarray(path)[:, 7:14], axis=0) / 0.01
        assert np.max(np.abs(speed)) < 1.5
        assert np.max(np.abs(speed[[0, -1]])) < 0.001


def test_source_approach_stays_above_obstacles_until_vertical_descent():
    point = motion_symbols()["_clearance_point"]
    start, target = [0.3, -0.7, 0.65], [0.6, -0.1, 0.675]
    for fraction in np.linspace(0.2, 0.8, 31):
        assert point(start, target, fraction, 0.81)[2] == 0.81
    for fraction in np.linspace(0.8, 1.0, 11):
        assert np.allclose(point(start, target, fraction, 0.81)[:2], target[:2])
    assert np.allclose(point(start, target, 0.0, 0.81), start)
    assert np.allclose(point(start, target, 1.0, 0.81), target)


def test_initial_elbow_settling_does_not_push_thumb_into_conveyor_support():
    ik = Scene1V2RightArmIK()
    ik.set_base_translation_world((0.015, 0.0, 0.0))
    thumb = ik.model.geom("r_thumb_fingertip_collision").id
    rail = ik.model.geom("target_conveyor_front_support").id
    # The controller settles a nominally straight elbow slightly below zero.
    # Checking q=0 alone previously missed the startup collision.
    for elbow in np.linspace(-0.03, 0.0, 31):
        joints = np.zeros(7)
        joints[3] = elbow
        ik._forward(joints)
        mujoco.mj_forward(ik.model, ik.data)
        clearance = mujoco.mj_geomDistance(ik.model, ik.data, thumb, rail, 0.1, None)
        assert clearance > 0.015, (elbow, clearance)
