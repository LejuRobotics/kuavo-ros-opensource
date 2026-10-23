"""Replay the second-pick docking error that previously rejected the right arm."""
import json
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from utils.task2_pick_planner import Task2PickPlanner
from utils.task2_randomization import Task2RandomizationPlanner


@pytest.mark.parametrize("yaw_degrees", [-1.0, 0.0, 1.0])
def test_second_pick_remains_reachable_with_observed_parking_error(yaw_degrees):
    package = Path(__file__).resolve().parents[1]
    config = json.loads((package / "config/task2_pick.json").read_text())
    # Measured ready arm state; apply parking error relative to the new box location.
    arm_seed = [
        -0.8020532, 0.0238218, -0.6935304, -0.1560061, -0.9097437, 0.6612684, 0.2844702,
        -0.8020565, -0.0238175, 0.6932746, -0.1558732, 0.9172203, -0.6607027, 0.2869042,
    ]
    layout = Task2RandomizationPlanner().plan(1)
    boxes = {box.name: (box.position, Rotation.from_euler("z", box.yaw).as_quat())
             for box in layout.boxes}
    planner = Task2PickPlanner(ik_config=config["ik"])
    assert config["ik"]["maximum_position_error_m"] == 0.03
    dock = layout.boxes[0].docking_base
    plan = planner.plan(
        "box_1", (dock[0] + 0.015, dock[1] - 0.007, math.radians(yaw_degrees)),
        boxes, arm_seed, config["grasp"])
    assert np.isfinite(plan.contact_joints).all()
    assert np.isfinite(plan.lift_waypoints).all()


def test_docking_does_not_skip_lateral_alignment():
    from utils.task2_pick_planner import select_front_docking_target

    package = Path(__file__).resolve().parents[1]
    config = json.loads((package / "config/task2_pick.json").read_text())
    layout = Task2RandomizationPlanner().plan(2)
    boxes = {box.name: (box.position, Rotation.from_euler("z", box.yaw).as_quat())
             for box in layout.boxes}
    target = select_front_docking_target((0.049, 0.090, 0), boxes, config["docking"])
    assert target.box_name == "box_2"
    assert not target.already_comfortable
    aligned = select_front_docking_target(
        (target.x, target.y, target.yaw), boxes, config["docking"])
    assert aligned.already_comfortable


def test_minimum_box_gap_keeps_grasp_clear_of_the_other_box():
    package = Path(__file__).resolve().parents[1]
    config = json.loads((package / "config/task2_pick.json").read_text())
    planner = Task2PickPlanner(ik_config=config["ik"])
    boxes = {
        "box_1": ((0.755842, -0.42, 0.6), (0, 0, 0, 1)),
        "box_2": ((0.746199, 0.12, 0.6), (0, 0, 0, 1)),
    }
    seed = [-0.802, 0.024, -0.694, -0.156, -0.910, 0.661, 0.284,
            -0.802, -0.024, 0.693, -0.156, 0.917, -0.661, 0.287]
    # Include the measured stopping offset from the failed close-box scene.
    base = (0.746199 - 0.68 + 0.015, 0.12 + 0.019, 0)
    plan = planner.plan("box_2", base, boxes, seed, config["grasp"])
    for start, end, closure in (
            (plan.above_joints, plan.contact_joints, 0),
            (plan.contact_joints, plan.contact_joints, 1)):
        for ratio in np.linspace(0, 1, 21):
            joints = (1 - ratio) * np.asarray(start) + ratio * np.asarray(end)
            planner._set_configuration(
                base, boxes, {"l": joints[:7], "r": joints[7:]}, closure)
            for contact in planner.data.contact:
                a, b = int(contact.geom1), int(contact.geom2)
                touches_other = (
                    a in planner.robot_geom_ids and b in planner.box_geom_ids["box_1"]
                    or b in planner.robot_geom_ids and a in planner.box_geom_ids["box_1"])
                assert not touches_other or contact.dist >= 0


def test_return_turn_clears_table_and_conveyor_legs():
    import mujoco

    package = Path(__file__).resolve().parents[1]
    config = json.loads((package / "config/task2_pick.json").read_text())
    planner = Task2PickPlanner(ik_config=config["ik"])
    seed = [-0.802, 0.024, -0.694, -0.156, -0.910, 0.661, 0.284,
            -0.802, -0.024, 0.693, -0.156, 0.917, -0.661, 0.287]
    boxes = {"box_1": ((0.755, -0.42, 0.6), (0, 0, 0, 1)),
             "box_2": ((0.11, 1.4, 0.56), (0, 0, 0, 1))}
    # Include the full docking X range and 3 cm stopping tolerance.
    turn_y = config["transport"]["return_turn_base_y_m"] + 0.03
    for x in np.linspace(0.05, 0.16, 5):
        for yaw in np.linspace(0, math.pi / 2, 91):
            planner._set_configuration(
                (x, turn_y, yaw), boxes, {"l": seed[:7], "r": seed[7:]}, 0)
            # Existing wheel allocation for a clockwise turn.
            for prefix, dx, dy in (("LF", 1, 1), ("RF", 1, -1),
                                   ("LB", -1, 1), ("RB", -1, -1)):
                joint = mujoco.mj_name2id(
                    planner.model, mujoco.mjtObj.mjOBJ_JOINT,
                    prefix + "_wheel_yaw_joint")
                planner.data.qpos[planner.model.jnt_qposadr[joint]] = math.atan2(-dx, dy)
            mujoco.mj_forward(planner.model, planner.data)
            for contact in planner.data.contact:
                a, b = int(contact.geom1), int(contact.geom2)
                robot_static = (
                    a in planner.robot_geom_ids and b in planner.static_obstacle_geom_ids
                    or b in planner.robot_geom_ids and a in planner.static_obstacle_geom_ids)
                assert not robot_static or contact.dist >= 0


def test_loaded_turn_clears_legs_with_stopping_error():
    import mujoco

    package = Path(__file__).resolve().parents[1]
    config = json.loads((package / "config/task2_pick.json").read_text())
    planner = Task2PickPlanner(ik_config=config["ik"])
    # Measured lifted arm state from the blocked second pick in seed 5.
    joints = [-0.323438, -0.076801, -0.202379, -1.604051, -1.447397, -0.304596, 0.535448,
              -0.300341, 0.048576, 0.192734, -1.617077, 1.461653, 0.325203, 0.536432]
    boxes = {"box_1": ((0.77, -0.42, 0.78), (0, 0, 0, 1)),
             "box_2": ((0.1, 1.4, 0.56), (0, 0, 0, 1))}
    turn_x = config["transport"]["loaded_turn_max_base_x_m"] + 0.03
    for y in (-0.45, -0.42, -0.39, 0.09, 0.12, 0.23, 0.30, 0.33):
        for yaw in np.linspace(-3, 94, 98):
            planner._set_configuration(
                (turn_x, y, math.radians(yaw)), boxes,
                {"l": joints[:7], "r": joints[7:]}, 1)
            for prefix, dx, dy in (("LF", 1, 1), ("RF", 1, -1),
                                   ("LB", -1, 1), ("RB", -1, -1)):
                joint = mujoco.mj_name2id(
                    planner.model, mujoco.mjtObj.mjOBJ_JOINT,
                    prefix + "_wheel_yaw_joint")
                planner.data.qpos[planner.model.jnt_qposadr[joint]] = math.atan2(dx, -dy)
            mujoco.mj_forward(planner.model, planner.data)
            for contact in planner.data.contact:
                a, b = int(contact.geom1), int(contact.geom2)
                robot_static = (
                    a in planner.robot_geom_ids and b in planner.static_obstacle_geom_ids
                    or b in planner.robot_geom_ids and a in planner.static_obstacle_geom_ids)
                assert not robot_static or contact.dist >= 0
