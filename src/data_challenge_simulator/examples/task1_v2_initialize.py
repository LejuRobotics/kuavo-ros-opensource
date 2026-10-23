#!/usr/bin/env python3
"""Task 1 model initialization: place the layout and prepare above its first object.

The base stays at its fixed launch pose. The caller owns the arm publisher
and hands control to the external policy after this one smooth motion.
"""

import math
import time


from utils.gripper_controller import GripperController
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.task1_v2_randomization import Task1V2RandomizationPlanner
from utils.arm_state import wait_for_first_arm_state
from utils.scene1_v2_right_arm_ik import Scene1V2RightArmIK

from task2_base_motion import ChassisMotion


ARM_JOINT_COUNT = 14
CYLINDERS = ("cylinder_1", "cylinder_2", "cylinder_3")
TARGET_BIN = "target_bin"
LEFT_THUMB_SAFE_J1_RAD = 1.75
LEFT_THUMB_INITIALIZATION_DURATION_S = 1.2
TRAJECTORY_SLEEP = 0.02
CHASSIS_LINEAR_SPEED = 0.40
CHASSIS_ANGULAR_SPEED = 0.30
CHASSIS_MIN_LINEAR_SPEED = 0.08
CHASSIS_MIN_ANGULAR_SPEED = 0.08
CHASSIS_POSITION_TOLERANCE = 0.03
CHASSIS_YAW_TOLERANCE_DEG = 3.0


def make_chassis():
    """Build the model-only Task 1 chassis command publisher."""
    return ChassisMotion(
        linear_speed=CHASSIS_LINEAR_SPEED,
        angular_speed=CHASSIS_ANGULAR_SPEED,
        minimum_linear_speed=CHASSIS_MIN_LINEAR_SPEED,
        minimum_angular_speed=CHASSIS_MIN_ANGULAR_SPEED,
        position_tolerance=CHASSIS_POSITION_TOLERANCE,
        yaw_tolerance_deg=CHASSIS_YAW_TOLERANCE_DEG,
    )


def wait_for_arm_state(robot_state, timeout=30.0):
    """Block until 14 measured arm joints are available."""
    del robot_state
    return wait_for_first_arm_state(timeout=timeout)


def publish_trajectory(trajectory, points, sleep_time=TRAJECTORY_SLEEP):
    """Execute through the caller-owned publisher used by the accepted task."""
    trajectory.execute_trajectory(points, sleep_time=sleep_time)


def initialize_left_thumb(gripper):
    """Move only l_thumb_j1 and preserve every other measured hand joint."""
    starts = gripper._wait_for_measured_positions(("left",))["left"]
    target = list(starts)
    target[gripper.left_names.index("l_thumb_j1")] = LEFT_THUMB_SAFE_J1_RAD
    steps = max(1, int(round(
        LEFT_THUMB_INITIALIZATION_DURATION_S * gripper.publish_frequency)))
    for step in range(1, steps + 1):
        alpha = float(step) / steps
        positions = [
            start + alpha * (end - start)
            for start, end in zip(starts, target)
        ]
        with gripper.command_lock:
            gripper.current_left_positions = positions
            gripper.command_initialized.set()
        time.sleep(1.0 / gripper.publish_frequency)


def place_scene(randomizer, seed, chassis):
    """Place Task 1 objects without entering the accepted task's base gate."""
    plan = Task1V2RandomizationPlanner().plan(seed)
    for name, spawn in zip(CYLINDERS, plan.cylinders):
        x, y, z = spawn.position
        yaw = spawn.yaw
        result = randomizer.set_object_position(
            name,
            position={"x": x, "y": y, "z": z},
            orientation={
                "w": math.cos(yaw / 2.0),
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(yaw / 2.0),
            },
        )
        if not result["success"]:
            raise RuntimeError(result["message"])
    print(
        "V2 model seed {} selected precomputed layout {}: initial_base={} "
        "task_base={} objects={}".format(
            seed,
            plan.catalogue_seed,
            [round(value, 6) for value in plan.initial_base],
            [round(value, 6) for value in plan.task_base],
            [[round(value, 6) for value in spawn.position]
             for spawn in plan.cylinders],
        ))
    time.sleep(1.0)
    chassis.wait_until_ready()
    measured = chassis.pose()
    if measured is not None:
        print("V2 model seed {} launch-time measured base pose: {}".format(
            seed, [round(value, 6) for value in measured]))
    return plan.task_base


def run_initialization(robot, robot_state, seed, trajectory):
    """Place objects, hold the base, and return the measured ready joints."""
    chassis = make_chassis()
    gripper = GripperController()
    try:
        chassis.stop()

        initialize_left_thumb(gripper)
        randomizer = ObjectRandomizer(timeout=60.0)
        base_translation_world = place_scene(randomizer, seed, chassis)
        print("V2 seed {} robot base translation: {}".format(
            seed, [round(value, 6) for value in base_translation_world]))

        poses = ObjectPose()
        for name in CYLINDERS + (TARGET_BIN,):
            poses.wait_for_position(name, timeout=60.0)
        print("V2 target bin live position: {}".format(
            [round(value, 6)
             for value in poses.get_position(TARGET_BIN)]))

        measured_b0 = tuple(chassis.pose())
        scene_ik = Scene1V2RightArmIK()
        scene_ik.set_base_translation_world((measured_b0[0], measured_b0[1], 0.0))
        points = scene_ik.preparation_trajectory(
            wait_for_arm_state(robot_state), poses.get_position(CYLINDERS[0]))
        publish_trajectory(trajectory, points, sleep_time=0.01)
        staging_arm_rad = wait_for_arm_state(robot_state)
        print(
            "V2 model initialization complete: measured B0={} staging={}"
            .format(
                [round(value, 6) for value in measured_b0],
                [round(value, 6) for value in staging_arm_rad[7:14]]))
        return measured_b0, staging_arm_rad
    finally:
        gripper.stop()
        # This file is model-entry-only. Release its ROS endpoints before the
        # external policy is admitted; the accepted task keeps its own
        # ChassisMotion lifecycle unchanged.
        from utils.model_base_motion import close_chassis
        close_chassis(chassis)
