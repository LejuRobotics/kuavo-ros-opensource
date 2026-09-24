#!/usr/bin/env python3
"""Task 1 V2 one-time initialization for the external model entry.

The accepted ``task1.py`` performs this action sequence inline before its
first task motion: place the saved layout, retreat to a safe base, raise the
right arm through shoulder clearance / four-joint ready / full seven-joint
ready, then return once to the measured random base B0.  The model entry needs
that prefix and none of the task policy.  Its small fixed initialization values
are regression-checked against Task 1, without importing the full Task 1 IK and
policy module at runtime.  Its base legs are timed open-loop commands with no
arrival decision.  ``task1.py`` itself is not modified.

The model entry passes the same continuously publishing ``TrajectoryController``
shape used by the accepted task entry.  This helper never owns or stops that
publisher; the caller closes it after the complete fixed initialization.
"""

import math
import time

import rospy

from utils.gripper_controller import GripperController
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.task1_v2_randomization import Task1V2RandomizationPlanner
from utils.arm_state import wait_for_first_arm_state
from utils.utils import Utils

from task2_base_motion import ChassisMotion


ARM_JOINT_COUNT = 14
CYLINDERS = ("cylinder_1", "cylinder_2", "cylinder_3")
TARGET_BIN = "target_bin"
INITIALIZATION_SAFE_RETREAT_M = 0.20
LEFT_THUMB_SAFE_J1_RAD = 1.75
LEFT_THUMB_INITIALIZATION_DURATION_S = 1.2
SHOULDER_LIFT_DEG = 60.0
RIGHT_ARM_READY_RAD = (-1.0, -0.5, 1.0, -1.4)
RIGHT_ARM_READY_FULL_RAD = (-0.9, -0.265, 1.0, -0.8, 0.58, -0.4, 0.35)
TRAJECTORY_POINTS = 80
TRAJECTORY_SLEEP = 0.02
STOP_SPEED_THRESHOLD = 0.01
STOP_WATCHDOG_S = 10.0
CHASSIS_LINEAR_SPEED = 0.08
CHASSIS_ANGULAR_SPEED = 0.20
CHASSIS_MIN_LINEAR_SPEED = 0.06
CHASSIS_MIN_ANGULAR_SPEED = 0.06
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


def right_only_trajectory(target_deg, start_deg, left_hold_rad, num):
    """Interpolate the right arm while holding the measured left arm."""
    points = Utils.interpolate_joint_trajectory(
        target_deg, start_deg, num=num)
    for point in points:
        point[:7] = left_hold_rad
    return points


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


def move_base_open_loop(chassis, target_world, phase):
    """Drive once by elapsed time and continue without an arrival decision."""
    chassis.wait_until_ready()
    plan = chassis.move_open_loop(target_world[0], target_world[1])
    chassis.wait_until_stopped(STOP_SPEED_THRESHOLD, STOP_WATCHDOG_S)
    measured = tuple(chassis.pose())
    position_error = math.hypot(
        float(target_world[0]) - measured[0],
        float(target_world[1]) - measured[1],
    )
    rospy.loginfo(
        "Task1 model %s open-loop move finished: target=(%.3f, %.3f) "
        "measured=(%.3f, %.3f, %.1f deg) duration=%.3f s "
        "error=%.4f m; continuing without an arrival decision",
        phase, target_world[0], target_world[1], measured[0], measured[1],
        math.degrees(measured[2]), plan["duration"], position_error)
    return measured


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
    # ``initial_base`` is deliberately behind the transverse conveyor so the
    # default open thumb cannot spawn inside it.  Keep the robot there until
    # the safe-retreat step closes the thumb and prepares the right arm; the
    # existing final return then advances to ``task_base``.
    return plan.task_base


def run_initialization(robot, robot_state, seed, trajectory):
    """Randomize the scene, retreat, raise the right arm, and return to B0.

    Returns the measured random base pose ``B0`` and the measured ready joints.
    """
    chassis = make_chassis()
    gripper = GripperController()
    try:
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

        initial_arm_rad = wait_for_arm_state(robot_state)
        left_hold_rad = list(initial_arm_rad[:7])
        current_target_deg = [
            math.degrees(value) for value in initial_arm_rad]

        safe_base_world = (
            base_translation_world[0] - INITIALIZATION_SAFE_RETREAT_M,
            base_translation_world[1],
            base_translation_world[2],
        )
        print(
            "V2 initialization retreat: B0={} safe={} distance={:.3f} m"
            .format(
                [round(value, 6) for value in base_translation_world],
                [round(value, 6) for value in safe_base_world],
                INITIALIZATION_SAFE_RETREAT_M))
        move_base_open_loop(chassis, safe_base_world, "safe retreat")
        initialize_left_thumb(gripper)

        # Shoulder clearance, four-joint ready, then the complete seven-joint
        # ready.  Identical to the accepted task1 sequence; the measured
        # left arm is held at every trajectory point.
        shoulder_only_deg = list(current_target_deg)
        shoulder_only_deg[8] = -SHOULDER_LIFT_DEG
        shoulder_only_deg[12] = math.degrees(-0.5)
        publish_trajectory(trajectory, right_only_trajectory(
            shoulder_only_deg, current_target_deg, left_hold_rad,
            num=TRAJECTORY_POINTS))
        current_target_deg = shoulder_only_deg

        ready_deg = list(current_target_deg)
        ready_deg[7:11] = [
            math.degrees(value) for value in RIGHT_ARM_READY_RAD]
        publish_trajectory(trajectory, right_only_trajectory(
            ready_deg, current_target_deg, left_hold_rad,
            num=TRAJECTORY_POINTS))
        current_target_deg = ready_deg

        full_ready_deg = list(current_target_deg)
        full_ready_deg[7:14] = [
            math.degrees(value) for value in RIGHT_ARM_READY_FULL_RAD]
        publish_trajectory(trajectory, right_only_trajectory(
            full_ready_deg, current_target_deg, left_hold_rad,
            num=TRAJECTORY_POINTS))
        current_target_deg = full_ready_deg
        time.sleep(0.2)

        # Return once to this run's random B0.  The arm holds the completed
        # ready posture throughout the translation.
        measured_b0 = move_base_open_loop(
            chassis, base_translation_world, "return to B0")
        ready_arm_rad = wait_for_arm_state(robot_state)
        print(
            "V2 model initialization complete: measured B0={} ready={}"
            .format(
                [round(value, 6) for value in measured_b0],
                [round(value, 6) for value in ready_arm_rad[7:14]]))
        return measured_b0, ready_arm_rad
    finally:
        gripper.stop()
        # This file is model-entry-only. Release its ROS endpoints before the
        # external policy is admitted; the accepted task keeps its own
        # ChassisMotion lifecycle unchanged.
        from utils.model_base_motion import close_chassis
        close_chassis(chassis)
