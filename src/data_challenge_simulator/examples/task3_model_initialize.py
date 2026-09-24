#!/usr/bin/env python3
"""Model-only Task 3 preprocessing without a chassis arrival decision."""

import math
from pathlib import Path
import time

import rospy

from utils.task3_initialization import right_ready_target
from utils.utils import Utils


PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_DIR / "config/task3_initialization.json"
ARM_JOINT_COUNT = 14
TRAJECTORY_SLEEP = 0.02
STOP_SPEED_THRESHOLD = 0.01
STOP_WATCHDOG_S = 10.0


def wait_for_arm_state(robot_state, timeout=30.0):
    """Block until the measured 14-joint arm state is available."""
    deadline = time.time() + float(timeout)
    while not rospy.is_shutdown() and time.time() < deadline:
        positions = list(robot_state.arm_joint_state().position)
        if len(positions) == ARM_JOINT_COUNT:
            return positions
        rospy.sleep(0.05)
    raise RuntimeError("timed out waiting for 14 measured arm joints")


def wait_for_controller_initialization(timeout=45.0):
    """Wait for the controller parameters used by Task 3 preprocessing."""
    required_parameters = (
        "/armRealDof",
        "/headRealDof",
        "/legRealDof",
        "/waistRealDof",
        "/robot_init_state_param",
    )
    deadline = time.time() + float(timeout)
    while not rospy.is_shutdown() and time.time() < deadline:
        if all(rospy.has_param(name) for name in required_parameters):
            return
        rospy.sleep(0.1)
    missing = [
        name for name in required_parameters if not rospy.has_param(name)]
    raise RuntimeError(
        "timed out waiting for controller parameters: {}".format(missing))


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
        "Task3 model %s open-loop move finished: target=(%.3f, %.3f) "
        "measured=(%.3f, %.3f, %.1f deg) duration=%.3f s "
        "error=%.4f m; continuing without an arrival decision",
        phase, target_world[0], target_world[1], measured[0], measured[1],
        math.degrees(measured[2]), plan["duration"], position_error)
    return measured


def run_initialization(robot_state, chassis, config, trajectory):
    """Retreat, command the accepted right-arm ready pose, and return."""
    wait_for_arm_state(robot_state)
    initial_base = tuple(chassis.pose())
    retreat_m = float(config["safe_retreat_m"])
    safe_base = (
        initial_base[0] - retreat_m * math.cos(initial_base[2]),
        initial_base[1] - retreat_m * math.sin(initial_base[2]),
        initial_base[2],
    )
    rospy.loginfo(
        "Task3 model preprocessing: B0=(%.3f, %.3f, %.1f deg), "
        "retreat=%.3f m",
        initial_base[0], initial_base[1], math.degrees(initial_base[2]),
        retreat_m)

    move_base_open_loop(chassis, safe_base, "safe retreat")
    measured_safe_arm = wait_for_arm_state(robot_state)
    measured_target = right_ready_target(
        measured_safe_arm, config["right_arm_ready_rad"])
    trajectory.set_target_positions(list(measured_safe_arm))
    trajectory.execute_trajectory(
        Utils.interpolate_joint_trajectory(
            [math.degrees(value) for value in measured_target],
            [math.degrees(value) for value in measured_safe_arm],
            num=int(config["ready_trajectory_points"])),
        sleep_time=TRAJECTORY_SLEEP)
    wait_for_arm_state(robot_state)

    returned_base = move_base_open_loop(
        chassis, initial_base, "return to B0")
    measured_seed = wait_for_arm_state(robot_state)
    rospy.set_param("/task3_initial_base_pose", initial_base)
    rospy.set_param("/task3_returned_base_pose", returned_base)
    rospy.set_param("/task3_initialized_arm_seed", measured_seed)
    rospy.loginfo(
        "Task3 model preprocessing complete: returned base=(%.3f, %.3f, "
        "%.1f deg); stored measured 14-joint IK seed",
        returned_base[0], returned_base[1],
        math.degrees(returned_base[2]))
    return returned_base, measured_seed
