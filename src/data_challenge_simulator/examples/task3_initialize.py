#!/usr/bin/env python3
"""Execute Task 3 preprocessing only: retreat, right-arm ready, return."""

import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import time

import rospy


CURRENT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = CURRENT_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
from utils.task3_initialization import right_ready_target
from utils.trajectory_controller import TrajectoryController
from utils.arm_state import wait_for_first_arm_state
from utils.utils import Utils

from task2_base_motion import ChassisMotion


DEFAULT_CONFIG = PACKAGE_DIR / "config/task3_initialization.json"
DEFAULT_SCENE = PACKAGE_DIR / (
    "models/biped_s400062/xml/task3.xml")
ARM_JOINT_COUNT = 14
TRAJECTORY_SLEEP = 0.02


def wait_for_arm_state(robot_state, timeout=30.0):
    del robot_state
    return wait_for_first_arm_state(timeout=timeout)


def wait_for_controller_initialization(timeout=45.0):
    """Wait for SDK parameters and the controller's reset marker."""
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


def run_initialization(
        robot, robot_state, chassis, config, scene_path,
        motion_timeout=40.0, dry_run=False, trajectory=None,
        retain_owned_trajectory=False, on_trajectory_started=None):
    """Execute the one-time Task 3 retreat/ready/return preprocessing.

    ``trajectory`` is the workflow's shared arm publisher.  When it is omitted
    the stage builds and stops its own, which is what running this file alone
    still does.  In the merged workflow the shared publisher is reseeded with
    the same live measurement and left running, so ``/kuavo_arm_traj`` does not
    break between this stage and the first grasp.
    """
    del scene_path
    initial_arm = wait_for_arm_state(robot_state)
    initial_base = tuple(chassis.pose())
    retreat_m = float(config["safe_retreat_m"])
    safe_base = (
        initial_base[0] - retreat_m * math.cos(initial_base[2]),
        initial_base[1] - retreat_m * math.sin(initial_base[2]),
        initial_base[2],
    )
    target = right_ready_target(
        initial_arm, config["right_arm_ready_rad"])
    rospy.loginfo(
        "Task3 preprocessing: B0=(%.3f, %.3f, %.1f deg), retreat=%.3f m",
        initial_base[0], initial_base[1], math.degrees(initial_base[2]),
        retreat_m)
    if dry_run:
        return initial_base, initial_arm

    chassis.move_to_pose(
        safe_base[0], safe_base[1], safe_base[2], timeout=motion_timeout)
    measured_safe_arm = wait_for_arm_state(robot_state)
    measured_target = right_ready_target(
        measured_safe_arm, config["right_arm_ready_rad"])

    owns_trajectory = trajectory is None
    if owns_trajectory:
        trajectory = TrajectoryController(
            robot, initial_positions=measured_safe_arm)
        if on_trajectory_started is not None:
            on_trajectory_started(trajectory)
    else:
        # Same live start the stage used to pass into its own controller.
        trajectory.set_target_positions(list(measured_safe_arm))
    try:
        trajectory.execute_trajectory(
            Utils.interpolate_joint_trajectory(
                [math.degrees(value) for value in measured_target],
                [math.degrees(value) for value in measured_safe_arm],
                num=int(config["ready_trajectory_points"])),
            sleep_time=TRAJECTORY_SLEEP)
    finally:
        if owns_trajectory and not retain_owned_trajectory:
            trajectory.stop()
    wait_for_arm_state(robot_state)

    chassis.move_to_pose(
        initial_base[0], initial_base[1], initial_base[2],
        timeout=motion_timeout)
    returned_base = tuple(chassis.pose())
    measured_seed = wait_for_arm_state(robot_state)
    rospy.set_param("/task3_initial_base_pose", initial_base)
    rospy.set_param("/task3_returned_base_pose", returned_base)
    rospy.set_param("/task3_initialized_arm_seed", measured_seed)
    rospy.loginfo(
        "Task3 preprocessing complete: returned base=(%.3f, %.3f, "
        "%.1f deg); stored measured 14-joint IK seed",
        returned_base[0], returned_base[1],
        math.degrees(returned_base[2]))
    return returned_base, measured_seed


def init_runtime(args):
    """Perform the one-time ROS/SDK setup shared by every Task 3 stage."""
    rospy.init_node("task3_initialize", anonymous=False)
    wait_for_controller_initialization()
    if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
        raise RuntimeError("KuavoSDK initialization failed")
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    chassis = ChassisMotion(
        linear_speed=args.linear_speed,
        angular_speed=0.20,
        minimum_linear_speed=min(
            args.minimum_linear_speed, args.linear_speed),
        minimum_angular_speed=0.06,
        position_tolerance=args.position_tolerance,
        yaw_tolerance_deg=3.0,
    )
    chassis.wait_until_ready(timeout=30.0)
    return SimpleNamespace(
        robot=robot,
        robot_state=robot_state,
        chassis=chassis,
    )


def main(runtime=None, argv=None, retain_trajectory=False):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--linear-speed", type=float, default=0.08)
    parser.add_argument("--minimum-linear-speed", type=float, default=0.06)
    parser.add_argument("--position-tolerance", type=float, default=0.03)
    parser.add_argument("--motion-timeout", type=float, default=40.0)
    parser.add_argument("--dry-run", action="store_true")
    # Called from the merged workflow the stage must not read the entry
    # point's command line.
    merged = runtime is not None
    args = parser.parse_args([] if merged else argv)

    with args.config.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    if config.get("schema_version") != 1:
        raise RuntimeError("unsupported Task 3 initialization config schema")

    if runtime is None:
        runtime = init_runtime(args)
    run_initialization(
        runtime.robot, runtime.robot_state, runtime.chassis, config,
        args.scene,
        motion_timeout=args.motion_timeout, dry_run=args.dry_run,
        trajectory=getattr(runtime, "trajectory", None),
        retain_owned_trajectory=retain_trajectory,
        on_trajectory_started=(
            (lambda value: setattr(runtime, "trajectory", value))
            if retain_trajectory else None))


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
