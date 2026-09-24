#!/usr/bin/env python3
"""Run Task 3's Task-1-style one-shot feed-forward second IK."""

import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import rospy
from sensor_msgs.msg import JointState


CURRENT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = CURRENT_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
from utils.gripper_controller import GripperController
from utils.object_pos import ObjectPose
from utils.task3_second_ik import Task3SecondIKPlanner
from utils.trajectory_controller import TrajectoryController

from task2_base_motion import ChassisMotion
from task3_initialize import (
    DEFAULT_SCENE,
    wait_for_arm_state,
    wait_for_controller_initialization,
)


RING_NAME = "task3_hollow_cylinder"
DEFAULT_CONFIG = PACKAGE_DIR / "config/task3_transfer.json"
HELD_ARM_COMMAND_TOPIC = "/vr_incremental/kuavo_arm_traj_filtered"


def wait_for_held_arm_command(timeout=2.0):
    """Read the continuously held 14-joint command, returned in radians."""
    message = rospy.wait_for_message(
        HELD_ARM_COMMAND_TOPIC, JointState, timeout=float(timeout))
    positions_deg = np.asarray(message.position, dtype=float)
    if positions_deg.shape != (14,) or not np.all(np.isfinite(positions_deg)):
        raise RuntimeError(
            "Task 3 held arm command must contain 14 finite positions")
    return tuple(np.radians(positions_deg))


def execute_right_target(
        trajectory, measured, target_right, trajectory_points):
    left_arm = np.asarray(measured[:7], dtype=float)
    start_right = np.asarray(measured[7:14], dtype=float)
    target_right = np.asarray(target_right, dtype=float)
    points = [
        np.concatenate((
            left_arm,
            (1.0 - ratio) * start_right + ratio * target_right,
        )).tolist()
        for ratio in np.linspace(
            1.0 / trajectory_points, 1.0, trajectory_points)
    ]
    trajectory.execute_trajectory(points, sleep_time=0.02)


def init_runtime(args, gripper=None, trajectory=None):
    """Perform the one-time ROS/SDK setup shared by every Task 3 stage."""
    rospy.init_node("task3_second_ik", anonymous=False)
    wait_for_controller_initialization()
    if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
        raise RuntimeError("KuavoSDK initialization failed")
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    chassis = ChassisMotion(
        linear_speed=0.08,
        angular_speed=0.20,
        minimum_linear_speed=0.06,
        minimum_angular_speed=0.06,
        position_tolerance=0.03,
        yaw_tolerance_deg=3.0,
    )
    chassis.wait_until_ready(timeout=30.0)
    return SimpleNamespace(
        robot=robot,
        robot_state=robot_state,
        chassis=chassis,
        poses=ObjectPose(),
        planner=None,
        gripper=GripperController() if gripper is None else gripper,
        trajectory=trajectory,
    )


def make_planner(scene, ring_name):
    """Build the per-ring IK planner the accepted stage used to create."""
    return Task3SecondIKPlanner(scene, ring_name=ring_name)


def run_descend(
        robot, robot_state, chassis, object_pose, planner, gripper,
        config, trajectory_points, settle_seconds,
        ring_name, docking_base=None, trajectory=None):
    """Run the accepted one-shot feed-forward second IK in an existing node."""
    if trajectory_points <= 0:
        raise ValueError("trajectory points must be positive")
    if settle_seconds < 0.0:
        raise ValueError("settle seconds cannot be negative")
    rospy.loginfo(
        "Task3 direct sequence: shaping compact hand before one-shot IK")
    gripper.control_right_internal_expansion(
        255.0 * float(config["compact_expansion"]),
        duration=float(config["hand_motion_duration_s"]))
    # Move to the ring's own docking pose first.  After a transfer the base is
    # parked at the destination table, where the IK for the next ring cannot
    # solve at all, so attempt-then-fallback would only waste a failed solve.
    if docking_base is not None:
        rospy.loginfo(
            "Task3 moving to verified docking=(%.3f, %.3f, %.1f deg) "
            "before solving for %s",
            docking_base[0], docking_base[1],
            math.degrees(docking_base[2]), ring_name)
        chassis.move_to_pose(
            docking_base[0], docking_base[1],
            docking_base[2], timeout=45.0)
    ring_position = object_pose.wait_for_position(
        ring_name, timeout=30.0)
    measured = tuple(wait_for_arm_state(robot_state))
    base_pose = tuple(chassis.pose())
    result = planner.solve_feedforward(
        base_pose, ring_position, measured)
    if result is None:
        raise RuntimeError(
            "Task 3 docking pose has no IK solution for {}".format(
                ring_name))
    rospy.loginfo(
        "Task3 docking pose has an IK solution for %s", ring_name)

    trajectory_start = wait_for_held_arm_command()
    if trajectory is None:
        trajectory = TrajectoryController(
            robot, initial_positions=trajectory_start)
    else:
        # Same live start the stage used to pass into its own controller.
        # set_target_positions copies in place, so it needs a list.
        trajectory.set_target_positions(list(trajectory_start))
    desired_position, desired_rotation = planner.reference_pose(
        ring_position)
    command_position, command_rotation = planner.feedforward_command_pose(
        ring_position)
    rospy.loginfo(
        "Task3 one-shot feed-forward IK: desired_position=%s, "
        "command_position=%s, command_rotation_offset=%.2f deg",
        np.round(desired_position, 6),
        np.round(command_position, 6),
        math.degrees((
            desired_rotation.inv() * command_rotation).magnitude()))
    execute_right_target(
        trajectory, trajectory_start, result.right_joints,
        trajectory_points)
    rospy.sleep(settle_seconds)

    measured_after = tuple(wait_for_arm_state(robot_state))
    target_right = np.asarray(result.right_joints, dtype=float)
    actual_right = np.asarray(measured_after[7:14], dtype=float)
    joint_error = actual_right - target_right
    actual_position, actual_rotation = planner.measured_pose(
        tuple(chassis.pose()), ring_position, actual_right)
    position_error = desired_position - actual_position
    rotation_error = desired_rotation * actual_rotation.inv()
    observed_tracking_rotation = command_rotation.inv() * actual_rotation
    rospy.loginfo(
        "TASK3 SECOND IK JOINTS: target_rad=%s, actual_rad=%s, "
        "actual_minus_target_deg=%s",
        np.round(target_right, 6), np.round(actual_right, 6),
        np.round(np.degrees(joint_error), 3))
    rospy.loginfo(
        "TASK3 SECOND IK POSES: desired_quat_xyzw=%s, "
        "command_quat_xyzw=%s, actual_quat_xyzw=%s, "
        "observed_actual_minus_command_position_mm=%s, "
        "observed_command_to_actual_quat_xyzw=%s",
        np.round(desired_rotation.as_quat(), 8),
        np.round(command_rotation.as_quat(), 8),
        np.round(actual_rotation.as_quat(), 8),
        np.round(1000.0 * (actual_position - command_position), 3),
        np.round(observed_tracking_rotation.as_quat(), 8))
    rospy.loginfo(
        "TASK3 SECOND IK COMMAND COMPLETE: measured error "
        "position_mm=%s norm=%.2f, rotation=%.2f deg",
        np.round(1000.0 * position_error, 2),
        1000.0 * np.linalg.norm(position_error),
        math.degrees(rotation_error.magnitude()))
    return trajectory


def main(runtime=None, argv=None, ring_name=None, docking_base=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--trajectory-points", type=int, default=200)
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--ring-name", default=RING_NAME)
    parser.add_argument("--docking-base", type=float, nargs=3,
                        metavar=("X", "Y", "YAW"))
    # Called from the merged workflow the stage must not read the entry
    # point's command line; the ring target arrives as an argument instead.
    merged = runtime is not None
    args = parser.parse_args([] if merged else argv)
    if ring_name is not None:
        args.ring_name = ring_name
    if docking_base is not None:
        args.docking_base = tuple(docking_base)
    with args.config.open("r", encoding="utf-8") as stream:
        config = json.load(stream)

    owns_runtime = runtime is None
    if owns_runtime:
        runtime = init_runtime(args)
    if runtime.planner is None:
        runtime.planner = make_planner(args.scene, args.ring_name)
    trajectory = runtime.trajectory
    try:
        trajectory = run_descend(
            runtime.robot, runtime.robot_state, runtime.chassis,
            runtime.poses, runtime.planner, runtime.gripper,
            config, args.trajectory_points, args.settle_seconds,
            args.ring_name, docking_base=args.docking_base,
            trajectory=trajectory)
    finally:
        if owns_runtime:
            if trajectory is not None:
                trajectory.stop()
            runtime.gripper.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
