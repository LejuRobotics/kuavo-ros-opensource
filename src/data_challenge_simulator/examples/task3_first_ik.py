#!/usr/bin/env python3
"""Run only Task 3's first online IK from the accepted ready posture."""

import argparse
import json
import math
from pathlib import Path
import sys

import rospy


CURRENT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = CURRENT_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
from utils.gripper_controller import GripperController
from utils.object_pos import ObjectPose
from utils.task3_right_arm_ik import Task3RightArmIK
from utils.trajectory_controller import TrajectoryController
from utils.utils import Utils

from task2_base_motion import ChassisMotion
from task3_initialize import (
    DEFAULT_SCENE,
    wait_for_arm_state,
    wait_for_controller_initialization,
)


DEFAULT_CONFIG = PACKAGE_DIR / "config/task3_transfer.json"
RING_NAME = "task3_hollow_cylinder"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    args = parser.parse_args()
    with args.config.open("r", encoding="utf-8") as stream:
        config = json.load(stream)

    rospy.init_node("task3_first_ik", anonymous=False)
    wait_for_controller_initialization()
    if not KuavoSDK().Init(options=KuavoSDK.Options.Normal):
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
    measured = tuple(wait_for_arm_state(robot_state))
    base_pose = tuple(chassis.pose())
    ring_position = ObjectPose().wait_for_position(RING_NAME, timeout=30.0)
    planner = Task3RightArmIK(args.scene)
    result = planner.solve_above_ring(
        base_pose, ring_position, measured,
        config["above_ring_clearance_m"])
    if result is None:
        raise RuntimeError(
            "current base pose is outside the exact Task 3 IK workspace")

    target = measured[:7] + result.joints
    rospy.loginfo(
        "Task3 first IK: base=(%.3f, %.3f, %.1f deg), "
        "ring=(%.3f, %.3f, %.3f), "
        "target_compact_fingers=(%.3f, %.3f, %.3f)",
        base_pose[0], base_pose[1], math.degrees(base_pose[2]),
        ring_position[0], ring_position[1], ring_position[2],
        *result.target_compact_finger_center)
    trajectory = TrajectoryController(robot, initial_positions=measured)
    gripper = GripperController()
    try:
        trajectory.execute_trajectory(
            Utils.interpolate_joint_trajectory(
                [math.degrees(value) for value in target],
                [math.degrees(value) for value in measured],
                num=int(config["approach_trajectory_points"])),
            sleep_time=0.02)
        rospy.loginfo("TASK3 FIRST IK COMPLETE; shaping compact hand")
        gripper.control_right_internal_expansion(
            255.0 * float(config["compact_expansion"]),
            duration=float(config["hand_motion_duration_s"]))
    finally:
        trajectory.stop()
        gripper.stop()
    rospy.loginfo("TASK3 FIRST IK AND COMPACT HAND COMPLETE")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
