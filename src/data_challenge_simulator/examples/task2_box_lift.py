#!/usr/bin/env python3
"""Validate the first Scene 2 box grasp, lift, and loaded chassis motion.

This is deliberately not the final Task 2 policy: the destination table and
placement pose are not defined yet.  The standardized grasp itself lives in
``config/task2_box2_grasp.json`` so later policies can reuse it without copying
joint targets into task code.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

import rospy

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(CURRENT_DIR)
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
from utils.gripper_controller import GripperController
from utils.object_pos import ObjectPose
from utils.trajectory_controller import TrajectoryController
from utils.utils import Utils

from task2_base_motion import ChassisMotion, normalize_angle


DEFAULT_CONFIG = os.path.join(
    PACKAGE_DIR, "config", "task2_box2_grasp.json")
ARM_JOINT_COUNT = 14
TRAJECTORY_SLEEP = 0.02


def positive(value):
    value = float(value)
    if value < 0.0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return value


def yaw_from_xyzw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def wait_for_arm_state(robot_state, timeout=30.0):
    deadline = time.time() + float(timeout)
    while time.time() < deadline:
        positions = list(robot_state.arm_joint_state().position)
        if len(positions) == ARM_JOINT_COUNT:
            return positions
        time.sleep(0.05)
    raise RuntimeError("Timed out waiting for 14 measured arm joints")


def arm_target(config, name):
    group = config["arm_joints_rad"][name]
    target = list(group["left"]) + list(group["right"])
    if len(target) != ARM_JOINT_COUNT:
        raise RuntimeError("{} must contain 14 arm joints".format(name))
    return target


def execute_arm_target(
        trajectory, robot_state, label, target_rad, points):
    live_rad = wait_for_arm_state(robot_state)
    rospy.loginfo("Arm stage %s: %d points", label, points)
    trajectory.execute_trajectory(
        Utils.interpolate_joint_trajectory(
            [math.degrees(value) for value in target_rad],
            [math.degrees(value) for value in live_rad],
            num=points,
        ),
        sleep_time=TRAJECTORY_SLEEP,
    )


def execute_ready_stages(trajectory, robot_state, ready_target):
    """Use the collision-clear shoulder-first initialization from Task 1."""
    stage_1 = wait_for_arm_state(robot_state)
    stage_1[1] = math.radians(60.0)
    stage_1[5] = 0.5
    stage_1[8] = math.radians(-60.0)
    stage_1[12] = -0.5
    execute_arm_target(
        trajectory, robot_state, "ready_shoulders", stage_1, 80)

    stage_2 = list(stage_1)
    stage_2[:4] = ready_target[:4]
    stage_2[7:11] = ready_target[7:11]
    execute_arm_target(
        trajectory, robot_state, "ready_upper_arms", stage_2, 80)
    execute_arm_target(
        trajectory, robot_state, "ready_wrists", ready_target, 80)


def execute_grasp_transition(
        trajectory, robot_state, gripper, grasp_target,
        grasp_command, points):
    errors = []
    duration = points * TRAJECTORY_SLEEP

    def close_hands():
        try:
            gripper.set_box_gripper_position(
                grasp_command, grasp_command, duration=duration)
        except Exception as error:
            errors.append(error)

    hand_thread = threading.Thread(target=close_hands)
    hand_thread.start()
    execute_arm_target(
        trajectory, robot_state, "grasp", grasp_target, points)
    hand_thread.join()
    if errors:
        raise errors[0]
    for side in ("left", "right"):
        if not gripper.wait_for_box_command(
                side, grasp_command, tolerance=0.12, timeout=4.0):
            raise RuntimeError(
                "{} hand did not reach whole-box grasp posture".format(
                    side))


def log_pose(label, chassis, poses, target_box):
    base = chassis.pose()
    box = poses.get_position(target_box)
    box_quaternion = poses.get_orientation(target_box)
    if base is None or box is None or box_quaternion is None:
        raise RuntimeError("Pose data unavailable while logging {}".format(label))
    box_yaw = yaw_from_xyzw(box_quaternion)
    rospy.loginfo(
        "%s: base=(%.3f, %.3f, %.1f deg) box=(%.3f, %.3f, %.3f, %.1f deg)",
        label, base[0], base[1], math.degrees(base[2]),
        box[0], box[1], box[2], math.degrees(box_yaw))
    return base, box, box_yaw


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Grasp and lift box_2, then optionally validate loaded backward "
            "motion and a relative turn. No placement is performed."))
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--backward", type=positive, default=0.15)
    parser.add_argument("--turn-deg", type=float, default=90.0)
    parser.add_argument("--linear-speed", type=positive, default=0.08)
    parser.add_argument("--angular-speed", type=positive, default=0.15)
    parser.add_argument("--minimum-linear-speed", type=positive, default=0.06)
    parser.add_argument("--minimum-angular-speed", type=positive, default=0.06)
    parser.add_argument("--position-tolerance", type=positive, default=0.03)
    parser.add_argument("--yaw-tolerance-deg", type=positive, default=3.0)
    parser.add_argument("--timeout", type=positive, default=40.0)
    args = parser.parse_args()

    if args.linear_speed <= 0.0 or args.angular_speed <= 0.0:
        parser.error("linear and angular speeds must be positive")
    if args.minimum_linear_speed > args.linear_speed:
        parser.error("minimum linear speed cannot exceed linear speed")
    if args.minimum_angular_speed > args.angular_speed:
        parser.error("minimum angular speed cannot exceed angular speed")

    with open(args.config, "r", encoding="utf-8") as stream:
        config = json.load(stream)
    if config.get("schema_version") != 1:
        raise RuntimeError("Unsupported Task 2 grasp config schema")
    if not config.get("approved_for_task2", False):
        raise RuntimeError(
            "Task 2 motion config is not approved; refusing robot motion")

    if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
        raise RuntimeError("KuavoSDK initialization failed")

    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    initial_arm = wait_for_arm_state(robot_state)
    trajectory = TrajectoryController(
        robot, initial_positions=initial_arm)
    gripper = GripperController()
    poses = ObjectPose()
    chassis = ChassisMotion(
        linear_speed=args.linear_speed,
        angular_speed=args.angular_speed,
        minimum_linear_speed=args.minimum_linear_speed,
        minimum_angular_speed=args.minimum_angular_speed,
        position_tolerance=args.position_tolerance,
        yaw_tolerance_deg=args.yaw_tolerance_deg,
    )

    target_box = config["target_box"]
    base_target = config["base_pose_world"]
    pregrasp_command = 255.0 * config["hand_closure"]["pregrasp"]
    grasp_command = 255.0 * config["hand_closure"]["grasp"]
    initial_box = None

    try:
        poses.wait_for_position(target_box, timeout=30.0)
        chassis.wait_until_ready(timeout=30.0)
        log_pose("initial", chassis, poses, target_box)

        chassis.move_to_pose(
            base_target["x"], base_target["y"],
            math.radians(base_target["yaw_deg"]), timeout=args.timeout)
        execute_ready_stages(
            trajectory, robot_state, arm_target(config, "ready"))

        gripper.set_box_gripper_position(
            pregrasp_command, pregrasp_command, duration=1.5)
        for side in ("left", "right"):
            if not gripper.wait_for_box_command(
                    side, pregrasp_command, tolerance=0.12, timeout=4.0):
                raise RuntimeError(
                    "{} hand did not reach pregrasp posture".format(side))

        execute_arm_target(
            trajectory, robot_state, "pregrasp",
            arm_target(config, "pregrasp"), 160)
        _, initial_box, _ = log_pose(
            "pregrasp", chassis, poses, target_box)

        execute_grasp_transition(
            trajectory, robot_state, gripper,
            arm_target(config, "grasp"), grasp_command, 120)
        time.sleep(0.5)
        log_pose("grasped", chassis, poses, target_box)

        execute_arm_target(
            trajectory, robot_state, "lift",
            arm_target(config, "lift"), 120)
        time.sleep(0.8)
        lifted_base, lifted_box, lifted_box_yaw = log_pose(
            "lifted", chassis, poses, target_box)
        measured_lift = lifted_box[2] - initial_box[2]
        minimum_lift = max(0.12, config["lift_height_m"] - 0.06)
        if measured_lift < minimum_lift:
            raise RuntimeError(
                "Box lift was {:.3f} m; expected at least {:.3f} m".format(
                    measured_lift, minimum_lift))

        if args.backward:
            chassis.translate_relative(-args.backward, timeout=args.timeout)
        if args.turn_deg:
            chassis.rotate_relative(
                math.radians(args.turn_deg), timeout=args.timeout)
        time.sleep(0.8)
        final_base, final_box, final_box_yaw = log_pose(
            "loaded_motion_complete", chassis, poses, target_box)

        base_yaw_delta = normalize_angle(final_base[2] - lifted_base[2])
        box_yaw_delta = normalize_angle(final_box_yaw - lifted_box_yaw)
        relative_yaw_error = normalize_angle(box_yaw_delta - base_yaw_delta)
        height_loss = lifted_box[2] - final_box[2]
        if abs(relative_yaw_error) > math.radians(5.0):
            raise RuntimeError(
                "Held box yaw did not follow base: error {:.1f} deg".format(
                    math.degrees(relative_yaw_error)))
        if height_loss > 0.04:
            raise RuntimeError(
                "Held box lost {:.3f} m of height during loaded motion".format(
                    height_loss))

        rospy.loginfo(
            "TASK2 BOX LIFT VALIDATION PASSED: lift=%.3f m, "
            "base_yaw_delta=%.1f deg, box_yaw_delta=%.1f deg",
            measured_lift, math.degrees(base_yaw_delta),
            math.degrees(box_yaw_delta))
    finally:
        chassis.stop()
        gripper.stop()
        trajectory.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
