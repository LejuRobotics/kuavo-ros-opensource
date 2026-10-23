#!/usr/bin/env python3
"""Continue Task 2 from B1 through front docking and a lift probe."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, UInt8
from geometry_msgs.msg import PoseStamped


CURRENT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = CURRENT_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
from utils.gripper_controller import GripperController
from utils.hand_postures import box_gripper_targets
from utils.object_pos import ObjectPose
from utils.task2_pick_planner import (
    BOX_NAMES,
    Task2PickPlanner,
    select_front_docking_target,
)
from utils.task2_randomization import Task2RandomizationPlanner
from utils.trajectory_controller import TrajectoryController
from utils.utils import Utils

from task2_base_motion import ChassisMotion
from task2_initialize import wait_for_arm_state


DEFAULT_CONFIG = PACKAGE_DIR / "config/task2_pick.json"
TRAJECTORY_SLEEP = 0.02


class BimanualGraspState:
    def __init__(self):
        self.held = {name: False for name in BOX_NAMES}
        self.subscribers = [
            rospy.Subscriber(
                "/mujoco/{}/bimanual_grasped".format(name), Bool,
                self._callback, callback_args=name)
            for name in BOX_NAMES
        ]

    def _callback(self, message, box_name):
        self.held[box_name] = bool(message.data)

    def wait_until_held(self, box_name, timeout):
        deadline = time.time() + float(timeout)
        rate = rospy.Rate(50)
        while time.time() < deadline and not rospy.is_shutdown():
            if self.held[box_name]:
                return True
            rate.sleep()
        return self.held[box_name]

    def wait_until_released(self, box_name, timeout):
        deadline = time.time() + float(timeout)
        rate = rospy.Rate(50)
        while time.time() < deadline and not rospy.is_shutdown():
            if not self.held[box_name]:
                return True
            rate.sleep()
        return not self.held[box_name]


class ConveyorCommand:
    def __init__(self):
        self.publisher = rospy.Publisher(
            "/mujoco/task2_conveyor_enabled", Bool,
            queue_size=1, latch=True)

    def set_enabled(self, enabled):
        self.publisher.publish(Bool(data=bool(enabled)))


def positive(value):
    value = float(value)
    if value <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def load_config(path):
    with path.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    if config.get("schema_version") != 1:
        raise RuntimeError("unsupported Task2 pick config schema")
    return config


def read_box_poses(poses):
    result = {}
    for name in BOX_NAMES:
        position = poses.get_position(name)
        orientation = poses.get_orientation(name)
        if position is None or orientation is None:
            raise RuntimeError("live pose is unavailable for {}".format(name))
        result[name] = (tuple(position), tuple(orientation))
    return result


def read_hand_box_transforms(poses, box_name):
    """Read both hand_base transforms in the selected box frame."""
    box_position = poses.get_position(box_name)
    box_orientation = poses.get_orientation(box_name)
    if box_position is None or box_orientation is None:
        raise RuntimeError("live pose is unavailable for {}".format(box_name))
    box_rotation = Rotation.from_quat(box_orientation)
    transforms = {}
    for side in ("l", "r"):
        body_name = "{}_hand_base".format(side)
        hand_position = poses.get_position(body_name)
        hand_orientation = poses.get_orientation(body_name)
        if hand_position is None or hand_orientation is None:
            raise RuntimeError(
                "live pose is unavailable for {}".format(body_name))
        relative_position = box_rotation.inv().apply(
            np.asarray(hand_position) - np.asarray(box_position))
        relative_rotation = (
            box_rotation.inv() * Rotation.from_quat(hand_orientation))
        transforms[side] = (relative_position, relative_rotation)
    return transforms


def log_hand_box_transform_change(label, before, after):
    for side in ("l", "r"):
        before_position, before_rotation = before[side]
        after_position, after_rotation = after[side]
        position_delta = after_position - before_position
        orientation_delta_deg = math.degrees(np.linalg.norm(
            (before_rotation.inv() * after_rotation).as_rotvec()))
        rospy.loginfo(
            "TASK2 HAND-BOX %s: side=%s before=(%.6f,%.6f,%.6f) "
            "after=(%.6f,%.6f,%.6f) delta=(%.6f,%.6f,%.6f) "
            "orientation_delta=%.6f deg",
            label, side,
            before_position[0], before_position[1], before_position[2],
            after_position[0], after_position[1], after_position[2],
            position_delta[0], position_delta[1], position_delta[2],
            orientation_delta_deg)


def execute_arm_waypoint(
        trajectory, robot_state, label, target_rad, point_count):
    measured = wait_for_arm_state(robot_state)
    rospy.loginfo("Task2 arm stage %s: %d points", label, point_count)
    trajectory.execute_trajectory(
        Utils.interpolate_joint_trajectory(
            [math.degrees(value) for value in target_rad],
            [math.degrees(value) for value in measured],
            num=int(point_count),
        ),
        sleep_time=TRAJECTORY_SLEEP,
    )


def wait_for_hand_targets(targets, tolerance=0.015, timeout=2.0):
    """Advance once fresh hand poses reach the target and stop oscillating."""
    started = time.monotonic()
    previous = None
    consecutive = 0
    error = float("inf")
    while not rospy.is_shutdown():
        measured = []
        for side in ("l", "r"):
            remaining = timeout - (time.monotonic() - started)
            if remaining <= 0:
                raise RuntimeError(
                    "Task2 hands did not settle at target: max_error={:.4f} m".format(error))
            message = rospy.wait_for_message(
                "/mujoco/{}_hand_base/pose".format(side), PoseStamped,
                timeout=remaining)
            position = message.pose.position
            measured.append([position.x, position.y, position.z])
        measured = np.asarray(measured)
        error = float(np.max(np.linalg.norm(measured - targets, axis=1)))
        stable = (previous is not None and
                  np.max(np.linalg.norm(measured - previous, axis=1)) <= 0.002)
        consecutive = consecutive + 1 if error <= tolerance and stable else 0
        if consecutive >= 3:
            rospy.loginfo("Task2 hands arrived: error=%.4f m after %.3f s",
                          error, time.monotonic() - started)
            return
        previous = measured
        rospy.sleep(0.02)
    raise rospy.ROSInterruptException()


def box_grippers_are_open(gripper, command):
    for side in ("left", "right"):
        measured = gripper.measured_positions(side)
        target = box_gripper_targets(side[0], command / 255.0)
        if (measured is None or len(measured) != len(target) or
                not all(math.isfinite(a) and abs(a - b) <= 0.10
                        for a, b in zip(measured, target))):
            return False
    return True


def execute_lift_joint_path(
        trajectory, label, waypoints_rad, point_count):
    """Execute Cartesian-solved descent or lift waypoints in joint space."""
    waypoints = [tuple(float(value) for value in point)
                 for point in waypoints_rad]
    if len(waypoints) < 2 or any(len(point) != 14 for point in waypoints):
        raise ValueError(
            "lift path must contain at least two 14-joint waypoints")
    segment_count = len(waypoints) - 1
    base_points, remainder = divmod(int(point_count), segment_count)
    if base_points < 1:
        raise ValueError("point_count is too small for the lift path")
    commands_rad = []
    for segment in range(segment_count):
        count = base_points + (1 if segment < remainder else 0)
        start = waypoints[segment]
        end = waypoints[segment + 1]
        for step in range(1, count + 1):
            ratio = float(step) / float(count)
            commands_rad.append([
                a + ratio * (b - a)
                for a, b in zip(start, end)
            ])
    rospy.loginfo(
        "Task2 arm stage %s: %d commands through %d joint-space waypoints",
        label, len(commands_rad), len(waypoints))
    trajectory.execute_trajectory(commands_rad, sleep_time=TRAJECTORY_SLEEP)


def wait_for_b1_parameters():
    try:
        returned_pose = tuple(float(value) for value in rospy.get_param(
            "/task2_returned_base_pose"))
        initialized_seed = tuple(float(value) for value in rospy.get_param(
            "/task2_initialized_arm_seed"))
    except KeyError as error:
        raise RuntimeError(
            "Task2 B1 is missing; run task2_initialize.py first") from error
    if len(returned_pose) != 3 or len(initialized_seed) != 14:
        raise RuntimeError("stored Task2 B1 or arm seed has invalid length")
    return returned_pose, initialized_seed


def wait_for_conveyor_contact(poses, box_name, maximum_z_m, timeout_s):
    """Confirm real top-surface support, using the same live sensor as scoring."""
    deadline = time.monotonic() + float(timeout_s)
    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        message = rospy.wait_for_message(
            "/mujoco/{}/scoring_state".format(box_name), UInt8, timeout=remaining)
        if message.data & 4:
            position = poses.get_position(box_name)
            rospy.loginfo("Task2 conveyor contact: box=%s position=%s", box_name, position)
            return position
        rospy.sleep(0.02)
    raise RuntimeError("Task2 box {} has no conveyor top support after {:.1f} s".format(
        box_name, timeout_s))


def main(runtime=None, transport_and_return=None):
    parser = argparse.ArgumentParser(
        description=(
            "Continue from stored Task2 B1, select and dock in front of one "
            "box, then execute live-pose bimanual IK grasp and lift probe."))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--linear-speed", type=positive, default=None)
    parser.add_argument("--minimum-linear-speed", type=positive, default=None)
    parser.add_argument("--angular-speed", type=positive, default=None)
    parser.add_argument("--minimum-angular-speed", type=positive, default=None)
    parser.add_argument("--position-tolerance", type=positive, default=0.03)
    parser.add_argument("--motion-timeout", type=positive, default=40.0)
    parser.add_argument("--transport-and-return", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if transport_and_return is not None:
        args.transport_and_return = bool(transport_and_return)

    config = load_config(args.config)
    if args.linear_speed is None:
        args.linear_speed = float(config["chassis"]["linear_speed_mps"])
    if args.minimum_linear_speed is None:
        args.minimum_linear_speed = float(
            config["chassis"]["minimum_linear_speed_mps"])
    if args.angular_speed is None:
        args.angular_speed = float(config["chassis"]["angular_speed_radps"])
    if args.minimum_angular_speed is None:
        args.minimum_angular_speed = float(
            config["chassis"]["minimum_angular_speed_radps"])
    if args.minimum_linear_speed > args.linear_speed:
        parser.error("minimum linear speed cannot exceed linear speed")
    if args.minimum_angular_speed > args.angular_speed:
        parser.error("minimum angular speed cannot exceed angular speed")
    # Use the same saved box layout as initialization, with the configured
    # stand-off providing room for an in-place loaded turn.
    saved_layout = Task2RandomizationPlanner().plan(
        int(os.environ.get("TASK_SEED", "28")))
    owns_runtime = runtime is None
    if owns_runtime:
        rospy.init_node("task2_pick", anonymous=False)
        if not KuavoSDK().Init(options=KuavoSDK.Options.Normal):
            raise RuntimeError("KuavoSDK initialization failed")
        robot = KuavoRobot()
        robot_state = KuavoRobotState()
        poses = ObjectPose()
        grasp_state = BimanualGraspState()
        conveyor_command = ConveyorCommand()
        chassis = ChassisMotion(
            linear_speed=args.linear_speed,
            angular_speed=args.angular_speed,
            minimum_linear_speed=args.minimum_linear_speed,
            minimum_angular_speed=args.minimum_angular_speed,
            position_tolerance=args.position_tolerance,
            yaw_tolerance_deg=3.0,
        )
        planner = Task2PickPlanner(ik_config=config["ik"])
        gripper = None
    else:
        robot = runtime.robot
        robot_state = runtime.robot_state
        poses = runtime.poses
        grasp_state = runtime.grasp_state
        conveyor_command = runtime.conveyor_command
        chassis = runtime.chassis
        planner = runtime.planner
        gripper = runtime.gripper
    chassis.settle_on_velocity = True
    # A caller that owns publishers for the whole workflow passes them in;
    # this stage must then neither build nor stop either one between its own
    # steps, because stopping them opens holes in the recorded command topics.
    shared_trajectory = None if owns_runtime else runtime.trajectory
    owns_trajectory = shared_trajectory is None
    trajectory = None
    motion_worker = ThreadPoolExecutor(max_workers=1)
    try:
        _, initialized_seed = wait_for_b1_parameters()
        chassis.wait_until_ready(timeout=30.0)
        for name in BOX_NAMES:
            poses.wait_for_position(name, timeout=30.0)
        live_boxes = read_box_poses(poses)

        # Live box poses select undelivered boxes and their approach targets.
        standoff = float(config["docking"]["front_standoff_m"])
        docking_bases = {
            box.name: (
                live_boxes[box.name][0][0] - standoff * math.cos(box.yaw),
                live_boxes[box.name][0][1] - standoff * math.sin(box.yaw), box.yaw)
            for box in saved_layout.boxes}
        current_base = tuple(chassis.pose())
        docking = select_front_docking_target(
            current_base, live_boxes, config["docking"], docking_bases)
        rospy.set_param("/task2_selected_box", docking.box_name)
        rospy.loginfo(
            "Task2 docking selection at current base=(%.3f, %.3f, %.1f deg): "
            "%s target=(%.3f, %.3f, %.1f deg), cost=%.3f m, "
            "already_comfortable=%s",
            current_base[0], current_base[1], math.degrees(current_base[2]),
            docking.box_name, docking.x, docking.y,
            math.degrees(docking.yaw), docking.cost,
            docking.already_comfortable)
        if args.dry_run:
            return

        if not docking.already_comfortable:
            chassis.move_to_pose(
                docking.x, docking.y, docking.yaw,
                timeout=args.motion_timeout)

        # Fix the target selection only. All IK below uses this box as
        # the target and uses the newly measured stopped chassis/object state.
        docked_base = tuple(chassis.pose())
        docked_boxes = read_box_poses(poses)
        measured_seed = tuple(wait_for_arm_state(robot_state))
        planning_started = time.perf_counter()
        rospy.loginfo(
            "Task2 planning bimanual IK grasp waypoints")
        plan = planner.plan(
            docking.box_name, docked_base, docked_boxes,
            initialized_seed, config["grasp"],
            execution_arm_start=measured_seed)
        rospy.loginfo(
            "Task2 bimanual IK plan solved before arm motion: box=%s, "
            "planning_time=%.2f s",
            docking.box_name,
            time.perf_counter() - planning_started)

        if any(abs(a - b) > 1e-6 for a, b in zip(
                initialized_seed, measured_seed)):
            rospy.loginfo(
                "IK used the stored B1 arm seed; execution starts at the "
                "post-docking measured arm state")

        open_command = 255.0 * float(config["grasp"]["open_closure"])
        grasp_command_value = (
            255.0 * float(config["grasp"]["grasp_closure"]))
        other_box = next(
            name for name in BOX_NAMES if name != docking.box_name)
        trajectory = shared_trajectory
        if owns_trajectory:
            trajectory = TrajectoryController(
                robot, initial_positions=measured_seed)
        else:
            trajectory.set_target_positions(list(measured_seed))
        if gripper is None:
            gripper = GripperController()
        opening = None
        if not box_grippers_are_open(gripper, open_command):
            opening = motion_worker.submit(
                gripper.set_box_gripper_position, open_command, open_command,
                duration=float(config["grasp"]["gripper_duration_s"]))
        execute_arm_waypoint(
            trajectory, robot_state, "above",
            plan.above_joints,
            config["grasp"]["above_trajectory_points"])
        if opening is not None:
            opening.result()
        execute_lift_joint_path(
            trajectory, "vertical_descent",
            plan.descent_waypoints,
            config["grasp"]["descent_trajectory_points"])
        wait_for_hand_targets(planner.hand_positions(
            plan.descent_waypoints[-1], docked_base, docked_boxes))
        gripper.set_box_gripper_position(
            grasp_command_value, grasp_command_value,
            duration=float(config["grasp"]["gripper_duration_s"]))
        if not grasp_state.wait_until_held(
                docking.box_name,
                config["grasp"]["contact_confirmation_timeout_s"]):
            raise RuntimeError(
                "Task2 grasp failed: no force-bearing contacts on both hands for {}".format(
                    docking.box_name))
        rospy.set_param("/task2_grasped_box", docking.box_name)
        rospy.loginfo(
            "TASK2 TWO-SIDED PHYSICAL CONTACT: box=%s", docking.box_name)
        contact_hand_box = read_hand_box_transforms(
            poses, docking.box_name)

        execute_lift_joint_path(
            trajectory, "lift_to_confirmed_pose",
            plan.lift_waypoints,
            config["grasp"]["lift_trajectory_points"])
        wait_for_hand_targets(planner.hand_positions(
            plan.lift_waypoints[-1], docked_base, docked_boxes), tolerance=0.02)
        lifted_hand_box = read_hand_box_transforms(
            poses, docking.box_name)
        log_hand_box_transform_change(
            "LIFT", contact_hand_box, lifted_hand_box)
        final_boxes = read_box_poses(poses)
        selected_rise = (
            final_boxes[docking.box_name][0][2] -
            docked_boxes[docking.box_name][0][2])
        other_rise = (
            final_boxes[other_box][0][2] - docked_boxes[other_box][0][2])
        rospy.loginfo(
            "TASK2 LIFT COMPLETE: box=%s selected_rise=%.3f m, "
            "unselected_rise=%.3f m",
            docking.box_name, selected_rise, other_rise)

        minimum_rise = float(config["grasp"]["minimum_lift_m"])
        if other_rise >= minimum_rise:
            raise RuntimeError(
                "Task2 grasp disturbed the unselected box: {} rose {:.3f} m".format(
                    other_box, other_rise))
        if selected_rise < minimum_rise or not grasp_state.held[docking.box_name]:
            raise RuntimeError(
                "Task2 physical lift failed: {} rise={:.3f} m, two_hand_contact={}".format(
                    docking.box_name, selected_rise, grasp_state.held[docking.box_name]))
        rospy.loginfo("TASK2 TWO-HAND LIFT VERIFIED: box=%s", docking.box_name)

        if args.transport_and_return:
            transport = config["transport"]
            rospy.loginfo(
                "Task2 loaded transport: clear the table legs, turn, then direct to conveyor")
            chassis.move_to_pose(
                min(docked_base[0], float(transport["loaded_turn_max_base_x_m"])),
                docked_base[1], docked_base[2],
                timeout=args.motion_timeout, straight_line=True)
            transport_base = chassis.pose()
            chassis.rotate_relative(
                math.radians(float(transport["turn_deg"])),
                timeout=args.motion_timeout)
            # Keep the cleared turning X: after the quarter turn, drive
            # forward to the right side of the belt without a sideways leg.
            # Keep the direct route where safe. Shift the target only if the
            # translated belt would leave the held box too close to a rail.
            box_position, box_quaternion = read_box_poses(poses)[docking.box_name]
            box_rotation = Rotation.from_quat(box_quaternion).as_matrix()
            half_width = float(np.abs(box_rotation[0]) @ np.array([0.32, 0.22, 0.06]))
            belt_x_min, belt_x_max = saved_layout.conveyor_bounds[:2]
            safe_min, safe_max = belt_x_min + half_width + 0.06, belt_x_max - half_width - 0.06
            if safe_min > safe_max:
                raise RuntimeError("Held Task2 box does not fit the conveyor")
            target_box_x = min(max(box_position[0], safe_min), safe_max)
            current_base = chassis.pose()
            destination = (
                current_base[0] + target_box_x - box_position[0],
                float(transport["destination_base_y_m"]), math.pi / 2)
            chassis.move_to_pose(
                destination[0], destination[1], destination[2],
                timeout=args.motion_timeout, straight_line=True)

            rospy.loginfo(
                "Task2 placing %s on destination conveyor",
                docking.box_name)
            placed_count = int(rospy.get_param(
                "/task2_placed_box_count", 0))
            is_first_box = placed_count == 0
            gripper.set_box_gripper_position(
                open_command, open_command,
                duration=float(config["grasp"]["gripper_duration_s"]))
            for side in ("left", "right"):
                if not gripper.wait_for_box_command(
                        side, open_command, tolerance=0.12, timeout=4.0):
                    raise RuntimeError(
                        "{} hand failed to open for placement".format(side))
            if not grasp_state.wait_until_released(
                    docking.box_name, timeout=1.0):
                raise RuntimeError(
                    "Task2 hand contact persisted after opening for {}".format(
                        docking.box_name))
            wait_for_conveyor_contact(
                poses, docking.box_name,
                transport["place_contact_max_z_m"],
                transport["place_contact_timeout_s"])
            if is_first_box:
                rospy.loginfo(
                    "Task2 first release settled; returning both arms to "
                    "the stored ready posture for the second pick")
                returning_arms = motion_worker.submit(
                    execute_arm_waypoint,
                    trajectory, robot_state, "post_release_ready",
                    initialized_seed,
                    transport["release_ready_trajectory_points"])
                rospy.set_param("/task2_conveyor_box", docking.box_name)
                rospy.set_param("/task2_conveyor_complete", False)
            else:
                rospy.loginfo(
                    "Task2 final box contacted conveyor; ending task without "
                    "arm reset or chassis return")
            rospy.set_param(
                "/task2_placed_box_count", placed_count + 1)
            rospy.loginfo(
                "Task2 direct release complete: box=%s conveyor=%s",
                docking.box_name,
                "awaiting-arm-clearance" if is_first_box else "unchanged")

            if not is_first_box:
                rospy.loginfo(
                    "TASK2 FINAL BOX CONTACTED CONVEYOR: box=%s",
                    docking.box_name)
                return

            rospy.loginfo("Task2 returning directly to the next box")
            # Clear the table/conveyor legs along the arrival line before
            # sweeping the chassis and wheels through the return turn.
            released_base = chassis.pose()
            chassis.move_to_pose(
                released_base[0], float(transport["return_turn_base_y_m"]),
                released_base[2], timeout=args.motion_timeout,
                straight_line=True)
            # Both actions move away from the released box. Join before the
            # return turn and before enabling the conveyor; propagate errors.
            returning_arms.result()
            conveyor_command.set_enabled(True)
            chassis.rotate_relative(
                -math.radians(float(transport["turn_deg"])),
                timeout=args.motion_timeout)
            next_docking = docking_bases[other_box]
            chassis.move_to_pose(
                next_docking[0], next_docking[1], next_docking[2],
                timeout=args.motion_timeout, straight_line=True)
            rospy.loginfo(
                "TASK2 BOX PLACED AND RETURNED: box=%s",
                docking.box_name)
    finally:
        chassis.stop()
        motion_worker.shutdown(wait=True)
        if owns_runtime and gripper is not None:
            gripper.stop()
        if owns_trajectory and trajectory is not None:
            trajectory.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
