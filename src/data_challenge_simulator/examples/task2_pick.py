#!/usr/bin/env python3
"""Continue Task 2 from B1 through front docking and a lift probe."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool


CURRENT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = CURRENT_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
from utils.gripper_controller import GripperController
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


class BimanualGraspCommand:
    """Explicitly engage or release the Task 2 kinematic latch."""

    def __init__(self):
        self.enabled = False
        self.publisher = rospy.Publisher(
            "/mujoco/task2_grasp_enabled", Bool,
            queue_size=1, latch=True)

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        self.publisher.publish(Bool(data=self.enabled))


class RepositionGraspState:
    def __init__(self):
        self.held = {name: False for name in BOX_NAMES}
        self.subscribers = [
            rospy.Subscriber(
                "/mujoco/{}/reposition_grasped".format(name), Bool,
                self._callback, callback_args=name)
            for name in BOX_NAMES
        ]

    def _callback(self, message, box_name):
        self.held[box_name] = bool(message.data)

    def wait_for(self, box_name, expected, timeout):
        deadline = time.time() + float(timeout)
        rate = rospy.Rate(50)
        while time.time() < deadline and not rospy.is_shutdown():
            if self.held[box_name] == bool(expected):
                return True
            rate.sleep()
        return self.held[box_name] == bool(expected)


class RepositionGraspCommand:
    """Freeze both arms, both hands, and the selected box to the base."""

    def __init__(self):
        self.enabled = False
        self.publisher = rospy.Publisher(
            "/mujoco/task2_reposition_enabled", Bool,
            queue_size=1, latch=True)

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        self.publisher.publish(Bool(data=self.enabled))


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


def one_arm_target(measured_rad, solved_rad, side):
    """Keep the other seven measured joints unchanged."""
    measured = list(float(value) for value in measured_rad)
    solved = list(float(value) for value in solved_rad)
    if len(measured) != 14 or len(solved) != 14:
        raise ValueError("arm targets must contain 14 joints")
    if side == "left":
        measured[:7] = solved[:7]
    elif side == "right":
        measured[7:] = solved[7:]
    else:
        raise ValueError("side must be left or right")
    return measured


def execute_lift_joint_path(
        trajectory, label, waypoints_rad, point_count):
    """Interpolate from grasp joints to the confirmed terminal-pose IK."""
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


def main(
        runtime=None, transport_and_return=None,
        reposition_before_right=True):
    parser = argparse.ArgumentParser(
        description=(
            "Continue from stored Task2 B1, select and dock in front of one "
            "box, then execute live-pose bimanual IK grasp and lift probe."))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--linear-speed", type=positive, default=0.08)
    parser.add_argument("--minimum-linear-speed", type=positive, default=0.06)
    parser.add_argument("--position-tolerance", type=positive, default=0.03)
    parser.add_argument("--motion-timeout", type=positive, default=40.0)
    parser.add_argument("--transport-and-return", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if transport_and_return is not None:
        args.transport_and_return = bool(transport_and_return)
    if args.minimum_linear_speed > args.linear_speed:
        parser.error("minimum linear speed cannot exceed linear speed")

    config = load_config(args.config)
    # Same seed the workflow entry used to place both boxes; this stage only
    # reads the saved docking base poses out of it.
    saved_layout = Task2RandomizationPlanner().plan(
        int(os.environ.get("TASK_SEED", "1")))
    owns_runtime = runtime is None
    if owns_runtime:
        rospy.init_node("task2_pick", anonymous=False)
        if not KuavoSDK().Init(options=KuavoSDK.Options.Normal):
            raise RuntimeError("KuavoSDK initialization failed")
        robot = KuavoRobot()
        robot_state = KuavoRobotState()
        poses = ObjectPose()
        grasp_state = BimanualGraspState()
        grasp_command = BimanualGraspCommand()
        reposition_state = RepositionGraspState()
        reposition_command = RepositionGraspCommand()
        conveyor_command = ConveyorCommand()
        chassis = ChassisMotion(
            linear_speed=args.linear_speed,
            angular_speed=0.20,
            minimum_linear_speed=args.minimum_linear_speed,
            minimum_angular_speed=0.06,
            position_tolerance=args.position_tolerance,
            yaw_tolerance_deg=3.0,
        )
        planner = Task2PickPlanner(ik_config=config["ik"])
        gripper = None
        # Clear a latch left armed by an interrupted previous process.
        grasp_command.set_enabled(False)
        reposition_command.set_enabled(False)
    else:
        robot = runtime.robot
        robot_state = runtime.robot_state
        poses = runtime.poses
        grasp_state = runtime.grasp_state
        grasp_command = runtime.grasp_command
        reposition_state = runtime.reposition_state
        reposition_command = runtime.reposition_command
        conveyor_command = runtime.conveyor_command
        chassis = runtime.chassis
        planner = runtime.planner
        gripper = runtime.gripper
    # A caller that owns publishers for the whole workflow passes them in;
    # this stage must then neither build nor stop either one between its own
    # steps, because stopping them opens holes in the recorded command topics.
    shared_trajectory = None if owns_runtime else runtime.trajectory
    owns_trajectory = shared_trajectory is None
    shared_gripper = None if owns_runtime else runtime.gripper
    owns_gripper = shared_gripper is None
    trajectory = None
    grasp_confirmed = False
    try:
        stored_b1, initialized_seed = wait_for_b1_parameters()
        chassis.wait_until_ready(timeout=30.0)
        for name in BOX_NAMES:
            poses.wait_for_position(name, timeout=30.0)
        live_boxes = read_box_poses(poses)

        # The chassis drives to the docking base pose the saved layout
        # verified offline.  Which box to dock is still decided from the live
        # poses, so a box already delivered to the conveyor is never selected.
        docking_bases = {
            box.name: box.docking_base for box in saved_layout.boxes}
        docking = select_front_docking_target(
            stored_b1, live_boxes, config["docking"], docking_bases)
        rospy.set_param("/task2_selected_box", docking.box_name)
        rospy.loginfo(
            "Task2 docking selection at B1=(%.3f, %.3f, %.1f deg): "
            "%s target=(%.3f, %.3f, %.1f deg), cost=%.3f m, "
            "already_comfortable=%s",
            stored_b1[0], stored_b1[1], math.degrees(stored_b1[2]),
            docking.box_name, docking.x, docking.y,
            math.degrees(docking.yaw), docking.cost,
            docking.already_comfortable)
        if args.dry_run:
            return

        if not docking.already_comfortable:
            chassis.move_to_pose(
                docking.x, docking.y, docking.yaw,
                timeout=args.motion_timeout)

        # The selected box is locked here.  All IK below uses only this box as
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
        if reposition_before_right:
            execute_arm_waypoint(
                trajectory, robot_state, "left_above",
                one_arm_target(measured_seed, plan.above_joints, "left"),
                config["grasp"]["above_trajectory_points"])

            # Repeat the accepted Task2 approach/open/descent/close sequence
            # for the left arm and hand without moving the right side.
            if gripper is None:
                gripper = GripperController()
            gripper.control_left_box_gripper(
                open_command,
                duration=float(config["grasp"]["gripper_duration_s"]))

            left_above_measured = tuple(wait_for_arm_state(robot_state))
            execute_arm_waypoint(
                trajectory, robot_state, "left_vertical_descent",
                one_arm_target(
                    left_above_measured, plan.contact_joints, "left"),
                config["grasp"]["descent_trajectory_points"])
            reposition_command.set_enabled(True)
            rospy.sleep(0.05)
            gripper.control_left_box_gripper(
                grasp_command_value,
                duration=float(config["grasp"]["gripper_duration_s"]))
            if not reposition_state.wait_for(
                    docking.box_name, True,
                    config["grasp"]["latch_confirmation_timeout_s"]):
                raise RuntimeError(
                    "Task2 left grasp failed before repositioning {}".format(
                        docking.box_name))

            # The Task2-only MuJoCo latch keeps both arms, both hands, and the
            # box fixed.  Shared publishers are not ours to stop: the latch
            # holds their joints regardless, and stopping either publisher
            # opens a silent window in the recorded bag.
            if owns_trajectory:
                trajectory.stop()
                trajectory = None
            if owns_gripper:
                gripper.stop()
                gripper = None
            reposition = config["reposition"]
            current_box_y = float(poses.get_position(docking.box_name)[1])
            target_box_y = (
                float(reposition["source_table_left_edge_y_m"]) -
                float(reposition["box_half_width_y_m"]))
            lateral_distance = target_box_y - current_box_y
            if lateral_distance < -args.position_tolerance:
                raise RuntimeError(
                    "Task2 reposition target is not to the robot's left")
            current_base = tuple(chassis.pose())
            rospy.loginfo(
                "Task2 silent locked reposition: box=%s left_distance=%.3f m",
                docking.box_name, lateral_distance)
            chassis.move_to_pose(
                current_base[0], current_base[1] + lateral_distance,
                current_base[2], timeout=args.motion_timeout)

            reposition_command.set_enabled(False)
            if not reposition_state.wait_for(docking.box_name, False, 1.0):
                raise RuntimeError(
                    "Task2 reposition latch release was not acknowledged")

            # Recreate stage-owned publishers only after the relocation latch
            # is gone.  Workflow-owned publishers stayed alive throughout.
            shifted_seed = tuple(wait_for_arm_state(robot_state))
            trajectory = shared_trajectory
            if owns_trajectory:
                trajectory = TrajectoryController(
                    robot, initial_positions=shifted_seed)
            else:
                trajectory.set_target_positions(list(shifted_seed))
            gripper = shared_gripper
            if owns_gripper:
                gripper = GripperController()
            execute_arm_waypoint(
                trajectory, robot_state, "right_above",
                one_arm_target(shifted_seed, plan.above_joints, "right"),
                config["grasp"]["above_trajectory_points"])
            gripper.control_right_box_gripper(
                open_command,
                duration=float(config["grasp"]["gripper_duration_s"]))
            right_above_measured = tuple(wait_for_arm_state(robot_state))
            execute_arm_waypoint(
                trajectory, robot_state, "right_vertical_descent",
                one_arm_target(
                    right_above_measured, plan.contact_joints, "right"),
                config["grasp"]["descent_trajectory_points"])
            grasp_command.set_enabled(True)
            rospy.sleep(0.05)
            gripper.control_right_box_gripper(
                grasp_command_value,
                duration=float(config["grasp"]["gripper_duration_s"]))
        else:
            # The second box follows the accepted Task2 grasp unchanged.
            execute_arm_waypoint(
                trajectory, robot_state, "above",
                plan.above_joints,
                config["grasp"]["above_trajectory_points"])
            if gripper is None:
                gripper = GripperController()
            gripper.set_box_gripper_position(
                open_command, open_command,
                duration=float(config["grasp"]["gripper_duration_s"]))
            execute_arm_waypoint(
                trajectory, robot_state, "vertical_descent",
                plan.contact_joints,
                config["grasp"]["descent_trajectory_points"])
            grasp_command.set_enabled(True)
            rospy.sleep(0.05)
            gripper.set_box_gripper_position(
                grasp_command_value, grasp_command_value,
                duration=float(config["grasp"]["gripper_duration_s"]))
        if not grasp_state.wait_until_held(
                docking.box_name,
                config["grasp"]["latch_confirmation_timeout_s"]):
            raise RuntimeError(
                "Task2 grasp failed: kinematic latch did not engage for {}".format(
                    docking.box_name))
        rospy.set_param("/task2_grasped_box", docking.box_name)
        grasp_confirmed = True
        rospy.loginfo(
            "TASK2 TWO-SIDED GRASP LATCHED: box=%s", docking.box_name)
        latched_hand_box = read_hand_box_transforms(
            poses, docking.box_name)

        execute_lift_joint_path(
            trajectory, "lift_to_confirmed_pose",
            plan.lift_waypoints,
            config["grasp"]["lift_trajectory_points"])
        # Keep publishing the final lift target long enough for the measured
        # arm state to catch up before evaluating the visible rise.
        rospy.sleep(float(config["grasp"]["settle_time_s"]))
        lifted_hand_box = read_hand_box_transforms(
            poses, docking.box_name)
        log_hand_box_transform_change(
            "LIFT", latched_hand_box, lifted_hand_box)
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

        if args.transport_and_return:
            transport = config["transport"]
            rospy.loginfo("Task2 loaded transport: retreat from source table")
            chassis.translate_relative(
                -float(transport["clearance_retreat_m"]),
                timeout=args.motion_timeout)
            chassis.rotate_relative(
                math.radians(float(transport["turn_deg"])),
                timeout=args.motion_timeout)
            destination = tuple(
                float(value) for value in
                transport["destination_base_pose"])
            chassis.move_to_pose(
                destination[0], destination[1], destination[2],
                timeout=args.motion_timeout)

            rospy.loginfo(
                "Task2 placing %s on destination conveyor",
                docking.box_name)
            placed_count = int(rospy.get_param(
                "/task2_placed_box_count", 0))
            is_first_box = placed_count == 0
            grasp_command.set_enabled(False)
            grasp_confirmed = False
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
                    "Task2 explicit release was not acknowledged for {}".format(
                        docking.box_name))
            rospy.sleep(float(transport["place_settle_time_s"]))
            rospy.loginfo(
                "Task2 release settled; returning both arms to the stored "
                "ready posture before chassis motion")
            execute_arm_waypoint(
                trajectory, robot_state, "post_release_ready",
                initialized_seed,
                transport["release_ready_trajectory_points"])
            if is_first_box:
                rospy.set_param("/task2_conveyor_box", docking.box_name)
                rospy.set_param("/task2_conveyor_complete", False)
                conveyor_command.set_enabled(True)
            rospy.set_param(
                "/task2_placed_box_count", placed_count + 1)
            rospy.loginfo(
                "Task2 direct release complete: box=%s conveyor=%s",
                docking.box_name,
                "armed-after-contact" if is_first_box else "unchanged")

            rospy.loginfo("Task2 returning to the source start")
            if not is_first_box:
                chassis.translate_relative(
                    -float(transport["clearance_retreat_m"]),
                    timeout=args.motion_timeout)
            chassis.rotate_relative(
                -math.radians(float(transport["turn_deg"])),
                timeout=args.motion_timeout)
            chassis.move_to_pose(
                stored_b1[0], stored_b1[1], stored_b1[2],
                timeout=args.motion_timeout)
            rospy.loginfo(
                "TASK2 BOX PLACED AND RETURNED: box=%s",
                docking.box_name)
    finally:
        if reposition_command.enabled:
            reposition_command.set_enabled(False)
        if grasp_command.enabled and not grasp_confirmed:
            grasp_command.set_enabled(False)
        chassis.stop()
        if owns_runtime and gripper is not None:
            gripper.stop()
        if owns_trajectory and trajectory is not None:
            trajectory.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
