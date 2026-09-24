#!/usr/bin/env python3
"""Descend slightly, then independently latch Task 3's internal fingers."""

import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import time

import numpy as np
import rospy
from std_msgs.msg import Bool, UInt8


CURRENT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = CURRENT_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
from utils.gripper_controller import GripperController
from utils.hand_postures import advance_internal_expansions
from utils.object_pos import ObjectPose
from utils.task3_second_ik import Task3SecondIKPlanner
from utils.trajectory_controller import TrajectoryController

from task2_base_motion import ChassisMotion
from task3_descend import execute_right_target, wait_for_held_arm_command
from task3_initialize import (
    DEFAULT_SCENE,
    wait_for_arm_state,
    wait_for_controller_initialization,
)


RING_NAME = "task3_hollow_cylinder"
TABLE_NAME = "task3_destination_table"
DEFAULT_CONFIG = PACKAGE_DIR / "config/task3_grasp.json"
FINGER_NAMES = ("index", "middle", "little")
FINGER_JOINTS = (
    ("r_index_j1", "r_index_j2", "r_index_j3"),
    ("r_middle_j1", "r_middle_j2"),
    ("r_little_j1", "r_little_j2", "r_little_j3"),
)


class Task3GraspFeedback:
    """Collect the follower's latched state without guessing its timing."""

    def __init__(self, ring_name=RING_NAME):
        self.ring_name = ring_name
        self.armed = False
        self.collision_suppressed = False
        self.latch_mask = 0
        self.held = False
        self.armed_received = False
        self.collision_suppressed_received = False
        self.mask_received = False
        self.held_received = False
        self.subscribers = []
        self.subscribers.append(rospy.Subscriber(
            "/mujoco/task3_grasp_armed", Bool,
            self._armed_callback, queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            "/mujoco/task3_fingertip_collision_suppressed", Bool,
            self._collision_suppressed_callback, queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            "/mujoco/{}/internal_finger_latch_mask".format(ring_name),
            UInt8, self._mask_callback, queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            "/mujoco/{}/internal_grasped".format(ring_name),
            Bool, self._held_callback, queue_size=1))

    def rebind(self, ring_name):
        """Point the ring-scoped subscriptions at another ring.

        Task 3's three rings are separate bodies with their own topics, so a
        single long-lived feedback object has to follow the ring under work.
        """
        for subscriber in self.subscribers:
            subscriber.unregister()
        self.__init__(ring_name)

    def _armed_callback(self, message):
        self.armed = bool(message.data)
        self.armed_received = True

    def _collision_suppressed_callback(self, message):
        self.collision_suppressed = bool(message.data)
        self.collision_suppressed_received = True

    def _mask_callback(self, message):
        self.latch_mask = int(message.data) & 0b111
        self.mask_received = True

    def _held_callback(self, message):
        self.held = bool(message.data)
        self.held_received = True

    @staticmethod
    def _wait(predicate, timeout):
        deadline = time.monotonic() + float(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return True
            rate.sleep()
        return bool(predicate())

    def wait_until_ready(self, timeout):
        return self._wait(
            lambda: self.armed_received and
            self.collision_suppressed_received and
            self.mask_received and self.held_received, timeout)

    def wait_for_collision_suppressed(self, timeout):
        return self._wait(lambda: self.collision_suppressed, timeout)

    def wait_for_armed(self, timeout):
        return self._wait(lambda: self.armed, timeout)

    def wait_for_held(self, timeout):
        return self._wait(lambda: self.held, timeout)


class Task3GraspCommand:
    def __init__(self):
        self.grasp_publisher = rospy.Publisher(
            "/mujoco/task3_grasp_enabled", Bool,
            queue_size=1, latch=True)
        self.grasp_finalize_publisher = rospy.Publisher(
            "/mujoco/task3_grasp_finalize_enabled", Bool,
            queue_size=1, latch=True)
        self.collision_suppression_publisher = rospy.Publisher(
            "/mujoco/task3_fingertip_collision_suppression_enabled", Bool,
            queue_size=1, latch=True)

    def set_enabled(self, enabled):
        self.grasp_publisher.publish(Bool(data=bool(enabled)))

    def set_finalize_enabled(self, enabled):
        self.grasp_finalize_publisher.publish(Bool(data=bool(enabled)))

    def set_collision_suppression_enabled(self, enabled):
        self.collision_suppression_publisher.publish(
            Bool(data=bool(enabled)))


def load_config(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    if config.get("schema_version") != 1:
        raise RuntimeError("unsupported Task 3 grasp config schema")
    descent = float(config["descent_distance_m"])
    limits = tuple(float(value) for value in config["finger_expansion_limits"])
    if descent <= 0.0:
        raise ValueError("descent_distance_m must be positive")
    if len(limits) != 3 or any(not 0.0 < value <= 1.0 for value in limits):
        raise ValueError("finger_expansion_limits must contain three values in (0, 1]")
    if float(config["finger_expansion_step"]) <= 0.0:
        raise ValueError("finger_expansion_step must be positive")
    if float(config["max_ring_motion_during_descent_m"]) <= 0.0:
        raise ValueError("max_ring_motion_during_descent_m must be positive")
    return config, limits


def log_new_latches(gripper, previous_mask, current_mask, expansions):
    positions = gripper.measured_positions("right")
    measured = None
    if positions is not None:
        measured = dict(zip(gripper.right_names, positions))
    for index, name in enumerate(FINGER_NAMES):
        bit = 1 << index
        if previous_mask & bit or not current_mask & bit:
            continue
        joint_text = "unavailable"
        if measured is not None:
            joint_text = ", ".join(
                "{}={:.4f}".format(joint, measured[joint])
                for joint in FINGER_JOINTS[index])
        rospy.loginfo(
            "Task3 finger latch: finger=%s, expansion=%.4f, measured=[%s], mask=0x%02x",
            name, expansions[index], joint_text, current_mask)


def expand_until_search_complete(gripper, feedback, config, limits):
    expansions = (0.0, 0.0, 0.0)
    step = float(config["finger_expansion_step"])
    period = float(config["finger_command_period_s"])
    deadline = time.monotonic() + float(config["latch_timeout_s"])
    # The follower may latch a compact finger immediately after grasp enable,
    # before this loop starts.  Compare against the required unlatched start
    # so that transition is still logged instead of treating it as baseline.
    observed_mask = 0

    gripper.command_right_internal_fingers(*expansions)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        current_mask = int(feedback.latch_mask)
        if current_mask != observed_mask:
            log_new_latches(gripper, observed_mask, current_mask, expansions)
            observed_mask = current_mask
        complete = all(
            current_mask & (1 << index) or
            expansions[index] >= limits[index]
            for index in range(3))
        if complete:
            gripper.command_right_internal_fingers(*expansions)
            if bin(current_mask).count("1") < 2:
                raise RuntimeError(
                    "Task 3 finger motion completed with fewer than two contacts: "
                    "expansions={}, mask=0x{:02x}".format(
                        expansions, current_mask))
            return expansions, current_mask
        following = advance_internal_expansions(
            expansions, current_mask, step, limits=limits)
        if following == expansions:
            break
        expansions = following
        gripper.command_right_internal_fingers(*expansions)
        rospy.sleep(period)

    raise RuntimeError(
        "Task 3 fingers did not complete before timeout/search limits: "
        "expansions={}, mask=0x{:02x}".format(expansions, feedback.latch_mask))


def init_runtime(args, gripper=None, trajectory=None):
    """Perform the one-time ROS/SDK setup shared by every Task 3 stage."""
    rospy.init_node("task3_grasp", anonymous=False)
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
        feedback=None,
        command=None,
    )


def make_planner(scene, ring_name):
    """Build the per-ring IK planner the accepted stage used to create."""
    return Task3SecondIKPlanner(scene, ring_name=ring_name)


def run_grasp(
        robot, robot_state, chassis, object_pose, planner, gripper,
        feedback, command, config, limits, ring_name, destination_xy,
        trajectory=None):
    """Run the accepted descend/latch/lift/transfer in an existing node."""
    grasp_succeeded = False

    if getattr(feedback, "ring_name", None) != ring_name:
        feedback.rebind(ring_name)
    if not feedback.wait_until_ready(timeout=2.0):
        raise RuntimeError("Task 3 grasp feedback topics are unavailable")
    if feedback.held or feedback.latch_mask:
        raise RuntimeError(
            "Task 3 grasp stage requires an unlatched start: held={}, mask=0x{:02x}".format(
                feedback.held, feedback.latch_mask))
    command.set_enabled(False)
    command.set_finalize_enabled(False)
    command.set_collision_suppression_enabled(True)
    if not feedback.wait_for_collision_suppressed(
            config["armed_timeout_s"]):
        raise RuntimeError(
            "Task 3 follower did not suppress fingertip collision")
    rospy.loginfo(
        "Task3 fingertip collision suppressed before insertion; "
        "geometric latch remains disabled")

    measured = tuple(wait_for_arm_state(robot_state))
    trajectory_start = wait_for_held_arm_command()
    if trajectory is None:
        trajectory = TrajectoryController(
            robot, initial_positions=trajectory_start)
    else:
        # Same live start the stage used to pass into its own controller.
        # set_target_positions copies in place, so it needs a list.
        trajectory.set_target_positions(list(trajectory_start))
    base_pose = tuple(chassis.pose())
    ring_position = object_pose.wait_for_position(
        ring_name, timeout=30.0)
    desired_position, desired_rotation = planner.reference_pose(ring_position)
    command_position, command_rotation = planner.feedforward_command_pose(
        ring_position)
    descent = float(config["descent_distance_m"])
    desired_position = desired_position + np.asarray((0.0, 0.0, -descent))
    command_position = command_position + np.asarray((0.0, 0.0, -descent))
    result = planner.solve_target(
        base_pose, ring_position, measured,
        command_position, command_rotation,
        validate_insertion=False)
    if result is None:
        raise RuntimeError("Task 3 small-descent IK did not solve")
    rospy.loginfo(
        "Task3 grasp descent: distance=%.1f mm, desired_position=%s, command_position=%s",
        1000.0 * descent, np.round(desired_position, 6),
        np.round(command_position, 6))
    execute_right_target(
        trajectory, trajectory_start, result.right_joints,
        int(config["descent_trajectory_points"]))
    rospy.sleep(float(config["descent_settle_seconds"]))

    ring_position_after = object_pose.get_position(ring_name)
    if ring_position_after is None:
        raise RuntimeError("Task 3 ring pose unavailable after descent")
    ring_motion = np.asarray(ring_position_after) - np.asarray(ring_position)
    rospy.loginfo(
        "Task3 insertion ring motion_mm=%s norm=%.2f",
        np.round(1000.0 * ring_motion, 2),
        1000.0 * np.linalg.norm(ring_motion))
    if np.linalg.norm(ring_motion) > float(
            config["max_ring_motion_during_descent_m"]):
        raise RuntimeError(
            "Task 3 insertion moved the ring before latch")

    command.set_enabled(True)
    if not feedback.wait_for_armed(config["armed_timeout_s"]):
        raise RuntimeError(
            "Task 3 follower did not confirm collision suppression")
    rospy.loginfo(
        "Task3 force-free grasp armed; expanding fingers independently")
    expansions, latch_mask = expand_until_search_complete(
        gripper, feedback, config, limits)
    rospy.loginfo(
        "Task3 finger motion complete; finalizing object latch with mask=0x%02x",
        latch_mask)
    command.set_finalize_enabled(True)
    if not feedback.wait_for_held(config["held_timeout_s"]):
        raise RuntimeError(
            "Task 3 has two finger latches but object did not latch")
    grasp_succeeded = True
    rospy.loginfo(
        "TASK3 INTERNAL GRASP COMPLETE: expansions=%s, mask=0x%02x, held=true",
        tuple(round(value, 4) for value in expansions), latch_mask)

    measured_lift = tuple(wait_for_arm_state(robot_state))
    base_lift = tuple(chassis.pose())
    ring_lift = object_pose.wait_for_position(
        ring_name, timeout=30.0)
    actual_hand_position, actual_hand_rotation = planner.measured_pose(
        base_lift, ring_lift, measured_lift[7:14])
    lift_distance = float(config["lift_distance_m"])
    lift_desired_position = actual_hand_position + np.asarray(
        (0.0, 0.0, lift_distance))
    lift_command_position = (
        lift_desired_position - planner.TRACKING_POSITION_BIAS_WORLD_M)
    lift_command_rotation = (
        actual_hand_rotation * planner.tracking_rotation.inv())
    lift_result = planner.solve_target(
        base_lift, ring_lift, measured_lift,
        lift_command_position, lift_command_rotation,
        validate_insertion=False)
    if lift_result is None:
        raise RuntimeError("Task 3 loaded lift IK did not solve")
    rospy.loginfo("Task3 loaded lift: distance=%.1f mm", 1000.0 * lift_distance)
    execute_right_target(
        trajectory, measured_lift, lift_result.right_joints,
        int(config["lift_trajectory_points"]))
    rospy.sleep(float(config["lift_settle_seconds"]))

    held_position = object_pose.wait_for_position(
        ring_name, timeout=30.0)
    stopped_base = tuple(chassis.pose())
    destination_base = (
        stopped_base[0] + destination_xy[0] - held_position[0],
        stopped_base[1] + destination_xy[1] - held_position[1],
        stopped_base[2],
    )
    rospy.loginfo(
        "Task3 chassis transfer left: target=(%.3f, %.3f, %.1f deg)",
        destination_base[0], destination_base[1],
        np.degrees(destination_base[2]))
    chassis.move_to_pose(
        destination_base[0], destination_base[1], destination_base[2],
        timeout=float(config["base_motion_timeout_s"]))

    grasp_succeeded = False
    command.set_finalize_enabled(False)
    command.set_enabled(False)
    gripper.control_right_internal_expansion(
        0.0, duration=float(config["hand_motion_duration_s"]))
    if not feedback._wait(
            lambda: not feedback.held, config["release_timeout_s"]):
        raise RuntimeError(
            "Task 3 object did not release on inward finger motion")
    rospy.loginfo(
        "TASK3 TRANSFER COMPLETE: release acknowledged for ring=%s",
        ring_name)
    return trajectory


def main(runtime=None, argv=None, ring_name=None, destination_xy=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--ring-name", default=RING_NAME)
    # Called from the merged workflow the stage must not read the entry
    # point's command line; the ring target arrives as an argument instead,
    # so the CLI requirement does not apply.
    merged = runtime is not None
    parser.add_argument("--destination-xy", type=float, nargs=2,
                        required=not merged, metavar=("X", "Y"))
    args = parser.parse_args([] if merged else argv)
    if ring_name is not None:
        args.ring_name = ring_name
    if destination_xy is not None:
        args.destination_xy = tuple(destination_xy)
    config, limits = load_config(args.config)

    owns_runtime = runtime is None
    if owns_runtime:
        runtime = init_runtime(args)
    if runtime.feedback is None:
        runtime.feedback = Task3GraspFeedback(args.ring_name)
    if runtime.command is None:
        runtime.command = Task3GraspCommand()
    if runtime.planner is None:
        runtime.planner = make_planner(args.scene, args.ring_name)
    trajectory = runtime.trajectory
    try:
        trajectory = run_grasp(
            runtime.robot, runtime.robot_state, runtime.chassis,
            runtime.poses, runtime.planner, runtime.gripper,
            runtime.feedback, runtime.command, config, limits,
            args.ring_name, args.destination_xy, trajectory=trajectory)
    finally:
        if owns_runtime:
            if not getattr(runtime, "grasp_succeeded", True):
                runtime.command.set_finalize_enabled(False)
                runtime.command.set_enabled(False)
                runtime.command.set_collision_suppression_enabled(False)
            if trajectory is not None:
                trajectory.stop()
            runtime.gripper.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
