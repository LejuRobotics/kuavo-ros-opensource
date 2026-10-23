#!/usr/bin/env python3
"""Run the canonical Task 1 right-arm pick/place and lever workflow."""

import json
import math
import os
import sys
import time
from contextlib import contextmanager
from pathlib import Path

import rospy
from scipy.spatial.transform import Rotation, Slerp
from std_msgs.msg import Bool, UInt8

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(CURRENT_DIR)
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
from task2_base_motion import ChassisMotion
from utils.gripper_controller import GripperController
from utils.hand_postures import single_gripper_targets
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.scene1_v2_right_arm_ik import Scene1V2RightArmIK
from utils.scorer_clock import start_score_clock
from utils.arm_state import wait_for_first_arm_state
from utils.task1_v2_randomization import Task1V2RandomizationPlanner
from utils.task1_grasp_diagnostics import Task1GraspDiagnosticRecorder
from utils.trajectory_controller import TrajectoryController
from utils.utils import Utils


CYLINDERS = ("cylinder_1", "cylinder_2", "cylinder_3")
TARGET_BIN = "target_bin"
RESULT_PATH = Path("task_result.txt")

LEFT_THUMB_SAFE_J1_RAD = 1.75
LEFT_THUMB_INITIALIZATION_DURATION_S = 1.2

# V2 target-bin wall inner bounds.  _fully_in_target tests the cylinder centre
# against this box directly: the whole bin interior counts, with no cylinder
# radius subtracted from it.
TARGET_MIN = (0.2075, -0.5852, 0.515)
TARGET_MAX = (0.4075, -0.4708, 0.590)

# Initial approach bias; measured pose correction removes the residual
# before the fingers close.
GRASP_TRACKING_BIASES_WORLD = (
    (0.015, 0.010, 0.010),
    (0.015, 0.010, 0.010),
    (0.015, 0.010, 0.010),
)
GRASP_YAW_ADJUSTMENT_RAD = math.radians(-15.0)
# Carry above the rod and bin walls, then release directly over the target.
DROP_GRASP_CENTER_Z = 0.81
# Keep the first release slot 10 mm farther inside the narrow Y boundary.
# Wide source randomization can preserve about 13 mm of object/hand tracking
# offset during transport, so the former -22 mm slot had only nominal margin.
DROP_CENTER_OFFSETS = (
    (-0.045, -0.015),
    (0.000, 0.015),
    (0.045, -0.015),
)

ARM_JOINT_COUNT = 14
HEAD_SCAN_PITCH_RAD = math.radians(20.0)
# Fixed wrist reference for the cylinder grasp, independent of the outward
# initialization pose.
RIGHT_ARM_READY_FULL_RAD = (-0.9, -0.265, 1.0, -0.8, 0.58, -0.4, 0.35)
# Loaded transport retains physical contacts; allow the wrist to track
# without throwing the cylinder during a large change in arm configuration.
TARGET_TRANSPORT_TRAJECTORY_POINTS = 55
SOURCE_GRASP_SETTLE_SECONDS = 0.10

# Keep the existing chassis publisher and stop lifecycle. Task 1 does not
# issue any base-motion command.
CHASSIS_LINEAR_SPEED = 0.40
CHASSIS_ANGULAR_SPEED = 0.30
CHASSIS_MIN_LINEAR_SPEED = 0.08
CHASSIS_MIN_ANGULAR_SPEED = 0.08
CHASSIS_POSITION_TOLERANCE = 0.03
CHASSIS_YAW_TOLERANCE_DEG = 3.0

LEVER_PIVOT_WORLD = (0.62, -0.196, 0.62)
LEVER_GRIP_CENTER_IN_R7 = (0.0, 0.074, -0.150)
LEVER_PATH_STEP_RAD = math.radians(2.5)
LEVER_PATH_FINE_STEP_RAD = math.radians(0.2)
# Keep the accepted coarse pull until the old 29.8-degree acceptance edge.
# The last coarse target is capped here, then only 0.2-degree increments are
# allowed while waiting for the source-bin handoff event.
LEVER_PATH_FINE_START_RAD = math.radians(29.8)
# Measured angles are noisy and logged to only 0.1 degree.  Without a small
# switch tolerance, a settled 29.79-degree lever repeatedly receives the same
# capped 29.8-degree target and is falsely classified as stalled.
LEVER_PATH_FINE_START_TOLERANCE_RAD = math.radians(0.1)
LEVER_PATH_SAFETY_MAX_RAD = math.radians(35.0)
LEVER_PATH_MAX_ATTEMPTS = 60
LEVER_PATH_MIN_PROGRESS_RAD = math.radians(0.2)
LEVER_PATH_MAX_STALLS = 5
# Preserve the accepted zero-angle hand pose while allowing the high-angle
# continuation to use the extra physical joint travel it needs.
LEVER_PATH_TRAJECTORY_POINTS = 12
# Preserve approximately the same angular speed after the 29.8-degree
# handoff edge.  Reusing the coarse point count for a 0.2-degree correction
# would make each fine step take as long as a 2.5-degree pull.
LEVER_PATH_FINE_TRAJECTORY_POINTS = 2
SOURCE_CONVEYOR_COMPLETE_TOPIC = (
    "/mujoco/source_bin_conveyor_complete")
SOURCE_BIN_LATCH_RELEASED_TOPIC = (
    "/mujoco/source_bin_latch_released")
SOURCE_CONVEYOR_TIMEOUT_S = 10.0


class TaskTimingRecorder:
    """Emit machine-readable Task 1 timings without changing control flow."""

    def __init__(self):
        self.task_started = time.monotonic()
        self.score_started = None
        self.segment_count = 0

    @contextmanager
    def segment(self, label, **details):
        started = time.monotonic()
        status = "ok"
        try:
            yield
        except BaseException:
            status = "error"
            raise
        finally:
            ended = time.monotonic()
            self.segment_count += 1
            payload = {
                "event": "segment",
                "label": label,
                "duration_s": round(ended - started, 6),
                "task_elapsed_s": round(ended - self.task_started, 6),
                "score_elapsed_s": (
                    None if self.score_started is None
                    else round(ended - self.score_started, 6)),
                "status": status,
            }
            payload.update(details)
            print("[TASK1_TIMING] {}".format(
                json.dumps(payload, sort_keys=True)))

    def mark_score_start(self, scorer_started):
        self.score_started = time.monotonic()
        print("[TASK1_TIMING] {}".format(json.dumps({
            "event": "score_start",
            "scorer_started": bool(scorer_started),
            "task_elapsed_s": round(
                self.score_started - self.task_started, 6),
        }, sort_keys=True)))

    def print_summary(self, success):
        ended = time.monotonic()
        print("[TASK1_TIMING_SUMMARY] {}".format(json.dumps({
            "event": "summary",
            "segment_count": self.segment_count,
            "success": bool(success),
            "task_elapsed_s": round(ended - self.task_started, 6),
            "score_elapsed_s": (
                None if self.score_started is None
                else round(ended - self.score_started, 6)),
        }, sort_keys=True)))


class SourceBinReleaseMonitor:
    """Latch the source-bin handoff event published by the scene script."""

    def __init__(self):
        self.released = False
        self.subscriber = rospy.Subscriber(
            SOURCE_BIN_LATCH_RELEASED_TOPIC, Bool,
            self._callback, queue_size=1)

    def _callback(self, message):
        if message.data:
            self.released = True


def _quaternion_error_degrees(target_xyzw, actual_xyzw):
    """Return the shortest angular distance between two quaternions."""
    target_norm = math.sqrt(sum(value * value for value in target_xyzw))
    actual_norm = math.sqrt(sum(value * value for value in actual_xyzw))
    if target_norm == 0.0 or actual_norm == 0.0:
        raise ValueError("orientation quaternion must be non-zero")
    dot = sum(
        target * actual
        for target, actual in zip(target_xyzw, actual_xyzw)
    ) / (target_norm * actual_norm)
    return math.degrees(2.0 * math.acos(min(1.0, abs(dot))))


def _initialize_left_thumb(gripper):
    """Rotate only l_thumb_j1 while preserving every other measured joint."""
    starts = gripper._wait_for_measured_positions(("left",))["left"]
    target = list(starts)
    thumb_index = gripper.left_names.index("l_thumb_j1")
    target[thumb_index] = LEFT_THUMB_SAFE_J1_RAD
    steps = max(1, int(round(
        LEFT_THUMB_INITIALIZATION_DURATION_S * gripper.publish_frequency)))

    print(
        "V2 left-thumb initialization: measured_start={} target_j1={:.6f} "
        "rad; left arm and all other hand joints remain unchanged".format(
            [round(starts[gripper.left_names.index(name)], 6)
             for name in ("l_thumb_j1", "l_thumb_j2", "l_thumb_j3")],
            LEFT_THUMB_SAFE_J1_RAD,
        ))
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

    time.sleep(0.2)
    measured = gripper.measured_positions("left")
    if measured is None:
        print("V2 left-thumb initialized; measured hand state unavailable")
        return
    print("V2 left-thumb initialized measured joints: {}".format(
        [round(measured[gripper.left_names.index(name)], 6)
         for name in ("l_thumb_j1", "l_thumb_j2", "l_thumb_j3")]))


def _randomize_scene(randomizer, seed, chassis):
    """Place the selected layout while retaining the fixed launch-time base.

    Preparation then raises the right hand above the first cylinder.
    """
    planner = Task1V2RandomizationPlanner()
    plan = planner.plan(seed)

    for name, spawn in zip(CYLINDERS, plan.cylinders):
        x, y, z = spawn.position
        yaw = spawn.yaw
        result = randomizer.set_object_position(
            name,
            position={
                "x": x,
                "y": y,
                "z": z,
            },
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
        "V2 seed {} generated scene {}: initial_base={} "
        "task_base={} grasp_bases={} profile={} objects={}".format(
            seed,
            plan.catalogue_seed,
            [round(value, 6) for value in plan.initial_base],
            [round(value, 6) for value in plan.task_base],
            [[round(value, 6) for value in base]
             for base in plan.grasp_bases],
            plan.profile,
            [[round(value, 6) for value in spawn.position]
             for spawn in plan.cylinders],
        ))
    time.sleep(1.0)
    chassis.wait_until_ready()
    if chassis.pose() is not None:
        print("V2 seed {} launch-time measured base pose: {}".format(
            seed, [round(value, 6) for value in chassis.pose()]))
    return plan


def _fully_in_target(position, target_position=(0.3075, -0.528, 0.50)):
    if position is None:
        return False
    return all(
        TARGET_MIN[index] + target_position[index] - (0.3075, -0.528)[index] <= position[index] <= TARGET_MAX[index] + target_position[index] - (0.3075, -0.528)[index]
        for index in range(2)
    )


def _right_only_trajectory(target_deg, start_deg, left_hold_rad, num):
    """Interpolate the right arm while holding the measured left arm exactly."""
    points = Utils.interpolate_joint_trajectory(
        target_deg, start_deg, num=num)
    for point in points:
        point[:7] = left_hold_rad
    return points


def _smootherstep(phase):
    """Quintic blend with zero first and second derivatives at both ends."""
    phase = min(1.0, max(0.0, float(phase)))
    return phase ** 3 * (10.0 - 15.0 * phase + 6.0 * phase ** 2)


def _right_only_smooth_trajectory(
        target_deg, start_deg, left_hold_rad, num):
    """Move the right arm with zero velocity at both segment boundaries."""
    if num <= 0:
        raise ValueError("num must be positive")
    points = []
    for step in range(1, num + 1):
        phase = float(step) / num
        blend = _smootherstep(phase)
        point = [
            math.radians(start + blend * (target - start))
            for start, target in zip(start_deg, target_deg)
        ]
        point[:7] = left_hold_rad
        points.append(point)
    return points


def _clearance_point(start, target, fraction, clearance_height):
    height = max(clearance_height, start[2], target[2])
    if fraction < 0.2:
        return [start[0], start[1], start[2] + fraction / 0.2 * (height - start[2])]
    if fraction <= 0.8:
        alpha = (fraction - 0.2) / 0.6
        return [start[0] + alpha * (target[0] - start[0]),
                start[1] + alpha * (target[1] - start[1]), height]
    return [target[0], target[1], height + (fraction - 0.8) / 0.2 * (target[2] - height)]


def _rounded_waypoint_point(start, via, target, fraction):
    """Join two straight moves with a 15 mm quadratic corner, without a stop."""
    incoming = [b - a for a, b in zip(start, via)]
    outgoing = [b - a for a, b in zip(via, target)]
    first = math.sqrt(sum(v * v for v in incoming))
    second = math.sqrt(sum(v * v for v in outgoing))
    if min(first, second) < 1e-6:
        return [a + fraction * (b - a) for a, b in zip(start, target)]
    radius = min(0.015, first / 2.0, second / 2.0)
    enter = [v - radius * d / first for v, d in zip(via, incoming)]
    leave = [v + radius * d / second for v, d in zip(via, outgoing)]
    distance = fraction * (first + second)
    if distance < first - radius:
        alpha = distance / (first - radius)
        return [a + alpha * (b - a) for a, b in zip(start, enter)]
    if distance <= first + radius:
        alpha = (distance - first + radius) / (2.0 * radius)
        return [(1 - alpha) ** 2 * a + 2 * alpha * (1 - alpha) * v + alpha ** 2 * b
                for a, v, b in zip(enter, via, leave)]
    alpha = (distance - first - radius) / (second - radius)
    return [a + alpha * (b - a) for a, b in zip(leave, target)]


def _lever_angle_from_orientation(quaternion_xyzw):
    if quaternion_xyzw is None:
        return None
    x, y, z, w = quaternion_xyzw
    vector_norm = math.sqrt(x * x + y * y + z * z)
    angle = 2.0 * math.atan2(vector_norm, w)
    return 2.0 * math.pi - angle if angle > math.pi else angle


def _rotate_vector_about_world_y(vector, angle):
    """Rotate a free vector about world Y using the lever's convention."""
    cosine = math.cos(angle)
    sine = math.sin(angle)
    x, y, z = vector
    return (
        cosine * x + sine * z,
        y,
        -sine * x + cosine * z,
    )


def _make_chassis():
    """Closed-loop wheel-drive chassis, identical contract to Task 2/3."""
    return ChassisMotion(
        linear_speed=CHASSIS_LINEAR_SPEED,
        angular_speed=CHASSIS_ANGULAR_SPEED,
        minimum_linear_speed=CHASSIS_MIN_LINEAR_SPEED,
        minimum_angular_speed=CHASSIS_MIN_ANGULAR_SPEED,
        position_tolerance=CHASSIS_POSITION_TOLERANCE,
        yaw_tolerance_deg=CHASSIS_YAW_TOLERANCE_DEG,
    )


def close_on_cylinder(gripper, name):
    mask = [0]
    subscriber = rospy.Subscriber(
        "/mujoco/{}/internal_finger_latch_mask".format(name), UInt8,
        lambda msg: mask.__setitem__(0, msg.data), queue_size=1)
    closure = 0.0
    preload_target = None
    deadline = time.monotonic() + 6.0
    try:
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if preload_target is None and mask[0] & 3 == 3:
                preload_target = min(1.0, closure + 0.02)
            limit = 1.0 if preload_target is None else preload_target
            closure = min(limit, closure + (0.001 if mask[0] & 3 else 0.009))
            with gripper.command_lock:
                gripper.current_right_positions = single_gripper_targets("r", closure)
                gripper.command_initialized.set()
            if preload_target is not None and closure >= preload_target:
                time.sleep(0.2)
                if mask[0] & 3 == 3:
                    return
                preload_target = None
            time.sleep(0.01)
        raise RuntimeError("{} did not establish two physical finger contacts: mask={}, closure={}, hand={}".format(name, mask[0], closure, gripper.measured_positions("right")))
    finally:
        subscriber.unregister()


def release_cylinder(gripper, name, timing):
    with timing.segment("{}.gripper_release".format(name)):
        # Keep the verified gentle release speed and public 0..255 units.
        gripper.control_right_gripper(0.65 * 255, duration=0.5)
    with timing.segment("{}.gripper_open".format(name)):
        gripper.control_right_gripper(0, duration=0.3)
    # The caller checks that the released object is inside the target region.
    # Do not wait for rest or sustained floor support before the next motion.


def main():
    timing = TaskTimingRecorder()
    diagnostics = None
    experiment_mode = os.environ.get("TASK_EXPERIMENT_MODE", "0") == "1"
    with timing.segment("setup.sdk_init"):
        sdk_initialized = KuavoSDK().Init(options=KuavoSDK.Options.Normal)
    if not sdk_initialized:
        raise RuntimeError("KuavoSDK initialization failed")

    with timing.segment("setup.controllers"):
        robot = KuavoRobot()
        robot_state = KuavoRobotState()
        gripper = GripperController()
        source_bin_release = SourceBinReleaseMonitor()
        chassis = _make_chassis()
    success = False
    trajectory = None

    try:
        with timing.segment("initialization.first_arm_state_wait"):
            initial_arm_rad = wait_for_first_arm_state()
        trajectory = TrajectoryController(
            robot, initial_positions=initial_arm_rad)
        left_hold_rad = list(initial_arm_rad[:7])
        current_target_deg = [math.degrees(value) for value in initial_arm_rad]

        with timing.segment("initialization.left_thumb"):
            _initialize_left_thumb(gripper)

        if not robot.control_head(yaw=0.0, pitch=HEAD_SCAN_PITCH_RAD):
            raise RuntimeError("Failed to command the 20-degree head scan pose")
        scene_ik = Scene1V2RightArmIK()
        seed = int(os.environ.get("TASK_SEED", "1"))
        diagnostics = Task1GraspDiagnosticRecorder(seed)
        diagnostics.write(
            "round_start",
            diagnostics_path=str(diagnostics.path.resolve()),
            experiment_mode=experiment_mode,
        )
        print("[TASK1_DIAGNOSTICS] writing {}".format(
            diagnostics.path.resolve()))
        randomizer = ObjectRandomizer(timeout=60.0)
        with timing.segment("initialization.randomize_scene", seed=seed):
            randomization_plan = _randomize_scene(
                randomizer, seed, chassis)
            base_translation_world = randomization_plan.task_base

        scene_ik.set_base_translation_world(base_translation_world)
        print("V2 seed {} robot base translation: {}".format(
            seed, [round(value, 6) for value in base_translation_world]))
        with timing.segment("initialization.scene_settle_wait"):
            time.sleep(1.0)

        poses = ObjectPose()
        with timing.segment("initialization.object_pose_wait"):
            for name in CYLINDERS + (TARGET_BIN,):
                poses.wait_for_position(name, timeout=60.0)
        target_bin_position = poses.get_position(TARGET_BIN)
        print("V2 target bin live position: {}".format(
            [round(value, 6) for value in target_bin_position]))

        chassis.wait_until_ready()
        chassis.stop()
        fixed_base = chassis.pose()
        scene_ik.set_base_translation_world((fixed_base[0], fixed_base[1], 0.0))
        initial_arm_rad = list(robot_state.arm_joint_state().position)
        preparation = scene_ik.preparation_trajectory(
            initial_arm_rad, poses.get_position(CYLINDERS[0]))
        with timing.segment("initialization.above_first_object", trajectory_points=len(preparation)):
            trajectory.execute_trajectory(preparation, sleep_time=0.01)
        current_target_deg = [math.degrees(value) for value in preparation[-1]]

        def latch_achieved_right_target(label):
            nonlocal current_target_deg
            live_arm_rad = list(robot_state.arm_joint_state().position)
            if len(live_arm_rad) != ARM_JOINT_COUNT:
                raise RuntimeError(
                    "Expected 14 live arm joints after {}, got {}".format(
                        label, len(live_arm_rad)))
            live_arm_rad[7:14] = [max(lower, min(upper, value))
                                 for value, lower, upper in zip(
                                     live_arm_rad[7:14], scene_ik.lower, scene_ik.upper)]
            achieved_deg = [math.degrees(value) for value in live_arm_rad]
            current_target_deg[7:14] = achieved_deg[7:14]
            trajectory.set_target_positions(
                left_hold_rad + live_arm_rad[7:14])
            print("{} accepted right-arm achieved joints: {}".format(
                label,
                [round(value, 6) for value in live_arm_rad[7:14]],
            ))
            return live_arm_rad

        last_right_seed_rad = None
        last_right_ik_rad = None
        last_right_command_rad = None
        last_tcp_target_world = None
        last_tcp_quaternion_world = None

        def execute_right_trajectory(points, sleep_time=0.02):
            for point in points:
                point[7:14] = [max(lower, min(upper, value))
                               for value, lower, upper in zip(
                                   point[7:14], scene_ik.lower, scene_ik.upper)]
            trajectory.execute_trajectory(points, sleep_time=sleep_time)

        def move_right_tcp(
                label, tcp_position_world, tcp_quaternion_world,
                point_in_r7=None,
                trajectory_points=TARGET_TRANSPORT_TRAJECTORY_POINTS,
                arc_height=None, joint_bounds=None, clearance_height=None,
                via_position_world=None, via_quaternion_world=None,
                on_clearance=None):
            nonlocal current_target_deg
            nonlocal last_right_seed_rad
            nonlocal last_right_ik_rad, last_right_command_rad
            nonlocal last_tcp_target_world, last_tcp_quaternion_world
            print("Moving V2 right task TCP to {}: {}".format(
                label, tcp_position_world))
            live_arm_rad = list(robot_state.arm_joint_state().position)
            if arc_height is not None:
                start = scene_ik._forward(live_arm_rad[7:14], point_in_r7)[0]
                rotations = Slerp([0.0, 1.0], Rotation.from_quat([
                    scene_ik.eef_quaternion_world(live_arm_rad[7:14]),
                    tcp_quaternion_world]))
                via_fraction = None
                clearance_fraction = None
                if via_position_world is not None:
                    first = math.dist(start, via_position_world)
                    second = math.dist(via_position_world, tcp_position_world)
                    via_fraction = first / (first + second)
                    clearance_fraction = (first - min(0.015, first / 2, second / 2)) / (first + second)
                    rotations = Slerp([0.0, via_fraction, 1.0], Rotation.from_quat([
                        scene_ik.eef_quaternion_world(live_arm_rad[7:14]),
                        via_quaternion_world, tcp_quaternion_world]))
                knots = [list(live_arm_rad)]
                right_joints = live_arm_rad[7:14]
                knot_count = (60 if via_position_world is not None else
                              30 if point_in_r7 is scene_ik.CLOSED_GRASP_CENTER_IN_R7 else 20)
                with timing.segment("{}.ik_solve".format(label)):
                    for index in range(1, knot_count + 1):
                        fraction = index / float(knot_count)
                        target = [a + fraction * (b - a)
                                  for a, b in zip(start, tcp_position_world)]
                        if via_position_world is not None:
                            target = _rounded_waypoint_point(
                                start, via_position_world, tcp_position_world, fraction)
                        elif clearance_height is not None:
                            target = _clearance_point(
                                start, tcp_position_world, fraction, clearance_height)
                        else:
                            target[2] += 4.0 * fraction * (1.0 - fraction) * arc_height
                        right_joints = scene_ik.solve(
                            target, rotations(min(1.0, fraction / 0.6) if clearance_height is not None else fraction).as_quat(), right_joints,
                            point_in_r7=point_in_r7, seed_only=True,
                            joint_bounds=joint_bounds)
                        if max(abs(a - b) for a, b in zip(right_joints, knots[-1][7:14])) > 0.5:
                            raise RuntimeError("{} IK path changes joint branch at {}: {} -> {}".format(label, index, knots[-1][7:14], right_joints))
                        knots.append(left_hold_rad + right_joints)
                points = []
                clearance_index = None
                # One smooth time law for the whole Cartesian curve, without
                # stopping at the internal IK samples.
                for index in range(1, trajectory_points + 1):
                    fraction = _smootherstep(index / float(trajectory_points))
                    if (clearance_fraction is not None and clearance_index is None
                            and fraction >= clearance_fraction):
                        clearance_index = index
                    phase = fraction * knot_count
                    lower = min(knot_count - 1, int(phase))
                    alpha = phase - lower
                    points.append([a + alpha * (b - a)
                                   for a, b in zip(knots[lower], knots[lower + 1])])
                with timing.segment("{}.arm_motion".format(label),
                                    trajectory_points=trajectory_points):
                    if on_clearance is not None:
                        execute_right_trajectory(points[:clearance_index])
                        current_target_deg = [math.degrees(value)
                                              for value in points[clearance_index - 1]]
                        on_clearance()
                        execute_right_trajectory(points[clearance_index:])
                    else:
                        execute_right_trajectory(points)
                current_target_deg = [math.degrees(value) for value in knots[-1]]
                last_right_seed_rad = list(live_arm_rad[7:14])
                last_right_ik_rad = list(right_joints)
                last_right_command_rad = list(right_joints)
                last_tcp_target_world = list(tcp_position_world)
                last_tcp_quaternion_world = list(tcp_quaternion_world)
                return
            with timing.segment("{}.ik_solve".format(label)):
                right_joints = scene_ik.solve(
                    tcp_position_world,
                    tcp_quaternion_world,
                    live_arm_rad[7:14],
                    point_in_r7=point_in_r7,
                    seed_only=True, joint_bounds=joint_bounds,
                )
            print("{} right-arm IK solution: {}".format(
                label, [round(value, 6) for value in right_joints]))
            commanded_right = list(right_joints)
            live_start_deg = [math.degrees(value) for value in live_arm_rad]
            next_target_deg = list(current_target_deg)
            next_target_deg[7:14] = [
                math.degrees(value) for value in commanded_right]
            with timing.segment(
                    "{}.arm_motion".format(label),
                    trajectory_points=trajectory_points):
                execute_right_trajectory(
                    _right_only_smooth_trajectory(
                        next_target_deg, live_start_deg, left_hold_rad,
                        num=trajectory_points),
                    sleep_time=0.02)
            current_target_deg = next_target_deg
            last_right_seed_rad = list(live_arm_rad[7:14])
            last_right_ik_rad = list(right_joints)
            last_right_command_rad = list(commanded_right)
            last_tcp_target_world = [
                float(value) for value in tcp_position_world]
            last_tcp_quaternion_world = [
                float(value) for value in tcp_quaternion_world]

        def print_measured_grasp_center(label):
            live_arm_rad = list(robot_state.arm_joint_state().position)
            measured = scene_ik.grasp_center_world(live_arm_rad[7:14])
            print("{} measured grasp centre: {}".format(
                label, [round(value, 6) for value in measured]))
            if last_right_ik_rad is not None:
                actual_right = live_arm_rad[7:14]
                print(
                    "{} right-arm IK/command/actual/error: {} / {} / {} / "
                    "{} rad".format(
                        label,
                        [round(value, 6) for value in last_right_ik_rad],
                        [round(value, 6)
                         for value in last_right_command_rad],
                        [round(value, 6) for value in actual_right],
                        [round(actual - target, 6)
                         for target, actual in zip(
                             last_right_ik_rad, actual_right)],
                    ))

        def print_measured_grasp_geometry(label, object_position):
            live_arm_rad = list(robot_state.arm_joint_state().position)
            hand_rad = gripper.measured_positions("right")
            if hand_rad is None:
                print("{} measured hand state is unavailable".format(label))
                return
            geometry = scene_ik.measured_pad_geometry(
                live_arm_rad[7:14], hand_rad)
            print(
                "{} measured pads: thumb={} index={} midpoint={} "
                "gap={:.6f} m object={}".format(
                    label,
                    [round(value, 6) for value in geometry["thumb"]],
                    [round(value, 6) for value in geometry["index"]],
                    [round(value, 6) for value in geometry["midpoint"]],
                    geometry["gap"],
                    [round(value, 6) for value in object_position],
                ))

        def record_grasp_snapshot(phase, cylinder, target_world,
                                  object_reference_world):
            arm_state = robot_state.arm_joint_state()
            arm_position = [float(value) for value in arm_state.position]
            if len(arm_position) != ARM_JOINT_COUNT:
                raise RuntimeError(
                    "Expected 14 arm joints for Task1 diagnostics, got {}"
                    .format(len(arm_position)))
            velocity = getattr(arm_state, "velocity", None)
            arm_velocity = None
            if velocity is not None and len(velocity) == ARM_JOINT_COUNT:
                arm_velocity = [float(value) for value in velocity]

            actual_right = arm_position[7:14]
            actual_center = scene_ik.closed_grasp_center_world(actual_right)
            actual_quaternion = scene_ik.eef_quaternion_world(actual_right)
            target = [float(value) for value in target_world]
            object_position = poses.get_position(cylinder)
            object_orientation = poses.get_orientation(cylinder)
            hand_position = gripper.measured_positions("right")
            geometry = None
            if hand_position is not None:
                geometry = scene_ik.measured_pad_geometry(
                    actual_right, hand_position)

            reference = (
                None if object_reference_world is None
                else [float(value) for value in object_reference_world])
            object_delta = None
            if object_position is not None and reference is not None:
                object_delta = [
                    float(object_position[axis] - reference[axis])
                    for axis in range(3)
                ]

            diagnostics.write(
                phase,
                cylinder=cylinder,
                base_pose_world=(
                    None if chassis.pose() is None
                    else [float(value) for value in chassis.pose()]),
                object_position_world=(
                    None if object_position is None
                    else [float(value) for value in object_position]),
                object_orientation_xyzw=(
                    None if object_orientation is None
                    else [float(value) for value in object_orientation]),
                object_reference_world=reference,
                object_delta_world=object_delta,
                grasp_target_world=target,
                actual_closed_grasp_center_world=[
                    float(value) for value in actual_center],
                target_minus_actual_world=[
                    float(target[axis] - actual_center[axis])
                    for axis in range(3)],
                ik_seed_right_rad=last_right_seed_rad,
                ik_nominal_right_rad=last_right_ik_rad,
                command_right_rad=last_right_command_rad,
                actual_right_rad=actual_right,
                command_minus_actual_right_rad=(
                    None if last_right_command_rad is None else [
                        float(command - actual)
                        for command, actual in zip(
                            last_right_command_rad, actual_right)]),
                actual_right_velocity_rad_s=(
                    None if arm_velocity is None else arm_velocity[7:14]),
                tcp_quaternion_target_xyzw=last_tcp_quaternion_world,
                tcp_quaternion_actual_xyzw=[
                    float(value) for value in actual_quaternion],
                orientation_error_deg=(
                    None if last_tcp_quaternion_world is None else
                    _quaternion_error_degrees(
                        last_tcp_quaternion_world, actual_quaternion)),
                measured_hand_right_rad=(
                    None if hand_position is None else [
                        float(value) for value in hand_position]),
                pad_geometry_world=geometry,
                fully_in_target=_fully_in_target(object_position, target_bin_position),
            )

        def run_right_lever_stage():
            """Close the hand around the rod, confirm a grip, then follow its arc."""
            nonlocal current_target_deg
            palm_seated = [False]
            palm_subscriber = rospy.Subscriber(
                "/mujoco/lever/palm_seated", Bool,
                lambda msg: palm_seated.__setitem__(0, msg.data), queue_size=1)
            grasp_confirmed = [False]
            contact_subscriber = rospy.Subscriber(
                "/mujoco/lever/grasp_confirmed", Bool,
                lambda msg: grasp_confirmed.__setitem__(0, msg.data), queue_size=1)
            grip_contacts = [0, time.monotonic()]
            palm_contact_changed = [time.monotonic()]
            def observe_grip(msg):
                if msg.data & 1 or grip_contacts[0] & 1:
                    palm_contact_changed[0] = time.monotonic()
                if msg.data & 16 or grip_contacts[0] & 16:
                    grip_contacts[1] = time.monotonic()
                grip_contacts[0] = msg.data
            grip_subscriber = rospy.Subscriber(
                "/mujoco/lever/grasp_contacts", UInt8, observe_grip, queue_size=1)
            with timing.segment("lever.pose_wait"):
                poses.wait_for_position("lever", timeout=5.0)

            live_arm_rad = list(robot_state.arm_joint_state().position)
            if len(live_arm_rad) != ARM_JOINT_COUNT:
                raise RuntimeError(
                    "Expected 14 live arm joints before lever, got {}".format(
                        len(live_arm_rad)))
            current_target_deg = [
                math.degrees(value) for value in live_arm_rad]
            trajectory.set_target_positions(live_arm_rad)

            def move_right_joint_target(
                    label, right_target_rad, point_count, smooth=False,
                    r5_command_rad=None, continuous=False):
                nonlocal current_target_deg
                live = list(robot_state.arm_joint_state().position)
                nominal_r5 = right_target_rad[4]
                commanded_right = list(right_target_rad)
                commanded_right[4] = (
                    nominal_r5
                    if r5_command_rad is None
                    else r5_command_rad)
                target_deg = list(current_target_deg)
                target_deg[7:14] = [
                    math.degrees(value) for value in commanded_right]
                print("Executing V2 right-arm lever motion: {}".format(label))
                print(
                    "{} nominal_r5={:.9f} rad commanded_r5={:.9f} rad".format(
                        label, nominal_r5, commanded_right[4]))
                # The lever pull is a stream of adjacent waypoints.  Start
                # those segments at the preceding command endpoint instead
                # of snapping the command back to the lagging measured pose.
                start_deg = (
                    list(current_target_deg)
                    if continuous else
                    [math.degrees(value) for value in live])
                trajectory_points = (
                    _right_only_smooth_trajectory(
                        target_deg,
                        start_deg,
                        left_hold_rad,
                        num=point_count)
                    if smooth else
                    _right_only_trajectory(
                        target_deg,
                        start_deg,
                        left_hold_rad,
                        num=point_count)
                )
                with timing.segment(
                        "{}.arm_motion".format(label),
                        trajectory_points=point_count):
                    execute_right_trajectory(
                        trajectory_points,
                        sleep_time=0.02)
                actual = list(robot_state.arm_joint_state().position)
                print("{} actual_r5={:.9f} rad".format(label, actual[11]))
                current_target_deg = target_deg

            def latch_achieved_lever_target(label):
                nonlocal current_target_deg
                live = list(robot_state.arm_joint_state().position)
                live[7:14] = [max(lower, min(upper, value))
                             for value, lower, upper in zip(
                                 live[7:14], scene_ik.lower, scene_ik.upper)]
                achieved_deg = [math.degrees(value) for value in live]
                current_target_deg[7:14] = achieved_deg[7:14]
                trajectory.set_target_positions(left_hold_rad + live[7:14])
                print("{} accepted right-arm achieved joints: {}".format(
                    label, [round(value, 6) for value in live[7:14]]))
                return live

            def lever_hand_targets(flexions):
                index, middle, little = flexions
                return [0.0, 0.0, 1.28, 0.0, index, min(index, 1.57),
                        middle, min(middle, 1.57), 0.0, little, min(little, 1.57)]

            contact_target = (0.30, -0.30, 0.70)
            approach_target = (0.30, -0.30, 0.84)
            contact_rotation_world = (
                Rotation.from_euler("z", 90.0, degrees=True)
                * Rotation.from_euler("x", -60.0, degrees=True)).as_matrix()
            lever_lower = scene_ik.lower.copy()
            lever_lower[1] = -1.6
            lever_lower[4] = 0.0
            lever_bounds = (lever_lower, scene_ik.upper)

            def print_lever_state(label):
                measured_arm = list(robot_state.arm_joint_state().position)
                measured_tcp = scene_ik._forward(
                    measured_arm[7:14], LEVER_GRIP_CENTER_IN_R7)[0]
                measured_angle = _lever_angle_from_orientation(
                    poses.get_orientation("lever"))
                angle_deg = (
                    float("nan") if measured_angle is None
                    else math.degrees(measured_angle))
                print(
                    "{} measured right grip={} lever={:.1f} deg".format(
                        label,
                        [round(value, 6) for value in measured_tcp],
                        angle_deg))

            with timing.segment("lever.hand_initialize"):
                measured = gripper.measured_positions("right")
                target = lever_hand_targets((0.6, 0.6, 0.6))
                for step in range(1, 51):
                    alpha = step / 50.0
                    with gripper.command_lock:
                        gripper.current_right_positions = [a + alpha * (b - a)
                            for a, b in zip(measured, target)]
                        gripper.command_initialized.set()
                    time.sleep(0.01)
            move_right_tcp(
                "lever_approach_contact", contact_target,
                Rotation.from_matrix(contact_rotation_world).as_quat(),
                point_in_r7=LEVER_GRIP_CENTER_IN_R7,
                trajectory_points=155, arc_height=0.0, joint_bounds=lever_bounds,
                via_position_world=approach_target,
                via_quaternion_world=Rotation.from_matrix(contact_rotation_world).as_quat())
            time.sleep(0.15)
            grip_command = list(contact_target)
            for alignment in range(3):
                live_right = list(robot_state.arm_joint_state().position)[7:14]
                actual_grip = scene_ik._forward(live_right, LEVER_GRIP_CENTER_IN_R7)[0]
                error = [goal - actual for goal, actual in zip(contact_target, actual_grip)]
                if max(abs(v) for v in error) < 0.003:
                    break
                grip_command = [command + max(-0.025, min(0.025, delta))
                                for command, delta in zip(grip_command, error)]
                move_right_tcp(
                    "lever_grip_align_{}".format(alignment), grip_command,
                    Rotation.from_matrix(contact_rotation_world).as_quat(),
                    point_in_r7=LEVER_GRIP_CENTER_IN_R7,
                    trajectory_points=25, arc_height=0.0, joint_bounds=lever_bounds)
                time.sleep(0.15)
            # First seat the palm face. The hinge remains locked while the
            # open fingers surround the rod; palm contact alone cannot lift it.
            normal = contact_rotation_world[:, 1]
            for seating in range(16):
                if palm_seated[0]:
                    break
                grip_command = [v + 0.0015 * n for v, n in zip(grip_command, normal)]
                joints = scene_ik.solve(
                    grip_command, Rotation.from_matrix(contact_rotation_world).as_quat(),
                    [math.radians(v) for v in current_target_deg[7:14]],
                    point_in_r7=LEVER_GRIP_CENTER_IN_R7,
                    seed_only=True, joint_bounds=lever_bounds)
                move_right_joint_target("lever_palm_contact_{}".format(seating),
                                        joints, 10, continuous=True)
                time.sleep(0.06)
            palm_subscriber.unregister()
            if not palm_seated[0]:
                raise RuntimeError("lever palm face contact not established")
            print("V2 lever palm face seated; closing fingers")
            print_lever_state("v2_lever_before_close")
            with timing.segment("lever.grasp_close"):
                flexions = [0.6, 0.6, 0.6]
                contact_limits = [None, None, None]
                deadline = time.monotonic() + 10.0
                palm_corrections = 0
                while not grasp_confirmed[0] and time.monotonic() < deadline:
                    # Closing fingers can push the compliant wrist away from
                    # the rod. Restore palm contact before closing further.
                    if not grip_contacts[0] & 1 and time.monotonic() - palm_contact_changed[0] > 0.12:
                        if palm_corrections >= 24:
                            raise RuntimeError("lever palm contact lost while closing fingers")
                        grip_command = [v + 0.0015 * n
                                        for v, n in zip(grip_command, normal)]
                        joints = scene_ik.solve(
                            grip_command, Rotation.from_matrix(contact_rotation_world).as_quat(),
                            [math.radians(v) for v in current_target_deg[7:14]],
                            point_in_r7=LEVER_GRIP_CENTER_IN_R7,
                            seed_only=True, joint_bounds=lever_bounds)
                        move_right_joint_target("lever_palm_hold_{}".format(palm_corrections),
                                                joints, 5, continuous=True)
                        palm_corrections += 1
                        time.sleep(0.04)
                        continue
                    for finger in range(3):
                        touching = grip_contacts[0] & (1 << (finger + 1))
                        if touching and contact_limits[finger] is None:
                            contact_limits[finger] = flexions[finger] + 0.03
                        limit = contact_limits[finger]
                        if limit is None:
                            limit = 1.7
                        flexions[finger] = min(limit, flexions[finger] +
                                              (0.001 if touching else 0.006))
                    with gripper.command_lock:
                        gripper.current_right_positions = lever_hand_targets(flexions)
                        gripper.command_initialized.set()
                    time.sleep(0.01)
                time.sleep(0.10)
            contact_subscriber.unregister()
            if not grasp_confirmed[0]:
                raise RuntimeError("lever palm/finger grip not established: contacts={}".format(grip_contacts[0]))
            print("V2 lever palm/finger physical grip confirmed: flexions={}".format(flexions))

            lever_angle = _lever_angle_from_orientation(
                poses.get_orientation("lever"))
            if lever_angle is None:
                raise RuntimeError("lever orientation is unavailable")
            grip_anchor_angle = lever_angle
            # Keep the command endpoint so confirming contact cannot unload
            # the grip by resetting the target to lagging measured joints.
            grip_anchor_q = [math.radians(v) for v in current_target_deg[7:14]]
            grip_anchor_tcp = scene_ik._forward(
                grip_anchor_q, LEVER_GRIP_CENTER_IN_R7)[0]
            grip_anchor_rotation_world = scene_ik.eef_rotation_world(grip_anchor_q)
            stalls = 0
            attempts = 0
            # Advance the hand along the lever arc while checking the real
            # hinge angle and the scene's conveyor handoff event.
            commanded_angle = grip_anchor_angle
            while (not source_bin_release.released
                   and attempts < LEVER_PATH_MAX_ATTEMPTS
                   and lever_angle < LEVER_PATH_SAFETY_MAX_RAD):
                fine_pull = (
                    commanded_angle + LEVER_PATH_FINE_START_TOLERANCE_RAD
                    >= LEVER_PATH_FINE_START_RAD)
                step_rad = (
                    LEVER_PATH_FINE_STEP_RAD
                    if fine_pull else LEVER_PATH_STEP_RAD)
                target_angle = min(
                    commanded_angle + step_rad,
                    LEVER_PATH_SAFETY_MAX_RAD)
                if not fine_pull:
                    target_angle = min(
                        target_angle, LEVER_PATH_FINE_START_RAD)
                commanded_angle = target_angle
                anchor_delta_angle = target_angle - grip_anchor_angle
                rotated_grip = _rotate_vector_about_world_y(
                    tuple(value - pivot for value, pivot in
                          zip(grip_anchor_tcp, LEVER_PIVOT_WORLD)),
                    anchor_delta_angle)
                anchored_target = tuple(pivot + value for pivot, value in
                                        zip(LEVER_PIVOT_WORLD, rotated_grip))
                target_rotation_world = scene_ik.rotate_about_world_y(
                    grip_anchor_rotation_world, anchor_delta_angle)
                live_arm_rad = list(robot_state.arm_joint_state().position)
                label = "v2_lever_closed_loop_{:02d}".format(attempts)
                with timing.segment("{}.ik_solve".format(label)):
                    path_target = scene_ik.solve(
                        anchored_target,
                        Rotation.from_matrix(target_rotation_world).as_quat(),
                        [math.radians(value) for value in current_target_deg[7:14]],
                        point_in_r7=LEVER_GRIP_CENTER_IN_R7,
                        seed_only=True, joint_bounds=lever_bounds)
                # The current URDF tracks the requested r5 closely.  Use the
                # live step's IK value from the first pull step onward instead
                # of carrying over a fixed compensation from an older model.
                r5_command = path_target[4]
                point_count = (
                    LEVER_PATH_FINE_TRAJECTORY_POINTS
                    if fine_pull else LEVER_PATH_TRAJECTORY_POINTS)
                # Allow the newly loaded finger grip to settle at lift-off.
                if lever_angle < math.radians(1.0):
                    point_count = max(point_count, 25)
                # Do not use the zero-velocity smootherstep here.  Applying
                # it independently to every 2.5-degree waypoint forces the
                # wrist to brake to a stop and restart at every boundary.
                # Linear command interpolation keeps the pull moving while
                # retaining the measured-angle/release check between steps.
                move_right_joint_target(
                    label, path_target, point_count,
                    r5_command_rad=r5_command, continuous=True)
                if not grip_contacts[0] & 16 and time.monotonic() - grip_contacts[1] > 0.35:
                    raise RuntimeError("lever grip lost during lift: contacts={}".format(grip_contacts[0]))
                new_angle = _lever_angle_from_orientation(
                    poses.get_orientation("lever"))
                if new_angle is None:
                    raise RuntimeError("lever orientation is unavailable")
                progress = new_angle - lever_angle
                print_lever_state(label)
                progress_threshold = min(
                    LEVER_PATH_MIN_PROGRESS_RAD, step_rad * 0.5)
                stalls = stalls + 1 if progress < progress_threshold else 0
                lever_angle = new_angle
                attempts += 1
                if (not source_bin_release.released
                        and stalls >= LEVER_PATH_MAX_STALLS):
                    raise RuntimeError(
                        "V2 right lever stalled for {} steps at {:.1f} deg"
                        .format(stalls, math.degrees(lever_angle)))

            if not source_bin_release.released:
                raise RuntimeError(
                    "V2 source bin did not enter conveyor motion after "
                    "{} pull steps (lever {:.1f} deg)".format(
                        attempts, math.degrees(lever_angle)))
            latch_achieved_lever_target("v2_lever_handoff_hold")
            print("V2 source bin handoff triggered at lever {:.1f} deg".format(
                math.degrees(lever_angle)))
            with timing.segment("lever.conveyor_wait"):
                conveyor_complete = rospy.wait_for_message(
                    SOURCE_CONVEYOR_COMPLETE_TOPIC, Bool,
                    timeout=SOURCE_CONVEYOR_TIMEOUT_S)
            if not conveyor_complete.data:
                raise RuntimeError(
                    "V2 source conveyor reported an incomplete move")
            grip_subscriber.unregister()
            deadline = time.monotonic() + SOURCE_CONVEYOR_TIMEOUT_S
            while time.monotonic() < deadline:
                state = rospy.wait_for_message("/mujoco/source_bin/scoring_state", UInt8, timeout=2.0)
                if state.data & 16:
                    break
            else:
                raise RuntimeError("V2 source bin reached conveyor end but did not settle")
            print("V2 source bin conveyor motion complete and settled")

        scorer_started = start_score_clock()
        timing.mark_score_start(scorer_started)
        task_tcp_quaternion_world = (
            Rotation.from_euler("y", 15.0, degrees=True)
            * Rotation.from_quat(scene_ik.eef_quaternion_world_with_yaw(
                RIGHT_ARM_READY_FULL_RAD, GRASP_YAW_ADJUSTMENT_RAD))).as_quat()

        # Form the source-grasp posture above the first cylinder, keeping
        # the moving fingers clear before the direct downward approach.
        with timing.segment("source_grasp.hand_prepare"):
            gripper.control_right_gripper(0, duration=0.5)

        grasp_lower = scene_ik.lower.copy()
        grasp_lower[1] = -1.6
        for index, name in enumerate(CYLINDERS):
            with timing.segment("{}.target_prepare".format(name)):
                object_position = poses.get_position(name)
                grasp_bias_world = GRASP_TRACKING_BIASES_WORLD[index]
                grasp_center_world = [
                    object_position[axis] - grasp_bias_world[axis]
                    for axis in range(3)
                ]
                drop_center_world = [
                    target_bin_position[0] + DROP_CENTER_OFFSETS[index][0],
                    target_bin_position[1] + DROP_CENTER_OFFSETS[index][1],
                    DROP_GRASP_CENTER_Z,
                ]

            move_right_tcp(
                "{}_above".format(name),
                grasp_center_world,
                task_tcp_quaternion_world,
                point_in_r7=scene_ik.CLOSED_GRASP_CENTER_IN_R7,
                joint_bounds=(grasp_lower, scene_ik.upper),
                trajectory_points=60 if index == 0 else 120,
                arc_height=0.0,
                clearance_height=None if index == 0 else DROP_GRASP_CENTER_Z,
            )
            print_measured_grasp_center("{}_open".format(name))
            with timing.segment("{}.pre_close_wait".format(name)):
                time.sleep(SOURCE_GRASP_SETTLE_SECONDS)
            # Correct the measured approach error before closing. A fixed
            # feedforward bias alone cannot account for wrist load/settling.
            measured_right = list(robot_state.arm_joint_state().position)[7:14]
            actual_center = scene_ik.closed_grasp_center_world(measured_right)
            object_center = poses.get_position(name)
            corrected_center = [command + target - actual for command, target, actual
                                in zip(grasp_center_world, object_center, actual_center)]
            actual_rotation = Rotation.from_quat(
                scene_ik.eef_quaternion_world(measured_right))
            desired_rotation = Rotation.from_quat(task_tcp_quaternion_world)
            corrected_rotation = (desired_rotation * actual_rotation.inv()
                                  * desired_rotation).as_quat()
            move_right_tcp(
                "{}_align".format(name), corrected_center, corrected_rotation,
                point_in_r7=scene_ik.CLOSED_GRASP_CENTER_IN_R7,
                joint_bounds=(grasp_lower, scene_ik.upper),
                trajectory_points=15)
            time.sleep(0.05)
            pre_close_object_position = poses.get_position(name)
            record_grasp_snapshot(
                "pre_close", name, grasp_center_world,
                pre_close_object_position)
            try:
                with timing.segment("{}.gripper_close".format(name)):
                    close_on_cylinder(gripper, name)
            finally:
                record_grasp_snapshot(
                    "post_close", name, grasp_center_world,
                    pre_close_object_position)
            print_measured_grasp_geometry(
                "{}_closed".format(name), poses.get_position(name))
            live_right = list(robot_state.arm_joint_state().position)[7:14]
            held_center = scene_ik.closed_grasp_center_world(live_right)
            held_quaternion = scene_ik.eef_quaternion_world(live_right)
            lift_center = list(held_center)
            lift_center[2] = DROP_GRASP_CENTER_Z
            drop_center_world[2] = DROP_GRASP_CENTER_Z
            drop_rotation = (Rotation.from_euler("z", 75.0, degrees=True)
                             * Rotation.from_euler("x", -90.0, degrees=True))
            object_offset = [obj - hand for obj, hand in
                             zip(poses.get_position(name), held_center)]
            drop_offset = drop_rotation.apply(
                Rotation.from_quat(held_quaternion).inv().apply(object_offset))
            for axis in (0, 1):
                drop_center_world[axis] -= drop_offset[axis]

            def check_physical_lift():
                if poses.get_position(name)[2] - pre_close_object_position[2] < 0.08:
                    raise RuntimeError("{} did not lift off the source bin: before={}, after={}".format(
                        name, pre_close_object_position, poses.get_position(name)))
                record_grasp_snapshot(
                    "post_lift", name, grasp_center_world, pre_close_object_position)

            # Plan once before moving. The height checkpoint does not replan
            # or restart a zero-velocity profile at the lift/transfer join.
            move_right_tcp(
                "{}_lift_and_transfer".format(name), drop_center_world,
                drop_rotation.as_quat(),
                point_in_r7=scene_ik.CLOSED_GRASP_CENTER_IN_R7,
                joint_bounds=(grasp_lower, scene_ik.upper),
                trajectory_points=130, arc_height=0.0,
                via_position_world=lift_center,
                via_quaternion_world=held_quaternion,
                on_clearance=check_physical_lift)
            print("{}_above_target object: {}".format(
                name,
                [round(value, 6) for value in poses.get_position(name)]))
            release_cylinder(gripper, name, timing)
            final_position = poses.get_position(name)
            record_grasp_snapshot(
                "post_release", name, drop_center_world,
                pre_close_object_position)
            print("{}_settled object: {}".format(
                name, [round(value, 6) for value in final_position]))
            if not _fully_in_target(final_position, target_bin_position):
                message = "{} is not fully inside the V2 target bin: {}".format(
                    name, final_position)
                if experiment_mode:
                    print("[V2 EXPERIMENT] {} ; continuing".format(message))
                else:
                    raise RuntimeError(message)


        right_task_success = all(
            _fully_in_target(poses.get_position(name), target_bin_position) for name in CYLINDERS)
        print("v2_right_task_final objects: {}".format({
            name: [round(value, 6) for value in poses.get_position(name)]
            for name in CYLINDERS
        }))
        latch_achieved_right_target("v2_right_task_complete")
        with timing.segment("lever.total"):
            run_right_lever_stage()
        success = right_task_success
    finally:
        if diagnostics is not None:
            diagnostics.write(
                "round_end",
                success=bool(success),
            )
        with timing.segment("cleanup.result_write"):
            RESULT_PATH.write_text("success" if success else "fail")
        with timing.segment("cleanup.gripper_stop"):
            gripper.stop()
        with timing.segment("cleanup.chassis_stop"):
            chassis.stop()
        if trajectory is not None:
            with timing.segment("cleanup.trajectory_stop"):
                trajectory.stop()
        timing.print_summary(success)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print("Task 1 V2 failed: {}".format(error))
        import traceback
        traceback.print_exc()
        sys.exit(1)
