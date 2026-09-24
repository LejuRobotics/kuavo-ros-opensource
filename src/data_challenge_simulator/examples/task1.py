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
from std_msgs.msg import Bool

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(CURRENT_DIR)
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
from task2_base_motion import ChassisMotion
from utils.gripper_controller import GripperController
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.scene1_v2_right_arm_ik import Scene1V2RightArmIK
from utils.scorer_clock import start_score_clock
from utils.arm_state import wait_for_first_arm_state
from utils.task1_v2_randomization import Task1V2RandomizationPlanner
from utils.trajectory_controller import TrajectoryController
from utils.utils import Utils


CYLINDERS = ("cylinder_1", "cylinder_2", "cylinder_3")
TARGET_BIN = "target_bin"
RESULT_PATH = Path("task_result.txt")

INITIALIZATION_SAFE_RETREAT_M = 0.20
LEFT_THUMB_SAFE_J1_RAD = 1.75
LEFT_THUMB_INITIALIZATION_DURATION_S = 1.2

# V2 target-bin wall inner bounds.  _fully_in_target tests the cylinder centre
# against this box directly: the whole bin interior counts, with no cylinder
# radius subtracted from it.
TARGET_MIN = (0.2075, -0.5852, 0.515)
TARGET_MAX = (0.4075, -0.4708, 0.590)

# Current-URDF calibration for the closed thumb/index pad midpoint.  The old
# (8, 15, 18) mm world bias already included the former wrist tracking error;
# retaining it together with the joint-space correction below overcompensates
# the grasp.  Keep a common 10 mm Z bias so the achieved pad midpoint remains
# about 2--4 mm below the cylinder centre.  Y is pose-dependent: each source
# region needs the closing fingers to approach from the same safe side.
GRASP_TRACKING_BIASES_WORLD = (
    (0.002, 0.014, 0.010),
    (0.002, 0.010, 0.010),
    (0.002, 0.005, 0.010),
)
GRASP_CLOSURE_CMD = 255
GRASP_YAW_ADJUSTMENT_RAD = math.radians(14.0)

# Current-URDF source-grasp calibration from three settled Task1 poses.  Only
# r5/r7 showed material tracking error; invert their measured affine response
# so the achieved joints, rather than merely the commands, match the IK pose.
GRASP_R5_ACTUAL_PER_COMMAND = 1.14754746
GRASP_R5_ACTUAL_OFFSET_RAD = -0.16735713
GRASP_R7_ACTUAL_PER_COMMAND = 0.97868007
GRASP_R7_ACTUAL_OFFSET_RAD = -0.04604092

# V2 has taller obstacles around both bins.  Raise the release centre by
# 60 mm while retaining the accepted X/Y offsets and live target-bin centre.
DROP_GRASP_CENTER_Z = 0.82
# Keep the first release slot 10 mm farther inside the narrow Y boundary.
# Wide source randomization can preserve about 13 mm of object/hand tracking
# offset during transport, so the former -22 mm slot had only nominal margin.
DROP_CENTER_OFFSETS = (
    (-0.060, -0.012),
    (0.000, 0.022),
    (0.060, -0.012),
)
FIXED_GRASP_LIFTS_RIGHT_RAD = (
    (0.210334, -0.338870, -0.036279, -0.850745,
     -0.089775, 0.299690, 0.193743),
    (0.189635, -0.314196, -0.020128, -0.875594,
     -0.022019, 0.340597, 0.248531),
    (0.210334, -0.338870, -0.036279, -0.850745,
     -0.089775, 0.299690, 0.193743),
)

ARM_JOINT_COUNT = 14
HEAD_SCAN_PITCH_RAD = math.radians(20.0)
SHOULDER_LIFT_DEG = 60.0
# Keep the accepted grasp/lever orientation reference independent from the
# lower task-only staging posture used before each source grasp.
RIGHT_ARM_READY_FULL_RAD = (-0.9, -0.265, 1.0, -0.8, 0.58, -0.4, 0.35)
RIGHT_ARM_STAGING_RAD = (
    -0.569927, -0.301512, 0.658253, -0.488704,
    0.698827, -0.653404, -0.289423,
)
# Same nearby workspace and wrist attitude as staging, with the closed-grasp
# centre 0.10 m higher.  This waypoint keeps the joint-space route from the
# shoulder-clearance pose above the source-bin and lever obstacles.
RIGHT_ARM_STAGING_ABOVE_RAD = (
    -0.466591, -0.579416, 0.652630, -1.211603,
    0.699260, -0.286390, 0.045867,
)
TRAJECTORY_POINTS = 80
IK_TRAJECTORY_POINTS = 120
SOURCE_GRASP_TRAJECTORY_POINTS = 120
SOURCE_GRASP_SETTLE_SECONDS = 0.3
STAGING_RETURN_TRAJECTORY_POINTS = 80
STAGING_DESCENT_TRAJECTORY_POINTS = 80
GRASP_LIFT_TRAJECTORY_POINTS = 80
# Scaling the accepted fixed-joint lift from 1.20 to 1.65 raises the measured
# grasp centre by about 53--55 mm for both source objects.
GRASP_LIFT_SCALE = 1.65

# The V2 right-arm lever stance is the user's lower-left direction in the
# scene: world -X and +Y.  Reducing world Y from 0.30 m to 0.20 m makes the
# complete handle-relative wrist rotation reachable through 30 degrees.
LEVER_BASE_WORLD = (-0.20, 0.20, 0.0)
# Closed-loop wheel-drive base motion (same contract as Task 2/3): the
# physical wheels roll and the returned pose is the MEASURED one, never the
# nominal target.  Speeds/tolerances mirror the accepted Task 2 values.
CHASSIS_LINEAR_SPEED = 0.08
CHASSIS_ANGULAR_SPEED = 0.20
CHASSIS_MIN_LINEAR_SPEED = 0.06
CHASSIS_MIN_ANGULAR_SPEED = 0.06
CHASSIS_POSITION_TOLERANCE = 0.03
CHASSIS_YAW_TOLERANCE_DEG = 3.0
CHASSIS_MOVE_TIMEOUT_S = 30.0

LEVER_PIVOT_WORLD = (0.62, -0.196, 0.62)
LEVER_HANDLE_LOCAL = (-0.32, 0.0, 0.08)
LEVER_HOOK_TANGENT_OFFSET = 0.026
LEVER_TRACKING_BIAS_WORLD = (0.0, 0.0, 0.010)
LEVER_APPROACH_HEIGHT = 0.08
LEVER_NOMINAL_HANDOFF_RAD = math.radians(30.0)
LEVER_PATH_STEP_RAD = math.radians(2.5)
LEVER_PATH_FINE_STEP_RAD = math.radians(0.2)
# Keep the accepted coarse pull until the old 29.8-degree acceptance edge.
# The last coarse target is capped here, then only 0.2-degree increments are
# allowed while waiting for the source-bin handoff event.
LEVER_PATH_FINE_START_RAD = math.radians(29.8)
LEVER_PATH_SAFETY_MAX_RAD = math.radians(35.0)
LEVER_PATH_MAX_ATTEMPTS = 60
LEVER_PATH_MIN_PROGRESS_RAD = math.radians(0.2)
LEVER_PATH_MAX_STALLS = 5
# Preserve the accepted zero-angle hand pose while allowing the high-angle
# continuation to use the extra physical joint travel it needs.
LEVER_INITIAL_JOINT_LIMIT_MARGIN_FRACTION = 0.10
LEVER_PATH_JOINT_LIMIT_MARGIN_FRACTION = 0.02
LEVER_IK_PREPARE_TRAJECTORY_POINTS = 200
LEVER_APPROACH_TRAJECTORY_POINTS = 200
LEVER_CONTACT_TRAJECTORY_POINTS = 80
LEVER_PATH_TRAJECTORY_POINTS = 30
# One-time pre-IK posture measured immediately before the successful seed-2
# lever solve.  It is only a physical preparation target: after reaching it,
# the live achieved joints are read back and the normal lever IK still solves
# the current world target for the current measured base pose.
LEVER_IK_PREPARE_RIGHT_RAD = (
    0.715789,
    -1.078706,
    -0.254394,
    -1.257083,
    0.034689,
    -0.591775,
    -0.698122,
)
# Task1 lever initial r5 command calibrated for the current URDF.  The current
# contact response is nearly one-to-one, so only a small offset above the
# nominal contact IK value (about 0.98 rad) is required.
TASK1_LEVER_INITIAL_R5_COMMAND_RAD = 1.0
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


def _compensate_source_grasp_joints(right_ik_rad):
    """Return the current-URDF command that achieves the nominal grasp IK."""
    commanded = list(right_ik_rad)
    commanded[4] = (
        commanded[4] - GRASP_R5_ACTUAL_OFFSET_RAD
    ) / GRASP_R5_ACTUAL_PER_COMMAND
    commanded[6] = (
        commanded[6] - GRASP_R7_ACTUAL_OFFSET_RAD
    ) / GRASP_R7_ACTUAL_PER_COMMAND
    return commanded


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
    """Apply one reproducible scene sampled from the verified feasible set.

    The base spawn itself comes from the launch ``initial_base_x/y`` args
    (same mechanism as Task 2/3); this function only places the task
    objects.  ``initial_base`` is a thumb-safe spawn while ``task_base`` is
    the final working pose.  Do not drive to the working pose here: fixed
    initialization first moves backward to its safe arm-clearance pose,
    closes the left thumb, raises the right arm, and only then drives forward.
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
        "V2 seed {} selected precomputed layout {}: initial_base={} "
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
    if chassis.pose() is not None:
        print("V2 seed {} launch-time measured base pose: {}".format(
            seed, [round(value, 6) for value in chassis.pose()]))
    return plan.task_base


def _fully_in_target(position):
    if position is None:
        return False
    return all(
        TARGET_MIN[index] <= position[index] <= TARGET_MAX[index]
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


def _lever_angle_from_orientation(quaternion_xyzw):
    if quaternion_xyzw is None:
        return None
    x, y, z, w = quaternion_xyzw
    vector_norm = math.sqrt(x * x + y * y + z * z)
    angle = 2.0 * math.atan2(vector_norm, w)
    return 2.0 * math.pi - angle if angle > math.pi else angle


def _lever_hand_target_world(angle):
    """Return the hook row behind the extended handle's circular motion."""
    cosine = math.cos(angle)
    sine = math.sin(angle)
    local_x, _, local_z = LEVER_HANDLE_LOCAL
    handle_x = (
        LEVER_PIVOT_WORLD[0] + cosine * local_x + sine * local_z)
    handle_z = (
        LEVER_PIVOT_WORLD[2] - sine * local_x + cosine * local_z)
    tangent_x = -sine * local_x + cosine * local_z
    tangent_z = -cosine * local_x - sine * local_z
    tangent_norm = math.hypot(tangent_x, tangent_z)
    return (
        handle_x - LEVER_HOOK_TANGENT_OFFSET * tangent_x / tangent_norm,
        LEVER_PIVOT_WORLD[1],
        handle_z - LEVER_HOOK_TANGENT_OFFSET * tangent_z / tangent_norm,
    )


def _lever_command_target_world(target_world):
    return tuple(
        value - bias
        for value, bias in zip(target_world, LEVER_TRACKING_BIAS_WORLD))


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


def _move_base_with_wheels(chassis, target_world):
    """Drive the base on physical wheels to a world XY target.

    Returns the MEASURED settled pose (x, y, yaw) — the task IK must be
    seeded from this, never from the nominal target, because wheel slip
    and closed-loop tolerances replace the old weld teleport's exactness.
    """
    chassis.wait_until_ready()
    settled = chassis.move_to_pose(
        target_world[0], target_world[1], 0.0,
        timeout=CHASSIS_MOVE_TIMEOUT_S)
    print("V2 base wheel-drive reached: measured={} nominal={}".format(
        [round(value, 6) for value in settled],
        [round(value, 6) for value in target_world]))
    return settled


def main():
    timing = TaskTimingRecorder()
    experiment_mode = os.environ.get("TASK_EXPERIMENT_MODE", "1") == "1"
    with timing.segment("setup.sdk_init"):
        sdk_initialized = KuavoSDK().Init(options=KuavoSDK.Options.WithIK)
    if not sdk_initialized:
        raise RuntimeError("KuavoSDK initialization with IK failed")

    with timing.segment("setup.controllers"):
        robot = KuavoRobot()
        robot_state = KuavoRobotState()
        gripper = GripperController()
        source_bin_release = SourceBinReleaseMonitor()
        chassis = _make_chassis()
    success = False
    trajectory = None

    # Head motion is independent of arm ownership.  Let it start before the
    # chassis retreat; the arm stays in its controller-owned startup mode
    # until the robot has reached the safe initialization base.
    if not robot.control_head(yaw=0.0, pitch=HEAD_SCAN_PITCH_RAD):
        raise RuntimeError("Failed to command the 20-degree head scan pose")

    try:
        seed = int(os.environ.get("TASK_SEED", "1"))
        randomizer = ObjectRandomizer(timeout=60.0)
        with timing.segment("initialization.randomize_scene", seed=seed):
            base_translation_world = _randomize_scene(
                randomizer, seed, chassis)

        scene_ik = Scene1V2RightArmIK()
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

        initialization_safe_base_world = (
            base_translation_world[0] - INITIALIZATION_SAFE_RETREAT_M,
            base_translation_world[1],
            base_translation_world[2],
        )
        print(
            "V2 initialization retreat: B0={} safe={} distance={:.3f} m"
            .format(
                [round(value, 6) for value in base_translation_world],
                [round(value, 6)
                 for value in initialization_safe_base_world],
                INITIALIZATION_SAFE_RETREAT_M,
            ))
        with timing.segment("initialization.base_retreat"):
            measured_safe_base = _move_base_with_wheels(
                chassis, initialization_safe_base_world)
        scene_ik.set_base_translation_world(
            (measured_safe_base[0], measured_safe_base[1],
             initialization_safe_base_world[2]))

        # Recording starts only after fixed initialization.  Delay the first
        # arm command and ExternalControl handoff until the safe retreat has
        # finished, then seed it from the arm pose measured at that location.
        with timing.segment("initialization.first_arm_state_wait"):
            initial_arm_rad = wait_for_first_arm_state()
        trajectory = TrajectoryController(
            robot, initial_positions=initial_arm_rad)
        left_hold_rad = list(initial_arm_rad[:7])
        current_target_deg = [math.degrees(value) for value in initial_arm_rad]

        # The left arm remains fixed for the entire task.  At the temporary
        # safe base, rotate only the thumb opposition joint so the thumb no
        # longer points along user +Y when the chassis returns to final B0.
        with timing.segment("initialization.left_thumb"):
            _initialize_left_thumb(gripper)

        def latch_achieved_right_target(label):
            nonlocal current_target_deg
            live_arm_rad = list(robot_state.arm_joint_state().position)
            if len(live_arm_rad) != ARM_JOINT_COUNT:
                raise RuntimeError(
                    "Expected 14 live arm joints after {}, got {}".format(
                        label, len(live_arm_rad)))
            achieved_deg = [math.degrees(value) for value in live_arm_rad]
            current_target_deg[7:14] = achieved_deg[7:14]
            trajectory.set_target_positions(
                left_hold_rad + live_arm_rad[7:14])
            print("{} accepted right-arm achieved joints: {}".format(
                label,
                [round(value, 6) for value in live_arm_rad[7:14]],
            ))
            return live_arm_rad

        last_right_ik_rad = None
        last_right_command_rad = None

        def move_right_tcp(
                label, tcp_position_world, tcp_quaternion_world,
                point_in_r7=None, compensate_source_grasp=False,
                trajectory_points=IK_TRAJECTORY_POINTS):
            nonlocal current_target_deg
            nonlocal last_right_ik_rad, last_right_command_rad
            print("Moving V2 right task TCP to {}: {}".format(
                label, tcp_position_world))
            live_arm_rad = list(robot_state.arm_joint_state().position)
            with timing.segment("{}.ik_solve".format(label)):
                right_joints = scene_ik.solve(
                    tcp_position_world,
                    tcp_quaternion_world,
                    live_arm_rad[7:14],
                    point_in_r7=point_in_r7,
                )
            print("{} right-arm IK solution: {}".format(
                label, [round(value, 6) for value in right_joints]))
            commanded_right = (
                _compensate_source_grasp_joints(right_joints)
                if compensate_source_grasp else list(right_joints))
            if compensate_source_grasp:
                print("{} compensated right-arm command: {}".format(
                    label,
                    [round(value, 6) for value in commanded_right]))
            live_start_deg = [math.degrees(value) for value in live_arm_rad]
            next_target_deg = list(current_target_deg)
            next_target_deg[7:14] = [
                math.degrees(value) for value in commanded_right]
            with timing.segment(
                    "{}.arm_motion".format(label),
                    trajectory_points=trajectory_points):
                trajectory.execute_trajectory(
                    _right_only_trajectory(
                        next_target_deg, live_start_deg, left_hold_rad,
                        num=trajectory_points),
                    sleep_time=0.02)
            current_target_deg = next_target_deg
            last_right_ik_rad = list(right_joints)
            last_right_command_rad = commanded_right

        def move_right_by_fixed_delta(label, right_delta_rad):
            nonlocal current_target_deg
            target_deg = list(current_target_deg)
            target_deg[7:14] = [
                target_deg[7 + index] + math.degrees(delta)
                for index, delta in enumerate(right_delta_rad)
            ]
            live_arm_rad = list(robot_state.arm_joint_state().position)
            with timing.segment(
                    "{}.arm_motion".format(label),
                    trajectory_points=GRASP_LIFT_TRAJECTORY_POINTS):
                trajectory.execute_trajectory(
                    _right_only_trajectory(
                        target_deg,
                        [math.degrees(value) for value in live_arm_rad],
                        left_hold_rad,
                        num=GRASP_LIFT_TRAJECTORY_POINTS),
                    sleep_time=0.02)
            current_target_deg = target_deg
            print("Executed fixed V2 right-arm motion: {}".format(label))

        def return_right_to_staging(label):
            nonlocal current_target_deg
            live_start_deg = [
                math.degrees(value)
                for value in robot_state.arm_joint_state().position]
            above_target_deg = list(current_target_deg)
            above_target_deg[7:14] = [
                math.degrees(value)
                for value in RIGHT_ARM_STAGING_ABOVE_RAD]
            with timing.segment(
                    "{}.above_motion".format(label),
                    trajectory_points=STAGING_RETURN_TRAJECTORY_POINTS):
                trajectory.execute_trajectory(
                    _right_only_trajectory(
                        above_target_deg,
                        live_start_deg,
                        left_hold_rad,
                        num=STAGING_RETURN_TRAJECTORY_POINTS),
                    sleep_time=0.02)
            current_target_deg = above_target_deg

            live_arm_rad = list(robot_state.arm_joint_state().position)
            staging_target_deg = list(current_target_deg)
            staging_target_deg[7:14] = [
                math.degrees(value) for value in RIGHT_ARM_STAGING_RAD]
            with timing.segment(
                    "{}.descent_motion".format(label),
                    trajectory_points=STAGING_DESCENT_TRAJECTORY_POINTS):
                trajectory.execute_trajectory(
                    _right_only_trajectory(
                        staging_target_deg,
                        [math.degrees(value) for value in live_arm_rad],
                        left_hold_rad,
                        num=STAGING_DESCENT_TRAJECTORY_POINTS),
                    sleep_time=0.02)
            current_target_deg = staging_target_deg
            latch_achieved_right_target(label)

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

        def run_right_lever_stage():
            """Move lower-left, reshape the raised right hand, then pull."""
            nonlocal current_target_deg
            with timing.segment("lever.pose_wait"):
                poses.wait_for_position("lever", timeout=5.0)

            with timing.segment("lever.base_motion"):
                measured_lever_base = _move_base_with_wheels(
                    chassis, LEVER_BASE_WORLD)
            scene_ik.set_base_translation_world(
                (measured_lever_base[0], measured_lever_base[1],
                 LEVER_BASE_WORLD[2]))

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
                    r5_command_rad=None):
                nonlocal current_target_deg
                live = list(robot_state.arm_joint_state().position)
                nominal_r5 = right_target_rad[4]
                commanded_right = list(right_target_rad)
                commanded_right[4] = (
                    TASK1_LEVER_INITIAL_R5_COMMAND_RAD
                    if r5_command_rad is None
                    else r5_command_rad)
                target_deg = list(current_target_deg)
                target_deg[7:14] = [
                    math.degrees(value) for value in commanded_right]
                print("Executing V2 right-arm lever motion: {}".format(label))
                print(
                    "{} nominal_r5={:.9f} rad commanded_r5={:.9f} rad".format(
                        label, nominal_r5, commanded_right[4]))
                trajectory_points = (
                    _right_only_smooth_trajectory(
                        target_deg,
                        [math.degrees(value) for value in live],
                        left_hold_rad,
                        num=point_count)
                    if smooth else
                    _right_only_trajectory(
                        target_deg,
                        [math.degrees(value) for value in live],
                        left_hold_rad,
                        num=point_count)
                )
                with timing.segment(
                        "{}.arm_motion".format(label),
                        trajectory_points=point_count):
                    trajectory.execute_trajectory(
                        trajectory_points,
                        sleep_time=0.02)
                actual = list(robot_state.arm_joint_state().position)
                print("{} actual_r5={:.9f} rad".format(label, actual[11]))
                current_target_deg = target_deg

            def latch_achieved_lever_target(label):
                nonlocal current_target_deg
                live = list(robot_state.arm_joint_state().position)
                achieved_deg = [math.degrees(value) for value in live]
                current_target_deg[7:14] = achieved_deg[7:14]
                trajectory.set_target_positions(left_hold_rad + live[7:14])
                print("{} accepted right-arm achieved joints: {}".format(
                    label, [round(value, 6) for value in live[7:14]]))
                return live

            def initialize_right_lever_hand_before_descent():
                """Flatten the thumb base while preserving its measured bent tip."""
                measured = gripper.measured_positions("right")
                if measured is None:
                    raise RuntimeError(
                        "Right-hand state is unavailable before lever descent")
                initialization_targets = {
                    "r_thumb_j1": 0.0,
                    "r_thumb_j2": 0.0,
                    "r_index_j3": 0.0,
                }
                steps = max(1, int(round(
                    gripper.motion_duration * gripper.publish_frequency)))
                for step in range(1, steps + 1):
                    alpha = float(step) / steps
                    positions = list(measured)
                    for name, end in initialization_targets.items():
                        joint = gripper.right_names.index(name)
                        positions[joint] = (
                            measured[joint]
                            + alpha * (end - measured[joint]))
                    with gripper.command_lock:
                        gripper.current_right_positions = positions
                        gripper.command_initialized.set()
                    time.sleep(1.0 / gripper.publish_frequency)
                settled = gripper.measured_positions("right")
                print("v2_lever_hand_initialized {}".format({
                    name: round(settled[gripper.right_names.index(name)], 3)
                    if settled is not None else float("nan")
                    for name in initialization_targets
                }))

            def form_right_lever_hook():
                """Form the Task1 V2 hook with a less-curled little finger."""
                measured = gripper.measured_positions("right")
                if measured is None:
                    raise RuntimeError(
                        "Right-hand state is unavailable at lever contact")
                target_by_name = {
                    "r_index_j1": 0.0,
                    "r_index_j2": 0.0,
                    "r_index_j3": 1.5,
                    "r_middle_j1": 0.0,
                    "r_middle_j2": 1.5,
                    "r_little_j1": 0.0,
                    "r_little_j2": 0.0,
                    "r_little_j3": 1.3,
                }
                # Preserve the already-set thumb exactly; only the three hook
                # fingers transform after descent.  Their unspecified joints
                # retain the current measured values rather than being zeroed.
                target = list(measured)
                for index, name in enumerate(gripper.right_names):
                    if name.startswith("r_thumb_"):
                        continue
                    if name in target_by_name:
                        target[index] = target_by_name[name]
                steps = max(1, int(round(
                    gripper.motion_duration * gripper.publish_frequency)))
                for step in range(1, steps + 1):
                    alpha = float(step) / steps
                    positions = [
                        start + alpha * (end - start)
                        for start, end in zip(measured, target)
                    ]
                    with gripper.command_lock:
                        gripper.current_right_positions = positions
                        gripper.command_initialized.set()
                    time.sleep(1.0 / gripper.publish_frequency)
                settled = gripper.measured_positions("right")
                little_j3 = gripper.right_names.index("r_little_j3")
                print("v2_lever_hook_formed r_little_j3={:.3f} rad".format(
                    settled[little_j3]
                    if settled is not None else float("nan")))

            move_right_joint_target(
                "lever_ik_prepare",
                LEVER_IK_PREPARE_RIGHT_RAD,
                LEVER_IK_PREPARE_TRAJECTORY_POINTS,
                r5_command_rad=LEVER_IK_PREPARE_RIGHT_RAD[4])
            live_arm_rad = latch_achieved_lever_target(
                "v2_lever_ik_prepare")
            contact_target = _lever_hand_target_world(0.0)
            approach_target = (
                contact_target[0], contact_target[1],
                contact_target[2] + LEVER_APPROACH_HEIGHT)
            right_seed = live_arm_rad[7:14]
            # Derive one deterministic zero-angle wrist frame from the fixed
            # ready seed.  The live post-place seed can otherwise select a
            # different free roll about the handle axis whose 40-degree
            # continuation is not reachable from this base stance.
            with timing.segment("lever.canonical_contact.ik_solve"):
                canonical_contact_solution = scene_ik.solve_lever(
                    _lever_command_target_world(contact_target),
                    RIGHT_ARM_READY_FULL_RAD,
                    joint_limit_margin_fraction=(
                        LEVER_INITIAL_JOINT_LIMIT_MARGIN_FRACTION))
            contact_rotation_world = scene_ik.eef_rotation_world(
                canonical_contact_solution)
            with timing.segment("lever.approach.ik_solve"):
                approach_solution = scene_ik.solve_lever(
                    approach_target,
                    right_seed,
                    joint_limit_margin_fraction=(
                        LEVER_INITIAL_JOINT_LIMIT_MARGIN_FRACTION))
            with timing.segment("lever.contact.ik_solve"):
                contact_solution = scene_ik.solve_lever(
                    _lever_command_target_world(contact_target),
                    approach_solution,
                    target_rotation_world=contact_rotation_world,
                    joint_limit_margin_fraction=(
                        LEVER_INITIAL_JOINT_LIMIT_MARGIN_FRACTION))

            def print_lever_state(label):
                measured_arm = list(robot_state.arm_joint_state().position)
                measured_tcp = scene_ik.hook_tcp_world(
                    measured_arm[7:14])
                measured_angle = _lever_angle_from_orientation(
                    poses.get_orientation("lever"))
                angle_deg = (
                    float("nan") if measured_angle is None
                    else math.degrees(measured_angle))
                print(
                    "{} measured right finger row={} lever={:.1f} deg".format(
                        label,
                        [round(value, 6) for value in measured_tcp],
                        angle_deg))

            move_right_joint_target(
                "lever_approach", approach_solution,
                LEVER_APPROACH_TRAJECTORY_POINTS)
            latch_achieved_lever_target("v2_lever_approach")
            print_lever_state("v2_lever_approach")

            # Flatten only the thumb base and straighten the leftover bent
            # index in one motion above the handle.  Preserve the measured
            # bent thumb tip, then descend holding that posture.
            with timing.segment("lever.hand_initialize"):
                initialize_right_lever_hand_before_descent()
            move_right_joint_target(
                "lever_contact", contact_solution,
                LEVER_CONTACT_TRAJECTORY_POINTS)
            print_lever_state("v2_lever_contact")

            # Curl the three distal fingers only after the arm has descended;
            # the V2 little finger bends less so its pad aligns with the other
            # two instead of visibly cutting through the yellow handle.
            with timing.segment("lever.form_hook"):
                form_right_lever_hook()
            with timing.segment("lever.post_hook_wait"):
                time.sleep(0.5)
            print_lever_state("v2_lever_hook_formed")

            # Capture the achieved hook-to-handle transform after finger
            # closure.  Finger contact already moves the lever several
            # degrees, so continuing from the nominal zero-angle contact pose
            # creates a large catch-up command and visible penetration.
            lever_angle = _lever_angle_from_orientation(
                poses.get_orientation("lever"))
            if lever_angle is None:
                raise RuntimeError("lever orientation is unavailable")
            hook_anchor_angle = lever_angle
            live_arm_rad = list(robot_state.arm_joint_state().position)
            if len(live_arm_rad) != ARM_JOINT_COUNT:
                raise RuntimeError(
                    "Expected 14 live arm joints at lever hook, got {}".format(
                        len(live_arm_rad)))
            hook_anchor_q = live_arm_rad[7:14]
            hook_anchor_tcp = scene_ik.hook_tcp_world(hook_anchor_q)
            hook_anchor_rotation_world = scene_ik.eef_rotation_world(
                hook_anchor_q)
            nominal_anchor_target = _lever_command_target_world(
                _lever_hand_target_world(hook_anchor_angle))
            hook_anchor_offset = tuple(
                measured - nominal
                for measured, nominal in zip(
                    hook_anchor_tcp, nominal_anchor_target))
            print(
                "V2 lever hook anchor: angle={:.1f} deg offset={} m".format(
                    math.degrees(hook_anchor_angle),
                    [round(value, 6) for value in hook_anchor_offset]))
            stalls = 0
            attempts = 0
            while (not source_bin_release.released
                   and attempts < LEVER_PATH_MAX_ATTEMPTS
                   and lever_angle < LEVER_PATH_SAFETY_MAX_RAD):
                fine_pull = lever_angle >= LEVER_PATH_FINE_START_RAD
                step_rad = (
                    LEVER_PATH_FINE_STEP_RAD
                    if fine_pull else LEVER_PATH_STEP_RAD)
                target_angle = min(
                    lever_angle + step_rad,
                    LEVER_PATH_SAFETY_MAX_RAD)
                if not fine_pull:
                    target_angle = min(
                        target_angle, LEVER_PATH_FINE_START_RAD)
                anchor_delta_angle = target_angle - hook_anchor_angle
                path_fraction = (
                    anchor_delta_angle
                    / (LEVER_NOMINAL_HANDOFF_RAD - hook_anchor_angle))
                canonical_blend = _smootherstep(path_fraction)
                rotated_anchor_offset = _rotate_vector_about_world_y(
                    hook_anchor_offset, anchor_delta_angle)
                nominal_target = _lever_command_target_world(
                    _lever_hand_target_world(target_angle))
                anchored_target = tuple(
                    nominal + (1.0 - canonical_blend) * offset
                    for nominal, offset in zip(
                        nominal_target, rotated_anchor_offset))
                anchored_rotation_world = scene_ik.rotate_about_world_y(
                    hook_anchor_rotation_world, anchor_delta_angle)
                canonical_rotation_world = scene_ik.rotate_about_world_y(
                    contact_rotation_world, target_angle)
                target_rotation_world = scene_ik.interpolate_rotation_world(
                    anchored_rotation_world,
                    canonical_rotation_world,
                    canonical_blend)
                live_arm_rad = list(robot_state.arm_joint_state().position)
                label = "v2_lever_closed_loop_{:02d}".format(attempts)
                with timing.segment("{}.ik_solve".format(label)):
                    path_target = scene_ik.solve_lever(
                        anchored_target,
                        live_arm_rad[7:14],
                        target_rotation_world=target_rotation_world,
                        joint_limit_margin_fraction=(
                            LEVER_PATH_JOINT_LIMIT_MARGIN_FRACTION))
                # The current URDF tracks the requested r5 closely.  Use the
                # live step's IK value from the first pull step onward instead
                # of carrying over a fixed compensation from an older model.
                r5_command = path_target[4]
                move_right_joint_target(
                    label, path_target, LEVER_PATH_TRAJECTORY_POINTS,
                    smooth=True, r5_command_rad=r5_command)
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
            print("V2 source bin conveyor motion complete")

        # Raise the shoulder first to clear the chassis, move to a waypoint
        # directly above the task-only staging posture, carry that safe height
        # back to B0, and only then descend.  Do not route through the old
        # distant high ready posture: staging remains the intended endpoint.
        # V2 still holds the measured left arm at every trajectory point.
        shoulder_only_deg = list(current_target_deg)
        shoulder_only_deg[8] = -SHOULDER_LIFT_DEG
        shoulder_only_deg[12] = math.degrees(-0.5)
        with timing.segment(
                "initialization.shoulder_clearance",
                trajectory_points=TRAJECTORY_POINTS):
            trajectory.execute_trajectory(
                _right_only_trajectory(
                    shoulder_only_deg, current_target_deg, left_hold_rad,
                    num=TRAJECTORY_POINTS),
                sleep_time=0.02)
        current_target_deg = shoulder_only_deg

        staging_above_deg = list(current_target_deg)
        staging_above_deg[7:14] = [
            math.degrees(value) for value in RIGHT_ARM_STAGING_ABOVE_RAD]
        with timing.segment(
                "initialization.staging_above",
                trajectory_points=TRAJECTORY_POINTS):
            trajectory.execute_trajectory(
                _right_only_trajectory(
                    staging_above_deg, current_target_deg, left_hold_rad,
                    num=TRAJECTORY_POINTS),
                sleep_time=0.02)
        current_target_deg = staging_above_deg

        latch_achieved_right_target("v2_staging_above_at_safe_base")

        # Return once to this run's saved random B0 before computing any task
        # IK.  Keep the arm at the high nearby waypoint throughout the base
        # translation so the hand cannot sweep through the source bin.
        with timing.segment("initialization.base_return_b0"):
            measured_b0 = _move_base_with_wheels(
                chassis, base_translation_world)
        scene_ik.set_base_translation_world(
            (measured_b0[0], measured_b0[1], base_translation_world[2]))

        # Only descend after the base has reached B0.  This final short motion
        # establishes the low staging point used by the first source IK.
        staging_deg = list(current_target_deg)
        staging_deg[7:14] = [
            math.degrees(value) for value in RIGHT_ARM_STAGING_RAD]
        with timing.segment(
                "initialization.staging_descent",
                trajectory_points=STAGING_DESCENT_TRAJECTORY_POINTS):
            trajectory.execute_trajectory(
                _right_only_trajectory(
                    staging_deg, current_target_deg, left_hold_rad,
                    num=STAGING_DESCENT_TRAJECTORY_POINTS),
                sleep_time=0.02)
        current_target_deg = staging_deg

        with timing.segment("initialization.staging_settle_wait"):
            time.sleep(0.2)
        latch_achieved_right_target("v2_staging_after_return_to_b0")
        scorer_started = start_score_clock()
        timing.mark_score_start(scorer_started)
        task_tcp_quaternion_world = scene_ik.eef_quaternion_world_with_yaw(
            RIGHT_ARM_READY_FULL_RAD, GRASP_YAW_ADJUSTMENT_RAD)

        # Form the open source-grasp posture at the safe staging pose.  Start
        # the first source IK immediately afterward so the thumb cannot sweep
        # through the cylinder while the arm is already at the grasp point.
        with timing.segment("source_grasp.hand_prepare"):
            gripper.control_right_gripper(0)

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
                compensate_source_grasp=True,
                trajectory_points=SOURCE_GRASP_TRAJECTORY_POINTS,
            )
            print_measured_grasp_center("{}_open".format(name))
            with timing.segment("{}.pre_close_wait".format(name)):
                time.sleep(SOURCE_GRASP_SETTLE_SECONDS)
            with timing.segment("{}.gripper_close".format(name)):
                gripper.control_right_gripper(GRASP_CLOSURE_CMD)
            with timing.segment("{}.post_close_wait".format(name)):
                time.sleep(0.5)
            print_measured_grasp_geometry(
                "{}_closed".format(name), poses.get_position(name))
            move_right_by_fixed_delta(
                "{}_fixed_grasp_lift".format(name),
                tuple(
                    GRASP_LIFT_SCALE * delta
                    for delta in FIXED_GRASP_LIFTS_RIGHT_RAD[index]),
            )
            print_measured_grasp_center("{}_lifted".format(name))
            print("{}_after_lift object: {}".format(
                name,
                [round(value, 6) for value in poses.get_position(name)]))
            move_right_tcp(
                "{}_above_target_bin".format(name),
                drop_center_world,
                task_tcp_quaternion_world,
                point_in_r7=scene_ik.CLOSED_GRASP_CENTER_IN_R7,
                trajectory_points=IK_TRAJECTORY_POINTS,
            )
            with timing.segment("{}.pre_release_wait".format(name)):
                time.sleep(1.0)
            print("{}_above_target object: {}".format(
                name,
                [round(value, 6) for value in poses.get_position(name)]))
            with timing.segment("{}.gripper_release".format(name)):
                gripper.control_right_gripper(0)
            with timing.segment("{}.object_settle_wait".format(name)):
                time.sleep(1.0)

            final_position = poses.get_position(name)
            print("{}_settled object: {}".format(
                name, [round(value, 6) for value in final_position]))
            if not _fully_in_target(final_position):
                message = "{} is not fully inside the V2 target bin: {}".format(
                    name, final_position)
                if experiment_mode:
                    print("[V2 EXPERIMENT] {} ; continuing".format(message))
                else:
                    raise RuntimeError(message)

            if index + 1 < len(CYLINDERS):
                return_right_to_staging("v2_between_objects_staging")

        right_task_success = all(
            _fully_in_target(poses.get_position(name)) for name in CYLINDERS)
        print("v2_right_task_final objects: {}".format({
            name: [round(value, 6) for value in poses.get_position(name)]
            for name in CYLINDERS
        }))
        latch_achieved_right_target("v2_right_task_complete")
        with timing.segment("lever.total"):
            run_right_lever_stage()
        success = right_task_success
    finally:
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
