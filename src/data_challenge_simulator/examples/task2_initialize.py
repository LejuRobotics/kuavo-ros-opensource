#!/usr/bin/env python3
"""Execute only Task 2 safe initialization; no approach or grasp."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

import rospy


CURRENT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = CURRENT_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
from utils.trajectory_controller import TrajectoryController
from utils.arm_state import wait_for_first_arm_state
from utils.task2_initialization import Task2InitializationPlanner
from utils.task2_pick_planner import Task2PickPlanner
from utils.utils import Utils

from task2_base_motion import ChassisMotion


DEFAULT_CONFIG = PACKAGE_DIR / "config/task2_initialization.json"
DEFAULT_SCENE = (
    PACKAGE_DIR / "models/biped_s400062/xml/task2.xml")
ARM_JOINT_COUNT = 14
TRAJECTORY_SLEEP = 0.02
DEFAULT_STOP_SPEED_THRESHOLD = 0.01
DEFAULT_SETTLE_TIMEOUT = 10.0
# Settled-residual band for the one decision in ``run_initialization``: inside
# it the initialized pose is accepted, outside it one open-loop correction is
# sent and then accepted too.  ``/ground_truth/state`` anchors its sensors on
# the torso ``imu`` site, not on ``base_link``, so this residual and any
# nearer/farther reading derived from it carry an unverified constant offset.
INITIALIZATION_SETTLED_DISTANCE_M = 0.025


def wait_for_arm_state(robot_state, timeout=30.0):
    del robot_state
    return wait_for_first_arm_state(timeout=timeout)


def make_chassis(args):
    """Build the chassis for the Task 2 initialization path.

    The initialization path is open loop and does not consult
    ``position_tolerance`` or ``yaw_tolerance_deg``; they are passed only
    because ``ChassisMotion`` requires them, and they keep the values Task 2
    used before so the shared constructor stays comparable.
    """
    return ChassisMotion(
        linear_speed=args.linear_speed,
        angular_speed=0.20,
        minimum_linear_speed=min(args.minimum_linear_speed, args.linear_speed),
        minimum_angular_speed=0.06,
        position_tolerance=0.03,
        yaw_tolerance_deg=3.0,
    )


def describe_residual(measured_pose, target_pose):
    """Report the world-frame residual vector without judging it.

    Deviation is never a failure here -- it is printed and the caller moves on.
    """
    error_x = target_pose[0] - measured_pose[0]
    error_y = target_pose[1] - measured_pose[1]
    return (math.hypot(error_x, error_y),
            "residual vector=(%+.4f, %+.4f) m, i.e. the robot sits "
            "%+.4f m in x and %+.4f m in y of the target"
            % (error_x, error_y, -error_x, -error_y))


def execute_lift(trajectory, robot_state, target_rad, points):
    measured = wait_for_arm_state(robot_state)
    rospy.loginfo(
        "Raising both arms by changing only the two shoulder-pitch joints in "
        "one %d-point "
        "trajectory", points)
    trajectory.execute_trajectory(
        Utils.interpolate_joint_trajectory(
            [math.degrees(value) for value in target_rad],
            [math.degrees(value) for value in measured],
            num=points,
        ),
        sleep_time=TRAJECTORY_SLEEP,
    )


def execute_ready(trajectory, robot_state, target_rad, points):
    measured = wait_for_arm_state(robot_state)
    rospy.loginfo(
        "Turning both palms toward the table at unchanged hand height in "
        "one %d-point trajectory", points)
    trajectory.execute_trajectory(
        Utils.interpolate_joint_trajectory(
            [math.degrees(value) for value in target_rad],
            [math.degrees(value) for value in measured],
            num=points,
        ),
        sleep_time=TRAJECTORY_SLEEP,
    )


def shoulder_only_target(measured_arm, shoulder_pitch_rad):
    target = [float(value) for value in measured_arm]
    if len(target) != ARM_JOINT_COUNT:
        raise ValueError("measured_arm must contain 14 joints")
    target[0] = float(shoulder_pitch_rad)
    target[7] = float(shoulder_pitch_rad)
    return target


def run_initialization(
        robot, robot_state, chassis, config, scene,
        ready_planner, ready_outward_angle_deg,
        stop_speed_threshold=DEFAULT_STOP_SPEED_THRESHOLD,
        settle_timeout=DEFAULT_SETTLE_TIMEOUT, dry_run=False,
        trajectory=None, retain_owned_trajectory=False,
        on_trajectory_started=None):
    """Run the one-time Task 2 arm initialization in an existing ROS node.

    The chassis legs are open loop: retreat, return, settle, one decision.
    Any deviation is reported through the log and the run still completes --
    this path has no failure branch and never raises for a chassis miss.
    ``stop_speed_threshold`` and ``settle_timeout`` belong to the wait for the
    chassis to actually stop, not to a target-reached test.
    """
    safe_retreat_m = float(config["safe_retreat_m"])
    shoulder_pitch_rad = float(config["shoulder_pitch_rad"])
    lift_points = int(config["lift_trajectory_points"])
    ready_points = int(config["ready_trajectory_points"])
    if safe_retreat_m <= 0.0 or lift_points < 2 or ready_points < 2:
        raise RuntimeError("invalid safe retreat or trajectory length")

    chassis.wait_until_ready(timeout=30.0)
    wait_for_arm_state(robot_state)
    initial_base_pose = tuple(chassis.pose())
    safe_base_pose = (
        initial_base_pose[0] -
        safe_retreat_m * math.cos(initial_base_pose[2]),
        initial_base_pose[1] -
        safe_retreat_m * math.sin(initial_base_pose[2]),
        initial_base_pose[2],
    )
    rospy.loginfo(
        "Task2 initialization plan: initial=(%.3f, %.3f, %.1f deg) "
        "retreat=%.3f m safe=(%.3f, %.3f, %.1f deg), shoulder-only "
        "lift target=%.3f rad",
        initial_base_pose[0], initial_base_pose[1],
        math.degrees(initial_base_pose[2]), safe_retreat_m,
        safe_base_pose[0], safe_base_pose[1],
        math.degrees(safe_base_pose[2]), shoulder_pitch_rad)
    if dry_run:
        return initial_base_pose, tuple(wait_for_arm_state(robot_state))

    # Step 1: retreat.  Open loop, no arrival test, no tolerance.
    retreat_plan = chassis.move_open_loop(safe_base_pose[0], safe_base_pose[1])
    chassis.wait_until_stopped(stop_speed_threshold, settle_timeout)
    retreat_measured = tuple(chassis.pose())
    retreat_residual, retreat_detail = describe_residual(
        retreat_measured, retreat_plan["target"])
    rospy.loginfo(
        "Task2 retreat: target=(%.3f, %.3f) measured=(%.3f, %.3f, %.1f deg) "
        "residual=%.4f m; %s",
        retreat_plan["target"][0], retreat_plan["target"][1],
        retreat_measured[0], retreat_measured[1],
        math.degrees(retreat_measured[2]), retreat_residual, retreat_detail)

    measured_arm = wait_for_arm_state(robot_state)
    collision_planner = Task2InitializationPlanner(
        scene,
        collision_margin_m=float(config["collision_margin_m"]),
        report_limit_m=float(config["collision_report_limit_m"]))
    target_lift = shoulder_only_target(
        measured_arm, shoulder_pitch_rad)
    lift_clearance = collision_planner.path_clearance(
        safe_base_pose, measured_arm, [target_lift],
        samples_per_segment=int(config["path_samples"]))
    collision_margin = float(config["collision_margin_m"])
    if lift_clearance < collision_margin:
        raise RuntimeError(
            "shoulder-only arm lift is not collision-clear: lift={:.4f} m, "
            "required={:.4f} m".format(
                lift_clearance, collision_margin))

    # The first arm publisher is deliberately created only after the chassis
    # has completed its safe retreat.  A merged workflow can retain this same
    # instance for every later stage.
    owns_trajectory = trajectory is None
    if owns_trajectory:
        trajectory = TrajectoryController(
            robot, initial_positions=measured_arm)
        if on_trajectory_started is not None:
            on_trajectory_started(trajectory)
    else:
        trajectory.set_target_positions(list(measured_arm))
    try:
        execute_lift(trajectory, robot_state, target_lift, lift_points)
        measured_lift = wait_for_arm_state(robot_state)
        target_ready = ready_planner.solve_palm_down_ready(
            safe_base_pose, measured_lift, ready_outward_angle_deg)
        ready_clearance = collision_planner.path_clearance(
            safe_base_pose, measured_lift, [target_ready],
            samples_per_segment=int(config["path_samples"]))
        return_clearance = collision_planner.configuration_clearance(
            initial_base_pose, target_ready)
        if (ready_clearance < collision_margin or
                return_clearance < collision_margin):
            raise RuntimeError(
                "palm-down ready is not collision-clear: ready={:.4f} m, "
                "return={:.4f} m, required={:.4f} m".format(
                    ready_clearance, return_clearance, collision_margin))
        execute_ready(
            trajectory, robot_state, target_ready, ready_points)
        wait_for_arm_state(robot_state)
    finally:
        if owns_trajectory and not retain_owned_trajectory:
            trajectory.stop()

    # Step 3: return.  Planned from the measured pose read here, not from the
    # retreat command's prediction.  Open loop, cut when the time is up.
    return_plan = chassis.move_open_loop(
        initial_base_pose[0], initial_base_pose[1])
    # Step 4: wait for the measured twist to actually reach zero.
    chassis.wait_until_stopped(stop_speed_threshold, settle_timeout)
    returned_pose = tuple(chassis.pose())
    return_residual, return_detail = describe_residual(
        returned_pose, return_plan["target"])
    rospy.loginfo(
        "Task2 return: cut-off pose=(%.3f, %.3f, %.1f deg), settled "
        "measured=(%.3f, %.3f, %.1f deg), target=(%.3f, %.3f), "
        "residual=%.4f m; %s",
        return_plan["cut_pose"][0], return_plan["cut_pose"][1],
        math.degrees(return_plan["cut_pose"][2]),
        returned_pose[0], returned_pose[1],
        math.degrees(returned_pose[2]),
        return_plan["target"][0], return_plan["target"][1],
        return_residual, return_detail)

    # Step 5: the single decision.  This asks whether initialization is over,
    # not whether the chassis landed exactly on target.
    adjustments = 0
    if return_residual > INITIALIZATION_SETTLED_DISTANCE_M:
        adjustments = 1
        rospy.loginfo(
            "Task2 initialization decision: residual %.4f m is beyond the "
            "%.3f m settled band; running the single correction pass",
            return_residual, INITIALIZATION_SETTLED_DISTANCE_M)
    else:
        rospy.loginfo(
            "Task2 initialization decision: residual %.4f m is within the "
            "%.3f m settled band; no correction pass",
            return_residual, INITIALIZATION_SETTLED_DISTANCE_M)
    if adjustments:
        # Step 6: one correction from the freshly measured pose, then stop and
        # report.  Nothing further is judged.
        adjustment_plan = chassis.move_open_loop(
            initial_base_pose[0], initial_base_pose[1])
        chassis.wait_until_stopped(stop_speed_threshold, settle_timeout)
        returned_pose = tuple(chassis.pose())
        adjusted_residual, adjusted_detail = describe_residual(
            returned_pose, adjustment_plan["target"])
        rospy.loginfo(
            "Task2 adjustment: cut-off pose=(%.3f, %.3f, %.1f deg), settled "
            "measured=(%.3f, %.3f, %.1f deg), target=(%.3f, %.3f), "
            "residual=%.4f m; %s",
            adjustment_plan["cut_pose"][0], adjustment_plan["cut_pose"][1],
            math.degrees(adjustment_plan["cut_pose"][2]),
            returned_pose[0], returned_pose[1],
            math.degrees(returned_pose[2]),
            adjustment_plan["target"][0], adjustment_plan["target"][1],
            adjusted_residual, adjusted_detail)
        return_residual = adjusted_residual
    rospy.loginfo(
        "Task2 initialization final deviation: %.4f m after %d adjustment "
        "pass(es); initialization is complete regardless of this number",
        return_residual, adjustments)

    measured_seed = tuple(wait_for_arm_state(robot_state))

    rospy.set_param("/task2_random_initial_base_pose", initial_base_pose)
    rospy.set_param("/task2_returned_base_pose", returned_pose)
    rospy.set_param("/task2_initialized_arm_seed", measured_seed)
    rospy.loginfo(
        "Task2 initialization complete: palms face the table at the raised "
        "hand height; "
        "return command finished at measured base="
        "(%.3f, %.3f, %.1f deg); stored measured 14-joint IK seed",
        returned_pose[0], returned_pose[1],
        math.degrees(returned_pose[2]))
    return returned_pose, measured_seed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--linear-speed", type=float, default=0.08)
    parser.add_argument("--minimum-linear-speed", type=float, default=0.06)
    parser.add_argument(
        "--stop-speed-threshold", type=float,
        default=DEFAULT_STOP_SPEED_THRESHOLD)
    parser.add_argument(
        "--settle-timeout", type=float, default=DEFAULT_SETTLE_TIMEOUT)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.linear_speed <= 0.0 or args.minimum_linear_speed <= 0.0:
        parser.error("speeds must be positive")
    if args.stop_speed_threshold <= 0.0:
        parser.error("stop speed threshold must be positive")

    with args.config.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    if config.get("schema_version") != 1:
        raise RuntimeError("unsupported Task2 initialization config schema")
    rospy.init_node("task2_initialize", anonymous=False)
    # Task 2 uses its own MuJoCo/SciPy planners, not the SDK IK services.
    if not KuavoSDK().Init(options=KuavoSDK.Options.Normal):
        raise RuntimeError("KuavoSDK initialization failed")

    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    chassis = make_chassis(args)
    with (PACKAGE_DIR / "config/task2_pick.json").open(
            "r", encoding="utf-8") as stream:
        pick_config = json.load(stream)
    ready_planner = Task2PickPlanner(
        args.scene, ik_config=pick_config["ik"])
    run_initialization(
        robot, robot_state, chassis, config, args.scene,
        ready_planner,
        pick_config["grasp"]["outward_angle_deg"],
        stop_speed_threshold=args.stop_speed_threshold,
        settle_timeout=args.settle_timeout, dry_run=args.dry_run)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
