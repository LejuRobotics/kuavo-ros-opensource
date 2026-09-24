#!/usr/bin/env python3
"""Verify or replay a recorded Scene 1 demonstration bag."""

import argparse
import os
import re
import subprocess
import sys
import time

import rosbag
import rospy

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(CURRENT_DIR)
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

from helperfunc import ensure_clean_simulator_graph, simulator_launch_command, wait_for_topics
from task1_v2 import (
    CYLINDERS,
    _fully_in_target,
    _randomize_scene,
)
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.shutdown_guard import ShutdownGuard
from utils.task_scorer import Task1Scorer, relative_local_x


COMMAND_TOPICS = (
    "/kuavo_arm_traj",
    "/sg100_hand_command",
)
RESULT_TOPICS = tuple(
    "/mujoco/{}/pose".format(name) for name in CYLINDERS
) + ("/mujoco/lever/pose", "/mujoco/source_bin/pose")
REQUIRED_TOPICS = COMMAND_TOPICS + RESULT_TOPICS


def _pose_values(message):
    pose = message.pose
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w),
    )


def inspect_bag(bag_path):
    """Return counts and final task-object poses from one bag."""
    counts = {topic: 0 for topic in REQUIRED_TOPICS}
    final_poses = {}
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, message, _ in bag.read_messages(topics=REQUIRED_TOPICS):
            counts[topic] += 1
            if topic in RESULT_TOPICS:
                final_poses[topic] = _pose_values(message)
        duration = max(0.0, bag.get_end_time() - bag.get_start_time())
    return counts, final_poses, duration


def evaluate_final_poses(final_poses):
    failures = []
    for name in CYLINDERS:
        topic = "/mujoco/{}/pose".format(name)
        values = final_poses.get(topic)
        if values is None:
            failures.append("missing final pose for {}".format(name))
        elif not _fully_in_target(values[0]):
            failures.append("{} final position is outside target: {}".format(
                name, [round(value, 6) for value in values[0]]))

    lever_values = final_poses.get("/mujoco/lever/pose")
    source_bin_values = final_poses.get("/mujoco/source_bin/pose")
    source_bin_slide = None
    if lever_values is None:
        failures.append("missing final lever pose")
    if source_bin_values is None:
        failures.append("missing final source_bin pose")
    if lever_values is not None and source_bin_values is not None:
        source_bin_slide = relative_local_x(
            source_bin_values[0], lever_values[0], lever_values[1])
        if (source_bin_slide is None
                or source_bin_slide < Task1Scorer.CONVEYOR_STARTED_MIN_M):
            failures.append(
                "source_bin slide {:.4f} m has not started".format(
                    float("nan") if source_bin_slide is None
                    else source_bin_slide))
    return failures, source_bin_slide


def verify_bag(bag_path):
    counts, final_poses, duration = inspect_bag(bag_path)
    failures = [
        "missing topic {}".format(topic)
        for topic, count in counts.items() if count == 0
    ]
    pose_failures, source_bin_slide = evaluate_final_poses(final_poses)
    failures.extend(pose_failures)

    print("Bag: {}".format(os.path.abspath(bag_path)))
    print("Duration: {:.2f} s".format(duration))
    for topic in REQUIRED_TOPICS:
        print("  {:36s} {} messages".format(topic, counts[topic]))
    if source_bin_slide is not None:
        print("Final source_bin slide: {:.4f} m".format(source_bin_slide))
    if failures:
        for failure in failures:
            print("[FAIL] {}".format(failure))
        return False
    print("[OK] Scene 1 bag contains commands and successful final state")
    return True


def infer_seed(bag_path):
    match = re.search(r"data_round_(\d+)\.bag$", os.path.basename(bag_path))
    if match is None:
        raise ValueError(
            "cannot infer seed from bag name; pass --seed explicitly")
    return int(match.group(1))


def replay_bag(bag_path, seed, headless):
    if not verify_bag(bag_path):
        raise RuntimeError("refusing to replay a bag that failed verification")

    ensure_clean_simulator_graph(1)
    launch_env = os.environ.copy()
    if headless:
        launch_env["MUJOCO_HEADLESS"] = "1"

    with ShutdownGuard(1) as guard:
        return _replay_bag(bag_path, seed, launch_env, guard)


def _replay_bag(bag_path, seed, launch_env, guard):
    guard.add(subprocess.Popen(
        simulator_launch_command(1, seed),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
        env=launch_env,
    ), process_group=True)
    try:
        time.sleep(2.0)
        try:
            rospy.init_node(
                "scene1_bag_replay", anonymous=True, disable_signals=True)
        except rospy.exceptions.ROSException:
            pass
        wait_for_topics(task_id=1, timeout=60)
        rospy.wait_for_service("/set_object_position", timeout=60)

        randomizer = ObjectRandomizer(timeout=60.0)
        _randomize_scene(randomizer, seed)
        poses = ObjectPose()
        for name in CYLINDERS + ("lever", "source_bin"):
            poses.wait_for_position(name, timeout=60.0)
        time.sleep(1.0)

        command = ["rosbag", "play", "--quiet", bag_path, "--topics"]
        command.extend(COMMAND_TOPICS)
        print("Replaying command topics with seed {}...".format(seed))
        play_process = guard.add(subprocess.Popen(
            command, start_new_session=True), process_group=True)
        returncode = play_process.wait()
        if returncode != 0:
            raise RuntimeError(
                "rosbag play exited with code {}".format(returncode))
        time.sleep(0.5)

        final_poses = {}
        for name in CYLINDERS + ("lever", "source_bin"):
            position = poses.get_position(name)
            orientation = poses.get_orientation(name)
            final_poses["/mujoco/{}/pose".format(name)] = (
                position, orientation)
        failures, source_bin_slide = evaluate_final_poses(final_poses)
        if source_bin_slide is not None:
            print("Replay final source_bin slide: {:.4f} m".format(
                source_bin_slide))
        if failures:
            for failure in failures:
                print("[FAIL] {}".format(failure))
            return False
        print("[OK] Scene 1 replay reached the recorded task acceptance")
        return True
    finally:
        # Ctrl+C while replay is still running unwinds through here with the
        # simulator (and possibly rosbag play) still up; the guard keeps a
        # second Ctrl+C from aborting this teardown half way.
        guard.sweep()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    verify_parser = subparsers.add_parser("verify")
    verify_parser.add_argument("bag")

    replay_parser = subparsers.add_parser("replay")
    replay_parser.add_argument("bag")
    replay_parser.add_argument("--seed", type=int)
    replay_parser.add_argument("--headless", action="store_true")

    args = parser.parse_args()
    bag_path = os.path.abspath(args.bag)
    if not os.path.isfile(bag_path):
        parser.error("bag does not exist: {}".format(bag_path))

    if args.command == "verify":
        return 0 if verify_bag(bag_path) else 1
    seed = args.seed if args.seed is not None else infer_seed(bag_path)
    return 0 if replay_bag(bag_path, seed, args.headless) else 1


if __name__ == "__main__":
    raise SystemExit(main())
