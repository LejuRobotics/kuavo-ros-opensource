#!/usr/bin/env python3
"""Run the accepted Task 3 stages in one persistent ROS process.

Every accepted stage keeps its own action body.  This entry point performs the
shared setup once and then calls the stages in order.  One
``TrajectoryController`` lives from the first stage to the last, so
``/kuavo_arm_traj`` keeps publishing across stage boundaries instead of being
torn down and rebuilt seven times.
"""

import argparse
import math
import os
from pathlib import Path
from types import SimpleNamespace
import sys

import rospy


CURRENT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = CURRENT_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoSDK
from utils.gripper_controller import GripperController
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.scorer_clock import start_score_clock
from utils.task3_randomization import Task3RandomizationPlanner

from task2_base_motion import ChassisMotion
from task3_descend import make_planner, main as run_descend
from task3_grasp import (
    Task3GraspCommand,
    Task3GraspFeedback,
    main as run_grasp,
)
from task3_initialize import main as run_initialize


RESULT_PATH = Path("task_result.txt")
DEFAULT_SCENE = PACKAGE_DIR / (
    "models/biped_s400062/xml/task3.xml")
ARM_JOINTS = 14
HEAD_SCAN_PITCH_RAD = math.radians(20.0)


def apply_plan(plan):
    """Apply the verified deterministic layout before any arm motion.

    The node is created once by ``main`` because a merged process cannot call
    ``rospy.init_node`` twice; ``task3.py`` got away with it only because the
    randomizer ran in its own process.
    """
    randomizer = ObjectRandomizer(timeout=30.0)
    placements = tuple(
        (ring.name, ring.position) for ring in plan.rings)
    for name, position in placements:
        result = randomizer.set_object_position(
            name,
            position={"x": position[0], "y": position[1], "z": position[2]},
            orientation={"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        )
        if not result["success"]:
            raise RuntimeError(
                "failed to place {}: {}".format(name, result["message"]))
    rospy.sleep(1.0)


def stage_plan(plan):
    """Return initialization followed by three accepted grasp sequences."""
    stages = [("initialize", None)]
    for index, ring in enumerate(plan.rings, start=1):
        stages.append((
            "direct_ik_{}/3".format(index),
            SimpleNamespace(
                kind="descend",
                ring_name=ring.name,
                docking_base=tuple(ring.docking_base),
            )))
        stages.append((
            "grasp_transfer_{}/3".format(index),
            SimpleNamespace(
                kind="grasp",
                ring_name=ring.name,
                destination_xy=tuple(ring.destination_xy),
            )))
    return tuple(stages)


def build_runtime(robot, robot_state, gripper, feedback, command):
    """Create the objects every accepted stage used to build for itself.

    ``robot`` and ``robot_state`` are passed in rather than built here because
    the shared publisher must be created from them before the scene work
    starts, exactly where the upstream entry point creates its publisher.
    """
    return SimpleNamespace(
        robot=robot,
        robot_state=robot_state,
        chassis=ChassisMotion(
            linear_speed=0.08,
            angular_speed=0.20,
            minimum_linear_speed=0.06,
            minimum_angular_speed=0.06,
            position_tolerance=0.03,
            yaw_tolerance_deg=3.0,
        ),
        poses=ObjectPose(),
        planner=None,
        gripper=gripper,
        trajectory=None,
        feedback=feedback,
        command=command,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Run the accepted complete Task 3 workflow in one process")
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    args = parser.parse_args()

    RESULT_PATH.write_text("fail", encoding="utf-8")
    os.environ.setdefault("KUAVO_LOG_SERVER", "0")
    os.environ.setdefault("KUAVO_LEG_SERVICE", "0")

    rospy.init_node("task3", anonymous=False)
    if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
        raise RuntimeError("KuavoSDK initialization failed")

    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    if not robot.control_head(yaw=0.0, pitch=HEAD_SCAN_PITCH_RAD):
        raise RuntimeError("Failed to command the 20-degree head scan pose")
    gripper = GripperController()

    seed = int(os.environ.get("TASK_SEED", "1"))
    plan = Task3RandomizationPlanner().plan(seed)
    print(
        "TASK3 seed {} saved layout {}: initial_base={} rings={}".format(
            seed,
            plan.catalogue_seed,
            tuple(round(value, 4) for value in plan.initial_base),
            [{
                "name": ring.name,
                "position": tuple(round(value, 4) for value in ring.position),
                "docking": tuple(round(value, 4) for value in ring.docking_base),
                "margin_rad": round(ring.joint_margin_rad, 3),
            } for ring in plan.rings]), flush=True)
    apply_plan(plan)

    scene_path = args.scene.resolve()
    runtime = build_runtime(
        robot, robot_state, gripper, Task3GraspFeedback(), Task3GraspCommand())
    success = False
    try:
        for label, stage in stage_plan(plan):
            print("[TASK3] Starting accepted stage: {}".format(label),
                  flush=True)
            if stage is None:
                # ChassisMotion subscribes asynchronously.  On later collect
                # rounds /ground_truth/state can lag behind the other topics
                # that helperfunc uses as its simulator-ready barrier.  Do not
                # let initialization snapshot a missing pose.
                runtime.chassis.wait_until_ready(timeout=30.0)
                run_initialize(runtime=runtime, retain_trajectory=True)
                start_score_clock()
            elif stage.kind == "descend":
                # The IK planner is bound to one ring, exactly as it was when
                # each stage ran in its own process.
                runtime.planner = make_planner(scene_path, stage.ring_name)
                run_descend(
                    runtime=runtime, ring_name=stage.ring_name,
                    docking_base=stage.docking_base)
            else:
                # Same one-shot construction Task 2 uses: the grasp command
                # and follower state live for the whole workflow and are never
                # rebuilt per object.
                runtime.planner = make_planner(scene_path, stage.ring_name)
                run_grasp(
                    runtime=runtime, ring_name=stage.ring_name,
                    destination_xy=stage.destination_xy)
            print("[TASK3] Completed accepted stage: {}".format(label),
                  flush=True)
        success = True
        print("TASK3 FULL WORKFLOW PASSED", flush=True)
    finally:
        runtime.chassis.stop()
        if runtime.trajectory is not None:
            runtime.trajectory.stop()
        gripper.stop()
        RESULT_PATH.write_text(
            "success" if success else "fail", encoding="utf-8")


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    except Exception as error:
        print("Task 3 failed: {}".format(error), flush=True)
        import traceback
        traceback.print_exc()
        sys.exit(1)
