#!/usr/bin/env python3
"""Run the complete Scene 2 two-box workflow in one persistent ROS process."""

import json
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
from utils.task2_pick_planner import Task2PickPlanner
from utils.task2_randomization import Task2RandomizationPlanner

from task2_base_motion import ChassisMotion
from task2_initialize import (
    DEFAULT_CONFIG as INITIALIZATION_CONFIG,
    DEFAULT_SCENE,
    run_initialization,
)
from task2_pick import (
    BimanualGraspCommand,
    BimanualGraspState,
    ConveyorCommand,
    RepositionGraspCommand,
    RepositionGraspState,
    DEFAULT_CONFIG as PICK_CONFIG,
    main as run_pick,
)


RESULT_PATH = Path("task_result.txt")
HEAD_SCAN_PITCH_RAD = math.radians(20.0)


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    if config.get("schema_version") != 1:
        raise RuntimeError("unsupported Task2 config schema: {}".format(path))
    return config


def main():
    success = False
    RESULT_PATH.write_text("fail", encoding="utf-8")
    os.environ.setdefault("KUAVO_LOG_SERVER", "0")
    os.environ.setdefault("KUAVO_LEG_SERVICE", "0")

    initialization_config = load_json(INITIALIZATION_CONFIG)
    pick_config = load_json(PICK_CONFIG)
    rospy.init_node("task2", anonymous=False)
    # All Task 2 IK is local MuJoCo/SciPy optimization.  The SDK IK service is
    # neither called nor a valid reason to repeat communication setup.
    if not KuavoSDK().Init(options=KuavoSDK.Options.Normal):
        raise RuntimeError("KuavoSDK initialization failed")

    robot = KuavoRobot()
    robot_state = KuavoRobotState()

    # Scene 2 places both boxes from the saved layout before any arm motion,
    # the same position Task 1 V2 and Task 3 randomize their objects.  The
    # boxes are free joints, so the service is the right write; the base
    # translation still comes from the launch arguments, which the controller
    # needs before it builds its origin reference.
    seed = int(os.environ.get("TASK_SEED", "1"))
    layout = Task2RandomizationPlanner().plan(seed)
    randomizer = ObjectRandomizer(timeout=60.0)
    for box in layout.boxes:
        result = randomizer.set_object_position(
            box.name,
            position={"x": box.position[0], "y": box.position[1],
                      "z": box.position[2]},
            orientation={"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
        if not result["success"]:
            raise RuntimeError(
                "failed to place {}: {}".format(box.name, result["message"]))
    rospy.loginfo(
        "Task2 seed %d selected saved layout %d: initial_base=%s boxes=%s",
        seed, layout.catalogue_seed,
        [round(value, 6) for value in layout.initial_base],
        [[box.name, [round(value, 6) for value in box.position]]
         for box in layout.boxes])
    rospy.sleep(1.0)

    if not robot.control_head(yaw=0.0, pitch=HEAD_SCAN_PITCH_RAD):
        raise RuntimeError("Failed to command the 20-degree head scan pose")

    initialization_chassis = ChassisMotion(
        linear_speed=0.08,
        angular_speed=0.20,
        minimum_linear_speed=0.06,
        minimum_angular_speed=0.06,
        position_tolerance=0.03,
        yaw_tolerance_deg=3.0,
    )
    pick_chassis = ChassisMotion(
        linear_speed=0.08,
        angular_speed=0.20,
        minimum_linear_speed=0.06,
        minimum_angular_speed=0.06,
        position_tolerance=0.03,
        yaw_tolerance_deg=3.0,
    )
    gripper = GripperController()
    runtime = SimpleNamespace(
        robot=robot,
        robot_state=robot_state,
        poses=ObjectPose(),
        grasp_state=BimanualGraspState(),
        grasp_command=BimanualGraspCommand(),
        reposition_state=RepositionGraspState(),
        reposition_command=RepositionGraspCommand(),
        conveyor_command=ConveyorCommand(),
        chassis=pick_chassis,
        planner=Task2PickPlanner(ik_config=pick_config["ik"]),
        gripper=gripper,
        trajectory=None,
    )
    # Clear a latch left armed by an interrupted earlier Task 2 run once, at
    # workflow startup.  Do not reset it by recreating the controller per box.
    runtime.grasp_command.set_enabled(False)
    runtime.reposition_command.set_enabled(False)

    try:
        rospy.loginfo("TASK2 WORKFLOW START: one-time initialization")
        run_initialization(
            robot, robot_state, initialization_chassis,
            initialization_config, DEFAULT_SCENE,
            runtime.planner,
            pick_config["grasp"]["outward_angle_deg"],
            trajectory=None,
            retain_owned_trajectory=True,
            on_trajectory_started=lambda value: setattr(
                runtime, "trajectory", value))
        rospy.loginfo("TASK2 WORKFLOW COMPLETE: one-time initialization")
        start_score_clock()

        for index in range(2):
            rospy.loginfo(
                "TASK2 WORKFLOW START: pick_transport_release_%d/2",
                index + 1)
            run_pick(
                runtime=runtime, transport_and_return=True,
                reposition_before_right=(index == 0))
            rospy.loginfo(
                "TASK2 WORKFLOW COMPLETE: pick_transport_release_%d/2",
                index + 1)

        placed_count = int(rospy.get_param("/task2_placed_box_count", 0))
        if placed_count != 2:
            raise RuntimeError(
                "Task2 finished both transfers but placed count is {}".format(
                    placed_count))
        success = True
        rospy.loginfo("TASK2 FULL WORKFLOW PASSED: placed_box_count=2")
    finally:
        runtime.grasp_command.set_enabled(False)
        runtime.reposition_command.set_enabled(False)
        initialization_chassis.stop()
        pick_chassis.stop()
        runtime.gripper.stop()
        if runtime.trajectory is not None:
            runtime.trajectory.stop()
        RESULT_PATH.write_text(
            "success" if success else "fail", encoding="utf-8")


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    except Exception as error:
        print("Task 2 failed: {}".format(error), flush=True)
        import traceback
        traceback.print_exc()
        sys.exit(1)
