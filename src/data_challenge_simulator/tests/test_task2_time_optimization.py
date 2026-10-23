"""Static contracts for the explicitly approved Task 2 speed changes."""

import json
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
INITIALIZATION_CONFIG = PACKAGE / "config/task2_initialization.json"
PICK_CONFIG = PACKAGE / "config/task2_pick.json"
TASK = PACKAGE / "examples/task2.py"
PICK = PACKAGE / "examples/task2_pick.py"
MODEL_ENTRY = PACKAGE / "scripts/model_entry.py"


def load(path):
    return json.loads(path.read_text())


def test_task2_initialization_finishes_near_the_above_ik_height_quickly():
    config = load(INITIALIZATION_CONFIG)
    assert config["shoulder_pitch_rad"] == -1.0
    assert config["lift_trajectory_points"] == 80
    assert config["ready_trajectory_points"] == 60
    assert config["open_loop_linear_speed_mps"] == 0.16


def test_task2_uses_reduced_ik_budget_and_relaxed_tolerance():
    ik = load(PICK_CONFIG)["ik"]
    assert ik["maximum_function_evaluations"] == 120
    assert ik["maximum_position_error_m"] == 0.03
    assert ik["solver_tolerance"] == 1e-5


def test_task2_entries_read_the_transport_chassis_config():
    config = load(PICK_CONFIG)["chassis"]
    assert config == {
        "linear_speed_mps": 0.60,
        "angular_speed_radps": 1.20,
        "minimum_linear_speed_mps": 0.08,
        "minimum_angular_speed_radps": 0.08,
    }
    task_source = TASK.read_text()
    assert 'pick_config["chassis"]["linear_speed_mps"]' in task_source
    assert 'pick_config["chassis"]["angular_speed_radps"]' in task_source
    assert 'initialization_config["open_loop_linear_speed_mps"]' in task_source
    model_source = MODEL_ENTRY.read_text()
    assert "linear_speed=None" in model_source
    assert "        config)" in model_source


def test_final_release_ends_at_conveyor_contact_without_return_route():
    source = PICK.read_text()
    release = source[source.index("if is_first_box:", source.index(
        '"Task2 placing %s on destination conveyor"')):
        source.index('rospy.loginfo("Task2 returning directly to the next box")')]
    assert release.count('"post_release_ready"') == 1
    assert release.index('"post_release_ready"') < release.index("else:")
    assert "final box contacted conveyor" in release
    contact_wait = source.index("wait_for_conveyor_contact(", source.index(
        '"Task2 placing %s on destination conveyor"'))
    final_branch = source.index("if not is_first_box:", contact_wait)
    assert contact_wait < final_branch
    assert release.index("if not is_first_box:") < release.index(
        "\n                return\n")
    return_route = source[source.index(
        'rospy.loginfo("Task2 returning directly to the next box")'):
        source.index('"TASK2 BOX PLACED AND RETURNED: box=%s"')]
    assert "translate_relative(" not in source
    assert "stored_b1[" not in return_route
    assert "next_docking = docking_bases[other_box]" in return_route
    assert return_route.index("move_to_pose(") < return_route.index("returning_arms.result()")
    assert return_route.index("returning_arms.result()") < return_route.index("conveyor_command.set_enabled(True)") < return_route.index("rotate_relative(")
    assert return_route.index("move_to_pose(") < return_route.index("rotate_relative(") < return_route.rindex("move_to_pose(")


def test_task2_arm_execution_uses_the_verified_trajectory_lengths():
    config = load(PICK_CONFIG)
    grasp = config["grasp"]
    assert grasp["above_trajectory_points"] == 20
    assert grasp["descent_trajectory_points"] == 35
    assert grasp["lift_trajectory_points"] == 60
    assert grasp["gripper_duration_s"] == 0.35
    assert "settle_time_s" not in grasp
    assert "hold_time_s" not in grasp
    transport = config["transport"]
    assert transport["release_ready_trajectory_points"] == 30
    assert transport["place_contact_max_z_m"] == 0.605
    assert transport["place_contact_timeout_s"] == 2.0


def test_direct_translation_preserves_direction_when_speed_is_limited():
    import importlib.util
    import math
    from types import SimpleNamespace
    import pytest

    spec = importlib.util.spec_from_file_location(
        "task2_direct_motion", PACKAGE / "examples/task2_base_motion.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.rospy = SimpleNamespace(
        is_shutdown=lambda: False, Rate=lambda _: None)
    motion = module.ChassisMotion.__new__(module.ChassisMotion)
    motion.linear_speed = 0.20
    motion.linear_gain = 1.0
    motion.minimum_linear_speed = 0.08
    motion.angular_speed = 0.25
    motion.minimum_angular_speed = 0.08
    motion.position_tolerance = 0.03
    motion.yaw_tolerance = math.radians(3)
    motion.publish_rate = 100

    class CommandCaptured(Exception):
        pass

    commands = []
    def capture(*command):
        commands.append(command)
        raise CommandCaptured
    motion.publish = capture
    for yaw in (0, math.pi / 2, -math.pi / 2):
        motion.pose = lambda: (0, 0, yaw)
        for x, y in ((3, 4), (0.1, 1), (0.02, 0.04)):
            with pytest.raises(CommandCaptured):
                motion.move_to_pose(x, y, yaw, 1, straight_line=True)
            vx, vy, wz = commands[-1]
            world_vx = math.cos(yaw) * vx - math.sin(yaw) * vy
            world_vy = math.sin(yaw) * vx + math.cos(yaw) * vy
            assert abs(world_vx * y - world_vy * x) < 1e-12
            assert world_vx * x + world_vy * y > 0
            assert math.hypot(vx, vy) <= motion.linear_speed + 1e-12
            assert wz == 0
    source = PICK.read_text()
    assert source.count("straight_line=True") == 4
    assert 'current_base[0] + target_box_x - box_position[0]' in source
    assert 'saved_layout.conveyor_bounds[:2]' in source
    assert "destination_base_pose" not in source
