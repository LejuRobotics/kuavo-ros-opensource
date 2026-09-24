"""Static assertions for the external-model entry's initialization stage."""

import ast
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
ENTRY = PACKAGE / "scripts/model_entry.py"
INIT = PACKAGE / "examples/task1_v2_initialize.py"
TASK3_MODEL_INIT = PACKAGE / "examples/task3_model_initialize.py"
MODEL_BASE = PACKAGE / "utils/model_base_motion.py"
ARM_STATE = PACKAGE / "utils/arm_state.py"
TASK1_MODEL_LAUNCH = PACKAGE / "launch/load_kuavo_mujoco_model1.launch"
TASK3_MODEL_LAUNCH = PACKAGE / "launch/load_kuavo_mujoco_model3.launch"
TASK1 = PACKAGE / "examples/task1.py"
TASK2 = PACKAGE / "examples/task2.py"
TASK2_INIT = PACKAGE / "examples/task2_initialize.py"
TASK3_SINGLE = PACKAGE / "examples/task3.py"
TASK3 = PACKAGE / "examples/task3_initialize.py"


def test_model_entry_initializes_before_publishing_ready():
    """The arm may only report ready after the scene work has been done.

    The call itself now lives in ``EvaluationProtocol._initialize``; what this
    still pins is that ``main`` drives it and that initialization precedes the
    latched True completion signal.  A latched False is published first to
    clear stale state from the previous round.
    """
    source = ENTRY.read_text()
    assert "protocol._initialize()" in source
    assert '"/model_simulator/ready"' in source
    initialize = source.index("def _initialize(self):")
    body = source[initialize:source.index("def publish_success(self, value):")]
    assert body.index("INITIALIZERS[self.task_id](") < body.index(
        "self.ready_publisher.publish(Bool(data=True))")


def test_task1_model_launch_publishes_required_hand_poses():
    source = TASK1_MODEL_LAUNCH.read_text()
    body_names = source[source.index('<rosparam param="task_body_names">'):
                        source.index("</rosparam>")]
    assert "l_hand_base" in body_names
    assert "r_hand_base" in body_names


def test_task3_model_launch_publishes_required_hand_poses():
    source = TASK3_MODEL_LAUNCH.read_text()
    body_names = source[source.index('<rosparam param="task_body_names">'):
                        source.index("</rosparam>")]
    assert "l_hand_base" in body_names
    assert "r_hand_base" in body_names


def test_model_entry_waits_for_controller_before_fixed_initialization():
    source = ENTRY.read_text()
    initialize = source.index("def _initialize(self):")
    body = source[initialize:source.index("def publish_success(self, value):")]
    assert body.index("self._wait_for_controller_ready()") < body.index(
        "INITIALIZERS[self.task_id](")


def test_external_commands_open_only_after_initialization_and_seed_docking():
    source = ENTRY.read_text()
    initialize = source.index("def _initialize(self):")
    body = source[initialize:source.index("def publish_success(self, value):")]
    fixed = body.index("INITIALIZERS[self.task_id](")
    close_gate = body.index("self._close_external_arm_input()")
    dock = body.index("base_initializer.prepare()")
    release = body.index("base_initializer.close()")
    complete = body.index("self.ready_publisher.publish(Bool(data=True))")
    handover = body.index("self._hand_over_arm_control()")
    accept = body.index(
        "self.command_accept_publisher.publish(Bool(data=bool(handed_over)))")
    failure_gate = body.index('if not handed_over:')
    assert (fixed < close_gate < dock < release < handover < accept
            < failure_gate < complete)
    assert '"/model_simulator/accept_commands"' in source


def test_model_entry_passes_seed_derived_initial_base_to_roslaunch():
    source = ENTRY.read_text()
    assert "initial_base_for(args.task_id, seed)" in source
    assert '"initial_base_x:={:.9f}".format(base_x)' in source
    assert '"initial_base_y:={:.9f}".format(base_y)' in source


def test_model_entry_waits_for_post_reset_mpc_observation():
    source = ENTRY.read_text()
    required = source[source.index("required = ("):
                      source.index("for topic in required:")]
    assert '"/mobile_manipulator_mpc_observation"' in required


def test_model_entry_reports_startup_failure_before_teardown():
    source = ENTRY.read_text()
    failure = source.index("except Exception as error:", source.index("def main():"))
    teardown = source.index("finally:", failure)
    assert failure < teardown
    assert '"[ERROR] model entry: task {} seed {} failed before "' in source
    assert "model entry: waiting for {}" in source


def test_model_entry_draws_its_own_seed():
    source = ENTRY.read_text()
    assert "random.SystemRandom().randint(" in source
    assert 'parser.add_argument("--seed"' in source


def test_model_sdk_does_not_require_ik_before_seed_placement():
    source = ENTRY.read_text()
    assert 'SDK_OPTIONS = {1: "Normal", 2: "Normal", 3: "Normal"}' in source


def test_task2_uses_its_own_seed_instead_of_a_pinned_baseline():
    source = ENTRY.read_text()
    assert "TASK2_PINNED_SEED" not in source
    assert "task2_initial_base_translation(seed)" in source


def test_task2_initialize_places_both_boxes_from_the_saved_layout():
    source = ENTRY.read_text()
    initialize = source.index("def initialize_task2(")
    body = source[initialize:source.index("def initialize_task3(")]
    assert "Task2RandomizationPlanner().plan(seed)" in body
    assert "set_object_position(" in body
    # The placement has to happen before the initialization motion.
    assert body.index("set_object_position(") < body.index("run_initialization(")


def _literal_assignments(path):
    tree = ast.parse(path.read_text())
    values = {}
    for node in tree.body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)):
            try:
                values[node.targets[0].id] = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                pass
    return values


def test_task1_model_initializer_is_independent_of_full_task_policy():
    source = INIT.read_text()
    entry = ENTRY.read_text()
    assert "from task1_v2 import" not in source
    task1_initialize = entry[entry.index("def initialize_task1("):
                             entry.index("def load_config(")]
    assert "from task1_v2 import" not in task1_initialize


def test_task1_model_shared_initialization_values_match_the_task_entry():
    model_values = _literal_assignments(INIT)
    task_values = _literal_assignments(TASK1)
    for name in (
            "ARM_JOINT_COUNT", "CYLINDERS", "TARGET_BIN",
            "INITIALIZATION_SAFE_RETREAT_M", "SHOULDER_LIFT_DEG",
            "TRAJECTORY_POINTS", "CHASSIS_LINEAR_SPEED",
            "CHASSIS_ANGULAR_SPEED", "CHASSIS_MIN_LINEAR_SPEED",
            "CHASSIS_MIN_ANGULAR_SPEED", "CHASSIS_POSITION_TOLERANCE",
            "CHASSIS_YAW_TOLERANCE_DEG"):
        assert model_values[name] == task_values[name], name


def test_task1_task_entry_staging_does_not_change_model_initialization():
    model_values = _literal_assignments(INIT)
    task_values = _literal_assignments(TASK1)
    assert model_values["RIGHT_ARM_READY_RAD"] == (-1.0, -0.5, 1.0, -1.4)
    assert model_values["RIGHT_ARM_READY_FULL_RAD"] == (
        -0.9, -0.265, 1.0, -0.8, 0.58, -0.4, 0.35)
    assert task_values["RIGHT_ARM_STAGING_RAD"] == (
        -0.569927, -0.301512, 0.658253, -0.488704,
        0.698827, -0.653404, -0.289423)
    assert task_values["RIGHT_ARM_STAGING_ABOVE_RAD"] == (
        -0.466591, -0.579416, 0.652630, -1.211603,
        0.699260, -0.286390, 0.045867)


def test_task1_model_base_initializer_has_no_episode_route():
    source = MODEL_BASE.read_text()
    assert "from task1_v2 import" not in source
    assert "class Task1ModelBaseInitializer" in source
    assert "TASK1_LEVER_BASE_WORLD" not in source


def test_task1_initialize_leaves_no_arm_publisher_running():
    """Task 1 must use the same caller-owned publisher as accepted task1."""
    source = INIT.read_text()
    assert "from utils.trajectory_controller import" not in source
    assert "trajectory.execute_trajectory" in source
    assert "trajectory.stop()" not in source


def test_task1_model_initializer_has_no_removed_lever_latch_publisher():
    """Cleanup must not reference the retired ROS-controlled lever latch."""
    source = INIT.read_text()
    assert "lever_latch_publisher" not in source
    assert "/mujoco/task1_lever_latch_enabled" not in source


def test_task1_initialize_sequence_matches_the_accepted_order():
    source = INIT.read_text()
    order = [
        "place_scene(randomizer, seed, chassis)",
        'move_base_open_loop(chassis, safe_base_world, "safe retreat")',
        "shoulder_only_deg[8] = -SHOULDER_LIFT_DEG",
        "ready_deg[7:11] = [",
        "full_ready_deg[7:14] = [",
        "move_base_open_loop(\n            chassis, base_translation_world, "
        '"return to B0")',
    ]
    positions = [source.index(token) for token in order]
    assert positions == sorted(positions), "initialization order changed"


def test_task1_model_initialization_has_no_closed_loop_arrival_gate():
    source = INIT.read_text()
    for forbidden in (
            "move_to_pose", "translate_relative", "rotate_relative",
            "_move_base_with_wheels", "_randomize_scene"):
        assert forbidden not in source


def test_task1_v2_is_not_modified_by_the_model_entry():
    """The accepted task entry must keep performing its own initialization."""
    source = TASK1.read_text()
    assert "measured_safe_base = _move_base_with_wheels(" in source
    assert 'latch_achieved_right_target("v2_staging_after_return_to_b0")' in source


def test_task1_waits_for_a_real_sensor_message_before_arm_publishing():
    task = TASK1.read_text()
    guard = ARM_STATE.read_text()
    assert "initial_arm_rad = wait_for_first_arm_state()" in task
    assert task.index("initial_arm_rad = wait_for_first_arm_state()") < task.index(
        "trajectory = TrajectoryController(")
    assert 'rospy.wait_for_message(' in guard
    assert 'SENSOR_TOPIC = "/sensors_data_raw"' in guard
    assert "joints[\n        ARM_START_INDEX:ARM_START_INDEX + ARM_JOINT_COUNT]" in guard


def test_every_model_waits_for_the_first_real_arm_state():
    source = ENTRY.read_text()
    initialize = source.index("def _initialize(self):")
    body = source[initialize:source.index("def publish_success(self, value):")]
    assert "initial_arm = wait_for_first_arm_state()" in body
    assert body.index("initial_arm = wait_for_first_arm_state()") < body.index(
        "initialization_trajectory = TrajectoryController(")


def test_tasks_start_arm_publishers_only_after_safe_base_motion():
    task1 = TASK1.read_text()
    task2 = TASK2.read_text()
    task3 = TASK3_SINGLE.read_text()
    task2_init = TASK2_INIT.read_text()
    task3_init = TASK3.read_text()

    task1_main = task1[task1.index("def main():"):]
    assert task1_main.index(
        "measured_safe_base = _move_base_with_wheels(") < task1_main.index(
            "trajectory = TrajectoryController(")
    assert "TrajectoryController(" not in task2
    assert task2_init.index(
        "chassis.wait_until_stopped(stop_speed_threshold, settle_timeout)") \
        < task2_init.index("trajectory = TrajectoryController(")
    assert "TrajectoryController(" not in task3
    assert task3_init.index("chassis.move_to_pose(") < task3_init.index(
        "trajectory = TrajectoryController(")


def test_all_task_initializers_use_the_real_message_gate():
    for path in (INIT, TASK2_INIT, TASK3):
        source = path.read_text()
        function = source[source.index("def wait_for_arm_state("):]
        assert "return wait_for_first_arm_state(timeout=timeout)" in function


def test_task3_model_uses_isolated_open_loop_initialization():
    entry = ENTRY.read_text()
    initialize = entry.index("def initialize_task3(")
    body = entry[initialize:entry.index("INITIALIZERS =")]
    assert "from task3_model_initialize import (" in body
    assert "from task3_initialize import (" not in body

    source = TASK3_MODEL_INIT.read_text()
    order = [
        'move_base_open_loop(chassis, safe_base, "safe retreat")',
        "trajectory.execute_trajectory(",
        "move_base_open_loop(\n        chassis, initial_base, "
        '"return to B0")',
    ]
    positions = [source.index(token) for token in order]
    assert positions == sorted(positions), "Task 3 initialization order changed"
    for forbidden in (
            "move_to_pose", "translate_relative", "rotate_relative"):
        assert forbidden not in source


def test_accepted_task3_initialization_is_unchanged():
    source = TASK3.read_text()
    assert source.count("chassis.move_to_pose(") == 2
