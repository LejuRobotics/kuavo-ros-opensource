from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
TASK = PACKAGE / "examples/task1.py"
NODE = PACKAGE.parents[0] / "mujoco/src/mujoco_node.cc"
LAUNCH = PACKAGE / "launch/load_kuavo_mujoco_sim1.launch"
SCENE = PACKAGE / "models/biped_s400062/xml/task1.xml"


def test_task1_emits_segmented_monotonic_timings():
    source = TASK.read_text()
    assert "class TaskTimingRecorder:" in source
    assert "time.monotonic()" in source
    assert 'print("[TASK1_TIMING] {}"' in source
    assert 'print("[TASK1_TIMING_SUMMARY] {}"' in source
    for label in (
            "initialization.base_retreat",
            "initialization.staging_above",
            "initialization.staging_descent",
            "initialization.base_return_b0",
            "{}.ik_solve",
            "{}.arm_motion",
            "source_grasp.hand_prepare",
            "{}.gripper_close",
            "{}.object_settle_wait",
            "lever.base_motion",
            "lever.conveyor_wait",
            "cleanup.trajectory_stop"):
        assert label in source
    assert source.index("start_score_clock()") < source.index(
        "timing.mark_score_start(scorer_started)")


def test_task1_uses_short_staging_approaches_without_changing_ik_targets():
    source = TASK.read_text()
    expected_staging = (
        "RIGHT_ARM_STAGING_RAD = (\n"
        "    -0.569927, -0.301512, 0.658253, -0.488704,\n"
        "    0.698827, -0.653404, -0.289423,\n"
        ")"
    )
    assert expected_staging in source
    expected_staging_above = (
        "RIGHT_ARM_STAGING_ABOVE_RAD = (\n"
        "    -0.466591, -0.579416, 0.652630, -1.211603,\n"
        "    0.699260, -0.286390, 0.045867,\n"
        ")"
    )
    assert expected_staging_above in source
    assert "SOURCE_GRASP_TRAJECTORY_POINTS = 120" in source
    assert "STAGING_RETURN_TRAJECTORY_POINTS = 80" in source
    assert "IK_TRAJECTORY_POINTS = 120" in source

    initialization = source[
        source.index("shoulder_only_deg = list(current_target_deg)"):
        source.index("scorer_started = start_score_clock()")]
    assert "staging_above_deg[7:14] = [" in initialization
    assert "staging_deg[7:14] = [" in initialization
    assert "ready_deg[7:11] = [" not in initialization
    assert "full_ready_deg[7:14] = [" not in initialization
    assert initialization.index("shoulder_only_deg[8]") < initialization.index(
        "staging_above_deg[7:14]") < initialization.index(
            "initialization.base_return_b0") < initialization.index(
                "staging_deg[7:14]")

    source_grasp = source[
        source.index('"{}_above".format(name)'):
        source.index('"{}_above_target_bin".format(name)')]
    assert "compensate_source_grasp=True" in source_grasp
    assert "trajectory_points=SOURCE_GRASP_TRAJECTORY_POINTS" in source_grasp
    assert "open_gripper_during_motion" not in source
    assert '"{}.gripper_open".format(name)' not in source
    assert '"{}.post_open_wait".format(name)' not in source
    initialization = source[:source.index("scorer_started = start_score_clock()")]
    assert "control_right_gripper(0)" not in initialization
    hand_prepare = source[
        source.index('with timing.segment("source_grasp.hand_prepare")'):
        source.index("for index, name in enumerate(CYLINDERS)")]
    assert "gripper.control_right_gripper(0)" in hand_prepare

    drop = source[
        source.index('"{}_above_target_bin".format(name)'):
        source.index('with timing.segment("{}.pre_release_wait"')]
    assert "trajectory_points=IK_TRAJECTORY_POINTS" in drop
    assert "return_right_to_staging" in source
    assert 'return_right_to_staging("v2_between_objects_staging")' in source
    return_helper = source[
        source.index("def return_right_to_staging"):
        source.index("def print_measured_grasp_center")]
    assert return_helper.index("RIGHT_ARM_STAGING_ABOVE_RAD") < (
        return_helper.index("RIGHT_ARM_STAGING_RAD"))
    assert (
        "scene_ik.eef_quaternion_world_with_yaw(\n"
        "            RIGHT_ARM_READY_FULL_RAD, GRASP_YAW_ADJUSTMENT_RAD)"
    ) in source


def test_task1_v2_lever_uses_fixed_r5_command():
    source = TASK.read_text()
    assert "TASK1_LEVER_INITIAL_R5_COMMAND_RAD = 1.0" in source
    helper_start = source.index("def move_right_joint_target")
    helper_end = source.index("def latch_achieved_lever_target", helper_start)
    helper = source[helper_start:helper_end]
    assert "nominal_r5 = right_target_rad[4]" in helper
    assert "TASK1_LEVER_INITIAL_R5_COMMAND_RAD" in helper
    assert "nominal_r5={:.9f} rad commanded_r5={:.9f} rad" in helper
    assert "actual_r5={:.9f} rad" in source
    assert "r5_command = path_target[4]" in source


def test_task1_v2_source_grasp_uses_current_urdf_compensation():
    source = TASK.read_text()
    assert "GRASP_TRACKING_BIASES_WORLD = (" in source
    assert "(0.002, 0.014, 0.010)" in source
    assert "(0.002, 0.010, 0.010)" in source
    assert "(0.002, 0.005, 0.010)" in source
    assert "grasp_bias_world = GRASP_TRACKING_BIASES_WORLD[index]" in source
    assert "def _compensate_source_grasp_joints" in source
    assert "GRASP_R5_ACTUAL_PER_COMMAND" in source
    assert "GRASP_R7_ACTUAL_PER_COMMAND" in source
    source_grasp = source.index("compensate_source_grasp=True")
    drop = source.index('"{}_above_target_bin".format(name)', source_grasp)
    assert source_grasp < drop
    assert "compensate_source_grasp=True" not in source[drop:]


def test_task1_v2_lever_stops_raising_on_source_bin_handoff():
    source = TASK.read_text()
    assert "LEVER_NOMINAL_HANDOFF_RAD = math.radians(30.0)" in source
    assert "LEVER_PATH_FINE_STEP_RAD = math.radians(0.2)" in source
    assert "LEVER_PATH_FINE_START_RAD = math.radians(29.8)" in source
    assert '"/mujoco/source_bin_latch_released"' in source
    assert "while (not source_bin_release.released" in source
    assert 'latch_achieved_lever_target("v2_lever_handoff_hold")' in source
    assert "expected 30.0 deg" not in source


def test_task1_v2_lever_sequence_is_hand_init_descent_three_finger():
    source = TASK.read_text()
    prepare = source.index('"lever_ik_prepare"')
    prepare_latch = source.index(
        '"v2_lever_ik_prepare"', prepare)
    solve = source.index("approach_solution = scene_ik.solve_lever(")
    approach = source.index('"lever_approach", approach_solution')
    initialize = source.index(
        "initialize_right_lever_hand_before_descent()", approach)
    descent = source.index('"lever_contact", contact_solution', initialize)
    hook = source.index("form_right_lever_hook()", descent)
    assert prepare < prepare_latch < solve < approach
    assert approach < initialize < descent < hook
    assert "task1_lever_latch_enabled" not in source
    assert "LEVER_IK_PREPARE_RIGHT_RAD" in source
    assert "r5_command_rad=LEVER_IK_PREPARE_RIGHT_RAD[4]" in source

    initialization_body = source[
        source.index("def initialize_right_lever_hand_before_descent"):
        source.index("def form_right_lever_hook")]
    for joint in ("r_thumb_j1", "r_thumb_j2", "r_index_j3"):
        assert joint in initialization_body
    assert '"r_thumb_j3"' not in initialization_body
    assert "positions = list(measured)" in initialization_body
    assert '"r_index_j3": 0.0' in initialization_body
    for finger in ("r_middle", "r_little"):
        assert finger not in initialization_body

    hook_body = source[source.index("def form_right_lever_hook"):
                       source.index("live_arm_rad =", source.index("def form_right_lever_hook"))]
    assert '"r_index_j3": 1.5' in hook_body
    assert '"r_middle_j2": 1.5' in hook_body
    assert '"r_little_j3": 1.3' in hook_body
    assert 'name.startswith("r_thumb_")' in hook_body


def test_task1_latch_is_isolated_from_task3_and_excludes_thumb():
    source = NODE.read_text()
    scene = SCENE.read_text()
    assert "Task1LeverFingerLatchState" in source
    assert '"/mujoco/task1_lever_latch_enabled"' not in source
    assert '"/mujoco/task1_lever_finger_latch_mask"' not in source
    assert '"lever_handle_collision"' in source
    assert '"task1_lever_lock"' in source
    assert '"task1_source_bin_lock"' in source
    assert "d->eq_active[task1_lever_latch.lever_lock_equality_id] = 0" in source
    assert "contacting_finger_count >= required_fingers" in source
    assert '<joint name="task1_lever_lock" joint1="lever_hinge"' in scene
    assert '<joint name="task1_source_bin_lock" joint1="source_bin_slide"' in scene
    assert 'name="task1_lever_unlock_required_fingers" data="2"' in scene
    assert 'name="task1_lever_unlock_contact_duration" data="0.05"' in scene
    task1_section = source[source.index("Task1LeverFingerLatchState"):
                           source.index("// Scene 2")]
    assert "task3_" not in task1_section
    # Limit the assertion to mapped geom/joint identifiers; the explanatory
    # comment may state explicitly that thumb is excluded.
    assert '"r_thumb_' not in task1_section
    assert "r_index_fingertip_collision" in source
    assert "r_middle_fingertip_collision" in source
    assert "r_little_fingertip_collision" in source


def test_task1_cylinder_grasp_latches_thumb_and_index_independently():
    source = NODE.read_text()
    scene = SCENE.read_text()
    task1_grasp = source[
        source.index("Task1GraspFingerLatchState"):
        source.index("// Task 1 V2 lever hook")]

    assert 'name="task1_grasp_independent_finger_latch_enabled" data="1"' in scene
    assert 'name="task1_grasp_finger_latch_contact_depth" data="0.0001"' in scene
    assert '"r_thumb_fingertip_collision"' in source
    assert '"r_index_fingertip_collision"' in source
    assert '"r_thumb_j1", "r_thumb_j2", "r_thumb_j3"' in source
    assert '"r_index_j1", "r_index_j2", "r_index_j3"' in source
    assert "task1_grasp_latch.object_body_id == state.body_id" in source
    assert "task1GraspLatchMask() == 0x03" in source
    assert 'clearTask1GraspFingerLatch("right hand opening")' in source
    assert "applyTask1GraspLatchControls();" in source
    assert "task3_" not in task1_grasp


def test_source_bin_lock_target_only_moves_through_joint_service():
    source = NODE.read_text()
    callback = source[
        source.index("bool setJointPositionCallback"):
        source.index("#ifdef USE_DDS", source.index("bool setJointPositionCallback"))]

    assert 'mjOBJ_EQUALITY, "task1_source_bin_lock"' in callback
    assert "m->eq_obj1id[source_bin_lock_id] == joint_id" in callback
    assert "m->eq_data[mjNEQDATA * source_bin_lock_id] = position" in callback
    assert "d->eq_active[source_bin_lock_id] = 1" in callback


def test_task1_v2_launch_does_not_enable_task3_follower():
    source = LAUNCH.read_text()
    assert "internal_contact_follow_body_names" not in source
