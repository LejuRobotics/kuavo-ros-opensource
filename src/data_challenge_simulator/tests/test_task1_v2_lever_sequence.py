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
            "{}.ik_solve",
            "{}.arm_motion",
            "source_grasp.hand_prepare",
            "{}.gripper_close",
            "{}.gripper_open",
            "lever.conveyor_wait",
            "cleanup.trajectory_stop"):
        assert label in source
    assert source.index("start_score_clock()") < source.index(
        "timing.mark_score_start(scorer_started)")


def test_task1_moves_directly_and_carries_above_bin_walls():
    source = TASK.read_text()
    assert "return_right_to_staging" not in source
    assert "scene_ik.preparation_trajectory(" in source
    assert "lift_center[2] = DROP_GRASP_CENTER_Z" in source
    assert "DROP_GRASP_CENTER_Z = 0.81" in source
    assert source.index('"{}_lift_and_transfer".format(name)') < source.index(
        'release_cylinder(gripper, name, timing)', source.index('def main():'))
    assert "on_clearance=check_physical_lift" in source
    assert source.index("on_clearance()") < source.index(
        "execute_right_trajectory(points[clearance_index:])")
    assert '"{}_lower_to_bin".format(name)' not in source
    assert "seed_only=True" in source
    assert "IK path changes joint branch" in source


def test_task1_lever_uses_ik_without_a_fixed_joint_override():
    source = TASK.read_text()
    assert "TASK1_LEVER_INITIAL_R5_COMMAND_RAD" not in source
    assert "r5_command = path_target[4]" in source
    assert "joint_bounds=lever_bounds" in source


def test_task1_checks_physical_lift_and_releases_without_withdrawal():
    source = TASK.read_text()
    assert "did not lift off the source bin" in source
    assert '"{}_withdraw_from_bin".format(name)' not in source
    assert "applyTask1GraspLatchControls" not in NODE.read_text()


def test_task1_v2_lever_stops_raising_on_source_bin_handoff():
    source = TASK.read_text()
    assert "LEVER_PATH_FINE_STEP_RAD = math.radians(0.2)" in source
    assert "LEVER_PATH_FINE_START_RAD = math.radians(29.8)" in source
    assert "LEVER_PATH_FINE_START_TOLERANCE_RAD = math.radians(0.1)" in source
    assert '"/mujoco/source_bin_latch_released"' in source
    assert "while (not source_bin_release.released" in source
    assert 'latch_achieved_lever_target("v2_lever_handoff_hold")' in source
    assert "expected 30.0 deg" not in source


def test_task1_v2_lever_pull_keeps_velocity_across_segment_boundaries():
    source = TASK.read_text()
    assert "LEVER_PATH_TRAJECTORY_POINTS = 12" in source
    assert "LEVER_PATH_FINE_TRAJECTORY_POINTS = 2" in source

    helper = source[
        source.index("def move_right_joint_target"):
        source.index("def latch_achieved_lever_target")]
    assert "if continuous else" in helper
    assert "list(current_target_deg)" in helper

    pull = source[
        source.index("while (not source_bin_release.released"):
        source.index("if not source_bin_release.released:",
                     source.index("while (not source_bin_release.released"))]
    assert "LEVER_PATH_FINE_TRAJECTORY_POINTS" in pull
    assert "commanded_angle + LEVER_PATH_FINE_START_TOLERANCE_RAD" in pull
    assert "commanded_angle + step_rad" in pull
    assert "lever_angle + step_rad" not in pull
    assert "commanded_angle = target_angle" in pull
    assert "continuous=True" in pull
    assert "smooth=True" not in pull


def test_task1_lever_localizes_and_confirms_contact_before_lifting():
    source = TASK.read_text()
    sequence = source[source.index('with timing.segment("lever.hand_initialize")'):]
    stages = ['"lever_approach_contact", contact_target', 'via_position_world=approach_target',
              "V2 lever palm face seated; closing fingers",
              'with timing.segment("lever.grasp_close")',
              "lever palm/finger grip not established", "V2 lever palm/finger physical grip confirmed",
              "grip_anchor_tcp = scene_ik._forward(", "while (not source_bin_release.released"]
    positions = [sequence.index(stage) for stage in stages]
    assert positions == sorted(positions)
    assert '"/mujoco/lever/grasp_confirmed"' in source
    assert 'latch_achieved_lever_target("v2_lever_hook_hold")' not in source


def test_task1_lever_unlock_requires_opposing_grip_and_keeps_dynamics():
    source = NODE.read_text()
    body = source[source.index("void initializeTask1LeverFingerLatch()"):
                  source.index("mjtNum numericScalarOrDefault(const char *name, mjtNum fallback)\n{")]
    assert '"task2_r_hand_base_collision"' in body
    assert "mj_contactForce" in body
    assert "mju_dot3(normals[0], normals[finger]) < -0.5 * palm_force * finger_force" in body
    assert "grip_duration" in body
    assert "d->eq_active[task1_lever_latch.lever_lock_equality_id] = 0" in body
    assert 'name="task1_lever_unlock_contact_duration" data="0.05"' in SCENE.read_text()
    for forbidden in ("d->qpos[", "d->qvel[", "m->geom_contype[", "m->geom_conaffinity[", "d->qfrc_applied["):
        assert forbidden not in body
    assert "task1_lever_latch.palm_seated && opposing_fingers >= 2" in body
    assert "front_dot < 0.5" in body


def test_task1_cylinder_feedback_observes_thumb_and_index_without_freezing():
    source = NODE.read_text()
    assert '"r_thumb_link1"' in source
    assert '"r_index_link1"' in source
    assert "applyTask1GraspLatchControls" not in source
    assert "Task1GraspFingerLatchState" not in source


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


def test_lever_contact_timer_ignores_short_gaps_but_resets_after_grip_loss(tmp_path):
    import subprocess
    source = NODE.read_text()
    start = source.index("  if (opposing_grip) {", source.index("bool updateTask1LeverFingerLatch()"))
    end = source.index("  d->eq_active[", start)
    # Compile the actual timer branch, including its configured threshold.
    code = """
#include <cassert>
struct State { double grip_duration=0, grip_gap_duration=0; } task1_lever_latch;
struct Model { struct { double timestep=.002; } opt; } model, *m=&model;
double numericScalarOrDefault(const char*, double fallback) { return fallback; }
bool step(bool opposing_grip) {
""" + source[start:end] + """
  return true;
}
int main() {
  for (int i=0;i<20;i++) assert(!step(true));
  for (int i=0;i<6;i++) assert(!step(false));
  assert(task1_lever_latch.grip_duration==0);
  for (int i=0;i<10;i++) assert(!step(true));
  for (int i=0;i<3;i++) assert(!step(false));
  for (int i=0;i<14;i++) assert(!step(true));
  assert(step(true));
  task1_lever_latch=State{};
  for (int i=0;i<100;i++) assert(!step(false));
}
"""
    cpp = tmp_path / "contact_timer.cc"
    cpp.write_text(code)
    executable = tmp_path / "contact_timer"
    subprocess.run(["g++", "-std=c++11", str(cpp), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
