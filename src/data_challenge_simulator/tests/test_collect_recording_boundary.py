"""Static contracts for the shared post-initialization rosbag boundary."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
HELPER = PACKAGE / "examples/helperfunc.py"
SCORER_CLOCK = PACKAGE / "utils/scorer_clock.py"


def test_collect_uses_task_initialization_boundary_to_start_rosbag():
    helper = HELPER.read_text()
    run_once = helper[helper.index("def _run_once("):
                      helper.index("def main(", helper.index("def _run_once("))]
    assert "advertise_recording_gate(" in run_once
    assert "TASK_RECORD_AFTER_INIT'] = '1'" in run_once
    assert run_once.index("advertise_recording_gate(") < run_once.index(
        "subprocess.Popen(\n        ['python3', task_script]")
    assert "['rosbag', 'record'" not in run_once


def test_recording_gate_waits_for_rosbag_before_releasing_task():
    helper = HELPER.read_text()
    gate = helper[helper.index("def advertise_recording_gate("):
                  helper.index("def wait_for_topics(")]
    assert "['rosbag', 'record'" in gate
    assert gate.index("time.sleep(1)") < gate.index(
        'TriggerResponse(success=True, message="recording started")')


def test_all_three_tasks_share_the_boundary_call():
    for name in ("task1.py", "task2.py", "task3.py"):
        source = (PACKAGE / "examples" / name).read_text()
        assert "start_score_clock()" in source


def test_collection_gate_opens_before_score_clock():
    source = SCORER_CLOCK.read_text()
    function = source[source.index("def start_score_clock():"):]
    assert function.index("start_recording_after_initialization()") < \
        function.index("rospy.wait_for_service(START_SERVICE")


def test_task3_waits_for_chassis_state_before_initialization():
    source = (PACKAGE / "examples/task3.py").read_text()
    initialization = source[
        source.index("if stage is None:"):
        source.index('elif stage.kind == "descend"')]
    assert initialization.index(
        "runtime.chassis.wait_until_ready(timeout=30.0)") < \
        initialization.index("run_initialize(")


def test_each_round_removes_stale_mujoco_initial_state_after_old_nodes():
    helper = HELPER.read_text()
    cleanup = helper[
        helper.index("def ensure_clean_simulator_graph("):
        helper.index("def record_topics(")]
    assert cleanup.index("clear_simulator_nodes(timeout=timeout)") < \
        cleanup.index("rospy.delete_param('/robot_init_state_param')")
