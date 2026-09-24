"""Contracts for recoverable simulator startup failures in collect loops."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
HELPER = PACKAGE / "examples/helperfunc.py"


def function_source(source, name, next_name):
    return source[source.index("def {}(".format(name)):
                  source.index("def {}(".format(next_name))]


def test_topic_timeout_is_not_followed_by_service_wait():
    source = HELPER.read_text()
    ready = function_source(
        source, "wait_for_simulator_ready", "report_startup_failure")
    assert "if not wait_for_topics(" in ready
    assert "raise RoundStartupError(" in ready
    assert ready.index("raise RoundStartupError(") < ready.index(
        "rospy.wait_for_service('/set_object_position'")


def test_recording_round_converts_startup_failure_to_failed_result():
    source = HELPER.read_text()
    run_once = function_source(source, "run_once", "_run_once")
    assert "except RoundStartupError as error:" in run_once
    assert "report_startup_failure(" in run_once
    assert "round_id, start_time, bag_filename" in run_once
    assert "return duration, is_success, None" in run_once


def test_non_recording_round_converts_startup_failure_to_failed_result():
    source = HELPER.read_text()
    run_only = function_source(source, "run_only_task", "_run_only_task")
    assert "clear_task_result()" in run_only
    assert "except RoundStartupError as error:" in run_only
    assert "report_startup_failure(round_id, start_time)" in run_only


def test_keyboard_interrupt_is_not_treated_as_startup_failure():
    source = HELPER.read_text()
    wait_topics = function_source(
        source, "wait_for_topics", "wait_for_simulator_ready")
    run_only = function_source(source, "run_only_task", "_run_only_task")
    run_once = function_source(source, "run_once", "_run_once")
    assert "except:" not in wait_topics
    assert "except Exception" not in run_only
    assert "except Exception" not in run_once


def test_mixed_success_and_startup_failure_scores_are_reportable():
    source = HELPER.read_text()
    main = source[source.index("def _main("):]
    assert "if score is not None else 'N/A'" in main


def test_only_non_recording_attempt_clears_a_stale_score():
    source = HELPER.read_text()
    run_only = function_source(source, "run_only_task", "_run_only_task")
    run_once = function_source(source, "run_once", "_run_once")
    expected = "clear_score_result(task_id, round_id)"
    assert expected in run_only
    assert expected not in run_once


def test_recording_attempt_allocates_and_reads_its_persistent_score_path():
    source = HELPER.read_text()
    collect = function_source(source, "_run_once", "main")
    main = source[source.index("def _main("):]
    assert "allocate_collect_score_file(" in collect
    assert "score_file=score_file" in collect
    assert "return duration, is_success, score_file" in collect
    assert "duration, is_success, score_file = run_once(" in main
    assert "None if score_file is None else read_score(" in main
    assert "score_file=score_file" in main
