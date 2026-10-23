"""Task 2 consumes real geometry/contact observations, not fixed XYZ gates."""
import json
from types import SimpleNamespace
import pytest
from utils.task_scorer import Task2Scorer


class FakePoseTap:
    def __init__(self):
        self.task2_state = {"box_1": 8, "box_2": 8}
        self.last_message_time = 0


def scorer_at_start(tmp_path):
    tap = FakePoseTap()
    scorer = Task2Scorer(tap, str(tmp_path / "score.txt"))
    scorer.sample()  # Observe the initial tabletop support.
    scorer.start_time = 100.0
    return scorer, tap


def test_contacts_record_each_milestone_once_and_keep_earned_points(tmp_path):
    scorer, tap = scorer_at_start(tmp_path)
    tap.task2_state["box_1"] = 1
    assert scorer.sample()["lifted"] == ["box_1"]
    tap.task2_state["box_1"] = 2
    assert scorer.sample()["base_visits"] == 1
    assert scorer.sample()["placed"] == []
    tap.task2_state["box_1"] = 4
    assert scorer.sample()["placed"] == ["box_1"]
    tap.task2_state["box_1"] = 0
    state = scorer.sample()
    assert scorer.final_score(state, 101)[0] == 45
    assert state["base_visits"] == 1


def test_no_grasp_credit_before_start_or_without_initial_table_support(tmp_path):
    tap = FakePoseTap()
    tap.task2_state = {"box_1": 1}
    scorer = Task2Scorer(tap, str(tmp_path / "score.txt"))
    assert scorer.sample()["lifted"] == []
    scorer.start_time = 100
    assert scorer.sample()["lifted"] == []
    tap.task2_state["box_1"] = 8
    scorer.sample()
    tap.task2_state["box_1"] = 1
    assert scorer.sample()["lifted"] == ["box_1"]


def test_completed_round_and_forty_five_second_penalty_boundary(tmp_path):
    scorer, tap = scorer_at_start(tmp_path)
    for box in tap.task2_state:
        tap.task2_state[box] = 1
    scorer.sample()
    for box in tap.task2_state:
        tap.task2_state[box] = 2 | 4
    state = scorer.sample()
    for elapsed, expected in [(44.5,100), (45,100), (45.5,99.5), (50,95), (100,80)]:
        total, components = scorer.final_score(state, 100 + elapsed)
        assert total == expected
        assert components["bonus"] == 10
    assert scorer.final_score({}, 200)[0] == 0


def test_scorer_finishes_cleanly_when_ros_sleep_is_interrupted(monkeypatch, tmp_path):
    import json
    import time
    from types import SimpleNamespace
    from utils import task_scorer

    class Interrupted(Exception):
        pass

    def interrupted_sleep():
        raise Interrupted()

    monkeypatch.setattr(task_scorer, "rospy", SimpleNamespace(
        Service=lambda *args: None,
        Rate=lambda hz: SimpleNamespace(sleep=interrupted_sleep),
        ROSInterruptException=Interrupted,
        is_shutdown=lambda: False,
        signal_shutdown=lambda reason: None,
        loginfo=lambda *args: None,
    ))
    tap = FakePoseTap()
    tap.last_message_time = time.time()
    scorer = Task2Scorer(tap, str(tmp_path / "score.txt"))
    assert scorer.run() == 0
    assert json.loads((tmp_path / "score.json").read_text())["total"] == 0
