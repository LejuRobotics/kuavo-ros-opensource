#!/usr/bin/env python3
"""Regression tests for Task 2 pose-based scoring thresholds."""

from utils.task_scorer import Task2Scorer


class FakePoseTap(object):
    def __init__(self):
        self.positions = {
            "box_1": (0.74, -0.42, 0.60),
            "box_2": (0.74, 0.10, 0.60),
            "base_link": (0.0, 0.0, 0.0),
        }

    def position(self, name):
        return self.positions.get(name)


def test_task2_short_confirmed_lift_awards_grasp_once():
    tap = FakePoseTap()
    scorer = Task2Scorer(tap, "/tmp/unused-task2-score.txt")

    assert scorer._evaluate()["lifted"] == []

    tap.positions["box_1"] = (0.74, -0.42, 0.611)
    assert scorer._evaluate()["lifted"] == ["box_1"]

    # Grasp credit is historical and remains after the box is placed/released.
    tap.positions["box_1"] = (-0.04, 1.52, 0.60)
    state = scorer._evaluate()
    assert state["lifted"] == ["box_1"]
    assert state["placed"] == ["box_1"]
    assert scorer.final_score(state, 0.0)[1]["grasp"] == 10


def test_task2_resting_box_does_not_award_grasp():
    tap = FakePoseTap()
    scorer = Task2Scorer(tap, "/tmp/unused-task2-score.txt")

    tap.positions["box_1"] = (0.74, -0.42, 0.609)
    assert scorer._evaluate()["lifted"] == []


def test_task2_completed_round_can_receive_full_score():
    scorer = Task2Scorer(FakePoseTap(), "/tmp/unused-task2-score.txt")
    state = {
        "lifted": ["box_1", "box_2"],
        "placed": ["box_1", "box_2"],
        "base_visits": 2,
    }

    total, components = scorer.final_score(state, 0.0)

    assert total == 100
    assert components["grasp"] == 20
    assert components["bonus"] == 10
