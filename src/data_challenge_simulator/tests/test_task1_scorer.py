#!/usr/bin/env python3
"""Regression tests for Task 1 conveyor-based scoring semantics."""

import math

import pytest

from utils.episode_success import _task1_complete
from utils.task_scorer import Task1Scorer, relative_local_x


class FakePoseTap(object):
    def __init__(self):
        self.positions = {
            "cylinder_1": (0.0, 0.0, 0.0),
            "cylinder_2": (0.0, 0.0, 0.0),
            "cylinder_3": (0.0, 0.0, 0.0),
            "lever": (1.0, 2.0, 3.0),
            "source_bin": (1.0, 2.0, 3.03),
        }
        half_sqrt_two = math.sqrt(0.5)
        self.orientations = {
            # Local +X points along world +Y.
            "lever": (0.0, 0.0, half_sqrt_two, half_sqrt_two),
        }

    def position(self, name):
        return self.positions.get(name)

    def orientation(self, name):
        return self.orientations.get(name)

    def set_slide(self, slide_m):
        self.positions["source_bin"] = (1.0, 2.0 + slide_m, 3.03)


def test_relative_local_x_tracks_slide_with_rotated_parent():
    tap = FakePoseTap()
    tap.set_slide(0.123)

    assert relative_local_x(
        tap.position("source_bin"),
        tap.position("lever"),
        tap.orientation("lever")) == pytest.approx(0.123)


def test_task1_lever_points_start_on_actual_bin_motion_and_latch():
    tap = FakePoseTap()
    scorer = Task1Scorer(tap, "/tmp/unused-task1-score.txt")

    tap.set_slide(0.004)
    assert scorer._evaluate()["lever_done"] is False

    tap.set_slide(0.006)
    state = scorer._evaluate()
    assert state["lever_done"] is True
    assert state["conveyor_complete"] is False
    assert scorer.final_score(state, 0.0)[1]["lever"] == 25

    tap.set_slide(0.0)
    assert scorer._evaluate()["lever_done"] is True


def test_task1_episode_waits_for_conveyor_end_not_point_award():
    tap = FakePoseTap()
    scorer = Task1Scorer(tap, "/tmp/unused-task1-score.txt")
    placed = list(scorer.CYLINDERS)

    tap.set_slide(0.006)
    state = scorer._evaluate()
    state["placed"] = placed
    assert state["lever_done"] is True
    assert _task1_complete(scorer, state) is False

    tap.set_slide(0.441)
    state = scorer._evaluate()
    state["placed"] = placed
    assert state["conveyor_complete"] is True
    assert _task1_complete(scorer, state) is True
