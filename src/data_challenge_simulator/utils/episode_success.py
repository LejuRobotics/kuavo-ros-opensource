#!/usr/bin/env python3
"""Decide when one automatic-evaluation episode is finished.

The upstream model harness (``kuavo_data_challenge``'s
``kuavo_deploy/src/eval/sim_auto_test.py``) ends an episode the moment
``/simulator/success`` carries ``True`` -- its rollout loop is literally
``done = done or success_evt.is_set()``, and ``success_evt`` is only set by a
``True`` on that topic.  So the simulator side has to publish that ``True``.

This module supplies the decision without inventing a second definition of
"done": it drives the accepted ``utils/task_scorer.py`` scorer classes' own
``_evaluate()`` and reports success when every full-credit condition those
scorers already encode holds at the same instant.  The same scorer instance
also writes the model episode's numeric score when the episode ends.

The conditions mirror each scorer's ``all_done`` test:

* Task 1 -- three cylinders in the target bin and the source bin at the end
  of its conveyor transfer;
* Task 2 -- both boxes lifted and on the conveyor, and both chassis arrivals;
* Task 3 -- three rings lifted and placed, and all three chassis arrivals.

Time penalties are intentionally excluded from the success decision: a round
that finishes the work after the base time still succeeds, but its numeric
score is lower.  The model clock starts at ``/simulator/start``, after fixed
initialization, and finishes on success, reset, or simulator shutdown.
"""

import os
import threading
import time

import rospy

from utils.task_scorer import DEFAULT_SCORE_FILE, SCORERS, PoseTap


def _task1_complete(scorer, state):
    """Three cylinders placed and the source-bin transfer fully finished."""
    placed = set(state.get("placed", ()))
    return (
        all(name in placed for name in scorer.CYLINDERS)
        and bool(state.get("conveyor_complete", False)))


def _task2_complete(scorer, state):
    """Both boxes lifted and released on the conveyor, chassis arrived twice."""
    lifted = set(state.get("lifted", ()))
    placed = set(state.get("placed", ()))
    return (
        all(name in lifted for name in scorer.BOXES)
        and all(name in placed for name in scorer.BOXES)
        and state.get("base_visits", 0) >= scorer.BASE_ARRIVALS_NEEDED)


def _task3_complete(scorer, state):
    """Three rings lifted and placed, chassis arrived at the table each time."""
    lifted = set(state.get("lifted", ()))
    placed = set(state.get("placed", ()))
    return (
        all(name in lifted for name in scorer.RINGS)
        and all(name in placed for name in scorer.RINGS)
        and state.get("base_visits", 0) >= scorer.ARRIVALS_NEEDED)


COMPLETION_RULES = {1: _task1_complete, 2: _task2_complete, 3: _task3_complete}


class EpisodeSuccessObserver(object):
    """Answer "is this episode done?" from the task's own scoring state.

    Sampling uses the scorer classes' ``_evaluate()`` rather than a copy of
    their rules.  That reaches one private method across a module boundary,
    which is the deliberate price of keeping ``task_scorer.py`` -- and the
    score baseline the accepted entries depend on -- unmodified.
    """

    def __init__(self, task_id, score_file=None):
        try:
            scorer_class = SCORERS[task_id]
        except KeyError:
            raise ValueError("unsupported task id: {}".format(task_id))
        self.task_id = task_id
        self.rule = COMPLETION_RULES[task_id]
        self._lock = threading.Lock()
        self._finished_result = None
        # Same /mujoco/<body>/pose ground truth the scorer process watches.
        # Bodies that have not published yet read as None, never as success.
        self.tap = PoseTap(scorer_class.BODIES)
        self.scorer = scorer_class(
            self.tap,
            score_file or os.environ.get("SCORE_FILE", DEFAULT_SCORE_FILE))

    def start(self):
        """Start the score clock at the post-initialization episode boundary."""
        with self._lock:
            if self.scorer.start_time is None:
                self.scorer.start_time = time.time()
                rospy.loginfo("model score clock started")

    def sample(self):
        """Return ``(state, complete)`` for the current simulation state."""
        with self._lock:
            state = self.scorer._evaluate()
            self.scorer.state = state
            return state, bool(self.rule(self.scorer, state))

    def finish(self):
        """Write this episode's score once and return ``(total, components)``."""
        with self._lock:
            if self._finished_result is not None:
                return self._finished_result
            if self.scorer.start_time is None:
                return None
            # Take a final sample so a reset arriving between polling ticks does
            # not lose the last completed scoring event.
            self.scorer.state = self.scorer._evaluate()
            self._finished_result = self.scorer.finish(write_only=True)
            return self._finished_result

    def close(self):
        """Drop the pose subscriptions.

        One observer serves one round -- the scorers keep sticky "ever
        grasped"/"ever placed" state that must not carry into the next round's
        scene -- so the entry builds a fresh one per round and must release
        the old round's subscriptions rather than leave them firing against
        every pose message for the rest of the session.
        """
        for subscriber in self.tap._subscribers:
            try:
                subscriber.unregister()
            except Exception:  # pragma: no cover - ROS teardown guard
                pass
        self.tap._subscribers = []

    def describe(self):
        """One-line summary of what the observer is still waiting for."""
        return "task {} success observer on {}".format(
            self.task_id, ", ".join(self.scorer.BODIES))


def _log_observer_ready():
    """Kept separate so importing this module never needs a ROS node."""
    rospy.logdebug("episode success observer loaded")
