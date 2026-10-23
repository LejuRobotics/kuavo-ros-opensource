#!/usr/bin/env python3
"""Standalone task scorers for the three data-challenge tasks.

Pattern borrowed from the upstream deploy/eval*.py flow: scoring is a
separate observer process that subscribes to the /mujoco/<body>/pose
ground truth, watches the official scoring rules at 10 Hz, and writes the
final score plus per-component details when the round ends.  It never
commands the robot, so a scorer crash cannot affect the task itself.

Rules (user-confirmed 2026-09-17):

Task 1 (full 100): 10 per cylinder lifted off the source-bin floor (30),
15 per cylinder fully inside the target bin (45), 25 for completing the
lever operation after the bin reaches the conveyor end and settles.
Base time 50 s; 0.5 per second over, capped at 10.

Task 2 (full 100): 10 per box lifted off the table (20), 20 per box
transported over the conveyor (40), 15 per box placed anywhere on the
destination conveyor (30), 10 bonus when everything is done.
Transport no longer requires a fixed chassis parking position.
Base time 45 s; 1 per second over, capped at 20.

Task 3 (full 100): 5 per small part grasped (15), 15 per ring moved above
the destination table (45), 10 per part placed in the target region (30),
10 bonus when all parts are placed.
Base time 45 s; 1 per second over, capped at 20.

Timing protocol: the scorer observes state from launch, but the time-penalty
clock starts only when the task calls /task_scorer/start after its fixed
initialization completes.  helperfunc.py calls /task_scorer/finalize after
the task script exits; the scorer then writes SCORE_FILE (one line, total)
plus SCORE_FILE-with-.json (total + component breakdown) and shuts down.  If
topics fall silent for SILENCE_SEC first (simulation died), the scorer
finalises on its own so a score still lands on disk.
"""

import argparse
import json
import math
import os
import sys
import time
import traceback
import threading
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import UInt8

# Also support launching this file directly as a standalone scorer.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utils.score_output import write_score


DEFAULT_SCORE_FILE = "/tmp/simulator_score_last.txt"
FINALIZE_SERVICE = "/task_scorer/finalize"
START_SERVICE = "/task_scorer/start"
SILENCE_SEC = 30.0


class PoseTap(object):
    """Minimal /mujoco/<body>/pose subscriber, independent of ObjectPose.

    ObjectPose does not list every scoring body (e.g. task 1 needs
    source_bin), and it is a shared file; the scorer subscribes on its own.
    """

    def __init__(self, names):
        self.data = {}
        self.task2_state = {}
        self.last_message_time = time.time()
        self._subscribers = []
        for name in names:
            self.data[name] = None
            self._subscribers.append(rospy.Subscriber(
                "/mujoco/{}/pose".format(name), PoseStamped,
                self._callback, callback_args=name, queue_size=1))
            if name in ("box_1", "box_2", "cylinder_1", "cylinder_2", "cylinder_3",
                        "source_bin", "task3_hollow_cylinder", "task3_hollow_cylinder_2",
                        "task3_hollow_cylinder_3"):
                self._subscribers.append(rospy.Subscriber(
                    "/mujoco/{}/scoring_state".format(name), UInt8,
                    self._task2_callback, callback_args=name, queue_size=1))

    def _callback(self, message, name):
        self.data[name] = message.pose
        self.last_message_time = time.time()

    def _task2_callback(self, message, name):
        self.task2_state[name] = int(message.data)
        self.last_message_time = time.time()

    def position(self, name):
        pose = self.data.get(name)
        if pose is None:
            return None
        return (pose.position.x, pose.position.y, pose.position.z)

    def orientation(self, name):
        pose = self.data.get(name)
        if pose is None:
            return None
        q = pose.orientation
        return (q.x, q.y, q.z, q.w)


def in_xy_rect(position, x_range, y_range):
    if position is None:
        return False
    return x_range[0] <= position[0] <= x_range[1] and \
        y_range[0] <= position[1] <= y_range[1]


def base_distance(position, xy):
    if position is None:
        return None
    return math.hypot(position[0] - xy[0], position[1] - xy[1])


def relative_local_x(position, parent_position, parent_quaternion_xyzw):
    """Project a world-space offset onto the parent's local +X axis."""
    if (position is None or parent_position is None
            or parent_quaternion_xyzw is None):
        return None
    x, y, z, w = parent_quaternion_xyzw
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return None
    x, y, z, w = (value / norm for value in (x, y, z, w))
    local_x_world = (
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + z * w),
        2.0 * (x * z - y * w),
    )
    offset = tuple(
        position[index] - parent_position[index] for index in range(3))
    return sum(offset[index] * local_x_world[index] for index in range(3))


class BaseScorer(object):
    """Shared 10 Hz observer loop over an _evaluate() pure function."""

    RATE_HZ = 10.0

    def __init__(self, tap, score_file, metadata=None, latest_file=None):
        self.tap = tap
        self.score_file = score_file
        self.start_time = None
        self.state = {}
        self.metadata = (json.loads(os.environ.get("SCORE_METADATA", "{}"))
                         if metadata is None else dict(metadata))
        self.latest_file = latest_file or os.environ.get("SCORE_LATEST_FILE")
        self._score_lock = threading.RLock()
        self._last_snapshot = None
        self._finished_result = None
        self._invalid = False
        self.publish_score(force=True)

    # -- interface -------------------------------------------------------
    def _evaluate(self):
        """Sample the pose tap; return the JSON-safe component state."""
        raise NotImplementedError

    def final_score(self, state, end_time):
        """Return (total, components) for the sampled state."""
        raise NotImplementedError

    # -- loop ------------------------------------------------------------
    def run(self):
        rospy.Service(START_SERVICE, Trigger, self._on_start)
        rospy.Service(FINALIZE_SERVICE, Trigger, self._on_finalize)
        rate = rospy.Rate(self.RATE_HZ)
        while not rospy.is_shutdown():
            self.sample()
            if time.time() - self.tap.last_message_time > SILENCE_SEC:
                rospy.logwarn(
                    "[scorer] pose topics silent for {}s; finalising".format(
                        SILENCE_SEC))
                self.finish()
                return 0
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        # rospy shutdown (e.g. simulation gone): still write what we saw.
        self.finish()
        return 0

    def sample(self):
        with self._score_lock:
            if self._finished_result is None and not self._invalid:
                self.state = self._evaluate()
                self.publish_score()
            return self.state

    def publish_score(self, status=None, force=False):
        with self._score_lock:
            if self._finished_result is not None:
                return self._finished_result
            if self._invalid:
                return self.final_score(self.state, time.time())
            now = time.time()
            total, components = self.final_score(self.state, now)
            payload = dict(self.metadata, total=int(round(total)),
                           components=components,
                           status=status or ("running" if self.start_time is not None
                                             else "preparing"))
            if force or payload != self._last_snapshot:
                write_score(self.score_file, payload, self.latest_file)
                self._last_snapshot = payload
            return total, components

    def invalidate(self, reason):
        with self._score_lock:
            payload = dict(self._last_snapshot, status="invalid", finish_reason=str(reason))
            write_score(self.score_file, payload, self.latest_file)
            self._invalid = True

    def _on_start(self, _request):
        with self._score_lock:
            if self.start_time is None:
                self.start_time = time.time()
                self.publish_score(force=True)
                rospy.loginfo("[scorer] time-penalty clock started")
        return TriggerResponse(success=True, message="score clock started")

    def _on_finalize(self, _request):
        total, components = self.finish(write_only=True)
        return TriggerResponse(success=True, message=json.dumps(
            {"total": total, "components": components}))

    def finish(self, write_only=False):
        with self._score_lock:
            if self._invalid:
                raise RuntimeError("cannot finalize an invalid score")
            if self._finished_result is None:
                self.state = self._evaluate()
                self._finished_result = self.publish_score(status="completed", force=True)
                total, components = self._finished_result
                rospy.loginfo("[scorer] total=%s components=%s -> %s",
                              int(round(total)), components, self.score_file)
        if not write_only:
            rospy.signal_shutdown("scorer finalised")
        return self._finished_result

    def time_penalty(self, end_time, base_sec, per_sec, cap):
        """Return (penalty, overtime_seconds) for a finished round."""
        if self.start_time is None:
            return 0.0, 0.0
        over = end_time - self.start_time - base_sec
        if over <= 0:
            return 0.0, 0.0
        return min(cap, over * per_sec), over


class Task1Scorer(BaseScorer):
    """Three cylinders out of the source bin into the target bin + lever."""

    BODIES = ("cylinder_1", "cylinder_2", "cylinder_3",
              "source_bin", "lever", "base_link")
    CYLINDERS = ("cylinder_1", "cylinder_2", "cylinder_3")

    GRASP_SCORE = 10
    PLACE_SCORE = 15
    LEVER_SCORE = 25
    BASE_TIME_SEC = 50.0
    OVER_PENALTY_PER_SEC = 0.5
    OVER_PENALTY_CAP = 10.0
    def __init__(self, tap, score_file, **output_options):
        super(Task1Scorer, self).__init__(tap, score_file, **output_options)
        self.seen_source = set()
        self.ever_grasped = set()
        self.ever_placed = set()
        self.ever_conveyor_complete = False

    @staticmethod
    def lever_angle(quaternion_xyzw):
        """Unsigned 0..pi hinge rotation, same math as task1.py."""
        if quaternion_xyzw is None:
            return None
        x, y, z, w = quaternion_xyzw
        vector_norm = math.sqrt(x * x + y * y + z * z)
        angle = 2.0 * math.atan2(vector_norm, w)
        if angle > math.pi:
            angle = 2.0 * math.pi - angle
        return angle

    def _evaluate(self):
        for name in self.CYLINDERS:
            flags = self.tap.task2_state.get(name, 0)
            if flags & 8:
                self.seen_source.add(name)
            if self.start_time is None:
                continue
            if flags & 1 and name in self.seen_source:
                self.ever_grasped.add(name)
            if flags & 4:
                self.ever_placed.add(name)
        settled = bool(self.tap.task2_state.get("source_bin", 0) & 16)
        if self.start_time is not None and settled:
            self.ever_conveyor_complete = True
        return {
            "grasped": sorted(self.ever_grasped),
            "placed": sorted(self.ever_placed),
            "lever_done": self.ever_conveyor_complete,
            "conveyor_complete": self.ever_conveyor_complete,
        }

    def final_score(self, state, end_time):
        lever_done = state.get("lever_done", False)
        components = {
            "grasp": self.GRASP_SCORE * len(state.get("grasped", [])),
            "place": self.PLACE_SCORE * len(state.get("placed", [])),
            "lever": self.LEVER_SCORE if lever_done else 0,
        }
        penalty, over = self.time_penalty(
            end_time, self.BASE_TIME_SEC,
            self.OVER_PENALTY_PER_SEC, self.OVER_PENALTY_CAP)
        components["time_penalty"] = -round(penalty, 3)
        components["over_sec"] = round(over, 3)
        total = (
            components["grasp"] + components["place"] + components["lever"]
            - penalty)
        return max(0.0, total), components


class Task2Scorer(BaseScorer):
    """Bimanual box grasp, chassis transfer, release on the conveyor."""

    BODIES = ("box_1", "box_2", "base_link")
    BOXES = ("box_1", "box_2")

    # These world bounds are retained for the task motion's release guard.
    # Scoring uses simulator geometry/contact flags, never fixed XYZ heights.
    CONVEYOR_X = (-0.48, 0.40)
    CONVEYOR_Y = (0.62, 2.42)
    CONVEYOR_Z = (0.55, 0.68)
    # Keep the existing score field name for report consumers.
    BASE_ARRIVALS_NEEDED = 2

    GRASP_SCORE = 10
    BASE_SCORE = 20
    PLACE_SCORE = 15
    BONUS_SCORE = 10
    BASE_TIME_SEC = 45.0
    OVER_PENALTY_PER_SEC = 1.0
    OVER_PENALTY_CAP = 20.0

    def __init__(self, tap, score_file, **output_options):
        super(Task2Scorer, self).__init__(tap, score_file, **output_options)
        self.seen_table = set()
        self.ever_lifted = set()
        self.ever_placed = set()
        self.ever_arrived = set()

    def _evaluate(self):
        state = {}
        for name in self.BOXES:
            flags = self.tap.task2_state.get(name, 0)
            if flags & 8:
                self.seen_table.add(name)
            if self.start_time is None:
                continue
            if flags & 1 and name in self.seen_table:
                self.ever_lifted.add(name)
            if flags & 2:
                self.ever_arrived.add(name)
            if flags & 4:
                self.ever_placed.add(name)
        state["lifted"] = sorted(self.ever_lifted)
        state["placed"] = sorted(self.ever_placed)
        state["base_visits"] = len(self.ever_arrived)
        return state

    def final_score(self, state, end_time):
        lifted = state.get("lifted", [])
        placed = state.get("placed", [])
        visits = state.get("base_visits", 0)
        components = {
            "grasp": self.GRASP_SCORE * len(lifted),
            "base_arrivals": self.BASE_SCORE * min(
                visits, self.BASE_ARRIVALS_NEEDED),
            "place": self.PLACE_SCORE * len(placed),
        }
        all_done = (len(lifted) == len(self.BOXES)
                    and len(placed) == len(self.BOXES)
                    and visits >= self.BASE_ARRIVALS_NEEDED)
        if all_done:
            components["bonus"] = self.BONUS_SCORE
        penalty, over = self.time_penalty(
            end_time, self.BASE_TIME_SEC,
            self.OVER_PENALTY_PER_SEC, self.OVER_PENALTY_CAP)
        components["time_penalty"] = -round(penalty, 3)
        components["over_sec"] = round(over, 3)
        total = (components["grasp"] + components["base_arrivals"]
                 + components["place"] + components.get("bonus", 0)
                 - penalty)
        return max(0.0, total), components


class Task3Scorer(BaseScorer):
    """Three rings: grasp, move to the target table, place, repeat."""

    BODIES = ("task3_hollow_cylinder", "task3_hollow_cylinder_2",
              "task3_hollow_cylinder_3", "base_link")
    RINGS = ("task3_hollow_cylinder", "task3_hollow_cylinder_2",
             "task3_hollow_cylinder_3")

    ARRIVALS_NEEDED = 3

    GRASP_SCORE = 5
    BASE_SCORE = 15
    PLACE_SCORE = 10
    BONUS_SCORE = 10
    BASE_TIME_SEC = 45.0
    OVER_PENALTY_PER_SEC = 1.0
    OVER_PENALTY_CAP = 20.0

    def __init__(self, tap, score_file, **output_options):
        super(Task3Scorer, self).__init__(tap, score_file, **output_options)
        self.ever_lifted = set()
        self.ever_placed = set()
        self.seen_source = set()
        self.ever_arrived = set()

    def _evaluate(self):
        for name in self.RINGS:
            flags = self.tap.task2_state.get(name, 0)
            if flags & 8:
                self.seen_source.add(name)
            if self.start_time is None:
                continue
            if flags & 1 and name in self.seen_source:
                self.ever_lifted.add(name)
            if flags & 2:
                self.ever_arrived.add(name)
            if flags & 4:
                self.ever_placed.add(name)
        return {
            "lifted": sorted(self.ever_lifted),
            "placed": sorted(self.ever_placed),
            "arrived": sorted(self.ever_arrived),
            # Preserve the existing reporting field, now counted by ring.
            "base_visits": len(self.ever_arrived),
        }

    def final_score(self, state, end_time):
        lifted = state.get("lifted", [])
        placed = state.get("placed", [])
        visits = state.get("base_visits", 0)
        components = {
            "grasp": self.GRASP_SCORE * len(lifted),
            "base_arrivals": self.BASE_SCORE * min(
                visits, self.ARRIVALS_NEEDED),
            "place": self.PLACE_SCORE * len(placed),
        }
        all_done = (len(lifted) == len(self.RINGS)
                    and len(placed) == len(self.RINGS)
                    and visits >= self.ARRIVALS_NEEDED)
        if all_done:
            components["bonus"] = self.BONUS_SCORE
        penalty, over = self.time_penalty(
            end_time, self.BASE_TIME_SEC,
            self.OVER_PENALTY_PER_SEC, self.OVER_PENALTY_CAP)
        components["time_penalty"] = -round(penalty, 3)
        components["over_sec"] = round(over, 3)
        total = (components["grasp"] + components["base_arrivals"]
                 + components["place"] + components.get("bonus", 0)
                 - penalty)
        return max(0.0, total), components


SCORERS = {1: Task1Scorer, 2: Task2Scorer, 3: Task3Scorer}


def main():
    parser = argparse.ArgumentParser(description="Run one task scorer")
    parser.add_argument("--task-id", type=int, required=True, choices=(1, 2, 3))
    parser.add_argument("--score-file", default=DEFAULT_SCORE_FILE)
    args = parser.parse_args()

    rospy.init_node(
        "task{}_scorer".format(args.task_id), anonymous=False)
    scorer_class = SCORERS[args.task_id]
    tap = PoseTap(scorer_class.BODIES)
    score_file = os.environ.get("SCORE_FILE", args.score_file)
    scorer = scorer_class(tap, score_file)
    sys.exit(scorer.run())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as error:
        print("Task scorer failed: {}".format(error), flush=True)
        traceback.print_exc()
        sys.exit(1)
