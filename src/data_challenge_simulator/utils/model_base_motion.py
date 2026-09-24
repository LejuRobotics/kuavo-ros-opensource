#!/usr/bin/env python3
"""Model-entry chassis preparation before external control handoff.

The simulator owns only the fixed initialization stage. Task 1 and Task 2 are
ready when that stage returns the base to the table-front pose. Task 3 also
drives to the first seed-derived docking pose. Once ``prepare()`` returns and
``close()`` releases ``/cmd_vel``, every episode-time base command belongs to
the external policy.
"""

import math

import rospy

from task2_base_motion import ChassisMotion, normalize_angle
from utils.task3_randomization import Task3RandomizationPlanner


STOP_SPEED_THRESHOLD = 0.01
STOP_WATCHDOG_S = 10.0


def _make_chassis(yaw_tolerance_deg):
    """Use the accepted Task 3 initialization parameters unchanged."""
    return ChassisMotion(
        linear_speed=0.08,
        angular_speed=0.20,
        minimum_linear_speed=0.06,
        minimum_angular_speed=0.06,
        position_tolerance=0.03,
        yaw_tolerance_deg=float(yaw_tolerance_deg),
    )


def close_chassis(chassis):
    """Stop and unregister a model-initialization ``ChassisMotion``."""
    if chassis is None:
        return
    try:
        chassis.stop()
    finally:
        for endpoint_name in ("_publisher", "_subscriber"):
            endpoint = getattr(chassis, endpoint_name, None)
            if endpoint is not None:
                endpoint.unregister()


class ModelBaseInitializer(object):
    """Prepare one model round, then release every chassis ROS endpoint."""

    def __init__(self, task_id, seed):
        self.task_id = int(task_id)
        self.seed = int(seed)
        self.chassis = None

    def prepare(self):
        """Perform any task-specific base preparation before model handoff."""

    def close(self):
        """Stop and release a chassis endpoint when this task owns one."""
        chassis = self.chassis
        self.chassis = None
        # ChassisMotion is shared with accepted task/collect entries and
        # intentionally has no lifecycle API. This model-only owner must
        # unregister its endpoints before ready so this initializer can never
        # emit an episode-time /cmd_vel command. KuavoSDK itself may keep an
        # idle publisher registered on the shared rospy node.
        close_chassis(chassis)

    def _wait_until_stopped(self):
        # This only observes whether a zero command has physically settled.
        # It never judges whether the requested pose was reached.
        self.chassis.wait_until_stopped(
            STOP_SPEED_THRESHOLD, STOP_WATCHDOG_S)

    def _rotate_open_loop(self, angle_rad):
        plan = self.chassis.rotate_open_loop(float(angle_rad))
        self._wait_until_stopped()
        settled = tuple(self.chassis.pose())
        requested_yaw = normalize_angle(plan["start"][2] + angle_rad)
        yaw_error = normalize_angle(requested_yaw - settled[2])
        rospy.loginfo(
            "Model initialization rotation finished: requested_yaw=%.1f deg "
            "measured_yaw=%.1f deg error=%.1f deg; continuing without an "
            "arrival decision",
            math.degrees(requested_yaw), math.degrees(settled[2]),
            math.degrees(yaw_error))
        return settled

    def _move_open_loop(self, target_pose):
        target_x = float(target_pose[0])
        target_y = float(target_pose[1])
        target_yaw = float(target_pose[2])
        current = tuple(self.chassis.pose())
        self._rotate_open_loop(normalize_angle(target_yaw - current[2]))
        self.chassis.move_open_loop(target_x, target_y)
        self._wait_until_stopped()
        settled = tuple(self.chassis.pose())
        position_error = math.hypot(
            target_x - settled[0], target_y - settled[1])
        yaw_error = normalize_angle(target_yaw - settled[2])
        rospy.loginfo(
            "Model initialization move finished: target=(%.3f, %.3f, %.1f "
            "deg) measured=(%.3f, %.3f, %.1f deg) error=(%.4f m, %.1f "
            "deg); continuing without an arrival decision",
            target_x, target_y, math.degrees(target_yaw),
            settled[0], settled[1], math.degrees(settled[2]),
            position_error, math.degrees(yaw_error))
        return settled


class Task1ModelBaseInitializer(ModelBaseInitializer):
    """Task 1 needs no extra docking after its fixed initialization."""

    def __init__(self, seed):
        super(Task1ModelBaseInitializer, self).__init__(1, seed)


class Task2ModelBaseInitializer(ModelBaseInitializer):
    """Task 2 finishes at the table-front pose from fixed initialization."""

    def __init__(self, seed):
        super(Task2ModelBaseInitializer, self).__init__(2, seed)


class Task3ModelBaseInitializer(ModelBaseInitializer):
    """Finish initialization at the first ring's docking pose."""

    def __init__(self, seed):
        super(Task3ModelBaseInitializer, self).__init__(3, seed)
        self.plan = Task3RandomizationPlanner().plan(seed)
        self.chassis = _make_chassis(yaw_tolerance_deg=3.0)

    def prepare(self):
        first_ring = self.plan.rings[0]
        self.chassis.wait_until_ready(timeout=30.0)
        self._move_open_loop(first_ring.docking_base)
        rospy.loginfo(
            "Task3 model initialization ready at first docking pose for %s",
            first_ring.name)


def make_model_base_initializer(task_id, seed):
    builders = {
        1: Task1ModelBaseInitializer,
        2: Task2ModelBaseInitializer,
        3: Task3ModelBaseInitializer,
    }
    return builders[int(task_id)](seed)
