#!/usr/bin/env python3
"""Start the optional task scorer clock at the task-action boundary."""

import os

import rospy
from std_srvs.srv import Trigger


START_SERVICE = "/task_scorer/start"
RECORDING_SERVICE = "/data_collector/start_recording"


def start_recording_after_initialization():
    """Open the collect-only recording gate before task action begins."""
    if os.environ.get("TASK_RECORD_AFTER_INIT") != "1":
        return False
    try:
        rospy.wait_for_service(RECORDING_SERVICE, timeout=5.0)
        response = rospy.ServiceProxy(RECORDING_SERVICE, Trigger)()
    except (rospy.ROSException, rospy.ServiceException) as error:
        raise RuntimeError(
            "failed to start post-initialization rosbag: {}".format(error))
    if not response.success:
        raise RuntimeError(
            "failed to start post-initialization rosbag: {}".format(
                response.message))
    return True


def start_score_clock():
    """Start timing when helperfunc launched a scorer for this round."""
    start_recording_after_initialization()
    if os.environ.get("TASK_SCORER_ENABLED") != "1":
        return False
    try:
        rospy.wait_for_service(START_SERVICE, timeout=5.0)
        response = rospy.ServiceProxy(START_SERVICE, Trigger)()
        if not response.success:
            rospy.logwarn(
                "Failed to start score clock: %s", response.message)
        return bool(response.success)
    except (rospy.ROSException, rospy.ServiceException) as error:
        rospy.logwarn("Failed to start score clock: %s", error)
        return False
