"""Startup guard for the first real wheel-arm state sample."""

import math

import rospy
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import sensorsData


SENSOR_TOPIC = "/sensors_data_raw"
ARM_START_INDEX = 4
ARM_JOINT_COUNT = 14


def wait_for_first_arm_state(timeout=30.0):
    """Return arm joints from a sensor message received by this process.

    ``KuavoRobotState`` initially contains a correctly-sized all-zero cache.
    Its length therefore cannot tell callers whether the subscriber callback
    has run.  Waiting for the ROS message itself keeps the arm command
    publisher closed until a real simulator sample is available.
    """
    try:
        message = rospy.wait_for_message(
            SENSOR_TOPIC, sensorsData, timeout=float(timeout))
    except rospy.ROSException as error:
        raise RuntimeError(
            "timed out waiting for the first real arm state") from error

    joints = list(message.joint_data.joint_q)
    required = ARM_START_INDEX + ARM_JOINT_COUNT
    if len(joints) < required:
        raise RuntimeError(
            "{} has {} joints; expected at least {}".format(
                SENSOR_TOPIC, len(joints), required))

    arm = [float(value) for value in joints[
        ARM_START_INDEX:ARM_START_INDEX + ARM_JOINT_COUNT]]
    if not all(math.isfinite(value) for value in arm):
        raise RuntimeError("received a non-finite arm state")
    return arm
