#!/usr/bin/env python3
"""Bridge the shared SG100 model action to BLACK_MAN_11 MuJoCo topics."""

import os
import sys

import rospy
from kuavo_msgs.msg import SG100HandCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)

from utils.hand_postures import joint_names


class BlackManSg100Bridge:
    def __init__(self):
        self.task_id = int(rospy.get_param("~task_id"))
        if self.task_id not in (1, 2, 3):
            raise ValueError("~task_id must be 1, 2, or 3")
        self.names = {
            "left": tuple(joint_names("l")),
            "right": tuple(joint_names("r")),
        }
        self.commands_enabled = False

        self.command_publishers = {
            "left": rospy.Publisher(
                "/cb_left_hand_control_cmd", JointState, queue_size=10),
            "right": rospy.Publisher(
                "/cb_right_hand_control_cmd", JointState, queue_size=10),
        }
        self.command_subscriber = rospy.Subscriber(
            "/sg100_hand_command", SG100HandCommand, self._command_callback,
            queue_size=1, tcp_nodelay=True)
        self.command_gate_subscriber = rospy.Subscriber(
            "/model_simulator/accept_commands", Bool,
            self._command_gate_callback, queue_size=1)
        rospy.loginfo(
            "BLACK_MAN_11 model bridge ready for Task %d: "
            "/sg100_hand_command -> /cb_*_hand_control_cmd", self.task_id)

    def _command_gate_callback(self, message):
        self.commands_enabled = bool(message.data)
        rospy.loginfo(
            "BLACK_MAN_11 external command gate: %s",
            "open" if self.commands_enabled else "closed")

    @staticmethod
    def _is_supported(message):
        if message.control_mode != SG100HandCommand.MODE_JOINT_POSITION:
            return False
        if message.left_enable_mask not in (0, 0x07FF):
            return False
        if message.right_enable_mask not in (0, 0x07FF):
            return False
        for modes in (
                message.left_hand_control_mode,
                message.right_hand_control_mode):
            if modes and (
                    len(modes) != 11
                    or any(mode != SG100HandCommand.MODE_JOINT_POSITION
                           for mode in modes)):
                return False
        return True

    def _command_callback(self, message):
        if not self.commands_enabled:
            rospy.logwarn_throttle(
                2.0, "Ignoring /sg100_hand_command while model "
                "initialization is incomplete")
            return
        if not self._is_supported(message):
            rospy.logwarn_throttle(
                2.0, "Ignoring SG100 command that is not full-hand joint "
                "position control")
            return
        values = (
            message.left_hand_positions,
            message.right_hand_positions,
        )
        if any(len(positions) != 11 for positions in values):
            rospy.logwarn_throttle(
                2.0, "/sg100_hand_command requires 11 positions per hand")
            return
        stamp = rospy.Time.now()
        for side, positions in zip(("left", "right"), values):
            output = JointState()
            output.header.stamp = stamp
            output.name = list(self.names[side])
            output.position = [float(value) for value in positions]
            self.command_publishers[side].publish(output)


def main():
    rospy.init_node("blackman_gripper_compat")
    BlackManSg100Bridge()
    rospy.spin()


if __name__ == "__main__":
    main()
