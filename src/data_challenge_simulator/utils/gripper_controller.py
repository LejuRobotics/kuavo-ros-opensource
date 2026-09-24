"""Upstream gripper API adapted to the standard SG100 ROS interface.

The task scripts keep the upstream 0..255 calls.  This adapter converts that
single closure command into calibrated 11-joint postures and publishes one
dual-hand ``SG100HandCommand``.  Existing methods
retain the task1 thumb/index posture; double-gripper methods preserve the older
two-pair posture, dedicated box-gripper methods expose the Task 2 whole-box
posture, and lever-hook methods expose the fixed inverted-L posture.
"""

import threading
import time

import rospy
from kuavo_msgs.msg import SG100HandCommand, SG100HandState

from utils.hand_postures import (
    box_gripper_targets,
    double_gripper_targets,
    internal_independent_expansion_targets,
    internal_expansion_targets,
    joint_names,
    lever_hook_targets,
    single_gripper_targets,
)


def _joint_names(side):
    return joint_names(side)


def _single_gripper_targets(side, closure):
    """Return the calibrated thumb/index gripper posture for one hand."""
    return single_gripper_targets(side, closure)


class GripperController:
    """Keep the upstream controller surface while driving SG100 directly."""

    def __init__(self, publish_frequency=100.0, motion_duration=1.2):
        self.publish_frequency = float(publish_frequency)
        self.motion_duration = float(motion_duration)
        self.current_left_cmd = 0.0
        self.current_right_cmd = 0.0
        self.current_left_positions = None
        self.current_right_positions = None
        self.command_lock = threading.Lock()
        self.command_initialized = threading.Event()
        self.state_lock = threading.Lock()
        self.left_state = None
        self.right_state = None
        self.is_running = True

        self.left_names = _joint_names("l")
        self.right_names = _joint_names("r")
        self.command_pub = rospy.Publisher(
            "/sg100_hand_command", SG100HandCommand, queue_size=10)
        self.state_sub = rospy.Subscriber(
            "/sg100_hand_state", SG100HandState, self._state_callback)

        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        rospy.loginfo(
            "SG100 gripper controller initialized at %.1f Hz",
            self.publish_frequency)

    def _state_callback(self, message):
        left = list(message.left_hand_positions)
        right = list(message.right_hand_positions)
        if len(left) != 11 or len(right) != 11:
            rospy.logwarn_throttle(
                2.0, "SG100 state requires 11 positions per hand")
            return
        with self.state_lock:
            self.left_state = left
            self.right_state = right

        # Start the command stream from the first measured posture instead of
        # waiting for the first scripted hand motion.  This keeps rosbag
        # command topics continuous without pulling either hand toward a
        # fabricated all-zero target.  A real command wins if _move() has
        # already initialized this side.
        with self.command_lock:
            if self.current_left_positions is None:
                self.current_left_positions = left
            if self.current_right_positions is None:
                self.current_right_positions = right
            self.command_initialized.set()

    @staticmethod
    def _clamp_command(command):
        return max(0.0, min(255.0, float(command)))

    @staticmethod
    def _message(left, right):
        message = SG100HandCommand()
        message.header.stamp = rospy.Time.now()
        message.control_mode = SG100HandCommand.MODE_JOINT_POSITION
        message.left_enable_mask = 0x07FF
        message.right_enable_mask = 0x07FF
        message.left_hand_positions = list(left)
        message.right_hand_positions = list(right)
        return message

    def _publish_loop(self):
        rate = rospy.Rate(self.publish_frequency)
        while not rospy.is_shutdown() and self.is_running:
            if not self.command_initialized.wait(timeout=0.1):
                continue
            with self.command_lock:
                left = (
                    None if self.current_left_positions is None
                    else list(self.current_left_positions))
                right = (
                    None if self.current_right_positions is None
                    else list(self.current_right_positions))
            if left is not None and right is not None:
                self.command_pub.publish(self._message(left, right))
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _measured_positions(self, side):
        with self.state_lock:
            state = self.left_state if side == "left" else self.right_state
            if state is None:
                return None
            return list(state)

    def measured_positions(self, side):
        """Return the latest ordered measured hand joints, or ``None``."""
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        return self._measured_positions(side)

    def _wait_for_measured_positions(self, sides, timeout=2.0):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            positions = {
                side: self._measured_positions(side) for side in sides}
            if all(value is not None for value in positions.values()):
                return positions
            time.sleep(0.02)
        raise RuntimeError(
            "Timed out waiting for SG100 state before gripper motion")

    def _move(
            self, commands, duration=None,
            target_factory=single_gripper_targets):
        """Interpolate joint commands from measured posture to targets."""
        duration = self.motion_duration if duration is None else float(duration)
        if duration <= 0.0:
            raise ValueError("Gripper motion duration must be positive")

        clamped = {
            side: self._clamp_command(command)
            for side, command in commands.items()}
        starts = self._wait_for_measured_positions(tuple(clamped))
        targets = {
            side: target_factory(
                side[0], command / 255.0)
            for side, command in clamped.items()}
        steps = max(1, int(round(duration * self.publish_frequency)))

        for step in range(1, steps + 1):
            alpha = float(step) / steps
            with self.command_lock:
                for side in clamped:
                    positions = [
                        start + alpha * (target - start)
                        for start, target in zip(starts[side], targets[side])]
                    if side == "left":
                        self.current_left_cmd = clamped[side]
                        self.current_left_positions = positions
                    else:
                        self.current_right_cmd = clamped[side]
                        self.current_right_positions = positions
                self.command_initialized.set()
            time.sleep(1.0 / self.publish_frequency)

    def set_gripper_position(self, left_cmd, right_cmd, duration=None):
        self._move(
            {"left": left_cmd, "right": right_cmd}, duration=duration)

    def control_right_gripper(self, right_cmd, duration=None):
        self._move({"right": right_cmd}, duration=duration)

    def control_left_gripper(self, left_cmd, duration=None):
        self._move({"left": left_cmd}, duration=duration)

    def set_double_gripper_position(
            self, left_cmd, right_cmd, duration=None):
        """Move both hands on the canonical double-gripper path."""
        self._move(
            {"left": left_cmd, "right": right_cmd},
            duration=duration,
            target_factory=double_gripper_targets,
        )

    def control_right_double_gripper(self, right_cmd, duration=None):
        self._move(
            {"right": right_cmd}, duration=duration,
            target_factory=double_gripper_targets)

    def control_left_double_gripper(self, left_cmd, duration=None):
        self._move(
            {"left": left_cmd}, duration=duration,
            target_factory=double_gripper_targets)

    def set_box_gripper_position(
            self, left_cmd, right_cmd, duration=None):
        """Move both hands on the dedicated whole-box gripping path."""
        self._move(
            {"left": left_cmd, "right": right_cmd},
            duration=duration,
            target_factory=box_gripper_targets,
        )

    def control_right_box_gripper(self, right_cmd, duration=None):
        self._move(
            {"right": right_cmd}, duration=duration,
            target_factory=box_gripper_targets)

    def control_left_box_gripper(self, left_cmd, duration=None):
        self._move(
            {"left": left_cmd}, duration=duration,
            target_factory=box_gripper_targets)

    def control_right_internal_expansion(self, expansion_cmd, duration=None):
        """Drive the right index/middle/little fingers outward inside a rim."""
        self._move(
            {"right": expansion_cmd}, duration=duration,
            target_factory=internal_expansion_targets)

    def command_right_internal_fingers(
            self, index_expansion, middle_expansion, little_expansion,
            little_j3_compact=0.0, little_j3_expanded=0.0):
        """Publish one non-blocking Task 3 target for independent fingers.

        The caller owns pacing and latch-mask handling.  This deliberately
        bypasses ``_move`` so a searching finger can advance one small step
        without interpolating a latched finger toward a new shared endpoint.
        """
        positions = internal_independent_expansion_targets(
            "r", index_expansion, middle_expansion, little_expansion,
            little_j3_compact=little_j3_compact,
            little_j3_expanded=little_j3_expanded)
        with self.command_lock:
            self.current_right_positions = list(positions)
            self.command_initialized.set()
        return tuple(positions)

    def control_left_internal_expansion(self, expansion_cmd, duration=None):
        """Drive the left index/middle/little fingers outward inside a rim."""
        self._move(
            {"left": expansion_cmd}, duration=duration,
            target_factory=internal_expansion_targets)

    @staticmethod
    def _lever_hook_target_factory(side, _unused_closure):
        return lever_hook_targets(side)

    def control_left_lever_hook(self, duration=None):
        """Shape the left hand into the fixed inverted-L lever hook."""
        self._move(
            {"left": 0.0}, duration=duration,
            target_factory=self._lever_hook_target_factory)

    def control_right_lever_hook(self, duration=None):
        """Shape the right hand into the mirrored fixed lever hook."""
        self._move(
            {"right": 0.0}, duration=duration,
            target_factory=self._lever_hook_target_factory)

    def open_grippers(self):
        self.set_gripper_position(0.0, 0.0)

    def close_grippers(self):
        self.set_gripper_position(255.0, 255.0)

    def wait_for_command(
            self, side, command, tolerance=0.10, timeout=3.0,
            target_factory=single_gripper_targets):
        """Wait until one hand reaches the requested open/close posture."""
        target = target_factory(
            side[0], self._clamp_command(command) / 255.0)
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.state_lock:
                state = self.left_state if side == "left" else self.right_state
                if state is not None:
                    error = max(
                        abs(actual - desired)
                        for actual, desired in zip(state, target))
                    if error <= tolerance:
                        return True
            time.sleep(0.02)
        return False

    def wait_for_double_command(
            self, side, command, tolerance=0.10, timeout=3.0):
        """Wait until one hand reaches its requested double-gripper pose."""
        return self.wait_for_command(
            side, command, tolerance=tolerance, timeout=timeout,
            target_factory=double_gripper_targets)

    def wait_for_box_command(
            self, side, command, tolerance=0.10, timeout=3.0):
        """Wait until one hand reaches its requested whole-box pose."""
        return self.wait_for_command(
            side, command, tolerance=tolerance, timeout=timeout,
            target_factory=box_gripper_targets)

    def wait_for_internal_expansion(
            self, side, command, tolerance=0.10, timeout=3.0):
        """Wait until one hand reaches its internal-expansion target."""
        return self.wait_for_command(
            side, command, tolerance=tolerance, timeout=timeout,
            target_factory=internal_expansion_targets)

    def wait_for_open(self, timeout=3.0):
        return (
            self.wait_for_command("left", 0.0, timeout=timeout)
            and self.wait_for_command("right", 0.0, timeout=timeout)
        )

    def get_current_commands(self):
        with self.command_lock:
            return self.current_left_cmd, self.current_right_cmd

    def stop(self):
        self.is_running = False
        if self.publish_thread.is_alive():
            self.publish_thread.join(timeout=2.0)
