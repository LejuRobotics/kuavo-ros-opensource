#!/usr/bin/env python3
"""Apply glove gesture buttons only to fresh Quest controller samples."""

import json
import math
import sys
import threading
import time
from pathlib import Path


BUTTONS = ("left_first", "left_second", "right_first", "right_second")
MODE_TOGGLE = 0x10
FIST = 0x20
STOP_CHORD = 0x03
RIGHT_ACTIONS = 0x0c
STOP_HOLD_SECONDS = 0.2


class GloveQuestInput:
    def __init__(self, timeout, interaction_changed):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("glove gesture timeout must be finite and positive")
        self.timeout = timeout
        self.interaction_changed = interaction_changed
        self.lock = threading.Lock()
        self.last_buttons = None
        self.buttons = 0
        self.interactive = False
        self.following = False
        self.started = False
        self.fist_until = 0.0
        self.stop_chord_since = None

    def _set_interaction(self, enabled):
        if self.interactive != enabled:
            self.interactive = enabled
            self.stop_chord_since = None
            self.interaction_changed(enabled)
        if not enabled:
            self.fist_until = 0.0
            self.stop_chord_since = None

    def expire(self, now):
        with self.lock:
            self._expire_locked(now)

    def _expire_locked(self, now):
        if self.last_buttons is not None and now - self.last_buttons > self.timeout:
            self.buttons = 0
            self._set_interaction(False)

    def on_buttons(self, buttons, now):
        with self.lock:
            self._expire_locked(now)
            rising = buttons & ~self.buttons
            self.last_buttons = now
            self.buttons = buttons
            if rising & MODE_TOGGLE:
                self._set_interaction(not self.interactive)
            if self.interactive and (buttons & 0x0f) == STOP_CHORD:
                if self.stop_chord_since is None:
                    self.stop_chord_since = now
            else:
                # A/B appearing during X+Y identifies an action chord. Releasing
                # A/B must begin a fresh stop hold instead of inheriting time.
                self.stop_chord_since = None
            if rising & FIST and self.interactive and not self.started:
                # The recognition event may fall between Quest samples.
                # Bound its lifetime and consume it only with live Quest input.
                self.fist_until = now + min(self.timeout, 0.15)

    def on_following(self, following):
        with self.lock:
            if following and not self.following:
                self.started = True
                self._set_interaction(False)
            self.following = following

    def apply(self, joy, now):
        with self.lock:
            self._expire_locked(now)
            if self.interactive:
                buttons = self.buttons
                # X+Y is an intentional glove stop only after a stable hold.
                # Nearby fingertip overlap with A/B remains an action chord.
                if (buttons & STOP_CHORD) == STOP_CHORD:
                    if (buttons & RIGHT_ACTIONS or joy.right_first_button_pressed or
                            joy.right_second_button_pressed):
                        buttons &= ~0x02
                        self.stop_chord_since = None
                    elif (self.stop_chord_since is None or
                          now - self.stop_chord_since < STOP_HOLD_SECONDS):
                        buttons &= ~STOP_CHORD

                # Synthetic and physical single buttons cannot provide one half
                # each of the stop chord. A fully physical X+Y remains immediate.
                physical_x = joy.left_first_button_pressed
                physical_y = joy.left_second_button_pressed
                if physical_x and not physical_y:
                    buttons &= ~0x02
                elif physical_y and not physical_x:
                    buttons &= ~0x01
                for bit, field in enumerate(BUTTONS):
                    if buttons & (1 << bit):
                        setattr(joy, field + "_button_pressed", True)
                        setattr(joy, field + "_button_touched", True)
                if now < self.fist_until:
                    joy.left_trigger = joy.right_trigger = 1.0
        return joy


def connect(timeout):
    import rospy
    from std_msgs.msg import Bool, UInt8

    publisher = rospy.Publisher("/glove_gesture/fsm_interaction_mode", Bool,
                                queue_size=1, latch=True)
    publisher.publish(False)
    state = GloveQuestInput(timeout, publisher.publish)
    state.subscribers = (
        rospy.Subscriber("/glove_gesture/buttons", UInt8,
                         lambda msg: state.on_buttons(msg.data, time.monotonic()),
                         queue_size=1, tcp_nodelay=True),
        rospy.Subscriber("/glove_gesture/teleop_active", Bool,
                         lambda msg: state.on_following(msg.data), queue_size=1),
    )
    state.timer = rospy.Timer(rospy.Duration(min(0.1, timeout)),
                              lambda _: state.expire(time.monotonic()))
    return state


if __name__ == "__main__":
    # roslaunch executes this check while loading the launch, before spawning nodes.
    if len(sys.argv) != 3 or sys.argv[1] != "--check-config":
        raise SystemExit("usage: glove_quest_input.py --check-config KUAVO_JSON")
    path = Path(sys.argv[2])
    if json.loads(path.read_text()).get("EndEffectorType") != ["heiman", "heiman"]:
        raise SystemExit("qianshi requires SG100 on both hands in " + str(path))
    print("heiman")
