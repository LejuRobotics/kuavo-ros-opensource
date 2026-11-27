#!/usr/bin/env python3
"""Exercise the production SG100 packet gate without starting ROS or hardware."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
SOURCES = (
    ROOT / "scripts" / "ik_ros_velocity.py",
    ROOT / "incremental_velocity" / "scripts" / "ik_ros_velocity.py",
)


class Command:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.vector = SimpleNamespace(x=0.0, y=0.0, z=0.0)


def load_publish(source):
    tree = ast.parse(source.read_text())
    mapper = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "IkRos")
    method = next(n for n in mapper.body if isinstance(n, ast.FunctionDef)
                  and n.name == "_publish_sg100_triggers")
    namespace = dict(math=math, Vector3Stamped=Command,
                     rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda: 123)))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
    return namespace["_publish_sg100_triggers"]


PUBLISHERS = tuple(load_publish(source) for source in SOURCES)


class Gesture:
    def __init__(self):
        self.calls = []

    def update(self, msg, valid):
        self.calls.append(valid)


class Sg100MapperGateTest(unittest.TestCase):
    def fixture(self, left=None, right=None, stop=False, gestures=False):
        transformer = SimpleNamespace(left_finger_joints=left, right_finger_joints=right,
                                      is_hand_tracking=False, is_stop_gesture=Mock(return_value=stop))
        owner = SimpleNamespace(sg100_trigger_pub=Mock(), stop_event=threading.Event(),
                                _absolute_lock=threading.RLock(), _tracking_source_hold_latched=False,
                                quest3_arm_info_transformer=transformer)
        if gestures:
            owner.sg100_gesture_client = Gesture()
        return owner, transformer

    def send(self, owner, left=.25, right=.75, y=False, publish=None):
        publish = publish or PUBLISHERS[0]
        publish(owner, SimpleNamespace(left_trigger=left, right_trigger=right,
                                       left_second_button_pressed=y))
        owner.sg100_trigger_pub.publish.assert_called_once()
        sent = owner.sg100_trigger_pub.publish.call_args[0][0].vector
        owner.sg100_trigger_pub.publish.reset_mock()
        return sent

    def test_first_joystick_before_bones_revokes(self):
        owner, transformer = self.fixture()
        transformer.is_stop_gesture.side_effect = TypeError("uninitialized finger data")
        self.assertEqual(self.send(owner).z, 0)
        transformer.is_stop_gesture.assert_not_called()

    def test_partial_bone_initialization_revokes(self):
        for left, right in [(None, [0]*6), ([0]*6, None)]:
            with self.subTest(left=left, right=right):
                owner, transformer = self.fixture(left, right)
                self.assertEqual(self.send(owner).z, 0)
                transformer.is_stop_gesture.assert_not_called()

    def test_ready_controller_packet_preserves_values(self):
        owner, _ = self.fixture([0]*6, [0]*6)
        out = self.send(owner)
        self.assertEqual((out.x, out.y, out.z), (.25, .75, 1.0))

    def test_explicit_stop_revokes(self):
        owner, _ = self.fixture([0]*6, [0]*6, stop=True)
        self.assertEqual(self.send(owner).z, 0)

    def test_hold_hand_tracking_and_shutdown_revoke(self):
        for reason in ("hold", "tracking", "shutdown"):
            with self.subTest(reason=reason):
                owner, transformer = self.fixture([0]*6, [0]*6)
                if reason == "hold":
                    owner._tracking_source_hold_latched = True
                elif reason == "tracking":
                    transformer.is_hand_tracking = True
                else:
                    owner.stop_event.set()
                self.assertEqual(self.send(owner).z, 0)
                transformer.is_stop_gesture.assert_not_called()

    def test_nonfinite_packet_revokes(self):
        owner, _ = self.fixture([0]*6, [0]*6)
        self.assertEqual(self.send(owner, left=float("nan")).z, 0)

    def test_y_press_locks_triggers_and_blocks_gesture_until_second_press(self):
        for publish in PUBLISHERS:
            with self.subTest(publish=publish.__code__.co_filename):
                owner, _ = self.fixture([0]*6, [0]*6, gestures=True)
                self.assertEqual((self.send(owner, .2, .4, publish=publish).x,
                                  self.send(owner, .2, .4, y=True, publish=publish).x), (.2, .2))
                held = self.send(owner, .9, .1, y=True, publish=publish)
                self.assertEqual((held.x, held.y, held.z), (.2, .4, 1.0))
                self.assertEqual(owner.sg100_gesture_client.calls, [True, False])
                still = self.send(owner, .9, .1, publish=publish)
                self.assertEqual((still.x, still.y), (.2, .4))
                self.assertEqual(owner.sg100_gesture_client.calls, [True, False])
                released = self.send(owner, .9, .1, y=True, publish=publish)
                self.assertEqual((released.x, released.y, released.z), (.9, .1, 1.0))
                self.assertEqual(owner.sg100_gesture_client.calls, [True, False, False])
                live = self.send(owner, .3, .5, publish=publish)
                self.assertEqual((live.x, live.y), (.3, .5))
                self.assertEqual(owner.sg100_gesture_client.calls, [True, False, False, True])

    def test_locked_hand_still_revokes_on_explicit_stop(self):
        owner, _ = self.fixture([0]*6, [0]*6, gestures=True)
        self.send(owner, .2, .4, y=True)
        owner.quest3_arm_info_transformer.is_stop_gesture.return_value = True
        self.assertEqual(self.send(owner, .9, .1, y=True).z, 0)


if __name__ == "__main__":
    unittest.main()
