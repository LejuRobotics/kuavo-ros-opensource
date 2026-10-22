#!/usr/bin/env python3
"""Exercise the production SG100 packet gate without starting ROS or hardware."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock

SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "ik_ros_velocity.py"
tree = ast.parse(SOURCE.read_text())
mapper = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "IkRos")
method = next(n for n in mapper.body if isinstance(n, ast.FunctionDef) and n.name == "_publish_sg100_triggers")


class Command:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.vector = SimpleNamespace(x=0.0, y=0.0, z=0.0)


namespace = dict(math=math, Vector3Stamped=Command,
                 rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda: 123)))
exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
publish = namespace["_publish_sg100_triggers"]


class Sg100MapperGateTest(unittest.TestCase):
    def fixture(self, left=None, right=None, stop=False):
        transformer = SimpleNamespace(left_finger_joints=left, right_finger_joints=right,
                                      is_hand_tracking=False, is_stop_gesture=Mock(return_value=stop))
        owner = SimpleNamespace(sg100_trigger_pub=Mock(), stop_event=threading.Event(),
                                _absolute_lock=threading.RLock(), _tracking_source_hold_latched=False,
                                quest3_arm_info_transformer=transformer)
        return owner, transformer

    def send(self, owner, left=.25, right=.75):
        publish(owner, SimpleNamespace(left_trigger=left, right_trigger=right))
        owner.sg100_trigger_pub.publish.assert_called_once()
        return owner.sg100_trigger_pub.publish.call_args[0][0].vector

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


if __name__ == "__main__":
    unittest.main()
