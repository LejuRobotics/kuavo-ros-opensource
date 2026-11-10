#!/usr/bin/env python3
"""Real Mapper numerical-error/recovery regression, offline and without ROS I/O.

Pass the same --params and one --bag used by verify_mapper_replay.py.
--source optionally selects an older quest3_utils.py for regression reproduction.
"""
import argparse
import copy
import importlib.util
from pathlib import Path
import sys
import unittest
import json
import numpy as np
import rosbag
import rospy

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--params', required=True)
parser.add_argument('--bag', required=True)
parser.add_argument('--source')
args, unittest_args = parser.parse_known_args()
parameters = json.loads(Path(args.params).read_text())
def get_param(name, *default):
    if name in ('~arm_marker_async_enabled', '~noncontrol_publish_async'):
        return False
    keys = ('ik_ros_uni/' + name[1:] if name.startswith('~') else name).strip('/').split('/')
    value = parameters
    try:
        for key in keys:
            value = value[key]
        return value
    except (KeyError, TypeError):
        if default:
            return default[0]
        raise KeyError(name)
def has_param(name):
    try:
        get_param(name)
        return True
    except KeyError:
        return False
class NullPublisher:
    def __init__(self, *a, **kw): pass
    def publish(self, *a, **kw): pass
    def get_num_connections(self): return 0
    def unregister(self): pass
rospy.Publisher = rospy.Subscriber = NullPublisher
rospy.get_param, rospy.has_param = get_param, has_param
rospy.set_param = lambda *a, **kw: None
for name in ('loginfo', 'logwarn', 'logerr', 'logdebug', 'logwarn_throttle', 'logerr_throttle'):
    setattr(rospy, name, lambda *a, **kw: None)
clock = [10.]
rospy.Time.now = staticmethod(lambda: rospy.Time.from_sec(clock[0]))
rospy.get_time = lambda: clock[0]
scripts = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
sys.path.insert(0, str(scripts / 'tools'))
import legacy_dependencies
source = Path(args.source) if args.source else scripts / 'tools/quest3_utils_velocity.py'
spec = importlib.util.spec_from_file_location('tools.quest3_utils_velocity', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Mapper = module.Quest3ArmInfoTransformer

joy = frame = None
with rosbag.Bag(args.bag) as bag:
    for topic, message, stamp in bag.read_messages(topics=['/quest_joystick_data', '/leju_quest_bone_poses']):
        if topic == '/quest_joystick_data':
            joy = message
        elif joy is not None and len(message.poses) >= 24:
            frame = message
            break
assert frame is not None, 'bag must contain joystick and complete skeleton data'

def mapper():
    obj = Mapper('', vis_pub=False, predict_gesture=False)
    obj.read_joySticks_msg(joy)
    return obj

def zero_frame(side=None):
    msg = copy.deepcopy(frame)
    # Finite, valid quaternions: the error must specifically be arm scaling.
    msg.is_hand_tracking = False
    msg.is_high_confidence = False
    for i in (range(24) if side is None else ((0, 1) if side == 'Left' else (2, 3))):
        p = msg.poses[i]
        p.position.x = p.position.y = p.position.z = 0.
        p.orientation.x = p.orientation.y = p.orientation.z = 0.
        p.orientation.w = 1.
    if side is not None:
        # Make projected shoulder and elbow coincide with the chest origin.
        for i in ((0, 1) if side == 'Left' else (2, 3)):
            msg.poses[i].position = copy.deepcopy(msg.poses[23].position)
    return msg

class NumericalFrameTest(unittest.TestCase):
    def test_zero_frame_then_valid_recovers_without_exception(self):
        for side in (None, 'Left', 'Right'):
            with self.subTest(side=side):
                obj = mapper()
                self.assertTrue(obj.read_msg(frame, defer_visuals=False))
                self.assertFalse(obj.read_msg(zero_frame(side), defer_visuals=False))
                self.assertIsNone(obj.left_tcp_target)
                self.assertIsNone(obj.right_tcp_target)
                self.assertEqual(obj._pending_arm_markers, [])
                recovered = copy.deepcopy(frame)
                recovered.timestamp_ms += 20
                self.assertTrue(obj.read_msg(recovered, defer_visuals=False))
                for target in (obj.left_tcp_target, obj.right_tcp_target):
                    self.assertIsNotNone(target)
                    self.assertTrue(all(np.isfinite(value).all() for value in target))

    def test_low_confidence_does_not_change_valid_mapping(self):
        high, low = copy.deepcopy(frame), copy.deepcopy(frame)
        high.is_high_confidence, low.is_high_confidence = True, False
        a, b = mapper(), mapper()
        self.assertTrue(a.read_msg(high, defer_visuals=False))
        self.assertTrue(b.read_msg(low, defer_visuals=False))
        for name in ('left_tcp_target', 'right_tcp_target', 'left_hand_pose', 'right_hand_pose'):
            for x, y in zip(getattr(a, name), getattr(b, name)):
                np.testing.assert_array_equal(x, y)

    def test_guard_checks_actual_divisor_before_measurement_state(self):
        obj = mapper()
        origin, elbow, hand = np.zeros(3), np.array([0.2, 0.1, 0.]), np.array([0.4, 0.1, 0.])
        for measuring in (False, True):
            obj.measure_arm_length = measuring
            before = list(obj.left_upper_arm_lengths)
            with self.assertRaises(module.UndefinedArmScale):
                obj.scale_arm_positions(origin, origin, hand, origin, None, 'Left', 0.)
            self.assertEqual(before, obj.left_upper_arm_lengths)
        obj.measure_arm_length = False
        with self.assertRaises(module.UndefinedArmScale):
            obj.scale_arm_positions(origin, elbow, elbow, origin, None, 'Left', 0.)
        # A small but nonzero segment is defined, not a confidence/length cutoff.
        result = obj.scale_arm_positions(origin, elbow * 1e-6, hand * 1e-6,
                                         origin, None, 'Left', 0.)
        self.assertTrue(np.isfinite(result).all())
        # Preserve measurement mode's original combined-length denominator.
        obj.measure_arm_length = True
        result = obj.scale_arm_positions(origin, elbow, elbow, origin, None, 'Left', 0.)
        self.assertTrue(np.isfinite(result).all())

if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]] + unittest_args)
