#!/usr/bin/env python3
"""Frozen test-branch oracle and actual private Mapper packet-to-TCP regression.

The fixture was generated from test 7ea9a70d0, not from this implementation.
No ROS node, controller, or hardware transport is started.
"""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import time
import unittest
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts/tools'))
import pose_velocity_kalman as cv
import lightweight_transforms as transforms


def mapper_class(path, observer=cv):
    tree = ast.parse(Path(path).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Quest3ArmInfoTransformer')
    names = {'read_msg', 'compute_hand_pose', '_compute_unified_tcp_target',
             '_chest_frame_geometry', '_unit_vector', '_active_source_to_tcp',
             'pose_info2_transform'}
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assignments = [n for n in tree.body if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'bone_names' for t in n.targets)]
    env = dict(vars(transforms), math=math, time=time,
               SkeletonCV=observer.SkeletonCV, mapped_rates=observer.mapped_rates,
               _NATIVE=getattr(observer, '_NATIVE', None),
               bias_chest_to_base_link=[0., 0., .42])
    exec(compile(ast.Module(body=assignments+[cls], type_ignores=[]), str(path), 'exec'), env)
    env['bone_name_to_index'] = {v: i for i, v in enumerate(env['bone_names'])}
    return env['Quest3ArmInfoTransformer']


def make_mapper(cls, observer=cv):
    n = cls.__new__(cls)
    n.skeleton_filter_enabled = True
    n.skeleton_filter = observer.SkeletonCV(.001, .63, .003, 5.7, hand_q_r=1.42)
    n.skeleton_contract = (1, 1, 0)
    n.left_joystick = [0., 0.]
    n.init_R_wC = transforms.rpy_to_matrix([math.pi/2, 0., math.pi/2])
    n.upper_arm_length = .22
    n.tcp_forearm_retarget_length = .19
    n.shoulder_width = .15
    tool = np.eye(4); tool[:3, 3] = [.02, -.01, .03]
    tool[:3, :3] = transforms.rpy_to_matrix([.1, -.2, .15])
    n.tcp_source_to_tcp = {k: {s: tool.copy() for s in ('Left','Right')}
                          for k in ('controller','hand_tracking')}
    n.tcp_geometry_mode = 'LEGACY_PALM_6D'
    n.head_body_pose = NS()
    n.vis_pub = False
    n.ok_gesture_counts = n.shot_gesture_counts = 0
    n.is_runing = False
    n.compute_finger_joints = n.compute_finger_joints_joy = lambda side: None
    n.is_runing_gesture = n.is_stop_gesture = lambda: False
    return n


def packets():
    stamp = 1.
    for frame in range(360):
        stamp += (0. if frame in (70, 145) else .25 if frame == 240
                  else (1/72., 1/90., 1/120.)[frame % 3])
        poses = []
        for i in range(26):
            side = 1. if i in (0,1,4) else -1.
            p = np.array([.1+.04*math.sin(.8*frame/90+i), side*.2, 1.])
            if i in (1,3): p += [.12, .01*math.sin(frame/31), -.16]
            if i in (4,5): p += [.30, .03*math.cos(frame/27), -.25]
            if i == 23: p = np.array([.01*math.sin(frame/29), 0., .9])
            R = cv.exp_so3(np.array([.12*math.sin(frame/35+i), .08*math.cos(frame/49), .15*math.sin(frame/67)]))
            q = transforms.matrix_to_quaternion(R)
            if frame % 2: q = -q
            poses.append(NS(position=NS(**dict(zip('xyz',p))), orientation=NS(**dict(zip('xyzw',q)))))
        yield frame, NS(poses=poses, timestamp_ms=stamp*1000,
                        is_high_confidence=frame % 13 != 0, is_hand_tracking=frame >= 180)


def flatten(n):
    result = []
    for i in cv.SkeletonCV.indices:
        T, v, w = n.estimated_bones[i]
        result.extend(np.r_[T.ravel(), v, w])
    for side in ('Left','Right'):
        pose = n.left_tcp_target if side == 'Left' else n.right_tcp_target
        elbow = n.left_tcp_elbow_hint if side == 'Left' else n.right_tcp_elbow_hint
        # Compare rotation matrices, independent of quaternion sign.
        result.extend(np.r_[pose[0], transforms.quaternion_to_matrix(pose[1]).ravel(), elbow,
                            np.concatenate(n.tcp_estimated_rates[side])])
    return result


class SkeletonAlignmentTest(unittest.TestCase):
    def mapper(self):
        return make_mapper(mapper_class(ROOT/'scripts/tools/quest3_utils_velocity.py'))

    def test_packet_to_tcp_matches_frozen_test_branch(self):
        self.assertIsNotNone(cv._NATIVE)  # Exercise the retained production acceleration.
        expected = json.loads((ROOT/'test/data/test_7ea9a70d0_skeleton.json').read_text())
        n = self.mapper()
        for frame, packet in packets():
            if frame == 280: n.skeleton_contract = (1, 2, 0)
            self.assertTrue(n.read_msg(packet, defer_visuals=True))
            if str(frame) in expected['samples']:
                np.testing.assert_allclose(flatten(n), expected['samples'][str(frame)],
                                           atol=2e-7, rtol=2e-7, err_msg='frame %d' % frame)
        self.assertEqual(expected['source_commit'], '7ea9a70d06d8dc744d6a4e3ec427d7b71523a6f0')

    def test_invalid_and_disallowed_frames_do_not_advance_skeleton(self):
        n = self.mapper(); stream = iter(packets())
        _, p = next(stream); self.assertTrue(n.read_msg(p, defer_visuals=True))
        count = n.skeleton_filter.updates
        _, p = next(stream); self.assertTrue(n.read_msg(p, defer_visuals=True, accept_hands=False))
        self.assertEqual(n.skeleton_filter.updates, count)
        self.assertIsNone(n.left_tcp_target)
        _, p = next(stream); p.poses[4].position.x = float('nan')
        self.assertFalse(n.read_msg(p, defer_visuals=True))
        self.assertEqual(n.skeleton_filter.updates, count)
        self.assertEqual(n.ok_gesture_counts, 0)

    def test_root_is_optional_and_optical_does_not_require_joystick(self):
        n = self.mapper(); _, p = next(packets())
        p.poses[22].position.x = float('nan'); p.is_hand_tracking = True
        n.left_joystick = None
        self.assertTrue(n.read_msg(p, defer_visuals=True))
        self.assertIsNotNone(n.left_tcp_target)
        self.assertTrue(np.isfinite(flatten(n)).all())


if __name__ == '__main__': unittest.main()
