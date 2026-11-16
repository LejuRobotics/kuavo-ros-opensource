import ast
import math
import unittest
from pathlib import Path

source = Path(__file__).resolve().parents[1]/'scripts/absolute_head_control_node.py'
tree = ast.parse(source.read_text())
cls = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == '_HandActivity')
ns = {'math': math, '_MAX_HAND_SPEED_M_S': 2.0}
exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), ns)
Activity = ns['_HandActivity']
node = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == 'HeadTrackHandNode')
method = next(x for x in node.body if isinstance(x, ast.FunctionDef) and x.name == '_velocity_biased_point')
from types import SimpleNamespace
clock = SimpleNamespace(now=0.)
ns['rospy'] = SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: clock.now)))
exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), ns)
point = ns['_velocity_biased_point']


class ActivityTest(unittest.TestCase):
    def test_weight_moves_right_then_returns_to_midpoint(self):
        owner = SimpleNamespace(velocity_bias=.6, velocity_bias_hold_sec=.8, hand_bias=.5,
            _left_activity=Activity(), _right_activity=Activity(),
            _bias_right_weight=.5, _bias_last_time=None, _bias_last_motion=None, _bias_motion_since=None)
        for i in range(200):
            clock.now=i*.02
            point(owner,[.4,.3,0],[.4,-.3-i*.002,0])
        self.assertGreater(owner._bias_right_weight,.65)
        previous=owner._bias_right_weight
        for i in range(200,800):
            clock.now=i*.02
            point(owner,[.4,.3,0],[.4,-.698,0])
            self.assertLessEqual(abs(owner._bias_right_weight-previous),.03)
            previous=owner._bias_right_weight
        self.assertAlmostEqual(owner._bias_right_weight,.5,places=3)

    def test_slow_right_motion_receives_full_relative_bias(self):
        for rate in (25, 50, 100):
            owner = SimpleNamespace(velocity_bias=.6, velocity_bias_hold_sec=.8, hand_bias=.5,
                _left_activity=Activity(), _right_activity=Activity(),
                _bias_right_weight=.5, _bias_last_time=None, _bias_last_motion=None,
                _bias_motion_since=None)
            for i in range(rate*3):
                clock.now=i/rate
                point(owner,[.4,.3,0],[.4,-.3-i/rate*.02,0])
            self.assertGreater(owner._bias_right_weight,.78)

    def test_noise_and_brief_motion_do_not_shift_target(self):
        owner = SimpleNamespace(velocity_bias=.6, velocity_bias_hold_sec=.8, hand_bias=.5,
            _left_activity=Activity(), _right_activity=Activity(),
            _bias_right_weight=.5, _bias_last_time=None, _bias_last_motion=None,
            _bias_motion_since=None)
        for i in range(150):
            clock.now=i*.02
            point(owner,[.4,.3,0],[.4,-.3+(-1)**i*.00001,0])
        self.assertEqual(owner._bias_right_weight,.5)
        for i in range(10):
            clock.now=3.+i*.02
            point(owner,[.4,.3,0],[.4,-.3-i*.002,0])
        self.assertEqual(owner._bias_right_weight,.5)

    def test_configured_baseline_and_fixed_endpoints(self):
        for baseline in (0., .25, .5, .75, 1.):
            owner = SimpleNamespace(velocity_bias=.6, velocity_bias_hold_sec=.8,
                hand_bias=baseline, _left_activity=Activity(), _right_activity=Activity(),
                _bias_right_weight=baseline, _bias_last_time=None, _bias_last_motion=None,
                _bias_motion_since=None)
            clock.now=0.
            self.assertAlmostEqual(point(owner,[0,0,0],[1,0,0])[0],baseline)
            for i in range(1,150):
                clock.now=i*.02
                result=point(owner,[0,0,0],[1+i*.002,0,0])
            if baseline in (0.,1.):
                self.assertEqual(result, [0,0,0] if baseline==0 else [1+149*.002,0,0])
            else:
                self.assertGreater(owner._bias_right_weight,baseline)
                self.assertLessEqual(owner._bias_right_weight,1.)

    def test_rate_independence(self):
        values = []
        for rate in (25, 50, 100):
            a = Activity()
            for i in range(rate*5+1):
                a.update([i/rate*.1, 0, 0], i/rate, .8)
            values.append(a.activity)
        self.assertLess(max(values)-min(values), .001)

    def test_stationary_noise(self):
        a = Activity()
        for i in range(500):
            a.update([(-1)**i*.00001, 0, 0], i*.02, .8)
        self.assertEqual(a.activity, 0)

    def test_jump_rejected(self):
        a = Activity()
        a.update([0, 0, 0], 0, .8)
        self.assertEqual(a.update([10, 0, 0], .02, .8), 0)

    def test_gap_and_duplicate_timestamp(self):
        a = Activity()
        a.update([0, 0, 0], 0, .8)
        self.assertEqual(a.update([1, 0, 0], 0, .8), 0)
        self.assertEqual(a.update([1, 0, 0], 1, .8), 0)


if __name__ == '__main__':
    unittest.main()
