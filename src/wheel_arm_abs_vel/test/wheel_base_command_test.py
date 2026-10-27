#!/usr/bin/env python3
"""Offline mapping, lifecycle and legacy incremental equivalence regression."""
import ast
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from tools.wheel_base_command import WheelBaseCommandMapper


def joy(x=0., y=0., w=0., torso=0.):
    return NS(left_x=x, left_y=y, right_x=w, right_y=torso)


class BaseTests(unittest.TestCase):
    def test_mapping_and_stop(self):
        m = WheelBaseCommandMapper({'threshold': .02, 'alpha': .4})
        m.sample(joy(), True, .99)
        self.assertEqual(m.sample(joy(.5, 1., -.5), True, 1.), (.24, -.12, .075))
        self.assertEqual(m.sample(joy(), True, 1.01), (0., 0., 0.))
        self.assertEqual(m.sample(joy(1, 1, 1, .81), True, 1.02), (.24, -.24, -.15))
        self.assertEqual(m.sample(joy(1, 1, 1), False, 1.03), (0., 0., 0.))
        self.assertEqual(m.sample(joy(1, math.nan, 1), True, 1.04), (0., 0., 0.))
        m.sample(joy(y=1), True, 2.)
        self.assertEqual(m.sample(joy(y=1), True, 2.3)[0], 0.)
        self.assertEqual(m.sample(joy(y=1), True, 1.9)[0], 0.)
        for config in ({'alpha': 1.}, {'alpha': math.nan}, {'threshold': -1.}):
            with self.assertRaises(ValueError):
                WheelBaseCommandMapper(config)

    def test_first_connection_and_reconnect_require_real_neutral(self):
        m=WheelBaseCommandMapper({'alpha':0.})
        for now in (1.,1.01,1.02):
            self.assertEqual(m.sample(joy(y=1),True,now),(0.,0.,0.))
            self.assertEqual(m.reason,'WAIT_NEUTRAL')
        m.sample(joy(),True,1.03)
        self.assertEqual(m.sample(joy(y=1,w=1,torso=1),True,1.04),(.4,0.,-.25))
        self.assertEqual(m.sample(joy(y=1),True,1.3),(0.,0.,0.))
        m.sample(joy(),True,1.31)
        self.assertEqual(m.sample(joy(y=1),True,1.32),(.4,0.,0.))

    def test_requested_yaw_limit(self):
        m = WheelBaseCommandMapper({'alpha': 0.}, (.4, .4, .4))
        m.sample(joy(), True, .99)
        self.assertEqual(m.sample(joy(w=1), True, 1.), (0., 0., -.4))
        self.assertEqual(m.sample(joy(w=-1), True, 1.01), (0., 0., .4))

    def test_actual_mapper_lifecycle_method(self):
        tree = ast.parse((ROOT/'scripts/ik_ros_velocity.py').read_text())
        methods = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                   and n.name in ('_publish_wheel_base_command', '_base_input_ready_locked')]
        def twist():
            return NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.))
        scope = {'Twist': twist, 'AbsoluteEnableState': NS(ACTIVE=3, WAITING=0, WAIT_RELEASE=4, PREPARED=8),
                 'time': NS(monotonic=lambda: 1.), 'rospy': NS(loginfo=lambda *a:None)}
        exec(compile(ast.Module(body=methods, type_ignores=[]), '<production>', 'exec'), scope)
        publish = scope['_publish_wheel_base_command']
        sent = []
        state = dict(_absolute_state=3, _fixed_control_enabled=True,
                     _absolute_mode_status=(2, 2, False), _fixed_pending_operation=None,
                     _fixed_operation_error=False, _fixed_operation_block=False,
                     _absolute_stop_pending=False, _tracking_source_hold_latched=False,
                     _absolute_fault_hold_pending=False, _absolute_external_mode_reset_pending=False,
                     _joystick_receive_monotonic=1., _wheel_mpc_obs_receive_monotonic=1., _absolute_latest_measured_q_time=1.,
                     absolute_enable_max_joint_state_age=.1)
        def instance():
            result = NS(**state, _absolute_lock=threading.RLock(), stop_event=threading.Event(),
                      _wheel_mpc_stable_for_mm_cmd=lambda: True,
                      _wheel_base_mapper=WheelBaseCommandMapper({'alpha': 0.}),
                      _wheel_base_pub=NS(publish=sent.append))
            result._base_input_ready_locked = lambda now: scope['_base_input_ready_locked'](result, now)
            result._wheel_base_mapper.sample(joy(),True,.99)
            return result
        s = instance()
        publish(s, joy(y=1))
        self.assertEqual(sent[-1].linear.x, .4)
        count = len(sent)
        publish(s)  # status updates must not keep an old joystick alive
        self.assertEqual(len(sent), count)
        # Initial arm hold needs no hand pose, gesture unlock or ACTIVE session.
        for status in (0, 4):
            s = instance();s._absolute_state=status;s._absolute_mode_status=(1, 1, False)
            s._fixed_operation_block=True;s._absolute_latest_target=None
            publish(s, joy(y=1));self.assertEqual(sent[-1].linear.x, .4)
        # Arm reset/acquisition, source failure and missing body do not inhibit joy.
        for status in (0,1,2,3,4,7,8):
            s=instance();s._absolute_state=status;s._absolute_latest_target=None
            s._tracking_source_hold_latched=True;s._fixed_pending_operation={'mode':1}
            s._absolute_mode_status=(1,2,True)
            publish(s,joy(y=1));self.assertEqual(sent[-1].linear.x,.4)
        for key,value in [('_fixed_control_enabled',False),
                ('_wheel_mpc_obs_receive_monotonic',.7),('_wheel_mpc_obs_receive_monotonic',1.1),
                ('_absolute_latest_measured_q_time',None),('_absolute_latest_measured_q_time',.8),
                ('_joystick_receive_monotonic',.7)]:
            s=instance();setattr(s,key,value);publish(s,joy(y=1))
            self.assertEqual(sent[-1].linear.x,0.,key)
        s=instance();s.stop_event.set();publish(s,joy(y=1));self.assertEqual(sent[-1].linear.x,0.)
        s=instance();s._wheel_mpc_stable_for_mm_cmd=lambda:False
        publish(s,joy(y=1));self.assertEqual(sent[-1].linear.x,0.)
        publish(NS())  # startup before publisher construction

    def test_against_original_incremental_cpp_filter(self):
        source = (ROOT.parent/'manipulation_nodes/motion_capture_ik/src/WheelJoyStickHandler.cpp').read_text()
        begin = source.index('  if (std::abs(rightJoyStickYValue) > 0.8)')
        end = source.index('  const bool sg100_valid', begin)
        block = source[begin:end]
        code = r"""
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
struct Joy { double left_x, left_y, right_x, right_y; };
int main() {
 double leftJoyStickX_=0, leftJoyStickY_=0, rightJoyStickX_=0, rightJoyStickY_=0;
 double joyStickThreshold_=.02, joyStickAlpha_=.4;
 Joy data;
 while(std::cin>>data.left_x>>data.left_y>>data.right_x>>data.right_y) {
 const Joy* msg=&data; double rightJoyStickYValue=data.right_y;
""" + block + r"""
 std::cout<<std::setprecision(17)<<std::clamp(leftJoyStickY_,-1.,1.)*.4<<' '
 <<std::clamp(-leftJoyStickX_,-1.,1.)*.4<<' '
 <<std::clamp(-rightJoyStickX_,-1.,1.)*.25<<'\n';
 }}
"""
        # Compare the unchanged mapping on the legacy non-suppressed domain.
        random.seed(73)
        packets = [joy(*(random.uniform(-1, 1) for _ in range(4))) for _ in range(200)]
        packets += [joy(.02, -.02, .02, .8), joy()]
        packets=[p for p in packets if abs(p.right_y)<=.8]
        with tempfile.TemporaryDirectory() as d:
            cpp, binary = Path(d)/'legacy.cpp', Path(d)/'legacy'
            cpp.write_text(code)
            subprocess.run(['g++', '-std=c++17', str(cpp), '-o', str(binary)], check=True)
            text = ''.join(f'{p.left_x} {p.left_y} {p.right_x} {p.right_y}\n' for p in packets)
            result = subprocess.run([str(binary)], input=text, text=True, capture_output=True, check=True)
        mapper = WheelBaseCommandMapper({'threshold': .02, 'alpha': .4})
        mapper.sample(joy(),True,.99)
        for i, (packet, row) in enumerate(zip(packets, result.stdout.splitlines())):
            expected = tuple(map(float, row.split()))
            actual = mapper.sample(packet, True, 1+i*.01)
            for a, b in zip(actual, expected):
                self.assertAlmostEqual(a, b, places=14)
        self.assertEqual(len(result.stdout.splitlines()), len(packets))


if __name__ == '__main__':
    unittest.main()
