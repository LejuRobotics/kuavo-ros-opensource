#!/usr/bin/env python3
"""Exercise production button/gesture branches without ROS or actuators."""
import ast
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]

class ControlsTest(unittest.TestCase):
    def test_formal_trigger_gesture_cannot_bypass_one_shot_and_stop_remains(self):
        tree = ast.parse((ROOT/'scripts/tools/quest3_utils_velocity.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Quest3ArmInfoTransformer')
        names = {'is_runing_gesture', 'is_stop_gesture', 'ok_gesture_check',
                 'shot_gesture_check', 'joy_ok_gesture_check', 'joy_shot_gesture_check'}
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        definition = ast.ClassDef(name='Transformer', bases=[], keywords=[], decorator_list=[], body=methods)
        scope = {}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[definition], type_ignores=[])), '<production>', 'exec'), scope)
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        flag = next((n for n in init.body if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Attribute)
                    and n.targets[0].attr == 'controller_trigger_start_enabled'), None)
        counter = next(n for n in ast.walk(cls) if isinstance(n, ast.If)
                       and isinstance(n.test, ast.Call) and isinstance(n.test.func, ast.Attribute)
                       and n.test.func.attr == 'is_runing_gesture')
        mapper_tree = ast.parse((ROOT/'scripts/ik_ros_velocity.py').read_text())
        formal_gate = next(n for n in ast.walk(mapper_tree) if isinstance(n, ast.If)
                          and isinstance(n.test, ast.Attribute)
                          and n.test.attr == 'enable_velocity_ik_takeover'
                          and any(isinstance(a, ast.Assign)
                                  and isinstance(a.targets[0], ast.Attribute)
                                  and a.targets[0].attr == 'controller_trigger_start_enabled'
                                  for a in n.body))
        for formal in (True, False):
            t = scope['Transformer']()
            t.is_hand_tracking = False
            t.measure_arm_length = True
            t.left_upper_arm_lengths = t.right_upper_arm_lengths = [.3]
            t.left_lower_arm_lengths = t.right_lower_arm_lengths = [.25]
            t.left_finger_joints = t.right_finger_joints = [0.]*6
            t.left_joystick = t.right_joystick = [1., 0.]
            t.ok_gesture_counts = t.shot_gesture_counts = 0
            t.is_runing = False
            env = {'self': t, 'max_counts': 50, 'bool': bool,
                   'rospy': NS(get_param=lambda *args: formal)}
            t.controller_trigger_start_enabled = True  # original gesture-only startup
            if flag is not None:
                exec(compile(ast.Module(body=[flag], type_ignores=[]), '<production>', 'exec'), env)
            exec(compile(ast.Module(body=[formal_gate], type_ignores=[]), '<mapper-init>', 'exec'),
                 {'self': NS(enable_velocity_ik_takeover=formal, quest3_arm_info_transformer=t)})
            step = compile(ast.Module(body=[counter], type_ignores=[]), '<production>', 'exec')
            for _ in range(60): exec(step, env)
            self.assertEqual(t.is_runing, not formal)
            self.assertFalse(t.measure_arm_length)
            self.assertEqual(t.avg_left_upper_arm_length, .3)
            t.is_runing = True  # explicit operation service has authorized follow
            for _ in range(60): exec(step, env)
            self.assertTrue(t.is_runing)
            t.left_joystick = t.right_joystick = [0., 1.]
            for _ in range(60): exec(step, env)
            self.assertFalse(t.is_runing)
            # Stale grips from the put-down controllers cannot veto optical OK.
            t.is_hand_tracking = True
            t.left_finger_joints = t.right_finger_joints = [0., 0., 1.2, 0., 0., 0.]
            self.assertTrue(t.is_runing_gesture())
            self.assertFalse(t.is_stop_gesture())
            t.left_joystick = t.right_joystick = None
            self.assertTrue(t.is_runing_gesture())
            self.assertFalse(t.is_stop_gesture())
            t.left_finger_joints = t.right_finger_joints = [0., 0., 0., 1.2, 1.2, 0.]
            self.assertTrue(t.is_stop_gesture())

    def test_production_xa_branch_does_not_switch_legacy_torso(self):
        source = (ROOT/'src/QuestAbsoluteControlFSMNode.cpp').read_text()
        section = source[source.index('void updateStateLegacyWheelVr()'):]
        begin = section.index('            if (joystick_data_.left_first_button_pressed)\n')
        end = section.index('            const bool legacy_torso_toggle_modifier')
        branch = section[begin:end]
        begin = source.index('        static int resolveVrArmModeForXb(')
        end = source.index('        bool callSetArmModeSrv(', begin)
        resolve = source[begin:end]
        helper = source[source.index('        void callWheelMpcControlMode('):]
        guard = helper[helper.index('            if (enable_velocity_ik_takeover_)'):helper.index('            kuavo_msgs::changeTorsoCtrlMode srv;')]
        code = r'''
#include <cassert>
#include <iostream>
#include <string>
#define ROS_WARN_THROTTLE(...)
namespace ros { namespace param { template<class T> void getCached(const char*, T&) {} } }
struct Joy { bool left_first_button_pressed=false, right_first_button_pressed=false, right_second_button_pressed=false; };
struct FSM {
 Joy joystick_data_, joystick_data_prev_;
 bool enable_velocity_ik_takeover_=true, arm_collision_control_=false;
 bool use_cpp_incremental_ik_=false, arm_ctrl_mode_dual_feed_=false;
 bool use_auto_track_=false, only_half_up_body_=false;
 bool quest3_arm_reset_head_snapshot_active_=false, suppress_next_quest3_head_fixed_intent_=false;
 int arm_ctrl_mode_=1, arm_ctrl_mode_current_=1, robot_type_=1;
 int arm_calls=0, torso_calls=0; bool accepted=true;
 std::string head_ctrl_mode_, last_head_ctrl_mode_;
 bool callSetArmModeSrv(int mode) { ++arm_calls; if(accepted) arm_ctrl_mode_=arm_ctrl_mode_current_=mode; return accepted; }
 void callVRSetArmModeSrv(int mode) { callSetArmModeSrv(mode); }
 void callVRSetHeadModeSrv(const std::string&) {}
 void callWheelMpcControlMode(int mode) {
''' + guard + r'''
 assert(mode==3); ++torso_calls; }
''' + resolve + '\nvoid update() {\n' + branch + r'''
 }
 void sample(bool x, bool a) {
   joystick_data_.left_first_button_pressed=x; joystick_data_.right_first_button_pressed=a;
   update(); joystick_data_prev_=joystick_data_;
 }
};
int main() {
 for(bool xFirst : {false,true}) {
   FSM f;
   f.sample(xFirst,!xFirst); f.sample(true,true);
   assert(f.arm_calls==1 && f.arm_ctrl_mode_==2 && f.torso_calls==0);
   for(int i=0;i<50;++i) f.sample(true,true);
   assert(f.arm_calls==1);
   f.sample(false,false); f.sample(true,true);
   assert(f.arm_calls==2 && f.arm_ctrl_mode_==1 && f.torso_calls==0);
   f.sample(false,false); f.sample(true,true);
   assert(f.arm_calls==3 && f.arm_ctrl_mode_==2 && f.torso_calls==0);
 }
 FSM rejected; rejected.accepted=false; rejected.sample(true,true);
 assert(rejected.torso_calls==0 && rejected.arm_ctrl_mode_==1);
 FSM legacy; legacy.enable_velocity_ik_takeover_=false; legacy.sample(true,true);
 assert(legacy.arm_calls==1 && legacy.torso_calls==1);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path/'test.cpp').write_text(code)
            subprocess.run(['g++', '-std=c++17', str(path/'test.cpp'), '-o', str(path/'test')], check=True)
            subprocess.run([str(path/'test')], check=True, stdout=subprocess.DEVNULL)

if __name__ == '__main__':
    unittest.main()
