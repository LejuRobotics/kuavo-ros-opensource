#!/usr/bin/env python3
"""Closed-loop assertions driven through the production Quest/Pico button FSM."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rospy
from kuavo_msgs.msg import jointCmd, sensorsData
from wheel_arm_abs_vel.msg import TeleopTcpTarget

source, output, launch_file = sys.argv[1:]
root = Path(output)
root.mkdir(parents=True, exist_ok=True)
package = Path(__file__).resolve().parents[1]
rospy.init_node('xa_torso_regression', anonymous=True)
assert rospy.get_param('/real', True) is False
latest, checks, rows, commands = {}, [], [], []

def target(msg):
    latest['time'] = time.monotonic()
    latest['target'] = {k: getattr(msg, k) for k in
                        ('absolute_state', 'control_state', 'control_valid',
                         'torso_control_enabled', 'arm_hold', 'generation')}

def sensors(msg):
    latest['q'] = list(msg.joint_data.joint_q)
    rows.append((time.time(), latest['q'], list(msg.joint_data.joint_v)))

rospy.Subscriber('/mm/teleop_tcp_target', TeleopTcpTarget, target, queue_size=10)
rospy.Subscriber('/sensors_data_raw', sensorsData, sensors, queue_size=200)
rospy.Subscriber('/joint_cmd', jointCmd,
                 lambda m: commands.append((time.time(), list(m.joint_q))), queue_size=200)
processes, logs = [], []

def launch(args, name):
    log = (root / (name + '.log')).open('w')
    logs.append(log)
    proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(proc)
    return proc

def command(label, joy=None, **pose):
    data = dict(label=label, joy=joy or {}, **pose)
    tmp = root / 'command.tmp'
    tmp.write_text(json.dumps(data))
    tmp.replace(root / 'command.json')
    with (root / 'phases.jsonl').open('a') as out:
        out.write(json.dumps(dict(time=time.time(), **data)) + '\n')
    print(label, latest.get('target'), flush=True)

def check(label, condition, **metrics):
    checks.append(dict(label=label, passed=bool(condition), **metrics))
    if not condition:
        raise AssertionError((label, metrics, latest))

def wait(label, predicate, timeout=25):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if time.monotonic() - latest.get('time', 0) < .3 and predicate(latest.get('target', {})):
            check(label, True, target=latest['target'])
            return
        time.sleep(.05)
    check(label, False)

XA = dict(left_first_button_pressed=True, right_first_button_pressed=True)
XY = dict(left_first_button_touched=True, left_second_button_touched=True)
CHORD = dict(XY, right_second_button_pressed=True) if source == 'quest' else dict(left_second_button_pressed=True, right_trigger=.8)
active = lambda t: t.get('absolute_state') == 3 and t.get('control_state') == 2 and t.get('control_valid')

def press(label, joy, **pose):
    command(label, joy, **pose)
    time.sleep(.35)
    command(label + '_release', **pose)
    time.sleep(.4)

def motion(label, expect_lower):
    command(label, arm=.05, torso=.09)
    time.sleep(1.5)
    start = time.time()
    time.sleep(4)
    values = np.array([q for t, q, v in rows if t >= start])
    cmds = np.array([q for t, q in commands if t >= start])
    check(label + '_samples', len(values) > 200 and len(cmds) > 200)
    lower = float(np.max(np.ptp(values[:, :4], axis=0)))
    arm = float(np.max(np.ptp(cmds[:, 4:18], axis=0)))
    check(label + '_lower', lower > .015 if expect_lower else lower < .002,
          lower_range_rad=lower)
    check(label + '_arms', arm > .01, arm_command_range_rad=arm)

def displace(label):
    command(label, arm=.04, torso_offset=.15, chest_height_offset=-.04)
    time.sleep(5)
    q = np.array(latest['q'])[:18]
    check(label + '_nonzero_lower', np.max(np.abs(q[:4])) > .025, q=q.tolist())

def reset(label):
    before = np.array(latest['q'])[:18]
    press(label, XA)
    wait(label + '_preparing', lambda t: t.get('absolute_state') == 7)
    wait(label + '_complete', lambda t: t.get('absolute_state') == 8, 25)
    time.sleep(.5)
    after = np.array(latest['q'])[:18]
    check(label + '_lower_ready', np.max(np.abs(after[:4])) < .016,
          before=before[:4].tolist(), after=after[:4].tolist())
    check(label + '_lower_moved', np.max(np.abs(after[:4]-before[:4])) > .015)
    check(label + '_human_follow_off', not latest['target']['torso_control_enabled'])

try:
    command('initial')
    launch(['rosbag', 'record', '-O', str(root/'closed-loop.bag'),
            '/mm/teleop_tcp_target', '/quest_joystick_data', '/leju_quest_bone_poses',
            '/sensors_data_raw', '/joint_cmd', '/whole_body_velocity_ik/diagnostics/status'], 'bag')
    launch(['roslaunch', launch_file, 'input_source:='+source, 'enable_terminal_start:=false',
            'reference_shm_key:=542421', 'owner_shm_key:=542422'], 'input')
    launch(['python3', str(package/'test/simulation_control_fixture.py'), str(root)], 'fixture')
    wait('input_ready', lambda t: bool(t), 45)
    time.sleep(4)
    # Both devices explicitly exercise the formerly broken cold PREPARE path.
    press('cold_prepare', XA)
    wait('cold_preparing', lambda t: t.get('absolute_state') in (7, 8), 20)
    press('cold_first_follow', XA)
    wait('default_torso_mode', lambda t: active(t) and t.get('torso_control_enabled'), 35)
    motion('default_follow', True)
    if source == 'quest':
        command('b_before_xy', dict(right_second_button_pressed=True))
        time.sleep(.3)
        command('xy_after_held_b', CHORD)
        time.sleep(.4)
        check('b_must_be_fresh_press', latest['target']['torso_control_enabled'])
        command('release_chord')
        time.sleep(.4)
    displace('torso_displaced')
    reset('reset_from_torso_mode')
    press('resume_torso_mode', XA)
    wait('selected_torso_restored', lambda t: active(t) and t.get('torso_control_enabled'))
    motion('after_reset_follow', True)
    displace('before_arm_mode')
    press('select_arm_mode', CHORD, torso_offset=.15, chest_height_offset=-.04)
    wait('arm_mode', lambda t: active(t) and not t.get('torso_control_enabled'))
    motion('arm_mode_holds_lower', False)
    reset('reset_from_arm_mode')
    press('resume_arm_mode', XA)
    wait('selected_arm_mode_preserved', lambda t: active(t) and not t.get('torso_control_enabled'))
    motion('after_reset_arm_mode', False)
    press('select_torso_again', CHORD)
    wait('explicit_torso_reenabled', lambda t: active(t) and t.get('torso_control_enabled'))
    displace('before_interrupt')
    press('interruptible_prepare', XA)
    wait('interruptible_preparing', lambda t: t.get('absolute_state') == 7)
    generation = latest['target']['generation']
    press('interrupt_reset', XA)
    wait('interrupted_follow_restored', lambda t: active(t) and t.get('torso_control_enabled')
         and t.get('generation', 0) > generation)
    motion('interrupted_torso_follow', True)
    check('finite_feedback_and_commands', all(np.isfinite(q).all() and np.isfinite(v).all() for t,q,v in rows)
          and all(np.isfinite(q).all() for t,q in commands))
except Exception as exc:
    checks.append(dict(label='exception', passed=False, error=repr(exc)))
    print('FAILED', repr(exc), flush=True)
finally:
    (root/'checks.json').write_text(json.dumps(checks, indent=2))
    for proc in reversed(processes):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
    for proc in processes:
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)
    for log in logs:
        log.close()
print(json.dumps(checks, indent=2), flush=True)
sys.exit(0 if checks and all(c['passed'] for c in checks) else 1)
