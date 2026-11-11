#!/usr/bin/env python3
"""Offline same-input gate. No ROS node, publisher, service, or hardware access.

Run --worker separately for before/after script trees, then --compare the NPZs.
Timing excludes bag reading, message decoding and output comparison. It measures
real read_msg/SkeletonCV/hand-pose math; ROS diagnostic publication is stubbed.
Source bag order, joystick inputs and controller-switch events are preserved.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time


def worker(args):
    import numpy as np
    import rosbag
    import rospy
    parameters = json.loads(Path(args.params).read_text())
    # Force only observation side effects off; numerical/safety settings untouched.
    overrides = {'~arm_marker_async_enabled': False, '~noncontrol_publish_async': False}
    def get_param(name, *default):
        if name in overrides:
            return overrides[name]
        parts = ('ik_ros_uni/'+name[1:] if name.startswith('~') else name).strip('/').split('/')
        value = parameters
        try:
            for key in parts:
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
    for name in ('loginfo','logwarn','logerr','logdebug','logwarn_throttle','logerr_throttle'):
        setattr(rospy, name, lambda *a, **kw: None)
    bag_clock = [0.0]
    rospy.Time.now = staticmethod(lambda: rospy.Time.from_sec(bag_clock[0]))
    rospy.get_time = lambda: bag_clock[0]
    os.environ.pop('KUAVO_RUNTIME_PARTITION_JSON', None)
    scripts = Path(args.scripts_dir).resolve()
    # Select the requested revision before any fallback to common helpers.
    sys.path.insert(0, str(scripts / 'tools'))
    sys.path.insert(0, str(scripts))
    import legacy_dependencies
    from tools.quest3_utils_velocity import Quest3ArmInfoTransformer

    fields = {
        'left_hand_pose': 7, 'right_hand_pose': 7,
        'left_elbow_pos': 3, 'right_elbow_pos': 3,
        'left_tcp_target': 7, 'right_tcp_target': 7,
        'left_tcp_elbow_hint': 3, 'right_tcp_elbow_hint': 3,
        'left_finger_joints': 6, 'right_finger_joints': 6,
        'left_shoulder_rpy_in_robot': 3, 'right_shoulder_rpy_in_robot': 3,
        'chest_axis_agl': 3,
    }
    def flatten(value):
        if isinstance(value, (tuple, list)):
            return np.concatenate([flatten(v) for v in value]) if value else np.array([])
        return np.asarray(value, dtype=float).reshape(-1)
    def capture(obj):
        result = []
        for name, size in fields.items():
            v = getattr(obj, name)
            a = np.full(size, np.nan) if v is None else flatten(v)
            assert len(a) == size, (name, a.shape, size)
            result.extend(a)
        for name in ('head_pitch','head_yaw','body_yaw','body_x','body_y','body_roll','body_pitch','body_height'):
            result.append(getattr(obj.head_body_pose, name))
        result.extend([obj.is_runing, obj.ok_gesture_counts, obj.shot_gesture_counts])
        for side in ('Left', 'Right'):
            rates = obj.tcp_estimated_rates.get(side)
            result.extend(np.full(9, np.nan) if rates is None else flatten(rates))
            d = obj.tcp_mapping_diagnostics[side]
            for name, size in [('mapped_joint7',16), ('mapped_source_position',3),
                               ('mapped_source_quaternion',4), ('upper_direction',3),
                               ('forearm_direction',3), ('source_to_tcp',16)]:
                value = None if d is None else d[name]
                result.extend(np.full(size, np.nan) if value is None else flatten(value))
        for i in obj.skeleton_filter.indices:
            state = obj.estimated_bones.get(i)
            result.extend(np.full(22, np.nan) if state is None else flatten(state))
        return result

    rows, stamps, accepted, timing, labels = [], [], [], [], []
    counts = {}
    # Consecutive split files belong to one session; new --group resets state.
    for group, paths in enumerate(args.group):
        obj = Quest3ArmInfoTransformer('', vis_pub=False, predict_gesture=False)
        stage = [0.0]*6
        update = obj.skeleton_filter.update
        def timed_update(*a, **kw):
            wall, cpu = time.monotonic_ns(), time.thread_time_ns()
            try: return update(*a, **kw)
            finally:
                stage[0] += (time.monotonic_ns()-wall)*1e-6
                stage[1] += (time.thread_time_ns()-cpu)*1e-6
        obj.skeleton_filter.update = timed_update
        hand = obj.compute_hand_pose
        def timed_hand(side, *a, **kw):
            wall, cpu = time.monotonic_ns(), time.thread_time_ns()
            try: return hand(side, *a, **kw)
            finally:
                j = 2 if side == 'Left' else 4
                stage[j] += (time.monotonic_ns()-wall)*1e-6
                stage[j+1] += (time.thread_time_ns()-cpu)*1e-6
        obj.compute_hand_pose = timed_hand
        frames = 0
        for path in paths:
            with rosbag.Bag(path) as bag:
                topics = ['/leju_quest_bone_poses','/quest_joystick_data',
                          '/humanoid_controller/controller_switch_event']
                for topic, msg, stamp in bag.read_messages(topics=topics):
                    bag_clock[0] = stamp.to_sec()
                    if topic == '/quest_joystick_data':
                        obj.read_joySticks_msg(msg)
                        continue
                    if topic.endswith('controller_switch_event'):
                        obj._controller_switch_event_callback(msg)
                        continue
                    if args.limit and frames >= args.limit:
                        break
                    stage[:] = [0.0]*6
                    wall, cpu = time.monotonic_ns(), time.thread_time_ns()
                    ok = obj.read_msg(msg, defer_visuals=True)
                    whole = [(time.monotonic_ns()-wall)*1e-6, (time.thread_time_ns()-cpu)*1e-6]
                    rows.append(capture(obj));stamps.append(msg.timestamp_ms)
                    accepted.append(ok);timing.append(whole+stage[:]);labels.append(group)
                    frames += 1
                    if frames % 2000 == 0:
                        print('PROGRESS',group,frames,flush=True)
        counts[str(group)] = frames
    np.savez(args.output, output=np.array(rows), stamp=np.array(stamps),
             accepted=np.array(accepted), timing=np.array(timing), group=np.array(labels))
    metadata = dict(frames=counts, scripts_dir=args.scripts_dir, groups=args.group,
                    params=args.params, timing_columns=['read_wall','read_cpu','filter_wall','filter_cpu',
                    'left_wall','left_cpu','right_wall','right_cpu'],
                    geometry=parameters['quest3']['teleop_tcp']['geometry_binding'],
                    scope='actual read_msg math + hand/TCP/elbow/finger/body/rates/filter state; ROS side effects stubbed; no actuator replay')
    Path(args.output+'.json').write_text(json.dumps(metadata, indent=2))
    print('REPLAY_COMPLETE',counts,flush=True)


def compare(args):
    import numpy as np
    a,b = [np.load(p) for p in args.compare]
    for key in ('stamp','accepted','group'):
        assert np.array_equal(a[key],b[key]), key+' differs'
    x,y = a['output'],b['output']
    assert x.shape == y.shape, (x.shape,y.shape)
    assert np.array_equal(np.isnan(x),np.isnan(y)), 'validity mask differs'
    assert np.all(np.isfinite(x[~np.isnan(x)])) and np.all(np.isfinite(y[~np.isnan(y)]))
    error = float(np.nanmax(np.abs(x-y)))
    report = dict(frames=len(x), output_values_per_frame=x.shape[1], max_abs_error=error,
                  tolerance=1e-6, passed=error<1e-6, groups={})
    for group in np.unique(a['group']):
        selected=(a['group']==group)&a['accepted']
        group_result={}
        for index,name in enumerate(['read_wall','read_cpu','filter_wall','filter_cpu','left_wall','left_cpu','right_wall','right_cpu']):
            av,bv=a['timing'][selected,index],b['timing'][selected,index]
            group_result[name]=dict(before_median_ms=float(np.median(av)),after_median_ms=float(np.median(bv)),
                before_p95_ms=float(np.percentile(av,95)),after_p95_ms=float(np.percentile(bv,95)),
                median_reduction_pct=float(100*(1-np.median(bv)/np.median(av))) if np.median(av)>0 else 0.)
        report['groups'][str(group)]=dict(frames=int(selected.sum()),timing=group_result)
    Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    if not report['passed']: raise SystemExit(1)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--worker',action='store_true');p.add_argument('--scripts-dir');p.add_argument('--params')
    p.add_argument('--group',nargs='+',action='append');p.add_argument('--limit',type=int,default=0)
    p.add_argument('--compare',nargs=2);p.add_argument('--output',required=True)
    args=p.parse_args()
    worker(args) if args.worker else compare(args)
