#!/usr/bin/env python3
"""Compare frozen/candidate mapper math on extracted raw rosbag arrays.

This is an unpaced, hardware-independent core benchmark, NOT ROS callback or
AGX end-to-end timing. Input NPZ uses rows [bag_s, source_s, confidence, hand,
26*(position_xyz, quaternion_xyzw)]. No ROS or SciPy is needed.
"""
import argparse
import hashlib
import importlib.util
import json
import pathlib
import platform
import time

import numpy as np


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rotation(quaternion):
    quaternion = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(quaternion).all() or norm < 1e-6:
        raise ValueError('invalid quaternion')
    x, y, z, w = quaternion/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def summary(values):
    return dict(zip(('p50', 'p95', 'p99', 'max'),
                    np.percentile(values, [50, 95, 99, 100]).tolist()))


def replay(path, baseline, candidate):
    raw = np.load(path)['raw']
    observers = [module.SkeletonCV(q_p=.63, q_r=5.7, hand_q_r=1.42)
                 for module in (baseline, candidate)]
    axes = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    # Both fixed calibration cases exercise the same production API. Their
    # exact values do not affect the test's baseline/candidate equivalence.
    fixed = (baseline.exp_so3([.1, -.2, .3]), baseline.exp_so3([-.1, .2, -.3]))
    costs = [[], []]
    maximum = dict(position=0., rotation_matrix=0., velocity=0., angular_velocity=0.,
                   position_covariance=0., rotation_covariance=0., mapped_rate=0.)
    accepted = rejected = repeated = resets = 0
    previous_stamp = previous_key = None
    for frame, row in enumerate(raw):
        transforms = {}
        try:
            for index in baseline.SkeletonCV.indices:
                pose = row[4+index*7:11+index*7]
                transform = np.eye(4)
                if index != 22 or (np.isfinite(pose).all() and np.linalg.norm(pose[3:]) > .5):
                    if not np.isfinite(pose).all(): raise ValueError('invalid position')
                    transform[:3, :3] = rotation(pose[3:])
                    transform[:3, 3] = pose[:3]
                transforms[index] = transform
        except ValueError:
            rejected += 1
            continue
        stamp = float(row[1]); key = ('replay', bool(row[3]))
        repeated += int(previous_stamp == stamp and previous_key == key)
        resets += int(previous_stamp is None or previous_key != key
                      or stamp < previous_stamp or stamp-previous_stamp > .1)
        previous_stamp, previous_key = stamp, key
        states = [None, None]; rates = [None, None]
        # Alternate order so one variant does not always receive warm CPU/cache.
        for index in ((0, 1) if frame % 2 == 0 else (1, 0)):
            module = (baseline, candidate)[index]
            started = time.perf_counter_ns()
            state = observers[index].update(transforms, stamp, key)
            rate = []
            for side, (shoulder, elbow, palm) in enumerate(((0, 1, 4), (2, 3, 5))):
                rate.extend(module.mapped_rates(
                    state[23], state[shoulder], state[elbow], state[palm],
                    axes, .28440409, .45, state[palm][0][:3, :3]@fixed[side],
                    np.array([.01, -.02, .03])))
            costs[index].append((time.perf_counter_ns()-started)*1e-6)
            states[index], rates[index] = state, np.asarray(rate)
        for bone in baseline.SkeletonCV.indices:
            a, b = states[0][bone], states[1][bone]
            for name, error in (
                    ('position', abs(a[0][:3, 3]-b[0][:3, 3]).max()),
                    ('rotation_matrix', abs(a[0][:3, :3]-b[0][:3, :3]).max()),
                    ('velocity', abs(a[1]-b[1]).max()),
                    ('angular_velocity', abs(a[2]-b[2]).max()),
                    ('position_covariance', abs(observers[0].positions[bone].P-observers[1].positions[bone].P).max())):
                maximum[name] = max(maximum[name], float(error))
            if bone in baseline.SkeletonCV.rotating:
                error = abs(observers[0].rotations[bone].P-observers[1].rotations[bone].P).max()
                maximum['rotation_covariance'] = max(maximum['rotation_covariance'], float(error))
        maximum['mapped_rate'] = max(maximum['mapped_rate'], float(abs(rates[0]-rates[1]).max()))
        accepted += 1
    if max(maximum.values()) > 1e-8:
        raise AssertionError('numeric replay differs: '+str(maximum))
    return dict(path=str(path), frames=len(raw), accepted=accepted, invalid_rejected=rejected,
                repeated_stamp_frames=repeated, resets=resets, max_abs_errors=maximum,
                baseline_core_ms=summary(costs[0]), candidate_core_ms=summary(costs[1]),
                median_speedup=float(np.median(costs[0])/np.median(costs[1])),
                baseline_mean_ms=float(np.mean(costs[0])),
                candidate_mean_ms=float(np.mean(costs[1])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True, type=pathlib.Path)
    parser.add_argument('--candidate', required=True, type=pathlib.Path)
    parser.add_argument('--npz', required=True, action='append', type=pathlib.Path)
    parser.add_argument('--output', required=True, type=pathlib.Path)
    args = parser.parse_args()
    source_hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in (args.baseline, args.candidate)}
    modules = [load_module(args.baseline, 'frozen_math'), load_module(args.candidate, 'candidate_math')]
    result = dict(platform=platform.platform(), python=platform.python_version(), numpy=np.__version__,
                  source_sha256=source_hashes,
                  scope='Unpaced SkeletonCV plus two mapped_rates calls; excludes ROS, conversion, other callback work and scheduling',
                  replays=[replay(path, *modules) for path in args.npz])
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__': main()
