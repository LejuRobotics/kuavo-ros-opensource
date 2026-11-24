#!/usr/bin/env python3
"""Offline three-pose controller calibration; never publishes ROS commands."""
import argparse
import datetime
import itertools
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

BONES = {'left': (0, 1, 4), 'right': (2, 3, 5)}
LABELS = ('up', 'facing', 'down')
POSE_ORDERS = {
    'up,facing,down': LABELS,
    'facing,down': ('facing', 'down'),
}
RAW_TOPIC = '/leju_quest_bone_poses_raw'
LEGACY_TOPIC = '/leju_quest_bone_poses'
CONVENTIONS = ('legacy', 'sg100_400062')


class CalibrationRejected(ValueError):
    def __init__(self, message, diagnostics=None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}


def unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if not np.all(np.isfinite(v)) or n < 1e-8:
        raise ValueError('Degenerate direction in recorded geometry')
    return v / n


def angle(a, b):
    return float(np.degrees(np.arccos(np.clip(np.dot(unit(a), unit(b)), -1, 1))))


def rotation_errors(rotations, reference):
    return np.degrees((reference.inv() * rotations).magnitude())


def pose_sample(msg, seconds):
    if len(msg.poses) < 24 or msg.is_hand_tracking or not msg.is_high_confidence:
        return None
    def pos(i):
        p = msg.poses[i].position
        return np.array([p.x, p.y, p.z], dtype=float)
    # ROS bone poses have already been converted to robot axes (Z up).
    left_axis = pos(0) - pos(2)
    left_axis[2] = 0.
    left_axis = unit(left_axis)
    forward = unit(np.cross(left_axis, [0., 0., 1.]))
    row = {'t': seconds, 'left_axis': left_axis, 'forward': forward}
    for side, (shoulder, elbow, palm) in BONES.items():
        p, s, e = pos(palm), pos(shoulder), pos(elbow)
        q = msg.poses[palm].orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        if not np.all(np.isfinite(np.r_[p, s, e, quat])) or np.linalg.norm(quat) < 1e-8:
            return None
        # Reject rest poses and crossed/backward arms, before stable-window search.
        reach = p - s
        if np.dot(reach, forward) < .25 or angle(reach, forward) > 35:
            return None
        if angle(e - s, p - e) > 45:
            return None
        row[side] = {'p': p, 'q': Rotation.from_quat(quat / np.linalg.norm(quat)).as_quat()}
    return row


def _split_part(path):
    # rosbag --split timestamps every newly opened file, so the timestamp is
    # not a session key. The stable source identity is the user prefix before
    # the final YYYY-MM-DD-HH-MM-SS_INDEX suffix.
    match = re.match(
        r'^(?P<prefix>.*)(?P<stamp>\d{4}(?:-\d{2}){5})_(?P<index>\d+)\.bag(?:\.active)?$',
        Path(path).name)
    if not match:
        return None
    return {'prefix': match.group('prefix'), 'stamp': match.group('stamp'),
            'index': int(match.group('index'))}


def _bag_interval(path):
    import rosbag
    with rosbag.Bag(str(path), skip_index=True) as bag:
        return float(bag.get_start_time()), float(bag.get_end_time())


def _completed_split_session(latest, completed, boundary_tolerance=2.0):
    latest_part = _split_part(latest)
    if latest_part is None or latest_part['index'] == 0:
        return [latest]
    parsed = [(path, _split_part(path)) for path in completed]
    chain = [latest]
    following = latest
    for wanted in range(latest_part['index'] - 1, -1, -1):
        following_start, _ = _bag_interval(following)
        candidates = []
        for path, part in parsed:
            if part is None or part['prefix'] != latest_part['prefix'] or part['index'] != wanted:
                continue
            start, end = _bag_interval(path)
            boundary_gap = following_start - end
            if start < following_start and abs(boundary_gap) <= boundary_tolerance:
                candidates.append(path)
        if len(candidates) != 1:
            reason = 'missing/non-continuous' if not candidates else 'ambiguous'
            raise ValueError(
                'Cannot infer latest split recording: %s predecessor index %d for %s' %
                (reason, wanted, latest))
        following = candidates[0]
        chain.append(following)
    return list(reversed(chain))


def select_bag_files(explicit=None, bag_dir=Path.home() / '.ros'):
    """Select one completed recording session without crossing session boundaries."""
    if explicit is not None:
        path = Path(explicit)
        if path.name.endswith('.active') or path.suffix != '.bag':
            raise ValueError('Explicit recording must be a completed .bag file')
        return [path]
    bag_dir = Path(bag_dir).expanduser()
    if not bag_dir.is_dir():
        raise ValueError('Bag directory does not exist: ' + str(bag_dir))
    completed = [path for path in bag_dir.glob('*.bag') if path.is_file()]
    active = [path for path in bag_dir.glob('*.bag.active') if path.is_file()]
    if not completed:
        if active:
            raise ValueError('Latest recording is still active and has not been finalized: ' +
                             str(max(active, key=lambda path: path.stat().st_mtime)))
        raise ValueError('No completed .bag recordings in ' + str(bag_dir))
    latest = max(completed, key=lambda path: path.stat().st_mtime)
    newest_completed = latest.stat().st_mtime
    newer_active = [path for path in active if path.stat().st_mtime >= newest_completed]
    if newer_active:
        raise ValueError('Latest recording is still active and has not been finalized: ' +
                         str(max(newer_active, key=lambda path: path.stat().st_mtime)))
    selected = _completed_split_session(latest, completed)
    latest_part = _split_part(latest)
    if latest_part is not None:
        same_session_active = []
        oldest_member_mtime = min(path.stat().st_mtime for path in selected)
        for path in active:
            part = _split_part(path)
            if (part is not None and part['prefix'] == latest_part['prefix'] and
                    part['index'] == latest_part['index'] + 1 and
                    path.stat().st_mtime >= oldest_member_mtime):
                same_session_active.append(path)
        if same_session_active:
            raise ValueError('Latest recording is still active and has not been finalized: ' +
                             str(max(same_session_active, key=lambda path: path.stat().st_mtime)))
    return selected


def _ordered_bags(paths):
    return [path for _, path in sorted((_bag_interval(path)[0], Path(path)) for path in paths)]


def read_bags(paths, topic):
    import rosbag  # Pure numerical tests/help do not need ROS.
    rows = []
    last = -float('inf')
    timed_paths = sorted((_bag_interval(path), Path(path)) for path in paths)
    paths = [path for _, path in timed_paths]
    origin = min(interval[0] for interval, _ in timed_paths)
    for path in paths:
        with rosbag.Bag(str(path)) as bag:
            for _, msg, stamp in bag.read_messages(topics=[topic]):
                t = stamp.to_sec() - origin
                try:
                    row = pose_sample(msg, t)
                except ValueError:
                    row = None
                # Validate every packet before downsampling; short tracking failures
                # must still break a stable hold even between retained samples.
                if row is None:
                    rows.append({'t': t, 'invalid': True})
                    last = t
                    continue
                if t - last < .045:
                    continue
                last = t
                rows.append(row)
    if not rows:
        raise ValueError('No VR bone messages in bag: ' + topic)
    return rows


def read_bag(path, topic):
    return read_bags([path], topic)


def bag_metadata(paths):
    result = []
    for path in paths:
        path = Path(path)
        digest = hashlib.sha256()
        with path.open('rb') as source:
            for block in iter(lambda: source.read(1024 * 1024), b''):
                digest.update(block)
        result.append({'path': str(path.resolve()), 'sha256': digest.hexdigest()})
    return result


def summarize(rows, minimum):
    if not rows or any(row.get('invalid') for row in rows):
        raise ValueError('Invalid/controller-tracking-lost frames inside hold')
    duration = rows[-1]['t'] - rows[0]['t']
    if duration < minimum or len(rows) < minimum * 10:
        raise ValueError('Each hold needs at least %.1f seconds and 10 Hz coverage' % minimum)
    if np.max(np.diff([r['t'] for r in rows])) > .2:
        raise ValueError('Tracking gap over 0.2 seconds inside hold')
    result = {'start': rows[0]['t'], 'end': rows[-1]['t'], 'duration': duration,
              'samples': len(rows)}
    heading = unit(np.mean([r['left_axis'] for r in rows], axis=0))
    if max(angle(r['left_axis'], heading) for r in rows) > 5:
        raise ValueError('Body heading changes more than 5 degrees inside hold')
    result['left_axis'] = heading
    result['forward'] = unit(np.cross(heading, [0., 0., 1.]))
    for side in BONES:
        rotations = Rotation.from_quat([r[side]['q'] for r in rows])
        mean = rotations.mean()
        error = rotation_errors(rotations, mean)
        positions = np.array([r[side]['p'] for r in rows])
        scatter = np.linalg.norm(positions - np.median(positions, axis=0), axis=1)
        if np.percentile(error, 95) > 3 or np.max(error) > 8:
            raise ValueError(side + ': unstable orientation during hold')
        if np.percentile(scatter, 95) > .015:
            raise ValueError(side + ': position drift exceeds 1.5 cm during hold')
        result[side] = {'q': mean.as_quat(), 'angular_p95_deg': float(np.percentile(error, 95)),
                        'angular_max_deg': float(np.max(error)),
                        'position_p95_m': float(np.percentile(scatter, 95)),
                        'p': np.median(positions, axis=0).tolist()}
    return result


def discover_holds(rows, minimum):
    runs, current = [], []
    for row in rows:
        good = not row.get('invalid')
        if good and current and row['t'] - current[-1]['t'] > .2:
            good = False
        if not good:
            if current:
                runs.append(current)
            current = []
        if not row.get('invalid'):
            current.append(row)
    if current:
        runs.append(current)

    windows = []
    for run in runs:
        end = 0
        for start in range(len(run)):
            end = max(end, start)
            while end < len(run) and run[end]['t'] - run[start]['t'] < minimum:
                end += 1
            if end == len(run):
                break
            try:
                windows.append(summarize(run[start:end + 1], minimum))
            except ValueError:
                pass

    def same_hold(anchor, previous, candidate):
        overlap = min(previous['end'], candidate['end']) - max(previous['start'], candidate['start'])
        if overlap < minimum * .5:
            return False
        for side in BONES:
            a = Rotation.from_quat(anchor[side]['q'])
            b = Rotation.from_quat(candidate[side]['q'])
            if np.degrees((a.inv() * b).magnitude()) > 6:
                return False
            if np.linalg.norm(np.asarray(anchor[side]['p']) - candidate[side]['p']) > .03:
                return False
        return True

    def quality(hold):
        values = []
        for side in BONES:
            values.extend((hold[side]['angular_p95_deg'] / 3.,
                           hold[side]['angular_max_deg'] / 8.,
                           hold[side]['position_p95_m'] / .015))
        return max(values)

    clusters = []
    for window in windows:
        if clusters and same_hold(clusters[-1]['anchor'], clusters[-1]['last'], window):
            clusters[-1]['last'] = window
            if quality(window) < quality(clusters[-1]['best']):
                clusters[-1]['best'] = window
        else:
            clusters.append({'anchor': window, 'last': window, 'best': window})
    return [cluster['best'] for cluster in clusters]


def _rpy_matrix(value):
    return Rotation.from_euler('xyz', np.asarray(value, dtype=float)).as_matrix()


def inspect_sg100_400062_model(path):
    """Bind the SG100 semantic axes to the exact URDF used by velocity IK."""
    path = Path(path)
    root = ET.parse(str(path)).getroot()
    joints = {joint.get('name'): joint for joint in root.findall('joint')}

    def origin(name):
        try:
            node = joints[name].find('origin')
            xyz = np.fromstring(node.get('xyz', '0 0 0'), sep=' ')
            rpy = np.fromstring(node.get('rpy', '0 0 0'), sep=' ')
        except (KeyError, AttributeError) as exc:
            raise ValueError('SG100 model is missing joint ' + name) from exc
        if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[xyz, rpy]).all():
            raise ValueError('Invalid origin in SG100 model joint ' + name)
        return xyz, rpy

    for side in ('l', 'r'):
        xyz, rpy = origin('zarm_' + side + '7_end_effector_joint')
        if not np.allclose(xyz, [0., 0., -.17], atol=1e-8) or not np.allclose(rpy, 0., atol=1e-8):
            raise ValueError('SG100 400062 EEF must be aligned with joint7 and extend along -Z')
    expected = {'l': [0., np.pi, 0.], 'r': [np.pi, 0., 0.]}
    axes = {}
    for side in ('l', 'r'):
        _, rpy = origin(side + '_hand_base_joint')
        if not np.allclose(_rpy_matrix(rpy), _rpy_matrix(expected[side]), atol=1e-6):
            raise ValueError('SG100 400062 hand-base orientation mismatch for ' + side)
        base_to_eef = _rpy_matrix(rpy)
        axes['left' if side == 'l' else 'right'] = {
            'finger_axis_eef': base_to_eef @ np.array([0., 0., 1.]),
            'palm_normal_axis_eef': base_to_eef @ np.array([0., -1., 0.]),
        }
    if not np.allclose(axes['left']['finger_axis_eef'], [0, 0, -1], atol=1e-6):
        raise ValueError('SG100 400062 left finger-axis contract mismatch')
    if not np.allclose(axes['right']['finger_axis_eef'], [0, 0, -1], atol=1e-6):
        raise ValueError('SG100 400062 right finger-axis contract mismatch')
    if not np.allclose(axes['left']['palm_normal_axis_eef'], [0, -1, 0], atol=1e-6):
        raise ValueError('SG100 400062 left palm-axis contract mismatch')
    if not np.allclose(axes['right']['palm_normal_axis_eef'], [0, 1, 0], atol=1e-6):
        raise ValueError('SG100 400062 right palm-axis contract mismatch')
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return {'path': str(path.resolve()), 'sha256': digest, 'axes': axes}


def _validated_labels(labels):
    labels = tuple(labels)
    if labels not in POSE_ORDERS.values():
        raise ValueError('Unsupported pose order: ' + ','.join(labels))
    return labels


def calibrate(holds, distance=None, orientation=False, convention='legacy', model=None,
              labels=LABELS, _quality_only=False):
    labels = _validated_labels(labels)
    if len(holds) != len(labels):
        raise ValueError('Expected %d labelled holds for %s' %
                         (len(labels), ','.join(labels)))
    if distance is None:
        if not orientation and not _quality_only:
            raise ValueError('Translation distance is unknown; provide measured --outward-distance '
                             'or use --calibrate-orientation for rotation-only calibration')
    elif not np.isfinite(distance) or not .001 <= distance <= .25:
        raise ValueError('Known outward distance must be between 0.001 and 0.25 m')
    if convention not in CONVENTIONS:
        raise ValueError('Unsupported EEF convention: ' + str(convention))
    if convention == 'sg100_400062' and model is None:
        raise ValueError('sg100_400062 requires a verified model contract')
    pose_count = 'three' if len(labels) == 3 else 'two'
    method = ('semantic_%s_pose_known_normal_offset' % pose_count if distance is not None else
              'semantic_%s_pose_orientation_only' % pose_count)
    translation_calibrated = distance is not None
    result = {'enabled': True, 'schema_version': 1 if convention == 'legacy' else 2,
              'method': method,
              'metadata': {'orientation_calibrated': orientation,
                           'translation_calibrated': translation_calibrated,
                           'distance_source': 'user_prior' if translation_calibrated else 'unknown',
                           'translation_status': ('known_normal_distance_prior' if translation_calibrated
                                                  else 'not_identifiable_from_recording'),
                           'distance_is_prior_not_measured': translation_calibrated,
                           'full_translation_observable': False, 'labels': list(labels),
                           'method': method, 'pose_order': ','.join(labels)}}
    if translation_calibrated:
        result['metadata']['known_distance_m'] = distance
    rejected = {}
    for side in BONES:
        normals, raw_to_eef = [], []
        for label, hold in zip(labels, holds):
            if label == 'up':
                normal = np.array([0., 0., 1.])
            elif label == 'down':
                normal = np.array([0., 0., -1.])
            else:
                normal = hold['left_axis'] * (-1 if side == 'left' else 1)
            raw = Rotation.from_quat(hold[side]['q'])
            normals.append(raw.inv().apply(normal))
            # The SG100 400062 hand mount mirrors the right palm axis and both
            # EEF finger axes are -Z. Legacy keeps the historical +Z/-Y map.
            y = unit(normal) if convention == 'sg100_400062' and side == 'right' else -unit(normal)
            finger = unit(hold['forward'] - np.dot(hold['forward'], y) * y)
            z = -finger if convention == 'sg100_400062' else finger
            desired = Rotation.from_matrix(np.column_stack([np.cross(y, z), y, z]))
            raw_to_eef.append((raw.inv() * desired).as_quat())
        normal = unit(np.mean(normals, axis=0))
        residual = [angle(v, normal) for v in normals]
        if np.sqrt(np.mean(np.square(residual))) > 5 or max(residual) > 10:
            rejected.setdefault(side, {}).update({
                'normal_residual_deg': residual,
                'normal_rms_deg': float(np.sqrt(np.mean(np.square(residual))))})
        fitted = Rotation.from_quat(raw_to_eef).mean()
        orientation_residual = rotation_errors(Rotation.from_quat(raw_to_eef), fitted)
        # Match the existing unmounted controller map, including its +/-15deg bias.
        base = Rotation.from_euler('xyz', [-np.pi/2, 0., -np.pi if side == 'left' else 0.])
        base = base * Rotation.from_euler('x', np.radians(-15 if side == 'left' else 15))
        correction = base.inv() * fitted
        if orientation and max(orientation_residual) > 10:
            rejected.setdefault(side, {}).update({
                'semantic_rotation_residual_deg': orientation_residual.tolist()})
        translation = (-distance * normal if translation_calibrated else np.zeros(3))
        result[side] = {'palm_to_controller_offset': translation.tolist(),
                        'eef_post_rotation_xyzw': correction.as_quat().tolist() if orientation else [0., 0., 0., 1.]}
        result['metadata'][side] = {'normal_residual_deg': residual,
                                    'semantic_rotation_residual_deg': orientation_residual.tolist(),
                                    'semantic_rotation_candidate_xyzw': correction.as_quat().tolist()}
    if convention == 'sg100_400062':
        result.update({'eef_convention': convention, 'robot_version': 400062,
                       'model_sha256': model['sha256']})
        result['metadata'].update({
            'model_path': model['path'],
            'eef_axes': {side: {name: np.asarray(axis).tolist() for name, axis in values.items()}
                         for side, values in model['axes'].items()},
        })
    if rejected:
        raise CalibrationRejected(
            'calibration quality gates failed for ' + ', '.join(sorted(rejected)),
            {'sides': rejected})
    return result


def parse_intervals(intervals, labels):
    labels = _validated_labels(labels)
    if intervals is None:
        return None
    if len(intervals) != len(labels):
        raise ValueError('--intervals needs %d START:END ranges in %s order' %
                         (len(labels), ','.join(labels)))
    parsed = []
    for value in intervals:
        try:
            start, end = map(float, value.split(':'))
        except (TypeError, ValueError):
            raise ValueError('Invalid interval; expected START:END: ' + str(value))
        if not np.isfinite([start, end]).all() or start >= end:
            raise ValueError('Each interval requires finite START < END')
        if parsed and start < parsed[-1][1]:
            raise ValueError('--intervals must be time-ordered and non-overlapping')
        parsed.append((start, end))
    return parsed


def choose_holds(rows, minimum, intervals, labels=LABELS):
    labels = _validated_labels(labels)
    parsed_intervals = parse_intervals(intervals, labels)
    if parsed_intervals:
        holds = []
        for start, end in parsed_intervals:
            holds.append(summarize([r for r in rows if start <= r['t'] <= end], minimum))
        calibrate(holds, labels=labels, _quality_only=True)  # Same quality gate as automatic selection.
        return holds
    candidates = discover_holds(rows, minimum)
    windows = [[round(h['start'], 3), round(h['end'], 3)] for h in candidates]
    if len(candidates) > 12:
        raise ValueError('Too many stable holds; specify --intervals: %s' % windows)
    if len(labels) == 2 and len(candidates) != 2:
        raise ValueError(
            'Two-pose protocol found %d stable holds; provide exactly two --intervals in '
            'facing,down order (required when selecting from a three-pose bag); holds=%s' %
            (len(candidates), windows))
    valid = []
    for selected in itertools.combinations(candidates, len(labels)):
        try:
            calibrate(selected, labels=labels, _quality_only=True)
            valid.append(selected)
        except ValueError:
            pass
    if len(valid) != 1:
        raise ValueError('Found %d stable holds and %d valid %d-pose selections; '
                         'use --intervals to disambiguate; holds=%s' %
                         (len(candidates), len(valid), len(labels), windows))
    return valid[0]


def atomic_yaml(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink():
        raise ValueError('Refusing to replace a symbolic-link configuration')
    if path.exists():
        backup = path.with_name(path.name + '.before-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        shutil.copy2(path, backup)
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w') as handle:
            yaml.safe_dump(document, handle, sort_keys=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, (path.stat().st_mode & 0o777) if path.exists() else 0o644)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def atomic_json(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink():
        raise ValueError('Refusing to replace a symbolic-link report')
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w') as handle:
            json.dump(document, handle, ensure_ascii=False, indent=2)
            handle.write('\n')
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def resolve_active_config(convention, requested=None, script_path=Path(__file__)):
    if requested is not None:
        return Path(requested)
    script_path = Path(script_path).resolve()
    if convention == 'legacy':
        return script_path.parents[1] / 'config/quest_controller_calibration.yaml'
    wheel_package = script_path.parents[3] / 'wheel_arm_abs_vel'
    if not wheel_package.is_dir():
        raise ValueError(
            'sg100_400062 has no full-body calibration consumer in this repository; '
            'pass --config explicitly only for a verified consumer')
    return wheel_package / 'config/quest_controller_calibration.yaml'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, nargs='?',
                        help='Completed bag file; omitted selects the latest completed session')
    parser.add_argument('--bag-dir', type=Path, default=Path.home() / '.ros',
                        help='Directory searched when BAG is omitted (default: ~/.ros)')
    parser.add_argument('--topic', default=RAW_TOPIC)
    parser.add_argument('--allow-uncompensated-legacy-topic', action='store_true',
                        help='Explicitly admit an old bag whose legacy pose topic is known to be uncompensated')
    parser.add_argument('--eef-convention', choices=CONVENTIONS, default='legacy')
    parser.add_argument('--model', type=Path,
                        help='Exact 400062 velocity-IK URDF; required by sg100_400062')
    parser.add_argument('--pose-order', default='up,facing,down', choices=tuple(POSE_ORDERS),
                        help='Operator-declared recording protocol. For facing,down on an existing '
                             'three-pose bag, provide the two explicit --intervals that skip up; '
                             'semantics are never inferred from local rotations.')
    parser.add_argument('--outward-distance', type=float, default=None,
                        help='Measured mounting displacement in metres. Omitted means unknown '
                             'translation; the bag does not estimate it.')
    parser.add_argument('--min-hold', type=float, default=10.)
    parser.add_argument('--intervals', nargs='+', metavar='START:END',
                        help='Seconds from bag start; count and order must match --pose-order')
    parser.add_argument('--calibrate-orientation', action='store_true',
                        help='Also use semantic fingers-forward/neutral-wrist prior; not independent 6DOF ground truth')
    parser.add_argument('--output', type=Path, default=Path('quest_controller_calibration.candidate.yaml'))
    parser.add_argument('--report', type=Path,
                        help='Write diagnostics as JSON on success or rejection; never activates configuration')
    parser.add_argument('--apply', action='store_true', help='Atomically install after all quality gates; next launch uses it')
    parser.add_argument('--config', type=Path,
                        help='Active configuration path; default follows the selected EEF consumer')
    args = parser.parse_args()
    safe_report = None
    report = {'passed': False, 'applied': False, 'input': {
        'bag': str(args.bag) if args.bag is not None else None,
        'bag_dir': str(args.bag_dir), 'topic': args.topic, 'min_hold': args.min_hold,
        'eef_convention': args.eef_convention, 'outward_distance': args.outward_distance,
        'pose_order': args.pose_order,
    }}
    try:
        labels = POSE_ORDERS[args.pose_order]
        parse_intervals(args.intervals, labels)  # Reject malformed protocols before opening any bag.
        bags = select_bag_files(args.bag, args.bag_dir)
        report['input']['bags'] = [str(path) for path in bags]
        args.config = resolve_active_config(args.eef_convention, args.config)
        protected = {
            'candidate output': args.output.resolve(),
            'active configuration': args.config.resolve(),
        }
        protected.update({'input bag ' + str(index): path.resolve()
                          for index, path in enumerate(bags)})
        if args.report is not None:
            report_path = args.report.resolve()
            conflicts = [name for name, path in protected.items() if path == report_path]
            if conflicts:
                raise ValueError('--report must differ from ' + ', '.join(conflicts))
            safe_report = args.report
        report['input']['configuration'] = str(args.config)
        if not np.isfinite(args.min_hold) or args.min_hold < 3:
            raise ValueError('--min-hold must be at least 3 seconds')
        if args.apply and args.min_hold < 10:
            raise ValueError('--apply requires --min-hold of at least 10 seconds')
        if (args.eef_convention == 'sg100_400062' and args.min_hold >= 10 and
                not args.calibrate_orientation):
            raise ValueError(
                'sg100_400062 activation-eligible calibration requires --calibrate-orientation')
        if args.topic == LEGACY_TOPIC and not args.allow_uncompensated_legacy_topic:
            raise ValueError('Legacy pose topic may already contain compensation; pass '
                             '--allow-uncompensated-legacy-topic only for a verified old bag')
        if args.eef_convention == 'sg100_400062' and args.model is None:
            raise ValueError('--model is required by --eef-convention sg100_400062')
        if args.outward_distance is None and not args.calibrate_orientation:
            raise ValueError('Translation distance is unknown; provide measured --outward-distance '
                             'or use --calibrate-orientation for rotation-only calibration')
        if any(path.resolve() in (args.output.resolve(), args.config.resolve()) for path in bags):
            raise ValueError('Input bag must differ from candidate and active configuration paths')
        if args.output.resolve() == args.config.resolve():
            raise ValueError('--output must differ from active --config; activation requires --apply')
        bags = _ordered_bags(bags)
        report['input']['bags'] = [str(path) for path in bags]
        rows = read_bags(bags, args.topic)
        candidates = discover_holds(rows, args.min_hold)
        report['stable_holds'] = [[h['start'], h['end']] for h in candidates]
        holds = choose_holds(rows, args.min_hold, args.intervals, labels)
        model = inspect_sg100_400062_model(args.model) if args.eef_convention == 'sg100_400062' else None
        result = calibrate(holds, args.outward_distance, args.calibrate_orientation,
                           args.eef_convention, model, labels)
        recorded_bags = bag_metadata(bags)
        recording_metadata = {'bags': recorded_bags}
        if len(recorded_bags) == 1:
            recording_metadata.update({'bag': recorded_bags[0]['path'],
                                       'bag_sha256': recorded_bags[0]['sha256']})
        result['metadata'].update({**recording_metadata,
                                  'coordinate_convention': 'ROS HandPalm local; robot-world Z up; left axis from shoulders',
                                  'pose_order_source': 'operator_declared',
                                  'algorithm_version': 1,
                                  'holds_seconds': [[h['start'], h['end']] for h in holds],
                                  'created_at': datetime.datetime.now().astimezone().isoformat()})
        diagnostic_only = args.min_hold < 10
        result['metadata']['quality_gate'] = (
            'diagnostic_only_short_hold' if diagnostic_only else 'activation_eligible')
        document = {'quest3': {'controller_calibration': result}}
        if not diagnostic_only:
            atomic_yaml(args.output, document)
        if args.apply:
            atomic_yaml(args.config, document)
        report.update({'passed': True, 'output': None if diagnostic_only else str(args.output),
                       'applied': args.apply, 'effective_after': 'next teleop launch',
                       'configuration': str(args.config), 'result': result})
        if safe_report:
            atomic_json(safe_report, report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
    except (ValueError, OSError) as exc:
        report['error'] = str(exc)
        diagnostics = getattr(exc, 'diagnostics', None)
        if diagnostics:
            report['quality_diagnostics'] = diagnostics
        if safe_report:
            atomic_json(safe_report, report)
        parser.exit(2, 'Calibration rejected; active configuration not changed: %s\n' % exc)


if __name__ == '__main__':
    main()
