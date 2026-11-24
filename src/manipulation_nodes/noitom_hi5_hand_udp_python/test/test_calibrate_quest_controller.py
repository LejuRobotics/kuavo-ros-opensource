import importlib.util
import inspect
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xmlrpc.client
from unittest import mock
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/calibrate_quest_controller.py'
if not SCRIPT.exists():
    SCRIPT = Path(__file__).with_name('calibrate_quest_controller.py')
spec = importlib.util.spec_from_file_location('calibration', SCRIPT)
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
ASSETS = Path(__file__).resolve().parents[3] / 'kuavo_assets'
MODEL_400062 = ASSETS / 'models/biped_s400062/urdf/biped_s400062.urdf'
MODEL_HAS_SG100 = MODEL_400062.exists() and 'l_hand_base_joint' in MODEL_400062.read_text()


def samples(yaw=0., extra=False):
    heading = Rotation.from_euler('z', yaw)
    left_axis, forward = heading.apply([0., 1., 0.]), heading.apply([1., 0., 0.])
    transforms = {'left': Rotation.from_euler('xyz', [.3, -.2, .7]),
                  'right': Rotation.from_euler('xyz', [-.4, .6, -.2])}
    rows = []
    for index, label in enumerate(c.LABELS):
        rotations = {}
        for side in c.BONES:
            normal = np.array([0., 0., 1. if label == 'up' else -1.])
            if label == 'facing':
                normal = left_axis * (-1 if side == 'left' else 1)
            y, z = -normal, forward
            desired = Rotation.from_matrix(np.column_stack([np.cross(y, z), y, z]))
            rotations[side] = desired * transforms[side].inv()
        for j, t in enumerate(np.arange(0., 12.051, .05)):
            row = {'t': index*16. + float(t), 'left_axis': left_axis, 'forward': forward}
            for side in c.BONES:
                row[side] = {'q': rotations[side].as_quat() * (-1 if j % 2 else 1),
                             'p': np.array([.6, .25 if side == 'left' else -.25, 1.])}
            rows.append(row)
        rows.append({'t': index*16.+13., 'invalid': True})
    return rows, transforms


def replay_row(template, t, rotation=None, position_shift=None):
    rotation = rotation or Rotation.identity()
    position_shift = np.zeros(3) if position_shift is None else np.asarray(position_shift)
    row = {'t': float(t), 'left_axis': template['left_axis'].copy(),
           'forward': template['forward'].copy()}
    for side in c.BONES:
        row[side] = {
            'q': (rotation * Rotation.from_quat(template[side]['q'])).as_quat(),
            'p': template[side]['p'].copy() + position_shift,
        }
    return row


class CalibrationTest(unittest.TestCase):
    def test_discovery_finds_stable_middle_inside_transient_run(self):
        source, _ = samples()
        template = next(row for row in source if not row.get('invalid'))
        rows = []
        for t in np.arange(0., 19.001, .05):
            rotation = (Rotation.from_euler('z', .25*np.sin(t*7.))
                        if t < 2. or t > 17. else Rotation.identity())
            rows.append(replay_row(template, t, rotation))
        holds = c.discover_holds(rows, 10.)
        self.assertEqual(len(holds), 1)
        self.assertGreaterEqual(holds[0]['start'], 1.9)
        self.assertLessEqual(holds[0]['end'], 17.1)

    def test_discovery_uses_valid_window_when_whole_run_slow_drift_fails(self):
        source, _ = samples()
        template = next(row for row in source if not row.get('invalid'))
        rows = [replay_row(template, t, position_shift=[.0025*t, 0, 0])
                for t in np.arange(0., 15.001, .05)]
        with self.assertRaisesRegex(ValueError, 'position drift'):
            c.summarize(rows, 10.)
        holds = c.discover_holds(rows, 10.)
        self.assertEqual(len(holds), 1)
        self.assertGreaterEqual(holds[0]['end'] - holds[0]['start'], 10.)

    def test_discovery_never_bridges_invalid_gap(self):
        source, _ = samples()
        template = next(row for row in source if not row.get('invalid'))
        rows = [replay_row(template, t) for t in np.arange(0., 6.001, .05)]
        rows.append({'t': 6.05, 'invalid': True})
        rows.extend(replay_row(template, t) for t in np.arange(6.1, 12.101, .05))
        self.assertEqual(c.discover_holds(rows, 10.), [])

    def test_discovery_keeps_distinct_repeated_holds_ambiguous(self):
        source, _ = samples()
        template = next(row for row in source if not row.get('invalid'))
        rows = [replay_row(template, t) for t in np.arange(0., 11.001, .05)]
        rows.extend(replay_row(template, t, Rotation.from_euler('z', (t-11.)*np.pi/2.))
                    for t in np.arange(11.05, 12.001, .05))
        rows.extend(replay_row(template, t, Rotation.from_euler('z', np.pi/2.))
                    for t in np.arange(12.05, 23.051, .05))
        holds = c.discover_holds(rows, 10.)
        self.assertEqual(len(holds), 2)
        self.assertLess(holds[0]['end'], holds[1]['start'])

    def test_default_bag_directory_selects_latest_completed_session(self):
        self.assertEqual(inspect.signature(c.select_bag_files).parameters['bag_dir'].default,
                         Path.home()/'.ros')
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = root/'2026-11-19-13-00-00_0.bag'
            latest = root/'2026-11-19-14-00-00_0.bag'
            old.write_bytes(b'old')
            latest.write_bytes(b'latest')
            os.utime(old, (100., 100.))
            os.utime(latest, (200., 200.))
            self.assertEqual(c.select_bag_files(None, root), [latest])

    def test_empty_bag_directory_and_unfinalized_recording_are_explicit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with self.assertRaisesRegex(ValueError, 'No completed'):
                c.select_bag_files(None, root)
            active = root/'2026-11-19-14-00-00_0.bag.active'
            active.write_bytes(b'recording')
            with self.assertRaisesRegex(ValueError, 'still active'):
                c.select_bag_files(None, root)
            completed = root/'2026-11-19-13-00-00_0.bag'
            completed.write_bytes(b'old')
            os.utime(completed, (100., 100.))
            os.utime(active, (200., 200.))
            with self.assertRaisesRegex(ValueError, 'has not been finalized'):
                c.select_bag_files(None, root)

    def test_latest_split_session_selects_every_completed_part_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = root/'2026-11-19-13-00-00_0.bag'
            # Real rosbag --split changes the timestamp when it opens part 1.
            part0 = root/'2026-11-19-14-46-24_0.bag'
            part1 = root/'2026-11-19-14-48-33_1.bag'
            for index, path in enumerate((old, part0, part1)):
                path.write_bytes(path.name.encode())
                os.utime(path, (100. + index*100, 100. + index*100))
            intervals = {old: (1795060000., 1795060100.),
                         part0: (1795070784., 1795070913.396),
                         part1: (1795070913.367, 1795070934.257)}
            with mock.patch.object(c, '_bag_interval', side_effect=lambda path: intervals[path]):
                self.assertEqual(c.select_bag_files(None, root), [part0, part1])
            metadata = c.bag_metadata([part0, part1])
            self.assertEqual([Path(item['path']) for item in metadata], [part0, part1])
            self.assertTrue(all(len(item['sha256']) == 64 for item in metadata))

    def test_split_inference_rejects_missing_and_ambiguous_predecessors(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            latest = root/'2026-11-19-14-48-33_1.bag'
            other_source = root/'operator_2026-11-19-14-46-24_0.bag'
            latest.write_bytes(b'latest')
            other_source.write_bytes(b'other source')
            os.utime(latest, (300., 300.))
            os.utime(other_source, (100., 100.))
            intervals = {latest: (200., 220.), other_source: (100., 199.9)}
            with mock.patch.object(c, '_bag_interval', side_effect=lambda path: intervals[path]), \
                    self.assertRaisesRegex(ValueError, 'missing/non-continuous'):
                c.select_bag_files(None, root)

            first = root/'2026-11-19-14-46-24_0.bag'
            duplicate = root/'2026-11-19-14-46-25_0.bag'
            first.write_bytes(b'first')
            duplicate.write_bytes(b'duplicate')
            os.utime(first, (100., 100.))
            os.utime(duplicate, (200., 200.))
            intervals.update({first: (100., 199.9), duplicate: (101., 200.1)})
            with mock.patch.object(c, '_bag_interval', side_effect=lambda path: intervals[path]), \
                    self.assertRaisesRegex(ValueError, 'ambiguous'):
                c.select_bag_files(None, root)

    def test_new_index_zero_session_does_not_attach_an_older_session(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old0 = root/'2026-11-19-13-00-00_0.bag'
            old1 = root/'2026-11-19-13-02-00_1.bag'
            new0 = root/'2026-11-19-14-00-00_0.bag'
            for index, path in enumerate((old0, old1, new0)):
                path.write_bytes(path.name.encode())
                os.utime(path, (100. + index*100, 100. + index*100))
            self.assertEqual(c.select_bag_files(None, root), [new0])

    def test_explicit_bag_wins_without_adding_siblings_or_active_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            explicit = root/'session_0.bag'
            explicit.write_bytes(b'wanted')
            (root/'session_1.bag').write_bytes(b'unwanted sibling')
            (root/'newer_0.bag.active').write_bytes(b'active')
            self.assertEqual(c.select_bag_files(explicit, root), [explicit])

    def test_split_bags_are_read_in_bag_time_order_and_keep_inter_part_gap(self):
        starts = {'late.bag': 11., 'early.bag': 10.}
        messages = {'late.bag': [11., 11.1], 'early.bag': [10., 10.1]}
        open_modes = []
        class FakeBag:
            def __init__(self, path, skip_index=False):
                self.name = Path(path).name
                open_modes.append(skip_index)
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def get_start_time(self): return starts[self.name]
            def get_end_time(self): return messages[self.name][-1]
            def read_messages(self, **kwargs):
                for value in messages[self.name]:
                    yield '/poses', object(), SimpleNamespace(to_sec=lambda stamp=value: stamp)
        with mock.patch.dict(sys.modules, {'rosbag': SimpleNamespace(Bag=FakeBag)}), \
                mock.patch.object(c, 'pose_sample', side_effect=lambda msg, t: {'t': t}):
            rows = c.read_bags([Path('late.bag'), Path('early.bag')], '/poses')
        np.testing.assert_allclose([row['t'] for row in rows], [0., .1, 1., 1.1])
        self.assertEqual(open_modes, [True, True, False, False])
        with self.assertRaisesRegex(ValueError, 'Tracking gap'):
            c.summarize(rows, .3)

    def test_bag_interval_is_header_only(self):
        calls = []
        class HeaderBag:
            def __init__(self, path, skip_index=False): calls.append(skip_index)
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def get_start_time(self): return 12.5
            def get_end_time(self): return 19.75
            def read_messages(self, **kwargs): raise AssertionError('header helper read messages')
        with mock.patch.dict(sys.modules, {'rosbag': SimpleNamespace(Bag=HeaderBag)}):
            self.assertEqual(c._bag_interval('large.bag'), (12.5, 19.75))
        self.assertEqual(calls, [True])

    def test_known_direction_and_ten_cm_prior_recovered_with_quaternion_sign_flips(self):
        rows, transforms = samples()
        holds = c.choose_holds(rows, 10., None)
        result = c.calibrate(holds, distance=.1)
        for side in c.BONES:
            expected = -.1 * transforms[side].apply([0., -1., 0.])
            np.testing.assert_allclose(result[side]['palm_to_controller_offset'], expected, atol=1e-12)
            self.assertEqual(result[side]['eef_post_rotation_xyzw'], [0., 0., 0., 1.])
        self.assertFalse(result['metadata']['full_translation_observable'])
        self.assertTrue(result['metadata']['distance_is_prior_not_measured'])
        self.assertTrue(result['metadata']['translation_calibrated'])
        self.assertEqual(result['metadata']['known_distance_m'], .1)
        self.assertIsInstance(result['metadata']['known_distance_m'], float)
        self.assertEqual(result['metadata']['distance_source'], 'user_prior')
        self.assertEqual(result['metadata']['translation_status'], 'known_normal_distance_prior')

    def test_unknown_distance_orientation_only_never_invents_ten_cm(self):
        rows, _ = samples()
        holds = c.choose_holds(rows, 10., None)
        result = c.calibrate(holds, orientation=True)
        self.assertEqual(result['method'], 'semantic_three_pose_orientation_only')
        for side in c.BONES:
            np.testing.assert_array_equal(result[side]['palm_to_controller_offset'], [0., 0., 0.])
        metadata = result['metadata']
        self.assertIsNone(metadata.get('known_distance_m'))
        self.assertNotIn('known_distance_m', metadata)
        self.assertFalse(metadata['translation_calibrated'])
        self.assertFalse(metadata['distance_is_prior_not_measured'])
        self.assertFalse(metadata['full_translation_observable'])
        self.assertEqual(metadata['distance_source'], 'unknown')
        self.assertEqual(metadata['translation_status'], 'not_identifiable_from_recording')

    def test_unknown_distance_candidate_yaml_is_ros_xmlrpc_serializable(self):
        rows, _ = samples()
        result = c.calibrate(c.choose_holds(rows, 10., None), orientation=True)
        document = {'quest3': {'controller_calibration': result}}
        with tempfile.TemporaryDirectory() as tmp:
            candidate = Path(tmp)/'candidate.yaml'
            c.atomic_yaml(candidate, document)
            loaded = c.yaml.safe_load(candidate.read_text())
        xmlrpc.client.dumps((loaded,))
        metadata = loaded['quest3']['controller_calibration']['metadata']
        self.assertNotIn('known_distance_m', metadata)
        self.assertEqual(metadata['distance_source'], 'unknown')
        self.assertFalse(metadata['translation_calibrated'])
        self.assertNotEqual(metadata.get('known_distance_m'), .1)

    def test_body_yaw_does_not_change_mounting_vector(self):
        results = []
        for yaw in (0., 1.2):
            rows, _ = samples(yaw)
            results.append(c.calibrate(c.choose_holds(rows, 10., None), distance=.1))
        for side in c.BONES:
            np.testing.assert_allclose(results[0][side]['palm_to_controller_offset'], results[1][side]['palm_to_controller_offset'], atol=1e-12)

    def test_optional_semantic_orientation_recovers_post_rotation(self):
        rows, transforms = samples()
        result = c.calibrate(c.choose_holds(rows, 10., None), distance=.1, orientation=True)
        for side in c.BONES:
            base = Rotation.from_euler('xyz', [-np.pi/2, 0., -np.pi if side == 'left' else 0.]) * Rotation.from_euler('x', np.radians(-15 if side == 'left' else 15))
            expected = base.inv() * transforms[side]
            actual = Rotation.from_quat(result[side]['eef_post_rotation_xyzw'])
            self.assertLess((expected.inv()*actual).magnitude(), 1e-10)

    def test_two_pose_facing_down_protocol_recovers_consistent_mount(self):
        rows, transforms = samples()
        two_rows = [row for row in rows if row['t'] >= 16.]
        labels = ('facing', 'down')
        holds = c.choose_holds(two_rows, 10., None, labels)
        result = c.calibrate(holds, distance=.1, orientation=True, labels=labels)
        self.assertEqual(result['method'], 'semantic_two_pose_known_normal_offset')
        self.assertEqual(result['metadata']['labels'], list(labels))
        self.assertEqual(result['metadata']['pose_order'], 'facing,down')
        self.assertEqual(result['metadata']['method'], result['method'])
        for side in c.BONES:
            base = (Rotation.from_euler('xyz', [-np.pi/2, 0., -np.pi if side == 'left' else 0.]) *
                    Rotation.from_euler('x', np.radians(-15 if side == 'left' else 15)))
            actual = Rotation.from_quat(result[side]['eef_post_rotation_xyzw'])
            self.assertLess(((base.inv()*transforms[side]).inv()*actual).magnitude(), 1e-10)

    def test_two_pose_rejects_inconsistent_mount_and_wrong_order(self):
        rows, _ = samples()
        holds = c.choose_holds(rows, 10., ['16:28', '32:44'], ('facing', 'down'))
        bad = [dict(hold) for hold in holds]
        bad[1] = dict(bad[1], left=dict(bad[1]['left']))
        bad[1]['left']['q'] = Rotation.from_euler('xyz', [.9, -.7, .4]).as_quat()
        with self.assertRaises(c.CalibrationRejected):
            c.calibrate(bad, distance=.1, labels=('facing', 'down'))
        with self.assertRaises(ValueError):
            c.calibrate(list(reversed(holds)), distance=.1, labels=('facing', 'down'))
        with self.assertRaisesRegex(ValueError, 'Unsupported pose order'):
            c.calibrate(holds, distance=.1, labels=('down', 'facing'))

    def test_two_pose_auto_selection_never_guesses_which_hold_to_drop(self):
        rows, _ = samples()
        with self.assertRaisesRegex(ValueError, 'provide exactly two --intervals'):
            c.choose_holds(rows, 10., None, ('facing', 'down'))

    def test_intervals_match_pose_count_and_are_time_ordered_non_overlapping(self):
        with self.assertRaisesRegex(ValueError, 'needs 2'):
            c.parse_intervals(['0:11', '12:23', '24:35'], ('facing', 'down'))
        for intervals in (['12:23', '0:11'], ['0:12', '11:23'], ['1:1', '12:23']):
            with self.subTest(intervals=intervals), self.assertRaises(ValueError):
                c.parse_intervals(intervals, ('facing', 'down'))
        self.assertEqual(c.parse_intervals(['0:11', '11:22'], ('facing', 'down')),
                         [(0., 11.), (11., 22.)])

    def test_cli_rejects_interval_count_before_opening_bag(self):
        result = subprocess.run([
            sys.executable, str(SCRIPT), '/definitely/missing.bag',
            '--pose-order', 'facing,down', '--intervals', '0:11', '12:23', '24:35',
        ], capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--intervals needs 2', result.stderr)

    def test_legacy_schema_and_numbers_remain_compatible(self):
        rows, _ = samples()
        result = c.calibrate(c.choose_holds(rows, 10., None), distance=.1, orientation=True,
                             convention='legacy')
        self.assertEqual(result['schema_version'], 1)
        self.assertEqual(result['method'], 'semantic_three_pose_known_normal_offset')
        self.assertEqual(result['metadata']['pose_order'], 'up,facing,down')
        self.assertNotIn('eef_convention', result)
        self.assertNotIn('robot_version', result)
        self.assertNotIn('model_sha256', result)

    @unittest.skipUnless(MODEL_HAS_SG100, 'this checkout has no SG100 links in its 400062 URDF')
    def test_sg100_400062_axes_come_from_real_urdf(self):
        model = c.inspect_sg100_400062_model(MODEL_400062)
        self.assertEqual(len(model['sha256']), 64)
        np.testing.assert_allclose(model['axes']['left']['finger_axis_eef'], [0, 0, -1], atol=1e-6)
        np.testing.assert_allclose(model['axes']['right']['finger_axis_eef'], [0, 0, -1], atol=1e-6)
        np.testing.assert_allclose(model['axes']['left']['palm_normal_axis_eef'], [0, -1, 0], atol=1e-6)
        np.testing.assert_allclose(model['axes']['right']['palm_normal_axis_eef'], [0, 1, 0], atol=1e-6)

    @unittest.skipUnless(MODEL_HAS_SG100, 'this checkout has no SG100 links in its 400062 URDF')
    def test_sg100_orientation_uses_model_specific_mirrored_axes(self):
        model = c.inspect_sg100_400062_model(MODEL_400062)
        rows, transforms = samples()
        holds = c.choose_holds(rows, 10., None)
        # Rebuild raw samples so their desired EEF pose obeys SG100's -Z
        # finger axis and mirrored right palm-normal axis.
        for label, hold in zip(c.LABELS, holds):
            for side in c.BONES:
                normal = np.array([0., 0., 1. if label == 'up' else -1.])
                if label == 'facing':
                    normal = hold['left_axis'] * (-1 if side == 'left' else 1)
                y = normal if side == 'right' else -normal
                z = -hold['forward']
                desired = Rotation.from_matrix(np.column_stack([np.cross(y, z), y, z]))
                hold[side]['q'] = (desired * transforms[side].inv()).as_quat()
        result = c.calibrate(holds, distance=.1, orientation=True,
                             convention='sg100_400062', model=model)
        self.assertEqual(result['schema_version'], 2)
        self.assertEqual(result['eef_convention'], 'sg100_400062')
        self.assertEqual(result['robot_version'], 400062)
        self.assertEqual(result['model_sha256'], model['sha256'])
        for side in c.BONES:
            base = (Rotation.from_euler('xyz', [-np.pi/2, 0., -np.pi if side == 'left' else 0.]) *
                    Rotation.from_euler('x', np.radians(-15 if side == 'left' else 15)))
            actual = Rotation.from_quat(result[side]['eef_post_rotation_xyzw'])
            expected = base.inv() * transforms[side]
            self.assertLess((expected.inv()*actual).magnitude(), 1e-10)

    @unittest.skipUnless(MODEL_HAS_SG100, 'this checkout has no SG100 links in its 400062 URDF')
    def test_sg100_unknown_translation_produces_valid_rotation_only_schema(self):
        model = c.inspect_sg100_400062_model(MODEL_400062)
        rows, transforms = samples()
        holds = c.choose_holds(rows, 10., None)
        for label, hold in zip(c.LABELS, holds):
            for side in c.BONES:
                normal = np.array([0., 0., 1. if label == 'up' else -1.])
                if label == 'facing':
                    normal = hold['left_axis'] * (-1 if side == 'left' else 1)
                y = normal if side == 'right' else -normal
                z = -hold['forward']
                desired = Rotation.from_matrix(np.column_stack([np.cross(y, z), y, z]))
                hold[side]['q'] = (desired * transforms[side].inv()).as_quat()
        result = c.calibrate(holds, orientation=True, convention='sg100_400062', model=model)
        self.assertEqual(result['schema_version'], 2)
        self.assertEqual(result['method'], 'semantic_three_pose_orientation_only')
        self.assertEqual(result['model_sha256'], model['sha256'])
        for side in c.BONES:
            self.assertEqual(result[side]['palm_to_controller_offset'], [0., 0., 0.])
        self.assertFalse(result['metadata']['translation_calibrated'])

    def test_unknown_distance_without_orientation_rejects_before_writing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = root/'active.yaml'
            output = root/'candidate.yaml'
            report = root/'report.json'
            config.write_text('precious: active\n')
            result = subprocess.run([
                sys.executable, str(SCRIPT), str(root/'missing.bag'),
                '--config', str(config), '--output', str(output), '--report', str(report),
            ], capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Translation distance is unknown', result.stderr)
            self.assertEqual(config.read_text(), 'precious: active\n')
            self.assertFalse(output.exists())
            diagnostic = c.json.loads(report.read_text())
            self.assertIsNone(diagnostic['input']['outward_distance'])
            self.assertEqual(diagnostic['input']['pose_order'], 'up,facing,down')

    def test_short_recording_rejected(self):
        rows, _ = samples()
        rows = [r for r in rows if r['t'] < 9]
        with self.assertRaises(ValueError): c.choose_holds(rows, 10., None)

    def test_multiple_valid_triples_rejected(self):
        rows, _ = samples()
        extra = [dict(row, t=row['t']+60.) for row in rows]
        with self.assertRaises(ValueError): c.choose_holds(rows+extra, 10., None)

    def test_bad_labels_or_mount_change_rejected(self):
        rows, _ = samples()
        holds = c.choose_holds(rows, 10., None)
        holds[1]['left']['q'] = Rotation.from_euler('xyz', [.4, .5, .7]).as_quat()
        with self.assertRaises(c.CalibrationRejected) as rejected:
            c.calibrate(holds, distance=.1)
        self.assertIn('left', rejected.exception.diagnostics['sides'])
        self.assertIn('normal_residual_deg', rejected.exception.diagnostics['sides']['left'])

    def test_tracking_gap_and_drift_rejected(self):
        rows, _ = samples()
        first = [r for r in rows if r['t'] <= 12.05]
        with self.assertRaises(ValueError): c.summarize([r for r in first if not 4 < r['t'] < 5], 10.)
        for row in first: row['left']['p'] = row['left']['p'] + [row['t']*.02, 0., 0.]
        with self.assertRaises(ValueError): c.summarize(first, 10.)

    def test_path_collision_rejected_before_reading_bag_or_writing_configuration(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp)/'source.bag'
            cfg = Path(tmp)/'active.yaml'
            bag.write_bytes(b'precious original bag')
            cfg.write_bytes(b'precious active configuration')
            cases = [ ['--output', str(bag), '--config', str(cfg), '--apply'],
                      ['--output', str(Path(tmp)/'candidate.yaml'), '--config', str(bag), '--apply'],
                      ['--output', str(cfg), '--config', str(cfg)] ]
            for args in cases:
                result = subprocess.run([sys.executable, str(SCRIPT), str(bag), *args],
                                        capture_output=True, text=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(bag.read_bytes(), b'precious original bag')
                self.assertEqual(cfg.read_bytes(), b'precious active configuration')

    def test_short_hold_apply_is_rejected_and_report_preserves_active_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp)/'missing.bag'
            cfg = Path(tmp)/'active.yaml'
            report = Path(tmp)/'diagnostic.json'
            output = Path(tmp)/'candidate.yaml'
            cfg.write_text('precious: active\n')
            result = subprocess.run([
                sys.executable, str(SCRIPT), str(bag), '--min-hold', '3', '--apply',
                '--config', str(cfg), '--output', str(output), '--report', str(report),
            ], capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(cfg.read_text(), 'precious: active\n')
            self.assertFalse(output.exists())
            diagnostic = c.json.loads(report.read_text())
            self.assertFalse(diagnostic['passed'])
            self.assertIn('--apply requires', diagnostic['error'])

    def test_legacy_pose_topic_requires_explicit_uncompensated_declaration(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = Path(tmp)/'diagnostic.json'
            result = subprocess.run([
                sys.executable, str(SCRIPT), str(Path(tmp)/'missing.bag'),
                '--topic', c.LEGACY_TOPIC, '--report', str(report),
            ], capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('may already contain compensation', c.json.loads(report.read_text())['error'])

    def test_default_active_config_follows_consumer_and_explicit_path_wins(self):
        legacy = c.resolve_active_config('legacy', script_path=SCRIPT)
        self.assertEqual(legacy, SCRIPT.resolve().parents[1] / 'config/quest_controller_calibration.yaml')
        explicit = Path('/tmp/operator-selected-calibration.yaml')
        self.assertEqual(c.resolve_active_config('sg100_400062', explicit, SCRIPT), explicit)
        wheel_package = SCRIPT.resolve().parents[3] / 'wheel_arm_abs_vel'
        if wheel_package.is_dir():
            self.assertEqual(c.resolve_active_config('sg100_400062', script_path=SCRIPT),
                             wheel_package / 'config/quest_controller_calibration.yaml')
        else:
            with self.assertRaisesRegex(ValueError, 'no full-body calibration consumer'):
                c.resolve_active_config('sg100_400062', script_path=SCRIPT)

    def test_sg100_activation_eligible_run_requires_explicit_orientation(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = Path(tmp)/'diagnostic.json'
            result = subprocess.run([
                sys.executable, str(SCRIPT), str(Path(tmp)/'missing.bag'),
                '--eef-convention', 'sg100_400062', '--model', str(MODEL_400062),
                '--config', str(Path(tmp)/'explicit-active.yaml'), '--report', str(report),
            ], capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('requires --calibrate-orientation',
                          c.json.loads(report.read_text())['error'])

    def test_report_path_conflicts_never_overwrite_any_protected_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for conflict in ('bag', 'output', 'config'):
                with self.subTest(conflict=conflict):
                    bag = root/(conflict + '-source.bag')
                    output = root/(conflict + '-candidate.yaml')
                    config = root/(conflict + '-active.yaml')
                    paths = {'bag': bag, 'output': output, 'config': config}
                    for name, path in paths.items():
                        path.write_bytes(('precious-' + name).encode())
                    before = {name: path.read_bytes() for name, path in paths.items()}
                    result = subprocess.run([
                        sys.executable, str(SCRIPT), str(bag), '--output', str(output),
                        '--config', str(config), '--report', str(paths[conflict]),
                    ], capture_output=True, text=True, timeout=10)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn('--report must differ', result.stderr)
                    self.assertEqual({name: path.read_bytes() for name, path in paths.items()}, before)

    def test_tracking_failure_between_downsampled_frames_is_retained(self):
        class FakeBag:
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def get_start_time(self): return 0.
            def get_end_time(self): return .06
            def read_messages(self, **kwargs):
                for t, valid in ((0., True), (.01, False), (.02, True), (.06, True)):
                    yield '/poses', valid, SimpleNamespace(to_sec=lambda value=t: value)
        with mock.patch.dict(sys.modules, {'rosbag': SimpleNamespace(
                Bag=lambda path, **kwargs: FakeBag())}), \
                mock.patch.object(c, 'pose_sample', side_effect=lambda valid, t: {'t': t} if valid else None):
            rows = c.read_bag('unused.bag', '/poses')
        self.assertTrue(any(row.get('invalid') and row['t'] == .01 for row in rows))

    def test_atomic_configuration_has_backup(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'active.yaml'
            c.atomic_yaml(path, {'quest3': {'controller_calibration': {'enabled': False}}})
            before = path.read_bytes()
            c.atomic_yaml(path, {'quest3': {'controller_calibration': {'enabled': True}}})
            backups = list(Path(tmp).glob('*.before-*'))
            self.assertEqual(len(backups), 1)
            self.assertEqual(backups[0].read_bytes(), before)
            self.assertTrue(c.yaml.safe_load(path.read_text())['quest3']['controller_calibration']['enabled'])


if __name__ == '__main__': unittest.main()
