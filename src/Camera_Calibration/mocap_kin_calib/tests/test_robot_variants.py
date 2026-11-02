"""Wheel variant routing and installation; all writes stay in temporary copies."""

import argparse
from contextlib import redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np
import yaml

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import mocap_optimize_1010 as optimizer
import robot_variants as variants
import urdf_model_sync as sync
import write_urdf as installer
import cam_proj_validation as projection
from test_urdf_model_sync import fk

EXISTING = ('62', '63', '200062', '210062', '300062', '310062', '400062', '400063', '500062', '500063', '600062')


def without_origins(data):
    for start, end in sorted(sync._origin_spans(data).values(), reverse=True):
        data = data[:start] + data[end:]
    return data


class VariantTests(unittest.TestCase):
    def test_fit_keeps_full_version_model_metadata_and_output(self):
        cfg = yaml.safe_load((HERE / 'config/calib_s63.yaml').read_text())
        for version in (*EXISTING, '200063', '300063'):
            with self.subTest(version=version), patch.dict(os.environ, ROBOT_VERSION=version):
                result = optimizer._config_for_capture(cfg, {'robot_version': version})
                self.assertEqual(result['robot']['robot_version'], version)
                self.assertEqual(result['kinematics']['source_urdf'], str(variants.main_urdf(version)))
                self.assertEqual(result['kinematics']['output_dir'], '../output_s' + version)
                self.assertEqual(variants.default_config_name(version), 'calib_s63.yaml')
                self.assertEqual(result['robot']['joint_indices']['left_arm'], [4, 11])
        with patch.dict(os.environ, ROBOT_VERSION='500063'):
            with self.assertRaisesRegex(ValueError, 'conflicts'):
                optimizer._config_for_capture(cfg, {'robot_version': '63'})
        for invalid in ('63.5', 'abc', '500056', '120062', '210063', '162', ''):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                variants.default_config_name(invalid)
        biped = yaml.safe_load((HERE / 'config/calib_s56.yaml').read_text())
        for version in ('53', '56'):
            with patch.dict(os.environ, ROBOT_VERSION=version):
                self.assertEqual(optimizer._config_for_capture(biped, {})['kinematics']['source_urdf'],
                                 str(variants.main_urdf(version)))
        with patch.dict(os.environ, ROBOT_VERSION='500063'):
            with self.assertRaisesRegex(ValueError, 'CTB/ETK'):
                optimizer._config_for_capture(biped, {})

    def test_capture_adapter_owns_config_and_restores_colleague_hooks(self):
        spec = importlib.util.spec_from_file_location('kin_recorder', HERE / 'record_joint_poses.py')
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)
        original_loader = Mock(side_effect=AssertionError('colleague config loader must not run'))
        original_model = Mock(side_effect=AssertionError('colleague model defaults must not run'))
        # No default mapping or version resolver: neither is part of our contract.
        recorder = SimpleNamespace(
            _load_config=original_loader,
            _apply_robot_calibration_defaults=original_model,
        )
        modules = {'mocap_joint_calib': SimpleNamespace(record_joint_poses=recorder)}
        with tempfile.TemporaryDirectory() as temp:
            custom = Path(temp) / 'custom.yaml'
            cfg = yaml.safe_load((HERE / 'config/calib_s63.yaml').read_text())
            cfg['robot']['collect_sec'] = 9.25
            cfg['robot']['robot_layout'] = 'biped45'
            cfg['calibration']['fk_root'] = 'old_config_root'
            custom.write_text(yaml.safe_dump(cfg))
            custom_urdf = Path(temp) / 'custom.urdf'
            custom_json = Path(temp) / 'custom.json'
            for version, explicit in ((version, explicit)
                                      for version in ('200062', '210062', '310062')
                                      for explicit in ([], ['--config', str(custom), '--urdf', str(custom_urdf),
                                                           '--kuavo-json', str(custom_json), '--fk-root', 'custom_root'])):
                shutil.copyfile(variants.main_urdf(version), custom_urdf)
                shutil.copyfile(variants.ASSETS / f'config/kuavo_v{version}/kuavo.json', custom_json)
                argv = [str(HERE / 'record_joint_poses.py'), '--robot-version', version,
                        '--fixture', 'new', '--output', str(Path(temp) / 'capture.json'), *explicit]
                def run_recorder():
                    self.assertEqual(os.environ['ROBOT_VERSION'], '500063')
                    self.assertEqual(sys.argv[:len(argv)], argv)
                    parser = argparse.ArgumentParser()
                    for flag in ('robot-version', 'config', 'kuavo-json', 'urdf', 'robot-layout', 'fk-root'):
                        parser.add_argument('--' + flag)
                    options, _ = parser.parse_known_args()
                    self.assertEqual(options.robot_version, version)
                    self.assertEqual(Path(options.config), custom if explicit else HERE / 'config/calib_s63.yaml')
                    self.assertEqual(Path(options.urdf), custom_urdf if explicit else variants.main_urdf(version))
                    self.assertEqual(Path(options.kuavo_json), custom_json if explicit else variants.ASSETS / f'config/kuavo_v{version}/kuavo.json')
                    loaded = recorder._load_config('/colleague/changed/default.yaml')
                    self.assertEqual(loaded['robot']['collect_sec'], 9.25 if explicit else 1.0)
                    self.assertEqual(loaded['robot']['robot_version'], version)
                    for key in ('teach_left', 'teach_right', 'teach_head'):
                        self.assertTrue(Path(loaded['calibration'][key]).is_absolute())
                    model = recorder._apply_robot_calibration_defaults(loaded['robot'], loaded['calibration'], version)
                    self.assertEqual(model, {'robot_layout': 'wheel62', 'urdf': options.urdf,
                                             'fk_root': 'custom_root' if explicit else 'waist_yaw_link'})
                    loaded['robot']['collect_sec'] = -1
                    self.assertGreater(recorder._load_config('ignored')['robot']['collect_sec'], 0)
                    return 7
                recorder.main = run_recorder
                with self.subTest(version=version, explicit=bool(explicit)), patch.dict(sys.modules, modules), \
                        patch.dict(os.environ, ROBOT_VERSION='500063'), patch.object(sys, 'argv', argv):
                    self.assertEqual(entry.main(), 7)
                    self.assertIs(recorder._load_config, original_loader)
                    self.assertIs(recorder._apply_robot_calibration_defaults, original_model)
                    self.assertIs(sys.argv, argv)
                    recorder.main = Mock(side_effect=KeyboardInterrupt)
                    with self.assertRaises(KeyboardInterrupt):
                        entry.main()
                    self.assertIs(recorder._load_config, original_loader)
                    self.assertIs(recorder._apply_robot_calibration_defaults, original_model)
                    self.assertIs(sys.argv, argv)
        original_loader.assert_not_called()
        original_model.assert_not_called()

    def prepare(self, folder, version):
        target = folder / 'urdf' / f'biped_s{version}.urdf'
        shutil.copytree(variants.main_urdf(version).parent, target.parent)
        tree = optimizer._xml_tree(target)
        tree.getroot().insert(0, ET.Comment(' mocap_kin_calib:{"schema_version":1} '))
        for i, name in enumerate(sync._JOINT_NAMES, 1):
            origin = tree.find(f"./joint[@name='{name}']/origin")
            xyz = np.fromstring(origin.get('xyz'), sep=' ') + [i * .00001, 0, 0]
            origin.set('xyz', ' '.join(map(str, xyz)))
            origin.set('rpy', '.001 -.002 .003')
        candidate = folder / 'candidate.urdf'
        tree.write(candidate, encoding='utf-8', xml_declaration=True)
        return target, candidate

    def test_all_existing_variants_install_own_models_and_preserve_non_arm_content(self):
        poses = np.random.default_rng(62).uniform(-1, 1, (5, 7))
        for version in EXISTING:
            with self.subTest(version=version), tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
                folder = Path(temp)
                target, candidate = self.prepare(folder, version)
                before = {p: p.read_bytes() for p in target.parent.rglob('*') if p.is_file()}
                updates = sync.build_model_updates(target, candidate)
                self.assertEqual(set(updates), {p for p in before if p.suffix in ('.urdf', '.xacro')})
                installer.install_urdf_directory(target, candidate, apply=False)
                self.assertEqual(before, {p: p.read_bytes() for p in before})
                for answer in ('no', ''):
                    with patch('builtins.input', return_value=answer):
                        optimizer._confirm_apply_urdf(target, candidate, target)
                    self.assertEqual(before, {p: p.read_bytes() for p in before})
                self.assertEqual(list(folder.glob('urdf.bak.*')), [])
                with patch('builtins.input', return_value='yes'):
                    optimizer._confirm_apply_urdf(target, candidate, target)
                nominal = installer.nominal_backup_path(target)
                self.assertEqual(nominal.read_bytes(), before[target])
                self.assertEqual(installer.resolve_fit_source(target), nominal)
                for path, expected in updates.items():
                    self.assertEqual(path.read_bytes(), expected)
                    if path == target or path.name in ('Larm.urdf', 'Rarm.urdf'):
                        continue
                    if path.suffix == '.xacro':
                        self.assertEqual(expected, before[path])
                        continue
                    comment = sync._metadata_comment(candidate.read_bytes(), candidate)
                    self.assertEqual(without_origins(expected.replace(b'\n  ' + comment, b'', 1)),
                                     without_origins(before[path]))
                    for side in 'lr':
                        for q in poses:
                            np.testing.assert_allclose(fk(expected, side, q), fk(candidate.read_bytes(), side, q), atol=1e-13, rtol=0)
                for path in set(before) - set(updates):
                    self.assertEqual(path.read_bytes(), before[path])
                installer.install_urdf_directory(target, candidate, apply=True)
                self.assertEqual(len(list(folder.glob('urdf.bak.*'))), 1)
                self.assertEqual(nominal.read_bytes(), before[target])

    def test_cross_version_candidate_is_rejected_before_writing(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            folder = Path(temp)
            target, candidate = self.prepare(folder / 'small', '500063')
            _, other = self.prepare(folder / 'base', '63')
            before = target.read_bytes()
            with self.assertRaisesRegex(ValueError, 'robot name differs'):
                installer.install_urdf_directory(target, other, apply=True)
            self.assertEqual(target.read_bytes(), before)
            self.assertEqual(list(target.parent.parent.glob('urdf.bak.*')), [])

    def test_variant_partial_install_failure_restores_all_models(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            folder = Path(temp)
            target, candidate = self.prepare(folder, '500063')
            before = {p: p.read_bytes() for p in target.parent.rglob('*') if p.is_file()}
            replace = os.replace
            calls = []
            def fail_third(src, dst):
                calls.append(Path(dst))
                if len(calls) == 3:
                    raise OSError('injected variant install failure')
                return replace(src, dst)
            with patch.object(installer.os, 'replace', side_effect=fail_third):
                with self.assertRaisesRegex(RuntimeError, '已回滚'):
                    installer.install_urdf_directory(target, candidate, apply=True)
            self.assertEqual(before, {p: p.read_bytes() for p in target.parent.rglob('*') if p.is_file()})
            self.assertFalse(installer.nominal_backup_path(target).exists())

    def test_optimizer_exports_and_installs_variant_from_its_own_nominal(self):
        import test_mocap_output_limits as limits

        limits.OutputLimitTests.setUpClass()
        module = next(m for m in limits.MODULES if m.__name__.endswith('1010'))
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            folder = Path(temp)
            target, _ = self.prepare(folder / 'models/biped_s500063', '500063')
            original = target.read_bytes()
            config, capture, output = limits.OutputLimitTests().make_inputs(folder, module)
            # The mocked fits below use the shared-CTB model's parameter layout.
            cfg = yaml.safe_load(config.read_text())
            cfg['kinematics']['ctb']['mode'] = 'shared'
            config.write_text(yaml.safe_dump(cfg))
            data = json.loads(capture.read_text())
            data['meta']['robot_version'] = '500063'
            capture.write_text(json.dumps(data))
            zero = np.zeros(limits.OutputLimitTests.models[module].parameter_count)
            fitted = zero.copy()
            fitted[9] = 1.0
            success = SimpleNamespace(success=True, message='synthetic fit', nfev=1)
            argv = [module.__file__, str(capture), '--config', str(config), '--output-urdf', '--force-output']
            first = None
            for _ in range(2):
                with patch.object(variants, 'ASSETS', folder), patch.dict(os.environ, ROBOT_VERSION='500063'), \
                        patch.object(sys, 'argv', argv), patch('builtins.input', return_value='yes'), \
                        patch.object(module, 'fit', side_effect=[(zero, success), (fitted, success)]):
                    module.main()
                self.assertEqual(target.read_bytes(), (output / 'urdf_final.urdf').read_bytes())
                self.assertEqual(installer.nominal_backup_path(target).read_bytes(), original)
                self.assertFalse((folder / 'models/biped_s63').exists())
                if first is not None:
                    self.assertEqual(target.read_bytes(), first)
                first = target.read_bytes()

    def test_variant_camera_mount_requires_actual_tf(self):
        path = variants.main_urdf('500063')
        model = projection.ProjectionModel(path, camera_link='camera_base', root_link='waist_yaw_link', points_per_link=8)
        extrinsics = projection.CameraExtrinsics(model, '500063', mount_frame='camera')
        graph = projection.StaticTransforms()
        graph.update('head_camera_link', 'optical', np.eye(4))
        with self.assertRaisesRegex(ValueError, 'published camera mount'):
            extrinsics.resolve(graph, 'optical')
        mount = np.eye(4)
        mount[:3, 3] = [.01, -.02, .03]
        graph.update('camera', 'head_camera_link', mount)
        actual, source = extrinsics.resolve(graph, 'optical')
        np.testing.assert_allclose(actual, extrinsics.fixed.lookup('camera_base', 'camera') @ mount)
        self.assertEqual(source, 'published mount TF')


if __name__ == '__main__':
    unittest.main()
