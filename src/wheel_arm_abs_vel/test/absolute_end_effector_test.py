#!/usr/bin/env python3
"""Config/override and SG100 process selection without ROS or hardware."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('absolute_end_effector', ROOT/'scripts/absolute_end_effector.py')
ee = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ee)


class EndEffectorTest(unittest.TestCase):
    def test_each_model_can_select_each_requested_end_effector(self):
        for version in (62, 63, 400062):
            config = json.loads((ROOT.parent/'kuavo_assets/config'/('kuavo_v%d' % version)/'kuavo.json').read_text())
            self.assertEqual(ee.resolve(config), config['EndEffectorType'][0])
            for kind in ('qiangnao', 'heiman', 'lejuclaw'):
                with self.subTest(version=version, kind=kind):
                    self.assertEqual(ee.resolve(dict(config, EndEffectorType=[kind, kind])), kind)

    def test_override_and_normalization(self):
        self.assertEqual(ee.resolve({}, 'heiman'), 'heiman')
        self.assertEqual(ee.resolve({'EndEffectorType': ['HEIMAN', ' heiman ']}), 'heiman')
        for kind in ee.SUPPORTED:
            self.assertEqual(ee.resolve({'EndEffectorType': [kind, kind]}), kind)

    def test_invalid_or_asymmetric_config_is_not_silently_qiangnao(self):
        for pair in (None, [], ['qiangnao'], 'qiangnao', [None, None],
                     ['unknown', 'unknown'], ['qiangnao', 'heiman']):
            with self.subTest(pair=pair), self.assertRaises(ValueError):
                ee.resolve({'EndEffectorType': pair})
        with self.assertRaises(ValueError):
            ee.resolve({}, 'unknown')

    def test_bridge_executes_only_for_heiman_and_preserves_remaps(self):
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder)/'kuavo.json'
            for kind in ('qiangnao', 'lejuclaw', 'heiman'):
                config.write_text(json.dumps({'EndEffectorType': [kind, kind]}))
                with patch.object(ee.os, 'execvp') as execute:
                    self.assertEqual(ee.main(['--config', str(config), '--bridge',
                                              '__name:=sg100_mapper_bridge', '__log:=/tmp/log']), 0)
                    if kind == 'heiman':
                        execute.assert_called_once_with('rosrun', [
                            'rosrun', 'wheel_arm_abs_vel', 'sg100_mapper_bridge',
                            '__name:=sg100_mapper_bridge', '__log:=/tmp/log'])
                    else:
                        execute.assert_not_called()
            config.write_text('{broken')
            with patch.object(ee.os, 'execvp') as execute:
                self.assertEqual(ee.main(['--config', str(config), '--bridge']), 2)
                execute.assert_not_called()

    def test_bridge_override_is_authoritative_for_every_config(self):
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder)/'kuavo.json'
            for configured in ('qiangnao', 'heiman', 'lejuclaw'):
                config.write_text(json.dumps({'EndEffectorType': [configured, configured]}))
                for override in ('auto', 'qiangnao', 'heiman', 'lejuclaw', 'HEIMAN'):
                    expected = configured if override == 'auto' else override.lower()
                    with self.subTest(configured=configured, override=override):
                        with patch.object(ee.os, 'execvp') as execute:
                            self.assertEqual(ee.main(['--config', str(config), '--bridge',
                                                      '--ee-type', override]), 0)
                            self.assertEqual(execute.called, expected == 'heiman')

    def test_mapper_uses_same_selection_and_limits_heiman_flag(self):
        source = (ROOT/'scripts/ik_ros_velocity.py').read_text()
        self.assertIn('resolve(model_config, args.end_effector_type)', source)
        self.assertIn('and end_effector_type == "heiman"', source)
        self.assertNotIn('end_effector_mapping.get(', source)


if __name__ == '__main__':
    unittest.main()
