#!/usr/bin/env python3
"""Validate the test-branch resolver against private models and broken fixtures."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from tools.full_model_contract import resolve_model
ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT.parent/'kuavo_assets'
WHEEL = ROOT.parent/'humanoid-wheel-control/humanoid_wheel_interface'

class SharedModels(unittest.TestCase):
    def test_supported_full_models_and_private_limits(self):
        for version in (62, 63, 400062):
            overlay = json.loads((ROOT/f'config/model_s{version}.json').read_text())
            contract = resolve_model(version, ASSETS, WHEEL, overlay)
            self.assertEqual(contract['robot_version'], version)
            self.assertEqual(contract['velocity_ik_urdf'], overlay['velocity_ik_urdf'])
            self.assertEqual(set(contract['velocity_ik_fixed_joints']), set(overlay['velocity_ik_fixed_joints']))
            for joint, limits in overlay.get('velocity_ik_position_limits', {}).items():
                self.assertEqual(contract['velocity_ik_position_limits'][joint], limits)
            # The shared resolver must never produce a gain override.
            self.assertFalse({'kp','kd','ruiwo_kp','ruiwo_kd'} & contract.keys())
            for change in ({'velocity_ik_urdf': 'biped_s63/urdf/biped_s63.urdf' if version != 63 else 'biped_s62/urdf/biped_s62.urdf'},
                           {'velocity_ik_fixed_joints': []},
                           {'velocity_ik_position_limits': {'zarm_l3_joint': [3, -3]}},
                           {'ruiwo_kd': [18]*14}):
                with self.subTest(version=version, change=change), self.assertRaises(ValueError):
                    resolve_model(version, ASSETS, WHEEL, dict(overlay, **change))

    def test_shared_asset_validation_before_launch(self):
        original = json.loads((ASSETS/'config/kuavo_v63/kuavo.json').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            assets = Path(tmp)
            (assets/'models').symlink_to(ASSETS/'models', target_is_directory=True)
            path = assets/'config/kuavo_v63/kuavo.json'
            path.parent.mkdir(parents=True)
            for update in ({'NUM_ARM_JOINT': 12}, {'end_frames_name_ik': ['missing']},
                           {'shoulder_frame_names': ['missing', 'missing']}):
                path.write_text(json.dumps(dict(original, **update)))
                with self.subTest(update=update), self.assertRaises(ValueError):
                    resolve_model(63, assets, WHEEL)
        for value in ('../63', 0, '63.0', 400063):
            with self.subTest(version=value), self.assertRaises(ValueError):
                resolve_model(value, ASSETS, WHEEL)

if __name__ == '__main__':
    unittest.main()
