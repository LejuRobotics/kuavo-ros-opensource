"""End-effector selection and controller/Mapper agreement across machines."""
import contextlib
import importlib.util
import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('absolute_end_effector', ROOT / 'scripts/absolute_end_effector.py')
ee = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ee)


class SelectionTest(unittest.TestCase):
    def contract(self, version, real, override='auto'):
        model = ROOT.parent / 'kuavo_assets/config' / ('kuavo_v%d' % version) / 'kuavo.json'
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(ee.main(['--config', str(model), '--machine-mode', str(real).lower(),
                                      '--ee-type', override, '--contract-version', str(version)]), 0)
        return json.loads(output.getvalue())

    def test_auto_uses_model_even_with_stale_machine_inventory(self):
        devices = [{'name': side + 'hand', 'class': 'revo1_hand', 'device_id': address,
                    'ignore': False} for side, address in [('L', 17), ('R', 18)]]
        with tempfile.TemporaryDirectory() as directory, patch.dict(
                os.environ, KUAVO_MACHINE_CONFIG_DIR=directory):
            for filename in ('canbus_device_cofig.yaml', 'canbus_device_config.yaml'):
                (Path(directory) / filename).write_text(json.dumps({'canbus0_devices': devices}))
            for version, expected in [(410062, 'heiman'), (210062, 'lejuclaw'), (310062, 'qiangnao')]:
                for real in (True, False):
                    with self.subTest(version=version, real=real):
                        self.assertEqual(self.contract(version, real), {
                            'robot_version': version, 'real': real, 'types': [expected] * 2,
                            'source': 'model kuavo.json'})

    def test_explicit_controller_choice_is_authoritative(self):
        for kind in sorted(ee.SUPPORTED):
            for real in (True, False):
                with self.subTest(kind=kind, real=real):
                    contract = self.contract(410062, real, kind)
                    self.assertEqual(contract['source'], 'explicit ee_type')
                    self.assertEqual(contract['types'], [kind, kind])
                    for mapper in ('auto', kind):
                        self.assertEqual(ee.validate_contract(json.dumps(contract), 410062, real, mapper), kind)

    def test_incompatible_controller_contract_is_rejected(self):
        contract = json.dumps(self.contract(410062, True))
        for version, real, override in [(62, True, 'auto'), (410062, False, 'auto'),
                                        (410062, True, 'qiangnao')]:
            with self.subTest(version=version, real=real, override=override), self.assertRaises(ValueError):
                ee.validate_contract(contract, version, real, override)


if __name__ == '__main__':
    unittest.main()
