import io, contextlib
import importlib.util, json, os, tempfile, unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('absolute_end_effector',ROOT/'scripts/absolute_end_effector.py')
ee=importlib.util.module_from_spec(spec);spec.loader.exec_module(ee)
class MachineInventoryTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.config={'EndEffectorType':['qiangnao','qiangnao']}
    def inventory(self, entries, name='canbus_device_cofig.yaml'):
        import yaml
        path=self.root/name
        path.write_text(yaml.safe_dump({'canbus0_devices':entries}))
        return path
    def devices(self,kind='lejuclaw'):
        return [{'name':'Lhand','class':kind,'device_id':17,'ignore':False},
                {'name':'Rhand','class':kind,'device_id':18,'ignore':False}]
    def test_machine_claws_override_all_model_defaults_without_edits(self):
        self.inventory(self.devices())
        for kind in ('qiangnao','heiman','lejuclaw','none'):
            c={'EndEffectorType':[kind,kind]}
            pair,source=ee.select(c,use_machine=True,directory=self.root)
            self.assertEqual(pair,['lejuclaw']*2)
            self.assertTrue(source.endswith('canbus_device_cofig.yaml'))
            self.assertEqual(c,{'EndEffectorType':[kind,kind]})
            self.assertEqual(ee.select(c,use_machine=False,directory=self.root)[0],[kind]*2)
    def test_sdk_filename_precedence_and_hand_classes(self):
        self.inventory(self.devices('lejuclaw'))
        for cls,kind in ee.CAN_TYPES.items():
            self.inventory(self.devices(cls),'canbus_device_config.yaml')
            self.assertEqual(ee.select(self.config,use_machine=True,directory=self.root)[0],[kind]*2)
    def test_missing_or_ignored_inventory_retains_model_selection(self):
        self.assertEqual(ee.select(self.config,use_machine=True,directory=self.root)[0],['qiangnao']*2)
        devices=self.devices()
        for device in devices:device['ignore']=True
        self.inventory(devices)
        self.assertEqual(ee.select(self.config,use_machine=True,directory=self.root)[0],['qiangnao']*2)
    def test_partial_mixed_duplicate_and_malformed_inventories_rejected(self):
        cases=[self.devices()[:1],self.devices()+self.devices(),
               [self.devices()[0],self.devices('revo2_hand')[1]]]
        for devices in cases:
            self.inventory(devices)
            with self.assertRaises(ValueError):ee.select(self.config,use_machine=True,directory=self.root)
        devices=self.devices();devices[0]['ignore']='false';self.inventory(devices)
        with self.assertRaises(ValueError):ee.select(self.config,use_machine=True,directory=self.root)
    def test_explicit_override_and_simulation_do_not_probe_machine(self):
        with patch.object(ee,'machine_pair',side_effect=AssertionError('unwanted machine read')):
            self.assertEqual(ee.select(self.config,'heiman',True)[0],['heiman']*2)
            self.assertEqual(ee.select(self.config,use_machine=False)[0],['qiangnao']*2)
    def test_controller_contract_and_mapper_cli_agree(self):
        self.inventory(self.devices());config=self.root/'kuavo.json'
        config.write_text(json.dumps(self.config))
        with patch.dict(ee.os.environ,{'KUAVO_MACHINE_CONFIG_DIR':str(self.root)}):
            for version in (60,61,62,63,200062,300062,400062,400063):
                output=io.StringIO()
                with contextlib.redirect_stdout(output):
                    self.assertEqual(ee.main(['--config',str(config),'--machine-mode','true',
                                             '--contract-version',str(version)]),0)
                contract=json.loads(output.getvalue())
                self.assertEqual(contract['robot_version'],version)
                self.assertEqual(contract['types'],['lejuclaw']*2)
            output=io.StringIO()
            with contextlib.redirect_stdout(output):
                ee.main(['--config',str(config),'--machine-mode','true'])
            self.assertEqual(output.getvalue(),'lejuclaw')

class ContractTest(unittest.TestCase):
    def test_model_mode_and_override(self):
        text=json.dumps(dict(robot_version=400062,real=True,types=['heiman']*2,source='explicit'))
        self.assertEqual(ee.validate_contract(text,400062,True),'heiman')
        self.assertEqual(ee.validate_contract(text,400062,True,'HEIMAN'),'heiman')
        for version,real,override in [(62,True,'auto'),(400062,False,'auto'),(400062,True,'qiangnao')]:
            with self.assertRaises(ValueError):ee.validate_contract(text,version,real,override)

if __name__=='__main__':unittest.main()
