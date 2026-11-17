"""All installed wheel models and end-effector contracts; no device access."""
import json
import sys
import unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from tools.full_model_contract import resolve_model
from tools.target_geometry_contract import neutral_retarget_geometry, model_geometry
from tools.incremental_clutch import HandClutch
from tools.incremental_controls import RotationTransfer
from tools.pose_velocity_kalman import exp_so3
from absolute_end_effector import SUPPORTED, resolve
ASSETS=ROOT.parents[1]/'kuavo_assets'
WHEEL=ROOT.parents[1]/'humanoid-wheel-control/humanoid_wheel_interface'

class VariantMatrix(unittest.TestCase):
    def test_every_installed_wheel_model_and_effector(self):
        tested=[]
        for folder in sorted((WHEEL/'config').glob('kuavo_s*')):
            version=folder.name.split('_s')[-1]
            config=json.loads((ASSETS/f'config/kuavo_v{version}/kuavo.json').read_text())
            # The biped entry sharing this directory is outside the 18-DoF
            # wheel contract; it must remain rejected, never mapped to S62.
            if config.get('NUM_JOINT',0)-config.get('NUM_HEAD_JOINT',0)-14 != 4:
                with self.assertRaises(ValueError):resolve_model(version,ASSETS,WHEEL)
                continue
            overlay=ROOT/f'config/model_s{version}.json'
            contract=resolve_model(version,ASSETS,WHEEL,json.loads(overlay.read_text()) if overlay.exists() else {})
            model=ASSETS/'models'/contract['velocity_ik_urdf']
            geometry=neutral_retarget_geometry(model,config['end_frames_name_ik'][0],
                config['shoulder_frame_names'],config['end_frames_name_ik'][3:5])
            tools=model_geometry(model,config['end_frames_name_ik'][1:3],{s:np.eye(4) for s in ('Left','Right')})
            for kind in sorted(SUPPORTED):
                with self.subTest(version=version,effector=kind):
                    self.assertEqual(resolve(config,kind),kind)
                    for side in ('Left','Right'):
                        shoulder=np.array(geometry[side.lower()+'_shoulder'])
                        self.assertGreater(geometry['upper_arm_length'],.05)
                        # Physical configured tool transform, not a fixed S62
                        # 193-mm lever. Includes either orientation convention.
                        offset=np.array(tools['arms'][side]['joint7_to_tcp'])[:3,3]
                        for incremental in (False,True):
                            c=HandClutch([1.26]*3,[1.]*3,incremental,2.)
                            z=np.zeros(3);eye=np.eye(3);held=shoulder+np.array([.25,0.,-.1])
                            c.sample(1,1,False,z,eye,z,z,(held,eye),shoulder,0.)
                            p,R,v,w=c.sample(1,2,True,np.ones(3),exp_so3(np.array([2.,-.3,.4])),z,z,
                                (held-np.array([0,0,.03]),eye),shoulder,.01)
                            np.testing.assert_allclose(p,held)
                            np.testing.assert_allclose(R,eye)
                            np.testing.assert_array_equal(v,z);np.testing.assert_array_equal(w,z)
                            transfer=RotationTransfer();transfer.engage(R,0.)
                            for t in (.049,.05,.058,.15,.5):
                                rot,_=transfer.step(exp_so3(np.array([.3,-.4,2.8])),z,t)
                                tcp=p-rot@offset
                                np.testing.assert_allclose(tcp+rot@offset,p,atol=1.e-12)
            tested.append(version)
        self.assertEqual(set(tested), {'60','61','62','63','200062','200063',
            '210062','300062','300063','310062','400062','400063',
            '500062','500063','600062'})
        self.assertIn('400062',tested)
        print('VARIANT_MATRIX models='+','.join(tested)+' effectors='+','.join(sorted(SUPPORTED)))

if __name__=='__main__':unittest.main()
