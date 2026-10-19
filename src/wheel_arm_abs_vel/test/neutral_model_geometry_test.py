#!/usr/bin/env python3
"""Canonical robot geometry tests; no ROS, retarget input, or hardware access."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'scripts'))
from tools.target_geometry_contract import neutral_retarget_geometry, model_geometry
ASSETS=PACKAGE.parent/'kuavo_assets'
class GeometryTest(unittest.TestCase):
    version = '62'
    @classmethod
    def setUpClass(cls):
        cls.config=json.loads((ASSETS/f'config/kuavo_v{cls.version}/kuavo.json').read_text())
        cls.config.update(json.loads((PACKAGE/f'config/model_s{cls.version}.json').read_text()))
        cls.canonical=ASSETS/'models'/cls.config['velocity_ik_urdf']
        cls.legacy=ASSETS/'models'/cls.config['arm_urdf']
    def geometry(self,path):
        return neutral_retarget_geometry(path,'base_link',self.config['shoulder_frame_names'],self.config['end_frames_name_ik'][3:5])
    def test_main_shoulder_and_legacy_height(self):
        new,old=self.geometry(self.canonical),self.geometry(self.legacy)
        # dev 8445d0d lowers each S62 shoulder origin by 15 mm.
        np.testing.assert_allclose(new['left_shoulder'],[.11575,.2527,1.19016],rtol=0,atol=1e-12)
        # Current dev also aligns the legacy arm-only base_to_joint origin to 0.31 m.
        self.assertAlmostEqual(new['base_height_offset'],old['base_height_offset'],12)
        self.assertAlmostEqual(new['upper_arm_length'],old['upper_arm_length'],12)
    def test_tool_transform_is_not_applied_twice(self):
        transform={side:np.eye(4) for side in ('Left','Right')}
        old,new=[model_geometry(p,self.config['end_frames_name_ik'][1:3],transform) for p in (self.legacy,self.canonical)]
        self.assertEqual(old['arms'],new['arms'])
        for side in transform:self.assertAlmostEqual(new['arms'][side]['joint7_to_tcp'][2][3],-.17,12)
    def test_no_operating_limit_widening(self):
        a,b=[{j.get('name'):j for j in ET.parse(p).getroot().findall('joint')} for p in (self.legacy,self.canonical)]
        names=['knee_joint','leg_joint','waist_pitch_joint','waist_yaw_joint']+['zarm_'+s+str(i)+'_joint' for s in 'lr' for i in range(1,8)]
        for name in names:
            old=[float(a[name].find('limit').get(k)) for k in ('lower','upper')]
            new=[float(b[name].find('limit').get(k)) for k in ('lower','upper')]
            cap=self.config['velocity_ik_position_limits'].get(name,new)
            effective=[max(new[0],cap[0]),min(new[1],cap[1])]
            self.assertGreaterEqual(effective[0],old[0],name)
            self.assertLessEqual(effective[1],old[1],name)
            self.assertLess(effective[0],effective[1],name)
    def test_missing_or_asymmetric_geometry_rejected(self):
        with self.assertRaises(ValueError):neutral_retarget_geometry(self.canonical,'missing_base',self.config['shoulder_frame_names'],self.config['end_frames_name_ik'][3:5])
        tree=ET.parse(self.canonical)
        joint=next(j for j in tree.getroot().findall('joint') if j.get('name')=='zarm_r1_joint')
        origin=joint.find('origin');values=list(map(float,origin.get('xyz').split()));values[1]+=.01;origin.set('xyz',' '.join(map(str,values)))
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'asymmetric.urdf';tree.write(path)
            with self.assertRaises(ValueError):self.geometry(path)
class Geometry400062Test(GeometryTest):
    version = '400062'

if __name__=='__main__':unittest.main()
