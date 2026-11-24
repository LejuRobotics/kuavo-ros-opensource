#!/usr/bin/env python3
"""Mounting correction, raw-data preservation and record-only launch contracts."""
import ast
import copy
import hashlib
import json
import sys
import tempfile
import unittest
import xmlrpc.client
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'scripts/tools'))
from controller_mount_calibration import checked_calibration,controller_rotations,status_json
from lightweight_transforms import quaternion_to_matrix,vr_quat2robot_quat
MODEL=ROOT.parent/'kuavo_assets/models/biped_s400062/urdf/biped_s400062.urdf'

def config():
    return dict(enabled=True,schema_version=2,eef_convention='sg100_400062',
        robot_version=400062,metadata=dict(quality_gate='activation_eligible',orientation_calibrated=True),model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        **{s:dict(palm_to_controller_offset=[.02,-.03,.09],
                  eef_post_rotation_xyzw=Rotation.from_euler('xyz',[.3,-.2,.8]).as_quat().tolist())
           for s in ('left','right')})

class MountTest(unittest.TestCase):
    def test_disabled_legacy_model_and_bad_numeric_validation(self):
        self.assertIsNone(checked_calibration({'enabled':False},40))
        good=config();self.assertIsNotNone(checked_calibration(good,400062,good['model_sha256']))
        for change in ({'schema_version':1},{'robot_version':400063},
                       {'eef_convention':'legacy'},{'model_sha256':'bad'}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                checked_calibration(dict(good,**change),400062,good['model_sha256'])
        with self.assertRaises(ValueError):checked_calibration(good,400063)
        with self.assertRaises(ValueError):checked_calibration(good,400062,'f'*64)
        for key,value in [('palm_to_controller_offset',[np.nan,0,0]),
                          ('palm_to_controller_offset',[0,0,.5]),
                          ('eef_post_rotation_xyzw',[0,0,0,0])]:
            broken=copy.deepcopy(good);broken['left'][key]=value
            with self.assertRaises(ValueError):checked_calibration(broken,400062)

    def test_unknown_translation_preserves_rotation_but_cannot_apply_an_offset(self):
        cfg=config()
        cfg['metadata'].update(translation_calibrated=False,
            translation_status='not_identifiable_from_recording',
            distance_source='unknown')
        for side in ('left','right'):
            cfg[side]['palm_to_controller_offset']=[0.,0.,0.]
        # Unknown is represented by an absent numeric parameter. ROS 1's
        # default XML-RPC client cannot upload None/null values.
        xmlrpc.client.dumps((cfg,))
        checked=checked_calibration(cfg,400062,cfg['model_sha256'])
        for side in ('left','right'):
            np.testing.assert_allclose(checked[side]['eef_post_rotation_xyzw'],
                                       cfg[side]['eef_post_rotation_xyzw'])
        status=json.loads(status_json(checked,'receiver',{'Left':[0.,0.,0.],'Right':[0.,0.,0.]}))
        self.assertFalse(status['translation_calibrated'])
        self.assertEqual(status['translation_status'],'not_identifiable_from_recording')
        self.assertIsNone(status['known_distance_m'])
        self.assertEqual(status['distance_source'],'unknown')
        cfg['right']['palm_to_controller_offset']=[0.,0.,.1]
        with self.assertRaisesRegex(ValueError,'unknown translation'):
            checked_calibration(cfg,400062)

    def test_three_poses_reach_correct_sg100_axes_and_optical_is_unchanged(self):
        # Independently define the EEF's physical axes, then use a nontrivial mount.
        for normal_name in ('up','facing','down'):
            for side in ('Left','Right'):
                normal=np.array([0.,0.,1. if normal_name=='up' else -1.])
                if normal_name=='facing':normal=np.array([0.,-1. if side=='Left' else 1.,0.])
                z=np.array([-1.,0.,0.]);y=(-normal if side=='Left' else normal)
                desired=Rotation.from_matrix(np.column_stack((np.cross(y,z),y,z)))
                raw_to_eef=Rotation.from_euler('xyz',[.31,-.43,.27])
                raw=desired*raw_to_eef.inv()
                base=Rotation.from_matrix(quaternion_to_matrix(vr_quat2robot_quat([0,0,0,1],side,np.radians(15))))
                cfg=config();cfg[side.lower()]['eef_post_rotation_xyzw']=(base.inv()*raw_to_eef).as_quat().tolist()
                bias=Rotation.from_euler('x',np.radians(-15 if side=='Left' else 15)).as_matrix()
                adapters={k:{s:np.eye(4) for s in ('Left','Right')} for k in ('controller','hand_tracking')}
                for name in ('Left','Right'):
                    adapters['controller'][name][:3,:3]=Rotation.from_euler('x',np.radians(-15 if name=='Left' else 15)).as_matrix()
                controller_rotations(adapters,checked_calibration(cfg,400062),quaternion_to_matrix)
                final=quaternion_to_matrix(vr_quat2robot_quat(raw.as_quat(),side,0))@adapters['controller'][side][:3,:3]
                np.testing.assert_allclose(final@np.array([0,0,-1]),[1,0,0],atol=1e-12)
                np.testing.assert_allclose(final@np.array([0,-1 if side=='Left' else 1,0]),normal,atol=1e-12)
                np.testing.assert_array_equal(adapters['hand_tracking'][side],np.eye(4))

    def test_diagnostic_or_nondefault_adapter_is_rejected(self):
        for metadata in ({},dict(quality_gate='diagnostic_only_short_hold',orientation_calibrated=True),
                         dict(quality_gate='activation_eligible',orientation_calibrated=False)):
            cfg=config();cfg['metadata']=metadata
            with self.assertRaises(ValueError):checked_calibration(cfg,400062)
        adapters={'controller':{s:np.eye(4) for s in ('Left','Right')}}
        with self.assertRaisesRegex(ValueError,'baseline'):
            controller_rotations(adapters,checked_calibration(config(),400062),quaternion_to_matrix)

    def test_raw_recording_is_before_translation_and_does_not_change_hand_tracking(self):
        from monitor_quest3_velocity import Quest3BoneFramePublisher
        from noitom_hi5_hand_udp_python.msg import PoseInfoList
        receiver=Quest3BoneFramePublisher.__new__(Quest3BoneFramePublisher)
        receiver.bone_name_to_index={'LeftHandPalm':4,'RightHandPalm':5,
                                    'LeftHandIndexTip':10,'RightHandIndexTip':18}
        receiver.raw_pose_pub=Mock();receiver.raw_pose_pub.get_num_connections.return_value=1
        receiver.palm_offsets={'Left':np.array([.02,-.03,.09]),'Right':np.array([-.01,.04,.09])}
        poses=[SimpleNamespace(position=SimpleNamespace(x=.2+i*.01,y=.3,z=.4),
                              quaternion=SimpleNamespace(x=.1,y=.2,z=.3,w=np.sqrt(.86))) for i in range(26)]
        for hand_tracking in (False,True):
            event=SimpleNamespace(poses=poses,IsHandTracking=hand_tracking,
                                  IsDataHighConfidence=True,timestamp=12345)
            out=PoseInfoList();receiver.process_pose_data(event,out,None)
            raw=receiver._raw_pose_sample
            self.assertEqual(raw.timestamp_ms,12345)
            for i in range(26):
                p=lambda obj:np.array([obj.x,obj.y,obj.z])
                expected=p(raw.poses[i].position).copy()
                if not hand_tracking and i in (4,5,10,18):
                    side='Left' if i in (4,10) else 'Right'
                    q=raw.poses[4 if side=='Left' else 5].orientation
                    expected-=quaternion_to_matrix([q.x,q.y,q.z,q.w])@receiver.palm_offsets[side]
                np.testing.assert_allclose(p(out.poses[i].position),expected,atol=1e-12)
            self.assertAlmostEqual(poses[4].position.x,.24)

    def test_controller_recorder_includes_raw_and_gesture_evidence(self):
        import re, json, roslaunch
        cfg=roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(ROOT/'launch/load_kuavo_real_velocity_ik.launch'),cfg,
            argv=['robot_version:=400062','ee_type:=heiman'],verbose=False)
        topic_file=Path(cfg.params['/topic_config'].value)
        patterns=yaml.safe_load(topic_file.read_text())['topics']
        required=('/leju_quest_bone_poses_raw','/leju_quest_bone_poses',
                  '/quest3/controller_calibration_status','/quest3/controller_calibration_mapper_status',
                  '/glove_gesture/buttons','/glove_gesture/interaction_mode',
                  '/glove_gesture/debug/distances','/qianshi/glove_left/fingertips',
                  '/qianshi/glove_right/fingertips','/quest_joystick_data',
                  '/mm/teleop_tcp_target','/sensors_data_raw/ee_fk/zarm_l7_end_effector',
                  '/sensors_data_raw/ee_fk/zarm_r7_end_effector')
        for topic in required:
            self.assertTrue(any(re.fullmatch(p,topic) for p in patterns),topic)
        effective={'Left':np.array([0.,0.,0.]),'Right':np.array([.02,0.,.1])}
        payload=json.loads(status_json(checked_calibration(config(),400062),'receiver',effective))
        self.assertEqual(payload['effective_palm_to_controller_offset']['left'],[0.,0.,0.])
        self.assertEqual(payload['effective_palm_to_controller_offset']['right'],[.02,0.,.1])
        self.assertEqual(payload['model_sha256'],config()['model_sha256'])
        self.assertFalse(json.loads(status_json(None,'absolute_mapper'))['enabled'])

    def test_launch_loads_explicit_calibration_and_recorder_has_no_control_nodes(self):
        import roslaunch
        def load(name,args):
            cfg=roslaunch.config.ROSLaunchConfig()
            roslaunch.xmlloader.XmlLoader().load(str(ROOT/'launch'/name),cfg,argv=args,verbose=False)
            return cfg
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'mount.yaml';path.write_text(yaml.safe_dump({'quest3':{'controller_calibration':config()}}))
            cfg=load('quest_windows_udp.launch',['robot_version:=400062','ip_address:=192.0.2.1',
                                                'quest_controller_calibration_file:='+str(path)])
            self.assertTrue(cfg.params['/quest3/controller_calibration/enabled'].value)
            self.assertTrue(cfg.params['/monitor_quest3/qianshi_enabled'].value)
            self.assertEqual(cfg.params['/monitor_quest3/left_palm_to_controller_offset'].value,[])
        cfg=load('record_quest_calibration.launch',['ip_address:=192.0.2.1'])
        self.assertEqual({n.type for n in cfg.nodes},{'monitor_quest3_velocity.py','record'})
        for flag in ('qianshi_enabled','qianshi_gestures','enable_head_control','start_state_server'):
            self.assertFalse(cfg.params['/quest_calibration_receiver/'+flag].value)
        self.assertIn('/leju_quest_bone_poses_raw',next(n.args for n in cfg.nodes if n.type=='record'))

if __name__=='__main__':unittest.main()
