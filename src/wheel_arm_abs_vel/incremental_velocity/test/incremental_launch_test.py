#!/usr/bin/env python3
"""Resolve public mode selection and dual-input ownership without starting nodes."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
import roslaunch.config
ROOT=Path(__file__).resolve().parents[2]
LAUNCH=ROOT/'launch'
class IncrementalLaunchTest(unittest.TestCase):
 def load(self,name,args=()):
  return roslaunch.config.load_config_default([(str(LAUNCH/name),['robot_version:=62','ee_type:=none']+list(args))],None,verbose=False)
 def test_quest_and_pico_select_one_solver(self):
  for source in ('quest','pico'):
   for mode in (None,'absolute','incremental'):
    cfg=self.load(source+'_velocity_ik.launch',['teleop_mode:='+mode] if mode else [])
    kinds=[n.type for n in cfg.nodes]
    self.assertEqual(kinds.count('pinocchio_incremental_velocity_ik_node'),int(mode=='incremental'))
    self.assertEqual(kinds.count('pinocchio_velocity_ik_node'),int(mode!='incremental'))
    if mode=='incremental':
     self.assertEqual(cfg.params['/ik_ros_uni/teleop_mode'].value,'incremental')
     self.assertEqual(cfg.params['/ik_ros_uni/velocity_tcp_forearm_length'].value,.45)
     self.assertIn('UNCALIBRATED',cfg.params['/ik_ros_uni/velocity_tcp_geometry_id'].value)
 def test_dual_owner_and_shared_parameter_forwarding(self):
  for side in ('left','right'):
   cfg=self.load('dual_quest_incremental_velocity_ik.launch',['left_ip:=192.0.2.1','right_ip:=192.0.2.2','torso_control_side:='+side,'reference_transport:=ros','reference_shm_key:=454321','owner_shm_key:=454322'])
   nodes=[n for n in cfg.nodes if n.type=='monitor_quest3_velocity.py']
   self.assertEqual(len(nodes),2)
   self.assertEqual(sum(cfg.params['/'+n.name+'/start_state_server'].value for n in nodes),1)
   owner=next(n for n in nodes if n.name=='monitor_quest3')
   self.assertIn(['/leju_quest_bone_poses','/vr_'+side+'/leju_quest_bone_poses'],[list(r) for r in owner.remap_args])
   secondary=next(n for n in nodes if n.name!='monitor_quest3')
   remaps=dict(secondary.remap_args)
   for topic in ('/tf','/robot_head_motion_data','/quest3/set_head_control_mode'):
    self.assertNotEqual(remaps[topic],topic)
   self.assertFalse(cfg.params['/'+secondary.name+'/publish_quest3_globals'].value)
   self.assertEqual(cfg.params['/ik_ros_uni/torso_control_side'].value,side)
   self.assertEqual(cfg.params['/whole_body_velocity_ik/reference/transport'].value,'ros')
   self.assertEqual(cfg.params['/whole_body_velocity_ik/reference/shm_reference_key'].value,454321)
 def test_conflicting_modes_and_same_device_rejected(self):
  cases=[('quest_velocity_ik.launch',['teleop_mode:=incremental','use_cpp_incremental_ik:=true']),('quest_velocity_ik.launch',['teleop_mode:=incremental','wheel_ik:=false']),('dual_quest_incremental_velocity_ik.launch',['left_ip:=192.0.2.1','right_ip:=192.0.2.1'])]
  for launch,args in cases:
   with self.assertRaises(Exception):self.load(launch,args)
 def test_secondary_disconnect_does_not_revoke_global_connection(self):
  source=(ROOT/'scripts/monitor_quest3_velocity.py').read_text()
  cls=next(n for n in ast.parse(source).body if getattr(n,'name','')=='Quest3BoneFramePublisher')
  setter=next(n for n in cls.body if getattr(n,'name','')=='_set_connected')
  writes={};scope={'rospy':SimpleNamespace(set_param=lambda k,v:writes.update({k:v}))}
  exec(compile(ast.Module(body=[setter],type_ignores=[]),'<monitor connection>','exec'),scope)
  for primary in (True,False):
   writes.clear();scope['_set_connected'](SimpleNamespace(connected_param='/vr_right/connected',publish_quest3_globals=primary),False)
   self.assertEqual(writes,{'/vr_right/connected':False,**({'/quest3/connected':False} if primary else {})})
if __name__=='__main__':unittest.main()
