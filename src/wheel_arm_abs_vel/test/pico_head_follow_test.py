#!/usr/bin/env python3
"""Production Pico full_body -> TF -> head command regression (no actuators)."""
import ast
import importlib.util
import math
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import legacy_dependencies
import rospy
import tf
from tf.transformations import euler_matrix, quaternion_from_matrix
from noitom_hi5_hand_udp_python.msg import PoseInfoList
from monitor_pico import quest_event
from monitor_quest3_velocity import Quest3BoneFramePublisher
from pico_input.adapter import PoseAdapter,A,B,D
from pico_input_parity_test import body_packet
spec=importlib.util.spec_from_file_location('tested_head',ROOT.parent/'manipulation_nodes/noitom_hi5_hand_udp_python/scripts/head_control_node.py')
head=importlib.util.module_from_spec(spec);spec.loader.exec_module(head)

class HeadTest(unittest.TestCase):
    def test_full_body_to_head_command_sign(self):
        for axis in ('yaw','pitch'):
            for angle in (-.2,.2):
                packet=body_packet()
                physical=euler_matrix(0,0,angle) if axis=='yaw' else euler_matrix(0,angle,0)
                raw=np.eye(4);raw[:3,:3]=(D@A).T@physical[:3,:3]@(B@D).T
                q=quaternion_from_matrix(raw)
                pose=packet.full_body.full_body[15]
                pose.rot_qx,pose.rot_qy,pose.rot_qz,pose.rot_qw=q
                matrices=PoseAdapter().from_full_body(packet.full_body.full_body)
                receiver=Quest3BoneFramePublisher.__new__(Quest3BoneFramePublisher)
                init=next(n for n in ast.walk(ast.parse((ROOT/'scripts/monitor_quest3_velocity.py').read_text()))
                          if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Attribute)
                          and n.targets[0].attr=='bone_names')
                receiver.bone_names=ast.literal_eval(init.value)
                receiver._input_max_age=.15;receiver.observation_tf_rate_hz=10
                receiver._last_observation_tf=time.monotonic();receiver.control_finger_tf_enabled=False
                receiver.bone_tf_pub=Mock()
                poses=PoseInfoList();receiver.process_pose_data(quest_event(matrices,100000000000),poses,None)
                receiver.publish_pose_tf(poses,rospy.Time(1),time.monotonic(),0.)
                transforms=receiver.bone_tf_pub.publish.call_args[0][0].transforms
                self.assertEqual({t.child_frame_id for t in transforms},{'Chest','Head'})
                listener=tf.Transformer(True,rospy.Duration(10))
                for t in transforms:listener.setTransform(t)
                node=head.HeadTrackHandNode.__new__(head.HeadTrackHandNode)
                node.listener=listener;node.vr_chest_frame='Chest';node.vr_head_frame='Head'
                node.vr_pitch_range=(-25,25);node.vr_yaw_range=(-85,85)
                node.head_pub=Mock();node.head_pose_pub=Mock();node.manager=NS()
                with patch.object(head.rospy.Time,'now',return_value=rospy.Time(1)):
                    node._update_vr_follow()
                command=node.head_pub.publish.call_args[0][0].joint_data
                index=0 if axis=='yaw' else 1
                self.assertAlmostEqual(command[index],math.degrees(angle),delta=.011)
                self.assertAlmostEqual(command[1-index],0.,delta=.011)

    def test_real_launch_default_head_owner(self):
        import roslaunch
        cfg=roslaunch.config.load_config_default([(str(ROOT/'launch/load_kuavo_real_velocity_ik.launch'),
            ['robot_version:=62','with_ik:=false'])],None,verbose=False)
        nodes=[n for n in cfg.nodes if n.name=='head_control_node' and 'head_control_node.py' in n.args]
        self.assertEqual(len(nodes),1)
        self.assertEqual(cfg.params['/head_control_node/head_control_mode'].value,'fixed')
        # Receiver explicitly disables its old head publisher.
        source=(ROOT/'scripts/monitor_quest3_velocity.py').read_text()
        init=next(n for n in ast.walk(ast.parse(source)) if isinstance(n,ast.FunctionDef) and n.name=='__init__')
        self.assertNotIn('/robot_head_motion_data', [a.value for a in ast.walk(init) if isinstance(a,ast.Constant) and isinstance(a.value,str) and a.value=='/robot_head_motion_data'])

if __name__=='__main__':unittest.main()
