#!/usr/bin/env python3
"""Production Pico full_body -> TF -> head command regression (no actuators)."""
import ast
import importlib.util
import math
import queue
import sys
import time
import threading
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
from monitor_quest3_velocity import Quest3BoneFramePublisher, HeadControlMode
from vr_head_control_manager import HeadControlManager
from pico_input.adapter import PoseAdapter,A,B,D
from pico_input_parity_test import body_packet

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
                receiver.listener=listener
                receiver.enable_head_control=True
                receiver._standalone_head_track_active=lambda:False
                receiver.head_motion_range={'pitch':[-25,25],'yaw':[-85,85]}
                receiver.head_data_pub=Mock();receiver.head_pose_pub=Mock()
                receiver._head_control_lock=threading.RLock()
                receiver.head_control_manager=HeadControlManager()
                with patch.object(rospy.Time,'now',return_value=rospy.Time(1)):
                    receiver.pub_head_motion_data([0,0,0,1])
                command=receiver.head_data_pub.publish.call_args[0][0].joint_data
                index=0 if axis=='yaw' else 1
                self.assertAlmostEqual(command[index],math.degrees(angle),delta=.011)
                self.assertAlmostEqual(command[1-index],0.,delta=.011)

    def test_real_launch_default_head_owner(self):
        import roslaunch
        def load(args, entry='load_kuavo_real_velocity_ik.launch'):
            flags=['robot_version:=62']
            if entry=='load_kuavo_real_velocity_ik.launch':flags.append('with_ik:=false')
            return roslaunch.config.load_config_default([(str(ROOT/'launch'/entry),
                flags+args)],None,verbose=False)
        cfg=load([])
        self.assertFalse(any(n.name=='head_control_node' for n in cfg.nodes))
        self.assertEqual(cfg.params['/absolute/head_mode_service'].value,'/quest3/set_head_control_mode')
        self.assertEqual(cfg.params['/absolute/head_control_mode'].value,'vr_follow')
        for args in (['head_track_hand:=true'],['head_control_mode:=track_hand']):
            cfg=load(args)
            self.assertEqual(sum(n.name=='head_control_node' for n in cfg.nodes),1)
            self.assertEqual(cfg.params['/absolute/head_mode_service'].value,'/head_control_node/set_head_control_mode')
        cfg=load(['head_control_mode:=fixed'])
        self.assertEqual(cfg.params['/absolute/head_control_mode'].value,'fixed')
        cfg=load([], 'load_kuavo_mujoco_velocity_ik.launch')
        self.assertFalse(any(n.name=='head_control_node' for n in cfg.nodes))
        self.assertEqual(cfg.params['/absolute/head_mode_service'].value,'/quest3/set_head_control_mode')
        cfg=load(['head_track_hand:=true'], 'load_kuavo_mujoco_velocity_ik.launch')
        self.assertEqual(sum(n.name=='head_control_node' for n in cfg.nodes),1)
        for entry in ('quest_velocity_ik.launch','pico_velocity_ik.launch'):
            cfg=roslaunch.config.load_config_default([(str(ROOT/'launch'/entry),['robot_version:=62'])],None,verbose=False)
            self.assertTrue(any(k.endswith('/enable_head_control') and p.value for k,p in cfg.params.items()))

    def test_disabled_or_standalone_head_never_publishes(self):
        for enabled,standalone in ((False,False),(True,True)):
            receiver=Quest3BoneFramePublisher.__new__(Quest3BoneFramePublisher)
            receiver.enable_head_control=enabled
            receiver._standalone_head_track_active=lambda:standalone
            receiver.head_data_pub=Mock()
            receiver.pub_head_motion_data([0,0,0,1])
            receiver.head_data_pub.publish.assert_not_called()

    def test_standalone_selection_reserves_head_before_node_ready(self):
        receiver=Quest3BoneFramePublisher.__new__(Quest3BoneFramePublisher)
        receiver._standalone_head_check_time=0.
        receiver._standalone_head_active=False
        params={'/absolute/head_mode_service':'/head_control_node/set_head_control_mode'}
        with patch.object(rospy,'get_param',side_effect=lambda k,d=None:params.get(k,d)), \
             patch.object(rospy,'loginfo'), patch('monitor_quest3_velocity.time.monotonic',return_value=1.):
            self.assertTrue(receiver._standalone_head_track_active())

    def test_invalid_pico_head_does_not_disable_chest_or_fixed_mode(self):
        packet=body_packet()
        packet.full_body.full_body[15].rot_qw=0.
        body=PoseAdapter().from_full_body(packet.full_body.full_body)
        self.assertEqual(len(body),24)
        for mode in (HeadControlMode.VR_FOLLOW,HeadControlMode.FIXED):
            receiver=Quest3BoneFramePublisher.__new__(Quest3BoneFramePublisher)
            receiver._control_requests=queue.Queue()
            receiver._aux_mailbox=Mock()
            receiver._aux_mailbox.take.return_value=(quest_event(body,100000000000),rospy.Time(1))
            receiver.enable_head_control=True
            receiver._standalone_head_track_active=lambda:False
            receiver.head_control_manager=Mock(mode=mode,fixed_main_hand='right')
            receiver.head_ctrl_mode_pub=Mock()
            receiver.pub_chest_pose_in_torso_frame=Mock()
            receiver.pub_head_motion_data=Mock()
            receiver._update_head_control_from_hands=Mock()
            with patch.object(rospy,'is_shutdown',side_effect=[False,True]), \
                 patch.object(rospy,'logwarn_throttle') as warning:
                receiver._auxiliary_loop()
            warning.assert_not_called()
            receiver.pub_chest_pose_in_torso_frame.assert_called_once()
            receiver.pub_head_motion_data.assert_not_called()
            self.assertEqual(receiver._update_head_control_from_hands.call_count,
                             int(mode==HeadControlMode.FIXED))

    def test_vr_to_non_vr_preserves_published_angles_and_slew_limits(self):
        for mode, missing_head in (('fixed',False),('fixed',True),
                                   ('fixed_main_hand',False),('auto_track_active',False)):
            with self.subTest(mode=mode,missing_head=missing_head):
                receiver=Quest3BoneFramePublisher.__new__(Quest3BoneFramePublisher)
                receiver._head_control_lock=threading.RLock()
                receiver.head_control_manager=manager=HeadControlManager()
                manager.set_smoothing_factor(.15)
                manager.set_joint_limits([-85,85],[-25,25])
                manager.set_active_hand_threshold(.001)
                receiver.enable_head_control=True
                receiver._standalone_head_track_active=lambda:False
                receiver.head_motion_range={'pitch':[-25,25],'yaw':[-85,85]}
                rotation=[quaternion_from_matrix(euler_matrix(math.radians(10),math.radians(30),0))]
                receiver.listener=NS(lookupTransform=lambda *a:([0,0,0],rotation[0]))
                receiver._get_robot_hand_tcp_positions=lambda:([.5,.2,.8],[.5,-.2,.8])
                receiver._get_robot_head_pos_from_tf=lambda:None if missing_head else [0,0,1]
                sent=[]
                receiver.head_data_pub=NS(publish=lambda msg:sent.append(list(msg.joint_data)))
                receiver.head_pose_pub=Mock();receiver._pub_head_fixed_user_intent=Mock()
                clock=[100.]
                # Patch the clock module used by the smoother (introduced by
                # the fix); baseline still executes normally and fails the
                # command-continuity assertion, not a missing-method check.
                with patch('time.monotonic',side_effect=lambda:clock[0]), \
                     patch.object(rospy.Time,'now',side_effect=lambda:rospy.Time(clock[0])):
                    for expected in ([30.,10.],[-20.,-5.]):
                        rotation[0]=quaternion_from_matrix(euler_matrix(
                            math.radians(expected[1]),math.radians(expected[0]),0))
                        receiver._handle_set_head_control_mode(NS(mode='vr_follow',fixed_hand=''))
                        receiver.pub_head_motion_data([0,0,0,1])
                        np.testing.assert_allclose(sent[-1],expected,atol=.011)
                        # A long VR interval must not leave an old non-VR time
                        # anchor that permits a large first step.
                        clock[0]+=5.
                        response=receiver._handle_set_head_control_mode(NS(mode=mode,fixed_hand='right'))
                        self.assertTrue(response.success)
                        receiver._update_head_control_from_hands()
                        np.testing.assert_allclose(sent[-1],expected,atol=.011)
                        for _ in range(90):
                            previous=sent[-1];clock[0]+=1./60.
                            receiver._update_head_control_from_hands()
                            delta=np.abs(np.asarray(sent[-1])-previous)
                            self.assertLessEqual(delta[0],manager.max_yaw_rate_deg_s/60.+1e-9)
                            self.assertLessEqual(delta[1],manager.max_pitch_rate_deg_s/60.+1e-9)
                        if mode=='fixed' and not missing_head:
                            np.testing.assert_allclose(sent[-1],[0.,0.],atol=.01)

    def test_head_slew_uses_elapsed_time_across_ros_clock_steps(self):
        manager=HeadControlManager()
        manager.set_smoothing_factor(1.)
        clock=[100.]
        with patch('time.monotonic',side_effect=lambda:clock[0]):
            manager.record_published_vr_head(30.,10.)
            manager.set_mode(HeadControlMode.FIXED)
            for wall in (1.e9,0.,3600.):
                previous=np.array([manager.target_yaw,manager.target_pitch])
                clock[0]+=.01
                with patch.object(rospy.Time,'now',return_value=rospy.Time(wall)):
                    manager.smooth_update(0.,0.)
                np.testing.assert_allclose(previous-[manager.target_yaw,manager.target_pitch],
                                           [1.,.5],atol=1e-9)

if __name__=='__main__':unittest.main()
