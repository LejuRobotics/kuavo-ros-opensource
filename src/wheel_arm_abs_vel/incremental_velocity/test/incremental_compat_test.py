import math
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import rospy
from geometry_msgs.msg import Pose
from kuavo_msgs.msg import JoySticks
from ik_ros_velocity import IkRos, TeleopTcpTarget, VelocityIkMapperStatus
from tools.incremental_clutch import HandClutch
from tools.incremental_controls import Workspace, ModeReturn, ArmModes, ReachStop, pose, xyz
from tools.lightweight_transforms import matrix_to_quaternion, vr_quat2robot_quat, quaternion_to_matrix
from tools.pose_velocity_kalman import exp_so3


class CompatTest(unittest.TestCase):
    def test_double_tap_clock(self):
        modes = ArmModes()
        for t, x, a in ((1,1,0),(1.1,0,0),(1.501,1,0),(1.51,0,0),(1.99,1,0),
                        (2,0,1),(2.1,0,0),(2.15,0,1)):
            modes.update(JoySticks(left_first_button_pressed=x,right_first_button_pressed=a),t,True)
        self.assertEqual(modes.active,[False,True])
        self.assertEqual(modes.transition,[True,False])

    def test_return_profile(self):
        start=np.array([.32,.25,.13]);goal=np.array([.12,.25,.08])
        ret=ModeReturn(start,1.)
        np.testing.assert_equal(ret.step(goal,1.,False),start)
        for k in range(1,400):ret.step(goal,1+k*.01,False)
        np.testing.assert_allclose(ret.p,goal,atol=1.e-8)
        np.testing.assert_equal(ret.step(start,5.,True),start)

    def test_stalled_joint_reverses_without_dead_travel(self):
        stop=ReachStop()
        p=np.array([.25,0.,0.]);measured=p.copy();v=np.array([.05,0.,0.])
        for i in range(50):
            p,_=stop.apply(p,p+.01*v,v,measured,True,i*.01)
        self.assertLess(p[0]-measured[0],.014)
        returned,_=stop.apply(p,p-.01*v,-v,measured,True,.51)
        self.assertLess(returned[0],measured[0])
        stop=ReachStop()
        for i in range(50):
            p,_=stop.apply(p,p+.01*v,v,measured,False,i*.01)
        self.assertIsNone(stop.normal)

    def test_workspace_axes(self):
        box=([-.6,-1,-1],[.47,1,1])
        R=exp_so3(np.array([0.,0.,.5]));t=np.array([.2,.02,.2])
        for sign,side in ((1,'left'),(-1,'right')):
            w=Workspace({'drake_chest_elbow_hand_bounds':{'min_p1_xy_norm':.22}},side,box)
            shoulder=np.array([0.,sign*.18,0.])
            for hand in ([-.38,sign*.35,.06],[.32,sign*.35,.6],[.8,sign*.24,.02]):
                p,v,e=w.project(np.array(hand),np.ones(3),np.zeros(3),(R,t),(R,t),shoulder,.57)
                self.assertLessEqual((R@p+t)[0],.470001)
                self.assertGreaterEqual(p[0],-.030001)
                self.assertLessEqual(p[2],.250001)
                self.assertLessEqual(np.linalg.norm(p-shoulder),.570001)
                self.assertGreaterEqual(sign*e[1],.22-1.e-8)

    def test_elbow_circle_and_midline(self):
        w=Workspace({'drake_chest_elbow_hand_bounds':{'min_p1_xy_norm':.22}},'left',([-.6,-1,-1],[.8,1,1]))
        base=np.eye(3),np.zeros(3); shoulder=np.array([0.,.18,0.])
        for goal in ((.45,.25,.05),(.55,.13,.09),(.35,.05,.4),(.9,.04,.01)):
            hand,_,e=w.project(np.array(goal),np.zeros(3),np.array([.1,0.,.1]),base,base,shoulder,.568,(.29,.28))
            self.assertAlmostEqual(np.linalg.norm(e-shoulder),.29,6)
            self.assertAlmostEqual(np.linalg.norm(e-hand),.28,6)
            self.assertGreaterEqual(e[1],.22-1.e-7)

    def test_mapper_clutch_edges_and_pivot(self):
        # Supply full-model FK feedback in a simple torso frame. The tool lever
        # is deliberately nonzero so wrist/TCP confusion cannot pass this test.
        n=IkRos.__new__(IkRos)
        n._grips=[False,False];n._grip_seq=[1,1];n._side_available=[True,True]
        n._reach_stops=[ReachStop(),ReachStop()]
        n._incremental_modes=ArmModes();n._mode_targets=[None,None];n._exit_capture=[None,None]
        n._orientation_until=[-math.inf]*2;n._next_transfer=[-math.inf]*2;n._orientation_hold=[None,None]
        n._mode_threshold=.01;n._reset_to_default=False;n._reach_margin=.002
        n._rotation_limits=[np.array([1.49]*3)]*3
        n._workspace=[Workspace({},side,([-.6,-1,-1],[.47,1,1])) for side in ('left','right')]
        n._clutches=[HandClutch([1.26]*3,[1.]*3,True,1.) for _ in range(2)]
        n.absolute_enable_max_vr_age=.25
        eye=np.eye(4)
        bones={i:(eye.copy(),np.zeros(3),np.zeros(3)) for i in (23,4,5)}
        n.quest3_arm_info_transformer=NS(estimated_bones=bones,init_R_wC=np.eye(3))
        f=VelocityIkMapperStatus()
        f.session_bound=f.incremental_geometry_valid=True;f.session_id=f.generation=1
        f.base_from_chest.orientation.w=1.
        lever=np.array([.06,0,0])
        for i,side in enumerate(('Left','Right')):
            # Seed robot orientation independently of the human's wrist.
            f.measured_mapper_tcp[i].orientation.w=1.
            f.measured_mapper_tcp[i].position.x=.26
            f.measured_mapper_tcp[i].position.y=.24*(1-2*i)
            f.mapper_link4[i].w=f.base_from_mapper[i].orientation.w=1.
            f.mapper_shoulder[i].y=.18*(1-2*i);f.arm_reach[i]=.60;f.upper_arm_length[i]=.30
            f.tcp_to_link6[i].x=lever[0]
        def step(stamp, grip, paused=False, seq=0):
            if grip!=n._grips[0]:n._grip_seq[0]+=1;n._grips[0]=grip
            m=TeleopTcpTarget();m.left_valid=m.right_valid=m.control_valid=True
            m.session_id=m.generation=1;m.control_state=m.CONTROL_ACTIVE
            m.header.stamp=rospy.Time.from_sec(stamp);m.arm_hold=paused;m.arm_operation_seq=seq
            n._fixed_consumer_status=(time.monotonic(),f)
            n._apply_incremental_target(m)
            self.assertTrue(m.control_valid)
            R,t=pose(m.left_tcp_pose)
            return t+R@lever,R,m
        p0,_,_=step(1.,False)
        p,_,_=step(1.01,True);np.testing.assert_allclose(p,p0)
        for k in range(1,20):
            bones[4][0][:3,:3]=exp_so3(np.array([0.,0.,k*.015]))
            p,R,m=step(1.01+.01*k,True)
            if k==1: self.assertGreater(np.linalg.norm(R-np.eye(3)),.005)
            np.testing.assert_allclose(p,p0,atol=1.e-8)
        self.assertGreater(np.linalg.norm(R-np.eye(3)),.1)
        for k in range(1,60):
            bones[4][0][0,3]+=.01
            p,_,m=step(1.20+.01*k,True)
        self.assertAlmostEqual(p[0],.47,6)
        bones[4][0][0,3]-=.01
        ret,_,_=step(1.80,True)
        self.assertAlmostEqual(p[0]-ret[0],.0126,6)
        held,_,_=step(1.81,False)
        bones[4][0][0,3]=0.
        resumed,_,_=step(1.82,True)
        np.testing.assert_allclose(held,resumed,atol=1.e-8)
        stopped,_,_=step(1.83,True,True,1)
        bones[4][0][0,3]=-.15
        ready,_,m=step(1.84,True,False,2)
        np.testing.assert_allclose(stopped,ready,atol=1.e-8)
        np.testing.assert_equal(xyz(m.left_tcp_twist.linear),0.)
        # Ordinary grip inherits command even with a 30-mm measured FK lag.
        held,held_R,_=step(1.85,False,False,2)
        f.measured_mapper_tcp[0].position.z-=.03
        resumed,resumed_R,_=step(1.86,True,False,2)
        np.testing.assert_allclose(resumed,held,atol=1.e-10)
        np.testing.assert_allclose(resumed_R,held_R,atol=1.e-10)
        # Kernel command feedback is a separate, sequence-qualified channel.
        f.command_geometry_valid=True
        f.command_base_chest.orientation.w=1.
        f.command_held=[True,True]
        f.command_clutch_seq=[n._grip_seq[0]+1,n._grip_seq[1]]
        f.command_operation_seq=2
        command=np.array([.34,.24,.06])
        f.command_mapper_tcp[0].orientation.w=1.
        f.command_mapper_tcp[0].position.x,f.command_mapper_tcp[0].position.y,f.command_mapper_tcp[0].position.z=command-lever
        f.command_mapper_elbow[0].x=.1;f.command_mapper_elbow[0].y=.24
        f.command_mapper_tcp[1].orientation.w=1.
        p,_,_=step(1.87,False,False,2)
        np.testing.assert_allclose(p,command,atol=1.e-10)
        p,_,_=step(1.88,True,False,2)
        np.testing.assert_allclose(p,command,atol=1.e-10)
        # !4141: only X+B plus torso displacement reanchors to measured FK.
        step(1.89,True,True,3)
        f.command_base_chest.position.x=.1
        step(1.895,True,True,3)  # torso settles before resume; retain pause origin
        p,_,m=step(1.90,True,False,4)
        measured_R,measured_p=pose(f.measured_mapper_tcp[0])
        np.testing.assert_allclose(p,measured_p+measured_R@lever,atol=1.e-10)
        np.testing.assert_equal(xyz(m.left_tcp_twist.linear),0.)
        n._side_available[0]=False;n._grip_seq[0]+=1
        _,_,m=step(1.91,True,False,4)
        self.assertTrue(m.clutch_hold[0] and m.robot_hold_target[0])
        self.assertFalse(m.left_elbow_hint_valid)

    def test_still_hand_keeps_robot_elbow_and_chest_clearance(self):
        n=IkRos.__new__(IkRos)
        n._grips=[False,False];n._grip_seq=[1,1];n._side_available=[True,True]
        n._reach_stops=[ReachStop(),ReachStop()]
        n._incremental_modes=ArmModes();n._mode_targets=[None,None];n._exit_capture=[None,None]
        n._orientation_until=[-math.inf]*2;n._next_transfer=[-math.inf]*2;n._orientation_hold=[None,None]
        n._mode_threshold=.01;n._reset_to_default=False;n._reach_margin=.002
        n._rotation_limits=[np.array([1.49]*3)]*3
        n._workspace=[Workspace({},side,([-.6,-1,-1],[.8,1,1])) for side in ('left','right')]
        n._clutches=[HandClutch([1.]*3,[1.]*3,True,1.) for _ in range(2)]
        n.absolute_enable_max_vr_age=.25
        eye=np.eye(4)
        bones={i:(eye.copy(),np.zeros(3),np.zeros(3)) for i in (23,4,5)}
        n.quest3_arm_info_transformer=NS(estimated_bones=bones,init_R_wC=np.eye(3))
        f=VelocityIkMapperStatus()
        f.session_bound=f.incremental_geometry_valid=True;f.session_id=f.generation=1
        f.base_from_chest.orientation.w=1.
        for i,side in enumerate(('Left','Right')):
            f.measured_mapper_tcp[i].orientation.w=1.
            f.measured_mapper_tcp[i].position.x=.30
            f.measured_mapper_tcp[i].position.y=.22*(1-2*i)
            f.mapper_link4[i].w=f.base_from_mapper[i].orientation.w=1.
            f.mapper_shoulder[i].y=.18*(1-2*i);f.arm_reach[i]=.60;f.upper_arm_length[i]=.30
            f.mapper_elbow[i].x=.05
            f.mapper_elbow[i].y=.26*(1-2*i)
            f.mapper_elbow[i].z=.05
        def step(stamp, grip, elbow):
            if grip!=n._grips[0]:n._grip_seq[0]+=1;n._grips[0]=grip
            m=TeleopTcpTarget();m.left_valid=m.right_valid=m.control_valid=True
            m.session_id=m.generation=1;m.control_state=m.CONTROL_ACTIVE
            m.header.stamp=rospy.Time.from_sec(stamp)
            m.left_elbow_hint_valid=True
            m.left_elbow_hint.x,m.left_elbow_hint.y,m.left_elbow_hint.z=elbow
            n._fixed_consumer_status=(time.monotonic(),f)
            n._apply_incremental_target(m)
            self.assertTrue(m.control_valid)
            self.assertTrue(m.left_elbow_hint_valid)
            return np.array(xyz(m.left_elbow_hint))
        held=np.array([.05,.26,.05])
        far=np.array([.20,.55,.05])
        step(1.,False,far)
        for k in range(1,121):
            elbow=step(1.+.01*k,True,far)
        self.assertLess(abs(elbow[1]-held[1]),.16)
        self.assertGreater(elbow[1],.19)
        self.assertLess(elbow[1],.40)
        # An elbow already inside the chest is still pushed out to the clearance.
        n._clutches[0].elbow=np.array([.05,.02,.05])
        n._clutches[0].elbow_offset=None
        inside=step(3.,True,far)
        self.assertGreaterEqual(inside[1],.19)
        self.assertLess(inside[1],.40)


if __name__=='__main__':
    unittest.main()
