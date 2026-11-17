import math
import os
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from tools.incremental_clutch import HandClutch
from tools.incremental_controls import bound_rotation, RotationTransfer, Workspace
from tools.pose_velocity_kalman import exp_so3, log_so3, _NATIVE
from tools.lightweight_transforms import rpy_to_matrix


class HandoffTest(unittest.TestCase):
    def test_absolute_orientation_handoff_and_lever(self):
        r=RotationTransfer(4.)
        eye=np.eye(3);goal=exp_so3(np.array([0.,0.,2.9]))
        r.engage(eye,0.,goal)
        lever=np.array([.193,0.,0.]);wrist=np.array([.32,.24,.1])
        rows=[]
        for t in np.arange(0.,1.,.001):
            R,w=r.step(goal,np.zeros(3),t)
            tcp=wrist-R@lever
            np.testing.assert_array_equal(w,np.zeros(3))
            np.testing.assert_allclose(tcp+R@lever,wrist,atol=1.e-12)
            if t==0.:np.testing.assert_allclose(R,eye,atol=1.e-12)
            rows.append(tcp)
        self.assertLess(np.max(np.linalg.norm(np.diff(rows,axis=0),axis=1)),.006)
        self.assertLess(np.linalg.norm(log_so3(R@goal.T)),1.e-7)

    def test_regrip_admits_human_rotation_on_the_next_frame(self):
        eye=np.eye(3);z=np.zeros(3)
        for goal in (eye,exp_so3(np.array([.3,-.4,.8]))):
            stationary=RotationTransfer();moving=RotationTransfer()
            stationary.engage(eye,0.,goal,goal,('raw',))
            moving.engage(eye,0.,goal,goal,('raw',))
            initial,_=moving.step(goal,z,0.,goal,('raw',))
            np.testing.assert_allclose(initial,eye,atol=1.e-12)
            delta=exp_so3(np.array([.02,-.01,.03]));current=delta@goal
            fixed,_=stationary.step(goal,z,.008,goal,('raw',))
            moved,_=moving.step(current,z,.008,current,('raw',))
            self.assertAlmostEqual(np.linalg.norm(log_so3(moved@fixed.T)),
                                   np.linalg.norm(log_so3(delta)),places=10)

    def test_default_orientation_remains_absolute(self):
        c=HandClutch([1.26]*3,[1.]*3,False,10.)
        z=np.zeros(3);eye=np.eye(3);goal=exp_so3(np.array([.2,-.3,.4]))
        c.sample(1,1,False,z,eye,z,z,(z,eye),z,0.)
        c.sample(1,2,True,z,eye,z,z,(z,eye),z,.01)
        _,R,_,_=c.sample(1,2,True,z,goal,z,z,(z,eye),z,.02)
        np.testing.assert_allclose(R,goal)

    def test_incremental_source_out_and_back_is_reversible(self):
        c=HandClutch([1.26]*3,[1.]*3,True,10.)
        eye=np.eye(3);z=np.zeros(3);limits=[np.full(3,1.4923)]*3
        c.sample(1,1,True,z,eye,z,z,(z,eye),z,0.)
        for k,angle in enumerate([*np.linspace(0,2.8,100),*np.linspace(2.8,0,100)]):
            _,raw,_,_=c.sample(1,1,True,z,exp_so3(np.array([angle,0.,0.])),z,z,(z,eye),z,.01*(k+1))
            c.rotation=bound_rotation(raw,eye,eye,limits,True,False)
        np.testing.assert_allclose(c.source_rotation,eye,atol=1.e-10)
        np.testing.assert_allclose(c.rotation,eye,atol=1.e-10)

    def test_small_and_gimbal_rotations_are_not_rebuilt(self):
        eye=np.eye(3);limits=[np.full(3,1.4923)]*3
        for v in ([1.e-5,2.e-5,-1.e-5],[.0,math.pi/2,0.]):
            R=exp_so3(np.array(v))
            np.testing.assert_array_equal(bound_rotation(R,eye,eye,limits,False,False),R)

    def test_pi_branch_cut_transfers_without_output_jump(self):
        eye=np.eye(3);limits=[np.full(3,1.4923)]*3
        raw_a=exp_so3(np.array([0.,0.,math.radians(179.9)]))
        raw_b=exp_so3(np.array([0.,0.,math.radians(180.1)]))
        a,tag_a=bound_rotation(raw_a,eye,eye,limits,False,False,True)
        b,tag_b=bound_rotation(raw_b,eye,eye,limits,False,False,True)
        r=RotationTransfer();r.step(a,np.zeros(3),1.,raw_a,tag_a)
        out,_=r.step(b,np.zeros(3),1.008,raw_b,tag_b)
        np.testing.assert_allclose(out,raw_b@raw_a.T@a,atol=1.e-10)
        for t in np.arange(1.016,2.,.008):out,_=r.step(b,np.zeros(3),t,raw_b,tag_b)
        self.assertLess(np.linalg.norm(log_so3(out@b.T)),1.e-7)

    def test_rapid_reengagement_keeps_new_hold_reference(self):
        r=RotationTransfer();eye=np.eye(3)
        r.engage(eye,0.)
        goal=exp_so3(np.array([1.,.2,-.3]))
        out,_=r.step(goal,np.zeros(3),.09)
        r.engage(out,.091,eye)
        resumed,velocity=r.step(eye,np.zeros(3),.091)
        np.testing.assert_allclose(resumed,out)
        np.testing.assert_array_equal(velocity,np.zeros(3))

    def test_gimbal_admission_handoff_preserves_human_increment(self):
        eye=np.eye(3);limits=[np.full(3,1.4923)]*3
        raw_a=rpy_to_matrix([1.9,math.asin(.939),2.])
        raw_b=rpy_to_matrix([1.9,math.asin(.941),2.])
        a,tag_a=bound_rotation(raw_a,eye,eye,limits,False,False,True)
        b,tag_b=bound_rotation(raw_b,eye,eye,limits,False,False,True)
        self.assertNotEqual(tag_a,tag_b)
        r=RotationTransfer();r.step(a,np.zeros(3),1.,raw_a,tag_a)
        out,_=r.step(b,np.zeros(3),1.008,raw_b,tag_b)
        np.testing.assert_allclose(out,raw_b@raw_a.T@a,atol=1.e-10)
        for t in np.arange(1.016,2.,.008):out,_=r.step(b,np.zeros(3),t,raw_b,tag_b)
        self.assertLess(np.linalg.norm(log_so3(out@b.T)),1.e-7)

    def test_chest_coordinate_recovery_reanchors_with_zero_increment(self):
        c=HandClutch([1.26]*3,[1.]*3,False,10.)
        z=np.zeros(3);eye=np.eye(3)
        c.sample((1,1),1,True,z,eye,z,z,(z,eye),z,0.)
        moved=np.array([.24,-.13,.08]);rot=exp_so3(np.array([.2,.1,.4]))
        # Chest recovery changes the coordinate epoch, ordinary grip does not.
        p,R,v,w=c.sample((1,2),2,True,np.ones(3),eye,np.ones(3),np.ones(3),
                         (moved,rot),z,.01)
        np.testing.assert_allclose(p,moved)
        np.testing.assert_allclose(R,rot)
        np.testing.assert_array_equal(v,z);np.testing.assert_array_equal(w,z)

    @unittest.skipIf(_NATIVE is None,'native build required')
    def test_workspace_native_matches_python_on_boundary_and_return(self):
        rng=np.random.default_rng(17)
        counts=[0,0]
        for side,sign in (('left',1),('right',-1)):
            w=Workspace({'drake_chest_elbow_hand_bounds':{'min_p1_xy_norm':.22}},side,([-.6,-1,-1],[.47,1,1]))
            shoulder=np.array([0.,sign*.18,0.])
            for _ in range(600):
                R=exp_so3(rng.normal(0,.2,3));t=rng.normal(0,.05,3)
                args=(rng.uniform([-.2,-.7,-.2],[.9,.7,.7]),rng.normal(0,.2,3),rng.normal(0,.2,3),(R,t),(R,t),shoulder,.568,(.29,.28))
                values=[]
                for fn in (w.project_python,w.project):
                    try:values.append(fn(*args))
                    except ValueError:values.append(None)
                self.assertEqual(values[0] is None,values[1] is None)
                if values[0] is None:counts[1]+=1;continue
                counts[0]+=1
                for old,new in zip(*values):np.testing.assert_allclose(old,new,atol=1.e-8,rtol=0)
        self.assertGreater(counts[0],400)

if __name__=='__main__':unittest.main()
