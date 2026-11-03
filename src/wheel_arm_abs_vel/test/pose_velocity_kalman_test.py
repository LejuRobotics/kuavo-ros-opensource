#!/usr/bin/env python3
"""Hand-only estimator contracts, source resets and geometric input smoothing."""
from pathlib import Path
import sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/tools'))
from pose_velocity_kalman import PositionCV,RotationCV,HandTargetCV,exp_so3,log_so3

class ObserverTests(unittest.TestCase):
    def test_constant_velocity_and_covariance(self):
        f=PositionCV(); v=np.array([.3,-.2,.1]); t=0.
        for i in range(1500):
            dt=.006+(i%5)*.001;t+=dt
            p,vel=f.update(v*t,None if i==0 else dt)
        np.testing.assert_allclose(vel,v,atol=1e-8)
        np.testing.assert_allclose(p,v*t,atol=1e-8)
        self.assertGreater(np.linalg.eigvalsh(f.P).min(),0.)

    def test_rotation_wrap_constant_velocity(self):
        f=RotationCV();w=np.array([.3,-.7,1.2]);dt=1/120
        for i in range(1500):R,v=f.update(exp_so3(w*i*dt),None if i==0 else dt)
        np.testing.assert_allclose(v,w,atol=1e-6)
        np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-10)
        self.assertGreater(np.linalg.eigvalsh(f.P).min(),0.)
        for axis in np.eye(3):
            R=exp_so3(np.pi*axis);np.testing.assert_allclose(exp_so3(log_so3(R)),R,atol=1e-8)

    def test_two_hands_only_and_reset(self):
        f=HandTargetCV()
        for i in range(200):
            f.bind(('session',1))
            for side in ('Left','Right'):f.update(side,np.ones(3)*i*.001,np.eye(3),1.+i/120)
        self.assertEqual(set(f.positions),{'Left','Right'})
        self.assertEqual(set(f.rotations),{'Left','Right'})
        for stamp,key in [(4.,('session',2)),(4.01,('session',3))]:
            f.bind(key)
            p,R,v,w=f.update('Right',np.zeros(3),np.eye(3),stamp)
            np.testing.assert_array_equal(v,np.zeros(3));np.testing.assert_array_equal(w,np.zeros(3))

    def test_elbow_step_has_no_overshoot(self):
        f=HandTargetCV();out=[]
        for i in range(120):
            f.bind(1)
            out.append(f.smooth_elbow('Right',np.ones(3)*(i>=10),1+i/120)[0])
        out=np.array(out)
        self.assertTrue(np.all(np.diff(out)>=0))
        self.assertTrue(np.all((out>=0)&(out<=1)))
        self.assertLess(out[10],.3)
        self.assertEqual(f.positions,{})

    def test_single_frame_impulse_and_settle(self):
        for hz in (72,120):
            f=PositionCV();out=[];vel=[]
            for i in range(1200):
                p,v=f.update(np.array([.00663 if i==200 else 0.,0.,0.]),None if i==0 else 1/hz)
                out.append(p[0]);vel.append(v[0])
            self.assertLess(out[200]/.00663,.40)  # Old 72/120Hz observer passes 80%/67%.
            self.assertLess(abs(out[-1]),1e-8);self.assertLess(abs(vel[-1]),1e-8)

    def test_per_hand_observation_clock_and_duplicates(self):
        f=HandTargetCV();f.bind((1,1,1))
        z=np.zeros(3);I=np.eye(3)
        for side in ('Left','Right'):f.update(side,z,I,1.)
        x=f.positions['Left'].x.copy();P=f.positions['Left'].P.copy()
        f.update('Left',np.ones(3),exp_so3(np.ones(3)),1.)
        np.testing.assert_array_equal(x,f.positions['Left'].x)
        np.testing.assert_array_equal(P,f.positions['Left'].P)
        f.update('Right',z,I,1.01)
        self.assertEqual(f.last_observation_stamp['Left'],1.)
        before=f.positions['Left'].P.copy()
        f.update('Left',z,I,1.03)
        oracle=PositionCV();oracle.update(z);oracle.update(z,.03)
        np.testing.assert_allclose(f.positions['Left'].P,oracle.P,atol=1e-14)
        self.assertFalse(np.array_equal(before,f.positions['Left'].P))
        self.assertEqual(f.last_observation_stamp['Right'],1.01)
        f.smooth_elbow('Left',z,1.)
        e=f.smooth_elbow('Left',np.ones(3),1.04)
        np.testing.assert_allclose(e,np.ones(3)*(1-np.exp(-1.)))
        f.bind((1,1,1));self.assertEqual(f.last_observation_stamp['Left'],1.03)
        f.bind((1,2,1));self.assertEqual(f.last_observation_stamp,{})

    def test_native_full_rotation_matches_oracle(self):
        import importlib.util,os
        import pose_velocity_kalman as native
        self.assertIsNotNone(native._NATIVE)
        os.environ['KUAVO_MAPPER_NUMERIC_BACKEND']='python'
        spec=importlib.util.spec_from_file_location('kalman_oracle',Path(native.__file__))
        oracle=importlib.util.module_from_spec(spec);spec.loader.exec_module(oracle)
        os.environ['KUAVO_MAPPER_NUMERIC_BACKEND']='native'
        for scenario in ('ordinary','fast','reverse','near_pi','sign'):
            a,b=native.RotationCV(),oracle.RotationCV()
            for i in range(400):
                dt=.006+(i%9)*.001
                angle=(.007 if scenario=='ordinary' else .25)*i
                if scenario=='reverse' and i>200:angle=100.-.25*i
                axis=np.array([.3,-.4,.5]);axis/=np.linalg.norm(axis)
                measured=oracle.exp_so3(axis*angle)
                if scenario=='sign':
                    from scipy.spatial.transform import Rotation
                    quat=Rotation.from_matrix(measured).as_quat()
                    measured=Rotation.from_quat(quat if i%2 else -quat).as_matrix()
                if scenario=='near_pi' and i==1:measured=oracle.exp_so3(axis*(np.pi-1e-7))
                R,w=a.update(measured,None if i==0 else dt)
                S,v=b.update(measured,None if i==0 else dt)
                self.assertLess(np.linalg.norm(oracle.log_so3(R@S.T)),2e-7,scenario)
                np.testing.assert_allclose(w,v,atol=2e-6,rtol=2e-6)
                np.testing.assert_allclose(a.P,b.P,atol=2e-8,rtol=2e-6)
                self.assertGreater(np.linalg.eigvalsh(a.P).min(),-1e-12)
            for delta in (0.,1e-9,1e-6):
                R=oracle.exp_so3(axis*(np.pi-delta))
                np.testing.assert_allclose(native._NATIVE.rotation_exp(native._NATIVE.rotation_log(R)),R,atol=2e-8)

    def test_missing_native_is_explicit_startup_failure(self):
        import os,subprocess,tempfile,shutil
        source=Path(__file__).resolve().parents[1]/'scripts/tools/pose_velocity_kalman.py'
        with tempfile.TemporaryDirectory() as d:
            shutil.copy(source,Path(d)/source.name)
            env=dict(os.environ,KUAVO_MAPPER_NUMERIC_BACKEND='native',PYTHONPATH=d)
            result=subprocess.run([sys.executable,'-c','import pose_velocity_kalman'],cwd=d,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('Mapper native extension is required',result.stderr)

if __name__=='__main__':unittest.main()
