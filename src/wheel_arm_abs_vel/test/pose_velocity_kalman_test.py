#!/usr/bin/env python3
"""Offline numeric contracts; no ROS/hardware required."""
import pathlib
import io
import os
import subprocess
import sys
import unittest
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]/'scripts/tools'))
from pose_velocity_kalman import (PositionCV, RotationCV, SkeletonCV,
                                  exp_so3, log_so3, mapped_rates)


class ObserverTests(unittest.TestCase):
    def test_palm_noise_split_and_reset(self):
        f = SkeletonCV(hand_q_r=.45)
        T = {i: np.eye(4) for i in f.indices}
        for stamp in (1., 1.014, 2.):
            f.update(T, stamp, 'same')
            self.assertEqual(f.rotations[4].q, .45)
            self.assertEqual(f.rotations[5].q, .45)
            self.assertEqual(f.rotations[23].q, 1.8)
            np.testing.assert_array_equal(f.states[4][2], f.states[5][2])
        with self.assertRaises(ValueError): SkeletonCV(hand_q_r=0.)

    def test_palm_rotation_noise_and_stop(self):
        dt=1/72.; t=np.arange(0,10,dt)
        outputs=[]
        for process_q in (1.8, .45, .2):
            f=RotationCV(q=process_q); out=[]
            for i,stamp in enumerate(t):
                angle=.35*np.sin(2*np.pi*stamp)+.006*np.sin(2*np.pi*15*stamp)
                _,w=f.update(exp_so3([angle,0,0]),None if i==0 else dt)
                out.append(w[0])
            M=np.array([np.sin(2*np.pi*t),np.cos(2*np.pi*t),
                        np.sin(2*np.pi*15*t),np.cos(2*np.pi*15*t)]).T
            c=np.linalg.lstsq(M[144:],np.array(out)[144:],rcond=None)[0]
            gain=np.linalg.norm(c[:2])/(.35*2*np.pi)
            lag=-np.arctan2(-c[0],c[1])/(2*np.pi)
            outputs.append(np.linalg.norm(c[2:]))
            print('palm q=%.2f 1Hz velocity_gain=%.4f phase_s=%.4f 15Hz_velocity=%.4f'%
                  (process_q,gain,lag,outputs[-1]))
            self.assertGreater(gain,.9);self.assertLess(gain,1.1)
            for _ in range(144): R,w=f.update(np.eye(3),dt)
            self.assertLess(np.linalg.norm(w),1e-6)
            self.assertLess(np.linalg.norm(log_so3(R)),1e-6)
        self.assertLess(outputs[1],.75*outputs[0])
        self.assertLess(outputs[2],outputs[1])

    def test_constant_velocity_variable_dt(self):
        f = PositionCV(); t = 0.
        for i in range(500):
            dt = .012+(i%5)*.001; t += dt
            p, v = f.update(np.array([.3, -.2, .1])*t, None if i==0 else dt)
        np.testing.assert_allclose(v, [.3, -.2, .1], atol=1e-8)
        self.assertGreater(np.linalg.eigvalsh(f.P).min(), 0)

    def test_rotation_and_wrap(self):
        f = RotationCV(); w = np.array([.3, -.7, 1.2]); dt = 1/72
        for i in range(600):
            R, omega = f.update(exp_so3(w*i*dt), None if i==0 else dt)
        np.testing.assert_allclose(omega, w, atol=1e-6)
        np.testing.assert_allclose(R.T@R, np.eye(3), atol=1e-10)
        self.assertGreater(np.linalg.eigvalsh(f.P).min(), 0)
        for axis in np.eye(3):
            v = np.pi*axis
            np.testing.assert_allclose(exp_so3(log_so3(exp_so3(v))), exp_so3(v), atol=1e-8)

    def test_equal_new_observation_and_reset(self):
        f = SkeletonCV(); T = {i: np.eye(4) for i in f.indices}
        f.update(T, 1., 'a'); f.update(T, 1., 'a')
        self.assertEqual(f.updates, 1)
        f.update(T, 1.014, 'a'); self.assertEqual(f.updates, 2)
        f.update(T, 1.028, 'b'); self.assertEqual(f.updates, 1)
        f.update(T, 2., 'b'); self.assertEqual(f.updates, 1)

    def test_low_frequency_and_noise(self):
        dt=1/72; t=np.arange(0,12,dt); f=PositionCV()
        raw=.10*np.sin(2*np.pi*t)+.001*np.sin(2*np.pi*20*t)
        out=[]
        for i,z in enumerate(raw): out.append(f.update([z,0,0],None if i==0 else dt)[0][0])
        M=np.array([np.sin(2*np.pi*t),np.cos(2*np.pi*t),
                    np.sin(2*np.pi*20*t),np.cos(2*np.pi*20*t)]).T
        c=np.linalg.lstsq(M[144:],np.array(out)[144:],rcond=None)[0]
        gain=np.linalg.norm(c[:2])/.1; lag=-np.arctan2(c[1],c[0])/(2*np.pi)
        noise=np.linalg.norm(c[2:])/.001
        print('position 1Hz gain=%.4f lag_ms=%.3f 20Hz_gain=%.4f'%(gain,lag*1000,noise))
        self.assertGreater(gain,.95);self.assertLess(gain,1.15)
        self.assertLess(abs(lag),.025);self.assertLess(noise,.70)

    def test_mapping_derivative_and_rigid_motion(self):
        rng=np.random.RandomState(7)
        states=[]
        for i in range(4):
            T=np.eye(4);T[:3,:3]=exp_so3(rng.randn(3)*.2);T[:3,3]=rng.randn(3)
            states.append((T,rng.randn(3)*.2,rng.randn(3)*.3))
        axes=exp_so3([.4,.2,-.3]);fixed=exp_so3([.1,.7,.2]);offset=np.array([.03,.02,-.04])
        def evaluate(s):
            c,sh,e,h=[a[0] for a in s];rem=axes@c[:3,:3].T
            d=e[:3,3]-sh[:3,3];d2=h[:3,3]-e[:3,3]
            hint=.28*rem@(d/np.linalg.norm(d))
            R=rem@h[:3,:3]@fixed
            return hint+.45*rem@(d2/np.linalg.norm(d2))+R@offset,R,hint
        v,w,e=mapped_rates(*states,axes,.28,.45,states[3][0][:3,:3]@fixed,offset)
        h=1e-6; shifted=[]
        for T,vel,omega in states:
            U=T.copy();U[:3,3]+=h*vel;U[:3,:3]=exp_so3(h*omega)@T[:3,:3]
            shifted.append((U,vel,omega))
        p0,R0,e0=evaluate(states);p1,R1,e1=evaluate(shifted)
        np.testing.assert_allclose(v,(p1-p0)/h,atol=2e-6)
        np.testing.assert_allclose(w,log_so3(R1@R0.T)/h,atol=2e-6)
        np.testing.assert_allclose(e,(e1-e0)/h,atol=2e-6)
        spin=np.array([.1,.2,.3]);vel=np.array([.3,.4,.5])
        rigid=[(T,vel+np.cross(spin,T[:3,3]),spin) for T,_,_ in states]
        for rate in mapped_rates(*rigid,axes,.28,.45,states[3][0][:3,:3]@fixed,offset):
            np.testing.assert_allclose(rate,0,atol=1e-12)

    def test_wrist_mapping_derivative(self):
        from target_geometry_contract import map_wrist_pose
        rng=np.random.RandomState(11); states=[]
        for i in range(4):
            T=np.eye(4);T[:3,:3]=exp_so3(rng.randn(3)*.2);T[:3,3]=rng.randn(3)
            states.append((T,rng.randn(3)*.2,rng.randn(3)*.3))
        X=np.eye(4);X[:3,:3]=exp_so3([.1,.2,.3]);X[:3,3]=[.03,.01,-.02]
        Y=np.eye(4);Y[:3,3]=[.02,-.01,.04];W=exp_so3([.2,-.1,.3])
        arm=dict(source_to_wrist=X,joint7_to_tcp=Y,wrist_to_joint7_rotation=W,forearm_to_joint7_m=.3)
        def evaluate(s):
            return map_wrist_pose(s[0][0],s[1][0][:3,3],s[2][0][:3,3],s[3][0],np.eye(3),np.zeros(3),.28,arm)
        v,w,e=mapped_rates(*states,np.eye(3),.28,.3,states[3][0][:3,:3],Y[:3,3],X[:3,3],X[:3,:3]@W)
        h=1e-6; shifted=[]
        for T,vel,omega in states:
            U=T.copy();U[:3,3]+=h*vel;U[:3,:3]=exp_so3(h*omega)@T[:3,:3]
            shifted.append((U,vel,omega))
        T0,e0,_=evaluate(states);T1,e1,_=evaluate(shifted)
        np.testing.assert_allclose(v,(T1[:3,3]-T0[:3,3])/h,atol=2e-6)
        np.testing.assert_allclose(w,log_so3(T1[:3,:3]@T0[:3,:3].T)/h,atol=2e-6)
        np.testing.assert_allclose(e,(e1-e0)/h,atol=2e-6)

    def test_reconstructed_hand_chain(self):
        # Numeric replay of the production exact CV-input reference step.
        # No future frames: 72 Hz measurements evaluated at 500 Hz ticks.
        dt=.002; times=np.arange(0,10,dt)
        # Fixed high-activity/A-B baseline restored to 12Hz and <=1Hz fidelity.
        for frequency in (.2,.5,1.,2.,4.,10.):
            f=PositionCV();stamp=-1.;out=[];p=0.;v=0.;w=2*np.pi*12
            for t in times:
                source=np.floor((t+1e-10)*72)/72
                if source>stamp:
                    measurement=.1*np.sin(2*np.pi*frequency*source)+.001*np.sin(2*np.pi*20*source)
                    pos,vel=f.update([measurement,0,0],None if stamp<0 else source-stamp)
                    stamp=source
                target=pos[0]+vel[0]*(t-stamp)
                guide=.1*vel[0]
                error=p-(target-guide*dt);d=v-guide;c=d+w*error
                p=target+(error+c*dt)*np.exp(-w*dt)
                v=guide+(d-w*c*dt)*np.exp(-w*dt)
                out.append(p)
            M=np.array([np.sin(2*np.pi*frequency*times),np.cos(2*np.pi*frequency*times)]).T
            c=np.linalg.lstsq(M[1000:],np.array(out)[1000:],rcond=None)[0]
            gain=np.linalg.norm(c)/.1;lag=-np.arctan2(c[1],c[0])/(2*np.pi*frequency)
            print('hand chain %.1fHz gain=%.4f lag_ms=%.3f'%(frequency,gain,lag*1000))
            if frequency<=1.:
                self.assertGreater(gain,.95);self.assertLess(gain,1.05);self.assertLess(abs(lag),.025)
            self.assertLess(gain,1.15)

    @unittest.skipUnless(os.environ.get('ADAPTIVE_REFERENCE_RUNNER'), 'C++ runner supplied by CTest')
    def test_adaptive_reconstructed_hand_chain(self):
        # Exercise the real C++ filter after 72Hz Kalman and causal prediction.
        dt=.002;times=np.arange(0,12,dt)
        for amplitude in (.001,.1):
            f=PositionCV();stamp=-1.;rows=[]
            for t in times:
                source=np.floor((t+1e-10)*72)/72
                if source>stamp:
                    z=amplitude*np.sin(2*np.pi*source)
                    pos,vel=f.update([z,0,0],None if stamp<0 else source-stamp);stamp=source
                rows.append([dt,*(pos+vel*(t-stamp)),*vel,0,0,0,1,0,0,0])
            data=io.StringIO();np.savetxt(data,rows,fmt='%.12g')
            run=subprocess.run([os.environ['ADAPTIVE_REFERENCE_RUNNER'],'adaptive'],
                               input=data.getvalue(),stdout=subprocess.PIPE,text=True,check=True)
            out=np.loadtxt(io.StringIO(run.stdout))[:,0]
            M=np.column_stack([np.sin(2*np.pi*times),np.cos(2*np.pi*times)])
            coef=np.linalg.lstsq(M[2000:],out[2000:],rcond=None)[0]
            gain=np.linalg.norm(coef)/amplitude;lag=-np.arctan2(coef[1],coef[0])/(2*np.pi)
            print('adaptive 1Hz amplitude_mm=%.1f gain=%.4f lag_ms=%.3f'%(amplitude*1000,gain,lag*1000))
            if amplitude>.01:
                self.assertGreater(gain,.95);self.assertLess(gain,1.05);self.assertLess(abs(lag),.035)
            else:
                self.assertGreater(gain,.1);self.assertLess(gain,.6)


if __name__=='__main__': unittest.main()
