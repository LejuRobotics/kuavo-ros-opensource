#!/usr/bin/env python3
"""Actual CV observer linearization, sine validation, and configured jerk alignment."""
import json
import pathlib
import sys
import unittest
import numpy as np
import yaml
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts/tools'))
from pose_velocity_kalman import PositionCV, RotationCV, exp_so3, log_so3


def transfer(q, sigma, gain, hz, frequencies):
    observer = PositionCV(sigma, q, gain)
    dt = 1./hz
    for _ in range(600): observer.update([0., 0., 0.], dt)
    F = np.array([[1., dt], [0., 1.]])
    Q = q*np.array([[dt**3/3, dt**2/2], [dt**2/2, dt]])
    P = F@observer.P@F.T+Q
    K = P[:, 0]/(P[0, 0]+sigma**2)
    K[1] *= gain
    A = (np.eye(2)-np.outer(K, [1., 0.]))@F
    assert max(abs(np.linalg.eigvals(A))) < 1.
    return np.array([np.linalg.solve(np.eye(2)-A*np.exp(-2j*np.pi*f*dt), K)[0] for f in frequencies])


class FrequencyTests(unittest.TestCase):
    def test_candidate_frequency_and_actual_rotation(self):
        config = yaml.safe_load((ROOT/'config/vr_kalman.yaml').read_text())
        candidate = config['vr_kalman_flattening']
        self.assertFalse(candidate['enabled'])  # broadband noise tradeoff is explicit
        frequencies = np.r_[np.linspace(.1, 5., 100), 10., 15., 20., 30., 40.]
        report = []
        for hz in (72, 90, 120):
            for name, qkey, sigmakey, gkey in (
                ('position', 'position_q', 'position_sigma', 'position_velocity_gain'),
                ('chest', 'rotation_q', 'rotation_sigma', 'rotation_velocity_gain'),
                ('palms', 'hand_rotation_q', 'rotation_sigma', 'hand_velocity_gain')):
                q, sigma = config['vr_kalman_'+qkey], config['vr_kalman_'+sigmakey]
                before = transfer(q, sigma, 1., hz, frequencies)
                after = transfer(candidate[qkey], sigma, candidate[gkey], hz, frequencies)
                delay = (-np.angle(after[:100])+np.angle(before[:100]))/(2*np.pi*frequencies[:100])*1000
                self.assertLessEqual(abs(after[100]), 1.03)
                self.assertLess(max(delay), 1.)
                row = dict(hz=hz, signal=name, gain10_before=abs(before[100]), gain10_after=abs(after[100]),
                           max_added_delay_below5_ms=max(delay), gain30_ratio=abs(after[103]/before[103]))
                report.append(row)
                if name != 'position':
                    dt=1./hz;f=10.;t=np.arange(0,6,dt)
                    observer=RotationCV(sigma,candidate[qkey],candidate[gkey]);out=[]
                    for stamp in t:
                        R,_=observer.update(exp_so3([.001*np.sin(2*np.pi*f*stamp),0.,0.]),dt)
                        out.append(log_so3(R)[0])
                    keep=t>2.;X=np.array([np.sin(2*np.pi*f*t[keep]),np.cos(2*np.pi*f*t[keep])]).T
                    coeff=np.linalg.lstsq(X,np.asarray(out)[keep]/.001,rcond=None)[0]
                    actual=coeff[0]+1j*coeff[1]
                    self.assertLess(abs(actual-after[100]), 1e-5)
        print(json.dumps(report, indent=2))

    def test_regularized_observer_constant_velocity_reset_and_covariance(self):
        for gain in (.425,.65,1.):
            p=PositionCV(q=.7875,velocity_correction_gain=gain)
            r=RotationCV(q=7.125,velocity_correction_gain=gain)
            t=0.
            for i in range(800):
                dt=[1/72,1/90,1/120][i%3];t+=dt
                _,v=p.update([.3*t,0.,0.],dt)
                R,w=r.update(exp_so3([.2*t,0.,0.]),dt)
                self.assertGreater(np.linalg.eigvalsh(p.P).min(),0.)
                self.assertGreater(np.linalg.eigvalsh(r.P).min(),0.)
            np.testing.assert_allclose(v,[.3,0,0],atol=1e-8)
            np.testing.assert_allclose(w,[.2,0,0],atol=1e-7)
            self.assertLess(np.linalg.norm(R.T@R-np.eye(3)),1e-10)
            np.testing.assert_array_equal(p.update([1,2,3],None)[1],np.zeros(3))
        for invalid in (0.,-1.,1.1,float('nan')):
            with self.assertRaises(ValueError): PositionCV(velocity_correction_gain=invalid)

    def test_configured_jerk_alignment(self):
        rnea=yaml.safe_load((ROOT/'config/rnea.yaml').read_text())
        fixed=yaml.safe_load((ROOT/'config/fixed_base_18d.yaml').read_text())
        j3=yaml.safe_load((ROOT/'config/j3_overshoot_profile.yaml').read_text())
        limits=rnea['jerk_limit'];self.assertEqual(len(limits),18)
        for arm in range(2):
            for joint in range(7):
                jerk=25000.
                if joint==2: jerk=j3['joint_dynamic_overrides']['zarm_l3_joint']['jerk']
                if joint>=4: jerk=fixed['wrist_envelope']['j'+str(joint+1)]['jerk']
                self.assertEqual(limits[4+arm*7+joint],jerk*rnea['acceleration_gain'])
        self.assertEqual(rnea['velocity_cutoff_hz'],60.)
        # Exact sampled 1-pole, not the continuous-time 1/(2*pi*fc) approximation.
        for freq in (5.,15.,30.,40.):
            h=[]
            for cutoff in (120.,60.):
                pole=np.exp(-2*np.pi*cutoff*.002)
                h.append((1-pole)/(1-pole*np.exp(-2j*np.pi*freq*.002)))
            print('RNEA f=%g gain120=%g gain60=%g added_phase_delay_ms=%g' %
                  (freq,abs(h[0]),abs(h[1]),(-np.angle(h[1])+np.angle(h[0]))/(2*np.pi*freq)*1000))

if __name__=='__main__':unittest.main()
