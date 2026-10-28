#!/usr/bin/env python3
"""Numerical and input-lifetime regressions for fixed-size mapper algebra."""
import math
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]/'scripts/tools'))
from pose_velocity_kalman import (PositionCV, SkeletonCV, exp_so3,
                                  left_jacobian, mapped_rates, skew)


class MapperMathOptimizationTests(unittest.TestCase):
    def test_so3_against_matrix_formula_at_branch_and_wrap(self):
        rng = np.random.RandomState(51)
        for magnitude in (0., 1e-12, .999999e-6, 1e-6, 1.000001e-6,
                          .02, 1., math.pi-1e-8, math.pi, 2*math.pi):
            for _ in range(20):
                axis = rng.randn(3); axis /= np.linalg.norm(axis)
                vector = magnitude*axis
                angle = float(np.linalg.norm(vector)); V = skew(vector)
                if angle < 1e-6:
                    s, c, j = 1-angle*angle/6, .5-angle*angle/24, 1/6-angle*angle/120
                else:
                    s = math.sin(angle)/angle
                    c = (1-math.cos(angle))/(angle*angle)
                    j = (angle-math.sin(angle))/(angle**3)
                np.testing.assert_allclose(exp_so3(vector), np.eye(3)+s*V+c*(V@V), atol=3e-15, rtol=0.)
                np.testing.assert_allclose(left_jacobian(vector), np.eye(3)+c*V+j*(V@V), atol=3e-15, rtol=0.)

    def test_position_joseph_covariance_with_variable_dt_and_gain(self):
        rng = np.random.RandomState(73)
        for gain in (.01, .425, .65, 1.):
            observer = PositionCV(q=.63, velocity_correction_gain=gain)
            x = P = None
            for frame in range(1500):
                z = rng.randn(3)
                dt = None if frame % 271 == 0 else [1e-6, 1/120, 1/72, .099][frame % 4]
                if x is None or dt is None:
                    x = np.array([z, np.zeros(3)]); P = np.diag([observer.r, .25])
                else:
                    F = np.array([[1., dt], [0., 1.]])
                    Q = observer.q*np.array([[dt**3/3, dt*dt/2], [dt*dt/2, dt]])
                    x = F@x; P = F@P@F.T+Q
                    K = P[:, 0]/(P[0, 0]+observer.r); K[1] *= gain
                    x += K[:, None]*(z-x[0])
                    A = np.eye(2); A[:, 0] -= K
                    P = A@P@A.T+observer.r*np.outer(K, K); P = .5*(P+P.T)
                result = observer.update(z, dt)
                np.testing.assert_allclose(np.asarray(result), x, atol=2e-10, rtol=2e-13)
                np.testing.assert_allclose(observer.P, P, atol=2e-15, rtol=2e-13)
                self.assertGreater(np.linalg.eigvalsh(observer.P).min(), 0.)
                # Returned arrays must not alias the estimator.
                result[0][:] = 999.; result[1][:] = 999.
                np.testing.assert_allclose(observer.x, x, atol=2e-10, rtol=2e-13)

    def test_skeleton_rejection_does_not_partially_update_and_reset_contract(self):
        observer = SkeletonCV(q_p=.63, q_r=5.7, hand_q_r=1.42)
        transforms = {i: np.eye(4) for i in observer.indices}
        observer.update(transforms, 1., ('same', False))
        before = {i: f.x.copy() for i, f in observer.positions.items()}
        bad = {i: t.copy() for i, t in transforms.items()}
        bad[23][0, 3] = np.nan
        with self.assertRaises(ValueError): observer.update(bad, 1.01, ('same', False))
        self.assertEqual(observer.stamp, 1.)
        for i in before: np.testing.assert_array_equal(observer.positions[i].x, before[i])
        missing = dict(transforms); missing.pop(23)
        with self.assertRaises(KeyError): observer.update(missing, 1.01, ('same', False))
        for invalid in (float('nan'), float('inf')):
            with self.assertRaises(ValueError): observer.update(transforms, invalid, ('same', False))
        # Repeated stamps return the prior observation without validating or
        # updating covariance; a new equal-valued observation still updates.
        states = observer.update(missing, 1., ('same', False))
        self.assertIs(states, observer.states)
        self.assertEqual(observer.updates, 1)
        observer.update(transforms, 1.01, ('same', False))
        self.assertEqual(observer.updates, 2)
        for stamp, key in ((.9, ('same', False)), (2., ('same', False)),
                           (2.01, ('same', True))):
            observer.update(transforms, stamp, key)
            self.assertEqual(observer.updates, 1)
            for _, velocity, omega in observer.states.values():
                np.testing.assert_array_equal(velocity, np.zeros(3))
                np.testing.assert_array_equal(omega, np.zeros(3))

    def test_mapping_input_arrays_are_unchanged_with_wrist_offsets(self):
        rng = np.random.RandomState(91)
        states = []
        for _ in range(4):
            transform = np.eye(4); transform[:3, :3] = exp_so3(rng.randn(3))
            transform[:3, 3] = rng.randn(3)
            states.append((transform, rng.randn(3), rng.randn(3)))
        snapshots = [[a.copy() for a in state] for state in states]
        for offset in (None, np.array([.03, -.01, .02])):
            mapped_rates(*states, np.eye(3), .28, .45, states[3][0][:3, :3],
                         np.array([.01, .02, -.03]), offset, exp_so3([.1, .2, .3]))
        for state, snapshot in zip(states, snapshots):
            for actual, prior in zip(state, snapshot): np.testing.assert_array_equal(actual, prior)
        for bone in (1, 3):
            degenerate = [(T.copy(), v.copy(), w.copy()) for T, v, w in states]
            degenerate[bone][0][:3, 3] = degenerate[2][0][:3, 3]
            with self.assertRaises(ValueError):
                mapped_rates(*degenerate, np.eye(3), .28, .45,
                             states[3][0][:3, :3], np.zeros(3))


if __name__ == '__main__': unittest.main()
