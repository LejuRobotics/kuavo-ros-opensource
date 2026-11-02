#!/usr/bin/env python3
"""Numerical and input-lifetime regressions for fixed-size mapper algebra."""
import math
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]/'scripts/tools'))
from pose_velocity_kalman import (PositionCV, HandTargetCV, exp_so3,
                                  left_jacobian, skew)


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
            observer = PositionCV(q=.63, velocity_gain=gain)
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

    def test_hand_reset_repeated_stamps_and_input_lifetime(self):
        observer = HandTargetCV()
        observer.bind(('controller', 1))
        position = np.array([.1, .2, .3]); rotation = exp_so3([.1, -.2, .3])
        saved = (position.copy(), rotation.copy())
        observer.update(0, position, rotation, 1.)
        state = observer.update(0, position + .01, rotation, 1.01)
        repeated = observer.update(0, position + 10., rotation, 1.01)
        for actual, expected in zip(repeated, state):
            np.testing.assert_array_equal(actual, expected)
        np.testing.assert_array_equal(position, saved[0])
        np.testing.assert_array_equal(rotation, saved[1])
        repeated[0][:] = 999.
        self.assertLess(observer.positions[0].x[0].max(), 1.)
        observer.bind(('controller', 2))
        self.assertEqual(observer.positions, {})
        _, _, velocity, omega = observer.update(0, position, rotation, 1.02)
        np.testing.assert_array_equal(velocity, np.zeros(3))
        np.testing.assert_array_equal(omega, np.zeros(3))


if __name__ == '__main__': unittest.main()
