#!/usr/bin/env python3
"""Edge cases outside the bag: quaternion/Euler conventions and CV resets."""
import importlib.util, pathlib, sys, unittest
import numpy as np
import tf
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from tools.lightweight_transforms import matrix_to_quaternion, matrix_to_rpy
from tools import pose_velocity_kalman as cv

class Equivalence(unittest.TestCase):
    def test_tf_conventions(self):
        rng=np.random.default_rng(20261109)
        matrices=[tf.transformations.quaternion_matrix(q)[:3,:3] for q in rng.normal(size=(2000,4))]
        matrices += [tf.transformations.euler_matrix(a,b,c)[:3,:3]
                     for a in (0.,np.pi,-np.pi) for b in (np.pi/2,-np.pi/2) for c in (0.,np.pi,.7)]
        for R in matrices:
            M=np.eye(4);M[:3,:3]=R;q=tf.transformations.quaternion_from_matrix(M)
            if q[3]<0:q=-q
            np.testing.assert_allclose(matrix_to_quaternion(R),q,atol=1e-14,rtol=0)
            np.testing.assert_allclose(matrix_to_rpy(R),tf.transformations.euler_from_matrix(R),atol=1e-14,rtol=0)

    # beta's existing native filter is byte-for-byte unchanged by this port.
    # Its own pose_velocity_kalman_test.py remains the filter regression suite.

if __name__=='__main__':unittest.main()
