import sys
import unittest
from pathlib import Path

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from tools.incremental_clutch import HandClutch
from tools.pose_velocity_kalman import exp_so3


class ClutchTest(unittest.TestCase):
    def setUp(self):
        self.clutch = HandClutch((1.26, 1, 2), (1, 1, 1), True, .6)
        self.seed = np.array([.2, 0., 0.])
        self.robot = exp_so3(np.array([.4, -.3, .1]))
        self.hand = exp_so3(np.array([.1, .2, .3]))

    def sample(self, x, seq, grip=True, hand=None, key=1):
        return self.clutch.sample(key, seq, grip, np.array([x, 0., 0.]),
                                  self.hand if hand is None else hand,
                                  np.array([.05, 0., 0.]), np.ones(3),
                                  (self.seed, self.robot), np.zeros(3))

    def test_clutch_and_rotation(self):
        p, R, v, w = self.sample(.15, 1)
        np.testing.assert_array_equal(p, self.seed)
        np.testing.assert_array_equal(v, np.zeros(3))
        delta = exp_so3(np.array([0., .5, 0.]))
        p, R, v, w = self.sample(.15, 1, hand=delta @ self.hand)
        np.testing.assert_array_equal(p, self.seed)
        np.testing.assert_allclose(R, delta @ self.robot, atol=1e-14)
        p, _, v, _ = self.sample(.25, 1)
        self.assertAlmostEqual(p[0], .326)
        self.assertAlmostEqual(v[0], .063)
        for x, seq, grip in ((.25, 2, False), (-.35, 2, False), (-.35, 3, True)):
            pose, _, v, w = self.sample(x, seq, grip)
            np.testing.assert_array_equal(pose, p)
            np.testing.assert_array_equal(v, np.zeros(3))
            np.testing.assert_array_equal(w, np.zeros(3))
        self.assertAlmostEqual(self.sample(-.34, 3)[0][0], .3386)
        np.testing.assert_array_equal(self.sample(.3, 3, key=2)[0], self.seed)

    def test_workspace_return(self):
        self.sample(0, 1)
        for x in (1., 1.5, 2.):
            p, _, v, _ = self.sample(x, 1)
            self.assertAlmostEqual(p[0], .6)
            self.assertAlmostEqual(v[0], 0.)
        self.assertAlmostEqual(self.sample(1.99, 1)[0][0], .5874)


if __name__ == '__main__':
    unittest.main()
