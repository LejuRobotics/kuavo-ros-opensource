import sys
import unittest
from pathlib import Path

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from tools.incremental_clutch import HandClutch
from tools.pose_velocity_kalman import SkeletonCV, exp_so3, log_so3


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

    def test_both_grips_during_rise_with_held_chest_samples(self):
        # Quest can publish the palm at 120 Hz while repeating chest rotation
        # at 30 Hz. Rising and straightening the torso must not look like a
        # large wrist adjustment when both hands stay fixed relative to it.
        for hold_seconds in (0., .05):
            with self.subTest(hold_seconds=hold_seconds):
                observer = SkeletonCV(q_p=.63, q_r=5.7, hand_q_r=1.42,
                                      chest_sample_hold_seconds=hold_seconds)
                clutches = [HandClutch((1., 1., 1.), (1., 1., 1.), True, 1.)
                            for _ in range(2)]
                wrists = [np.array([.32, .22, -.24]), np.array([.32, -.22, -.24])]
                peak_shift = peak_angle = 0.

                def rise_at(stamp):
                    progress = np.clip((stamp - .5) / .45, 0., 1.)
                    return progress**2 * (3. - 2. * progress)

                def pitch(progress):
                    return exp_so3(np.array([0., .9 * (1. - progress), .2 * progress]))

                for frame in range(241):
                    t = frame / 120.
                    rise = rise_at(t)
                    rotation = pitch(rise)
                    origin = np.array([.13 * (1. - rise), 0., 1. + .35 * rise])
                    bones = {index: np.eye(4) for index in observer.indices}
                    bones[23][:3, :3] = pitch(rise_at((frame // 4) / 30.))
                    bones[23][:3, 3] = origin
                    # Move only the left wrist after the torso has settled.
                    hand_delta = .04 * np.clip((t - 1.4) / .25, 0., 1.)
                    for arm, wrist in enumerate(wrists):
                        bone = bones[4 + arm]
                        bone[:3, :3] = rotation
                        bone[:3, 3] = origin + rotation @ (
                            wrist + np.array([hand_delta if arm == 0 else 0., 0., 0.]))

                    states = observer.update(bones, t, 'follow')
                    chest, v_chest, w_chest = states[23]
                    axes = chest[:3, :3].T
                    gripping = t >= .25
                    for arm, clutch in enumerate(clutches):
                        palm, v_palm, w_palm = states[4 + arm]
                        delta = palm[:3, 3] - chest[:3, 3]
                        target, target_rotation, _, _ = clutch.sample(
                            'follow', 2 if gripping else 1, gripping,
                            axes @ delta, axes @ palm[:3, :3],
                            axes @ (v_palm - v_chest - np.cross(w_chest, delta)),
                            axes @ (w_palm - w_chest),
                            (self.seed, self.robot), np.zeros(3), t)
                        if t < 1.4:
                            peak_shift = max(peak_shift, np.linalg.norm(target - self.seed))
                            peak_angle = max(peak_angle, np.linalg.norm(
                                log_so3(target_rotation @ self.robot.T)))

                if hold_seconds:
                    self.assertLess(peak_shift, .010)
                    self.assertLess(peak_angle, np.deg2rad(1.5))
                else:
                    self.assertGreater(peak_shift, .015)
                    self.assertGreater(peak_angle, np.deg2rad(2.5))
                np.testing.assert_allclose(clutches[0].position, self.seed + [.04, 0., 0.],
                                           atol=1.e-5)
                np.testing.assert_allclose(clutches[1].position, self.seed, atol=1.e-5)

    def test_held_chest_sample_preserves_measurement_clock_until_refresh(self):
        observer = SkeletonCV(chest_sample_hold_seconds=.05)
        bones = {index: np.eye(4) for index in observer.indices}
        observer.update(bones, 0., 'follow')
        bones[23][:3, :3] = exp_so3(np.array([0., .08, 0.]))
        rotation, _, _ = observer.update(bones, .033, 'follow')[23]
        covariance = observer.rotations[23].P.copy()
        predicted, _, _ = observer.update(bones, .041, 'follow')[23]
        self.assertGreater(np.linalg.norm(predicted - rotation), 0.)
        self.assertEqual(observer.chest_rotation_stamp, .033)
        np.testing.assert_array_equal(observer.rotations[23].P, covariance)

        observer.update(bones, .084, 'follow')
        self.assertEqual(observer.chest_rotation_stamp, .084)
        for stamp, key in ((.200, 'follow'), (.208, 'new-session')):
            pose, _, omega = observer.update(bones, stamp, key)[23]
            np.testing.assert_allclose(pose, bones[23])
            np.testing.assert_array_equal(omega, np.zeros(3))


if __name__ == '__main__':
    unittest.main()
