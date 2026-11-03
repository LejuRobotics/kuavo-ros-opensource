"""Offline timing regression checks; no ROS master or robot connection."""
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import cam_proj_validation as projection


class ProjectionTimingTests(unittest.TestCase):
    def test_default_header_matching_rejects_unsynchronized_source_clocks(self):
        history = projection.JointHistory()
        history.append(1061.188, (0.1,), received=50.002)
        with self.assertRaises(ValueError):
            history.match(1100.0, 50.0, 0.1, now=50.03)

    def test_explicit_receipt_matching_preserves_evidence_of_header_offset(self):
        history = projection.JointHistory()
        angles = (0.1, -0.2)
        history.append(1061.188, angles, received=50.002)
        values, stamp, skew, timing = history.match(
            1100.0, 50.0, 0.1, mode="receipt", now=50.03)
        self.assertEqual(values, angles)
        self.assertEqual(stamp, 1061.188)
        self.assertAlmostEqual(skew, 0.002)
        self.assertEqual(timing["sync_mode"], "receipt")
        self.assertAlmostEqual(timing["header_skew_ms"], -38812.0)
        self.assertAlmostEqual(timing["receipt_skew_ms"], 2.0)
        self.assertAlmostEqual(timing["matching_skew_ms"], 2.0)
        self.assertAlmostEqual(timing["image_arrival_age_ms"], 30.0)
        self.assertAlmostEqual(timing["joint_arrival_age_ms"], 28.0)
        self.assertEqual(timing["capture_time_alignment"], "unverified_receipt")
        self.assertFalse(timing["header_stamps_within_tolerance"])

    def test_receipt_matching_keeps_100_ms_limit_in_both_directions(self):
        for received in (49.85, 50.15):
            with self.subTest(joint_received=received):
                history = projection.JointHistory()
                history.append(100.0, (0.1,), received=received)
                with self.assertRaises(ValueError):
                    history.match(100.0, 50.0, 0.1, mode="receipt", now=50.2)

    def test_stale_image_or_feedback_is_rejected_even_with_loose_skew(self):
        for mode in ("header", "receipt"):
            for image_received, joint_received in ((19.0, 20.0), (20.0, 19.0)):
                with self.subTest(mode=mode, image_received=image_received,
                                  joint_received=joint_received):
                    history = projection.JointHistory()
                    history.append(100.0, (0.1,), received=joint_received)
                    with self.assertRaises(ValueError):
                        history.match(100.0, image_received, 1.1, mode=mode,
                                      stale_after=0.5, now=20.1)

    def test_fresh_latest_feedback_does_not_make_selected_old_sample_valid(self):
        for mode in ("header", "receipt"):
            with self.subTest(mode=mode):
                history = projection.JointHistory()
                history.append(100.0, (0.1,), received=10.98)
                history.append(100.3, (0.2,), received=11.25)
                # The image is fresh, but its nearest sample is already stale.
                with self.assertRaises(ValueError):
                    history.match(100.0, 11.05, 0.1, mode=mode,
                                  stale_after=1.0, now=12.0)

    def test_receipt_matching_uses_image_arrival_not_later_render_time(self):
        history = projection.JointHistory()
        history.append(100.0, (0.1,), received=50.002)
        history.append(100.2, (0.9,), received=50.2)
        values, stamp, skew, timing = history.match(
            138.812, 50.0, 0.1, mode="receipt", now=50.21)
        self.assertEqual(values, (0.1,))
        self.assertEqual(stamp, 100.0)
        self.assertAlmostEqual(skew, 0.002)
        self.assertAlmostEqual(timing["image_arrival_age_ms"], 210.0)

    def test_default_header_matching_still_selects_by_source_stamp(self):
        history = projection.JointHistory()
        history.append(1000.010, (0.1,), received=12.01)
        history.append(1000.040, (0.9,), received=12.08)
        values, stamp, skew, timing = history.match(
            1000.012, 12.08, 0.1, now=12.09)
        self.assertEqual(values, (0.1,))
        self.assertEqual(stamp, 1000.010)
        self.assertAlmostEqual(skew, -0.002)
        self.assertEqual(timing["sync_mode"], "header")
        self.assertAlmostEqual(timing["header_skew_ms"], -2.0)
        self.assertAlmostEqual(timing["receipt_skew_ms"], -70.0)
        self.assertAlmostEqual(timing["matching_skew_ms"], -2.0)
        self.assertEqual(timing["capture_time_alignment"], "header")
        self.assertTrue(timing["header_stamps_within_tolerance"])

    def test_history_bound_and_header_reset_apply_to_both_time_bases(self):
        history = projection.JointHistory(size=2)
        for stamp, received in ((100.0, 10.0), (100.1, 10.1), (100.2, 10.2)):
            history.append(stamp, (stamp,), received=received)
        self.assertEqual(len(history.samples), 2)
        self.assertEqual(len(history.arrivals), 2)
        self.assertEqual(history.samples[0], (100.1, (100.1,)))
        self.assertEqual(history.arrivals[0], (100.1, 10.1, (100.1,)))
        with self.assertRaises(ValueError):
            history.match(100.0, 10.0, 0.03, mode="receipt", now=10.21)
        # A source clock reset must not leave the preceding epoch selectable.
        history.append(1.0, (7.0,), received=10.3)
        self.assertEqual(list(history.samples), [(1.0, (7.0,))])
        self.assertEqual(list(history.arrivals), [(1.0, 10.3, (7.0,))])
        with self.assertRaises(ValueError):
            history.match(100.1, 10.1, 0.03, mode="receipt", now=10.31)
        values, stamp, _, _ = history.match(
            1.0, 10.3, 0.03, mode="receipt", now=10.31)
        self.assertEqual((values, stamp), ((7.0,), 1.0))


if __name__ == "__main__":
    unittest.main()
