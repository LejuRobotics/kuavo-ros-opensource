"""Independent transform constructions exercise capture stability diagnostics."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parents[1]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
from mocap_stability import analyze_capture


def transform(xyz, rpy):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def pose(matrix):
    return {"xyz_mm": matrix[:3, 3].tolist(),
            "quaternion_xyzw": Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
            "valid_frames": 90}


def capture_with_changes(translation=0.0, rotation=0.0, changed_arm="left"):
    samples = []
    for row, index in enumerate((5, 2, 9)):
        channels = {}
        for arm, hand in (("left", "l_hand"), ("right", "r_hand")):
            # A large constant rotation/translation and moving hand should not
            # count as instability. Only differences between samples are measured.
            torso_shoulder = transform([100, 600, -200], [23, -17, 58])
            if row == 2 and arm == changed_arm:
                torso_shoulder = torso_shoulder @ transform([translation, 0, 0], [rotation, 0, 0])
            torso_hand = transform([100 + 50 * row, -340 + row * 20, 90],
                                   [10 * row + 20, -45, 15 * row])
            shoulder_hand = np.linalg.inv(torso_shoulder) @ torso_hand
            channels[f"{hand}_in_torso"] = pose(torso_hand)
            channels[f"{hand}_in_l_shoulder"] = pose(shoulder_hand)
        samples.append({"index": index, "bodies": channels})
    return {"samples": samples}


class StabilityTests(unittest.TestCase):
    def analyze(self, capture, **kwargs):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.json"
            path.write_text(json.dumps(capture), encoding="utf-8")
            report = analyze_capture(path, **kwargs)
            json.dumps(report, allow_nan=False)
            return report

    def test_fixed_transform_and_moving_hands_have_zero_variation(self):
        report = self.analyze(capture_with_changes())
        self.assertNotIn("mode", report)
        self.assertEqual(report["sample_count"], 3)
        for arm in report["arms"].values():
            self.assertLess(arm["max_translation_mm"], 1e-10)
            self.assertLess(arm["max_rotation_deg"], 1e-10)

    def test_either_arm_or_metric_reports_exceedance_without_selecting_mode(self):
        for arm in ("left", "right"):
            for changes, metric, expected in (({"translation": 15}, "max_translation_mm", 10),
                                               ({"rotation": 2}, "max_rotation_deg", 2)):
                with self.subTest(arm=arm, changes=changes):
                    report = self.analyze(capture_with_changes(changed_arm=arm, **changes))
                    self.assertNotIn("mode", report)
                    flag = "exceeds_translation" if "translation" in changes else "exceeds_rotation"
                    self.assertTrue(report["arms"][arm][flag])
                    self.assertAlmostEqual(report["arms"][arm][metric], expected, places=10)
                    if "translation" in changes:
                        self.assertEqual(report["arms"][arm]["max_translation_pose_index"], 9)
                    else:
                        self.assertIn(9, report["arms"][arm]["rotation_pair_indices"])

    def test_configurable_thresholds(self):
        report = self.analyze(capture_with_changes(translation=12, rotation=2),
                              max_translation_mm=15, max_rotation_deg=3)
        self.assertNotIn("mode", report)
        self.assertFalse(report["arms"]["left"]["exceeds_translation"])
        self.assertFalse(report["arms"]["left"]["exceeds_rotation"])

    def test_default_eight_mm_boundary_and_original_mean_deviations(self):
        for distance, expected in ((11.9, False), (12.1, True)):
            with self.subTest(distance=distance):
                report = self.analyze(capture_with_changes(translation=distance))
                self.assertNotIn("mode", report)
                self.assertIs(report["arms"]["left"]["exceeds_translation"], expected)
                self.assertEqual(report["thresholds"]["max_translation_mm"], 8.0)
                # Radii are distance/3, distance/3, 2*distance/3.
                self.assertAlmostEqual(report["arms"]["left"]["mean_translation_mm"],
                                       distance * 4 / 9, places=10)
                self.assertAlmostEqual(report["arms"]["left"]["pairwise"]["mean_translation_mm"],
                                       distance * 2 / 3, places=10)
        report = self.analyze(capture_with_changes(rotation=0.9))
        self.assertAlmostEqual(report["arms"]["left"]["mean_rotation_deg"], 0.6, places=10)

    def test_recorded_hand_offsets_cancel_without_urdf_or_body_metadata(self):
        capture = capture_with_changes()
        hand_offset = transform([70, -30, 100], [0, 0, 0])
        for sample in capture["samples"]:
            for observation in sample["bodies"].values():
                matrix = np.eye(4)
                matrix[:3, :3] = Rotation.from_quat(observation["quaternion_xyzw"]).as_matrix()
                matrix[:3, 3] = observation["xyz_mm"]
                observation.update(pose(matrix @ hand_offset))
        report = self.analyze(capture)
        self.assertNotIn("mode", report)
        self.assertLess(report["arms"]["left"]["max_translation_mm"], 1e-10)

    def test_invalid_or_missing_channel_rejects_entire_capture(self):
        for key in ("l_hand_in_torso", "r_hand_in_torso", "l_hand_in_l_shoulder", "r_hand_in_l_shoulder"):
            for fault in ("missing", "null", "position", "quaternion", "frames"):
                capture = capture_with_changes()
                channels = capture["samples"][1]["bodies"]
                if fault == "missing":
                    del channels[key]
                elif fault == "null":
                    channels[key] = None
                elif fault == "position":
                    channels[key]["xyz_mm"] = [0, float("nan"), 0]
                elif fault == "quaternion":
                    channels[key]["quaternion_xyzw"] = [0, 0, 0, 0]
                else:
                    channels[key]["valid_frames"] = 0
                with self.subTest(channel=key, fault=fault), self.assertRaisesRegex(ValueError, f"sample 2 {key}"):
                    self.analyze(capture)

    def test_bad_capture_indices_or_count_are_errors(self):
        original = capture_with_changes()
        for bad in ([], {}, {"samples": []}, {"samples": original["samples"][:1]},
                    {"samples": original["samples"] + [original["samples"][0]]}):
            with self.subTest(capture=bad), self.assertRaises(ValueError):
                self.analyze(bad)
        capture = copy.deepcopy(original)
        capture["samples"][0]["index"] = 1.5
        with self.assertRaisesRegex(ValueError, "index"):
            self.analyze(capture)

    def test_invalid_thresholds_are_errors(self):
        for key in ("max_translation_mm", "max_rotation_deg", "exclude_translation_mm"):
            for value in (0, -1, float("inf"), float("nan"), True, None):
                with self.subTest(key=key, value=value), self.assertRaisesRegex(ValueError, key):
                    self.analyze(capture_with_changes(), **{key: value})


if __name__ == "__main__":
    unittest.main()
