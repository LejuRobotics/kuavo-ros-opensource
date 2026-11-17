"""Fixed-original-mean quality gates and selected-pose export regressions."""
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from test_mocap_stability import pose, transform
from test_mocap_ei_weights import model_for_config, synthetic_capture, opt
import mocap_optimize_fpf as workflow
from mocap_stability import analyze_capture, check_capture_quality


def reference_capture(left, right=None, indices=None):
    right = left if right is None else right
    indices = list(range(1, len(left) + 1)) if indices is None else indices
    samples = []
    for row, index in enumerate(indices):
        bodies = {}
        for hand, positions in (("l_hand", left), ("r_hand", right)):
            ts = transform([100 + positions[row], 200, 300], [0, 0, 0])
            th = transform([400 + row, -200, 100], [row, -10, 20])
            bodies[hand + "_in_torso"] = pose(th)
            bodies[hand + "_in_l_shoulder"] = pose(np.linalg.inv(ts) @ th)
        samples.append({"index": index, "bodies": bodies})
    return {"samples": samples}


def disturb_reference(capture, indices, shift, corrupt_hand=False):
    for sample in capture["samples"]:
        if sample["index"] not in indices:
            continue
        for hand in ("l_hand", "r_hand"):
            bodies = sample["bodies"]
            th = opt._capture_pose_to_matrix(bodies[hand + "_in_torso"])
            sh = opt._capture_pose_to_matrix(bodies[hand + "_in_l_shoulder"])
            ts = th @ np.linalg.inv(sh)
            ts[0, 3] += shift
            if corrupt_hand:
                # This large residual must never enter FIT, held-out validation,
                # or FINAL after the quality rule removes this original pose.
                th[:3, 3] += [500, -300, 200]
                bodies[hand + "_in_torso"] = pose(th)
            bodies[hand + "_in_l_shoulder"] = pose(np.linalg.inv(ts) @ th)


class QualityTests(unittest.TestCase):
    def analyze(self, capture, **options):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "capture.json"
            path.write_text(json.dumps(capture))
            report = analyze_capture(path, **options)
            json.dumps(report, allow_nan=False)
            return report

    def test_strict_thresholds_and_hard_rejection_precedes_possible_removal(self):
        for delta, status in ((3.0, "pass"), (3.000001, "filtered"),
                              (8.0, "filtered"), (8.000001, "error")):
            with self.subTest(delta=delta):
                # The last pose's distance from the 30-point mean is delta.
                report = self.analyze(reference_capture([0.] * 29 + [delta * 30 / 29]))
                quality = report["quality"]
                self.assertEqual(quality["status"], status)
                self.assertEqual(quality["excluded_indices"], [30] if status == "filtered" else [])
                if status == "error":
                    self.assertEqual(quality["reason"], "hard_translation_limit")
                    self.assertIn(30, quality["problem_pose_indices"])

    def test_exclusions_combine_both_arms_and_count_shared_pose_once(self):
        left, right = np.zeros(30), np.zeros(30)
        left[[1, 5]] = 4.0
        right[[5, 17]] = 4.0
        report = self.analyze(reference_capture(left, right))
        self.assertEqual(report["quality"]["excluded_indices"], [2, 6, 18])
        self.assertEqual(len(report["quality"]["retained_indices"]), 27)
        for value in report["quality"]["retained_max_translation_mm"].values():
            self.assertAlmostEqual(value, 8 / 30, places=9)

    def test_four_required_exclusions_reject_without_partially_filtering(self):
        report = self.analyze(reference_capture([4.] * 4 + [0.] * 26))
        self.assertEqual(report["quality"]["reason"], "exclusion_limit")
        self.assertEqual(report["quality"]["excluded_indices"], [])
        self.assertEqual(len(report["quality"]["retained_indices"]), 30)
        self.assertEqual(report["quality"]["required_excluded_indices"], [1, 2, 3, 4])

    def test_selection_keeps_original_ids_under_capture_reordering(self):
        capture = reference_capture([0.] * 29 + [4.], indices=list(range(10, 310, 10)))
        for samples in (capture["samples"], capture["samples"][::-1]):
            quality = self.analyze({"samples": samples})["quality"]
            self.assertEqual(quality["excluded_indices"], [300])

    def test_pairwise_spread_does_not_trigger_mean_distance_gates(self):
        report = self.analyze(reference_capture([-3., 3.]))
        self.assertEqual(report["quality"]["status"], "pass")
        self.assertAlmostEqual(report["arms"]["left"]["pairwise"]["max_translation_mm"], 6.)
        report = self.analyze(reference_capture([-8., 8.] + [0.] * 28))
        self.assertEqual(report["quality"]["excluded_indices"], [1, 2])
        self.assertAlmostEqual(report["arms"]["left"]["pairwise"]["max_translation_mm"], 16.)

    def test_mean_is_not_recomputed_after_removal(self):
        # Recentring after removing 29/30 would wrongly remove pose 1 too.
        positions = np.array([3.4] + [0.] * 27 + [6., 6.])
        report = self.analyze(reference_capture(positions))
        self.assertEqual(report["schema_version"], 3)
        self.assertEqual(report["quality_metric"], "distance_from_original_mean")
        self.assertEqual(report["quality"]["excluded_indices"], [29, 30])
        self.assertAlmostEqual(report["arms"]["left"]["reference_mean_xyz_mm"][0], 100 + positions.mean())
        self.assertAlmostEqual(report["quality"]["retained_max_translation_mm"]["left"], 3.4 - positions.mean())
        self.assertGreater(3.4 - positions[:28].mean(), 3.)

    def test_deviation_is_a_3d_norm_not_independent_axis_limits(self):
        capture = reference_capture([0.] * 29 + [3.0])
        sample = capture["samples"][-1]
        for hand in ("l_hand", "r_hand"):
            bodies = sample["bodies"]
            th = opt._capture_pose_to_matrix(bodies[hand + "_in_torso"])
            sh = opt._capture_pose_to_matrix(bodies[hand + "_in_l_shoulder"])
            ts = th @ np.linalg.inv(sh)
            ts[1, 3] += 3.0
            bodies[hand + "_in_l_shoulder"] = pose(np.linalg.inv(ts) @ th)
        report = self.analyze(capture)
        self.assertEqual(report["quality"]["excluded_indices"], [30])
        self.assertAlmostEqual(report["arms"]["left"]["max_translation_mm"], np.sqrt(2) * 2.9)

    def test_all_excluded_is_rejected_without_empty_reductions(self):
        self.assertEqual(self.analyze(reference_capture([-4., 4.]))["quality"]["reason"],
                         "insufficient_retained_poses")

    def test_invalid_exclusion_limits_are_rejected(self):
        for value in (-1, 4, 3.5, True, None):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "max_excluded_poses"):
                self.analyze(reference_capture([0., 1., 2.]), max_excluded_poses=value)
        with self.assertRaisesRegex(ValueError, "exclude_translation_mm"):
            self.analyze(reference_capture([0., 1., 2.]), exclude_translation_mm=9.)

    def test_quality_failure_persists_report_and_identifies_pose_deviations(self):
        with tempfile.TemporaryDirectory() as temp:
            path, report_path = Path(temp) / "capture.json", Path(temp) / "stability.json"
            path.write_text(json.dumps(reference_capture([0., 0., 15.], indices=[5, 2, 9])))
            output = io.StringIO()
            with redirect_stdout(output), self.assertRaisesRegex(ValueError, "数据质量错误"):
                check_capture_quality(path, {}, report_path)
            report = json.loads(report_path.read_text())
            self.assertEqual(report["quality"]["status"], "error")
            self.assertIn("pose 9:", output.getvalue())
            self.assertEqual(report["quality"]["hard_limit_pose_indices"], [9])
            self.assertIn("10.000000 mm", output.getvalue())
            self.assertIn("距原始均值", output.getvalue())

    def test_direct_optimizer_quality_errors_never_solve_or_replace_outputs(self):
        _, cfg, model = model_for_config("63")
        for bad_ids, delta in (([2], 9.), ([1, 2, 3, 4], 4.)):
            with self.subTest(bad_ids=bad_ids), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                capture = synthetic_capture(cfg, model, 30, 7)
                disturb_reference(capture, bad_ids, delta)
                capture_path, config_path = root / "fit.json", root / "config.yaml"
                capture_path.write_text(json.dumps(capture))
                cfg["kinematics"]["output_dir"] = str(root / "output")
                config_path.write_text(yaml.safe_dump(cfg))
                output = root / "output"
                output.mkdir()
                existing = output / "urdf_final.urdf"
                existing.write_text("existing result")
                args = [opt.__file__, str(capture_path), "--config", str(config_path),
                        "--output-urdf", "--force-output", "--no-install"]
                with patch.dict(os.environ, ROBOT_VERSION="63"), patch.object(sys, "argv", args), \
                        patch.object(opt, "fit") as solver, redirect_stdout(io.StringIO()), \
                        self.assertRaisesRegex(ValueError, "数据质量错误"):
                    opt.main()
                solver.assert_not_called()
                self.assertEqual(existing.read_text(), "existing result")
                self.assertEqual(list(output.iterdir()), [existing])

    def test_filtered_one_click_real_solve_validation_and_exports_for_both_families(self):
        for version in ("56", "63"):
            with self.subTest(version=version), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                config_path, cfg, model = model_for_config(version)
                fit_capture = synthetic_capture(cfg, model, 30, 7)
                removed = [2, 6, 18]  # One FIT and two held-out validation poses.
                disturb_reference(fit_capture, removed, 4.0, corrupt_hand=True)
                fit_path, validation_path = root / "fit.json", root / "validation.json"
                fit_path.write_text(json.dumps(fit_capture))
                validation_path.write_text(json.dumps(synthetic_capture(cfg, model, 10, 8)))
                original = fit_path.read_bytes()
                output = root / "output"
                args = ["--config", str(config_path), "--fit-capture", str(fit_path),
                        "--validation-capture", str(validation_path), "--output-dir", str(output)]
                with patch.dict(os.environ, ROBOT_VERSION=version, OPENBLAS_NUM_THREADS="1"), \
                        patch("builtins.input", side_effect=["2", "r", "r", "y", "y"]), \
                        patch.object(workflow.optimizer, "_confirm_apply_urdf") as install, \
                        redirect_stdout(io.StringIO()):
                    self.assertEqual(workflow.main(args), 0)
                install.assert_called_once()
                self.assertEqual(fit_path.read_bytes(), original)
                run_dir = next(output.glob("fpf_*"))
                report = json.loads((run_dir / "stability.json").read_text())
                self.assertEqual(report["quality"]["excluded_indices"], removed)
                fit_log = (run_dir / "fit.log").read_text()
                for label in ("24-point fit: success=True", "FIXED VALIDATION(3)",
                              "27-point fit: success=True", "ALL(27)"):
                    self.assertIn(label, fit_log)
                self.assertIn("FIXED VALIDATION(10)", (run_dir / "validation.log").read_text())
                frames = opt._read_frame_metadata(output / "urdf_final.urdf")
                offsets = json.loads((output / "kinematic_offsets.json").read_text())
                selection = offsets["data_selection"]
                self.assertEqual(selection, frames["data_selection"])
                self.assertEqual(selection["quality_metric"], "distance_from_original_mean")
                self.assertEqual(selection["quality_report_schema_version"], 3)
                self.assertEqual(selection["reference_mean_xyz_mm"],
                                 {arm: value["reference_mean_xyz_mm"] for arm, value in report["arms"].items()})
                self.assertEqual(selection["excluded_indices"], removed)
                self.assertEqual(len(selection["fit_indices"]), 24)
                self.assertEqual(selection["validation_indices"], [12, 24, 30])
                self.assertFalse(set(removed) & set(selection["fit_indices"] + selection["validation_indices"]))


if __name__ == "__main__":
    unittest.main()
