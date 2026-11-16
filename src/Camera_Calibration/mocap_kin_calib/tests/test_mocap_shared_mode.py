"""Shared-CTB training and candidate-only export checks for the guided workflow."""
from contextlib import redirect_stderr, redirect_stdout
import io
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import yaml

import test_mocap_output_limits as limits


class SharedModeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        limits.OutputLimitTests.setUpClass()
        cls.inputs = limits.OutputLimitTests()
        cls.opt = next(module for module in limits.MODULES if module.__name__.endswith("1010"))

    def make_inputs(self, root, mode="shared"):
        config, capture, output = self.inputs.make_inputs(root, self.opt)
        cfg = yaml.safe_load(config.read_text())
        if mode is None:
            cfg["kinematics"]["ctb"].pop("mode", None)
        else:
            cfg["kinematics"]["ctb"]["mode"] = mode
        config.write_text(yaml.safe_dump(cfg))
        return config, capture, output

    def test_shared_heldout_uses_unchanged_fit25_and_exports_schema1_without_ctb_bound(self):
        with tempfile.TemporaryDirectory() as temp:
            config, capture, output = self.make_inputs(Path(temp))
            p25 = np.zeros(93)
            p25[:9] = [0.1, -0.1, 0.1, 0.1, 0.0, -0.1, 0.1, 0.1, 0.0]
            p30 = p25.copy()
            # The original shared model has no new 3-degree CTB limit.
            p30[:3] = [4.5, 0, 0]
            before = p25.copy()

            def fit(model, data, subset, initial=None):
                self.assertIs(type(model), self.opt.FixedPositionModel)
                self.assertEqual(model.parameter_count, 93)
                if subset == "fit":
                    self.assertIsNone(initial)
                    return p25, SimpleNamespace(success=True, message="fit25", nfev=1)
                self.assertEqual(subset, "all")
                np.testing.assert_array_equal(initial, before)
                return p30, SimpleNamespace(success=True, message="fit30", nfev=1)

            args = [self.opt.__file__, str(capture), "--config", str(config),
                    "--output-urdf", "--no-install"]
            stdout = io.StringIO()
            with patch.dict(os.environ, ROBOT_VERSION="63"), patch.object(sys, "argv", args), \
                    patch.object(self.opt, "fit", side_effect=fit) as fitter, \
                    patch.object(self.opt, "fit_frame_angles", side_effect=AssertionError("heldout refit")), \
                    patch.object(self.opt, "error_rows", wraps=self.opt.error_rows) as metrics, \
                    patch.object(self.opt, "_confirm_apply_urdf", side_effect=AssertionError("install")), \
                    patch("builtins.input", side_effect=AssertionError("prompt")), redirect_stdout(stdout):
                self.opt.main()
            self.assertEqual(fitter.call_count, 2)
            heldout = next(call for call in metrics.call_args_list if call.kwargs["subset"] == "validation")
            np.testing.assert_array_equal(heldout.args[1], before)
            np.testing.assert_array_equal(p25, before)
            self.assertIn("FIXED VALIDATION(5)", stdout.getvalue())
            self.assertNotIn("CONDITIONAL VALIDATION", stdout.getvalue())
            metadata = json.loads((output / "frame_transforms.json").read_text())
            self.assertEqual(metadata["schema_version"], 1)
            np.testing.assert_allclose(metadata["ctb"]["rotvec_deg"], [4.5, 0, 0])
            self.assertEqual(self.opt._read_frame_metadata(output / "urdf_final.urdf")["schema_version"], 1)

    def test_default_and_explicit_per_sample_keep_conditional_validation_and_no_install(self):
        for mode in (None, "per_sample"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as temp:
                config, capture, output = self.make_inputs(Path(temp), mode)

                def fit(model, data, subset, initial=None):
                    self.assertIsInstance(model, self.opt.PerPoseCTBModel)
                    self.assertEqual(model.parameter_count, 180)
                    return np.zeros(model.parameter_count), SimpleNamespace(success=True, message="fit", nfev=1)

                args = [self.opt.__file__, str(capture), "--config", str(config),
                        "--output-urdf", "--no-install"]
                with patch.dict(os.environ, ROBOT_VERSION="63"), patch.object(sys, "argv", args), \
                        patch.object(self.opt, "fit", side_effect=fit), \
                        patch.object(self.opt, "fit_frame_angles", wraps=self.opt.fit_frame_angles) as heldout, \
                        patch.object(self.opt, "_confirm_apply_urdf", side_effect=AssertionError("install")), \
                        patch("builtins.input", side_effect=AssertionError("prompt")), redirect_stdout(io.StringIO()):
                    self.opt.main()
                heldout.assert_called_once()
                self.assertFalse(heldout.call_args.kwargs["fit_etk"])
                metadata = json.loads((output / "frame_transforms.json").read_text())
                self.assertEqual(metadata["schema_version"], 2)
                self.assertEqual(len(metadata["ctb"]["samples"]), 30)

    def test_no_install_requires_output_urdf(self):
        args = [self.opt.__file__, "unused_capture.json", "--no-install"]
        with patch.object(sys, "argv", args), redirect_stderr(io.StringIO()), \
                self.assertRaises(SystemExit) as caught:
            self.opt.main()
        self.assertEqual(caught.exception.code, 2)

    def test_validation_ignores_training_ctb_mode_and_keeps_qingtong_switch(self):
        for mode, qingtong in (("shared", True), ("per_sample", False)):
            with self.subTest(mode=mode, qingtong=qingtong), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                config, capture, output = self.make_inputs(root, mode)
                cfg = yaml.safe_load(config.read_text())
                cfg["kinematics"]["validation"]["qingtong"] = qingtong
                config.write_text(yaml.safe_dump(cfg))
                model = self.inputs.models[self.opt]
                frames = root / "frames.json"
                self.opt._write_frame_metadata_file(frames, self.opt._frame_metadata(model, np.zeros(93)))
                args = [self.opt.__file__, str(capture), "--config", str(config),
                        "--validate-urdf", str(limits.SOURCE), "--frame-transforms", str(frames),
                        "--no-freeze-frame-angles"]

                def frame_fit(candidate, data, subset, initial=None, fit_etk=True):
                    expected = self.opt.PerPoseCTBModel if qingtong else self.opt.FixedPositionModel
                    self.assertIs(type(candidate), expected)
                    self.assertEqual(subset, "all")
                    self.assertTrue(fit_etk)
                    return np.zeros(candidate.parameter_count), SimpleNamespace(success=True, message="frames", nfev=1)

                with patch.dict(os.environ, ROBOT_VERSION="63"), patch.object(sys, "argv", args), \
                        patch.object(self.opt, "fit_frame_angles", side_effect=frame_fit) as fitter, \
                        patch.object(self.opt, "fit", side_effect=AssertionError("Ei fit")), \
                        patch("builtins.input", side_effect=AssertionError("prompt")), redirect_stdout(io.StringIO()):
                    self.assertEqual(self.opt.main(), 0)
                fitter.assert_called_once()
                self.assertFalse(output.exists())

    def test_unknown_training_mode_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            config, capture, _ = self.make_inputs(Path(temp), "typo")
            args = [self.opt.__file__, str(capture), "--config", str(config)]
            with patch.dict(os.environ, ROBOT_VERSION="63"), patch.object(sys, "argv", args), \
                    self.assertRaisesRegex(ValueError, "ctb.mode"):
                self.opt.main()


if __name__ == "__main__":
    unittest.main()
