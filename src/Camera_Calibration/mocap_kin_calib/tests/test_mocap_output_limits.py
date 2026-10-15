"""Offline regression checks for the final Ei publication limits."""
from contextlib import redirect_stdout
import copy
import importlib.util
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
from scipy.spatial.transform import Rotation
import yaml


HERE = Path(__file__).resolve().parents[1]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
SOURCE = HERE.parents[1] / "kuavo_assets/models/biped_s63/urdf/biped_s63.urdf"
OUTPUT_NAMES = ("frame_transforms.json", "kinematic_offsets.json", "urdf_final.urdf")


def load_optimizer(stem):
    spec = importlib.util.spec_from_file_location("output_limits_" + stem, HERE / (stem + ".py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


MODULES = [load_optimizer(stem) for stem in ("mocap_optimize_backup", "mocap_optimize_1010")]


class OutputLimitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = yaml.safe_load((HERE / "config/calib_s63.yaml").read_text())
        cls.models = {}
        for module in MODULES:
            kin = copy.deepcopy(cls.cfg["kinematics"])
            chains = {arm: module.load_chain(SOURCE, arm, kin["root_link"], kin[arm + "_tip_link"])
                      for arm in module.ARMS}
            cls.models[module] = module.FixedPositionModel(kin, chains)

    @staticmethod
    def joint_block(model, p, arm="left", index=3):
        return p[model.ei_slice[arm]].reshape(7, 6)[index]

    def test_vector_norm_limits_reject_both_signs_and_identify_joint(self):
        for module, model in self.models.items():
            for arm, index in (("left", 0), ("right", 6)):
                for vector, part, limit in (([12., 10., 0.], slice(0, 3), "15"),
                                            ([8., 7., 0.], slice(3, 6), "10")):
                    for sign in (-1, 1):
                        with self.subTest(module=module.__name__, arm=arm, part=part, sign=sign):
                            p = np.zeros(model.parameter_count)
                            self.joint_block(model, p, arm, index)[part] = np.array(vector) * sign
                            original = p.copy()
                            with self.assertRaises(ValueError) as caught:
                                module._check_ei_output_limits(model, p)
                            self.assertIn(model.chains[arm].joint_names[index], str(caught.exception))
                            self.assertIn(limit, str(caught.exception))
                            np.testing.assert_array_equal(p, original)

    def test_exact_norm_boundaries_pass_without_modifying_parameters(self):
        for module, model in self.models.items():
            with self.subTest(module=module.__name__):
                p = np.zeros(model.parameter_count)
                self.joint_block(model, p)[:] = [9., 12., 0., 6., 8., 0.]
                self.joint_block(model, p, "right", 6)[:] = [-9., -12., 0., -6., -8., 0.]
                original = p.copy()
                module._check_ei_output_limits(model, p)
                np.testing.assert_array_equal(p, original)

    def test_frame_rotations_are_not_subject_to_joint_limits(self):
        for module, model in self.models.items():
            with self.subTest(module=module.__name__):
                p = np.zeros(model.parameter_count)
                p[:9] = [120., -180., 240.] * 3
                module._check_ei_output_limits(model, p)

    def test_raw_rotvec_is_checked_without_wrapping_to_small_physical_angle(self):
        for module, model in self.models.items():
            for angle in (-359., 359.):
                with self.subTest(module=module.__name__, angle=angle):
                    p = np.zeros(model.parameter_count)
                    self.joint_block(model, p)[5] = angle
                    with self.assertRaises(ValueError) as caught:
                        module._check_ei_output_limits(model, p)
                    self.assertIn("zarm_l4_joint", str(caught.exception))
                    self.assertIn("359", str(caught.exception))

    def test_smallest_float_above_each_limit_fails_and_is_reported(self):
        for module, model in self.models.items():
            for component, limit in ((0, 15.), (3, 10.)):
                with self.subTest(module=module.__name__, component=component):
                    p = np.zeros(model.parameter_count)
                    value = np.nextafter(limit, np.inf)
                    self.joint_block(model, p)[component] = value
                    with self.assertRaises(ValueError) as caught:
                        module._check_ei_output_limits(model, p)
                    self.assertIn("zarm_l4_joint", str(caught.exception))
                    self.assertIn(format(value, ".17g"), str(caught.exception))

    def test_nonfinite_joint_values_fail(self):
        for module, model in self.models.items():
            for component in (0, 3):
                for value in (np.nan, np.inf, -np.inf):
                    with self.subTest(module=module.__name__, component=component, value=value):
                        p = np.zeros(model.parameter_count)
                        self.joint_block(model, p, "right", 6)[component] = value
                        with self.assertRaises(ValueError):
                            module._check_ei_output_limits(model, p)

    def make_inputs(self, root, module):
        model = self.models[module]
        cfg = copy.deepcopy(self.cfg)
        cfg["kinematics"]["output_dir"] = str(root / "output")
        config = root / "config.yaml"
        config.write_text(yaml.safe_dump(cfg))
        zero = np.zeros(model.parameter_count)
        observations = {}
        for arm, body in (("left", "l_hand"), ("right", "r_hand")):
            transform = model.predict(zero, arm, np.zeros((1, 7)))[0]
            observations[body + "_in_torso"] = {
                "xyz_mm": transform[:3, 3].tolist(),
                "quaternion_xyzw": Rotation.from_matrix(transform[:3, :3]).as_quat().tolist(),
            }
        capture = root / "capture.json"
        capture.write_text(json.dumps({
            "meta": {"robot_version": "63", "robot_layout": cfg["robot"]["robot_layout"],
                     "fk_root": "waist_yaw_link", "joint_q_indices": cfg["robot"]["joint_indices"]},
            "bodies": [{"name": name, "link_offset_mm": [0., 0., 0.]}
                       for name in ("l_hand", "r_hand", "torso")],
            "samples": [{"index": index, "joint_q_full": [0.] * 20, "bodies": observations}
                        for index in range(1, 31)],
        }))
        return config, capture, root / "output"

    @staticmethod
    def modes(module):
        return (False, True) if module.__name__.endswith("1010") else (True,)

    def run_fit_main(self, module, config, capture, p_final, output_urdf):
        args = [module.__file__, str(capture), "--config", str(config), "--force-output"]
        if output_urdf and module.__name__.endswith("1010"):
            args.append("--output-urdf")
        result = SimpleNamespace(success=True, message="mocked fit", nfev=1)
        fits = [(np.zeros_like(p_final), result), (p_final, result)]
        with patch.dict(os.environ, ROBOT_VERSION="63"), patch.object(sys, "argv", args), \
                patch.object(module, "fit", side_effect=fits) as fit, \
                patch("builtins.input", return_value="no") as prompt, redirect_stdout(io.StringIO()):
            try:
                result = module.main()
                self.assertEqual(prompt.call_count, int(output_urdf and module.__name__.endswith("1010")))
                return result
            except SystemExit:
                prompt.assert_not_called()
                raise
            finally:
                self.assertEqual([call.kwargs["subset"] for call in fit.call_args_list], ["fit", "all"])

    def test_rejected_final_fit_preserves_outputs_even_with_force(self):
        for module, model in self.models.items():
            for output_urdf in self.modes(module):
                for existing in (False, True):
                    for block in ([12., 10., 0., 0., 0., 0.], [0., 0., 0., 8., 7., 0.]):
                        with self.subTest(module=module.__name__, urdf=output_urdf,
                                          existing=existing, block=block), tempfile.TemporaryDirectory() as temp:
                            config, capture, output = self.make_inputs(Path(temp), module)
                            original = {name: ("existing " + name).encode() for name in OUTPUT_NAMES}
                            if existing:
                                output.mkdir()
                                for name, value in original.items():
                                    (output / name).write_bytes(value)
                            p = np.zeros(model.parameter_count)
                            self.joint_block(model, p)[:] = block
                            with self.assertRaises(SystemExit) as caught:
                                self.run_fit_main(module, config, capture, p, output_urdf)
                            self.assertIn("zarm_l4_joint", str(caught.exception))
                            if existing:
                                self.assertEqual({file.name: file.read_bytes() for file in output.iterdir()}, original)
                            else:
                                self.assertFalse(output.exists())

    def test_exact_boundary_final_fit_publishes_real_exports(self):
        for module, model in self.models.items():
            for output_urdf in self.modes(module):
                with self.subTest(module=module.__name__, urdf=output_urdf), tempfile.TemporaryDirectory() as temp:
                    config, capture, output = self.make_inputs(Path(temp), module)
                    p = np.zeros(model.parameter_count)
                    self.joint_block(model, p)[:] = [9., 12., 0., 6., 8., 0.]
                    self.run_fit_main(module, config, capture, p, output_urdf)
                    expected = set(OUTPUT_NAMES if output_urdf else OUTPUT_NAMES[:2])
                    self.assertEqual({file.name for file in output.iterdir()}, expected)
                    offsets = json.loads((output / "kinematic_offsets.json").read_text())
                    self.assertEqual(len(offsets["joints"]), 14)
                    ei = offsets["joints"]["zarm_l4_joint"]["Ei"]
                    self.assertEqual(ei["translation_mm"], [9., 12., 0.])
                    self.assertEqual(ei["rotvec_deg"], [6., 8., 0.])
                    if output_urdf:
                        for arm in module.ARMS:
                            checked = module.load_chain(output / "urdf_final.urdf", arm,
                                                        model.kin["root_link"], model.kin[arm + "_tip_link"])
                            expected_fk = model.chains[arm].fk(np.zeros((1, 7)), model.unpack(p, arm)[2])
                            np.testing.assert_allclose(checked.fk(np.zeros((1, 7))), expected_fk, atol=1e-7)

    def test_validation_only_does_not_apply_publication_gate(self):
        for module, model in self.models.items():
            with self.subTest(module=module.__name__), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                config, capture, output = self.make_inputs(root, module)
                frames = root / "frames.json"
                module._write_frame_metadata_file(frames, module._frame_metadata(model, np.zeros(model.parameter_count)))
                args = [module.__file__, str(capture), "--config", str(config), "--validate-urdf", str(SOURCE),
                        "--frame-transforms", str(frames)]
                with patch.dict(os.environ, ROBOT_VERSION="63"), patch.object(sys, "argv", args), \
                        patch.object(module, "_check_ei_output_limits", side_effect=AssertionError("unexpected output gate")), \
                        patch.object(module, "fit", side_effect=AssertionError("unexpected fit")), \
                        patch("builtins.input", side_effect=AssertionError("unexpected install prompt")), \
                        redirect_stdout(io.StringIO()):
                    self.assertEqual(module.main(), 0)
                self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
