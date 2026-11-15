"""Ei objective checks and real offline one-click solves across robot variants."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

HERE = Path(__file__).resolve().parents[1]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import mocap_optimize_1010 as opt
from robot_variants import ASSETS, default_config_name
from variant_cases import BIPED_VERSIONS, WHEEL_VERSIONS


def model_for_config(version):
    path = HERE / "config" / default_config_name(version)
    cfg = yaml.safe_load(path.read_text())
    with patch.dict(os.environ, ROBOT_VERSION=version):
        cfg = opt._config_for_capture(cfg, {})
    source = opt.write_urdf.resolve_fit_source(Path(cfg["kinematics"]["source_urdf"]))
    kin = cfg["kinematics"]
    chains = {arm: opt.load_chain(source, arm, kin["root_link"], kin[arm + "_tip_link"])
              for arm in opt.ARMS}
    return path, cfg, opt.FixedPositionModel(kin, chains)


def synthetic_capture(cfg, model, count, seed):
    """Independent FK observations with nonzero local Ei and fresh joint poses."""
    rng = np.random.default_rng(seed)
    q = {arm: rng.uniform(-0.7, 0.7, (count, 7)) for arm in opt.ARMS}
    markers = {}
    ctb = opt._rigid(model.ctb_t, model.ctb_r0)
    for arm in opt.ARMS:
        corrections = np.repeat(np.eye(4)[None], 7, axis=0)
        corrections[:, :3, 3] = np.array([0.03, -0.02, 0.01])
        corrections[:, :3, :3] = Rotation.from_rotvec(
            np.deg2rad(np.linspace(-0.3, 0.3, 7))[:, None] * model.chains[arm].axes).as_matrix()
        etk = opt._rigid(model.etk_t[arm], model.etk_r0[arm])
        markers[arm] = np.linalg.inv(ctb) @ model.chains[arm].fk(q[arm], corrections) @ etk
    shoulder = opt._rigid([100., 200., -50.], [4., -2., 3.])

    def pose(matrix):
        return {"xyz_mm": matrix[:3, 3].tolist(),
                "quaternion_xyzw": Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
                "valid_frames": 90}

    samples = []
    for row in range(count):
        full_q = np.zeros(cfg["robot"]["joint_q_length"])
        bodies = {}
        for arm, hand in (("left", "l_hand"), ("right", "r_hand")):
            lo, hi = cfg["robot"]["joint_indices"][arm + "_arm"]
            full_q[lo:hi] = q[arm][row]
            bodies[hand + "_in_torso"] = pose(markers[arm][row])
            bodies[hand + "_in_l_shoulder"] = pose(np.linalg.inv(shoulder) @ markers[arm][row])
        samples.append({"index": row + 1, "joint_q_full": full_q.tolist(), "bodies": bodies})
    return {
        "meta": {"robot_version": cfg["robot"]["robot_version"],
                 "robot_layout": cfg["robot"]["robot_layout"], "fk_root": model.kin["root_link"],
                 "joint_q_indices": cfg["robot"]["joint_indices"]},
        "bodies": [{"name": name, "link_offset_mm": [0., 0., 0.]}
                   for name in ("l_hand", "r_hand", "torso", "l_shoulder")],
        "samples": samples,
    }


class EiWeightTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.models = {version: model_for_config(version)[2] for version in ("56", "63")}

    def test_objective_matches_notebook_diagonal_weights_for_all_joint_axes(self):
        for version, model in self.models.items():
            with self.subTest(version=version):
                p = np.random.default_rng(39).normal(size=model.parameter_count)
                expected = 0.0
                for arm in opt.ARMS:
                    blocks = p[model.ei_slice[arm]].reshape(7, 6)
                    diagonal = np.full((7, 6), 1.0)
                    diagonal[:, :3] = 100.0
                    axis_components = np.argmax(np.abs(model.chains[arm].axes), axis=1)
                    np.testing.assert_allclose(np.abs(model.chains[arm].axes), np.eye(3)[axis_components])
                    diagonal[np.arange(7), 3 + axis_components] = 0.0001
                    expected += np.sum(diagonal * blocks ** 2)
                residual = model.ei_weight_residual(p)
                self.assertAlmostEqual(float(residual @ residual), expected, places=9)
                # Reference transforms must not be regularized with the Ei.
                changed = p.copy()
                changed[:9] += 100
                np.testing.assert_array_equal(model.ei_weight_residual(changed), residual)

    def test_signed_and_oblique_axes_separate_theta_from_transverse_rotation(self):
        original = self.models["63"]
        for axis in ([0., -1., 0.], [1., 2., -3.]):
            chains = copy.deepcopy(original.chains)
            a = np.asarray(axis) / np.linalg.norm(axis)
            b = np.cross(a, [1., 0., 0.])
            b /= np.linalg.norm(b)
            chains["left"].axes[0] = a
            model = opt.FixedPositionModel(copy.deepcopy(original.kin), chains)
            for name, block, expected in (
                    ("xyz", [2., 0., 0., 0., 0., 0.], 400.0),
                    ("theta", np.r_[np.zeros(3), 3 * a], 9e-4),
                    ("other", np.r_[np.zeros(3), 5 * b], 25.0)):
                with self.subTest(axis=axis, component=name):
                    p = np.zeros(model.parameter_count)
                    p[model.ei_slice["left"]][:6] = block
                    residual = model.ei_weight_residual(p)
                    self.assertAlmostEqual(float(residual @ residual), expected, places=10)

    def test_zero_and_missing_weights_preserve_legacy_residual_and_scaling(self):
        original = self.models["63"]
        missing = copy.deepcopy(original.kin)
        for name in opt.EI_WEIGHT_NAMES:
            missing["optimizer"].pop(name)
        zero = copy.deepcopy(missing)
        zero["optimizer"].update({name: 0.0 for name in opt.EI_WEIGHT_NAMES})
        models = [opt.FixedPositionModel(kin, original.chains) for kin in (missing, zero)]
        p = np.random.default_rng(6).normal(size=93) * 0.1
        data = {"fit_count": 2, "validation_count": 0, "fit_indices": [1, 2], "validation_indices": []}
        for arm in opt.ARMS:
            data[arm + "_q"] = np.zeros((2, 7))
            data[arm + "_marker_T"] = original.predict(np.zeros(93), arm, data[arm + "_q"])
        for model in models:
            self.assertEqual(model.ei_weight_residual(p).size, 0)
            self.assertEqual(model.residual(p, data, "fit").size, 24)
            expected_scale = np.r_[np.ones(9), np.tile([2., 2., 2., 1., 1., 1.], 14)]
            np.testing.assert_array_equal(model.x_scale(), expected_scale)
        np.testing.assert_array_equal(models[0].residual(p, data, "fit"), models[1].residual(p, data, "fit"))
        # The prior is appended once, regardless of how many poses are fitted.
        data_squared_norm = np.sum(models[0].residual(p, data, "fit") ** 2)
        weighted_squared_norm = np.sum(original.residual(p, data, "fit") ** 2)
        self.assertAlmostEqual(weighted_squared_norm - data_squared_norm,
                               np.sum(original.ei_weight_residual(p) ** 2), places=8)

    def test_invalid_weights_are_reported_by_name(self):
        original = self.models["63"]
        for name in opt.EI_WEIGHT_NAMES:
            for value in (-1, float("nan"), float("inf"), None, True, "bad"):
                kin = copy.deepcopy(original.kin)
                kin["optimizer"][name] = value
                with self.subTest(name=name, value=value), self.assertRaisesRegex(ValueError, name):
                    opt.FixedPositionModel(kin, original.chains)

    def test_per_sample_model_has_same_shared_ei_penalty(self):
        shared = self.models["63"]
        per_sample = opt.PerPoseCTBModel(copy.deepcopy(shared.kin), shared.chains, [1, 2, 3])
        p = np.random.default_rng(8).normal(size=93)
        pp = np.zeros(per_sample.parameter_count)
        for arm in opt.ARMS:
            pp[per_sample.ei_slice[arm]] = p[shared.ei_slice[arm]]
        np.testing.assert_array_equal(shared.ei_weight_residual(p), per_sample.ei_weight_residual(pp))

    def test_one_click_real_solver_export_and_validation_for_existing_variants(self):
        for version in (*BIPED_VERSIONS, *WHEEL_VERSIONS):
            with self.subTest(version=version), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                config_path, cfg, model = model_for_config(version)
                original_config = config_path.read_bytes()
                target = Path(cfg["kinematics"]["source_urdf"])
                original_models = {p: p.read_bytes() for p in target.parent.rglob("*") if p.is_file()}
                # Check the feedback layout against each variant's actual assets.
                kuavo = json.loads((ASSETS / "config" / f"kuavo_v{version}" / "kuavo.json").read_text())
                self.assertEqual(cfg["robot"]["joint_q_length"], kuavo["NUM_JOINT"])
                self.assertEqual(len(kuavo["MOTORS_TYPE"]), kuavo["NUM_JOINT"])
                self.assertEqual(kuavo["NUM_ARM_JOINT"], 14)
                first = kuavo["NUM_JOINT"] - kuavo["NUM_ARM_JOINT"] - kuavo["NUM_HEAD_JOINT"]
                self.assertEqual(cfg["robot"]["joint_indices"]["left_arm"], [first, first + 7])
                self.assertEqual(cfg["robot"]["joint_indices"]["right_arm"], [first + 7, first + 14])
                fit_path, validation_path = root / "fit.json", root / "validation.json"
                fit_path.write_text(json.dumps(synthetic_capture(cfg, model, 30, 7)))
                validation_path.write_text(json.dumps(synthetic_capture(cfg, model, 10, 8)))
                output = root / "output"
                args = ["--fit-capture", str(fit_path),
                        "--validation-capture", str(validation_path), "--output-dir", str(output)]
                # Exercise the real top-level shell and default config routing,
                # including installation dry-run; answer no to runtime writes.
                result = subprocess.run(
                    ["bash", str(HERE / "mocap_optimize_fpf.sh"), *args],
                    input="2\nr\nr\ny\ny\nno\n", text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60,
                    env=dict(os.environ, ROBOT_VERSION=version, OPENBLAS_NUM_THREADS="1"),
                )
                self.assertEqual(result.returncode, 0, result.stdout)
                self.assertIn("未替换运行模型", result.stdout)
                self.assertEqual({p: p.read_bytes() for p in target.parent.rglob("*") if p.is_file()},
                                 original_models)
                self.assertEqual(config_path.read_bytes(), original_config)
                run_dir = next(output.glob("fpf_*"))
                for stage in ("fit", "validation"):
                    effective = yaml.safe_load((run_dir / (stage + ".yaml")).read_text())
                    self.assertEqual(effective["robot"]["robot_version"], version)
                    self.assertEqual(Path(effective["kinematics"]["source_urdf"]), target)
                    self.assertEqual(Path(effective["kinematics"]["output_dir"]), output)
                    for name, value in model.ei_weights.items():
                        self.assertEqual(effective["kinematics"]["optimizer"][name], value)
                        self.assertIn(f"{name}={value:g}", (run_dir / (stage + ".log")).read_text())
                fit_log = (run_dir / "fit.log").read_text()
                self.assertIn("25-point fit: success=True", fit_log)
                self.assertIn("30-point fit: success=True", fit_log)
                self.assertIn("FIXED VALIDATION(5)", fit_log)
                self.assertIn("PASS", (run_dir / "validation.log").read_text())
                frames = opt._read_frame_metadata(output / "urdf_final.urdf")
                self.assertEqual(frames["schema_version"], 1)
                offsets = json.loads((output / "kinematic_offsets.json").read_text())
                self.assertEqual(len(offsets["joints"]), 14)
                self.assertGreater(sum(np.linalg.norm(j["Ei"]["rotvec_deg"])
                                       for j in offsets["joints"].values()), 0.1)


if __name__ == "__main__":
    unittest.main()
