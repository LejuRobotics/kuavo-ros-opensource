"""Offline checks of bounded, capture-indexed CTB rotations on real S63 chains."""
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
SPEC = importlib.util.spec_from_file_location("per_pose_ctb_optimizer", HERE / "mocap_optimize_1010.py")
OPT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = OPT
SPEC.loader.exec_module(OPT)
SOURCE = HERE.parents[1] / "kuavo_assets/models/biped_s63/urdf/biped_s63.urdf"


class PerPoseCTBTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = yaml.safe_load((HERE / "config/calib_s63.yaml").read_text())
        kin = cls.cfg["kinematics"]
        cls.chains = {
            arm: OPT.load_chain(SOURCE, arm, kin["root_link"], kin[arm + "_tip_link"])
            for arm in OPT.ARMS
        }

    def make_model(self, indices, bound=3.0):
        kin = copy.deepcopy(self.cfg["kinematics"])
        kin["ctb"]["rotation_bound_deg"] = bound
        # A legacy shared-frame starting direction must not offset the bound.
        kin["ctb"]["initial_rotvec_deg"] = [11.0, -7.0, 4.0]
        kin["optimizer"].update(max_nfev=150, ftol=1e-11, xtol=1e-11, gtol=1e-10)
        return OPT.PerPoseCTBModel(kin, self.chains, indices)

    def make_data(self, model, rotations, fit_count=None):
        """Generate raw marker poses independently from the optimizer predictor."""
        indices = list(model.sample_indices)
        n = len(indices)
        if fit_count is None:
            fit_count = n
        data = {
            "fit_indices": indices[:fit_count], "validation_indices": indices[fit_count:],
            "fit_count": fit_count, "validation_count": n - fit_count,
            "capture_path": "synthetic_capture.json", "capture_sha256": "a" * 64,
        }
        ctb = np.repeat(np.eye(4)[None], n, axis=0)
        ctb[:, :3, :3] = Rotation.from_rotvec(np.deg2rad(rotations)).as_matrix()
        ctb[:, :3, 3] = model.kin["ctb"]["translation_mm"]
        rng = np.random.default_rng(20261023)
        for arm in OPT.ARMS:
            q = rng.uniform(-0.7, 0.7, (n, 7))
            etk = np.eye(4)
            etk_cfg = model.kin["etk"][arm]
            etk[:3, 3] = etk_cfg["translation_mm"]
            etk[:3, :3] = Rotation.from_rotvec(np.deg2rad(etk_cfg["initial_rotvec_deg"])).as_matrix()
            data[arm + "_q"] = q
            data[arm + "_marker_T"] = np.linalg.inv(ctb) @ self.chains[arm].fk(q) @ etk
        return data

    def test_recovers_distinct_rotations_from_identity_with_fixed_shared_geometry(self):
        model = self.make_model([9, 2, 17, 4])
        rotations = np.array([[0.0, 0.0, 0.0], [1.0, -1.2, 0.8],
                              [-1.7, 0.4, 1.0], [0.6, 1.8, -0.9]])
        data = self.make_data(model, rotations)
        p, result = OPT.fit_frame_angles(model, data, "all", fit_etk=False)
        self.assertTrue(result.success, result.message)
        np.testing.assert_allclose(p[model.ctb_slice].reshape(-1, 3), rotations, atol=1e-7)
        np.testing.assert_array_equal(p[model.ctb_slice.stop:], 0)
        np.testing.assert_allclose(OPT.error_rows(model, p, data, "all")[0]["position_mm"], 0, atol=1e-7)
        for arm in OPT.ARMS:
            np.testing.assert_allclose(model.predict(p, arm, data[arm + "_q"]),
                                       data[arm + "_marker_T"], atol=1e-7)

    def test_multiaxis_out_of_range_observations_fit_on_total_angle_boundary(self):
        model = self.make_model([1, 2, 3])
        data = self.make_data(model, [[3.0, 3.0, 3.0], [-4.0, 3.0, 1.0], [1.0, -4.0, 4.0]])
        p, result = OPT.fit_frame_angles(model, data, "all", fit_etk=False)
        self.assertTrue(result.success, result.message)
        angles = np.linalg.norm(p[model.ctb_slice].reshape(-1, 3), axis=1)
        self.assertTrue(np.all(angles <= 3.0 + 1e-10), angles)
        np.testing.assert_allclose(angles, 3.0, atol=1e-5)
        self.assertGreater(np.linalg.norm(model.residual(p, data, "all")), 1.0)
        np.testing.assert_array_equal(p[model.ctb_slice.stop:], 0)

    def test_spherical_parameterization_round_trip_interior_and_boundary(self):
        cube = np.vstack((np.random.default_rng(6).uniform(-1, 1, (16, 3)),
                          [[0, 0, 0], [1, 0, 0], [-1, 1, 0], [1, 1, 1], [-1, -1, -1]]))
        rotations = OPT._ctb_cube_to_rotvec(cube, 3.0).reshape(-1, 3)
        self.assertTrue(np.all(np.linalg.norm(rotations, axis=1) <= 3.0 + 1e-12))
        np.testing.assert_allclose(np.linalg.norm(rotations[-4:], axis=1), 3.0, atol=1e-12)
        restored = OPT._ctb_cube_to_rotvec(OPT._ctb_rotvec_to_cube(rotations, 3.0), 3.0)
        np.testing.assert_allclose(restored.reshape(-1, 3), rotations, atol=1e-6)

    def test_limit_rejects_multiaxis_overflow_and_invalid_configuration(self):
        for bound in (0, -3, np.inf, np.nan, 181):
            with self.subTest(bound=bound), self.assertRaises(ValueError):
                self.make_model([1], bound)
        model = self.make_model([1])
        for rotation in ([2, 2, 2], [np.nan, 0, 0], [np.inf, 0, 0]):
            with self.subTest(rotation=rotation):
                p = np.zeros(model.parameter_count)
                p[model.ctb_slice] = rotation
                with self.assertRaises(ValueError):
                    model.check_ctb_angles(p)
        p = np.zeros(model.parameter_count)
        p[model.ctb_slice] = [1, 2, 2]
        model.check_ctb_angles(p)

    def test_predictions_select_capture_indices_and_reject_length_mismatch(self):
        model = self.make_model([11, 5, 20])
        rotations = np.array([[1, 0, 0], [0, -2, 0], [0, 0, 2.5]])
        data = self.make_data(model, rotations)
        p = np.zeros(model.parameter_count)
        p[model.ctb_slice] = rotations.ravel()
        for arm in OPT.ARMS:
            q = data[arm + "_q"][[2, 0]]
            np.testing.assert_allclose(model.predict(p, arm, q, indices=[20, 11]),
                                       data[arm + "_marker_T"][[2, 0]], atol=1e-10)
            with self.assertRaises(ValueError):
                model.predict(p, arm, q, indices=[20])
            with self.assertRaises((KeyError, ValueError)):
                model.predict(p, arm, q, indices=[20, 999])

    def test_fit25_excludes_heldout_angles_and_observations(self):
        fit_indices = self.cfg["kinematics"]["fit_indices"]
        validation_indices = self.cfg["kinematics"]["validation_indices"]
        model = self.make_model(fit_indices + validation_indices)
        data = self.make_data(model, np.zeros((30, 3)), fit_count=25)
        contaminated = copy.deepcopy(data)
        for arm in OPT.ARMS:
            contaminated[arm + "_marker_T"][25:, :3, 3] += 1e6
            contaminated[arm + "_q"][25:] += 0.8
        # Exact nominal observations put the real mechanical fit at its optimum
        # at initialization, so this also checks the full solver without a slow
        # or arbitrary solution to the underdetermined mechanical parameters.
        with patch.object(OPT, "least_squares", wraps=OPT.least_squares) as solver:
            clean_p, clean_result = OPT.fit(model, data, "fit")
            clean_call = solver.call_args
            contaminated_p, contaminated_result = OPT.fit(model, contaminated, "fit")
            contaminated_call = solver.call_args
        self.assertTrue(clean_result.success, clean_result.message)
        self.assertTrue(contaminated_result.success, contaminated_result.message)
        for call in (clean_call, contaminated_call):
            self.assertEqual(len(call.args[1]), 3 * 25 + 6 + 84)
        np.testing.assert_array_equal(clean_p, contaminated_p)
        np.testing.assert_array_equal(clean_result.fun, contaminated_result.fun)
        trial = clean_call.args[1].copy()
        trial[:75] = np.linspace(-0.3, 0.3, 75)
        np.testing.assert_array_equal(clean_call.args[0](trial), contaminated_call.args[0](trial))
        np.testing.assert_array_equal(clean_p[75:90], 0)

    def test_validation_fits_only_five_ctb_rotations_with_etk_and_ei_bitwise_frozen(self):
        model = self.make_model(self.cfg["kinematics"]["fit_indices"] +
                                self.cfg["kinematics"]["validation_indices"])
        initial = np.zeros(model.parameter_count)
        initial[:75] = np.tile([0.2, -0.1, 0.3], 25)
        for arm in OPT.ARMS:
            initial[model.etk_slice[arm]] = [1.0, -0.7, 0.4]
            initial[model.ei_slice[arm]] = np.linspace(-0.1, 0.1, 42)
        before = initial.copy()
        target = initial.copy()
        target[75:90] = np.array([[0.7, -0.5, 0.3], [-0.3, 0.8, 0.4],
                                  [0.4, 0.1, -0.7], [1.2, -0.8, 0.2],
                                  [-0.9, 0.3, 0.6]]).ravel()
        data = self.make_data(model, np.zeros((30, 3)), fit_count=25)
        for arm in OPT.ARMS:
            data[arm + "_marker_T"] = model.predict(target, arm, data[arm + "_q"])
        p, result = OPT.fit_frame_angles(model, data, "validation", initial=initial, fit_etk=False)
        self.assertTrue(result.success, result.message)
        np.testing.assert_allclose(p[75:90], target[75:90], atol=1e-7)
        frozen = np.r_[0:75, 90:model.parameter_count]
        np.testing.assert_array_equal(p[frozen], before[frozen])
        np.testing.assert_array_equal(initial, before)
        model.check_ctb_angles(p)

    def test_validate_urdf_uses_bounded_ctb_only_when_qingtong_is_enabled(self):
        model = self.make_model([8, 2, 19])
        data = self.make_data(model, np.zeros((3, 3)))
        samples = []
        for row, index in enumerate(model.sample_indices):
            q_full = np.zeros(20)
            observations = {}
            for arm, name in (("left", "l_hand"), ("right", "r_hand")):
                lo, hi = self.cfg["robot"]["joint_indices"][arm + "_arm"]
                q_full[lo:hi] = data[arm + "_q"][row]
                transform = data[arm + "_marker_T"][row]
                observations[name + "_in_torso"] = {
                    "xyz_mm": transform[:3, 3].tolist(),
                    "quaternion_xyzw": Rotation.from_matrix(transform[:3, :3]).as_quat().tolist(),
                }
            samples.append({"index": index, "joint_q_full": q_full.tolist(), "bodies": observations})
        capture = {
            "meta": {"robot_version": "63", "robot_layout": self.cfg["robot"]["robot_layout"],
                     "fk_root": "waist_yaw_link", "joint_q_indices": self.cfg["robot"]["joint_indices"]},
            "bodies": [{"name": name, "link_offset_mm": [0, 0, 0]}
                       for name in ("l_hand", "r_hand", "torso")],
            "samples": samples,
        }
        legacy = OPT.FixedPositionModel(copy.deepcopy(self.cfg["kinematics"]), self.chains)
        legacy_p = np.zeros(legacy.parameter_count)
        legacy_p[:3] = [11, -7, 4]
        legacy_p[legacy.etk_slice["left"]] = [0.4, -0.2, 0.3]
        legacy_p[legacy.etk_slice["right"]] = [-0.3, 0.2, -0.4]
        metadata = OPT._frame_metadata(legacy, legacy_p)
        per_pose_p = np.zeros(model.parameter_count)
        for arm in OPT.ARMS:
            per_pose_p[model.etk_slice[arm]] = legacy_p[legacy.etk_slice[arm]]
        per_pose_metadata = OPT._frame_metadata(model, per_pose_p, data)
        cases = [(qingtong, freeze_etk, version) for version in (1, 2)
                 for qingtong in (False, True) for freeze_etk in (False, True)]
        for qingtong, freeze_etk, version in cases:
            with self.subTest(qingtong=qingtong, freeze_etk=freeze_etk, schema=version), \
                    tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                cfg = copy.deepcopy(self.cfg)
                cfg["kinematics"]["output_dir"] = str(root / "output")
                cfg["kinematics"]["source_urdf"] = str(SOURCE)
                if qingtong:
                    cfg["kinematics"]["validation"]["qingtong"] = True
                else:
                    # Missing switch must retain the original validation mode.
                    cfg["kinematics"]["validation"].pop("qingtong", None)
                config_path, capture_path, frames_path = (root / name for name in
                                                          ("config.yaml", "capture.json", "frames.json"))
                config_path.write_text(yaml.safe_dump(cfg))
                capture_path.write_text(json.dumps(capture))
                OPT._write_frame_metadata_file(frames_path, metadata if version == 1 else per_pose_metadata)
                args = [OPT.__file__, str(capture_path), "--config", str(config_path),
                        "--validate-urdf", str(SOURCE), "--frame-transforms", str(frames_path),
                        "--validation-max-mm", "1000"]
                if not freeze_etk:
                    args.append("--no-freeze-frame-angles")

                def probe_validation(candidate, observations, subset, initial=None, fit_etk=True):
                    if qingtong:
                        self.assertIsInstance(candidate, OPT.PerPoseCTBModel)
                        self.assertEqual(candidate.sample_indices, (8, 2, 19))
                        self.assertEqual(candidate.ctb_rotation_bound_deg, 3.0)
                        np.testing.assert_array_equal(candidate.ctb_r0, 0)
                    else:
                        self.assertIs(type(candidate), OPT.FixedPositionModel)
                        np.testing.assert_allclose(candidate.ctb_r0, legacy_p[:3] if version == 1 else 0, atol=1e-12)
                    self.assertEqual(subset, "all")
                    self.assertEqual(fit_etk, not freeze_etk)
                    for arm in OPT.ARMS:
                        self.assertEqual(candidate.chains[arm].urdf_path, SOURCE.resolve())
                        np.testing.assert_allclose(candidate.etk_r0[arm], legacy_p[legacy.etk_slice[arm]], atol=1e-12)
                    p = np.zeros(candidate.parameter_count)
                    if qingtong:
                        candidate.check_ctb_angles(p)
                    return p, SimpleNamespace(success=True, message="validation probe", nfev=1)

                stdout = io.StringIO()
                incompatible_saved_frames = version == 2 and not qingtong and freeze_etk
                with patch.dict(os.environ, ROBOT_VERSION="63"), patch.object(sys, "argv", args), \
                        patch.object(OPT, "fit_frame_angles", side_effect=probe_validation) as frame_fit, \
                        patch.object(OPT, "error_rows", wraps=OPT.error_rows) as metrics, \
                        patch.object(OPT, "fit", side_effect=AssertionError("unexpected Ei fit")), \
                        patch("builtins.input", side_effect=AssertionError("unexpected install prompt")), \
                        redirect_stdout(stdout):
                    if incompatible_saved_frames:
                        with self.assertRaisesRegex(SystemExit, "qingtong"):
                            OPT.main()
                    else:
                        self.assertEqual(OPT.main(), 0)
                if incompatible_saved_frames:
                    frame_fit.assert_not_called()
                    metrics.assert_not_called()
                elif not qingtong and freeze_etk:
                    frame_fit.assert_not_called()
                    self.assertIs(type(metrics.call_args.args[0]), OPT.FixedPositionModel)
                    self.assertIn("FIXED VALIDATION", stdout.getvalue())
                else:
                    frame_fit.assert_called_once()
                if qingtong:
                    self.assertIn("CONDITIONAL", stdout.getvalue())
                self.assertFalse((root / "output").exists())

    def test_legacy_frame_fit_still_has_nine_shared_angles_without_three_degree_bound(self):
        sample_model = self.make_model([1, 2, 3, 4, 5, 6])
        target = np.array([4.5, -1.0, 1.0])
        data = self.make_data(sample_model, np.tile(target, (6, 1)))
        kin = copy.deepcopy(self.cfg["kinematics"])
        kin["ctb"]["initial_rotvec_deg"] = [0.0, 0.0, 0.0]
        model = OPT.FixedPositionModel(kin, self.chains)
        p, result = OPT.fit_frame_angles(model, data, "all")
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.x.shape, (9,))
        self.assertGreater(np.linalg.norm(p[model.ctb_slice]), 3.0)
        np.testing.assert_allclose(p[model.ctb_slice], target, atol=1e-7)
        np.testing.assert_array_equal(p[9:], 0)

    def test_frame_metadata_round_trip_keeps_each_index_and_capture_identity(self):
        model = self.make_model([19, 8, 4])
        p = np.zeros(model.parameter_count)
        rotations = np.array([[1, 2, 2], [-1.2, 0.5, -0.8], [0, 0, 0]])
        p[model.ctb_slice] = rotations.ravel()
        data = self.make_data(model, rotations)
        metadata = OPT._frame_metadata(model, p, data)
        self.assertEqual(metadata["schema_version"], 2)
        self.assertEqual(metadata["capture_sha256"], data["capture_sha256"])
        self.assertEqual(metadata["ctb"]["reference"], "identity")
        self.assertEqual(metadata["ctb"]["rotation_bound_deg"], 3.0)
        self.assertEqual([sample["index"] for sample in metadata["ctb"]["samples"]], [19, 8, 4])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "frame_transforms.json"
            OPT._write_frame_metadata_file(path, metadata)
            restored = OPT._read_frame_metadata(path)
        np.testing.assert_allclose([sample["rotvec_deg"] for sample in restored["ctb"]["samples"]],
                                   rotations, atol=1e-12)
        self.assertEqual(set(restored["etk"]), {"left", "right"})

    def test_metadata_requires_provenance_and_rejects_duplicate_or_overbound_samples(self):
        model = self.make_model([4, 11])
        p = np.zeros(model.parameter_count)
        with self.assertRaises(ValueError):
            OPT._frame_metadata(model, p)
        metadata = OPT._frame_metadata(model, p, self.make_data(model, np.zeros((2, 3))))
        invalid = []
        bad_hash = copy.deepcopy(metadata)
        bad_hash["capture_sha256"] = "not-a-capture-hash"
        invalid.append(bad_hash)
        duplicate = copy.deepcopy(metadata)
        duplicate["ctb"]["samples"].append(copy.deepcopy(duplicate["ctb"]["samples"][0]))
        invalid.append(duplicate)
        overflow = copy.deepcopy(metadata)
        del overflow["ctb"]["samples"][0]["rotation_matrix"]
        overflow["ctb"]["samples"][0]["rotvec_deg"] = [2.0, 2.0, 2.0]
        invalid.append(overflow)
        for item in invalid:
            with self.subTest(item=item), self.assertRaises(ValueError):
                OPT._normalise_frame_metadata(item, "test-metadata.json")

    def test_legacy_shared_frame_metadata_remains_readable(self):
        model = OPT.FixedPositionModel(copy.deepcopy(self.cfg["kinematics"]), self.chains)
        p = np.zeros(model.parameter_count)
        p[:3] = [1.2, -2.1, 0.4]
        metadata = OPT._normalise_frame_metadata(OPT._frame_metadata(model, p), "legacy.json")
        self.assertEqual(metadata["schema_version"], 1)
        np.testing.assert_allclose(metadata["ctb"]["rotvec_deg"], p[:3], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
