"""Offline guided-workflow checks; all motion, solving and installation are mocked."""
from contextlib import redirect_stdout
import copy
import hashlib
import io
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
import mocap_optimize_fpf as workflow


def capture_fixture(cfg, count, *, qingtong=False, version="62"):
    """Use recorder-format four-channel measurements without importing ROS."""
    samples = []
    for index in range(1, count + 1):
        shoulder_rotation = Rotation.from_euler(
            "z", 2.0 if qingtong and index == count else 0.0, degrees=True)
        shoulder_position = np.array([120.0, 240.0, 360.0])
        bodies = {}
        for side, hand in ((-1, "l_hand"), (1, "r_hand")):
            hand_position = np.array([150.0 + index, side * 350.0, 40.0])
            hand_rotation = Rotation.from_euler("x", index, degrees=True)
            bodies[hand + "_in_torso"] = {
                "xyz_mm": hand_position.tolist(),
                "quaternion_xyzw": hand_rotation.as_quat().tolist(),
                "valid_frames": 90,
            }
            bodies[hand + "_in_l_shoulder"] = {
                "xyz_mm": shoulder_rotation.inv().apply(hand_position - shoulder_position).tolist(),
                "quaternion_xyzw": (shoulder_rotation.inv() * hand_rotation).as_quat().tolist(),
                "valid_frames": 90,
            }
        samples.append({"index": index, "joint_q_full": [0.0] * 20, "bodies": bodies})
    return {
        "meta": {"robot_version": version, "robot_layout": "wheel62",
                 "fk_root": "waist_yaw_link", "joint_q_indices": cfg["robot"]["joint_indices"]},
        "bodies": [{"name": name, "link_offset_mm": [0.0, 0.0, 0.0]}
                   for name in ("l_hand", "r_hand", "torso", "l_shoulder")],
        "samples": samples,
    }


class FpfWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config_path = HERE / "config/calib_s63.yaml"
        self.config_bytes = self.config_path.read_bytes()
        self.cfg = yaml.safe_load(self.config_bytes)
        self.output = self.root / "output"
        self.fit_path = self.root / "capture_fit.json"
        self.validation_path = self.root / "capture_vali.json"
        self.events = []
        self.version = "62"
        self.commands = []
        self.command_configs = []
        self.args = ["--config", str(self.config_path), "--fit-capture", str(self.fit_path),
                     "--validation-capture", str(self.validation_path), "--output-dir", str(self.output)]

    def write_capture(self, path, count, *, qingtong=False):
        path.write_text(json.dumps(capture_fixture(self.cfg, count, qingtong=qingtong,
                                                  version=self.version)), encoding="utf-8")

    def fake_runner(self, *, qingtong=False, fail_at=None, partial_at=None, missing_candidate=False,
                    wrong_source_hash=False):
        def run(command, log_path):
            command = [str(value) for value in command]
            self.commands.append(command)
            config_path = Path(command[command.index("--config") + 1])
            cfg = yaml.safe_load(config_path.read_text())
            self.command_configs.append(cfg)
            self.assertTrue(Path(cfg["kinematics"]["output_dir"]).is_absolute())
            for key in ("teach_left", "teach_right", "urdf", "teach_head"):
                self.assertTrue(Path(cfg["calibration"][key]).is_absolute(), key)
            if str(workflow.RECORDER) in command:
                self.assertEqual(workflow.RECORDER, HERE / "record_joint_poses.py")
                self.assertEqual(command[command.index("--robot-version") + 1], self.version)
                self.assertEqual(command[command.index("--urdf") + 1], cfg["kinematics"]["source_urdf"])
                self.assertEqual(cfg["calibration"]["urdf"], cfg["kinematics"]["source_urdf"])
                is_fit = Path(cfg["calibration"]["teach_left"]).name.endswith("_fit.json")
                event = "record_fit" if is_fit else "record_validation"
                self.assertIn("--fixture", command)
                self.assertEqual(command[command.index("--fixture") + 1], "new")
                self.assertNotIn("--motion-only", command)
                destination = Path(command[command.index("--output") + 1])
                self.assertNotIn(destination, (self.fit_path, self.validation_path))
                count = 30 if is_fit else 10
                if partial_at == event:
                    count -= 1
                self.write_capture(destination, count, qingtong=qingtong and is_fit)
            elif "--validate-urdf" in command:
                event = "validate"
                self.assertNotIn("--mocap-source", command)
                self.assertIn("--no-freeze-frame-angles", command)
                self.assertNotIn("--output-urdf", command)
                self.assertTrue(Path(command[command.index("--validate-urdf") + 1]).is_file())
            else:
                event = "fit"
                self.assertNotIn("--mocap-source", command)
                self.assertIn("--output-urdf", command)
                self.assertIn("--force-output", command)
                self.assertIn("--no-install", command)
                if not missing_candidate:
                    (self.output / cfg["kinematics"]["urdf_final_name"]).write_text("<robot name='mock'/>\n")
                source = Path(command[command.index("--urdf") + 1])
                source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
                (self.output / "kinematic_offsets.json").write_text(json.dumps({
                    "source_urdf_sha256": "wrong" if wrong_source_hash else source_hash,
                }))
            self.events.append(event)
            log_path.write_text(event + "\n")
            if event == fail_at:
                raise RuntimeError("mock failed " + event)
        return run

    def invoke(self, *, answers=("y", "y", "y", "y"), source_answer="2", version=None, **runner_kwargs):
        self.version = version or "62"

        def install(source, candidate, target):
            self.assertEqual(self.events[-1], "validate")
            self.assertEqual(target.name, f"biped_s{self.version}.urdf")
            self.assertTrue(source.is_file())
            self.assertEqual(candidate, self.output / "urdf_final.urdf")
            self.events.append("install")

        with patch.dict(os.environ, {}, clear=False):
            os.environ.pop("ROBOT_VERSION", None)
            if version is not None:
                os.environ["ROBOT_VERSION"] = version
            with patch("builtins.input", side_effect=(source_answer, *answers)) as input_mock, \
                    patch.object(workflow, "run_command", side_effect=self.fake_runner(**runner_kwargs)), \
                    patch.object(workflow.optimizer, "_confirm_apply_urdf", side_effect=install) as installer, \
                    redirect_stdout(io.StringIO()):
                try:
                    result = workflow.main(self.args)
                finally:
                    self.prompts = [call.args[0] for call in input_mock.call_args_list]
                    self.assertEqual(os.environ["ROBOT_VERSION"], self.version)
                    self.assertEqual(self.config_path.read_bytes(), self.config_bytes)
            return result, installer

    def test_fresh_shared_capture_runs_stages_and_installs_only_after_validation(self):
        result, installer = self.invoke()
        self.assertEqual(result, 0)
        installer.assert_called_once()
        self.assertEqual(self.events, ["record_fit", "record_validation", "fit", "validate", "install"])
        self.assertEqual(len(json.loads(self.fit_path.read_text())["samples"]), 30)
        self.assertEqual(len(json.loads(self.validation_path.read_text())["samples"]), 10)
        self.check_recording_source("motive", self.cfg["mocap_bodies"])
        self.assertIn("青瞳", self.prompts[0])
        self.assertIn("OptiTrack", self.prompts[0])
        self.assertIn("开始拟合采点", self.prompts[1])
        self.check_effective_mode("shared", False)

    def test_qingtong_capture_keeps_per_sample_mode_for_fit_and_validation(self):
        result, installer = self.invoke(source_answer="qingtong", qingtong=True)
        self.assertEqual(result, 0)
        installer.assert_called_once()
        self.check_effective_mode("per_sample", True)
        report = json.loads(next(self.output.glob("fpf_*/stability.json")).read_text())
        self.assertEqual(report["mode"], "qingtong")
        self.assertGreater(report["arms"]["left"]["max_rotation_deg"], 1.0)

    def test_requested_version_is_used_through_capture_fit_validation_and_install(self):
        self.args = self.args[2:]  # Exercise the default family-config selection.
        for version in ("62", "63", "500062", "500063"):
            with self.subTest(version=version):
                self.events.clear()
                self.commands.clear()
                self.command_configs.clear()
                result, installer = self.invoke(version=version)
                self.assertEqual(result, 0)
                installer.assert_called_once()
                target = HERE.parents[1] / f"kuavo_assets/models/biped_s{version}/urdf/biped_s{version}.urdf"
                self.assertEqual(installer.call_args.args[2], target)
                for cfg in self.command_configs:
                    self.assertEqual(cfg["robot"]["robot_version"], version)
                    self.assertEqual(Path(cfg["kinematics"]["source_urdf"]), target)
                    self.assertEqual(Path(cfg["kinematics"]["output_dir"]), self.output)
                fit_command = self.commands[2]
                self.assertEqual(Path(fit_command[fit_command.index("--urdf") + 1]),
                                 workflow.optimizer.write_urdf.resolve_fit_source(target))

    def test_default_model_and_output_paths_follow_requested_version(self):
        for version in ("62", "63", "500062", "500063"):
            with self.subTest(version=version), patch.dict(os.environ, ROBOT_VERSION=version):
                effective = workflow.optimizer._config_for_capture(self.cfg, {})
                self.assertEqual(effective["robot"]["robot_version"], version)
                self.assertTrue(effective["kinematics"]["source_urdf"].endswith(
                    f"/biped_s{version}/urdf/biped_s{version}.urdf"))
                self.assertEqual(effective["kinematics"]["output_dir"], f"../output_s{version}")
                self.assertEqual(self.cfg, yaml.safe_load(self.config_bytes))

    def test_reused_capture_from_other_version_is_rejected(self):
        self.write_capture(self.fit_path, 30)
        with self.assertRaisesRegex(ValueError, "conflicts with capture"):
            self.invoke(version="63", answers=("r",))
        self.assertEqual(self.events, [])

    def check_recording_source(self, source, expected_bodies):
        recorder_count = 0
        for command, cfg in zip(self.commands, self.command_configs):
            self.assertEqual(cfg["mocap_bodies"], expected_bodies)
            if str(workflow.RECORDER) in command:
                recorder_count += 1
                self.assertEqual(command[command.index("--mocap-source") + 1], source)
        self.assertEqual(recorder_count, 2)

    def test_qingtong_topics_apply_to_both_recordings_without_forcing_per_sample_ctb(self):
        result, installer = self.invoke(source_answer="1")
        self.assertEqual(result, 0)
        installer.assert_called_once()
        expected_bodies = copy.deepcopy(self.cfg["mocap_bodies"])
        self.assertEqual({body["name"] for body in expected_bodies},
                         {"l_hand", "r_hand", "torso", "l_shoulder"})
        for body in expected_bodies:
            body["topic"] = f"/vrpn_client_node/{body['name']}/pose"
            body["msg_type"] = "PoseStamped"
        self.check_recording_source("qingtong", expected_bodies)
        self.check_effective_mode("shared", False)

    def test_optitrack_preserves_custom_body_topics_and_message_types(self):
        for body in self.cfg["mocap_bodies"]:
            body["topic"] = f"/custom_motive/{body['name']}/pose"
            body["msg_type"] = "Pose"
        self.config_path = self.root / "custom.yaml"
        self.config_bytes = yaml.safe_dump(self.cfg).encode("utf-8")
        self.config_path.write_bytes(self.config_bytes)
        self.args[self.args.index("--config") + 1] = str(self.config_path)
        result, installer = self.invoke(source_answer="optitrack")
        self.assertEqual(result, 0)
        installer.assert_called_once()
        self.check_recording_source("motive", self.cfg["mocap_bodies"])

    def test_source_selection_aliases_return_independent_config(self):
        for answer, expected_source in (("1", "qingtong"), ("q", "qingtong"),
                                        ("qingtong", "qingtong"), ("2", "motive"),
                                        ("o", "motive"), ("optitrack", "motive"),
                                        ("motive", "motive")):
            with self.subTest(answer=answer), patch("builtins.input", return_value=answer), \
                    redirect_stdout(io.StringIO()):
                before = copy.deepcopy(self.cfg)
                selected, source = workflow.select_mocap_source(self.cfg)
                self.assertEqual(source, expected_source)
                self.assertIsNot(selected, self.cfg)
                self.assertEqual(self.cfg, before)
                if source == "motive":
                    self.assertEqual(selected, before)
                selected["mocap_bodies"][0]["topic"] = "/changed_after_selection"
                self.assertEqual(self.cfg, before)

    def test_cancel_initial_source_selection_has_no_commands_or_output_files(self):
        for answer in ("n", "", EOFError(), KeyboardInterrupt()):
            with self.subTest(answer=answer), \
                    patch.object(workflow.optimizer.write_urdf, "resolve_fit_source") as resolve_source:
                with self.assertRaises(workflow.Cancelled):
                    self.invoke(source_answer=answer)
                resolve_source.assert_not_called()
                self.assertEqual(len(self.prompts), 1)
                self.assertFalse(self.output.exists())
                self.assertFalse(self.fit_path.exists())
                self.assertFalse(self.validation_path.exists())
                self.assertEqual(self.commands, [])

    def check_effective_mode(self, mode, qingtong):
        for cfg in self.command_configs[1:]:
            self.assertEqual(cfg["kinematics"]["ctb"]["mode"], mode)
            self.assertIs(cfg["kinematics"]["validation"]["qingtong"], qingtong)
            self.assertEqual(cfg["kinematics"]["ctb"]["rotation_bound_deg"], 3.0)
        for name in ("fit", "validation"):
            cfg = yaml.safe_load(next(self.output.glob(f"fpf_*/{name}.yaml")).read_text())
            self.assertEqual(cfg["kinematics"]["ctb"]["mode"], mode)
            self.assertIs(cfg["kinematics"]["validation"]["qingtong"], qingtong)
            for arm in ("left", "right"):
                self.assertEqual(Path(cfg["calibration"]["teach_" + arm]).name,
                                 f"teach_{arm}_joint_{name}.json")

    def test_reusing_complete_captures_skips_recorder(self):
        self.write_capture(self.fit_path, 30)
        self.write_capture(self.validation_path, 10)
        fit_before, validation_before = self.fit_path.read_bytes(), self.validation_path.read_bytes()
        result, installer = self.invoke(answers=("r", "r", "y", "y"))
        self.assertEqual(result, 0)
        installer.assert_called_once()
        self.assertEqual(self.events, ["fit", "validate", "install"])
        self.assertEqual(self.fit_path.read_bytes(), fit_before)
        self.assertEqual(self.validation_path.read_bytes(), validation_before)

    def test_failed_stages_stop_without_installing(self):
        for stage in ("record_fit", "record_validation", "fit", "validate"):
            with self.subTest(stage=stage), self.assertRaisesRegex(RuntimeError, "mock failed " + stage):
                self.events.clear()
                self.invoke(fail_at=stage)
            self.assertNotIn("install", self.events)
            self.assertEqual(self.events[-1], stage)

    def test_partial_fresh_capture_preserves_existing_file(self):
        for stage, destination in (("record_fit", self.fit_path),
                                   ("record_validation", self.validation_path)):
            with self.subTest(stage=stage):
                before = b"existing capture must survive\n"
                destination.write_bytes(before)
                self.events.clear()
                with self.assertRaises(ValueError):
                    self.invoke(partial_at=stage)
                self.assertEqual(destination.read_bytes(), before)
                self.assertNotIn("fit", self.events)
                self.assertNotIn("install", self.events)
                fresh_name = "capture_fit.new.json" if stage == "record_fit" else "capture_vali.new.json"
                self.assertTrue(list(self.output.glob("fpf_*/" + fresh_name)))

    def test_reused_partial_validation_is_rejected(self):
        self.write_capture(self.fit_path, 30)
        self.write_capture(self.validation_path, 9)
        with self.assertRaisesRegex(ValueError, "采集不完整"):
            self.invoke(answers=("r", "r"))
        self.assertNotIn("fit", self.events)
        self.assertNotIn("install", self.events)

    def test_missing_candidate_or_changed_source_hash_prevents_install(self):
        with self.assertRaisesRegex(RuntimeError, "拟合未生成候选"):
            self.invoke(missing_candidate=True)
        self.assertNotIn("validate", self.events)
        self.assertNotIn("install", self.events)
        self.events.clear()
        with self.assertRaisesRegex(RuntimeError, "拟合基准发生变化"):
            self.invoke(wrong_source_hash=True)
        self.assertEqual(self.events[-1], "validate")
        self.assertNotIn("install", self.events)

    def test_cancel_before_recording_does_not_run_commands(self):
        with self.assertRaises(workflow.Cancelled):
            self.invoke(answers=("n",))
        self.assertEqual(self.events, [])
        self.assertFalse(self.fit_path.exists())

    def test_shell_help_is_safe_and_has_no_frame_angle_switch(self):
        environment = dict(os.environ)
        environment.pop("ROBOT_VERSION", None)
        result = subprocess.run(["bash", str(HERE / "mocap_optimize_fpf.sh"), "--help"],
                                env=environment, capture_output=True, text=True, check=True)
        self.assertIn("默认 ROBOT_VERSION=62", result.stdout)
        self.assertIn("--validation-capture", result.stdout)
        self.assertNotIn("--no-freeze-frame-angles", result.stdout)


if __name__ == "__main__":
    unittest.main()
