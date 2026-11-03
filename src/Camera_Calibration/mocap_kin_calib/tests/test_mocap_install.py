"""URDF installation and confirmation checks using temporary runtime models."""
from contextlib import redirect_stdout
import copy
import io
import os
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import yaml

import test_mocap_output_limits as limits
import write_urdf as installer


class MocapInstallTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        limits.OutputLimitTests.setUpClass()
        cls.inputs = limits.OutputLimitTests()
        cls.optimizer = next(module for module in limits.MODULES if module.__name__.endswith("1010"))
        cls.original = limits.SOURCE.read_bytes()
        model = cls.inputs.models[cls.optimizer]
        parameters = np.zeros(model.parameter_count)
        parameters[9] = 1.
        tree = cls.optimizer._xml_tree(limits.SOURCE)
        root = tree.getroot()
        cls.optimizer._write_frame_metadata(root, cls.optimizer._frame_metadata(model, parameters))
        for arm in cls.optimizer.ARMS:
            for name, correction in zip(model.chains[arm].joint_names, model.unpack(parameters, arm)[2]):
                joint = root.find("joint[@name='" + name + "']")
                cls.optimizer._set_origin(joint, cls.optimizer._origin(joint.find("origin")) @ correction)
        stream = io.BytesIO()
        tree.write(stream, encoding="utf-8", xml_declaration=True)
        cls.calibrated = stream.getvalue()

    def files(self, root):
        runtime = root / "runtime"
        runtime.mkdir()
        target = runtime / "other_robot.urdf"
        target.write_bytes(self.original)
        source = root / "nominal_source.urdf"
        source.write_bytes(self.original)
        candidate = root / "candidate.urdf"
        candidate.write_bytes(self.calibrated)
        return target, source, candidate

    def directory_files(self, root):
        runtime = root / "urdf"
        shutil.copytree(limits.SOURCE.parent, runtime)
        target = runtime / "biped_s63.urdf"
        source = root / "nominal_source.urdf"
        source.write_bytes(self.original)
        candidate = root / "candidate.urdf"
        candidate.write_bytes(self.calibrated)
        return target, source, candidate

    @staticmethod
    def snapshot(root):
        return {str(path.relative_to(root)): path.read_bytes()
                for path in root.rglob("*") if path.is_file()}

    @staticmethod
    def move_arm_origin(path):
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True, insert_pis=True))
        tree = ET.parse(path, parser=parser)
        origin = tree.getroot().find("joint[@name='zarm_l4_joint']/origin")
        xyz = np.fromstring(origin.get("xyz"), sep=" ")
        xyz[0] += .0001
        origin.set("xyz", " ".join(format(value, ".17g") for value in xyz))
        tree.write(path, encoding="utf-8", xml_declaration=True)

    def test_dry_run_never_creates_backups_or_replaces_runtime(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)
            target, source, candidate = self.files(root)
            before = self.snapshot(root)
            installer.install_urdf(target, candidate, apply=False, nominal_source=source)
            self.assertEqual(self.snapshot(root), before)

    def test_yes_installs_and_repeat_uses_unchanged_nominal(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)
            target, source, candidate = self.files(root)
            self.assertEqual(installer.resolve_fit_source(target), target)
            with patch("builtins.input", return_value="yes") as prompt:
                self.optimizer._confirm_apply_urdf(source, candidate, target)
            prompt.assert_called_once()
            nominal = installer.nominal_backup_path(target)
            self.assertEqual(nominal, target.with_name("other_robot.nominal.urdf"))
            self.assertEqual(nominal.read_bytes(), self.original)
            self.assertEqual(target.read_bytes(), self.calibrated)
            self.assertEqual(installer.resolve_fit_source(target), nominal)
            backups = list(target.parent.glob(target.name + ".bak.*"))
            self.assertEqual(len(backups), 1)
            self.assertEqual(backups[0].read_bytes(), self.original)
            baseline_mtime = nominal.stat().st_mtime_ns

            second = root / "second_candidate.urdf"
            shutil.copyfile(candidate, second)
            self.move_arm_origin(second)
            previous_runtime = target.read_bytes()
            with patch("builtins.input", return_value="yes"):
                self.optimizer._confirm_apply_urdf(nominal, second, target)
            self.assertEqual(target.read_bytes(), second.read_bytes())
            self.assertEqual(nominal.read_bytes(), self.original)
            self.assertEqual(nominal.stat().st_mtime_ns, baseline_mtime)
            self.assertEqual(installer.resolve_fit_source(target), nominal)
            self.assertEqual(sorted(path.read_bytes() for path in target.parent.glob(target.name + ".bak.*")),
                             sorted([self.original, previous_runtime]))

    def test_no_empty_and_eof_leave_every_file_unchanged(self):
        for answer in ("no", "", EOFError()):
            with self.subTest(answer=repr(answer)), tempfile.TemporaryDirectory() as temp, \
                    redirect_stdout(io.StringIO()):
                root = Path(temp)
                target, source, candidate = self.files(root)
                before = self.snapshot(root)
                options = {"side_effect": answer} if isinstance(answer, Exception) else {"return_value": answer}
                with patch("builtins.input", **options) as prompt:
                    self.optimizer._confirm_apply_urdf(source, candidate, target)
                prompt.assert_called_once()
                self.assertEqual(self.snapshot(root), before)

    def test_invalid_answer_reprompts_without_installing(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)
            target, source, candidate = self.files(root)
            before = self.snapshot(root)
            with patch("builtins.input", side_effect=["maybe", "no"]) as prompt:
                self.optimizer._confirm_apply_urdf(source, candidate, target)
            self.assertEqual(prompt.call_count, 2)
            self.assertEqual(self.snapshot(root), before)

    def test_structural_mismatch_is_rejected_without_any_write(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)
            target, source, candidate = self.files(root)
            parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
            tree = ET.parse(candidate, parser=parser)
            mass = tree.getroot().find("link/inertial/mass")
            mass.set("value", str(float(mass.get("value")) + 1.))
            tree.write(candidate, encoding="utf-8", xml_declaration=True)
            before = self.snapshot(root)
            with self.assertRaises(ValueError):
                installer.install_urdf(target, candidate, apply=True, nominal_source=source)
            self.assertEqual(self.snapshot(root), before)

    def test_existing_nominal_is_never_overwritten_on_hash_mismatch(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)
            target, source, candidate = self.files(root)
            nominal = installer.nominal_backup_path(target)
            nominal.write_bytes(self.original + b"\n")
            before = self.snapshot(root)
            with self.assertRaises(ValueError):
                installer.install_urdf(target, candidate, apply=True, nominal_source=source)
            self.assertEqual(self.snapshot(root), before)

    def test_calibrated_fit_source_without_valid_nominal_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            target, source, candidate = self.files(root)
            installer.ensure_nominal_source(source)
            with self.assertRaises(ValueError):
                installer.ensure_nominal_source(candidate)
            target.write_bytes(candidate.read_bytes())
            with self.assertRaises(ValueError):
                installer.resolve_fit_source(target)
            nominal = installer.nominal_backup_path(target)
            nominal.write_bytes(candidate.read_bytes())
            with self.assertRaises(ValueError):
                installer.resolve_fit_source(target)
            nominal.write_bytes(self.original)
            self.assertEqual(installer.resolve_fit_source(target), nominal)

    @staticmethod
    def without_arm_origins(payload):
        root = ET.fromstring(payload)
        for joint in root.findall("joint"):
            if joint.get("name") in installer.ARM_JOINTS:
                origin = joint.find("origin")
                if origin is not None:
                    joint.remove(origin)

        def canonical(element):
            return (element.tag, tuple(sorted(element.attrib.items())), (element.text or "").strip(),
                    tuple(canonical(child) for child in element))

        return canonical(root)

    def assert_directory_models_updated(self, runtime, original_files, candidate):
        expected = ET.fromstring(candidate.read_bytes())
        origins = {joint.get("name"): joint.find("origin").attrib for joint in expected.findall("joint")
                   if joint.get("name") in installer.ARM_JOINTS}
        changed = []
        for relative, original in original_files.items():
            actual = (runtime / relative).read_bytes()
            if relative.endswith(".xacro"):
                self.assertEqual(actual, original)
                continue
            self.assertTrue(relative.endswith(".urdf"), relative)
            changed.append(relative)
            self.assertNotEqual(actual, original, relative)
            if Path(relative).name in ("Larm.urdf", "Rarm.urdf"):
                self.assertIsNotNone(ET.fromstring(actual))
                continue
            self.assertEqual(self.without_arm_origins(original), self.without_arm_origins(actual), relative)
            joints = {joint.get("name"): joint for joint in ET.fromstring(actual).findall("joint")}
            for name, origin in origins.items():
                self.assertEqual(joints[name].find("origin").attrib, origin, (relative, name))
        self.assertEqual(len(changed), 7)

    def test_directory_install_updates_all_models_and_preserves_original_baseline(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)
            target, source, candidate = self.directory_files(root)
            before = self.snapshot(target.parent)
            self.assertEqual(len(before), 8)
            installer.install_urdf_directory(target, candidate, apply=True, nominal_source=source)
            self.assert_directory_models_updated(target.parent, before, candidate)
            backup = list(root.glob("urdf.bak.*"))
            self.assertEqual(len(backup), 1)
            self.assertEqual(self.snapshot(backup[0]), before)
            nominal = installer.nominal_backup_path(target)
            self.assertEqual(nominal.read_bytes(), self.original)
            self.assertEqual(installer.resolve_fit_source(target), nominal)
            nominal_mtime = nominal.stat().st_mtime_ns

            second = root / "second_candidate.urdf"
            shutil.copyfile(candidate, second)
            self.move_arm_origin(second)
            previous_runtime = self.snapshot(target.parent)
            installer.install_urdf_directory(target, second, apply=True, nominal_source=nominal)
            self.assert_directory_models_updated(target.parent, before, second)
            self.assertEqual(nominal.read_bytes(), self.original)
            self.assertEqual(nominal.stat().st_mtime_ns, nominal_mtime)
            self.assertEqual(installer.resolve_fit_source(target), nominal)
            backups = list(root.glob("urdf.bak.*"))
            self.assertEqual(len(backups), 2)
            self.assertIn(previous_runtime, [self.snapshot(path) for path in backups])

    def test_directory_dry_run_no_and_eof_create_no_backup(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)
            target, source, candidate = self.directory_files(root)
            before = self.snapshot(root)
            installer.install_urdf_directory(target, candidate, apply=False, nominal_source=source)
            self.assertEqual(self.snapshot(root), before)
            for options in ({"return_value": "no"}, {"return_value": ""}, {"side_effect": EOFError()}):
                with patch("builtins.input", **options):
                    self.optimizer._confirm_apply_urdf(source, candidate, target)
                self.assertEqual(self.snapshot(root), before)
            self.assertEqual(list(root.glob("urdf.bak.*")), [])

    def test_incompatible_directory_member_blocks_every_change(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)
            target, source, candidate = self.directory_files(root)
            damaged = target.parent / "biped_s63_gazebo.urdf"
            tree = ET.parse(damaged)
            joint = tree.getroot().find("joint[@name='zarm_r7_joint']")
            tree.getroot().remove(joint)
            tree.write(damaged, encoding="utf-8", xml_declaration=True)
            before = self.snapshot(root)
            with self.assertRaises((ValueError, RuntimeError)):
                installer.install_urdf_directory(target, candidate, apply=True, nominal_source=source)
            self.assertEqual(self.snapshot(root), before)
            self.assertEqual(list(root.glob("urdf.bak.*")), [])

    def test_directory_partial_replace_failure_rolls_back_all_runtime_files(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)
            target, source, candidate = self.directory_files(root)
            before = self.snapshot(target.parent)
            original_replace = os.replace
            replacements = []

            def fail_third_runtime_replace(src, dst, *args, **kwargs):
                destination = Path(dst)
                try:
                    destination.relative_to(target.parent)
                    inside_runtime = True
                except ValueError:
                    inside_runtime = False
                if inside_runtime and destination.suffix == ".urdf":
                    replacements.append(destination)
                    if len(replacements) == 3:
                        raise OSError("injected third runtime replacement failure")
                return original_replace(src, dst, *args, **kwargs)

            with patch.object(installer.os, "replace", side_effect=fail_third_runtime_replace):
                with self.assertRaises((OSError, RuntimeError)):
                    installer.install_urdf_directory(target, candidate, apply=True, nominal_source=source)
            self.assertGreaterEqual(len(replacements), 3)
            after = self.snapshot(target.parent)
            self.assertEqual(after, before)
            backups = list(root.glob("urdf.bak.*"))
            self.assertEqual(len(backups), 1)
            self.assertEqual(self.snapshot(backups[0]), before)

    def main_inputs(self, root):
        config, capture, output = self.inputs.make_inputs(root, self.optimizer)
        target, source, _ = self.directory_files(root)
        cfg = yaml.safe_load(config.read_text())
        cfg["kinematics"]["source_urdf"] = str(target)
        config.write_text(yaml.safe_dump(cfg))
        return config, capture, output, target, source, cfg

    def test_main_success_then_yes_installs_only_temporary_runtime(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)
            config, capture, output, target, _, cfg = self.main_inputs(root)
            p = np.zeros(self.inputs.models[self.optimizer].parameter_count)
            p[9] = 1.
            success = SimpleNamespace(success=True, message="test", nfev=1)
            args = [self.optimizer.__file__, str(capture), "--config", str(config), "--output-urdf", "--force-output"]
            with patch.dict(os.environ, ROBOT_VERSION="63"), patch.object(sys, "argv", args), \
                    patch.object(self.optimizer, "_config_for_capture", return_value=copy.deepcopy(cfg)), \
                    patch.object(self.optimizer, "fit", side_effect=[(np.zeros_like(p), success), (p, success)]), \
                    patch("builtins.input", return_value="yes") as prompt:
                self.optimizer.main()
            prompt.assert_called_once()
            self.assertEqual(target.read_bytes(), (output / "urdf_final.urdf").read_bytes())
            nominal = installer.nominal_backup_path(target)
            self.assertEqual(nominal.read_bytes(), self.original)
            self.assertEqual(installer.resolve_fit_source(target), nominal)
            self.assertEqual(len(list(root.glob("urdf.bak.*"))), 1)

    def test_unsuccessful_fit_never_exports_or_prompts(self):
        for failed_stage in ("fit", "all"):
            with self.subTest(stage=failed_stage), tempfile.TemporaryDirectory() as temp, \
                    redirect_stdout(io.StringIO()):
                root = Path(temp)
                config, capture, output, target, _, cfg = self.main_inputs(root)
                p = np.zeros(self.inputs.models[self.optimizer].parameter_count)
                success = SimpleNamespace(success=True, message="test", nfev=1)
                failure = SimpleNamespace(success=False, message="max evaluations", nfev=250)
                fits = [(p, failure)] if failed_stage == "fit" else [(p, success), (p, failure)]
                args = [self.optimizer.__file__, str(capture), "--config", str(config), "--output-urdf", "--force-output"]
                before = self.snapshot(root)
                with patch.dict(os.environ, ROBOT_VERSION="63"), patch.object(sys, "argv", args), \
                        patch.object(self.optimizer, "_config_for_capture", return_value=copy.deepcopy(cfg)), \
                        patch.object(self.optimizer, "fit", side_effect=fits) as fit, \
                        patch("builtins.input", side_effect=AssertionError("unexpected prompt")) as prompt:
                    with self.assertRaises(SystemExit):
                        self.optimizer.main()
                self.assertEqual(fit.call_count, len(fits))
                prompt.assert_not_called()
                self.assertFalse(output.exists())
                self.assertEqual(self.snapshot(root), before)
                self.assertEqual(target.read_bytes(), self.original)

    def test_forced_export_into_runtime_drake_directory_is_rejected_before_writing(self):
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)
            config, capture, _, target, _, cfg = self.main_inputs(root)
            cfg["kinematics"]["output_dir"] = str(target.parent / "drake")
            cfg["kinematics"]["urdf_final_name"] = "biped_v3_arm.urdf"
            config.write_text(yaml.safe_dump(cfg))
            before = self.snapshot(root)
            p = np.zeros(self.inputs.models[self.optimizer].parameter_count)
            success = SimpleNamespace(success=True, message="test", nfev=1)
            args = [self.optimizer.__file__, str(capture), "--config", str(config), "--output-urdf", "--force-output"]
            with patch.dict(os.environ, ROBOT_VERSION="63"), patch.object(sys, "argv", args), \
                    patch.object(self.optimizer, "_config_for_capture", return_value=copy.deepcopy(cfg)), \
                    patch.object(self.optimizer, "fit", return_value=(p, success)), \
                    patch("builtins.input", side_effect=AssertionError("unexpected prompt")) as prompt:
                with self.assertRaisesRegex(SystemExit, "运行模型目录之外"):
                    self.optimizer.main()
            prompt.assert_not_called()
            self.assertEqual(self.snapshot(root), before)
            self.assertEqual(list(root.glob("urdf.bak.*")), [])


if __name__ == "__main__":
    unittest.main()
