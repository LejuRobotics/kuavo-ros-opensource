#!/usr/bin/env python3
"""Parse ROS launch contracts only; never start nodes or hardware."""
import os
from pathlib import Path
import unittest
from unittest.mock import patch
import roslaunch

PACKAGE = Path(__file__).resolve().parents[1]


class LaunchModelTest(unittest.TestCase):
    def load(self, file, args):
        return roslaunch.config.load_config_default(
            [(str(PACKAGE / "launch" / file), args)], None, verbose=False)


    def test_automatic_end_effector_uses_config_resolver(self):
        import json
        import shlex
        import subprocess
        for version in (62, 63, 400062):
            config = PACKAGE.parent / "kuavo_assets" / "config" / ("kuavo_v%d" % version) / "kuavo.json"
            expected = json.loads(config.read_text())["EndEffectorType"][0]
            for entry in ("quest_velocity_ik.launch", "pico_velocity_ik.launch"):
                for kind in ("auto", "qiangnao", "heiman", "lejuclaw", "HEIMAN"):
                    with self.subTest(version=version, entry=entry, kind=kind):
                        args = ["robot_version:=" + str(version)]
                        if kind != "auto":
                            args.append("ee_type:=" + kind)
                        cfg = self.load(entry, args)
                        mapper = next(n for n in cfg.nodes if n.name == "ik_ros_uni")
                        self.assertIn("--ee_type=" + kind, mapper.args)
                        bridge = [n for n in cfg.nodes if n.name == "sg100_mapper_bridge"]
                        self.assertTrue(bridge)  # Always resolve before selecting the bridge.
                        if bridge:
                            self.assertEqual(bridge[0].type, "absolute_end_effector.py")
                            command = shlex.split(bridge[0].args)
                            command.remove("--bridge")  # Read-only resolver: never launch a bridge in a test.
                            result = subprocess.run(["/usr/bin/python3",
                                str(PACKAGE/"scripts/absolute_end_effector.py")] + command,
                                capture_output=True, text=True, check=True)
                            self.assertEqual(result.stdout.strip(), expected if kind == "auto" else kind.lower())

    def test_saved_model_without_version_argument(self):
        entries = (
            ("load_kuavo_real_velocity_ik.launch", []),
            ("load_kuavo_mujoco_velocity_ik.launch", []),
            ("quest_velocity_ik.launch", ["ip_address:=127.0.0.1"]),
            ("pico_velocity_ik.launch", []),
            ("control.launch", ["controller_type:=wheel_arm_abs_vel", "real:=true"]),
            ("control.launch", ["controller_type:=wheel_arm_abs_vel", "real:=false"]),
        )
        for version in (62, 63, 400062):
            for entry, args in entries:
                with self.subTest(version=version, entry=entry, args=args), \
                        patch.dict(os.environ, ROBOT_VERSION=str(version)):
                    cfg = self.load(entry, args)
                    versions = [int(p.value) for key, p in cfg.params.items()
                                if key.endswith("/robot_version")]
                    self.assertTrue(versions)
                    self.assertEqual(set(versions), {version})
        with patch.dict(os.environ):
            os.environ.pop("ROBOT_VERSION", None)
            with self.assertRaises(Exception):
                self.load("load_kuavo_real_velocity_ik.launch", [])

    def test_real_manager_version_overrides_missing_or_stale_environment(self):
        for version in (62, 63, 400062):
            for inherited in (None, "", "40", "invalid"):
                for entry, extra in (
                    ("load_kuavo_real_velocity_ik.launch", []),
                    ("control.launch", ["controller_type:=wheel_arm_abs_vel", "real:=true"]),
                ):
                    with self.subTest(version=version, inherited=inherited, entry=entry), \
                            patch.dict(os.environ):
                        os.environ.pop("ROBOT_VERSION", None)
                        if inherited is not None:
                            os.environ["ROBOT_VERSION"] = inherited
                        cfg = self.load(entry, extra + ["robot_version:=" + str(version)])
                        managers = [n for n in cfg.nodes if n.name == "nodelet_manager"]
                        self.assertEqual(len(managers), 1)
                        self.assertEqual(managers[0].args, "manager")
                        self.assertEqual(dict(managers[0].env_args)["ROBOT_VERSION"], str(version))
                        self.assertEqual(int(cfg.params["/robot_version"].value), version)

    def test_s62_and_s63_entries(self):
        for version in (62, 63, 400062):
            for file, extra in (
                ("load_kuavo_real_velocity_ik.launch", []),
                ("load_kuavo_mujoco_velocity_ik.launch", []),
                ("quest_velocity_ik.launch", ["ip_address:=127.0.0.1"]),
                ("pico_velocity_ik.launch", []),
                ("control.launch", ["controller_type:=wheel_arm_abs_vel"]),
                ("control.launch", ["controller_type:=wheel_arm_abs_vel", "real:=false"]),
            ):
                with self.subTest(version=version, file=file, extra=extra):
                    cfg = self.load(file, extra + ["robot_version:=" + str(version)])
                    params = {k: v.value for k, v in cfg.params.items()}
                    if version == 63:
                        self.assertFalse(any(n.type == "arms_ik_node" for n in cfg.nodes))
                        # The unchanged shared kuavo_configuration JSON also contains
                        # unused legacy model metadata; inspect actual path parameters.
                        self.assertFalse(any(isinstance(v, str) and v.endswith(".urdf")
                                             and "/urdf/drake/" in v for v in params.values()))
                    if "/urdfFile" in params:
                        self.assertTrue(params["/urdfFile"].endswith(
                            "/biped_s{0}/urdf/biped_s{0}.urdf".format(version)))
                        self.assertIn("/kuavo_s" + str(version) + "/", params["/taskFile"])
                    if file in ("quest_velocity_ik.launch", "pico_velocity_ik.launch"):
                        prefix = "/humanoid_quest_control_with_arm/"
                        self.assertEqual(params[prefix + "robot_version"], version)
                        self.assertEqual(params[prefix + "robot_type"], 1)
                        self.assertTrue(params[prefix + "referenceFile"].endswith(
                            "/config/kuavo_v%d/command/reference.info" % version))
                        self.assertTrue(params[prefix + "gaitCommandFile"].endswith(
                            "/config/kuavo_v%d/command/gait.info" % version))
                        fsm = next(n for n in cfg.nodes if n.name == "humanoid_quest_control_with_arm")
                        self.assertIn("bash -o pipefail", fsm.launch_prefix)
                        self.assertTrue(any(k.endswith("/robot_version") and v == version
                                            for k, v in params.items()))
                        self.assertTrue(any(k.endswith("/velocity_tcp_geometry_id")
                                            and v.startswith("V" + str(version) + "_")
                                            for k, v in params.items()))
                        for node in cfg.nodes:
                            if node.name == 'robot_state_server':
                                self.assertIn('/scripts/sidecar_runtime.py', node.launch_prefix)
                    if "/j3_overshoot_profile/enabled" in params:
                        self.assertEqual(params["/j3_overshoot_profile/enabled"], version in (62, 63))

    def test_strict_and_driver_settings_reach_nested_runtime_nodes(self):
        for file, extra in (
            ("control.launch", ["controller_type:=wheel_arm_abs_vel"]),
            ("pico_velocity_ik.launch", []),
            ("quest_velocity_ik.launch", [])):
            cfg = self.load(file, extra + ["robot_version:=63",
                "require_ik_hard_isolation:=true", "arm_driver_reserved_cpus:=5-7"])
            nodes = [node for node in cfg.nodes if node.launch_prefix
                     and "whole_body_ik_runtime.sh" in node.launch_prefix]
            self.assertTrue(nodes)
            for node in nodes:
                env = dict(node.env_args)
                self.assertEqual(env["KUAVO_REQUIRE_IK_HARD_ISOLATION"].lower(), "true")
                self.assertEqual(env["KUAVO_ARM_DRIVER_RESERVED_CPUS"], "5-7")

    def test_absolute_entries_do_not_write_shared_hardware_switches(self):
        for version in (62, 63, 400062):
            for entry in ("load_kuavo_real_velocity_ik.launch",
                          "quest_velocity_ik.launch", "pico_velocity_ik.launch"):
                cfg = self.load(entry, ["robot_version:=" + str(version)])
                params = {key: value.value for key, value in cfg.params.items()}
                # Existing shared startup readiness flags are part of dev's lifecycle.
                hardware_keys = {key for key in params if key.startswith("/hardware/")}
                self.assertLessEqual(hardware_keys, {"/hardware/is_ready", "/hardware/ready_to_start"})
        cfg = self.load("bypass_communication_profile.launch", ["full_rate_audit:=true"])
        params = {key: value.value for key, value in cfg.params.items()}
        self.assertFalse(any(key.startswith("/hardware/") for key in params))
        self.assertEqual(params["/absolute_velocity_ik/runtime_probe_detail_stride"], 1)

    def test_legacy_selector_matches_original_launch_without_absolute_environment(self):
        import rospkg
        old = Path(rospkg.RosPack().get_path("humanoid_controllers")) / "launch"
        for real, filename in ((True, "load_kuavo_real_wheel.launch"),
                               (False, "load_kuavo_mujoco_sim_wheel.launch")):
            args = ["robot_version:=62"]
            direct = roslaunch.config.load_config_default(
                [(str(old / filename), args)], None, verbose=False)
            selected = self.load("control.launch", args + [
                "controller_type:=legacy", "real:=" + str(real).lower()])
            self.assertEqual({k: v.value for k, v in direct.params.items()},
                             {k: v.value for k, v in selected.params.items()})
            fields = ("name", "namespace", "package", "type", "args",
                      "launch_prefix", "remap_args", "env_args", "required", "respawn")
            def nodes(cfg):
                return [tuple(getattr(n, field) for field in fields) for n in cfg.nodes]
            self.assertEqual(nodes(direct), nodes(selected))

    def test_unsupported_versions_and_modes_rejected(self):
        for version in (61, 64):
            for file in ("load_kuavo_real_velocity_ik.launch",
                         "quest_velocity_ik.launch", "load_kuavo_mujoco_velocity_ik.launch"):
                with self.subTest(version=version, file=file), self.assertRaises(Exception):
                    self.load(file, ["robot_version:=" + str(version)])
        with self.assertRaises(Exception):
            self.load("load_kuavo_real_velocity_ik.launch",
                      ["robot_version:=63", "with_ik:=true"])
        for args in (["wheel_ik:=false"], ["use_cpp_incremental_ik:=true"],
                     ["enable_velocity_ik_takeover:=false"], ["use_arm_only_ik:=true"]):
            with self.subTest(args=args), self.assertRaises(Exception):
                self.load("quest_velocity_ik.launch", ["robot_version:=63"] + args)


if __name__ == "__main__":
    unittest.main()
