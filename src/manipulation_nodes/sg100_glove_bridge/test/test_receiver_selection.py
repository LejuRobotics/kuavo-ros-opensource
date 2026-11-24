"""Offline tests for safe, machine-independent SG100 receiver selection."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import types
import unittest
from unittest import mock


def load_node():
    for name, module in (("rospy", {}), ("rospkg", {}),
                         ("std_msgs", {}), ("std_msgs.msg", {"Bool": object, "Float32MultiArray": object})):
        if name not in sys.modules:
            stub = types.ModuleType(name)
            for key, value in module.items():
                setattr(stub, key, value)
            sys.modules[name] = stub
    path = Path(__file__).resolve().parents[1] / "scripts/sg100_glove_node.py"
    spec = importlib.util.spec_from_file_location("sg100_glove_node_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


node = load_node()


class ReceiverSelectionTest(unittest.TestCase):
    def test_candidate_order_uses_verified_alias_not_tty_number(self):
        identities = {"/dev/ttyUSB3": {"vid": 0x1A86, "pid": 0x7523, "product": "USB Serial"},
                      "/dev/ttyUSB4": {"vid": 0x1A86, "pid": 0x7523, "product": "USB Serial"}}
        with mock.patch.object(node.os.path, "islink", return_value=True), \
                mock.patch.object(node.os.path, "realpath", side_effect=lambda p: "/dev/ttyUSB3" if "glove_receiver" in p else p), \
                mock.patch.object(node.os, "listdir", return_value=["ttyUSB4", "ttyUSB3"]), \
                mock.patch.object(node, "_port_identity", side_effect=lambda p: identities[p]), \
                mock.patch.object(node, "_reserved_devices", return_value=set()):
            self.assertEqual(node._candidate_ports(), ["/dev/ttyUSB3", "/dev/ttyUSB4"])

    def test_known_hands_remote_and_can_are_never_candidates(self):
        identities = {
            "/dev/ttyUSB0": {"vid": 0x0403, "pid": 0x6001, "serial": "A10RA5IA"},
            "/dev/ttyUSB1": {"vid": 0x0403, "pid": 0x6001, "serial": "A10RA5IC"},
            "/dev/ttyUSB2": {"vid": 0x0403, "pid": 0x6001, "serial": "A10RABMP", "product": "LJREMOTE"},
            "/dev/ttyUSB9": {"vid": 0x1A86, "pid": 0x7523, "serial": "", "product": "USB Serial"},
            "/dev/ttyACM0": {"vid": 0x16D0, "pid": 0x117E, "serial": "CAN1"},
        }
        with mock.patch.object(node.os.path, "islink", return_value=False), \
                mock.patch.object(node.os, "listdir", side_effect=lambda path: [name.rsplit("/", 1)[-1] for name in identities] if path == "/sys/class/tty" else []), \
                mock.patch.object(node.os.path, "exists", return_value=False), \
                mock.patch.object(node, "_port_identity", side_effect=lambda p: identities[p]):
            self.assertEqual(node._candidate_ports(), ["/dev/ttyUSB9"])
            self.assertTrue({"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyACM0"}.issubset(node._reserved_devices()))

    def test_shared_chip_power_and_remote_aliases_are_excluded(self):
        aliases = {"/dev/power_board": "/dev/ttyUSB0", "/dev/usb_remote": "/dev/ttyUSB1"}
        with mock.patch.object(node.os.path, "exists", side_effect=aliases.__contains__), \
                mock.patch.object(node.os.path, "realpath", side_effect=lambda p: aliases.get(p, p)), \
                mock.patch.object(node.os, "listdir", return_value=["ttyUSB0", "ttyUSB1", "ttyUSB2"]), \
                mock.patch.object(node, "_port_identity", return_value={"vid": 0x1A86, "pid": 0x7523}):
            self.assertEqual(node._candidate_ports(), ["/dev/ttyUSB2"])

    def test_only_candidates_are_scanned_and_no_target_means_no_choice(self):
        completed = subprocess.CompletedProcess([], 0, json.dumps({"chosen": None, "reports": [{"device": "/dev/ttyUSB9", "ids": []}]}), "")
        with mock.patch.object(node, "_candidate_ports", return_value=["/dev/ttyUSB9"]), \
                mock.patch.object(node, "_reserved_devices", return_value={"/dev/ttyUSB0"}), \
                mock.patch.object(node.subprocess, "run", return_value=completed) as run:
            result = node._select_receiver("python", "/runtime", "auto", 921600, {2, 12})
        self.assertIsNone(result["chosen"])
        argv = run.call_args.args[0]
        self.assertIn("/dev/ttyUSB9", argv)
        self.assertNotIn("/dev/ttyUSB0", argv)
        self.assertNotIn("auto", argv)

    def test_verified_target_boards_choose_receiver(self):
        completed = subprocess.CompletedProcess([], 0, json.dumps({"chosen": "/dev/ttyUSB9", "reports": [{"device": "/dev/ttyUSB9", "ids": [2, 12]}]}), "")
        with mock.patch.object(node, "_candidate_ports", return_value=["/dev/ttyUSB9"]), \
                mock.patch.object(node, "_reserved_devices", return_value=set()), \
                mock.patch.object(node.subprocess, "run", return_value=completed):
            self.assertEqual(node._select_receiver("python", "/runtime", "", 921600, {2, 12})["chosen"], "/dev/ttyUSB9")


    def test_configured_left_id_is_not_overwritten_by_right_id(self):
        # Execute the production constructor through its hand-ID setup, without
        # constructing ROS publishers or touching a serial port.
        source = Path(node.__file__).read_text()
        tree = ast.parse(source)
        klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SG100GloveNode")
        init = next(n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
        cutoff = next(i for i, n in enumerate(init.body)
                      if isinstance(n, ast.Assign) and any(isinstance(t, ast.Attribute) and t.attr == "command_topic" for t in n.targets))
        init.body = init.body[:cutoff]
        namespace = {"rospy": types.SimpleNamespace(), "SUPPORTED_SIDES": node.SUPPORTED_SIDES,
                     "_board_id": node._board_id}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[init], type_ignores=[])), str(node.__file__), "exec"), namespace)
        for side, expected in (("left", {"left": 12}), ("right", {"right": 2}),
                               ("both", {"left": 12, "right": 2})):
            params = {"~side": side, "~auto_detect": True, "~left_id": 12, "~right_id": 2}
            namespace["rospy"].get_param = lambda name, default=None: params.get(name, default)
            obj = node.SG100GloveNode.__new__(node.SG100GloveNode)
            namespace["__init__"](obj)
            self.assertEqual(obj.hand_board_ids, expected)

    def test_non_integral_and_out_of_range_ids_are_rejected(self):
        for bad_id in (True, -1, 2.5, "12.0", 0x100000000):
            with self.subTest(board_id=bad_id), self.assertRaisesRegex(ValueError, "~left_id"):
                node._board_id(bad_id, "~left_id")

    def test_preferred_port_does_not_restrict_search_or_bypass_allowlist(self):
        devices = ["/dev/ttyUSB2", "/dev/ttyUSB3"]
        completed = subprocess.CompletedProcess([], 0, '{"chosen": null}', "")
        with mock.patch.object(node, "_candidate_ports", return_value=devices), \
                mock.patch.object(node.subprocess, "run", return_value=completed) as run:
            node._select_receiver("python", "/runtime", "/dev/ttyUSB3", 921600, {2, 12})
            self.assertEqual(run.call_args.args[0][-2:], ["/dev/ttyUSB3", "/dev/ttyUSB2"])
            self.assertEqual(run.call_args.kwargs["timeout"], 19.0)
            node._select_receiver("python", "/runtime", "/dev/ttyUSB9", 921600, {2, 12})
            self.assertEqual(run.call_args.args[0][-2:], devices)
            node._select_receiver("python", "/runtime", "/dev/ttyUSB3", 921600, {2, 12},
                                  allowed_ports="/dev/ttyUSB2")
            self.assertEqual(run.call_args.args[0][8:], ["/dev/ttyUSB2"])

    def test_fixed_port_does_not_fall_back_or_bypass_validation(self):
        devices = ["/dev/ttyUSB2", "/dev/ttyUSB3"]
        completed = subprocess.CompletedProcess([], 0, '{"chosen": null}', "")
        with mock.patch.object(node, "_candidate_ports", return_value=devices), \
                mock.patch.object(node.subprocess, "run", return_value=completed) as run:
            self.assertIsNone(node._select_receiver("python", "/runtime", devices[1],
                                                   921600, {2, 12}, fixed_port=True)["chosen"])
            self.assertEqual(run.call_args.args[0][8:], [devices[1]])
            run.reset_mock()
            for path in ("/dev/ttyUSB9", "/etc/passwd"):
                with self.subTest(path=path), self.assertRaises(ValueError):
                    node._select_receiver("python", "/runtime", path, 921600, {2, 12},
                                          fixed_port=True)
            run.assert_not_called()

    def test_auto_detection_rescans_all_candidates_after_port_number_changes(self):
        obj = node.SG100GloveNode.__new__(node.SG100GloveNode)
        obj.auto_detect = True
        obj.port = ""
        obj.requested_port = ""
        obj.allowed_ports = ""
        obj.hand_board_ids = {"left": 12, "right": 2}
        obj.sdk_python = "python"
        obj.runtime_dir = "/runtime"
        obj.baud = 921600
        fake_ros = types.SimpleNamespace(logwarn=mock.Mock(), logerr_throttle=mock.Mock(),
                                         logwarn_throttle=mock.Mock())
        choices = [{"chosen": "/dev/ttyUSB3", "reports": []},
                   {"chosen": "/dev/ttyUSB7", "reports": []}]
        with mock.patch.object(node, "rospy", fake_ros), \
                mock.patch.object(node, "_select_receiver", side_effect=choices) as select:
            self.assertTrue(obj._use_receiver_with_target_boards())
            self.assertTrue(obj._use_receiver_with_target_boards())
        self.assertEqual(obj.port, "/dev/ttyUSB7")
        self.assertEqual([call.args[2] for call in select.call_args_list], ["", ""])

    def scan(self, responses, preferred="", wanted="2,12"):
        def scan(device, baud, seconds):
            return {"boards": [{"board_id": board} for board in responses[device]]}
        connection = types.ModuleType("sg100_native.connection")
        connection.scan_boards = scan
        namespace = {"__name__": "__main__"}
        argv = ["scan", "/runtime", "921600", "2", wanted, preferred, *responses]
        with mock.patch.dict(sys.modules, {"sg100_native": types.ModuleType("sg100_native"),
                                           "sg100_native.connection": connection}), \
                mock.patch.object(sys, "argv", argv), \
                mock.patch("builtins.print"), mock.patch.object(sys, "path", list(sys.path)):
            exec(node._RECEIVER_SCAN_SCRIPT, namespace)
        return namespace["chosen"], namespace["matches"]

    def test_scanner_requires_both_target_ids_on_the_same_port(self):
        self.assertEqual(self.scan({"/dev/ttyUSB3": [12], "/dev/ttyUSB7": [2]}), (None, []))
        self.assertEqual(self.scan({"/dev/ttyUSB3": [12], "/dev/ttyUSB7": [12, 2]}),
                         ("/dev/ttyUSB7", ["/dev/ttyUSB7"]))

    def test_single_hand_scans_for_only_its_configured_id(self):
        for wanted, other in (("12", 2), ("2", 12)):
            with self.subTest(wanted=wanted):
                self.assertEqual(self.scan({"/dev/ttyUSB3": [other]}, wanted=wanted),
                                 (None, []))
                self.assertEqual(self.scan({"/dev/ttyUSB3": [int(wanted)]}, wanted=wanted),
                                 ("/dev/ttyUSB3", ["/dev/ttyUSB3"]))

    def test_ambiguous_detection_requires_an_explicit_verified_preference(self):
        responses = {"/dev/ttyUSB3": [12, 2], "/dev/ttyUSB7": [12, 2]}
        self.assertEqual(self.scan(responses), (None, list(responses)))
        self.assertEqual(self.scan(responses, "/dev/ttyUSB3"),
                         ("/dev/ttyUSB3", ["/dev/ttyUSB3"]))

    def test_sdk_reconnects_via_bridge(self):
        obj = node.SG100GloveNode.__new__(node.SG100GloveNode)
        obj.port, obj.baud, obj.side, obj.launcher = "/dev/ttyUSB2", 921600, "both", "/sdk/run"
        obj.left_id, obj.right_id, obj.emit_gesture_poses = 12, 2, True
        argv = obj._sdk_argv()
        self.assertIn("--no-reconnect", argv)
        self.assertEqual(argv[argv.index("--serial") + 1], "/dev/ttyUSB2")


if __name__ == "__main__":
    unittest.main()
