#!/usr/bin/env python3
"""H12 udev 部署脚本安全测试，使用伪设备且不执行真实 sudo。"""

import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

NODE_DIR = Path(__file__).resolve().parents[1]
LOADER = NODE_DIR / "scripts" / "load_h12_log_serial_rule.sh"
DEPLOY = NODE_DIR / "scripts" / "deploy_autostart.sh"
RULE = NODE_DIR / "scripts" / "udev" / "H12_log_serial.rules"
REMOTE_RULE_LOADER = NODE_DIR / "lib" / "kuavo_remote" / "creat_remote_udev_rule.sh"


class H12UdevStaticTest(unittest.TestCase):
    def test_rule_has_no_stty_and_is_add_only(self):
        rule = RULE.read_text()
        self.assertIn('ACTION=="add"', rule)
        self.assertIn('ATTRS{product}=="CP2102 USB to UART Bridge Controller"', rule)
        self.assertNotIn("RUN", rule)
        self.assertNotIn("stty", rule)

    def test_no_global_udev_trigger_in_h12_deployment_path(self):
        trigger = re.compile(r"(?m)^\s*(?:sudo\s+)?udevadm\s+trigger(?:\s|$)")
        for script in (LOADER, DEPLOY, REMOTE_RULE_LOADER):
            self.assertIsNone(trigger.search(script.read_text()), str(script))

    def test_deploy_cleans_before_y_install_and_on_n(self):
        deploy = DEPLOY.read_text()
        self.assertIn("cleanup_h12_log_rule()", deploy)
        self.assertIn('sudo rm -f "$rule_target"', deploy)
        self.assertIn('sudo rm -f "$log_link"', deploy)
        self.assertIn("n=禁用并删除旧规则", deploy)
        self.assertNotIn("遥控器设备文件已存在，无需加载规则", deploy)


class H12LoaderDetectionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.fake_bin = self.base / "bin"
        self.fake_bin.mkdir()
        self.sudo_marker = self.base / "sudo-called"
        (self.fake_bin / "sudo").write_text(
            "#!/bin/bash\n"
            "printf 'unexpected sudo\\n' >> \"$SUDO_MARKER\"\n"
            "exit 99\n"
        )
        (self.fake_bin / "udevadm").write_text(
            "#!/usr/bin/env python3\n"
            "import sys\n"
            "dev = sys.argv[3].split('=', 1)[1]\n"
            "with open(dev + '.props') as stream:\n"
            "    print(stream.read(), end='')\n"
        )
        for executable in (self.fake_bin / "sudo", self.fake_bin / "udevadm"):
            executable.chmod(0o755)

    def tearDown(self):
        self.temp.cleanup()

    def add_device(self, name, model, vendor="10c4", serial="SERIAL"):
        device = self.base / name
        device.touch()
        Path(str(device) + ".props").write_text(
            "DEVNAME={}\nID_VENDOR_ID={}\nID_MODEL={}\nID_SERIAL_SHORT={}\n".format(
                device, vendor, model, serial
            )
        )

    def run_loader(self):
        env = os.environ.copy()
        env["PATH"] = str(self.fake_bin) + os.pathsep + env["PATH"]
        env["H12_TTY_GLOB"] = str(self.base / "ttyUSB*")
        env["SUDO_MARKER"] = str(self.sudo_marker)
        result = subprocess.run(
            [str(LOADER), "--detect-only"],
            capture_output=True,
            text=True,
            env=env,
            timeout=10,
            check=False,
        )
        self.assertFalse(self.sudo_marker.exists(), "detect-only 不得调用 sudo")
        return result

    def test_only_hipnuc_cp2102n_is_rejected(self):
        self.add_device("ttyUSB2", "CP2102N_USB_to_UART_Bridge_Controller")
        result = self.run_loader()
        self.assertEqual(result.returncode, 1)
        self.assertIn("拒绝安装规则", result.stdout)
        self.assertNotIn("DETECT_OK", result.stdout)

    def test_single_cp2102_is_accepted_without_writes(self):
        self.add_device("ttyUSB3", "CP2102_USB_to_UART_Bridge_Controller", serial="H12-1")
        result = self.run_loader()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("DETECT_OK", result.stdout)
        self.assertIn("H12-1", result.stdout)

    def test_cp2102n_is_excluded_when_h12_is_present(self):
        self.add_device("ttyUSB2", "CP2102N_USB_to_UART_Bridge_Controller")
        self.add_device("ttyUSB3", "CP2102_USB_to_UART_Bridge_Controller")
        result = self.run_loader()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("ttyUSB3", result.stdout)
        self.assertNotIn("DETECT_OK {}\n".format(self.base / "ttyUSB2"), result.stdout)

    def test_multiple_h12_candidates_are_rejected(self):
        self.add_device("ttyUSB3", "CP2102_USB_to_UART_Bridge_Controller")
        self.add_device("ttyUSB4", "CP2102_USB_to_UART_Bridge_Controller")
        result = self.run_loader()
        self.assertEqual(result.returncode, 1)
        self.assertIn("多个普通 CP2102", result.stdout)

    def test_other_vendor_is_rejected(self):
        self.add_device("ttyUSB0", "USB2.0-Serial", vendor="1a86")
        result = self.run_loader()
        self.assertEqual(result.returncode, 1)


if __name__ == "__main__":
    unittest.main()
