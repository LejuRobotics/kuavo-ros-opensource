#!/usr/bin/env python3
"""H12 日志串口身份校验测试，不访问真实硬件。"""

import subprocess
import unittest
from unittest.mock import Mock, patch

from utils.h12_log_serial import get_h12_log_channel_error, is_valid_h12_log_channel


def udev_result(properties, returncode=0, stderr=""):
    stdout = "\n".join("{}={}".format(key, value) for key, value in properties.items())
    return Mock(returncode=returncode, stdout=stdout, stderr=stderr)


class H12LogSerialValidationTest(unittest.TestCase):
    def test_missing_or_non_symlink_is_rejected(self):
        with patch("os.path.islink", return_value=False):
            self.assertIn("不是 udev 创建的软链接", get_h12_log_channel_error())

    @patch("os.path.islink", return_value=True)
    @patch("subprocess.run")
    def test_expected_cp2102_is_accepted(self, run, _islink):
        run.return_value = udev_result({
            "DEVNAME": "/dev/ttyUSB3",
            "ID_VENDOR_ID": "10c4",
            "ID_MODEL": "CP2102_USB_to_UART_Bridge_Controller",
            "ID_SERIAL_SHORT": "H12-SERIAL",
        })
        with patch("os.path.realpath", side_effect=lambda path: {
            "/dev/H12_log_channel": "/dev/ttyUSB3",
            "/dev/ttyUSB3": "/dev/ttyUSB3",
        }.get(path, path)):
            self.assertEqual(get_h12_log_channel_error(), "")
            self.assertTrue(is_valid_h12_log_channel())

    @patch("os.path.islink", return_value=True)
    @patch("subprocess.run")
    def test_hipnuc_cp2102n_is_rejected(self, run, _islink):
        run.return_value = udev_result({
            "DEVNAME": "/dev/ttyUSB2",
            "ID_VENDOR_ID": "10c4",
            "ID_MODEL": "CP2102N_USB_to_UART_Bridge_Controller",
        })
        with patch("os.path.realpath", side_effect=lambda path: path):
            self.assertIn("拒绝打开以避免误用 IMU CP2102N", get_h12_log_channel_error())

    @patch("os.path.islink", return_value=True)
    @patch("subprocess.run")
    def test_wrong_vendor_is_rejected(self, run, _islink):
        run.return_value = udev_result({"ID_VENDOR_ID": "1a86", "ID_MODEL": "USB2.0-Serial"})
        self.assertIn("USB 厂商不是", get_h12_log_channel_error())

    @patch("os.path.islink", return_value=True)
    @patch("subprocess.run")
    def test_udev_failure_is_rejected(self, run, _islink):
        run.return_value = udev_result({}, returncode=1, stderr="unknown device")
        self.assertIn("unknown device", get_h12_log_channel_error())

    @patch("os.path.islink", return_value=True)
    @patch("subprocess.run", side_effect=subprocess.TimeoutExpired(cmd="udevadm", timeout=2))
    def test_timeout_is_rejected(self, _run, _islink):
        self.assertIn("udev 属性失败", get_h12_log_channel_error())

    @patch("os.path.islink", return_value=True)
    @patch("subprocess.run")
    def test_target_mismatch_is_rejected(self, run, _islink):
        run.return_value = udev_result({
            "DEVNAME": "/dev/ttyUSB2",
            "ID_VENDOR_ID": "10c4",
            "ID_MODEL": "CP2102_USB_to_UART_Bridge_Controller",
        })
        with patch("os.path.realpath", side_effect=lambda path: {
            "/dev/H12_log_channel": "/dev/ttyUSB3",
            "/dev/ttyUSB2": "/dev/ttyUSB2",
        }.get(path, path)):
            self.assertIn("目标与软链接不一致", get_h12_log_channel_error())


if __name__ == "__main__":
    unittest.main()
