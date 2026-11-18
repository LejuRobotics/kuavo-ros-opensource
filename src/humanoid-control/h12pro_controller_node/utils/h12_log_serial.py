"""H12 日志串口身份校验，避免误开 HIPNUC CP2102N。"""

import os
import subprocess

H12_LOG_CHANNEL = "/dev/H12_log_channel"
H12_LOG_VENDOR_ID = "10c4"
H12_LOG_MODEL = "CP2102_USB_to_UART_Bridge_Controller"


def get_h12_log_channel_error(device=H12_LOG_CHANNEL):
    """返回空字符串表示可安全打开，否则返回拒绝原因。"""
    if not os.path.islink(device):
        return "{} 不存在或不是 udev 创建的软链接".format(device)

    try:
        result = subprocess.run(
            ["udevadm", "info", "--query=property", "--name={}".format(device)],
            capture_output=True,
            text=True,
            timeout=2,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return "读取 {} 的 udev 属性失败：{}".format(device, exc)

    if result.returncode != 0:
        detail = result.stderr.strip() or "udevadm 返回 {}".format(result.returncode)
        return "读取 {} 的 udev 属性失败：{}".format(device, detail)

    properties = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            properties[key] = value

    vendor = properties.get("ID_VENDOR_ID", "")
    model = properties.get("ID_MODEL", "")
    if vendor != H12_LOG_VENDOR_ID:
        return "{} 的 USB 厂商不是 H12 日志转接板：{}".format(device, vendor or "未知")
    if model != H12_LOG_MODEL:
        return "{} 的型号不是独立 H12 CP2102：{}；拒绝打开以避免误用 IMU CP2102N".format(
            device, model or "未知"
        )

    reported_device = properties.get("DEVNAME", "")
    if reported_device and os.path.realpath(reported_device) != os.path.realpath(device):
        return "{} 的 udev 目标与软链接不一致".format(device)

    return ""


def is_valid_h12_log_channel(device=H12_LOG_CHANNEL):
    """仅在设备身份明确符合 H12 日志转接板时返回 True。"""
    return get_h12_log_channel_error(device) == ""
