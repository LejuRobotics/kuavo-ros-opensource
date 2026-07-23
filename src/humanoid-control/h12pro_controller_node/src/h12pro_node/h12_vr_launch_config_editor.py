#!/usr/bin/env python3
"""H12/G12 启动 VR 配置（h12_vr_launch.yaml）的交互式编辑脚本。

以包内默认为基底、叠加用户目录现有配置作为起始值，逐字段命令行交互修改，
保存前复用 h12_vr_launch_config.validate_config 校验，最终写入
~/.config/lejuconfig/h12_vr_launch.yaml。
"""

import os
import sys

# 将包根目录加入 sys.path，使脚本脱离完整 ROS 环境直接运行时也能导入 utils
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from utils.h12_vr_launch_config import (  # noqa: E402
    BOOL_FIELDS,
    DEFAULT_CONFIG_PATH,
    USER_CONFIG_PATH,
    H12VrLaunchConfigError,
    _load_yaml,
    validate_config,
)

# 字段顺序（ip_address 在前，其余为 BOOL_FIELDS）与各字段用途说明，
# 用于编辑前的表格展示，让现场先看清每个开关干嘛的再逐项改。
_FIELD_ORDER = ("ip_address",) + BOOL_FIELDS
_FIELD_DESC = {
    "ip_address": "VR 设备 IP，留空走 launch 默认（广播发现）",
    "use_cpp_incremental_ik": "位置遥操是否用增量模式",
    "use_incremental_hand_orientation": "姿态遥操是否用增量模式（位置非增量时不能开）",
    "control_torso": "是否启用控腰，false 可避免 VR 重连腰部猛转",
    "enable_vr_stop_robot": "VR 手柄 X+Y 是否触发急停，false 后仅遥控器可关机",
}

# 用户文件内容模板：保存后仍自带字段说明，便于现场手动核对
_USER_FILE_TEMPLATE = """\
# H12/G12 启动 VR 遥操配置（由 h12_vr_launch_config_editor.py 生成）。
# 缺省字段沿用包内默认 config/h12_vr_launch.yaml。

# VR 设备 IP，留空则不注入、走 launch 文件默认
ip_address: "{ip_address}"

# 遥操形式：位置 / 姿态是否使用增量模式
# (use_cpp_incremental_ik=false, use_incremental_hand_orientation=true) 为无效组合
use_cpp_incremental_ik: {use_cpp_incremental_ik}
use_incremental_hand_orientation: {use_incremental_hand_orientation}

# 是否启用控腰（false 可避免 VR 重连时腰部猛转）
control_torso: {control_torso}

# VR 手柄 X+Y 是否允许触发 stop_robot（false 后仅遥控器可关机）
enable_vr_stop_robot: {enable_vr_stop_robot}
"""


def _load_start_config():
    """包内默认为基底，叠加用户目录现有配置（不校验，便于修复非法文件）。"""
    config = _load_yaml(DEFAULT_CONFIG_PATH)
    if os.path.exists(USER_CONFIG_PATH):
        user_config = _load_yaml(USER_CONFIG_PATH)
        if user_config:
            config.update(user_config)
    return config


def _display_width(text):
    """估算终端显示宽度：CJK 字符占 2 列，其余占 1 列。"""
    width = 0
    for ch in str(text):
        width += 2 if ord(ch) > 0x2E7F else 1
    return width


def _pad(text, width):
    return str(text) + " " * max(0, width - _display_width(text))


def _fmt_value(field, value):
    if field == "ip_address":
        return repr(value)  # 显式区分空串 '' 与广播
    return "true" if value else "false"


def _print_table(config):
    """编辑前展示：字段 / 当前值 / 用途说明。"""
    rows = [("字段", "当前值", "说明")]
    for field in _FIELD_ORDER:
        rows.append((field, _fmt_value(field, config.get(field)), _FIELD_DESC[field]))

    col0 = max(_display_width(r[0]) for r in rows)
    col1 = max(_display_width(r[1]) for r in rows)
    sep = f"+-{'-' * col0}-+-{'-' * col1}-+-{'-' * 48}-+"

    print(sep)
    for i, (c0, c1, c2) in enumerate(rows):
        print(f"| {_pad(c0, col0)} | {_pad(c1, col1)} | {_pad(c2, 48)} |")
        if i == 0:
            print(sep)
    print(sep)


def _prompt_str(name, current):
    value = input(f"{name} [{current!r}]（回车保留，输入空格清空）: ").strip("\n")
    if value == "":
        return current
    if value.strip() == "":
        return ""
    return value.strip()


def _prompt_bool(name, current):
    while True:
        value = input(f"{name} [{current}]（y/n，回车保留）: ").strip().lower()
        if value == "":
            return current
        if value in ("y", "yes", "true", "1"):
            return True
        if value in ("n", "no", "false", "0"):
            return False
        print("  请输入 y 或 n")


def main():
    config = _load_start_config()
    print("=== H12/G12 启动 VR 配置 ===")
    print(f"用户配置文件: {USER_CONFIG_PATH}")
    print("当前各字段值与说明：\n")
    _print_table(config)
    print("\n开始逐项编辑（回车保留当前值）：\n")
    config["ip_address"] = _prompt_str("ip_address", config.get("ip_address", ""))
    for field in BOOL_FIELDS:
        config[field] = _prompt_bool(field, config.get(field))

    try:
        validate_config(config)
    except H12VrLaunchConfigError as e:
        print(f"\n校验失败，未保存: {e}")
        return 1

    os.makedirs(os.path.dirname(USER_CONFIG_PATH), exist_ok=True)
    with open(USER_CONFIG_PATH, "w") as f:
        f.write(_USER_FILE_TEMPLATE.format(
            ip_address=config["ip_address"],
            use_cpp_incremental_ik=str(config["use_cpp_incremental_ik"]).lower(),
            use_incremental_hand_orientation=str(config["use_incremental_hand_orientation"]).lower(),
            control_torso=str(config["control_torso"]).lower(),
            enable_vr_stop_robot=str(config["enable_vr_stop_robot"]).lower(),
        ))
    print(f"\n已保存到 {USER_CONFIG_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
