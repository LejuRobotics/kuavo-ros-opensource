"""H12/G12 启动 VR 遥操的 yaml 配置加载、校验与 launch 参数拼装。

加载顺序：包内 config/h12_vr_launch.yaml 为默认基底，用户目录
~/.config/lejuconfig/h12_vr_launch.yaml 覆盖同名字段。校验失败直接抛
H12VrLaunchConfigError，不做回退，由调用方输出异常日志。
"""

import os

import yaml

# 包内默认配置：<pkg>/config/h12_vr_launch.yaml（本文件在 <pkg>/utils/ 下）
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG_PATH = os.path.join(_PKG_DIR, "config", "h12_vr_launch.yaml")
USER_CONFIG_PATH = os.path.expanduser("~/.config/lejuconfig/h12_vr_launch.yaml")

# 注入到 launch 的布尔字段，顺序即拼装顺序
BOOL_FIELDS = (
    "use_cpp_incremental_ik",
    "use_incremental_hand_orientation",
    "control_torso",
    "enable_vr_stop_robot",
)


class H12VrLaunchConfigError(Exception):
    """yaml 字段缺失、类型错误或组合非法时抛出。"""


def _load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def load_config(user_path=USER_CONFIG_PATH, default_path=DEFAULT_CONFIG_PATH):
    """以包内默认为基底，用户目录配置覆盖，返回校验通过的配置 dict。"""
    config = _load_yaml(default_path)
    if os.path.exists(user_path):
        user_config = _load_yaml(user_path)
        if user_config:
            config.update(user_config)
    validate_config(config)
    return config


def validate_config(config):
    """校验字段类型与组合，非法直接抛 H12VrLaunchConfigError。"""
    if not isinstance(config.get("ip_address", ""), str):
        raise H12VrLaunchConfigError("ip_address 必须为字符串")
    for field in BOOL_FIELDS:
        if not isinstance(config.get(field), bool):
            raise H12VrLaunchConfigError(f"{field} 必须为布尔值 (true/false)")
    # 位置非增量 + 姿态增量 为无效组合
    if not config["use_cpp_incremental_ik"] and config["use_incremental_hand_orientation"]:
        raise H12VrLaunchConfigError(
            "无效组合: use_cpp_incremental_ik=false 时 "
            "use_incremental_hand_orientation 不能为 true"
        )


def build_launch_args(config):
    """把配置拼成 roslaunch 的 key:=value 串；ip_address 为空则不注入。"""
    args = []
    ip_address = config.get("ip_address", "")
    if ip_address:
        args.append(f"ip_address:={ip_address}")
    for field in BOOL_FIELDS:
        args.append(f"{field}:={'true' if config[field] else 'false'}")
    return " ".join(args)


def build_vr_launch_args():
    """加载并校验配置后返回 launch 参数串，供 VR 启动回调直接拼接。"""
    return build_launch_args(load_config())
