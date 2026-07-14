# -*- coding: utf-8 -*-
"""
配置加载模块：加载 config.yaml，用结构体接收
"""
import os
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Optional

try:
    import yaml
except ImportError:
    yaml = None


@dataclass
class ArrayConfig:
    """单个阵列配置结构体"""
    name: str
    joint: str
    T_rigid_body_to_joint: np.ndarray  # 4x4，刚体到 joint 的变换，单位 mm


@dataclass
class AllArraysConfig:
    """所有阵列的配置，通过名称索引"""
    arrays: Dict[str, ArrayConfig]
    hand_side: int = 0  # 0=左臂, 1=右臂

    def get(self, name: str) -> Optional[ArrayConfig]:
        """根据阵列名称获取配置"""
        return self.arrays.get(name)

    def names(self) -> List[str]:
        """返回所有阵列名称"""
        return list(self.arrays.keys())


def load_config(config_path: Optional[str] = None) -> AllArraysConfig:
    """
    加载配置文件，返回 AllArraysConfig 结构体。

    参数:
        config_path: 配置文件路径，默认为 config/config.yaml

    返回:
        AllArraysConfig: 所有阵列配置
    """
    if yaml is None:
        raise ImportError("需要安装 PyYAML: pip install pyyaml")

    if config_path is None:
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(pkg_root, "config", "config.yaml")

    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    arrays_data = data.get("arrays", {})
    hand_side = data.get("hand_side", 0)
    arrays = {}

    for name, arr in arrays_data.items():
        T = np.array(arr["T_rigid_body_to_joint"], dtype=float)
        arrays[name] = ArrayConfig(
            name=name,
            joint=arr["joint"],
            T_rigid_body_to_joint=T,
        )

    return AllArraysConfig(arrays=arrays, hand_side=hand_side)
