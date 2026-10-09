"""
ROBOT_VERSION 新格式解析模块 - 参考实现
最终实现以 src/robot_version/ 包为准，本文件供实现时参考

格式：<brand>-<product>-<locomotion>[-<end_effector>[-<camera>]][+tags...]

示例：
    export ROBOT_VERSION="kuavo-4pro-biped-revo1hand"
    export ROBOT_VERSION="kuavo-4pro-biped-revo1hand_lejuclaw-Gemini330_none_none_none"
    export ROBOT_VERSION="kuavo-4pro-biped-dummy"
    export ROBOT_VERSION="kuavo-5-wheeled-lejuclaw"
"""

import os
import re
import enum
from typing import Set, Optional


# ==================== Enum 定义 ====================

class Brand(enum.Enum):
    Kuavo = "kuavo"
    Roban = "roban"

class Locomotion(enum.Enum):
    Biped = "biped"
    Wheeled = "wheeled"

class EndEffectorType(enum.Enum):
    NoneType = "none"
    Lejuclaw = "lejuclaw"
    Revo1hand = "revo1hand"
    Revo2hand = "revo2hand"
    LinkhandO6 = "linkhandO6"
    Revo1touch = "revo1touch"
    Dummy = "dummy"

class CameraModel(enum.Enum):
    NoneType = "none"
    D435 = "D435"
    D405 = "D405"
    Gemini330 = "Gemini330"
    Gemini335L = "Gemini335L"


class RobotVersion:
    """机器人版本解析器"""

    _PATTERN = re.compile(
        r"^(kuavo|roban)-([a-zA-Z0-9]+)-(biped|wheeled)"
        r"(?:-((none|lejuclaw|revo1hand|revo2hand|linkhandO6|revo1touch|dummy)"
        r"(_(none|lejuclaw|revo1hand|revo2hand|linkhandO6|revo1touch|dummy))?))?"
        r"(?:-((none|D435|D405|Gemini330|Gemini335L)_"
        r"(none|D435|D405|Gemini330|Gemini335L)_"
        r"(none|D435|D405|Gemini330|Gemini335L)_"
        r"(none|D435|D405|Gemini330|Gemini335L)))?"
        r"((?:\+[a-zA-Z0-9_]+)*)$"
    )

    # ==================== 映射表 ====================

    # Legacy 兼容表：旧 int → 新格式字符串
    _LEGACY_MAP = {
        40:     "kuavo-4-biped-none",
        42:     "kuavo-4V2-biped-none",
        45:     "kuavo-4pro-biped-revo1hand",
        47:     "kuavo-4pro-biped-lejuclaw",
        100045: "kuavo-4pro-biped-dummy",
        49:     "kuavo-4proEDU-biped-revo1hand",
        100049: "kuavo-4proEDU-biped-dummy",
        50:     "kuavo-5-biped-none",
        52:     "kuavo-5V2-biped-none",
        60:     "kuavo-5-wheeled-none",
        14:     "roban-2v1-biped-none",
        # ... 实际需覆盖全部 33 个版本号
    }

    # 反向映射：新格式字符串 → 旧 int（用于 legacy_int()）
    _REVERSE_LEGACY_MAP = {v: k for k, v in _LEGACY_MAP.items()}

    class ModelInfo:
        """模型信息（按 brand-product-locomotion 索引）"""
        __slots__ = ("model_name", "config_name")

        def __init__(self, model_name: str, config_name: str):
            self.model_name = model_name      # "biped_s45"
            self.config_name = config_name    # "kuavo_v45"

    # 模型信息表：brand-product-locomotion → 路径字段
    _MODEL_MAP = {
        "kuavo-4-biped":       ModelInfo("biped_s40", "kuavo_v40"),
        "kuavo-4V2-biped":     ModelInfo("biped_s42", "kuavo_v42"),
        "kuavo-4pro-biped":    ModelInfo("biped_s45", "kuavo_v45"),
        "kuavo-4proEDU-biped": ModelInfo("biped_s49", "kuavo_v49"),
        "kuavo-5-biped":       ModelInfo("biped_s50", "kuavo_v50"),
        "kuavo-5V2-biped":     ModelInfo("biped_s52", "kuavo_v52"),
        "kuavo-5-wheeled":     ModelInfo("biped_s60", "kuavo_v60"),
        "roban-2v1-biped":     ModelInfo("biped_s14", "kuavo_v14"),
        # ... 实际需覆盖全部 base model
    }

    def __init__(self, version_input):
        """构造函数，自动检测输入类型"""
        raw = str(version_input).strip()
        if raw.isdigit():
            legacy_int = int(raw)
            if legacy_int in self._LEGACY_MAP:
                raw = self._LEGACY_MAP[legacy_int]
            else:
                raise ValueError(f"Unknown legacy version: {raw}")
        self._raw = raw
        self._robot_id = ""
        self._brand: Brand = Brand.Kuavo
        self._product = ""
        self._locomotion: Locomotion = Locomotion.Biped
        self._end_effector = ""
        self._left_end = ""
        self._right_end = ""
        self._tags: Set[str] = set()
        self._parse()

    @staticmethod
    def from_env(default: str = "") -> "RobotVersion":
        return RobotVersion(os.environ.get("ROBOT_VERSION", default))

    def _parse(self):
        if not self._raw:
            return

        match = self._PATTERN.match(self._raw)
        if not match:
            raise ValueError(f"Invalid ROBOT_VERSION format: {self._raw}")

        self._brand = Brand(match.group(1))
        self._product = match.group(2)
        self._locomotion = Locomotion(match.group(3))
        self._end_effector = match.group(4) or "none"
        self._robot_id = f"{self._brand.value}-{self._product}-{self._locomotion.value}-{self._end_effector}"

        if "_" in self._end_effector:
            self._left_end, self._right_end = self._end_effector.split("_")
        else:
            self._left_end = self._right_end = self._end_effector

        tags_str = match.group(8)
        if tags_str:
            self._tags = set(tags_str.split("+")[1:])

    # ==================== 核心属性 ====================

    @property
    def raw(self) -> str:
        return self._raw

    @property
    def robot_id(self) -> str:
        return self._robot_id

    @property
    def brand(self) -> Brand:
        return self._brand

    @property
    def product(self) -> str:
        return self._product

    @property
    def locomotion(self) -> Locomotion:
        return self._locomotion

    @property
    def end_effector(self) -> str:
        return self._end_effector

    @property
    def left_end(self) -> str:
        return self._left_end

    @property
    def right_end(self) -> str:
        return self._right_end

    @property
    def tags(self) -> Set[str]:
        return self._tags.copy()

    # ==================== 品牌/运动方式判断 ====================

    @property
    def is_kuavo(self) -> bool:
        return self._brand == Brand.Kuavo

    @property
    def is_roban(self) -> bool:
        return self._brand == Brand.Roban

    @property
    def is_biped(self) -> bool:
        return self._locomotion == Locomotion.Biped

    @property
    def is_wheeled(self) -> bool:
        return self._locomotion == Locomotion.Wheeled

    # 末端判断由调用方通过 left_end / right_end 自行判断，不提供 has_xxx 方法

    # ==================== 标签/硬件 ====================

    def has_tag(self, tag: str) -> bool:
        return tag in self._tags

    def get_upper_computer(self) -> Optional[str]:
        for tag in self._tags:
            if tag.startswith("upper_"):
                return tag[6:]
        return None

    def get_lower_computer(self) -> Optional[str]:
        for tag in self._tags:
            if tag.startswith("lower_"):
                return tag[6:]
        return None

    # ==================== 语义路径字段 ====================

    def _get_model_info(self) -> Optional["RobotVersion.ModelInfo"]:
        base_key = f"{self._brand.value}-{self._product}-{self._locomotion.value}"
        return self._MODEL_MAP.get(base_key)

    def model_name(self) -> str:
        """模型目录名，如 biped_s45"""
        info = self._get_model_info()
        return info.model_name if info else ""

    def config_name(self) -> str:
        """配置目录名，如 kuavo_v45"""
        info = self._get_model_info()
        return info.config_name if info else ""

    def legacy_int(self) -> int:
        """旧版本号整数，用于 /robot_version int 参数兼容。新格式专属机器人返回 0。"""
        return self._REVERSE_LEGACY_MAP.get(self._raw, 0)

    def __repr__(self) -> str:
        return f"RobotVersion('{self._raw}')"

    def __str__(self) -> str:
        return self._raw

    def __bool__(self) -> bool:
        return bool(self._raw)


if __name__ == "__main__":
    # 测试新格式输入
    test_cases = [
        "kuavo-4pro-biped-revo1hand",
        "kuavo-4pro-biped-revo1hand_lejuclaw",
        "kuavo-4pro-biped-dummy",
        "kuavo-4pro-biped-revo1hand-Gemini330_none_none_none",
        "kuavo-5-wheeled-lejuclaw+upper_nuci7+lower_amr7000",
        "roban-2v1-biped-lejuclaw",
    ]

    for version in test_cases:
        print(f"\n{'='*60}")
        print(f"Input: {version}")
        rv = RobotVersion(version)
        print(f"  brand: {rv.brand}")
        print(f"  product: {rv.product}")
        print(f"  locomotion: {rv.locomotion}")
        print(f"  end_effector: {rv.end_effector}")
        print(f"  left_end: {rv.left_end}, right_end: {rv.right_end}")
        print(f"  robot_id: {rv.robot_id}")
        print(f"  tags: {rv.tags}")
        print(f"  is_biped: {rv.is_biped}")
        print(f"  is_wheeled: {rv.is_wheeled}")
        print(f"  model_name: {rv.model_name()}")
        print(f"  config_name: {rv.config_name()}")
        print(f"  legacy_int: {rv.legacy_int()}")

    # 测试旧格式输入
    print(f"\n{'='*60}")
    print("Testing legacy int input:")
    for legacy_int in [45, 60, 14]:
        rv = RobotVersion(legacy_int)
        print(f"  {legacy_int} -> robot_id={rv.robot_id}, brand={rv.brand}, legacy_int={rv.legacy_int()}")
