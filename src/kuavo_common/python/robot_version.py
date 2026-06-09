"""
机器人版本号类

机器人版本号格式为：
- MMMM 表示次版本号, 范围为0-9999
- N 表示修订号, 范围为0-9
- PPP 表示主版本号, 范围为0-9999
- BIGNUMBER: PPPPMMMMN
- STRING: PPPPMMMMN  Patch 为 0 时，不显示 Patch 部分, 比如 M(4), N(5),P(0) 表示为 45 而非 000000045 (0000,0004,5)
"""

import warnings


class RobotVersion:
    """机器人版本号类，用于表示机器人版本号"""

    def __init__(self, major: int = 0, minor: int = 0, patch: int = 0):
        """
        从版本号数字创建 RobotVersion 对象

        Args:
            major: 主版本号 (0-9999)
            minor: 次版本号 (0-9)
            patch: 补丁版本号 (0-9999), 默认为0

        Raises:
            ValueError: 如果版本号超出有效范围
        """
        if major < 0 or major > 9999 or minor < 0 or minor > 9 or patch < 0 or patch > 9999:
            raise ValueError(
                f"RobotVersion: Invalid version numbers: {major}.{minor}.{patch} (must be non-negative and within valid ranges)"
            )
        self._major = major
        self._minor = minor
        self._patch = patch

    @staticmethod
    def _extract_major_minor_patch(big_number: int):
        """
        从大整数中提取 major, minor, patch

        Args:
            big_number: 大整数 (PPPPMMMMN 格式)

        Returns:
            tuple: (major, minor, patch)
        """
        minor = big_number % 10
        big_number //= 10
        major = big_number % 10000
        patch = big_number // 10000
        return major, minor, patch

    @staticmethod
    def create(big_number: int) -> "RobotVersion":
        """
        从大整数创建 RobotVersion 对象

        Args:
            big_number: 大整数

        Returns:
            RobotVersion 实例

        Raises:
            ValueError: 如果版本号超出有效范围
        """
        if not RobotVersion.is_valid(big_number):
            raise ValueError(
                f"RobotVersion.create: Invalid version number: {big_number} (extracted values out of valid range)"
            )
        major, minor, patch = RobotVersion._extract_major_minor_patch(big_number)
        return RobotVersion(major, minor, patch)

    @staticmethod
    def is_valid(big_number: int) -> bool:
        """
        判断版本号是否合法

        Args:
            big_number: 版本号数字 (应为 int，非 int 时会 warn 并尝试转为 int)

        Returns:
            是否合法（无法转为 int 或超出范围时返回 False）
        """
        if not isinstance(big_number, int):
            warnings.warn(
                f"RobotVersion.is_valid: big_number should be int, got {type(big_number).__name__}, converting to int",
                stacklevel=2,
            )
            try:
                big_number = int(big_number)
            except (TypeError, ValueError):
                return False
        if big_number < 0:
            return False
        major, minor, patch = RobotVersion._extract_major_minor_patch(big_number)
        if major < 0 or major > 9999 or minor < 0 or minor > 9 or patch < 0 or patch > 9999:
            return False
        return True

    def to_string(self) -> str:
        """
        将版本号转换为字符串

        Returns:
            版本号字符串 如 45, 100045, 4000045, 100000049
        """
        return str(self.version_number())

    def start_with(self, major: int, minor: int = None) -> bool:
        """
        判断版本号是否属于某个版本系列

        Args:
            major: 主版本号
            minor: 次版本号（可选）

        Returns:
            是否以 major（或 major.minor）开头

        Note:
            例如：45, 100045, 4000045, 100000049 都是属于 4 代系列
            例如：45, 100045, 4000045 都是属于 45 版本系列
        """
        if minor is None:
            return self._major == major
        else:
            return self._major == major and self._minor == minor

    def version_number(self) -> int:
        """
        获取版本号对应的 PPPPMMMMN 数字

        Returns:
            PPPPMMMMN 数字

        Note:
            例如：45 --> 45
            100045 --> 00045.1 --> 45.1 --> 4.5.1
            2000105 --> 00105.20 --> 105.20 --> 10.5.20
        """
        return self._minor + self._major * 10 + self._patch * 100000

    def version_name(self) -> str:
        """
        获取版本号标准的 semantic version 字符串

        Returns:
            版本号字符串 如 4.5.0, 10.5.20

        Note:
            例如：45 --> 4.5.0
            100045 --> 4.5.1
            2000105 --> 10.5.20
        """
        return f"{self._major}.{self._minor}.{self._patch}"

    def major(self) -> int:
        """获取主版本号"""
        return self._major

    def minor(self) -> int:
        """获取次版本号"""
        return self._minor

    def patch(self) -> int:
        """获取补丁版本号"""
        return self._patch

    def __eq__(self, other: "RobotVersion") -> bool:
        """判断两个版本号是否相等"""
        if not isinstance(other, RobotVersion):
            return NotImplemented
        return (
            self._major == other._major
            and self._minor == other._minor
            and self._patch == other._patch
        )

    def __ne__(self, other: "RobotVersion") -> bool:
        """判断两个版本号是否不相等"""
        return not self == other

    def __lt__(self, other: "RobotVersion") -> bool:
        """判断版本号是否小于另一个版本号"""
        if not isinstance(other, RobotVersion):
            return NotImplemented
        return (
            self._major < other._major
            or (self._major == other._major and self._minor < other._minor)
            or (
                self._major == other._major
                and self._minor == other._minor
                and self._patch < other._patch
            )
        )

    def __le__(self, other: "RobotVersion") -> bool:
        """判断版本号是否小于等于另一个版本号"""
        return self < other or self == other

    def __gt__(self, other: "RobotVersion") -> bool:
        """判断版本号是否大于另一个版本号"""
        return not (self <= other)

    def __ge__(self, other: "RobotVersion") -> bool:
        """判断版本号是否大于等于另一个版本号"""
        return not (self < other)

    def __str__(self) -> str:
        """返回版本号的字符串表示"""
        return f"RobotVersion({self.to_string()})"

    def __repr__(self) -> str:
        """返回版本号的详细字符串表示"""
        return f"RobotVersion(major={self._major}, minor={self._minor}, patch={self._patch})"


# ============================================================================
# 版本注册表 —— 全项目 Python 端唯一源头
# 新增版本只需在此处加一行，其余逻辑（setup 脚本、工具链）自动生效。
# 格式: "显示版本": {"internal": 内部版本号, "series": 资源系列, "desc": 描述}
#   - 显示版本: 用户输入/展示用的版本号
#   - internal: 写入 ROBOT_VERSION 的数值
#   - series: 用于选择资源目录和机型判断 (kuavo4/kuavo5/kuavo5w/roban)
#   - desc: 版本说明
#
# Shell 端对应注册表: src/kuavo_common/scripts/robot_version.sh (保持同步)
# ============================================================================
VERSION_REGISTRY = {
    "42":   {"internal": 42,     "series": "kuavo4",  "desc": "短臂版本"},
    "45":   {"internal": 45,     "series": "kuavo4",  "desc": "长臂版本"},
    "49":   {"internal": 49,     "series": "kuavo4",  "desc": "pro max版本"},
    "45.1": {"internal": 100045, "series": "kuavo4",  "desc": "假手版"},
    "49.1": {"internal": 100049, "series": "kuavo4",  "desc": "展厅版"},
    "52":   {"internal": 52,     "series": "kuavo5",  "desc": "普通kuavo5"},
    "53":   {"internal": 53,     "series": "kuavo5",  "desc": "手臂pitch电机改ruiwo"},
    "55":   {"internal": 55,     "series": "kuavo5",  "desc": "手臂部分电机改ruiwoPA4310"},
    "60":   {"internal": 60,     "series": "kuavo5w", "desc": "悟时底盘轮臂"},
    "61":   {"internal": 61,     "series": "kuavo5w", "desc": "玖物底盘轮臂"},
    "13":   {"internal": 13,     "series": "roban",   "desc": "roban2.0版本"},
    "14":   {"internal": 14,     "series": "roban",   "desc": "roban2.1版本"},
    "15":   {"internal": 15,     "series": "roban",   "desc": "roban2.2版本"},
}


def get_valid_display_versions():
    """获取所有合法的显示版本号列表"""
    return list(VERSION_REGISTRY.keys())


def is_valid_version(version_str):
    """校验版本号是否合法"""
    return str(version_str) in VERSION_REGISTRY


def get_version_internal(version_str):
    """获取版本对应的内部版本号 (处理 45.1 -> 100045 等转换)"""
    info = VERSION_REGISTRY.get(str(version_str))
    return info["internal"] if info else version_str


def get_version_series(version_str):
    """获取版本所属的资源系列 (kuavo4/kuavo5/kuavo5w/roban)"""
    info = VERSION_REGISTRY.get(str(version_str))
    return info["series"] if info else "kuavo4"

