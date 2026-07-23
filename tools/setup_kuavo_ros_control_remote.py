import os as _os
import sys as _sys

# ============================================================================
# 版本注册表 —— 新增版本只需在此字典加一行，其余逻辑自动生效。
# 注意: 此文件可能被单独拉取部署，不依赖仓库内其他文件。
#       与 src/kuavo_common/python/robot_version.py 保持同步。
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


# 尝试从仓库内 robot_version.py 同步更完整的 RobotVersion 类（可选）
try:
    _robot_version_path = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'src', 'kuavo_common', 'python')
    if _robot_version_path not in _sys.path:
        _sys.path.insert(0, _robot_version_path)
    from robot_version import RobotVersion  # noqa: F401
except ImportError:
    pass

FACTORY_URL = "git://10.11.99.175:9418/kuavo-ros-opensource.git"
GITCODE_URL = "https://gitcode.com/OpenLET/kuavo-ros-opensource.git"
GITEE_URL = "https://gitee.com/leju-robot/kuavo-ros-opensource.git"
KUAVO_ROS_ALLOWED_URLS = (FACTORY_URL, GITCODE_URL, GITEE_URL)

SOURCE_MODE_AUTO = "auto"
SOURCE_MODE_FACTORY = "factory"
SOURCE_MODE_GITCODE = "gitcode"
SOURCE_MODE_GITEE = "gitee"


def is_allowed_remote(remote_url, allowed_urls):
    remote = (remote_url or "").strip()
    return any(remote == url for url in allowed_urls)


def expected_urls_text(allowed_urls):
    return " 或 ".join(allowed_urls)


def clone_sources(mode=SOURCE_MODE_AUTO):
    if mode == SOURCE_MODE_FACTORY:
        return [FACTORY_URL]
    if mode == SOURCE_MODE_GITCODE:
        return [GITCODE_URL]
    if mode == SOURCE_MODE_GITEE:
        return [GITEE_URL]
    return [GITCODE_URL, FACTORY_URL, GITEE_URL]


def branch_exists(ls_remote_output, branch_name):
    return f"refs/heads/{branch_name}" in (ls_remote_output or "")


def commit_exists_in_origin_factory(branch_contains_output):
    return any("origin_factory/" in line for line in (branch_contains_output or "").splitlines())
