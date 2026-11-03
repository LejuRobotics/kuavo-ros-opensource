#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将动捕标定得到的关节 bias 安全写入零点文件，重启控制程序后生效。

修复理由：
1. 旧脚本固定假设 12 个腿关节和 14 轴手臂，无法覆盖轮臂及早期 8 轴手臂型号。
   本脚本从 kuavo.json 的 NUM_ARM_JOINT、NUM_HEAD_JOINT 和 MOTORS_TYPE 推导布局。
2. 旧脚本先修改 offset.csv，再读取已修改的 EC 零点，可能把同一 bias 重复扣除。
   本脚本始终从原文件计算一次目标值，再统一写回。
3. 不调用存在 EC 索引和执行器语义差异的 adjust_zero_point service，只修改
   offset.csv 和 arms_zero.yaml，避免运行期间零点写错槽位或触发机器人运动。
4. offset.csv 按驱动实际格式保留全部槽位（通常为 30 行），只修改目标 EC 槽位。
5. 所有输入和槽位长度均先严格校验，文件采用备份加原子替换，避免半更新状态。

安全说明：
- 默认写入零点文件；传 --dry-run 时仅打印计划。
- 脚本不会修改 hardware_node 当前内存中的零点，也不会控制机器人运动。
- 写入完成后，停止当前控制程序并重新 roslaunch，启动时才会加载新零点。

示例：
  ROBOT_VERSION=56 python3 write_zero_via_service_fixed.py --parts arm --dry-run
  ROBOT_VERSION=56 python3 write_zero_via_service_fixed.py --parts arm
  ROBOT_VERSION=100049 python3 write_zero_via_service_fixed.py --parts arm
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import pwd
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_DIR = SCRIPT_DIR.parents[1] / "kuavo_assets" / "config"
SUPPORTED_VERSIONS = tuple(
    sorted(
        (
            path.name[len("kuavo_v") :]
            for path in CONFIG_DIR.glob("kuavo_v*")
            if (path / "kuavo.json").is_file()
        ),
        key=int,
    )
)


@dataclass(frozen=True)
class JointLocation:
    joint: str
    motor_index: int
    motor_type: str
    file_kind: str
    file_index: int


@dataclass(frozen=True)
class ZeroChange:
    location: JointLocation
    bias_rad: float
    old_file_value: float
    new_file_value: float


def config_root_default() -> Path:
    sudo_user = os.environ.get("SUDO_USER")
    if sudo_user:
        try:
            return Path(pwd.getpwnam(sudo_user).pw_dir) / ".config" / "lejuconfig"
        except KeyError:
            pass
    return Path.home() / ".config" / "lejuconfig"


def load_json(path: Path) -> dict:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise SystemExit(f"JSON 文件不存在：{path}")
    except Exception as exc:
        raise SystemExit(f"无法读取 JSON {path}：{exc}")
    if not isinstance(data, dict):
        raise SystemExit(f"JSON 顶层必须是字典：{path}")
    return data


def load_yaml(path: Path) -> dict:
    try:
        import yaml
    except ImportError as exc:
        raise SystemExit(f"缺少 PyYAML：{exc}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except FileNotFoundError:
        raise SystemExit(f"YAML 文件不存在：{path}")
    except Exception as exc:
        raise SystemExit(f"无法读取 YAML {path}：{exc}")
    if not isinstance(data, dict):
        raise SystemExit(f"YAML 顶层必须是字典：{path}")
    return data


def dump_yaml(data: dict) -> str:
    import yaml

    return yaml.safe_dump(
        data,
        default_flow_style=False,
        sort_keys=False,
        allow_unicode=True,
    )


def read_offset_csv(path: Path) -> List[float]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        raise SystemExit(f"EC offset 文件不存在：{path}")
    values: List[float] = []
    for line_number, line in enumerate(lines, 1):
        text = line.strip().rstrip(",")
        if not text:
            values.append(0.0)
            continue
        try:
            value = float(text)
        except ValueError:
            raise SystemExit(f"无法解析 {path} 第 {line_number} 行：{line!r}")
        if not math.isfinite(value):
            raise SystemExit(f"{path} 第 {line_number} 行不是有限数值：{line!r}")
        values.append(value)
    return values


def dump_offset_csv(values: Sequence[float]) -> str:
    return "".join(f"{value:.9f}\n" for value in values)


def resolve_robot_version(value: str) -> str:
    if value in SUPPORTED_VERSIONS:
        return value
    env_value = os.environ.get("ROBOT_VERSION", "").strip()
    if env_value in SUPPORTED_VERSIONS:
        return env_value
    raise SystemExit(
        f"无法确定机型：请设置 ROBOT_VERSION，或传入 --robot-version。"
        f"当前可用型号：{', '.join(SUPPORTED_VERSIONS)}"
    )


def find_kuavo_json(robot_version: str, override: Optional[Path]) -> Path:
    if override is not None:
        if not override.is_file():
            raise SystemExit(f"kuavo.json 不存在：{override}")
        return override
    candidates = [
        Path(
            f"/home/lab/xzc_test/kuavo-ros-control/src/kuavo_assets/"
            f"config/kuavo_v{robot_version}/kuavo.json"
        ),
        Path(
            f"/root/kuavo_ws/src/kuavo_assets/config/"
            f"kuavo_v{robot_version}/kuavo.json"
        ),
        SCRIPT_DIR.parents[1]
        / "kuavo_assets"
        / "config"
        / f"kuavo_v{robot_version}"
        / "kuavo.json",
    ]
    for candidate in candidates:
        try:
            if candidate.is_file():
                return candidate
        except OSError:
            continue
    raise SystemExit(
        f"找不到 kuavo_v{robot_version}/kuavo.json，请用 --kuavo-json 指定"
    )


def is_ruiwo_motor_type(motor_type: object) -> bool:
    return str(motor_type).lower().startswith("ruiwo")


def build_joint_locations(
    robot_version: str,
    kuavo_json: Path,
) -> Tuple[Dict[str, JointLocation], int, List[str], List[str], int]:
    config = load_json(kuavo_json)
    motors = config.get("MOTORS_TYPE")
    if not isinstance(motors, list):
        raise SystemExit(f"{kuavo_json} 缺少 MOTORS_TYPE 数组")

    try:
        num_arm_joints = int(config["NUM_ARM_JOINT"])
        num_head_joints = int(config.get("NUM_HEAD_JOINT", 0))
    except (KeyError, TypeError, ValueError):
        raise SystemExit(f"{kuavo_json} 的 NUM_ARM_JOINT/NUM_HEAD_JOINT 无效")
    if num_arm_joints <= 0 or num_arm_joints % 2 != 0:
        raise SystemExit(f"NUM_ARM_JOINT 必须是正偶数，实际={num_arm_joints}")
    if num_head_joints < 0:
        raise SystemExit(f"NUM_HEAD_JOINT 不能为负数，实际={num_head_joints}")

    arm_joints_per_side = num_arm_joints // 2
    arm_joints = [
        *(f"zarm_l{i}_joint" for i in range(1, arm_joints_per_side + 1)),
        *(f"zarm_r{i}_joint" for i in range(1, arm_joints_per_side + 1)),
    ]
    head_joints = [f"zhead_{i}_joint" for i in range(1, num_head_joints + 1)]
    target_joints = arm_joints + head_joints
    target_count = len(target_joints)
    prefix_count = len(motors) - target_count
    if prefix_count < 0:
        raise SystemExit(
            f"MOTORS_TYPE 长度={len(motors)}，不足以容纳 {target_count} 个手臂/头部关节"
        )
    configured_joint_count = config.get("NUM_JOINT")
    if configured_joint_count is not None and int(configured_joint_count) != len(motors):
        raise SystemExit(
            f"NUM_JOINT={configured_joint_count} 与 MOTORS_TYPE 长度={len(motors)} 不一致"
        )

    joint_order = [f"prefix_joint_{index}" for index in range(prefix_count)] + target_joints
    locations: Dict[str, JointLocation] = {}
    ec_count = 0
    ruiwo_count = 0

    for motor_index, (joint, motor_type) in enumerate(zip(joint_order, motors)):
        ruiwo_type = is_ruiwo_motor_type(motor_type)
        current_ec_idx = None if ruiwo_type else ec_count
        current_ruiwo_slot = ruiwo_count if ruiwo_type else None

        if joint in target_joints:
            if not ruiwo_type:
                locations[joint] = JointLocation(
                    joint=joint,
                    motor_index=motor_index,
                    motor_type=str(motor_type),
                    file_kind="ec",
                    file_index=int(current_ec_idx),
                )
            else:
                locations[joint] = JointLocation(
                    joint=joint,
                    motor_index=motor_index,
                    motor_type=str(motor_type),
                    file_kind="arms",
                    file_index=int(current_ruiwo_slot),
                )

        if ruiwo_type:
            ruiwo_count += 1
        else:
            ec_count += 1

    target_slots = [
        location.file_index
        for location in locations.values()
        if location.file_kind == "arms"
    ]
    if len(target_slots) != len(set(target_slots)):
        raise SystemExit(f"Ruiwo/Motorevo 槽位重复：{target_slots}")
    if set(locations) != set(target_joints):
        missing = sorted(set(target_joints) - set(locations))
        raise SystemExit(f"关节映射不完整，缺少：{missing}")
    return locations, ec_count, arm_joints, head_joints, ruiwo_count


def discover_calibration_files(
    output_dir: Path,
    explicit_files: Optional[Sequence[Path]],
) -> List[Path]:
    if explicit_files:
        files = [path.resolve() for path in explicit_files]
    else:
        files = sorted(path.resolve() for path in output_dir.rglob("calibration.yaml"))
    missing = [path for path in files if not path.is_file()]
    if missing:
        raise SystemExit(f"calibration.yaml 不存在：{missing}")
    if not files:
        raise SystemExit(f"未找到 calibration.yaml：{output_dir}")
    return files


def merge_biases(files: Iterable[Path], target_joints: Sequence[str]) -> Dict[str, float]:
    target_joint_set = set(target_joints)
    biases: Dict[str, float] = {}
    for path in files:
        data = load_yaml(path)
        for key, value in data.items():
            if not isinstance(key, str) or key not in target_joint_set:
                continue
            try:
                bias = float(value)
            except (TypeError, ValueError):
                raise SystemExit(f"{path} 中 {key} 不是有效数值：{value!r}")
            if not math.isfinite(bias):
                raise SystemExit(f"{path} 中 {key} 不是有限数值：{value!r}")
            biases[key] = biases.get(key, 0.0) + bias
    return biases


def select_biases(
    biases: Dict[str, float],
    parts: str,
    joints: Optional[Sequence[str]],
    arm_joints: Sequence[str],
    head_joints: Sequence[str],
) -> Dict[str, float]:
    if parts == "arm":
        allowed = set(arm_joints)
    elif parts == "head":
        allowed = set(head_joints)
    else:
        allowed = set(arm_joints) | set(head_joints)
    if joints:
        unknown = sorted(set(joints) - allowed)
        if unknown:
            raise SystemExit(f"--joints 包含未知关节：{unknown}")
        allowed &= set(joints)
    selected = {joint: value for joint, value in biases.items() if joint in allowed}
    if not selected:
        raise SystemExit("所选部位没有可写入的 joint bias")
    return selected


def build_changes(
    biases: Dict[str, float],
    locations: Dict[str, JointLocation],
    ec_values: Sequence[float],
    arms_values: Sequence[float],
) -> List[ZeroChange]:
    changes: List[ZeroChange] = []
    for joint in sorted(biases, key=lambda name: locations[name].motor_index):
        location = locations[joint]
        bias_rad = biases[joint]
        if location.file_kind == "ec":
            old_value = float(ec_values[location.file_index])
            new_value = old_value - math.degrees(bias_rad)
        else:
            old_value = float(arms_values[location.file_index])
            new_value = old_value - bias_rad
        changes.append(
            ZeroChange(
                location=location,
                bias_rad=bias_rad,
                old_file_value=old_value,
                new_file_value=new_value,
            )
        )
    return changes


def backup_and_replace(files: Sequence[Tuple[Path, str]]) -> List[Path]:
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    original_text = {path: path.read_text(encoding="utf-8") for path, _ in files}
    backups: List[Path] = []
    for path, _ in files:
        backup = path.with_name(f"{path.name}.bak.{timestamp}")
        suffix = 1
        while backup.exists():
            backup = path.with_name(f"{path.name}.bak.{timestamp}.{suffix}")
            suffix += 1
        shutil.copy2(path, backup)
        backups.append(backup)

    def atomic_replace(path: Path, content: str, marker: str) -> None:
        file_stat = path.stat()
        temporary = path.with_name(f".{path.name}.{marker}.{os.getpid()}")
        try:
            temporary.write_text(content, encoding="utf-8")
            shutil.copystat(path, temporary)
            os.chown(temporary, file_stat.st_uid, file_stat.st_gid)
            os.replace(str(temporary), str(path))
        finally:
            if temporary.exists():
                temporary.unlink()

    replaced: List[Path] = []
    try:
        for path, content in files:
            atomic_replace(path, content, "tmp")
            replaced.append(path)
    except Exception:
        for path in replaced:
            atomic_replace(path, original_text[path], "rollback")
        raise
    return backups


def print_plan(changes: Sequence[ZeroChange]) -> None:
    print("\n== 写零计划 ==")
    for change in changes:
        location = change.location
        unit = "deg" if location.file_kind == "ec" else "rad"
        print(
            f"{location.joint:16s} motor={location.motor_index:>2} "
            f"{location.file_kind:4s} slot={location.file_index:>2} "
            f"{change.old_file_value:+.9f} -> {change.new_file_value:+.9f} {unit} "
            f"(bias={change.bias_rad:+.9f} rad)"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="离线零点文件写入脚本（默认写入，--dry-run 仅预览，重启后生效）"
    )
    parser.add_argument("--output-dir", type=Path, default=SCRIPT_DIR / "output")
    parser.add_argument(
        "--calibration-yaml",
        type=Path,
        action="append",
        help="显式指定 calibration.yaml；可重复传入，bias 会累加",
    )
    parser.add_argument(
        "--robot-version",
        choices=("auto",) + SUPPORTED_VERSIONS,
        default="auto",
    )
    parser.add_argument("--kuavo-json", type=Path)
    parser.add_argument("--config-root", type=Path, default=config_root_default())
    parser.add_argument("--parts", choices=("arm", "head", "both"), required=True)
    parser.add_argument("--joints", nargs="+", help="进一步限制到指定关节")
    parser.add_argument("--dry-run", action="store_true", help="只打印计划，不修改文件")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    robot_version = resolve_robot_version(args.robot_version)
    kuavo_json = find_kuavo_json(robot_version, args.kuavo_json)
    locations, ec_count, arm_joints, head_joints, expected_arms_count = (
        build_joint_locations(robot_version, kuavo_json)
    )
    calibration_files = discover_calibration_files(
        args.output_dir,
        args.calibration_yaml,
    )
    biases = select_biases(
        merge_biases(calibration_files, arm_joints + head_joints),
        args.parts,
        args.joints,
        arm_joints,
        head_joints,
    )

    zero_file = args.config_root / "arms_zero.yaml"
    offset_file = args.config_root / "offset.csv"
    zero_config = load_yaml(zero_file)
    raw_arms_values = zero_config.get("arms_zero_position")
    if not isinstance(raw_arms_values, list) or len(raw_arms_values) != expected_arms_count:
        actual = len(raw_arms_values) if isinstance(raw_arms_values, list) else "非 list"
        raise SystemExit(
            f"arms_zero_position 长度={actual}，ROBOT_VERSION={robot_version} "
            f"要求长度={expected_arms_count}"
        )
    try:
        arms_values = [float(value) for value in raw_arms_values]
    except (TypeError, ValueError):
        raise SystemExit("arms_zero_position 包含非数值项")
    if not all(math.isfinite(value) for value in arms_values):
        raise SystemExit("arms_zero_position 包含 NaN 或 Inf")

    selected_has_ec = any(locations[joint].file_kind == "ec" for joint in biases)
    selected_has_arms = any(locations[joint].file_kind == "arms" for joint in biases)
    ec_values = read_offset_csv(offset_file) if selected_has_ec else []
    if selected_has_ec and len(ec_values) < ec_count:
        raise SystemExit(
            f"offset.csv 长度={len(ec_values)}，少于 MOTORS_TYPE 推导的 EC 数量={ec_count}"
        )

    changes = build_changes(biases, locations, ec_values, arms_values)
    print("== 输入 ==")
    print(f"robot_version : {robot_version}")
    print(f"kuavo_json    : {kuavo_json}")
    print(f"config_root   : {args.config_root}")
    print(f"calibration   : {[str(path) for path in calibration_files]}")
    print_plan(changes)

    if args.dry_run:
        print("\n== dry-run ==")
        print("未写文件。")
        return 0

    new_ec_values = list(ec_values)
    new_arms_values = list(arms_values)
    for change in changes:
        if change.location.file_kind == "ec":
            new_ec_values[change.location.file_index] = change.new_file_value
        else:
            new_arms_values[change.location.file_index] = change.new_file_value

    files_to_write: List[Tuple[Path, str]] = []
    if selected_has_arms:
        zero_config["arms_zero_position"] = new_arms_values
        files_to_write.append((zero_file, dump_yaml(zero_config)))
    if selected_has_ec:
        files_to_write.append((offset_file, dump_offset_csv(new_ec_values)))

    try:
        backups = backup_and_replace(files_to_write)
    except Exception as exc:
        raise SystemExit(f"文件写入失败，已恢复原文件：{exc}")

    print("\n== 完成 ==")
    print("未调用 ROS service；新零点将在 hardware_node 重启后生效。")
    for backup in backups:
        print(f"backup: {backup}")
    for path, _ in files_to_write:
        print(f"written: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
