#!/usr/bin/env python3
"""
62/63 轮臂专用：把 output/**/calibration.yaml 的 joint bias 写入 Motorevo 16 维零点。

适用版本：基础型号 62/63 及对应小版本（200062/300062/400062/500062、400063/500063 等），
小版本按 %100 折回基础型号后由 ``robot_versions.py`` 机型表判定。

槽位顺序（device_id 1~16，与 kuavo5w_v62_dual_canbus 配置一致）：
  [zarm_l1..l7, zarm_r1..r7, zhead_1, zhead_2]

对齐目标（与 plot_board_error_from_csv.py 一致）：
  q_post = q_raw + bias  =>  new_zero = old_zero - bias（rad）

不写 EC offset.csv（62 臂/头均在 arms_zero.yaml）。
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import pwd
import sys
from pathlib import Path
from typing import Dict, List, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

import robot_versions as rv  # noqa: E402  (同目录模块，需先插入 sys.path)


def _get_config_root() -> Path:
    sudo_user = os.environ.get("SUDO_USER")
    if sudo_user:
        try:
            home = pwd.getpwnam(sudo_user).pw_dir
            return Path(home) / ".config" / "lejuconfig"
        except KeyError:
            pass
    return Path.home() / ".config" / "lejuconfig"


DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "output"
ZERO_FILE = _get_config_root() / "arms_zero.yaml"

# Motorevo 16 维槽位：Larm 1~7, Rarm 1~7, Head yaw/pitch
MOTOREVO_SLOT_JOINTS: List[str] = [
    "zarm_l1_joint",
    "zarm_l2_joint",
    "zarm_l3_joint",
    "zarm_l4_joint",
    "zarm_l5_joint",
    "zarm_l6_joint",
    "zarm_l7_joint",
    "zarm_r1_joint",
    "zarm_r2_joint",
    "zarm_r3_joint",
    "zarm_r4_joint",
    "zarm_r5_joint",
    "zarm_r6_joint",
    "zarm_r7_joint",
    "zhead_1_joint",
    "zhead_2_joint",
]


def _load_yaml(path: Path) -> dict:
    import yaml  # type: ignore

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise SystemExit(f"YAML 顶层不是 dict: {path}")
    return data


def _dump_yaml(data: dict) -> str:
    import yaml  # type: ignore

    return yaml.safe_dump(data, default_flow_style=False, sort_keys=False, allow_unicode=True)


def _discover_calibration_yamls(output_dir: Path) -> List[Path]:
    return sorted([p for p in output_dir.rglob("calibration.yaml") if p.is_file()])


def _read_biases(path: Path) -> Dict[str, float]:
    data = _load_yaml(path)
    out: Dict[str, float] = {}
    for k, v in data.items():
        if not isinstance(k, str) or not k.endswith("_joint"):
            continue
        try:
            out[k] = float(v)
        except Exception:
            pass
    return out


def _merge_biases(files: List[Path]) -> Dict[str, float]:
    merged: Dict[str, float] = {}
    for p in files:
        b = _read_biases(p)
        for j, v in b.items():
            merged[j] = merged.get(j, 0.0) + float(v)
    return merged


def _resolve_robot_version(arg: str):
    """按 robot_versions 机型表校验版本；只接受基础型号 62/63 及其小版本。"""
    candidate = arg if arg and arg != "auto" else os.environ.get("ROBOT_VERSION", "").strip()
    if not candidate:
        raise SystemExit(
            "无法确定人形机器人版本：请设置 ROBOT_VERSION=62/63 或对应小版本"
            "（如 200062/500062/400063），或通过 --robot-version 指定"
        )
    spec_ = rv.spec(candidate)
    if spec_ is None:
        raise SystemExit(
            f"不支持的机器人版本：ROBOT_VERSION={candidate}"
            f"（按 %100 折回为 {rv.fold(candidate)}）；"
            f"支持基础型号 62/63 及对应小版本"
        )
    if spec_.apply_script != Path(__file__).name:
        raise SystemExit(
            f"ROBOT_VERSION={candidate}（基础型号 {spec_.base}，{spec_.zero_slots} 维）"
            f"应使用 {Path(__file__).resolve().parent / spec_.apply_script} 写零点，"
            "本脚本只负责 62/63 系列的 Motorevo 16 维槽位。"
        )
    return candidate, spec_


def main() -> None:
    ap = argparse.ArgumentParser(description="62/63 轮臂：应用相机标定 joint bias 到 Motorevo 16 维零点")
    ap.add_argument("--output_dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    ap.add_argument(
        "--robot-version",
        default="auto",
        help=(
            "机器人版本；auto 时读 ROBOT_VERSION。"
            "支持 62/63 及对应小版本（200062/300062/400062/500062、400063/500063），"
            "按 %%100 折回基础型号"
        ),
    )
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    raw_version, spec_ = _resolve_robot_version(args.robot_version)

    calib_files = _discover_calibration_yamls(args.output_dir)
    if not calib_files:
        raise SystemExit(f"未找到 calibration.yaml: {args.output_dir}")
    biases = _merge_biases(calib_files)
    if not biases:
        raise SystemExit("calibration.yaml 里没有读到任何 *_joint bias")

    if not ZERO_FILE.exists():
        raise SystemExit(f"Motorevo 零点文件不存在：{ZERO_FILE}")

    expected_zero_count = spec_.zero_slots
    zero_cfg = _load_yaml(ZERO_FILE)
    zeros_raw = zero_cfg.get("arms_zero_position")
    if not isinstance(zeros_raw, list) or len(zeros_raw) != expected_zero_count:
        raise SystemExit(
            f"ROBOT_VERSION={raw_version}（基础型号 {spec_.base}）"
            f"的 arms_zero_position 必须是 {expected_zero_count} 维 list（当前 len="
            f"{len(zeros_raw) if isinstance(zeros_raw, list) else 'n/a'}）"
        )
    zeros = [float(x) for x in zeros_raw]

    new_zeros = list(zeros)
    logs: List[str] = []

    print("== Motorevo 16 维槽位映射（wheel62）==")
    for idx, joint in enumerate(MOTOREVO_SLOT_JOINTS):
        print(f"[{idx:02d}] slot -> {joint}")

    for idx, joint in enumerate(MOTOREVO_SLOT_JOINTS):
        if joint not in biases:
            continue
        before = float(new_zeros[idx])
        bias = float(biases[joint])
        # q_post = q_reported + bias => new_zero = old_zero - bias
        delta_zero = -bias
        after = before + delta_zero
        new_zeros[idx] = after
        logs.append(
            f"{joint}[{idx}]: {before:.9f} -> {after:.9f} "
            f"(bias={bias:+.9f}, applied_to_zero={delta_zero:+.9f})"
        )

    print("== 输入 ==")
    print(f"output_dir : {args.output_dir}")
    print(f"zero_file  : {ZERO_FILE}")
    print(f"found calibration.yaml: {len(calib_files)}")
    for p in calib_files:
        print(f"  - {p}")

    print("== Motorevo 应用结果 ==")
    if not logs:
        print("（无匹配 joint bias，未修改任何槽位）")
    for line in logs:
        print(line)

    print("== EC offset.csv ==")
    print("wheel62 不写 EC offset.csv")

    if args.dry_run:
        print("== dry-run ==")
        print("未写回文件（你指定了 --dry-run）")
        return

    ts = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    zbak = ZERO_FILE.with_name(ZERO_FILE.name + f".bak.{ts}")
    zbak.write_text(ZERO_FILE.read_text(encoding="utf-8"), encoding="utf-8")
    zero_cfg["arms_zero_position"] = new_zeros
    ZERO_FILE.write_text(_dump_yaml(zero_cfg), encoding="utf-8")

    print("== 写入完成 ==")
    print(f"backup : {zbak}")
    print(f"written: {ZERO_FILE}")


if __name__ == "__main__":
    main()
