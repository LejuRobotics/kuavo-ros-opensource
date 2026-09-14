#!/usr/bin/env python3
"""
人形机器人零点写入脚本（支持 52 / 56）：
把 src/Camera_Calibration/output/**/calibration.yaml 的 joint bias 合并后，
写入零点文件：

- 52 Ruiwo：~/.config/lejuconfig/arms_zero.yaml（固定 14 维）
    [zarm_l2..l7, zarm_r2..r7, zhead_1, zhead_2]
  52 EC：~/.config/lejuconfig/offset.csv（只更新 l1/r1 两项）

- 56 Ruiwo：~/.config/lejuconfig/arms_zero.yaml（固定 16 维）
    [zarm_l1..l7, zarm_r1..r7, zhead_1, zhead_2]
  56 的手臂零点不写 EC offset.csv。

对齐目标（与 plot_board_error_from_csv.py 一致）：
  q_post = q_raw + bias

Ruiwo negtive：
  不需要传参，固定从 ~/.config/lejuconfig/config.yaml 读取：
  - address（顺序：Left_joint_arm_1..6, Right_joint_arm_1..6, Head_joint_low/high）
  - negtive_address（反向电机地址列表）
  然后映射到 14 维零点槽位，自动决定对每个槽位是 +bias 还是 -bias。

EC 方向（按仓库 EC_Master 实现）：
  该驱动侧读取时：position = raw_deg - pos_offset
  因此要实现 q_post = q_raw + bias_deg，需要 pos_offset -= bias_deg
"""

from __future__ import annotations

import argparse
import datetime as dt
import math
import os
import pwd
from pathlib import Path
from typing import Dict, List, Tuple


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
EC_OFFSET_FILE = _get_config_root() / "offset.csv"
RUIWO_CONFIG_FILE = _get_config_root() / "config.yaml"

# 56 与 62 的零点文件槽位一致；56 实机的 config.yaml 不包含完整 16 槽位，
# 因此不能用 config.yaml 推导，必须按驱动零点文件的固定顺序处理。
BIPED56_SLOT_JOINTS: List[str] = [
    *(f"zarm_l{i}_joint" for i in range(1, 8)),
    *(f"zarm_r{i}_joint" for i in range(1, 8)),
    "zhead_1_joint",
    "zhead_2_joint",
]


def _ruiwo_slots_from_config(cfg: dict, robot_version: str) -> List[Tuple[str, bool, int, str]]:
    """
    复刻 ruiwo_actuator.cpp 的槽位顺序：
    - 遍历 config.yaml 的 address 节点（保持 YAML 键的遍历顺序）
    - 只收集 key 包含 Left_joint_arm / Right_joint_arm / Head_joint
    - 该顺序即 ruiwo_mtr_config_ 的顺序，也就是 arms_zero_position 的槽位顺序

    52 的 arm_1..6 映射到 l2..l7 / r2..r7；
    56 的 arm_1..7 映射到 l1..l7 / r1..r7。

    额外返回该槽位是否属于 negtive_address（与槽位顺序绑定，避免错位）。
    """
    addr_map = cfg.get("address", {})
    if not isinstance(addr_map, dict):
        raise SystemExit("config.yaml 缺少 address 字典，无法推导 Ruiwo 槽位顺序")

    neg = cfg.get("negtive_address", [])
    neg_set = set()
    if isinstance(neg, list) and neg:
        neg_list = neg[0] if isinstance(neg[0], list) else neg
        neg_set = {x for x in (_to_int(v) for v in neg_list) if isinstance(x, int)}

    ordered_keys: List[str] = []
    for k in addr_map.keys():
        if not isinstance(k, str):
            continue
        if ("Left_joint_arm" in k) or ("Right_joint_arm" in k) or ("Head_joint" in k):
            ordered_keys.append(k)

    def map_key(k: str) -> str | None:
        if k.startswith("Left_joint_arm_"):
            n = int(k.split("_")[-1])
            joint_n = n + 1 if robot_version == "52" else n
            return f"zarm_l{joint_n}_joint"
        if k.startswith("Right_joint_arm_"):
            n = int(k.split("_")[-1])
            joint_n = n + 1 if robot_version == "52" else n
            return f"zarm_r{joint_n}_joint"
        if k == "Head_joint_low":
            return "zhead_1_joint"
        if k == "Head_joint_high":
            return "zhead_2_joint"
        return None

    slots: List[Tuple[str, bool, int, str]] = []
    for k in ordered_keys:
        try:
            j = map_key(k)
        except Exception:
            j = None
        if not j:
            continue
        addr = _to_int(addr_map.get(k))
        if not isinstance(addr, int):
            raise SystemExit(f"config.yaml address 里 {k} 的值不是有效整数：{addr_map.get(k)!r}")
        is_neg = addr in neg_set
        slots.append((j, is_neg, addr, k))

    expected = 14 if robot_version == "52" else 16
    if len(slots) != expected:
        raise SystemExit(
            f"ROBOT_VERSION={robot_version} 应有 {expected} 个 Ruiwo 零点槽位"
            f"（实际={len(slots)}），请检查 config.yaml 的 address 键"
        )
    return slots

# EC_MASTER 两个关节的固定索引（与你们之前对齐一致）
EC_L1_IDX = 13
EC_R1_IDX = 14


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


def _read_offset_csv(path: Path) -> List[float]:
    if not path.exists():
        raise SystemExit(f"EC offset 文件不存在：{path}")
    out: List[float] = []
    for i, line in enumerate(path.read_text(encoding="utf-8").splitlines()):
        s = line.strip()
        if not s:
            out.append(0.0)
            continue
        try:
            out.append(float(s))
        except Exception:
            raise SystemExit(f"无法解析 {path} 第 {i+1} 行：{line!r}")
    return out


def _write_offset_csv(path: Path, values: List[float]) -> None:
    path.write_text("\n".join(f"{v:.9f}" for v in values) + "\n", encoding="utf-8")


def _rad2deg(x: float) -> float:
    return x * 180.0 / math.pi


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


def _to_int(v):
    try:
        if isinstance(v, int):
            return int(v)
        if isinstance(v, str):
            s = v.strip().lower()
            return int(s, 16) if s.startswith("0x") else int(s)
    except Exception:
        return None
    return None


def _load_ruiwo_slots(robot_version: str) -> List[Tuple[str, bool, int, str]]:
    if robot_version == "56":
        # 与 wheel62 相同的 16 维固定零点布局，不依赖 config.yaml。
        return [
            (joint, False, idx + 1, f"fixed_slot_{idx + 1}")
            for idx, joint in enumerate(BIPED56_SLOT_JOINTS)
        ]
    if not RUIWO_CONFIG_FILE.exists():
        raise SystemExit(f"Ruiwo config.yaml 不存在：{RUIWO_CONFIG_FILE}")
    cfg = _load_yaml(RUIWO_CONFIG_FILE)
    return _ruiwo_slots_from_config(cfg, robot_version)


def _resolve_robot_version(arg: str) -> str:
    if arg in ("52", "56"):
        return arg
    version = os.environ.get("ROBOT_VERSION", "").strip()
    if version not in ("52", "56"):
        raise SystemExit(
            "无法确定人形机器人版本：请设置 ROBOT_VERSION=52/56，"
            "或传入 --robot-version 52/56"
        )
    return version


def main() -> None:
    ap = argparse.ArgumentParser(description="应用相机标定 joint bias 到 52/56 人形机器人零点文件")
    ap.add_argument("--output_dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    ap.add_argument("--robot-version", choices=["auto", "52", "56"], default="auto")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    robot_version = _resolve_robot_version(args.robot_version)
    expected_zero_count = 14 if robot_version == "52" else 16

    calib_files = _discover_calibration_yamls(args.output_dir)
    if not calib_files:
        raise SystemExit(f"未找到 calibration.yaml: {args.output_dir}")
    biases = _merge_biases(calib_files)
    if not biases:
        raise SystemExit("calibration.yaml 里没有读到任何 *_joint bias")

    zero_cfg = _load_yaml(ZERO_FILE)
    zeros_raw = zero_cfg.get("arms_zero_position")
    if not isinstance(zeros_raw, list) or len(zeros_raw) != expected_zero_count:
        raise SystemExit(
            f"ROBOT_VERSION={robot_version} 的 arms_zero_position 必须是 "
            f"{expected_zero_count} 维 list（当前 len="
            f"{len(zeros_raw) if isinstance(zeros_raw, list) else 'n/a'}）"
        )
    zeros = [float(x) for x in zeros_raw]

    # Ruiwo：严格按驱动解析 config.yaml 的顺序，并同时绑定 negtive 标志，避免错位
    ruiwo_slots = _load_ruiwo_slots(robot_version)
    slot_joint_names = [j for (j, _is_neg, _addr, _key) in ruiwo_slots]
    neg_mask = [is_neg for (_j, is_neg, _addr, _key) in ruiwo_slots]

    new_zeros = list(zeros)
    ruiwo_logs: List[str] = []
    print("== Ruiwo 槽位映射（用于核对方向）==")
    for idx, (joint, is_neg, addr, key) in enumerate(ruiwo_slots):
        print(f"[{idx:02d}] key={key} addr={addr} negtive={is_neg} -> joint={joint}")

    for idx, joint in enumerate(slot_joint_names):
        if joint not in biases:
            continue
        before = float(new_zeros[idx])
        bias = float(biases[joint])
        # 目标（与 plot_board_error_from_csv.py 一致）：
        #   在 FK/误差计算里使用的是 q_used = q_reported + bias
        # 因此我们希望“硬件上报角”满足：q_post = q_reported + bias
        #
        # 关键：Ruiwo 驱动在 set_joint_state() 里对外发布的角度是
        #   q_reported = signed_raw_pos - zero_offset
        # （negtive 仅影响 signed_raw_pos 的符号；zero_offset 永远以减号参与上报）
        # 因此要让 q_reported 增加 bias：new_zero = old_zero - bias
        delta_zero = -bias
        after = before + delta_zero
        new_zeros[idx] = after
        ruiwo_logs.append(
            f"{joint}[{idx}]: {before:.9f} -> {after:.9f} (bias={bias:+.9f}, ruiwo_negtive={neg_mask[idx]}, applied_to_zero={delta_zero:+.9f})"
        )

    print("== 输入 ==")
    print(f"robot_version: {robot_version}")
    print(f"output_dir: {args.output_dir}")
    print(f"ruiwo_zero_file : {ZERO_FILE}")
    if robot_version == "52":
        print(f"ruiwo_config_yaml: {RUIWO_CONFIG_FILE}")
    else:
        print("ruiwo_slots: fixed 16-slot layout (same as wheel62; config.yaml not used)")
    print(f"ec_offset_file  : {EC_OFFSET_FILE}")
    print(f"found calibration.yaml: {len(calib_files)}")
    for p in calib_files:
        print(f"  - {p}")

    print("== Ruiwo 应用结果 ==")
    for line in ruiwo_logs:
        print(line)

    # 仅 52 的手臂 l1/r1 属于 EC；56 的 14 个手臂关节均属于 Ruiwo。
    new_ec: List[float] = []
    ec_logs: List[str] = []
    if robot_version == "52":
        new_ec = _read_offset_csv(EC_OFFSET_FILE)
        for joint, idx in (("zarm_l1_joint", EC_L1_IDX), ("zarm_r1_joint", EC_R1_IDX)):
            if joint not in biases:
                continue
            if idx < 0 or idx >= len(new_ec):
                raise SystemExit(f"{joint} ec_idx={idx} 超出 offset.csv 长度={len(new_ec)}")
            before = float(new_ec[idx])
            bias_deg = _rad2deg(float(biases[joint]))
            delta_offset = -bias_deg
            after = before + delta_offset
            new_ec[idx] = after
            ec_logs.append(
                f"{joint}[ec_idx={idx}]: {before:.9f} -> {after:.9f} "
                f"(bias_deg={bias_deg:+.9f}, applied_to_offset={delta_offset:+.9f})"
            )
    else:
        print("== EC offset.csv ==")
        print("ROBOT_VERSION=56：手臂零点全在 arms_zero.yaml，不修改 EC offset.csv")

    if ec_logs:
        print("== EC offset.csv 计划修改 ==")
        for line in ec_logs:
            print(line)

    if args.dry_run:
        print("== dry-run ==")
        print("未写回文件（你指定了 --dry-run）")
        return

    ts = dt.datetime.now().strftime("%Y%m%d-%H%M%S")

    zbak = ZERO_FILE.with_name(ZERO_FILE.name + f".bak.{ts}")
    zbak.write_text(ZERO_FILE.read_text(encoding="utf-8"), encoding="utf-8")
    zero_cfg["arms_zero_position"] = new_zeros
    ZERO_FILE.write_text(_dump_yaml(zero_cfg), encoding="utf-8")

    if ec_logs:
        ebak = EC_OFFSET_FILE.with_name(EC_OFFSET_FILE.name + f".bak.{ts}")
        ebak.write_text(EC_OFFSET_FILE.read_text(encoding="utf-8"), encoding="utf-8")
        _write_offset_csv(EC_OFFSET_FILE, new_ec)
        print(f"ec_backup: {ebak}")

    print("== 写入完成 ==")
    print(f"backup : {zbak}")
    print(f"written: {ZERO_FILE}")
    if ec_logs:
        print(f"written: {EC_OFFSET_FILE}")


if __name__ == "__main__":
    main()
