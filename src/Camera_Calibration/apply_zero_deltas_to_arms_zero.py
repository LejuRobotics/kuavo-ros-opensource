#!/usr/bin/env python3
"""
硬编码精简版（覆盖旧通用版）：
把 src/Camera_Calibration/output/**/calibration.yaml 的 joint bias 合并后，
写入零点文件：

- Ruiwo：~/.config/lejuconfig/arms_zero.yaml (arms_zero_position, 固定 14 维)
  槽位顺序固定：
    [zarm_l2..l7, zarm_r2..r7, zhead_1, zhead_2]

- EC：~/.config/lejuconfig/offset.csv（只更新 2 项）
    zarm_l1_joint ec_idx=13
    zarm_r1_joint ec_idx=14

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


def _ruiwo_slots_from_config(cfg: dict) -> List[Tuple[str, bool, int, str]]:
    """
    复刻 ruiwo_actuator.cpp 的槽位顺序：
    - 遍历 config.yaml 的 address 节点（保持 YAML 键的遍历顺序）
    - 只收集 key 包含 Left_joint_arm / Right_joint_arm / Head_joint
    - 该顺序即 ruiwo_mtr_config_ 的顺序，也就是 arms_zero_position 的槽位顺序

    然后把这些 key 映射到 robot_calibration 使用的 joint 名：
      Left_joint_arm_1..6  -> zarm_l2..l7
      Right_joint_arm_1..6 -> zarm_r2..r7
      Head_joint_low/high  -> zhead_1/zhead_2

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
            return f"zarm_l{n+1}_joint"  # 1->l2 ... 6->l7
        if k.startswith("Right_joint_arm_"):
            n = int(k.split("_")[-1])
            return f"zarm_r{n+1}_joint"  # 1->r2 ... 6->r7
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

    if len(slots) != 14:
        raise SystemExit(f"推导到的 Ruiwo 槽位 joint 数量不是 14（实际={len(slots)}），请检查 config.yaml 的 address 键")
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


def _load_ruiwo_slots() -> List[Tuple[str, bool, int, str]]:
    if not RUIWO_CONFIG_FILE.exists():
        raise SystemExit(f"Ruiwo config.yaml 不存在：{RUIWO_CONFIG_FILE}")
    cfg = _load_yaml(RUIWO_CONFIG_FILE)
    return _ruiwo_slots_from_config(cfg)


def main() -> None:
    ap = argparse.ArgumentParser(description="应用相机标定 joint bias 到零点文件（硬编码精简版）")
    ap.add_argument("--output_dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    calib_files = _discover_calibration_yamls(args.output_dir)
    if not calib_files:
        raise SystemExit(f"未找到 calibration.yaml: {args.output_dir}")
    biases = _merge_biases(calib_files)
    if not biases:
        raise SystemExit("calibration.yaml 里没有读到任何 *_joint bias")

    zero_cfg = _load_yaml(ZERO_FILE)
    zeros_raw = zero_cfg.get("arms_zero_position")
    if not isinstance(zeros_raw, list) or len(zeros_raw) != 14:
        raise SystemExit(f"arms_zero_position 必须是 14 维 list（当前 len={len(zeros_raw) if isinstance(zeros_raw, list) else 'n/a'}）")
    zeros = [float(x) for x in zeros_raw]

    # Ruiwo：严格按驱动解析 config.yaml 的顺序，并同时绑定 negtive 标志，避免错位
    ruiwo_slots = _load_ruiwo_slots()
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
    print(f"output_dir: {args.output_dir}")
    print(f"ruiwo_zero_file : {ZERO_FILE}")
    print(f"ruiwo_config_yaml: {RUIWO_CONFIG_FILE}")
    print(f"ec_offset_file  : {EC_OFFSET_FILE}")
    print(f"found calibration.yaml: {len(calib_files)}")
    for p in calib_files:
        print(f"  - {p}")

    print("== Ruiwo 应用结果 ==")
    for line in ruiwo_logs:
        print(line)

    # EC: 固定更新 l1/r1 两项（若 bias 存在）
    ec = _read_offset_csv(EC_OFFSET_FILE)
    new_ec = list(ec)
    ec_logs: List[str] = []
    for joint, idx in (("zarm_l1_joint", EC_L1_IDX), ("zarm_r1_joint", EC_R1_IDX)):
        if joint not in biases:
            continue
        if idx < 0 or idx >= len(new_ec):
            raise SystemExit(f"{joint} ec_idx={idx} 超出 offset.csv 长度={len(new_ec)}")
        before = float(new_ec[idx])
        bias_deg = _rad2deg(float(biases[joint]))
        # 目标：q_post = q_raw + bias_deg
        # EC_Master 侧：q = raw_deg - pos_offset
        # => q_post = raw_deg - (pos_offset + delta_offset)
        # 令 q_post = q_raw + bias_deg = raw_deg - pos_offset + bias_deg
        # 得 delta_offset = -bias_deg
        delta_offset = -bias_deg
        after = before + delta_offset
        new_ec[idx] = after
        ec_logs.append(
            f"{joint}[ec_idx={idx}]: {before:.9f} -> {after:.9f} (bias_deg={bias_deg:+.9f}, applied_to_offset={delta_offset:+.9f})"
        )

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

