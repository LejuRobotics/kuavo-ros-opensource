#!/usr/bin/env python3
"""
人形机器人零点写入脚本（支持基础型号 45 / 49 / 52 / 56 及对应小版本；62/63 走轮臂专用脚本）：
把 src/Camera_Calibration/output/**/calibration.yaml 的 joint bias 合并后，
写入零点文件：

- 45/49/52 Ruiwo：~/.config/lejuconfig/arms_zero.yaml（固定 14 维）
    [zarm_l2..l7, zarm_r2..r7, zhead_1, zhead_2]
  45/49/52 EC：~/.config/lejuconfig/offset.csv（只更新 l1/r1 两项）
- 53/55/56 Ruiwo：~/.config/lejuconfig/arms_zero.yaml（固定 16 维）
    [zarm_l1..l7, zarm_r1..r7, zhead_1, zhead_2]
  53/55/56 的手臂零点不写 EC offset.csv。
- 62/63（Motorevo）：改用 apply_zero_deltas_to_arms_zero_wheel62.py。

小版本按 %100 折回基础型号后查 ``robot_versions.py`` 的机型表（100049→49、
200053→53、400055→55 …）。注意 ``kuavo_v${ROBOT_VERSION}/kuavo.json`` 这类**资源
路径仍用原始版本号**，只有查机型表时才折回。

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
EC_OFFSET_FILE = _get_config_root() / "offset.csv"
RUIWO_CONFIG_FILE = _get_config_root() / "config.yaml"
KUAVO_CONFIG_DIR = Path(__file__).resolve().parent.parent / "kuavo_assets" / "config"

# 56 与 62 的零点文件槽位一致；56 实机的 config.yaml 不包含完整 16 槽位，
# 因此不能用 config.yaml 推导，必须按驱动零点文件的固定顺序处理。
BIPED56_SLOT_JOINTS: List[str] = [
    *(f"zarm_l{i}_joint" for i in range(1, 8)),
    *(f"zarm_r{i}_joint" for i in range(1, 8)),
    "zhead_1_joint",
    "zhead_2_joint",
]


# EC offset.csv 槽位的兜底表（kuavo.json 不可用时）：45/49 系列 l1/r1 = 12/13，52 = 13/14。
# 正常路径按 kuavo.json 的 MOTORS_TYPE 推导，不依赖这张表。
_EC_INDEX_FALLBACK: Dict[str, Tuple[int, int]] = {
    "45": (12, 13),
    "49": (12, 13),
    "52": (13, 14),
}


def _is_ruiwo_motor_type(motor_type: object) -> bool:
    return str(motor_type).lower().startswith("ruiwo")


def _ec_indices_from_kuavo_json(raw_version: str) -> Tuple[int, int]:
    """从 kuavo_v${raw_version}/kuavo.json 推导 zarm_l1/r1 在 offset.csv 的槽位。

    与 mocap_joint_calib/write_zero.py 的 build_joint_locations() 同一套规则：
    MOTORS_TYPE 的末段（NUM_ARM_JOINT 个手臂 + NUM_HEAD_JOINT 个头部）才是
    目标关节，非 Ruiwo 电机按出现顺序占用 EC offset.csv 槽位。

    以前这里写死 13/14（那是 52 的值），45/49 系列实际是 12/13，会静默错写槽位。
    """
    import json  # type: ignore

    config_path = KUAVO_CONFIG_DIR / f"kuavo_v{raw_version}" / "kuavo.json"
    with config_path.open("r", encoding="utf-8") as f:
        config = json.load(f)
    motors = config.get("MOTORS_TYPE")
    if not isinstance(motors, list):
        raise SystemExit(f"{config_path} 缺少 MOTORS_TYPE 数组")
    num_arm_joints = int(config["NUM_ARM_JOINT"])
    num_head_joints = int(config.get("NUM_HEAD_JOINT", 0))
    arm_joints_per_side = num_arm_joints // 2
    target_joints = [
        *(f"zarm_l{i}_joint" for i in range(1, arm_joints_per_side + 1)),
        *(f"zarm_r{i}_joint" for i in range(1, arm_joints_per_side + 1)),
        *(f"zhead_{i}_joint" for i in range(1, num_head_joints + 1)),
    ]
    prefix_count = len(motors) - len(target_joints)
    if prefix_count < 0:
        raise SystemExit(
            f"{config_path} 的 MOTORS_TYPE 长度={len(motors)}，"
            f"不足以容纳 {len(target_joints)} 个手臂/头部关节"
        )
    joint_order = [f"prefix_joint_{i}" for i in range(prefix_count)] + target_joints

    ec_slot: Dict[str, int] = {}
    ec_count = 0
    for joint, motor_type in zip(joint_order, motors):
        ruiwo_type = _is_ruiwo_motor_type(motor_type)
        if joint in ("zarm_l1_joint", "zarm_r1_joint") and not ruiwo_type:
            ec_slot[joint] = ec_count
        if not ruiwo_type:
            ec_count += 1
    missing = [j for j in ("zarm_l1_joint", "zarm_r1_joint") if j not in ec_slot]
    if missing:
        raise SystemExit(f"{config_path} 里 {missing} 不是 EC 电机，无法推导 offset.csv 槽位")
    return int(ec_slot["zarm_l1_joint"]), int(ec_slot["zarm_r1_joint"])


def _ec_indices(raw_version: str, spec_: "rv.ModelSpec") -> Tuple[int, int]:
    """按机型取 EC offset.csv 里 zarm_l1/r1 的槽位；优先从 kuavo.json 推导。"""
    if (KUAVO_CONFIG_DIR / f"kuavo_v{raw_version}" / "kuavo.json").is_file():
        return _ec_indices_from_kuavo_json(raw_version)
    fallback = _EC_INDEX_FALLBACK.get(spec_.base)
    if fallback is None:
        raise SystemExit(
            f"无法确定 ROBOT_VERSION={raw_version} 的 EC offset.csv 槽位："
            f"缺少 kuavo_v{raw_version}/kuavo.json，且基础型号 {spec_.base} 不在兜底表中"
        )
    return fallback


def _ruiwo_slots_from_config(cfg: dict, spec_: "rv.ModelSpec") -> List[Tuple[str, bool, int, str]]:
    """
    复刻 ruiwo_actuator.cpp 的槽位顺序：
    - 遍历 config.yaml 的 address 节点（保持 YAML 键的遍历顺序）
    - 只收集 key 包含 Left_joint_arm / Right_joint_arm / Head_joint
    - 该顺序即 ruiwo_mtr_config_ 的顺序，也就是 arms_zero_position 的槽位顺序

    14 维机型（45/49/52）的 arm_1..6 映射到 l2..l7 / r2..r7；
    16 维机型（53/55/56）的 arm_1..7 映射到 l1..l7 / r1..r7。

    额外返回该槽位是否属于 negtive_address（与槽位顺序绑定，避免错位）。
    """
    addr_map = cfg.get("address", {})
    if not isinstance(addr_map, dict):
        raise SystemExit("config.yaml 缺少 address 字典，无法推导 Ruiwo 槽位顺序")

    zero_based = spec_.zero_slots == 14

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
            joint_n = n + 1 if zero_based else n
            return f"zarm_l{joint_n}_joint"
        if k.startswith("Right_joint_arm_"):
            n = int(k.split("_")[-1])
            joint_n = n + 1 if zero_based else n
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

    expected = spec_.zero_slots
    if len(slots) != expected:
        raise SystemExit(
            f"ROBOT_VERSION 基础型号={spec_.base} 应有 {expected} 个 Ruiwo 零点槽位"
            f"（实际={len(slots)}），请检查 config.yaml 的 address 键"
        )
    return slots


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


def _load_ruiwo_slots(raw_version: str, spec_: "rv.ModelSpec") -> List[Tuple[str, bool, int, str]]:
    if spec_.zero_slots == 16:
        # 53/55/56/62/63 共用 wheel62 的 16 维固定零点布局，不依赖 config.yaml。
        return [
            (joint, False, idx + 1, f"fixed_slot_{idx + 1}")
            for idx, joint in enumerate(BIPED56_SLOT_JOINTS)
        ]
    if not RUIWO_CONFIG_FILE.exists():
        raise SystemExit(f"Ruiwo config.yaml 不存在：{RUIWO_CONFIG_FILE}")
    cfg = _load_yaml(RUIWO_CONFIG_FILE)
    return _ruiwo_slots_from_config(cfg, spec_)


def _resolve_robot_version(arg: str) -> Tuple[str, "rv.ModelSpec"]:
    """返回 (原始版本号, 机型特征)。支持小版本按 %100 折回基础型号。"""
    candidate = arg if arg and arg != "auto" else os.environ.get("ROBOT_VERSION", "").strip()
    if not candidate:
        raise SystemExit(
            "无法确定人形机器人版本：请设置 "
            f"ROBOT_VERSION={rv.supported_text('/')} 或对应小版本（如 100049/400055/500062），"
            "或通过 --robot-version 指定"
        )
    spec_ = rv.spec(candidate)
    if spec_ is None:
        raise SystemExit(
            f"不支持的机器人版本：ROBOT_VERSION={candidate}"
            f"（按 %100 折回为 {rv.fold(candidate)}）；"
            f"支持基础型号 {rv.supported_text('/')} 及其小版本"
        )
    if spec_.apply_script != Path(__file__).name:
        raise SystemExit(
            f"ROBOT_VERSION={candidate}（基础型号 {spec_.base}，{spec_.zero_slots} 维）"
            f"应使用 {Path(__file__).resolve().parent / spec_.apply_script} 写零点，"
            "本脚本只负责 45/49/52/53/55/56 的 Ruiwo/EC 槽位。"
        )
    return candidate, spec_


def _robot_version_arg(value: str) -> str:
    """argparse 类型检查：仍允许 auto 与非纯数字覆盖值，具体校验交给 _resolve_robot_version。"""
    return value


def main() -> None:
    ap = argparse.ArgumentParser(
        description="应用相机标定 joint bias 到 45/49 系列/52/53/55/56 人形机器人零点文件"
    )
    ap.add_argument("--output_dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    ap.add_argument(
        "--robot-version",
        type=_robot_version_arg,
        default="auto",
        help=(
            "机器人版本；auto 时读 ROBOT_VERSION。"
            "支持 45/49/52/53/55/56 及对应小版本"
            "（100049/200049/300049/400049、200053/300053、400055），按 %%100 折回基础型号；"
            "62/63 系列用 apply_zero_deltas_to_arms_zero_wheel62.py"
        ),
    )
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    raw_version, spec_ = _resolve_robot_version(args.robot_version)
    robot_version = spec_.base
    ec_slots = spec_.ec_l1_r1
    expected_zero_count = spec_.zero_slots

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
            f"ROBOT_VERSION={raw_version}（基础型号 {robot_version}）的 arms_zero_position 必须是 "
            f"{expected_zero_count} 维 list（当前 len="
            f"{len(zeros_raw) if isinstance(zeros_raw, list) else 'n/a'}）"
        )
    zeros = [float(x) for x in zeros_raw]

    # Ruiwo：严格按驱动解析 config.yaml 的顺序，并同时绑定 negtive 标志，避免错位
    ruiwo_slots = _load_ruiwo_slots(raw_version, spec_)
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
    print(f"robot_version: {raw_version} (基础型号 {robot_version})")
    print(f"机型: {rv.describe(raw_version)}")
    print(f"output_dir: {args.output_dir}")
    print(f"ruiwo_zero_file : {ZERO_FILE}")
    if ec_slots:
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

    # 仅 45/49/52 的手臂 l1/r1 属于 EC；53/55/56 的手臂关节均属于 Ruiwo。
    new_ec: List[float] = []
    ec_logs: List[str] = []
    if ec_slots:
        ec_l1_idx, ec_r1_idx = _ec_indices(raw_version, spec_)
        print(f"ec_slots: zarm_l1_joint -> offset.csv[{ec_l1_idx}], zarm_r1_joint -> offset.csv[{ec_r1_idx}]")
        new_ec = _read_offset_csv(EC_OFFSET_FILE)
        for joint, idx in (("zarm_l1_joint", ec_l1_idx), ("zarm_r1_joint", ec_r1_idx)):
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
        print(
            f"ROBOT_VERSION={raw_version}（基础型号 {robot_version}）："
            "手臂零点全在 arms_zero.yaml，不修改 EC offset.csv"
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
