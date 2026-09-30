#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯动捕标定回归测试（Ceres 全链路 + 噪声鲁棒性）。

流程：
1. 用真实关节角(joints.csv) + 已知 bias_true 生成"理想动捕观测"
2. 可选加噪声（位置/姿态扰动），模拟动捕误差
3. 跑 optimize_mocap（Ceres，通过 mocap_optimize.sh）
4. 读 output/calibration.yaml，对比 bias_true
5. 判定：bias 还原误差是否在阈值内 → 通过/失败

用法：
    python3 regression_test.py                    # 默认：无噪声，bias ±0.03rad
    python3 regression_test.py --noise real       # 真实噪声：位置σ=2mm, 姿态σ=0.3°
    python3 regression_test.py --noise extreme    # 极端噪声：位置±50mm, 姿态±5°
    python3 regression_test.py --layout biped45   # 测 s45（无腰，fk_root=base_link）
    python3 regression_test.py --bias-max 0.05    # bias_true 范围 ±0.05rad
    python3 regression_test.py --threshold 1.0    # 判定阈值(deg)

退出码：0=通过, 1=失败, 2=运行错误
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
_MODULE_DIR = _SCRIPT_DIR.parent  # mocap_joint_calib/
_CAMERA_CAL_DIR = _MODULE_DIR.parent
_MOCAP_DIR = _CAMERA_CAL_DIR / "mocap_checkerboard_pose"
for _p in (str(_CAMERA_CAL_DIR), str(_MOCAP_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from plot_board_error_from_csv import load_urdf_joints, fk_root_to_tip_transform  # noqa: E402
from mocap_pose_utils import R_to_quat_xyzw  # noqa: E402

# 各机型配置（与 config/*.yaml 的 joint_q_indices / fk_root / URDF 对应）
LAYOUTS = {
    "wheel62": {
        "urdf": "biped_v3_arm_s62.urdf",
        "fk_root": "waist_yaw_link",
        "joint_q_indices": {"waist": 12, "left_arm": [13, 20], "right_arm": [20, 27], "head": [27, 29]},
        "joint_q_len": 29,
        "joints_csv": "output_csv/kuavo_right_wrist/joints.csv",
    },
    "biped45": {
        "urdf": "biped_v3_arm_s45.urdf",
        "fk_root": "base_link",
        "joint_q_indices": {"left_arm": [12, 19], "right_arm": [19, 26], "head": [26, 28]},
        "joint_q_len": 28,
        "joints_csv": "output_csv/kuavo_right_wrist/joints.csv",
    },
}

FREE_JOINTS = [
    "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
    "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",
    "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
    "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint",
]
TIP_LINKS = {"l_hand": "zarm_l7_link", "r_hand": "zarm_r7_link"}


def _rodrigues(axis, angle):
    x, y, z = axis
    c = float(np.cos(angle)); s = float(np.sin(angle)); C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def _load_joints_csv(path: Path):
    js_by_sample = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            js_by_sample.setdefault(int(r["sample_id"]), {})[r["joint_name"]] = float(r["position"])
    return js_by_sample


def _build_capture(layout_cfg: dict, bias_true: dict, noise: str, seed: int) -> dict:
    """生成合成动捕 capture（FK 生成观测 + 可选噪声）。"""
    urdf_path = _CAMERA_CAL_DIR / layout_cfg["urdf"]
    joints = load_urdf_joints(urdf_path)
    js_by_sample = _load_joints_csv(_CAMERA_CAL_DIR / layout_cfg["joints_csv"])

    rng = np.random.default_rng(seed)
    ji = layout_cfg["joint_q_indices"]
    q_len = layout_cfg["joint_q_len"]

    # 噪声配置
    if noise == "extreme":
        pos_std, rot_std = 50.0, 5.0  # ±50mm, ±5°
    elif noise == "real":
        pos_std, rot_std = 2.0, 0.3  # σ=2mm, σ=0.3°
    else:
        pos_std, rot_std = 0.0, 0.0

    samples = []
    for sid in sorted(js_by_sample):
        q_rep = js_by_sample[sid]
        q = [0.0] * q_len
        if "waist" in ji:
            q[ji["waist"]] = q_rep.get("waist_yaw_joint", 0.0)
        lo_l, hi_l = ji["left_arm"]
        lo_r, hi_r = ji["right_arm"]
        for k, name in enumerate(FREE_JOINTS[:7]):
            q[lo_l + k] = q_rep.get(name, 0.0)
        for k, name in enumerate(FREE_JOINTS[7:]):
            q[lo_r + k] = q_rep.get(name, 0.0)

        bodies = {}
        for body, tip in TIP_LINKS.items():
            q_used = {j: q_rep.get(j, 0.0) + bias_true.get(j, 0.0) for j in FREE_JOINTS}
            if "waist" in ji:
                q_used["waist_yaw_joint"] = q_rep.get("waist_yaw_joint", 0.0)
            T = fk_root_to_tip_transform(joints, layout_cfg["fk_root"], tip, q_used)

            pos_noise = rng.normal(0, pos_std, 3) if pos_std > 0 else np.zeros(3)
            if rot_std > 0:
                axis = rng.normal(size=3); axis /= np.linalg.norm(axis)
                angle = rng.normal(0, rot_std) * np.pi / 180
                R_obs = _rodrigues(axis, angle) @ T[:3, :3]
            else:
                R_obs = T[:3, :3]
            p_obs = T[:3, 3] * 1000.0 + pos_noise

            bodies[f"{body}_in_torso"] = {
                "xyz_mm": p_obs.tolist(),
                "quaternion_xyzw": R_to_quat_xyzw(R_obs).tolist(),
                "valid_frames": 100, "pos_std_mm": 1.0, "rot_std_deg": 0.2,
            }
        samples.append({"index": sid, "joint_q_full": q, "commanded": {}, "bodies": bodies})

    return {
        "meta": {
            "urdf": layout_cfg["urdf"],
            "fk_root": layout_cfg["fk_root"],
            "joint_q_indices": ji,
            "mocap_frame_align": {"enabled": False},
        },
        "bodies": [
            {"name": "l_hand", "tip_link": TIP_LINKS["l_hand"], "link_offset_mm": [0, 0, 0], "record_only": False},
            {"name": "r_hand", "tip_link": TIP_LINKS["r_hand"], "link_offset_mm": [0, 0, 0], "record_only": False},
        ],
        "samples": samples,
    }


def _read_calibration_yaml(path: Path) -> dict:
    sol = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if ":" in line:
                k, _, v = line.partition(":")
                sol[k.strip()] = float(v.strip())
    return sol


def _run_optimize(capture_path: Path, layout: str) -> int:
    """调 mocap_optimize.sh 跑 Ceres 优化。返回退出码。"""
    script = _MODULE_DIR / "mocap_optimize.sh"
    # 复制 capture 到模块目录（脚本自动找最新 capture_*.json）
    test_capture = _MODULE_DIR / "capture_regression.json"
    import shutil
    shutil.copy2(capture_path, test_capture)
    # 清理旧 output
    output_dir = _MODULE_DIR / "output"
    if output_dir.exists():
        shutil.rmtree(output_dir)
    env = os.environ.copy()
    p = subprocess.run(
        ["bash", str(script), "--layout", layout],
        cwd=str(_CAMERA_CAL_DIR.parent.parent),
        env=env,
        capture_output=True,
        text=True,
        timeout=180,
    )
    # 清理临时 capture
    test_capture.unlink(missing_ok=True)
    return p.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description="纯动捕标定回归测试")
    parser.add_argument("--layout", choices=["wheel62", "biped45"], default="wheel62")
    parser.add_argument("--noise", choices=["none", "real", "extreme"], default="none",
                        help="噪声档：none(无) / real(σ=2mm,0.3°) / extreme(±50mm,±5°)")
    parser.add_argument("--bias-max", type=float, default=0.03, help="bias_true 范围 ±bias_max rad")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--threshold", type=float, default=0.5,
                        help="判定阈值：bias 还原误差 > 此值(deg) 判失败")
    args = parser.parse_args()

    layout_cfg = LAYOUTS[args.layout]
    rng = np.random.default_rng(args.seed)
    bias_true = {j: rng.uniform(-args.bias_max, args.bias_max) for j in FREE_JOINTS}

    print(f"=== 回归测试: layout={args.layout}, noise={args.noise}, "
          f"bias_true ±{math.degrees(args.bias_max):.1f}° ===")

    # 生成 capture
    capture = _build_capture(layout_cfg, bias_true, args.noise, args.seed)
    capture_path = Path("/tmp") / f"mocap_regression_{args.layout}_{args.noise}.json"
    with open(capture_path, "w") as f:
        json.dump(capture, f)
    print(f"[1/3] 生成合成数据: {len(capture['samples'])} samples -> {capture_path}")

    # 跑优化
    print("[2/3] 运行 optimize_mocap (Ceres) ...")
    rc = _run_optimize(capture_path, args.layout)
    if rc != 0:
        print(f"[FAIL] optimize_mocap 返回码 {rc}")
        return 2

    # 对比
    calib_yaml = _MODULE_DIR / "output" / "calibration.yaml"
    if not calib_yaml.exists():
        print(f"[FAIL] 未生成 calibration.yaml: {calib_yaml}")
        return 2
    solved = _read_calibration_yaml(calib_yaml)

    print("[3/3] 对比 bias_true ...")
    errs = {}
    for j in FREE_JOINTS:
        errs[j] = abs(math.degrees(solved.get(j, 0.0)) - math.degrees(bias_true[j]))

    print(f"{'joint':<16}{'true(deg)':>10}{'solved(deg)':>12}{'err(deg)':>10}")
    for j in FREE_JOINTS:
        print(f"{j:<16}{math.degrees(bias_true[j]):>10.3f}{math.degrees(solved.get(j,0.0)):>12.3f}{errs[j]:>10.3f}")

    max_err = max(errs.values())
    mean_err = sum(errs.values()) / len(errs)
    print(f"\nmax |err| = {max_err:.3f} deg, mean |err| = {mean_err:.3f} deg")

    if max_err <= args.threshold:
        print(f"[PASS] bias 还原误差 ≤ {args.threshold}°")
        return 0
    print(f"[FAIL] bias 还原误差 {max_err:.3f}° > 阈值 {args.threshold}°")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
