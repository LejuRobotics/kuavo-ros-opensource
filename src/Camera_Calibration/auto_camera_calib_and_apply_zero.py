#!/usr/bin/env python3
"""
一键相机标定接口（按 layout 分流 52 / 62）：
1) 自动对「头部 + 左右手」三个 demo 进行采样（capture -> 写 CSV）
2) 自动进行优化（optimize -> 读 CSV 输出 calibration.yaml / calibrated URDF）
3) 打印优化得到的关节修正量（dry-run 零点脚本）
4) 提示是否按 's' 保存优化结果；若按 s，则把修正量应用到零点文件

依赖：
- src/Camera_Calibration/run_chessboard_calibration.sh
- biped52: apply_zero_deltas_to_arms_zero.py
- wheel62: apply_zero_deltas_to_arms_zero_wheel62.py
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


def _ws_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _cc_dir() -> Path:
    return _ws_root() / "src" / "Camera_Calibration"


def _resolve_robot_layout(arg: str) -> str:
    """解析 robot_layout：auto 时读 ROBOT_VERSION。"""
    if arg in ("biped52", "wheel62"):
        return arg
    rv = os.environ.get("ROBOT_VERSION", "").strip()
    if rv == "52":
        return "biped52"
    if rv in ("62", "63"):
        return "wheel62"
    if rv:
        print(f"[WARN] 未知 ROBOT_VERSION={rv}，回退 biped52", file=sys.stderr)
    return "biped52"


def _run_chessboard(stage: str, robot_layout: str) -> int:
    """调用 run_chessboard_calibration.sh 的 capture/optimize（固定选 4：三路并行）。"""
    script = _cc_dir() / "run_chessboard_calibration.sh"
    if not script.exists():
        print(f"[ERROR] 找不到脚本: {script}", file=sys.stderr)
        return 2

    cmd = [str(script), stage, "--robot_layout", robot_layout]
    p = subprocess.run(
        cmd,
        input="4\n",
        text=True,
        cwd=str(_ws_root()),
        env=os.environ.copy(),
    )
    return int(p.returncode)


def _apply_script_for_layout(robot_layout: str) -> Path:
    if robot_layout == "wheel62":
        return _cc_dir() / "apply_zero_deltas_to_arms_zero_wheel62.py"
    return _cc_dir() / "apply_zero_deltas_to_arms_zero.py"


def _apply_deltas(dry_run: bool, robot_layout: str) -> int:
    apply_py = _apply_script_for_layout(robot_layout)
    if not apply_py.exists():
        print(f"[ERROR] 找不到脚本: {apply_py}", file=sys.stderr)
        return 2

    cmd = [sys.executable, str(apply_py)]
    if dry_run:
        cmd.append("--dry-run")
    p = subprocess.run(cmd, cwd=str(_ws_root()), env=os.environ.copy())
    return int(p.returncode)


def main() -> int:
    ap = argparse.ArgumentParser(description="一键相机标定 + 零点回写（52/62 分流）")
    ap.add_argument(
        "--robot_layout",
        default="auto",
        choices=["auto", "biped52", "wheel62"],
        help="机型布局；auto 时读 ROBOT_VERSION（52→biped52，62/63→wheel62）",
    )
    args = ap.parse_args()
    robot_layout = _resolve_robot_layout(args.robot_layout)

    print("\n========== 相机标定一键流程（capture→optimize→提示写入零点） ==========")
    print(f"[INFO] robot_layout: {robot_layout}")

    print("[INFO] 1/4 采样 capture（头部 + 左右手）...")
    rc = _run_chessboard("capture", robot_layout)
    if rc != 0:
        print(f"[ERROR] capture 失败 (rc={rc})", file=sys.stderr)
        print(
            "[HINT] 常见原因：某一路相机话题没数据（尤其 /right_wrist_camera/color/image_raw）。\n"
            "       请先确认三路相机话题都在发布后再重试。",
            file=sys.stderr,
        )
        return rc

    print("[INFO] 2/4 优化 optimize（head→right→left）...")
    rc = _run_chessboard("optimize", robot_layout)
    if rc != 0:
        print(f"[ERROR] optimize 失败 (rc={rc})", file=sys.stderr)
        return rc

    apply_name = _apply_script_for_layout(robot_layout).name
    print(f"[INFO] 3/4 打印优化结果（{apply_name} --dry-run）...")
    rc = _apply_deltas(dry_run=True, robot_layout=robot_layout)
    if rc != 0:
        print(f"[ERROR] 打印修正量失败 (rc={rc})", file=sys.stderr)
        return rc

    print("\n按 's' 保存优化结果并写入零点文件，其它键跳过。")
    choice = input("输入: ").strip().lower()
    if choice == "s":
        print(f"[INFO] 4/4 写入零点偏置（{apply_name}，自动备份）...")
        rc = _apply_deltas(dry_run=False, robot_layout=robot_layout)
        if rc != 0:
            print(f"[ERROR] 写入零点偏置失败 (rc={rc})", file=sys.stderr)
            return rc
        print("[INFO] 写入完成。")
    else:
        print("[INFO] 已跳过写入。")

    print("========== 完成 ==========\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

