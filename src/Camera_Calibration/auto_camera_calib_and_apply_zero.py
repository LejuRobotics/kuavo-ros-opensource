#!/usr/bin/env python3
"""
一键相机标定接口（按你的流程封装）：
1) 自动对「头部 + 左右手」三个 demo 进行采样（capture -> 写 CSV）
2) 自动进行优化（optimize -> 读 CSV 输出 calibration.yaml / calibrated URDF）
3) 打印优化得到的关节修正量（调用 apply_zero_deltas_to_arms_zero.py --dry-run）
4) 提示是否按 's' 保存优化结果；若按 s，则把修正量应用到零点文件（Ruiwo arms_zero.yaml + EC offset.csv）

依赖：
- src/Camera_Calibration/run_chessboard_calibration.sh
- src/Camera_Calibration/apply_zero_deltas_to_arms_zero.py

注意：
- 若右腕相机话题未发布（/right_wrist_camera/color/image_raw），采样阶段会按原脚本预检失败并退出。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


def _ws_root() -> Path:
    # .../src/Camera_Calibration/auto_camera_calib_and_apply_zero.py -> workspace root
    return Path(__file__).resolve().parents[2]


def _cc_dir() -> Path:
    return _ws_root() / "src" / "Camera_Calibration"


def _run_chessboard(stage: str) -> int:
    """
    调用 run_chessboard_calibration.sh 的 capture/optimize。
    该脚本会询问 demo 选择，这里固定回答 4（all_parallel: 头 + 左右手）。
    """
    script = _cc_dir() / "run_chessboard_calibration.sh"
    if not script.exists():
        print(f"[ERROR] 找不到脚本: {script}", file=sys.stderr)
        return 2

    p = subprocess.run(
        [str(script), stage],
        input="4\n",
        text=True,
        cwd=str(_ws_root()),
        env=os.environ.copy(),
    )
    return int(p.returncode)


def _apply_deltas(dry_run: bool) -> int:
    # 精简版：不传任何“符号/方向”参数，全部在 apply_zero_deltas_to_arms_zero.py 内硬编码
    apply_py = _cc_dir() / "apply_zero_deltas_to_arms_zero.py"
    if not apply_py.exists():
        print(f"[ERROR] 找不到脚本: {apply_py}", file=sys.stderr)
        return 2

    cmd = [sys.executable, str(apply_py)]
    if dry_run:
        cmd.append("--dry-run")
    p = subprocess.run(cmd, cwd=str(_ws_root()), env=os.environ.copy())
    return int(p.returncode)


def main() -> int:
    print("\n========== 相机标定一键流程（capture→optimize→提示写入零点） ==========")

    print("[INFO] 1/4 采样 capture（头部 + 左右手）...")
    rc = _run_chessboard("capture")
    if rc != 0:
        print(f"[ERROR] capture 失败 (rc={rc})", file=sys.stderr)
        print(
            "[HINT] 常见原因：某一路相机话题没数据（尤其 /right_wrist_camera/color/image_raw）。\n"
            "       请先确认三路相机话题都在发布后再重试。",
            file=sys.stderr,
        )
        return rc

    print("[INFO] 2/4 优化 optimize（head→right→left）...")
    rc = _run_chessboard("optimize")
    if rc != 0:
        print(f"[ERROR] optimize 失败 (rc={rc})", file=sys.stderr)
        return rc

    print("[INFO] 3/4 打印优化结果（关节修正量，dry-run）...")
    rc = _apply_deltas(dry_run=True)
    if rc != 0:
        print(f"[ERROR] 打印修正量失败 (rc={rc})", file=sys.stderr)
        return rc

    print("\n按 's' 保存优化结果并写入零点文件，其它键跳过。")
    choice = input("输入: ").strip().lower()
    if choice == "s":
        print("[INFO] 4/4 写入零点偏置（自动备份）...")
        rc = _apply_deltas(dry_run=False)
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

