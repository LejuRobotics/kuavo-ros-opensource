#!/usr/bin/env python3
"""
一键相机标定接口（头部 + 左右腕三路；按基础型号分流）：

机型处理与其它标定入口一致——``ROBOT_VERSION`` 的小版本按 ``%100`` 折回基础型号
（100049→49、200053→53、500062→62 …），布局/标定 URDF/零点脚本统一查
``robot_versions.py`` 的机型表：

- 基础 52 / 53 / 55 / 56：``biped52`` / ``biped56`` 布局，零点走
  ``apply_zero_deltas_to_arms_zero.py``（内部按版本选 14/16 维与 EC 归属）
- 基础 62 / 63：``wheel62`` 布局，零点走 ``apply_zero_deltas_to_arms_zero_wheel62.py``
- 基础 45 / 49：**只支持头部**（腕相机帧不匹配），请用
  ``run_chessboard_calibration.sh --demo head`` 采集/优化，再单独跑零点脚本

流程：
1) 自动对「头部 + 左右手」三个 demo 进行采样（capture -> 写 CSV）
2) 自动进行优化（optimize -> 读 CSV 输出 calibration.yaml / calibrated URDF）
3) 打印优化得到的关节修正量（dry-run 零点脚本）
4) 提示是否按 's' 保存优化结果；若按 s，则把修正量应用到零点文件
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import robot_versions as rv  # noqa: E402  (同目录模块，需先插入 sys.path)


def _ws_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _cc_dir() -> Path:
    return _ws_root() / "src" / "Camera_Calibration"


def _raw_version() -> str:
    return os.environ.get("ROBOT_VERSION", "").strip()


def _resolve_robot_layout(arg: str) -> str:
    """解析 robot_layout：显式覆盖优先，否则按 ROBOT_VERSION 折回基础型号查表。"""
    if arg in ("biped45", "biped52", "biped56", "wheel62"):
        return arg
    rv_version = _raw_version()
    if not rv_version:
        print("[WARN] 未设置 ROBOT_VERSION，按 biped52 处理；请显式 export ROBOT_VERSION", file=sys.stderr)
        return "biped52"
    spec = rv.spec(rv_version)
    if spec is None:
        print(
            f"[WARN] ROBOT_VERSION={rv_version} 不在受支持机型表内"
            f"（支持 {rv.supported_text()} 及对应小版本），回退 biped52",
            file=sys.stderr,
        )
        return "biped52"
    return spec.layout


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
    # wheel62/63 为 Motorevo 16 维，单独走轮臂脚本；
    # biped45/52/56 共用入口，脚本内部按版本区分 14 维+EC 与 16 维 Ruiwo。
    if robot_layout == "wheel62":
        return _cc_dir() / "apply_zero_deltas_to_arms_zero_wheel62.py"
    return _cc_dir() / "apply_zero_deltas_to_arms_zero.py"


def _apply_deltas(dry_run: bool, robot_layout: str) -> int:
    apply_py = _apply_script_for_layout(robot_layout)
    if not apply_py.exists():
        print(f"[ERROR] 找不到脚本: {apply_py}", file=sys.stderr)
        return 2

    cmd = [sys.executable, str(apply_py)]
    # 把原始版本号（可能是 100049 这类小版本）透传给零点脚本，让它自己折回查表；
    # 未设置时按布局回退到基础型号，避免零点脚本要求环境变量。
    robot_version = _raw_version()
    if not robot_version:
        robot_version = {"biped45": "45", "biped52": "52", "biped56": "56", "wheel62": "62"}.get(
            robot_layout, ""
        )
    if robot_version:
        cmd.extend(["--robot-version", robot_version])
    if dry_run:
        cmd.append("--dry-run")
    p = subprocess.run(cmd, cwd=str(_ws_root()), env=os.environ.copy())
    return int(p.returncode)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="一键相机标定 + 零点回写（头部 + 左右腕三路；支持 52/53/55/56/62/63 及小版本）"
    )
    ap.add_argument(
        "--robot_layout",
        default="auto",
        choices=["auto", "biped45", "biped52", "biped56", "wheel62"],
        help=(
            "机型布局；auto 时按 ROBOT_VERSION 折回基础型号查表"
            "（45/49→biped45，52→biped52，53/55/56→biped56，62/63→wheel62）"
        ),
    )
    args = ap.parse_args()
    robot_layout = _resolve_robot_layout(args.robot_layout)

    # 45/49 只有头部流程：本包装脚本固定跑三路，必须先拦住，避免用错布局继续跑。
    if robot_layout == "biped45":
        spec = rv.spec(_raw_version()) if _raw_version() else None
        base = spec.base if spec else "45/49"
        apply_script = spec.apply_script if spec else "apply_zero_deltas_to_arms_zero.py"
        print(
            f"[ERROR] 基础型号 {base}（布局 biped45）不支持左右腕三路相机标定。\n"
            "        请改用：\n"
            "          bash src/Camera_Calibration/run_chessboard_calibration.sh capture  --demo head\n"
            "          bash src/Camera_Calibration/run_chessboard_calibration.sh optimize --demo head\n"
            f"          python3 src/Camera_Calibration/{apply_script} --dry-run",
            file=sys.stderr,
        )
        return 2

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
