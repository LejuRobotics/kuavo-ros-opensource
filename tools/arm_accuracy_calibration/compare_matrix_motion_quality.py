#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取一批 matrix_data_motion（当前批次），计算平移速度/加速度/jerk 的 RMS，
并与固定基准 0318 的数值做对比。
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd

DEFAULT_JOINT_FILES: List[Tuple[str, str]] = [
    ("joint_1", "T_joint_2_in_joint_1.csv"),
    ("joint_2", "T_joint_3_in_joint_2.csv"),
    ("joint_3", "T_joint_4_in_joint_3.csv"),
    ("joint_4", "T_joint_5_in_joint_4.csv"),
    ("joint_5", "T_joint_6_in_joint_5.csv"),
    ("joint_6", "T_joint_7_in_joint_6.csv"),
]

# 固定基准：0318（用于与当前批次对比）
# 单位分别为 mm/s、mm/s^2、mm/s^3
BASELINE_0318: Dict[str, Dict[str, float]] = {
    "joint_1": {"trans_rms_vel": 5.0, "trans_rms_acc": 700.0, "trans_rms_jerk": 8.5e4},
    "joint_2": {"trans_rms_vel": 11.0, "trans_rms_acc": 1800.0, "trans_rms_jerk": 2.4e5},
    "joint_3": {"trans_rms_vel": 15.0, "trans_rms_acc": 2200.0, "trans_rms_jerk": 3.0e5},
    "joint_4": {"trans_rms_vel": 10.0, "trans_rms_acc": 1400.0, "trans_rms_jerk": 1.6e5},
    "joint_5": {"trans_rms_vel": 14.0, "trans_rms_acc": 1600.0, "trans_rms_jerk": 2.0e5},
    "joint_6": {"trans_rms_vel": 4.0, "trans_rms_acc": 650.0, "trans_rms_jerk": 7.5e4},
}


def _script_dir() -> Path:
    return Path(__file__).resolve().parent


def load_csv(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    df = pd.read_csv(path)
    t = df["timestamp"].values.astype(np.float64)
    p = df[["m03", "m13", "m23"]].values.astype(np.float64)
    return t, p


def metrics(t: np.ndarray, p: np.ndarray) -> Optional[Dict[str, Any]]:
    n = len(t)
    if n < 5:
        return None
    dt = np.diff(t)
    dt = np.maximum(dt, 1e-9)
    v = np.diff(p, axis=0) / dt[:, None]
    rms_vel = float(np.sqrt(np.mean(np.sum(v * v, axis=1))))
    denom_a = (t[2:] - t[:-2])[:, None] / 2.0
    denom_a = np.maximum(denom_a, 1e-9)
    a = (v[1:] - v[:-1]) / denom_a
    rms_acc = float(np.sqrt(np.mean(np.sum(a * a, axis=1))))
    if len(a) >= 2:
        dt_j = (t[3:] - t[:-3])[:, None] / 2.0
        dt_j = np.maximum(dt_j, 1e-9)
        j = (a[1:] - a[:-1]) / dt_j
        rms_jerk = float(np.sqrt(np.mean(np.sum(j * j, axis=1))))
    else:
        rms_jerk = float("nan")

    return {
        "n": n,
        "dt_med_ms": float(np.median(dt) * 1000),
        "trans_rms_vel": rms_vel,
        "trans_rms_acc": rms_acc,
        "trans_rms_jerk": rms_jerk,
    }


def compute_current_batch(
    matrix_dir: Path,
    pairs: List[Tuple[str, str]],
    current_label: str,
) -> pd.DataFrame:
    rows: List[Dict[str, Any]] = []
    for jdir, fname in pairs:
        path = matrix_dir / jdir / fname
        if not path.is_file():
            continue
        t, p = load_csv(path)
        m = metrics(t, p)
        if not m:
            continue
        m["joint"] = jdir
        m["file"] = fname
        m["batch"] = current_label
        rows.append(m)
    return pd.DataFrame(rows)


def _joint_order(joints: Sequence[str]) -> List[str]:
    def key(j: str) -> Tuple[int, str]:
        try:
            return (int(j.split("_")[1]), j)
        except (IndexError, ValueError):
            return (999, j)

    return sorted(set(joints), key=key)


def save_motion_quality_plots(
    df: pd.DataFrame,
    label_a: str,
    label_b: str,
    plot_dir: Path,
    dpi: int = 140,
) -> List[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["axes.unicode_minus"] = False
    plot_dir.mkdir(parents=True, exist_ok=True)
    joints_all = _joint_order(df["joint"].tolist())
    joints = []
    for j in joints_all:
        has_a = np.any((df["joint"] == j) & (df["batch"] == label_a))
        has_b = np.any((df["joint"] == j) & (df["batch"] == label_b))
        if has_a and has_b:
            joints.append(j)
    if not joints:
        raise ValueError("没有可绘图的共同关节（需同时有基准与当前数据）")
    x = np.arange(len(joints))
    width = 0.36
    la, lb = label_a, label_b

    # Translation only: velocity → acceleration → jerk (left to right).
    # Tuple: (column, summary title, ratio-panel title, log y on summary)
    trans_specs: List[Tuple[str, str, str, bool]] = [
        ("trans_rms_vel", "Translation RMS velocity", "Translation RMS velocity", False),
        ("trans_rms_acc", "Translation RMS acceleration", "Translation RMS acceleration", False),
        ("trans_rms_jerk", "Translation RMS jerk (log y)", "Translation RMS jerk", True),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(12, 4.2), constrained_layout=True)
    axes_flat = np.atleast_1d(axes).ravel()
    for ax, (col, title, _ratio_title, use_log) in zip(axes_flat, trans_specs):
        va = [df.loc[(df["joint"] == j) & (df["batch"] == la), col].values[0] for j in joints]
        vb = [df.loc[(df["joint"] == j) & (df["batch"] == lb), col].values[0] for j in joints]
        ax.bar(x - width / 2, va, width, label=la, color="#4C72B0", alpha=0.9)
        ax.bar(x + width / 2, vb, width, label=lb, color="#DD8452", alpha=0.9)
        ax.set_xticks(x)
        ax.set_xticklabels([j.replace("joint_", "J") for j in joints])
        ax.set_title(title, fontsize=10)
        if use_log:
            ax.set_yscale("log")
        ax.grid(axis="y", linestyle=":", alpha=0.5)
        ax.legend(fontsize=8, loc="upper left")
    fig.suptitle(
        "matrix_data_motion — translation: RMS velocity / acceleration / jerk",
        fontsize=12,
    )
    summary_path = plot_dir / "motion_quality_summary.png"
    fig.savefig(summary_path, dpi=dpi)
    plt.close(fig)

    p = df.pivot(index="joint", columns="batch", values=[c[0] for c in trans_specs])
    p = p.reindex(joints)
    ratio_cols = [c[0] for c in trans_specs]
    ratio_title_by_col = {c: rt for c, _st, rt, _lg in trans_specs}
    fig2, axes2 = plt.subplots(1, 3, figsize=(12, 4.2), constrained_layout=True)
    for ax, col in zip(np.atleast_1d(axes2).ravel(), ratio_cols):
        ra = p[col][la].values.astype(float)
        rb = p[col][lb].values.astype(float)
        ratio = rb / np.maximum(ra, 1e-30)
        ax.bar(x, ratio, color="#55A868", edgecolor="#333333", linewidth=0.4)
        ax.axhline(1.0, color="#C44E52", linestyle="--", linewidth=1.0, label="=1")
        ax.set_xticks(x)
        ax.set_xticklabels([j.replace("joint_", "J") for j in joints])
        ax.set_title(f"{ratio_title_by_col[col]} ratio ({lb}/{la})", fontsize=10)
        ax.grid(axis="y", linestyle=":", alpha=0.5)
        rmax = float(np.nanmax(ratio))
        if np.isfinite(rmax) and rmax > 0:
            ax.set_ylim(0, max(1.08, rmax * 1.12))
        ax.legend(fontsize=7, loc="upper right")
    fig2.suptitle(
        f"Translation vel / acc / jerk: ratio {lb}/{la} (>1 → {lb} larger)",
        fontsize=12,
    )
    ratio_path = plot_dir / "motion_quality_ratio.png"
    fig2.savefig(ratio_path, dpi=dpi)
    plt.close(fig2)

    return [summary_path, ratio_path]


def _baseline_df(label_a: str, pairs: List[Tuple[str, str]]) -> pd.DataFrame:
    rows: List[Dict[str, Any]] = []
    for jdir, fname in pairs:
        if jdir not in BASELINE_0318:
            continue
        b = BASELINE_0318[jdir]
        rows.append(
            {
                "n": np.nan,
                "dt_med_ms": np.nan,
                "trans_rms_vel": float(b["trans_rms_vel"]),
                "trans_rms_acc": float(b["trans_rms_acc"]),
                "trans_rms_jerk": float(b["trans_rms_jerk"]),
                "joint": jdir,
                "file": fname,
                "batch": label_a,
            }
        )
    return pd.DataFrame(rows)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="读取当前 matrix_data_motion，并与 0318 基准值对比"
    )
    ap.add_argument(
        "--data-dir",
        type=Path,
        default=_script_dir() / "calibration_output" / "matrix_data_motion",
        help="当前批次 matrix_data_motion 目录",
    )
    ap.add_argument("--label-a", default="0318", help="基准标签（默认 0318）")
    ap.add_argument("--label-b", default="current", help="当前批次标签（默认 current）")
    ap.add_argument(
        "--out-csv",
        type=Path,
        default=None,
        help="可选：将长表写入该 CSV",
    )
    ap.add_argument(
        "--plot-dir",
        type=Path,
        default=None,
        help="保存对比图 PNG 的目录（默认：脚本目录下 motion_quality_plots）",
    )
    ap.add_argument(
        "--no-plot",
        action="store_true",
        help="不生成图片",
    )
    ap.add_argument(
        "--plot-dpi",
        type=int,
        default=140,
        help="导出 PNG 的 DPI（默认 140）",
    )
    args = ap.parse_args()
    data_dir = args.data_dir.resolve()
    if not data_dir.is_dir():
        raise SystemExit(f"目录不存在: {data_dir}")

    df_current = compute_current_batch(data_dir, DEFAULT_JOINT_FILES, args.label_b)
    df_base = _baseline_df(args.label_a, DEFAULT_JOINT_FILES)
    df = pd.concat([df_base, df_current], ignore_index=True)
    if df.empty:
        raise SystemExit("未找到可用数据（检查 matrix_data_motion 下 joint_1..6 CSV）")

    metric_cols = [
        "trans_rms_vel",
        "trans_rms_acc",
        "trans_rms_jerk",
        "dt_med_ms",
        "n",
    ]
    wide = df.pivot(index=["joint", "file"], columns="batch", values=metric_cols)
    pd.set_option("display.width", 200)
    pd.set_option("display.max_columns", 20)
    print(wide.to_string())
    print()

    # 当前/0318 比值：>1 表示当前更大（通常更抖）
    p = df.pivot(index="joint", columns="batch", values=metric_cols)
    la, lb = args.label_a, args.label_b
    print(f"--- 比值 {lb}/{la}（速度/加速度/jerk；>1 表示 {lb} 更大）---")
    for c in metric_cols:
        if c in ("n", "dt_med_ms"):
            continue
        if la not in p[c].columns or lb not in p[c].columns:
            continue
        ratio = p[c][lb] / p[c][la]
        print(f"{c}:\n{ratio}\n")

    if args.out_csv:
        args.out_csv.parent.mkdir(parents=True, exist_ok=True)
        df.to_csv(args.out_csv, index=False)
        print(f"已写入: {args.out_csv}")

    if not args.no_plot:
        plot_dir = args.plot_dir
        if plot_dir is None:
            plot_dir = _script_dir() / "motion_quality_plots"
        try:
            paths = save_motion_quality_plots(
                df, args.label_a, args.label_b, plot_dir.resolve(), dpi=args.plot_dpi
            )
            for p in paths:
                print(f"已写入图表: {p}")
        except ImportError as e:
            raise SystemExit(
                "绘图需要 matplotlib，请安装: pip install matplotlib"
            ) from e


if __name__ == "__main__":
    main()
