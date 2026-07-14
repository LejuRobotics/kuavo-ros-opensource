#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
汇总所有 calibration_output*/mocap_axis_consistency/adjacent_axis_angles.csv，
生成类似对比表格并写入 Markdown。

表格结构（列为 pair i-j）：
- run_k_deviation (deg)
- mean / var / std / min / max
- plot（可选：生成 PNG 折线图并在 md 内嵌）
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd


@dataclass(frozen=True)
class RunCsv:
    run_name: str  # e.g. calibration_output_0
    run_index: int
    csv_path: Path


_RUN_RE = re.compile(r"calibration_output_(\d+)$")


def _discover_runs(root: Path) -> List[RunCsv]:
    runs: List[RunCsv] = []
    for p in root.glob("calibration_output_*"):
        if not p.is_dir():
            continue
        m = _RUN_RE.search(p.name)
        if not m:
            continue
        idx = int(m.group(1))
        csv_path = p / "mocap_axis_consistency" / "adjacent_axis_angles.csv"
        if not csv_path.is_file():
            continue
        runs.append(RunCsv(run_name=p.name, run_index=idx, csv_path=csv_path))
    runs.sort(key=lambda r: r.run_index)
    return runs


def _pair_label(i: int, j: int) -> str:
    return f"pair {i}-{j}"


def _load_one(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    required = {"joint_i", "joint_j", "deviation_deg"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"CSV 缺少列 {sorted(missing)}: {csv_path}")
    df = df.copy()
    df["joint_i"] = df["joint_i"].astype(int)
    df["joint_j"] = df["joint_j"].astype(int)
    df["deviation_deg"] = df["deviation_deg"].astype(float)
    df["pair"] = [
        _pair_label(int(i), int(j)) for i, j in zip(df["joint_i"].tolist(), df["joint_j"].tolist())
    ]
    return df[["pair", "deviation_deg"]]


def _collect_deviations(runs: Sequence[RunCsv]) -> Tuple[List[str], pd.DataFrame]:
    """
    返回:
      - pairs: 列名（pair i-j）
      - wide: index 为 run_0..N，columns 为 pairs，值为 deviation_deg
    """
    all_pairs: List[str] = []
    per_run: List[Tuple[int, Dict[str, float]]] = []
    for r in runs:
        df = _load_one(r.csv_path)
        d = dict(zip(df["pair"].tolist(), df["deviation_deg"].tolist()))
        per_run.append((r.run_index, d))
        for k in d.keys():
            if k not in all_pairs:
                all_pairs.append(k)

    # 让 pair 顺序更稳定：按数字排序（pair 3-4 在 pair 4-5 前）
    def pair_key(s: str) -> Tuple[int, int]:
        m = re.search(r"(\d+)-(\d+)", s)
        if not m:
            return (999, 999)
        return (int(m.group(1)), int(m.group(2)))

    pairs = sorted(all_pairs, key=pair_key)

    rows: List[Dict[str, float]] = []
    idx: List[str] = []
    for run_index, d in per_run:
        row = {p: float(d.get(p, np.nan)) for p in pairs}
        rows.append(row)
        idx.append(f"run_{run_index}")
    wide = pd.DataFrame(rows, index=idx, columns=pairs)
    return pairs, wide


def _stats_rows(wide: pd.DataFrame) -> Dict[str, pd.Series]:
    # 与截图一致：var/std 使用总体方差（ddof=0）
    values = wide.values.astype(float)
    stats = {
        "mean (deg)": pd.Series(np.nanmean(values, axis=0), index=wide.columns),
        "var (deg^2)": pd.Series(np.nanvar(values, axis=0, ddof=0), index=wide.columns),
        "std (deg)": pd.Series(np.nanstd(values, axis=0, ddof=0), index=wide.columns),
        "min (deg)": pd.Series(np.nanmin(values, axis=0), index=wide.columns),
        "max (deg)": pd.Series(np.nanmax(values, axis=0), index=wide.columns),
    }
    return stats


def _fmt(x: float) -> str:
    if x is None or (isinstance(x, float) and (np.isnan(x) or np.isinf(x))):
        return ""
    return f"{x:.3f}"


def _markdown_table(pairs: Sequence[str], wide: pd.DataFrame, plot_paths: Optional[Dict[str, Path]]) -> str:
    # header
    header = ["Metric", *pairs]
    sep = ["---"] * len(header)
    lines = ["| " + " | ".join(header) + " |", "| " + " | ".join(sep) + " |"]

    # run rows
    for run_name in wide.index.tolist():
        row = [f"{run_name}_deviation (deg)"]
        for p in pairs:
            row.append(_fmt(float(wide.loc[run_name, p])))
        lines.append("| " + " | ".join(row) + " |")

    # stats rows
    stats = _stats_rows(wide)
    for metric, s in stats.items():
        row = [metric]
        for p in pairs:
            row.append(_fmt(float(s[p])))
        lines.append("| " + " | ".join(row) + " |")

    # plot row
    if plot_paths is not None:
        row = ["plot"]
        for p in pairs:
            img = plot_paths.get(p)
            if img is None:
                row.append("")
            else:
                # md 里用相对路径更稳
                row.append(f"![]({img.as_posix()})")
        lines.append("| " + " | ".join(row) + " |")

    return "\n".join(lines) + "\n"


def _make_plots(
    pairs: Sequence[str],
    wide: pd.DataFrame,
    out_dir: Path,
    dpi: int = 140,
) -> Dict[str, Path]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return {}

    out_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams["axes.unicode_minus"] = False

    x = np.arange(len(wide.index))
    plot_paths: Dict[str, Path] = {}
    for p in pairs:
        y = wide[p].values.astype(float)
        fig = plt.figure(figsize=(3.0, 1.8), constrained_layout=True)
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(x, y, marker="o", linewidth=1.2, color="#4C72B0")
        ax.axhline(0.0, linestyle="--", linewidth=0.9, color="#C44E52", alpha=0.8)
        ax.set_title(p, fontsize=9)
        ax.set_xlabel("run", fontsize=8)
        ax.set_ylabel("deviation (deg)", fontsize=8)
        ax.set_xticks(x)
        ax.set_xticklabels([s.replace("run_", "") for s in wide.index.tolist()], fontsize=7)
        ax.tick_params(axis="y", labelsize=7)
        ax.grid(True, linestyle=":", alpha=0.4)
        fname = p.replace(" ", "_").replace("/", "_") + ".png"
        path = out_dir / fname
        fig.savefig(path, dpi=dpi)
        plt.close(fig)
        plot_paths[p] = path
    return plot_paths


def _rel_paths(plot_paths: Dict[str, Path], md_path: Path) -> Dict[str, Path]:
    base = md_path.parent.resolve()
    out: Dict[str, Path] = {}
    for k, p in plot_paths.items():
        try:
            out[k] = p.resolve().relative_to(base)
        except Exception:
            out[k] = p
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        description="汇总 calibration_output*/mocap_axis_consistency/adjacent_axis_angles.csv 并写入 Markdown"
    )
    ap.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="arm_accuracy_calibration 目录（默认脚本所在目录）",
    )
    ap.add_argument(
        "--out-md",
        type=Path,
        default=None,
        help="输出 Markdown 路径（默认：root/adjacent_axis_angles_summary.md）",
    )
    ap.add_argument(
        "--no-plots",
        action="store_true",
        help="不生成 plot 行和 PNG 图片",
    )
    ap.add_argument(
        "--plot-dpi",
        type=int,
        default=140,
        help="导出 PNG 的 DPI（默认 140）",
    )
    args = ap.parse_args()

    root = args.root.resolve()
    if args.out_md is None:
        out_md = root / "adjacent_axis_angles_summary.md"
    else:
        out_md = args.out_md.resolve()

    runs = _discover_runs(root)
    if not runs:
        raise SystemExit(f"未找到任何 calibration_output_*/.../adjacent_axis_angles.csv（root={root}）")

    pairs, wide = _collect_deviations(runs)
    if wide.empty or not pairs:
        raise SystemExit("未收集到有效 deviation 数据")

    plot_rel: Optional[Dict[str, Path]] = None
    if not args.no_plots:
        plot_dir = out_md.parent / "adjacent_axis_angles_plots"
        plot_abs = _make_plots(pairs, wide, plot_dir, dpi=args.plot_dpi)
        plot_rel = _rel_paths(plot_abs, out_md)

    md_lines: List[str] = []
    md_lines.append("## mocap_axis_consistency / adjacent_axis_angles 汇总\n")
    md_lines.append(f"- **root**: `{root}`")
    md_lines.append(f"- **runs**: {', '.join([r.run_name for r in runs])}\n")
    md_lines.append(_markdown_table(pairs, wide, plot_rel))

    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text("\n".join(md_lines), encoding="utf-8")
    print(f"已写入: {out_md}")


if __name__ == "__main__":
    main()

