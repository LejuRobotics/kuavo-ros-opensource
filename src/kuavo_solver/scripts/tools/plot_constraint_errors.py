#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
绘制 constraint position verify 的误差分布。

输入：`mujoco_unified_cli.py ... position-verify --json` 的输出 JSON。
输出：matplotlib 图窗（或保存到 --out）。
"""

from __future__ import annotations

import argparse
import json
from typing import Any, Dict, List, Tuple


def _collect(result: Dict[str, Any]) -> Tuple[List[str], List[float], List[float]]:
    cases = result.get("cases") or []
    if not isinstance(cases, list):
        raise ValueError("json 缺少 cases 列表")

    # point -> list(abs_max)
    per_point: Dict[str, List[float]] = {}
    overall_abs: List[float] = []

    for c in cases:
        if not isinstance(c, dict):
            continue
        ev = (c.get("err_vec") or {})
        if isinstance(ev, dict) and "abs_max" in ev:
            overall_abs.append(float(ev["abs_max"]))
        pts = c.get("points") or {}
        if not isinstance(pts, dict):
            continue
        for name, e in pts.items():
            if not isinstance(e, dict) or "abs_max" not in e:
                continue
            per_point.setdefault(str(name), []).append(float(e["abs_max"]))

    names = sorted(per_point.keys())
    medians = []
    p99 = []
    for n in names:
        xs = sorted(per_point[n])
        if not xs:
            medians.append(0.0)
            p99.append(0.0)
            continue
        medians.append(xs[len(xs) // 2])
        p99_idx = min(int(round(0.99 * (len(xs) - 1))), len(xs) - 1)
        p99.append(xs[p99_idx])
    return names, medians, p99


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="plot constraint position verify errors")
    ap.add_argument("json_path", help="position-verify --json 输出文件")
    ap.add_argument("--out", help="保存图片路径（png/pdf）；不填则只显示")
    args = ap.parse_args(argv)

    with open(args.json_path, "r", encoding="utf-8") as f:
        result = json.load(f)

    names, med, p99 = _collect(result)

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(max(10, 0.25 * len(names)), 5))
    x = list(range(len(names)))
    ax.plot(x, med, label="median(abs_max)", linewidth=2)
    ax.plot(x, p99, label="p99(abs_max)", linewidth=2)
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=75, ha="right", fontsize=8)
    ax.set_ylabel("abs_max (m)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    if args.out:
        fig.savefig(args.out, dpi=200)
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

