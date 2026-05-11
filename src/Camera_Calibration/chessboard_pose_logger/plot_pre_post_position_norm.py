#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


@dataclass
class Series:
    name: str
    t: List[float]
    norm: List[float]
    kf: List[Optional[int]]


def _expected_keyframe_count(cam: str) -> Optional[int]:
    """从 teach JSON 推断关键帧数量，用于绘图对齐（即使某些关键帧未采到也要占位）。"""
    cc_dir = Path("src/Camera_Calibration").resolve()
    teach_dir = (cc_dir / "teach_capture_output").resolve()
    if cam == "head":
        p = teach_dir / "teach_head_joint.json"
        key_ok = lambda s: isinstance(s, dict) and ("head_yaw" in s) and ("head_pitch" in s)
    elif cam == "left_wrist":
        p = teach_dir / "teach_left_joint.json"
        key_ok = lambda s: isinstance(s, dict) and ("left_arm_joints" in s)
    elif cam == "right_wrist":
        p = teach_dir / "teach_right_joint.json"
        key_ok = lambda s: isinstance(s, dict) and ("right_arm_joints" in s)
    else:
        return None

    if not p.is_file():
        return None
    data = json.loads(p.read_text(encoding="utf-8", errors="ignore"))
    samples = data.get("samples", [])
    n = 0
    for s in samples:
        if key_ok(s):
            n += 1
    if n <= 0:
        return None
    # 左右手采样阶段默认去掉首尾关键帧（第1/最后1），因此有效关键帧数为 N-2
    if cam in ("left_wrist", "right_wrist"):
        return max(0, n - 2) or None
    return n


def _pad_or_truncate(values: List[float], n: int) -> List[float]:
    if n <= 0:
        return []
    if len(values) >= n:
        return list(values[:n])
    return list(values) + [float("nan")] * (n - len(values))


def _read_position_norm_series(csv_path: Path) -> Series:
    if not csv_path.is_file():
        raise FileNotFoundError(str(csv_path))

    t: List[float] = []
    norm: List[float] = []
    kf: List[Optional[int]] = []

    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        required = {"ros_time", "t_x", "t_y", "t_z"}
        if reader.fieldnames is None or not required.issubset(set(reader.fieldnames)):
            raise ValueError(f"CSV 列缺失（需要 {sorted(required)}），实际: {reader.fieldnames}")

        for row in reader:
            try:
                tt = float(row["ros_time"])
                tx = float(row["t_x"])
                ty = float(row["t_y"])
                tz = float(row["t_z"])
            except Exception:
                continue
            t.append(tt)
            # CSV 的 t_x/t_y/t_z 单位为米；这里统一转为 mm 以便更直观对比
            norm.append(math.sqrt(tx * tx + ty * ty + tz * tz) * 1000.0)
            if "keyframe_index" in row and row.get("keyframe_index"):
                try:
                    kk = int(row["keyframe_index"])
                    kf.append(kk if kk >= 1 else None)
                except Exception:
                    kf.append(None)
            else:
                kf.append(None)

    if not t:
        raise ValueError(f"CSV 无有效数据行: {csv_path}")

    # normalize time to start at 0 for nicer plotting
    t0 = t[0]
    t = [x - t0 for x in t]
    return Series(name=csv_path.stem, t=t, norm=norm, kf=kf)


def _basic_stats(xs: List[float]) -> Dict[str, float]:
    if not xs:
        return {"n": 0.0}
    ys = sorted(xs)
    n = len(ys)

    def q(p: float) -> float:
        if n == 1:
            return ys[0]
        i = p * (n - 1)
        lo = int(math.floor(i))
        hi = int(math.ceil(i))
        if lo == hi:
            return ys[lo]
        r = i - lo
        return ys[lo] * (1.0 - r) + ys[hi] * r

    mean = sum(ys) / n
    var = sum((y - mean) ** 2 for y in ys) / max(1, n - 1)
    return {
        "n": float(n),
        "min": ys[0],
        "p50": q(0.5),
        "p90": q(0.9),
        "max": ys[-1],
        "mean": mean,
        "std": math.sqrt(var),
    }


def _plot_one_camera_keyframe_bars(cam: str, pre: Series, post: Series, out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    out_png = out_dir / f"pos_norm_keyframes_{cam}_pre_post.png"

    # 按 teach JSON 关键帧数对齐：缺失关键帧也占位（NaN），避免错位
    n_expected = _expected_keyframe_count(cam) or max(len(pre.norm), len(post.norm))

    # 若 CSV 里带 keyframe_index，则按 index 落位；否则退化为顺序落位
    pre_vals = [float("nan")] * n_expected
    post_vals = [float("nan")] * n_expected
    if any(k is not None for k in pre.kf):
        for v, k in zip(pre.norm, pre.kf):
            if k is None:
                continue
            if 1 <= k <= n_expected:
                pre_vals[k - 1] = float(v)
    else:
        pre_vals = _pad_or_truncate(pre.norm, n_expected)

    if any(k is not None for k in post.kf):
        for v, k in zip(post.norm, post.kf):
            if k is None:
                continue
            if 1 <= k <= n_expected:
                post_vals[k - 1] = float(v)
    else:
        post_vals = _pad_or_truncate(post.norm, n_expected)

    pre_valid = [v for v in pre_vals if not math.isnan(v)]
    post_valid = [v for v in post_vals if not math.isnan(v)]
    pre_s = _basic_stats(pre_valid)
    post_s = _basic_stats(post_valid)

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(1, 1, 1)
    # 柱状图：横坐标=关键帧序号(1..N_expected)，纵坐标=||t||（mm）
    n = int(n_expected)
    x = list(range(1, n + 1))
    width = 0.38
    xp = [xi - width / 2 for xi in x]
    xq = [xi + width / 2 for xi in x]
    ax.bar(xp, pre_vals, width=width, color="red", alpha=0.65, label=f"pre (valid={len(pre_valid)}/{n})")
    ax.bar(xq, post_vals, width=width, color="blue", alpha=0.65, label=f"post (valid={len(post_valid)}/{n})")

    ax.set_title(f"{cam}: ||t|| per keyframe (pre vs post)")
    ax.set_xlabel("keyframe index")
    ax.set_ylabel("||t|| (mm)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    ax.set_xticks(x)

    # 纵轴不从 0 开始：根据数据范围自适应缩放，增强可读性
    all_vals = [v for v in (pre_vals + post_vals) if not math.isnan(v)]
    if all_vals:
        y_min = min(all_vals)
        y_max = max(all_vals)
        y_range = max(1e-9, y_max - y_min)
        pad = 0.08 * y_range
        ax.set_ylim(y_min - pad, y_max + pad)

    txt = (
        f"pre : mean={pre_s['mean']:.4f} std={pre_s['std']:.4f} p50={pre_s['p50']:.4f}\n"
        f"post: mean={post_s['mean']:.4f} std={post_s['std']:.4f} p50={post_s['p50']:.4f}"
    )
    ax.text(
        0.01,
        0.01,
        txt,
        transform=ax.transAxes,
        fontsize=9,
        va="bottom",
        ha="left",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8, edgecolor="none"),
    )

    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)
    return out_png


def _plot_one_camera_hist(cam: str, pre: Series, post: Series, out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    out_png = out_dir / f"pos_norm_hist_{cam}_pre_post.png"

    pre_s = _basic_stats(pre.norm)
    post_s = _basic_stats(post.norm)

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(1, 1, 1)
    ax.hist(pre.norm, bins=60, alpha=0.55, color="red", label="pre", density=True)
    ax.hist(post.norm, bins=60, alpha=0.55, color="blue", label="post", density=True)
    ax.set_title(f"{cam}: ||t|| histogram (pre vs post)")
    ax.set_xlabel("||t|| (mm)")
    ax.set_ylabel("density")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")

    txt = (
        f"pre : n={int(pre_s['n'])} mean={pre_s['mean']:.4f} std={pre_s['std']:.4f} p50={pre_s['p50']:.4f} p90={pre_s['p90']:.4f}\n"
        f"post: n={int(post_s['n'])} mean={post_s['mean']:.4f} std={post_s['std']:.4f} p50={post_s['p50']:.4f} p90={post_s['p90']:.4f}"
    )
    ax.text(
        0.01,
        0.01,
        txt,
        transform=ax.transAxes,
        fontsize=9,
        va="bottom",
        ha="left",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8, edgecolor="none"),
    )

    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)
    return out_png


def main() -> int:
    ap = argparse.ArgumentParser(description="读取 chessboard_pose_logger 的 pre/post CSV，绘制关键帧序号-位置模长 ||t|| 对比图")
    ap.add_argument(
        "--base_dir",
        type=str,
        default="src/Camera_Calibration/output_csv/chessboard_pose",
        help="包含 head/left_wrist/right_wrist 子目录的根目录",
    )
    ap.add_argument("--out_dir", type=str, default="src/Camera_Calibration/output/chessboard_pose_norm_keyframes", help="输出图片目录")
    args = ap.parse_args()

    base_dir = Path(args.base_dir).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()

    cams = ["head", "left_wrist", "right_wrist"]
    ok_any = False

    for cam in cams:
        pre_csv = base_dir / cam / f"board_pose_{cam}_pre.csv"
        post_csv = base_dir / cam / f"board_pose_{cam}_post.csv"
        if not pre_csv.is_file() or not post_csv.is_file():
            continue

        pre = _read_position_norm_series(pre_csv)
        post = _read_position_norm_series(post_csv)
        _plot_one_camera_keyframe_bars(cam, pre, post, out_dir)
        ok_any = True

    if not ok_any:
        raise SystemExit(
            f"未找到可用的 pre/post CSV。\n"
            f"期望存在: <base_dir>/<cam>/board_pose_<cam>_pre.csv 和 board_pose_<cam>_post.csv\n"
            f"当前 base_dir={base_dir}"
        )

    print(f"[OK] saved plots to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

