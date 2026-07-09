#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def parse_origin(el):
    if el is None:
        return np.eye(3), np.zeros(3)
    xyz = [float(x) for x in el.get("xyz", "0 0 0").split()]
    rpy = [float(x) for x in el.get("rpy", "0 0 0").split()]
    return rpy_to_R(rpy[0], rpy[1], rpy[2]), np.array(xyz, dtype=float)


def parse_axis(joint_el):
    ax = joint_el.find("axis")
    if ax is None:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    v = np.array([float(x) for x in ax.get("xyz", "0 0 1").split()], dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.array([0.0, 0.0, 1.0], dtype=float)


def rodrigues(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ],
        dtype=float,
    )


def joint_T(q, joint_el):
    jtype = joint_el.get("type", "fixed")
    R0, t0 = parse_origin(joint_el.find("origin"))
    T = np.eye(4)
    T[:3, :3] = R0
    T[:3, 3] = t0
    if jtype == "fixed":
        return T
    if jtype != "revolute":
        raise RuntimeError(f"unsupported joint type: {jtype}")
    Rq = rodrigues(parse_axis(joint_el), float(q))
    Tq = np.eye(4)
    Tq[:3, :3] = Rq
    return T @ Tq


def load_urdf_joints(urdf_path: Path):
    root = ET.parse(str(urdf_path)).getroot()
    joints = {}
    for j in root.findall("joint"):
        name = j.get("name")
        if name:
            joints[name] = j
    return joints


def find_chain_joint_names(joints, root_link, tip_link):
    joint_by_child = {}
    parent_of_child = {}
    for jname, jel in joints.items():
        par = jel.find("parent")
        chi = jel.find("child")
        if par is None or chi is None:
            continue
        pl, cl = par.get("link"), chi.get("link")
        if not pl or not cl:
            continue
        joint_by_child[cl] = jname
        parent_of_child[cl] = pl
    chain_rev = []
    cur = tip_link
    while cur != root_link:
        if cur not in joint_by_child:
            raise RuntimeError(f"cannot walk from {tip_link} to {root_link}, stuck at {cur}")
        chain_rev.append(joint_by_child[cur])
        cur = parent_of_child[cur]
    chain_rev.reverse()
    return chain_rev


def fk_base_to_tip(joints, chain, q_by_joint):
    T = np.eye(4)
    for jn in chain:
        T = T @ joint_T(float(q_by_joint.get(jn, 0.0)), joints[jn])
    return T


def fk_root_to_tip_transform(joints, fk_root: str, tip_link: str, q_by_joint: dict):
    """FK: p_fk_root = T @ p_tip（与 fk_base_to_tip 一致）。

    当 fk_root 为 zarm_l1_ref_link 而相机在头部/右臂分支时，URDF 上二者无父子链，
    find_chain_joint_names 会失败；此时经 waist_yaw_link 桥接（与标定里 fk_base=zarm_l1_ref_link 的几何一致）。
    """
    try:
        chain = find_chain_joint_names(joints, fk_root, tip_link)
        return fk_base_to_tip(joints, chain, q_by_joint)
    except RuntimeError:
        if fk_root != "zarm_l1_ref_link":
            raise
        chain_w_t = find_chain_joint_names(joints, "waist_yaw_link", tip_link)
        chain_w_z = find_chain_joint_names(joints, "waist_yaw_link", "zarm_l1_ref_link")
        t_w_t = fk_base_to_tip(joints, chain_w_t, q_by_joint)
        t_w_z = fk_base_to_tip(joints, chain_w_z, q_by_joint)
        return np.linalg.inv(t_w_z) @ t_w_t


def load_offsets_yaml(path: Path):
    out = {}
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        k, _, rest = line.partition(":")
        k = k.strip()
        try:
            v = float(rest.strip())
        except ValueError:
            continue
        if "_joint" in k or k == "waist_yaw_joint":
            out[k] = v
    return out


def load_joints_csv(path: Path):
    by_sample = {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            sid = int(row["sample_id"])
            by_sample.setdefault(sid, {})[row["joint_name"].strip()] = float(row["position"])
    return by_sample


def load_features_camera_points(path: Path, sample_id: int, sensor_name: str):
    rows = []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            if int(row["sample_id"]) != int(sample_id):
                continue
            if row["sensor_name"].strip() != sensor_name:
                continue
            idx = int(row["feature_idx"])
            p = np.array([float(row["x"]), float(row["y"]), float(row["z"])], dtype=float)
            rows.append((idx, p))
    rows.sort(key=lambda x: x[0])
    if not rows:
        raise RuntimeError(f"no features for sample={sample_id}, sensor={sensor_name}")
    return np.stack([x[1] for x in rows], axis=0)


def board_object_points(nx: int, ny: int, square: float, remap_to_center: bool):
    pts = np.array([[(i % nx) * square, (i // nx) * square, 0.0] for i in range(nx * ny)], dtype=float)
    if remap_to_center:
        pts[:, 0] -= ((nx - 1) * square * 0.5)
        pts[:, 1] -= ((ny - 1) * square * 0.5)
    return pts


def rigid_board_to_cam(P_board, Q_cam):
    Pc, Qc = P_board.T, Q_cam.T
    pbar = Pc.mean(axis=1, keepdims=True)
    qbar = Qc.mean(axis=1, keepdims=True)
    H = (Pc - pbar) @ (Qc - qbar).T
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = (qbar - R @ pbar).flatten()
    return R, t


def T_to_rpy(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    if sy > 1e-8:
        return (
            math.atan2(R[2, 1], R[2, 2]),
            math.atan2(-R[2, 0], sy),
            math.atan2(R[1, 0], R[0, 0]),
        )
    return math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0


def rotation_angle_deg(R_est, R_ref):
    R_err = R_ref.T @ R_est
    c = (np.trace(R_err) - 1.0) * 0.5
    c = float(np.clip(c, -1.0, 1.0))
    return math.degrees(math.acos(c))


def parse_checkerboard_joint(urdf_path: Path):
    root = ET.parse(str(urdf_path)).getroot()
    for j in root.findall("joint"):
        if j.get("name") != "checkerboard_joint":
            continue
        origin = j.find("origin")
        if origin is None:
            break
        xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
        rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
        return np.array(xyz, dtype=float), np.array(rpy, dtype=float)
    raise RuntimeError(f"checkerboard_joint not found in {urdf_path}")


def latest_glob(directory: Path, pattern: str):
    files = sorted(directory.glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def load_optimization_used_sample_ids(path: Path):
    """One integer sample_id per line; # starts comments. Order preserved."""
    ids = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        tok = line.split()[0]
        ids.append(int(tok))
    return ids


def compute_T_base_board(
    urdf: Path,
    offsets_yaml: Path,
    csv_dir: Path,
    sample: int,
    sensor_name: str,
    camera_tip_link: str,
    nx: int,
    ny: int,
    square: float,
    remap_to_center: bool,
    fk_root: str = "base_link",
):
    joints = load_urdf_joints(urdf)
    offsets = load_offsets_yaml(offsets_yaml)
    jcsv = load_joints_csv(csv_dir / "joints.csv")
    js = jcsv[int(sample)]
    # 需要整条链上的关节名以收集 q；桥接时用 waist 两段链的并集
    try:
        chain = find_chain_joint_names(joints, fk_root, camera_tip_link)
    except RuntimeError:
        if fk_root != "zarm_l1_ref_link":
            raise
        chain = list(
            dict.fromkeys(
                find_chain_joint_names(joints, "waist_yaw_link", camera_tip_link)
                + find_chain_joint_names(joints, "waist_yaw_link", "zarm_l1_ref_link")
            )
        )
    q_by_joint = {jn: float(js.get(jn, 0.0)) + float(offsets.get(jn, 0.0)) for jn in chain}
    T_base_cam = fk_root_to_tip_transform(joints, fk_root, camera_tip_link, q_by_joint)

    feats = load_features_camera_points(csv_dir / "features.csv", sample, sensor_name)
    P_board = board_object_points(nx, ny, square, remap_to_center=remap_to_center)
    if len(feats) != len(P_board):
        raise RuntimeError(f"feature count mismatch: {len(feats)} != {len(P_board)}")
    R_cb, t_cb = rigid_board_to_cam(P_board, feats)
    T_cam_board = np.eye(4)
    T_cam_board[:3, :3] = R_cb
    T_cam_board[:3, 3] = t_cb
    return T_base_cam @ T_cam_board


def main():
    ap = argparse.ArgumentParser(
        description=(
            "Plot board pose error vs URDF checkerboard_joint using capture CSV. "
            "By default, compares nominal URDF (zero offsets) vs optimized URDF+YAML. "
            "Use --use_nominal_only to force FK from the original URDF only."
        )
    )
    ap.add_argument("--csv_dir", required=True, type=Path)
    ap.add_argument("--nominal_urdf", required=True, type=Path)
    ap.add_argument("--output_dir", required=True, type=Path, help="Directory containing calibrated_*.urdf and calibration_*.yaml, and where plots will be written.")
    ap.add_argument("--calibrated_urdf", default=None, type=Path, help="Override calibrated URDF path (default: output_dir/calibrated.urdf or latest calibrated_*.urdf).")
    ap.add_argument("--calibration_yaml", default=None, type=Path, help="Override calibration offsets YAML path (default: output_dir/calibration.yaml or latest calibration_*.yaml).")
    ap.add_argument(
        "--use_nominal_only",
        action="store_true",
        help="Do FK using nominal_urdf only (zero joint offsets). Ignores optimized URDF/YAML even if present.",
    )
    ap.add_argument("--camera_tip_link", required=True, help="FK tip link for camera optical frame (must match URDF + image header.frame_id).")
    ap.add_argument("--sensor_name", required=True, help="sensor_name used in features.csv for camera observations (e.g. camera_to_base).")
    ap.add_argument(
        "--fk_root",
        default="base_link",
        help="FK chain root link (must match *_chessboard_calibration.yaml base_link / optimize_from_csv fk_base), e.g. zarm_l1_ref_link for Kuavo wrist/head demos.",
    )
    ap.add_argument("--points_x", type=int, default=11)
    ap.add_argument("--points_y", type=int, default=8)
    ap.add_argument("--square_size", type=float, default=0.03)
    ap.add_argument("--remap_to_center", action="store_true", help="Remap board model points from corner-origin to center-origin before rigid alignment.")
    ap.add_argument("--reject_outliers_pos_m", type=float, default=0.0, help="Drop samples with pre_err position > this (m). 0 disables.")
    ap.add_argument("--reject_outliers_rot_deg", type=float, default=0.0, help="Drop samples with pre_err rotation > this (deg). 0 disables.")
    ap.add_argument(
        "--used_sample_ids_file",
        type=Path,
        default=None,
        help="Only plot these CSV sample_id values (one per line). "
        "Default: if csv_dir/optimization_used_sample_ids.txt exists (written by optimize_from_csv), use it.",
    )
    ap.add_argument(
        "--ignore_optimization_used_sample_ids",
        action="store_true",
        help="Plot all samples from joints.csv even when optimization_used_sample_ids.txt exists.",
    )
    ap.add_argument("--out_err_png", type=Path, default=None)
    ap.add_argument("--out_abs_png", type=Path, default=None)
    ap.add_argument(
        "--table_format",
        choices=["markdown", "plain"],
        default="markdown",
        help=(
            "Console table format for per-sample errors. "
            "'markdown' prints a standard Markdown table (for Feishu/Docs), "
            "'plain' keeps the legacy pipe-separated text."
        ),
    )
    args = ap.parse_args()

    csv_dir = args.csv_dir
    if not csv_dir.is_dir():
        raise SystemExit(f"csv_dir is not a directory: {csv_dir}")
    if not args.nominal_urdf.is_file():
        raise SystemExit(f"nominal urdf not found: {args.nominal_urdf}")

    args.output_dir.mkdir(parents=True, exist_ok=True)

    zero_yaml = args.output_dir / "_zero_offsets_tmp.yaml"
    zero_yaml.write_text("# no joint offsets\n", encoding="utf-8")

    # Resolve calibrated URDF / offsets YAML
    if args.use_nominal_only:
        calib_urdf = args.nominal_urdf
        calib_yaml = zero_yaml
        optimized_found = False
    else:
        if args.calibrated_urdf is not None:
            calib_urdf = args.calibrated_urdf
        else:
            fixed = args.output_dir / "calibrated.urdf"
            calib_urdf = fixed if fixed.is_file() else latest_glob(args.output_dir, "calibrated_*.urdf")
        if args.calibration_yaml is not None:
            calib_yaml = args.calibration_yaml
        else:
            fixed = args.output_dir / "calibration.yaml"
            calib_yaml = fixed if fixed.is_file() else latest_glob(args.output_dir, "calibration_*.yaml")
        optimized_found = (
            calib_urdf is not None
            and calib_yaml is not None
            and Path(calib_urdf).is_file()
            and Path(calib_yaml).is_file()
        )
        if not optimized_found:
            raise SystemExit(
                "optimized urdf/yaml not found. Tried:\n"
                f"  calibrated_urdf={calib_urdf}\n"
                f"  calibration_yaml={calib_yaml}\n"
                f"  output_dir={args.output_dir}\n"
                "Hint: run optimize_from_csv first, or pass --use_nominal_only."
            )

    jcsv = load_joints_csv(csv_dir / "joints.csv")
    use_opt_ids_file = False
    if args.ignore_optimization_used_sample_ids:
        sids = sorted(jcsv.keys())
    elif args.used_sample_ids_file is not None:
        ordered = load_optimization_used_sample_ids(Path(args.used_sample_ids_file))
        sids = [sid for sid in ordered if sid in jcsv]
        use_opt_ids_file = True
    else:
        auto_used = csv_dir / "optimization_used_sample_ids.txt"
        if auto_used.is_file():
            ordered = load_optimization_used_sample_ids(auto_used)
            sids = [sid for sid in ordered if sid in jcsv]
            use_opt_ids_file = True
        else:
            sids = sorted(jcsv.keys())
    if not sids:
        if use_opt_ids_file and jcsv:
            print(
                "[plot] WARN: optimization_used_sample_ids 为空或与 joints.csv 无交集 "
                "（例如优化阶段离群点剔除掉全部样本）；回退为 joints.csv 全部样本以便出图。"
            )
            sids = sorted(jcsv.keys())
            use_opt_ids_file = False
        else:
            raise SystemExit("no sample rows found in joints.csv (after optional sample-id filter)")
    if use_opt_ids_file:
        print(
            "[plot] 使用 optimize_from_csv 写入的样本列表（与优化阶段一致），"
            "忽略 --reject_outliers_* 的二次筛选"
        )

    out_err = args.out_err_png or (args.output_dir / "board_pose_error_pre_post_vs_urdf.png")
    out_abs = args.out_abs_png or (args.output_dir / "board_pose_bars_vs_urdf.png")

    xyz_ref, rpy_ref = parse_checkerboard_joint(args.nominal_urdf)
    R_ref = rpy_to_R(float(rpy_ref[0]), float(rpy_ref[1]), float(rpy_ref[2]))
    p_ref = xyz_ref.copy()

    n = len(sids)
    P_pre = np.zeros((n, 3))
    P_post = np.zeros((n, 3))
    RPY_post = np.zeros((n, 3))
    e_pos_pre = np.zeros(n)
    e_pos_post = np.zeros(n)
    e_rot_pre = np.zeros(n)
    e_rot_post = np.zeros(n)

    label_pre = "URDF (nominal)"
    label_post = "URDF (optimized)" if optimized_found else "URDF (nominal)"

    try:
        for i, sid in enumerate(sids):
            T_pre = compute_T_base_board(
                args.nominal_urdf,
                zero_yaml,
                csv_dir,
                sid,
                args.sensor_name,
                args.camera_tip_link,
                args.points_x,
                args.points_y,
                args.square_size,
                remap_to_center=args.remap_to_center,
                fk_root=args.fk_root,
            )
            T_post = compute_T_base_board(
                calib_urdf,
                calib_yaml,
                csv_dir,
                sid,
                args.sensor_name,
                args.camera_tip_link,
                args.points_x,
                args.points_y,
                args.square_size,
                remap_to_center=args.remap_to_center,
                fk_root=args.fk_root,
            )

            P_pre[i] = T_pre[:3, 3]
            P_post[i] = T_post[:3, 3]
            rr, pp, yy = T_to_rpy(T_post[:3, :3])
            RPY_post[i] = [math.degrees(rr), math.degrees(pp), math.degrees(yy)]

            e_pos_pre[i] = np.linalg.norm(P_pre[i] - p_ref)
            e_pos_post[i] = np.linalg.norm(P_post[i] - p_ref)
            e_rot_pre[i] = rotation_angle_deg(T_pre[:3, :3], R_ref)
            e_rot_post[i] = rotation_angle_deg(T_post[:3, :3], R_ref)
    finally:
        try:
            zero_yaml.unlink()
        except OSError:
            pass

    if args.table_format == "markdown":
        print("| sample | pos_err_pre(m) | pos_err_post(m) | rot_err_pre(deg) | rot_err_post(deg) |")
        print("| ---: | ---: | ---: | ---: | ---: |")
        for i, sid in enumerate(sids):
            print(f"| {sid:d} | {e_pos_pre[i]:.6f} | {e_pos_post[i]:.6f} | {e_rot_pre[i]:.4f} | {e_rot_post[i]:.4f} |")
        print(f"| mean | {e_pos_pre.mean():.6f} | {e_pos_post.mean():.6f} | {e_rot_pre.mean():.4f} | {e_rot_post.mean():.4f} |")
    else:
        print("sample | pos_err_pre(m) | pos_err_post(m) | rot_err_pre(deg) | rot_err_post(deg)")
        for i, sid in enumerate(sids):
            print(f"{sid:6d} | {e_pos_pre[i]:.6f} | {e_pos_post[i]:.6f} | {e_rot_pre[i]:.4f} | {e_rot_post[i]:.4f}")
        print(f"mean   | {e_pos_pre.mean():.6f} | {e_pos_post.mean():.6f} | {e_rot_pre.mean():.4f} | {e_rot_post.mean():.4f}")

    # Optional: outlier rejection for plotting only (keep CSV unchanged).
    # When optimization_used_sample_ids.txt is used, sample set already matches optimize_from_csv — do not re-filter.
    pos_thr = float(args.reject_outliers_pos_m or 0.0)
    rot_thr = float(args.reject_outliers_rot_deg or 0.0)
    if not use_opt_ids_file and (pos_thr > 0.0 or rot_thr > 0.0):
        keep_mask = np.ones(n, dtype=bool)
        if pos_thr > 0.0:
            keep_mask &= (e_pos_pre <= pos_thr)
        if rot_thr > 0.0:
            keep_mask &= (e_rot_pre <= rot_thr)
        dropped = [int(sids[i]) for i in range(n) if not bool(keep_mask[i])]
        kept = [int(sids[i]) for i in range(n) if bool(keep_mask[i])]
        print(f"[plot] outlier rejection enabled: pos_thr_m={pos_thr}, rot_thr_deg={rot_thr}, kept={len(kept)}/{n}, dropped={dropped}")

        # Apply mask
        sids = kept
        P_pre = P_pre[keep_mask]
        P_post = P_post[keep_mask]
        RPY_post = RPY_post[keep_mask]
        e_pos_pre = e_pos_pre[keep_mask]
        e_pos_post = e_pos_post[keep_mask]
        e_rot_pre = e_rot_pre[keep_mask]
        e_rot_post = e_rot_post[keep_mask]
        n = len(sids)

    fig1, axes = plt.subplots(1, 2, figsize=(10.5, 4.8), constrained_layout=True)
    fig1.suptitle("Error vs URDF checkerboard_joint", fontsize=12, fontweight="600")
    x = np.arange(n)
    w = 0.35
    axes[0].bar(x - w / 2, e_pos_pre, width=w, label=label_pre, color="#C73E1D", edgecolor="0.15")
    axes[0].bar(x + w / 2, e_pos_post, width=w, label=label_post, color="#1B998B", edgecolor="0.15")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels([f"S{s}" for s in sids])
    axes[0].set_ylabel("position error (m)")
    axes[0].set_title(r"$\|p - p_{\mathrm{ref}}\|$")
    axes[0].legend()
    axes[0].grid(axis="y", alpha=0.35)

    axes[1].bar(x - w / 2, e_rot_pre, width=w, label=label_pre, color="#C73E1D", edgecolor="0.15")
    axes[1].bar(x + w / 2, e_rot_post, width=w, label=label_post, color="#1B998B", edgecolor="0.15")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels([f"S{s}" for s in sids])
    axes[1].set_ylabel("rotation error (deg)")
    axes[1].set_title(r"$\angle(R_{\mathrm{ref}}^{\top} R_{\mathrm{est}})$")
    axes[1].legend()
    axes[1].grid(axis="y", alpha=0.35)
    fig1.savefig(out_err, dpi=160)

    # Summary that matches the plotted (possibly filtered) samples
    pos_pre_mean = float(e_pos_pre.mean()) if n > 0 else float("nan")
    pos_post_mean = float(e_pos_post.mean()) if n > 0 else float("nan")
    rot_pre_mean = float(e_rot_pre.mean()) if n > 0 else float("nan")
    rot_post_mean = float(e_rot_post.mean()) if n > 0 else float("nan")
    pos_drop = pos_pre_mean - pos_post_mean
    rot_drop = rot_pre_mean - rot_post_mean
    pos_drop_pct = (pos_drop / pos_pre_mean * 100.0) if (n > 0 and abs(pos_pre_mean) > 1e-12) else float("nan")
    rot_drop_pct = (rot_drop / rot_pre_mean * 100.0) if (n > 0 and abs(rot_pre_mean) > 1e-12) else float("nan")

    r_ref, p_ref_ang, y_ref = T_to_rpy(R_ref)
    rpy_ref_deg = np.array([math.degrees(r_ref), math.degrees(p_ref_ang), math.degrees(y_ref)])
    labels = [f"S{sid}" for sid in sids] + ["URDF\nmodel"]
    xa = np.arange(len(labels))
    pos_data = np.vstack([P_post, p_ref.reshape(1, 3)])
    rpy_data = np.vstack([RPY_post, rpy_ref_deg.reshape(1, 3)])
    colors = ["#2E86AB", "#A23B72", "#F18F01", "#C73E1D"][:n] + ["#1B998B"]

    fig2, axes2 = plt.subplots(2, 3, figsize=(11.5, 7.2), constrained_layout=True)
    fig2.suptitle(f"{label_post}: board pose vs URDF model", fontsize=13, fontweight="600")
    for col in range(3):
        ax = axes2[0, col]
        ax.bar(xa, pos_data[:, col], width=0.6, color=colors, edgecolor="0.15", linewidth=0.5)
        ax.get_children()[-1].set_hatch("//")
        ax.set_xticks(xa)
        ax.set_xticklabels(labels, fontsize=9)
        ax.set_title(["$p_x$ (m)", "$p_y$ (m)", "$p_z$ (m)"][col], fontsize=11)
        ax.axhline(0, color="0.65", linewidth=0.6)
        ax.grid(axis="y", alpha=0.35)

    for col in range(3):
        ax = axes2[1, col]
        ax.bar(xa, rpy_data[:, col], width=0.6, color=colors, edgecolor="0.15", linewidth=0.5)
        ax.get_children()[-1].set_hatch("//")
        ax.set_xticks(xa)
        ax.set_xticklabels(labels, fontsize=9)
        ax.set_title(["roll (deg)", "pitch (deg)", "yaw (deg)"][col], fontsize=11)
        ax.axhline(0, color="0.65", linewidth=0.6)
        ax.grid(axis="y", alpha=0.35)

    fig2.savefig(out_abs, dpi=160)
    print(f"Wrote {out_err}")
    print(f"Wrote {out_abs}")
    print(
        "[summary] 平均误差下降（与输出图片一致，基于最终用于绘图的样本）\n"
        f"  position: mean {pos_pre_mean:.6f} -> {pos_post_mean:.6f} m, drop {pos_drop:.6f} m ({pos_drop_pct:.2f}%)\n"
        f"  rotation: mean {rot_pre_mean:.4f} -> {rot_post_mean:.4f} deg, drop {rot_drop:.4f} deg ({rot_drop_pct:.2f}%)"
    )


if __name__ == "__main__":
    main()

