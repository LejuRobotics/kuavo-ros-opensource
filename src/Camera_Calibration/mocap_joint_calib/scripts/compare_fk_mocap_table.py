#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FK vs 动捕 逐点分量误差表格：加偏置(pre=0) / 不加偏置(post=bias)。
输出各位置(sample) 左右手 的 x/y/z(mm) + roll/pitch/yaw(deg)。

用法:
    python3 compare_fk_mocap_table.py --capture capture_xxx.json \
        [--calibration_yaml output/calibration.yaml] [--output out.csv]
"""
import argparse, sys, json
from pathlib import Path
import numpy as np

_DIR = Path(__file__).resolve().parent
_MOD = _DIR.parent
_CAM = _MOD.parent
for p in (str(_CAM), str(_CAM/"mocap_checkerboard_pose"), str(_MOD)):
    if p not in sys.path:
        sys.path.insert(0, p)
from plot_board_error_from_csv import load_urdf_joints
from scripts.plot_mocap_error import compute_components

def main():
    ap = argparse.ArgumentParser(description="FK vs 动捕 逐点分量误差表格")
    ap.add_argument("--capture", required=True, help="capture_*.json")
    ap.add_argument("--calibration_yaml", default=None, help="calibration.yaml (bias)")
    ap.add_argument("--output", default=None, help="CSV 输出路径 (默认只打印)")
    args = ap.parse_args()

    cap = json.loads(Path(args.capture).read_text())
    urdf_name = Path(cap["meta"]["urdf"]).name
    urdf_path = _CAM / urdf_name
    if not urdf_path.exists():
        urdf_path = _CAM / "biped_v3_arm_s45.urdf"
    urdf_joints = load_urdf_joints(urdf_path)
    fk_root = cap["meta"].get("fk_root", "base_link")

    bias = {}
    if args.calibration_yaml:
        for line in Path(args.calibration_yaml).read_text().splitlines():
            if ":" in line:
                k, _, v = line.partition(":"); bias[k.strip()] = float(v.strip())

    pre = compute_components(cap, urdf_joints, fk_root, {})
    post = compute_components(cap, urdf_joints, fk_root, bias)

    sids = sorted(set(r["sample"] for r in pre))
    bodies = ["l_hand", "r_hand"]
    comps = ["x", "y", "z", "roll", "pitch", "yaw"]

    lines = []
    header = "sample,hand,pre_x,pre_y,pre_z,pre_roll,pre_pitch,pre_yaw,post_x,post_y,post_z,post_roll,post_pitch,post_yaw"
    lines.append(header)
    for sid in sids:
        for b in bodies:
            pr = next((r for r in pre if r["sample"]==sid and r["body"]==b), None)
            po = next((r for r in post if r["sample"]==sid and r["body"]==b), None)
            if pr is None: continue
            row = [str(sid), b]
            for c in comps: row.append(f"{pr[c]:+.3f}")
            if po: 
                for c in comps: row.append(f"{po[c]:+.3f}")
            else:
                for c in comps: row.append("")
            lines.append(",".join(row))
    out = "\n".join(lines)
    print(out)
    if args.output:
        Path(args.output).write_text(out + "\n")
        print(f"\n[已写入] {args.output}")

    # 均值汇总
    print("\n== 均值 ==")
    print(f"{'hand':<8}{'metric':<8}{'pre':>10}{'post':>10}")
    for b in bodies:
        for c, unit in [("x","mm"),("y","mm"),("z","mm"),("roll","deg"),("pitch","deg"),("yaw","deg")]:
            pv = [r[c] for r in pre if r["body"]==b]
            qv = [r[c] for r in post if r["body"]==b]
            print(f"{b:<8}{c:<8}{np.mean(pv):>10.3f}{np.mean(qv):>10.3f}")

if __name__ == "__main__":
    main()
