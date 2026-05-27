#!/usr/bin/env python3
import argparse
import math
import re
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple


@dataclass(frozen=True)
class FrameOffset:
    x: float
    y: float
    z: float
    a: float
    b: float
    c: float


def _parse_floats(attr: str, expected: int) -> Tuple[float, ...]:
    parts = [p for p in (attr or "").strip().split() if p]
    if len(parts) != expected:
        raise ValueError(f"Expected {expected} floats, got {len(parts)} from '{attr}'")
    return tuple(float(p) for p in parts)


def _axis_angle_to_R(a: float, b: float, c: float):
    theta = math.sqrt(a * a + b * b + c * c)
    if theta < 1e-12:
        return (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    x, y, z = a / theta, b / theta, c / theta
    ct = math.cos(theta)
    st = math.sin(theta)
    vt = 1.0 - ct
    return (
        (ct + x * x * vt, x * y * vt - z * st, x * z * vt + y * st),
        (y * x * vt + z * st, ct + y * y * vt, y * z * vt - x * st),
        (z * x * vt - y * st, z * y * vt + x * st, ct + z * z * vt),
    )


def _rpy_to_R(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _R_to_rpy(R):
    # Inverse of R = Rz(yaw) * Ry(pitch) * Rx(roll)
    r20 = R[2][0]
    if abs(r20) < 1.0 - 1e-12:
        pitch = math.asin(-r20)
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
        return roll, pitch, yaw

    # Gimbal lock: cos(pitch) ~ 0
    pitch = math.pi / 2 if r20 <= -1.0 + 1e-12 else -math.pi / 2
    roll = 0.0
    yaw = math.atan2(-R[0][1], R[1][1])
    return roll, pitch, yaw


def _matmul3(A, B):
    return tuple(
        tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _matvec3(A, v):
    return (
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    )


def _vecadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def load_offsets_yaml(path: Path) -> Dict[str, FrameOffset]:
    # Offsets YAML is a simple "key: value" list. We group by "<name>_(x|y|z|a|b|c)".
    line_re = re.compile(r"^\s*([A-Za-z0-9_]+)\s*:\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*$")
    raw: Dict[str, Dict[str, float]] = {}
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        m = line_re.match(line)
        if not m:
            continue
        key, val_s = m.group(1), m.group(2)
        try:
            val = float(val_s)
        except ValueError:
            continue
        if "_" not in key:
            continue
        base, suffix = key.rsplit("_", 1)
        if suffix not in {"x", "y", "z", "a", "b", "c"}:
            continue
        raw.setdefault(base, {})[suffix] = val

    out: Dict[str, FrameOffset] = {}
    for name, d in raw.items():
        if all(k in d for k in ("x", "y", "z", "a", "b", "c")):
            out[name] = FrameOffset(d["x"], d["y"], d["z"], d["a"], d["b"], d["c"])
    return out


def apply_offsets_to_urdf(urdf_text: str, offsets: Dict[str, FrameOffset]) -> Tuple[str, Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float]]]]:
    root = ET.fromstring(urdf_text)
    updates: Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = {}

    for joint in root.findall(".//joint"):
        name = joint.get("name")
        if not name or name not in offsets:
            continue
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
            origin.set("xyz", "0 0 0")
            origin.set("rpy", "0 0 0")

        xyz = _parse_floats(origin.get("xyz", "0 0 0"), 3)
        rpy = _parse_floats(origin.get("rpy", "0 0 0"), 3)
        R_old = _rpy_to_R(rpy[0], rpy[1], rpy[2])
        p_old = (xyz[0], xyz[1], xyz[2])

        off = offsets[name]
        R_off = _axis_angle_to_R(off.a, off.b, off.c)
        p_off = (off.x, off.y, off.z)

        # Match robot_calibration's URDF update rule: origin = origin * frame_offset
        R_new = _matmul3(R_old, R_off)
        p_new = _vecadd(p_old, _matvec3(R_old, p_off))
        roll, pitch, yaw = _R_to_rpy(R_new)

        origin.set("xyz", f"{p_new[0]:.8f} {p_new[1]:.8f} {p_new[2]:.8f}")
        origin.set("rpy", f"{roll:.8f} {pitch:.8f} {yaw:.8f}")

        updates[name] = (p_new, (roll, pitch, yaw))

    xml_out = ET.tostring(root, encoding="unicode")
    return xml_out, updates


def main():
    ap = argparse.ArgumentParser(description="Apply robot_calibration offsets YAML to a URDF, writing a new URDF.")
    ap.add_argument("--urdf-in", required=True, type=Path, help="Input URDF path")
    ap.add_argument("--offsets", required=True, type=Path, help="Offsets YAML path (calibration_*.yaml)")
    ap.add_argument("--urdf-out", type=Path, default=None, help="Output URDF path (default: ./output/calibrated_*.urdf next to offsets)")
    args = ap.parse_args()

    urdf_text = args.urdf_in.read_text(encoding="utf-8", errors="ignore")
    offsets = load_offsets_yaml(args.offsets)
    if not offsets:
        raise SystemExit(f"No frame offsets found in {args.offsets}")

    updated_urdf, updates = apply_offsets_to_urdf(urdf_text, offsets)

    if args.urdf_out is None:
        datecode = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
        out_dir = args.offsets.parent
        args.urdf_out = out_dir / f"calibrated_{datecode}.urdf"

    args.urdf_out.write_text(updated_urdf, encoding="utf-8")

    print(f"Wrote: {args.urdf_out}")
    for name, (p, rpy) in updates.items():
        print(f"- {name}: xyz=({p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f}) rpy=({rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f})")


if __name__ == "__main__":
    main()

