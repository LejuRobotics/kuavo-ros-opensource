#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mirror STL geometry across a coordinate plane and fix triangle winding.

Supported:
- Binary STL
- ASCII STL (best-effort parser)

By default mirrors across Y=0 plane (y -> -y), which is commonly the left/right
axis in ROS-style coordinate frames for humanoids. Winding is corrected by
swapping v1/v2 for each triangle after mirroring.
"""

from __future__ import annotations

import argparse
import os
import struct
from dataclasses import dataclass


@dataclass(frozen=True)
class Triangle:
    normal: tuple[float, float, float]
    v0: tuple[float, float, float]
    v1: tuple[float, float, float]
    v2: tuple[float, float, float]
    attr: int = 0


def _mirror_vec(v: tuple[float, float, float], axis: str) -> tuple[float, float, float]:
    x, y, z = v
    if axis == "x":
        return (-x, y, z)
    if axis == "y":
        return (x, -y, z)
    if axis == "z":
        return (x, y, -z)
    raise ValueError(f"unsupported axis: {axis}")


def _is_probably_binary_stl(data: bytes) -> bool:
    # Binary STL: 80-byte header + 4-byte uint32 count + n*(50 bytes)
    if len(data) < 84:
        return False
    tri_count = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + tri_count * 50
    if expected == len(data):
        return True
    # ASCII STL often starts with "solid". Binary can also start with "solid",
    # so fall back to structural length check first.
    return False


def read_binary_stl(path: str) -> tuple[bytes, list[Triangle]]:
    with open(path, "rb") as f:
        data = f.read()
    if not _is_probably_binary_stl(data):
        raise ValueError("not a valid binary STL by size heuristic")
    header = data[:80]
    tri_count = struct.unpack_from("<I", data, 80)[0]
    tris: list[Triangle] = []
    off = 84
    for _ in range(tri_count):
        n = struct.unpack_from("<3f", data, off)
        v0 = struct.unpack_from("<3f", data, off + 12)
        v1 = struct.unpack_from("<3f", data, off + 24)
        v2 = struct.unpack_from("<3f", data, off + 36)
        attr = struct.unpack_from("<H", data, off + 48)[0]
        tris.append(Triangle(tuple(n), tuple(v0), tuple(v1), tuple(v2), attr))
        off += 50
    return header, tris


def write_binary_stl(path: str, header: bytes, tris: list[Triangle]) -> None:
    if len(header) != 80:
        raise ValueError("binary STL header must be 80 bytes")
    with open(path, "wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<3f", *t.normal))
            f.write(struct.pack("<3f", *t.v0))
            f.write(struct.pack("<3f", *t.v1))
            f.write(struct.pack("<3f", *t.v2))
            f.write(struct.pack("<H", int(t.attr) & 0xFFFF))


def read_ascii_stl(path: str) -> tuple[str, list[Triangle]]:
    # Best-effort ASCII parser; ignores "outer loop"/"endloop" structure.
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = [ln.strip() for ln in f.readlines()]
    name = ""
    tris: list[Triangle] = []
    i = 0
    while i < len(lines):
        ln = lines[i]
        if ln.startswith("solid ") and not name:
            name = ln[6:].strip()
            i += 1
            continue
        if ln.startswith("facet normal"):
            parts = ln.split()
            if len(parts) >= 5:
                normal = (float(parts[2]), float(parts[3]), float(parts[4]))
            else:
                normal = (0.0, 0.0, 0.0)
            vs: list[tuple[float, float, float]] = []
            j = i + 1
            while j < len(lines) and len(vs) < 3:
                if lines[j].startswith("vertex"):
                    p = lines[j].split()
                    vs.append((float(p[1]), float(p[2]), float(p[3])))
                j += 1
            if len(vs) == 3:
                tris.append(Triangle(normal=normal, v0=vs[0], v1=vs[1], v2=vs[2], attr=0))
            i = j
            continue
        i += 1
    return name, tris


def write_ascii_stl(path: str, name: str, tris: list[Triangle]) -> None:
    if not name:
        name = "mirrored"
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"solid {name}\n")
        for t in tris:
            f.write(f"  facet normal {t.normal[0]} {t.normal[1]} {t.normal[2]}\n")
            f.write("    outer loop\n")
            f.write(f"      vertex {t.v0[0]} {t.v0[1]} {t.v0[2]}\n")
            f.write(f"      vertex {t.v1[0]} {t.v1[1]} {t.v1[2]}\n")
            f.write(f"      vertex {t.v2[0]} {t.v2[1]} {t.v2[2]}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write(f"endsolid {name}\n")


def mirror_triangles(tris: list[Triangle], axis: str) -> list[Triangle]:
    out: list[Triangle] = []
    for t in tris:
        n = _mirror_vec(t.normal, axis)
        v0 = _mirror_vec(t.v0, axis)
        v1 = _mirror_vec(t.v1, axis)
        v2 = _mirror_vec(t.v2, axis)
        # Mirroring flips handedness; swap v1/v2 to keep winding consistent.
        out.append(Triangle(normal=n, v0=v0, v1=v2, v2=v1, attr=t.attr))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True, help="input STL path")
    ap.add_argument("--out", dest="outp", required=True, help="output STL path")
    ap.add_argument("--axis", choices=["x", "y", "z"], default="y", help="mirror axis (default: y)")
    args = ap.parse_args()

    inp = os.path.abspath(args.inp)
    outp = os.path.abspath(args.outp)
    os.makedirs(os.path.dirname(outp), exist_ok=True)

    with open(inp, "rb") as f:
        data = f.read()

    if _is_probably_binary_stl(data):
        header, tris = read_binary_stl(inp)
        mirrored = mirror_triangles(tris, args.axis)
        write_binary_stl(outp, header, mirrored)
        return 0

    # Fallback to ASCII
    name, tris = read_ascii_stl(inp)
    if not tris:
        raise RuntimeError(f"failed to parse STL (binary/ASCII): {inp}")
    mirrored = mirror_triangles(tris, args.axis)
    write_ascii_stl(outp, name, mirrored)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

