#!/usr/bin/env python3
"""CoACD 凸分解 K5 座椅 mesh，生成与视觉更接近的碰撞 STL。"""
from __future__ import annotations

import json
import sys
from pathlib import Path

try:
    import coacd
    import trimesh
except ImportError as e:
    print("需要: pip install coacd trimesh numpy", file=sys.stderr)
    raise SystemExit(1) from e

ROOT = Path(__file__).resolve().parent
MESH_DIR = ROOT / "meshes"
OUT_DIR = MESH_DIR / "collision"
SCALE = 0.001
PARTS = ("part_01", "part_03")


def decompose_one(stl_path: Path, prefix: str) -> list[str]:
    mesh = trimesh.load(stl_path, force="mesh")
    mesh.apply_scale(SCALE)
    coacd_mesh = coacd.Mesh(mesh.vertices, mesh.faces)
    hulls = coacd.run_coacd(
        coacd_mesh,
        threshold=0.05,
        max_convex_hull=16,
        preprocess_mode="auto",
        decimate=False,
    )
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    names: list[str] = []
    for i, (verts, faces) in enumerate(hulls):
        hull = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
        out_name = f"{prefix}_col_{i:02d}.stl"
        out_path = OUT_DIR / out_name
        hull.export(out_path)
        names.append(out_name)
    return names


def main() -> None:
    manifest: dict[str, list[str]] = {}
    for part in PARTS:
        src = MESH_DIR / f"{part}.stl"
        if not src.is_file():
            print(f"缺少 {src}", file=sys.stderr)
            raise SystemExit(1)
        manifest[part] = decompose_one(src, part)
        print(f"{part}: {len(manifest[part])} convex hulls")
    manifest_path = OUT_DIR / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2))
    print(f"写入 {manifest_path}")


if __name__ == "__main__":
    main()
