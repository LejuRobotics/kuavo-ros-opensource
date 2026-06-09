#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MJCF mesh 资源审计与 kinematics-only 模型生成。

用于排查 MuJoCo viewer 大量 "mesh not found" 警告：
- 解析 ``<compiler meshdir="...">`` 与 ``<mesh file="...">``
- 列出缺失 STL 的绝对路径与期望的 kuavo_assets 目录
- 可选生成去掉 mesh geom 的临时 MJCF（sites/joints 保留，便于调试运动学）
"""

from __future__ import annotations

import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

_LIB = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib"))
if _LIB not in sys.path:
    sys.path.insert(0, _LIB)

from kuavo_paths import kuavo_solver_package_root


@dataclass
class MeshEntry:
    name: str
    file: str
    resolved_path: str
    exists: bool


@dataclass
class MjcfAssetReport:
    mjcf_path: str
    meshdir_raw: Optional[str]
    meshdir_resolved: Optional[str]
    meshdir_exists: bool
    kuavo_assets_root: str
    mesh_entries: List[MeshEntry] = field(default_factory=list)
    mujoco_load_ok: bool = False
    mujoco_error: Optional[str] = None
    nq: int = 0
    nv: int = 0
    nbody: int = 0
    kinematics_only_path: Optional[str] = None

    @property
    def mesh_total(self) -> int:
        return len(self.mesh_entries)

    @property
    def mesh_found(self) -> int:
        return sum(1 for m in self.mesh_entries if m.exists)

    @property
    def mesh_missing(self) -> int:
        return self.mesh_total - self.mesh_found

    @property
    def all_meshes_ok(self) -> bool:
        return self.mesh_total > 0 and self.mesh_missing == 0 and self.meshdir_exists

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mjcf_path": self.mjcf_path,
            "meshdir_raw": self.meshdir_raw,
            "meshdir_resolved": self.meshdir_resolved,
            "meshdir_exists": self.meshdir_exists,
            "kuavo_assets_root": self.kuavo_assets_root,
            "mesh_total": self.mesh_total,
            "mesh_found": self.mesh_found,
            "mesh_missing": self.mesh_missing,
            "all_meshes_ok": self.all_meshes_ok,
            "mujoco_load_ok": self.mujoco_load_ok,
            "mujoco_error": self.mujoco_error,
            "nq": self.nq,
            "nv": self.nv,
            "nbody": self.nbody,
            "kinematics_only_path": self.kinematics_only_path,
            "meshes": [
                {
                    "name": m.name,
                    "file": m.file,
                    "resolved_path": m.resolved_path,
                    "exists": m.exists,
                }
                for m in self.mesh_entries
            ],
        }


def default_kuavo_assets_root(*, about_file: Optional[str] = None) -> str:
    """``src/kuavo_assets``（与 MJCF 中 ``../../../../../kuavo_assets`` 约定一致）。"""
    ref = about_file or __file__
    pkg = Path(kuavo_solver_package_root(ref))
    return str((pkg.parent / "kuavo_assets").resolve())


def _parse_meshdir(mjcf_path: str) -> Optional[str]:
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None:
        return None
    raw = compiler.get("meshdir")
    if not raw:
        return None
    mjcf_dir = Path(mjcf_path).resolve().parent
    return str((mjcf_dir / raw).resolve())


def _collect_mesh_entries(mjcf_path: str, meshdir: Optional[str]) -> List[MeshEntry]:
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    asset = root.find("asset")
    if asset is None:
        return []
    out: List[MeshEntry] = []
    base = Path(meshdir).resolve() if meshdir else Path(mjcf_path).resolve().parent
    for mesh_el in asset.findall("mesh"):
        name = mesh_el.get("name") or ""
        fname = mesh_el.get("file") or ""
        if not fname:
            continue
        resolved = str((base / fname).resolve())
        out.append(
            MeshEntry(
                name=name or fname,
                file=fname,
                resolved_path=resolved,
                exists=os.path.isfile(resolved),
            )
        )
    return out


def write_kinematics_only_mjcf(
    mjcf_path: str,
    *,
    out_path: Optional[str] = None,
) -> str:
    """
    生成无 mesh 几何的 MJCF：删除 ``<mesh>`` 与 ``type=mesh`` 的 geom，保留 joint/site/tendon。
    """
    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    asset = root.find("asset")
    if asset is not None:
        for mesh_el in list(asset.findall("mesh")):
            asset.remove(mesh_el)

    for parent in root.iter():
        for geom in list(parent.findall("geom")):
            if geom.get("mesh") is not None or geom.get("type") == "mesh":
                parent.remove(geom)

    if out_path is None:
        fd, out_path = tempfile.mkstemp(suffix="_kinematics_only.xml", prefix="kuavo_mjcf_")
        os.close(fd)
    tree.write(out_path, encoding="unicode", xml_declaration=True)
    return out_path


def try_mujoco_load(xml_path: str) -> tuple[bool, Optional[str], int, int, int]:
    try:
        import mujoco
    except Exception as e:
        return False, f"mujoco import failed: {e}", 0, 0, 0
    try:
        model = mujoco.MjModel.from_xml_path(xml_path)
        return True, None, int(model.nq), int(model.nv), int(model.nbody)
    except Exception as e:
        return False, str(e), 0, 0, 0


def audit_mjcf(
    mjcf_path: str,
    *,
    assets_root_hint: Optional[str] = None,
    try_load: bool = True,
    write_kinematics_only: bool = False,
) -> MjcfAssetReport:
    """
    审计单个 MJCF 的 mesh 资源；``assets_root_hint`` 仅用于报告展示（如 ``KUAVO_ASSETS_ROOT``）。
    """
    mjcf_path = str(Path(mjcf_path).resolve())
    if not os.path.isfile(mjcf_path):
        raise FileNotFoundError(f"MJCF 不存在: {mjcf_path}")

    kuavo_assets = assets_root_hint or os.environ.get("KUAVO_ASSETS_ROOT") or default_kuavo_assets_root(
        about_file=mjcf_path
    )
    meshdir_raw: Optional[str] = None
    tree = ET.parse(mjcf_path)
    compiler = tree.getroot().find("compiler")
    if compiler is not None:
        meshdir_raw = compiler.get("meshdir")

    meshdir_resolved = _parse_meshdir(mjcf_path)
    meshdir_exists = bool(meshdir_resolved and os.path.isdir(meshdir_resolved))
    entries = _collect_mesh_entries(mjcf_path, meshdir_resolved)

    report = MjcfAssetReport(
        mjcf_path=mjcf_path,
        meshdir_raw=meshdir_raw,
        meshdir_resolved=meshdir_resolved,
        meshdir_exists=meshdir_exists,
        kuavo_assets_root=str(Path(kuavo_assets).resolve()),
        mesh_entries=entries,
    )

    if try_load:
        ok, err, nq, nv, nb = try_mujoco_load(mjcf_path)
        report.mujoco_load_ok = ok
        report.mujoco_error = err
        report.nq, report.nv, report.nbody = nq, nv, nb

    if write_kinematics_only:
        kin_path = write_kinematics_only_mjcf(mjcf_path)
        report.kinematics_only_path = kin_path
        if try_load:
            ok_k, err_k, nq_k, nv_k, nb_k = try_mujoco_load(kin_path)
            if ok_k:
                report.mujoco_load_ok = True
                report.mujoco_error = None
                report.nq, report.nv, report.nbody = nq_k, nv_k, nb_k
            elif not report.mujoco_load_ok:
                report.mujoco_error = f"original: {report.mujoco_error}; kinematics_only: {err_k}"

    return report


def format_report_text(report: MjcfAssetReport, *, max_missing: int = 20) -> str:
    lines = [
        f"MJCF: {report.mjcf_path}",
        f"meshdir (raw): {report.meshdir_raw or '(none)'}",
        f"meshdir (resolved): {report.meshdir_resolved or '(none)'}",
        f"meshdir exists: {report.meshdir_exists}",
        f"kuavo_assets root: {report.kuavo_assets_root}",
        f"meshes: {report.mesh_found}/{report.mesh_total} found, {report.mesh_missing} missing",
    ]
    if report.mujoco_load_ok:
        lines.append(f"MuJoCo load: OK  nq={report.nq} nv={report.nv} nbody={report.nbody}")
    else:
        lines.append(f"MuJoCo load: FAIL — {report.mujoco_error}")
    if report.kinematics_only_path:
        lines.append(f"kinematics-only MJCF: {report.kinematics_only_path}")

    missing = [m for m in report.mesh_entries if not m.exists]
    if missing:
        lines.append("")
        lines.append("Missing meshes (name → resolved path):")
        for m in missing[:max_missing]:
            lines.append(f"  [{m.name}] {m.file}")
            lines.append(f"      → {m.resolved_path}")
        if len(missing) > max_missing:
            lines.append(f"  ... and {len(missing) - max_missing} more")

    if report.mesh_missing > 0:
        lines.append("")
        lines.append("Hint: STL 通常在 kuavo_assets 子模块中，例如：")
        lines.append("  cd kuavo-ros-control && git submodule update --init src/kuavo_assets")
        if report.meshdir_resolved:
            rel = os.path.relpath(report.meshdir_resolved, report.kuavo_assets_root)
            lines.append(f"  Expected under: {report.kuavo_assets_root}/{rel}")

    return "\n".join(lines)


__all__ = [
    "MeshEntry",
    "MjcfAssetReport",
    "audit_mjcf",
    "default_kuavo_assets_root",
    "format_report_text",
    "try_mujoco_load",
    "write_kinematics_only_mjcf",
]
