#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import dataclasses
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional

import yaml


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 该文件位于：`src/kuavo_solver/scripts/lib/solver_selection.py`
# 所以向上两级才是 `src/kuavo_solver/`（包含 `config/` 与 `robot-descriptions/`）
KUAVO_SOLVER_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))


def normalize_version(v: str) -> str:
    """
    统一外部版本字符串的归一化规则。

    约定：
    - 纯数字（例如 "70"、"56"）不强行加前缀；由 index 里显式映射。
    - 空白会被 strip。
    """
    return (v or "").strip()


@dataclass(frozen=True)
class SolverSelection:
    raw_version: str
    normalized_version: str
    module: str
    token: str
    config_dir: str
    params_yaml: Optional[str] = None
    spec_yaml: Optional[str] = None
    impl: Optional[str] = None


class VersionIndex:
    def __init__(self, index_path: str) -> None:
        self.index_path = index_path
        with open(index_path, "r", encoding="utf-8") as f:
            root = yaml.safe_load(f) or {}
        if not isinstance(root, dict) or int(root.get("version", 0)) != 1:
            raise ValueError(f"unsupported version index format: {index_path}")
        entries = root.get("entries") or {}
        if not isinstance(entries, dict):
            raise ValueError(f"invalid entries in version index: {index_path}")
        self.entries: Dict[str, Dict[str, Any]] = {}
        for k, v in entries.items():
            if not isinstance(k, str) or not isinstance(v, dict):
                continue
            self.entries[k.strip().lower()] = v

    def resolve(self, version: str, *, module: str = "") -> SolverSelection:
        raw = version
        v = normalize_version(version).lower()
        if not v:
            raise ValueError("empty version string")
        e = self.entries.get(v)
        if e is None:
            raise KeyError(f"version not found in index: '{version}' (normalized='{v}')")

        # v1 legacy format: {module, token, config_dir, ...}
        if "modules" not in e:
            module_legacy = str(e.get("module") or "").strip()
            token_legacy = str(e.get("token") or "").strip()
            config_dir_rel_legacy = str(e.get("config_dir") or "").strip()
            if not module_legacy or not token_legacy or not config_dir_rel_legacy:
                raise ValueError(f"invalid entry for version='{version}': {e}")
            config_dir = os.path.join(KUAVO_SOLVER_ROOT, config_dir_rel_legacy)
            params_yaml_rel = e.get("params_yaml")
            spec_yaml_rel = e.get("spec_yaml")
            params_yaml = os.path.join(KUAVO_SOLVER_ROOT, str(params_yaml_rel)) if params_yaml_rel else None
            spec_yaml = os.path.join(KUAVO_SOLVER_ROOT, str(spec_yaml_rel)) if spec_yaml_rel else None
            impl = str(e.get("impl")) if e.get("impl") is not None else None
            return SolverSelection(
                raw_version=str(raw),
                normalized_version=v,
                module=module_legacy,
                token=token_legacy,
                config_dir=config_dir,
                params_yaml=params_yaml,
                spec_yaml=spec_yaml,
                impl=impl,
            )

        modules = e.get("modules") or {}
        if not isinstance(modules, dict) or not modules:
            raise ValueError(f"invalid modules entry for version='{version}': {e}")

        mod = (module or "").strip().lower()
        if not mod:
            # 默认模块选择：优先 ankle（兼容现有使用），否则取第一个
            mod = "ankle" if "ankle" in modules else next(iter(modules.keys()))

        me = modules.get(mod)
        if not isinstance(me, dict):
            raise KeyError(f"module '{mod}' not found in version entry: version='{version}'")

        token = str(me.get("token") or "").strip()
        config_dir_rel = str(me.get("config_dir") or "").strip()
        if not token or not config_dir_rel:
            raise ValueError(f"invalid module entry for version='{version}', module='{mod}': {me}")

        config_dir = os.path.join(KUAVO_SOLVER_ROOT, config_dir_rel)
        params_yaml_rel = me.get("params_yaml")
        spec_yaml_rel = me.get("spec_yaml")
        impl = str(me.get("impl")) if me.get("impl") is not None else None
        params_yaml = os.path.join(KUAVO_SOLVER_ROOT, str(params_yaml_rel)) if params_yaml_rel else None
        spec_yaml = os.path.join(KUAVO_SOLVER_ROOT, str(spec_yaml_rel)) if spec_yaml_rel else None

        return SolverSelection(
            raw_version=str(raw),
            normalized_version=v,
            module=mod,
            token=token,
            config_dir=config_dir,
            params_yaml=params_yaml,
            spec_yaml=spec_yaml,
            impl=impl,
        )


def default_version_index_path() -> str:
    return os.path.join(KUAVO_SOLVER_ROOT, "config", "solver_version_index.yaml")


def load_default_index() -> VersionIndex:
    return VersionIndex(default_version_index_path())

