#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kuavo solver 工程公共路径工具。

用途
----
为 `solver_validation_spec.py` 提供 `kuavo_solver_package_root()`：
从任意“当前脚本所在文件”反推出 `src/kuavo_solver` 根目录，
用于定位 `robot-descriptions/`、`config/` 等资源。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Union


def kuavo_solver_package_root(about_file: Union[str, os.PathLike[str]]) -> str:
    """
    给定一个文件路径（通常是 `__file__`），向上查找名为 `kuavo_solver` 的目录。

    约束
    ----
    - 不做静默 fallback：找不到或校验失败就抛异常；
    - 找到后还会校验该目录下存在 `robot-descriptions/`，避免“误匹配”。
    """

    p0 = Path(about_file).expanduser().resolve()
    p = p0 if p0.is_dir() else p0.parent

    for parent in (p, *p.parents):
        if parent.name != "kuavo_solver":
            continue

        robot_desc = parent / "robot-descriptions"
        if robot_desc.is_dir():
            return str(parent)

        raise RuntimeError(
            f"定位到目录名为 'kuavo_solver' 的路径，但缺少 robot-descriptions：{parent}"
        )

    raise RuntimeError(f"无法从 about_file 推断 kuavo_solver 根目录：{p0}")


__all__ = ["kuavo_solver_package_root"]

