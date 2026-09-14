#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 process_mocap_poses.py 输出的 JSON 写入 URDF checkerboard_joint 的 xyz / rpy 及 parent。

两种模式：
  left_shoulder — parent=zarm_l1_ref_link，位姿来自 checkerboard_in_l_shoulder
  waist         — parent=waist_yaw_link，位姿来自 checkerboard_in_torso
"""

from __future__ import print_function

import argparse
import json
import os
import re
import shutil
import sys
from datetime import datetime
from typing import Any, Dict, Tuple

import xml.etree.ElementTree as ET

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_URDF = os.path.abspath(
    os.path.join(_SCRIPT_DIR, "..", "biped_v3_arm_s62.urdf")
)

# 写入模式：JSON 字段与 URDF parent link
MODES: Dict[str, Dict[str, str]] = {
    "left_shoulder": {
        "parent_link": "zarm_l1_ref_link",
        "json_key": "checkerboard_in_l_shoulder",
        "desc": "checkerboard 相对左肩 link → parent zarm_l1_ref_link",
    },
    "waist": {
        "parent_link": "waist_yaw_link",
        "json_key": "checkerboard_in_torso",
        "desc": "checkerboard 相对 torso link → parent waist_yaw_link",
    },
}

JOINT_NAME = "checkerboard_joint"


def _fmt_xyz_rpy(xyz, rpy) -> Tuple[str, str]:
    """格式化为 URDF origin 字符串（米、弧度）。"""
    xyz_s = " ".join(f"{float(v):.8g}" for v in xyz)
    rpy_s = " ".join(f"{float(v):.8g}" for v in rpy)
    return xyz_s, rpy_s


def load_pose_from_json(json_path: str, json_key: str) -> Tuple[list, list]:
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if json_key not in data:
        raise KeyError(
            f"JSON 中缺少 {json_key!r}，可用键: {[k for k in data if 'checkerboard' in k]}"
        )
    block = data[json_key]
    xyz = block["xyz"]
    rpy = block["rpy"]
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError(f"{json_key} 的 xyz/rpy 长度无效")
    return xyz, rpy


def _validate_joint_in_tree(root: ET.Element, parent_link: str) -> ET.Element:
    joint = None
    for j in root.findall("joint"):
        if j.get("name") == JOINT_NAME:
            joint = j
            break
    if joint is None:
        raise RuntimeError(f"URDF 中未找到 joint {JOINT_NAME!r}")

    parent = joint.find("parent")
    if parent is None:
        raise RuntimeError(f"{JOINT_NAME} 缺少 <parent> 元素")

    child = joint.find("child")
    if child is None or child.get("link") != "checkerboard_link":
        raise RuntimeError(f"{JOINT_NAME} 的 child 应为 checkerboard_link")

    # 确认 parent link 在 URDF 中存在（避免写错名）
    link_names = {lk.get("name") for lk in root.findall("link")}
    if parent_link not in link_names:
        raise RuntimeError(
            f"parent link {parent_link!r} 不在 URDF link 列表中"
        )
    return joint


def patch_checkerboard_joint_text(
    urdf_text: str,
    parent_link: str,
    xyz: list,
    rpy: list,
) -> str:
    """
    在原始 URDF 文本中替换 checkerboard_joint 的 origin / parent，尽量保持其余格式。
    """
    xyz_s, rpy_s = _fmt_xyz_rpy(xyz, rpy)

    # 匹配 <origin xyz="..." rpy="..." />（允许换行）
    origin_pat = re.compile(
        r'(<joint name="checkerboard_joint" type="fixed">\s*'
        r'<origin xyz=")([^"]+)("\s+rpy=")([^"]+)(" />)',
        re.DOTALL,
    )
    if not origin_pat.search(urdf_text):
        raise RuntimeError("无法在 URDF 文本中定位 checkerboard_joint 的 <origin>")
    urdf_text = origin_pat.sub(
        rf"\g<1>{xyz_s}\g<3>{rpy_s}\g<5>",
        urdf_text,
        count=1,
    )

    parent_pat = re.compile(
        r'(<joint name="checkerboard_joint" type="fixed">.*?<parent link=")([^"]+)(" />)',
        re.DOTALL,
    )
    if not parent_pat.search(urdf_text):
        raise RuntimeError("无法在 URDF 文本中定位 checkerboard_joint 的 <parent>")
    urdf_text = parent_pat.sub(
        rf"\g<1>{parent_link}\g<3>",
        urdf_text,
        count=1,
    )
    return urdf_text


def apply_checkerboard_to_urdf(
    json_path: str,
    urdf_path: str,
    mode: str,
    output_path: str = None,
    backup: bool = True,
) -> Dict[str, Any]:
    if mode not in MODES:
        raise ValueError(f"未知 mode {mode!r}，可选: {list(MODES.keys())}")

    cfg = MODES[mode]
    json_key = cfg["json_key"]
    parent_link = cfg["parent_link"]

    xyz, rpy = load_pose_from_json(json_path, json_key)

    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(urdf_path)

    urdf_text = open(urdf_path, "r", encoding="utf-8").read()
    root = ET.fromstring(urdf_text)
    _validate_joint_in_tree(root, parent_link)

    new_text = patch_checkerboard_joint_text(urdf_text, parent_link, xyz, rpy)

    out_path = output_path or urdf_path
    if os.path.abspath(out_path) == os.path.abspath(urdf_path) and backup:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        bak = f"{urdf_path}.bak.{ts}"
        shutil.copy2(urdf_path, bak)
        print(f"[apply] 已备份: {bak}")

    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(new_text)

    return {
        "mode": mode,
        "json_key": json_key,
        "parent_link": parent_link,
        "xyz": xyz,
        "rpy": rpy,
        "urdf_out": os.path.abspath(out_path),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="将动捕 JSON 位姿写入 biped URDF checkerboard_joint"
    )
    parser.add_argument(
        "--json",
        required=True,
        help="process_mocap_poses.py 输出的 JSON",
    )
    parser.add_argument(
        "--urdf",
        default=_DEFAULT_URDF,
        help=f"目标 URDF（默认 {_DEFAULT_URDF}）",
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=sorted(MODES.keys()),
        help="left_shoulder: parent=zarm_l1_ref_link; waist: parent=waist_yaw_link",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="输出 URDF（默认覆盖 --urdf；指定则写入新文件）",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="覆盖原 URDF 时不创建 .bak 备份",
    )
    args = parser.parse_args()

    try:
        info = apply_checkerboard_to_urdf(
            json_path=args.json,
            urdf_path=args.urdf,
            mode=args.mode,
            output_path=args.output,
            backup=not args.no_backup,
        )
    except Exception as e:
        print(f"[apply] 失败: {e}", file=sys.stderr)
        return 1

    print(f"[apply] 模式: {args.mode} ({MODES[args.mode]['desc']})")
    print(f"[apply] JSON 字段: {info['json_key']}")
    print(f"[apply] parent link: {info['parent_link']}")
    print(f"[apply] origin xyz(m): {[round(x, 6) for x in info['xyz']]}")
    print(f"[apply] origin rpy(rad): {[round(r, 6) for r in info['rpy']]}")
    print(f"[apply] 已写入: {info['urdf_out']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
