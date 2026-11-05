#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""标定流程统一机型表（唯一数据源）。

三条约定
--------
1. 内部小版本按 ``%100`` 折回基础型号：``100049→49``、``200053→53``、``400062→62`` …，
   与 ``humanoid_controllers/launch/robot_version_manager.launch`` 的折法一致。
   新增小版本一般不需要改代码，只要它的基础型号在表里。
2. **标定按基础型号走**：布局、标定 URDF、FK 根、头部相机是否倒装、是否支持左右腕
   三路相机、``arms_zero.yaml`` 槽位数、l1/r1 是否走 EC，全部查本表。
3. ``kuavo_v${ROBOT_VERSION}/kuavo.json``、``biped_s${ROBOT_VERSION}/`` 这类**资源路径
   仍然用原始内部版本号**，不要折回。

基础型号对照
------------
======  ==========  =============================  ===============  ======  ======  ======  ======
基础    布局        标定 URDF                      FK 根            头相机  腕相机  零点槽  l1/r1
                                                                   旋转    三路    位数    走 EC
======  ==========  =============================  ===============  ======  ======  ======  ======
45      biped45     biped_v3_arm_s45.urdf          base_link        是      不支持  14      是
49      biped45     biped_v3_arm_s49.urdf          base_link        否      不支持  14      是
52      biped52     biped_v3_arm.urdf              waist_yaw_link   否      支持    14      是
53      biped56     biped_v3_arm_s53.urdf          waist_yaw_link   否      支持    16      否
55      biped56     biped_v3_arm_s55.urdf          waist_yaw_link   否      支持    16      否
56      biped56     biped_v3_arm_s56.urdf          waist_yaw_link   否      支持    16      否
62      wheel62     biped_v3_arm_s62.urdf          waist_yaw_link   否      支持    16      否
63      wheel62     biped_v3_arm_s63.urdf          waist_yaw_link   否      支持    16      否
======  ==========  =============================  ===============  ======  ======  ======  ======

几点说明
--------
* 53/55/56 共用 ``biped56`` 布局，分别使用自身的标定 URDF 和输出文件。
  S53/S55/S56 的头部安装高度统一为 0.49125 m，小版本分别使用基础型号的模板。
  模板保留既有手臂几何；小版本的机械安装差异仍需按实机核对。
* 62/63 的 200062/300062/400062/500062、400063/500063 与 ``biped_s62`` 关节顺序一致，
  偏差 ≤0.3 mm（肩部安装位），共用 ``wheel62``，但 62/63 的标定文件及输出分开。
* 45 与 49 系列虽共用 ``biped45`` 布局，但标定 URDF 分开：``biped_v3_arm_s45.urdf``
  与 ``biped_v3_arm_s49.urdf``（由前者逐字复制，仅 ``<robot name>`` 不同），
  输出分别写 ``biped_v3_arm_s45_calibrated.urdf`` / ``biped_v3_arm_s49_calibrated.urdf``，
  采集配置也分开（``calib_s49.yaml`` / ``calib_qingtong_s49.yaml``）。
* 49 系列与 45 共用 ``biped45`` 布局，但头部相机翻转已进模型（``3.14159 0.38397 0``），
  且腕部相机帧是 ``l_camera_link``/``camera_l_base``/``camera_r_base``，与 s45 的
  ``l/r_hand_tripod``、``l/r_hand_camera`` 完全不同，因此只支持头部流程。
* 45 的腕部同理不支持：腕 demo launch 里只有 biped52/wheel62 两套 ``joint_q`` 索引，
  没有 s45（无腰，臂从 12 起）的分支。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass(frozen=True)
class ModelSpec:
    """按基础型号查到的标定机型特征。"""

    base: str
    layout: str
    urdf: str
    calibrated_urdf: str
    fk_root: str
    head_rot180: bool
    supports_wrist: bool
    zero_slots: int
    ec_l1_r1: bool
    arm_left: Tuple[int, int]
    arm_right: Tuple[int, int]
    calib_config: str
    calib_config_qingtong: str
    apply_script: str
    shoulder_profile: str
    note: str = ""


# 45 与 49 系列共用 biped45 布局（无腰、臂索引 [12,19)/[19,26)、头部相机链相同），
# 但标定模型、采集配置和输出各用各的文件：否则标完 49 会覆盖 45 的 *_calibrated.urdf。
_BIPED45_COMMON = dict(
    layout="biped45",
    fk_root="base_link",
    arm_left=(12, 19),
    arm_right=(19, 26),
    apply_script="apply_zero_deltas_to_arms_zero.py",
    shoulder_profile="s45",
)

_BIPED45 = dict(
    **_BIPED45_COMMON,
    urdf="biped_v3_arm_s45.urdf",
    calibrated_urdf="biped_v3_arm_s45_calibrated.urdf",
    calib_config="calib_s45.yaml",
    calib_config_qingtong="calib_qingtong_s45.yaml",
)

# S49 系列（100049/200049/300049/400049）：标定模型由 s45 逐字复制，只改机型名。
_BIPED49 = dict(
    **_BIPED45_COMMON,
    urdf="biped_v3_arm_s49.urdf",
    calibrated_urdf="biped_v3_arm_s49_calibrated.urdf",
    calib_config="calib_s49.yaml",
    calib_config_qingtong="calib_qingtong_s49.yaml",
)

_BIPED52 = dict(
    layout="biped52",
    urdf="biped_v3_arm.urdf",
    calibrated_urdf="biped_v3_arm_calibrated.urdf",
    fk_root="waist_yaw_link",
    arm_left=(13, 20),
    arm_right=(20, 27),
    calib_config="calib.yaml",
    calib_config_qingtong="calib_qingtong.yaml",
    apply_script="apply_zero_deltas_to_arms_zero.py",
    shoulder_profile="default",
)

_BIPED56 = dict(
    layout="biped56",
    urdf="biped_v3_arm_s56.urdf",
    calibrated_urdf="biped_v3_arm_s56_calibrated.urdf",
    fk_root="waist_yaw_link",
    arm_left=(13, 20),
    arm_right=(20, 27),
    calib_config="calib.yaml",
    calib_config_qingtong="calib_qingtong.yaml",
    apply_script="apply_zero_deltas_to_arms_zero.py",
    shoulder_profile="default",
)

# S53/S55/S56 使用独立模板及输出，头部高度统一为 0.49125 m。
_BIPED55 = {
    **_BIPED56,
    "urdf": "biped_v3_arm_s55.urdf",
    "calibrated_urdf": "biped_v3_arm_s55_calibrated.urdf",
}

_BIPED53 = {
    **_BIPED56,
    "urdf": "biped_v3_arm_s53.urdf",
    "calibrated_urdf": "biped_v3_arm_s53_calibrated.urdf",
}

_WHEEL62 = dict(
    layout="wheel62",
    urdf="biped_v3_arm_s62.urdf",
    calibrated_urdf="biped_v3_arm_s62_calibrated.urdf",
    fk_root="waist_yaw_link",
    arm_left=(4, 11),
    arm_right=(11, 18),
    calib_config="calib.yaml",
    calib_config_qingtong="calib_qingtong.yaml",
    apply_script="apply_zero_deltas_to_arms_zero_wheel62.py",
    shoulder_profile="default",
)

_WHEEL63 = {
    **_WHEEL62,
    "urdf": "biped_v3_arm_s63.urdf",
    "calibrated_urdf": "biped_v3_arm_s63_calibrated.urdf",
}

#: 基础型号 -> 机型特征。新增小版本时不要往这里加，改这里的判定即可自动覆盖。
TABLE: Dict[str, ModelSpec] = {
    "45": ModelSpec(
        base="45", head_rot180=True, supports_wrist=False, zero_slots=14, ec_l1_r1=True,
        note="S45 相机物理倒装：机器人模型 biped_s45.urdf 未含翻转（camera rpy 0 0.48869 0），驱动发 rotate_180 话题 + head_camera_color_optical_frame_rot180；标定 URDF biped_v3_arm_s45.urdf 已含翻转（3.14159 0.38392 0）且两帧齐全，不能据它反推",
        **_BIPED45,
    ),
    "49": ModelSpec(
        base="49", head_rot180=False, supports_wrist=False, zero_slots=14, ec_l1_r1=True,
        note="S49 系列（100049/200049/300049/400049）与 45 共用 biped45 布局，"
             "翻转已进模型，腕相机帧不同因而只支持头部流程；标定模型用 s49 独立文件",
        **_BIPED49,
    ),
    "52": ModelSpec(
        base="52", head_rot180=False, supports_wrist=True, zero_slots=14, ec_l1_r1=True,
        **_BIPED52,
    ),
    "53": ModelSpec(
        base="53", head_rot180=False, supports_wrist=True, zero_slots=16, ec_l1_r1=False,
        note="S53 系列使用独立 S53 标定模型，头部安装高度为 0.49125 m",
        **_BIPED53,
    ),
    "55": ModelSpec(
        base="55", head_rot180=False, supports_wrist=True, zero_slots=16, ec_l1_r1=False,
        note="S55 系列使用低头部安装的 biped_v3_arm_s55.urdf",
        **_BIPED55,
    ),
    "56": ModelSpec(
        base="56", head_rot180=False, supports_wrist=True, zero_slots=16, ec_l1_r1=False,
        **_BIPED56,
    ),
    "62": ModelSpec(
        base="62", head_rot180=False, supports_wrist=True, zero_slots=16, ec_l1_r1=False,
        note="S62 系列（200062/300062/400062/500062）与 63 共用 wheel62 布局",
        **_WHEEL62,
    ),
    "63": ModelSpec(
        base="63", head_rot180=False, supports_wrist=True, zero_slots=16, ec_l1_r1=False,
        note="S63 系列（400063/500063）与 62 共用 wheel62 布局",
        **_WHEEL63,
    ),
}

_BASE_ORDER = ("45", "49", "52", "53", "55", "56", "62", "63")


def fold(version: object) -> str:
    """内部小版本按 %100 折回基础型号：100049→49、200053→53、400062→62 …。

    非纯数字（含空值）原样返回，由调用方决定是否报错。
    """
    text = str(version).strip()
    if text.isdigit():
        return str(int(text) % 100)
    return text


def spec(version: object) -> Optional[ModelSpec]:
    """按版本号（原始或基础）取机型特征；不支持时返回 None。"""
    return TABLE.get(fold(version))


def supported_bases() -> List[str]:
    return list(_BASE_ORDER)


def supported_text(sep: str = "/") -> str:
    return sep.join(_BASE_ORDER)


def require(version: object, purpose: str = "标定") -> ModelSpec:
    """取机型特征；不支持时抛出带支持列表的 RuntimeError。"""
    spec_ = spec(version)
    if spec_ is None:
        raise RuntimeError(
            f"{purpose}不支持 ROBOT_VERSION={version!r}"
            f"（按 %100 折回为 {fold(version)!r}）；"
            f"支持基础型号 {supported_text()} 及其小版本"
            f"（如 100049/200049、200053/300053、400055、200062/500062、400063/500063）"
        )
    return spec_


def version_from_env() -> str:
    return os.environ.get("ROBOT_VERSION", "").strip()


def describe(version: object) -> str:
    """一行机型摘要，用于日志。"""
    spec_ = spec(version)
    if spec_ is None:
        return f"ROBOT_VERSION={version!r}（不支持，折回基础型号={fold(version)!r}）"
    suffix = ""
    if str(version).strip() != spec_.base:
        suffix = f"，按 %100 折回基础型号 {spec_.base}"
    return (
        f"ROBOT_VERSION={version}（布局 {spec_.layout}，URDF {spec_.urdf}，"
        f"FK 根 {spec_.fk_root}，头相机旋转 {'是' if spec_.head_rot180 else '否'}，"
        f"腕相机 {'支持' if spec_.supports_wrist else '不支持'}，"
        f"零点 {spec_.zero_slots} 维{'（l1/r1 走 EC）' if spec_.ec_l1_r1 else ''}{suffix}）"
    )


__all__ = [
    "ModelSpec",
    "TABLE",
    "fold",
    "spec",
    "require",
    "supported_bases",
    "supported_text",
    "version_from_env",
    "describe",
]
