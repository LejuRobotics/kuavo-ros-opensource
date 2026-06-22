#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np
import yaml


@dataclass(frozen=True)
class ArmLimits:
    elbow_motor: Tuple[float, float]
    wrist_a_motor: Tuple[float, float]
    wrist_b_motor: Tuple[float, float]
    elbow_left_joint: Tuple[float, float]
    elbow_right_joint: Tuple[float, float]
    wrist_roll_left_joint: Tuple[float, float]
    wrist_roll_right_joint: Tuple[float, float]
    wrist_pitch_left_joint: Tuple[float, float]
    wrist_pitch_right_joint: Tuple[float, float]


def _as_pair(x) -> Tuple[float, float]:
    if not isinstance(x, (list, tuple)) or len(x) < 2:
        raise ValueError(f"invalid limit pair: {x}")
    return float(x[0]), float(x[1])


def load_arm_limits(spec_yaml: str) -> ArmLimits:
    with open(spec_yaml, "r", encoding="utf-8") as f:
        root = yaml.safe_load(f) or {}
    arm = (root or {}).get("arm") or {}
    limits = (arm or {}).get("limits") or {}
    joint = (limits or {}).get("joint") or {}
    motor = (limits or {}).get("motor") or {}
    return ArmLimits(
        elbow_motor=_as_pair(motor.get("elbow")),
        wrist_a_motor=_as_pair(motor.get("wrist_a")),
        wrist_b_motor=_as_pair(motor.get("wrist_b")),
        elbow_left_joint=_as_pair(joint.get("elbow_left")),
        elbow_right_joint=_as_pair(joint.get("elbow_right")),
        wrist_roll_left_joint=_as_pair(joint.get("wrist_roll_left")),
        wrist_roll_right_joint=_as_pair(joint.get("wrist_roll_right")),
        wrist_pitch_left_joint=_as_pair(joint.get("wrist_pitch_left")),
        wrist_pitch_right_joint=_as_pair(joint.get("wrist_pitch_right")),
    )


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class ArmSolverStub:
    """
    占位手臂解算器（14D 双臂接口）。

    作用：
    - 让统一 CLI/版本索引/报告框架先跑通
    - 读取 YAML limits 做 clamp，避免明显越界
    - 其它维度先按透传处理

    14D 约定（与 kuavo_solver_full 一致的示意）：
    - 左臂 q7/p7 在 [0:7)，右臂在 [7:14)
    - elbow 在每个 7D 段的 index=3
    - wrist_roll / wrist_pitch 在 index=5/6
    - motor 侧：elbow(index=3), wrist_a(index=5), wrist_b(index=6)
    """

    def __init__(self, spec_yaml: str):
        self.spec_yaml = spec_yaml
        self.lim = load_arm_limits(spec_yaml)

    def _clamp_joint_segment(self, seg7: np.ndarray, is_left: bool) -> np.ndarray:
        out = np.array(seg7, dtype=np.float64).copy()
        if is_left:
            out[3] = _clamp(float(out[3]), *self.lim.elbow_left_joint)
            out[5] = _clamp(float(out[5]), *self.lim.wrist_roll_left_joint)
            out[6] = _clamp(float(out[6]), *self.lim.wrist_pitch_left_joint)
        else:
            out[3] = _clamp(float(out[3]), *self.lim.elbow_right_joint)
            out[5] = _clamp(float(out[5]), *self.lim.wrist_roll_right_joint)
            out[6] = _clamp(float(out[6]), *self.lim.wrist_pitch_right_joint)
        return out

    def _clamp_motor_segment(self, seg7: np.ndarray) -> np.ndarray:
        out = np.array(seg7, dtype=np.float64).copy()
        out[3] = _clamp(float(out[3]), *self.lim.elbow_motor)
        out[5] = _clamp(float(out[5]), *self.lim.wrist_a_motor)
        out[6] = _clamp(float(out[6]), *self.lim.wrist_b_motor)
        return out

    @staticmethod
    def _split14(v14: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        v = np.array(v14, dtype=np.float64).reshape((14,))
        return v[:7].copy(), v[7:].copy()

    @staticmethod
    def _merge14(l7: np.ndarray, r7: np.ndarray) -> np.ndarray:
        out = np.zeros((14,), dtype=np.float64)
        out[:7] = np.array(l7, dtype=np.float64).reshape((7,))
        out[7:] = np.array(r7, dtype=np.float64).reshape((7,))
        return out

    def joint_to_motor_position(self, q14: np.ndarray) -> np.ndarray:
        l7, r7 = self._split14(q14)
        l7 = self._clamp_joint_segment(l7, is_left=True)
        r7 = self._clamp_joint_segment(r7, is_left=False)
        # 占位：透传 + motor clamp
        return self._merge14(self._clamp_motor_segment(l7), self._clamp_motor_segment(r7))

    def motor_to_joint_position(self, p14: np.ndarray) -> np.ndarray:
        l7, r7 = self._split14(p14)
        l7 = self._clamp_motor_segment(l7)
        r7 = self._clamp_motor_segment(r7)
        # 占位：透传 + joint clamp
        return self._merge14(self._clamp_joint_segment(l7, is_left=True), self._clamp_joint_segment(r7, is_left=False))

