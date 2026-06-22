#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FullBodyPoseMapper：将 solver 输出映射到全身 MuJoCo 场景 qpos。

用途：
  - solver 验证 GUI 的"全身可视化"模式：把 leg12 / waist / arm14 的 solver 输出
    写入 biped_sNN/xml/scene.xml 的 qpos，让用户看到 solver 输出对完整机器人姿态的影响。
  - 可独立使用，也可被 solver_test_gui.py / solver_test_web_panel.py 调用。

接口：
  mapper = FullBodyPoseMapper(version="45")
  mapper.apply_leg12(leg12_joint)        # 写入 12D leg joint 到全身 qpos
  mapper.apply_leg12_motor(leg12_motor)  # 写入 12D leg motor (含 ankle solver j2m)
  mapper.apply_waist(waist_joint)        # 写入 3D waist joint
  mapper.apply_arm14(arm14_joint)        # 写入 14D arm joint
  mapper.forward()                       # mj_forward 并返回 (model, data)
"""

from __future__ import annotations

import json
import os
from typing import Any, Dict, Optional, Tuple

import numpy as np

try:
    import mujoco
except ImportError:
    raise RuntimeError("FullBodyPoseMapper 需要 mujoco Python 包")


# 资源路径辅助
_ASSETS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "kuavo_assets"))
_LIB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib"))
_SCRIPTS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

if _LIB_ROOT not in os.path.abspath(os.path.dirname(__file__)):
    import sys
    if _LIB_ROOT not in sys.path:
        sys.path.insert(0, _LIB_ROOT)
    if _SCRIPTS_ROOT not in sys.path:
        sys.path.insert(0, _SCRIPTS_ROOT)


def _scene_xml_for_version(version: str) -> str:
    """返回 biped_sNN/xml/scene.xml 的绝对路径。"""
    v = version.lstrip("v").lstrip("s")
    path = os.path.join(_ASSETS_ROOT, "models", f"biped_s{v}", "xml", "scene.xml")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"全身场景模型不存在: {path}（版本 {version} 可能尚无 scene.xml）")
    return path


def _kuavo_json_for_version(version: str) -> str:
    """返回 kuavo_vNN/kuavo.json 的绝对路径。"""
    v = version.lstrip("v").lstrip("s")
    path = os.path.join(_ASSETS_ROOT, "config", f"kuavo_v{v}", "kuavo.json")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"kuavo.json 不存在: {path}")
    return path


# leg12 → 全身 scene joint 的映射表
# scene.xml 中 leg joint 命名: leg_l1..l6 / leg_r1..r6
# leg12 solver 约定: [hip_yaw(0), hip_roll(1), hip_pitch(2), knee(3), ankle_pitch(4), ankle_roll(5),
#                     6,7,8(reserved), hip_yaw(9), hip_roll(10)... knee(9), ankle_pitch(10), ankle_roll(11)]
# 注意: leg12 中的 ankle_pitch/roll 只有 solver 意义；scene.xml 中 ankle pitch/roll 是
#       leg_l5/l6 (左) / leg_r5/r6 (右) — 映射: l4→l4, l5→l5, l6→l6

_LEG12_TO_SCENE_LEFT = {
    0: "leg_l1_joint",   # hip_yaw
    1: "leg_l2_joint",   # hip_roll
    2: "leg_l3_joint",   # hip_pitch
    3: "leg_l4_joint",   # knee
    4: "leg_l5_joint",   # ankle_pitch (solver 输出的 joint-space ankle)
    5: "leg_l6_joint",   # ankle_roll
}

_LEG12_TO_SCENE_RIGHT = {
    0: "leg_r1_joint",
    1: "leg_r2_joint",
    2: "leg_r3_joint",
    3: "leg_r4_joint",
    4: "leg_r5_joint",
    5: "leg_r6_joint",
}


class FullBodyPoseMapper:
    """将 solver 输出映射到全身 MuJoCo scene.xml 的 qpos。"""

    def __init__(self, version: str, scene_xml_path: Optional[str] = None):
        self.version = version.lstrip("v").lstrip("s")
        self.scene_xml_path = scene_xml_path or _scene_xml_for_version(self.version)
        self.kuavo_json_path = _kuavo_json_for_version(self.version)

        self.model = mujoco.MjModel.from_xml_path(self.scene_xml_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        # 构建 joint_name → qpos_adr 映射
        self.joint_qpos_map: Dict[str, int] = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.joint_qpos_map[name] = int(self.model.jnt_qposadr[i])

        # 读取 kuavo.json
        with open(self.kuavo_json_path, "r", encoding="utf-8") as f:
            self.kuavo_json = json.load(f)

        self.ankle_solver_type = self.kuavo_json.get("ankle_solver_type", "none")
        self.num_waist_joint = int(self.kuavo_json.get("NUM_WAIST_JOINT", 0))
        self.num_arm_joint = int(self.kuavo_json.get("NUM_ARM_JOINT", 0))
        self.num_head_joint = int(self.kuavo_json.get("NUM_HEAD_JOINT", 0))

    def _set_qpos(self, joint_name: str, value: float) -> None:
        """安全设置 qpos，忽略场景中不存在的关节。"""
        adr = self.joint_qpos_map.get(joint_name)
        if adr is not None:
            self.data.qpos[adr] = float(value)

    def apply_leg12(self, leg12_joint: np.ndarray) -> None:
        """将 12D leg12 joint 向量写入全身 qpos。

        leg12_joint 布局: [L_hip_yaw, L_hip_roll, L_hip_pitch, L_knee,
                           L_ankle_pitch, L_ankle_roll, 0, 0, 0,
                           R_hip_yaw, R_hip_roll, R_hip_pitch, R_knee,
                           R_ankle_pitch, R_ankle_roll]
        仅写前 6 列（左右各 6 关节）。
        """
        q = np.asarray(leg12_joint, dtype=float).reshape((12,))
        # 左腿: leg12[0:6] → leg_l1..l6
        for leg_idx, scene_name in _LEG12_TO_SCENE_LEFT.items():
            self._set_qpos(scene_name, float(q[leg_idx]))
        # 右腿: leg12[6:12] → leg_r1..r6
        for leg_idx, scene_name in _LEG12_TO_SCENE_RIGHT.items():
            self._set_qpos(scene_name, float(q[leg_idx + 6]))

    def apply_leg12_motor(self, leg12_motor: np.ndarray, solver: Any = None) -> None:
        """将 12D leg12 motor 向量（含 ankle bar）通过 solver 反解为 joint 并写入全身 qpos。

        如果提供 solver 实例，先用 motor_to_joint_position_leg12 反解；
        否则直接写入（仅 hip/knee 1:1 部分，ankle 部分跳过）。
        """
        p = np.asarray(leg12_motor, dtype=float).reshape((12,))
        if solver is not None:
            q = np.asarray(solver.motor_to_joint_position_leg12(p), dtype=float).reshape((12,))
            self.apply_leg12(q)
        else:
            # 无 solver 时仅写 1:1 部分（hip_yaw/roll/pitch/knee，不含 ankle）
            for leg_idx in (0, 1, 2, 3):
                self._set_qpos(_LEG12_TO_SCENE_LEFT[leg_idx], float(p[leg_idx]))
            for leg_idx in (0, 1, 2, 3):
                self._set_qpos(_LEG12_TO_SCENE_RIGHT[leg_idx], float(p[leg_idx + 6]))

    def apply_waist(self, waist_joint: np.ndarray) -> None:
        """将 3D waist joint [yaw, pitch, roll] 写入全身 qpos。"""
        q = np.asarray(waist_joint, dtype=float).reshape((3,))
        if self.num_waist_joint >= 1:
            self._set_qpos("waist_yaw_joint", float(q[0]))
        if self.num_waist_joint >= 2:
            self._set_qpos("waist_pitch_joint", float(q[1]))
        if self.num_waist_joint >= 3:
            self._set_qpos("waist_roll_joint", float(q[2]))

    def apply_arm14(self, arm14_joint: np.ndarray) -> None:
        """将 14D arm joint 写入全身 qpos。

        arm14 布局: [L_shoulder_yaw, L_shoulder_roll, L_shoulder_pitch, L_elbow,
                      L_elbow_rotation, L_wrist_roll, L_wrist_pitch,
                      R_shoulder_yaw, R_shoulder_roll, R_shoulder_pitch, R_elbow,
                      R_elbow_rotation, R_wrist_roll, R_wrist_pitch]
        scene 命名: zarm_l1..l7 / zarm_r1..r7
        """
        q = np.asarray(arm14_joint, dtype=float).reshape((14,))
        for i in range(7):
            self._set_qpos(f"zarm_l{i+1}_joint", float(q[i]))
            self._set_qpos(f"zarm_r{i+1}_joint", float(q[i + 7]))

    def apply_state24(self, state24: np.ndarray) -> None:
        """从 state24 向量写入全身姿态（兼容 pose_io.State24 格式）。

        state24 布局: [6 placeholder, base_xyz(3), base_rpy(3), leg12(12)]
        """
        v = np.asarray(state24, dtype=float).reshape((24,))
        # base pose (qpos[0:7])
        self.data.qpos[0] = 1.0   # root quaternion w (站立)
        self.data.qpos[1] = 0.0   # x
        self.data.qpos[2] = 0.0   # y
        self.data.qpos[3] = float(v[6])  # z (base height)
        # base orientation via Euler → quaternion (simplified: upright)
        # v[9:12] = base_rpy_rad — for now just keep upright
        # leg12
        leg12 = v[12:24]
        self.apply_leg12(leg12)

    def forward(self) -> Tuple[Any, Any]:
        """执行 mj_forward 并返回 (model, data)。"""
        mujoco.mj_forward(self.model, self.data)
        return self.model, self.data

    def reset(self) -> None:
        """重置到默认姿态。"""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def get_qpos_copy(self) -> np.ndarray:
        """返回当前 qpos 的拷贝。"""
        return np.array(self.data.qpos, dtype=float).copy()

    def render_offscreen(self, width: int = 640, height: int = 480) -> np.ndarray:
        """离屏渲染，返回 RGB 图像 (height, width, 3)。"""
        renderer = mujoco.Renderer(self.model, height=height, width=width)
        renderer.update_scene(self.data)
        return renderer.render()