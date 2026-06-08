#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一解算器验证框架的机构规格（Specification）。

目标
----
把"踝 / 膝 / 腰 / 肘 / 腕"五个并联机构抽象成同一份 `ValidationSpec`：
- MuJoCo 侧的 qpos/qvel 读写
- Solver 公共位置/速度互转接口（viewer/roundtrip 的唯一画图来源）
- Jacobian 探针（MuJoCo 真值：`mj_jacSite`/`efc_J`/点Jacobian 等与 Roban 验证脚本对齐；膝/肘 connect 双侧 site 的 Jacobian 差分行解 ``dp_bar``，与 solver 的 `joint_to_motor_velocity*` 对比。）

强约束
------
1. 所有 solver 调用都走原始的 `motor_to_joint_*` / `joint_to_motor_*` 接口；
2. 验证/画图不吞异常，调用失败立即抛出（user-rule：no silent fallback）；
3. 读写 MJ 状态统一通过 spec 内的 helper，保证"测什么就写什么"。
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Tuple

import numpy as np

_LIB = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib"))
if _LIB not in sys.path:
    sys.path.insert(0, _LIB)

# 让 `from kuavo_paths import ...` 在本文件被独立运行时也可用
_SCRIPTS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS_ROOT not in sys.path:
    sys.path.insert(0, _SCRIPTS_ROOT)

try:
    import mujoco  # noqa: F401  (仅做类型暗示，builder 调用时才真正需要)
except Exception as e:  # pragma: no cover
    raise RuntimeError("solver_validation_spec 需要 mujoco Python 包") from e

from kuavo_paths import kuavo_solver_package_root
from solver_selection import SolverSelection, load_default_index
from validation_common import default_mjcf_for

_KUAVO_SOLVER_PKG = kuavo_solver_package_root(__file__)


# ---------------------------------------------------------------------------
# 通用数据结构
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ChannelPlan:
    """
    GUI / 误差统计关心的一条通道：

    - ``key``      唯一 id（字母数字）
    - ``label``    显示名
    - ``side``     "left" | "right" | "shared"，驱动左右分 Tab
    - ``q_index``  在 spec.dim_joint 向量中的索引；为 None 表示该通道无关节量（例如纯 motor）
    - ``p_index``  在 spec.dim_motor 向量中的索引；为 None 表示纯关节通道
    - ``q_name``   用于 UI 的短名称（如 "L_pitch"）
    - ``p_name``   用于 UI 的短名称（如 "L_l_bar"）；None 时 GUI 不显示 motor 曲线
    """

    key: str
    label: str
    side: str
    q_index: Optional[int]
    p_index: Optional[int]
    q_name: Optional[str] = None
    p_name: Optional[str] = None


@dataclass
class JacobianProbe:
    """
    一次采样下的 Jacobian 对比结果。

    必填字段
    --------
    - ``dp_solver``     solver 原生接口 `joint_to_motor_velocity(q, p, dq)` 的输出截取
    - ``dp_mj``         MuJoCo 侧定义的 ``dp``（`mj_jacSite` 差 / `efc_J` / 点 Jac 等；同 shape）

    可选字段（当机构本身暴露干净的 (Jc, Ja) 对时填充）
    ---------------------------------------------------
    - ``J_constraint_sv`` / ``J_constraint_mj``  形状 (n_cons, n_joint_eff)
    - ``J_actuator_sv``   / ``J_actuator_mj``    形状 (n_cons, n_motor_eff)

    约定：同机构下 sv/mj 两侧的矩阵 shape 必须完全一致，列顺序由 spec 保持一致。
    """

    dp_solver: np.ndarray
    dp_mj: np.ndarray

    J_constraint_sv: Optional[np.ndarray] = None
    J_constraint_mj: Optional[np.ndarray] = None
    J_actuator_sv: Optional[np.ndarray] = None
    J_actuator_mj: Optional[np.ndarray] = None


@dataclass
class ValidationSpec:
    """
    一个并联机构的统一验证规格。GUI / jacobian / roundtrip runner 全走这个对象。
    """

    # 元信息
    module: str           # "ankle" | "knee" | "waist" | "arm_elbow" | "arm_wrist"
    label: str            # 菜单/窗口用
    token: str            # 解算器变体 token，如 "7gen"
    config_dir: str
    params_yaml: Optional[str]
    mjcf_path: str
    dim_joint: int
    dim_motor: int

    # GUI / 误差统计通道
    channels: List[ChannelPlan]

    # solver 原始接口（严格禁止绕过）
    joint_to_motor_position: Callable[[Any, np.ndarray], np.ndarray]
    motor_to_joint_position: Callable[[Any, np.ndarray], np.ndarray]
    joint_to_motor_velocity: Callable[[Any, np.ndarray, np.ndarray, np.ndarray], np.ndarray]
    motor_to_joint_velocity: Callable[[Any, np.ndarray, np.ndarray, np.ndarray], np.ndarray]

    # solver 构造（从 pybind module → solver 实例）
    construct_solver: Callable[[Any], Any]

    # MJ 状态读写
    read_q_from_mj: Callable[[Any, Any], np.ndarray]   # (model, data) -> q (dim_joint,)
    read_p_from_mj: Callable[[Any, Any], np.ndarray]   # -> p (dim_motor,)
    read_dq_from_mj: Callable[[Any, Any], np.ndarray]
    read_dp_from_mj: Callable[[Any, Any], np.ndarray]
    write_state_to_mj: Callable[[Any, Any, np.ndarray, np.ndarray], None]  # 写 q,p

    # 采样
    sample_q: Callable[[np.random.Generator], np.ndarray]
    sample_dq: Callable[[np.random.Generator, np.ndarray], np.ndarray]

    # Jacobian 探针（必填；内部可使用 efc_J / point-Jacobian 任一路径）
    jacobian_probe: Callable[[Any, Any, Any, np.ndarray, np.ndarray, np.ndarray], JacobianProbe]

    # 选配（带默认值的字段须排在无默认值字段之后）
    notes: str = ""
    extra: dict = field(default_factory=dict)
    # 写 q,p 后闭合 MuJoCo equality 被动（膝/肘）；缺省则 runner 仅 write + mj_forward
    sync_mujoco_after_write: Optional[Callable[[Any, Any, np.ndarray, np.ndarray], None]] = None
    # MJ 读回比较的 q/p 分量（缺省 None=全向量）。膝 leg12×最小膝 MJ 时须限到 MJ 真有自由度的条目。
    mj_roundtrip_q_indices: Optional[Tuple[int, ...]] = None
    mj_roundtrip_p_indices: Optional[Tuple[int, ...]] = None


# ---------------------------------------------------------------------------
# MJ 侧公共 helper
# ---------------------------------------------------------------------------


def _require_joint(model: Any, name: str) -> Any:
    try:
        return model.joint(name)
    except KeyError as e:
        raise RuntimeError(f"MJCF 中缺少关节: {name}") from e


def _qpos_adr(model: Any, name: str) -> int:
    j = _require_joint(model, name)
    if getattr(j, "qposadr", None) is None or j.qposadr.size <= 0:
        raise RuntimeError(f"关节无 qposadr: {name}")
    return int(j.qposadr[0])


def _dof_adr(model: Any, name: str) -> int:
    j = _require_joint(model, name)
    if getattr(j, "dofadr", None) is None or j.dofadr.size <= 0:
        raise RuntimeError(f"关节无 dofadr: {name}")
    return int(j.dofadr[0])


def _eq_id(mjc: Any, model: Any, name: str) -> int:
    eid = int(mjc.mj_name2id(model, mjc.mjtObj.mjOBJ_EQUALITY, name))
    if eid < 0:
        raise RuntimeError(f"MJCF 中缺少 equality: {name}")
    return eid


def _site_id(mjc: Any, model: Any, name: str) -> int:
    sid = int(mjc.mj_name2id(model, mjc.mjtObj.mjOBJ_SITE, name))
    if sid < 0:
        raise RuntimeError(f"MJCF 中缺少 site: {name}")
    return sid


def _efc_rows_for_eq(data: Any, eq_id: int) -> np.ndarray:
    rows = [i for i in range(int(data.nefc)) if int(data.efc_type[i]) == 0 and int(data.efc_id[i]) == int(eq_id)]
    if not rows:
        raise RuntimeError(f"efc_J 中无 eq_id={eq_id} 的位置约束行")
    return np.array(rows, dtype=int)


def _efc_j_matrix(model: Any, data: Any) -> np.ndarray:
    J = np.asarray(data.efc_J, dtype=float)
    if J.ndim == 1:
        J = J.reshape((int(data.nefc), int(model.nv)))
    return J


def _solve_min_norm(Ju: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    u, *_ = np.linalg.lstsq(np.asarray(Ju, dtype=float), np.asarray(rhs, dtype=float), rcond=None)
    return np.array(u, dtype=float).reshape((-1,))


def _jac_site_geom(model: Any, data: Any, mjc: Any, site_id: int) -> np.ndarray:
    jacp = np.zeros((3, int(model.nv)), dtype=float)
    mjc.mj_jacSite(model, data, jacp, None, site_id)
    return np.asarray(jacp, dtype=float)


def _dp_motor_from_connect_jac_site_3dof(
    model: Any, data: Any, mjc: Any, *,
    site_a_name: str,
    site_b_name: str,
    dof_drive_name: str,
    dof_motor_name: str,
    dof_passive_name: str,
    dq_drive: float,
    pair_det_tol: float = 5.0e-10,
    resid_tol: float = 5.0e-6,
) -> float:
    """
    MuJoCo 侧：与同目录 ``kuavo_solver_full/scripts/validate_roban_jacobian.py`` 相同思路：用 ``mj_jacSite``
    读出两锚点线性速度块，取差 ``Jdiff = jac_site_a - jac_site_b``（与世界系位置差 ``r=x_a-x_b`` 相容）。

    瞬时速率满足 ``jd * dq_drive + jac_motor * dp_motor + jac_pass * dq_pass = 0``（按行）。
    在 3 行中选行列式满足的 2 行做成 2×2，用 ``numpy.linalg.solve`` 解 ``dp_motor, dq_pass``（非 lstsq）。

    ``site_a/site_b`` 应对齐 equality ``connect`` 两侧的几何锚点（膝：tendon_site 与 knee_eq）。
    """
    sid_a = _site_id(mjc, model, site_a_name)
    sid_b = _site_id(mjc, model, site_b_name)
    ja = _jac_site_geom(model, data, mjc, sid_a)
    jb = _jac_site_geom(model, data, mjc, sid_b)
    Jdiff = ja - jb
    dof_drive = _dof_adr(model, dof_drive_name)
    dof_motor = _dof_adr(model, dof_motor_name)
    dof_pass = _dof_adr(model, dof_passive_name)

    jd = np.asarray(Jdiff[:, dof_drive], dtype=float).reshape((3,))
    rhs_row = jd * float(dq_drive)
    c0 = np.asarray(Jdiff[:, dof_motor], dtype=float).reshape((3,))
    c1 = np.asarray(Jdiff[:, dof_pass], dtype=float).reshape((3,))
    jbvec = -rhs_row

    pairs = ((0, 1), (0, 2), (1, 2))
    best_res = float("inf")
    best_x: Optional[np.ndarray] = None
    for ia, ib in pairs:
        A = np.array([[float(c0[ia]), float(c1[ia])], [float(c0[ib]), float(c1[ib])]], dtype=float)
        if abs(np.linalg.det(A)) < float(pair_det_tol):
            continue
        bt = np.array([jbvec[ia], jbvec[ib]], dtype=float).reshape((2,))
        try:
            x = np.linalg.solve(A, bt)
        except np.linalg.LinAlgError:
            continue
        residual3 = jbvec - c0 * float(x[0]) - c1 * float(x[1])
        rn = float(np.linalg.norm(np.asarray(residual3, dtype=float).reshape((3,))))
        if rn < best_res:
            best_res = rn
            best_x = x
    sid_list = ",".join((site_a_name, site_b_name))
    if best_x is None:
        raise RuntimeError(
            "connect Jacobian：无法从 mj_jacSite 差中找到可逆 2×2 行对 "
            f"({dof_drive_name}->{dof_motor_name}+{dof_passive_name}) "
            f"sites=({sid_list})"
        )
    if best_res > float(resid_tol):
        raise RuntimeError(
            f"connect Jacobian：三行一致性残差 ‖res‖ = {best_res:.3e} > tol={resid_tol} sites=({sid_list})"
        )
    return float(best_x[0])


def _mj_newton_solve_passive(
    mjc: Any, model: Any, data: Any, *,
    eq_name: str,
    driver_joint: str,
    driver_qpos: float,
    passive_joints: Tuple[str, ...],
    passive_qpos_guess: Tuple[float, ...],
    tol: float = 1e-6,
    max_iter: int = 40,
    stagnation_ok_below: float = 2e-5,
) -> Tuple[float, ...]:
    """
    纯 MJ 侧闭合环正解（不依赖 solver）：给定主动关节 qpos，
    用 equality 约束的 efc_J 与残差 efc_pos 做 Newton 迭代，
    解出被动关节（bar / prismatic / tendon / input_joint 等）的 qpos，
    使该 equality 对应的约束残差 → 0。

    返回解得的 passive_joints 的 qpos 元组（与入参顺序一致）。

    约束（位置类 equality）是 qpos 的非线性方程 r(qpos)=0；
    在 driver 固定的子空间内，对 passive 列做 Gauss-Newton 步：
        Ju · Δu = -r          →   u += Δu      直到 ||r||<tol。

    ``tol``：MuJoCo ``efc_pos`` 默认取 1e-6 量级即可（过小会导致 LM 在 ~1e-6 的平台无法继续下降却仍判失败）。
    ``stagnation_ok_below``：若 LM 单步无法再降低残差，但 ‖r‖ 低于该阈值，视为已闭合（单次被动自由度下常见数值平台）。
    """
    data.qpos[_qpos_adr(model, driver_joint)] = float(driver_qpos)
    passive_adr = [_qpos_adr(model, n) for n in passive_joints]
    passive_dof = [_dof_adr(model, n) for n in passive_joints]
    for adr, g in zip(passive_adr, passive_qpos_guess):
        data.qpos[adr] = float(g)

    eq_id = _eq_id(mjc, model, eq_name)
    # Levenberg-Marquardt：H = J^T J + mu·I；接受步则缩小 mu，否则放大 mu 重试。
    # 对非线性闭环 4-bar（knee）在大 q 下远比 Gauss-Newton 鲁棒。
    mu = 1e-3
    mu_min = 1e-12
    mu_max = 1e8
    last_norm = float("inf")
    for _ in range(max_iter):
        mjc.mj_forward(model, data)
        rows = _efc_rows_for_eq(data, eq_id)
        res = np.asarray(data.efc_pos[rows], dtype=float).reshape((-1,))
        res_norm = float(np.linalg.norm(res))
        last_norm = res_norm
        if res_norm < tol:
            break
        J = _efc_j_matrix(model, data)[rows, :]
        Ju = J[:, passive_dof]
        JTJ = Ju.T @ Ju
        g = Ju.T @ res
        snapshot = np.array([data.qpos[adr] for adr in passive_adr], dtype=float)
        # 内循环：调整 mu 直到找到能让残差下降的步
        accepted = False
        for _trial in range(40):
            H = JTJ + mu * np.eye(JTJ.shape[0])
            try:
                du = np.linalg.solve(H, -g)
            except np.linalg.LinAlgError:
                mu = min(mu * 10.0, mu_max)
                continue
            for k, adr in enumerate(passive_adr):
                data.qpos[adr] = float(snapshot[k] + du[k])
            mjc.mj_forward(model, data)
            rows2 = _efc_rows_for_eq(data, eq_id)
            res2 = np.asarray(data.efc_pos[rows2], dtype=float).reshape((-1,))
            if float(np.linalg.norm(res2)) < res_norm:
                accepted = True
                mu = max(mu * 0.5, mu_min)
                break
            # 回退并加大阻尼
            for k, adr in enumerate(passive_adr):
                data.qpos[adr] = float(snapshot[k])
            mu = min(mu * 5.0, mu_max)
            if mu >= mu_max:
                break
        if not accepted:
            # 单侧被动自由度近似闭链时常在 ~1e-6 ‖r‖ 处进入平台：LM 无法进一步下降但已优于工程阈值
            if res_norm <= float(stagnation_ok_below):
                break
            raise RuntimeError(
                f"_mj_newton_solve_passive({eq_name}) LM step failed for "
                f"{driver_joint}={driver_qpos:.6f}; residual={res_norm:.3e}, mu={mu:.3e}"
            )
    else:
        raise RuntimeError(
            f"_mj_newton_solve_passive({eq_name}) did not converge for "
            f"{driver_joint}={driver_qpos:.6f}; last residual={last_norm:.3e}"
        )
    return tuple(float(data.qpos[adr]) for adr in passive_adr)


# ---------------------------------------------------------------------------
# 解析：通用入口（按 module 名给出 spec）
# ---------------------------------------------------------------------------


# module → (solver 侧类名, 是否通过 version index 的 "arm"/"waist"/"ankle" module 查）
_MODULE_INDEX_KEY = {
    "ankle": "ankle",
    "knee": "ankle",
    "waist": "waist",
    "arm_elbow": "arm",
    "arm_wrist": "arm",
}


def resolve_selection(version: str, module: str) -> SolverSelection:
    idx = load_default_index()
    key = _MODULE_INDEX_KEY.get(module)
    if key is None:
        raise ValueError(f"未知 module: {module}")
    return idx.resolve(version, module=key)


def mjcf_for_module(module: str, token: str) -> str:
    if module == "ankle":
        return default_mjcf_for(package_root=_KUAVO_SOLVER_PKG, module="ankle", token=token)
    if module == "knee":
        return default_mjcf_for(package_root=_KUAVO_SOLVER_PKG, module="knee", token=token)
    if module == "waist":
        return default_mjcf_for(package_root=_KUAVO_SOLVER_PKG, module="waist", token=token)
    if module == "arm_elbow":
        return default_mjcf_for(package_root=_KUAVO_SOLVER_PKG, module="arm_elbow", token=token)
    if module == "arm_wrist":
        return default_mjcf_for(package_root=_KUAVO_SOLVER_PKG, module="arm_wrist", token=token)
    raise ValueError(f"未知 module: {module}")


def build_spec(*, module: str, version: str, solver_module: Any) -> ValidationSpec:
    """
    根据 module 名 + 版本号构造 ValidationSpec。

    solver_module: 已导入的 kuavo_solver_py（注入使得模块不依赖全局 import）。
    """
    sel = resolve_selection(version, module)
    mjcf_path = mjcf_for_module(module, sel.token)
    if not os.path.isfile(mjcf_path):
        raise FileNotFoundError(f"MJCF 不存在: {mjcf_path}")

    if module == "ankle":
        return _build_ankle_spec(sel=sel, mjcf_path=mjcf_path, solver_module=solver_module)
    if module == "knee":
        return _build_knee_spec(sel=sel, mjcf_path=mjcf_path, solver_module=solver_module)
    if module == "waist":
        return _build_waist_spec(sel=sel, mjcf_path=mjcf_path, solver_module=solver_module)
    if module == "arm_elbow":
        return _build_arm_elbow_spec(sel=sel, mjcf_path=mjcf_path, solver_module=solver_module)
    if module == "arm_wrist":
        return _build_arm_wrist_spec(sel=sel, mjcf_path=mjcf_path, solver_module=solver_module)
    raise ValueError(f"未知 module: {module}")


# ---------------------------------------------------------------------------
# 踝（AnkleSolver, 4 维 axisoffset）
# ---------------------------------------------------------------------------


def _build_ankle_spec(*, sel: SolverSelection, mjcf_path: str, solver_module: Any) -> ValidationSpec:
    joint_names_q = ("l_foot_pitch", "l_foot_roll", "r_foot_pitch", "r_foot_roll")
    joint_names_p = ("l_l_bar", "l_r_bar", "r_l_bar", "r_r_bar")

    # MJ site 命名在不同代次有差异，这里延续 MujocoAnkleAdapter 的兼容习惯
    td_aliases = {
        "l_l": ("l_l_tendon", "l_l_bar_tendon"),
        "l_r": ("l_r_tendon", "l_r_bar_tendon"),
        "r_l": ("r_l_tendon", "r_l_bar_tendon"),
        "r_r": ("r_r_tendon", "r_r_bar_tendon"),
    }

    def _site_id(mjc: Any, model: Any, *candidates: str) -> int:
        for name in candidates:
            try:
                return int(model.site(name).id)
            except KeyError:
                continue
        raise RuntimeError(f"MJCF 中缺少 site，候选: {candidates}")

    channels: List[ChannelPlan] = [
        ChannelPlan(key="L_pitch", label="L_pitch", side="left",  q_index=0, p_index=None, q_name="L_pitch"),
        ChannelPlan(key="L_roll",  label="L_roll",  side="left",  q_index=1, p_index=None, q_name="L_roll"),
        ChannelPlan(key="R_pitch", label="R_pitch", side="right", q_index=2, p_index=None, q_name="R_pitch"),
        ChannelPlan(key="R_roll",  label="R_roll",  side="right", q_index=3, p_index=None, q_name="R_roll"),
    ]

    # AxisOffsetAnkleSolver 的 4D 约定为 [L_pitch, L_roll, R_pitch, R_roll]，
    # 与 AnkleSolver 经 leg12 slot (4,5,10,11) / 左腿电机交叉 的 Pack4 一致后方可闭合 roundtrip。
    def construct(module_py: Any) -> Any:
        import yaml
        if not sel.params_yaml:
            raise RuntimeError("ankle spec 需要 axisoffsetanklesolver.yaml（index.params_yaml）")
        with open(sel.params_yaml, "r", encoding="utf-8") as f:
            root = yaml.safe_load(f) or {}
        variants = (root.get("variants") or {}) if isinstance(root, dict) else {}
        if sel.token not in variants:
            raise RuntimeError(
                f"axisoffset variants 中无 token={sel.token!r}；该 token 可能是 fixed-axis 变体，"
                "不适用于 ankle spec（请换定轴/非 axisoffset 的 module）"
            )
        params = module_py.AxisOffsetAnkleParams()
        v = variants[sel.token] or {}
        for attr in dir(params):
            if not attr.startswith("_") and attr in v:
                setattr(params, attr, v[attr])
        return module_py.AxisOffsetAnkleSolver(params)

    def read_q(model: Any, data: Any) -> np.ndarray:
        q = np.zeros(4, dtype=float)
        for i, n in enumerate(joint_names_q):
            q[i] = float(data.qpos[_qpos_adr(model, n)])
        return q

    def read_p(model: Any, data: Any) -> np.ndarray:
        p = np.zeros(4, dtype=float)
        for i, n in enumerate(joint_names_p):
            p[i] = float(data.qpos[_qpos_adr(model, n)])
        return p

    def read_dq(model: Any, data: Any) -> np.ndarray:
        dq = np.zeros(4, dtype=float)
        for i, n in enumerate(joint_names_q):
            dq[i] = float(data.qvel[_dof_adr(model, n)])
        return dq

    def read_dp(model: Any, data: Any) -> np.ndarray:
        dp = np.zeros(4, dtype=float)
        for i, n in enumerate(joint_names_p):
            dp[i] = float(data.qvel[_dof_adr(model, n)])
        return dp

    def write_state(model: Any, data: Any, q: np.ndarray, p: np.ndarray) -> None:
        for i, n in enumerate(joint_names_q):
            data.qpos[_qpos_adr(model, n)] = float(q[i])
        for i, n in enumerate(joint_names_p):
            data.qpos[_qpos_adr(model, n)] = float(p[i])

    def sample_q(rng: np.random.Generator) -> np.ndarray:
        return np.array([
            float(rng.uniform(-0.45, 0.45)),
            float(rng.uniform(-0.25, 0.25)),
            float(rng.uniform(-0.45, 0.45)),
            float(rng.uniform(-0.25, 0.25)),
        ], dtype=float)

    def sample_dq(rng: np.random.Generator, _q: np.ndarray) -> np.ndarray:
        return np.array(rng.uniform(-0.8, 0.8, size=4), dtype=float)

    def jacobian_probe(
        solver: Any, model: Any, data: Any, q: np.ndarray, p: np.ndarray, dq: np.ndarray
    ) -> JacobianProbe:
        import mujoco as mjc  # 本地 import 便于 spec 注入测试

        # 写入一致位形
        write_state(model, data, q, p)
        mjc.mj_forward(model, data)

        # === MJ 侧：对 4 条 tendon 各自算 hat·J 投影，组装 per-side 2×2 ===
        def _hat_and_jacs(side: str, which: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, int, Tuple[int, int]]:
            # 返回 (hat, jac_eq, jac_td, bar_dof, ankle_dofs)
            if side == "left":
                eq_name = "l_l_eq" if which == "l" else "l_r_eq"
                td_name = td_aliases["l_l" if which == "l" else "l_r"]
                bar_joint = "l_l_bar" if which == "l" else "l_r_bar"
                ank_pitch = _dof_adr(model, "l_foot_pitch")
                ank_roll = _dof_adr(model, "l_foot_roll")
            else:
                eq_name = "r_l_eq" if which == "l" else "r_r_eq"
                td_name = td_aliases["r_l" if which == "l" else "r_r"]
                bar_joint = "r_l_bar" if which == "l" else "r_r_bar"
                ank_pitch = _dof_adr(model, "r_foot_pitch")
                ank_roll = _dof_adr(model, "r_foot_roll")
            eq_id = int(model.site(eq_name).id)
            td_id = _site_id(mjc, model, *td_name)
            eq_pos = np.asarray(data.site_xpos[eq_id], dtype=float)
            td_pos = np.asarray(data.site_xpos[td_id], dtype=float)
            vec = td_pos - eq_pos
            hat = vec / max(float(np.linalg.norm(vec)), 1.0e-12)
            jac_eq = np.zeros((3, int(model.nv)), dtype=float)
            jac_td = np.zeros((3, int(model.nv)), dtype=float)
            mjc.mj_jacSite(model, data, jac_eq, None, eq_id)
            mjc.mj_jacSite(model, data, jac_td, None, td_id)
            return hat, jac_eq, jac_td, _dof_adr(model, bar_joint), (ank_pitch, ank_roll)

        # === 按 p 顺序 [l_l, l_r, r_l, r_r] 构造 Jc/Ja（每条 tendon 占一行）===
        # row i：第 i 条 tendon 约束  hat_i · d(tendon_i)/dq + hat_i · d(tendon_i)/dp = 0
        axo = solver  # ValidationSpec.construct_solver 已经构造好 AxisOffsetAnkleSolver

        ts_map = {
            0: ("left",  "l", solver_module.AnkleSide.LEFT,  solver_module.TendonSide.LEFT),
            1: ("left",  "r", solver_module.AnkleSide.LEFT,  solver_module.TendonSide.RIGHT),
            2: ("right", "l", solver_module.AnkleSide.RIGHT, solver_module.TendonSide.LEFT),
            3: ("right", "r", solver_module.AnkleSide.RIGHT, solver_module.TendonSide.RIGHT),
        }

        Jc_full_mj = np.zeros((4, 4), dtype=float)
        Ja_full_mj = np.zeros((4, 4), dtype=float)
        Jc_full_sv = np.zeros((4, 4), dtype=float)
        Ja_full_sv = np.zeros((4, 4), dtype=float)

        for row, (side, which, ankle_side, tendon_side) in ts_map.items():
            # MJ 侧
            hat, J_eq, J_td, bar_dof, (pd, rd) = _hat_and_jacs(side, which)
            q_cols = (0, 1) if side == "left" else (2, 3)
            Jc_full_mj[row, q_cols[0]] = float(-hat @ J_eq[:, pd])
            Jc_full_mj[row, q_cols[1]] = float(-hat @ J_eq[:, rd])
            Ja_full_mj[row, row] = float(hat @ J_td[:, bar_dof])

            # solver 侧（输入关节量来自同侧 q）
            pitch = float(q[0] if side == "left" else q[2])
            roll  = float(q[1] if side == "left" else q[3])
            act   = float(p[row])
            vec = np.asarray(axo.compute_tendon_vector(pitch, roll, act, ankle_side, tendon_side), dtype=float)
            hat_sv = vec / max(float(np.linalg.norm(vec)), 1.0e-12)
            j_ank = np.asarray(axo.compute_jacobian_ankle(pitch, roll, ankle_side, tendon_side), dtype=float).reshape((3, 2))
            j_act = np.asarray(axo.compute_jacobian_actuator(pitch, roll, act, ankle_side, tendon_side), dtype=float).reshape((3,))
            Jc_full_sv[row, q_cols[0]] = float(hat_sv @ j_ank[:, 0])
            Jc_full_sv[row, q_cols[1]] = float(hat_sv @ j_ank[:, 1])
            Ja_full_sv[row, row] = float(hat_sv @ j_act)

        # dp_solver：走 solver 原始 4D 速度接口（同样以 [L_pitch, L_roll, R_pitch, R_roll] 为输入）
        dp_sv_full = np.asarray(
            solver.joint_to_motor_velocity(np.asarray(q, dtype=float),
                                           np.asarray(p, dtype=float),
                                           np.asarray(dq, dtype=float)),
            dtype=float,
        ).reshape((4,))

        # dp_mj：由 Jc_mj @ dq + Ja_mj @ dp = 0 解出；Ja_mj 对角，直接元素相除
        diag_mj = np.diag(Ja_full_mj).copy()
        if float(np.min(np.abs(diag_mj))) < 1.0e-12:
            raise RuntimeError("ankle Ja 对角元过小，无法求 dp_mj")
        dp_mj_full = -(Jc_full_mj @ np.asarray(dq, dtype=float).reshape((4,))) / diag_mj

        return JacobianProbe(
            dp_solver=dp_sv_full,
            dp_mj=dp_mj_full,
            J_constraint_sv=Jc_full_sv, J_constraint_mj=Jc_full_mj,
            J_actuator_sv=Ja_full_sv,   J_actuator_mj=Ja_full_mj,
        )

    return ValidationSpec(
        module="ankle",
        label=f"踝 (ankle, token={sel.token})",
        token=sel.token, config_dir=sel.config_dir, params_yaml=sel.params_yaml,
        mjcf_path=mjcf_path, dim_joint=4, dim_motor=4,
        channels=channels,
        joint_to_motor_position=lambda s, q: np.asarray(s.joint_to_motor_position(np.asarray(q, dtype=float)), dtype=float).reshape((4,)),
        motor_to_joint_position=lambda s, p: np.asarray(s.motor_to_joint_position(np.asarray(p, dtype=float)), dtype=float).reshape((4,)),
        joint_to_motor_velocity=lambda s, q, p, dq: np.asarray(s.joint_to_motor_velocity(np.asarray(q, dtype=float), np.asarray(p, dtype=float), np.asarray(dq, dtype=float)), dtype=float).reshape((4,)),
        motor_to_joint_velocity=lambda s, q, p, dp: np.asarray(s.motor_to_joint_velocity(np.asarray(q, dtype=float), np.asarray(p, dtype=float), np.asarray(dp, dtype=float)), dtype=float).reshape((4,)),
        construct_solver=construct,
        read_q_from_mj=read_q, read_p_from_mj=read_p,
        read_dq_from_mj=read_dq, read_dp_from_mj=read_dp,
        write_state_to_mj=write_state,
        sample_q=sample_q, sample_dq=sample_dq,
        jacobian_probe=jacobian_probe,
        notes="axisoffset 4-bar；Jc/Ja 为 per-side 2×2（行序 outer/inner，列序 pitch/roll）",
    )


# ---------------------------------------------------------------------------
# 膝（AnkleSolver 的 leg12 接口，nv_eff=1/side）
# ---------------------------------------------------------------------------


def _build_knee_spec(*, sel: SolverSelection, mjcf_path: str, solver_module: Any) -> ValidationSpec:
    # leg12 里 knee=idx3/9, bar=idx3/9（q/p 共用 12 维向量）
    idx_L_knee = 3
    idx_R_knee = 9

    channels: List[ChannelPlan] = [
        ChannelPlan(key="L_knee", label="L_knee", side="left",  q_index=idx_L_knee, p_index=idx_L_knee, q_name="L_knee", p_name="L_bar"),
        ChannelPlan(key="R_knee", label="R_knee", side="right", q_index=idx_R_knee, p_index=idx_R_knee, q_name="R_knee", p_name="R_bar"),
    ]

    joint_L_knee, joint_L_bar, joint_L_tdn = "l_knee", "l_knee_bar", "l_knee_tendon"
    joint_R_knee, joint_R_bar, joint_R_tdn = "r_knee", "r_knee_bar", "r_knee_tendon"

    def construct(module_py: Any) -> Any:
        # 注意 knee 走的是 AnkleSolver 的 leg12 接口
        return module_py.AnkleSolver(sel.token, sel.config_dir)

    def read_q(model: Any, data: Any) -> np.ndarray:
        q = np.zeros(12, dtype=float)
        q[idx_L_knee] = float(data.qpos[_qpos_adr(model, joint_L_knee)])
        q[idx_R_knee] = float(data.qpos[_qpos_adr(model, joint_R_knee)])
        return q

    def read_p(model: Any, data: Any) -> np.ndarray:
        p = np.zeros(12, dtype=float)
        p[idx_L_knee] = float(data.qpos[_qpos_adr(model, joint_L_bar)])
        p[idx_R_knee] = float(data.qpos[_qpos_adr(model, joint_R_bar)])
        return p

    def read_dq(model: Any, data: Any) -> np.ndarray:
        dq = np.zeros(12, dtype=float)
        dq[idx_L_knee] = float(data.qvel[_dof_adr(model, joint_L_knee)])
        dq[idx_R_knee] = float(data.qvel[_dof_adr(model, joint_R_knee)])
        return dq

    def read_dp(model: Any, data: Any) -> np.ndarray:
        dp = np.zeros(12, dtype=float)
        dp[idx_L_knee] = float(data.qvel[_dof_adr(model, joint_L_bar)])
        dp[idx_R_knee] = float(data.qvel[_dof_adr(model, joint_R_bar)])
        return dp

    def write_state(model: Any, data: Any, q: np.ndarray, p: np.ndarray) -> None:
        data.qpos[_qpos_adr(model, joint_L_knee)] = float(q[idx_L_knee])
        data.qpos[_qpos_adr(model, joint_R_knee)] = float(q[idx_R_knee])
        data.qpos[_qpos_adr(model, joint_L_bar)] = float(p[idx_L_knee])
        data.qpos[_qpos_adr(model, joint_R_bar)] = float(p[idx_R_knee])
        data.qpos[_qpos_adr(model, joint_L_tdn)] = 0.0
        data.qpos[_qpos_adr(model, joint_R_tdn)] = 0.0

    def sync_mujoco_after_write(model: Any, data: Any, q: np.ndarray, p: np.ndarray) -> None:
        import mujoco as mjc
        write_state(model, data, q, p)
        qv = np.asarray(q, dtype=float).reshape((12,))
        _mj_newton_solve_passive(
            mjc, model, data, eq_name="l_knee_eq", driver_joint=joint_L_knee,
            driver_qpos=float(qv[idx_L_knee]), passive_joints=(joint_L_tdn,),
            passive_qpos_guess=(float(data.qpos[_qpos_adr(model, joint_L_tdn)]),),
        )
        _mj_newton_solve_passive(
            mjc, model, data, eq_name="r_knee_eq", driver_joint=joint_R_knee,
            driver_qpos=float(qv[idx_R_knee]), passive_joints=(joint_R_tdn,),
            passive_qpos_guess=(float(data.qpos[_qpos_adr(model, joint_R_tdn)]),),
        )
        mjc.mj_forward(model, data)

    def sample_q(rng: np.random.Generator) -> np.ndarray:
        q = np.zeros(12, dtype=float)
        q[idx_L_knee] = float(rng.uniform(0.05, 2.45))
        q[idx_R_knee] = float(rng.uniform(0.05, 2.45))
        return q

    def sample_dq(rng: np.random.Generator, _q: np.ndarray) -> np.ndarray:
        dq = np.zeros(12, dtype=float)
        dq[idx_L_knee] = float(rng.uniform(-0.8, 0.8))
        dq[idx_R_knee] = float(rng.uniform(-0.8, 0.8))
        return dq

    def jacobian_probe(
        solver: Any, model: Any, data: Any, q: np.ndarray, p: np.ndarray, dq: np.ndarray
    ) -> JacobianProbe:
        """
        MJ 侧：`validate_roban_jacobian.py` 风格：两轮 ``mj_jacSite`` 差 + 给定 ``dq_knee``
        对 ``(dof_bar,dof_tendon)`` 用 3×3 中取 2 行方阵 ``solve``（非 lstsq）得 ``dp_bar``。
        """
        import mujoco as mjc

        qf = np.asarray(q, dtype=float).reshape((12,))
        pf = np.asarray(p, dtype=float).reshape((12,))

        sync_mujoco_after_write(model, data, qf, pf)

        dp_sv_full = np.asarray(
            solver.joint_to_motor_velocity_leg12(qf, pf, np.asarray(dq, dtype=float)),
            dtype=float,
        ).reshape((12,))

        dp_mj_full = np.zeros(12, dtype=float)
        dp_mj_full[idx_L_knee] = _dp_motor_from_connect_jac_site_3dof(
            model, data, mjc,
            site_a_name="l_knee_tendon_site",
            site_b_name="l_knee_eq",
            dof_drive_name=joint_L_knee,
            dof_motor_name=joint_L_bar,
            dof_passive_name=joint_L_tdn,
            dq_drive=float(dq[idx_L_knee]),
        )
        dp_mj_full[idx_R_knee] = _dp_motor_from_connect_jac_site_3dof(
            model, data, mjc,
            site_a_name="r_knee_tendon_site",
            site_b_name="r_knee_eq",
            dof_drive_name=joint_R_knee,
            dof_motor_name=joint_R_bar,
            dof_passive_name=joint_R_tdn,
            dq_drive=float(dq[idx_R_knee]),
        )

        return JacobianProbe(dp_solver=dp_sv_full, dp_mj=dp_mj_full)

    return ValidationSpec(
        module="knee",
        label=f"膝 (knee, token={sel.token})",
        token=sel.token, config_dir=sel.config_dir, params_yaml=sel.params_yaml,
        mjcf_path=mjcf_path, dim_joint=12, dim_motor=12,
        channels=channels,
        joint_to_motor_position=lambda s, q: np.asarray(s.joint_to_motor_position_leg12(np.asarray(q, dtype=float)), dtype=float).reshape((12,)),
        motor_to_joint_position=lambda s, p: np.asarray(s.motor_to_joint_position_leg12(np.asarray(p, dtype=float)), dtype=float).reshape((12,)),
        joint_to_motor_velocity=lambda s, q, p, dq: np.asarray(s.joint_to_motor_velocity_leg12(np.asarray(q, dtype=float), np.asarray(p, dtype=float), np.asarray(dq, dtype=float)), dtype=float).reshape((12,)),
        motor_to_joint_velocity=lambda s, q, p, dp: np.asarray(s.motor_to_joint_velocity_leg12(np.asarray(q, dtype=float), np.asarray(p, dtype=float), np.asarray(dp, dtype=float)), dtype=float).reshape((12,)),
        construct_solver=construct,
        read_q_from_mj=read_q, read_p_from_mj=read_p,
        read_dq_from_mj=read_dq, read_dp_from_mj=read_dp,
        write_state_to_mj=write_state,
        sync_mujoco_after_write=sync_mujoco_after_write,
        sample_q=sample_q, sample_dq=sample_dq,
        jacobian_probe=jacobian_probe,
        mj_roundtrip_q_indices=(idx_L_knee, idx_R_knee),
        mj_roundtrip_p_indices=(idx_L_knee, idx_R_knee),
        notes="knee：`mj_jacSite(tendon_site)−jacSite(knee_eq)` 对齐 Roban Jacobian 口径解 dp_bar；位置读回仅限 L/R 膝与杆对应 leg12 列",
    )


# ---------------------------------------------------------------------------
# 腰（WaistSolver, 3 维 q=[yaw, pitch, roll] / p=[yaw, l_bar, r_bar]）
# ---------------------------------------------------------------------------


def _build_waist_spec(*, sel: SolverSelection, mjcf_path: str, solver_module: Any) -> ValidationSpec:
    joints_q = ("waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint")
    joints_p = ("waist_yaw_joint", "waist_l_bar_joint", "waist_r_bar_joint")

    channels: List[ChannelPlan] = [
        ChannelPlan(key="pitch", label="waist_pitch", side="shared", q_index=1, p_index=1, q_name="pitch", p_name="l_bar"),
        ChannelPlan(key="roll",  label="waist_roll",  side="shared", q_index=2, p_index=2, q_name="roll",  p_name="r_bar"),
    ]

    def construct(module_py: Any) -> Any:
        return module_py.WaistSolver(sel.token, sel.config_dir)

    def read_q(model: Any, data: Any) -> np.ndarray:
        return np.array([float(data.qpos[_qpos_adr(model, n)]) for n in joints_q], dtype=float)

    def read_p(model: Any, data: Any) -> np.ndarray:
        return np.array([float(data.qpos[_qpos_adr(model, n)]) for n in joints_p], dtype=float)

    def read_dq(model: Any, data: Any) -> np.ndarray:
        return np.array([float(data.qvel[_dof_adr(model, n)]) for n in joints_q], dtype=float)

    def read_dp(model: Any, data: Any) -> np.ndarray:
        return np.array([float(data.qvel[_dof_adr(model, n)]) for n in joints_p], dtype=float)

    def write_state(model: Any, data: Any, q: np.ndarray, p: np.ndarray) -> None:
        for i, n in enumerate(joints_q):
            data.qpos[_qpos_adr(model, n)] = float(q[i])
        # 注意 yaw 在 q/p 共享，这里 p 的 idx0 也写一次（冗余但无害）
        for i, n in enumerate(joints_p):
            data.qpos[_qpos_adr(model, n)] = float(p[i])

    def sample_q(rng: np.random.Generator) -> np.ndarray:
        return np.array([
            float(rng.uniform(-0.5, 0.5)),
            float(rng.uniform(-0.6, 0.6)),
            float(rng.uniform(-0.4, 0.4)),
        ], dtype=float)

    def sample_dq(rng: np.random.Generator, _q: np.ndarray) -> np.ndarray:
        return np.array([
            float(rng.uniform(-0.6, 0.6)),
            float(rng.uniform(-0.8, 0.8)),
            float(rng.uniform(-0.8, 0.8)),
        ], dtype=float)

    def jacobian_probe(
        solver: Any, model: Any, data: Any, q: np.ndarray, p: np.ndarray, dq: np.ndarray
    ) -> JacobianProbe:
        import mujoco as mjc

        write_state(model, data, q, p)
        mjc.mj_forward(model, data)

        # solver 侧（2×2 + 2×2）
        Jc_sv_raw, Ja_sv_raw = solver.parallel_jacobian_system(np.asarray(q, dtype=float), np.asarray(p, dtype=float))
        Jc_sv = np.asarray(Jc_sv_raw, dtype=float).reshape((2, 2))
        Ja_sv = np.asarray(Ja_sv_raw, dtype=float).reshape((2, 2))

        # MJ 侧：e=A−B；‖e‖^2 梯度除以 2‖e‖ 得 ∂‖e‖/∂·。
        b_ltd = int(model.body("waist_l_bar_tendon").id)
        b_rtd = int(model.body("waist_r_bar_tendon").id)
        b_roll = int(model.body("waist_roll").id)
        ball_L = np.asarray(model.geom("waist_roll_connect_L").pos, dtype=float).reshape((3,))
        ball_R = np.asarray(model.geom("waist_roll_connect_R").pos, dtype=float).reshape((3,))

        dof_pitch = _dof_adr(model, "waist_pitch_joint")
        dof_roll = _dof_adr(model, "waist_roll_joint")
        dof_lbar = _dof_adr(model, "waist_l_bar_joint")
        dof_rbar = _dof_adr(model, "waist_r_bar_joint")

        def _skew(r: np.ndarray) -> np.ndarray:
            x, y, z = float(r[0]), float(r[1]), float(r[2])
            return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)

        def _jac_point(body_id: int, pt_local: np.ndarray) -> np.ndarray:
            jacp = np.zeros((3, int(model.nv)), dtype=float)
            jacr = np.zeros((3, int(model.nv)), dtype=float)
            mjc.mj_jacBody(model, data, jacp, jacr, int(body_id))
            R = np.asarray(data.xmat[int(body_id)], dtype=float).reshape(3, 3)
            r_world = R @ np.asarray(pt_local, dtype=float).reshape((3,))
            return jacp - _skew(r_world) @ jacr

        def _df_sq_dvars(ball_in_roll: np.ndarray, tendon_body: int, bar_dof: int) -> Tuple[np.ndarray, float]:
            A = np.asarray(data.xpos[int(tendon_body)], dtype=float).reshape((3,))
            xB = np.asarray(data.xpos[b_roll], dtype=float).reshape((3,))
            RB = np.asarray(data.xmat[b_roll], dtype=float).reshape(3, 3)
            B = xB + RB @ np.asarray(ball_in_roll, dtype=float).reshape((3,))
            e = (A - B).reshape((3, 1))
            L_mag = float(np.linalg.norm(np.asarray(e).reshape((3,))))
            if not (L_mag > 1.0e-15) or not np.isfinite(L_mag):
                raise RuntimeError("waist MJ 探针：连杆长度过小或非法，无法换成长度 Jacobian")
            J_A = np.zeros((3, int(model.nv)), dtype=float)
            JAr = np.zeros((3, int(model.nv)), dtype=float)
            mjc.mj_jacBody(model, data, J_A, JAr, int(tendon_body))
            J_B = _jac_point(b_roll, ball_in_roll)
            de_dq = J_A - J_B
            df_dq = (2.0 * (e.T @ de_dq)).reshape((-1,))
            raw = np.array([df_dq[dof_pitch], df_dq[dof_roll], df_dq[bar_dof]], dtype=float)
            return raw, L_mag

        dfL_sq, lenL = _df_sq_dvars(ball_L, b_ltd, dof_lbar)
        dfR_sq, lenR = _df_sq_dvars(ball_R, b_rtd, dof_rbar)
        sL = 1.0 / (2.0 * lenL)
        sR = 1.0 / (2.0 * lenR)
        dfL = dfL_sq * sL
        dfR = dfR_sq * sR
        Jc_mj = np.array([[dfL[0], dfL[1]], [dfR[0], dfR[1]]], dtype=float)
        Ja_mj = np.array([[dfL[2], 0.0], [0.0, dfR[2]]], dtype=float)

        # dp 对比：solver 原始速度接口；MJ 侧由 (Jc, Ja) 代数求解
        dp_sv_full = np.asarray(
            solver.joint_to_motor_velocity(np.asarray(q, dtype=float),
                                           np.asarray(p, dtype=float),
                                           np.asarray(dq, dtype=float)),
            dtype=float,
        ).reshape((3,))

        dq_pr = np.array([float(dq[1]), float(dq[2])], dtype=float)
        try:
            dp_mj_bar = -np.linalg.solve(Ja_mj, Jc_mj @ dq_pr)
        except np.linalg.LinAlgError as e:
            raise RuntimeError("waist MJ 探针：无法解 Ja·dp = -Jc·dq（奇异）") from e
        dp_mj_full = np.array([float(dq[0]), float(dp_mj_bar[0]), float(dp_mj_bar[1])], dtype=float)

        return JacobianProbe(
            dp_solver=dp_sv_full, dp_mj=dp_mj_full,
            J_constraint_sv=Jc_sv, J_constraint_mj=Jc_mj,
            J_actuator_sv=Ja_sv,   J_actuator_mj=Ja_mj,
        )

    return ValidationSpec(
        module="waist",
        label=f"腰 (waist, token={sel.token})",
        token=sel.token, config_dir=sel.config_dir, params_yaml=sel.params_yaml,
        mjcf_path=mjcf_path, dim_joint=3, dim_motor=3,
        channels=channels,
        joint_to_motor_position=lambda s, q: np.asarray(s.joint_to_motor_position(np.asarray(q, dtype=float)), dtype=float).reshape((3,)),
        motor_to_joint_position=lambda s, p: np.asarray(s.motor_to_joint_position(np.asarray(p, dtype=float)), dtype=float).reshape((3,)),
        joint_to_motor_velocity=lambda s, q, p, dq: np.asarray(s.joint_to_motor_velocity(np.asarray(q, dtype=float), np.asarray(p, dtype=float), np.asarray(dq, dtype=float)), dtype=float).reshape((3,)),
        motor_to_joint_velocity=lambda s, q, p, dp: np.asarray(s.motor_to_joint_velocity(np.asarray(q, dtype=float), np.asarray(p, dtype=float), np.asarray(dp, dtype=float)), dtype=float).reshape((3,)),
        construct_solver=construct,
        read_q_from_mj=read_q, read_p_from_mj=read_p,
        read_dq_from_mj=read_dq, read_dp_from_mj=read_dp,
        write_state_to_mj=write_state,
        sample_q=sample_q, sample_dq=sample_dq,
        jacobian_probe=jacobian_probe,
        notes="WaistSolver.parallel_jacobian_system 直连；Jc/Ja 均为 2×2",
    )


# ---------------------------------------------------------------------------
# 肘（ParallelLinearArmSolver, 14 维 q/p，idx 3/10 为肘）
# ---------------------------------------------------------------------------


def _build_arm_elbow_spec(*, sel: SolverSelection, mjcf_path: str, solver_module: Any) -> ValidationSpec:
    idx_L, idx_R = 3, 10

    channels: List[ChannelPlan] = [
        ChannelPlan(key="L_elbow", label="L_elbow", side="left",  q_index=idx_L, p_index=idx_L, q_name="L_elbow", p_name="L_pris"),
        ChannelPlan(key="R_elbow", label="R_elbow", side="right", q_index=idx_R, p_index=idx_R, q_name="R_elbow", p_name="R_pris"),
    ]

    def construct(module_py: Any) -> Any:
        return module_py.ParallelLinearArmSolver(sel.token, sel.config_dir)

    def read_q(model: Any, data: Any) -> np.ndarray:
        q = np.zeros(14, dtype=float)
        q[idx_L] = float(data.qpos[_qpos_adr(model, "L_foream_joint")])
        q[idx_R] = float(data.qpos[_qpos_adr(model, "R_foream_joint")])
        return q

    def read_p(model: Any, data: Any) -> np.ndarray:
        p = np.zeros(14, dtype=float)
        p[idx_L] = float(data.qpos[_qpos_adr(model, "L_foream_Prismatic_joint")])
        p[idx_R] = float(data.qpos[_qpos_adr(model, "R_foream_Prismatic_joint")])
        return p

    def read_dq(model: Any, data: Any) -> np.ndarray:
        dq = np.zeros(14, dtype=float)
        dq[idx_L] = float(data.qvel[_dof_adr(model, "L_foream_joint")])
        dq[idx_R] = float(data.qvel[_dof_adr(model, "R_foream_joint")])
        return dq

    def read_dp(model: Any, data: Any) -> np.ndarray:
        dp = np.zeros(14, dtype=float)
        dp[idx_L] = float(data.qvel[_dof_adr(model, "L_foream_Prismatic_joint")])
        dp[idx_R] = float(data.qvel[_dof_adr(model, "R_foream_Prismatic_joint")])
        return dp

    def write_state(model: Any, data: Any, q: np.ndarray, p: np.ndarray) -> None:
        data.qpos[_qpos_adr(model, "L_foream_joint")] = float(q[idx_L])
        data.qpos[_qpos_adr(model, "R_foream_joint")] = float(q[idx_R])
        data.qpos[_qpos_adr(model, "L_foream_Prismatic_joint")] = float(p[idx_L])
        data.qpos[_qpos_adr(model, "R_foream_Prismatic_joint")] = float(p[idx_R])
        data.qpos[_qpos_adr(model, "L_foream_input_joint")] = 0.0
        data.qpos[_qpos_adr(model, "R_foream_input_joint")] = 0.0

    def sync_mujoco_after_write(model: Any, data: Any, q: np.ndarray, p: np.ndarray) -> None:
        import mujoco as mjc
        qf = np.asarray(q, dtype=float).reshape((14,))
        write_state(model, data, q, p)
        _mj_newton_solve_passive(
            mjc, model, data, eq_name="L_loop_foream", driver_joint="L_foream_joint",
            driver_qpos=float(qf[idx_L]), passive_joints=("L_foream_input_joint",),
            passive_qpos_guess=(float(data.qpos[_qpos_adr(model, "L_foream_input_joint")]),),
        )
        _mj_newton_solve_passive(
            mjc, model, data, eq_name="R_loop_foream", driver_joint="R_foream_joint",
            driver_qpos=float(qf[idx_R]), passive_joints=("R_foream_input_joint",),
            passive_qpos_guess=(float(data.qpos[_qpos_adr(model, "R_foream_input_joint")]),),
        )
        mjc.mj_forward(model, data)

    def sample_q(rng: np.random.Generator) -> np.ndarray:
        q = np.zeros(14, dtype=float)
        q[idx_L] = float(rng.uniform(-2.4, -0.05))
        q[idx_R] = float(rng.uniform(-2.4, -0.05))
        return q

    def sample_dq(rng: np.random.Generator, _q: np.ndarray) -> np.ndarray:
        dq = np.zeros(14, dtype=float)
        dq[idx_L] = float(rng.uniform(-0.8, 0.8))
        dq[idx_R] = float(rng.uniform(-0.8, 0.8))
        return dq

    def jacobian_probe(
        solver: Any, model: Any, data: Any, q: np.ndarray, p: np.ndarray, dq: np.ndarray
    ) -> JacobianProbe:
        """
        MJ 侧：`validate_roban_jacobian.py` 风格：``mj_jacSite(L_elbow_tip) − jacSite(L_elbow_anchor)``，
        对 ``dq_elbow`` 解 prismatic/input 两行未知（2×2 solve）。
        """
        import mujoco as mjc

        qf = np.asarray(q, dtype=float).reshape((14,))
        pf = np.asarray(p, dtype=float).reshape((14,))

        sync_mujoco_after_write(model, data, qf, pf)

        dp_sv_full = np.asarray(
            solver.joint_to_motor_velocity(qf, pf, np.asarray(dq, dtype=float)),
            dtype=float,
        ).reshape((14,))

        dp_mj_full = np.zeros(14, dtype=float)
        dp_mj_full[idx_L] = _dp_motor_from_connect_jac_site_3dof(
            model, data, mjc,
            site_a_name="L_elbow_tip",
            site_b_name="L_elbow_anchor",
            dof_drive_name="L_foream_joint",
            dof_motor_name="L_foream_Prismatic_joint",
            dof_passive_name="L_foream_input_joint",
            dq_drive=float(dq[idx_L]),
        )
        dp_mj_full[idx_R] = _dp_motor_from_connect_jac_site_3dof(
            model, data, mjc,
            site_a_name="R_elbow_tip",
            site_b_name="R_elbow_anchor",
            dof_drive_name="R_foream_joint",
            dof_motor_name="R_foream_Prismatic_joint",
            dof_passive_name="R_foream_input_joint",
            dq_drive=float(dq[idx_R]),
        )

        return JacobianProbe(dp_solver=dp_sv_full, dp_mj=dp_mj_full)

    return ValidationSpec(
        module="arm_elbow",
        label=f"肘 (arm_elbow, token={sel.token})",
        token=sel.token, config_dir=sel.config_dir, params_yaml=sel.params_yaml,
        mjcf_path=mjcf_path, dim_joint=14, dim_motor=14,
        channels=channels,
        joint_to_motor_position=lambda s, q: np.asarray(s.joint_to_motor_position(np.asarray(q, dtype=float)), dtype=float).reshape((14,)),
        motor_to_joint_position=lambda s, p: np.asarray(s.motor_to_joint_position(np.asarray(p, dtype=float)), dtype=float).reshape((14,)),
        joint_to_motor_velocity=lambda s, q, p, dq: np.asarray(s.joint_to_motor_velocity(np.asarray(q, dtype=float), np.asarray(p, dtype=float), np.asarray(dq, dtype=float)), dtype=float).reshape((14,)),
        motor_to_joint_velocity=lambda s, q, p, dp: np.asarray(s.motor_to_joint_velocity(np.asarray(q, dtype=float), np.asarray(p, dtype=float), np.asarray(dp, dtype=float)), dtype=float).reshape((14,)),
        construct_solver=construct,
        read_q_from_mj=read_q, read_p_from_mj=read_p,
        read_dq_from_mj=read_dq, read_dp_from_mj=read_dp,
        write_state_to_mj=write_state,
        sync_mujoco_after_write=sync_mujoco_after_write,
        sample_q=sample_q, sample_dq=sample_dq,
        jacobian_probe=jacobian_probe,
        notes="肘：`mj_jacSite(elbow_tip)−jacSite(elbow_anchor)` 对齐 Roban Jacobian 口径解 prismatic 速率",
    )


# ---------------------------------------------------------------------------
# 腕（ParallelLinearArmSolver, 14 维；每侧 (roll, pitch) → (slide A, slide B)）
# ---------------------------------------------------------------------------


def _build_arm_wrist_spec(*, sel: SolverSelection, mjcf_path: str, solver_module: Any) -> ValidationSpec:
    # 与 validate_arm_wrist_strict_compare 保持同一约定：
    # q14[5]=L_hand_roll, q14[6]=L_hand_pitch, q14[12]=R_hand_roll, q14[13]=R_hand_pitch
    # p14[5]=L_A,         p14[6]=L_B,          p14[12]=R_A,         p14[13]=R_B
    idx_L_roll, idx_L_pitch, idx_L_A, idx_L_B = 5, 6, 5, 6
    idx_R_roll, idx_R_pitch, idx_R_A, idx_R_B = 12, 13, 12, 13

    channels: List[ChannelPlan] = [
        ChannelPlan(key="L_roll",  label="L_wrist_roll",  side="left",  q_index=idx_L_roll,  p_index=None, q_name="L_roll"),
        ChannelPlan(key="L_pitch", label="L_wrist_pitch", side="left",  q_index=idx_L_pitch, p_index=None, q_name="L_pitch"),
        ChannelPlan(key="R_roll",  label="R_wrist_roll",  side="right", q_index=idx_R_roll,  p_index=None, q_name="R_roll"),
        ChannelPlan(key="R_pitch", label="R_wrist_pitch", side="right", q_index=idx_R_pitch, p_index=None, q_name="R_pitch"),
    ]

    def construct(module_py: Any) -> Any:
        return module_py.ParallelLinearArmSolver(sel.token, sel.config_dir)

    def read_q(model: Any, data: Any) -> np.ndarray:
        q = np.zeros(14, dtype=float)
        q[idx_L_roll] = float(data.qpos[_qpos_adr(model, "L_hand_roll_joint")])
        q[idx_L_pitch] = float(data.qpos[_qpos_adr(model, "L_hand_pitch_joint")])
        q[idx_R_roll] = float(data.qpos[_qpos_adr(model, "R_hand_roll_joint")])
        q[idx_R_pitch] = float(data.qpos[_qpos_adr(model, "R_hand_pitch_joint")])
        return q

    def read_p(model: Any, data: Any) -> np.ndarray:
        p = np.zeros(14, dtype=float)
        p[idx_L_A] = float(data.qpos[_qpos_adr(model, "L_Hand_Prismatic_joint_A")])
        p[idx_L_B] = float(data.qpos[_qpos_adr(model, "L_Hand_Prismatic_joint_B")])
        p[idx_R_A] = float(data.qpos[_qpos_adr(model, "R_Hand_Prismatic_joint_A")])
        p[idx_R_B] = float(data.qpos[_qpos_adr(model, "R_Hand_Prismatic_joint_B")])
        return p

    def read_dq(model: Any, data: Any) -> np.ndarray:
        dq = np.zeros(14, dtype=float)
        dq[idx_L_roll] = float(data.qvel[_dof_adr(model, "L_hand_roll_joint")])
        dq[idx_L_pitch] = float(data.qvel[_dof_adr(model, "L_hand_pitch_joint")])
        dq[idx_R_roll] = float(data.qvel[_dof_adr(model, "R_hand_roll_joint")])
        dq[idx_R_pitch] = float(data.qvel[_dof_adr(model, "R_hand_pitch_joint")])
        return dq

    def read_dp(model: Any, data: Any) -> np.ndarray:
        dp = np.zeros(14, dtype=float)
        dp[idx_L_A] = float(data.qvel[_dof_adr(model, "L_Hand_Prismatic_joint_A")])
        dp[idx_L_B] = float(data.qvel[_dof_adr(model, "L_Hand_Prismatic_joint_B")])
        dp[idx_R_A] = float(data.qvel[_dof_adr(model, "R_Hand_Prismatic_joint_A")])
        dp[idx_R_B] = float(data.qvel[_dof_adr(model, "R_Hand_Prismatic_joint_B")])
        return dp

    def write_state(model: Any, data: Any, q: np.ndarray, p: np.ndarray) -> None:
        data.qpos[_qpos_adr(model, "L_hand_roll_joint")] = float(q[idx_L_roll])
        data.qpos[_qpos_adr(model, "L_hand_pitch_joint")] = float(q[idx_L_pitch])
        data.qpos[_qpos_adr(model, "R_hand_roll_joint")] = float(q[idx_R_roll])
        data.qpos[_qpos_adr(model, "R_hand_pitch_joint")] = float(q[idx_R_pitch])
        data.qpos[_qpos_adr(model, "L_Hand_Prismatic_joint_A")] = float(p[idx_L_A])
        data.qpos[_qpos_adr(model, "L_Hand_Prismatic_joint_B")] = float(p[idx_L_B])
        data.qpos[_qpos_adr(model, "R_Hand_Prismatic_joint_A")] = float(p[idx_R_A])
        data.qpos[_qpos_adr(model, "R_Hand_Prismatic_joint_B")] = float(p[idx_R_B])

    def sample_q(rng: np.random.Generator) -> np.ndarray:
        q = np.zeros(14, dtype=float)
        q[idx_L_roll] = float(rng.uniform(-0.4, 0.6))
        q[idx_L_pitch] = float(rng.uniform(-0.25, 0.25))
        q[idx_R_roll] = float(rng.uniform(-0.6, 0.4))
        q[idx_R_pitch] = float(rng.uniform(-0.25, 0.25))
        return q

    def sample_dq(rng: np.random.Generator, _q: np.ndarray) -> np.ndarray:
        dq = np.zeros(14, dtype=float)
        for i in (idx_L_roll, idx_L_pitch, idx_R_roll, idx_R_pitch):
            dq[i] = float(rng.uniform(-0.8, 0.8))
        return dq

    def jacobian_probe(
        solver: Any, model: Any, data: Any, q: np.ndarray, p: np.ndarray, dq: np.ndarray
    ) -> JacobianProbe:
        """
        MJ 侧 ground truth（MuJoCo 雅可比直读）：
          - 先写入(q,p)并 mj_forward；
          - 对每根 wrist 线性执行器长度 L=||anchor_pitch - base||，取
                dL/dq = n^T (J_anchor - J_base),  n=(anchor-base)/||anchor-base||
            其中 J_anchor/J_base 均由 MuJoCo Jacobian API 得到；
          - 因 d = L - l0，故 dd = dL/dq * dq（不做 IK/反解）。
        """
        import mujoco as mjc

        dp_sv_full = np.asarray(
            solver.joint_to_motor_velocity(np.asarray(q, dtype=float),
                                           np.asarray(p, dtype=float),
                                           np.asarray(dq, dtype=float)),
            dtype=float,
        ).reshape((14,))

        write_state(model, data, np.asarray(q, dtype=float), np.asarray(p, dtype=float))
        mjc.mj_forward(model, data)
        dp_mj_full = np.zeros(14, dtype=float)

        def _skew(r: np.ndarray) -> np.ndarray:
            x, y, z = float(r[0]), float(r[1]), float(r[2])
            return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)

        def _point_jac(body_id: int, point_local_in_body: np.ndarray) -> np.ndarray:
            jacp = np.zeros((3, int(model.nv)), dtype=float)
            jacr = np.zeros((3, int(model.nv)), dtype=float)
            mjc.mj_jacBody(model, data, jacp, jacr, int(body_id))
            R = np.asarray(data.xmat[int(body_id)], dtype=float).reshape((3, 3))
            r_world = R @ np.asarray(point_local_in_body, dtype=float).reshape((3,))
            return jacp - _skew(r_world) @ jacr

        for side, (ir, ip, iA, iB) in (
            ("L", (idx_L_roll, idx_L_pitch, idx_L_A, idx_L_B)),
            ("R", (idx_R_roll, idx_R_pitch, idx_R_A, idx_R_B)),
        ):
            roll_dof = _dof_adr(model, f"{side}_hand_roll_joint")
            pitch_dof = _dof_adr(model, f"{side}_hand_pitch_joint")
            pitch_body_id = int(mjc.mj_name2id(model, mjc.mjtObj.mjOBJ_BODY, f"{side}_hand_pitch"))
            base_A_id = int(mjc.mj_name2id(model, mjc.mjtObj.mjOBJ_BODY, f"{side}_Hand_Prismatic_base_A"))
            base_B_id = int(mjc.mj_name2id(model, mjc.mjtObj.mjOBJ_BODY, f"{side}_Hand_Prismatic_base_B"))
            eq_A = _eq_id(mjc, model, f"{side}_loop_A")
            eq_B = _eq_id(mjc, model, f"{side}_loop_B")
            aA_local = np.asarray(model.eq_data[eq_A, 3:6], dtype=float).reshape((3,))
            aB_local = np.asarray(model.eq_data[eq_B, 3:6], dtype=float).reshape((3,))

            x_pitch = np.asarray(data.xpos[pitch_body_id], dtype=float).reshape((3,))
            R_pitch = np.asarray(data.xmat[pitch_body_id], dtype=float).reshape((3, 3))
            anchor_A_world = x_pitch + R_pitch @ aA_local
            anchor_B_world = x_pitch + R_pitch @ aB_local
            base_A_world = np.asarray(data.xpos[base_A_id], dtype=float).reshape((3,))
            base_B_world = np.asarray(data.xpos[base_B_id], dtype=float).reshape((3,))

            vec_A = anchor_A_world - base_A_world
            vec_B = anchor_B_world - base_B_world
            len_A = float(np.linalg.norm(vec_A))
            len_B = float(np.linalg.norm(vec_B))
            if len_A < 1.0e-12 or len_B < 1.0e-12:
                raise RuntimeError("arm_wrist MJ 探针：rod 长度过小，无法计算长度雅可比")
            nA = vec_A / len_A
            nB = vec_B / len_B

            J_anchor_A = _point_jac(pitch_body_id, aA_local)
            J_anchor_B = _point_jac(pitch_body_id, aB_local)
            J_base_A = np.zeros((3, int(model.nv)), dtype=float)
            J_base_B = np.zeros((3, int(model.nv)), dtype=float)
            jacr_tmp = np.zeros((3, int(model.nv)), dtype=float)
            mjc.mj_jacBody(model, data, J_base_A, jacr_tmp, base_A_id)
            mjc.mj_jacBody(model, data, J_base_B, jacr_tmp, base_B_id)

            dL_dq_A = nA @ (J_anchor_A - J_base_A)
            dL_dq_B = nB @ (J_anchor_B - J_base_B)
            dp_mj_full[iA] = float(dL_dq_A[roll_dof]) * float(dq[ir]) + float(dL_dq_A[pitch_dof]) * float(dq[ip])
            dp_mj_full[iB] = float(dL_dq_B[roll_dof]) * float(dq[ir]) + float(dL_dq_B[pitch_dof]) * float(dq[ip])

        return JacobianProbe(dp_solver=dp_sv_full, dp_mj=dp_mj_full)

    return ValidationSpec(
        module="arm_wrist",
        label=f"腕 (arm_wrist, token={sel.token})",
        token=sel.token, config_dir=sel.config_dir, params_yaml=sel.params_yaml,
        mjcf_path=mjcf_path, dim_joint=14, dim_motor=14,
        channels=channels,
        joint_to_motor_position=lambda s, q: np.asarray(s.joint_to_motor_position(np.asarray(q, dtype=float)), dtype=float).reshape((14,)),
        motor_to_joint_position=lambda s, p: np.asarray(s.motor_to_joint_position(np.asarray(p, dtype=float)), dtype=float).reshape((14,)),
        joint_to_motor_velocity=lambda s, q, p, dq: np.asarray(s.joint_to_motor_velocity(np.asarray(q, dtype=float), np.asarray(p, dtype=float), np.asarray(dq, dtype=float)), dtype=float).reshape((14,)),
        motor_to_joint_velocity=lambda s, q, p, dp: np.asarray(s.motor_to_joint_velocity(np.asarray(q, dtype=float), np.asarray(p, dtype=float), np.asarray(dp, dtype=float)), dtype=float).reshape((14,)),
        construct_solver=construct,
        read_q_from_mj=read_q, read_p_from_mj=read_p,
        read_dq_from_mj=read_dq, read_dp_from_mj=read_dp,
        write_state_to_mj=write_state,
        sample_q=sample_q, sample_dq=sample_dq,
        jacobian_probe=jacobian_probe,
        notes="ParallelLinearArmSolver 腕；MJ 几何长度差分（connect 锚点）得到 dp_ref",
    )
