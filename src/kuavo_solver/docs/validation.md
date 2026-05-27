# 严格验证：`validate_solver_jacobian_standard.py` 对齐规则

本文档解释**验证脚本怎么把 MuJoCo 与 solver 对齐**。目标是让两者都输出同一个
`JacobianSystem`（`Jc`, `Ja`），然后逐元素比对。

## 1. 脚本入口与 CLI 路由

| CLI | 命令 | 背后的 `run_*_strict_compare` |
|------|------|-------------------------------|
| `mujoco_ankle_cli.py jacobian/position-verify` | 踝 | `run_ankle_strict_compare` |
| `mujoco_ankle_cli.py knee-*` | 膝 | `run_knee_strict_compare` |
| `mujoco_arm_cli.py elbow-*` | 肘 | `run_arm_elbow_strict_compare` |
| `mujoco_arm_cli.py wrist-*` | 腕 | `run_arm_wrist_strict_compare` |
| `mujoco_waist_cli.py jacobian/position-verify` | 腰 | `run_waist_strict_compare` |

所有 CLI 都支持：

- 无参（默认）输出**结论一行**：`PASS 100% · worst 1.3x / 2e-5 abs_max`。
- `--json` 输出完整报告（`summary + worst_cases`）。

## 2. MuJoCo 侧数值雅可比的统一写法

### 2.1 单向量长度约束（踝/腕/腰/膝 / 肘都属于这类）

设约束是 `f = |A(q,p) − B(q,p)|² − L² = 0`（或 `|A−B| − L = 0`），
用 MuJoCo 的 **body/site point Jacobian** 拿到 A、B 两点对所有 dof 的导数：

```python
Jp_A = np.zeros((3, m.nv))  # ∂A/∂dof
mujoco.mj_jacBody(m, d, Jp_A, None, body_A_id)
Jp_B = np.zeros((3, m.nv))
mujoco.mj_jacBody(m, d, Jp_B, None, body_B_id)

vec = A - B
df_ddof = 2 * vec @ (Jp_A - Jp_B)   # 1 x m.nv
```

然后按 dof 索引切出"属于 q"的列 → `Jc_mj`，"属于 p"的列 → `Ja_mj`。
**这是所有部位通用的 MuJoCo 侧雅可比计算范式**，不依赖 `efc_J`。

### 2.2 闭环 `efc_J`（仅作参考，不再作为默认路径）

MuJoCo 的 `d.efc_J`（连接约束雅可比）是**对约束点的 3D 差值**求的，
维度是 `(nefc, nv)`。以前验证脚本曾用它解 `dp_mj_required`，但这要求：

- 把 slide/ball 等"辅助自由度"的列单独解出来；
- `efc_J` 的约束坐标系需要手动对齐；

现在统一改走上面 2.1 的 site/point Jacobian 路径，**更稳且和 solver 语义直接吻合**。

## 3. Solver 侧解析雅可比

每个 solver 都暴露同名接口：

| Solver | Python 接口 | 返回 |
|--------|-------------|------|
| `AxisOffsetAnkleSolver` | `jacobian_system(pitch, roll, lbar, rbar, side)` | `JacobianSystem`（2×2 / 2×2 对角） |
| `AxisOffsetAnkleSolver` | `knee_jacobians_`（C++ 私有，验证经 `knee_joint_to_motor_velocity` 等间接读） | 标量 |
| `ParallelLinearArmSolver` | `compute_elbow_jacobian(theta, d, is_left)` | 标量（`dtheta/dd`） |
| `ParallelLinearArmSolver` | `compute_wrist_jacobian_system(roll, pitch, is_left)` | `JacobianSystem`（2×2 / `-I₂`） |
| `ParallelRotateWaistSolver` | `jacobian_system(q2, p2)` | `JacobianSystem`（2×2 / 对角） |

**JacobianSystem 的 pybind 绑定**在 `python/kuavo_solver_pybind.cpp` 顶部：

```cpp
py::class_<kuavo_solver::JacobianSystem>(m, "JacobianSystem")
    .def(py::init<>())
    .def_readwrite("J_constraint", ...)
    .def_readwrite("J_actuator", ...);
```

验证脚本拿到后直接 `np.asarray(js.J_constraint)` / `np.asarray(js.J_actuator)` 即可。

## 4. 比对阈值

默认严格阈值（`scripts/validation/validation_common.py`）：

```python
abs_max  = 1e-4     # 绝对误差最大值
rel_l2   = 2e-3     # 相对 L2 误差
rmse     = 5e-5     # 均方根误差
```

判定 PASS 的三条件**同时**满足。CLI 显示的 "worst N.Nx threshold" 是 `max_err / 对应阈值` 的最坏比值，
> 1.0 即 FAIL。

## 5. 调试策略

当某个部位 FAIL 时：

1. **先看 worst_case** (`--json` 看 `worst_cases`)，通常集中在几个特定构型（接近奇异、或标定参数不一致）。
2. **对比 `J_c` 与 `J_a`**：哪块偏大，就从那块对应的参数链（点位、轴向、零位偏移）查起。
3. **检查几何参数源**：YAML ↔ MJCF ↔ solver。XML 侧的 `axis`、`pos`、`body` 层级要和 YAML 的分解一致。
4. **退化到标量约束重算**：手动写 `f(q,p)` 然后 `scipy.optimize.approx_fprime` 数值微分，
   对照 solver 的解析值。一般能在十分钟内定位到哪个 `∂f/∂x_i` 不对。

## 6. 历史路径 vs 当前推荐

| 路径 | 状态 |
|------|------|
| `dp_mj_required` 解 `efc_J` 反推电机速度 | **已废弃**（残留在部分旧测试里） |
| `mj_jacBody / mj_jacSite` → 组装 `Jc_mj, Ja_mj` → 对比 solver `JacobianSystem` | **当前推荐**（解析 vs 数值，一套语义） |
| `round_trip`（`q → p → q`）位置校验 | **不再作为充分条件**：参数错也可能往返闭合 |
| Mujoco 实时轨迹 vs solver 计算曲线（GUI） | 已统一成 solver 接口出数据，GUI 仅作可视化 |
