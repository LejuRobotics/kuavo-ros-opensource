# kuavo_solver 严格验证

本文说明 **MuJoCo 几何真值** 与 **solver 解析接口** 如何对齐、误差如何定义，以及多版本回归如何运行。

## 1. 软件入口

| 场景 | 入口 | 说明 |
|------|------|------|
| **Textual TUI（推荐）** | `validate_solver.sh`（无参数）或 `solver_validation_hub.py` | 终端 UI：验证 / 批量回归 / 矩阵；可视化 = MuJoCo 窗口 + Textual 实时对比表 |
| 命令行包装 | `validate_solver.sh smoke\|full\|…` | 脚本化 / CI |
| 单版本 CLI | `mujoco_unified_cli.py` | `jacobian` / `position-verify` / `viewer` / `audit-assets` |
| 多版本回归 | `solver_test_runner.py` | canonical 去重矩阵 + JSON/HTML 报告 |

### 1.1 前置条件

```bash
cd kuavo-ros-control
catkin build kuavo_solver
source devel/setup.bash   # 或 installed/setup.bash
# 需 mujoco Python 包（与 CI ci_install_dependencies 一致）
```

### 1.2 常用命令

```bash
# Textual 验证中心（需 pip install textual）
./src/kuavo_solver/scripts/validate_solver.sh
python3 src/kuavo_solver/scripts/solver_validation_hub.py

# L0 预检（import + 抽查 MJCF，不跑采样）
./src/kuavo_solver/scripts/validate_solver.sh preflight

# 打印 canonical 矩阵（无需 kuavo_solver_py）
python3 src/kuavo_solver/scripts/solver_test_runner.py --list-matrix

# L1 Smoke：去重矩阵 · 5 samples · 仅位置；默认输出 JSON/HTML/CSV/Markdown/plots
./src/kuavo_solver/scripts/validate_solver.sh smoke

# L2 Full：position + jacobian；报告目录可直接打开 HTML 或查看 plots/*.png
./src/kuavo_solver/scripts/validate_solver.sh full --samples 50 --output-dir /tmp/solver_test_results

# 单点
./src/kuavo_solver/scripts/validate_solver.sh s70 ankle jacobian --num-steps 50 --json

# 全 index 别名审计（不去重）
python3 src/kuavo_solver/scripts/solver_test_runner.py --by-version --all
```

退出码：`solver_test_runner` / `mujoco_unified_cli` 验证失败为 **1**（CLI 单点为 **2**）。

## 2. 多版本测试矩阵

### 2.1 Canonical case（默认）

**唯一单元** = `(spec_module, token)`：

- `spec_module` ∈ `ankle` | `knee` | `waist` | `arm_elbow` | `arm_wrist`
- `token` 来自 [`config/solver_version_index.yaml`](../config/solver_version_index.yaml)
- 跳过 `none` / `disabled`

| index `module` | 展开为 spec_module |
|----------------|-------------------|
| `ankle` | `ankle`, `knee` |
| `arm` | `arm_elbow`, `arm_wrist` |
| `waist` | `waist` |

**representative_version**：同一 `(spec_module, token)` 的多个外部版本别名只测一次；优先选含 `modules` 的多模块条目（如 `s70`、`7gen`），再优先非纯数字 key。

`build_spec(module, version)` 只依赖 resolve 后的 **token**，与别名无关；单点调试可传任意合法别名：`--version 45 --module knee`。

### 2.2 执行层级

| 层级 | 命令 | samples | 测试类型 |
|------|------|---------|----------|
| L0 | `--preflight` | - | import + MJCF 存在 |
| L1 | `--smoke` | 5 | `position_roundtrip` |
| L2 | `--all` | 50（可配） | `position_roundtrip` + `jacobian` |
| L3 | `mujoco_unified_cli` | 可配 | 单 module 任选 |

## 3. 代码框架

```text
solver_version_index.yaml
    → solver_selection.resolve(version, module)
    → solver_validation_spec.build_spec(module, version) → ValidationSpec
    → solver_validation_runner.run_position_roundtrip / run_jacobian_check
    → validation_common.vector_err + StrictThreshold
```

- **ValidationSpec**（[`solver_validation_spec.py`](../scripts/validation/solver_validation_spec.py)）：唯一机构抽象；必须走 `joint_to_motor_*` / `motor_to_joint_*`；`jacobian_probe` 失败即抛异常（无 silent fallback）。
- **Runner**（[`solver_validation_runner.py`](../scripts/validation/solver_validation_runner.py)）：每 case 独立 `MjModel`；跑满 `samples` 统计 `worst_cases`（不 fail-fast）。
- **批量**（[`solver_test_runner.py`](../scripts/solver_test_runner.py)）：`build_canonical_matrix()` 去重；报告含 `canonical_key`、`representative_version`。

## 4. 误差定义

所有向量误差经 `validation_common.vector_err`，**三项同时**过阈才 PASS：

| 指标 | 定义 |
|------|------|
| `abs_max` | \(\max_i \|a_i - b_i\|\) |
| `rmse` | \(\sqrt{\mathrm{mean}((a-b)^2)}\) |
| `rel_l2` | \(\|a-b\|_2 / \max(\|a\|_2, 10^{-12})\) |

默认阈值：`abs_max=1e-4`, `rmse=5e-5`, `rel_l2=2e-3`。

### 4.1 位置 `position_roundtrip`

对每个采样 `q`：

1. `p = joint_to_motor_position(q)`
2. `q_back = motor_to_joint_position(p)`（腰可用 `motor_to_joint_position_hint`）
3. 写入 MuJoCo（膝/肘经 `sync_mujoco_after_write` 闭链）
4. `read_q_from_mj` / `read_p_from_mj`

| 误差键 | 比较 | 含义 |
|--------|------|------|
| `err_solver_rt` | `q` vs `q_back` | Solver 正逆解自洽 |
| `err_q_mujoco` | `q` vs `q_mj` | 关节与 MJ 一致 |
| `err_p_mujoco` | `p` vs `p_mj` | 电机/杆与 MJ 一致 |

可选 `mj_roundtrip_{q,p}_indices` 只比较有效自由度（如定轴踝 4+4 分量）。

### 4.2 Jacobian `jacobian_check`

采样 `q` → `p = joint_to_motor_position(q)` → `dq` → `jacobian_probe`：

| 误差键 | 比较 | 含义 |
|--------|------|------|
| `err_dp` | `dp_solver` vs `dp_mj` | 速度映射（必有） |
| `err_j_constraint` | `J_constraint_sv` vs `J_constraint_mj` | 约束 Jacobian（若提供） |
| `err_j_actuator` | `J_actuator_sv` vs `J_actuator_mj` | 执行器 Jacobian（若提供） |

MuJoCo 真值路径（在 spec 内固化）：

- **踝 axis-offset**：site 雅可比 → 4 行 `Jc/Ja`
- **踝 fixed-axis / 膝 / 肘**：`mj_jacSite` 差分 + 2×2 `solve`（Roban 口径）
- **腰**：连杆长度梯度 + `parallel_jacobian_system`
- **腕**：杆长对 roll/pitch 的解析梯度

CLI 单行结论示例：

`[FAIL] ankle_position_roundtrip token=7gen pass=48/50 (96.0%) | worst err_q_mujoco: abs_max=2.1e-4 (2.1×th) ...`

### 4.3 标准报告产物

`solver_test_runner.py` / `validate_solver.sh smoke|full` 默认 `--format all`，在 `--output-dir`（默认 `/tmp/solver_test_results`）生成：

| 文件 | 用途 |
|------|------|
| `solver_test_report.json` | 机器可读完整结果，含 `summary` 与 `worst_cases` |
| `solver_test_report.html` | 浏览器查看的总览表 |
| `solver_test_report.csv` | 表格/趋势分析，展开最差误差及阈值倍率 |
| `solver_test_summary.md` | 适合复制到 issue/PR 的摘要 |
| `plots/status_counts.png` | PASS/FAIL/ERROR 数量图 |
| `plots/worst_error_ratio.png` | 每项最差误差相对阈值倍率，>1 表示超阈 |

如只需要某一种产物，可传 `--format json|html|csv|md|plots|both`。

### 4.4 JSON 报告字段

```json
{
  "canonical_key": "ankle:7gen",
  "representative_version": "s70",
  "version": "s70",
  "spec_module": "ankle",
  "token": "7gen",
  "test_type": "jacobian",
  "passed": true,
  "summary": { "overall_pass": true, "pass_rate": 1.0, "threshold": {} }
}
```

## 5. MuJoCo 侧雅可比范式

对长度类约束 \(f = \|A(q,p)-B(q,p)\|\)（或平方形式），用 **body/site point Jacobian**：

```python
mujoco.mj_jacBody(model, data, Jp_A, None, body_A_id)
mujoco.mj_jacBody(model, data, Jp_B, None, body_B_id)
df_ddof = hat @ (Jp_A - Jp_B)   # 或 2*vec@(Jp_A-Jp_B) 对平方约束
```

按 dof 索引切出 `Jc_mj`（关节列）与 `Ja_mj`（电机列）。**默认不再**用 `efc_J` 反推 `dp` 作为主路径。

## 6. 调试建议

1. 失败 case 用 `mujoco_unified_cli --json` 复现，`worst_cases` 看最大误差项。
2. `err_j_constraint` / `err_j_actuator` 分项偏大 → 查 YAML ↔ MJCF ↔ solver 几何链。
3. 仅 `err_solver_rt` 大 → 解算器正逆；仅 `err_*_mujoco` 大 → MJCF 命名/闭链同步。

### 6.1 Mesh / STL 找不到（viewer 大量警告）

MJCF 通过 `<compiler meshdir="../../../../../kuavo_assets/models/biped_sNN/meshes/">` 引用 STL。
若工作区未拉取 `kuavo_assets` 子模块，MuJoCo 会报 mesh not found，3D 窗口里连杆不可见，但 **joint/site 运动学仍可验证**。

| 方式 | 命令 / 操作 |
|------|-------------|
| **Hub Assets 页** | `validate_solver.sh` → **Assets** → *Audit current version+module*；表格列出每个 mesh 的解析路径与 OK/MISSING |
| **CLI 审计** | `python3 mujoco_unified_cli.py --version s70 audit-assets --module ankle` |
| **无 STL 的 viewer** | Hub 勾选 *Kinematics-only viewer*，或 `viewer --kinematics-only`（去掉 mesh geom，保留 site/关节） |
| **补齐资源** | `git submodule update --init src/kuavo_assets`（路径以仓库根为准） |

可选环境变量 `KUAVO_ASSETS_ROOT` 覆盖资产根目录（仅影响审计报告中的提示路径，不修改 MJCF）。

## 7. 与 hw_decouple_core 的边界

- **kuavo_solver 验证**：离线解析解 vs MuJoCo，无 ROS。
- **hw_decouple_core**（如 `hw_sim_test.py`）：硬件解耦集成测；并联 q↔p 应以本验证 PASS 的 token/MJCF 为几何基线。

---

**更新记录**

| 日期 | 改动 | 原因 |
|------|------|------|
| 2026-05-29 | 统一 CLI 与 runner 阈值 (`abs_max=1e-4`, `rmse=5e-5`, `rel_l2=2e-3`)；之前 CLI 默认阈值松 50× | 避免单点 CLI 假阴性 |
| 2026-05-29 | preflight 改为检查全部 canonical case（含 solver 构造），不再仅抽查前 3 个 | 提前发现全部配置问题 |
| 2026-05-29 | `--quiet` 模式：只输出 FAIL/ERROR 行，末尾汇总失败项 | CI 日志精简 |
| 2026-05-29 | 移除 `validation_common.normalize_version` 重复定义；统一用 `solver_selection.normalize_version` | 消除不一致行为 |
| 2026-05-29 | `mjcf_for_module` 委托到 `default_mjcf_for`，消除重复 if/elif 链 | 减少维护面 |
| 2026-05-29 | §6.1 mesh 审计、`audit-assets`、`--kinematics-only` viewer、Hub Assets 页 | 工作区常缺 kuavo_assets STL，需直观列出缺失路径与无 mesh 调试方式 |
