# kuavo_solver 几何解算原理（踝 / 膝 / 肘 / 腕 / 腰）

本文把五个几何解算器用同一套抽象写成一份速查文档。
重点是 **"关节 q ↔ 电机 p" 的封闭闭环长度约束如何写成统一的 `JacobianSystem`**，以及每个部位
几何约束的具体形式与代码入口。深入的 MJCF / WebUI 对齐说明见
`kuavo-knowledge/ankle_solver_geometry.md`。

---

## 0. 统一抽象：`JacobianSystem` 与虚功对偶

所有解算器的核心是一条（或多条）**封闭几何约束**：

\[
\mathbf{f}(\mathbf{q},\mathbf{p}) = \mathbf{0}
\]

- `q` = 关节角（被动侧，一般是 pitch/roll/knee 等人类直接感受到的维度）
- `p` = 电机角 / 线性执行器位移（主动侧）
- `f` = 标量约束，如"腱长固定"、"两点距离固定"

对 `f` 做微分就得到 **`JacobianSystem`**（`include/kuavo_solver/solver_tools.h` 中定义）：

```cpp
struct JacobianSystem {
    Eigen::MatrixXd J_constraint;  // ∂f/∂q
    Eigen::MatrixXd J_actuator;    // ∂f/∂p
};
```

满足

\[
J_c\,\dot{\mathbf q} + J_a\,\dot{\mathbf p} = \mathbf{0}.
\]

所有映射都由这一条基本关系派生（见 `SolverTools::MotorVelocityFromJoint` 等）：

| 方向 | 公式 | 适用 API |
|------|------|----------|
| 关节速度 → 电机速度 | `dp = -J_a⁻¹ · J_c · dq` | `joint_to_motor_velocity` |
| 电机速度 → 关节速度 | `dq = -J_c⁻¹ · J_a · dp`（或 QR 最小二乘） | `motor_to_joint_velocity` |
| 关节力矩 → 电机电流 | `τ_motor = -J_a⁻ᵀ · J_cᵀ · (-τ_joint) = J_a⁻ᵀ · J_cᵀ · τ_joint` | `joint_to_motor_current` |
| 电机电流 → 关节力矩 | `τ_joint = -J_c⁻ᵀ · J_aᵀ · τ_motor` | `motor_to_joint_torque` |

> 虚功对偶：`δp = -J_a⁻¹ J_c δq` ⇒ `τ_motorᵀ δp = τ_jointᵀ δq` 展开即得上述两条力矩公式。

**关键约定**：所有 5 个部位都遵循同一范式，差异仅在"`f`
是什么"和"`q`/`p` 的维度"。下表是速查表：

| 部位 | 约束 `f` | `dim(q)` | `dim(p)` | `J_c` | `J_a` | 关键源码 |
|------|----------|-----------|----------|-------|-------|----------|
| 踝（AxisOffset） | 两腱长度各不变 `\|eq−td\|=l₀`（×2） | 2（pitch,roll） | 2（lbar,rbar） | 2×2 | 2×2 对角 | `compute_ankle_jacobian_system_` |
| 膝（linkage） | 杆-腱-膝 四杆闭链（1 个长度约束） | 1（q_knee） | 1（p_bar） | 1×1 | 1×1 | `knee_jacobians_` |
| 肘 | 线性 4-bar（1 个长度约束） | 1（theta） | 1（d） | 1×1 | 1×1 | `ElbowJacobian` |
| 腕 | 两根直线执行器各连一根拉杆长度 | 2（roll,pitch） | 2（dA,dB） | 2×2 | **-I₂** | `compute_wrist_jacobian_system` |
| 腰 | 两根杆端到球心距离固定 `\|A−B\|=L`（×2） | 2（pitch,roll） | 2（lbar,rbar） | 2×2 | 2×2 对角 | `parallel_jacobian_system` |

腕的 `J_a = -I₂` 反映了"直线执行器 `d_X = L_X − const`"的平凡性——
和踝/腰的旋转执行器（`J_a` 需要算 `∂L/∂bar`）有本质区别，但**求解结构完全一致**。

---

## 1. 踝（Ankle）

两种几何实现共用同一抽象，**"两腱长度不变"是唯一真值**：

\[
f_L = \|\mathbf{p}^{K}_{eq,L}-\mathbf{p}^{K}_{td,L}\|-l_{0,L} = 0,\quad f_R = \|\mathbf{p}^{K}_{eq,R}-\mathbf{p}^{K}_{td,R}\|-l_{0,R}=0
\]

- `p_eq`：脚端等效力点，随 `(pitch, roll)` 运动
- `p_td`：杆端肌腱附着点，随执行器角 `bar` 运动
- `l₀`：YAML 给定的腱长

两个版本的差异仅在 `p_eq`、`p_td` 的坐标分解。

### 1.1 AxisOffsetAnkleSolver（`5gen_2 / s2gen_2`）

Pitch 与 Roll 轴**不共点**，代码闭式展开见 `compute_tendon_vector()`：

```
p_eq^K = t_P^K + R_pitch · ( t_R^P + R_roll · p_eq^R )
p_td^K = (x_bar, 0, z_bar) + R_bar_axis(act) · (x_td, y_td, z_td)
```

**JacobianSystem 构造**（重构后见 `compute_ankle_jacobian_system_`，走统一 `tendon_length_jacobian_` helper）：

对每条腱（LEFT/RIGHT tendon）独立算"长度对各变量的导数"：

```cpp
struct TendonLengthJacobian {
    Eigen::RowVector2d ankle;   // [∂L/∂pitch, ∂L/∂roll]
    double actuator;            // ∂L/∂bar
    double length;              // |p_eq - p_td|，IK 残差可复用
};
```

其中 `∂L/∂X = n̂ · ∂vec/∂X`，`n̂ = vec/|vec|` 是沿腱的单位向量。
拼成 `J_c` 的两行与 `J_a` 的对角即完成：

```
J_c = [ J_L.ankle ; J_R.ankle ]   // 2×2
J_a = diag(J_L.actuator, J_R.actuator)
```

> 语义约定：`J_l / J_r` 中的 `l/r` 指"当前踝侧（AnkleSide::LEFT 或 RIGHT）的**左腱 / 右腱**"，
> 不要和"左脚 / 右脚"混淆。旧代码中 `J_l_ankle_l` 这种双 `l` 后缀已重构掉。

**正解**（`q → p`）：在 yz 平面上对每条腱各自做余弦定理（腱长 = bar 半径 + eq 到 bar pivot 的 yz 投影）。
**反解**（`p → q`）：对 `(pitch, roll)` 用牛顿法，残差即上式 `f_L, f_R`，雅可比直接用 `J_c`。

### 1.2 FixedAxisAnkleSolver（`4gen / 4gen_pro / 5gen / s1gen / s2gen`）

Pitch 与 Roll 轴**共点**，几何约束相同但用 `xz` 或 `yz` 平面余弦定理闭式求解，
见 `joint_to_motor_position_legacy37_` / `_pro43_`（具体代数展开见 knowledge 库）。
它没有走 `JacobianSystem` 的显式构造，而是走三角代数，但数学上等价。

### 1.3 代码入口

| 接口 | 说明 |
|------|------|
| `AxisOffsetAnkleSolver::jacobian_system(pitch,roll,lbar,rbar,side)` | 单踝 `JacobianSystem`（2×2 / 2×2） |
| `AxisOffsetAnkleSolver::joint_to_motor_*` | 走 `SolverTools::MotorVelocityFromJoint(jac, dq)` 等统一入口 |
| `AnkleSolver`（上层） | 按版本 token 分发到 FixedAxis / AxisOffset，并做 12 维索引置换 |

---

## 2. 膝（Knee Linkage，只在 `5gen_2` 启用）

膝关节本身只是**一个旋转关节**，但电机侧驱动的是一根**摆杆（bar）**，
摆杆再通过一根**腱（tendon）**与膝连杆上的**等效力点（knee_eq）**相连，构成闭环 4-bar：

```
l_knee_bar 轴（W 原点） ──bar── bar_td ──tendon── knee_eq ──膝连杆── l_knee 轴（W: (x_bar_knee, z_bar_knee)）
```

### 2.1 约束

在 `W` 坐标系（以 `l_knee_bar` 轴为原点）：

- `knee_eq = (x_bar_knee + l_knee_eq·sin(q_k+q_o_k), z_bar_knee + l_knee_eq·cos(q_k+q_o_k))`
- `bar_td  = (l_bar_td·sin(-p_k-q_o_b), l_bar_td·cos(-p_k-q_o_b))`
- 约束：`|knee_eq − bar_td| = l_tendon`

### 2.2 正解

对约束两边平方展开，得到 `q_k ↔ p_k` 的余弦定理闭式（`knee_joint_to_motor` / `knee_motor_to_joint`）：

```
p_bar = π − acos( (l_bar_td² + l_BarEq² − l_tendon²) / (2·l_bar_td·l_BarEq) )
        + atan2(x_BarEq, −z_BarEq) − q_offset_bar
```

### 2.3 `JacobianSystem`（标量 1×1 版本）

对约束 `f = vec·vec − l_tendon²` 求微分：

- `J_c = ∂f/∂q_k = Jxx · J_knee_WEq`  （代码 `den_knee`）
- `J_a = ∂f/∂p_k = Jxx · J_bar_WTd`   （代码 `den_bar`）

`Jknee_bar = -J_c / J_a` 给出"膝关节速度→摆杆速度"，`Jbar_knee` 是它的倒数。
见 `knee_jacobians_`，奇异（`den≈0`）时**硬失败**。

### 2.4 与踝的串联

12 维腿路径中，**膝电机和踝 pitch/roll 电机三路都被同一条腱影响**：
`p_bar` 由"膝角贡献"+"踝 pitch/roll 贡献"叠加得到（`knee_linkage_enabled_==true` 的 5gen_2 特有）。
速度雅可比也对应做加法：

```
dp_bar = Jknee_bar · dq_knee + J_ankle_bar · [dq_pitch; dq_roll]
```

---

## 3. 肘（Elbow，2D 线性 4-bar）

`ParallelLinearArmSolver::ElbowInverseKinematics / ElbowForwardKinematics`。

### 3.1 结构

平面 4-bar，`theta` 是关节角（绕 `O_f`），`d` 是线性执行器位移（`P` 沿 z 方向平移）：

```
A = O_f + R_2D(theta) · (a_x, a_z)     // 活动铰链 A 的位置
P = (O_p_x, O_p_z − d)                  // 滑块铰链 P 的位置
约束: |A − P|² = L² = t_x² + t_z²      // 连杆 AP 长度固定
```

即 `f(theta, d) = |A(theta) − P(d)|² − L² = 0`。

### 3.2 正解 / 反解

- **IK（theta → d）**：从约束解出 `d = A_z − P_z = dz − A_z + O_p_z`，其中 `dz = -√(L² − (A_x − O_p_x)²)`。
- **FK（d → theta）**：把约束展开成 `A·cos(θ) + B·sin(θ) = C` 再 `atan2/acos` 求解（见 `ElbowForwardKinematics`）。

### 3.3 `JacobianSystem`（标量 1×1 版本）

对 `f = |A − P|² − L²` 求微分：

```
∂f/∂theta = 2(A − P) · ∂A/∂theta = 2·(dx·dAx + dz·dAz)
∂f/∂d     = 2(A − P) · (−∂P/∂d)  = 2·dz
```

代码 `ElbowJacobian` 返回 `-∂f/∂d / ∂f/∂theta`（即 `dtheta/dd`）。
这对应 `Jac = -J_a / J_c`，和 `JacobianSystem` 的 `dp = -J_a⁻¹ J_c dq` 一致。

**奇异**：`∂f/∂theta ≈ 0` 是 4-bar 死点构型，`fail-fast` 抛异常。

---

## 4. 腕（Wrist，3D 双直线执行器）

`ParallelLinearArmSolver`（重构后走 `wire_length_jacobian_` helper）。

### 4.1 结构

两根"拉杆"A、B 各有一个线性执行器：

```
anchor_A_cf(roll, pitch) = pitch_center_cf + R_roll·R_pitch · anchor_A_pitch
base_A                    = 固定点（腕基座坐标系）
L_A = |anchor_A_cf − base_A|
d_A = L_A − len_A_zero     ← 执行器位移（标定零位偏移后）
```

B 同理。`(roll, pitch)` 是腕两自由度关节角，`(d_A, d_B)` 是两根直线执行器位移。

### 4.2 `JacobianSystem`

由于 `d_X = L_X − const`，`∂d_X/∂d_X = 1`，所以：

```
J_c = [ ∂L_A/∂roll   ∂L_A/∂pitch ;     // compute_wrist_jacobian() 的 2x2
        ∂L_B/∂roll   ∂L_B/∂pitch ]
J_a = -I₂                                // 直线执行器平凡对偶
```

### 4.3 `WireLengthJacobian`（与踝腱同构）

为保持与踝腱一致的"每根腱单独算长度雅可比"结构：

```cpp
struct WireLengthJacobian {
    Eigen::RowVector2d wrist;   // [∂L/∂roll, ∂L/∂pitch]
    double actuator;            // 恒为 1（直线执行器）
    double length;              // |anchor_cf − base|
};
```

核心几何公式（`wire_length_jacobian_`）：

```
n̂ = vec/|vec|,  vec = anchor_cf − base
∂L/∂roll  = n̂ · ( ω_roll × (anchor_cf − roll_center) )
∂L/∂pitch = n̂ · ( ω_pitch × (anchor_cf − pitch_center_cf) )
ω_pitch = R_roll · pitch_axis      // pitch 轴随 roll 先转一下
```

这和脚踝 `hat_tendon^T · (J_ankle + J_actuator)` 是同一张公式，只是**腕的"执行器方向"不再是
bar 的旋转，而是沿 `n̂` 本身**，所以 `actuator` 部分退化成 `1`。

### 4.4 侧边符号

外部约定 `q_ext = side_sign · q_int`，`d_ext = d_int`，因此：

```
J_ext = side_sign · J_int(side_sign·roll, side_sign·pitch)
```

见 `wrist_jacobian(roll, pitch, is_left)` 封装。`compute_wrist_jacobian_system` 返回的已经是外部约定。

### 4.5 反解

`WristInverseKinematics`：牛顿迭代，残差 `[d_A − L_A + len_A_zero ; d_B − L_B + len_B_zero]`，
雅可比复用 `WristJacobian`。

---

## 5. 腰（Waist，Parallel Rotate）

`ParallelRotateWaistSolver`。结构最接近踝——两根旋转杆，但约束是**杆端到球心的 3D 距离固定**。

### 5.1 结构

在 `waist_yaw` 坐标系中：

```
A_side(bar)       = bar_pivot_side + R_y(bar) · tendon_joint_in_bar
B_side(pitch,roll) = pitch_origin_in_yaw + R_y(pitch) · ( roll_origin_in_pitch + R_x(roll) · ball_center_side_in_roll )
```

约束（左右各一条）：

\[
f_\text{side}(pitch, roll, bar) = \|A_\text{side} − B_\text{side}\|^2 − L^2 = 0
\]

### 5.2 `JacobianSystem` 构造

见 `ParallelRotateWaistSolver::jacobian_system` → `ComputeSideJacobian`：

```
e = A − B
∂f/∂pitch = -2·e·(R_y'(pitch) · (roll_origin + R_x(roll)·ball))   // 注意负号：B 对 pitch 的依赖
∂f/∂roll  = -2·e·(R_y(pitch) · R_x'(roll) · ball)
∂f/∂bar   = +2·e·(R_y'(bar) · tendon_joint_in_bar)
```

左右两侧堆叠：

```
J_c = [ dfL/dpitch  dfL/droll ;
        dfR/dpitch  dfR/droll ]              // 2×2
J_a = diag(dfL/dbar_left, dfR/dbar_right)    // 对角
```

### 5.3 正/反解

- `joint_to_motor_position(q2)`：对每侧独立做 `SolveBarForSide`（1D 牛顿/Brent 求 `f_side=0`）。
- `motor_to_joint_position(p2)`：`SolvePitchRoll` 牛顿迭代，残差用两侧 `f`，雅可比用 `J_c`。

---

## 6. 几何约束 → 代码实现的一张对照

| 部位 | 约束 `f` | 对 `q` 求导（`J_c`） | 对 `p` 求导（`J_a`） | Helper |
|------|----------|-----------------------|-----------------------|--------|
| 踝 | `\|p_eq − p_td\| − l₀ = 0`（×2） | `n̂ᵀ · (∂p_eq/∂q − ∂p_td/∂q)` | `n̂ᵀ · ∂p_td/∂bar` | `tendon_length_jacobian_` |
| 膝 | `\|knee_eq − bar_td\|² − l_tendon² = 0` | `2(·)·∂knee_eq/∂q_k` | `2(·)·∂bar_td/∂p_k` | `knee_jacobians_` |
| 肘 | `\|A − P\|² − L² = 0` | `2(A−P)·∂A/∂θ` | `2(A−P)·(−∂P/∂d) = 2·dz` | `ElbowJacobian` |
| 腕 | `\|anchor − base\| − len_zero − d_X = 0`（×2） | `n̂ᵀ · ∂anchor/∂q`（`q=roll,pitch`） | `−I`（`∂/∂d_X = −1`） | `wire_length_jacobian_` |
| 腰 | `\|A − B\|² − L² = 0`（×2） | `2e·(−∂B/∂q)` | `2e·(∂A/∂p)` | `ComputeSideJacobian` |

核心观察：**所有部位都是"某向量的范数（或其平方）= 定值"**——
这正是平行机构的共同特征。只要能把"那根向量"的两端 `P_actuator(p)` / `P_joint(q)` 写出闭式，
`JacobianSystem` 的两行/两列就只是**沿向量方向的点积**。

---

## 7. 与验证脚本的对接

`validate_solver_jacobian_standard.py` 对每个部位都按同一范式做严格验证：

1. 从 MuJoCo 对应 XML 加载机械结构，取 `(q, p)` 的随机样本。
2. 用 MuJoCo **site/body Jacobian**（`mj_jacSite` / `mj_jacBody`）数值算出约束 `f` 对每个 dof 的导数，
   拼成**数值 `JacobianSystem`**（`Jc_mj`, `Ja_mj`）。
3. 调用 solver 的 `jacobian_system` / `compute_wrist_jacobian_system` / `parallel_jacobian_system` 拿到
   **解析 `JacobianSystem`**（`Jc_sv`, `Ja_sv`）。
4. 按元素比对 `abs_max / rel_l2 / rmse`，以严格阈值（`1e-4 / 2e-3 / 5e-5`）判定 PASS/FAIL。

对于膝/肘这种 1×1 的情况，同样走这套流程（退化为标量比对）。

> 一句话总结：**验证脚本和 solver 用的是同一套 `JacobianSystem` 语言，只不过 MuJoCo 是数值
> 差分（经 `efc_J` 或 `mj_jac*`），solver 是解析展开。两者必须在严格阈值下逐元素重合。**

---

## 8. 文件速查

| 主文件 | 部位 | 关键函数 |
|--------|------|----------|
| `src/ankle/axis_offset_ankle_solver.cpp` | 踝（AxisOffset）+ 膝 linkage | `compute_ankle_jacobian_system_`, `tendon_length_jacobian_`, `knee_jacobians_` |
| `src/ankle/fixed_axis_ankle_solver.cpp` | 踝（FixedAxis） | `joint_to_motor_position_{legacy37,pro43,s2}_` |
| `src/arm/parallel_linear_arm_solver.cpp` | 肘 + 腕 | `ElbowJacobian`, `wire_length_jacobian_`, `compute_wrist_jacobian_system` |
| `src/waist/parallel_rotate_waist_solver.cpp` | 腰 | `jacobian_system`, `ComputeSideJacobian` |
| `src/common/solver_tools.cpp` | 公共 | `JacobianSystem`、`YamlLoader` 等共享工具 |
| `include/kuavo_solver/solver_tools.h` | 公共（头文件） | `JacobianSystem`, `MotorVelocityFromJoint`, `YamlLoader` |
| `python/kuavo_solver_pybind.cpp` | Python | 所有 `jacobian_system*` 绑定 |
| `scripts/mujoco_unified_cli.py` + `scripts/validation/solver_validation_*.py` | 验证 | `run_jacobian_check` / `run_position_roundtrip`（按 module） |

---

## 更新记录

- 2026-04-21：首次落地本文档。把五个几何解算器统一到 `JacobianSystem` 抽象下，
  对照踝腱 `tendon_length_jacobian_` 把腕改写为 `wire_length_jacobian_`，
  补充各部位的约束方程、雅可比展开形式、奇异点与验证对接。
