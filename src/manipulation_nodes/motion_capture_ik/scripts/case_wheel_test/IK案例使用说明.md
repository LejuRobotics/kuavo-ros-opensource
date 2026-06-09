# motion_capture_ik 轮臂案例脚本使用说明

本文档说明 `scripts/case_wheel_test/` 下 6 个案例脚本的使用方法、参数、依赖话题/服务，以及常见输出含义。

**坐标系**：case00~case05 中的末端期望位置、姿态（及 case04 肘部点）均在 **`waist_yaw_link`** 坐标系下给出（位置单位 m）。

## 1. 目录与脚本

目录：

- `src/manipulation_nodes/motion_capture_ik/scripts/case_wheel_test/`

脚本：

- `arms_ik_api.py`：公共 API 封装（FK/IK 调用、关节发布、误差计算）
- `case_00_desired_pose_ik.py`：给定期望位姿（欧拉角 RPY），通过 `--mode` 选择一种 IK 方式求解
- `case_01_fk_topic_ik_loop.py`：FK -> 轨迹执行 -> 回零 -> Topic IK
- `case_02_fk_srv_ik_loop.py`：FK -> 轨迹执行 -> 回零 -> Service IK
- `case_03_multi_ref_constraint_compare.py`：6 种 `constraint_mode` 对比
- `case_04_srv_with_elbow_constraint.py`：带肘部约束的 IK
- `case_05_fk_srv_muli_refer_ik_loop.py`：FK -> 轨迹执行 -> 回零 -> 多参考 Service IK

## 2. 公共参数与接口说明（arms_ik_api.py）

### 2.1 关键参数

- `build_two_arm_cmd(...)`
  - `left_pos_xyz` / `right_pos_xyz`：左右手目标位置（m）
  - `left_quat_xyzw` / `right_quat_xyzw`：左右手目标四元数（xyzw）
  - `ik_param`：IK 求解参数，默认由 `default_ik_param()` 构造
  - `joint_angles_as_q0`：是否将 `left/right_joint_angles` 作为 IK 初值
  - `left_joint_angles` / `right_joint_angles`：各 7 维初值（rad）
  - `left_elbow_pos_xyz` / `right_elbow_pos_xyz`：肘部约束点（m）
  - `frame`：坐标系模式（默认 `0`）

- `default_ik_param(constraint_mode=...)`
  - 可切换约束模式（详见 case3）
  - 默认容差均为 `1e-3`（见 2.4 节；求解失败时可适当放宽）

### 2.2 关节初值来源

- 若未传 `left_joint_angles/right_joint_angles`，API 会尝试从 `/sensors_data_raw` 获取当前手臂 14 维关节角作为初值。
- 轮臂下，手臂切片按配置自动计算：`start = NUM_JOINT - NUM_HEAD_JOINT - NUM_ARM_JOINT`（典型为 4）。

### 2.3 主要话题/服务

- 话题：
  - 发布 IK 目标：`/ik/two_arm_hand_pose_cmd`
  - IK 结果订阅：`/ik/result`
  - 关节跟踪发布：`/kuavo_arm_traj`
  - 传感器读当前关节：`/sensors_data_raw`

- 服务：
  - FK：`/ik/fk_srv`
  - IK（单参考）：`/ik/two_arm_hand_pose_cmd_srv`
  - IK（多参考）：`/ik/two_arm_hand_pose_cmd_srv_muli_refer`

### 2.4 IK 求解容差调整（求解失败时）

若某组末端位姿 IK 返回 `success=False`，或 Topic 方式长时间收不到 `/ik/result`，常见原因之一是**约束容差过严**，优化器无法在默认精度内找到可行解。

`arms_ik_api.py` 中 `default_ik_param()` 的默认容差均为 **`1e-3`**：

- `major_optimality_tol`
- `major_feasibility_tol`
- `minor_feasibility_tol`
- `oritation_constraint_tol`（姿态约束容差）
- `pos_constraint_tol`（位置约束容差）

**建议**：将上述容差适当改大，例如改为 **`5e-3`**，往往即可求解成功。可在 `arms_ik_api.py` 中直接修改默认值：

```python
p.major_optimality_tol = 5e-3
p.major_feasibility_tol = 5e-3
p.minor_feasibility_tol = 5e-3
p.oritation_constraint_tol = 5e-3
p.pos_constraint_tol = 5e-3
```

若只想在某个案例里临时放宽，不必改 API 文件，可在调用 `build_two_arm_cmd` 前单独构造参数：

```python
ik_param = api.default_ik_param(constraint_mode=3)
ik_param.pos_constraint_tol = 5e-3
ik_param.oritation_constraint_tol = 5e-3
ik_param.major_optimality_tol = 5e-3
ik_param.major_feasibility_tol = 5e-3
ik_param.minor_feasibility_tol = 5e-3
cmd = api.build_two_arm_cmd(..., ik_param=ik_param)
```

注意：容差越大，求解越容易成功，但末端位姿误差可能相应增大；请结合日志中的 `pos_error(mm)` / `ori_error(deg)` 判断是否可接受。

## 3. 六个案例分别做什么

## Case 0: `case_00_desired_pose_ik.py`

在脚本 `main()` 中配置双手期望末端位姿（位置 m + 欧拉角 RPY deg），通过命令行参数 **`--mode`** 选择**一种** IK 方式执行。

流程：

1. 读取 `left_pos` / `right_pos`、`left_rpy` / `right_rpy`，转换为 IK 命令
2. 按 `--mode` 调用对应接口求解
3. 打印 `success`、耗时（Service）、位置/姿态误差；失败时 Service 路径可能输出 `error_reason`

| `--mode` | 接口 | 说明 |
|----------|------|------|
| `topic` | `/ik/two_arm_hand_pose_cmd` | 订阅 `/ik/result` |
| `srv` | `/ik/two_arm_hand_pose_cmd_srv` | 单参考 Service |
| `muli_refer` | `/ik/two_arm_hand_pose_cmd_srv_muli_refer` | 多参考 Service |


## Case 1: `case_01_fk_topic_ik_loop.py`

流程：

1. 给定 `q_arm`（14 维）并调用 FK
2. 发布 `q_arm` 做一次关节跟踪
3. 发布全零 `q_zero` 回零
4. 把 FK 得到的末端位姿通过 Topic 发到 IK
5. 订阅 `/ik/result`，打印误差并执行 IK 输出关节

你可改的主要参数：

- `q_arm`（第 14-15 行）
- `constraint_mode`（`default_ik_param(constraint_mode=3)`）
- `time.sleep(...)` 等待时长

## Case 2: `case_02_fk_srv_ik_loop.py`

流程与 Case1 一样，但 IK 使用服务 `call_ik_srv(...)`，可直接获得：

- `success`
- `time_cost`
- `q_arm`
- `hand_poses`

你可改的主要参数：

- `q_arm`
- `constraint_mode`
- 回零等待时间

## Case 3: `case_03_multi_ref_constraint_compare.py`

固定同一目标位姿，循环 6 种模式做 `muli_refer` 服务 IK：

- `0`: `PosSoft_OriSoft`
- `1`: `PosSoft_OriHard`
- `2`: `PosHard_OriSoft`
- `3`: `PosHard_OriHard`
- `4`: `ThreePoint_Soft`
- `6`: `ThreePoint_Mixed`

可改参数：

- 左右手目标位置：`left_pos` / `right_pos`
- 左右手欧拉角：`left_rpy_deg` / `right_rpy_deg`（脚本内部转四元数）

## Case 4: `case_04_srv_with_elbow_constraint.py`

在末端位姿之外，额外给 `left_elbow` / `right_elbow`，使用多参考 IK 服务求解。

可改参数：

- `left_pos` / `right_pos`
- `left_quat` / `right_quat`
- `left_elbow` / `right_elbow`
- `constraint_mode`

## Case 5: `case_05_fk_srv_muli_refer_ik_loop.py`

流程与 Case2 相同（FK -> 轨迹执行 -> 回零 -> IK），但 IK 使用多参考初值服务：
- 服务：`/ik/two_arm_hand_pose_cmd_srv_muli_refer`
- API：`call_ik_multi_ref_srv(...)`
- 求解策略：依次尝试用户 q0、上一解、限位中点、伪逆/解析种子等多组初值，择优返回

你可改的主要参数：

- `q_arm`
- `constraint_mode`
- `joint_angles_as_q0` 及 `left_joint_angles` / `right_joint_angles`
- 回零等待时间

## 4. 运行命令（终端）

假设当前在工作区根目录 `~/kuavo_ws`：

```bash
# Case0：期望位姿 IK（三种mode选一）
python3 src/manipulation_nodes/motion_capture_ik/scripts/case_wheel_test/case_00_desired_pose_ik.py --mode topic
python3 src/manipulation_nodes/motion_capture_ik/scripts/case_wheel_test/case_00_desired_pose_ik.py --mode srv
python3 src/manipulation_nodes/motion_capture_ik/scripts/case_wheel_test/case_00_desired_pose_ik.py --mode muli_refer
# Case1 ~ Case5
python3 src/manipulation_nodes/motion_capture_ik/scripts/case_wheel_test/case_01_fk_topic_ik_loop.py
python3 src/manipulation_nodes/motion_capture_ik/scripts/case_wheel_test/case_02_fk_srv_ik_loop.py
python3 src/manipulation_nodes/motion_capture_ik/scripts/case_wheel_test/case_03_multi_ref_constraint_compare.py
python3 src/manipulation_nodes/motion_capture_ik/scripts/case_wheel_test/case_04_srv_with_elbow_constraint.py
python3 src/manipulation_nodes/motion_capture_ik/scripts/case_wheel_test/case_05_fk_srv_muli_refer_ik_loop.py
```

建议先确认仿真/机器人与 IK 节点已启动，再单独运行脚本。

## 5. 输出日志解读

典型打印：

- `FK success: True`
  - FK 服务成功返回
- `success=True, time_cost_ms=...`
  - IK 服务/流程求解成功，`time_cost_ms` 为耗时
- `pos_error(mm): L=... R=... max=...`
  - 左右手位置误差（毫米）
- `ori_error(deg): L=... R=... max=...`
  - 左右手姿态误差（角度）

Case3 还会输出总表：

- `mode=... (...) success=... time=... pos_error_max=... ori_error_max=...`
  - 用于横向比较不同 `constraint_mode`

Case5 失败时还可能打印：

- `error_reason: ...`
  - 多参考 IK 服务返回的失败原因说明

若 IK 持续失败，可参照 **2.4 节** 放宽 `default_ik_param()` 中的求解容差（如 `1e-3` → `5e-3`）。

