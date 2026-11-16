# Kuavo5 use_sit_init 坐姿启动流程说明

> 适用范围：Kuavo5 代（主版本 5，如 50–56），实物启动。本文档说明 `use_sit_init:=true` 时从上电到站稳的完整启动链路，供后续手柄改造、座椅起身流程对照参考。

---

## 1. 什么是 use_sit_init

机器人开机不在站立位姿，而**先在座椅上以坐姿启动**：硬件先把关节移动到坐姿准备位（含 P3 偏置终点），控制器接管后用 `standUpWbc` 把机器人从坐姿插值起立到站姿，再初始化 MPC 进入正常控制。

启用方式：`roslaunch ... load_kuavo_real.launch use_sit_init:=true`（仅 Kuavo5 生效，非 5 代忽略并回退到蹲姿启动）。

涉及三个层：
- **hardware_node**（`kuavo-ros-control-lejulib/hardware_node`）：上电、校准、关节移动到准备位、等用户确认。
- **SitControlManager::configureLaunchBoot**（`humanoid_controllers`）：根据 `seat_config_v5.json` 算出准备位关节角，发布到 `/hardware_prep_*` 参数供 hardware_node 执行。
- **humanoidController::preUpdate**（`humanoid_controllers`）：等硬件就绪后执行 sit→stand 起立插值 + MPC 初始化。

---

## 2. 启动时序总览

```
[上电/launch]
   │
   ▼
hardware_node 启动
   ├─ 等待 cppad 构建完成 (real_init_wait, /build_cppad_state==2)
   ├─ 读取准备姿态 squat_joint_pos_（默认蹲姿）
   │   └─ 若有 /hardware_prep_joint_pos_deg → 覆盖为坐姿准备位
   ├─ 若 use_sit_init 两阶段:
   │   ├─ phase1: jointMoveToPrepGoal(prep_offset_deg)   # sit+offset (P3 终点)
   │   ├─ phase2a(可选 reverse_leg_first): jointMoveToPrepGoal(prep_leg_first_deg)  # 反向 offset 中间态
   │   └─ phase2b: jointMoveToPrepGoal(moving_pos)        # sit_joint_pos (P3 起点)
   │      (各段速度由 /hardware_prep_moves 指定，腿 EC 增益由 /hardware_prep_ec_joint_kp/kd)
   ├─ 置 /hardware/ready_to_start=1, hardware_status_=0
   └─ 提示「输入 o 让机器人站起来」
        │ 用户按 'o'
        ▼
hardware_node 收到 'o'
   ├─ hardware_status_=1, 置 /hardware/is_ready=1
   └─ StandUpLoop 轮询 /bot_stand_up_complete
        │
        ▼ (并行的 controller 侧)
humanoidController::starting
   ├─ 读 /use_sit_init → use_sit_init_
   ├─ configureLaunchBoot() → 发布 /hardware_prep_* 参数 (同上, 供 HW 读)
   ├─ real_init_wait() 等 /hardware/is_ready==1   # 等 HW 移动到位 + 用户按 o
   └─ 进入 preUpdate
        ▼
humanoidController::preUpdate  (use_sit_init 起立)
   ├─ squatState = makeBootStartState(sit, ...)        # 起点 = 纯 sit_pos
   ├─ standState = getInitialState()(腿+腰) + defalutArmPosMPC_(臂)
   ├─ motionVel = sitBootStandUpMotionVel()
   ├─ 首帧块: 记录 robotStartStandTime_/robotStandUpCompleteTime_, yaw 刷新
   ├─ 每帧: smoothstep 插值 sit→stand, standUpWbc->update 出力
   ├─ 起立腿增益: sitBootLegGain (坐姿启动高刚度, 防软腿)
   ├─ 到时 (robotStandUpCompleteTime_+0.8):
   │   ├─ resetMpcNode(站立 target = initial_status_)
   │   ├─ 等 initialPolicyReceived → updatePolicy → evaluatePolicy
   │   ├─ isPreUpdateComplete=true
   │   └─ 发布 /bot_stand_up_complete=1
        ▼
hardware_node StandUpLoop 收到 complete=1 → 硬件启动完成
humanoidController::update 主循环接管 (MPC+WBC 正常运行, stance 就绪)
```

---

## 3. 准备位关节角计算（configureLaunchBoot）

`SitControlManager::configureLaunchBoot` 在 controller init 阶段调用，根据 `seat_config_v5.json` + Drake 的 `sit_initial_state`（MPC 状态）算出准备位，发布到 ROS 参数供 hardware_node 读取：

| 参数 | 含义 | 来源 |
|------|------|------|
| `/hardware_prep_joint_pos_deg` | **sit_joint_pos**（纯坐姿，P3 起点） | `sit_mpc_state` → 度 |
| `/hardware_prep_joint_pos_offset_deg` | **sit + seat_offset**（P3 终点，即反向 P3 起点） | sit + `seat_offset_leg/arm_joint_offset_rad` |
| `/hardware_prep_two_phase` | 是否两阶段移动 | true（use_sit_init 时） |
| `/hardware_prep_joint_pos_leg_first_deg` | 反向 offset 中间态（腿/腰回 sit，臂保持 offset） | 仅 `prep_reverse_leg_first=true` |
| `/hardware_prep_reverse_leg_first` | 是否先腿后手 | seat_boot.prep_reverse_leg_first |
| `/hardware_prep_moves` | 各段速度（度/秒）：`[phase1_speed, phase2_speed]` | seat_boot.prep_speed_deg / prep_settle_speed_deg |
| `/hardware_prep_ec_joint_kp/kd` | 腿 EC 电机刚度（坐姿启动专用，防软腿） | seat_boot.leg_joint_kp/kd |

> 关节顺序：腿 | 腰 | 臂 | 头（与 hardware_plant 一致），由 `fillHwPrepJointPosDegFromSitMpc` 从 MPC 状态（12 base + 12 leg + 1 waist + 14 arm）映射。

### 两阶段移动的意义
- **phase1 到 P3 终点（sit+offset）**：相当于「把机器人摆到反向 P3 的起始位置」，这样后续 stand_up 段0（CSP→sit 反向 P3）和开机准备位是同一个位姿。
- **phase2 回到 P3 起点（纯 sit）**：去掉 offset 回到 sit_joint_pos，作为 stand_up 起立的起点。
- `reverse_leg_first`：phase2 可再拆成「先腿/腰回 sit、后臂回 sit」，避免腿臂同时动碰撞座椅。

---

## 4. hardware_node 侧执行（关节移动）

`hardware_node.cc` 主流程：

1. **`real_init_wait()`**：等 cppad 编译完成（`/build_cppad_state==2`），设 `/hardware/is_ready=0`、`/hardware/ready_to_start=0`。
2. **读准备姿态**：默认 `squat_joint_pos_`；若读到 `/hardware_prep_joint_pos_deg` 则覆盖为坐姿准备位。
3. **执行两阶段移动**（`/hardware_prep_two_phase=true`）：
   ```
   jointMoveToPrepGoal(prep_offset_deg, 0)   # phase1: → sit+offset（move_speed）
   [if reverse_leg_first]
     jointMoveToPrepGoal(prep_leg_first_deg, 0)  # phase2a: 腿/腰→sit（move_speed）
   jointMoveToPrepGoal(moving_pos, 1)        # phase2b: 臂→sit 或全身→sit（settle_speed）
   ```
   - `jointMoveToPrepGoal` 从 `/hardware_prep_moves[speed_index]` 取速度，调 `hardware_plant_->jointMoveTo(goal, speed, dt)`。
   - 期间用 `/hardware_prep_ec_joint_kp/kd` 作为腿 EC 增益（`setPrepEcLegGains`），移动完 `clearPrepEcLegGains`。
4. **置就绪**：`/hardware/ready_to_start=1`、`hardware_status_=0`，提示「输入 o 站起来」。
5. **用户按 'o'** → `hardware_status_=1`、`/hardware/is_ready=1` → controller 侧 `real_init_wait()` 解除阻塞。
6. **`StandUpLoop`**：轮询 `/bot_stand_up_complete`（`bot_stand_up_complete_`）：
   - `1` → 站立完成，硬件启动结束。
   - `-1` → 起立失败（异常下蹲），回退到 `ready_to_start=1` 等用户重试。

---

## 5. controller 侧起立（preUpdate）

`humanoidController::preUpdate` 在 `use_sit_init_boot_` 时：

- **起点**：`squatState = makeBootStartState(sit_initial_state, ...)` → 用纯 `sit` 态填充 WBC 状态前 `12+jointNum_` 维（boot 路径，非 `seat_preupdate_reentry_`）。
- **终点**：`standState.head(12+jointNum_) = getInitialState()`（腿+腰站姿），`standState.tail(armNumReal_) = defalutArmPosMPC_`（臂默认位）。
- **速度**：`motionVel = sitBootStandUpMotionVel()`（来自 `seat_boot.stand_up.motion_vel` 或 `stand_up_boot_interpolation.sit.motion_vel`）。
- **起立增益**：腿关节用 `sitBootLegGain`（`seat_boot.leg_joint_kp/kd`，高刚度防软腿），覆盖行走 kp/kd。
- **插值**：`smoothstep01` 在 `[robotStartStandTime_, robotStandUpCompleteTime_]` 上插值，`standUpWbc_->update(curTargetState, ...)` 出力发关节命令。
- **接触保护**：`use_sit_init_boot_` 时跳过 stand_up 接触力保护（`!use_sit_init_boot_` 才查单脚支撑力），因为坐姿起身初期接触力不达标属正常。
- **完成判定**：`time > robotStandUpCompleteTime_ + 0.8`（且 `is_real_||use_sit_init_boot_`）→ 进 MPC 初始化。

### MPC 初始化（preUpdate 完成块）
1. `initial_observation.state = initial_status_`（站立态 + 当前 yaw）。
2. `resetMpcNode(target_trajectories)`。
3. reset 臂滤波到当前实测臂角。
4. spin 等 `initialPolicyReceived` → `updatePolicy` → `evaluatePolicy`。
5. `isPreUpdateComplete=true`、发布 `/bot_stand_up_complete=1`。
6. `loadSwitchParamsSetting` 切换 WBC 步行参数。

> 之后 `humanoidController::update` 主循环接管（MPC+WBC 正常运行），hardware_node 收到 complete=1 结束 StandUpLoop。

---

## 6. use_sit_init 与座椅 stand_up 的同构关系

座椅起身的 `stand_up`（preUpdate 两段重入）刻意复用了 use_sit_init 的起立轨迹：

|  | use_sit_init 启动 | 座椅 stand_up 重入 |
|---|---|---|
| 段0 | HW：0a 腿 `move_speed` → 0b 臂 `settle_speed`（cosine + min 0.5s） | 同构 CSP，同速键 |
| 等 start | 按 `o` / `real_initial_start` → `is_ready=1` | 同（段0 hold 后默认等 start，`seat_return.await_start_after_p5`） |
| 段1 起点纯 sit | HW phase2 摆到的 sit_joint_pos | 段0 终点 = snapshot `sit_joint_targets` |
| 段1 终点 | `getInitialState()+defalutArmPosMPC_` | 同 |
| 起立速度 | `sitToStandComVelocityMps()` | 同 |
| 起立增益 | `sitBootLegGain`（高刚度） | 同（段0/段1 均开） |
| 段0 控制 | 硬件位置跟踪 | CSP `mode=2` + `tau=0`（**不用** SitDownWbc/StandUpWbc） |
| 段1 WBC | `standUpWbc_->update` + smoothstep | 同 |
| MPC 初始化 | `resetMpcNode` + 等 initialPolicy | 同（+ 额外 `pauseResumeMpcNode(false)` 恢复暂停） |

**即：座椅 stand_up 的段0 一比一复刻 use_sit_init 硬件 settle；等 start 后段1 == use_sit_init 的起立段。**

---

## 7. 关键代码位置

| 功能 | 文件 | 函数 |
|------|------|------|
| 启用判定 + 准备位计算 | `humanoid_controllers/.../SitControlManager.cpp` | `configureLaunchBoot` |
| 准备位参数发布 | 同上 | `publishHardwarePrepPlan` |
| 读 use_sit_init 参数 | `humanoid_controllers/.../humanoidController.cpp` | `init`（`/use_sit_init`） |
| 仿真初始位姿设为 sit | 同上（`use_sit_init_ && !is_real_`） | `starting` |
| HW 就绪等待 | 同上 | `real_init_wait` |
| sit→stand 起立插值 + MPC 初始化 | 同上 | `preUpdate` |
| HW 关节移动到准备位 | `kuavo-ros-control-lejulib/hardware_node/.../hardware_node.cc` | 主流程 + `jointMoveToPrepGoal` |
| HW 等用户按 o + StandUpLoop | 同上 | `StandUpLoop` |
| 配置来源 | `kuavo_assets/config/seat_config_v5.json` | `seat_boot.*`、`seat_offset_*` |

---

## 8. 配置项速查（seat_config_v5.json 的 seat_boot）

| 字段 | 含义 |
|------|------|
| `prep_speed_deg` | phase1 速度（度/秒） |
| `prep_settle_speed_deg` | phase2 速度（度/秒） |
| `prep_reverse_leg_first` | phase2 是否先腿后手 |
| `leg_joint_kp` / `leg_joint_kd` | 坐姿启动腿 EC 刚度（12 维，防软腿） |
| `stand_up.motion_vel` | sit→stand 起立速度（被 `sitBootStandUpMotionVel` 读） |
