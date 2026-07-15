# RL控制框架ROS接口文档

本文档整理了RL控制框架中提供的所有ROS服务接口和监控话题，包括控制器管理、控制器状态查询、以及特定控制器的功能服务和监控调试话题。
- 其他关联文档：
  - [倒地起身说明](../src/humanoid-control/humanoid_controllers/docs/倒地起身操作说明.md)
  - [RLController 多控制器框架说明](../src/humanoid-control/humanoid_controllers/docs/RLController多控制器框架说明.md)（架构、类关系、多舞蹈与行走列表差异）

## 目录

1. [控制器管理服务（RLControllerManager）](#1-控制器管理服务rlcontrollermanager) — 行走列表/循环/倒地状态、VMP及**多舞蹈**接口
2. [控制器基础服务（RLControllerBase）](#2-控制器基础服务rlcontrollerbase) - 5个服务
3. [倒地起身控制器服务（FallStandController）](#3-倒地起身控制器服务fallstandcontroller) - 1个服务
4. [主控制器服务（humanoidController）](#4-主控制器服务humanoidcontroller) - 5个服务
5. [腰部控制器接口（WaistController）](#5-腰部控制器接口waistcontroller) - 2个话题
6. [监控与调试话题](#6-监控与调试话题) - 5个话题

---

## 1. 控制器管理服务（RLControllerManager）

这些服务由 `RLControllerManager` 提供，用于管理多个RL控制器的切换、查询和状态管理。所有服务位于 `/humanoid_controller` 命名空间下。

### 1.1 `/humanoid_controller/switch_controller`

**服务类型**: `kuavo_msgs/switchController`

**功能**: 切换到指定的控制器

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| controller_name | string | 请求数据, 要切换到的控制器名称。`"mpc"`：切回MPC基础控制器；其他名称（如 `"amp_controller"`、`"depth_loco_controller"` 等）：切换到对应的RL控制器（须为行走控制器列表中的名称，可用 `get_controller_list` 查询） |
| success | bool | 返回数据, 切换是否成功 |
| message | string | 返回数据, 返回消息，包含成功或失败的原因 |

**使用说明**:
- 只能切换到已加载且启用的控制器
- 控制器必须在 `walk_controllers_` 列表中（**不包含**舞蹈控制器；舞蹈请使用 **1.7 节** `switch_to_dance_controller`）
- 从RL切换到MPC时，如果RL控制器不在stance状态，切换会被阻止
- 从MPC切换到RL时，如果MPC不在stance状态，切换会被阻止（倒地起身控制器除外）

**示例**:
```bash
# 切换到MPC控制器
rosservice call /humanoid_controller/switch_controller "controller_name: 'mpc'"

# 切换到AMP行走控制器
rosservice call /humanoid_controller/switch_controller "controller_name: 'amp_controller'"
```
---

#### 1.2 `/humanoid_controller/set_rl_switch_mode`

**类型:** `std_srvs::SetBool`

**功能说明:**

设置 RL 控制器切换模式。

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| data | bool | 请求数据, true 表示开启 RL 控制器切换模式, false 表示关闭 |
| success | bool | 返回数据, 是否调用成功 |
| message | string | 返回数据, 调用结果描述信息 |

---

### 1.3 `/humanoid_controller/get_controller_list`

**服务类型**: `kuavo_msgs/getControllerList`

**功能**: 获取当前可用的控制器列表和当前激活的控制器信息

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| controller_names | string[] | 返回数据, 可用控制器名称列表（包含 `"mpc"`） |
| count | int32 | 返回数据, 控制器数量 |
| current_index | int32 | 返回数据, 当前控制器索引（-1表示未找到） |
| current_controller | string | 返回数据, 当前控制器名称（"mpc"表示MPC控制器） |
| success | bool | 返回数据, 获取是否成功 |
| message | string | 返回数据, 返回消息 |

**使用说明**:
- MPC控制器始终在索引0
- 返回的列表只包含行走控制器（walk_controllers_），不包括其他类型的控制器

**示例**:
```bash
rosservice call /humanoid_controller/get_controller_list
```

---

### 1.4 `/humanoid_controller/switch_to_next_controller`

**服务类型**: `kuavo_msgs/switchToNextController`

**功能**: 在控制器列表中循环切换到下一个控制器

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| success | bool | 返回数据, 切换是否成功 |
| message | string | 返回数据, 返回消息 |
| current_controller | string | 返回数据, 切换前的控制器名称 |
| next_controller | string | 返回数据, 切换后的控制器名称 |
| current_index | int32 | 返回数据, 切换前的控制器索引 |
| next_index | int32 | 返回数据, 切换后的控制器索引 |

**使用说明**:
- 按顺序循环切换：`mpc → 第一个RL → ... → 最后一个RL → mpc`
- 适合作为手柄或键盘的"一键切换模式"接口
- 切换逻辑与 `switch_controller` 相同，包含相同的保护机制

**示例**:
```bash
rosservice call /humanoid_controller/switch_to_next_controller
```

---

### 1.5 `/humanoid_controller/switch_to_previous_controller`

**服务类型**: `kuavo_msgs/switchToNextController`

**功能**: 在控制器列表中反向循环切换到上一个控制器

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| success | bool | 返回数据, 切换是否成功 |
| message | string | 返回数据, 返回消息 |
| current_controller | string | 返回数据, 切换前的控制器名称 |
| next_controller | string | 返回数据, 切换后的控制器名称 |
| current_index | int32 | 返回数据, 切换前的控制器索引 |
| next_index | int32 | 返回数据, 切换后的控制器索引 |

**使用说明**:
- 按逆序循环切换：`mpc → 最后一个RL → ... → 第一个RL → mpc`
- 适合作为手柄或键盘的"一键切换模式"接口（反向切换）
- 与 `switch_to_next_controller`（1.3）共享相同的退出保护机制：当前控制器不允许退出（如 AMP 还在行走）或为倒地起身控制器时，拒绝切换
- 使用 `findNextSwitchableIndex(current_index, -1)` 沿环形列表反向搜索下一个可切换的控制器（自动跳过深度话题未就绪的 `depth_loco_controller` 等）

**示例**:
```bash
rosservice call /humanoid_controller/switch_to_previous_controller
```

---

### 1.6 `/humanoid_controller/switch_to_vmp_controller`

**服务类型**: `std_srvs/Trigger`

**功能**: 一键切换到 VMP（Visual Motion Planning）控制器

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| success | bool | 返回数据, 切换是否成功 |
| message | string | 返回数据, 返回消息，包含切换结果或失败原因 |

**使用说明**:
- VMP 控制器**不在**行走控制器列表（`walk_controllers_`）中，因此**无法**通过 `switch_controller`（1.1）或 `switch_to_next_controller`（1.4）切换到此控制器
- 需要 `rl_controllers.yaml` 中启用至少一条 `type: VMP_CONTROLLER` 配置
- 切换逻辑与 `switch_controller` 一致，受相同的 stance / exit 保护限制

**示例**:
```bash
rosservice call /humanoid_controller/switch_to_vmp_controller
```

---

### 1.7 `/humanoid_controller/set_fall_down_state`

**服务类型**: `std_srvs/SetBool`

**功能**: 设置机器人的倒地状态，并自动切换到倒地起身控制器

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| data | bool | 请求数据, `true`: 设置为倒地状态（FALL_DOWN）；`false`: 设置为站立状态（STANDING） |
| success | bool | 返回数据, 设置是否成功 |
| message | string | 返回数据, 返回消息，包含状态设置和控制器切换的结果 |

**使用说明**:
- 当设置为倒地状态（`true`）时：
  1. 通过回调函数更新 `humanoidController` 的 `fall_down_state_` 成员变量
  2. 如果存在 `FALL_STAND_CONTROLLER`，会自动切换到倒地起身控制器
  3. 如果切换失败或控制器不存在，会在响应消息中说明
- 当设置为站立状态（`false`）时：
  - 仅更新 `fall_down_state_` 为 `STANDING`，不进行控制器切换
- 用于外部系统（如状态估计模块）通知主控制器机器人已倒地

**示例**:
```bash
# 设置为倒地状态（会自动切换到倒地起身控制器）
rosservice call /humanoid_controller/set_fall_down_state "data: true"

# 设置为站立状态
rosservice call /humanoid_controller/set_fall_down_state "data: false"
```

---

### 1.8 `/humanoid_controller/switch_to_dance_controller`

**服务类型**: `kuavo_msgs/SetString`

**功能**: 切换到指定舞蹈 RL 控制器实例。逻辑与 `RLControllerManager::switchDanceControllerByStringCallback` 一致（多支舞在 `rl_controllers.yaml` 中配置多条 `type: DANCE_CONTROLLER`）。

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| data | string | 请求数据。空字符串 `""`：切换到舞蹈列表中的第一项；`#` + 非负整数（如 `#0`、`#1`）：按 `get_dance_controller_list` 返回的 `data[]` 下标切换；其他字符串：按已注册的控制器名称切换 |
| success | bool | 返回数据, 是否切换成功 |
| message | string | 返回数据, 说明信息或失败原因 |

**使用说明**:
- 需在对应版本 `rl_controllers.yaml` 中启用至少一条 `DANCE_CONTROLLER`
- 从 MPC 切到舞蹈时，仍受「MPC 须在 stance」等与 `switchController` 相同的保护；**不同舞蹈实例之间**允许直接切换（由各自 `resume()` 重置轨迹）
- 从舞蹈切回 MPC/行走时，若舞蹈侧 `requestToExit()` 为 false，可能被 mimic 保护拦截（与倒地起身类似逻辑，详见框架说明文档）

**示例**:
```bash
# 切换到 yaml 中第一个 DANCE_CONTROLLER
rosservice call /humanoid_controller/switch_to_dance_controller "data: ''"

# 按列表下标（第二个舞蹈）
rosservice call /humanoid_controller/switch_to_dance_controller "data: '#1'"

# 按控制器名称
rosservice call /humanoid_controller/switch_to_dance_controller "data: 'dance_controller'"
```

---

### 1.9 `/humanoid_controller/get_dance_controller_list`

**服务类型**: `kuavo_msgs/GetStringList`

**功能**: 返回当前已加载、且类型为 `DANCE_CONTROLLER` 的控制器 **name** 列表，顺序与 `rl_controllers.yaml` 中声明顺序一致（内部为 `dance_controllers_`）。

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| data | string[] | 返回数据, 舞蹈控制器名称列表 |
| success | bool | 返回数据, 查询是否成功 |
| message | string | 返回数据, 简要说明（如舞蹈数量） |

**使用说明**:
- 与 `get_controller_list` 互补：后者只返回**行走**列表（含 `mpc`），本服务只列舞蹈项
- 可与 1.8 配合：先 `get_dance_controller_list` 再按名或 `#索引` 调用 `switch_to_dance_controller`

**示例**:
```bash
rosservice call /humanoid_controller/get_dance_controller_list
```

---

## 2. 控制器基础服务（RLControllerBase）

这些服务由所有 RL 控制器（如 `AmpWalkController`、`FallStandController`、`DanceController` 等）继承提供。服务命名空间为 `/humanoid_controllers/{controller_name}`，其中 `{controller_name}` 是控制器的名称（如 `amp_controller`、`fall_stand_controller`、`dance_controller` 等）。

### 2.1 `/humanoid_controllers/{controller_name}/reload`

**服务类型**: `std_srvs/Trigger`

**功能**: 重新加载控制器的配置文件

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| success | bool | 返回数据, 重新加载是否成功 |
| message | string | 返回数据, 返回消息 |

**使用说明**:
- 只有在控制器处于非运行状态（PAUSED或STOPPED）时才能重新加载
- 如果控制器正在运行，会返回失败并提示先停止或暂停控制器

**示例**:
```bash
# 重新加载amp_walk控制器的配置
rosservice call /humanoid_controllers/amp_controller/reload

# 重新加载fall_stand控制器的配置
rosservice call /humanoid_controllers/fall_stand_controller/reload
```

---

### 2.2 `/humanoid_controllers/{controller_name}/isActive`

**服务类型**: `std_srvs/Trigger`

**功能**: 查询控制器是否处于激活状态

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| success | bool | 返回数据, 控制器是否激活（true表示激活，false表示未激活） |
| message | string | 返回数据, 返回消息（"Controller is active" 或 "Controller is not active"） |

**使用说明**:
- 控制器处于 `RUNNING` 状态时返回 `true`
- 控制器处于 `PAUSED`、`STOPPED` 或 `INITIALIZING` 状态时返回 `false`

**示例**:
```bash
rosservice call /humanoid_controllers/amp_controller/isActive
```

---

### 2.3 `/humanoid_controllers/{controller_name}/getState`

**服务类型**: `std_srvs/Trigger`

**功能**: 获取控制器的当前状态

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| success | bool | 返回数据, 查询是否成功（始终为true） |
| message | string | 返回数据, 状态码（整数字符串）。`0`: INITIALIZING（初始化中）；`1`: RUNNING（运行中）；`2`: PAUSED（已暂停）；`3`: ERROR（运行错误）；`4`: STOPPED（已停止） |

**使用说明**:
- 状态码以字符串形式返回，需要解析为整数

**示例**:
```bash
rosservice call /humanoid_controllers/amp_controller/getState
```

---

### 2.4 `/humanoid_controllers/{controller_name}/getType`

**服务类型**: `std_srvs/Trigger`

**功能**: 获取控制器的类型

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| success | bool | 返回数据, 查询是否成功（始终为true） |
| message | string | 返回数据, 控制器类型码（整数字符串）。`0`: MPC（基础控制器）；`1`: AMP_CONTROLLER（AMP行走控制器）；`2`: FALL_STAND_CONTROLLER（倒地起身控制器）；`3`: PERCEPTION_LOCO_CONTROLLER（感知行走控制器）；`4`: DEPTH_LOCO_CONTROLLER；`5`: VMP_CONTROLLER（VMP控制器）；`6`: DANCE_CONTROLLER（跳舞控制器） |

**使用说明**:
- 类型码以字符串形式返回，需要解析为整数

**示例**:
```bash
rosservice call /humanoid_controllers/amp_controller/getType
```

---

### 2.5 `/humanoid_controllers/{controller_name}/reset`

**服务类型**: `std_srvs/Trigger`

**功能**: 重置控制器的内部状态

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| success | bool | 返回数据, 重置是否成功 |
| message | string | 返回数据, 返回消息 |

**使用说明**:
- 只有在控制器处于非运行状态（PAUSED或STOPPED）时才能重置
- 如果控制器正在运行，会返回失败并提示先停止或暂停控制器
- 重置会清除控制器的内部状态（如相位、动作历史等）

**示例**:
```bash
rosservice call /humanoid_controllers/amp_controller/reset
```

---

## 3. 倒地起身控制器服务（FallStandController）

这些服务由 `FallStandController` 提供，专门用于倒地起身功能。

### 3.1 `/humanoid_controller/trigger_fall_stand_up`

**服务类型**: `std_srvs/Trigger`

**功能**: 触发倒地起身流程

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| success | bool | 返回数据, 触发是否成功 |
| message | string | 返回数据, 返回消息，说明触发结果或失败原因 |

**使用说明**:
- 只有在控制器处于 `RUNNING` 状态时才能触发
- 只有在机器人处于 `FALL_DOWN` 状态时才能触发
- 触发后会：
  1. 根据当前机体姿态自动选择趴着/躺着模型
  2. 重置轨迹时间步
  3. 计算当前与轨迹参考yaw差
  4. 进入 `READY_FOR_STAND_UP` 状态，开始关节空间插值
  5. 插值完成后自动进入 `STAND_UP` 状态，开始RL控制起身

**状态机流程**:
- `FALL_DOWN` → `READY_FOR_STAND_UP` → `STAND_UP` → `STANDING`

**示例**:
```bash
rosservice call /humanoid_controller/trigger_fall_stand_up
```

---

## 4. 主控制器服务（humanoidController）

这些服务由 `humanoidController` 直接提供，用于控制搬运模式等全局状态。

### 4.1 `/humanoid_controller/transport_mode_command`

**服务类型**: `kuavo_msgs/TransportModeCommand`

**功能**: 搬运模式控制 —— 进入后全身僵直（电机锁死），可安全抬起移动；退出后恢复原有控制。

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| command | uint8 | 请求数据, `1 = TRANSPORT_ENTER`: 进入搬运（僵直）；`2 = TRANSPORT_EXIT`: 退出搬运；`3 = TRANSPORT_FALL_DOWN`: 瘫软倒地 |
| success | bool | 返回数据, 是否成功 |
| message | string | 返回数据, 说明信息或失败原因 |

**使用说明**:
- ENTER：需控制器处于 MPC 或 AMP 模式、非拉保护状态。进入后 1s 插值到搬运姿势，全身僵直锁死（腿+腰+臂+头关节覆盖）；同时暂停 MPC 优化节点、跳过 WBC/MPC/RL 计算，由搬运分支直接生成位控命令。语音播报「进入搬运模式」
- EXIT：先解算站立姿态，再检测（EMA 接触力 + 倾角 ≤0.10rad，20 帧确认）。通过后解除僵直：若进入前为 MPC，则触发 `reset_mpc_` 走 stance 恢复流程；若为 AMP/RL，则直接恢复 AMP/RL 运行。语音播报「退出搬运模式」
- FALL_DOWN：仅搬运中可用，瘫软倒地，电机掉使能，由**倒地起身控制器接管**，输出零力矩 limp。**正常模式下 LB+RB+B 彻底屏蔽防误触**
- 起身：倒地后 LB+RB+X 触发 `trigger_fall_stand_up`（见 [3.1 节](#31-humanoid_controllertrigger_fall_stand_up)），第 1 下回起身初始姿态，第 2 下完整起身，完成后自动回到 **MPC stance**。语音播报「退出搬运模式」
- 搬运期间拉保护检测关闭

**状态机流程**:
- `INACTIVE` → `INTERPOLATING`(1s) → `ACTIVE`(僵直) → EXIT(站立检测) → `INACTIVE` → 恢复 AMP/MPC
- `ACTIVE`(僵直) → `FALL_DOWN`(瘫软) → FallStandController 接管 → 两步起身 → **MPC stance**

**示例**:
```bash
rosservice call /humanoid_controller/transport_mode_command "command: 1"  # 僵直
rosservice call /humanoid_controller/transport_mode_command "command: 2"  # 退出
rosservice call /humanoid_controller/transport_mode_command "command: 3"  # 瘫软
```

---

## 5. 腰部控制器接口（WaistController）

`WaistController` 是集成在RL控制器中的腰部控制模块，提供外部控制腰部关节的功能。支持两种控制模式：模式1（RL控制）和模式2（外部控制）。

**话题接口**:
- `/humanoid_controller/enable_waist_control` (`std_msgs/Bool`): 启用/禁用腰部外部控制
  - `true`: 切换到模式2（外部控制）
  - `false`: 切换回模式1（RL控制），使用低通滤波器平滑过渡到默认位置
- `/robot_waist_motion_data` (`kuavo_msgs/robotWaistControl`): 发送外部腰部控制指令（仅在模式2时生效）
  - `data.data[]`: 腰部关节目标角度（度），超出范围的值会被自动限制

**控制模式**:
- **模式1（RL控制）**: 默认模式，由RL控制器完全控制。从模式2切换回时，如果误差大于阈值（0.02 rad），会使用低通滤波器平滑过渡到默认位置
- **模式2（外部控制）**: 通过 `/robot_waist_motion_data` 接收外部指令，经过低通滤波处理。仿真环境会计算PD前馈扭矩，实物环境不计算

**配置参数** (`waistControllerParam`):
- `mode2CutoffFreq`: 低通滤波器截止频率（Hz，默认0.8）
- `kp`: PD控制位置增益（默认10.0）
- `kd`: PD控制速度增益（默认2.0）

**启用条件**: 需在配置文件中设置 `use_external_waist_controller = true`，且机器人有腰部关节（`waist_dof_ > 0`）

---

## 6. 监控与调试话题

这些话题由 `humanoidController` 和 `RLControllerManager` 实时发布，用于监控控制器状态、切换事件和调试MPC↔RL模式切换过程。

### 6.1 `/humanoid_controller/is_rl_controller_`

**话题类型**: `std_msgs/Float64`

**发布频率**: 与控制循环频率相同（通常为100Hz或更高）

**功能**: 实时发布当前是否处于RL控制模式

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| data | float64 | `1.0`: 当前处于RL控制模式；`0.0`: 当前处于MPC控制模式 |

**使用说明**:
- 状态由 `!controller_manager_->isBaseControllerActive()` 决定
- 便于监控MPC↔RL模式切换
- 可用于外部系统（如可视化工具、日志记录）判断当前控制模式

**订阅示例**:
```bash
# 使用rostopic查看
rostopic echo /humanoid_controller/is_rl_controller_

# 使用rqt_plot可视化
rqt_plot /humanoid_controller/is_rl_controller_/data
```

---

### 6.2 `/humanoid_controller/resetting_mpc_state_`

**话题类型**: `std_msgs/Float64`

**发布频率**: 与控制循环频率相同（通常为100Hz或更高）

**功能**: 实时发布MPC重置状态，用于监控从RL切换到MPC时的重置过程

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| data | float64 | MPC重置状态码。`0` (`NOMAL`): 正常状态，MPC正常运行；`1` (`RESET_INITIAL_POLICY`): 重置MPC状态1，等待初始策略；`2` (`RESET_BASE`): 重置MPC状态2，更新躯干位置（插值阶段） |

**状态转换流程**:
- 当从RL切回MPC时，状态会依次经历：
  - `RESET_INITIAL_POLICY` (1) → `RESET_BASE` (2) → `NOMAL` (0)
- 便于监控MPC重置进度和调试切换过程

**使用说明**:
- 在RL→MPC切换过程中，可以通过此话题监控重置进度
- 当状态为 `NOMAL` (0) 时，表示MPC已完全重置并正常运行
- 可用于外部系统判断MPC是否已完成重置，避免在重置过程中执行其他操作

**订阅示例**:
```bash
# 使用rostopic查看
rostopic echo /humanoid_controller/resetting_mpc_state_

# 使用rqt_plot可视化
rqt_plot /humanoid_controller/resetting_mpc_state_/data
```

---

### 6.3 `/humanoid_controller/controller_switch_event`

**话题类型**: `kuavo_msgs/ControllerSwitchEvent`

**发布方**: `RLControllerManager`（latch=true）

**功能**: 发布控制器切换事件，记录每次切换的来源和目标控制器

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| header | std_msgs/Header | 时间戳 |
| from_controller | string | 切换前的控制器名称（`"mpc"` 表示 MPC） |
| to_controller | string | 切换后的控制器名称（`"mpc"` 表示 MPC） |

**使用说明**:
- 每次控制器切换成功后发布一条消息（含 index/name 切换、循环切换、舞蹈切换等所有切换路径）
- 可用于日志记录、切换事件追踪等
- latch 模式确保新订阅者能获取最后一次切换事件

**订阅示例**:
```bash
rostopic echo /humanoid_controller/controller_switch_event
```

---

### 6.4 `/humanoid_controller/depth_history_status`

**话题类型**: `std_msgs/Int32`

**发布方**: `RLControllerManager`（latch=true）

**功能**: 发布深度历史话题（`/camera/depth/depth_history_array`）的检测状态码，用于判断是否允许切换到 `DEPTH_LOCO_CONTROLLER`

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| data | int32 | 状态码。`0`：OK，话题正常（有发布、频率达标、消息未过期）；非 0：话题异常（原因各有对应状态码，详见 `TopicMonitor::statusCode`） |

**使用说明**:
- 状态码由 `TopicMonitor` 后台周期检测更新
- 切换到 `DEPTH_LOCO_CONTROLLER` 需此状态为 0（OK），否则 `switch_controller` / `switch_to_next_controller` 会自动跳过

**订阅示例**:
```bash
rostopic echo /humanoid_controller/depth_history_status
```

---

### 6.5 `/humanoid_controller/dance_trajectory_state`

**话题类型**: `kuavo_msgs/DanceTrajectoryState`

**发布方**: 所有 `DanceController` 实例（通过 `RLControllerManager` 注册的共享 publisher）

**功能**: 发布当前舞蹈轨迹的播放状态，供音乐播放等外部模块使用

**消息字段:**

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| header | std_msgs/Header | 时间戳 |
| dance_name | string | 当前舞蹈名称 |
| state | string | 轨迹状态（如 `"started"`、`"running"`、`"finished"` 等） |
| run_id | uint32 | 本次运行 ID |
| current_step | int32 | 当前步数 |
| total_steps | int32 | 总步数 |

**使用说明**:
- 所有舞蹈控制器共享同一个 publisher（由 `RLControllerManager` 统一管理，避免多 publisher 启动期 race）
- 可用于外部系统（如音乐播放模块）根据轨迹状态同步音频

**订阅示例**:
```bash
rostopic echo /humanoid_controller/dance_trajectory_state
```

---

## 服务调用示例

### 完整的控制器切换流程

```bash
# 1. 查询可用的控制器列表
rosservice call /humanoid_controller/get_controller_list

# 2. 切换到AMP行走控制器
rosservice call /humanoid_controller/switch_controller "controller_name: 'amp_controller'"

# 3. 查询控制器状态
rosservice call /humanoid_controllers/amp_controller/getState
rosservice call /humanoid_controllers/amp_controller/isActive

# 4. 切回MPC控制器
rosservice call /humanoid_controller/switch_controller "controller_name: 'mpc'"
```

### 多舞蹈切换流程

```bash
# 1. 查看已加载的舞蹈控制器名称（顺序与 rl_controllers.yaml 一致）
rosservice call /humanoid_controller/get_dance_controller_list

# 2. 进入第一个舞蹈（空 data）
rosservice call /humanoid_controller/switch_to_dance_controller "data: ''"

# 3. 切换到列表中的第二个舞蹈（若存在）
rosservice call /humanoid_controller/switch_to_dance_controller "data: '#1'"

# 4. 从舞蹈回到行走：须先满足 stance 等条件，再通过 switch_controller 切 amp 等
rosservice call /humanoid_controller/switch_controller "controller_name: 'amp_controller'"
```

### VMP 控制器切换流程

```bash
# 一键切换到 VMP 控制器（不经过行走列表循环）
rosservice call /humanoid_controller/switch_to_vmp_controller

# 切回 MPC
rosservice call /humanoid_controller/switch_controller "controller_name: 'mpc'"
```

### 双向循环切换流程

```bash
# 正向循环（下一个）
rosservice call /humanoid_controller/switch_to_next_controller

# 反向循环（上一个）
rosservice call /humanoid_controller/switch_to_previous_controller
```

### 倒地起身流程

```bash
# 1. 设置倒地状态（会自动切换到倒地起身控制器）
rosservice call /humanoid_controller/set_fall_down_state "data: true"

# 2. 触发起身流程
rosservice call /humanoid_controller/trigger_fall_stand_up

# 3. 等待起身完成后，设置站立状态
rosservice call /humanoid_controller/set_fall_down_state "data: false"
```

---

## 注意事项

1. **控制器切换保护机制**:
   - 从RL切换到MPC时，RL控制器必须在允许退出状态（`isAllowToExit()` 返回 true）
   - 从MPC切换到RL时，MPC必须在stance状态（倒地起身控制器除外）
   - 切换前需躯干速度稳定（`isTorsoVelocityStable()`），否则 `switch_controller` 和 `switch_to_next_controller` 均会拒绝切换
   - 切换到 `DEPTH_LOCO_CONTROLLER` 时，需深度历史话题（`/camera/depth/depth_history_array`）可用（最低频率 55Hz、最大过期 0.2s、最少 10 个采样）
   - 倒地起身控制器在未完成起身任务前，不允许切换到其他控制器
   - **舞蹈控制器**：`switch_controller` 的行走列表**不包含**舞蹈；请使用 `switch_to_dance_controller`（SetString，**1.7 节**）。**舞蹈 A → 舞蹈 B** 允许直接切换；**舞蹈 → MPC/行走** 仍受上述 stance / `requestToExit` 等限制
   - **VMP 控制器**：不在行走列表中，须使用专用 `switch_to_vmp_controller`（**1.5 节**）切换

2. **控制器状态**:
   - `INITIALIZING`: 控制器正在初始化，不能执行操作
   - `RUNNING`: 控制器正在运行，可以执行控制
   - `PAUSED`: 控制器已暂停，推理线程继续运行但不执行控制
   - `ERROR`: 控制器运行错误
   - `STOPPED`: 控制器已停止，推理线程已退出

3. **服务命名空间**:
   - 控制器管理服务：`/humanoid_controller/*`（包括行走切换/列表/循环、VMP切换、舞蹈切换/列表、倒地状态设置等）
   - 控制器基础服务：`/humanoid_controllers/{controller_name}/*`（每个RL控制器的独立服务）
   - 倒地起身服务：`/humanoid_controller/trigger_fall_stand_up`
   - 监控话题：`/humanoid_controller/` 命名空间下（含 `is_rl_controller_`、`resetting_mpc_state_`、`controller_switch_event`、`depth_history_status`、`dance_trajectory_state`）

4. **行走控制器列表（`walk_controllers_`）构成**:
   - 仅包含 `mpc` + `AMP_CONTROLLER` + `DEPTH_LOCO_CONTROLLER` 类型的控制器
   - `FALL_STAND_CONTROLLER`、`VMP_CONTROLLER`、`DANCE_CONTROLLER` **不在**行走列表中，需使用各自专用接口

---
