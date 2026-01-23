# RL控制框架ROS接口文档

本文档整理了RL控制框架中提供的所有ROS服务接口和监控话题，包括控制器管理、控制器状态查询、以及特定控制器的功能服务和监控调试话题。
- 其他关联文档：
  - [倒地起身说明](../src/humanoid-control/humanoid_controllers/docs/倒地起身操作说明.md)
  - [RLController多控制器框架说明（初期版本）](../src/humanoid-control/humanoid_controllers/docs/RLController多控制器框架说明.md)

## 目录

1. [控制器管理服务（RLControllerManager）](#1-控制器管理服务rlcontrollermanager) - 4个服务
2. [控制器基础服务（RLControllerBase）](#2-控制器基础服务rlcontrollerbase) - 5个服务
3. [倒地起身控制器服务（FallStandController）](#3-倒地起身控制器服务fallstandcontroller) - 1个服务
4. [主控制器服务（humanoidController）](#4-主控制器服务humanoidcontroller) - 5个服务
5. [腰部控制器接口（WaistController）](#5-腰部控制器接口waistcontroller) - 2个话题
6. [监控与调试话题](#6-监控与调试话题) - 2个话题

---

## 1. 控制器管理服务（RLControllerManager）

这些服务由 `RLControllerManager` 提供，用于管理多个RL控制器的切换、查询和状态管理。所有服务位于 `/humanoid_controller` 命名空间下。

### 1.1 `/humanoid_controller/switch_controller`

**服务类型**: `kuavo_msgs/switchController`

**功能**: 切换到指定的控制器

**请求参数**:
- `controller_name` (string): 要切换到的控制器名称
  - `"mpc"` 或空字符串：切回MPC基础控制器
  - 其他名称（如 `"amp_controller"`、`"fall_stand_controller"` 等）：切换到对应的RL控制器

**响应参数**:
- `success` (bool): 切换是否成功
- `message` (string): 返回消息，包含成功或失败的原因

**使用说明**:
- 只能切换到已加载且启用的控制器
- 控制器必须在 `walk_controllers_` 列表中
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

### 1.2 `/humanoid_controller/get_controller_list`

**服务类型**: `kuavo_msgs/getControllerList`

**功能**: 获取当前可用的控制器列表和当前激活的控制器信息

**请求参数**: 无

**响应参数**:
- `controller_names` (string[]): 可用控制器名称列表（包含 `"mpc"`）
- `count` (int32): 控制器数量
- `current_index` (int32): 当前控制器索引（-1表示未找到）
- `current_controller` (string): 当前控制器名称（"mpc"表示MPC控制器）
- `success` (bool): 获取是否成功
- `message` (string): 返回消息

**使用说明**:
- MPC控制器始终在索引0
- 返回的列表只包含行走控制器（walk_controllers_），不包括其他类型的控制器

**示例**:
```bash
rosservice call /humanoid_controller/get_controller_list
```

---

### 1.3 `/humanoid_controller/switch_to_next_controller`

**服务类型**: `kuavo_msgs/switchToNextController`

**功能**: 在控制器列表中循环切换到下一个控制器

**请求参数**: 无

**响应参数**:
- `success` (bool): 切换是否成功
- `message` (string): 返回消息
- `current_controller` (string): 切换前的控制器名称
- `next_controller` (string): 切换后的控制器名称
- `current_index` (int32): 切换前的控制器索引
- `next_index` (int32): 切换后的控制器索引

**使用说明**:
- 按顺序循环切换：`mpc → 第一个RL → ... → 最后一个RL → mpc`
- 适合作为手柄或键盘的"一键切换模式"接口
- 切换逻辑与 `switch_controller` 相同，包含相同的保护机制

**示例**:
```bash
rosservice call /humanoid_controller/switch_to_next_controller
```

---

### 1.4 `/humanoid_controller/set_fall_down_state`

**服务类型**: `std_srvs/SetBool`

**功能**: 设置机器人的倒地状态，并自动切换到倒地起身控制器

**请求参数**:
- `data` (bool):
  - `true`: 设置为倒地状态（FALL_DOWN）
  - `false`: 设置为站立状态（STANDING）

**响应参数**:
- `success` (bool): 设置是否成功
- `message` (string): 返回消息，包含状态设置和控制器切换的结果

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

## 2. 控制器基础服务（RLControllerBase）

这些服务由所有RL控制器（包括 `AmpWalkController` 和 `FallStandController`）继承提供。服务命名空间为 `/humanoid_controllers/{controller_name}`，其中 `{controller_name}` 是控制器的名称（如 `amp_controller`、`fall_stand_controller` 等）。

### 2.1 `/humanoid_controllers/{controller_name}/reload`

**服务类型**: `std_srvs/Trigger`

**功能**: 重新加载控制器的配置文件

**请求参数**: 无

**响应参数**:
- `success` (bool): 重新加载是否成功
- `message` (string): 返回消息

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

**请求参数**: 无

**响应参数**:
- `success` (bool): 控制器是否激活（true表示激活，false表示未激活）
- `message` (string): 返回消息（"Controller is active" 或 "Controller is not active"）

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

**请求参数**: 无

**响应参数**:
- `success` (bool): 查询是否成功（始终为true）
- `message` (string): 状态码（整数字符串）
  - `0`: INITIALIZING（初始化中）
  - `1`: RUNNING（运行中）
  - `2`: PAUSED（已暂停）
  - `3`: STOPPED（已停止）

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

**请求参数**: 无

**响应参数**:
- `success` (bool): 查询是否成功（始终为true）
- `message` (string): 控制器类型码（整数字符串）
  - `0`: MPC（基础控制器）
  - `1`: FALL_STAND_CONTROLLER（倒地起身控制器）
  - `2`: AMP_CONTROLLER（AMP行走控制器）
  - 其他: 未来可能扩展的类型

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

**请求参数**: 无

**响应参数**:
- `success` (bool): 重置是否成功
- `message` (string): 返回消息

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

**请求参数**: 无

**响应参数**:
- `success` (bool): 触发是否成功
- `message` (string): 返回消息，说明触发结果或失败原因

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

这些话题由 `humanoidController` 通过 `TopicLogger` 实时发布，用于监控控制器状态和调试MPC↔RL模式切换过程。

### 6.1 `/humanoid_controller/is_rl_controller_`

**话题类型**: `std_msgs/Float64`

**发布频率**: 与控制循环频率相同（通常为100Hz或更高）

**功能**: 实时发布当前是否处于RL控制模式

**消息内容**:
- `data` (float64): 
  - `1.0`: 当前处于RL控制模式
  - `0.0`: 当前处于MPC控制模式

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

**消息内容**:
- `data` (float64): MPC重置状态码
  - `0` (`NOMAL`): 正常状态，MPC正常运行
  - `1` (`RESET_INITIAL_POLICY`): 重置MPC状态1，等待初始策略
  - `2` (`RESET_BASE`): 重置MPC状态2，更新躯干位置（插值阶段）

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
   - 从RL切换到MPC时，RL控制器必须在stance状态
   - 从MPC切换到RL时，MPC必须在stance状态（倒地起身控制器除外）
   - 倒地起身控制器在未完成起身任务前，不允许切换到其他控制器

2. **控制器状态**:
   - `INITIALIZING`: 控制器正在初始化，不能执行操作
   - `RUNNING`: 控制器正在运行，可以执行控制
   - `PAUSED`: 控制器已暂停，推理线程继续运行但不执行控制
   - `STOPPED`: 控制器已停止，推理线程已退出

3. **服务命名空间**:
   - 控制器管理服务：`/humanoid_controller/*`（包括控制器切换、查询、倒地状态设置）
   - 控制器基础服务：`/humanoid_controllers/{controller_name}/*`（每个RL控制器的独立服务）
   - 倒地起身服务：`/humanoid_controller/trigger_fall_stand_up`

---


