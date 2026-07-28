# MoRE 控制器设计文档

## 目标与范围

MoRE 控制器是 `humanoid_controllers` 中基于 `RLControllerBase` 的一个强化学习控制器实现，控制器类型为 `MORE_CONTROLLER`。它面向 MoRE（Mixture-of-Residual-Experts）策略部署，负责从机器人状态、速度命令、姿态命令和运动风格门控中构造策略观测，通过 OpenVINO 执行 ONNX 推理，并将策略动作转换为腿部、腰部和手臂的关节命令。

当前实现主要文件如下：

- `src/humanoid-control/humanoid_controllers/include/humanoid_controllers/rl/MoREController.h`
- `src/humanoid-control/humanoid_controllers/src/rl/MoREController.cpp`
- `src/humanoid-control/humanoid_controllers/config/kuavo_v*/MoRE/more_rl_param.info`
- `src/humanoid-control/humanoid_controllers/config/kuavo_v*/rl_controllers.yaml`

本文档描述当前代码实现，不定义新的控制接口。

## 控制器接入

MoRE 通过现有 RL 控制器管理框架加载：

1. `rl_controller_types.h` 中增加 `RLControllerType::MORE_CONTROLLER`。
2. `RLControllerManager.cpp` 根据 yaml 中的 `type: "MORE_CONTROLLER"` 构造 `MoREController`。
3. `CMakeLists.txt` 将 `src/rl/MoREController.cpp` 编入 `humanoid_controllers`。
4. 各机器人版本在 `rl_controllers.yaml` 中配置控制器名称、类型和 info 文件路径。

典型 yaml 配置如下：

```yaml
- name: "more_controller"
  class: "BASE_CONTROLLER"
  type: "MORE_CONTROLLER"
  config_file: "MoRE/more_rl_param.info"
  enabled: true
```

是否实际启用由各版本 `rl_controllers.yaml` 决定。代码支持加载 MoRE，但如果 yaml 中该段被注释或 `enabled: false`，控制器不会进入运行列表。

## 初始化流程

`MoREController::initialize()` 完成以下工作：

1. 从 ROS 参数 `/wbc_frequency` 读取 WBC 主循环频率，并计算控制周期 `dt_ = 1 / wbc_frequency`。
2. 加载 RL IMU 滤波参数和 `more_rl_param.info`。
3. 创建 `RlGaitReceiver`，用于接收速度、站立/行走状态和 `/cmd_pose` 姿态命令。
4. 读取 `/is_real`、`/is_roban`、`/ankle_solver_type` 等部署参数。
5. 初始化 ankle solver，用于关节/电机空间转换。
6. 根据 `/urdfFile` 或 `/robot_version` 初始化外部手臂控制器。
7. 初始化外部腰部控制器。
8. 注册 `/humanoid_controller/change_more_mode` 服务。
9. 在 `gaitStyleMode=command` 时订阅 motion style 命令 topic。
10. 订阅 `/quest_joystick_data`，支持 VR 按键切换到 MoRE 和切换上身控制模式。

## 关键配置项

MoRE 的主要行为由 `more_rl_param.info` 驱动。

### 策略时序

- `inferenceFrequency`：策略推理频率，单位 Hz。
- `policyControlStepDt`：可选参数，显式配置策略控制步长。

当前实现优先根据 `inferenceFrequency` 推导：

```text
policy_control_step_dt_ = 1.0 / inferenceFrequency
```

如果 info 中存在 `policyControlStepDt`，则使用该值覆盖推导结果。代码会检查 `inferenceFrequency` 与 `1 / policyControlStepDt` 是否明显不一致，并打印告警。

主控制周期 `dt_` 来自 `/wbc_frequency`，策略推理节拍由 `RLControllerBase::shouldRunInference()` 结合 `inference_frequency_` 控制。也就是说，WBC 可以 500 Hz 运行，策略可以按 100 Hz 更新动作。

### ONNX 与观测维度

- `networkModelFile`：ONNX 模型文件名，运行时拼接 ROS 参数 `/network_model_file` 得到完整路径。
- `numGait`：motion style 权重维度。
- `numSingleObs`：策略 body observation 维度，不含 gait mixture 前缀。
- `obsHistoryLen`：历史观测帧数。
- `hasObsHistoryEncoder`：ONNX 是否包含独立 `history` 输入。
- `onnxActionsOnly`：标记导出模型是否只输出 actions。

当前 ONNX 输入支持两种形态：

1. 单输入模型：
   - `observations`: `[numGait + policy_body_dim]`
   - `hasObsHistoryEncoder=false`
2. 双输入模型：
   - `observations`: `[numGait + policy_body_dim]`
   - `history`: `[obsHistoryLen, policy_body_dim]`
   - `hasObsHistoryEncoder=true`

加载模型后，代码会根据 ONNX 实际输入数量和配置决定是否启用 history 输入。如果配置为 true 但模型只有一个输入，会自动降级为 false 并打印告警。

## 运行时数据流

每个控制周期的主流程如下：

1. `RLControllerBase` 获取并预处理传感器数据。
2. 按推理频率调用 `updateObservation()` 构造策略观测。
3. 调用 `inference()` 执行 OpenVINO 推理并更新当前 action。
4. 每个 WBC 周期调用 `updateImpl()`。
5. `updateImpl()` 中先更新 `RlGaitReceiver`，再调用 `updateRLcmd()` 将当前 action 转为 actuation。
6. `actionToJointCmd()` 将 actuation 写入 `kuavo_msgs::jointCmd`。
7. 基类后续可调用 `updateArmCommand()` 和 `updateWaistCommand()`，由外部手臂/腰部控制器覆盖对应关节命令。

推理不一定每个 WBC 周期执行，但关节命令每个 WBC 周期都会用当前 action 刷新。

## 观测设计

MoRE 的 ONNX `observations` 由两部分拼接：

```text
policy_obs = [motion_style_weights, body_obs]
```

其中：

- `motion_style_weights` 是 `numGait` 维 one-hot。
- `body_obs` 的字段和顺序由 info 中 `singleInputData` 定义。

当前代码支持的 body observation 来源包括：

- `velocity_commands`：`[vx, vy, wz]`
- `scalar_command`：`1 - cmdStance`
- `posture_commands`：下蹲高度增量和弯腰/躯干俯仰增量
- `bodyAngVel`
- `projected_gravity` 或 `gravity_body`
- `joint_pos`
- `joint_vel`
- `actions`

`singleInputData` 中每个字段包含：

- `startIdx`：从来源向量的哪个位置开始取
- `numIdx`：取多少维
- `obsScales`：观测缩放系数

这样可以让部署侧观测顺序严格跟训练侧 actor observation 对齐。

## History 缓冲

MoRE 使用独立 history 缓冲，不依赖通用 `frameStack`。`frameStack` 在 info 中保留为 1。

history 的内容是 body observation，不包含前置的 `motion_style_weights`。每次构造观测时，代码会将当前 body observation 打包到 history tensor 的最后一帧；推理成功后调用 `commitHistoryFrame()` 滚动历史帧。

这个顺序与训练环境中“用当前 body obs 参与本次 act，推理成功后写入下一拍 history”的节拍保持一致。

## Motion Style 设计

MoRE 通过 `motion_style_weights` 选择不同专家或行为风格。三风格配置下的语义为：

```text
index 0 -> [1, 0, 0] -> pose
index 1 -> [0, 1, 0] -> walk + policy arm
index 2 -> [0, 0, 1] -> walk + external arm
```

二风格配置下：

```text
index 0 -> pose
index 1 -> walk
```

风格选择由 `gaitStyleMode` 决定：

- `command`：订阅 `moreStyleCommandTopic`，使用外部 `std_msgs/Int32` 指令选择风格。
- `auto`：根据速度命令和外部手臂命令自动选择。
- `fixed` 或其他值：保持 `defaultGaitStyleIndex`。

在 `command` 模式下，如果外部命令仍停留在 pose 风格，但收到非零速度命令，代码会自动切到 walk 风格，避免“有速度但仍使用 pose expert”。

## 姿态命令

`posture_commands` 来自 `RlGaitReceiver`，当前用于 `/cmd_pose` 注入：

```text
posture_commands[0] -> 下蹲高度增量，单位 m
posture_commands[1] -> 弯腰/躯干俯仰增量，单位 rad
```

限幅由 `postureCommandLimits` 控制。`postureNonzeroGaitStyleIndex` 可将姿态命令限制为仅在某个 gait style 下透传；例如配置为 0 时，只在 pose 风格下给策略非零姿态命令，walk 风格下置零。

## 推理设计

`inference()` 每次创建 OpenVINO infer request，设置输入 tensor 后异步启动并等待完成。

推理前会做维度校验：

- `observations` 长度必须等于 ONNX 第一个输入 shape 的元素数。
- 如果启用 history encoder，`history_tensor_` 长度必须等于 ONNX 第二个输入 shape 的元素数。
- 输出长度必须等于期望 action 数：
  - `withArm=true`：`jointNum + jointArmNum + waistNum`
  - `withArm=false`：`jointNum + waistNum`

维度不匹配时，控制器会限频打印错误，返回零 action，并保持系统不因异常直接崩溃。

## Action 到关节命令

策略输出 action 表示相对于默认关节位置的归一化偏移。部署侧转换公式核心为：

```text
target_q = action * actionScale * actionScaleTest + defaultJointState
```

`updateRLcmd()` 根据传感器关节状态、PD 参数和控制模式生成 actuation：

- 非实物仿真中，`JointControlModeRL=0` 时输出 PD 力矩。
- 非实物仿真中，`JointControlModeRL=2` 时输出包含电机空间阻尼和策略位置误差项的 CSP 力矩。
- 实物中根据 `JointControlModeRL` 和 `JointPDModeRL` 分别输出力矩或位置命令。
- 所有力矩输出会受 `torqueLimits` 限制。

对于 CSP 模式，当前实现会把策略 PD 力矩叠加到电机阻尼力矩上，避免仅有阻尼导致控制力度不足。

如果 `use_jointcmd_filter=true`，actuation 会经过二阶低通滤波器。滤波器是否作用到某个关节由 `jointCmdFilterState` 决定。

## 上身控制与外部接管

MoRE 支持策略输出手臂/腰部，也支持由外部控制器覆盖上身命令。

相关配置：

- `use_external_arm_controller`
- `use_external_waist_controller`
- `armVelocityLimit`
- `waistControllerParam`

手臂和腰部模式与 gait style 联动：

- pose 风格下，默认允许 VR 或服务切换到外部控制。
- walk + policy arm 风格下，手臂使用策略输出。
- walk + external arm 风格下，手臂使用外部控制器。

`/humanoid_controller/change_more_mode` 服务可设置内部 `more_mode_`，用于影响上身控制模式。VR 控制中：

- 右手 B：请求切换到 `more_controller`。
- 左手 X + 右手 A：切换 pose 风格下手臂/腰部模式。

如果外部控制器处于策略模式，`updateArmCommand()` 或 `updateWaistCommand()` 返回 false，保留策略输出；如果外部控制器接管，则返回 true，并覆盖 `joint_cmd` 中对应关节。

### 离线手臂动作播放

MoRE 提供 `/humanoid_controller/more_execute_arm_action` 服务，用于播放离线手臂 .tact 动作。由于动作播放在独立 Python 节点中完成，MoRE 服务采用非阻塞计时器方案：仅做模式切换后立即返回，由系统 `/execute_arm_action` 完成轨迹播放，避免阻塞主控制循环。

各 style 的播放行为：

| 风格 | 动作前状态 | 行为 |
|------|----------|------|
| style 0（pose） | arm mode 1（标称） | 手臂锁定当前位 → 切 mode 2 → 播放 → 超时恢复 mode 1 |
| style 0（pose） | arm mode 2（外部） | 直接播放，不恢复 |
| style 1（walk_policy） | arm mode 1（策略摆臂） | 自动切 style 2 → 摆臂平滑过渡到动作 → 播放 → 超时恢复 style 1，**行走不中断** |
| style 2（walk_external） | arm mode 2（外部） | 直接播放，不恢复，**行走不中断** |

走不停腿通过三层旁路实现：`RlGaitReceiver` 动作期间不压制速度命令、`humanoidController` 跳过外部手臂位置保护、Python 服务对 MoRE 不强制进入 stance。动作结束后统一通过超时（15 s）恢复原模式和行走标志。

播放流程分两步：先调 `more_execute_arm_action` 完成模式准备，再调 `/execute_arm_action` 执行轨迹。此服务与 AMP 的 `/execute_arm_action` 调用方式保持对齐，不影响 AMP 功能。

## 机器人版本与关节顺序

代码对 `is_roban_` 做了特殊处理。普通机器人和 roban 在腰部、腿部、手臂的关节排列上不同，因此：

- `preprocessSensorData()` 会在 roban 下调整传感器关节顺序。
- `actionToJointCmd()` 会在 roban 下把腰部关节命令移回硬件期望顺序。
- 手臂、腰部默认位置和 kp/kd 取值也会根据 `is_roban_` 使用不同 segment。

部署时必须确认 `/is_roban` 与模型训练使用的关节顺序一致。

## 退出与安全行为

`requestToExit()` 使用当前机器人状态中的 roll 和 pitch 做跌倒判断：

```text
abs(roll) > 60 deg 或 abs(pitch) > 60 deg
```

触发后请求退出当前控制器。`isAllowToExit()` 要求 gait receiver 当前处于站立状态，避免在不合适的运动阶段直接切换。

推理、观测维度、输出维度异常时不会抛出到主循环，而是打印限频错误并返回零 action。

## 部署检查要点

1. 确认 `rl_controllers.yaml` 中 `more_controller` 已启用。
2. 确认 `networkModelFile` 指向的 ONNX 与 `/network_model_file` 拼接后的路径存在。
3. 确认 `hasObsHistoryEncoder` 与 ONNX 输入数量匹配。
4. 确认 `numGait + numSingleObs` 等于 ONNX `observations` 输入元素数。
5. 如果 ONNX 有 history 输入，确认 `obsHistoryLen * numSingleObs` 等于 ONNX `history` 输入元素数。
6. 确认 ONNX 输出维度与 `withArm`、`jointNum`、`jointArmNum`、`waistNum` 对齐。
7. 确认 `inferenceFrequency` 与训练环境 step 频率一致。
8. 确认 `/wbc_frequency` 与实际 WBC 主循环一致。
9. 确认 `JointControlMode`、`JointPDMode`、`jointKp`、`jointKd`、`torqueLimits` 与仿真或实物执行器模式一致。
10. 确认 `singleInputData` 顺序与训练侧 actor observation 完全一致。

## 常见问题

### 一运行就倒或动作明显异常

优先检查 ONNX 输入输出维度、`singleInputData` 顺序、`actionScale`、`actionScaleTest` 和默认关节位置。MoRE 对观测顺序敏感，维度正确但字段顺序错误也会导致策略输出异常。

### 仿真变慢

先确认 `inferenceFrequency` 是否过高、ONNX 是否在 CPU 上推理耗时过大，以及是否启用了过多日志。WBC 主循环频率来自 `/wbc_frequency`，策略动作更新频率来自 info 文件的 `inferenceFrequency`。

### 有速度命令但不走

检查 `gaitStyleMode` 和 motion style。pose 风格不应承载行走命令。建议部署阶段使用 `gaitStyleMode=auto`，或确保 command topic 在行走时切到 walk 风格。

### 上身不按预期动作

检查 `use_external_arm_controller`、`use_external_waist_controller`、当前 motion style、`more_mode_` 和 VR/服务切换状态。walk + policy arm 会保留策略手臂输出，walk + external arm 会优先外部手臂控制。

