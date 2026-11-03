# RL 控制器自动切换设计说明

## 目标

在 multi RL 控制器模式下，根据当前任务意图自动选择合适的 RL 控制器：

- 外部手臂或腰部控制时，使用 `amp_controller`。
- 行走速度或步态请求时，使用 `amp_wild_controller`。
- `amp_wild_controller` 不执行外部手臂和腰部控制，避免模型收到未训练过的上肢/腰部动作后失稳。

该功能设计为通用机制，不和具体机型硬编码绑定。只要对应机型加载的是 multi RL 控制器框架，并在某个已加载的 RL info 中正确配置 `autoControllerSwitch`，即可使用。目前先在 v55 的 `skw_rl_param.info` 中打开做验证。

## 配置入口

配置位于 RL info 文件的 `autoControllerSwitch` 块，例如：

```ini
autoControllerSwitch
{
    enabled                     true
    manipulationController       amp_controller
    walkingController            amp_wild_controller
    cmdVelLinearThreshold        0.05
    cmdVelAngularThreshold       0.05
    cmdVelCommandHoldTime        0.5
    externalCommandHoldTime      0.5
    minSwitchInterval            1.0
    walkingSwitchRequireStance  false
    switchCommandBufferTime     0.5
}
```

参数含义：

- `enabled`：是否启用自动切换。未配置或为 `false` 时，不改变原有控制器切换行为。
- `manipulationController`：允许执行外部手臂/腰部控制的控制器。
- `walkingController`：允许执行行走速度指令的控制器。
- `cmdVelLinearThreshold` / `cmdVelAngularThreshold`：速度指令超过阈值时认为有行走意图。
- `cmdVelCommandHoldTime`：速度/步态意图的有效保持时间。
- `externalCommandHoldTime`：外部手臂/腰部控制意图的有效保持时间。
- `minSwitchInterval`：自动切换最小尝试间隔，避免频繁切换。
- `walkingSwitchRequireStance`：切到行走控制器时是否要求 `SwitchMotionState::STANCE`。v55 允许站立到行走初期不强制 STANCE，以覆盖线速度刚发出时 motion state 已变为 WALKING 的情况。
- `switchCommandBufferTime`：切换发起后的指令缓冲时间。缓冲期间目标指令先缓存，不直接送给不匹配的控制器执行。

## 触发源

自动切换由 `RLControllerManager` 统一判断，主要输入如下：

- `/cmd_vel`：线速度或 yaw 角速度超过阈值，触发行走控制器切换。
- `/humanoid_mpc_gait_name_request`：`walk` / `trot` 触发行走控制器切换。
- `/humanoid/mpc/arm_control_mode`：当前或期望手臂模式为 2，触发外部控制器切换。
- `/robot_waist_motion_data` 与 `/humanoid_controller/enable_waist_control`：触发外部控制器切换。
- `/robot_action_state`：tact 动作播放期间屏蔽行走自动切换意图，避免手柄持续 `/cmd_vel` 干扰动作播放。

## 优先级

自动切换优先级为：

1. 外部手臂/腰部控制优先。
2. 外部手臂目标偏离默认位时，阻止行走速度指令执行。
3. 无外部控制需求时，行走速度或步态请求切到行走控制器。

这样可以保证站立操作上肢/腰部时不会因为速度指令误进入 `amp_wild_controller`，也能避免 `amp_wild_controller` 执行上肢/腰部外部控制。

## 指令缓冲

切换过程中不能让指令先进入错误模型执行，因此有两类缓冲：

- 行走指令缓冲：`RlGaitReceiver` 在切到 `walkingController` 前不把 `/cmd_vel` 送入 RL 推理；`getPolicyCommand()` 会输出零命令。
- 外部控制缓冲：`ArmController` 和 `WaistController` 在切到 `manipulationController` 前缓存最新外部目标，等控制器切换完成后再应用。

缓冲时间由 `switchCommandBufferTime` 控制。控制器最终是否允许执行由 `RLControllerManager::isWalkingCommandExecutionAllowed()` 和 `isExternalControlCommandExecutionAllowed()` 判断。

## tact 动作播放保护

tact 动作需要上肢和腰部控制，必须在 `amp_controller` 下执行。动作播放流程：

1. 先检查动作文件、机型和版本兼容性。
2. 发布 `/robot_action_state state=1`，进入动作保护窗口。
3. 如果正在行走，先发布零速度和 `stance` 请求，等待停止。
4. multi 模式下确认当前控制器为 `amp_controller`；如果不是，则调用 `/humanoid_controller/switch_controller` 请求切到 `amp_controller`。
5. 确认控制器正确后，再请求手臂模式 2。
6. 任一步失败时发布 `/robot_action_state state=0`，不继续播放动作。

`RLControllerManager` 和 `RlGaitReceiver` 都订阅 `/robot_action_state`：

- `RlGaitReceiver` 在动作 active 时屏蔽 `/cmd_vel` 执行。
- `RLControllerManager` 在动作 active 时忽略 `/cmd_vel` 产生的行走自动切换意图。

为了避免漏发 `state=0/2` 导致永久屏蔽，动作 active 状态带有超时保护，参数为：

```text
/rl_gait_receiver/robot_action_active_timeout
```

默认 0.5s，最小 0.1s。

## 与现有切换接口的关系

自动切换不新增新的控制器切换消息，复用现有接口和逻辑：

- 控制器查询：`/humanoid_controller/get_controller_list`
- 指定控制器切换：`/humanoid_controller/switch_controller`
- RL->RL 切换内部仍走 `RLControllerManager::switchController()` 与既有保护逻辑。

另外保留外部导航按名字切换接口：

```text
/humanoid_controller/nav_switch_rl_controller_by_name
```

消息类型为 `std_msgs/String`，`data` 填目标 RL 控制器名，例如：

```bash
rostopic pub -1 /humanoid_controller/nav_switch_rl_controller_by_name std_msgs/String "data: 'amp_controller'"
```

该接口只允许 `RL -> RL` 切换：

- 当前控制器必须已经是 RL 控制器。
- 目标控制器必须存在，且不能是 `mpc`。
- 切换仍复用 `RLControllerManager::switchController(name)`，因此遵守同一套站立/行走、躯干稳定性和 walking phase guard 保护。

自动切换不依赖该导航接口；二者只是共享底层按名字切换逻辑。

## 兼容性边界

- 未配置 `autoControllerSwitch` 或 `enabled=false` 的机型，保持原行为。
- `manipulationController` 和 `walkingController` 都必须是已加载的 RL 控制器。
- 配置所在 info 必须能被当前机型的 `rl_controllers.yaml` 加载。当前实现读取控制器配置文件时解析 `autoControllerSwitch`，因此建议放在主要的 `amp_controller` info 中，避免只加载 wild 模型时漏配。
- 当前仍依赖现有 `SwitchMotionState`、torso stability 和 RL->RL phase guard。自动切换不会绕过底层切换失败保护。
- 模型文件、关节默认位、Kp/Kd 等模型调参不属于自动切换机制本身，应尽量与此功能分开提交和回归。

## 建议测试场景

1. `amp_controller` 站立时给线速度，确认先切到 `amp_wild_controller`，切换完成前 RL 推理命令保持站立。
2. `amp_controller` 站立时给 yaw 角速度，确认同样能切到 `amp_wild_controller`。
3. `amp_wild_controller` 站立时启动外部手臂或腰部控制，确认先切到 `amp_controller`，再执行外部目标。
4. 外部手臂目标偏离默认位时给 `/cmd_vel`，确认速度指令被屏蔽，不进入行走。
5. `amp_wild_controller` 行走时执行 tact，确认先停步、切回 `amp_controller`，再播放动作。
6. tact active 期间持续发送 `/cmd_vel`，确认 `RLControllerManager` 日志出现 `Ignore autoSwitch cmd_vel while robot action is active`，不会切回行走控制器。
