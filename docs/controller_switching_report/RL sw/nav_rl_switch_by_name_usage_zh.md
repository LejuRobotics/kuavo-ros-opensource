# 外部导航按名字切换 RL 控制器接口说明

## 1. 接口目标

新增一个给外部导航使用的接口，用来按控制器名字触发 `RL -> RL` 切换。

这个接口的语义等价于遥控器的 `X`：

- 都是触发当前控制器切换
- 都复用 `RLControllerManager` 里现有的切换判据
- 都遵守当前代码中的站立/行走切换逻辑

不同点是：

- 遥控器 `X` 是按顺序切换
- 这个新接口是按名字切换到指定的 RL 控制器

## 2. 话题信息

话题名：

```text
/humanoid_controller/nav_switch_rl_controller_by_name
```

消息类型：

```text
std_msgs/String
```

消息内容：

- `data` 填目标 RL 控制器名字

例如：

- `amp_controller`
- `depth_loco_controller`

## 3. 使用限制

这个接口只允许 `RL -> RL` 切换。

也就是说：

- 当前控制器必须已经是一个 RL 控制器
- 目标控制器也必须是一个 RL 控制器

下面这些请求会被拒绝：

- 当前在 `mpc`，却通过这个接口要求切换
- `data` 为空
- `data` 指向不存在的控制器
- `data` 指向 `mpc`
- 当前切换时机不满足现有切换逻辑

## 4. 切换逻辑

该接口不会自己实现一套新的切换逻辑，而是直接复用当前 `RLControllerManager::switchController(name)`。

因此：

- 站立状态下，按当前站立切换逻辑执行
- 行走状态下，按当前行走切换逻辑执行
- 如果当前代码允许 `AMP <-> depth_loco` 在 walking 下切换，就按该逻辑执行
- 如果当前代码判断当前时机不允许切换，就会拒绝

另外，这个接口也复用了当前已有的躯干稳定性检查和 walking 特例放宽逻辑。

## 5. 调用示例

### 5.1 切到 depth 控制器

```bash
rostopic pub -1 /humanoid_controller/nav_switch_rl_controller_by_name std_msgs/String "data: 'depth_loco_controller'"
```

### 5.2 切到 AMP 控制器

```bash
rostopic pub -1 /humanoid_controller/nav_switch_rl_controller_by_name std_msgs/String "data: 'amp_controller'"
```

## 6. 推荐使用方式

推荐导航侧这样用：

1. 先确认当前控制器已经是 RL 控制器
2. 再根据地形或任务需要决定目标控制器名字
3. 发布一条 `std_msgs/String`
4. 监听控制器侧日志，确认是否切换成功

如果后续需要更强的闭环，可以继续增加一个专门的“切换结果反馈”话题。

## 7. 与现有服务的区别

现有服务：

```text
/humanoid_controller/switch_controller
```

也支持按名字切换控制器，但它不是导航专用接口，而且不限制必须是 `RL -> RL`。

新接口：

```text
/humanoid_controller/nav_switch_rl_controller_by_name
```

特点是：

- 明确给导航使用
- 只允许 `RL -> RL`
- 更接近“遥控器 X 的按名字版本”

## 8. 代码位置

本次接口实现位于：

- `src/humanoid-control/humanoid_controllers/include/humanoid_controllers/rl/RLControllerManager.h`
- `src/humanoid-control/humanoid_controllers/src/rl/RLControllerManager.cpp`

主要改动包括：

- 新增导航切换话题订阅
- 新增按名字处理公共函数
- 现有服务与新话题都复用同一套按名字切换处理逻辑
