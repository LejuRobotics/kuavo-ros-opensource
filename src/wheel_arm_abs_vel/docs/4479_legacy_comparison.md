# #4479 原绝对式与增量式对照及移植记录

日期：2026-10-21。以下为 yyb 开发机当前源码核查和离线软件验证，不代表实机通过。

## 对照基线

| 用途 | yyb 工作区 | HEAD |
| --- | --- | --- |
| 原独立绝对式 | `/home/yyb/research/absolute-guide-full-validation-20261021/kuavo-ros-control-absolute` | `debf39d63` |
| 原测试分支，含旧绝对式与 C++ 增量式路径 | `/home/yyb/workspace/kuavo-control-test-xb-preserve-mapping-20261020` | `7de8257a` |
| 本轮修改 | `/home/yyb/workspace/kuavo-abs-controls-issues-20261021` | 从 `b14c44ea4` 继续修改 |

前两处仅只读核查。原测试分支 `launch_quest3_ik.launch:135/218/328` 按 `use_cpp_incremental_ik` 选择 Python 旧绝对式或 `wheel_ik_ros_uni_cpp_node`，轮臂辅助 FSM 两者共用。不能只比较同名映射模块。

## 可以复用什么

| 功能 | 原实现与证据 | #4479 处理 |
| --- | --- | --- |
| Quest 躯干组合 | 原测试分支 `QuestControlFSMNode.cpp:1577` 和原独立式 `QuestAbsoluteControlFSMNode.cpp:1710` 均在触摸 X+Y 时检测 B 从未按到按下。 | 复用 B 按下沿语义。修正上一版使用组合形成沿的问题；B 已按住时补触摸或松开 X 都不再触发。 |
| 开启时采集零点 | 上述分支开启时记录 body pitch/yaw/height/x，后续 `headBodyPoseCallback` 使用相对值。 | 保留独立式求解器的机器人参考与人体姿态重新捕获，防止追赶关闭期间的积累位移；不复制旧位置发布器。 |
| Pico RT+Y | 原 `pico.py:765–784` 把 RT_PRESSED+Y_PRESSED 接到 `ToggleSwitch`，再调用 `set_control_torso_mode`。 | 保留同一用户按键；接入独立式同帧权限消息及现有末端组合仲裁。 |
| Pico 末端冲突 | 原 `pico.py:795–798` 的实际冻结条件为 `(callback_triggered and has_rg_lg_keys) or toggle_freeze_finger`。RT+Y 不含 RG/LG，仅复制组合回调不能保证夹爪不动。 | 保留 Y 组合消费、RT 输出保持、松开后新一次独立 RT 恢复接管。RT 先于 Y 的单独动作仍属于夹爪操作。 |
| 折叠臂保持 | 原独立式 `pinocchio_velocity_ik_node.cpp:6156/6236` 的求解和监督下肢 recruitment 为 1；旧 FSM 的布尔开关没有直接约束这一求解器。 | 保留已加入的显式权限、关闭时下肢约束及 PREPARE 保持。只移植按键不足以修复。 |
| X+B 与躯干独立 | 原 FSM 的 torso 标志和手臂模式分开；增量式 `WheelQuest3IkIncrementalROS.cpp:272` 在 arm mode 0/1 停止手臂求解。原独立式 `_fixed_arm_operation(0)` 则结束整个会话并请求底层 HOLD。 | #4481 仍未完成。后续要拆分手臂暂停与全身执行权限，不能只保留 torso 布尔值。本轮不宣称旧分支已通过这项实机验收。 |

源文件相对位置：

- 旧启动入口：`src/manipulation_nodes/noitom_hi5_hand_udp_python/launch/launch_quest3_ik.launch`
- 旧共享 FSM：`src/humanoid-control/humanoid_interface_ros/src/QuestControlFSMNode.cpp`
- 旧增量式：`src/manipulation_nodes/motion_capture_ik/src/WheelQuest3IkIncrementalROS.cpp`
- Pico：`src/manipulation_nodes/pico-body-tracking-server/scripts/core/ros/pico.py`
- 独立式 FSM、Mapper、求解器：`src/wheel_arm_abs_vel/src/QuestAbsoluteControlFSMNode.cpp`、`scripts/ik_ros_velocity.py`、`src/pinocchio_velocity_ik_node.cpp`。

## 本轮实际变更

生产修改仅在 `scripts/tools/torso_controls.py`：逐帧记录原始 B 状态，Quest 使用 B 按下沿，Pico 保持组合按下沿。保留原始输入不被末端仲裁改写、故障/过期门控及必须释放后重新操作的规则。没有更改 IK 算法、控制参数或其他问题的操作机制。

增加三个误切换序列回归，并覆盖保持 X+Y 触摸、反复松开再按 B 可以正常切换的操作。同步 `pico_quest_controls.md`，明确 #4481 尚未实现。

## 回归证据

证据目录：目标工作区 `output/4479-legacy-review/`。

- 上一版 `b14c44ea4` 的生产策略复现 3 项误切换；当前 15 项按键、末端仲裁及生产 Mapper 门控测试通过。
- CTest 定向 7 组通过：torso_controls、absolute_control_mechanism、fixed_follow_operation、pico_input_parity、absolute_end_effector、wheel_base_command、velocity_ik_base_command。
- S62、S63、400062 原生生产 callback、publishOnce、QP 回归全部通过，各 2552 周期。结果见 `torso-s62.log`、`torso-s63.log`、`torso-s400062.log`。场景包含双臂跟随而折叠臂保持、显式开启并重采零点、关闭后减速保持、PREPARE 保持、旧代次拒收和关节边界约束；锁定阶段最大折叠臂位移不超过 1.6e-19 rad（数值舍入量级）。

ROS 回归仅运行于 `kuavo-4479-regression-20261021` 的 `network=none` 隔离容器，使用模型及理想反馈。原生 C++ 本轮未改，复用上一轮已构建的测试程序。本轮未进行实机或可视仿真验收。
