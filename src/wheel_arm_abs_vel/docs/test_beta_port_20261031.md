# test 功能迁入 beta：绝对式限定版（2026-10-31）

MR !4194 以 beta `2e5f9d821` 为基线。复核后按用户要求收紧：最终净差异仅允许出现在 `src/wheel_arm_abs_vel/`。早期提交包含的共享硬件、增量控制器、公共头部节点和公共录包配置改动已通过追加提交撤回；旧提交保留历史，不应按旧 MR 描述判断最终范围。复核细节见 [MR 复核记录](mr4194_review_20261031.md)。

## 当前操作

- 同一 Mapper 进程首次手柄跟随只能用双前扳机；IK 和执行端确认 ACTIVE 后才算成功，失败后仍需松开双前扳机再重试。X+A 和其他普通模式操作在首次成功前均被拒绝。
- 后续重新启动用 X+A，双前扳机不再启动。X+B 只保持/续接已有 ACTIVE 会话，不创建新会话。停止手势和底盘独立控制保留。
- 旧 `/quest3/triger_arm_mode` 的模式 1/2 不再提供绝对式准备/恢复入口；模式 0 停止路径保留。控制使能、状态反馈和原始 running 标志均不能独立启动。
- Quest 裸手启动仍需原有 50 个有效连续双 OK 帧；授权绑定当前代次且只消费一次，随后仍检查输入、关节反馈和执行权。首次手柄请求等待期间切到裸手会取消该请求。
- `control_torso:=true` 默认在首次有效跟随开启躯干；false 初始保持躯干，随后可手动开启。Quest 右摇杆下拉仅保留 yaw、上推全部解锁；X+右摇杆下拉全部锁定、上推全部解锁。两种上推均清除位置锁定，单独下拉从全锁状态也可进入仅 yaw。超过 0.8 且连续超过 100 个新样本后生效，需回中后重新触发；触摸 X+Y 后按 B 的躯干入口已删除。Pico 原有操作保留。准备轨迹仍可独立执行。

## 保留的绝对式增强

| 功能 | 私有实现 |
| --- | --- |
| 输入与 QP | 双手 TCP native Kalman、胸相对肘 40 ms EMA、同源预测、clean10 默认 10/8 Hz、独立 QP 验收；保留硬限位、SHM、执行握手和传感器新鲜度保护。旧 skeleton/flattening 实验声明仅作 launch 兼容。 |
| 躯干 | 两级摇杆、操作序号、位置重新锚定和当日统一 control_torso。 |
| 录包 | 保留原消息结构和 record-only 标记，新增输出只发往 `/whole_body_velocity_ik/record/` 下的 `kuavo_arm_traj`、`lb_leg_traj`、`drake_ik/eef_pose`、`drake_ik/input_pos`；私有录包脚本显式订阅，避免公共轨迹被增量接收端执行。 |
| 模型与运动限制 | 私有 wheel_arm_gains、S6 限制及完整模型契约；共享资产只读，保留 beta 私有限位交集和 S63 路径。取消白名单不等于全部型号实机适配。 |
| 诊断 | 保留 beta 已有后台参数线程与亲和性，启动时预分配耗时发布器/消息，增加阶段/查询耗时和动力学失败信息；不放宽 dt/测量过期保护。 |
| 头跟手 | 私有 absolute_head_control_node 继承 beta 公共节点，仅替换活动量估计和偏置算法；fixed/vr_follow/track_hand 生命周期、服务、超时与头部安全限位继续来自 beta。只改绝对式 launch。 |

CAN/Leju 驱动、SG100 初始化/编码/增益透传和公共可视化修复已退出本 MR，全部恢复 beta 原样。本 MR 不修改增量脚本、共享驱动、公共头控、公共增益或公共录包列表。19 现场配置、SDK、编译产物未迁入。

## 构建与验证

新增 `pybind11-dev` 构建依赖。CMake 在目标平台生成 native 模块；需重新编译私有消息/包并 source 工作区，仓库不含预编译 .so，缺少 native 模块时明确失败。

本轮在隔离 ROS Noetic 容器重新编译绝对式包（含 Mapper native、IK、FSM、私有控制器及仿真可执行文件，仅构建不运行仿真）。76 组 CTest 全通过，覆盖 beta 既有流程以及新增绕过入口回归。四模型生产录制路径另做隔离 ROS master 联调，校验命名空间、FK、非零 TCP 偏置、单位/顺序和无效目标抑制。

没有部署或启动真实机器人，源码/离线回归不等于实机验收。

```bash
cmake --build <wheel_arm_abs_vel-build-dir> --target velocity_ik_absolute_record_test -j2
python3 src/wheel_arm_abs_vel/test/run_absolute_record_test.py <devel>/lib/wheel_arm_abs_vel/velocity_ik_absolute_record_test
```
