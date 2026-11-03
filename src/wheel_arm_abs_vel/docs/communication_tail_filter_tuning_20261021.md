# 全身绝对速度 IK：通信长尾与滤波改造交付

## 1. 结论与验收状态

本次修改的是 `wheel_arm_abs_vel` 的全身绝对速度 IK，21 维参考布局、固定底盘时 18 个活动自由度，不是增量 IK。

代码改造、编译、56 项 CTest、28 个 Python 测试文件均通过；Python 文件包含 232 个 unittest 用例，另含生产代码块编译执行与启动配置脚本。实际 `publishFormalReference` 的 SHM 分支在 20,000 次提交中，镜像与非镜像相位均未检测到 C++ `new/new[]` 分配。18:32 留存目标片段的两个包络配置离线求解均通过。

**闭环 MuJoCo 仿真验收未通过，尚不能部署或宣称实机长尾已经消除。** 可视仿真中机器人在获得正式 owner 前已出现大幅关节运动，StartupReadiness 持续等待；调优和参数回滚两组均没有有效 ACTIVE 参考。没有绕过稳定性门、owner 或安全约束。这是未完成的验收项，不是合格的闭环跟踪结果。

本次未操作真实硬件、未改生产启动实例、未提交、推送或合并代码。

## 2. 交付位置和修改边界

- 主机：`yyb@192.168.5.183`，开发机 `yyb-B760M`；不是 `lab@192.168.5.19` 机器人。
- 工作树：`/home/yyb/workspace/kuavo-abs-tail-filter-20261021`。
- 分支：`codex/abs-tail-filter-20261021`。
- 基线 HEAD：`ac4761feb6772d9a3d2cc30430392e6f01c489c9`。
- 所有代码、配置、测试、本文档的差异均位于 `src/wheel_arm_abs_vel/**`。
- `hardware_plant`、`motion_capture_ik`、旧 `humanoid_controllers` 以及增量/人形共享源码均为 0 diff。
- 工作树根目录的 `output/tail-filter/` 是本轮生成的构建与验证证据，不属于代码补丁；原始 bag 和其他工作树未改动。
- `git add -N` 仅使新增包内文件出现在完整 diff 中；没有创建提交或暂存输出文件。

## 3. 通信与周期改造

### 3.1 WBC 参数查询移出实时循环

文件：`src/absolute/humanoidController_wheel_absolute.cpp` 及其包内头文件。

`init()` 读取 `/ik_ros_uni_cpp_node/quest3/lock_knee_leg` 初值；后台线程按 `/absolute_controller/parameter_poll_hz`（默认 10 Hz，允许 0–50 Hz 之间的正值）刷新 `std::atomic<bool>`。`update()` 中仅做 relaxed 原子读取。有效 bool/int 参数沿用原语义；缺失或类型不合法时保留上次快照。

后台线程显式降为 SCHED_OTHER，并恢复 init 在 WBC 绑核前保存的 CPU mask；析构通知并 join。参数服务器阻塞不再直接卡住 WBC 500 Hz 更新，但可能延迟参数生效及后台线程退出。没有用恢复同步 XMLRPC 的方式提供“回滚”；安全契约和 owner 判定没有改变。

### 3.2 正式 SHM 与镜像分离

文件：`src/pinocchio_velocity_ik_node.cpp`、`velocity_ik/async_diagnostics.h`、包内 `WholeBodyReferenceShmRos.h`。

构造时一次性编码 DOF 名称、坐标系、时钟域和实例等固定元数据。每拍仅复制固定大小 Payload，并填充时间、owner、q/dq/ddq 和边界。保持原 SHM seqlock 提交协议：ABI 大小 2920 字节、tag `0xabb458e7489e217a` 不变。

镜像相位且存在订阅者时，提交定长快照到容量 16 的 SPSC 队列。生产者不构造 std::function、不获取队列 mutex、不执行 CAS 重试；后台 worker 构造 ROS 消息和字符串数组。队列满只丢镜像，正式 SHM 已先提交。终止发布锁和 stop latch 保留，不能为了性能移除安全互斥。

此保证只覆盖正式 SHM 分支。显式选择旧 ROS transport 时仍会构造动态 ROS 消息；IK 其他诊断消息仍有动态分配。镜像相位的 ROS 订阅者查询、线程唤醒和终止锁也仍可能带来调度成本，不能据此声称整个 500 Hz 线程无锁、无分配或有硬实时上界。

### 3.3 五相诊断

以不随 session/reference 重置的单调 tick 统一计算相位；默认 stride=5。

| tick % 5 | 发布/构造内容 |
|---|---|
| 0 | formal mirror POD 入队 |
| 1 | QP stats、operating diagnostics |
| 2 | joint jerk diagnostics |
| 3 | governor diagnostics |
| 4 | 普通向量和参考诊断 |

QP 求解自身、governor 约束计算保持每拍执行，只降低诊断物化频率。audit stride=1 会有意恢复每拍诊断，不能同时要求五相分散。

### 3.4 Mapper 控制优先与旁路

文件：`scripts/ik_ros_velocity.py`、`scripts/tools/quest3_utils_velocity.py`、`scripts/tools/noncontrol_publish_process.py`。

双手/躯干几何完成后先发布 TeleopTcpTarget；将之前藏在几何更新中的胸/肩观察消息和辅助头身发布延后到目标发布或拒绝之后，再 flush marker。控制状态、手势和许可检查仍保留在必要位置。

诊断 IPC 改为 KDP2：父进程冻结原始字段为 tuple 并做 primitive marshal packing，子进程恢复 ROS 消息并执行 ROS wire serialization。父回调不再 BytesIO/ROS serialize。父进程仍有字段遍历、tuple 分配和 IPC 编码/GIL 消耗，不能称为“父进程零序列化”。保留非阻塞、有界、允许丢弃的诊断通道和消息白名单；控制命令不经该旁路。

### 3.5 调度

AAEON/AGX 的 Mapper 输入 CPU 都为 4；其实际 TCPROS 输入回调线程在首次进入及重连线程首次进入时应用 FIFO 35。仅在运行时分区通过拓扑验证、成功绑定单个专用 CPU 后启用。IK FIFO 改为 70，WBC 保持 90，sidecar 保持普通调度。

FIFO 权限失败自动尝试 nice -10；nice 权限也失败会记录实际失败，不能承诺普通用户一定能获得 -10。未验证 profile 时不启用 FIFO。设置 `mapper_fifo_priority: 0` 可禁用该策略；恢复旧调度可同时设置 IK=40、Mapper nice=-5。

## 4. 参数对照与决策

| 项目 | 原值 | 本次默认/候选值 | 配置和回滚 |
|---|---|---|---|
| WBC 参数查询 | RT 内约 10 Hz XMLRPC | 后台 10 Hz，RT atomic | `/absolute_controller/parameter_poll_hz` |
| IK/Mapper 调度 | IK FIFO40；Mapper nice -5 | IK70；Mapper FIFO35，失败尝试 nice -10 | 两份 runtime_partition JSON；Mapper FIFO=0 禁用 |
| RNEA velocity cutoff | 120 Hz | 60 Hz | rnea.yaml；全局滤波回滚恢复 120 |
| RNEA acceleration_gain | 0.25 | 0.25，保持 | rnea.yaml |
| RNEA J1/J2/J4 jerk | 500 rad/s³ | 6250 rad/s³ | 18 元素 jerk_limit；回滚 500 |
| RNEA J3 jerk | 500 | 500，保持 | 同上 |
| RNEA J5 jerk | 500 | 1000 | 同上；匹配新 IK 4000 × 0.25 |
| RNEA J6/J7 jerk | 500 | 1500 | 同上；匹配新 IK 6000 × 0.25 |
| RNEA 手臂 torque slew | 1000 Nm/s | 1000，保持 | rnea.yaml |
| IK J5 dq/ddq/jerk | 8 / 100 / 50000 | 4.5 / 45 / 4000 | fixed_base_18d.yaml，wrist_envelope/enabled |
| IK J6/J7 dq/ddq/jerk | 8 / 100 / 50000 | 5.5 / 55 / 6000 | 同上，两臂同时生效 |
| 腕部 braking/outward accel | 原包络值 | 与新 ddq 同值 45/55 | 保证停止证书与命令包络一致 |
| CV position q / velocity gain | 0.63 / 1 | 候选 0.7875 / 0.425 | vr_kalman_flattening，默认关闭 |
| CV chest q / velocity gain | 5.7 / 1 | 候选 7.125 / 0.425 | 同上 |
| CV palm q / velocity gain | 1.42 / 1 | 候选 1.775 / 0.65 | 同上 |

RNEA 已有按关节覆盖机制，本次复用，而没有另建一套顺序。向量顺序为 lower4、L1–L7、R1–R7。下肢 jerk 保持 200。

本次采用严格的 `IK_jerk × 0.25` 对齐，因此 J1/J2/J4 使用 6250，J5 使用 1000。用户给出的 2500/1500 可作为另一个保守参数组，但不是当前新腕部配置的精确对应。独立修改 IK jerk、J3 profile 或 acceleration_gain 后，需要同步更新 RNEA 向量；系统没有动态自动推导，测试会检查交付默认值的一致性。

SkeletonCV 候选缩放 Kalman gain 的速度校正行，并用实际缩放后的 gain 做 Joseph 协方差更新；默认 gain=1 保持原算法。只下调 q 不能同时满足本次两项频响约束，因此候选适度增加 q 并降低速度校正 gain。

## 5. 频响和前馈验证

### 5.1 SkeletonCV

使用实际 PositionCV 稳态离散状态矩阵计算传递函数，并用实际 RotationCV 小角度正弦输入验证；另测变 dt、恒速、复位和协方差正定性。

| 信号（120 Hz 输入） | 10 Hz 原增益 | 候选增益 | 0.1–5 Hz 最大新增相位延迟 | 30 Hz 候选/原幅值 |
|---|---|---|---|---|
| position | 1.128612 | 1.026928 | 0.964 ms | 1.1153 |
| chest | 1.128458 | 1.027019 | 0.961 ms | 1.1150 |
| palms | 1.113041 | 1.026009 | 0.934 ms | 1.0921 |

72/90/120 Hz 三种采样率均满足 10 Hz 增益 ≤1.03、0.1–5 Hz 新增相位延迟 <1 ms。这里是相位除以角频率得到的 phase delay，不能等同所有频率的 group delay；旋转结论是局部小信号性质。

**候选没有解决宽频噪声问题。** 30 Hz 幅值在各组增加约 9–24%，可能增加后级高频输入。因此默认关闭，只交付可选候选与复算工具，不将其宣传为 15–40 Hz 能量下降方案。原始 18:32 骨骼流已不可用，不能做同源 raw SkeletonCV A/B。72 Hz 输入也不应将 40 Hz 当作 Nyquist 内响应解释。

### 5.2 RNEA 低通

按生产实现的 500 Hz 指数一阶离散低通计算，60 Hz 相对 120 Hz：

| 信号频率 | 原幅值 | 新幅值 | 新增相位延迟 |
|---|---|---|---|
| 5 Hz | 0.99928 | 0.99671 | 1.203 ms |
| 15 Hz | 0.99360 | 0.97154 | 1.162 ms |
| 30 Hz | 0.97531 | 0.89960 | 1.042 ms |
| 40 Hz | 0.95768 | 0.84063 | 0.941 ms |

60 Hz 对 15 Hz 抑制仍很弱，不能把这次截止频率调整当成所有高频能量或超调问题的完整解决方案。

### 5.3 RNEA 与 IK jerk 对齐

对实际 S62 RNEA 模型执行 1000 次换向，通过质量矩阵从 raw torque 反解加速度。匹配 jerk 的最大加速度误差为 `4.05e-13 rad/s²`，统一 500 的对照误差为 `18 rad/s²`。

该隔离测试为观测加速度级而放开测试内的 torque/slew 限制，证明这组输入不再被 RNEA jerk 二次削慢；不证明真实输出力矩没有延迟。实际 1000 Nm/s torque slew、动态 ramp、安全 jerk 放宽、饱和和 60 Hz 测量速度滤波仍可能引入非线性/相位变化。

## 6. 执行过的验证

| 验证 | 结果 | 证据文件（output/tail-filter 下） |
|---|---|---|
| 全包 Release 编译、tests target | 通过 | build-final.log、build-tests.log |
| 全部注册 CTest | 56/56 通过 | ctest.log，45.80 s |
| Python 单元与独立回归 | 28/28 文件通过，232 unittest 用例 | python/results.json、python/*.log |
| 实际正式 SHM 热路径 | 20,000 次；所有五相 C++ new/new[] 计数 0 | hotpath.log |
| POD 队列 | 固定容量、不可变快照、满队列丢镜像、stop 拒绝及 drain 通过 | CTest async_pod |
| 实际正常/governor QP | 1200 个变 dt tick 通过 | jerk-integration.log |
| 躯干许可/保持 | 3492 tick 通过 | torso-s62.log |
| RNEA 实际模型与换向 | S62/S63/400062 回归通过 | CTest；rnea-reversal.log |
| 18:32 目标片段复算 | 两组各 6000 tick 通过 | 1832-*.jsonl 及 .summary.json |
| 可视闭环仿真 | 未通过，调优和参数回滚均无有效 ACTIVE | simulation*-replay.json、simulation*-ik.log、simulation.xwd |

热路径各相位最大耗时：镜像相位 69.488 µs；其余相位 4.420、3.411、4.102、3.460 µs。快速压力循环中 3998 个镜像被消费、2 个镜像因队列满丢弃；正式提交未丢失。这里统计的是调用线程 C++ new/new[]，不是系统全部 malloc，也不是目标机器人 500 Hz 调度下的延迟上界。

Python 补测曾发现两个执行问题，已处理并重跑：旧 fixed_pipeline_hold 测试按失效注释提取旧底盘分支，现明确测试保留的 reference-twist fallback，当前 joystick 分支继续由 velocity_ik_base_command_test 覆盖；quest_startup_config 必须传入已编译 FSM 路径。S63 仿真旧测试预期与基线 launch 的 S62/400062 gate 不一致，测试现断言该 gate 继续拒绝 S63，没有扩大仿真模型许可。需要真实 SG100 服务的 integration 脚本不属于单元测试，本轮未将它计为通过。

### 6.1 18:32 离线片段边界

输入来自留存 `new_1832.npz`，起点 epoch `1791974055.0`、12 秒、6000 个 2 ms tick。抽取脚本为 `scripts/tools/extract_tail_filter_replay.py`。原始 bag 已不在当前保留数据中。

100 Hz 已整形 torso-relative TCP 插值到 500 Hz，下肢意图从记录参考 FK 重建，缺失的肘速度前馈置零；初始 dq=0，未排除预热片段。主复算采用生成参考反馈而非记录传感器闭环（`recorded_feedback=false`），因此是求解器和约束可行性回归，不是完整生命周期、Mapper、动力学或真实误差重现。

| 结果 | 原腕部包络 | 新腕部包络 |
|---|---|---|
| QP failures / owner rejections / unexpected braking | 0 / 0 / 0 | 0 / 0 / 0 |
| L5 最大 dq / ddq | 8 / 100 | 4.5 / 45 |
| L6 最大 dq / ddq | 6.542 / 100 | 5.5 / 55 |
| L7 最大 dq / ddq | 6.071 / 100 | 5.5 / 55 |
| R5 最大 dq / ddq | 8 / 100 | 4.5 / 45 |
| R6 最大 dq / ddq | 8 / 100 | 5.5 / 55 |
| R7 最大 dq / ddq | 7.762 / 100 | 5.5 / 55 |
| J3 未放宽拍最大 jerk | 2000.000239 | 2000.000245 |
| J3 安全放宽轴拍数量 | 84 | 61 |
| emergency_dynamics tick | 1 | 2 |

数值边界存在约 5e-7 加速度求解容差。测试逐拍检查腕部速度、加速度及正常 jerk 边界，并保留已有物理安全冲突优先的 jerk 放宽机制；不能称为“所有拍都绝不超过配置 jerk”。调优组 J3 包含放宽拍的最大 jerk 为 13301.77 rad/s³；该行为未被隐藏或删除。极窄区间的数值冲突仍可能触发 emergency dynamics，需在下一轮稳定闭环中审查。

### 6.2 可视仿真未通过

独立容器、私有 ROS master、`/real=false`、独立 SHM key 354421/354422；实际打开远端 MuJoCo GUI 并保存窗口截图。真实运行候选 IK/WBC/RNEA，按 ARMED→PREPARE→ACTIVE→HOLD 请求重放目标。

调优组采集 sensor=15180、reference mirror=3018；回滚组 sensor=15157、reference mirror=3013；两组有效参考与 ACTIVE 采纳均为 0。StartupReadiness 持续报告关节运动，画面显示机器人折落到轮座附近。没有获得安全接管，不能把镜像里的 dq 当作成功执行的命令。

回滚测试使用同一修改后代码、仅恢复滤波参数，所以只能说明故障不是仅在新参数下出现；未运行 pristine 基线闭环，不能断言已证明是旧问题。仿真容器已停止，日志保留。下一步先在此隔离工作树核对 MuJoCo 初姿、驱动模式、启动保持力矩与测量速度来源，取得原参数稳定启动基线后，再重复新参数 A/B；不得通过降低启动门槛使测试“通过”。

## 7. 复现与回滚

### 7.1 已存在验证环境中复跑

主机上的镜像为 `kuavo-4479-regression:20261021`，构建容器为 `kuavo-tail-filter-20261021`。候选挂载 `/candidate`，依赖 checkout 只读挂载在 `/home/sim/kuavo-ros-control-absolute`。无外部网络，不启动硬件。

```bash
ssh yyb@192.168.5.183
docker exec -it kuavo-tail-filter-20261021 bash
source /candidate/output/tail-filter/env.sh
cmake --build /candidate/output/tail-filter/build -j2
cmake --build /candidate/output/tail-filter/build --target tests -j2
ctest --test-dir /candidate/output/tail-filter/build --output-on-failure
python3 /candidate/output/tail-filter/run_python.py
python3 /candidate/src/wheel_arm_abs_vel/test/tail_filter_frequency_test.py
/candidate/output/tail-filter/devel/lib/wheel_arm_abs_vel/velocity_ik_formal_reference_hotpath_test
```

CMake 配置：Release、CATKIN_ENABLE_TESTING=ON、VELOCITY_IK_BUILD_OPERATING_REPLAY=ON、drake_DIR=/opt/drake/lib/cmake/drake。`env.sh` 按依赖 setup、候选 devel setup 顺序 source，并设置 localhost ROS master、候选 ROS_PACKAGE_PATH/PYTHONPATH/LD_LIBRARY_PATH；不能只 source 旧依赖导致 roslaunch 误选旧包。

### 7.2 片段复算

以下在构建容器的私有 ROS master 中运行；不得对真实机器人 master 执行。

```bash
source /candidate/output/tail-filter/env.sh
rosparam load /candidate/src/wheel_arm_abs_vel/config/fixed_base_18d.yaml /absolute_velocity_ik
rosparam load /candidate/src/wheel_arm_abs_vel/config/j3_overshoot_profile.yaml /j3_overshoot_profile
rosparam set /j3_overshoot_profile/enabled true
rosparam set /absolute_velocity_ik/robot_version 62
rosparam set /absolute_velocity_ik/operating_diagnostics/enabled true
rosparam set /absolute_velocity_ik/joint_jerk/enabled true
rosparam set /absolute_velocity_ik/wrist_envelope/enabled true
/candidate/output/tail-filter/devel/lib/wheel_arm_abs_vel/velocity_ik_j3_window_replay /candidate/output/tail-filter/abs_1832_replay.csv /candidate/output/tail-filter/1832-tuned.jsonl
rosparam set /absolute_velocity_ik/wrist_envelope/enabled false
/candidate/output/tail-filter/devel/lib/wheel_arm_abs_vel/velocity_ik_j3_window_replay /candidate/output/tail-filter/abs_1832_replay.csv /candidate/output/tail-filter/1832-baseline.jsonl
```

### 7.3 一键参数回滚

在控制器与 Quest/Pico 输入两个启动终端都设置同一开关，再按原启动方式重启对应进程：

```bash
export KUAVO_ABS_FILTER_TUNING=false
```

等价于在两条 launch 命令都传 `filter_tuning_enabled:=false`。恢复调优设 true。该开关将 RNEA 切换到完整 `rnea_baseline.yaml`（包括显式旧 jerk 向量，避免 rosparam 残留），禁用新腕部包络和 CV 候选；保留通信修复。显式另传 `wrist_envelope_enabled:=true` 会覆盖腕部默认，回滚时应移除此覆盖。候选 CV 单独开启需 `filter_tuning_enabled:=true vr_kalman_flattening_enabled:=true`，当前不建议把它用于未验证的实机。

本轮没有替用户执行真实控制器重启。所有调参均为启动时配置，不宣称在线原子切换整个参数组。

## 8. 后续验收清单

- 先解决隔离仿真的 pre-owner 启动不稳定，保留安全门，取得可重复的原参数及新参数闭环结果。
- 保存新一轮完整原始 PoseInfoList、TeleopTcpTarget、正式参考镜像、WBC stage timing、执行反馈、sensor/joint_cmd 及 CPU/调度证据，重新统计 P50/P99/P99.9/Max。
- 核验 WBC 100 ms 周期尖峰、IK 五相耗时和镜像丢弃、Mapper callback FIFO 的实际生效策略；开发容器结果不能代替 AAEON/AGX 实测。
- 若要启用 CV 候选，必须以同一 raw skeleton 做 A/B，联合检查 10 Hz 增益、15–40 Hz 能量、L5/L1/R1/L4 误差和执行力矩，不只看单点 10 Hz 指标。
- 本次没有修改硬件反馈 Kalman/joint_vd，硬件侧和总周期残余分配/调度机制仍需后续独立工作。

## 9. 完整 git diff --name-status

以下清单由交付工作树实际生成，包含新增包内文件；所有路径均在允许范围。构建证据目录不纳入补丁。

```text
M	src/wheel_arm_abs_vel/CMakeLists.txt
M	src/wheel_arm_abs_vel/config/fixed_base_18d.yaml
M	src/wheel_arm_abs_vel/config/rnea.yaml
A	src/wheel_arm_abs_vel/config/rnea_baseline.yaml
M	src/wheel_arm_abs_vel/config/runtime_partition.json
M	src/wheel_arm_abs_vel/config/runtime_partition_aaeon_255h.json
M	src/wheel_arm_abs_vel/config/vr_kalman.yaml
A	src/wheel_arm_abs_vel/docs/communication_tail_filter_tuning_20261021.md
M	src/wheel_arm_abs_vel/include/wheel_arm_abs_vel/humanoid_controllers/WholeBodyReferenceShmRos.h
M	src/wheel_arm_abs_vel/include/wheel_arm_abs_vel/humanoid_controllers/absolute/humanoidController_wheel_absolute.h
M	src/wheel_arm_abs_vel/launch/dynamics_backend.launch
M	src/wheel_arm_abs_vel/launch/load_kuavo_absolute_sim_base.launch
M	src/wheel_arm_abs_vel/launch/load_kuavo_mujoco_velocity_ik.launch
M	src/wheel_arm_abs_vel/launch/load_kuavo_real_velocity_ik.launch
M	src/wheel_arm_abs_vel/launch/pico_velocity_ik.launch
M	src/wheel_arm_abs_vel/launch/quest_velocity_ik.launch
M	src/wheel_arm_abs_vel/scripts/ik_ros_velocity.py
M	src/wheel_arm_abs_vel/scripts/runtime_partition.py
A	src/wheel_arm_abs_vel/scripts/tools/extract_tail_filter_replay.py
M	src/wheel_arm_abs_vel/scripts/tools/mapper_input_scheduling.py
M	src/wheel_arm_abs_vel/scripts/tools/noncontrol_publish_process.py
M	src/wheel_arm_abs_vel/scripts/tools/pose_velocity_kalman.py
M	src/wheel_arm_abs_vel/scripts/tools/quest3_utils_velocity.py
M	src/wheel_arm_abs_vel/src/RneaBackend.cpp
M	src/wheel_arm_abs_vel/src/absolute/humanoidController_wheel_absolute.cpp
M	src/wheel_arm_abs_vel/src/pinocchio_velocity_ik_node.cpp
M	src/wheel_arm_abs_vel/test/fixed_pipeline_hold_test.py
M	src/wheel_arm_abs_vel/test/launch_model_contract_test.py
A	src/wheel_arm_abs_vel/test/mapper_scheduling_test.py
M	src/wheel_arm_abs_vel/test/model_platform_matrix_test.py
M	src/wheel_arm_abs_vel/test/rnea_backend_test.cpp
A	src/wheel_arm_abs_vel/test/tail_filter_frequency_test.py
M	src/wheel_arm_abs_vel/test/test_bypass_communication.py
M	src/wheel_arm_abs_vel/test/x86_topology_test.py
M	src/wheel_arm_abs_vel/velocity_ik/CMakeLists.txt
M	src/wheel_arm_abs_vel/velocity_ik/async_diagnostics.h
A	src/wheel_arm_abs_vel/velocity_ik/async_pod_test.cpp
A	src/wheel_arm_abs_vel/velocity_ik/formal_reference_hotpath_test.cpp
M	src/wheel_arm_abs_vel/velocity_ik/j3_window_replay.cpp
```
