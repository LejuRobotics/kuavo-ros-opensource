# 仿真未接管保持与全身绝对速度 IK 闭环修复

## 交付边界

- 日期：2026-10-21；主机：yyb（192.168.5.183）。本轮未连接、部署或启动 19 实机。
- 独立工作区：`/home/yyb/workspace/kuavo-abs-sim-hold-20261021`。
- 分支：`fix/abs-sim-hold-20261021`；基线：`23c6a8750`。
- 所有提交文件位于 `src/wheel_arm_abs_vel/**`。共享 MuJoCo、hardware_plant、motion_capture_ik、旧 humanoid_controllers 和模型资产文件均未修改。
- 本次基线为按键整改分支，不是 19 上的长尾/滤波部署分支。没有把两条分支或验证结果混为一谈。

## 根因与证据

1. 共享 MuJoCo 的 ROS `jointCmdCallback()` 只提取 `tau`，物理执行端直接写 `d->ctrl`，没有执行命令中的位置/速度伺服。RNEA 提供的是前馈力矩，不能替代电机端位置闭环。因此即使 `joint_q` 固定、`joint_v=0`，仿真仍能持续漂移。
2. 头部只有控制器按 ROS 反馈计算的外层 PD，缺少物理步内的稳定伺服。原失败 bag 的头部瞬时速度超过 47 rad/s。修复将反馈移到 MuJoCo 原生仿射执行器、隐式积分器中，关闭该模式下重复的外层头部 PD。WBC 与 RNEA 均复测静止保持通过。
3. `AbsoluteWeightedWbc::update()` 原来忽略 `qpProblem.init()` 的求解状态；取解失败时还能返回上一拍解。现在失败抛出明确错误，现有控制器故障路径拒绝继续采用它，私有仿真响应 `/stop_robot` 暂停物理推进并随控制器清理退出。
4. 原人工骨骼测试的 Chest 四元数为单位旋转，但 Mapper 的参考姿态为 RPY `[pi/2,0,pi/2]`，这使手部目标落在错误方向。测试夹具已经按既有映射契约重建：用 URDF FK 产生可达 TCP，再反解骨骼方向和控制器姿态；生产 Mapper 映射未修改。

以上支持“底层闭环/仿真执行缺口”的结论。不能将原始 WBC QP 不可行全部归因于其中某一增益，也没有把它归咎于 19 的滤波参数。

## 实现

| 位置 | 变更 |
| --- | --- |
| `include/wheel_arm_abs_vel/SimulationServo.h` | 私有仿真伺服；`tau_ff + Kp(q_des-q) + Kd(dq_des-dq)`；MuJoCo 物理步内计算反馈；`implicitfast`；模型原有力矩上限限制总执行器力 |
| `scripts/prepare_absolute_sim.py` | 仅生成私有翻译单元，装配完整命令快照；初始化与命令编码由同一 mutex 同步；响应停止信号；共享源文件保持原样 |
| `src/absolute/ControlDataManager.cpp` | 仅在仿真原生伺服启用时清零原有头部外层 PD；真实机器人路径不变 |
| `src/absolute/AbsoluteWeightedWbc.cpp` | 检查 QP 求解和取解返回值，拒绝复用失败解 |
| `launch/load_kuavo_mujoco_velocity_ik.launch` | 默认明确使用 RNEA，转发仿真伺服开关；真实机器人 launch 未修改 |
| `config/simulation_servo.yaml` | 独立的关节侧 SI 增益，避免直接把 EtherCAT PDO 数值当作 Nm/rad 和 Nm·s/rad |

SHM q/dq/ddq 帧编码、owner 租约、接管门禁、IK 包络、RNEA 滤波参数均未改变。

## 仿真参数

| 参数 | 原路径 | 修复后 |
| --- | --- | --- |
| 私有仿真电机侧伺服 | 缺失 | `simulation_motor_servo=true` |
| 公开绝对 IK 仿真入口后端 | 默认 wbc | 默认 rnea；仍可显式选择 wbc |
| 下肢四关节 Kp / Kd | ROS 命令中的增益未参与仿真 | `[3000,3000,1000,500]` / `[150,150,80,30]` |
| 单臂 J1–J7 Kp / Kd | 同上 | `[270,210,256,180,120,120,120]` / `[20,10,10,10,6,6,6]` |
| 头部 Kp / Kd | 外层 ROS PD | `[10,10]` / `[1,1]`，由物理执行器统一计算 |
| 力矩限制 | XML 控制输入限幅 | 同一 XML 数值用于总执行器力限幅，避免仿射参考项被提前截断 |
| 关闭伺服 | 不适用 | 启动参数 `simulation_motor_servo:=false`，需重启；同时恢复旧头部外层 PD |

这些是仿真关节侧参数，不是实机 PDO 标定，不作为实机调参建议。回滚伺服不会撤销 QP 失败保护。

## 验证结果

- 完整构建通过；57/57 CTest 通过。最终补充的私有仿真源、launch 和原生伺服 3 项回归也通过。
- 28 个 Python 单元测试文件通过（含 CTest 已覆盖文件与额外 4 个文件）。另一个 `sg100_service_handover_test.py` 是依赖真实运行 SG100 服务的仿真集成脚本；当前 S62 配置为 qiangnao，缺少 `/sg100/vr_step_gesture`，该脚本等待超时退出，未算通过。
- 原生小惯量关节测试：保持、正反向跟随、总力矩限幅、非法命令拒绝通过。
- RNEA 未接管保持：15 秒约 7500 帧，最大速度小于 `1e-10 rad/s`；输入链启动但不接管的独立 8 秒窗口，最大速度小于 `2e-10 rad/s`。
- WBC 未接管保持：10 秒 5003 帧，最大速度 `1.35e-10 rad/s` 以下，位置命令范围为零。
- 手柄协议仿真：双前扳机接管 → 双臂周期运动 → Quest XY 触摸+B 躯干授权 → 躯干联动 → X+B 双臂暂停/恢复 → X+A 回准备位 → PREPARED 确认 → X+A 再接管 → X+Y 全局停止。全部观察到对应状态及实际命令/反馈。
- 停止按键按既有语义关闭控制器；私有仿真先暂停物理，再清理退出。停止后没有反馈帧，不将“无数据”记成“零速度保持”。
- 前台图形窗口已截图核验。仿真输入为可重复的合成 Quest 协议消息，尚未用真实头显/实体手柄验收。

完整闭环 bag 为 406.844 秒，非有限值样本数为 0。下表跳过每阶段前 2 秒，误差为反馈与最近已接收关节命令之差，不是网络延迟估计。

| 阶段 | 反馈帧数 | 单关节最大误差 ° | 最大单关节 RMS 误差 ° | 最大手臂命令变化 rad |
| --- | --- | --- | --- | --- |
| before_takeover | 13996 | 0.0007 | 0.0007 | 0 |
| arm_follow | 14147 | 0.0983 | 0.0119 | 0.1006 |
| torso_follow | 16311 | 0.1129 | 0.0170 | 0.1006 |
| arm_paused_torso_follow | 20574 | 0.0124 | 0.0095 | 1.723e-10 |
| arm_resumed | 7936 | 0.0808 | 0.0118 | 0.1006 |
| prepare_settle | 70962 | 0.0223 | 0.0024 | 0.00478 |
| prepared_resume_follow | 11100 | 0.1861 | 0.0415 | 0.08051 |

包含接管/准备/恢复过渡的全程最大手臂误差为 2.099°、最大手臂速度为 1.417 rad/s。不得把上表稳态窗口的误差上限推广到过渡阶段。头部没有重力前馈，当前低刚度设置存在约 1.5° 静态下垂，但无持续漂移。

有一次初始化运行触发原有 `stale_measurement_or_invalid_period` 保护（50 ms 门槛），已保留失败日志 `final-rnea.log`。没有放宽阈值；后续完整运行无该故障。其精确是周期还是测量年龄触发，现有错误字符串不能进一步区分，属于仍需保留的启动可用性风险。

## 证据与复现

证据目录：`output/sim-hold/`；主要文件：`final-closed-loop.bag`、`bag.sha256`、`closed-loop-report.json`、`phases.jsonl`、`final-rnea-retry.log`、`final-input.log`、`wbc-hold-metrics.json`、`final-desktop.png`。构建/测试日志位于 `output/`。

独立容器：`kuavo-abs-sim-fix-20261021`，私有 ROS master；不连接实机 ROS master。当前交付窗口重新启动为未接管静止保持，合成输入程序已经退出。

已有仿真退出后，在 yyb 启动：

```bash
docker exec -it kuavo-abs-sim-fix-20261021 bash -lc 'bash /candidate/output/sim-hold/run_sim.sh'
```

另开终端连接真实 Quest（把地址替换为头显 IP；此项尚未实测）：

```bash
docker exec -it kuavo-abs-sim-fix-20261021 bash
source /candidate/output/sim-hold/env.sh
roslaunch wheel_arm_abs_vel quest_velocity_ik.launch ip_address:=<Quest_IP> reference_shm_key:=374421 owner_shm_key:=374422
```

离线重算：

```bash
docker exec -it kuavo-abs-sim-fix-20261021 bash
source /candidate/output/sim-hold/env.sh
python3 /candidate/src/wheel_arm_abs_vel/test/simulation_hold_report.py /candidate/output/sim-hold/final-closed-loop.bag /candidate/output/sim-hold/phases.jsonl /candidate/output/sim-hold/closed-loop-report.json
```
