# X+A 全身复位与默认躯干模式回归（2026-10-23）

基于 dev `7853c303aedea9e64bddf95631f2454e777c2023`，修复分支 `fix/xa-full-reset-torso-default-20261023`。

## 行为与实现

- 首次有效遥操默认躯干模式，腰部、折叠臂和双臂可跟随；先 X+A 准备再进入跟随也保留默认选择。
- Quest 触摸 X+Y 后新按 B，Pico RT+Y，切换躯干／手臂模式。手臂模式保持腰部和折叠臂，双臂仍跟随。
- X+A 在两种模式下均将腰部、折叠臂和双臂送回配置准备姿态。复位中再次 X+A 可中断；恢复跟随后保留所选模式。
- 所选模式与实际运动授权分离。停止、故障、无效输入或失联撤销授权，单纯恢复数据不重开；显式跟随请求须通过原有检查，再等待有效输入和就绪握手。

原问题来自复位轨迹及下肢约束错误依赖人体躯干跟随权限，以及 X+A 撤销授权时丢失启动模式。现在通过原有执行权、时效、模式与故障检查的 PREPARE 独立开放四个下肢关节；复位完成仍检查位置与速度。普通跟随继续由躯干权限控制。

## 验证

Release 编译通过。完整 CTest **74/74** 通过，包括 S62、S63、400062 的生产求解器测试：关闭人体躯干权限时四个下肢关节和双臂仍完成复位，HOLD／指令失效时停止复位并保持，普通手臂模式保持下肢，躯干使能与 X+B 行为保持有效。Python 回归覆盖先准备再首次跟随、模式保留、组合键边沿及故障撤销。

S62 MuJoCo 闭环使用合成 Quest/Pico 协议输入，运行生产 Mapper、FSM、IK 和仿真执行器。记录 `/mm/teleop_tcp_target`、手柄／骨骼输入、`/sensors_data_raw`、`/joint_cmd` 及 IK 诊断。Quest **39/39**、Pico **38/38** 断言通过，覆盖两种模式全身复位、默认躯干、切换、复位后模式保留及复位中断。

| 反馈指标 | Quest | Pico |
| --- | ---: | ---: |
| 躯干模式复位后四个下肢关节最大准备姿态误差（rad） | 0.000835 | 0.000753 |
| 手臂模式复位后四个下肢关节最大准备姿态误差（rad） | 0.000966 | 0.000871 |
| 手臂模式下肢保持最大变化范围，4 秒窗口（rad） | 0.0000402 | 0.0000266 |
| 复位后手臂模式下肢保持最大变化范围，4 秒窗口（rad） | 0.0001594 | 0.0001737 |

这些结果属于软件与合成输入闭环仿真；未部署实机，未进行实体 Quest/Pico 或真机动作验收。四个下肢关节逐一位移检查另由三个机型的生产求解器回归覆盖。

## 复现与证据

编译并 source 当前工作区后，使用新的输出目录运行：

```bash
python3 src/wheel_arm_abs_vel/test/xa_torso_simulation_test.py /tmp/xa-torso-regression
```

脚本仅启动 MuJoCo，禁用硬件输入接收器，以独立 ROS master 和共享内存键运行两套输入流程；场景和输入发生器均检查 `/real=false`。

本次开发机工作区 `/home/yyb/workspace/kuavo-xa-torso-fix-20261023`；证据在 `output/xa-torso/`：`build.log`、`ctest-corrected.log`、`verified-summary.json`、`simulation-v3/{quest,pico}/checks.json`、`closed-loop.bag` 和 `simulation-v3/runtime-provenance.json`。已确认运行中的 Mapper、IK、MuJoCo 和控制器库来自修复候选。

最初 CTest 使用了旧 underlay 的嵌套 launch 且未编译 gtest 目标，修正源路径并编译 tests 后 74/74 通过。初次仿真遇到重复 launch 路径、测试源 overlay 遮蔽可执行文件，调整验证环境后完整闭环通过；生产控制逻辑未因此放宽条件。
