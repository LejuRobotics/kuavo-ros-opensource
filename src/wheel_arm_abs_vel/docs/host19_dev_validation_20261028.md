# 当前 19 的 dev 修复验证（2026-10-28）

目标为 192.168.5.19、AAEON，机器 ID `e04c08473cd146e084582b8e436586d9`，工作区 `/home/lab/yyb/dev`。基线为远端最新 `dev c613692f1522847dd5c257ec5405768b0e374ad2`；源代码验证提交 `8b9846224f6d7334ff1f5587908c9349447a6e36`。

- 新建独立 Git worktree，保留 `yyb/kuavo-ros-control-dev` 的 `cy/beta/fix_lb_torso_control` 分支，以及 lhm/merge-review、test 的未提交修改。
- 仅扩展 `/opt/ros/noetic`，在新工作区完整构建：62/62 包成功，0 失败，15 包有编译警告，耗时 16 分 43.5 秒。
- 补建测试目标后：`wheel_arm_abs_vel` CTest 65/65 通过，41.90 秒；`hardware_command_gate_test` 1/1 通过。GTest 结果为 28 用例，0 errors、0 failures、0 skipped。
- Pico 消息从本工作区 `devel/lib/python3/dist-packages` 加载，`PoseInfoList` MD5 为 `f56d5e4df1c0e38a423ead4bde9414b3`。Pico 输入回归 11/11 通过。
- 共享模型解析 2/2、模型/平台矩阵 22/22 通过；62/63/400062 的解析后限位与原私有配置完全相同。AGX 用例为拓扑夹具，不是 AGX 原生编译或实机验收。
- 当前 19 只读平台检查选择 AAEON 255H：WBC 2/3、驱动保留 6/7、IK 5、Mapper 4、sidecar 14/15。IK 为已有 soft isolation 模式，未修改系统绑核或隔离配置。
- CXX 硬件 Nodelet、绝对式控制 Nodelet、Pinocchio IK 可执行文件的 `ldd` 均无缺失库。ROS launch 仅解析，确认默认加载 `HardwareNodeletCXX`。
- Kp/Kd 相关模型配置、J3 profile YAML/launch、控制器增益源码与 `origin/dev` 差异为零。S62 原 dev 默认启用 J3 profile：J1/J3 Kp=250，J3 Kd=18；本候选未采用此前另一台机器试验过的 Kd=10 回退。
- 没有启动真机或硬件节点，没有删除启动静止检查，没有执行任何真实动作。此前另一台 AAEON 的操作日志不能当作当前 19 或 AGX/S63 的实机验收。

构建命令：

```bash
cd /home/lab/yyb/dev
source /opt/ros/noetic/setup.bash
catkin config --merge-devel --extend /opt/ros/noetic
catkin build wheel_arm_abs_vel -j6 -p3 --no-status --summarize --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON
source devel/setup.bash
cmake --build build/wheel_arm_abs_vel --target tests -j4
(cd build/wheel_arm_abs_vel && ctest --output-on-failure -j2)
(cd build/hardware_node && ctest -R hardware_command_gate_test --output-on-failure)
```

原始日志保留在目标工作区 `output/current19-validation/`：`build.log`、`build-tests.log`、`ctest.log`、`hardware-ctest.log`、`platform.json`、`source-manifest.json`。
