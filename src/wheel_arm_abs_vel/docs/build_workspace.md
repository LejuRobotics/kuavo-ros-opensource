# 全身 IK 首次构建与增量构建

在已经安装本机架构对应的 ROS Noetic、Drake、Pinocchio/hpp-fcl 和仓库系统依赖的环境执行。源码工作区可以没有任何 build/devel 产物；本入口不安装操作系统依赖，也不启动机器人。

包内新增模块还需要 `pybind11-dev`、`nlohmann-json3-dev`、`libssl-dev`；构建工具需要 `catkin_tools` 和 Python PyYAML。Ubuntu 22.04 的 Noetic 安装应使用对应平台的源码构建环境，不混用其他系统版本的 ROS/SDK 二进制。

## 首次构建

```bash
cd /home/lab/kuavo-ros-control
bash src/wheel_arm_abs_vel/scripts/build_workspace.sh && source devel/setup.bash
```

默认使用 `ROBOT_VERSION=62`、`-j16 -p6`、80% 内存调度阈值，使用 `velocity-ik` profile，输出到 `build/velocity-ik` 和 `devel`。如需其他机型，在调用前设置 `ROBOT_VERSION`；通过 `JOBS` 和 `PACKAGE_JOBS` 调整并发，通过 `ROS_PREFIX` 和 `DRAKE_PREFIX` 指定已安装的 ROS/Drake 前缀。

脚本先生成消息，再显式构建 `kuavo_assets` 库目标，随后根据当前 package.xml 自动计算并构建完整工作区依赖闭包。资产包的 ALL 安装/硬件配置钩子不在源码构建阶段执行。最后一阶段虽然使用 `--no-deps`，但已经显式列出全部依赖，避免漏编或重复执行资产包钩子。

## 已完成完整构建后的包内更新

```bash
cd /home/lab/kuavo-ros-control
source /opt/ros/noetic/setup.bash
catkin profile set velocity-ik
catkin build wheel_arm_abs_vel --no-deps -j16 -p6 --no-status && source devel/setup.bash
```

若本次更新涉及依赖包，重新运行首次构建入口，让 Catkin 增量更新完整依赖闭包。

## JSON / Eigen 编译兼容性

`direct_controller_tcp_node`、`direct_tcp_convergence`、`direct_tcp_parameter_audit` 和 `direct_tcp_preview` 共用 `sampleFromJson()`。该函数先用 `get<double>()` 提取 JSON 数值，再显式构造 Eigen 三维向量，避免 Eigen 3.4 的列表构造函数与 JSON 隐式转换相互作用。位置、四元数、线速度和角速度的字段顺序不变，仍由 `sample()` 验证有限值与单位四元数。

回归目标 `direct_tcp_json_sample_test` 检查混合整数/浮点字段的结果、非法类型、缺失字段和非法四元数。
