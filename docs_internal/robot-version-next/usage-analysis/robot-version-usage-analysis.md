# ROBOT_VERSION 现有使用场景梳理

## 省流总结

### 场景 1：资源路径拼接（占比 ~60%）

版本号直接拼进文件路径，用于定位 URDF 模型、MPC/RL 配置、碰撞检测配置等。三种前缀模式：

- `biped_s${V}/` — 模型目录（URDF、mesh、xml 场景文件）
- `kuavo_v${V}/` — 配置目录（kuavo.json、MPC task.info、RL 参数）
- `kuavo_s${V}/` — 轮式专用配置（仅 v60-v63）

Launch、C++、Python、Shell 四种语言中均存在，覆盖 30 个 `biped_s*` 模型目录和 32 个 `kuavo_v*` 配置目录。运行时数据文件也带版本号后缀：`TotalMassV${V}`、`.urdf_md5_v${V}`、`/var/ocs2/kuavo_v${V}`。

### 场景 2：版本判断 / 条件分支（占比 ~25%）

用数值比较或字符串匹配走不同代码分支。核心判断维度：

- **品牌**：`major()==1` / `>=40` / `[0]=='1'` / `^1[0-9]$` 区分 Roban 与 Kuavo
- **代次**：`major()==4/5` / `>=50` 区分 Kuavo 4 与 Kuavo 5（影响腰关节、手臂 DOF、MPC 配置）
- **运动方式**：`major()==6` / `60<=v%100<70` 区分双足与轮式
- **末端**：`=="49"` / `=="47"` 精确匹配判断灵巧手/夹爪/展厅假手
- **硬件修订**：`=="53"` / `=="54"` 区分不同电机型号的 EtherCAT 配置

C++ 侧有 `RobotVersion` 类 + 9 个宏（`IS_KUAVO_LEGGED` 等）封装判断逻辑。Python 侧用 `(int(v)//10)%10` 提取主版本号后做分支。

### 场景 3：ROS 参数服务器读写

Launch 通过 `<param name="robot_version" value="..."/>` 写入（当前为 int 类型），C++/Python 节点通过 `nh.getParam("/robot_version", int_var)` 或 `rospy.get_param('/robot_version')` 读取。所有读取方均假设 int 类型。

### 场景 4：环境变量直接读取

部分代码绕过 ROS 参数服务器，直接用 `std::getenv("ROBOT_VERSION")` 或 `os.environ.get('ROBOT_VERSION')` 读取。主要出现在硬件驱动层、碰撞检测、标定工具和 IK 脚本中。

### 场景 5：Docker / systemd Service / CI 配置

Docker 镜像和 systemd service 文件中硬编码 `ROBOT_VERSION=42/14/40` 等默认值，部署脚本通过 `sed` 替换为实际值。CI 流水线中分别设置 `ROBOT_VERSION=34`（Roban 构建）和 `ROBOT_VERSION=40`（Kuavo 构建）。

### 场景 6：特殊情况

- **v15→v14 别名**：v15 使用 v14 的全部资源，Launch/C++/Shell 中多处 `if v==15 then v=14` 特殊处理
- **展厅版编码**：100045/100049/200049/300049/400049，通过 `%10000` 或 `Patch` 字段兼容
- **LED 版本范围**：v52-v59 使用新 LED Strip 服务
- **v53 音乐排除**：v53 与上位机冲突，排除音乐服务
- **G12 轮臂模式**：同时读 ROS 参数和环境变量检测 `v>=60 && joystick==h12`
- **版本兼容回退**：找不到当前版本配置时回退到最近已知版本

---

## 概述

本文档对 kuavo-ros-control 代码库中 `ROBOT_VERSION` 的所有使用场景进行全面梳理，作为新旧格式兼容方案的输入依据。

---

## 统计概览

| 类别 | 文件数 | 说明 |
|------|--------|------|
| Launch 文件 | ~55 | 资源路径拼接、版本条件判断、参数传递 |
| C++ 文件 | ~87 | 环境变量读取、ROS 参数读取、RobotVersion 类、宏判断 |
| Python 文件 | ~54 | 环境变量读取、ROS 参数读取、主版本提取、关节配置 |
| Shell 脚本 | ~22 | 环境变量读取、模式匹配、路径拼接、服务部署 |
| CMake | 1 | 编译时宏定义（死代码） |
| Docker | 3 | 环境变量注入 |
| systemd Service | 4 | 环境变量设置 |
| CI 配置 | 1 | 流水线版本设置 |

---

## 现有资源目录结构

### 版本号全集

审计所有 `biped_s*`、`kuavo_v*`、`kuavo_s*` 目录，完整版本号集合如下：

| 分组 | 版本号 | 说明 |
|------|--------|------|
| Roban 系列 | 11, 13, 14, 16, 17 | Roban 各型号 |
| Kuavo 4 系列 | 40, 41, 42, 43, 45, 46, 47, 48, 49 | Kuavo 4 各型号 |
| Kuavo 5 系列 | 50, 51, 52, 53, 54, 55 | Kuavo 5 各型号 |
| 轮式系列 | 60, 61, 62, 63 | Kuavo 轮式各型号 |
| 展厅/特殊版 | 100045, 100049, 200049, 300049, 400049 | 展厅版和特殊变体 |

**共 33 个唯一版本号。**

### 目录分布

```
src/kuavo_assets/
├── models/                                     # 30 个 biped_s* 目录
│   ├── biped_s11/  biped_s13/  biped_s14/  biped_s16/  biped_s17/
│   ├── biped_s40/  biped_s41/  biped_s42/  biped_s43/  biped_s45/
│   ├── biped_s46/  biped_s47/  biped_s48/  biped_s49/  biped_s50/
│   ├── biped_s51/  biped_s52/  biped_s53/  biped_s54/  biped_s55/
│   ├── biped_s60/  biped_s61/  biped_s62/  biped_s63/
│   └── biped_s100045/  biped_s100049/  biped_s200049/  biped_s300049/  biped_s400049/
└── config/                                     # 32 个 kuavo_v* 目录
    ├── kuavo_v11/  kuavo_v13/  kuavo_v14/  kuavo_v16/  kuavo_v17/
    ├── kuavo_v40/ ... kuavo_v49/
    ├── kuavo_v50/ ... kuavo_v55/
    ├── kuavo_v60/ ... kuavo_v63/
    └── kuavo_v100045/  kuavo_v100049/  kuavo_v200049/  kuavo_v300049/  kuavo_v400049/

src/humanoid-control/humanoid_controllers/config/   # 32 个 kuavo_v* 目录（MPC/RL 参数）
    ├── kuavo_v11/ ... kuavo_v17/
    ├── kuavo_v40/ ... kuavo_v55/
    ├── kuavo_v60/ ... kuavo_v63/
    └── kuavo_v100045/ ... kuavo_v400049/

src/humanoid-wheel-control/humanoid_wheel_interface/config/  # 4 个 kuavo_s* 目录
    ├── kuavo_s60/  kuavo_s61/  kuavo_s62/  kuavo_s63/

src/demo/grab_box/cfg/                           # 11 个 kuavo_v* 目录
    ├── kuavo_v14/  kuavo_v40/ ... kuavo_v52/
```

### 版本编号文件（非目录）

| 文件模式 | 位置 | 已有版本 |
|---------|------|---------|
| `joint_control_params_v${V}.json` | `hardware_node/src/tests/motorTest/config/` | v13, v14, v42, v45, v50, v51, v52, v53 |
| `s${V}_collision_config.yaml` | `kuavo_arm_collision_check/config/` | s45, s49 |
| `arm_breakin_kuavo_v${V}_dual_config.yaml` | `joint_breakin_ros/config/arm_breakin/` | v52, v53 |
| `leg_breakin_*_v${V}_config.yaml` | `joint_breakin_ros/config/leg_breakin/` | v14, v17, v52, v53 |
| `kuavo5_v${V}_dual_canbus_cofig.yaml` | `kuavo_assets/config/` | v53, v62 |

---

## 场景 1：资源路径拼接

通过版本号拼接出资源文件路径，是**最大的使用场景（约 60%）**。

### 1.1 路径模式汇总

| 路径模式 | 使用位置 | 示例 |
|---------|---------|------|
| `models/biped_s${V}/urdf/biped_s${V}.urdf` | Launch(30+), C++, Python | `biped_s45/urdf/biped_s45.urdf` |
| `models/biped_s${V}/urdf/biped_s${V}_gazebo.urdf` | Launch（仿真） | `biped_s45/urdf/biped_s45_gazebo.urdf` |
| `models/biped_s${V}/urdf/drake/biped_v3_arm.urdf` | Python（IK/规划） | `biped_s45/urdf/drake/biped_v3_arm.urdf` |
| `models/biped_s${V}/xml/scene.xml` | Launch（Mujoco） | `biped_s45/xml/scene.xml` |
| `models/biped_s${V}/xml/biped_s${V}.xml` | Shell（update_mass） | `biped_s45/xml/biped_s45.xml` |
| `models/biped_s${V}/meshes/${filename}` | C++（碰撞检测） | `biped_s45/meshes/xxx.stl` |
| `config/kuavo_v${V}/kuavo.json` | Launch, Shell, C++, Python | `kuavo_v45/kuavo.json` |
| `config/kuavo_v${V}/mpc/task.info` | Launch | `kuavo_v45/mpc/task.info` |
| `config/kuavo_v${V}/command/gait.info` | Launch | `kuavo_v45/command/gait.info` |
| `config/kuavo_v${V}/command/reference.info` | Launch | `kuavo_v45/command/reference.info` |
| `config/kuavo_v${V}/mpc/dynamic_qr.info` | Launch | `kuavo_v45/mpc/dynamic_qr.info` |
| `config/kuavo_v${V}/rl/*.info` | Launch, C++ | `kuavo_v45/rl/amp_param.info` |
| `config/kuavo_v${V}/rl_controllers.yaml` | Launch, C++ | `kuavo_v45/rl_controllers.yaml` |
| `config/kuavo_s${V}/task.info` | Launch（轮式） | `kuavo_s60/task.info` |
| `config/s${V}_collision_config.yaml` | Launch | `s45_collision_config.yaml` |
| `config/TotalMassV${V}` | Shell, Python | `TotalMassV45` |
| `config/.urdf_md5_v${V}` | Shell | `.urdf_md5_v45` |
| `/var/ocs2/kuavo_v${V}` | Launch, Shell | `/var/ocs2/kuavo_v45` |
| `/var/ocs2/kuavo_s${V}` | Launch（轮式） | `/var/ocs2/kuavo_s60` |
| `cfg/kuavo_v${V}/bt_config.yaml` | C++（grab_box） | `cfg/kuavo_v45/bt_config.yaml` |
| `joint_control_params_v${V}.json` | Launch（motor_follow_test） | `joint_control_params_v45.json` |

### 1.2 关键文件

#### Launch 中央枢纽：`robot_version_manager.launch`

路径：`src/humanoid-control/humanoid_controllers/launch/robot_version_manager.launch`

被 30+ launch 文件 include，负责设置所有版本相关的资源路径：

```xml
<arg name="robot_version" default="$(optenv ROBOT_VERSION 40)"/>
<param name="robot_version" value="$(arg robot_version)"/>

<!-- URDF 路径 -->
<arg name="urdfFile" default="$(find kuavo_assets)/models/biped_s$(arg robot_version)/urdf/biped_s$(arg robot_version).urdf"/>

<!-- 配置路径 -->
<param name="taskFile" value="$(find humanoid_controllers)/config/kuavo_v$(arg robot_version)/mpc/task.info"/>
<param name="referenceFile" value="$(find humanoid_controllers)/config/kuavo_v$(arg robot_version)/command/reference.info"/>
<param name="dynamicQrFile" value="$(find humanoid_controllers)/config/kuavo_v$(arg robot_version)/mpc/dynamic_qr.info"/>
<param name="gaitCommandFile" value="$(find humanoid_controllers)/config/kuavo_v$(arg robot_version)/command/gait.info"/>
<arg name="kuavoConfigFile" default="$(find kuavo_assets)/config/kuavo_v$(arg robot_version)/kuavo.json"/>
```

特殊处理：版本 15 使用版本 14 的资源（通过 `if/unless` 条件）。

#### 根级别：`robot_setup.launch`

```xml
<param name="robot_version" type="string" value="$(env ROBOT_VERSION)" />
```

直接读取环境变量并设置为 ROS 参数。

#### C++ 直接拼接路径的文件

| 文件 | 代码模式 |
|------|---------|
| `kuavo_arm_collision_check/src/arm_collision_checker.cpp` | `"/models/biped_s" + robot_version + "/urdf/..."` 和 `"/config/kuavo_v" + robot_version + "/kuavo.json"` 和 `"/models/biped_s" + robot_version + "/meshes/"` |
| `humanoid_controllers/src/rl/AmpWalkController.cpp` | `"biped_s" + std::to_string(major) + std::to_string(minor)` |
| `humanoid_controllers/src/rl/DepthWalkController.cpp` | `"biped_s" + std::to_string(major) + std::to_string(minor)` |
| `humanoid_controllers/src/humanoidController.cpp` | `"biped_s" + rb_version.to_string()` |
| `demo/grab_box/src/grab_box_demo.cpp` | `"kuavo_v" + rb_version.to_string()` |
| `demo/grab_box/src/tagTracker.cpp` | `"kuavo_v" + std::to_string(robot_version_)` |
| `kuavo_common/src/kuavo_common.cpp` | `"kuavo_v" + effective_version` |
| `manipulation_nodes/motion_capture_ik/src/main_node.cpp` | `"/config/kuavo_v" + std::to_string(robotVersionInt)` |
| `manipulation_nodes/motion_capture_ik/src/ik_ros_uni_cpp_node.cpp` | `"/config/kuavo_v" + std::to_string(robotVersionInt)` |
| `manipulation_nodes/motion_capture_ik/src/wheel_ik_ros_uni_cpp_node.cpp` | `"/config/kuavo_v" + std::to_string(robotVersionInt)` |
| `manipulation_nodes/motion_capture_ik/src/arms_ik_node.cpp` | `"/models/biped_s" + rb_version.to_string()` |
| `manipulation_nodes/motion_capture_ik/test/plant_ik_test.cpp` | `"/models/biped_s" + std::to_string(robot_version_int)` |
| `hardware_node/src/tests/motorTest/motor_follow_test.cpp` | `"kuavo_v" + std::to_string(v)` |
| `hardware_node/src/tests/motorTest/motor_follow_test_sim.cpp` | `"kuavo_v" + std::to_string(robot_version_)` |
| `hardware_node/src/tests/ankleTest/ankle_solver_v17_test.cpp` | `"kuavo_v" + std::to_string(robot_version_)` |
| `mobile_manipulator_controllers/test/deadlockDetectionTest.cpp` | 硬编码 `biped_s45` 路径 |
| `mobile_manipulator_controllers/test/mobileManipulatorControllerBaseUnitTest.cpp` | 硬编码 `biped_s45` 路径 |

#### Python 拼接路径的文件

| 文件 | 代码模式 |
|------|---------|
| `tools/setup-kuavo-ros-control.py` | `f"kuavo_v{robot_version}/kuavo.json"` |
| `tools/calibration_python/scripts/test_ik_fk.py` | `f"/models/biped_s{robot_version}"` 和 `f"/config/kuavo_v{robot_version}/kuavo.json"` |
| `tools/calibration_python/scripts/function/marker_utils.py` | `"/models/biped_s" + str(robot_version)` 和 `f"/config/kuavo_v{robot_version}/kuavo.json"` |
| `tools/get_joint_data/generate_cali_data.py` | `f"models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf"` |
| `tools/extract_camera_pose/example_camera_pose_from_bag.py` | 硬编码 `biped_s45` |
| `tools/extract_camera_pose/endeffector_pose_from_bag.py` | 硬编码 `biped_s49` |
| `tools/extract_camera_pose/kuavo_pose_calculator.py` | 硬编码 `biped_s45` |
| `tools/check_tool/kuavo_wheel_test/torso_motion_test.py` | `f"kuavo_v{robot_version}"` |
| `scripts/joint_cali/joint_cali_by_hard_limit.py` | `f"models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf"` |
| `scripts/joint_cali/arm_kinematics.py` | `f"models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf"`（3 处） |
| `scripts/joint_cali/joint_cali_ui.py` | `f"models/biped_s{robot_version}/urdf/..."` |
| `scripts/joint_cali/identifiability_analyzer.py` | `f"models/biped_s{robot_version}/urdf/..."` |
| `scripts/joint_cali/target_tracker.py` | `f"models/biped_s{robot_version}/urdf/..."` |
| `scripts/joint_cali/arm_cali.py` | `f"models/biped_s{robot_version}/urdf/..."`（2 处） |
| `scripts/joint_cali/head_cali.py` | `f"models/biped_s{robot_version}/urdf/..."` |
| `scripts/joint_cali/arm_cail_noui.py` | `f"models/biped_s{robot_version}/urdf/..."`（2 处） |
| `src/automatic_test/.../test_robot_walk.py` | `f"biped_s{os.environ.get('ROBOT_VERSION', '45')}"` |
| `src/demo/trace_path/scripts/.../mpc_path_tracer.py` | `f"biped_s{os.environ.get('ROBOT_VERSION', '45')}"` |
| `src/demo/trace_path/scripts/.../mpc_client_example.py` | `f"biped_s{robot_version}"` |
| `src/humanoid-control/humanoid_arm_control/scripts/BezierWrap.py` | `f"/models/biped_s{robot_version}/urdf/drake"` |
| `src/humanoid-control/humanoid_arm_control/scripts/arm_control_with_keyboard.py` | `f"/models/biped_s{robot_version}/urdf/drake/biped_v3_arm.urdf"` |
| `src/humanoid-control/humanoid_arm_control/scripts/motion_capture_ik_packaged/scripts/ik_ros_uni.py` | `f"/models/biped_s{robot_version}"` |
| `src/humanoid-control/humanoid_arm_control/scripts/motion_capture_ik_packaged/scripts/quest3_node.py` | `f"/models/biped_s{robot_version}"` |
| `src/humanoid-control/humanoid_arm_control/scripts/motion_capture_ik_packaged/scripts/visulize_traj.py` | `f"/models/biped_s{robot_version}"` |
| `src/humanoid-control/h12pro_controller_node/robot_state/rl_before_callback.py` | `f"kuavo_v{robot_version}"` |
| `src/humanoid-control/h12pro_controller_node/robot_state/multi_before_callback.py` | `f"kuavo_v{robot_version}"`（2 处） |
| `src/humanoid-control/h12pro_controller_node/robot_state/ocs2_before_callback.py` | `f"kuavo_v{robot_version}"`（2 处） |
| `src/manipulation_nodes/motion_capture_ik/scripts/ik_ros_uni.py` | `f"/models/biped_s{robot_version}"` 和 `f"/config/kuavo_v{robot_version}"` |
| `src/manipulation_nodes/motion_capture_ik/scripts/quest3_node.py` | `f"/models/biped_s{robot_version}"` 和 `f"/config/kuavo_v{robot_version}"` |
| `src/manipulation_nodes/motion_capture_ik/scripts/quest3_node_incremental.py` | `f"/config/kuavo_v{robot_version}"` |
| `src/manipulation_nodes/motion_capture_ik/scripts/visulize_traj.py` | `f"/models/biped_s{robot_version}"` 和 `f"/config/kuavo_v{robot_version}"` |
| `src/manipulation_nodes/motion_capture_ik/scripts/ik/torso_ik.py` | `f"/models/biped_s{robot_version}/urdf/drake/biped_v3_arm.urdf"` |
| `src/manipulation_nodes/pico-body-tracking-server/scripts/core/ros/pico.py` | `f"/models/biped_s{robot_version}"` 和 `f"/config/kuavo_v{robot_version}"` |
| `src/manipulation_nodes/noitom_hi5_hand_udp_python/scripts/monitor_quest3.py` | `f"/config/kuavo_v{robot_version}"` |
| `src/manipulation_nodes/noitom_hi5_hand_udp_python/scripts/robot_state_server.py` | `f"config/kuavo_v{robot_version}"` |
| `src/manipulation_nodes/planarmwebsocketservice/scripts/handler.py` | `f"kuavo_v{robot_version_str}/kuavo.json"`（2 处） |
| `src/kuavo_assets/scripts/trim_mesh_to_ik.py` | 扫描 `biped_s*` 目录的正则匹配 |
| `src/kuavo-isaac-sim/nio-isaac/nio/env/env_base.py` | `f"models/biped_s{version}/urdf/biped_s{version}.usd"` |

#### Shell 拼接路径的文件

| 文件 | 代码模式 |
|------|---------|
| `tools/setup-kuavo-ros-control.sh` | `kuavo_v${ROBOT_VERSION}/kuavo.json`，`TotalMassV${ROBOT_VERSION}` |
| `tools/check_tool/change_EndEffectorType.sh` | `kuavo_v${ROBOT_VERSION}/kuavo.json`（2 处） |
| `src/kuavo_assets/scripts/update_mass.sh` | `biped_s${V}/urdf`，`biped_s${V}.xml`，`TotalMassV${V}`，`.urdf_md5_v${V}`，`/var/ocs2/kuavo_v${V}` |
| `scripts/leju_start.sh` | `TotalMassV$ROBOT_VERSION` |
| `ci_scripts/automatic_tests/grab_box_sim/grab_box_sim_test.sh` | `kuavo_v$ROBOT_VERSION/mpc/task.info` |

---

## 场景 2：版本判断 / 条件分支

根据版本号执行不同逻辑，是**第二大使用场景（约 25%）**。

### 2.1 Launch 中的数值比较

| 比较表达式 | 语义 | 使用文件 |
|-----------|------|---------|
| `arg('robot_version') >= 40` | Kuavo 系列 | load_kuavo_real.launch, load_kuavo_gazebo_sim.launch |
| `arg('robot_version') >= 50` | Kuavo 5 系列 | mobile_manipulator_controller.launch |
| `arg('robot_version') >= 30` | 支持 mobile manipulator | load_kuavo_real.launch, load_kuavo_mujoco_sim.launch, load_kuavo_isaac_sim.launch |
| `arg('robot_version') == 15` | 使用 v14 资源（别名） | robot_version_manager.launch, mujoco_sim.launch, nodelet.launch, nodelet_with_arm.launch, visualize.launch, half_body_arm_fk_ik.launch |
| `arg('robot_version') == 60` | 轮式桥接 | load_kuavo_real_wheel.launch |
| `arg('robot_version') != 53` | 排除 v53（音乐服务冲突） | load_kuavo_real.launch |
| `60 <= int(arg('robot_version')) % 100 < 70` | 轮式类型检测 | gazebo-sim.launch, mujoco_sim.launch, nodelet_with_arm.launch |
| `int(arg('robot_version')) >= 40` | use_xacro, use_kuavo 标志 | gazebo-sim.launch, mujoco_sim.launch |
| `60 <= arg('robot_version') and arg('robot_version') < 70` | 排除轮式（手臂规划） | humanoid_plan_arm_trajectory.launch |
| `52 <= (arg('ROBOT_VERSION') % 100) <= 59` | LED Strip 新版本 | set_led_mode.launch |
| 链式 `>=50 → kuavo5; >=40 → kuavo; >=10 → roban2` | 分层配置选择 | mobile_manipulator_controller*.launch (3 个) |

**典型代码**：

```xml
<!-- 轮式类型检测 -->
<arg name="robot_type" default="$(eval '1' if 60 &lt;= int(arg('robot_version')) % 100 &lt; 70 else '2')"/>

<!-- Kuavo 系列标志 -->
<arg name="use_kuavo" value="$(eval int(arg('robot_version')) >= 40)"/>

<!-- 排除 v53 音乐服务 -->
<group if="$(eval arg('music') != 'disable' and arg('robot_version') != 53)">

<!-- LED 版本范围检测 -->
<group if="$(eval 52 &lt;= (arg('ROBOT_VERSION') % 100) &lt;= 59)">

<!-- 版本分层配置选择 -->
<arg name="taskFile" default="$(eval find('ocs2_mobile_manipulator') +(
    '/config/kuavo5/task.info'  if arg('robot_version') >= 50 else
    '/config/kuavo/task.info'   if arg('robot_version') >= 40 else
    '/config/roban2/task.info'  if arg('robot_version') >= 10 else
    '/config/kuavo/task.info'
))"/>
```

### 2.2 C++ 中的版本判断

#### 现有 RobotVersion 类（`src/kuavo_common/include/kuavo_common/common/common.h`）

```cpp
class RobotVersion {
    uint16_t Major, Minor, Patch;
    static RobotVersion create(int big_number);  // 45 → Major=4, Minor=5, Patch=0
    bool start_with(uint16_t major) const;
    bool start_with(uint16_t major, uint16_t minor) const;
    // 比较运算符: ==, !=, <
};
```

#### 精简定义（`humanoid_interface_drake/include/.../common.h`）

```cpp
struct RobotVersion {
    u_int8_t Major, Minor;
    std::string versionName();  // → "4.5"
    int versionInt();           // → 45
};
```

#### 宏定义

```cpp
#define IS_KUAVO_LEGGED(rb)     (rb.major() == 4 || rb.major() == 5)
#define IS_KUAVO4_LEGGED(rb)    (rb.major() == 4)
#define IS_KUAVO5_LEGGED(rb)    (rb.major() == 5)
#define IS_KUAVO4PRO_LEGGED(rb) (rb.major() == 4 && (rb.minor() >= 5))
#define IS_ROBAN_LEGGED(rb)     (rb.major() == 1)
#define IS_ROBAN2_LEGGED(rb)    (rb.major() == 1)
#define IS_KUAVO_WHEELED(rb)    (rb.major() == 6)
#define IS_KUAVO(rb)            (IS_KUAVO_LEGGED(rb) || IS_KUAVO_WHEELED(rb))
#define IS_ROBAN(rb)            (IS_ROBAN_LEGGED(rb))
```

#### 使用方式

| 模式 | 示例文件 |
|------|---------|
| `RobotVersion::create(int)` + 宏 | humanoidController.cpp, playBackNodelet.cpp, TargetTrajectoriesPublisher.cpp, VMPController.cpp |
| `nh.getParam("/robot_version", int)` + 整数比较 | AmpWalkController.cpp, DepthWalkController.cpp, VMPController.cpp, SwitchedModelReferenceManager.cpp |
| `std::getenv("ROBOT_VERSION")` + `stoi` | arm_collision_checker.cpp, actuators_interface.cpp, motor_follow_test.cpp, ankle_solver_v17_test.cpp |
| `std::getenv("ROBOT_VERSION")` + 字符串比较 | arm_collision_checker.cpp（set 查找）, actuators_interface.cpp (`== "53"`, `== "54"`), ruiwo_actuator.cpp (`[0] == '1'`) |
| `robot_version < 30` → is_roban | SwitchedModelReferenceManager.cpp |
| `rb_version.major() != 6` | humanoid_interface_drake.cpp（排除轮式） |
| `rb_version_.major() <= 2`、`!= 1`、`== 1` | HumanoidJoyCommandNodeWithVel.cpp, HumanoidAutoGaitJoyCommandNodeWithVel.cpp（多处 Roban 判断） |
| `rb_version.start_with(1)` | hardware_node.cc（3 处） |
| `rb_version.start_with(1, 5)`、`start_with(1, 6)` | hardware_plant.cc |
| `rb_version.minor() <= 6` | hardware_plant.cc |
| `rb_version.major() == 5 && rb_version.minor() >= 3` | actuators_interface.cpp（EtherCAT CPU 亲和性） |
| `robot_version >= 40`、`>= 10 && < 30` | humanoid_plan_arm_trajectory.cpp |
| `robot_version >= 50`（has_waist） | bezier_curve_interpolator.cpp |
| `robot_version >= 50` | motor_follow_test_core.cpp（2 处） |
| `robot_version == 40`、`== 45` | isaac_sim_system.cpp（6 处，关节配置映射） |
| `robot_version == 42` | tagTracker.cpp |
| `robot_version == 15` → 映射为 14 | arms_ik_node.cpp |
| `int / 10`、`% 10` 整数运算 | AmpWalkController.cpp, DepthWalkController.cpp, ankle_solver_v17_test.cpp |

#### 关键文件明细

**arm_collision_checker.cpp** — 环境变量读取 + set 查找 + 路径拼接：
```cpp
char* robot_version_ = std::getenv("ROBOT_VERSION");
std::string robot_version = std::string(robot_version_);
if(dexterous_hand_versions.count(robot_version) > 0) { /* 添加额外碰撞链接 */ }
std::string urdf_file_path = kuavo_asset_path + "/models/biped_s" + robot_version + "/urdf/...";
std::string kuavo_json_path = kuavo_asset_path + "/config/kuavo_v" + robot_version + "/kuavo.json";
std::string mesh_path = kuavo_asset_path + "/models/biped_s" + robot_version + "/meshes/" + filename;
```

**actuators_interface.cpp** — 特定版本硬件配置：
```cpp
bool is_robot_version_53 = (robot_version == "53");
bool is_robot_version_54 = (robot_version == "54");
std::string ec_cpu_affinity = (rb_version.major() == 5 && rb_version.minor() >= 3) ? "4" : "7";
```

**ruiwo_actuator.cpp** — 首字符判断：
```cpp
bool is_roban1_series = (robot_version_env != nullptr && std::string(robot_version_env)[0] == '1');
```

**HumanoidJoyCommandNodeWithVel.cpp / HumanoidAutoGaitJoyCommandNodeWithVel.cpp** — 多处 major() 判断：
```cpp
if(rb_version_.major() <= 2) { /* Roban 运动限制 */ }
if(rb_version_.major() != 1) { /* 非 Roban 特有逻辑 */ }
if (is_rl_controller_ && rb_version_.major() == 1) { /* Roban RL 特殊处理 */ }
```

**isaac_sim_system.cpp** — 精确版本配置映射（6 处）：
```cpp
if (robot_version_ == 40) { /* Kuavo 4 短臂配置 */ }
else if (robot_version_ == 45) { /* Kuavo 4 Pro 长臂配置 */ }
```

**MobileManipulatorJoyCommandNode.cpp** — 环境变量 + ROS 参数双读取 + G12 轮臂模式检测：
```cpp
nodeHandle_.getParam("robot_version", robotVersion_);
const char* env_robot_version = std::getenv("ROBOT_VERSION");
int env_version = std::atoi(env_robot_version);
// G12轮臂模式: ROBOT_VERSION>=60 且 joystick_type==h12
```

**humanoid_plan_arm_trajectory.cpp** — 品牌/代次判断：
```cpp
if (robot_version_ >= 40) { /* Kuavo 系列 */ }
else if (robot_version_ >= 10 && robot_version_ < 30) { /* Roban 系列 */ }
```

**bezier_curve_interpolator.cpp** — 物理能力判断：
```cpp
bool has_waist = (robot_version_ >= 50);
```

**kuavo_settings.cpp** — 疑似 bug：
```cpp
if (rb_version.major() > 1 && rb_version.major() < 2) { /* 不可能同时满足 */ }
```

### 2.3 Python 中的版本判断

#### 主版本提取模式

```python
# src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo/core/ros/param.py
robot_version_major = (int(robot_version) // 10) % 10
# 结果: 45→4, 52→5, 14→1, 60→6

# src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo/core/core.py（展厅版兼容）
self._robot_version_major = (int(self._rb_info['robot_version']) // 10) % 10000
```

#### 使用方式

| 判断条件 | 语义 | 示例文件 |
|---------|------|---------|
| `robot_version_major == 1` | Roban 系列（关节配置不同） | param.py, core.py, robot_info.py（两套 SDK 共 6 个文件） |
| `robot_version_major == 4` | Kuavo 4 系列 | param.py |
| `robot_version_major == 5` | Kuavo 5 系列（有腰部关节） | param.py |
| `robot_version_major == 6` | 轮式系列 | robot_info.py |
| `robot_version_major == 4 or 5` | Kuavo 系列（速度限制） | core.py |
| `robot_version_major == 1 and link_name.startswith('zarm_')` | Roban 特定关节名过滤 | param.py |
| `robot_version_major == 1 or arm_dof == 8` | Roban 或 8 自由度臂 | param.py |

#### 关键文件明细

**param.py** — 关节配置分支（两套 SDK 各一份）：
```python
robot_version_major = (int(kuavo_ros_param.robot_version()) // 10) % 10

if robot_version_major == 1:  # ROBAN
    leg_link_names = [12 joints]
    waist_link_names = ['waist_yaw_link']
    arm_link_names = [4 joints]  # 4-DOF 手臂
    head_link_names = [2 joints]
elif robot_version_major == 4:  # KUAVO-4
    leg_link_names = [12 joints]
    waist_link_names = []        # 无腰部关节
    arm_link_names = [7 joints]  # 7-DOF 手臂
elif robot_version_major == 5:  # KUAVO-5
    leg_link_names = [12 joints]
    waist_link_names = ['waist_yaw_link']  # 有腰部关节
    arm_link_names = [7 joints]
```

**core.py** — 运动限制参数（两套 SDK 各一份）：
```python
if self._robot_version_major == 1:    # ROBAN
    MAX_LINEAR_X = 0.3; MAX_LINEAR_Y = 0.2; MAX_ANGULAR_Z = 0.3
elif self._robot_version_major == 4 or self._robot_version_major == 5:  # KUAVO
    MAX_LINEAR_X = 0.4; MAX_LINEAR_Y = 0.2; MAX_ANGULAR_Z = 0.4
```

**robot_info.py** — 机器人类型判断（两套 SDK 各一份）：
```python
self._robot_version_major = (int(self._robot_version) // 10) % 10
if self._robot_version_major == 1:
    # Roban 类型设置
if self._robot_version_major == 6:
    # 轮式机器人：轮臂关节索引
```

### 2.4 Shell 中的版本判断

| 判断模式 | 语义 | 示例文件 |
|---------|------|---------|
| `${ROBOT_VERSION:0:2}` + regex | 前缀提取 + 系列判断 | stress_test_all_cores.sh, stress_test_normal_cores.sh |
| `[[ "$ROBOT_VERSION" =~ ^1[0-9]$ ]]` | Roban 系列 | deploy_autostart.sh (joy), deploy_autostart.sh (h12pro), check_ecmaster_type.sh |
| `[[ "$ROBOT_VERSION" =~ ^5[0-9]$ ]]` | Kuavo 5 系列 | stress_test_*.sh |
| `[[ "$ROBOT_VERSION" =~ ^6[0-9]$ ]]` | 轮式系列 | stress_test_*.sh |
| `[[ "$ROBOT_VERSION" == 1* ]]` | Roban 首字符 | setup-kuavo-ros-control.sh |
| `[ "${ROBOT_VERSION}" != "60" ]` | 精确匹配 | start_wheel_bridge.sh |
| `[ "$ROBOT_VERSION" == "42" ]` | 精确匹配 | check_ecmaster_type.sh |
| `$ROBOT_VERSION -gt 50 && -lt 60` | 数值范围 | install_log_uploader.sh |
| `allowed_versions` 白名单 | 版本验证 | update_mass.sh |
| `== "14"`, `== "15"`, `== "17"` | 精确匹配（base_link 选择） | update_mass.sh |
| `== "52"`, `== "53"`, `== "54"`, `== "55"` | 精确匹配（waist_yaw_link） | update_mass.sh |
| `== "60"`, `== "61"`, `== "62"`, `== "63"` | 轮式版本跳过质量更新 | update_mass.sh |
| `== "15"` → 映射为 14 | 版本别名 | update_mass.sh |

**版本白名单**（`update_mass.sh`）：
```bash
allowed_versions=("11" "13" "14" "15" "16" "17" "40" "41" "42" "43" "45" "46" "47" "48" "49" "50" "51" "52" "53" "54" "55" "60" "61" "62" "63" "100045" "100049" "200049" "300049" "400049")
```

---

## 场景 3：CMake 编译时定义

仅 1 个文件：`src/kuavo-ros-control-lejulib/hardware_node/CMakeLists.txt`

```cmake
if(NOT DEFINED ROBOT_VERSION)
    if (NOT DEFINED ENV{ROBOT_VERSION})
        set(ROBOT_VERSION 32)  # 默认值
    else()
        set(ROBOT_VERSION $ENV{ROBOT_VERSION})
    endif()
endif()

add_compile_definitions(ROBOT_VERSION_INT=${ROBOT_VERSION})
```

**经验证：`ROBOT_VERSION_INT` 是死代码**——CMake 定义了该编译宏，但整个代码库中没有任何 C++ 文件使用它。迁移时可直接删除这段 CMake 代码。无编译时 `#if`/`#ifdef` 版本检查，版本判断完全在运行时。

---

## 场景 4：ROS 参数服务器

### 写入方式

Launch 文件设置：
```xml
<param name="robot_version" value="$(arg robot_version)"/>  <!-- 当前为 int -->
```

根级别 `robot_setup.launch`：
```xml
<param name="robot_version" type="string" value="$(env ROBOT_VERSION)" />
```

### 读取方式

| 语言 | 读取方式 | 示例文件 |
|------|---------|---------|
| C++ | `nh.param("/robot_version", robot_version_int, 45)` | AmpWalkController.cpp, DepthWalkController.cpp, VMPController.cpp |
| C++ | `ros::param::get("/robot_version", robot_version_int)` | tagTracker.cpp |
| C++ | `controllerNh_.getParam("/robot_version", rb_version_int)` | humanoidController.cpp |
| C++ | `nodeHandle.getParam("/robot_version", rb_version_int)` | 12 个 humanoid_interface_ros 节点文件 |
| C++ | `nh_.getParam("robot_version", robotVersion_)` | MobileManipulatorJoyCommandNode.cpp |
| C++ | `nh->getParam("robot_version", robot_version_)` | humanoid_plan_arm_trajectory.cpp |
| Python | `rospy.get_param('/robot_version')` | param.py (SDK) |
| Python | `roslibpy.Param(client, 'robot_version').get()` | param.py (WebSocket SDK) |

**注意**：所有 C++ 读取方都假设 `/robot_version` 是 **int 类型**。

---

## 场景 5：环境变量直接读取

部分代码直接读取 `ROBOT_VERSION` 环境变量（不通过 ROS 参数服务器）。

### C++ 直接读环境变量的文件

| 文件 | 读取后处理 |
|------|-----------|
| `arm_collision_checker.cpp` | `std::getenv` → string，set 查找 + 路径拼接 |
| `actuators_interface.cpp` | `std::getenv` → string 比较 (`== "53"`, `== "54"`) |
| `motor_follow_test.cpp` | `std::getenv` → `stoi` → int 比较 |
| `ankle_solver_v17_test.cpp` | `std::getenv` → `stoi` → int 比较 |
| `ruiwo_actuator.cpp` | `std::getenv` → 首字符检查 (`[0] == '1'`) |
| `quick_start_example.cpp` | `std::getenv` → 传入其他函数 |
| `test_humanoid_interface_drake.cpp` | `std::getenv` → `stoi` → `RobotVersion::create()` |
| `MobileManipulatorJoyCommandNode.cpp` | `std::getenv` → `atoi` → G12 轮臂模式检测 |

### Python 直接读环境变量的文件

| 文件 | 读取模式 |
|------|---------|
| `tools/check_tool/kuavo_wheel_test/torso_motion_test.py` | `os.getenv("ROBOT_VERSION", 60)` |
| `tools/check_tool/Hardware_tool.py` | 解析 `~/.bashrc` 中的 `export ROBOT_VERSION=` |
| `tools/calibration_python/scripts/test_ik_fk.py` | `os.environ.get('ROBOT_VERSION', '40')` |
| `tools/get_joint_data/generate_cali_data.py` | `os.environ.get('ROBOT_VERSION')` |
| `scripts/joint_cali/joint_cali_by_hard_limit.py` | `os.environ.get('ROBOT_VERSION', '40')` |
| `scripts/joint_cali/arm_kinematics.py` | `os.environ.get('ROBOT_VERSION', '40')`（3 处） |
| `scripts/joint_cali/joint_cali_ui.py` | 通过参数传入 |
| `scripts/joint_cali/identifiability_analyzer.py` | 通过参数传入 |
| `scripts/joint_cali/target_tracker.py` | 通过参数传入 |
| `scripts/joint_cali/arm_cali.py` | 通过参数传入 |
| `scripts/joint_cali/head_cali.py` | 通过参数传入 |
| `scripts/joint_cali/arm_cail_noui.py` | 通过参数传入 |
| `src/automatic_test/.../test_robot_walk.py` | `os.environ.get('ROBOT_VERSION', '45')` |
| `src/demo/trace_path/scripts/.../mpc_path_tracer.py` | `os.environ.get('ROBOT_VERSION', '45')` |
| `src/humanoid-control/humanoid_arm_control/scripts/BezierWrap.py` | `os.environ.get('ROBOT_VERSION', '40')` |
| `src/humanoid-control/h12pro_controller_node/robot_state/rl_before_callback.py` | `os.environ.get('ROBOT_VERSION')` |
| `src/humanoid-control/h12pro_controller_node/robot_state/multi_before_callback.py` | `os.environ.get('ROBOT_VERSION')` |
| `src/humanoid-control/h12pro_controller_node/robot_state/ocs2_before_callback.py` | `os.environ.get('ROBOT_VERSION')` |
| `tools/setup-kuavo-ros-control.py` | `os.environ.get('ROBOT_VERSION')` |
| `src/demo/examples_code/action_scripts/factory_reset_action.py` | 通过 `ROBOT_VERSION_MAP` 字典映射 |

### Shell 直接读环境变量的文件

| 文件 | 主要用法 |
|------|---------|
| `tools/setup-kuavo-ros-control.sh` | 用户交互设置、路径拼接、Roban 判断 |
| `src/kuavo_assets/scripts/update_mass.sh` | 白名单验证、路径拼接、版本别名 |
| `tools/check_tool/stress_test/stress_test_all_cores.sh` | 前缀提取 + 温度阈值 |
| `tools/check_tool/stress_test/stress_test_normal_cores.sh` | 前缀提取 + 温度阈值 |
| `tools/check_tool/change_EndEffectorType.sh` | 路径拼接 |
| `src/humanoid-control/h12pro_controller_node/scripts/deploy_autostart.sh` | Roban 判断、service 文件注入 |
| `src/humanoid-control/joystick_drivers/joy/services/deploy_autostart.sh` | Roban 判断、service 文件注入 |
| `src/kuavo_wheel/scripts/start_wheel_bridge.sh` | `!= "60"` 精确匹配 |
| `src/kuavo_assets/scripts/check_ecmaster_type.sh` | `== "42"` 精确匹配、Roban 正则 |
| `scripts/leju_start.sh` | 启动流程、TotalMass 路径 |
| `scripts/kuavo_environment_setup.sh` | `export ROBOT_VERSION=45` 默认值 |
| `tools/upload_log/install_log_uploader.sh` | 数值范围比较（51-59） |
| `src/demo/examples_code/action_scripts/kuavo_tools.sh` | 环境变量显示 |
| `src/manipulation_nodes/planarmwebsocketservice/service/websocket_deploy_script.sh` | sed 替换 service 文件 |
| CI 脚本（3 个） | `export ROBOT_VERSION=` 固定值 |

---

## 场景 6：Docker / Service / CI 配置

### 6.1 Docker 配置

| 文件 | 内容 |
|------|------|
| `docker/Dockerfile` | `echo 'export ROBOT_VERSION=42' >> /root/.bashrc` |
| `docker/run.sh` | `-e ROBOT_VERSION=42` |
| `docker/run_with_gpu.sh` | `-e ROBOT_VERSION=42` |
| `scripts/joint_cali/arm_cali_in_docker.sh` | `-e ROBOT_VERSION=$ROBOT_VERSION`（透传） |

### 6.2 systemd Service 文件

| 文件 | 内容 |
|------|------|
| `joystick_drivers/joy/services/roban_joy_monitor.service` | `Environment=ROBOT_VERSION=14` |
| `h12pro_controller_node/services/ocs2_h12pro_node.service` | `Environment=ROBOT_VERSION=40` |
| `h12pro_controller_node/services/ocs2_h12pro_monitor.service` | `Environment=ROBOT_VERSION=40` |
| `planarmwebsocketservice/service/websocket_start.service` | `Environment=ROBOT_VERSION=42` |

这些 service 文件中的 `ROBOT_VERSION` 值会被部署脚本（`deploy_autostart.sh`、`websocket_deploy_script.sh`）通过 `sed` 替换为实际值。

### 6.3 CI 配置

| 文件 | 内容 |
|------|------|
| `.gitlab-ci.yml` | 多处 `export ROBOT_VERSION=34` 和 `export ROBOT_VERSION=40`，分别用于 Roban 和 Kuavo 构建/测试 |

---

## 场景 7：特殊情况

### 7.1 版本 15 → 14 别名

版本 15 使用版本 14 的全部资源文件。在多个文件中有处理：

**Launch**（`robot_version_manager.launch` + 5 个其他 launch）：
```xml
<arg name="urdfFile" default="$(find kuavo_assets)/models/biped_s14/urdf/biped_s14.urdf"
     if="$(eval arg('robot_version') == 15)"/>
<arg name="urdfFile" default="$(find kuavo_assets)/models/biped_s$(arg robot_version)/urdf/..."
     unless="$(eval arg('robot_version') == 15)"/>
```

**C++**（`arms_ik_node.cpp`）：
```cpp
if (rb_version_int == 15) { rb_version_int = 14; }
```

**Shell**（`update_mass.sh`）：
```bash
if [ "${ROBOT_VERSION}" = "15" ]; then ACTUAL_ROBOT_VERSION="14"; fi
```

### 7.2 展厅版 / Patch 版本

- `100045` → Kuavo 4 Pro 展厅版（通过 `% 10000` 获取基础版本 45）
- `100049` → Kuavo 4 Pro EDU 展厅版
- `200049` → Kuavo 4 Pro EDU claw 版
- `300049`、`400049` → 其他特殊变体
- `RobotVersion::create(100045)` → Major=4, Minor=5, Patch=1
- core.py 使用 `% 10000` 而非 `% 10` 来兼容展厅版

### 7.3 版本号数字映射

`factory_reset_action.py` 中有一个完整映射字典：

```python
ROBOT_VERSION_MAP = {
    "40": "KUAVO-4-001",
    "41": "KUAVO-4-002",
    "42": "KUAVO-4-003",
    "45": "KUAVO-4PRO-001",
    "49": "KUAVO-4PRO-EDU-001",
    # ...
}
```

### 7.4 版本 52-54 特殊链接命名

`update_mass.sh` 中对版本 52、53、54、55 使用 `waist_yaw_link`，其他版本使用 `base_link`。

### 7.5 `setup-kuavo-ros-control.sh` 版本转换

用户可输入小数格式，脚本做转换：
```bash
if [[ "$version" == "45.1" ]]; then version=100045; fi
if [[ "$version" == "49.1" ]]; then version=100049; fi
```

### 7.6 轮式版本跳过质量更新

```bash
if [ "${ROBOT_VERSION}" = "60" ] || [ "${ROBOT_VERSION}" = "61" ] || [ "${ROBOT_VERSION}" = "62" ] || [ "${ROBOT_VERSION}" = "63" ]; then
    echo "Skip mass update for robot version ${ROBOT_VERSION}" >&2
```

### 7.7 LED 版本范围

`set_led_mode.launch` 对版本 52-59 使用新 LED Strip 服务，其他版本使用旧服务。

### 7.8 版本兼容回退

`motor_follow_test.cpp` 中有版本兼容回退逻辑——当找不到当前版本配置时，回退到最近的已知版本配置。

### 7.9 G12 轮臂模式

`MobileManipulatorJoyCommandNode.cpp` 中同时读取 ROS 参数和环境变量来检测 G12 轮臂模式（`ROBOT_VERSION>=60` 且 `joystick_type==h12`）。

### 7.10 v53 音乐服务排除

`load_kuavo_real.launch` 中排除 v53 的音乐服务（与上位机冲突）：
```xml
<group if="$(eval arg('music') != 'disable' and arg('robot_version') != 53)">
```

### 7.11 Protobuf 定义

`robot_info.pb.cc/h` 中有 protobuf 自动生成的 `robot_version` 字段处理代码，不需要手动迁移。

### 7.12 Hardware_tool.py 特殊读取

该工具通过解析 `~/.bashrc` 文件中的 `export ROBOT_VERSION=` 来获取版本号，而非直接读取环境变量：
```python
if line.startswith('export ROBOT_VERSION='):
    robot_version = line.split('=')[1].strip()
```

### 7.13 硬编码版本的 Launch 文件

部分 Launch 文件硬编码了特定版本而非使用动态变量：

| 文件 | 硬编码版本 |
|------|-----------|
| `load_kuavo_mujoco_sim_fixed.launch` | `robot_version=46`（固定版本） |
| `load_kuavo_mujoco_sim_minimal.launch` | `default="46"`（默认 46） |
| `humanoid_arm_trajectort_rviz.launch` | `default="42"`（默认 42） |
| `wheel_controller_mpc_test.launch` | `kuavo_s60/task.info`（硬编码 60） |
| `load_kuavo_gazebo_sim_wheel.launch` | `kuavo_v45` 路径（硬编码 45） |
| `load_kuavo_real_wheel.launch` | `kuavo_v45` 路径（硬编码 45） |

---

## 涉及文件完整清单

### Launch 文件（~55 个）

**控制器核心**：
1. `robot_setup.launch`（根级别）
2. `src/humanoid-control/humanoid_controllers/launch/robot_version_manager.launch` **[中央枢纽]**
3. `src/humanoid-control/humanoid_controllers/launch/load_kuavo_real.launch`
4. `src/humanoid-control/humanoid_controllers/launch/load_kuavo_gazebo_sim.launch`
5. `src/humanoid-control/humanoid_controllers/launch/load_kuavo_mujoco_sim.launch`
6. `src/humanoid-control/humanoid_controllers/launch/load_kuavo_isaac_sim.launch`
7. `src/humanoid-control/humanoid_controllers/launch/load_kuavo_real_wheel.launch`
8. `src/humanoid-control/humanoid_controllers/launch/load_kuavo_mujoco_sim_wheel.launch`
9. `src/humanoid-control/humanoid_controllers/launch/load_kuavo_gazebo_sim_wheel.launch`
10. `src/humanoid-control/humanoid_controllers/launch/load_kuavo_mujoco_sim_fixed.launch`
11. `src/humanoid-control/humanoid_controllers/launch/load_kuavo_mujoco_sim_minimal.launch`
12. `src/humanoid-control/humanoid_controllers/launch/load_kuavo_gazebo_manipulate.launch`
13. `src/humanoid-control/humanoid_controllers/launch/play_back.launch`
14. `src/humanoid-control/humanoid_controllers/launch/play_back_mpc.launch`
15. `src/humanoid-control/humanoid_controllers/launch/wheel_controller_mpc_test.launch`

**接口**：
16. `src/humanoid-control/humanoid_interface_ros/launch/humanoid_ddp.launch`
17. `src/humanoid-control/humanoid_interface_ros/launch/humanoid_sqp.launch`

**手臂规划**：
18. `src/humanoid-control/humanoid_plan_arm_trajectory/launch/humanoid_plan_arm_trajectory.launch`
19. `src/humanoid-control/humanoid_plan_arm_trajectory/launch/humanoid_arm_trajectort_rviz.launch`

**Mobile Manipulator**：
20. `src/humanoid-control/mobile_manipulator_controllers/launch/mobile_manipulator_controller.launch`
21. `src/humanoid-control/mobile_manipulator_controllers/launch/mobile_manipulator_controller_base.launch`
22. `src/humanoid-control/mobile_manipulator_controllers/launch/mobile_manipulator_controller_general.launch`

**Motion Capture IK**：
23. `src/humanoid-control/humanoid_arm_control/scripts/motion_capture_ik_packaged/launch/visualize.launch`
24. `src/manipulation_nodes/motion_capture_ik/launch/ik_node.launch`
25. `src/manipulation_nodes/motion_capture_ik/launch/visualize.launch`
26. `src/manipulation_nodes/planarmwebsocketservice/launch/plan_arm_action_websocket_server.launch`

**轮式控制**：
27. `src/humanoid-wheel-control/humanoid_wheel_interface_ros/launch/manipulator_kuavo_s60.launch`
28. `src/humanoid-wheel-control/humanoid_wheel_interface_ros/launch/manipulator_kuavo_s60_sqp.launch`
29. `src/humanoid-wheel-control/humanoid_wheel_interface_ros/launch/chassis_direct_control.launch`

**硬件测试**：
30. `src/kuavo-ros-control-lejulib/hardware_node/launch/hardware_test.launch`
31. `src/kuavo-ros-control-lejulib/hardware_node/launch/hardware_testRoban.launch`
32. `src/kuavo-ros-control-lejulib/hardware_node/launch/hardware_roban_test.launch`
33. `src/kuavo-ros-control-lejulib/hardware_node/launch/hardwareSelfCheck.launch`
34. `src/kuavo-ros-control-lejulib/hardware_node/launch/ankle_solver_v17_test.launch`
35. `src/kuavo-ros-control-lejulib/hardware_node/launch/motor_follow_test.launch`

**碰撞检测**：
36. `src/kuavo_arm_collision_check/launch/arm_collision_check.launch`

**仿真**：
37. `src/gazebo/gazebo-sim/launch/gazebo-sim.launch`
38. `src/gazebo/gazebo-sim/launch/gazebo-sim-ground-tags.launch`
39. `src/gazebo/gazebo-sim/launch/gazebo-sim-grab-box-ci.launch`
40. `src/mujoco/launch/mujoco_sim.launch`
41. `src/mujoco/launch/nodelet.launch`
42. `src/mujoco/launch/nodelet_with_arm.launch`

**Isaac Sim**：
43. `src/kuavo_isaac_sim/controller_tcp/launch/isaac_sim_nodelet_no_tcp.launch`

**自动测试**：
44. `src/automatic_test/automatic_test/launch/half_body_arm_fk_ik.launch`

**OCS2**：
45. `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/manipulator_kuavo.launch`

**LED**：
46. `src/kuavo_led/kuavo_led_controller/launch/set_led_mode.launch`

### C++ 文件（~87 个，主要列出）

**版本类定义**：
- `src/kuavo_common/include/kuavo_common/common/common.h` **[RobotVersion 类 + 宏]**
- `src/kuavo_common/src/common/common.cpp`
- `src/humanoid-control/humanoid_interface_drake/include/.../common.h` **[精简版 RobotVersion]**

**核心控制器**：
- `src/humanoid-control/humanoid_controllers/src/humanoidController.cpp`
- `src/humanoid-control/humanoid_controllers/src/humanoidController_wheel_wbc.cpp`
- `src/humanoid-control/humanoid_controllers/src/rl/AmpWalkController.cpp`
- `src/humanoid-control/humanoid_controllers/src/rl/DepthWalkController.cpp`
- `src/humanoid-control/humanoid_controllers/src/rl/VMPController.cpp`
- `src/humanoid-control/humanoid_controllers/src/TargetTrajectoriesPublisher.cpp`
- `src/humanoid-control/humanoid_controllers/src/playBackNodelet.cpp`
- `src/humanoid-control/humanoid_interface/src/reference_manager/SwitchedModelReferenceManager.cpp`

**接口节点**（12 个 humanoid_interface_ros 文件）：
- `HumanoidDdpMpcNode.cpp`, `HumanoidSqpMpcNode.cpp`, `HumanoidDummyNode.cpp`
- `HumanoidHandCommandNode.cpp`, `HumanoidVRHandCommandNode.cpp`
- `HumanoidJoyCommandNode.cpp`, `HumanoidJoyCommandNodeWithArm.cpp`
- `HumanoidPoseCommandNode.cpp`, `HumanoidPoseCommandNodeWithArm.cpp`
- `HumanoidAutoGaitJoyCommandNodeWithVel.cpp`（含多处 major() 判断）
- `HumanoidJoyCommandNodeWithVel.cpp`（含多处 major() 判断）
- `QuestControlFSMNode.cpp`

**手臂规划/IK**：
- `src/humanoid-control/humanoid_plan_arm_trajectory/src/humanoid_plan_arm_trajectory.cpp`
- `src/humanoid-control/humanoid_plan_arm_trajectory/src/bezier_curve_interpolator.cpp`
- `src/manipulation_nodes/motion_capture_ik/src/arms_ik_node.cpp`（含 v15→v14 映射）
- `src/manipulation_nodes/motion_capture_ik/src/main_node.cpp`
- `src/manipulation_nodes/motion_capture_ik/src/ik_ros_uni_cpp_node.cpp`
- `src/manipulation_nodes/motion_capture_ik/src/wheel_ik_ros_uni_cpp_node.cpp`

**硬件层**：
- `src/kuavo-ros-control-lejulib/hardware_plant/src/actuators_interface.cpp`
- `src/kuavo-ros-control-lejulib/hardware_plant/src/hardware_plant.cc`
- `src/kuavo-ros-control-lejulib/hardware_node/src/hardware_node.cc`
- `src/kuavo-ros-control-lejulib/hardware_plant/lib/ruiwo_controller_cxx/src/ruiwo_actuator.cpp`

**碰撞检测**：
- `src/kuavo_arm_collision_check/src/arm_collision_checker.cpp`

**轮式控制**：
- `src/humanoid-wheel-control/humanoid_wheel_interface_ros/src/MobileManipulatorJoyCommandNode.cpp`

**Isaac Sim**：
- `src/kuavo-isaac-sim/controller_tcp/src/isaac_sim_system.cpp`（6 处精确版本匹配）

**Demo**：
- `src/demo/grab_box/src/grab_box_demo.cpp`
- `src/demo/grab_box/src/tagTracker.cpp`

**kuavo_common**：
- `src/kuavo_common/src/kuavo_common.cpp`（含 v15 特殊处理）
- `src/kuavo_common/src/common/kuavo_settings.cpp`（含疑似 bug）

**测试文件**（10+ 个）

**Protobuf 自动生成**（不需手动迁移）：
- `src/manipulation_nodes/noitom_hi5_hand_udp_python/protos_c/robot_info.pb.cc`
- `src/manipulation_nodes/noitom_hi5_hand_udp_python/protos_c/robot_info.pb.h`

### Python 文件（~54 个，主要列出）

**SDK 核心**（6 个，含 WebSocket SDK 镜像）：
- `src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo/core/ros/param.py` **[关节配置核心]**
- `src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo/core/core.py` **[运动限制]**
- `src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo/robot_info.py`
- `src/kuavo_humanoid_websocket_sdk/kuavo_humanoid_sdk/kuavo/core/ros/param.py`
- `src/kuavo_humanoid_websocket_sdk/kuavo_humanoid_sdk/kuavo/core/core.py`
- `src/kuavo_humanoid_websocket_sdk/kuavo_humanoid_sdk/kuavo/robot_info.py`

**标定工具**（8 个）：
- `scripts/joint_cali/joint_cali_by_hard_limit.py`
- `scripts/joint_cali/arm_kinematics.py`
- `scripts/joint_cali/joint_cali_ui.py`
- `scripts/joint_cali/identifiability_analyzer.py`
- `scripts/joint_cali/target_tracker.py`
- `scripts/joint_cali/arm_cali.py`
- `scripts/joint_cali/head_cali.py`
- `scripts/joint_cali/arm_cail_noui.py`

**手臂控制/IK 脚本**（10+ 个）：
- `src/humanoid-control/humanoid_arm_control/scripts/BezierWrap.py`
- `src/humanoid-control/humanoid_arm_control/scripts/arm_control_with_keyboard.py`
- `src/humanoid-control/humanoid_arm_control/scripts/motion_capture_ik_packaged/scripts/ik_ros_uni.py`
- `src/humanoid-control/humanoid_arm_control/scripts/motion_capture_ik_packaged/scripts/quest3_node.py`
- `src/humanoid-control/humanoid_arm_control/scripts/motion_capture_ik_packaged/scripts/visulize_traj.py`
- `src/manipulation_nodes/motion_capture_ik/scripts/ik_ros_uni.py`
- `src/manipulation_nodes/motion_capture_ik/scripts/quest3_node.py`
- `src/manipulation_nodes/motion_capture_ik/scripts/quest3_node_incremental.py`
- `src/manipulation_nodes/motion_capture_ik/scripts/visulize_traj.py`
- `src/manipulation_nodes/motion_capture_ik/scripts/ik/torso_ik.py`

**H12 控制器回调**（3 个）：
- `src/humanoid-control/h12pro_controller_node/robot_state/rl_before_callback.py`
- `src/humanoid-control/h12pro_controller_node/robot_state/multi_before_callback.py`
- `src/humanoid-control/h12pro_controller_node/robot_state/ocs2_before_callback.py`

**其他脚本**：
- `src/manipulation_nodes/pico-body-tracking-server/scripts/core/ros/pico.py`
- `src/manipulation_nodes/noitom_hi5_hand_udp_python/scripts/monitor_quest3.py`
- `src/manipulation_nodes/noitom_hi5_hand_udp_python/scripts/robot_state_server.py`
- `src/manipulation_nodes/planarmwebsocketservice/scripts/handler.py`
- `src/kuavo_assets/scripts/trim_mesh_to_ik.py`
- `src/automatic_test/.../test_robot_walk.py`
- `src/demo/trace_path/scripts/.../mpc_path_tracer.py`
- `src/demo/trace_path/scripts/.../mpc_client_example.py`
- `src/demo/examples_code/action_scripts/factory_reset_action.py`
- `src/kuavo-isaac-sim/nio-isaac/nio/env/env_base.py`
- `tools/setup-kuavo-ros-control.py`
- `tools/calibration_python/scripts/test_ik_fk.py`
- `tools/calibration_python/scripts/function/marker_utils.py`
- `tools/get_joint_data/generate_cali_data.py`
- `tools/extract_camera_pose/example_camera_pose_from_bag.py`
- `tools/extract_camera_pose/endeffector_pose_from_bag.py`
- `tools/extract_camera_pose/kuavo_pose_calculator.py`
- `tools/check_tool/kuavo_wheel_test/torso_motion_test.py`
- `tools/check_tool/Hardware_tool.py`
- `tools/check_tool/joint_breakin_ros/src/breakin_control/scripts/arm_breakin_standalone.py`

### Shell 文件（~22 个）

- `tools/setup-kuavo-ros-control.sh` **[安装脚本]**
- `src/kuavo_assets/scripts/update_mass.sh`
- `src/kuavo_assets/scripts/check_ecmaster_type.sh`
- `tools/check_tool/stress_test/stress_test_all_cores.sh`
- `tools/check_tool/stress_test/stress_test_normal_cores.sh`
- `tools/check_tool/change_EndEffectorType.sh`
- `tools/upload_log/install_log_uploader.sh`
- `src/humanoid-control/h12pro_controller_node/scripts/deploy_autostart.sh`
- `src/humanoid-control/h12pro_controller_node/scripts/start_ocs2_h12pro_node.sh`
- `src/humanoid-control/joystick_drivers/joy/services/deploy_autostart.sh`
- `src/humanoid-control/joystick_drivers/joy/services/start_roban_joy_node.sh`
- `src/kuavo_wheel/scripts/start_wheel_bridge.sh`
- `src/manipulation_nodes/planarmwebsocketservice/service/websocket_deploy_script.sh`
- `src/demo/examples_code/action_scripts/kuavo_tools.sh`
- `scripts/leju_start.sh`
- `scripts/kuavo_environment_setup.sh`
- `scripts/joint_cali/arm_cali_in_docker.sh`
- `ci_scripts/automatic_tests/grab_box_sim/grab_box_sim_test.sh`
- `ci_scripts/automatic_tests/arm_ik_sim/arm_ik_sim_test.sh`
- `ci_scripts/automatic_tests/trace_path_sim/trace_path_sim_test.sh`
- `docker/run.sh`
- `docker/run_with_gpu.sh`

### CMake 文件（1 个）

- `src/kuavo-ros-control-lejulib/hardware_node/CMakeLists.txt`

### Docker / Service / CI 文件

- `docker/Dockerfile`
- `src/humanoid-control/joystick_drivers/joy/services/roban_joy_monitor.service`
- `src/humanoid-control/h12pro_controller_node/services/ocs2_h12pro_node.service`
- `src/humanoid-control/h12pro_controller_node/services/ocs2_h12pro_monitor.service`
- `src/manipulation_nodes/planarmwebsocketservice/service/websocket_start.service`
- `.gitlab-ci.yml`
