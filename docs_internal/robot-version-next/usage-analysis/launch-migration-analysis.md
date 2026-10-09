# Launch 文件迁移分析

**目标**：将 launch 文件中的版本号路径拼接（`biped_s$(arg robot_version)` 等）迁移为使用 `robot_version` 包提供的语义变量。

**方案**：每个文件顶部添加 arg 定义，默认值由 `robot_version` Python 包提供：

```xml
<arg name="model_name"  default="$(eval __import__('robot_version').model_name())"/>
<arg name="config_name" default="$(eval __import__('robot_version').config_name())"/>
<arg name="urdf_name"   default="$(eval __import__('robot_version').urdf_name())"/>
```

替换规则：

| 旧模式 | 新模式 | 使用的变量 |
|--------|--------|-----------|
| `biped_s$(arg robot_version)` | `$(arg model_name)` | model_name |
| `kuavo_v$(arg robot_version)` | `$(arg config_name)` | config_name |
| `kuavo_s$(arg robot_version)` | `$(arg config_name)` | config_name（目录需重命名 `kuavo_s` → `kuavo_v`） |
| `s$(arg robot_version)_collision_config` | `$(arg collision_config_name)` | collision_config_name |
| `joint_control_params_v$(arg robot_version)` | `$(arg joint_params_name)` | joint_params_name |

---

## 总计：26 个文件需要修改

---

## 一、只需要 model_name（12 个文件）

这些文件只用了 `biped_s$(arg robot_version)` 模式，添加 1 行 arg 即可。

| # | 文件路径 | 改动处数 |
|---|---------|---------|
| 1 | `src/mujoco/launch/mujoco_sim.launch` | 1 处 |
| 2 | `src/mujoco/launch/nodelet.launch` | 1 处 |
| 3 | `src/mujoco/launch/nodelet_with_arm.launch` | 1 处 |
| 4 | `src/manipulation_nodes/motion_capture_ik/launch/ik_node.launch` | 2 处 |
| 5 | `src/manipulation_nodes/motion_capture_ik/launch/visualize.launch` | 2 处 |
| 6 | `src/humanoid-control/humanoid_arm_control/scripts/motion_capture_ik_packaged/launch/visualize.launch` | 2 处 |
| 7 | `src/humanoid-control/mobile_manipulator_controllers/launch/mobile_manipulator_controller.launch` | 1 处 |
| 8 | `src/humanoid-control/mobile_manipulator_controllers/launch/mobile_manipulator_controller_base.launch` | 1 处 |
| 9 | `src/humanoid-control/mobile_manipulator_controllers/launch/mobile_manipulator_controller_general.launch` | 1 处 |
| 10 | `src/humanoid-control/humanoid_plan_arm_trajectory/launch/humanoid_arm_trajectort_rviz.launch` | 1 处 |
| 11 | `src/automatic_test/automatic_test/launch/half_body_arm_fk_ik.launch` | 2 处 |
| 12 | `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/manipulator_kuavo.launch` | 1 处 |

**示例改动**（mujoco_sim.launch）：

```xml
<!-- 添加 -->
<arg name="model_name" default="$(eval __import__('robot_version').model_name())"/>

<!-- 旧 -->
<arg name="scene_file" default="$(find kuavo_assets)/models/biped_s$(arg robot_version)/xml/scene.xml"/>
<!-- 新 -->
<arg name="scene_file" default="$(find kuavo_assets)/models/$(arg model_name)/xml/scene.xml"/>
```

---

## 二、需要 model_name + config_name（10 个文件）

同时用了 `biped_s` 和 `kuavo_v`/`kuavo_s` 模式，添加 2 行 arg。

| # | 文件路径 | 模式 | 改动处数 |
|---|---------|------|---------|
| 13 | `src/humanoid-control/humanoid_controllers/launch/robot_version_manager.launch` | 1 + 2 | 6 处 |
| 14 | `src/humanoid-control/humanoid_controllers/launch/load_kuavo_real.launch` | 1 + 2 | 2 处 |
| 15 | `src/humanoid-control/humanoid_controllers/launch/load_kuavo_mujoco_sim.launch` | 1 + 2 | 2 处 |
| 16 | `src/humanoid-control/humanoid_controllers/launch/load_kuavo_mujoco_sim_minimal.launch` | 1 + 2 | 2 处 |
| 17 | `src/humanoid-control/humanoid_controllers/launch/load_kuavo_gazebo_sim.launch` | 1 + 2 | 2 处 |
| 18 | `src/humanoid-control/humanoid_controllers/launch/load_kuavo_gazebo_manipulate.launch` | 1 + 2 | 2 处 |
| 19 | `src/humanoid-control/humanoid_controllers/launch/load_kuavo_gazebo_sim_wheel.launch` | 1 + 3 | 3 处 |
| 20 | `src/humanoid-control/humanoid_controllers/launch/load_kuavo_mujoco_sim_wheel.launch` | 1 + 3 | 3 处 |
| 21 | `src/humanoid-control/humanoid_controllers/launch/load_kuavo_real_wheel.launch` | 1 + 3 | 3 处 |
| 22 | `src/humanoid-control/humanoid_controllers/launch/wheel_controller_mpc_test.launch` | 1 + 3 | 3 处 |

**示例改动**（robot_version_manager.launch）：

```xml
<!-- 添加 -->
<arg name="model_name"  default="$(eval __import__('robot_version').model_name())"/>
<arg name="config_name" default="$(eval __import__('robot_version').config_name())"/>

<!-- 旧 -->
<arg name="urdfFile" default="$(find kuavo_assets)/models/biped_s$(arg robot_version)/urdf/biped_s$(arg robot_version).urdf"/>
<arg name="taskFile" default="$(find humanoid_controllers)/config/kuavo_v$(arg robot_version)/mpc/task.info"/>
<!-- 新 -->
<arg name="urdfFile" default="$(find kuavo_assets)/models/$(arg model_name)/urdf/$(arg model_name).urdf"/>
<arg name="taskFile" default="$(find humanoid_controllers)/config/$(arg config_name)/mpc/task.info"/>
```

**注**：模式 3（`kuavo_s`）出现在轮式控制相关文件中，`config_name` 函数需要根据 locomotion 类型返回不同前缀（biped → `kuavo_v`，wheeled → `kuavo_s`），或者单独提供 `wheel_config_name()`。

---

## 三、Gazebo 仿真文件（3 个文件）

这些文件有较多拼接点，包含 URDF、xacro、模型名等。

| # | 文件路径 | 改动处数 |
|---|---------|---------|
| 23 | `src/gazebo/gazebo-sim/launch/gazebo-sim.launch` | 4 处 |
| 24 | `src/gazebo/gazebo-sim/launch/gazebo-sim-ground-tags.launch` | 4 处 |
| 25 | `src/gazebo/gazebo-sim/launch/gazebo-sim-grab-box-ci.launch` | 2 处 |

---

## 四、特殊模式文件（3 个文件）

这些文件的命名模式与上述不同，需要专门的变量。

| # | 文件路径 | 旧模式 | 新变量 |
|---|---------|--------|--------|
| 26 | `src/kuavo_arm_collision_check/launch/arm_collision_check.launch` | `s$(arg robot_version)_collision_config.yaml` | collision_config_name |
| 27 | `src/kuavo-ros-control-lejulib/hardware_node/launch/motor_follow_test.launch` | `joint_control_params_v$(arg robot_version).json` | joint_params_name |
| 28 | `src/humanoid-wheel-control/humanoid_wheel_interface_ros/launch/manipulator_kuavo_s60.launch` | `kuavo_s$(arg robot_version)` | wheel_config_name |
| 29 | `src/humanoid-wheel-control/humanoid_wheel_interface_ros/launch/manipulator_kuavo_s60_sqp.launch` | `kuavo_s$(arg robot_version)` | wheel_config_name |

---

## 汇总

| 类别 | 文件数 | 每文件添加 arg 数 |
|------|:------:|:----------------:|
| 只需 model_name | 12 | 1 行 |
| model_name + config_name | 10 | 2 行 |
| Gazebo 仿真 | 3 | 1 行 |
| 特殊模式 | 4 | 1 行 |
| **合计** | **29** | **1-2 行** |

## 迁移方式

改动是机械性的，可用脚本批量完成：
1. 在文件顶部（`<arg name="robot_version">` 之后）插入 model_name / config_name arg 定义
2. 全文替换 `biped_s$(arg robot_version)` → `$(arg model_name)`
3. 全文替换 `kuavo_v$(arg robot_version)` → `$(arg config_name)`
4. 全文替换 `kuavo_s$(arg robot_version)` → `$(arg wheel_config_name)`
5. 处理特殊模式文件

## 迁移后效果

- 以后新增机器人只需修改 `robot_version` Python 包的映射表
- 所有 29 个 launch 文件自动适配新版本号格式
- 用户仍可通过 `roslaunch xxx.launch model_name:=custom_model` 进行 override
