# case_wheel_test_torso.py - 轮臂躯干控制测试

## 功能描述

- 演示机器人躯干的位置和姿态控制
- 精确控制躯干在 3D 空间中的位置和方向

## 主要特性

- 6DOF 躯干控制（位置 + 姿态）
- 异步执行模式
- 基于 BASE 坐标系的控制
- 实时运动状态监控

## 配置参数

```python
TORSO_TARGET_POS = (0.2, 0.0, 0.5)      # 目标位置 (x, y, z 米)
TORSO_TARGET_EULER = (0.0, 10°, 90°)    # 目标姿态 (roll, pitch, yaw 度)
```

## 启动说明

### 启动轮臂下位机程序

#### 启动下位机主程序

- **Gazebo 仿真**：使用 `load_kuavo_gazebo_sim_wheel.launch`
```bash
cd ~/kuavo_ros_opensouece
export ROBOT_VERSION=60  # 或者61,根据底盘选择版本
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_gazebo_sim_wheel.launch
```

- **MuJoCo 仿真**：使用 `load_kuavo_mujoco_sim_wheel.launch`
```bash
cd ~/kuavo_ros_opensouece
export ROBOT_VERSION=60
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
```

- **真实样机**：使用 `load_kuavo_real_wheel.launch`
```bash
cd ~/kuavo_ros_opensouece
export ROBOT_VERSION=60
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real_wheel.launch
```

说明：上述两个 launch 文件位于 `src/humanoid-control/humanoid_controllers/launch/`，分别用于 Gazebo 和 MuJoCo 两种不同仿真环境。

#### 2. 启动下位机相机坐标系转换机器人程序

仅在需要识别tag时使用：
```bash
cd ~/kuavo_ros_opensouece
export ROBOT_VERSION=60
source devel/setup.bash
rosrun ar_control ar_control_node.py
```

### 启动上位机程序

#### TF 树转发程序（websocket server）

在新终端启动：
```bash
cd ~/kuavo_ros_application
source devel/setup.bash
roslaunch kuavo_tf2_web_republisher start_websocket_server.launch
```

若未启动该程序，Python SDK/策略中通过 `/republish_tfs` 获取 TF 变换时会报类似 `kuavo_tf2_web_republisher` 节点未运行 的错误。

### 启动 PyTree 轮臂案例

在另一个终端启动（建议先 cd 到仓库根目录并 source 环境）：
```bash
cd ~/kuavo_ros_opensouece
source devel/setup.bash
cd src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box
python3 case_wheel_test_torso.py
```

## 使用方法

```bash
python3 case_wheel_test_torso.py
```

## 支持环境

- MuJoCo 仿真环境
- Gazebo 仿真环境

