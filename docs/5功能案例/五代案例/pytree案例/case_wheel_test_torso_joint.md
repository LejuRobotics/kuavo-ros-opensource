# case_wheel_test_torso_joint.py - 轮臂折叠臂控制测试

## 功能描述

- 演示轮臂机器人下肢关节的精确控制
- 直接控制4个关节的角度位置
- 专门针对轮臂机器人的下肢关节控制设计

## 主要特性

- 4关节独立角度控制
- 发布到ROS话题：/lb_leg_traj
- 异步执行模式
- 实时关节状态反馈

## 配置参数

```python
TARGET_JOINT_ANGLES = [14.90, -32.01, 18.03, 90.0]  # 目标关节角度（度）
TOTAL_TIME = 8.0                                     # 执行时间（秒）
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
python3 case_wheel_test_torso_joint.py
```

## 使用方法

```bash
python3 case_wheel_test_torso_joint.py
```

## 支持环境

- 真实样机
- MuJoCo 仿真环境
- Gazebo 仿真环境

