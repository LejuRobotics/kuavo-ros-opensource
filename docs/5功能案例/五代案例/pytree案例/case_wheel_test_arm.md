# case_wheel_test_arm.py - 轮臂手臂控制测试

## 功能描述

- 演示机器人手臂的精确控制
- 基于 AprilTag 位置生成手臂运动轨迹
- 支持多种控制模式：关节控制和末端控制

## 主要特性

- 设置虚假 AprilTag 位置用于测试（ODOM 坐标系）
- 自动生成左右手臂的抓取关键点
- 支持三种控制模式：
  - 关节控制模式 (joint)：关节轨迹控制，直接控制各关节角度
  - 世界坐标系末端控制 (eef_world)：基于世界坐标系的末端轨迹控制
  - 基坐标系末端控制 (eef_base)：基于机器人基坐标系的末端轨迹控制
- 使用行为树序列执行：设置标签 → 计算轨迹 → 执行运动
- 实时状态反馈和可视化

## 配置参数

```python
TAG_ID = config.pick.tag_id          # AprilTag ID
FAKE_TAG_POS = (0.50, 0.0, 0.75)    # 虚假标签位置 (x, y, z 米)
FAKE_TAG_EULER = (90, 0, -90)       # 虚假标签姿态 (roll, pitch, yaw 度)
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
python3 case_wheel_test_arm.py
```

## 使用方法

```bash
python3 case_wheel_test_arm.py
```

## 支持环境

- MuJoCo 仿真环境
- Gazebo 仿真环境
- 真实样机
