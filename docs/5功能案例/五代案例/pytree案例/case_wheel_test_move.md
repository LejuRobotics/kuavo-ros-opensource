# case_wheel_test_move.py - 轮臂移动控制测试

## 功能描述

- 演示机器人的导航和移动功能
- 基于检测到的 AprilTag 计算目标位置
- 支持多种移动控制模式

## 主要特性

- 三种控制模式：cmd_pos_world、cmd_pos、cmd_vel
- 坐标系转换（tag坐标系 → 世界坐标系）
- 可使用真实或模拟的 AprilTag 数据
- 自动路径规划和执行

## 启动说明

轮臂跑 PyTree 前，通常需要先启动两类程序（写法/顺序参考"原子技能"类文档）：

### 准备

Gazebo 仿真里如果使用 apriltag_ros 连续检测，通常需要先把系统的 tag 配置改成与仿真中 tag 实际尺寸一致（只需做一次）：
```bash
sudo vim /opt/ros/noetic/share/apriltag_ros/config/tags.yaml
```
参考写法见 `src/kuavo_humanoid_sdk/docs/markdown/pages/kuavo_strategy_v2.md` 的"准备"章节。

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
python3 case_wheel_test_move.py
```

## 使用方法

```bash
python3 case_wheel_test_move.py  # 使用默认 cmd_pos_world 模式

# 指定控制模式
python3 case_wheel_test_move.py --control-mode cmd_pos_world
python3 case_wheel_test_move.py --control-mode cmd_vel
```

## 支持环境

- MuJoCo 仿真环境
- Gazebo 仿真环境
- 真实样机
