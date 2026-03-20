# case_wheel_test_head.py - 轮臂头部搜索测试

## 功能描述

- 演示机器人头部的智能搜索功能
- 通过头部运动在不同角度搜索 AprilTag
- 使用并行处理同时进行感知和头部控制

## 主要特性

- 多角度搜索策略（偏航角 ±85°，俯仰角 ±10°）
- 并行执行感知和头部搜索
- 实时 AprilTag 检测和位置反馈
- 成功检测后自动停止搜索

## 配置参数

```python
TAG_ID = config.pick.tag_id                                    # 目标 AprilTag ID
HEAD_SEARCH_YAWS = [85°, 0°, -85°]                            # 头部搜索偏航角
HEAD_SEARCH_PITCHES = [-10°, 0°, 10°]                         # 头部搜索俯仰角
```

## 启动说明

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

#### 启动下位机相机坐标系转换机器人程序

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

#### 启动相机程序

在新终端启动：
```bash
cd ~/kuavo_ros_application
source devel/setup.bash
roslaunch dynamic_biped apriltag.launch
```

### 启动 PyTree 轮臂案例

在另一个终端启动（建议先 cd 到仓库根目录并 source 环境）：
```bash
cd ~/kuavo_ros_opensouece
source devel/setup.bash
cd src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box
python3 case_wheel_test_head.py
```

## 使用方法

```bash
python3 case_wheel_test_head.py
```

## 支持环境

- Gazebo 仿真环境

