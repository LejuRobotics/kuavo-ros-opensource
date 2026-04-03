# VR 自动化测试与动作回放说明

本目录包含 VR 自动化测试、动作录制、bag 拆分和 rosbag 回放脚本，用于录制、复现和评估 VR 控制动作。

## 相关脚本

| 文件 | 说明 |
|------|------|
| `start_VR_test.sh` | 一键启动 VR 自动化测试 |
| `test_VR.sh` | 执行测试流程的辅助脚本 |
| `VR_test.py` | 自动化测试与误差判定主逻辑 |
| `record_bag.py` | 录制 VR 动作 rosbag |
| `filter_bag.sh` | 拆分录制得到的 bag 文件 |
| `rosbag_vr_playback.py` | 回放 VR 相关话题数据 |

## 一键模拟 VR 测试

### 配置

- 安装依赖：

```bash
sudo pip install tqdm && sudo pip install questionary
```

- 编译：

```bash
cd <kuavo-ros-control>
catkin build automatic_test
```

- 加载环境变量：

```bash
source devel/setup.bash
```

### 启动测试

- 参数说明：

```bash
"test_round": 测试轮数，即每一个动作执行多少次。
```

- 启动脚本：

```bash
./start_VR_test.sh test_round
```

测试结束后，会统计误差并判断动作是否执行标准。如果有不标准的执行，则认为此次动作失败，并以红色标注。

## 关于阈值的说明

### 误差计算方法

1. 逆解的逐关节误差：
   比较每一个关节的每一个逆解结果，两个值作差值，将整个逆解结果的差值进行平方并求平均，获得平均平方差。
2. 机器人实际运动关节对比：
   由于每一次的运动，逆解并不一定是唯一解。因此还读取实际的关节电机读数，参考电机是否达到目标位置，同样使用逐关节误差平方的平均值。

### 阈值

经过仿真和实机测试，当前阈值设置为：

```python
# 文件: kuavo-ros-control/src/automatic_test/automatic_test/scripts/automatic_VR_test/VR_test.py
JOINT_ERROR_THRESHOLD = 0.015
IK_ERROR_THRESHOLD = 9.0
```

## 动作录制

- 启动机器人仿真和 VR：

```bash
source ~/kuavo-ros-control/devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch

# 可选配置参数：use_cpp_ik
# 启动 Python 版本 IK
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=false

# 启动 C++ 版本 IK
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=true

# 可选配置参数：use_incremental_ik
# 仅当 use_cpp_ik:=true 时，可选是否启用增量式 IK
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_ik:=true use_incremental_ik:=true
```

- 接入 VR。
- 运行程序开始录制：

```bash
python3 record_bag.py
```

- 解锁手臂，开始跟随运动。
- 执行待测试动作，建议在 5 秒以内完成。
- 录制完成后，rosbag 文件默认保存在：

```bash
~/rosbag_records/session_2025*.bag
```

## bag 拆分

将录制的 bag 拆分成两个待使用的 bag：

1. 包含实际参考的逆解轨迹以及电机值的 bag：

```bash
session_2025*_group1.bag
```

2. 包含 VR 发布的末端姿态和手肘姿态骨骼信息的 bag：

```bash
session_2025*_group2.bag
```

- 拷贝待拆分的 rosbag 文件到当前目录：

```bash
cp ~/rosbag_records/session_2025*.bag ./
```

- 运行脚本自动拆分：

```bash
./filter_bag.sh session_2025*.bag
```

- 拆分出的 rosbag 文件保存在当前目录。

## VR 动作回放

从 rosbag 文件回放 VR 相关话题数据，用于测试和复现 VR 控制动作。

### 功能

- 回放 VR 骨骼姿态、Joystick、手臂轨迹等话题
- 实时显示躯干、腰部、肩部等状态信息
- 支持自动切换手臂控制模式
- 可选启动 IK 节点处理骨骼数据
- 回放完成后持续监控并打印全身关节角度（1Hz）

### 回放话题

| 话题 | 说明 |
|------|------|
| `/quest_joystick_data` | Joystick 数据 |
| `/leju_quest_bone_poses` | VR 骨骼姿态 |
| `/cmd_torso_pose_vr` | 躯干姿态命令 |
| `/cmd_pose` | 位姿命令 |
| `/kuavo_arm_traj` | 手臂轨迹 |

### 监控话题

| 话题 | 说明 |
|------|------|
| `/sensors_data_raw` | 全身关节状态数据 |

### 关节索引对照表

| 部位 | 索引范围 | 关节名称 |
|------|----------|----------|
| 左腿 | 0-6 | leg_l1 ~ leg_l7 |
| 右腿 | 7-11 | leg_r2 ~ leg_r6 |
| 左臂 | 12-18 | zarm_l1 ~ zarm_l7 |
| 右臂 | 19-25 | zarm_r1 ~ zarm_r7 |
| 头部 | 26-27 | zhead_1, zhead_2 |

### 用法

```bash
# 回放单个 bag 文件
python3 rosbag_vr_playback.py /path/to/file.bag

# 回放目录下所有 bag 文件
python3 rosbag_vr_playback.py /path/to/bag_dir/

# 查看 bag 文件信息（不回放）
python3 rosbag_vr_playback.py /path/to/file.bag --info-only
```

### 参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--speed` | 回放速度倍率 | 1.0 |
| `--loop` | 循环回放 | 否 |
| `--no-display` | 不显示详细信息 | 否 |
| `--info-only` | 只显示 bag 信息 | 否 |
| `--arm-mode` | 手臂控制模式 | 2 |
| `--skip-arm-mode` | 跳过手臂模式切换 | 否 |
| `--use-ik` | 启动 IK 节点 | 否 |

### 示例

```bash
# 2 倍速循环回放
python3 rosbag_vr_playback.py demo.bag --speed 2.0 --loop

# 使用 IK 节点处理骨骼数据
python3 rosbag_vr_playback.py demo.bag --use-ik

# 跳过手臂模式切换，直接回放
python3 rosbag_vr_playback.py demo.bag --skip-arm-mode
```

### 输出示例

回放完成后，脚本会持续打印关节角度：

```text
======================================================================
回放完成，开始监控关节角度 (按 Ctrl+C 停止)
======================================================================

[1706123456.78] 当前关节角度 (28 个关节):
----------------------------------------------------------------------
  左腿 [0-6]:   [   0.00,  -5.20,  10.30,  45.00, -25.10,   0.50,   0.00]
  右腿 [7-11]:  [  -5.20,  10.30,  45.00, -25.10,   0.50]
  左臂 [12-18]: [  10.50, -20.30,  15.00, -45.20,  30.10, -10.00,   5.50]
  右臂 [19-25]: [ -10.50, -20.30, -15.00, -45.20, -30.10, -10.00,  -5.50]
  头部 [26-27]: [   0.00,   0.00]
```

### 前提条件

1. ROS 环境已配置
2. 机器人已进入站立模式
3. 相关控制节点已启动
