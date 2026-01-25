# Pick & Place Box 测试脚本说明

## 文件夹路径

src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box

## 文件夹内容说明

该目录包含了基于 PyTrees 行为树框架的机器人控制测试脚本，用于演示机器人的各个部位控制功能。所有脚本都使用 AprilTag 作为参考点进行定位和控制。

## 脚本概述

### 1. case_wheel_test_arm.py - 手臂控制测试

**功能描述：**
- 演示机器人手臂的精确控制
- 基于 AprilTag 位置生成手臂运动轨迹
- 支持多种控制模式：关节控制和末端控制

**主要特性：**
- 设置虚假 AprilTag 位置用于测试（ODOM 坐标系）
- 自动生成左右手臂的抓取关键点
- 支持三种控制模式：
  - **关节控制模式 (`joint`)**：关节轨迹控制，直接控制各关节角度
  - **世界坐标系末端控制 (`eef_world`)**：基于世界坐标系的末端轨迹控制
  - **基坐标系末端控制 (`eef_base`)**：基于机器人基坐标系的末端轨迹控制
- 使用行为树序列执行：设置标签 → 计算轨迹 → 执行运动 → 手臂归位
- 实时状态反馈和可视化

**配置参数：**
```python
TAG_ID = config.pick.tag_id          # AprilTag ID
FAKE_TAG_POS = (0.50, 0.0, 0.75)    # 虚假标签位置 (x, y, z 米)
FAKE_TAG_EULER = (90, 0, -90)       # 虚假标签姿态 (roll, pitch, yaw 度)
control_type = 'joint'               # 控制模式：'joint'（默认）、'eef_world' 或 'eef_base'
direct_to_wbc = False               # 是否直接发送到WBC（跳过MPC）⚠️轮臂必须设为False
back_default = False                # 运动结束后是否返回默认控制模式⚠️轮臂必须设为False
total_time = 3.0                    # 手臂执行时间（秒）
traj_point_num = 100                # 轨迹点数量
enable_joint_mirroring = True       # 是否启用关节镜像（仅在joint模式下有效）
enable_high_position_accuracy = False  # 是否启用位置约束优先,默认False是优先姿态约束
```

**控制模式说明：**
- **`joint`**: 关节轨迹控制，直接控制每个关节的角度
  - 支持**镜像模式**：根据条件自动进行左右臂关节角度的对称复制
  - 数据流：14维关节数据 → 左臂7维+右臂7维 → 镜像变换 → 重新合并为14维
  - 轨迹同步：确保左右臂轨迹点数一致，支持双臂协调运动
- **`eef_world`**: 世界坐标系末端控制，基于全局坐标系进行轨迹规划
- **`eef_base`**: 基坐标系末端控制，基于机器人基坐标系进行轨迹规划

**镜像模式工作原理：**
- **方向判断**：根据左臂J1关节角度符号(`left_joint[1] > 0`)自动决定镜像方向
- **左→右镜像**：当左臂J1>0时，将左臂关节角度复制到右臂并进行对称变换
- **右→左镜像**：当左臂J1≤0时，将右臂关节角度复制到左臂并进行对称变换  
- **对称规则**：J0,J3,J6保持原值；J1,J2,J4,J5取负号实现物理对称
- **空间约束**：通过Y轴对称的肘部偏移确保左右臂运动的合理性

**高级参数说明：**
- **`control_base`**: 是否联合控制底盘，实现全身协调运动
- **`direct_to_wbc`**: 控制流程选择，跳过MPC直接发送到WBC可获得更快响应
- **`back_default`**: 运动完成后的模式管理，False可保持当前控制状态便于连续操作
- **`enable_joint_mirroring`**: 关节镜像开关，仅在`joint`控制模式下有效
  - `True`（默认）：启用关节镜像功能，自动进行左右臂对称复制
  - `False`：禁用关节镜像，使用原始IK求解结果
  - **设置位置**：`NodeTagToArmGoal`节点构造函数参数

> **⚠️ 轮臂机器人重要提示**：
> - `direct_to_wbc = False` - 轮臂机器人必须通过MPC优化，不可直接发送到WBC
> - `back_default = False` - 轮臂机器人需要保持控制状态，避免模式切换导致的控制异常

**使用方法：**
```bash
python case_wheel_test_arm.py
```

**支持环境：**
- **MuJoCo 仿真环境**：物理引擎仿真，支持精确的动力学计算
- **Gazebo 仿真环境**：ROS集成仿真，支持传感器和环境交互

---

### 2. case_wheel_test_head.py - 头部搜索测试

**功能描述：**
- 演示机器人头部的智能搜索功能
- 通过头部运动在不同角度搜索 AprilTag
- 使用并行处理同时进行感知和头部控制

**主要特性：**
- 多角度搜索策略（偏航角 ±85°，俯仰角 ±10°）
- 并行执行感知和头部搜索
- 实时 AprilTag 检测和位置反馈
- 成功检测后自动停止搜索
- 超时保护机制（默认5秒）

**配置参数：**
```python
TAG_ID = config.pick.tag_id                                    # 目标 AprilTag ID
HEAD_SEARCH_YAWS = [np.deg2rad(85), 0, np.deg2rad(-85), 0]       # 头部搜索偏航角（弧度）
HEAD_SEARCH_PITCHES = [np.deg2rad(-10), np.deg2rad(0), np.deg2rad(10)]  # 头部搜索俯仰角（弧度）
TIMEOUT_SECONDS = 5.0                                           # 超时时间（秒）
```

**使用方法：**
```bash
python case_wheel_test_head.py
```

**环境要求：**
- 建议在 **Gazebo 仿真环境** 中运行此测试
- 确保 Gazebo 中已加载机器人模型和 AprilTag 标签
- 需要启动相应的坐标转换节点：
  ```bash
  rosrun ar_control ar_control_node.py
  ```
  > 此节点用于将 AprilTag 在相机坐标系的位置转换到 ODOM 和 BASE 坐标系下

---

### 3. case_wheel_test_move.py - 移动控制测试

**功能描述：**
- 演示机器人的导航和移动功能
- 基于检测到的 AprilTag 计算目标位置
- 支持多种移动控制模式

**主要特性：**
- 三种控制模式：`cmd_pos_world`（默认）、`cmd_pos`、`cmd_vel`
- 坐标系转换（tag坐标系 → 世界坐标系）
- 可使用真实或模拟的 AprilTag 数据
- 自动路径规划和执行

**配置参数：**
- 使用配置文件中的站立位置参数（`config.pick.stand_in_tag_pos`, `config.pick.stand_in_tag_euler`）
- 支持实时 tag 数据或预设测试数据

**使用方法：**
```bash
# 使用默认 cmd_pos_world 模式
python case_wheel_test_move.py

# 指定控制模式
python case_wheel_test_move.py --control-mode cmd_pos_world
python case_wheel_test_move.py --control-mode cmd_pos
python case_wheel_test_move.py --control-mode cmd_vel
```

**重要提示：**
- 运行前需要初始化 SDK：`KuavoSDK.Init(log_level="INFO")`
- 否则 `cmd_vel` 或 `walk` 模式可能因缺少 `_robot_version_major` 而崩溃

**支持环境：**
- **MuJoCo 仿真环境**：精确的移动动力学仿真
- **Gazebo 仿真环境**：完整的导航和环境交互仿真

---

### 4. case_wheel_test_torso.py - 躯干控制测试

**功能描述：**
- 演示机器人躯干的位置和姿态控制
- 精确控制躯干在 3D 空间中的位置和方向

**主要特性：**
- 6DOF 躯干控制（位置 + 姿态）
- 异步执行模式
- 基于 BASE 坐标系的控制
- 实时运动状态监控
- 可配置执行时间

**配置参数：**
```python
TORso_TARGET_POS = (0.0, 0.0, 1.2)      # 目标位置 (x, y, z 米) - 默认配置
TORso_TARGET_EULER = (0.0, 0.0, 90.0)   # 目标姿态 (roll, pitch, yaw 度) - 默认配置
total_time = 3.0                        # 执行时间（秒）
```

**使用方法：**
```bash
python case_wheel_test_torso.py
```

**支持环境：**
- **MuJoCo 仿真环境**：高精度躯干动力学仿真
- **Gazebo 仿真环境**：完整的机器人躯干控制仿真

---

### 5. case_wheel_test_torso_joint.py - 躯干关节控制测试

**功能描述：**
- 演示轮臂机器人下肢关节的精确控制
- 直接控制4个关节的角度位置
- 专门针对轮臂机器人的下肢关节控制设计

**主要特性：**
- 4关节独立角度控制
- 发布到ROS话题：`/lb_leg_traj`
- 异步执行模式
- 实时关节状态反馈

**配置参数：**
```python
TARGET_JOINT_ANGLES = [14.90, -32.01, 18.03, -90.0]  # 目标关节角度（度）
TOTAL_TIME = 3.0                                      # 执行时间（秒）- 默认3秒
```

**使用方法：**
```bash
python case_wheel_test_torso_joint.py
```

**支持环境：**
- **轮臂机器人实体**：真实硬件下肢关节控制
- **MuJoCo 仿真环境**：轮臂模型关节动力学仿真
- **Gazebo 仿真环境**：完整的轮臂机器人仿真

## 技术架构

### 依赖库
- **py_trees**: 行为树框架，用于复杂行为的组织和控制
- **numpy**: 数值计算和矩阵运算
- **kuavo_humanoid_sdk**: 机器人控制SDK

### 设计模式
- **行为树模式**: 所有脚本都采用行为树设计，便于扩展和维护
- **模块化设计**: 每个功能模块独立，可单独测试和调试
- **配置驱动**: 使用配置文件管理参数，便于调整

### 坐标系说明
- **ODOM**: 里程计坐标系
- **BASE**: 机器人基坐标系  
- **TAG**: AprilTag 坐标系

## 使用注意事项

1. **环境准备**: 确保机器人SDK已正确初始化
2. **硬件连接**: 确认所有传感器和执行器正常工作
3. **安全距离**: 测试时确保机器人周围有足够的安全空间
4. **参数调整**: 根据实际环境调整配置参数
5. **逐步测试**: 建议按顺序测试各个功能模块

## 故障排除

- **连接问题**: 检查机器人SDK连接状态
- **传感器问题**: 确认摄像头和其他传感器正常工作
- **运动异常**: 检查配置参数是否在安全范围内
- **行为树状态**: 使用控制台输出的状态树进行调试

## 扩展开发

这些脚本可作为更复杂机器人行为的基础模块，支持：
- 组合多个功能模块
- 添加新的传感器输入
- 实现复杂的任务序列
- 集成机器学习算法
