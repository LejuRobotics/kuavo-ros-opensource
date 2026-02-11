# 轮臂 PyTree 运动控制案例

## 概述

使用统一控制节点 `NodeWheelMoveTimedCmd` 通过 ROS 服务 `/mobile_manipulator_timed_single_cmd` 发送定时运动指令，服务返回实际执行时间。

可以使用的功能分为两个部分：   
1. 下发指令向量，和期望执行时间，返回实际执行所需时间，以及详细的服务消息  
2. 设置规划器支持的最大速度，加速度，急动度，以调整指令执行的最快时间  

## 快速开始

### 环境准备

1. 安装kuavo_humanoid_sdk
```bash
cd src/kuavo_humanoid_sdk
chmod +x install.sh
./install.sh
cd ../../
```

2. 安装py_trees
```bash
pip install py_trees
```

3. 启动主程序

仿真：
```bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch joystick_type:=bt2
```
实机：
```bash
roslaunch humanoid_controllers load_kuavo_real_wheel.launch joystick_type:=bt2
```

### 启动方式

所有案例文件位于 `src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/` 目录下。

**通用启动命令**:
```bash
cd /root/kuavo_ws
source devel/setup.bash
python3 src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_wheel_<案例名称>.py
```

**具体案例启动命令**:

1. **底盘控制（世界系）**:
```bash
python3 src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_wheel_chassis_world.py
```

2. **底盘控制（局部系）**:
```bash
python3 src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_wheel_chassis_local.py
```

3. **躯干控制**:
```bash
python3 src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_wheel_torso_pose.py
```

4. **下肢控制**:
```bash
python3 src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_wheel_leg_move.py
```

5. **上肢关节控制**:
```bash
python3 src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_wheel_arm_move.py
```

6. **手臂末端控制（局部系）**:
```bash
python3 src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_wheel_arm_ee_local.py
```

7. **手臂末端控制（世界系）**:
```bash
python3 src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_wheel_arm_ee_world.py
```

8. **规划器参数设置示例**:
```bash
python3 src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_wheel_set_ruckig_params.py
```

## 控制模式

| cmd_type | planner_index | 维度 | 说明 |
|----------|---------------|------|------|
| `chassis_world` | 0 | 3 | 底盘世界坐标系 |
| `chassis_local` | 1 | 3 | 底盘局部坐标系 |
| `torso` | 2 | 4 | 躯干位姿 |
| `leg` | 3 | 4 | 下肢关节 |
| `arm_ee_world` | 4 | 12 | 双臂末端世界坐标系 |
| `arm_ee_local` | 5 | 12 | 双臂末端局部坐标系 |
| `arm` | 6 | 14 | 上肢关节 |

---

## 案例说明

### 1. 底盘控制

**案例文件**: `case_wheel_chassis_local.py` / `case_wheel_chassis_world.py`

**输入格式**:
```python
{'time': 5.0, 'pose': [x, y, yaw]}  # 米, 米, 弧度
```

**示例**:
```python
CHASSIS_POSES = [
    {'time': 5.0, 'pose': [0.3, 0.0, 0.0]},   # 前进0.3米
    {'time': 5.0, 'pose': [0.0, 0.3, 1.57]},  # 右移0.3米 + 旋转90度
]
```

**坐标系**: `chassis_local`（局部系）或 `chassis_world`（世界系）

---

### 2. 躯干控制

**案例文件**: `case_wheel_torso_pose.py`

**输入格式**:
```python
{'time': 2.0, 'pose': [lx, lz, yaw, pitch]}  # 相对增量(米), 相对增量(米), 绝对角度(弧度), 绝对角度(弧度)
```

**示例**:
```python
TORSO_POSES = [
    {'time': 2.0, 'pose': [0.0, 0.5, 0.0, 0.0]},      # 初始抬高0.5米
    {'time': 2.0, 'pose': [0.2, 0.5, 0.0, 0.0]},       # 前移0.2米
    {'time': 2.0, 'pose': [0.2, 0.5, 3.14, 0.0]},      # 偏航旋转
    {'time': 2.0, 'pose': [0.2, 0.5, 0.0, -0.1745]},   # 俯仰-10°
    {'time': 2.0, 'pose': [0.0, 0.0, 0.0, 0.0]},       # 复位
]
```

**说明**:
- `lx`: 相对于初始位置的x方向增量（米），前后位置
- `lz`: 相对于初始位置的z方向增量（米），高度
- `yaw`: 绝对偏航角（弧度）
- `pitch`: 绝对俯仰角（弧度）

> **重要**: 案例会自动获取躯干初始位姿，然后将相对增量转换为绝对坐标。确保在运行前机器人已初始化。

---

### 3. 下肢控制

**案例文件**: `case_wheel_leg_move.py`

**输入格式**:
```python
{'time': 2.0, 'joints': [j1, j2, j3, j4]}  # 4个关节角度（度数）
```

**示例**:
```python
LEG_POSES = [
    {'time': 2.0, 'joints': [14.90, -32.01, 18.03, 0.0]},
    {'time': 2.0, 'joints': [14.90, -32.01, 18.03, 30.0]},
    {'time': 2.0, 'joints': [0.0, 0.0, 0.0, 0.0]},  # 回零位
]
```

> 注：输入为度数

---

### 4. 上肢关节控制

**案例文件**: `case_wheel_arm_move.py`

**输入格式**:
```python
{'time': 2.0, 'joints': [L1-L7, R1-R7]}  # 14个关节角度（度数）
```

**示例**:
```python
ARM_POSES = [
    {'time': 2.0, 'joints': [-30, 20, 15, -45, 25, 10, -35,   # 左臂7个
                              -30, -20, -15, -45, -25, -10, -35]},  # 右臂7个
    {'time': 2.0, 'joints': [0.0] * 14},  # 回零位
]
```

> 注：输入为度数

---

### 5. 手臂末端控制

**案例文件**: `case_wheel_arm_ee_local.py` / `case_wheel_arm_ee_world.py`

**输入格式**:
```python
{
    'time': 2.0,
    'pos_left': (x, y, z),           # 左臂位置（米）
    'euler_left': (roll, pitch, yaw), # 左臂姿态（度数）
    'pos_right': (x, y, z),          # 右臂位置（米）
    'euler_right': (roll, pitch, yaw) # 右臂姿态（度数）
}
```

**示例**:
```python
ARM_POSES = [
    {'time': 2.0, 'pos_left': (0.3, 0.4, 0.7), 'euler_left': (0, -90, 0),
                  'pos_right': (0.3, -0.4, 0.7), 'euler_right': (0, -90, 0)},
    {'time': 2.0, 'pos_left': (0.5, 0.2, 0.85), 'euler_left': (0, -90, 0),
                  'pos_right': (0.5, -0.2, 0.85), 'euler_right': (0, -90, 0)},
]
```

**坐标系**: `WheelArmFrame.BASE`（局部系）或 `WheelArmFrame.ODOM`（世界系）

> 注：欧拉角输入为 `[roll, pitch, yaw]`（度数），内部转换为服务需要的 `[yaw, pitch, roll]`（弧度）


---

## 规划器参数设置

**案例文件**: `case_wheel_set_ruckig_params.py`

**节点**: `NodeSetRuckigParams`

可以在运动控制前设置Ruckig规划器的参数，以调整运动特性（速度、加速度、急动度等）。

**使用示例**:
```python
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.nodes import NodeSetRuckigParams

# 设置底盘世界系规划器参数
set_params_node = NodeSetRuckigParams(
    name="set_ruckig_params",
    timed_cmd_api=timed_cmd_api,
    planner_index=0,  # 0: 底盘世界系, 1: 底盘局部系, 2: 躯干, 3: 下肢, 4: 双臂末端世界系, 5: 双臂末端局部系, 6: 上肢
    is_sync=True,  # 同步模式
    velocity_max=[0.2, 0.2, 0.2],  # 最大速度列表（维度需匹配规划器）
    acceleration_max=[2.0, 2.0, 1.5],  # 最大加速度列表
    jerk_max=[20.0, 15.0, 12.0],  # 最大急动度列表
    # velocity_min 和 acceleration_min 为可选参数
)

# 在行为树中使用
root = py_trees.composites.Sequence(name="demo", memory=True)
root.add_children([
    set_params_node,  # 先设置规划器参数
    move_node,  # 然后执行运动
])
```

**参数说明**:
- `planner_index`: 规划器索引，对应不同的控制模式
- `is_sync`: 是否同步模式（True/False）
- `velocity_max`: 最大速度列表，维度需匹配规划器（如底盘3维，躯干4维）
- `acceleration_max`: 最大加速度列表
- `jerk_max`: 最大急动度列表
- `velocity_min`: 最小速度列表（可选）
- `acceleration_min`: 最小加速度列表（可选）

---

## 服务说明

**服务**: `/mobile_manipulator_timed_single_cmd`

| 参数 | 说明 |
|------|------|
| `planner_index` | 控制模式索引 |
| `desireTime` | 期望执行时间（秒） |
| `cmdVec` | 命令向量 |

**返回**: `actualTime` - 实际执行时间（秒）

节点会自动等待实际执行时间后再发送下一个关键点。

**规划器参数设置服务**: `/mobile_manipulator_set_ruckig_planner_params`

用于设置各规划器的运动参数，可在运动前调用以调整运动特性。
