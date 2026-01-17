# 正弦波关节位置控制测试脚本

## 概述

`sine_joint_position_test.py` 用于单位置跟踪关节测试，使用关节序号选择关节，所有参数通过命令行指定。

## 使用方法

### 1. 基本用法

#### 编译硬件测试节点

```bash
cd <kuavo-ros-control>
source devel/setup.bash
catkin build hardware_node
```

#### 启动硬件节点

需要事先启动 **roscore**

**！该硬件节点会是所有电机运动到零点位置，注意事先设置零点！**

```bash
cd <kuavo-ros-control>
source devel/setup.bash
./devel/lib/hardware_node/hardware_node_jointcmd_test
```

可以使用参数将轮臂机器人手臂测试时的初始状态改为向前伸，避免手发生干涉
```bash
cd <kuavo-ros-control>
source devel/setup.bash
./devel/lib/hardware_node/hardware_node_jointcmd_test --default-pose 1
```

注意 **需要根据提示在命令行按 'o'**

#### 使用正弦 jointcmd 话题发送脚本

```
cd <kuavo-ros-control>/tools/check_tool/kuavo_wheel_follow_test/
python3 sine_joint_position_test.py 
```

这里需要指定测试的关节序号和正弦参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--joint_index` | int | \ | 关节序号 |
| `--amplitude` | float | \ | 正弦波振幅（弧度） |
| `--sine_freq` | float | \ | 正弦波频率（Hz） |
| `--duration` | float | `3.0` | 测试持续时间（秒） |
| `--frequency` | float | `500.0` | 控制频率（Hz） |

例如

```bash
# 测试关节0，自定义振幅和频率
python3 sine_joint_position_test.py --joint_index 0 --amplitude 0.2 --sine_freq 0.125
```


## 输出信息

测试过程中会显示以下信息：

1. **初始化信息**: 连接状态、基准位置设置
2. **测试配置**: 活动关节、正弦波参数（振幅、频率、相位）
3. **实时状态**: 每0.1秒显示关节的目标位置、当前位置、跟踪误差
4. **完成信息**: 测试结束提示

### 命令行输出示例

```
[INFO] 单关节测试: 关节[0]
[INFO] 参数: 振幅=0.300rad(17.2°), 频率=0.50Hz
[INFO] 设置关节[0]: 振幅=0.300, 频率=0.50Hz, 相位=0.000
[INFO] ========== 开始正弦波关节位置测试 ==========
[INFO] 测试持续时间: 10.0秒
[INFO] 控制频率: 100.0Hz
[INFO] 测试关节: [0]
[INFO]   关节[0]: 振幅=0.300rad(17.2°), 频率=0.50Hz, 相位=0.000rad(0.0°)
[INFO] [2.1s] 关节[0]: 目标=0.252, 当前=0.245, 误差=0.007
```