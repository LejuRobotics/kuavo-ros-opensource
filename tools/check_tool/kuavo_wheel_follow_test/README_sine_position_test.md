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
| `--joint_index` | int | \ | 关节序号（从1开始，1-20） |
| `--amplitude` | float | \ | 正弦波振幅（度） |
| `--sine_freq` | float | \ | 正弦波频率（Hz） |
| `--phase` | float | `0.0` | 正弦波相位（度），默认为0 |
| `--duration` | float | `None` | 测试持续时间（秒），不指定则持续运行直到手动退出 |
| `--frequency` | float | `500.0` | 控制频率（Hz） |

例如

```bash
# 测试关节4，振幅0.1度，频率0.25Hz
python3 sine_joint_position_test.py --joint_index 4 --amplitude 0.1 --sine_freq 0.25

# 测试关节1，振幅5度，频率0.5Hz，相位90度
python3 sine_joint_position_test.py --joint_index 1 --amplitude 5.0 --sine_freq 0.5 --phase 90.0

# 测试关节3，振幅10度，频率1.0Hz，持续10秒
python3 sine_joint_position_test.py --joint_index 3 --amplitude 10.0 --sine_freq 1.0 --duration 10.0
```

**注意：所有角度参数（振幅、相位）的单位都是度，不是弧度。例如 `--amplitude 0.1` 表示振幅为0.1度，而不是0.1弧度。**


## 输出信息

测试过程中会显示以下信息：

1. **初始化信息**: 连接状态、基准位置设置
2. **测试配置**: 活动关节、正弦波参数（振幅、频率、相位）
3. **实时状态**: 每0.1秒显示关节的目标位置、当前位置、跟踪误差
4. **完成信息**: 测试结束提示

### 命令行输出示例

```
[INFO] 单关节测试: 关节[4]
[INFO] 参数: 振幅=0.100°, 频率=0.25Hz, 相位=0.000°
[INFO] 设置关节[4]: 振幅=0.100°, 频率=0.25Hz, 相位=0.000°
[INFO] ========== 开始正弦波关节位置测试 ==========
[INFO] 测试持续时间: 持续运行（按Ctrl+C退出）
[INFO] 控制频率: 500.0Hz
[INFO] 测试关节: [4]
[INFO]   关节[4]: 振幅=0.100°, 频率=0.25Hz, 相位=0.000°
[INFO] ========== 步骤1: 移动关节[4]到预设目标位置（5秒插值） ==========
[INFO] 预设目标位置: 0.000度
[INFO] [2.1s] 关节[4]: 目标=0.052°, 当前=0.048°, 误差=0.004°
```