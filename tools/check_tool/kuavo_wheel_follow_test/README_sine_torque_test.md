# 正弦波关节力矩控制测试脚本

## 概述

`sine_joint_torque_test.py` 用于对单个关节进行正弦力矩控制测试。通过关节序号选择被测关节，所有参数通过命令行指定。脚本会将非被测关节保持在当前位置，仅对被测关节施加正弦力矩。

## 使用方法

### 1. 编译硬件测试节点

```bash
cd <kuavo-ros-control>
source devel/setup.bash
catkin build hardware_node
```


### 2. 修改电机 kp kd 配置

控制前需要现将测试对应的 ruiwo 电机（EC 电机不用）的 kp，kd 设置为 0，修改路径为 `~/.config/lejuconfig/config.yaml`

将 `parameter` 字段想要调试的 [Left|Right]\_joint_arm_\d 对应项 kp_pos, kd_pos 改为 0。

```yaml
parameter:
  # 关节参数[vel, kp_pos, kd_pos, tor, kp_vel, kd_vel, ki_vel]
  Left_joint_arm_1: [0, 30, 2, 0, 0, 0, 0]
  Left_joint_arm_2: [0, 20, 4, 0, 0, 0, 0]
  Left_joint_arm_3: [0, 20, 4, 0, 0, 0, 0]
  Left_joint_arm_4: [0, 10, 2, 0, 0, 0, 0]
  Left_joint_arm_5: [0, 10, 2, 0, 0, 0, 0]
  Left_joint_arm_6: [0, 10, 2, 0, 0, 0, 0]
  Right_joint_arm_1: [0, 30, 2, 0, 0, 0, 0]
  Right_joint_arm_2: [0, 20, 4, 0, 0, 0, 0]
  Right_joint_arm_3: [0, 20, 4, 0, 0, 0, 0]
  Right_joint_arm_4: [0, 10, 2, 0, 0, 0, 0]
  Right_joint_arm_5: [0, 10, 2, 0, 0, 0, 0]
  Right_joint_arm_6: [0, 10, 2, 0, 0, 0, 0]
  Head_joint_low: [0, 4, 3, 0, 0, 0, 0]
  Head_joint_high: [0, 10, 3, 0, 0, 0, 0]
  Claw_joint_left: [0, 45, 1, 0, 0, 0, 0]
  Claw_joint_right: [0, 45, 1, 0, 0, 0, 0]
```

如想要调试左臂第 2 个关节（左臂第一个 ruiwo 电机），则将 `Left_joint_arm_1` 字段改为

```yaml
Left_joint_arm_1: [0, 0, 0, 0, 0, 0, 0]
```


### 3. 启动硬件节点

需要事先启动 `roscore`

注意：该硬件节点会使所有电机运动到零点位置，务必确保已正确设置零点！

```bash
cd <kuavo-ros-control>
source devel/setup.bash
./devel/lib/hardware_node/hardware_node_jointcmd_test
```

按提示在命令行按 `o` 继续。


可以使用参数将轮臂机器人手臂测试时的初始状态改为向前伸，避免手发生干涉
```bash
cd <kuavo-ros-control>
source devel/setup.bash
./devel/lib/hardware_node/hardware_node_jointcmd_test --default-pose 1
```


### 4. 运行正弦力矩话题发送脚本


```bash
cd <kuavo-ros-control>/tools/check_tool/kuavo_wheel_follow_test/
python3 sine_joint_torque_test.py \
  --joint_index <关节序号> \
  --amplitude <Nm> \
  --sine_freq <Hz> \
  [--duration 3.0] \
  [--frequency 500.0]
```

参数说明：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--joint_index` | int | \ | 被测关节序号（范围：4-19）|
| `--amplitude` | float | \ | 正弦波力矩振幅（牛·米 Nm） |
| `--sine_freq` | float | \ | 正弦波频率（Hz） |
| `--duration` | float | `3.0` | 测试持续时间（秒） |
| `--frequency` | float | `500.0` | 控制频率（Hz） |

例如：

```bash
# 测试关节6，力矩振幅 1.5 Nm，频率 0.2 Hz，持续 5 秒
python3 sine_joint_torque_test.py --joint_index 6 --amplitude 1.5 --sine_freq 0.2 --duration 5
```

## 工作机制

- 被测关节：以 `tau = amplitude * sin(2π * f * t + phase)` 形式输出目标力矩。
- 其他关节：维持当前读取到的位置（位置保持）。
- 控制模式：被测关节使用力矩控制模式，其余关节使用位置保持。

## 输出信息

测试过程中会显示以下信息：

1. 初始化信息：连接状态。
2. 测试配置：被测关节、正弦力矩参数（振幅、频率、相位）。
3. 实时状态：每 0.1 秒显示被测关节的目标力矩、实测力矩、误差。
4. 完成信息：测试结束提示。

### 命令行输出示例

```
[INFO] 单关节测试: 关节[6]
[INFO] 参数: 力矩振幅=1.500Nm, 频率=0.20Hz
[INFO] 设置关节[6]: 力矩振幅=1.500Nm, 频率=0.20Hz, 相位=0.000rad
[INFO] ========== 开始正弦波关节力矩测试 ==========
[INFO] 测试持续时间: 5.0秒
[INFO] 控制频率: 500.0Hz
[INFO] 测试关节: [6]
[INFO]   关节[6]: 力矩振幅=1.500Nm, 频率=0.20Hz, 相位=0.000rad
[INFO] [2.1s] 关节[6]: 目标力矩=1.413Nm, 实测力矩=1.380Nm, 误差=0.033
```

## 注意事项

- 运行前请确认机械结构安全、环境允许施加力矩，避免碰撞和夹伤风险。
- 默认仅支持关节序号范围 `4-19`，若超出范围脚本会报错并退出。
- 力矩振幅请从小到大逐步测试，避免过载。
- 若需要相位参数，可在脚本中扩展或修改 `set_joint_sine_params` 的调用。 