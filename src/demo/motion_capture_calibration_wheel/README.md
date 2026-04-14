# Motion Capture Calibration Wheel

动捕标定工具集，用于机器人运动捕获系统的标定和误差分析。

## 📖 项目简介

本项目是一个用于机器人动捕系统标定的完整工具链，通过以下方式实现标定：

1. **轨迹播放**：根据预定义的关键帧配置，生成平滑的机器人运动轨迹并播放
2. **数据采集**：在轨迹播放过程中记录机器人的实际执行数据（位置、姿态、关节角度等）
3. **误差分析**：分析实际数据与期望数据之间的误差，生成详细的统计报告

## ✨ 功能特性

- ✅ **关键帧配置**：支持 YAML 格式的关键帧配置，灵活定义机器人运动轨迹
- ✅ **轨迹插值**：自动对关键帧进行线性插值，生成平滑的 100Hz 轨迹
- ✅ **循环播放**：支持多次循环播放轨迹，便于多次数据采集
- ✅ **关键帧标记**：支持标记关键帧，用于数据分析时的窗口提取
- ✅ **数据提取**：从 rosbag 文件中提取指定时间窗口的数据
- ✅ **统计分析**：计算位置、关节角度的均值、标准差等统计信息
- ✅ **误差计算**：计算实际数据与参考数据之间的误差

## 📁 目录结构

```
motion_capture_calibration_wheel/
├── README.md                    # 本文档
├── run.py                       # 主程序：播放轨迹
├── read_bag_topics.py           # 数据分析程序：读取rosbag并分析
├── config/
│   └── target_joint.yaml        # 关键帧配置文件
├── bag/                         # rosbag 数据存储目录
│   └── *.bag                    # 录制的数据包文件
├── output/                      # 分析结果输出目录
│   └── raw_data_records.txt     # 原始数据记录和统计报告
└── utils/                       # 工具模块
    ├── __init__.py
    ├── load_keyframes.py        # 关键帧加载工具
    ├── interpolation_utils.py   # 插值工具
    ├── publish_keyframes.py     # ROS 话题发布工具
    └── analysis_utils.py        # 数据分析工具
```


## ⚙️ 配置说明

### 关键帧配置文件 (`config/target_joint.yaml`)

关键帧配置文件定义了机器人的运动轨迹，格式如下：

```yaml
trajectory:
  - time: 0                    # 时间（秒）
    torso_pose: [0, 0, 0.4, 0, 0, 0]  # 躯干位姿 [x, y, z, roll, pitch, yaw]
    joint_angles: [20, 0, 0, -90, 0, 0, 0,    # 左臂7个关节（度）
                   20, 0, 0, -90, 0, 0, 0]     # 右臂7个关节（度）

  - time: 4
    torso_pose: [0, 0, 0.8, 0, 0, 0]
    joint_angles: [-20.0, 30.0, -25.0, -20.0, 40.0, -15.0, 25.0,
                   -20.0, -30.0, 25.0, -20.0, -40.0, 15.0, 25.0]
    mark: True                 # 标记此关键帧（用于数据分析）

  - time: 14
    torso_pose: [0, 0, 0.8, 0, 0, 0]
    joint_angles: [-20.0, 30.0, -25.0, -20.0, 40.0, -15.0, 25.0,
                   -20.0, -30.0, 25.0, -20.0, -40.0, 15.0, 25.0]

  - time: 18
    torso_pose: [0, 0, 0.4, 0, 0, 0]
    joint_angles: [20, 0, 0, -90, 0, 0, 0,
                   20, 0, 0, -90, 0, 0, 0]
```

**配置说明**：
- `time`: 关键帧时间点（秒），必须按时间顺序排列
- `torso_pose`: 躯干位姿，6维向量 `[x, y, z, roll, pitch, yaw]`
- `joint_angles`: 14个关节角度（度），前7个为左臂，后7个为右臂
- `mark`: 可选，标记此关键帧用于数据分析窗口提取

**关键帧设计建议**：
- 一般关键帧成对出现：一个用于插值和发布 mark（用于数据分析），一个用于保持姿态（用于动捕记录）
- 关键帧之间的时间间隔建议 ≥ 1 秒
- 标记的关键帧（`mark: True`）应该对应需要重点分析的姿态

## 📝 使用流程

### 步骤 1: 配置关键帧

编辑 `config/target_joint.yaml`，定义需要标定的运动轨迹。

### 步骤 2: 播放轨迹并录制数据

1. **启动机器人**：
```bash
catkin build humanoid_controllers  
source devel/setup.bash 
export ROBOT_VERSION=60
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
```

2. **启动动捕SDK**
catkin build motioncapture
source ./devel/setup.bash
rosrun motioncapture SampleClient

3. **开始录制 rosbag**：
```bash
catkin build kuavo_msgs
source ./devel/setup.bash  
python3 src/demo/motion_capture_calibration_wheel/run.py
```


4. **运行数据分析程序**：
把bag复制到src/demo/motion_capture_calibration_wheel/bag中
```bash
catkin build kuavo_msgs
source ./devel/setup.bash  
python3 src/demo/motion_capture_calibration_wheel/read_bag_topics.py
```

分析结果将保存在 `output/raw_data_records.txt` 文件中。

## 🔧 工具模块

### 1. `run.py` - 轨迹播放主程序

**功能**：
- 加载关键帧配置
- 对关键帧进行线性插值（100Hz）
- 发布轨迹到 ROS 话题
- 支持循环播放
- 发布关键帧标记信号

**参数配置**（在代码中修改）：
```python
loop_count = 5        # 循环次数
loop_interval = 2     # 循环间隔（秒）
```

**发布的 ROS 话题**：
- `/cmd_lb_torso_pose` (Twist): 躯干位姿
- `/kuavo_arm_traj` (JointState): 手臂关节角度
- `/keyframe_flag` (Float32): 关键帧标记（值为关键帧时间，0表示非关键帧）

**调用的服务**：
- `/mobile_manipulator_mpc_control`: 切换到 ArmOnly 模式
- `/enable_lb_arm_quick_mode`: 启用/禁用手臂快速模式

### 2. `read_bag_topics.py` - 数据分析程序

**功能**：
- 从 rosbag 文件中提取数据
- 根据关键帧标记提取时间窗口
- 计算统计信息（均值、标准差）
- 计算误差
- 生成分析报告

**配置参数**（在代码中修改）：
```python
bag_folder = 'src/demo/motion_capture_calibration_wheel/bag'
flag_topic = '/keyframe_flag'      # 关键帧标记话题
offset_sec = 5.0                    # 窗口偏移（秒）
window_sec = 1.0                    # 窗口大小（秒）
```

**分析的话题**：
- `/r_hand_pose`, `/l_hand_pose`: 左右手位置
- `/car_pose`, `/waist_pose`: 车辆和腰部位置
- `/r_shoulder_pose`, `/l_shoulder_pose`: 左右肩位置
- `/head_pose`: 头部位置
- `/joint_cmd`: 关节命令
- `/sensors_data_raw`: 传感器原始数据


## 📊 输出文件

### `output/raw_data_records.txt`

包含以下内容：

1. **原始数据记录**：
   - 按关键帧组织
   - 每个话题的所有数据点
   - 位置数据格式：`[x, y, z]`
   - 关节角度数据格式：数组

2. **均值统计**：
   - 每个话题的位置均值/标准差
   - 每个话题的关节角度均值/标准差
   - 统计信息紧跟在原始数据之后

3. **误差统计**：
   - 不同话题之间的位置误差
   - 误差的均值、标准差、最大值、最小值

**输出格式示例**：
```
====================================================================================================
原始数据记录
====================================================================================================

关键帧 #0 (时间: 4.000s)
----------------------------------------------------------------------------------------------------

话题: /r_hand_pose
  数据条数: 100
  记录 #1:
    位置: [0.123456, 0.234567, 0.345678]
  记录 #2:
    位置: [0.123789, 0.234890, 0.345901]
  ...

  位置均值: [0.123500, 0.234600, 0.345700]
  位置标准差: [0.000100, 0.000200, 0.000300]

====================================================================================================
误差统计
====================================================================================================
...
```

**特别注意**：
1.输出文档中的位置单位为mm，角度的单位为弧度
2.躯干控制的时候，接近目标点会变化的很缓慢，比如目标在0.8m位置，躯干运行到0.7之后，会很慢的变化到0.8,这就导致记录数据的时候躯干位置可能还没到目标位置