# kuavo_mocap_gmr

OptiTrack 动捕数据实时重定向到 Kuavo S52 机器人的 ROS 功能包。

## 功能

- 连接 OptiTrack 动捕系统，接收实时骨骼数据
- 使用 GMR (General Motion Retargeting) 进行运动重定向
- 支持仿真模式（合成运动或录制文件回放）
- 发布 3 个 ROS 话题：
  - `/gmr/vmp_input` - 重定向后的运动数据 (MocapPoseRetarget)
  - `/gmr/skeleton_frame` - 原始骨骼点数据 (SkeletonFrame)
  - `/gmr/rigid_body_desc` - T-pose 静态描述 (RigidBodyDescription, latched)

## 安装

### 1. 创建 Python 3.10 虚拟环境

```bash
cd $(rospack find kuavo_mocap_gmr)
bash scripts/setup_venv.sh
```

或手动创建：
```bash
python3.10 -m venv /path/to/venv
source /path/to/venv/bin/activate
pip install -e $(rospack find kuavo_mocap_gmr)/src/kuavo_mocap_gmr
```

### 2. 编译

```bash
cd ~/catkin_ws
catkin build kuavo_msgs kuavo_mocap_gmr
source devel/setup.bash
```

> **注意**: 消息定义位于 `kuavo_msgs` 包中，需要先编译 `kuavo_msgs`。

## 使用方式

### 方式一：集成启动（推荐）

通过主 launch 文件一起启动，与机器人控制器协同工作：

```bash
# 真实机器人 + OptiTrack 动捕（默认 gmr_use_sim=false）
roslaunch humanoid_controllers load_kuavo_real.launch with_mocap_gmr:=true

# 真实机器人 + 模拟动捕数据（用于测试）
roslaunch humanoid_controllers load_kuavo_real.launch with_mocap_gmr:=true gmr_use_sim:=true

# 自定义 OptiTrack 参数
roslaunch humanoid_controllers load_kuavo_real.launch with_mocap_gmr:=true \
    gmr_server_ip:=192.168.1.100 gmr_client_ip:=192.168.1.50

# MuJoCo 仿真 + 模拟动捕数据（默认 gmr_use_sim=true）
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch with_mocap_gmr:=true

# MuJoCo 仿真 + 真实 OptiTrack 动捕
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch with_mocap_gmr:=true gmr_use_sim:=false

# 仿真模式使用录制的运动文件
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch with_mocap_gmr:=true \
    gmr_data_file:=/path/to/motion.pkl
```

#### 集成启动参数

| 参数 | real 默认值 | sim 默认值 | 说明 |
|------|-------------|------------|------|
| `with_mocap_gmr` | false | false | 启用 GMR 模块 |
| `gmr_use_sim` | false | true | 使用仿真模式 (true) 或 OptiTrack (false) |
| `gmr_robot` | kuavo_s52 | kuavo_s52 | 机器人类型 |
| `gmr_format` | lafan1 | lafan1 | 动捕格式 |
| `gmr_fps` | 100.0 | 30.0 | 发布频率 |
| `gmr_cali_arm_pose_deg` | 75.0 | 75.0 | 标定手臂角度 |
| `gmr_enable_viewer` | false | false | 启用 MuJoCo 可视化 |
| `gmr_server_ip` | 192.168.200.160 | - | OptiTrack 服务器 IP |
| `gmr_client_ip` | 192.168.200.117 | - | 客户端 IP |
| `gmr_use_multicast` | false | - | 使用组播 |
| `gmr_data_file` | "" | "" | 运动文件路径 (.pkl/.bvh) |
| `gmr_loop` | true | true | 循环播放 |
| `gmr_human_height` | 1.75 | 1.75 | 模拟人体身高 (m) |

### 方式二：独立启动

单独启动 GMR 节点：

```bash
# 实时动捕模式
roslaunch kuavo_mocap_gmr gmr_streaming.launch

# 自定义 OptiTrack 服务器地址
roslaunch kuavo_mocap_gmr gmr_streaming.launch server_ip:=192.168.1.100 client_ip:=192.168.1.50

# 仿真模式（合成运动）
roslaunch kuavo_mocap_gmr gmr_sim.launch

# 仿真模式（录制文件）
roslaunch kuavo_mocap_gmr gmr_sim.launch data_file:=/path/to/motion.pkl

# 启用 MuJoCo 可视化
roslaunch kuavo_mocap_gmr gmr_streaming.launch enable_viewer:=true
```

#### Streaming Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `server_ip` | 192.168.200.160 | OptiTrack 服务器 IP |
| `client_ip` | 192.168.200.117 | 客户端 IP |
| `use_multicast` | false | 使用组播 |
| `robot` | kuavo_s52 | 机器人类型 |
| `format` | lafan1 | 动捕格式 |
| `cali_arm_pose_deg` | 75.0 | 标定手臂角度 |
| `fps` | 100.0 | 发布频率 |
| `enable_viewer` | false | 启用可视化 |

#### Sim Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot` | kuavo_s52 | 机器人类型 |
| `format` | lafan1 | 动捕格式 |
| `fps` | 30.0 | 发布频率 |
| `data_file` | "" | 运动文件 (.pkl/.bvh)，空则合成 |
| `loop` | true | 循环播放 |
| `human_height` | 1.75 | 模拟人体身高 (m) |
| `cali_arm_pose_deg` | 75.0 | 标定手臂角度 |
| `enable_viewer` | false | 启用可视化 |

## 话题

```bash
# 查看 VMP 输入数据
rostopic echo /gmr/vmp_input

# 查看骨骼帧
rostopic echo /gmr/skeleton_frame

# 查看刚体描述
rostopic echo /gmr/rigid_body_desc
```

## 消息类型

消息定义位于 `kuavo_msgs` 包中：

```bash
rosmsg show kuavo_msgs/MocapPoseRetarget
rosmsg show kuavo_msgs/SkeletonFrame
rosmsg show kuavo_msgs/RigidBodyFrame
rosmsg show kuavo_msgs/RigidBodyDescription
```

### MocapPoseRetarget (27 DOF)

```
Header header
Pose base_link_pose          # 基座位姿 (position + orientation xyzw)
float64[6] base_velocity     # 基座速度 [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z]
float64[27] joint_position   # 关节角度
float64[27] joint_velocity   # 关节角速度
Point[4] end_effector_poses  # 末端位置 [左脚, 右脚, 左手, 右手]
```

### 关节顺序 (27 DOF)

```
Motor 0-5:   左腿 (hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll)
Motor 6-11:  右腿 (hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll)
Motor 12:    腰部 (waist_yaw)
Motor 13-19: 左臂 (shoulder_pitch/roll/yaw, elbow, wrist_yaw/pitch/roll)
Motor 20-26: 右臂 (shoulder_pitch/roll/yaw, elbow, wrist_yaw/pitch/roll)
```

## 环境配置

如果虚拟环境路径不是默认值，设置环境变量：

```bash
export GMR_VENV_PATH=/your/custom/venv/path
```

## 文件结构

```
kuavo_mocap_gmr/
├── CMakeLists.txt
├── package.xml
├── README.md
├── setup.py
├── config/
│   └── gmr_params.yaml
├── data/
│   └── sample_optitrack.pkl
├── launch/
│   ├── gmr_streaming.launch    # 实时动捕
│   └── gmr_sim.launch          # 仿真模式
├── scripts/
│   ├── gmr_streaming_node.py   # 实时动捕节点
│   ├── gmr_sim_node.py         # 仿真节点
│   ├── create_sample_data.py   # 创建样本数据
│   ├── run_with_venv.sh        # venv 包装脚本
│   ├── setup_venv.sh           # 安装 venv
│   └── setup_env.bash          # 环境设置
└── src/kuavo_mocap_gmr/
    ├── gmr_core/               # GMR 核心代码
    │   ├── ik_configs/         # IK 配置文件
    │   └── optitrack_vendor/   # OptiTrack SDK
    └── assets/kuavo_s52/       # 机器人模型
        ├── urdf/
        └── meshes/
```

## 依赖

- `kuavo_msgs` - 消息定义
- `rospy`, `std_msgs`, `geometry_msgs` - ROS 基础
- Python 3.10 虚拟环境 (GMR 核心模块)
