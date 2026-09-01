# 环境搭建
## 基础环境
- Ubuntu 20.04
- ros-noetic 环境需求
- python 3.10 及以上的 python3 版本。

## Xsens Network Streamer 配置

运行 Xsens UDP 接收节点时，MVN Network Streamer 必须同时启用：

- `Position + Orientation (Quaternion)`，对应 `MXTP02`；
- `Scale`，对应 `MXTP13`。

节点需要从 Scale 数据中取得左右肱骨外上髁相对大臂的局部偏移。第一套完整
Scale 就绪前，`/xsense/world_bone_poses` 不发布姿态，并以节流日志提示检查
MVN 配置。运行中收到完整的新 Scale 后，左右偏移会一起切换；已经进入 Pose
FIFO 的旧帧保持不变，后续新帧使用新 Scale。

`/xsense/world_bone_poses` 继续使用
`kuavo_msgs/xsensePoseInfoList`，20 项的名称、顺序和 `segment_id` 均不变。
其中两个肘部条目是项目定义的混合 Pose：

| 条目 | `position` | `orientation` | `segment_id` |
|---|---|---|---:|
| `LeftElbow` | 左肱骨外上髁世界位置 | `Left Forearm` 四元数 | 14 |
| `RightElbow` | 右肱骨外上髁世界位置 | `Right Forearm` 四元数 | 10 |

肱骨外上髁是关键点而不是独立刚体，因此没有自身四元数。这里保留小臂朝向是
现有 GMR 接口的数据契约。

### 方法一：通过 PPA 安装（推荐，如果网络正常）
```bash
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt update
sudo apt install python3.10 python3.10-venv python3.10-dev
```

### 方法二：源码编译安装（如果 PPA 不可用）
```bash
# 安装编译依赖
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  libssl-dev zlib1g-dev \
  libncurses5-dev libncursesw5-dev \
  libreadline-dev libsqlite3-dev \
  libgdbm-dev libdb5.3-dev \
  libbz2-dev libexpat1-dev \
  liblzma-dev tk-dev libffi-dev \
  wget

# 下载并编译 Python 3.10（使用淘宝镜像加速）
cd /usr/src
sudo wget https://npmmirror.com/mirrors/python/3.10.14/Python-3.10.14.tgz
sudo tar -xzf Python-3.10.14.tgz
cd Python-3.10.14
sudo ./configure --enable-optimizations --prefix=/opt/python3.10
sudo make -j"$(nproc)"
sudo make altinstall

# 添加到 PATH（可添加到 ~/.bashrc 永久生效）
export PATH=/opt/python3.10/bin:$PATH

# 验证安装
python3.10 --version
```

## 虚拟运行环境
- 提供了一键部署脚本，可以直接创建可以运行 GMR 开源库的环境
- 执行脚本：
```bash
bash setup_venv.sh
```

## 编译运行
- 直接编译 humanoid_controllers 即可。

### 方式一：一键集成启动（推荐）

PICO GMR 已集成到 `load_kuavo_real.launch` 和 `load_kuavo_mujoco_sim.launch` 中，可通过 `with_pico_gmr:=true` 一键启动。

**注意：PICO 无需指定特定 IP（与 OptiTrack 动捕不同），PICO 通过 UDP 广播自动发现。**

- 真机一键启动（含 PICO GMR）：
```bash
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real.launch with_pico_gmr:=true
```

- Mujoco 仿真一键启动（含 PICO GMR）：
```bash
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch with_pico_gmr:=true
```

- 可选参数：
```bash
# 指定机器人型号（默认 kuavo_s45）
roslaunch humanoid_controllers load_kuavo_real.launch with_pico_gmr:=true pico_robot:=kuavo_s52

# 开启可视化（默认关闭）
roslaunch humanoid_controllers load_kuavo_real.launch with_pico_gmr:=true pico_enable_viewer:=true

# 调整帧率（需在单独启动时通过 pico_streaming.launch 的 fps 参数设置）
# roslaunch kuavo_pico_gmr pico_streaming.launch fps:=30

# 启用 GMR 平滑
roslaunch humanoid_controllers load_kuavo_real.launch with_pico_gmr:=true pico_enable_gmr_smoothing:=true

# 配置 GMR 平滑参数
roslaunch humanoid_controllers load_kuavo_real.launch with_pico_gmr:=true \
  pico_enable_gmr_smoothing:=true \
  pico_gmr_gaussian_sigma:=2.0 \
  pico_gmr_velocity_clip:=5.0 \
  pico_gmr_velocity_deadzone:=0.05
```

### 方式二：单独启动

- 使用新的标准化 launch 文件：
```bash
source devel/setup.bash
roslaunch kuavo_pico_gmr pico_streaming.launch
```

- 单独运行且不启用可视化：
```bash
source devel/setup.bash
roslaunch kuavo_pico_gmr pico_streaming.launch visualize:=false
```

- 使用旧版 launch 文件（向后兼容）：
```bash
source devel/setup.bash
roslaunch kuavo_pico_gmr pico_motion_retarget.launch
```

### 方式三：跟随 VMP 一起启动（旧方式）
- 跟随 vmp 一起启动运行：
```bash
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_vmp_sim.launch
```
- 跟随 vmp 一起启动运行且不启用可视化：
```bash
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_vmp_sim.launch visualize:=false
```

## VMPController 在线遥操作支持

VMPController 已支持 PICO 在线遥操作，通过 `vmp_param.info` 中的 `onlineVRDataSource` 参数配置：

| 参数值 | 数据来源 | 订阅话题 | 消息类型 | DOF |
|--------|---------|---------|---------|-----|
| `"vr_device"` | PICO VR | `/pico/retargeted_pose` | `picoPoseRetarget` | 27 DOF |
| `"mocap"` | OptiTrack 动捕 | `/gmr/vmp_input` | `MocapPoseRetarget` | 27 DOF |
| `"bin_file"` | 离线文件 | - | - | - |

### PICO 与 Mocap 统一 27 DOF 格式

PICO 现在与 Mocap 使用相同的 27 DOF 数据格式，包含 waist 关节：
- 关节布局: Lleg(6) + Rleg(6) + waist(1) + Larm(7) + Rarm(7)
- VMPController 不再需要在 index 12 处插入 waist = 0

### 配置方法
在 `vmp_param.info` 中设置：
```yaml
onlineVRDataSource = "vr_device"   # 使用 PICO
onlineVRControlMode = "upper_body" # 或 "full_body"
```

# 消息定义

## 重定向数据话题及 msg 
- msg 定义：
```bash
# Header with timestamp
std_msgs/Header header

# Base (root) pose in world frame
# Note: geometry_msgs/Pose.orientation is quaternion in (x, y, z, w) format
geometry_msgs/Pose base_link_pose

# Base velocity: [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
float64[6] base_velocity

# Joint positions for 27 DOF (excluding 2 head joints)
# Sequence:
#   Motors 0-5:   Left leg (hip, hip, hip, knee, ankle, ankle)
#   Motors 6-11:  Right leg (hip, hip, hip, knee, ankle, ankle)
#   Motor  12:    Waist (waist_yaw)
#   Motors 13-19: Left arm (shoulder, shoulder, shoulder, elbow, wrist, wrist, wrist)
#   Motors 20-26: Right arm (shoulder, shoulder, shoulder, elbow, wrist, wrist, wrist)
float64[27] joint_position

# Joint velocities for 27 DOF (excluding 2 head joints)
float64[27] joint_velocity

# End effector poses in world frame: [left_foot, right_foot, left_hand, right_hand]
geometry_msgs/Point[4] end_effector_poses
```
- 话题名：`/pico/retargeted_pose`

# 消息质量
- 主要依赖 pico 与接收机之间的网络质量
- 网络质量优的情况下，重定向后的数据更新频率可以稳定在 90 hz

# Pico 使用说明

## Pico App 端启动

打开2个体感追踪器，会闪蓝光，佩戴到左右脚踝处，注意裤子不要遮挡传感器，必要时可以卷起裤脚。

进入 Pico，点击任意左右手柄`HOME`按键，可以看到下方的菜单栏，在左下角**点击打开资源库**，找到并打开**LejuVRTeleopRobot**应用，找不到的话可以看看未知来源的分类。

进入 App，进行校准:
 - 选择全身追踪-->立即校准
 - 或者右上角的立即校准

校准成功，可以在视野前方出现一个由绿色方块组成的人形，可以随意运动看看人形的动作是否一致，如果不一致建议重新校准。

## 遥操作

在 App 左手方向，机器人服务器下拉列表找到对应的机器人，选择连接，点击遥操作即可开始遥操。
