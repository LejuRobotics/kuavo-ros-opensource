# handshake_node_standalone 部署文档

## 一、依赖清单

### 1.1 目标机器人必须具备的条件

| 类别 | 要求 | 说明 |
|---|---|---|
| ROS | ROS Noetic (Python 3) | `rospy`, `std_msgs`, `sensor_msgs` |
| Python 包 | `numpy`, `scipy` | `pip install numpy scipy` |
| IK 服务 | `/ik/two_arm_hand_pose_cmd_srv` | 逆运动学求解，必须在目标机器人上运行 |
| FK 服务 | `/ik/fk_srv` | 正运动学求解，必须在目标机器人上运行 |
| 手臂模式服务 | `/humanoid_change_arm_ctrl_mode` | 切换手臂控制模式 (0/1/2) |
| 电机数据话题 | `/sensor_data_motor/motor_pos` | 读取当前关节角 |
| 握手目标话题 | `/handshake/target` | 上位机发布的 HandshakeTarget 消息 |
| 手臂轨迹话题 | `/kuavo_arm_traj` | 本节点发布，需下游节点消费 |

### 1.2 需要打包的两个包

```
/home/lab/kuavo-ros-opensource/src/
├── kuavo_msgs_mini/                 # 精简消息包（仅 5 msg + 3 srv，零额外依赖）
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── msg/
│   │   ├── HandshakeTarget.msg
│   │   ├── armHandPose.msg
│   │   ├── twoArmHandPose.msg
│   │   ├── ikSolveParam.msg
│   │   └── twoArmHandPoseCmd.msg
│   └── srv/
│       ├── changeArmCtrlMode.srv
│       ├── twoArmHandPoseCmdSrv.srv
│       └── fkSrv.srv
│
└── slave_handshake_new/            # 握手功能包（纯 standalone，零 SDK 依赖）
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    │   └── handshake_standalone.launch
    └── scripts/
        ├── handshake_node_standalone.py
        └── test_handshake_pub.py
```

> **说明**：`kuavo_msgs_mini` 内部包名是 `kuavo_msgs`（Python import 不变），只包含 handshake 节点需要的 5 个消息和 3 个服务定义。只依赖 `std_msgs`，不依赖 `nav_msgs`、`trajectory_msgs` 等，编译不会报缺少依赖。
>
> 如果目标机器上**已有**完整 `kuavo_msgs` 且包含 `HandshakeTarget.msg`，可跳过部署 `kuavo_msgs_mini`，直接用已有的。

---

## 二、部署步骤

### 步骤 1：传输到目标机器人

```bash
# 在本机上
cd /home/lab/kuavo-ros-opensource/src
scp -r kuavo_msgs_mini/ slave_handshake_new/ user@<目标IP>:/tmp/
```

### 步骤 2：移动到工作空间并重命名

```bash
# 在目标机器人上执行
# 重命名为 kuavo_msgs（ROS 要求文件夹名与包名一致）
cp -r /tmp/kuavo_msgs_mini /home/lab/kuavo-ros-opensource/src/kuavo_msgs
cp -r /tmp/slave_handshake_new /home/lab/kuavo-ros-opensource/src/
```

### 步骤 3：编译

`kuavo_msgs` 是消息包，必须先编译生成 Python 消息类，`slave_handshake_new` 才能 import。

```bash
# 1. 加载 ROS 环境
source /opt/ros/noetic/setup.bash

# 2. 编译（二选一）

# 方式 A：只编译这两个包（推荐，不影响工作空间其他包）
cd /home/lab/kuavo-ros-opensource
catkin_make --pkg kuavo_msgs
catkin_make --pkg kuavo_handshake_standalone

# 方式 B：编译整个工作空间
cd /home/lab/kuavo-ros-opensource
catkin_make

# 3. 加载编译产物
source /home/lab/kuavo-ros-opensource/devel/setup.bash
```

> 如果 `catkin_make` 报 `package not found`，说明 `kuavo_msgs` 的 `package.xml` 依赖了目标机上未安装的 ROS 包，按提示 `apt install ros-noetic-<包名>` 即可。

### 步骤 4：验证编译

```bash
rosmsg show kuavo_msgs/HandshakeTarget
# 期望输出：
# std_msgs/Header header
# string hand
# float64 wrist_x
# float64 wrist_y
# float64 wrist_z
```

### 步骤 5：验证运行时依赖

```bash
rosservice list | grep ik          # 应有 /ik/two_arm_hand_pose_cmd_srv 和 /ik/fk_srv
rosservice list | grep arm_ctrl    # 应有 /humanoid_change_arm_ctrl_mode
rostopic list | grep motor_pos     # 应有 /sensor_data_motor/motor_pos
```

### 步骤 6：启动

```bash
# 加载 ROS 环境 + 编译产物
source /opt/ros/noetic/setup.bash
source /home/lab/kuavo-ros-opensource/devel/setup.bash

# 方式 A：roslaunch（推荐）
roslaunch kuavo_handshake_standalone handshake_standalone.launch

# 方式 B：rosrun
rosrun kuavo_handshake_standalone handshake_node_standalone.py
```

看到 `[Handshake] 初始化完成, 等待 /handshake/target ...` 表示启动成功。

### 步骤 7：测试

```bash
rostopic pub /handshake/target kuavo_msgs/HandshakeTarget \
  "{header: {stamp: now, frame_id: 'base_link'}, hand: 'right', wrist_x: 0.6, wrist_y: -0.25, wrist_z: 0.15}"
```

期望日志：
```
[Handshake] 目标 base_link=(0.600 -0.250 0.150) hand=right
[Handshake] FK inactive left_hand pos=(...)
[Handshake] IK 求解成功
[Handshake] Set Arm Mode 2: True
[Handshake] 开始接近 -> right
```

---

## 三、常见问题

| 现象 | 原因 | 解决 |
|---|---|---|
| `不能导入 kuavo_msgs` | 未编译或未 source | `cd /home/lab/kuavo-ros-opensource && catkin_make && source devel/setup.bash` |
| `IK 求解失败` | IK 服务异常或目标超出工作空间 | 检查目标坐标是否在可达范围内 |
| `FK 求解失败` | FK 服务未运行 | 确认 `/ik/fk_srv` 服务存在 |
| `无法进入外部控制模式` | 手臂模式切换服务异常 | 检查 `/humanoid_change_arm_ctrl_mode` |
| 手臂不动 | `/kuavo_arm_traj` 无消费者 | 确认下游轨迹执行节点在运行 |
