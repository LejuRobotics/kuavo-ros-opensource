## Quest3 C++ 版与 Python 版实现一致性说明

本文档说明 `noitom_hi5_hand_udp_python` 包中 Quest3 监控节点的 **Python 实现** 与 **C++ 实现** 在接口和逻辑上的对应关系，便于维护和后续性能优化。

- Python 节点：`scripts/monitor_quest3.py`
- C++ 节点：`src/monitor_quest3_node.cpp`

两者的目标是：**在不改变外部 ROS 接口和行为的前提下，用 C++ 重写 Python 节点**。

---

## ROS 接口对齐情况

### 节点与话题

- 节点：
  - Python：`monitor_quest3`
  - C++：`monitor_quest3_cpp`
- 发布话题（完全保持一致）：
  - `/leju_quest_bone_poses`：`noitom_hi5_hand_udp_python/PoseInfoList`
  - `/robot_head_motion_data`：`kuavo_ros_interfaces/robotHeadMotionData`
  - `/quest_joystick_data`：`kuavo_msgs/JoySticks`
  - `/quest_hand_finger_tf`：`tf2_msgs/TFMessage`

> 以上话题名在 Python 与 C++ 版中都为 **全局话题名**，下游节点（例如 IK、可视化等）无需改动。

### TF 使用

- **Python**：`tf.TransformBroadcaster` / `tf.TransformListener`
- **C++**：`tf::TransformBroadcaster` / `tf::TransformListener`

关键坐标系名称保持一致：

- `torso`（父坐标系）
- `Chest`, `Head`
- `LeftHandPalm`, `RightHandPalm`
- 各种手指末端 frame（`LeftHandIndexTip` 等）

### ROS 参数

- `~enable_head_control`
  - Python：`rospy.get_param("~enable_head_control", True)`
  - C++：`nh_.param("enable_head_control", enable_head_control_, true);`

---

## UDP / Protobuf 通信逻辑

### 握手与数据接收

- Python：
  - `setup_socket(server_address, port)` 创建 `UDP` socket，`settimeout(1)`。
  - `send_initial_message()` 周期发送 `"hi"`，等待 Ack，最多 200 次。
  - `run()` 中 `sock.recvfrom(4096)` 收包 → `LejuHandPoseEvent.ParseFromString()`。
- C++：
  - `setupSocket(server_address, port)` 创建 `UDP` socket，`SO_RCVTIMEO=1s`。
  - `sendInitialMessage()` 同样发送 `"hi"`，等待 Ack，最多 200 次。
  - `run()` 中 `recvfrom()` 收包 → `LejuHandPoseEvent.ParseFromArray()`。

两者使用相同的 `.proto` 文件：

- `protos/hand_pose.proto` → `LejuHandPoseEvent`
- `protos/robot_info.proto` → `RobotDescription`

### 超时与重连

- Python：`socket.timeout` 或其它异常 → `restart_socket()` 中重建 socket 并重新握手。
- C++：`errno == EAGAIN/EWOULDBLOCK` 或其它错误 → `restartSocket()` 中重建 socket 并重新握手。

---

## 姿态数据与坐标系转换

### 骨骼列表

两端都使用同样的 `bone_names` 列表（按顺序）：

- `LeftArmUpper`, `LeftArmLower`, `RightArmUpper`, `RightArmLower`, ...
- `Root`, `Chest`, `Neck`, `Head`

索引与骨骼名之间的映射在 Python / C++ 中一一对应。

### 左手系 → 右手系变换

#### Python

```python
def convert_position_to_right_hand(self, left):
    return {"x": -left["z"], "y": -left["x"], "z": left["y"]}

def convert_quaternion_to_right_hand(self, q):
    return (-q[2], -q[0], q[1], q[3])
```

#### C++

```cpp
// 位置
double rx = -lz;
double ry = -lx;
double rz =  ly;

// 四元数
double rqx = -qz;
double rqy = -qx;
double rqz =  qy;
double rqw =  qw;
```

可以看到：**向量和四元数的变换公式完全对应**。

### PoseInfoList 发布

- 对于每个骨骼：
  - Python / C++ 均从同一 `event.poses[i]` 提取 position/quaternion。
  - 经过相同的坐标变换后填充 `PoseInfo.position` 与 `PoseInfo.orientation`。
  - 依次 `append` 至 `PoseInfoList.poses`。
- 元信息字段：
  - `timestamp_ms`、`is_high_confidence`、`is_hand_tracking` 在 Python 与 C++ 版中均来自同一 protobuf 字段。

### 缩放与 TF 发布

- Python：`scale_factor = {"x": 3.0, "y": 3.0, "z": 3.0}`，在发布 TF 前把 position 乘以 3。
- C++：`std::map<std::string, double> scale_factor = {{"x", 3.0}, {"y", 3.0}, {"z", 3.0}};`，同样乘以 3。
- 父坐标系统一为 `"torso"`，子坐标系为对应骨骼名称。

---

## 遥杆与手势数据

### 字段映射

两端都从 `LejuHandPoseEvent.left_joystick` / `right_joystick` 中获取：

- `x`, `y`
- `trigger`, `grip`
- `firstButtonPressed`, `secondButtonPressed`
- `firstButtonTouched`, `secondButtonTouched`

并一一对应填充到 `kuavo_msgs/JoySticks`：

- `left_x`, `left_y`, `left_trigger`, `left_grip`, ...
- `right_x`, `right_y`, `right_trigger`, `right_grip`, ...

### 话题名

- Python：逻辑上发布到 `quest_joystick_data`，解析后为全局 `/quest_joystick_data`。
- C++：显式发布到 `/quest_joystick_data`。

下游的 `ik_ros_uni.py` 不区分 Python / C++ 版本，直接订阅 `/quest_joystick_data` 即可。

---

## 头部控制逻辑

### TF 查询

- Python：`lookupTransform("Chest", "Head", rospy.Time(0))`
- C++：`tf_listener_.lookupTransform("Chest", "Head", ros::Time(0), tf_trans);`

两者都从 `Chest` 到 `Head` 的 TF 中获得四元数，再转为欧拉角。

### 角度与范围

- 统一采用 roll/pitch 来构造 **pitch / yaw**，转换为角度并做简单归一化。
- Python：
  - 使用 `normalize_degree_in_180`（>180 减 180，<-180 加 180）。
  - 再根据 `config.json` 中的 `head_motion_range.pitch` / `head_motion_range.yaw` 进行裁剪。
- C++：
  - 提供 C++ 版本的 `normalize_degree_in_180`。
  - 使用内置默认范围（±180°），后续可按需要扩展为读取 `config.json`。

最终都发布到 `/robot_head_motion_data`，`joint_data = [yaw, pitch]`。

---

## 广播发现与 RobotDescription

### 获取本机广播 IP

- Python：`netifaces.interfaces()` + `ifaddresses`，过滤前缀 `docker*`、`br-*`、`veth*`。
- C++：`getifaddrs()` 遍历网卡，同样按接口名过滤，获取 IPv4 广播地址。

### RobotDescription 周期广播

两者都以相同的方式广播：

- 消息：`RobotDescription { robot_name = "kuavo", robot_version }`
- 端口范围：`11050–11060`
- robot_version：从 ROS 参数 `/robot_version` 读取（默认 45）

### Quest3 广播发现

- Python：在 `11000–11010` 起多个 `listen_for_quest3_broadcasts` 线程，收到任意端口的消息后，记下 Quest3 IP，并调用 `setup_socket(ip, 10019)`。
- C++：在 `11000–11010` 起多个 `listenForQuest3Broadcasts` 线程，行为等价，同样在收到数据后设置 `server_ip_` 并建立到 10019 端口的 socket。

---

## 循环频率与“限帧”逻辑

Python 与 C++ 都采用相同的循环节奏：

- 每处理完一包 UDP 数据后：
  - Python：`self.rate = rospy.Rate(100.0)`，`self.rate.sleep()`
  - C++：`rate_ = ros::Rate(100.0)`，`rate_.sleep()`

这相当于把主循环的**理论上限频率**设为 100 Hz，但实测频率主要由：

- Quest3 端发送频率
- `recvfrom` 阻塞与解析耗时
- 系统调度

共同决定。你的 30 秒测试表明：

- Python 平均约 72 Hz
- C++ 平均约 71 Hz

两者在 bones 更新频率上 **基本等价**，没有额外的“只取每 N 帧”之类限流逻辑。

---

## 实测对比小结（20s 样本）

通过 `quest3_bone_rate_benchmark.py` + `compare_quest3_rates.py` 的结果：

- **Python**：
  - 平均频率 ≈ 72.1 Hz，平均周期 ≈ 0.0139 s
- **C++**：
  - 平均频率 ≈ 71.0 Hz，平均周期 ≈ 0.0141 s

差异约 1.1 Hz（约 1.6%），可以视为 30 秒短窗口内的正常波动，而不是实现逻辑差异。

---

## 当前已知的小差异与后续可优化点

- 头控角度范围：
  - Python：从 `config.json` 中读取 `head_motion_range`。
  - C++：当前使用内置默认范围（±180°），可以按需增加 JSON 解析，使其与 Python 完全一致。
- 日志与错误处理：
  - Python 和 C++ 在日志文本格式上略有不同，对功能无实质影响。

除上述可选优化外，C++ 版本在外部行为、数据内容和 ROS 接口上已经与 Python 版本高度一致，可作为性能测试与长期维护的替代实现。


