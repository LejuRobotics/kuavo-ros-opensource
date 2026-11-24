# sg100_glove_bridge

> **双手手套 + VR 遥操入口：**请阅读 [拳师手套与 Quest3 全身速度 IK](../../wheel_arm_abs_vel/docs/qianshi_glove.md)。
> 两只手套共用一个 USB 接收板。默认以同一串口上的左手帧 ID 12、右手帧 ID 2 识别接收板。
> 本页下文主要保留 SDK 与单手调试说明；其中固定串口和早期验证范围描述不代表当前融合入口的默认行为。启动流程、按键和参数以新用户说明为准。

拳师智能电磁动捕手套 → 乐聚 SG100 灵巧手 ROS 桥（Linux 原生）。

## 1. 功能与数据链

```text
真实拳师手套 (UWB 串口)
  ↓
sg100_glove_node.py (ROS Noetic Python 3.8)
  ↓ subprocess (隔离环境)
sg100_runtime Linux SDK (Python 3.12 + Pinocchio 4.0.0)
  串口解码 → PoseTracker 姿态消歧 → HandRouter 板卡路由 → SG100 IK
  ↓ stdout 行协议: "J seq q0 ... q10"
sg100_glove_node.py 解析校验
  ↓
/sg100_sim_hand_command (std_msgs/Float32MultiArray, 11 维, rad)
  ↓
sg100_mujoco_viewer_float.py → MuJoCo SG100 仿真显示
```

- 串口解码、姿态消歧、板卡路由、IK 全部由包内 Linux SDK 完成，ROS 节点只做子进程管理、协议解析与发布，**不叠加任何 EMA/deadband 滤波**。
- 仿真话题：右手 `/sg100_sim_hand_command`，左手 `/sg100_sim_left_hand_command`。
- `publish_real:=true`（默认）时，左右手 11 维弧度合成 `kuavo_msgs/SG100HandCommand` 发到真机话题 `/sg100_hand_command`，由已运行的 `sg100_hand_driver` 执行。`publish_real:=false` 只保留仿真。
- 当前正式验证右手单手模式；SDK 保留左手算法/INI/URDF（双手输出协议为 `J hand board_id seq q0..q10`，ROS 侧尚未定义对应话题语义，未启用）。

## 2. 文件结构

```text
sg100_glove_bridge/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── sg100_glove_bridge.launch      # 单命令启动全链路
├── scripts/
│   ├── sg100_glove_node.py            # ROS 桥接 + SDK 子进程管理
│   └── sg100_mujoco_viewer_float.py   # MuJoCo 仿真 viewer (已验证)
├── sg100_runtime/                     # 内置 Linux SG100 SDK
│   ├── sg100_live_native.py           # 实时入口
│   ├── run_sg100_native.sh            # SDK 启动脚本
│   ├── setup_sg100_native.sh          # 独立 venv 安装脚本
│   ├── requirements-sg100-native.txt / .lock.txt
│   └── sg100_native/                  # SDK Python 模块 + model/ (URDF+INI)
```

## 3. Python 3.12 独立环境安装

SDK 需要 Python 3.12 + Pinocchio 4.0.0，与 ROS Noetic 的 Python 3.8 完全隔离：

```bash
# 需要系统 python3.12 (Ubuntu 20.04 可用 deadsnakes PPA)
rosed  # 无需; 直接:
bash <pkg>/sg100_runtime/setup_sg100_native.sh
```

脚本会在 `sg100_runtime/.venv_sg100_native` 创建虚拟环境并安装锁定依赖（`requirements-sg100-native.lock.txt`）。虚拟环境**不提交仓库**。

已有可用环境时可直接指定（见 §7），无需重复安装。

## 4. ROS Noetic 依赖

- `rospy`、`std_msgs`
- MuJoCo viewer 另需：`mujoco`（Python 包）、`rospkg`、`kuavo_assets`（heiman_sg100 模型）、`motion_capture_ik`（`mujoco_viz.py` 工具）

## 5. 串口与板卡 ID 配置

- 默认自动枚举 CH340 接收器（921600 8N1）。双手配置要求同一串口观察到两个目标手套帧 ID。启动日志会显示验证通过的实际串口。
- 手头已有确认过的接收器路径时，可用 SDK 单独查看该口报告的 ID。先停掉使用该串口的手套桥。

```bash
pkg=$(rospack find sg100_glove_bridge)
env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH \
  bash "$pkg/sg100_runtime/run_sg100_native.sh" \
  --serial /dev/sg100_glove_receiver --baud 921600 --scan --scan-seconds 3
```

如果没有专用别名，将示例路径换成已确认的手套接收板串口。仅需核对左右归属时，可先只给左手套上电扫描，再给右手套上电扫描。

## 6. catkin 编译

```bash
catkin build sg100_glove_bridge
```

纯 Python/catkin 包，无 C++ 构建目标。

## 7. 单命令启动

```bash
# 使用包内 sg100_runtime/.venv_sg100_native (先跑 §3 安装)
roslaunch sg100_glove_bridge sg100_glove_bridge.launch \
  serial_port:=/dev/ttyUSB0 baud:=921600 side:=right right_id:=2

# 或指定已有 Python 3.12 解释器 (例如旧安装目录中的 venv)
roslaunch sg100_glove_bridge sg100_glove_bridge.launch \
  serial_port:=/dev/ttyUSB0 side:=right right_id:=2 \
  sdk_python:=/home/chenzibo/下载/sg100_linux_20261027/sg100_linux/.venv_sg100_native/bin/python

# 只跑手套+ROS发布, 不开 MuJoCo
roslaunch sg100_glove_bridge sg100_glove_bridge.launch use_mujoco:=false

# 左手套实例（左右各跑一个 launch, 独立串口与板卡 ID）
roslaunch sg100_glove_bridge sg100_glove_bridge.launch \
  side:=left serial_port:=/dev/ttyUSB1 left_id:=<左手板卡ID> use_mujoco:=false
```

解释器优先级：`~sdk_python` > 环境变量 `SG100_PYTHON` > 包内 `sg100_runtime/.venv_sg100_native/bin/python`。

启动后验证：

```bash
rostopic type /sg100_sim_hand_command   # std_msgs/Float32MultiArray
rostopic hz  /sg100_sim_hand_command
rostopic echo -n 1 /sg100_sim_hand_command
```

## 8. MuJoCo 仿真

`use_mujoco:=true`（默认）时 launch 自动启动 `sg100_mujoco_viewer_float.py`：加载 `kuavo_assets` heiman 左右手模型，订阅 `/sg100_sim_hand_command`，右手 11 维按 SDK 关节顺序更新；左手无输入保持零位；命令超时（默认 1 s）保持最后帧姿态。关闭 roslaunch（Ctrl-C）即关闭 viewer，不向真实硬件发布任何指令。

## 9. ROS 话题与 11 维关节顺序

| 话题 | 类型 | 内容 |
|---|---|---|
| `/sg100_sim_hand_command` | `std_msgs/Float32MultiArray` | 右手 11 关节角，rad |
| `/qianshi/glove_left/fingertips` | `std_msgs/Float32MultiArray` | 左手 35 维五指位姿（`--emit-gesture-poses`） |
| `/qianshi/glove_right/fingertips` | `std_msgs/Float32MultiArray` | 右手 35 维五指位姿（`--emit-gesture-poses`） |

五指位姿为 5 指 × `[x,y,z,qw,qx,qy,qz]`（位置 m、四元数 WXYZ、顺序
thumb/index/middle/ring/pinky），来源是五指电磁传感器模块经 PoseTracker
消歧后的跟踪位姿，**不是人体指尖 FK 结果**；仅供 `glove_gesture_node.py`
手势识别使用，不参与 IK。左右手套各运行一个本节点实例（`side=left/right`），
各自独立串口；仅右手实例发布 `/sg100_sim_hand_command`。

顺序（与 SDK `JOINT_ORDER` 一致）：

```text
thumb_j1 thumb_j2 thumb_j3
index_j1 index_j2 index_j3
middle_j1 middle_j2
little_j1 little_j2 little_j3
```

## 10. 串口权限与占用排查

```bash
# 权限: 把用户加入 dialout 组后重新登录
sudo usermod -aG dialout $USER

# 查看占用
fuser /dev/ttyUSB0
lsof /dev/ttyUSB0
```

常见错误：`Permission denied`（权限）、`could not open port`（被遗留进程占用）。

## 11. 停止后释放串口

正常 Ctrl-C 停止 roslaunch 时，ROS 节点会 terminate SDK 子进程并 wait 回收，串口随之释放。若异常残留：

```bash
fuser -k /dev/ttyUSB0
```

## 12. SDK 数值一致性

`sg100_runtime/sg100_native/` 内所有模块与已验证的 20261028 版 SDK 逐字节一致（SHA-256 校验，见 `sg100_runtime/SDK_MANIFEST.txt`）；算法未做任何修改。
