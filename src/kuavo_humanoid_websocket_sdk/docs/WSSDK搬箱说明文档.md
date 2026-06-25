# WSSDK 搬箱 说明文档

本文档介绍基于 WSSDK（WebSocket SDK）的 Kuavo 人形机器人搬箱子策略，包括环境部署和代码详细说明。

---

## 第一部分：环境部署

### (一) 代码仓库配置

**注意：代码仓库配置只需要执行一次，后续修改参数后直接重新运行程序即可**

**上位机仓库配置**

```bash
cd ~/kuavo_ros_application
git checkout dev
```

**上位机编译**

```bash
cd ~/kuavo_ros_application
source devel/setup.bash
catkin build apriltag_ros kuavo_camera dynamic_biped kuavo_tf2_web_republisher ar_control
```

**上位机安装 rosbridge**

WSSDK 通过 WebSocket 与 ROS 通信，需要安装 rosbridge：

```bash
sudo apt install ros-noetic-rosbridge-suite
```

**下位机代码编译**

- 下载依赖环境，编译过程中缺什么则安装对应依赖，以下只是示例

```bash
# 更新软件源
sudo apt-get update
# grab_box 功能包相关依赖
sudo apt-get install ros-noetic-geographic-msgs -y
# humanoid_controllers 包编译依赖
sudo apt-get install libudev-dev -y
```

- 编译相关功能包：

```bash
cd ~/kuavo-ros-control
sudo su
catkin clean -y
catkin build humanoid_controllers gazebo_sim ar_control mobile_manipulator_controllers kuavo_msgs grab_box
source devel/setup.bash
```

**下位机 WSSDK 安装**

```bash
cd src/kuavo_humanoid_websocket_sdk
chmod +x install.sh
./install.sh
cd ../../
```

安装完 SDK，通过终端指令查看版本：

```bash
pip show kuavo-humanoid-sdk-ws | grep Version
```

### (二) 预设参数调整

⚠️⚠️⚠️ **注意：调整参数后建议先在 gazebo 仿真中进行测试，确认效果无误后再在实物上运行**

- 用户使用时需将 AprilTag ID 和标签尺寸修改为与实际一致

**1. 本地电脑仿真**

```bash
vim /opt/ros/noetic/share/apriltag_ros/config/tags.yaml
```

将 tag 尺寸修改为：

```yaml
standalone_tags:
  [
    {id: 0, size: 0.088, name: 'tag_0'},
    {id: 1, size: 0.088, name: 'tag_1'},
    {id: 2, size: 0.088, name: 'tag_2'},
    {id: 3, size: 0.088, name: 'tag_3'},
    {id: 4, size: 0.088, name: 'tag_4'},
    {id: 5, size: 0.088, name: 'tag_5'},
    {id: 6, size: 0.088, name: 'tag_6'},
    {id: 7, size: 0.088, name: 'tag_7'},
    {id: 8, size: 0.088, name: 'tag_8'},
    {id: 9, size: 0.088, name: 'tag_9'}
  ]
```

**2. 实机运行**

在 `~/kuavo_ros_application` 上位机仓库下修改：

```bash
vim ./src/ros_vision/detection_apriltag/apriltag_ros/config/tags.yaml
```

将 tag 的 `size` 尺寸修改为实际大小（如 0.1 米），注意要与实际 tag 码的长度一致。

**箱子与 tag 码相对位置设定**

在示例代码 `grasp_box_example.py` 中，通过 `AprilTagData` 对象配置目标箱子上的 AprilTag 信息：

```python
target_april_tag = AprilTagData(
    id=[2],           # AprilTag ID
    size=[0.088],     # 标签尺寸(米)
    pose=[PoseQuaternion(
        position=(0.0, -1.0, 0.8),           # 基于 odom 坐标系下的大致位姿
        orientation=(0.0, 0.0, 0.0, 1.0)     # 四元数朝向
    )]
)
```

**箱子信息设定**

通过 `BoxInfo` 对象配置箱子的位置、尺寸和质量信息：

```python
box_info = BoxInfo(
    pose=KuavoPose(
        position=(0.5, 0.0, 0.4),
        orientation=(0.0, 0.0, 0.0, 1.0)
    ),
    size=(0.3, 0.2, 0.15),  # 箱子尺寸(长、宽、高)，单位：米
    mass=1.0                 # 箱子重量，单位：千克
)
```

**策略参数调整**

在 `KuavoGraspBox` 类以及 `grasp_box_example.py` 示例中可调整以下关键参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `search_timeout` | 搜索超时时间（秒） | 20.0 |
| `approach_timeout` | 接近超时时间（秒） | 30.0 |
| `grasp_height_offset` | 抓取高度偏移量（米） | 0.1 |
| `target_distance` | 与目标箱子的期望距离（米） | 0.6 |
| `approach_speed` | 接近速度（米/秒） | 0.2 |
| `head_search_yaws` | 头部搜索偏航角度列表（度） | [-30, -15, 0, 15, 30] |
| `head_search_pitchs` | 头部搜索俯仰角度列表（度） | [12, -12] |
| `FOV_HALF_ANGLE` | 机器人半视场角（度） | 35 |

**SDK 初始化（重要）**

> ⚠️ WSSDK 与 RossDK 的关键区别：运行前必须在代码中调用 `KuavoSDK().Init(websocket_mode=True)` 初始化 WebSocket 连接。

在搬箱子示例 `grasp_box_example.py` 中，需确保 `main()` 函数开头有以下初始化：

```python
from kuavo_humanoid_sdk import KuavoSDK

def main():
    # WSSDK 必须：初始化 WebSocket 连接
    if not KuavoSDK().Init(websocket_mode=True, websocket_host='127.0.0.1', websocket_port=9090):
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot()
    # ... 后续代码
```

参数说明：
- `websocket_mode`：必须设置为 `True`，启用 WebSocket 通信模式
- `websocket_host`：rosbridge WebSocket 服务器 IP，默认为 `127.0.0.1`
- `websocket_port`：rosbridge WebSocket 服务器端口，默认为 `9090`

### (三) 仿真运行

**环境配置**

- 仿真使用时，需确认机器人版本 `ROBOT_VERSION=52`

**上位机运行（WebSocket 服务器）**

> WSSDK 通过 WebSocket 与 ROS 通信，需要上位机启动 TF 转发服务（含 rosbridge）。

- 需要自行配置上位机与自己电脑的主从机通信
- 若以自己电脑为 ROS 主机，需要先运行 `roscore`，再在上位机执行如下操作

- 上位机终端：启动 TF 转发 + rosbridge WebSocket 服务器

```bash
cd ~/kuavo_ros_application
source devel/setup.bash
roslaunch kuavo_tf2_web_republisher start_websocket_server.launch
```

**下位机运行**

⚠️⚠️⚠️ **确保已阅读[(二) 预设参数调整](#二-预设参数调整)部分，并完成相应配置内容的检查**

- 下位机终端一：启动 gazebo 场景

```bash
cd ~/kuavo-ros-control
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_gazebo_manipulate.launch
```

- 下位机终端二：启动 SDK WebSocket 服务节点

```bash
cd ~/kuavo-ros-control
source devel/setup.bash
roslaunch h12pro_controller_node kuavo_humanoid_sdk_ws_srv.launch
```

- 下位机终端三：启动 ar_tag 识别与策略节点

```bash
cd ~/kuavo-ros-control
source devel/setup.bash
roslaunch ar_control robot_strategies.launch
```

> 🚨 第一次启动 gazebo 场景前需要修改 tag 尺寸，详见[(二) 预设参数调整](#二-预设参数调整)

> 🚨 每次启动 gazebo 场景后需要在机器人腰部位置附近给个点光源，否则会找不到 tag

- 下位机终端四：运行搬箱子脚本（二选一）

**策略模式版本（简单顺序执行）：**

```bash
cd ~/kuavo-ros-control
source devel/setup.bash
python3 ./src/kuavo_humanoid_websocket_sdk/examples/strategies/grasp_box_example.py
```

**PyTree 行为树版本（并行感知 + 闭环控制，推荐扭腰搬箱）：**

```bash
cd ~/kuavo-ros-control
source devel/setup.bash
python3 ./src/kuavo_humanoid_websocket_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_new.py
```

### (四) 实物运行

**环境配置**

- 机器人会以站立时的位置作为其坐标系原点，请确保机器人面向要搬运的箱子

**下位机运行**

⚠️⚠️⚠️ **确保已阅读[(二) 预设参数调整](#二-预设参数调整)部分，并完成相应配置内容的检查**

- 1. 下位机终端一：检查并启动 SDK WebSocket 服务节点

首先检查是否已有 `websocket_sdk_start_node` 节点运行：

```bash
rosnode list | grep websocket_sdk_start_node
```

如果没有该节点，可手动启动：

```bash
sudo su
source devel/setup.bash
roslaunch h12pro_controller_node kuavo_humanoid_sdk_ws_srv.launch
```

或重新部署 h12 服务（确保开机自启）：

```bash
cd <kuavo-ros-opensource>/src/humanoid-control/h12pro_controller_node/scripts
sudo su
./deploy_autostart.sh
```

- 2. 下位机终端二：让机器人站立

```bash
sudo su
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real.launch with_mm_ik:=true
```

- 3. 下位机终端三：启动 Tag Tracker 节点

```bash
sudo su
source devel/setup.bash
roslaunch ar_control robot_strategies.launch real:=true
```

**上位机运行（WebSocket 服务器）**

> WSSDK 通过 WebSocket 与 ROS 通信，需要上位机启动 rosbridge WebSocket 服务器。

- 4. 上位机需要先修改 `~/kuavo_ros_application/src/ros_vision/detection_apriltag/apriltag_ros/config/tags.yaml` 文件，将 tag 的 size 尺寸修改为实际大小，比如 0.1 米，并将要识别的箱码 id 填入 tags.yaml 文件中，如下所示：

```yaml
standalone_tags:
    [
        {id: 0, size: 0.1, name: 'tag_0'},
        {id: 1, size: 0.1, name: 'tag_1'},
        {id: 2, size: 0.1, name: 'tag_2'},
        {id: 3, size: 0.1, name: 'tag_3'},
        {id: 4, size: 0.1, name: 'tag_4'},
        {id: 5, size: 0.1, name: 'tag_5'},
        {id: 6, size: 0.1, name: 'tag_6'},
        {id: 7, size: 0.1, name: 'tag_7'},
        {id: 8, size: 0.1, name: 'tag_8'},
        {id: 9, size: 0.1, name: 'tag_9'}
    ]
```

- 5. 上位机终端一：启动摄像头节点

```bash
cd ~/kuavo_ros_application
source devel/setup.bash
roslaunch dynamic_biped load_robot_head.launch use_orbbec:=true
```

- 6. 上位机终端二：启动 rosbridge WebSocket 服务器

```bash
cd ~/kuavo_ros_application
source devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

- 7. 上位机终端三：启动 TF 转发服务

```bash
cd ~/kuavo_ros_application
source devel/setup.bash
roslaunch kuavo_tf2_web_republisher start_websocket_server.launch
```

**下位机运行**

- 8. 下位机终端四：运行 WSSDK 搬箱子示例：

```bash
cd ~/kuavo-ros-control
source devel/setup.zsh
python3 ./src/kuavo_humanoid_websocket_sdk/examples/strategies/grasp_box_example.py
```

**实物运行参数调整**

- 因为每台机的机况不同，箱子也可能不同，因此用户可能需要根据实际情况调整抓取或放置的参数。在 `grasp_box_example.py` 中调整以下参数：

调整与目标箱子的距离：

```python
approach_success = grasp_strategy.walk_approach_target(
    target_april_tag,
    target_distance=0.6,  # 与目标箱子保持的距离，单位米
    approach_speed=0.2    # 接近速度，单位米/秒
)
```

调整搜索模式：

```python
find_success = grasp_strategy.head_find_target(
    target_april_tag,
    max_search_time=15.0,
    search_pattern="rotate_body"  # 可选 "rotate_head" 或 "rotate_body"
)
```

- `rotate_head`：仅通过头部转动搜索目标，机器人身体不动
- `rotate_body`：若目标超出视场角（70°），先旋转身体朝向目标，再通过头部扫描搜索

---

## 第二部分：代码说明

本部分详细介绍 WSSDK 中搬箱子策略的代码结构和实现细节。

### 1. 整体架构

WSSDK 搬箱子策略采用**策略模式（Strategy Pattern）** 设计，核心组件包括：

```
kuavo_humanoid_websocket_sdk/
├── kuavo_humanoid_sdk/
│   ├── kuavo_strategy/
│   │   ├── __init__.py
│   │   ├── kuavo_strategy.py            # 策略基类 KuavoRobotStrategyBase
│   │   └── grasp_box/
│   │       └── grasp_box_strategy.py    # 搬箱子策略 KuavoGraspBox
│   └── kuavo_strategy_pytree/
│       ├── common/
│       │   ├── ros_env.py               # WS/ROS 环境适配层
│       │   ├── data_type.py             # 通用数据类型
│       │   └── robot_sdk.py             # RobotSDK 封装
│       ├── nodes/
│       │   ├── nodes.py                 # 行为树节点（Walk/Percep/Arm等）
│       │   ├── api.py                   # ROS 服务/动作 API 封装
│       │   ├── funcs.py                 # 策略工具函数
│       │   └── utils.py                 # 通用工具
│       ├── configs/                     # 仿真/实机配置
│       ├── pick_place_box/              # 搬箱子 PyTree 案例集合
│       │   ├── case.py                  # 基础搬箱案例
│       │   ├── case_new.py              # 扭腰搬箱案例（推荐）
│       │   ├── case_boxes.py            # 多箱处理案例
│       │   └── case_wheel_*.py          # 轮臂机器人案例
│       └── bt_tree_example.py           # PyTrees 行为树示例
└── examples/
    └── strategies/
        └── grasp_box_example.py         # 策略模式搬箱示例

**WSSDK 提供两种策略架构：**

| 特性 | 策略模式 (Strategy Pattern) | PyTree 行为树模式 |
|------|---------------------------|-------------------|
| 实现位置 | `kuavo_strategy/grasp_box/` | `kuavo_strategy_pytree/pick_place_box/` |
| 通信方式 | WebSocket (通过 KuavoSDK) | WebSocket (通过 ros_env 适配层) |
| 控制流程 | 步骤式顺序调用 | 行为树节点并行/顺序执行 |
| 感知节点 | 按需调用 visual API | PERCEP 节点持续运行 |
| 适用场景 | 简单序列任务，快速上手 | 复杂并行任务，需要闭环感知 |
| 灵活性 | 方法调用，代码清晰直观 | 节点可组合，行为树编排 |

### 2. KuavoGraspBox 类介绍

**文件位置**：[grasp_box_strategy.py](kuavo_humanoid_sdk/kuavo_strategy/grasp_box/grasp_box_strategy.py)

`KuavoGraspBox` 继承自 `KuavoRobotStrategyBase`，实现了完整的箱子抓取策略。

#### 类初始化

```python
class KuavoGraspBox(KuavoRobotStrategyBase):
    def __init__(self, robot, robot_state, robot_tools, robot_vision):
        super().__init__(robot, robot_state, robot_tools, robot_vision)
        self.search_timeout = 20.0       # 搜索超时时间（秒）
        self.approach_timeout = 30.0     # 接近超时时间（秒）
        self.grasp_height_offset = 0.1   # 抓取高度偏移量（米）
```

**初始化参数：**
- `robot`（KuavoRobot）：机器人控制实例，提供运动控制接口
- `robot_state`（KuavoRobotState）：机器人状态实例，提供位置、姿态等状态信息
- `robot_tools`（KuavoRobotTools）：机器人工具实例，提供坐标转换等功能
- `robot_vision`（KuavoRobotVision）：机器人视觉实例，提供 AprilTag 识别能力

#### 关键方法详解

##### head_find_target — 头部搜索目标

```python
def head_find_target(self, target_info: AprilTagData, max_search_time=None,
                     search_pattern="rotate_head", **kwargs) -> bool
```

**功能**：使用头部旋转寻找 AprilTag 目标，支持两种搜索模式。

**参数：**
- `target_info`（AprilTagData）：目标 AprilTag 的信息（ID、尺寸、初始位姿猜测）
- `max_search_time`（float）：最大搜索时间，默认使用 `self.search_timeout`（20秒）
- `search_pattern`（str）：搜索模式
  - `"rotate_head"`：仅通过头部转动搜索目标
  - `"rotate_body"`：先检查目标是否在 70° 视场角内，若不在则先旋转身体朝向目标，再进行头部扫描

**执行逻辑：**

1. 获取目标位置和机器人当前位置
2. 计算机器人与目标之间的相对角度
3. 判断目标是否在 FOV 内（70° 视场角，半角 35°）
4. 若不在 FOV 内且为 `rotate_body` 模式，调用 `control_command_pose_world()` 旋转机器人身体，并等待旋转到位
5. 在超时时间内循环执行头部扫描：
   - pitch 角度：[12°, -12°]（两档抬头/低头）
   - yaw 角度：[-30°, -15°, 0°, 15°, 30°]（左右扫描）
   - 每次转头后等待 0.5 秒，通过 `vision.get_data_by_id_from_odom(target_id)` 检查是否识别到目标
6. 找到目标后自动开启头部追踪（`enable_head_tracking`）

**返回值：** `bool` — 是否成功找到目标

##### walk_approach_target — 走路接近目标

```python
def walk_approach_target(self, target_info: AprilTagData, target_distance=0.5,
                         approach_speed=0.15, **kwargs) -> bool
```

**功能**：行走到距离 AprilTag 目标指定距离的位置。

**参数：**
- `target_info`（AprilTagData）：目标 AprilTag 信息
- `target_distance`（float）：与目标的期望距离，默认 0.5 米
- `approach_speed`（float）：接近速度，默认 0.15 米/秒

**执行逻辑：**

1. 通过视觉获取目标在 ODOM 坐标系下的位姿
2. 调用 `_approach_target()` 内部方法，计算目标站立位置：
   - 根据目标位姿和目标距离，计算在目标前方 `target_distance` 处的站位
   - 计算面向目标所需的偏航角
3. 调用 `control_command_pose_world()` 发送行走目标
4. 通过 `_yaw_check()` 和 `_pos_check()` 闭环检查是否到位

**返回值：** `bool` — 是否成功接近目标

##### arm_move_to_target — 手臂移动到目标位置

```python
def arm_move_to_target(self, target_pose: KuavoPose, approach_speed=0.15,
                       **kwargs) -> bool
```

**功能**：控制手臂末端执行器移动到指定目标位姿（闭环控制）。目前为占位方法，可根据需要扩展实现。

**参数：**
- `target_pose`（KuavoPose）：目标位姿，包含 `position`（x, y, z）和 `orientation`（四元数）
- `approach_speed`（float）：接近速度（米/秒）

**返回值：** `bool` — 是否成功移动到目标

##### arm_transport_target_up — 手臂搬起箱子

```python
def arm_transport_target_up(self, target_info: BoxInfo,
                            arm_mode="manipulation_mpc") -> bool
```

**功能**：搬运箱子，控制手臂将箱子提起。目前为占位方法，可根据需要扩展实现。

**参数：**
- `target_info`（BoxInfo）：目标箱子信息（位姿、尺寸、质量）
- `arm_mode`（str）：手臂控制模式，可选 `"manipulation_mpc"` 等

**返回值：** `bool` — 是否成功搬起箱子

##### arm_transport_target_down — 手臂放下箱子

```python
def arm_transport_target_down(self, target_info: BoxInfo,
                              arm_mode="manipulation_mpc") -> bool
```

**功能**：放下箱子，控制手臂将箱子放置到目标位置。目前为占位方法，可根据需要扩展实现。

**参数：**
- `target_info`（BoxInfo）：目标放置位置的箱子信息
- `arm_mode`（str）：手臂控制模式

**返回值：** `bool` — 是否成功放下箱子

#### 辅助方法

##### _wait_for_orientation

```python
def _wait_for_orientation(self, target_angle, max_wait_time=10.0,
                          angle_threshold=0.1) -> bool
```

等待机器人旋转到指定朝向，以 10Hz 频率检查，支持超时。

##### _yaw_check

```python
def _yaw_check(self, yaw_angle_target, angle_threshold=0.1) -> bool
```

检查机器人当前朝向与目标朝向的差异是否小于阈值。

##### _pos_check

```python
def _pos_check(self, pos_target, pos_threshold=0.2) -> bool
```

检查机器人当前位置（x, y）与目标位置的欧氏距离是否小于阈值。

##### _extract_yaw_from_quaternion

```python
def _extract_yaw_from_quaternion(self, quaternion) -> float
```

从四元数（x, y, z, w）提取偏航角（yaw）。

### 3. 数据类型说明

#### AprilTagData

```python
@dataclass
class AprilTagData:
    id: list          # AprilTag ID 列表，如 [2]
    size: list        # 标签物理尺寸列表（米），如 [0.088]
    pose: list        # PoseQuaternion 列表，每个 tag 的位姿信息
```

#### PoseQuaternion

```python
@dataclass
class PoseQuaternion:
    position: Tuple[float, float, float]             # XYZ 坐标（米）
    orientation: Tuple[float, float, float, float]   # 四元数 (x, y, z, w)
```

#### BoxInfo

```python
@dataclass
class BoxInfo:
    pose: KuavoPose                             # 箱子的位姿信息
    size: Tuple[float, float, float] = (0.3, 0.2, 0.15)  # 箱子尺寸 (长, 宽, 高) 米
    mass: float = 1.0                           # 箱子质量（千克）
```

#### KuavoPose

```python
@dataclass
class KuavoPose:
    position: Tuple[float, float, float]             # XYZ 坐标（米）
    orientation: Tuple[float, float, float, float]   # 四元数 (x, y, z, w)
```

### 4. 策略基类 — KuavoRobotStrategyBase

**文件位置**：[kuavo_strategy.py](kuavo_humanoid_sdk/kuavo_strategy/kuavo_strategy.py)

`KuavoRobotStrategyBase` 是所有机器人策略的抽象基类，定义了策略执行的统一接口：

```python
class KuavoRobotStrategyBase(ABC):
    def __init__(self, robot, robot_state, robot_tools, robot_vision):
        self.robot = robot      # 机器人控制
        self.state = robot_state # 机器人状态
        self.tools = robot_tools # 工具
        self.vision = robot_vision # 视觉
```

**抽象方法（子类必须实现）：**

| 方法 | 功能 | 返回值 |
|------|------|--------|
| `head_find_target(target_info, **kwargs)` | 寻找特定 ID 的目标 | `bool` |
| `walk_approach_target(target_info, target_distance, **kwargs)` | 走近目标到指定距离 | `bool` |
| `arm_move_to_target(target_pose, **kwargs)` | 手臂移动到目标位姿 | `bool` |

新的机器人策略可以通过继承 `KuavoRobotStrategyBase` 并实现上述抽象方法来扩展。

### 5. 完整搬箱流程

`grasp_box_example.py` 示例中的搬箱流程如下：

```
1. 初始化机器人
   ├── KuavoSDK().Init()        # SDK 初始化（必须）
   ├── KuavoRobot()             # 机器人控制实例
   ├── KuavoRobotState()        # 状态获取实例
   ├── KuavoRobotTools()        # 工具实例
   └── KuavoRobotVision()       # 视觉实例

2. 配置目标信息
   ├── AprilTagData             # 目标箱子上的 tag 信息
   └── BoxInfo                  # 箱子信息（位姿、尺寸、质量）

3. 寻找目标箱子
   ├── disable_head_tracking()  # 关闭头部追踪
   └── head_find_target()       # 头部扫描寻找目标
       ├── 检查是否在 FOV 内
       ├── [可选] 旋转身体朝向目标
       ├── 头部 pitch/yaw 扫描
       ├── 视觉检测 AprilTag
       └── 开启头部追踪

4. 走路接近目标
   └── walk_approach_target()   # 行走到目标前方指定距离
       ├── 计算站位目标
       ├── control_command_pose_world()
       └── 闭环检查到位

5. 手臂抓取（占位，待扩展）
   ├── arm_move_to_target()     # 手臂移动到抓取位置
   └── arm_transport_target_up()# 提起箱子

6. 手臂放置（占位，待扩展）
   └── arm_transport_target_down() # 放下箱子到目标位置
```

### 6. 示例代码结构

**文件位置**：[grasp_box_example.py](examples/strategies/grasp_box_example.py)

示例代码的核心结构如下：

```python
from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, AprilTagData, PoseQuaternion
from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox, BoxInfo

def main():
    # 步骤0：初始化机器人及相关组件
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    robot_tools = KuavoRobotTools()
    robot_vision = KuavoRobotVision()
    grasp_strategy = KuavoGraspBox(robot, robot_state, robot_tools, robot_vision)

    # 步骤1：寻找目标箱子
    grasp_strategy.head_find_target(target_april_tag, max_search_time=15.0,
                                     search_pattern="rotate_body")

    # 步骤2：走路接近目标
    grasp_strategy.walk_approach_target(target_april_tag, target_distance=0.6,
                                         approach_speed=0.2)

    # 步骤3-5：手臂抓取与放置（占位，待扩展）
    # grasp_strategy.arm_move_to_target(grasp_pose, approach_speed=0.15)
    # grasp_strategy.arm_transport_target_up(box_info, arm_mode="manipulation_mpc")
    # grasp_strategy.arm_transport_target_down(placement_box_info, arm_mode="manipulation_mpc")
```

### 7. 扩展指南

基于 `KuavoRobotStrategyBase` 基类，用户可以轻松实现自定义策略：

```python
class MyCustomStrategy(KuavoRobotStrategyBase):
    def __init__(self, robot, robot_state, robot_tools, robot_vision):
        super().__init__(robot, robot_state, robot_tools, robot_vision)
        # 自定义初始化参数

    def head_find_target(self, target_info, **kwargs):
        # 自定义目标搜索逻辑
        pass

    def walk_approach_target(self, target_info, target_distance=0.5, **kwargs):
        # 自定义接近逻辑
        pass

    def arm_move_to_target(self, target_pose, **kwargs):
        # 自定义手臂移动逻辑
        pass
```

---

## 第三部分：PyTree 行为树策略（高级）

WSSDK 现已集成 `kuavo_strategy_pytree` 行为树策略模块。通过 `ros_env` 环境适配层，PyTree 策略可同时运行在 ROS 原生环境和 WebSocket 环境下。

### 与策略模式的区别

PyTree 行为树模式相比策略模式（`KuavoGraspBox`）具有以下优势：

- **并行执行**：感知节点（PERCEP）与动作节点（ACTION）并行运行，实现持续环境感知
- **闭环控制**：行走过程中持续更新 tag 位置，自适应修正导航目标
- **可视化调试**：运行时可打印行为树状态（`py_trees.display.unicode_tree`）
- **节点复用**：`NodeWalk`、`NodeArm`、`NodePercep` 等节点可自由组合编排

### 仿真运行（Kuavo5 扭腰搬箱子）

**前置条件：** 需在轮臂版仿真环境下运行，确认 `ROBOT_VERSION=52`。

**上位机（WebSocket 服务器）：**

```bash
cd ~/kuavo_ros_application
source devel/setup.bash
roslaunch kuavo_tf2_web_republisher start_websocket_server.launch
```

**下位机：**

```bash
cd ~/kuavo-ros-control
source devel/setup.bash

# 终端一：启动 gazebo 场景
roslaunch humanoid_controllers load_kuavo_gazebo_manipulate.launch

# 终端二：启动 SDK WebSocket 服务节点
roslaunch h12pro_controller_node kuavo_humanoid_sdk_ws_srv.launch

# 终端三：启动 ar_tag 识别与策略节点
roslaunch ar_control robot_strategies.launch

# 终端四：运行 PyTree 搬箱子案例
python3 ./src/kuavo_humanoid_websocket_sdk/kuavo_humanoid_sdk/kuavo_strategy_pytree/pick_place_box/case_new.py
```

### 案例脚本说明

| 脚本 | 说明 | 适用场景 |
|------|------|----------|
| `case_new.py` | 扭腰搬箱子（推荐） | Kuavo5 仿真/实机搬箱 |
| `case.py` | 基础搬箱案例 | 入门学习 |
| `case_boxes.py` | 多箱处理 | 多目标箱子场景 |
| `case_wheel_pick_and_place.py` | 轮臂机器人搬箱+放置 | 完整的 pick & place 流水线 |
| `case_wheel_pick_and_place_time.py` | 带时间控制的搬箱+放置 | 需要精确时序控制的场景 |
| `case_wheel_pick_and_place_by_reach_topic.py` | 通过 reach topic 的搬箱+放置 | 通过话题触发的场景 |

### 初始化要求

> ⚠️ PyTree 案例脚本运行时必须在 `__main__` 中调用 `KuavoSDK.Init(log_level="INFO", websocket_mode=True)` 完成 WebSocket 连接初始化，否则会因无法连接到 ROS 而失败。

代码示例：

```python
from kuavo_humanoid_sdk import KuavoSDK

if __name__ == '__main__':
    # WSSDK 必须：初始化 WebSocket 连接
    if not KuavoSDK.Init(log_level="INFO", websocket_mode=True):
        print("Init KuavoSDK failed, exit!")
        exit(1)
    # ... 后续建树与运行
```

### 配置切换

PyTree 案例通过修改 import 语句切换仿真/实机配置。以 `case_new.py` 为例：

```python
# 仿真：使用 config_sim
from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_sim import config

# 实机：使用 config_boxes_real
# from kuavo_humanoid_sdk.kuavo_strategy_pytree.configs.config_boxes_real import config
```

### 关键参数调整

PyTree 案例在配置文件中设置参数（`configs/config_*.py`），关键参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `common.walk_pos_threshold` | 行走到位阈值（米） | 0.1 |
| `common.arm_pos_threshold` | 手臂到位阈值（米） | 0.02 |
| `common.arm_angle_threshold` | 手臂角度到位阈值（弧度） | 0.05 |
| `common.head_search_yaws` | 头部搜索偏航角度列表（度） | [-30, -15, 0, 15, 30] |
| `common.head_search_pitchs` | 头部搜索俯仰角度列表（度） | [12, -12] |
| `common.waist_degree` | 扭腰角度（度） | 150 |
| `common.grab_box_num` | 搬箱循环次数 | 1 |
| `common.box_width` | 箱子宽度（米） | 0.3 |
| `pick.stand_in_tag_pos` | 相对于 tag 的站位偏移（米） | [0.6, 0.0, 0.0] |
| `pick.backward_mode` | 是否倒退模式 | True |
| `pick.arm_total_time` | 手臂运动总时间（秒） | 3.0 |

### 环境适配层（ros_env）

PyTree 策略通过 `ros_env.py` 实现 WS/ROS 双模式运行：

```python
# 不再直接依赖 rospy
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common import ros_env

# WS 模式和 ROS 模式下均可使用：
ros_env.loginfo("message")           # 日志
pub = ros_env.Publisher(topic, type)  # 发布者
sub = ros_env.Subscriber(topic, type) # 订阅者
```

`ros_env` 在模块加载时自动检测运行环境，无需手动切换代码。

---

## 附录：常见问题

**Q1：运行示例报错 "Init KuavoSDK failed"？**

确保已先启动机器人（仿真或实物），且 WebSocket 服务已正常运行。

**Q2：找不到 AprilTag？**

- 检查 tags.yaml 中的 tag ID 和尺寸是否正确
- 检查 `AprilTagData` 中的 `id` 是否匹配
- 增大 `max_search_time` 搜索超时时间
- 确认 tag 在机器人视场角范围内

**Q3：机器人走路不到位？**

- 调整 `target_distance` 参数
- 调整 `_pos_check()` 中的 `pos_threshold`（默认 0.2 米）
- 调整 `_yaw_check()` 中的 `angle_threshold`（默认 0.1 弧度）

**Q4：手臂抓取/放置功能不完整？**

当前 WSSDK 版本的手臂抓取（`arm_transport_target_up`）和放置（`arm_transport_target_down`）方法为占位实现。如需完整的手臂操作功能，建议参考 RossDK 中的 PyTree 版本实现，或者通过 `robot.control_arm_*` 系列接口自行扩展。
