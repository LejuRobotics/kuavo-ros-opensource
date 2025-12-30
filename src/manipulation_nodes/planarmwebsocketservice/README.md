# ARM ACTION SERVER

## 安装

1. 克隆仓库
   
   ```shell
   # 此仓库需要依赖 ocs2_msgs 和 kuavo_ros_interfaces, 请此仓库克隆到在 kuavo-ros-control 或者 kuavo_ros1_workspace 工作空间中
   git clone https://www.lejuhub.com/highlydynamic/planarmwebsocketservice.git
   ```

2. 编译功能包
   
   ```shell
   catkin build humanoid_controllers humanoid_plan_arm_trajectory planarmwebsocketservice
   ```

3. 安装依赖
   
   ```shell
   cd planarmwebsocketservice
   pip install -r requirements.txt
   ```

4. 运行
   
   ```shell
   cd <catkin_workspace>
   source devel/setup.bash
   # kuavo 仿真
   roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
   # bezier 曲线
   roslaunch humanoid_plan_arm_trajectory humanoid_plan_arm_trajectory.launch
   # server 如果是ocs2，请使用参数 robot_type:=ocs2
   roslaunch planarmwebsocketservice plan_arm_action_websocket_server.launch robot_type:=kuavo
   # client 成功后可以在 RViz 和 Gazebo 中看到动作
   python3 ./src/manipulation_nodes/planarmwebsocketservice/scripts/websocket_client_demo.py --action_file_path src/manipulation_nodes/planarmwebsocketservice/action_files/wave_hand.tact
   ```

   action_file_path为保存动作的tact文件名，该文件需要事先保存在manipulation_nodes/planarmwebsocketservice/action_files文件夹中

## 注意事项

1. 运行之前要确认当前功能包有 `action_files`, 且包含需要的 `.tact` 文件。

2. 如果在实机运行，确保 kuavo 程序以 ros 节点运行，并且机器人已经在站立状态。

3. 检查参数设置是否指向同一个位置，确保通信中 MD5 校验通过，如：
   
   - `handler.py` 中的 `ACTION_FILE_FOLDER`
   
   - `plan_arm_action_websocket_server.py` 中的 `ROBOT_ACTION_FILE_FOLDER`
   
   - `websocket_client_demo.py` 中的 `message`

## 手臂动作服务器

这是一个 websocket 服务器在启动后会在所属网络中广播 `robot_info` 话题，内容如下：

```json
{
    "data": {
        "robot_name": ROBOT_NAME,
        "robot_ip": ROBOT_IP,
        "robot_connect_wifi": ROBOT_CONNECT_WIFI,
        "robot_ws_address": ROBOT_WS_ADDRESS,
        "robot_ws_logger_address": ROBOT_WS_LOGGER_ADDRESS,
        "robot_upload_folder": ROBOT_UPLOAD_FOLDER,
        "robot_action_file_folder": ROBOT_ACTION_FILE_FOLDER,
        "robot_username": ROBOT_USERNAME,
        "robot_mac_address": ROBOT_MAC_ADDRESS,
    }
}
```

### 获取机器人信息

request:

```json
{
    "cmd": "get_robot_info"
}
```

response:

```json
{
    "cmd": "get_robot_info",
    "data": {
        "code": 0,
        "robot_type": "40"
    }
}
```

| 名称            | 类型     | 描述             |
| ------------- | ------ | -------------- |
| code          | int    | 错误码 0: 成功      |
| robot_type | string | 机器人的类型         |

### 获取机器人状态

request:

```json
{
    "cmd": "get_robot_status"
}
```

response:

```json
{
    "cmd": "get_robot_status",
    "data": {
        "code": 0,
        "is_run": True
    }
}
```

| 名称     | 类型   | 描述        |
| ------ | ---- | --------- |
| code   | int  | 错误码 0: 成功 |
| is_run | bool | 机器人当前运行状态 |

### 执行脚本命令

request:

```json
{
    "cmd": "run_node",
    "data": {
        "path": "**/main.py"
    }
}
```

| 名称   | 类型     | 描述                      |
| ---- | ------ | ----------------------- |
| path | string | 脚本路径 |

response:

```json
{
    "cmd": "run_node",
    "data": {
        "code": 0,
        "msg": "msg"
    }
}
```

| 名称   | 类型     | 描述                 |
| ---- | ------ | ------------------ |
| code | int    | 错误码 0: 成功 1: 文件不存在 |
| msg  | string | 信息                 |

### 关闭脚本执行

request:

```json
{
    "cmd": "stop_run_node"
}
```

response:

```json
{
    "cmd": "stop_run_node",
    "data": {
        "code": 0
    }
}
```

| 名称   | 类型  | 描述                   |
| ---- | --- | -------------------- |
| code | int | 错误码 0: 成功 1: 超过5s未关闭 |

### 预览动作

request:

```json
{
    "cmd": "preview_action",
    "data": {
        "action_filename": "action_name",
        "action_file_MD5": "action_file_MD5",
    }
}
```

| 名称              | 类型     | 描述      |
| --------------- | ------ | ------- |
| action_filename | string | 动作文件名   |
| action_file_MD5 | string | 动作文件MD5 |

response:

```json
{
    "cmd": "preview_action",
    "data": {
        "code": 0,
        "status": 0,
        "progress": 0
    }
}
```

| 名称       | 类型  | 描述                                                  |
| -------- | --- | --------------------------------------------------- |
| code     | int | 错误码，0: 成功 1: 动作文件不存在 2: 请求动作文件 MD5 与本地动作文件的 MD5 不一致 |
| status   | int | 状态，0: 完成 1: 执行中                                     |
| progress | int | 动作执行进度， 单位为毫秒                                       |

#### 停止预览

request:

```json
{
    "cmd": "stop_preview_action",
}
```

response:

```json
{
    "cmd": "stop_preview_action",
    "data": {
        "code": 0,
    }
}
```

| 名称   | 类型  | 描述        |
| ---- | --- | --------- |
| code | int | 错误码 0: 成功 |

### 下载音频文件后确认由上位机还是下位机来执行

request:

```json
{
    "cmd": "check_music_path",
    "data": {
        "is_reset_cmd": True,
        "music_filename": "music_filename",
    }
}
```

| 名称       | 类型  | 描述                                                  |
| -------- | --- | --------------------------------------------------- |
| is_reset_cmd     | boolean | 是否是重置遥控器配置指令，重置则该值为 True，单独下载音频该值为 False |
| music_filename   | string |   下载音乐文件的名字，单独下载音频的时候传入完整带后缀的文件名，如果改次是重置配置则传入空值  |

response:

```json
{
    "cmd": "check_music_path",
    "data": {
        "code": 0,
        "msg": "Body NUC"
    }
}
```

| 名称       | 类型  | 描述                                                  |
| -------- | --- | --------------------------------------------------- |
| code     | int | 执行结果码，0: 成功，1：失败 |
| msg   | string |   执行结果信息，如果 code 为 0，则返回 Body NUC/Head NUC，表示路径；如果 code 为 1 返回错误信息 |

### 更新 h12 遥控器配置文件

request:

```json
{
    "cmd": "update_h12_config"
}
```

response:

```json
{
    "cmd": "update_h12_config",
    "data": {
        "code": 0,
        "msg": "Body NUC"
    }
}
```

| 名称       | 类型  | 描述                                                  |
| -------- | --- | --------------------------------------------------- |
| code     | int | 执行结果码，0: 成功，1：失败 |
| msg   | string |   执行结果信息，如果 code 为 0，则返回 Body NUC/Head NUC，表示音乐存放路径；如果 code 为 1 返回错误信息 |

### 更新数据采集程序

request:

```json
{
    "cmd": "update_data_pilot"
}
```

response:

```json
{
    "cmd": "update_data_pilot",
    "data": {
        "code": 0,
        "msg": "Success"
    }
}
```

| 名称       | 类型     | 描述                                                  |
| -------- | ------ | --------------------------------------------------- |
| code     | int | 执行结果码，0: 成功，1：失败 |
| msg   | string |   执行结果信息，如果 code 为 0，则返回 "Success"；如果 code 为 1 返回具体错误信息 |

**功能说明：**

该接口用于更新训练场上位机的数据采集程序。系统会：

1. 远程登录到上位机（后面通过 DHCP 配置获取 IP 地址）
2. 在上位机指定目录创建必要的文件夹
3. 下载最新的数据采集程序文件
4. 返回执行结果

**注意事项：**

- 需要确保环境变量 `KUAVO_REMOTE_PASSWORD` 已正确设置（base64 编码的远程登录密码）
- 需要确保网络连接正常，能够访问下载地址
- 上位机需要有足够的磁盘空间存储下载的文件

### logger 日志 websocket

这是一个用于实时获取机器人日志的 websocket 服务。服务器会在启动时广播 `robot_ws_logger_address` 地址，客户端可以通过该地址连接获取实时日志。

#### 连接日志服务

客户端可以通过以下地址连接日志服务：

```
ws://{robot_ws_logger_address}
```

#### 日志消息格式

日志消息格式如下：

```json
{
    "level": "LEVEL",
    "timestamp": "TIMESTAMP",
    "message": "MESSAGE",
    "module": "MODULE",
    "function": "FUNCTION"
}
```

| 名称        | 类型     | 描述                                       |
| --------- | ------ | ---------------------------------------- |
| level     | string | 日志级别 (DEBUG/INFO/WARNING/ERROR/CRITICAL) |
| timestamp | string | 时间戳，格式：YYYY-MM-DD HH:MM:SS.mmm           |
| message   | string | 日志消息内容                                   |
| module    | string | 产生日志的模块名                                 |
| function  | string | 产生日志的函数名                                 |

#### 日志级别说明

| 级别       | 描述              |
| -------- | --------------- |
| DEBUG    | 调试信息，用于开发调试     |
| INFO     | 一般信息，用于记录程序运行状态 |
| WARNING  | 警告信息，表示可能的问题    |
| ERROR    | 错误信息，表示程序错误     |
| CRITICAL | 严重错误，表示程序无法继续运行 |

#### 示例

```json
{
    "level": "INFO",
    "timestamp": "2024-03-21 14:30:45.123",
    "message": "running finish",
    "module": "robot_control",
    "function": "start_robot"
}
```

## YOLO目标检测

`model_utils.py` 中 `YOLO_detection` 为 YOLO目标检测类，用于处理图像检测和结果发布。

### load_model(model_path)

加载YOLO模型。

**参数:**

- `model_path` (str): YOLO模型文件的路径

**返回:**

- model: 加载成功的YOLO模型对象
- None: 加载失败时返回

### get_detections(camera, model)

获取当前图像的目标检测结果。

**参数:**

- `camera` (str): 相机名称
- `model`: YOLO模型对象

**返回:**

- results: 检测结果列表
- None: 无图像数据时返回

### get_max_area_object(results)

从检测结果中返回最大面积的目标。

**参数:**

- `results`: YOLO检测结果

**返回:**

```python
{
    'x': float,      # x坐标
    'y': float,      # y坐标
    'w': float,      # 宽度
    'h': float,      # 高度
    'area': float,   # 面积
    'class_name': str  # 目标名称
}
```

### get_min_area_object(results)

从检测结果中返回最小面积的目标。

**参数:**

- `results`: YOLO检测结果

**返回:**

```python
{
    'x': float,      # x坐标
    'y': float,      # y坐标
    'w': float,      # 宽度
    'h': float,      # 高度
    'area': float,   # 面积
    'class_name': str  # 目标名称
}
```

## Rosbag 到 Tact 文件转换工具

这是一个用于将 ROS bag 文件转换为 Tact 文件格式的工具。它主要用于处理机器人手臂、头部和手指的运动数据，并生成可用于动画或其他目的的 Tact 文件。

## 功能

- 录制手臂、头部和手指的 rosbag 数据
- 将 rosbag 数据转换为 tact 文件
- 数据平滑处理和控制点生成

## 使用方法

1. 运行主程序：
   
   ```
   cd <catkin_workspace>
   source devel/setup.bash
   cd planarmwebsocketservice
   python3 rosbag_to_act_frames.py
   ```

2. 在主菜单中选择所需的操作：
   
   - 录制手臂头部手指 rosbag 数据
   - 将 rosbag 数据转成 tact 文件

3. 按照屏幕上的提示进行操作。

main menu:

![main_menu](./imgs/main_menu.png)

record arm, head and hand rosbag:

![record_arm_head_hand_rosbag](./imgs/record_rosbag.png)

rosbag to tact:

![rosbag_to_tact](./imgs/rosbag_to_tact.png)

## 注意事项

- 确保您有足够的磁盘空间来存储生成的 tact 文件。
- 处理大型 rosbag 文件可能需要较长时间，请耐心等待。

## 故障排除

如果遇到问题，请检查以下几点：

- 确保所有依赖都已正确安装
- 检查 rosbag 文件是否完整且未损坏

## 贡献

欢迎提交 issues 和 pull requests 来帮助改进这个工具。
