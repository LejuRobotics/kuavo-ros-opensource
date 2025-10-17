# KUAVO Humanoid WebSocket SDK

[![Version](https://img.shields.io/pypi/v/kuavo-humanoid-sdk.svg)](https://pypi.org/project/kuavo-humanoid-sdk/) [![License](https://img.shields.io/pypi/l/kuavo-humanoid-sdk.svg)](#) [![Supported Python Versions](https://img.shields.io/pypi/pyversions/kuavo-humanoid-sdk.svg)](https://pypi.python.org/pypi/kuavo-humanoid-sdk)

KUAVO Humanoid WebSocket SDK是一个用于控制KUAVO人形机器人的综合Python SDK。该SDK提供了机器人状态管理、手臂和头部控制以及末端执行器操作的接口。它可以通过局域网传输数据以控制机器人，只要安装相应Python包即可。
该SDK旨在简化机器人控制的复杂性，使开发者能够快速实现各种机器人应用。

> **注意**: 该SDK目前仅支持ROS1，暂不支持ROS2。

![image](https://kuavo.lejurobot.com/manual/assets/images/kuavo_4pro-cf84d43f1c370666c6e810d2807ae3e4.png)

## Features 特性

### Robot State Management 机器人状态管理
- **IMU data**: acceleration, angular velocity, euler angles  
    **IMU 数据**: 加速度、角速度、欧拉角  
- **Joint/motor states**: position, velocity, torque  
    **关节/电机状态**: 位置、速度、扭矩  
- **Torso state**: position, orientation, velocity  
    **躯干状态**: 位置、方向、速度  
- **Odometry information**  
    **里程计信息**  
- **End-effector states**:  
    - Gripper (LejuClaw): position, velocity, torque, grasp status  
        **夹爪 (LejuClaw)**: 位置、速度、扭矩、抓握状态  
    - Dexterous hand (QiangNao): position, velocity, torque  
        **灵巧手 (QiangNao)**: 位置、速度、扭矩  
    - Touch Dexterous hand (QiangNao_Touch): position, velocity, torque, touch state  
        **触觉灵巧手 (QiangNao_Touch)**: 位置、速度、扭矩、触觉状态  
    - End-effector position and orientation  
        **末端执行器位置和方向**  
- **Motion states**: stand, walk, step_control  
    **运动状态**: 站立、行走、步态控制  

### Motion Control 运动控制
- **Arm Control 手臂控制**  
    - Joint position control  
        **关节位置控制**  
    - End-effector 6D control via inverse kinematics  
        **通过逆运动学控制末端执行器 6D**  
    - Forward kinematics (FK) for computing end-effector pose  
        **正向运动学 (FK) 用于计算末端执行器姿态**  
    - Keyframe sequence control for complex motions  
        **关键帧序列控制复杂动作**  

- **End-effector Control 末端执行器控制**  
    - Gripper control (position control with configurable velocity and torque)  
        **夹爪控制 (可配置速度和扭矩的位置控制)**  
    - Dexterous hand control 灵巧手控制  
        - Position control  
            **位置控制**  
        - Pre-defined hand gestures (OK, 666, fist, etc.)  
            **预定义手势 (OK、666、拳头等)**  

- **Head Control 头部控制**  
    - Position control  
        **位置控制**  

- **Dynamic Motion Control 动态运动控制**  
    - Stance  
        **站立**  
    - Walking (xy and yaw velocity control)  
        **行走 (xy 和偏航速度控制)**  
    - Stepping (gait switching)  
        **步态切换**  

### Robot Basic Information 机器人基本信息
- **Robot type**: kuavo  
    **机器人类型**: kuavo  
- **Robot version**  
    **机器人版本**  
- **End-effector type**  
    **末端执行器类型**  
- **Joint names**  
    **关节名称**  
- **Total degrees of freedom**: 28  
    **总自由度**: 28  
- **Arm degrees of freedom**: 7 per arm  
    **手臂自由度**: 每只手臂 7 个  
- **Head degrees of freedom**: 2  
    **头部自由度**: 2  
- **Leg degrees of freedom**: 12  
    **腿部自由度**: 12  

## Installation 安装

**提示：对于本 SDK 目前存在两个版本，正式发布版与beta内测版, 他们的区别是：**
- 正式发布版：稳定版，对应[kuavo-ros-opensource](https://gitee.com/leju-robot/kuavo-ros-opensource/)的`master` 分支提供的功能，
- beta内测版：该版本较正式版会激进一些，同时也会提供更丰富的功能，对应[kuavo-ros-opensource](https://gitee.com/leju-robot/kuavo-ros-opensource/)的`beta` 分支提供的功能。

**温馨提示：请务必明确您需要安装的版本，如果您的SDK版本与`kuavo-ros-opensource`未匹配，可能会出现某些功能不可用的错误。**

安装最新的**正式版** Kuavo Humanoid SDK，可以使用 pip：
```bash
pip install kuavo-humanoid-sdk-ws
```

安装最新的**beta版** Kuavo Humanoid SDK，可以使用 pip：
```bash
pip install --pre kuavo-humanoid-sdk-ws

```
对于本地开发安装（可编辑模式），请使用：
```bash
cd src/kuavo_humanoid_sdk_ws
chmod +x install.sh
./install.sh
```

## Upgrade Instructions 升级说明

在升级更新之前，您可以先执行以下命令来查看当前安装的版本：
```bash
# 需要注意home目录和根目录的版本是否一致，两者都可以运行，以运行目录sdk版本为准
pip show kuavo-humanoid-sdk-ws
# Output:
Name: kuavo-humanoid-sdk-ws
Version: 0.1.2
...
```
**提示：如果您的版本号中包含字母`b`，则表示该版本为测试版, 比如`Version: 0.1.2b113`**

**当前为正式版**，升级到最新正式版:
```bash
pip install --upgrade kuavo-humanoid-sdk-ws
```
**当前为beta版**，升级到最新正式版:
```bash
pip install --upgrade --force-reinstall kuavo-humanoid-sdk-ws
# 或者
pip uninstall kuavo-humanoid-sdk-ws && pip install kuavo-humanoid-sdk-ws
```
**当前为正式版/beta版**，升级到最新beta版:
```bash
pip install --upgrade --pre kuavo-humanoid-sdk-ws
```


## Quick Start 快速入门

在运行任何代码之前，请确保通过以下命令之一启动机器人：
- 对于模拟：
    ```bash
    roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
    ```
- 对于真实机器人：
    ```bash
    roslaunch humanoid_controllers load_kuavo_real.launch
    ```

- 启动Websocket服务器（如果使用Websocket通信）：
    详细说明请参考[Websockets通信文档](./Websockts通信.md)

## Documentation 文档

文档以两种格式提供：
- **HTML 格式**：需要通过运行脚本生成。
- **Markdown 格式**：直接查看。

生成文档：
```bash
cd <kuavo-ros-opensource>/src/kuavo_humanoid_websocket_sdk
chmod +x gen_docs.sh
./gen_docs.sh
```


## Examples 示例

在运行任何代码示例之前，请确保启动机器人。

示例代码链接：
- [机器人信息](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_websocket_sdk/examples/atomic_skills/robot_info_example.py)
- [基本机器人控制](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_websocket_sdk/examples/atomic_skills/motion_example.py)
- [末端执行器控制](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_websocket_sdk/examples/atomic_skills/lejuclaw_example.py)
- [手臂控制](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_websocket_sdk/examples/atomic_skills/ctrl_arm_example.py)
- [头部控制](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_websocket_sdk/examples/atomic_skills/ctrl_head_example.py)

## License 许可证

此项目根据 MIT 许可证授权 - 详细内容请参阅 LICENSE 文件。

## Contact & Support 联系方式与支持

- **Email**: edu@lejurobot.com  
- **Website**: [Leju Robotics](https://gitee.com/leju-robot/kuavo-ros-opensource/)  
- **Issue Tracker**: [Issues](https://gitee.com/leju-robot/kuavo-ros-opensource/issues)
