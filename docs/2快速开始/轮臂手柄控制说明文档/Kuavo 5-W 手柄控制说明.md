---
title: "Kuavo 5-W 手柄控制说明"
---

# Kuavo 5-W 手柄控制说明

- [Kuavo 5-W 手柄控制说明](#kuavo-5-w-手柄控制说明)
  - [1. 概述](#1-概述)
  - [2. 下位机部署](#2-下位机部署)
    - [2.1 下位机分支编译](#21-下位机分支编译)
    - [2.2 下位机启动（站立）](#22-下位机启动站立)
      - [终端1 - 加载机器人控制器](#终端1---加载机器人控制器)
      - [终端3 - SSH连接并重启服务](#终端3---ssh连接并重启服务)
  - [3. 控制模式](#3-控制模式)
  - [4. 遥控器按键说明](#4-遥控器按键说明)
  - [5. 模式切换](#5-模式切换)
    - [切换按键组合](#切换按键组合)
    - [默认模式](#默认模式)
    - [其他功能按键](#其他功能按键)
  - [6. 模式详细说明](#6-模式详细说明)
    - [6.1 cmd\_vel 模式 / cmd\_vel\_world 模式](#61-cmd_vel-模式--cmd_vel_world-模式)
    - [6.2 躯干控制模式（cmd\_lb\_torso\_pose）](#62-躯干控制模式cmd_lb_torso_pose)
  - [7. 视频演示](#7-视频演示)
  - [8. 使用提示](#8-使用提示)

## 1. 概述

本文档说明如何使用手柄控制轮臂机器人，包括三种控制模式及其切换方式。

## 2. 下位机部署

### 2.1 下位机分支编译

- **下位机仓库**: `kuavo-ros-opensource`

```bash
cd kuavo-ros-opensource #仓库目录
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release # Important! 
source installed/setup.bash # 加载一些已经安装的ROS包依赖环境，包括硬件包等
catkin build  humanoid_controllers
```


### 2.2 下位机启动（站立）

下位机需要开启**4个终端**来完成基础站立功能：

#### 终端1 - 加载机器人控制器

```bash
cd kuavo-ros-opensource
sudo su
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real_wheel.launch joystick_type:=bt2pro
```

#### 终端3 - SSH连接并重启服务

> **密码**: `133233`

```bash
ssh -oKexAlgorithms=+diffie-hellman-group14-sha1 \
    -oHostKeyAlgorithms=+ssh-rsa \
    -oCiphers=+aes128-cbc,3des-cbc \
    ucore@192.168.26.22

sudo systemctl restart urobot.service
```

## 3. 控制模式

系统提供三种控制模式：

1. **cmd_vel 模式**
2. **cmd_vel_world 模式**
3. **躯干控制模式**（cmd_lb_torso_pose）

## 4. 遥控器按键说明

1. 手柄顶部按键示意图

    <img src="./img/controller_top.png" alt="controller_top" width="66.67%" />

    图中标注了手柄顶部的按键位置，包括：
    - **LT**（Left Trigger）：左扳机键
    - **LB**（Left Bumper）：左肩键
    - **RT**（Right Trigger）：右扳机键（图中用红色框标注）
    - **RB**（Right Bumper）：右肩键

2. 手柄正面按键示意图

    <img src="./img/controller_front.png" alt="controller_front" width="66.67%" />

    图中显示了手柄正面的按键布局，包括：
    - **左摇杆**：位于左上方的蓝色摇杆
    - **右摇杆**：位于右下方的蓝色摇杆
    - **D-pad**：方向键
    - **ABXY按钮**：位于右上方的功能按钮（A、B、X、Y）
    - **BACK键**：位于左侧的功能键
    - **START键**：位于右侧的功能键

## 5. 模式切换

### 切换按键组合

| 目标模式 | 按键组合 | 说明 |
|---------|---------|------|
| cmd_vel | LB + A键（BUTTON_STANCE） | 切换到 cmd_vel 模式 |
| cmd_vel_world | LB + Y键（BUTTON_WALK） | 切换到 cmd_vel_world 模式 |
| 躯干控制 | LB + B键（BUTTON_TROT） | 切换到躯干控制模式 |

### 默认模式

系统启动时默认模式为 **cmd_vel** 模式。

### 其他功能按键

| 按键 | 功能 | 说明 |
|-----|------|------|
| BACK键 | 杀掉程序 | 紧急停止功能 |

## 6. 模式详细说明

### 6.1 cmd_vel 模式 / cmd_vel_world 模式

**话题**：`/cmd_vel` 或 `/cmd_vel_world `

**控制方式**：

| 摇杆操作 | 控制参数 | 功能说明 |
|---------|---------|---------|
| 左摇杆上下方向（X轴） | `linear.x` | 前进/后退线速度 |
| 左摇杆左右方向（Y轴） | `linear.y` | 左右移动线速度 |
| 右摇杆左右方向（Yaw轴） | `angular.z` | 绕Z轴旋转角速度 |

**死区**：0.05（摇杆输入值小于死区时视为0）

### 6.2 躯干控制模式（cmd_lb_torso_pose）

**话题**：`/cmd_lb_torso_pose`

**控制方式**：

此模式下需要配合按键使用摇杆进行控制。

| 按键组合 | 摇杆操作 | 控制参数 | 功能说明 | 注意事项 |
|---------|---------|---------|---------|---------|
| LB + 左摇杆 | 上下方向 | `linear.x` | 躯干X位置 | 需要按住LB键才触发控制，松开LB键时保持当前值 |
| LB + 右摇杆 | 上下方向 | `linear.z` | 躯干Z位置 | 需要按住LB键才触发控制，松开LB键时保持当前值 |
| RB + 右摇杆 | 左右方向 | `angular.z` | 躯干Yaw角度 | 需要按住RB键才触发控制，松开RB键时保持当前值 |
| RB + 左摇杆 | 上下方向 | `angular.y` | 躯干Pitch角度 | 需要按住RB键才触发控制，松开RB键时保持当前值 |
| RT键 | - | - | 复位功能 | 按下RT键时，所有躯干控制量清零（复位到初始位置），按下瞬间触发复位，复位后可以继续使用其他控制 |

**死区**：0.05

**特殊说明**：

- 进入躯干控制模式时，系统会自动切换到MPC的ArmOnly模式
- 退出躯干控制模式时，系统会恢复之前的MPC模式
- 所有控制值采用积分方式：摇杆为正则增加，为0则保持不变，为负则减小
- 按下RT键可以快速复位所有躯干控制量到初始位置

## 7. 视频演示

演示视频: [手柄遥控视频](https://www.bilibili.com/video/BV1GVPrz8EFM/?spm_id_from=333.1387.homepage.video_card.click&vd_source=10295fd49ef0dea57383a0d994ea9143 "手柄遥控视频")

## 8. 使用提示

1. **模式切换**：使用LB键配合A/Y/B键进行模式切换，切换时会有控制台输出提示
2. **死区处理**：摇杆输入值小于0.05时会被视为0，避免微小抖动
3. **躯干控制**：
   - 在躯干控制模式下，需要按住对应按键才能调整控制值，松开按键后保持当前值
   - 控制值采用积分方式：摇杆为正则增加，为0则保持不变，为负则减小
   - 按下RT键可以快速复位所有躯干控制量到初始位置
4. **紧急停止**：按下BACK键可以杀死程序
5. **模式切换**：模式切换前要保证手柄控制量为0，且保持2秒钟，之后才能进行模式切换
