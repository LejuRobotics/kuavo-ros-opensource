# 运动学MPC VR遥操作

本目录包含基于运动学模型预测控制（Kinematic MPC）的VR遥操作系统的启动文件。系统支持Quest3头显进行实时手部追踪和遥操作控制。

## 🎯 系统概述

运动学MPC VR遥操作：

- **运动学模型预测控制（Kinematic MPC）**: 提供平滑的机器人运动控制
- **Quest3 VR头显**: 实时手部追踪和姿态捕获
- **多种控制模式**: 直接控制和增量控制

## 📋 启动文件

### 仿真模式

| 启动文件                                | 控制方式 | 描述                                             |
| --------------------------------------- | -------- | ------------------------------------------------ |
| `kinematic_mpc_vr.launch`             | 直接控制 | 基础仿真模式，手部位置直接映射到机器人末端执行器 |
| `kinematic_mpc_vr_incremental.launch` | 增量控制 | 增量仿真模式，通过手部移动增量控制机器人运动     |

### 实物模式

| 启动文件                                     | 控制方式 | 描述                   |
| -------------------------------------------- | -------- | ---------------------- |
| `kinematic_mpc_vr_real.launch`             | 直接控制 | 实物机器人直接控制模式 |
| `kinematic_mpc_vr_incremental_real.launch` | 增量控制 | 实物机器人增量控制模式 |

## 🔧 控制模式说明

### 直接控制模式

- **特点**: VR中的手部位置直接映射到机器人的末端执行器位置

### 增量控制模式

- **特点**: 通过手部移动的增量来控制机器人运动，支持重新定位

## 🚀 快速开始

### 1. 仿真模式启动

```bash
# 基础仿真模式
roslaunch motion_capture_ik kinematic_mpc_vr.launch ip_address:=<QUEST3_IP>

# 增量仿真模式  
roslaunch motion_capture_ik kinematic_mpc_vr_incremental.launch ip_address:=<QUEST3_IP>
```

### 2. 实物模式启动

⚠️ **注意**: 实物模式需要确保机器人安全，建议先在仿真中熟练操作

```bash
# 实物直接控制
roslaunch motion_capture_ik kinematic_mpc_vr_real.launch ip_address:=<QUEST3_IP>

# 实物增量控制
roslaunch motion_capture_ik kinematic_mpc_vr_incremental_real.launch ip_address:=<QUEST3_IP>
```

## 📝 参数配置

### 基础参数

| 参数名                  | 默认值           | 描述                       |
| ----------------------- | ---------------- | -------------------------- |
| `ip_address`          | `192.168.3.17` | Quest3头显的IP地址         |
| `hand_reference_mode` | `thumb_index`  | 手部参考点模式             |
| `ee_type`             | `qiangnao`     | 末端执行器类型             |
| `control_torso`       | `0`            | 是否控制躯干（0=否，1=是） |
| `incremental_control` | `1`            | 增量控制模式（仅增量版本） |

### 安全保护参数

| 参数名            | 默认值   | 描述                           | 建议范围 |
| ----------------- | -------- | ------------------------------ | -------- |
| `enable_safety` | `1`    | 启用安全保护（0=禁用，1=启用） | 0-1      |
| `max_pos_diff`  | `0.45` | 最大位置差异（米）             | 0.1-1.0  |
| `max_quat_diff` | `1`    | 最大姿态差异（弧度）           | 0.05-0.5 |

### 安全保护参数

| 参数名            | 默认值   | 描述                                  | 建议范围 |
| ----------------- | -------- | ------------------------------------- | -------- |
| `enable_safety` | `true` | 启用安全保护（false=禁用，true=启用） | bool     |
| `max_pos_diff`  | `0.45` | 最大位置差异（米）                    | 0.1-1.0  |
| `max_quat_diff` | `1`    | 最大姿态差异（弧度）                  | 0.5-1    |

### 手部参考点模式

| 模式              | 描述                 | 适用场景           |
| ----------------- | -------------------- | ------------------ |
| `thumb_index`   | 拇指食指中点（默认） | 抓取操作，精细控制 |
| `fingertips`    | 所有手指尖中心       | 精确定位任务       |
| `middle_finger` | 中指尖               | 指向操作，按钮点击 |
| `palm`          | 手掌中心             | 通用操作           |

### 末端执行器类型

| 类型         | 描述       |
| ------------ | ---------- |
| `qiangnao` | 强脑灵巧手 |
| `jodell`   | Jodell夹爪 |
| `lejuclaw` | 乐聚夹爪   |

## 🎮 操作指南

### Quest3手势控制

1. **启动手势**: 双手做"OK"手势并保持1-2秒
2. **停止手势**: 双手做"射击"手势并保持1-2秒

## 📊 使用示例

### 直接控制仿真

```bash

roslaunch motion_capture_ik kinematic_mpc_vr.launch \
    ip_address:=10.10.20.130 \
    hand_reference_mode:=thumb_index \
    ee_type:=qiangnao

```

### 直接控制实物操作

```bash

roslaunch motion_capture_ik kinematic_mpc_vr_real.launch \
    ip_address:=192.168.3.17 \
    hand_reference_mode:=thumb_index \
    ee_type:=qiangnao \
    control_torso:=0

```

### 增量控制仿真

```bash

roslaunch motion_capture_ik kinematic_mpc_vr_incremental.launch \
    ip_address:=192.168.3.17 \
    incremental_control:=1 \
    hand_reference_mode:=fingertips \
    enable_safety:=true \
    max_pos_diff:=0.45 \
    max_quat_diff:=1
```

### 增量控制实物操作

```bash

roslaunch motion_capture_ik kinematic_mpc_vr_incremental_real.launch \
    ip_address:=192.168.3.17 \
    incremental_control:=1 \
    hand_reference_mode:=fingertips \
    enable_safety:=false \
    max_pos_diff:=0.45 \
    max_quat_diff:=1

```


## 📚 补充

### 轮臂机器人

#### 机器人版本

在终端执行 `echo $ROBOT_VERSION` 查看当前设置的版本号，如果没有设置，通过以下设置版本号 (当前轮臂所用的版本号为45)：

1. 在当前终端执行 (临时设置):

```shell
 export ROBOT_VERSION=45
```

2. 将其添加到你的 `~/.bashrc` 或者 `~/.zshrc` 终端配置文件中:
   如执行:
   ```
   echo 'export ROBOT_VERSION=40' >> ~/.bashrc 
   ```

添加到 `~/.bashrc` 文件 ( bash 终端) 末尾，重启终端后生效

#### 编译

执行以下命令开始编译：

```shell
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release # Important! 
# -DCMAKE_ASM_COMPILER=/usr/bin/as 为配置了ccache 必要操作，否则可能出现找不到编译器的情况
catkin build humanoid_controllers #会编译所有依赖项
```

#### 操作

当前可通过直接控制模式来控制轮臂的手臂

在仿真环境中：

```bash
# 基础仿真模式
roslaunch motion_capture_ik kinematic_mpc_vr.launch ip_address:=<QUEST3_IP>
```

在实物环境中：

```bash
# 实物直接控制
roslaunch motion_capture_ik kinematic_mpc_vr_real.launch ip_address:=<QUEST3_IP>
```

### 相关文档

- [Hand Reference Mode配置](./scripts/tools/README_hand_reference.md)
