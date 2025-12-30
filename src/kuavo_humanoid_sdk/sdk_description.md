# Kuavo Humanoid SDK

一个用于控制 Kuavo 人形机器人的 Python SDK，提供了机器人状态管理、机械臂和头部控制以及末端执行器操作的接口, 它设计用于在 ROS 环境中工作。

**警告**: 该SDK目前仅支持**ROS1**。暂不支持ROS2。

![Kuavo 4Pro Robot](https://kuavo.lejurobot.com/manual/assets/images/kuavo_4pro-cf84d43f1c370666c6e810d2807ae3e4.png)

## 特性

- 机器人状态管理
  - IMU数据(加速度、角速度、欧拉角)
  - 关节/电机状态(位置、速度、力矩)
  - 躯干状态(位置、姿态、速度)
  - 里程计信息
  - 末端执行器状态:
    - 夹持器(lejuclaw): 位置、速度、力矩、抓取状态
    - 灵巧手(qiangnao): 位置、速度、力矩
    - 触觉灵巧手(qiangnao_touch): 位置、速度、力矩、触觉状态
    - 末端执行器位置和姿态
  - 运动状态: 站立、行走、自定义单步控制

- 运动控制
  - 手臂控制
    - 关节位置控制
    - 通过逆运动学(IK)的末端执行器6D控制
    - 用于计算末端执行器位姿的正运动学(FK)
    - 复杂动作的关键帧序列控制
  - 末端执行器控制
    - 夹持器控制(可配置速度和力矩的位置控制)
    - 灵巧手控制
      - 位置控制
      - 预定义手势(OK、666、握拳等)控制
  - 头部控制
    - 位置控制
  - 躯干控制
    - 高度控制(下蹲)
    - 前/后倾控制
  - 动态运动控制
    - 站立
    - 踏步
    - 行走(xy 和 yaw 偏航速度控制)
    - 自定义单步控制
- 机器人基本信息
  - 机器人类型(kuavo)
  - 机器人版本
  - 末端执行器类型
  - 关节名称
  - 总自由度(28)
  - 手臂自由度(每臂7个)
  - 头部自由度(2个)
  - 腿部自由度(12个)
- 音频接口
  - 播放指定的音频文件
  - 停止播放音频
  - TTS 文本合成音频
- 视觉接口
  - 获取指定坐标系下的 AprilTag 检测数据
- 观测接口
   - 获取机器人控制指令
- 搬箱子策略模块
   - 查找 Apriltag 
   - 到达世界坐标系下的目标点位姿
   - 搬箱子
   - 放置箱子   
## 安装

**注意：目前SDK有两个版本，稳定版和测试版。它们的区别如下：**

- 稳定版：对应 [kuavo-ros-opensource](https://gitee.com/leju-robot/kuavo-ros-opensource/) 仓库 `master` 分支提供的功能。
- 测试版：比官方版本更激进，也提供更丰富的功能，对应 [kuavo-ros-opensource](https://gitee.com/leju-robot/kuavo-ros-opensource/) 仓库 `beta` 分支提供的功能。

**友情提醒：请明确您需要安装的版本。如果您的SDK版本与 `kuavo-ros-opensource` 不匹配，某些功能可能无法使用。**

使用 pip 安装最新的 **稳定版** Kuavo Humanoid SDK：
```bash
pip install kuavo-humanoid-sdk  
```

使用  pip 安装最新的 **测试版** Kuavo Humanoid SDK：
```bash
pip install --pre kuavo-humanoid-sdk  
```

对于本地开发安装（可编辑模式），请使用：
```bash
cd src/kuavo_humanoid_sdk  
chmod +x install.sh  
./install.sh   
```

## 版本升级
在升级版本之前，您可以使用以下命令查看当前安装的版本：
```bash
pip show kuavo-humanoid-sdk  
# Output:  
Name: kuavo-humanoid-sdk  
Version: 1.1.6
...  
```

**提示：如果版本号包含字母 b，则表示是测试版，例如，Version: 0.1.2b113**

从稳定版本更新到最新的稳定版本：
```bash
pip install --upgrade kuavo_humanoid_sdk  
```

从测试版更新到最新的稳定版本：
```bash
pip install --upgrade --force-reinstall kuavo_humanoid_sdk  
# or  
pip uninstall kuavo_humanoid_sdk && pip install kuavo_humanoid_sdk  
```

从测试版/稳定版更新到最新的测试版：
```bash
pip install --upgrade --pre kuavo_humanoid_sdk  
```

## 安装包信息

您可以使用 pip 来查看包信息：
```bash
pip show kuavo-humanoid-sdk
```

## 快速开始

以下是一个简单的示例，用于快速使用 Kuavo Humanoid SDK：

> **警告**: 
>  在运行任何代码示例之前，请确保已经启动机器人， 否则 SDK 无法正常工作：
>   
>   - 如果是命令行启动，则请确保类似下面的命令已经执行:
>       - 仿真模式: ``roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch`` (示例命令)
>       - 真实机器人: ``roslaunch humanoid_controllers load_kuavo_real.launch`` (示例命令)
>   - 如果是 h12 遥控器等启动方式，也请确保已经让机器人启动(站立)

```python3
# Copyright (c) 2025 Leju Robotics. Licensed under the MIT License.
import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot

def main():
    if not KuavoSDK().Init():  # Init! !!! IMPORTANT !!!
        print("Init KuavoSDK failed, exit!")
        exit(1)
    robot = KuavoRobot()    
    
    """ arm reset """
    print("Switching to arm reset mode...")
    robot.arm_reset()
    
    """ stance """
    print("Switching to stance mode...")
    robot.stance()

    """ trot """
    print("Switching to trot mode...")
    robot.trot()
    
    """ walk forward """
    print("Starting forward walk...")
    duration = 4.0  # seconds
    speed = 0.3     # m/s
    start_time = time.time()
    while (time.time() - start_time < duration):
        robot.walk(linear_x=speed, linear_y=0.0, angular_z=0.0)
        time.sleep(0.1)  # Small sleep to prevent busy loop
    
if __name__ == "__main__":
    main()
```

## 文档
我们提供两种文档格式：
- HTML 格式: [docs/html](docs/html), **但是需要您自己在SDK目录下执行`gen_docs.sh`脚本生成**
- Markdown 格式: [docs/markdown](docs/markdown)

您可以在 SDK 的源码目录下执行以下命令生成文档， 文档会输出到 `docs/html` 和 `docs/markdown` 文件夹中：
```bash
cd <kuavo-ros-opensource>/src/kuavo_humanoid_sdk
chmod +x gen_docs.sh
./gen_docs.sh
```

**我们强烈推荐您阅读 `html` 文档， 因为它更适合阅读。**

对于Markdown 文档， 请访问：

https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/docs/markdown/index.md

## 使用示例

> **警告**: 
>  在运行任何代码示例之前，请确保已经启动机器人， 否则 SDK 无法正常工作：
>   
>   - 如果是命令行启动，则请确保类似下面的命令已经执行:
>       - 仿真模式: ``roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch`` (示例命令)
>       - 真实机器人: ``roslaunch humanoid_controllers load_kuavo_real.launch`` (示例命令)
>   - 如果是 h12 遥控器等启动方式，也请确保已经让机器人启动(站立)

### 基本信息示例

一个获取机器人基本信息的示例。

[https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/robot_info_example.py](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/robot_info_example.py)

### 运动控制示例

一个基本示例，用于初始化 SDK 并控制机器人运动。

[https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/motion_example.py](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/motion_example.py)

### 末端执行器控制示例

#### LejuClaw 夹爪

展示如何控制 LejuClaw 夹爪末端执行器的示例，包括位置、速度和力矩控制。

[https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/lejuclaw_example.py](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/lejuclaw_example.py)

#### QiangNao 灵巧手

展示如何控制 QiangNao 灵巧手的示例，这是一个具有多个自由度的灵巧机器人手，可用于复杂的操作任务。

[https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/dexhand_example.py](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/dexhand_example.py)

### 手臂控制示例

展示手臂轨迹控制和目标姿态控制的示例。

[https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/ctrl_arm_example.py](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/ctrl_arm_example.py)

### 手臂正向运动学和逆向运动学示例

展示如何使用正向运动学(FK)从关节角度计算末端执行器位置，以及如何使用逆向运动学(IK)计算实现期望末端执行器姿态所需的关节角度的示例。

[https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/arm_ik_example.py](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/arm_ik_example.py)

### 头部控制示例

展示如何控制机器人头部运动的示例，包括点头(俯仰)和摇头(偏航)动作。

[https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/ctrl_head_example.py](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/ctrl_head_example.py)

### 单步控制示例

展示如何控制机器人按照自定义落足点轨迹运动的示例。

[https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/step_control_example.py](https://gitee.com/leju-robot/kuavo-ros-opensource/tree/master/src/kuavo_humanoid_sdk/examples/step_control_example.py)


## 许可证

本项目使用 MIT 许可证授权，详情信息请查看 LICENSE 文件。

## 联系与支持

如有任何问题、支持需求或错误报告，请通过以下方式联系我们：

- 邮箱: edu@lejurobot.com
- 网站: https://gitee.com/leju-robot/kuavo-ros-opensource/
- 源代码: https://gitee.com/leju-robot/kuavo-ros-opensource/
- 问题追踪: https://gitee.com/leju-robot/kuavo-ros-opensource/issues
