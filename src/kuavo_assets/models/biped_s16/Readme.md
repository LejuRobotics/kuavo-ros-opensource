# biped_s16

## 概述

biped_s16 是 Roban 人形机器人的 URDF 描述包，用于定义机器人的运动学和动力学模型。该版本基于 biped_s14，主要修正了头部相机为 RealSense 相机的 URDF 描述。

## 版本说明

- **基础版本**: biped_s14
- **主要更新**: 
  - 修正头部相机为 RealSense 相机模型
  - 更新相机坐标系和基座连接关系

## 目录结构

```
biped_s16/
├── config/                         # 配置文件
│   ├── controllers.yaml           # 控制器配置
│   └── joint_names_biped_s1.yaml  # 关节名称映射
├── launch/                         # 启动文件
│   ├── display.launch             # RViz 可视化启动文件
│   └── gazebo.launch              # Gazebo 仿真启动文件
├── meshes/                         # 3D 网格模型（STL 格式）
│   ├── camera_base.STL            # 相机底座
│   ├── camera.STL                 # 相机主体
│   ├── head_pitch.STL             # 头部俯仰关节
│   ├── head_yaw.STL               # 头部偏航关节
│   ├── torso.STL                  # 躯干
│   ├── waist.STL                  # 腰部
│   ├── l_*/r_*                    # 左右臂、腿部组件
│   └── ...
├── urdf/                           # URDF 机器人描述文件
│   ├── biped_s16.urdf             # 主 URDF 文件
│   ├── biped_s16_gazebo.urdf      # Gazebo 专用 URDF
│   ├── biped_s16_gazebo.xacro     # Gazebo Xacro 宏文件
│   └── drake/                     # Drake 仿真专用 URDF
│       ├── biped_v3.urdf          # 完整模型
│       ├── biped_v3_arm.urdf      # 仅手臂模型
│       ├── biped_v3_full.urdf     # 全身模型
│       ├── Larm.urdf / Rarm.urdf  # 左右手臂
│       └── Lleg.urdf / Rleg.urdf  # 左右腿
├── xml/                            # MuJoCo 仿真描述文件
│   ├── biped_s16.xml              # 机器人模型
│   └── scene.xml                  # 仿真场景
├── rviz/                           # RViz 配置
│   └── urdf.rviz                  # 可视化配置文件
├── roban1.6.png                    # 机器人外观图
├── roban1.6_rviz.png              # RViz 显示效果图
├── package.xml                     # ROS 包描述文件
├── CMakeLists.txt                  # CMake 构建文件
└── Readme.md                       # 本文档
```

## 机器人关节配置

### 头部关节
- `head_yaw`: 头部偏航（左右转动）
- `head_pitch`: 头部俯仰（上下转动）
- `camera`: RealSense 深度相机

### 躯干关节
- `waist`: 腰部旋转关节

### 手臂关节（左右对称）
- `{l/r}_arm_yaw`: 肩部偏航
- `{l/r}_arm_roll`: 肩部横滚
- `{l/r}_arm_pitch`: 肩部俯仰
- `{l/r}_forearm`: 肘关节

### 腿部关节（左右对称）
- `{l/r}_leg_yaw`: 髋部偏航
- `{l/r}_leg_roll`: 髋部横滚
- `{l/r}_leg_pitch`: 髋部俯仰
- `{l/r}_knee`: 膝关节
- `{l/r}_foot_pitch`: 踝关节俯仰
- `{l/r}_foot_roll`: 踝关节横滚

### 连杆机构
- `{l/r}_{l/r}_bar`: 并联连杆
- `{l/r}_{l/r}_tendon`: 腱驱动机构

## 使用方法

### 1. 在 RViz 中查看模型

```bash
roslaunch biped_s16 display.launch
```

这将启动：
- `joint_state_publisher_gui`: 关节状态发布器（带 GUI 滑块）
- `robot_state_publisher`: 机器人状态发布器
- `rviz`: 三维可视化工具

### 2. 在 Gazebo 中仿真

```bash
roslaunch biped_s16 gazebo.launch
```

### 3. 在代码中加载 URDF

```python
# Python
import rospy
from urdf_parser_py.urdf import URDF

robot = URDF.from_parameter_server()
```

```cpp
// C++
#include <urdf/model.h>

urdf::Model model;
model.initParam("robot_description");
```

### 4. 使用 MuJoCo 仿真

```bash
# 加载 MuJoCo 场景
cd xml/
mujoco scene.xml
```

### 5. 使用 Drake 仿真

Drake URDF 文件位于 `urdf/drake/` 目录，可根据需求选择：
- `biped_v3.urdf`: 标准完整模型
- `biped_v3_full.urdf`: 包含所有细节的全身模型
- `biped_v3_arm.urdf`: 仅上肢模型（用于手臂控制测试）

## 依赖项

- `ros-noetic-robot-state-publisher`
- `ros-noetic-joint-state-publisher-gui`
- `ros-noetic-rviz`
- `ros-noetic-gazebo-ros`
- `ros-noetic-xacro`

## 注意事项

1. **相机配置**: 本版本使用 RealSense 相机，请确保相机驱动和参数正确配置
2. **坐标系**: 机器人使用 ROS 标准坐标系（x 前，y 左，z 上）
3. **质量参数**: URDF 中的质量和惯性参数已根据实际机器人测量值设定
4. **碰撞模型**: 碰撞检测使用简化的几何体，以提高仿真性能

## 相关链接

- [Kuavo 机器人文档](../../docs/)
- [biped_s14 版本说明](../biped_s14/)
- [URDF 官方教程](http://wiki.ros.org/urdf/Tutorials)

## 更新日志

### v1.0.0
- 基于 biped_s14 创建
- 修正头部相机为 RealSense 模型
- 更新相机坐标系定义