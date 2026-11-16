# IMU测试脚本使用说明

## 概述

这个IMU测试脚本支持同时测试Xsens IMU和HIPNUC IMU，并提供ROS数据发布功能。

## 编译方法


```bash
# 确保在工作空间根目录
cd /path/to/kuavo-ros-control

# source ROS环境
source /opt/ros/noetic/setup.bash  # 根据你的ROS版本调整

# 编译依赖包
catkin_make hardware_node

# source工作空间
source devel/setup.bash
```


## 运行方法

```bash
# 1. 启动ROS master
roscore &

# 2. 运行测试脚本
./devel/lib/hardware_node/kuavo_imu_test
```


### 查看发布的ROS话题

```bash
# 查看所有话题
rostopic list

# 查看IMU数据
rostopic echo /imu/data
rostopic echo /kuavo/imu_data
rostopic echo /kuavo/sensors_data

# 查看话题信息
rostopic info /imu/data
```

## 使用界面

### 方法1: 交互式启动脚本（推荐）

使用提供的交互式启动脚本：

```bash
# 进入测试目录
cd tools/check_tool

# 运行交互式启动脚本
./run_imu_test.sh
```

脚本会显示菜单：

```
=========================================
         KUAVO IMU 测试工具
=========================================

请选择要执行的操作:

  1) 测试 Xsens IMU
  2) 测试 HIPNUC IMU
  q) 退出

```

### 方法2: 直接运行测试程序

也可以直接运行编译后的程序：

```bash
# 运行IMU测试脚本
./imu_test_script <IMU类型>

# 示例
./imu_test_script xsens
./imu_test_script hipnuc
```

### 功能说明

**交互式脚本功能**：
1. **测试 Xsens IMU**: 运行Xsens IMU测试
2. **测试 HIPNUC IMU**: 运行HIPNUC IMU测试

**自动测试流程**：
1. **IMU初始化检查**: 验证IMU设备连接和初始化
2. **持续ROS数据发布**: 100Hz频率持续发布IMU数据到ROS话题

## ROS话题说明

脚本会发布以下ROS话题：

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/imu/data` | `sensor_msgs/Imu` | 标准ROS IMU消息 |
| `/kuavo/imu_data` | `kuavo_msgs/imuData` | Kuavo自定义IMU消息 |
| `/kuavo/sensors_data` | `kuavo_msgs/sensorsData` | 完整传感器数据消息 |

### 消息内容

- **加速度计数据**: 三轴加速度 (m/s²)
- **陀螺仪数据**: 三轴角速度 (rad/s)
- **姿态数据**: 四元数表示的姿态
- **无重力加速度**: 去除重力影响的加速度
