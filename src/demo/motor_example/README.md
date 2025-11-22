# Motor Example

> !!! WARN: **重要提示：本示例仅适用于开发者使用**

## 项目简介

这是一个手臂电机控制示例项目，包含RuiWo电机控制器的使用示例和ROS1封装。

## 目录结构

```
motor_example/
├── cpp_examples/            # 简单的电机控制示例（C++）
│   ├── motor_example.cpp    # 电机控制源代码
│   └── CMakeLists.txt       # 构建配置文件
└── ros1_ws/                 # ROS1工作空间
    └── src/
        └── ruiwo_controller_example/  # RuiWo控制器ROS包
            ├── src/
            │   └── motor_control_node.cpp   # ROS节点源码
            ├── launch/
            │   └── motor_control.launch     # 启动文件
            ├── CMakeLists.txt              # ROS包构建配置
            └── package.xml                 # ROS包描述文件
```

## 环境搭建与配置

### 前置依赖

在运行示例之前，请确保以下依赖已安装：

```bash
sudo apt-get install libyaml-cpp-dev pkg-config libusb-1.0-0-dev
```

### 环境配置

1. **确认您的机械臂是左臂 or 右臂**

2. 拷贝配置文件到配置目录:

```bash
# 如果有，请先备份您的配置文件
cp ~/.config/lejuconfig/config.yaml ~/.config/lejuconfig/config.yaml.bak
mv ~/.config/lejuconfig/arms_zero.yaml ~/.config/lejuconfig/arms_zero.yaml.bak # 清除备份旧的零点，防止之前不正确的零点造成电机抽搐

# 如果是左臂
mkdir -p ~/.config/lejuconfig
cp left_arm_config.yaml ~/.config/lejuconfig/config.yaml

# 如果是右臂
mkdir -p ~/.config/lejuconfig
cp right_arm_config.yaml ~/.config/lejuconfig/config.yaml
```

3. 由于电机的控制线程会绑定到CPU 7 核心上，因此需要您对该核进行隔核操作，隔核脚本可以参考:

**注意该脚本会隔阂2，3，7核心，如果您只需要7核，请把2，3删除！！**

**注意该脚本会隔阂2，3，7核心，如果您只需要7核，请把2，3删除！！**

**注意该脚本会隔阂2，3，7核心，如果您只需要7核，请把2，3删除！！**

```bash
tools/check_tool/isolate_cores.sh
```

### 标定零点

**注意：您必须要执行一次标定零点，以生成保存零点文件!**

**注意：您必须要执行一次标定零点，以生成保存零点文件!**

**注意：您必须要执行一次标定零点，以生成保存零点文件!**

首先将机械臂各个关节摆正到零点位置，执行如下命令标定（只需要标定一次， 或者您觉得当前零点已经不对了， 那可以重新标定）。

对于 C++ 的示例, 您需要修改源码, 然后重新编译执行:

```cpp
RuiWoActuator("unused", true /* bool is_cali = false*/);
int init_result = actuator.initialize();

```

对于 ROS 的示例:
```bash
sudo su # 需要root权限
source devel/setup.bash
export MOTOR_DEPS_DIR=$(pwd)/../../../../installed/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MOTOR_DEPS_DIR

# 左臂 关键参数 cali_arm:=true
roslaunch ruiwo_controller_example motor_control.launch arm_type:=left cali_arm:=true

# 右臂 cali_arm:=true
roslaunch ruiwo_controller_example motor_control.launch arm_type:=right cali_arm:=true
```

## 快速开始

### 1. 基础C++示例

```bash
cd motor_example/cpp_examples
mkdir build && cd build
cmake ..
make

sudo su # 需要root权限
export MOTOR_DEPS_DIR=$(pwd)/../../../../../installed/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MOTOR_DEPS_DIR
./motor_example
```

这个简单示例会：
- 初始化电机控制模块，会使能电机
- 读取当前电机位置
- 输出位置信息
- 退出，失能电机

### 2. ROS1节点

```bash
cd motor_example/ros1_ws
catkin_make

sudo su # 需要root权限
source devel/setup.bash
export MOTOR_DEPS_DIR=$(pwd)/../../../../installed/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MOTOR_DEPS_DIR

# 左臂
roslaunch ruiwo_controller_example motor_control.launch arm_type:=left

# 右臂
roslaunch ruiwo_controller_example motor_control.launch arm_type:=right
```

## ROS1节点功能

### 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `cali_arm` | bool | false | 是否启用校准模式 |
| `arm_type` | string | "left" | 控制的机械臂类型 ("left" 或 "right") |


我们提供一个简单的正弦脚本，用于演示基于话题控制电机和获取状态:

```bash
# 需要先运行roslaunch启动电机控制节点
# 停止时，先CTRL+C 停止正弦脚本，手臂会慢慢归位
python3 scripts/sin_test.py
```

## 接口说明

### 头文件位置
RuiWo电机控制器的接口头文件位于：
```
installed/include/ruiwo_controller_cxx/include/ruiwo_actuator.h
```

### 主要类和接口

#### RuiWoActuator类
主要的电机控制器类，提供以下关键接口：

**初始化和生命周期**
```cpp
RuiWoActuator(std::string unused, bool cali);  // 构造函数
int initialize();                               // 初始化模块
int enable();                                  // 使能所有电机
int disable();                                 // 失能所有电机
void close();                                  // 关闭和清理
```

## 配置文件说明

您不必关注配置文件中的所有内容，我们主要关注如下内容:

- address.Left_joint_arm_1 : 对应电机的CANID/MOTORID ==> 当电机损坏时，您可以将ID改成0，这样内部就会忽略此电机
- ratio: 不必关心
- online : 不必关系
- parameter 电机参数，目前只用到 kp_pos/kd_pos， 您也可以通过调用 set_joint_gains 来设置电机参数（运行时）
- 其他参数可忽略，不用管


## 故障排除

### 常见问题

1. **无法找到节点**
   ```
   ERROR: cannot launch node of type [ruiwo_controller_example/motor_control_node]
   ```
   解决方案：确保已正确编译并source环境：
   ```bash
   cd ros1_ws
   catkin_make
   source devel/setup.bash
   ```

2. **动态库加载失败**
   ```
   error while loading shared libraries: libbmapi64.so: cannot open shared object file
   ```
   解决方案：检查`installed/lib/`目录中是否存在所需的库文件

3. **终端日志输出电机存在故障码（大于 0x83的)**

如果遇到终端日志输出电机存在故障码（大于 0x83的），这说明电机当前存在严重故障，需要**重新标定零点**（标定会清除故障码）来清除故障码，如果还出现可以多试几次或重启。

还不行，那就试试电机motorevo studio软件清除，或者联系工程师。