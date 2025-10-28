# 箱体搬运行为树模块 (grab_box)

## 1. 功能简介
通过AprilTag标签识别或视觉识别模型，得到目标的位姿。然后采用基于行为树（Behavior Tree）的方法，将寻找目标、移动、抓取、放置等行为节点通过逻辑连接，完成箱体搬运任务流程。

## 2. 环境与依赖

### 2.1. 软件依赖
- **BehaviorTree.CPP (v3)**: 本模块依赖`BehaviorTree.CPP`框架。如果环境中未安装，可通过以下命令安装：
  ```bash
  sudo apt install ros-noetic-behaviortree-cpp-v3
  ```
- **Groot2**: 推荐使用`Groot2`对行为树进行可视化编辑和监控。
- 其他环境配置和本代码库 (`kuavo_ros_control`) 所需一致。

### 2.2. 硬件与配置 (实物)
- **AprilTag标签**: 默认使用AprilTag进行目标定位，请确保标签已正确摆放。
- **AprilTag配置**: AprilTag标签的ID和尺寸在上位机 `kuavo_ros_application` 代码库的 `src/ros_vision/detection_apriltag/apriltag_ros/config/tags.yaml` 文件中配置。请确保该配置与实际使用的标签一致。

## 3. 编译
在catkin工作空间下，执行以下命令编译本功能包以及相关依赖：
```bash
catkin build humanoid_controllers grab_box
```

## 4. 使用说明
本模块支持在仿真和实物机器人上运行。

### 4.1. 仿真

#### 4.1.1. 启动流程
1.  **启动 Launch 文件**
    *   在终端中`source`工作空间：
        ```bash
        source devel/setup.zsh
        # 或 source devel/setup.bash
        ```
    *   运行基础仿真启动指令：
        ```bash
        roslaunch grab_box grab_box_sim.launch
        ```
    *   如需使用北通手柄控制，请根据手柄型号选择以下指令：
        ```bash
        # 北通2
        roslaunch grab_box grab_box_sim.launch joystick_type:=bt2
        # 北通2 Pro
        roslaunch grab_box grab_box_sim.launch joystick_type:=bt2pro
        ```

2.  **控制行为树**
    行为树的启动和停止可以通过**命令行**或**手柄**进行控制。

    *   **命令行控制**:
        - 开启新终端并 `source` 工作空间。
        - **开始执行**: `sh scripts/start_bt_tree.sh`
        - **停止执行**: `sh scripts/stop_bt_tree.sh`

    *   **北通手柄控制**:
        - **开始执行**: `RT + A`
        - **停止执行**: 需要两步操作！
            1.  `RT + B`: 停止行为树运行。
            2.  `RT + X`: 停止手臂运动控制器。
![alt text](./joystick1.jpg)
![alt text](./joystick2.jpg)
### 4.2. 实物机器人

#### 4.2.1. 准备工作
1.  **ROS主从机配置**: 确保上下位机的ROS网络配置正确，可以互相通信。
2.  **机器人零点标定**: 全身关节的零点需标定完成。
    *   腿部零点可通过零点工装完成标定。
    *   头部和手臂的零点可通过视觉标定程序完成，具体请参考[视觉标定文档](../../../scripts/joint_cali/readme.md)。
3.  **代码分支与编译**:
    *   **下位机 (`kuavo_ros_control`)**: 推荐使用 `dev` 分支，并执行过 `catkin build`。
    *   **上位机 (`kuavo_ros_application`)**: 推荐使用 `dev` 分支，并按照其 `README` 说明完成编译。

#### 4.2.2. 启动与控制
> **[IMPORTANT]** 启动时请务必遵循以下顺序：**先启动下位机程序，再启动上位机程序**。

1.  **下位机**
    *   在`kuavo_ros_control`工作空间下，运行启动命令：
      ```bash
      # 基础启动
      roslaunch grab_box grab_box_real.launch

      # 如需使用北通手柄，请根据型号选用：
      # 北通2
      roslaunch grab_box grab_box_real.launch joystick_type:=bt2
      # 北通2 Pro
      roslaunch grab_box grab_box_real.launch joystick_type:=bt2pro
      ```

2.  **上位机**
    *   在 `kuavo_ros_application` 的工作空间下，运行启动命令：
      ```bash
      roslaunch dynamic_biped sensor_apriltag_only_enable.launch
      ```

3.  **控制行为树**
    控制方式与仿真环境一致，可通过**命令行**或**手柄**操作。

    *   **命令行控制**:
        - 开启新终端并 `source` 工作空间。
        - **开始执行**: `sh scripts/start_bt_tree.sh`
        - **停止执行**: `sh scripts/stop_bt_tree.sh`

    *   **北通手柄控制**:
        - **开始执行**: `RT + A`
        - **停止执行**: 需要两步操作！
            1.  `RT + B`: 停止行为树运行。
            2.  `RT + X`: 停止手臂运动控制器。

## 5. 行为树 (Behavior Tree)

### 5.1. 行为树文件
行为树的XML定义文件位于本功能包的 `cfg/` 目录下。可以根据需求修改或创建新的行为树文件。

### 5.2. 可视化与修改
使用`Groot2`可以对箱体搬运的流程进行可视化和修改。
1.  下载并安装 `Groot2`。
2.  启动`Groot2`，并打开需要查看或修改的XML文件。

更多关于Groot2的使用方法，请参考其官方网站：[behaviortree.dev/groot/](https://www.behaviortree.dev/groot/)

### 5.3. 关键节点与参数

#### 5.3.1. 参数配置文件 `bt_config.yaml`
该文件位于 `./cfg/ROBOT_VERSION/bt_config.yaml` 中，`ROBOT_VERSION` 为使用的机器人型号对应的文件夹。文件中主要包含所使用的行为树文件路径、二维码识别的安全区域、以及关键节点的使用参数。

#### 5.3.2. 自定义行为树节点
如需了解行为树节点的代码结构或开发自定义节点，请参考 BehaviorTree.CPP 的官方文档教程：[BT.CPP Tutorials](https://www.behaviortree.dev/docs/category/tutorial-basics/)

## 6. 注意事项与技巧
* **机器人姿态调整**: 如果发现机器人在站立或行走时有明显的前倾或后仰，可以通过微调腿部3号、9号电机的零点（例如，±1～3度）来进行补偿，以获得更稳定的运动效果。具体操作可参考[此文档](../../../readme.md)。
