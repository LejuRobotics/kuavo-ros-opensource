---
title: "YOLOV8识别及抓取案例"
---

- [案例介绍](#案例介绍)
  - [简介](#简介)
  - [功能说明](#功能说明)
  - [流程逻辑](#流程逻辑)
  - [实机视频展示](#实机视频展示)
- [调整配置文件(上位机)](#调整配置文件上位机)
  - [定位高度参数配置(重要)](#定位高度参数配置重要)
- [调整配置文件(下位机)](#调整配置文件下位机)
  - [代码检查](#代码检查)
  - [程序运行配置参数](#程序运行配置参数)
    - [启动参数](#启动参数)
    - [坐标偏移量](#坐标偏移量)
    - [欧拉角设定](#欧拉角设定)
- [代码编译](#代码编译)
  - [上位机部署遥控器服务(已部署可跳过此步)](#上位机部署遥控器服务已部署可跳过此步)
  - [上位机代码编译](#上位机代码编译)
  - [下位机依赖安装](#下位机依赖安装)
- [运行示例](#运行示例)
  - [运行步骤](#运行步骤)
    - [1. **下位机 使机器人站立**](#1-下位机-使机器人站立)
    - [2. **下位机 启动ik求解服务**](#2-下位机-启动ik求解服务)
    - [3. **上位机 启动yoloV8检测程序**](#3-上位机-启动yolov8检测程序)
    - [4. **下位机 检测是否能收到标签信息**](#4-下位机-检测是否能收到标签信息)
    - [5. **下位机 启动yoloV8抓取流程**](#5-下位机-启动yolov8抓取流程)
- [ros话题与服务](#ros话题与服务)
  - [上位机](#上位机)
  - [下位机](#下位机)


## 案例介绍
### 简介
  - 机器人通过头部摄像头识别目标物体，解算坐标信息并计算抓取姿态，最后通过ik逆解计算手臂关节角度进行抓取
### 功能说明
  
  - 通过YOLO识别目标物体，得到抓取目标在坐标系中的位置
  - 自主判断左右手，并计算手臂末端期望位置与姿态
  - 通过ik逆解服务，得到手臂各关节的目标角度
  - 实现"抓取-拿起-归位"流程，过程流畅

### 流程逻辑

1. 机器人低头，短暂延时后获取指定ID的目标物体的平均位姿数据
2. 设置手臂运动模式为外部控制
3. 松开手部，移动到准备姿态
4. 计算ik求解参数，进行ik求解，使用ik结果进行移动
5. 握紧手部，拿起物体，松开手部，手臂复位，机器人抬头，流程结束

### 实机视频展示
<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=115681773297318&bvid=BV1e12rBdEZf&cid=34583219574&p=1" width="320" height="640" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"></iframe>

## 调整配置文件(上位机)
- 配置文件位于 `~/kuavo_ros_application/src/ros_vision/detection_yolo_v8/config/params.yaml`

### 定位高度参数配置(重要)
- **height_table**  
  作用：设置放置目标的桌面高度（单位：米），作为空间定位的基准高度。  
  示例值：0.864  

- **height_bottle**  
  作用：设置瓶子目标的实际高度（单位：米），用于结合图像像素坐标计算目标实际三维位置。  
  示例值：0.22  

- **height_orange**  
  作用：设置橙子目标的实际高度（单位：米），用于结合图像像素坐标计算目标实际三维位置。
  示例值：0.071  

- 注意：
  - height_bottle和height_orange为物体本体的高度，非物体在空间中的高度
  - 若检测置信度偏低，可适当降低 `conf_threshold`（默认 `0.5`）

## 调整配置文件(下位机)

### 代码检查

**下位机1.4.0上的此demo包为不带转腰的版本,需要先执行如下命令更新.**

```bash
cd ~/kuavo-ros-opensource
git fetch origin beta
git restore --source origin/beta --worktree --staged src/demo/yolo_object_capture
```

### 程序运行配置参数
- 抓取程序位于 `~/kuavo-ros-opensource/src/demo/yolo_object_capture/`
  - 抓取水瓶：`yolo_cylinder_capture.py`
  - 抓取橙子：`yolo_sphere_capture.py`
- 偏移量配置文件：`yolo_object_capture/config/offset.yaml`

#### 启动参数
- `--use_offset`：启用抓取偏移量（推荐）
- `--no_waist`：禁用转腰抓取逻辑
- 说明：两个参数不存在绑定关系，可单独使用，也可同时使用

#### 坐标偏移量
- 通过 `yolo_object_capture/config/offset.yaml` 配置抓取偏移量
- 以 `base_link` 坐标系为参考（右手系，机器人面朝方向为 x 正方向，向上为 z 正方向）
- 生效条件：
  - 仅当启动参数包含 `--use_offset` 且当前 `target_id` 在 `targets` 中存在时生效
  - 否则偏移量按 `0.0` 处理，`offset_angle` 按 `1.0` 处理
- 文件结构示例：
```yaml
targets:
  "39":    # 瓶子
    offset:
      z: 0.00
      left:
        x: -0.00
        y: 0.03
      right:
        x: -0.00
        y: -0.03
    offset_angle: 0.15
```
- 字段说明：
  - `targets`：按目标 ID 分组配置，不同目标可使用不同偏移参数
  - `"39" / "41" / "49"`：YOLO 目标 ID（当前示例对应瓶子/杯子/橙子）
  - `offset.z`：抓取点 z 方向修正量（单位：米），参与 `set_z = yolo_object_z + offset_z`
  - `offset.left.x / offset.left.y`：左手抓取时 x/y 修正量（单位：米）
  - `offset.right.x / offset.right.y`：右手抓取时 x/y 修正量（单位：米）
  - `offset_angle`：yaw 角缩放系数（无单位），参与 `relative_angle * offset_angle`
- 调参建议：
  - 抓取点偏高：减小 `offset.z`；抓取点偏低：增大 `offset.z`
  - 抓取点偏前：减小对应手的 `offset.*.x`；抓取点偏后：增大 `offset.*.x`
  - 抓取点偏左：减小对应手的 `offset.*.y`；抓取点偏右：增大对应手的 `offset.*.y`
  - 末端朝向不正：优先微调 `offset_angle`
- 补充说明：
  - `yolo_cylinder_capture.py` 默认目标 ID 为 `39`，`yolo_sphere_capture.py` 默认目标 ID 为 `49`
  - 若新增目标类别，需要在 `offset.yaml` 中增加对应 ID 配置，否则 `--use_offset` 不会对该目标生效

#### 欧拉角设定
- 抓取脚本会根据目标位姿自动计算末端欧拉角
- 若末端朝向偏差较大，优先通过 `offset.yaml` 中 `offset_angle` 对 yaw 方向进行微调

## 代码编译

### 上位机部署遥控器服务(已部署可跳过此步)
```bash
cd ~/kuavo_ros_application/src/ros_audio/kuavo_audio_player/scripts
./deploy_autostart_h12pro.sh
```

### 上位机代码编译
```bash
cd ~/kuavo_ros_application #仓库目录
catkin build detection_yolo_v8
```

### 下位机依赖安装
```bash
sudo apt-get install ros-noetic-vision-msgs
```

## 运行示例

### 运行步骤
**启动前建议先完成头部和手臂限位自动标定：**  
https://kuavo.lejurobot.com/manual/basic_usage/kuavo-ros-control/docs/3%E8%B0%83%E8%AF%95%E6%95%99%E7%A8%8B/%E6%9C%BA%E5%99%A8%E4%BA%BA%E5%85%B3%E8%8A%82%E6%A0%87%E5%AE%9A/#%E5%A4%B4%E9%83%A8%E5%92%8C%E6%89%8B%E8%87%82%E9%9B%B6%E7%82%B9%E8%87%AA%E5%8A%A8%E6%A0%87%E5%AE%9A

#### 1. **下位机 使机器人站立**
- **注意:若已使用遥控器等方式让机器人站立,可跳过此步骤**
```bash
cd ~/kuavo-ros-opensource  # 进入下位机工作空间
sudo su
source devel/setup.bash
# 仿真
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
# 实物
roslaunch humanoid_controllers load_kuavo_real.launch cali:=true
```

#### 2. **下位机 启动ik求解服务**
  - **注意: 部分版本的ik逆解服务会在上一步启动机器人时自动启动,注意不要重复启动**
  - 判断方式: 终端输入`rosnode list | grep ik`
  - 若已存在`/arms_ik_node`, 则跳过此步
  - 若不存在`/arms_ik_node`, 则运行:
      ```bash
      cd ~/kuavo-ros-opensource  # 进入下位机工作空间
      sudo su
      source devel/setup.bash
      roslaunch motion_capture_ik ik_node.launch 
      ```

#### 3. **上位机 启动yoloV8检测程序**
- 启动传感器
```bash
cd ~/kuavo_ros_application  # 进入上位机工作空间
source devel/setup.bash
# 五代进阶版(无手腕相机)
roslaunch dynamic_biped kuavo5_sensor_only_enable.launch
# 五代MaxA版,MaxB版(有手腕相机)
roslaunch dynamic_biped kuavo5_sensor_only_enable.launch enable_wrist_camera:=true
```
- 启动检测程序
```bash
cd ~/kuavo_ros_application  # 进入上位机工作空间
source devel/setup.bash
roslaunch detection_yolo_v8 detection.launch
```

#### 4. **下位机 检测是否能收到标签信息**
- 执行 `rostopic list | grep yolov8`
- 如果存在 `/robot_yolov8_info`
  - 执行 `rostopic echo /robot_yolov8_info`
  - 观察是否存在标签的坐标信息
- 注意事项:
  - 坐标信息为基于机器人坐标系base_link的位置信息
  - 如果在实物上运行，需测量得到的坐标信息是否准确
  - 要下位机启动程序使机器人站立后，上位机才能检测到机器人各关节的角度，以计算出基于机器人坐标系的结果

#### 5. **下位机 启动yoloV8抓取流程**
- **确保步骤1-3已正常运行**
- 抓取水瓶示例：
```bash
cd ~/kuavo-ros-opensource  # 进入下位机工作空间
source devel/setup.bash
# 带抓取偏移量, 自动转腰(推荐)
python3 src/demo/yolo_object_capture/yolo_cylinder_capture.py --use_offset
# 不带抓取偏移量, 不带转腰
python3 src/demo/yolo_object_capture/yolo_cylinder_capture.py --no_waist
```
- 抓取橙子示例：
```bash
cd ~/kuavo-ros-opensource  # 进入下位机工作空间
sudo su
source devel/setup.bash
# 带抓取偏移量, 自动转腰(推荐)
python3 src/demo/yolo_object_capture/yolo_sphere_capture.py --use_offset
# 不带抓取偏移量, 不带转腰
python3 src/demo/yolo_object_capture/yolo_sphere_capture.py --no_waist
```
- 注：若仿真环境卡顿，可适当增加延时，以确保机器人手臂每个动作都能执行到位，示例如下：
  - `publish_arm_target_poses([1.5], [20.0, ...])`修改为`publish_arm_target_poses([3], [20.0, ...])`
  - `time.sleep(2.5)`修改为`time.sleep(5)`

## ros话题与服务
### 上位机
  - 启动传感器，实时识别yolo目标物体并解算出其在机器人基坐标系的位置
  - 发布`/robot_yolov8_info`话题，传递信息

### 下位机
1. 设置手臂运动模式
  - 调用 ROS 服务 `/arm_traj_change_mode` ,设置手臂运动模式为外部控制模式
2. 启动ik逆解服务
  - 计算ik逆解参数，调用 ROS 服务 `/ik/two_arm_hand_pose_cmd_srv` 计算给定坐标与姿态的逆运动学解。
  - 获取ik逆解结果： q_arm: 手臂关节值,（单位弧度）
3. 控制机器人头部
  - 发布到`/robot_head_motion_data`话题
  - 设置关节数据，包含偏航和俯仰角
4. 控制机器人手部开合
  - 发布到`/control_robot_hand_position`话题
  - 设置握紧或松开的关节角度
5. 控制机器人夹爪开合
  - 调用 ROS 服务 `/control_robot_leju_claw`
  - 设置夹爪开合的角度
6. 获取YOLO标签信息
   - 从话题`/robot_yolov8_info`接收到Detection2DArray消息
   - 获取指定ID的yolo目标物体的平均位置(基于机器人基坐标系)
