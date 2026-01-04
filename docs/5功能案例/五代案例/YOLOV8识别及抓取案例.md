---
title: "YOLOV8识别及抓取案例"
---

- [案例介绍](#案例介绍)
  - [简介](#简介)
  - [功能说明](#功能说明)
  - [流程逻辑](#流程逻辑)
  - [实机视频展示](#实机视频展示)
- [调整配置文件(上位机)](#调整配置文件上位机)
  - [1. 模型相关配置](#1-模型相关配置)
  - [2. 检测目标配置](#2-检测目标配置)
  - [3. 输入数据源配置](#3-输入数据源配置)
  - [4. 定位高度参数配置](#4-定位高度参数配置)
- [调整配置文件(下位机)](#调整配置文件下位机)
  - [程序运行配置参数](#程序运行配置参数)
    - [启动参数](#启动参数)
    - [坐标偏移量](#坐标偏移量)
    - [欧拉角设定](#欧拉角设定)
- [代码编译](#代码编译)
  - [上位机代码编译](#上位机代码编译)
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

### 1. 模型相关配置
- **model_path**  
  - 作用：指定YOLOv8预训练模型文件的路径，模型用于目标特征提取与识别。  
  - 示例值："models/yolov8n.pt"  
  - 说明：支持.pt格式的YOLOv8系列模型，可根据识别精度与速度需求选择。

- **conf_threshold**  
  - 作用：设置目标检测的置信度阈值，过滤置信度低于该值的检测结果，减少误检。  
  - 示例值：0.5  
  - 说明：取值范围0~1，值越高检测越严格，漏检率可能上升；值越低误检率可能上升。

### 2. 检测目标配置
- **target_class**  
  - 作用：定义需要识别的目标类别，基于COCO数据集标注体系。  
  - 示例值：["bottle", "cup"]  
  - 说明：可根据实际需求添加/删减类别，需与模型训练的类别标签一致。

### 3. 输入数据源配置
- **input_image**  
  作用：指定输入图像的ROS话题名称，接收相机采集的彩色图像流。  
  示例值："/camera/color/image_raw"  
  说明：需与实际相机发布的图像话题匹配，确保数据链路畅通。

### 4. 定位高度参数配置
- **height_table**  
  作用：设置放置目标的桌面高度（单位：米），作为空间定位的基准高度。  
  示例值：0.864  

- **height_bottle**  
  作用：设置瓶子目标的实际高度（单位：米），用于结合图像像素坐标计算目标实际三维位置。  
  示例值：0.22  

## 调整配置文件(下位机)

### 程序运行配置参数
- 程序位于 `/home/lab/kuavo-ros-opensource/src/demo/yolo_object_capture/yolo_object_capture.py`

#### 启动参数
- `offset_start` : 是否启动坐标偏移量
  - 参数输入 `True` ：启用坐标偏移量，一般在实机中使用，以观察抓取效果
  - 参数输入 `False` ：不启用坐标偏移量，一般用来观察求解效果及定位准确度

#### 坐标偏移量
- 主要参数：
  - `offset_z`  z方向偏移量，默认默认的抓取点在二维码正下方，因此为负值
  - `temp_x_l temp_x_r` x方向偏移量，左右都为负值
  - `temp_y_l temp_y_r` y方向偏移量，均为正值，左加右减
  - `offset_angle` z轴角度偏移量缩放倍率，在进行ik求解时，若觉得yaw角不符合预期，可适当增加或降低该值
- 调参说明：（以右手为例，机器人面朝方向为前方）
  - 若抓取点偏上，则降低 `offset_z` 的值，反之则调高
  - 若抓取点偏右，则增大 `temp_y_r` 的值，反之则降低
  - 若抓取点偏前，则降低 `temp_x_r` 的值，反之则调高
- 参数位置：
  - `yolo_object_capture.py`文件，主函数中进行设置
  - 使用示例：
```
    # offset_start="True"表示启用偏移量 否则不启用偏移量
    if args.offset_start == "True":
        # 偏向侧后边一点
        offset_z=-0.10  # 抓取点位于标签正下方
        temp_x_l=-0.035
        temp_y_l=0.035
        temp_x_r=-0.045
        temp_y_r=0.035
    else :
        offset_z=0.00
        temp_x_l=0.00
        temp_y_l=0.00
        temp_x_r=0.00
        temp_y_r=0.00
    # 角度偏移量（修正绕z轴的偏移角度）
    offset_angle=1.00
```

#### 欧拉角设定
- 使用示例：
  - `quat=ToQuaternion(relative_angle*offset_angle, -1.57 , 0)`
  - `eef_pose_msg.hand_poses.left_pose.quat_xyzw = [quat.x,quat.y,quat.z,quat.w]`
- ToQuaternion参数：
  - 偏航角yaw：通过当前手臂末端位置与目标手臂末端位置计算
  - 俯仰角pitch：左右手均固定为负90度
  - 横滚角度roll：一般置零即可

## 代码编译

### 上位机代码编译
```bash
cd kuavo_ros_application #仓库目录
sudo su
catkin build detection_yolo_v8
```

## 运行示例

### 运行步骤

#### 1. **下位机 使机器人站立**
- **注意:若已使用遥控器等方式让机器人站立,可跳过此步骤**
```bash
cd kuavo-ros-opensource  # 进入下位机工作空间
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
      cd kuavo-ros-opensource  # 进入下位机工作空间
      sudo su
      source devel/setup.bash
      roslaunch motion_capture_ik ik_node.launch 
      ```

#### 3. **上位机 启动yoloV8检测程序**
- 启动传感器
```bash
cd kuavo_ros_application  # 进入上位机工作空间
sudo su
source devel/setup.bash
# 五代进阶版
roslaunch dynamic_biped kuavo5_sensor_robot_enable.launch
# 五代MaxA版,MaxB版
roslaunch dynamic_biped kuavo5_sensor_robot_enable.launch enable_wrist_camera:=true
```
- 启动检测程序
```bash
cd kuavo_ros_application  # 进入上位机工作空间
sudo su
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
- 执行 
```bash
cd kuavo-ros-opensource  # 进入下位机工作空间
sudo su
source devel/setup.bash
# 运行启用偏移量的抓取流程(二选一)
python3 src/demo/yolo_object_capture/yolo_object_capture.py --offset_start True
# 运行不启用偏移量的抓取流程(二选一)
python3 src/demo/yolo_object_capture/yolo_object_capture.py --offset_start False
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
6. 获取二维码标签信息
   - 从话题`/robot_yolov8_info`接收到Detection2DArray消息
   - 获取指定ID的yolo目标物体的平均位置(基于机器人基坐标系)