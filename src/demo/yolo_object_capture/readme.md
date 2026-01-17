# yolo_object_capture.py
## 基于YOLO的手臂抓取

#### 功能说明
  
  - 通过识别Yolo标签，得到抓取目标在坐标系中的位置
  - 自主判断左右手，并计算手臂末端期望位置与姿态
  - 通过ik逆解服务，得到手臂各关节的目标角度
  - 实现抓水、递水流程，过程流畅

#### 程序功能
  - get_yolov8_info.py 获取yolo检测结果
  - yolo_object_capture.py 完整流程

#### 启动参数
- `offset_start` : 是否启动坐标偏移量
  - 参数输入 `True` ：启用坐标偏移量，一般在实机中使用，以观察抓取效果
  - 参数输入 `False` ：不启用坐标偏移量，一般在仿真中使用，以观察求解效果

#### 坐标偏移量
- 主要参数：
  - `offset_z`  z方向偏移量，默认默认的抓取点在二维码正下方，因此为负值
  - `temp_x_l temp_x_r` x方向偏移量，左右都为负值
  - `temp_y_l temp_y_r` y方向偏移量，均为正值，左加右减
  - `offset_angle` z轴角度偏移量，在进行ik求解时，若觉得yaw角不符合预期，可适当增加或降低该值
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
  - 横滚角度roll：左手为负、右手为正，以右手为例：经过测试设置负(0~20)效果会比较好

#### ros话题与服务
###### 上位机
  - 启动传感器，实时识别yolo目标物体并解算出其在机器人基坐标系的位置
  - 发布`/robot_yolov8_info`话题，传递信息

###### 下位机
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
5. 获取二维码标签信息
- 从话题`/robot_yolov8_info`接收到Detection2DArray消息
- 获取指定ID的YOLO目标物体的position位置

#### 流程逻辑
1. 机器人低头，短暂延时后获取指定ID的YOLO目标物体的平均数据
2. 设置手臂运动模式为外部控制
3. 松开手部，移动到准备姿态
4. 计算ik求解参数，进行ik求解，使用ik结果进行移动
5. 握紧手部，递水，松开手部，手臂复位，机器人抬头，流程结束

#### 函数功能说明
  - 通过 `set_head_target` 控制机器人头部
  - 调用 `call_ik_srv` 函数获取IK逆解结果
  - 调用 `set_arm_control_mode` 函数来设置手臂的控制模式
  - 调用 `publish_arm_target_poses` 函数发布手臂目标姿态
  - 调用 `ToQuaternion` 将欧拉角转换为四元数
  - 通过 `ObjectPositionTracker` 类完成识别yolo标签相关功能

#### 核心流程说明
**1.初始化节点和服务**
  - 初始化ROS节点`object_position_tracker`
  - 订阅 `/robot_yolov8_info` 话题 获取二维码基于机器人基坐标系的位置

**2.获取标签信息**
  - 从话题"/robot_yolov8_info"接收到Detection2DArray消息
  - 使用 `get_latest_position_by_id` 函数获取指定ID的YOLO目标物体的平均位置和姿态数据

**3.ik逆解流程**
1. 目标位置参数
  - 读取 `/robot_yolov8_info` 提供的标签位置
  - 添加xyz的偏移量，得到手臂末端目标位置
2. 目标姿态参数
  - 偏航角yaw：通过当前手臂末端位置与目标手臂末端位置计算
  - 俯仰角pitch：左右手均固定为负90度
  - 横滚角度roll：左手为负、右手为正，以右手为例：经过测试设置负(0~20)效果会比较好
3. ik求解
  - 使用 `call_ik_srv` 函数发送请求以获取ik的解


