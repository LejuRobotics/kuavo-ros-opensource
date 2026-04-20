# 轮臂控制API

本文档详细介绍了 Kuavo 机器人轮式基座和机械臂控制的 ROS 话题和服务接口。开发者可以通过这些接口实现对机器人各个部位的精确控制，以及分析指令的时间可达性和控制效果

## 目录

### 相关文件目录

src/demo/test_kuavo_wheel_real 包含实物测试案例，可供参考

### 发布话题 (Published Topics)

功能话题可分为三个类别：底盘控制，下肢控制，上肢控制，

注意事项如下：

1. 各个类别均有多个话题，各话题具有不同的优先级，在调用时需要注意；
2. 各个位置控制话题，包括关节控制，均采用时间最优规划，由 MPC 自动调节到达时间，因此，阶跃，三角，正弦等形式期望均支持；
3. 所有位置控制话题，在发出指令后，都由对应的时间话题反馈到达时间，表示对应动作所需要的执行时间，自由选择是否使用；
4. 所有位置控制话题，支持单次发送，和多次连发；  
    * 在多次连发情况下，会保持二阶连续和三阶可导，有失真，但满足运动学限制，请放心采用多次连发；
5. 不同类别的指令不会互相打断，相同类别的指令会，打断后，以满足运动学限制的插值轨迹，跟上新的指令；
6. 实物留有完整测试案例，请参考当前目录下各个文件；

#### 底盘接口

- [`/cmd_vel`](#1-cmd_vel---本体坐标系速度控制) - 本体坐标系速度控制
- [`/cmd_vel_world`](#2-cmd_vel_world---世界坐标系速度控制) - 世界坐标系速度控制
- [`/cmd_pose`](#3-cmd_pose---本体坐标系位置控制) - 本体坐标系位置控制（位置话题）
- [`/cmd_pose_world`](#4-cmd_pose_world---世界坐标系位置控制) - 世界坐标系位置控制（位置话题）

#### 下肢控制接口

- [`/lb_leg_traj`](#5-lb_leg_traj---腿部关节控制) - 腿部关节控制（位置话题）
- [`/cmd_lb_torso_pose`](#6-cmd_lb_torso_pose---躯干相对基座的位姿控制) - 躯干相对基座的笛卡尔位姿控制（位置话题）

#### 上肢控制接口

- [`/kuavo_arm_traj`](#7-kuavo_arm_traj---手臂关节控制) - 手臂关节控制（位置话题）
- [`/mm/two_arm_hand_pose_cmd`](#8-mmtwo_arm_hand_pose_cmd---双臂手部笛卡尔位姿控制) - 双臂手部笛卡尔位姿控制（位置话题）

### 接收话题 (Subscribed Topics)

#### 到达时间反馈

- [`/lb_cmd_pose_reach_time`](#1-lb_cmd_pose_reach_time---底盘位姿到达时间反馈) - 底盘位姿到达时间反馈
- [`/lb_leg_joint_reach_time`](#2-lb_leg_joint_reach_time---下肢关节指令到达时间反馈) - 下肢关节指令到达时间反馈
- [`/lb_torso_pose_reach_time`](#3-lb_torso_pose_reach_time---躯干笛卡尔位姿到达时间反馈) - 躯干笛卡尔位姿到达时间反馈
- [`/lb_arm_joint_reach_time`](#4-lb_arm_joint_reach_time---上肢关节指令到达时间反馈) - 上肢关节指令到达时间反馈
- [`/lb_arm_ee_reach_time`](#5-lb_arm_ee_reach_time---双臂手部笛卡尔位姿到达时间反馈) - 双臂手部笛卡尔位姿到达时间反馈

#### 轮臂MPC当前模式反馈

- [`/mobile_manipulator/lb_mpc_control_mode`](#6-mobile_manipulatorlb_mpc_control_mode---轮臂MPC当前的控制模式反馈) - 轮臂MPC当前的控制模式反馈

#### MPC调试信息反馈(存入bag)

- [`/mobile_manipulator/currentMpcTarget/state`](#7-mobile_manipulatorcurrentMpcTargetstate---轮臂MPC当前时刻的cost期望state) - 轮臂MPC当前时刻的cost期望state
- [`/mobile_manipulator/currentMpcTarget/input`](#8-mobile_manipulatorcurrentMpcTargetinput---轮臂MPC当前时刻的cost期望input) - 轮臂MPC当前时刻的cost期望input
- [`/mobile_manipulator/torso_target_6D`](#9-mobile_manipulatortorso_target_6D---轮臂MPC当前时刻的躯干笛卡尔期望) - 轮臂MPC当前时刻的躯干笛卡尔期望
- [`/mobile_manipulator/ee_target_6D/point`](#10-mobile_manipulatoree_target_6Dpoint---轮臂MPC当前时刻的手臂末端笛卡尔期望) - 轮臂MPC当前时刻的手臂末端笛卡尔期望

#### 控制器调试信息反馈(存入bag)

- [`/mobile_manipulator_wbc_observation`](#11-mobile_manipulator_wbc_observation---机器人收到的原生传感器数据) - 机器人收到的原生传感器数据
- [`/mobile_manipulator_mpc_observation`](#12-mobile_manipulator_mpc_observation---输入到轮臂MPC的原生传感器数据经过滤波) - 输入到轮臂MPC的原生传感器数据(经过滤波)
- [`/humanoid_wheel/optimizedState_mrt`](#13-humanoid_wheeloptimizedState_mrt---MPC输出的原生MRT的state期望) - MPC输出的原生MRT的state期望
- [`/humanoid_wheel/optimizedInput_mrt`](#14-humanoid_wheeloptimizedInput_mrt---MPC输出的原生MRT的input期望) - MPC输出的原生MRT的input期望
- [`/humanoid_wheel/optimizedState_mrt_kinemicLimit`](#15-humanoid_wheeloptimizedState_mrt_kinemicLimit---经过运动学限制滤波的MRT的state期望) - 经过运动学限制滤波的MRT的state期望
- [`/humanoid_wheel/optimizedInput_mrt_kinemicLimit`](#16-humanoid_wheeloptimizedInput_mrt_kinemicLimit---经过运动学限制滤波的MRT的input期望) - 经过运动学限制滤波的MRT的input期望
- [`/humanoid_wheel/bodyAcc`](#17-humanoid_wheelbodyAcc---WBC求解的底盘的加速度) - WBC求解的底盘的加速度
- [`/humanoid_wheel/jointAcc`](#18-humanoid_wheeljointAcc---WBC求解的关节加速度包括下肢上肢) - WBC求解的关节加速度(包括下肢上肢)
- [`/humanoid_wheel/torque`](#19-humanoid_wheeltorque---WBC求解的关节扭矩包括下肢上肢) - WBC求解的关节扭矩(包括下肢上肢)
- [`/humanoid_wheel/eePoses`](#20-humanoid_wheeleePoses---双臂末端的世界坐标系6D位姿) - 双臂末端的世界坐标系6D位姿反馈

### 调用服务 (Called Services)

- [`/enable_lb_arm_quick_mode`](#1-enable_lb_arm_quick_mode---关节快速模式切换) - 关节快速模式切换
- [`/mobile_manipulator_mpc_control`](#2-mobile_manipulator_mpc_control---mpc控制模式切换) - MPC控制模式切换

### 新增轮臂VR增量遥操作接口

---

## ROS 话题接口

### 发布话题 (Published Topics)

#### 1. `/cmd_vel` - 本体坐标系速度控制

**消息类型：** `geometry_msgs/Twist`

**功能描述：** 发送机器人基座的速度控制命令，x方向指向机器人的当前正前方。

**注意事项：** 

1. 在底盘指令中优先级最高；
2. 发0速度或不发msg，1秒后自动停下；

**字段说明：**
```yaml
geometry_msgs/Vector3 linear:
  float64 x    # X方向线速度（米/秒），正值向前
  float64 y    # Y方向线速度（米/秒），正值向左
  float64 z    # Z方向线速度（米/秒），不起作用，可不设置

geometry_msgs/Vector3 angular:
  float64 x    # Roll轴角速度（弧度/秒），不起作用，可不设置
  float64 y    # Pitch轴角速度（弧度/秒），不起作用，可不设置
  float64 z    # Yaw轴角速度（弧度/秒），正值逆时针旋转
```

**相关演示脚本：** `cmd_vel_base_test.py`

---

#### 2. `/cmd_vel_world` - 世界坐标系速度控制

**消息类型：** `geometry_msgs/Twist`

**功能描述：** 发送机器人基座的速度控制命令，x方向指向机器人启动时所指向的正前方。

**注意事项：** 

1. 在底盘指令中优先级小于 cmd_vel；
2. 发0速度或不发msg，1秒后自动停下；

**字段说明：**
```yaml
geometry_msgs/Vector3 linear:
  float64 x    # X方向线速度（米/秒），世界系的x方向前进
  float64 y    # Y方向线速度（米/秒），世界系的y方向前进
  float64 z    # Z方向线速度（米/秒），不起作用，可不设置

geometry_msgs/Vector3 angular:
  float64 x    # Roll轴角速度（弧度/秒），不起作用，可不设置
  float64 y    # Pitch轴角速度（弧度/秒），不起作用，可不设置
  float64 z    # Yaw轴角速度（弧度/秒），正值逆时针旋转
```

**相关演示脚本：** `cmd_vel_world_test.py`

---

#### 3. `/cmd_pose` - 本体坐标系位置控制

**消息类型：** `geometry_msgs/Twist`

**功能描述：** 发送机器人基座的相对位置控制命令，基于当前位置进行相对移动

**注意事项：** 

1. yaw期望发送多圈会旋转多圈(如：6.28, 机器人旋转一圈)；
2. 优先级与cmd_pose_world相同，可互相打断；

**字段说明：**
```yaml
geometry_msgs/Vector3 linear:
  float64 x    # X方向相对位移（米），正值向前，负值向后
  float64 y    # Y方向相对位移（米），正值向左，负值向右  
  float64 z    # Z方向相对位移（米），不起作用，可不设置

geometry_msgs/Vector3 angular:
  float64 x    # Roll轴相对旋转角度（弧度），不起作用，可不设置
  float64 y    # Pitch轴相对旋转角度（弧度），不起作用，可不设置
  float64 z    # Yaw轴相对旋转角度（弧度），正值逆时针，负值顺时针
```

**相关演示脚本：** `cmd_pos_base_test.py`

---

#### 4. `/cmd_pose_world` - 世界坐标系位置控制

**消息类型：** `geometry_msgs/Twist`

**功能描述：** 发送机器人基座在世界坐标系下的绝对位置控制命令。

**注意事项：** 

1. yaw期望发送多圈会校准回[-$\pi$, $\pi$]；
2. 优先级与cmd_pose相同，可互相打断；

**字段说明：**
```yaml
geometry_msgs/Vector3 linear:
  float64 x    # 世界坐标系X位置（米）
  float64 y    # 世界坐标系Y位置（米）
  float64 z    # 世界坐标系Z位置（米），通常为0.0

geometry_msgs/Vector3 angular:
  float64 x    # Roll轴绝对角度（弧度），通常为0.0
  float64 y    # Pitch轴绝对角度（弧度），通常为0.0
  float64 z    # Yaw轴绝对角度（弧度），0为机器人启动时的正前方
```

**相关演示脚本：** `cmd_pos_world_test.py`

---

#### 5. `/lb_leg_traj` - 腿部关节控制

**消息类型：** `sensor_msgs/JointState`

**功能描述：** 发布关节腿部轨迹控制命令，用于控制机器人腿部关节运动。

**注意事项：** 

1. 自由度为4，维度错误会触发错误打印，指令不执行，单位：角度；
2. 优先级与 /cmd_lb_torso_pose 同级，混发则相互打断；

**字段说明：**
```yaml
Header header:
  uint32 seq          # 序列号，不起作用，可不设置
  time stamp          # 时间戳，不起作用，可不设置
  string frame_id     # 坐标系ID，不起作用，可不设置

string[] name         # 关节名称数组，4个腿部关节，不起作用，可不设置
                     # ['joint1', 'joint2', 'joint3', 'joint4']

float64[] position   # 关节位置数组，4个关节角度值（角度）
float64[] velocity   # 关节速度数组，不起作用，可不设置
float64[] effort     # 关节力矩数组，不起作用，可不设置
```

**相关演示脚本：** `cmd_leg_joint_test.py`

---

#### 6. `/cmd_lb_torso_pose` - 躯干相对基座的位姿控制

**消息类型：** `geometry_msgs/Twist`

**功能描述：** 控制机器人躯干的6DOF位姿（3个位置+3个姿态角度），基于底盘坐标系。

**注意事项：** 

1. 该机器人无 y，和 roll 自由度，设置不起作用；
2. 优先级与 /lb_leg_traj 同级，混发则相互打断；
3. 位置相对于基座坐标系，姿态相对于躯干坐标系；

**字段说明：**
```yaml
geometry_msgs/Vector3 linear:
  float64 x    # 躯干X位置（米），相对于基座坐标系
  float64 y    # 躯干Y位置（米），相对于基座坐标系，不起作用，可不设置
  float64 z    # 躯干Z位置（米），相对于基座坐标系

geometry_msgs/Vector3 angular:
  float64 x    # 躯干Roll角度（弧度），绕X轴旋转，不起作用，可不设置
  float64 y    # 躯干Pitch角度（弧度），绕Y轴旋转，正值前倾
  float64 z    # 躯干Yaw角度（弧度），绕Z轴旋转，正值左转
```

**相关演示脚本：** `cmd_torso_pose_test.py`

---

#### 7. `/kuavo_arm_traj` - 手臂关节控制

**消息类型：** `sensor_msgs/JointState`

**功能描述：** 发布关节手臂轨迹控制命令，用于控制机器人手臂关节运动。

**注意事项：** 

1. 自由度为14，维度错误会触发错误打印，指令不执行，单位：角度；
2. 优先级与 /mm/two_arm_hand_pose_cmd 同级，混发则相互打断；

**字段说明：**
```yaml
Header header:
  uint32 seq          # 序列号，不起作用，可不设置
  time stamp          # 时间戳，不起作用，可不设置
  string frame_id     # 坐标系ID，不起作用，可不设置

  string[] name        # 关节名称数组，14个关节，不起作用，可不设置
                       # ['joint1', 'joint2', ..., 'joint14']
                       # 前7个为左臂关节，后7个为右臂关节

  float64[] position   # 关节位置数组，14个关节角度值（角度）
  float64[] velocity   # 关节速度数组，不起作用，可不设置
  float64[] effort     # 关节力矩数组，不起作用，可不设置
```

**相关演示脚本：** `cmd_arm_joint_test.py`

---

#### 8. `/mm/two_arm_hand_pose_cmd` - 双臂手部笛卡尔位姿控制

**消息类型：** `kuavo_msgs/twoArmHandPoseCmd`

**功能描述：** 控制双臂手部的位姿，

**注意事项：** 

1. 不支持逆解参数的设置；
2. 支持世界坐标系，局部坐标系和关节空间坐标系的指令发送，通过 frame 来切换，hand_poses 来构造信息
3. 优先级与 /kuavo_arm_traj 相同，可互相打断

**字段说明：**
```yaml
twoArmHandPose hand_poses:    # 双臂手部位姿数据
  Header header:              # 消息头，不使用，可不设置
    uint32 seq
    time stamp
    string frame_id
    
  armHandPose left_pose:      # 左臂位姿
    float64[3] pos_xyz        # 位置 [x, y, z] (米)
    float64[4] quat_xyzw      # 姿态四元数 [x, y, z, w]
    float64[3] elbow_pos_xyz  # 肘部位置 [x, y, z] (米)，暂不支持，可不设置
    float64[7] joint_angles   # 7个关节角度 (度)
    
  armHandPose right_pose:     # 右臂位姿
    float64[3] pos_xyz        # 位置 [x, y, z] (米)
    float64[4] quat_xyzw      # 姿态四元数 [x, y, z, w]
    float64[3] elbow_pos_xyz  # 肘部位置 [x, y, z] (米)，暂不支持，可不设置
    float64[7] joint_angles   # 7个关节角度 (度)

bool use_custom_ik_param      # 是否使用自定义IK参数，暂不支持，可不设置
bool joint_angles_as_q0       # 关节角度是否作为IK求解初值，暂不支持，可不设置
ikSolveParam ik_param         # IK 逆解参数配置，暂不支持，可不设置

int32 frame                   # 坐标系选择，与人形兼容，但仅支持部分
                              # 0: 保持当前坐标系 (keep current frame)
                              # 1: 世界坐标系 (world frame, based on odom)
                              # 2: 本地坐标系 (local frame)
                              # 5: 关节空间坐标系 (Joint Space frame)
```


**相关演示脚本：**
- `cmd_arm_ee_world_test.py` - 世界坐标系末端位姿控制示例（frame = 1）
- `cmd_arm_ee_local_test.py` - 局部坐标系末端位姿控制示例（frame = 2）
- `cmd_arm_ee_joint_test.py` - 关节角度控制示例（frame = 5）

---

### 订阅话题 (Subscribed Topics)

#### 1. `/lb_cmd_pose_reach_time` - 底盘位姿到达时间反馈

**消息类型：** `std_msgs/Float32`

**功能描述：** 下发新的 cmd_pose, cmd_pose_world 话题时，与此同时，会发布一个执行所需时间

**字段说明：**
```yaml
float32 data    # 预计到达时间（秒），假如收到2，则表示2秒后执行完成
```

**相关演示脚本：**
- `cmd_pos_base_test.py` 
- `cmd_pos_world_test.py`

---

#### 2. `/lb_leg_joint_reach_time` - 下肢关节指令到达时间反馈

**消息类型：** `std_msgs/Float32`

**功能描述：** 下发新的 lb_leg_traj 话题时，与此同时，会发布一个执行所需时间

**字段说明：**
```yaml
float32 data    # 预计到达时间（秒），假如收到2，则表示2秒后执行完成
```

**相关演示脚本：**
- `cmd_leg_joint_test.py` 

---

#### 3. `/lb_torso_pose_reach_time` - 躯干笛卡尔位姿到达时间反馈

**消息类型：** `std_msgs/Float32`

**功能描述：** 下发新的 cmd_lb_torso_pose 话题时，与此同时，会发布一个执行所需时间

**字段说明：**
```yaml
float32 data    # 预计到达时间（秒），假如收到2，则表示2秒后执行完成
```

**相关演示脚本：**
- `cmd_torso_pose_test.py` 

---

#### 4. `/lb_arm_joint_reach_time` - 上肢关节指令到达时间反馈

**消息类型：** `std_msgs/Float32`

**功能描述：** 下发新的 kuavo_arm_traj 话题或 /mm/two_arm_hand_pose_cmd 话题的关节空间控制时，与此同时，会发布一个执行所需时间

**字段说明：**
```yaml
float32 data    # 预计到达时间（秒），假如收到2，则表示2秒后执行完成
```

**相关演示脚本：**
- `cmd_arm_joint_test.py` 
- `cmd_arm_ee_joint_test.py` 

---
#### 5. `/lb_arm_ee_reach_time` - 双臂手部笛卡尔位姿到达时间反馈

**消息类型：** `std_msgs/Float32`

**功能描述：** 下发新的 /mm/two_arm_hand_pose_cmd 话题的世界系和局部系控制时，与此同时，会发布一个执行所需时间

**字段说明：**
```yaml
float32 data    # 预计到达时间（秒），假如收到2，则表示2秒后执行完成
```

**相关演示脚本：**
- `cmd_arm_ee_world_test.py` 
- `cmd_arm_ee_local_test.py` 

---

#### 6. `/mobile_manipulator/lb_mpc_control_mode` - 轮臂MPC当前的控制模式反馈

**消息类型：** `std_msgs/Int8`

**功能描述：** 按照 50Hz 的频率，发布当前轮臂机器人所在的MPC控制模式

**字段说明：**
```yaml
Int8 data    # MPC模式，分别包括: 1. ArmOnly, 2. BaseOnly, 3. BaseArm, 4. ArmEeOnly
              # 0: NoControl - 无控制
              # 1: ArmOnly - 仅控制手臂，基座固定
              # 2: BaseOnly - 仅控制基座，手臂固定  
              # 3: BaseArm - 同时控制基座和手臂
              # 4: ArmEeOnly - 仅控制手臂末端
```

---

#### 7. `/mobile_manipulator/currentMpcTarget/state` - 轮臂MPC当前时刻的cost期望state

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 保存底盘，下肢和上肢关节的位置参考，根据控制模式，未使能时候可不参考

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共3+4+14自由度)：[base位姿(x, y, yaw)(3), 下肢关节位置(4), 上肢关节位置(14)]
```

---

#### 8. `/mobile_manipulator/currentMpcTarget/input` - 轮臂MPC当前时刻的cost期望input

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 保存底盘，下肢和上肢关节的速度参考，根据控制模式，未使能时候可不参考

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共3+4+14自由度)：[base速度(x, y, yaw)(3), 下肢关节速度(4), 上肢关节速度(14)]
```

---

#### 9. `/mobile_manipulator/torso_target_6D` - 轮臂MPC当前时刻的躯干笛卡尔期望

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 保存躯干笛卡尔6D期望，根据控制模式，未使能时候可不参考

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共6自由度)：[躯干位姿(x, y, z, yaw, pitch, roll)]
```

---

#### 10. `/mobile_manipulator/ee_target_6D/point` - 轮臂MPC当前时刻的手臂末端笛卡尔期望

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 保存双臂末端笛卡尔期望，根据控制模式，未使能时候可不参考

**注意事项：** 在 point 后需要加上索引，目前支持 point0，point1，分别是左臂右臂

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共6自由度)：[末端位姿(x, y, z, yaw, pitch, roll)]
```

---

#### 11. `/mobile_manipulator_wbc_observation` - 机器人收到的原生传感器数据

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 机器人收到的原生传感器数据

**字段说明：**
```yaml
Float64 time               # MPC 模块内部时间
int64   mode               # MPC 的步态，不是人形，无效，不需要注意
Float64MultiArray state    # 数据格式(共3+4+14自由度)：[base位姿(x, y, yaw)(3), 下肢关节位置(4), 上肢关节位置(14)]
Float64MultiArray input    # 数据格式(共3+4+14自由度)：[base速度(x, y, yaw)(3), 下肢关节速度(4), 上肢关节速度(14)]
```

---

#### 12. `/mobile_manipulator_mpc_observation` - 输入到轮臂MPC的原生传感器数据(经过滤波)

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 输入到轮臂MPC的原生传感器数据，使用 kinemicLimitFilter 进行时域滤波，二阶连续

**字段说明：**
```yaml
Float64 time               # MPC 模块内部时间
int64   mode               # MPC 的步态，不是人形，无效，不需要注意
Float64MultiArray state    # 数据格式(共3+4+14自由度)：[base位姿(x, y, yaw)(3), 下肢关节位置(4), 上肢关节位置(14)]
Float64MultiArray input    # 数据格式(共3+4+14自由度)：[base速度(x, y, yaw)(3), 下肢关节速度(4), 上肢关节速度(14)]
```

---

#### 13. `/humanoid_wheel/optimizedState_mrt` - MPC输出的原生MRT的state期望

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** MPC 计算，并通过 MRT 模块插值后输出的state期望

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共3+4+14自由度)：[base位姿(x, y, yaw)(3), 下肢关节位置(4), 上肢关节位置(14)]
```

---

#### 14. `/humanoid_wheel/optimizedInput_mrt` - MPC输出的原生MRT的input期望

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** MPC 计算，并通过 MRT 模块插值后输出的input期望

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共3+4+14自由度)：[base速度(x, y, yaw)(3), 下肢关节速度(4), 上肢关节速度(14)]
```

---

#### 15. `/humanoid_wheel/optimizedState_mrt_kinemicLimit` - 经过运动学限制滤波的MRT的state期望

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** MPC 计算，并通过 MRT 模块插值后输出的state期望，使用 KinemicLimitFilter 进行滤波，保证二阶连续

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共3+4+14自由度)：[base位姿(x, y, yaw)(3), 下肢关节位置(4), 上肢关节位置(14)]
```

---

#### 16. `/humanoid_wheel/optimizedInput_mrt_kinemicLimit` - 经过运动学限制滤波的MRT的input期望

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** MPC 计算，并通过 MRT 模块插值后输出的input期望，使用 KinemicLimitFilter 进行滤波，保证二阶连续

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共3+4+14自由度)：[base速度(x, y, yaw)(3), 下肢关节速度(4), 上肢关节速度(14)]
```

---

#### 17. `/humanoid_wheel/bodyAcc` - WBC求解的底盘的加速度

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** WBC 模块输出的底盘加速度

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共3自由度)：[base加速度(x, y, yaw)(3)]
```

---

#### 18. `/humanoid_wheel/jointAcc` - WBC求解的关节加速度(包括下肢上肢)

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** WBC 模块输出的关节加速度

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共4+14自由度)：[下肢加速度(4), 上肢加速度(14)]
```

---

#### 19. `/humanoid_wheel/torque` - WBC求解的关节扭矩(包括下肢上肢)

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** WBC 模块输出的关节加速度

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共4+14自由度)：[下肢扭矩(4), 上肢扭矩(14)]
```

---

#### 20. `/humanoid_wheel/eePoses` - 双臂末端的世界坐标系6D位姿

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 双臂末端的6D位姿世界系反馈，位姿格式: [x, y, z, yaw, pitch, roll]

**字段说明：**
```yaml
Float64MultiArray data    # 数据格式(共6+6自由度)：[左臂位姿(6), 右臂位姿(6)]
```

---

## ROS 服务接口

### 调用服务 (Called Services)

#### 1. `/enable_lb_arm_quick_mode` - 关节快速模式切换

**服务类型：** `kuavo_msgs::changeLbQuickModeSrv`

**功能描述：** 
启用或禁用手臂快速模式，
1. 启用下肢则下肢电机直接获取lb_leg_traj的指令，不从MPC获取下肢电机指令
2. 启用上肢则手臂电机直接获取kuavo_arm_traj的指令，不从MPC获取手臂电机指令

**请求字段：**
```yaml
int8 quickMode    # 模式切换类型
                  # 0-关闭, 1-下肢快, 2-上肢快, 3-上下肢快
```

**响应字段：**
```yaml
bool success    # 操作是否成功
string message  # 状态消息
```

**相关演示脚本：** 
- `cmd_arm_joint_test.py`
- `cmd_leg_joint_test.py`

---

#### 2. `/mobile_manipulator_mpc_control` - MPC控制模式切换

**服务类型：** `kuavo_msgs/changeTorsoCtrlMode`

**功能描述：** 切换移动操作机器人的MPC控制模式，控制不同部位的协调工作。

**请求字段：**
```yaml
int32 control_mode    # 控制模式选择
                      # 0: NoControl - 无控制
                      # 1: ArmOnly - 仅控制手臂，基座固定
                      # 2: BaseOnly - 仅控制基座，手臂固定  
                      # 3: BaseArm - 同时控制基座和手臂
                      # 4: ArmEeOnly - 仅控制手臂末端
```

**响应字段：**
```yaml
bool result        # 切换是否成功
string message     # 状态消息
```
**相关演示脚本：** 
- `lb_ctrl_api.py`

**控制模式详细说明：**
- **Mode 0 (NoControl)**: 关闭所有MPC控制，机器人进入被动状态
- **Mode 1 (ArmOnly)**: 仅控制双臂运动，基座保持固定，适用于固定位置的操作任务
- **Mode 2 (BaseOnly)**: 仅控制基座移动，手臂保持当前位置，适用于导航任务
- **Mode 3 (BaseArm)**: 协调控制基座和手臂，实现移动操作，适用于复杂的移动抓取任务
- **Mode 4 (ArmEeOnly)**: 仅控制手臂末端执行器，优化末端轨迹跟踪

**相关演示脚本：** `lb_ctrl_api.py`

---

## 新增轮臂VR增量遥操作接口
### 启动方式

```bash
# 命令行1： 启动仿真环境
export ROBOT_VERSION=60 # 暂时适配60， 待所有功能开发验证完成后再进行61适配
source ./devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
```

```bash
# 命令行2: 启动VR节点: 增量位置✅ + 增量姿态✅
export ROBOT_VERSION=60
source ./devel/setup.bash

roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch \
    ip_address:=10.10.31.34 \
    use_cpp_incremental_ik:=true \
    use_incremental_hand_orientation:=true
```
---