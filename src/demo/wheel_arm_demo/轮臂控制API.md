# 轮臂控制API

本文档详细介绍了 Kuavo 机器人轮式基座和机械臂控制的 ROS 话题和服务接口。开发者可以通过这些接口实现对机器人各个部位的精确控制。

## 目录

### 发布话题 (Published Topics)

- [`/kuavo_arm_traj`](#1-kuavo_arm_traj---手臂轨迹控制) - 手臂轨迹控制
- [`/lb_leg_traj`](#2-lb_leg_traj---腿部轨迹控制) - 腿部轨迹控制
- [`/cmd_pose`](#3-cmd_pose---相对位置控制) - 相对位置控制
- [`/cmd_pose_world`](#4-cmd_pose_world---世界坐标系位置控制) - 世界坐标系位置控制
- [`/cmd_lb_torso_pose`](#5-cmd_lb_torso_pose---躯干位姿控制) - 躯干位姿控制
- [`/cmd_vel`](#6-cmd_vel---速度控制) - 速度控制
- [`/mm/two_arm_hand_pose_cmd`](#7-mmtwo_arm_hand_pose_cmd---双臂手部位姿控制) - 双臂手部位姿控制

### 订阅话题 (Subscribed Topics)

- [`/cmd_pose_flag`](#1-cmd_pose_flag---位置控制状态反馈) - 位置控制状态反馈
- [`/lb_torso_pose_reach_time`](#2-lb_torso_pose_reach_time---躯干位姿到达时间反馈) - 躯干位姿到达时间反馈

### 调用服务 (Called Services)

- [`/enable_lb_arm_quick_mode`](#1-enable_lb_arm_quick_mode---手臂快速模式切换) - 手臂快速模式切换
- [`/mobile_manipulator_mpc_control`](#2-mobile_manipulator_mpc_control---mpc控制模式切换) - MPC控制模式切换

### 其他章节

- [话题和服务总览表](#话题和服务总览表)
- [演示脚本说明](#演示脚本说明)

---

## ROS 话题接口

### 发布话题 (Published Topics)

#### 1. `/kuavo_arm_traj` - 手臂轨迹控制

**消息类型：** `sensor_msgs/JointState`

**功能描述：** 发布14关节手臂轨迹控制命令，用于控制机器人双臂的精确运动。

**字段说明：**
```yaml
Header header:
  uint32 seq          # 序列号
  time stamp          # 时间戳
  string frame_id     # 坐标系ID

string[] name         # 关节名称数组，14个关节
                     # ['joint1', 'joint2', ..., 'joint14']
                     # 前7个为左臂关节，后7个为右臂关节

float64[] position   # 关节位置数组，14个关节角度值（弧度）
                    # 对应name数组中的关节顺序

float64[] velocity   # 关节速度数组（可选，一般为空）
float64[] effort     # 关节力矩数组（可选，一般为空）
```

**相关演示脚本：** `cmd_arm_quickMode_test.py`

---

#### 2. `/lb_leg_traj` - 腿部轨迹控制

**消息类型：** `sensor_msgs/JointState`

**功能描述：** 发布4关节腿部轨迹控制命令，用于控制机器人腿部关节运动。

**字段说明：**
```yaml
Header header:
  uint32 seq          # 序列号
  time stamp          # 时间戳
  string frame_id     # 坐标系ID

string[] name         # 关节名称数组，4个腿部关节
                     # ['joint1', 'joint2', 'joint3', 'joint4']

float64[] position   # 关节位置数组，4个关节角度值（弧度）
float64[] velocity   # 关节速度数组（可选）
float64[] effort     # 关节力矩数组（可选）
```

**相关演示脚本：** `cmd_leg_joint_test.py`

---

#### 3. `/cmd_pose` - 相对位置控制

**消息类型：** `geometry_msgs/Twist`

**功能描述：** 发送机器人基座的相对位置控制命令，基于当前位置进行相对移动。

**字段说明：**
```yaml
geometry_msgs/Vector3 linear:
  float64 x    # X方向相对位移（米），正值向前，负值向后
  float64 y    # Y方向相对位移（米），正值向左，负值向右  
  float64 z    # Z方向相对位移（米），通常为0.0

geometry_msgs/Vector3 angular:
  float64 x    # Roll轴相对旋转角度（弧度），通常为0.0
  float64 y    # Pitch轴相对旋转角度（弧度），通常为0.0
  float64 z    # Yaw轴相对旋转角度（弧度），正值逆时针，负值顺时针
```

**相关演示脚本：** `cmd_pos_base_test.py`

---

#### 4. `/cmd_pose_world` - 世界坐标系位置控制

**消息类型：** `geometry_msgs/Twist`

**功能描述：** 发送机器人基座在世界坐标系下的绝对位置控制命令。

**字段说明：**
```yaml
geometry_msgs/Vector3 linear:
  float64 x    # 世界坐标系X位置（米）
  float64 y    # 世界坐标系Y位置（米）
  float64 z    # 世界坐标系Z位置（米），通常为0.0

geometry_msgs/Vector3 angular:
  float64 x    # Roll轴绝对角度（弧度），通常为0.0
  float64 y    # Pitch轴绝对角度（弧度），通常为0.0
  float64 z    # Yaw轴绝对角度（弧度），0为正北方向
```

**相关演示脚本：** `cmd_pos_world_test.py`

---

#### 5. `/cmd_lb_torso_pose` - 躯干位姿控制

**消息类型：** `geometry_msgs/Twist`

**功能描述：** 控制机器人躯干的6DOF位姿（3个位置+3个姿态角度），基于底盘坐标系。

**字段说明：**
```yaml
geometry_msgs/Vector3 linear:
  float64 x    # 躯干X位置（米），相对于基座坐标系
  float64 y    # 躯干Y位置（米），相对于基座坐标系
  float64 z    # 躯干Z位置（米），相对于基座坐标系

geometry_msgs/Vector3 angular:
  float64 x    # 躯干Roll角度（弧度），绕X轴旋转，注意：轮臂躯干不支持roll旋转
  float64 y    # 躯干Pitch角度（弧度），绕Y轴旋转，正值前倾
  float64 z    # 躯干Yaw角度（弧度），绕Z轴旋转，正值左转
```

**相关演示脚本：** `cmd_torso_pose_test.py`

---

#### 6. `/cmd_vel` - 速度控制

**消息类型：** `geometry_msgs/Twist`

**功能描述：** 发送机器人基座的速度控制命令，实现连续的运动控制。

**字段说明：**
```yaml
geometry_msgs/Vector3 linear:
  float64 x    # X方向线速度（米/秒），正值向前
  float64 y    # Y方向线速度（米/秒），正值向左
  float64 z    # Z方向线速度（米/秒），通常为0.0

geometry_msgs/Vector3 angular:
  float64 x    # Roll轴角速度（弧度/秒），通常为0.0
  float64 y    # Pitch轴角速度（弧度/秒），通常为0.0
  float64 z    # Yaw轴角速度（弧度/秒），正值逆时针旋转
```

**相关演示脚本：** `cmd_vel_test.py`

---

#### 7. `/mm/two_arm_hand_pose_cmd` - 双臂手部位姿控制

**消息类型：** `kuavo_msgs/twoArmHandPoseCmd`

**功能描述：** 控制双臂手部的位姿，支持末端位姿控制和关节角度控制两种模式。

**字段说明：**
```yaml
int32 frame                    # 坐标系选择
                              # 0: 保持当前坐标系 (keep current frame)
                              # 1: 世界坐标系 (world frame, based on odom)
                              # 2: 本地坐标系 (local frame)
                              # 3: VR坐标系 (VR frame)
                              # 4: 操作世界坐标系 (manipulation world frame)

bool use_custom_ik_param      # 是否使用自定义IK参数
bool joint_angles_as_q0       # 关节角度是否作为IK求解初值

int32 desire_mode             # 控制模式
                              # 0: 世界坐标系末端执行器控制 (end-effector command in World Frame)
                              # 1: 本地坐标系末端执行器控制 (end-effector command in Local Frame)
                              # 2: 关节角度控制 (joint command)

twoArmHandPose hand_poses:    # 双臂手部位姿数据
  Header header:              # 消息头
    uint32 seq
    time stamp
    string frame_id
    
  armHandPose left_pose:      # 左臂位姿
    float64[3] pos_xyz        # 位置 [x, y, z] (米)
    float64[4] quat_xyzw      # 姿态四元数 [x, y, z, w]
    float64[3] elbow_pos_xyz  # 肘部位置 [x, y, z] (米)
    float64[7] joint_angles   # 7个关节角度 (度)
    
  armHandPose right_pose:     # 右臂位姿
    float64[3] pos_xyz        # 位置 [x, y, z] (米)
    float64[4] quat_xyzw      # 姿态四元数 [x, y, z, w]
    float64[3] elbow_pos_xyz  # 肘部位置 [x, y, z] (米)
    float64[7] joint_angles   # 7个关节角度 (度)

ikSolveParam ik_param:        # IK求解参数（当use_custom_ik_param=true时使用）
  float64 major_optimality_tol        # 主要最优性容差
  float64 major_feasibility_tol       # 主要可行性容差
  float64 minor_feasibility_tol       # 次要可行性容差
  int32 major_iterations_limit        # 主要迭代次数限制
  float64 oritation_constraint_tol    # 方向约束容差
  float64 pos_constraint_tol          # 位置约束容差
  float64 pos_cost_weight             # 位置代价权重
```


**相关演示脚本：**
- `simple_two_arm_publisher.py` - 世界坐标系末端位姿控制示例（模式0）
- `simple_two_arm_publisher_local.py` - 本地坐标系末端位姿控制示例（模式1）
- `simple_two_arm_publisher_joint.py` - 关节角度控制示例（模式2）

---

### 订阅话题 (Subscribed Topics)

#### 1. `/cmd_pose_flag` - 位置控制状态反馈

**消息类型：** `std_msgs/Bool`

**功能描述：** 反馈位置控制命令的执行状态。

**字段说明：**
```yaml
bool data    # 控制状态标志
             # true: 命令执行完成
             # false: 命令正在执行中
```

**相关演示脚本：** `cmd_pos_base_test.py`, `cmd_pos_world_test.py`

---

#### 2. `/lb_torso_pose_reach_time` - 躯干位姿到达时间反馈

**消息类型：** `std_msgs/Float32`

**功能描述：** 反馈躯干位姿控制命令的预计到达时间。

**字段说明：**
```yaml
float32 data    # 预计到达时间（秒）
                # 表示从当前时刻到达目标位姿需要的时间
```

**相关演示脚本：** `cmd_torso_pose_test.py`

---


## ROS 服务接口

### 调用服务 (Called Services)

#### 1. `/enable_lb_arm_quick_mode` - 手臂快速模式切换

**服务类型：** `std_srvs/SetBool`

**功能描述：** 启用或禁用手臂快速模式，影响手臂的响应速度和控制精度。

**请求字段：**
```yaml
bool data    # 模式切换标志
             # true: 启用快速模式
             # false: 禁用快速模式
```

**响应字段：**
```yaml
bool success    # 操作是否成功
string message  # 状态消息
```

**相关演示脚本：** `cmd_arm_quickMode_test.py`

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


**控制模式详细说明：**
- **Mode 0 (NoControl)**: 关闭所有MPC控制，机器人进入被动状态
- **Mode 1 (ArmOnly)**: 仅控制双臂运动，基座保持固定，适用于固定位置的操作任务
- **Mode 2 (BaseOnly)**: 仅控制基座移动，手臂保持当前位置，适用于导航任务
- **Mode 3 (BaseArm)**: 协调控制基座和手臂，实现移动操作，适用于复杂的移动抓取任务
- **Mode 4 (ArmEeOnly)**: 仅控制手臂末端执行器，优化末端轨迹跟踪

**相关演示脚本：** `wheel_control_mode_swither.py`

---

## 话题和服务总览表

### 话题总览

| 话题名称 | 消息类型 | 方向 | 功能描述 |
|----------|----------|------|----------|
| `/kuavo_arm_traj` | `sensor_msgs/JointState` | 发布 | 14关节手臂轨迹控制 |
| `/lb_leg_traj` | `sensor_msgs/JointState` | 发布 | 4关节腿部轨迹控制 |
| `/cmd_pose` | `geometry_msgs/Twist` | 发布 | 相对位置控制命令 |
| `/cmd_pose_world` | `geometry_msgs/Twist` | 发布 | 世界坐标系位置控制 |
| `/cmd_lb_torso_pose` | `geometry_msgs/Twist` | 发布 | 躯干位姿控制命令 |
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 | 速度控制命令 |
| `/mm/two_arm_hand_pose_cmd` | `kuavo_msgs/twoArmHandPoseCmd` | 发布 | 双臂手部位姿控制 |
| `/cmd_pose_flag` | `std_msgs/Bool` | 订阅 | 位置控制状态反馈 |
| `/lb_torso_pose_reach_time` | `std_msgs/Float32` | 订阅 | 躯干位姿到达时间 |

### 服务总览

| 服务名称 | 服务类型 | 功能描述 |
|----------|----------|----------|
| `/enable_lb_arm_quick_mode` | `std_srvs/SetBool` | 启用/禁用手臂快速模式 |
| `/mobile_manipulator_mpc_control` | `kuavo_msgs/changeTorsoCtrlMode` | MPC控制模式切换 |

---

## 演示脚本说明

本目录提供了以下演示脚本，展示各个接口的具体使用方法：

1. **`cmd_arm_quickMode_test.py`** - 手臂快速模式和轨迹控制演示
2. **`cmd_leg_joint_test.py`** - 腿部关节控制演示
3. **`cmd_pos_base_test.py`** - 相对位置控制演示
4. **`cmd_pos_world_test.py`** - 世界坐标系位置控制演示
5. **`cmd_torso_pose_test.py`** - 躯干位姿控制演示
6. **`cmd_vel_test.py`** - 基于MPC观测的速度控制演示
7. **`simple_two_arm_publisher_joint.py`** - 双臂关节角度控制演示
8. **`simple_two_arm_publisher_local.py`** - 本地坐标系双臂位姿控制演示
9. **`simple_two_arm_publisher.py`** - 世界坐标系双臂位姿控制演示
10. **`wheel_control_mode_swither.py`** - MPC控制模式交互式切换工具
