---
title: "Kuavo 5-W 接口使用文档"
---
# Kuavo 5-W 接口使用文档

本文档详细介绍了 Kuavo 5-W 机器人轮式基座和机械臂控制的 ROS 话题和服务接口。开发者可以通过这些接口实现对机器人各个部位的精确控制，以及分析指令的时间可达性和控制效果

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
   - 在多次连发情况下，会保持二阶连续和三阶可导，有失真，但满足运动学限制，请放心采用多次连发；
5. 不同类别的指令不会互相打断，相同类别的指令会，打断后，以满足运动学限制的插值轨迹，跟上新的指令；
6. 实物留有完整测试案例，请参考当前目录下各个文件；

#### 底盘接口

- [/cmd_vel -本体坐标系速度控制](#1-cmd_vel---本体坐标系速度控制)
- [/cmd_vel_world -世界坐标系速度控制](#2-cmd_vel_world---世界坐标系速度控制)
- [/cmd_pose -本体坐标系位置控制](#3-cmd_pose---本体坐标系位置控制)
- [/cmd_pose_world -世界坐标系位置控制（位置话题）](#4-cmd_pose_world---世界坐标系位置控制)

#### 下肢控制接口

- [/lb_leg_traj - 腿部关节控制](#5-lb_leg_traj---腿部关节控制)
- [/cmd_lb_torso_pose - 躯干相对基座的位姿控制](#6-cmd_lb_torso_pose---躯干相对基座的位姿控制)

#### 上肢控制接口

- [/kuavo_arm_traj - 手臂关节控制](#7-kuavo_arm_traj---手臂关节控制)
- [/mm/two_arm_hand_pose_cmd - 双臂手部笛卡尔位姿控制](#8-mmtwo_arm_hand_pose_cmd---双臂手部笛卡尔位姿控制)

### 接收话题 (Subscribed Topics)

#### 到达时间反馈

- [/lb_cmd_pose_reach_time - 底盘位姿到达时间反馈](#1-lb_cmd_pose_reach_time---底盘位姿到达时间反馈)
- [/lb_leg_joint_reach_time - 下肢关节指令到达时间反馈](#2-lb_leg_joint_reach_time---下肢关节指令到达时间反馈)
- [/lb_torso_pose_reach_time - 躯干笛卡尔位姿到达时间反馈](#3-lb_torso_pose_reach_time---躯干笛卡尔位姿到达时间反馈)
- [/lb_arm_joint_reach_time - 上肢关节指令到达时间反馈](#4-lb_arm_joint_reach_time---上肢关节指令到达时间反馈)
- [/lb_arm_ee_reach_time - 双臂手部笛卡尔位姿到达时间反馈](#5-lb_arm_ee_reach_time---双臂手部笛卡尔位姿到达时间反馈)

#### 轮臂MPC当前模式反馈

- [/mobile_manipulator/lb_mpc_control_mode - 轮臂MPC当前的控制模式反馈](#6-mobile_manipulatorlb_mpc_control_mode---轮臂mpc当前的控制模式反馈)

#### MPC调试信息反馈(存入bag)

- [/mobile_manipulator/currentMpcTarget/state - 轮臂MPC当前时刻的cost期望state](#7-mobile_manipulatorcurrentmpctargetstate---轮臂mpc当前时刻的cost期望state)
- [/mobile_manipulator/currentMpcTarget/input - 轮臂MPC当前时刻的cost期望input](#8-mobile_manipulatorcurrentmpctargetinput---轮臂mpc当前时刻的cost期望input)
- [/mobile_manipulator/torso_target_6D - 轮臂MPC当前时刻的躯干笛卡尔期望](#9-mobile_manipulatortorso_target_6d---轮臂mpc当前时刻的躯干笛卡尔期望)
- [/mobile_manipulator/ee_target_6D/point - 轮臂MPC当前时刻的手臂末端笛卡尔期望](#10-mobile_manipulatoree_target_6dpoint---轮臂mpc当前时刻的手臂末端笛卡尔期望)

#### 控制器调试信息反馈(存入bag)

- [/mobile_manipulator_wbc_observation - 机器人收到的原生传感器数据](#11-mobile_manipulator_wbc_observation---机器人收到的原生传感器数据)
- [/mobile_manipulator_mpc_observation - 输入到轮臂MPC的原生传感器数据经过滤波](#12-mobile_manipulator_mpc_observation---输入到轮臂mpc的原生传感器数据经过滤波)
)
- [/humanoid_wheel/optimizedState_mrt - MPC输出的原生MRT的state期望](#13-humanoid_wheeloptimizedstate_mrt---mpc输出的原生mrt的state期望)
- [/humanoid_wheel/optimizedInput_mrt - MPC输出的原生MRT的input期望](#14-humanoid_wheeloptimizedinput_mrt---mpc输出的原生mrt的input期望)
- [/humanoid_wheel/optimizedState_mrt_kinemicLimit - 经过运动学限制滤波的MRT的state期望](#15-humanoid_wheeloptimizedstate_mrt_kinemiclimit---经过运动学限制滤波的mrt的state期望)
- [/humanoid_wheel/optimizedInput_mrt_kinemicLimit - 经过运动学限制滤波的MRT的input期望](#16-humanoid_wheeloptimizedinput_mrt_kinemiclimit---经过运动学限制滤波的mrt的input期望)
- [/humanoid_wheel/bodyAcc - WBC求解的底盘的加速度](#17-humanoid_wheelbodyacc---wbc求解的底盘的加速度)
- [/humanoid_wheel/jointAcc - WBC求解的关节加速度包括下肢上肢](#18-humanoid_wheeljointacc---wbc求解的关节加速度包括下肢上肢)
- [/humanoid_wheel/torque - WBC求解的关节扭矩包括下肢上肢](#19-humanoid_wheeltorque---wbc求解的关节扭矩包括下肢上肢)
- [/humanoid_wheel/eePoses - 双臂末端的世界坐标系6D位姿](#20-humanoid_wheeleeposes---双臂末端的世界坐标系6d位姿)

### 调用服务 (Called Services)

- [/enable_lb_arm_quick_mode - 关节快速模式切换](#1-enable_lb_arm_quick_mode---关节快速模式切换)
- [/mobile_manipulator_mpc_control - mpc控制模式切换](#2-mobile_manipulator_mpc_control---mpc控制模式切换)

------

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

使用示例

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

# 初始化ROS节点
rospy.init_node('cmd_vel_publisher')
# 创建发布者
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# 等待发布器建立连接
time.sleep(1)
# 创建Twist消息对象
cmd_vel_msg = Twist()
cmd_vel_msg.linear.x = 0.5  # 本体系 x 方向速度 (m/s)
cmd_vel_msg.linear.y = 0.0  # 本体系 y 方向速度 (m/s)
cmd_vel_msg.linear.z = 0.0
cmd_vel_msg.angular.x = 0.0
cmd_vel_msg.angular.y = 0.0
cmd_vel_msg.angular.z = 1.57  # 本体系旋转（偏航）角速度，单位为弧度/秒 (rad/s)

cmd_vel_pub.publish(cmd_vel_msg)
print("cmd_vel 消息已发布")
```

------

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

使用示例

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

# 初始化ROS节点
rospy.init_node('cmd_vel_world_publisher')

# 创建发布者
cmd_vel_world_pub = rospy.Publisher('/cmd_vel_world', Twist, queue_size=10)

# 等待发布器建立连接
time.sleep(1)

# 创建Twist消息对象
cmd_vel_world_msg = Twist()
cmd_vel_world_msg.linear.x = 0.1  # 世界系 x 方向速度 (m/s)
cmd_vel_world_msg.linear.y = 0.0  # 世界系 y 方向速度 (m/s)
cmd_vel_world_msg.linear.z = 0.0
cmd_vel_world_msg.angular.x = 0.0
cmd_vel_world_msg.angular.y = 0.0
cmd_vel_world_msg.angular.z = 0.0  # 世界系旋转（偏航）角速度，单位为弧度/秒 (rad/s)

# 发布消息
cmd_vel_world_pub.publish(cmd_vel_world_msg)
print("cmd_vel_world 消息已发布")
```



------

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

使用示例

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

# 初始化ROS节点
rospy.init_node('cmd_pose_publisher')

# 创建发布者
cmd_pose_pub = rospy.Publisher('/cmd_pose', Twist, queue_size=10)

# 等待发布器建立连接
time.sleep(1)

# 创建Twist消息对象
cmd_pose_msg = Twist()
cmd_pose_msg.linear.x = 0.5  # 基于当前位置的 x 方向值 (m)
cmd_pose_msg.linear.y = 0.0  # 基于当前位置的 y 方向值 (m)
cmd_pose_msg.linear.z = 0.0  # 增量高度
cmd_pose_msg.angular.z = 0.0  # 基于当前位置旋转（偏航）的角度，单位为弧度 (radian)

# 发布消息
cmd_pose_pub.publish(cmd_pose_msg)
print("cmd_pose 消息已发布")
```



------

#### 4. `/cmd_pose_world` - 世界坐标系位置控制

**消息类型：** `geometry_msgs/Twist`

**功能描述：** 发送机器人基座在世界坐标系下的绝对位置控制命令。

**注意事项：**

1. yaw期望发送多圈会校准回；
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

使用示例

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

# 初始化ROS节点
rospy.init_node('cmd_pose_world_publisher')

# 创建发布者
cmd_pose_world_pub = rospy.Publisher('/cmd_pose_world', Twist, queue_size=10)

# 等待发布器建立连接
time.sleep(1)

# 创建Twist消息对象
cmd_pose_world_msg = Twist()
cmd_pose_world_msg.linear.x = 0.3  # 世界系 x 方向位置 (m)
cmd_pose_world_msg.linear.y = 0.0  # 世界系 y 方向位置 (m)
cmd_pose_world_msg.linear.z = 0.0  # 增量高度
cmd_pose_world_msg.angular.z = 0.0  # 基于当前位置旋转（偏航）的角度，单位为弧度 (radian)

# 发布消息
cmd_pose_world_pub.publish(cmd_pose_world_msg)

print("cmd_pose_world 消息已发布")
```



------

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

使用示例

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import time

# 初始化ROS节点
rospy.init_node('cmd_leg_joint_publisher')

# 创建发布者
cmd_leg_joint_pub = rospy.Publisher('/lb_leg_traj', JointState, queue_size=10)

# 等待发布器建立连接
time.sleep(1)

# 创建JointState消息对象
cmd_leg_joint_msg = JointState()
cmd_leg_joint_msg.header.stamp = rospy.Time.now()
cmd_leg_joint_msg.position = [14.90, -32.01, 18.03, 0.0]  # 腿部关节角度 (度)

# 发布消息
cmd_leg_joint_pub.publish(cmd_leg_joint_msg)
print("cmd_leg_joint 消息已发布")
```



------

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

使用示例

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

# 初始化ROS节点
rospy.init_node('cmd_torso_pose_publisher')

# 创建发布者
cmd_torso_pose_pub = rospy.Publisher('/cmd_lb_torso_pose', Twist, queue_size=10)

# 等待发布器建立连接
time.sleep(1)

# 创建Twist消息对象
cmd_torso_pose_msg = Twist()
cmd_torso_pose_msg.linear.x = 0.3  # 躯干 x 方向位置 (m)
cmd_torso_pose_msg.linear.y = 0.0     # 躯干 y 方向位置 (m)
cmd_torso_pose_msg.linear.z = 1.4  # 躯干 z 方向位置 (m)
cmd_torso_pose_msg.angular.x = 0.0       # 滚转角 (rad)
cmd_torso_pose_msg.angular.y = 0.0       
cmd_torso_pose_msg.angular.z = -1.57       # 偏航角 (rad)

# 发布消息
cmd_torso_pose_pub.publish(cmd_torso_pose_msg)

print("cmd_torso_pose 消息已发布")
```



------

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

使用示例

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import time
import sys
import os

# 添加父目录到路径以便导入 lb_ctrl_api
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import lb_ctrl_api as ct

# 初始化ROS节点
rospy.init_node('cmd_arm_joint_publisher')

# 创建发布者
cmd_arm_joint_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)

# 等待发布器建立连接
time.sleep(1)

# 设置控制模式为 ArmOnly (模式1)
ct.set_control_mode(1)

# 创建JointState消息对象
cmd_arm_joint_msg = JointState()
cmd_arm_joint_msg.header.stamp = rospy.Time.now()
cmd_arm_joint_msg.name = [f'joint{i}' for i in range(1, 15)]  # 14 个关节名称
cmd_arm_joint_msg.position = [-30, 20, 15, -45, 25, 10, -35,
                         -30,-20,-15, -45,-25,-10, -35] # 14 个关节角度 (度)

# 发布消息
cmd_arm_joint_pub.publish(cmd_arm_joint_msg)
print("cmd_arm_joint 消息已发布")
```



------

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
- 使用示例

  ```python
  #!/usr/bin/env python
  import rospy
  import numpy as np
  from std_msgs.msg import Header
  from kuavo_msgs.msg import twoArmHandPoseCmd, twoArmHandPose, armHandPose, ikSolveParam
  import tf.transformations as tf_trans
  import time
  import os
  import sys
  
  # 添加父目录到路径以便导入 lb_ctrl_api
  sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
  import lb_ctrl_api as ct
  
  # 初始化ROS节点
  rospy.init_node('cmd_arm_ee_publisher')
  
  # 创建发布者
  cmd_arm_ee_pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
  
  # 等待发布器建立连接
  time.sleep(1)
  
  # 设置控制模式为 ArmOnly (模式1)
  ct.set_control_mode(1)
  
  # 构建手部位姿函数
  def build_hand_pose(x, y, z, yaw, pitch, roll):
      """根据 x,y,z,yaw,pitch,roll 构造 armHandPose"""
      pose = armHandPose()
      pose.pos_xyz = [x, y, z]
      quat = tf_trans.quaternion_from_euler(np.radians(roll), np.radians(pitch), np.radians(yaw))
      quat_norm = quat / np.linalg.norm(quat) if np.linalg.norm(quat) > 1e-8 else [0, 0, 0, 1]
      pose.quat_xyzw = quat_norm.tolist()
      pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
      pose.joint_angles = [0.0] * 7
      return pose
  
  # 构建IK参数
  def build_ik_param():
      """构造默认 IK 参数"""
      p = ikSolveParam()
      p.major_optimality_tol = 1e-6
      p.major_feasibility_tol = 1e-6
      p.minor_feasibility_tol = 1e-6
      p.major_iterations_limit = 1000
      p.oritation_constraint_tol = 0.01
      p.pos_constraint_tol = 0.01
      p.pos_cost_weight = 1.0
      return p
  
  # 创建双臂手部位姿命令消息
  cmd_arm_ee_msg = twoArmHandPoseCmd()
  cmd_arm_ee_msg.hand_poses = twoArmHandPose()
  cmd_arm_ee_msg.hand_poses.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
  # 左手位置: [x, y, z, yaw, pitch, roll]
  cmd_arm_ee_msg.hand_poses.left_pose = build_hand_pose(0.0, 0.4, 0.0, 0.0, 0.0, 0.0)
  # 右手位置: [x, y, z, yaw, pitch, roll]
  cmd_arm_ee_msg.hand_poses.right_pose = build_hand_pose(0.0, -0.4, 0.0, 0.0, 0.0, 0.0)
  cmd_arm_ee_msg.use_custom_ik_param = True
  cmd_arm_ee_msg.joint_angles_as_q0 = False
  cmd_arm_ee_msg.ik_param = build_ik_param()
  cmd_arm_ee_msg.frame = 2  # local frame (2=local, 1=world)
  
  # 发布消息
  cmd_arm_ee_pub.publish(cmd_arm_ee_msg)
  print("cmd_arm_ee 消息已发布")
  ```

  

------

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

  使用示例

  ```python
  #!/usr/bin/env python
  import rospy
  from geometry_msgs.msg import Twist
  from std_msgs.msg import Float32
  import time
  
  # 全局变量
  reachTime = 0.0
  
  def time_callback(msg):
      """时间回调函数"""
      global reachTime
      reachTime = msg.data
      print(f"reach_time is {reachTime:.3f} s")
  
  # 初始化ROS节点
  rospy.init_node('cmd_pose_reach_time_subscriber', anonymous=True)
  
  # 创建发布者
  cmd_pose_pub = rospy.Publisher('/cmd_pose', Twist, queue_size=10)
  
  # 创建订阅者
  time_sub = rospy.Subscriber('/lb_cmd_pose_reach_time', Float32, time_callback)
  
  # 等待发布器和订阅器建立连接
  time.sleep(1)
  
  # 创建Twist消息对象
  cmd_pose_msg = Twist()
  cmd_pose_msg.linear.x = 0.5  # 基于当前位置的 x 方向值 (m)
  cmd_pose_msg.linear.y = 0.0  # 基于当前位置的 y 方向值 (m)
  cmd_pose_msg.linear.z = 0.0  # 增量高度
  cmd_pose_msg.angular.z = 0.0  # 基于当前位置旋转（偏航）的角度，单位为弧度 (radian)
  
  # 发布消息
  cmd_pose_pub.publish(cmd_pose_msg)
  print("cmd_pose 消息已发布，开始订阅 /lb_cmd_pose_reach_time 话题...")
  
  # 保持节点运行
  rospy.spin()
  ```

  

------

#### 2. `/lb_leg_joint_reach_time` - 下肢关节指令到达时间反馈

**消息类型：** `std_msgs/Float32`

**功能描述：** 下发新的 lb_leg_traj 话题时，与此同时，会发布一个执行所需时间

**字段说明：**

```yaml
float32 data    # 预计到达时间（秒），假如收到2，则表示2秒后执行完成
```

**相关演示脚本：**

- `cmd_leg_joint_test.py`

使用示例

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import time

# 全局变量
reachTime = 0.0

def time_callback(msg):
    """时间回调函数"""
    global reachTime
    reachTime = msg.data
    print(f"reach_time is {reachTime:.3f} s")

# 初始化ROS节点
rospy.init_node('cmd_leg_joint_reach_time_subscriber', anonymous=True)

# 创建发布者
cmd_leg_joint_pub = rospy.Publisher('/lb_leg_traj', JointState, queue_size=10)

# 创建订阅者
time_sub = rospy.Subscriber('/lb_leg_joint_reach_time', Float32, time_callback)

# 等待发布器和订阅器建立连接
time.sleep(1)

# 创建JointState消息对象
cmd_leg_joint_msg = JointState()
cmd_leg_joint_msg.header.stamp = rospy.Time.now()
cmd_leg_joint_msg.position = [14.90, -32.01, 18.03, 0.0]  # 腿部关节角度 (度)

# 发布消息
cmd_leg_joint_pub.publish(cmd_leg_joint_msg)
print("cmd_leg_joint 消息已发布，开始订阅 /lb_leg_joint_reach_time 话题...")

# 保持节点运行
rospy.spin()
```



------

#### 3. `/lb_torso_pose_reach_time` - 躯干笛卡尔位姿到达时间反馈

**消息类型：** `std_msgs/Float32`

**功能描述：** 下发新的 cmd_lb_torso_pose 话题时，与此同时，会发布一个执行所需时间

**字段说明：**

```yaml
float32 data    # 预计到达时间（秒），假如收到2，则表示2秒后执行完成
```

**相关演示脚本：**

- `cmd_torso_pose_test.py`

使用示例

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

# 全局变量
reachTime = 0.0

def time_callback(msg):
    """时间回调函数"""
    global reachTime
    reachTime = msg.data
    print(f"reach_time is {reachTime:.3f} s")

# 初始化ROS节点
rospy.init_node('cmd_torso_pose_reach_time_subscriber', anonymous=True)

# 创建发布者
cmd_torso_pose_pub = rospy.Publisher('/cmd_lb_torso_pose', Twist, queue_size=10)

# 创建订阅者
time_sub = rospy.Subscriber('/lb_torso_pose_reach_time', Float32, time_callback)

# 等待发布器和订阅器建立连接
time.sleep(1)

# 创建Twist消息对象
cmd_torso_pose_msg = Twist()
cmd_torso_pose_msg.linear.x = 0.196123  # 躯干 x 方向位置 (m)
cmd_torso_pose_msg.linear.y = 0.0005     # 躯干 y 方向位置 (m)
cmd_torso_pose_msg.linear.z = 0.789919  # 躯干 z 方向位置 (m)
cmd_torso_pose_msg.angular.x = 0.0       # 滚转角 (rad)
cmd_torso_pose_msg.angular.y = 0.0       # 俯仰角 (rad)
cmd_torso_pose_msg.angular.z = 0.0       # 偏航角 (rad)

# 发布消息
cmd_torso_pose_pub.publish(cmd_torso_pose_msg)
print("cmd_torso_pose 消息已发布，开始订阅 /lb_torso_pose_reach_time 话题...")

# 保持节点运行
rospy.spin()
```



------

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

使用示例

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import time
import sys
import os

# 添加父目录到路径以便导入 lb_ctrl_api
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import lb_ctrl_api as ct

# 全局变量
reachTime = 0.0

def time_callback(msg):
    """时间回调函数"""
    global reachTime
    reachTime = msg.data
    print(f"reach_time is {reachTime:.3f} s")

# 初始化ROS节点
rospy.init_node('cmd_arm_joint_reach_time_subscriber', anonymous=True)

# 创建发布者
cmd_arm_joint_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)

# 创建订阅者
time_sub = rospy.Subscriber('/lb_arm_joint_reach_time', Float32, time_callback)

# 等待发布器和订阅器建立连接
time.sleep(1)

# 设置控制模式为 ArmOnly (模式1)
ct.set_control_mode(1)

# 创建JointState消息对象
cmd_arm_joint_msg = JointState()
cmd_arm_joint_msg.header.stamp = rospy.Time.now()
cmd_arm_joint_msg.name = [f'joint{i}' for i in range(1, 15)]  # 14 个关节名称
cmd_arm_joint_msg.position = [-30, 20, 15, -45, 25, 10, -35,
                         -30,-20,-15, -45,-25,-10, -35] # 14 个关节角度 (度)

# 发布消息
cmd_arm_joint_pub.publish(cmd_arm_joint_msg)
print("cmd_arm_joint 消息已发布，开始订阅 /lb_arm_joint_reach_time 话题...")

# 保持节点运行
rospy.spin()
```



------

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

使用示例

```python
#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Header, Float32
from kuavo_msgs.msg import twoArmHandPoseCmd, twoArmHandPose, armHandPose, ikSolveParam
import tf.transformations as tf_trans

import os
import sys
# 添加父目录到路径以便导入 lb_ctrl_api
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import lb_ctrl_api as ct

# 全局变量
reachTime = 0.0

def time_callback(msg):
    """时间回调函数"""
    global reachTime
    reachTime = msg.data
    print(f"reach_time is {reachTime:.3f} s")

# 初始化ROS节点
rospy.init_node('cmd_arm_ee_reach_time_subscriber', anonymous=True)

# 创建发布者
cmd_arm_ee_pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)

# 创建订阅者
time_sub = rospy.Subscriber('/lb_arm_ee_reach_time', Float32, time_callback)

# 等待发布器和订阅器建立连接
rospy.sleep(1.0)

# 设置控制模式为 ArmOnly (模式1)
ct.set_control_mode(1)

# 构建手部位姿函数
def build_hand_pose(x, y, z, yaw, pitch, roll):
    """根据 x,y,z,yaw,pitch,roll 构造 armHandPose"""
    pose = armHandPose()
    pose.pos_xyz = [x, y, z]
    quat = tf_trans.quaternion_from_euler(np.radians(roll), np.radians(pitch), np.radians(yaw))
    quat_norm = quat / np.linalg.norm(quat) if np.linalg.norm(quat) > 1e-8 else [0, 0, 0, 1]
    pose.quat_xyzw = quat_norm.tolist()
    pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
    pose.joint_angles = [0.0] * 7
    return pose

# 构建IK参数
def build_ik_param():
    """构造默认 IK 参数"""
    p = ikSolveParam()
    p.major_optimality_tol = 1e-6
    p.major_feasibility_tol = 1e-6
    p.minor_feasibility_tol = 1e-6
    p.major_iterations_limit = 1000
    p.oritation_constraint_tol = 0.01
    p.pos_constraint_tol = 0.01
    p.pos_cost_weight = 1.0
    return p

# 创建双臂手部位姿命令消息
cmd_arm_ee_msg = twoArmHandPoseCmd()
cmd_arm_ee_msg.hand_poses = twoArmHandPose()
cmd_arm_ee_msg.hand_poses.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
# 左手位置: [x, y, z, yaw, pitch, roll]
cmd_arm_ee_msg.hand_poses.left_pose = build_hand_pose(0.0, 0.4, 0.0, 0.0, 0.0, 0.0)
# 右手位置: [x, y, z, yaw, pitch, roll]
cmd_arm_ee_msg.hand_poses.right_pose = build_hand_pose(0.0, -0.4, 0.0, 0.0, 0.0, 0.0)
cmd_arm_ee_msg.use_custom_ik_param = True
cmd_arm_ee_msg.joint_angles_as_q0 = False
cmd_arm_ee_msg.ik_param = build_ik_param()
cmd_arm_ee_msg.frame = 2  # local frame (2=local, 1=world)

# 发布消息
cmd_arm_ee_pub.publish(cmd_arm_ee_msg)
print("cmd_arm_ee 消息已发布，开始订阅 /lb_arm_ee_reach_time 话题...")

# 保持节点运行
rospy.spin()
```



------

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



------

#### 7. `/mobile_manipulator/currentMpcTarget/state` - 轮臂MPC当前时刻的cost期望state

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 保存底盘，下肢和上肢关节的位置参考，根据控制模式，未使能时候可不参考

**字段说明：**

```yaml
Float64MultiArray data    # 数据格式(共3+4+14自由度)：[base位姿(x, y, yaw)(3), 下肢关节位置(4), 上肢关节位置(14)]
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8

def control_mode_callback(msg):
    """控制模式回调函数"""
    mode = msg.data
    mode_names = {
        0: "NoControl - 无控制",
        1: "ArmOnly - 仅控制手臂，基座固定",
        2: "BaseOnly - 仅控制基座，手臂固定",
        3: "BaseArm - 同时控制基座和手臂",
        4: "ArmEeOnly - 仅控制手臂末端"
    }
    mode_name = mode_names.get(mode, f"未知模式({mode})")
    print(f"当前MPC控制模式: {mode} - {mode_name}")

# 初始化ROS节点
rospy.init_node('sub_lb_mpc_control_mode', anonymous=True)

# 创建订阅者
control_mode_sub = rospy.Subscriber(
    '/mobile_manipulator/lb_mpc_control_mode', 
    Int8, 
    control_mode_callback
)

print("开始订阅 /mobile_manipulator/lb_mpc_control_mode 话题...")

# 保持节点运行
rospy.spin()
```



------

#### 8. `/mobile_manipulator/currentMpcTarget/input` - 轮臂MPC当前时刻的cost期望input

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 保存底盘，下肢和上肢关节的速度参考，根据控制模式，未使能时候可不参考

**字段说明：**

```yaml
Float64MultiArray data    # 数据格式(共3+4+14自由度)：[base速度(x, y, yaw)(3), 下肢关节速度(4), 上肢关节速度(14)]

std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim #一个维度描述数组，每个元素描述一个维度
    string label #维度的名称
    uint32 size #维度的大小
    uint32 stride #步长
  uint32 data_offset #数据在 data数组中的起始偏移量
float64[] data
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def mpc_target_state_callback(msg):
    """MPC目标状态回调函数"""
    data = msg.data
    
    if len(data) != 21:
        print(f"警告: 数据长度不正确，期望21，实际{len(data)}")
        return
    
    # 解析数据
    base_pose = data[0:3]      # base位姿(x, y, yaw) - 3个
    leg_joints = data[3:7]     # 下肢关节位置 - 4个
    arm_joints = data[7:21]    # 上肢关节位置 - 14个
    
    print("=" * 60)
    print("MPC目标状态 (currentMpcTarget/state):")
    print(f"  底盘位姿 (x, y, yaw): [{base_pose[0]:.4f}, {base_pose[1]:.4f}, {base_pose[2]:.4f}]")
    print(f"  下肢关节位置 (4个): {[f'{x:.4f}' for x in leg_joints]}")
    print(f"  上肢关节位置 (14个): {[f'{x:.4f}' for x in arm_joints]}")

# 初始化ROS节点
rospy.init_node('sub_current_mpc_target_state', anonymous=True)

# 创建订阅者
mpc_target_state_sub = rospy.Subscriber(
    '/mobile_manipulator/currentMpcTarget/state', 
    Float64MultiArray, 
    mpc_target_state_callback
)

print("开始订阅 /mobile_manipulator/currentMpcTarget/state 话题...")

# 保持节点运行
rospy.spin()
```



------

#### 9. `/mobile_manipulator/torso_target_6D` - 轮臂MPC当前时刻的躯干笛卡尔期望

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 保存躯干笛卡尔6D期望，根据控制模式，未使能时候可不参考

**字段说明：**

```yaml
Float64MultiArray data    # 数据格式(共6自由度)：[躯干位姿(x, y, z, yaw, pitch, roll)]
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim #一个维度描述数组，每个元素描述一个维度
    string label #维度的名称
    uint32 size #维度的大小
    uint32 stride #步长
  uint32 data_offset #数据在data数组中的起始偏移量
float64[] data
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def torso_target_6d_callback(msg):
    """躯干6D目标回调函数"""
    data = msg.data
    
    if len(data) != 6:
        print(f"警告: 数据长度不正确，期望6，实际{len(data)}")
        return
    
    # 解析数据: [x, y, z, yaw, pitch, roll]
    x = data[0]
    y = data[1]
    z = data[2]
    yaw = data[3]
    pitch = data[4]
    roll = data[5]
    
    print("=" * 60)
    print("躯干6D目标 (torso_target_6D):")
    print(f"  位置 (x, y, z): [{x:.4f}, {y:.4f}, {z:.4f}]")
    print(f"  姿态 (yaw, pitch, roll): [{yaw:.4f}, {pitch:.4f}, {roll:.4f}]")
    print(f"  姿态 (度): [{yaw*180/3.14159:.2f}°, {pitch*180/3.14159:.2f}°, {roll*180/3.14159:.2f}°]")

# 初始化ROS节点
rospy.init_node('sub_torso_target_6d', anonymous=True)

# 创建订阅者
torso_target_6d_sub = rospy.Subscriber(
    '/mobile_manipulator/torso_target_6D', 
    Float64MultiArray, 
    torso_target_6d_callback
)

print("开始订阅 /mobile_manipulator/torso_target_6D 话题...")

# 保持节点运行
rospy.spin()
```



------

#### 10. `/mobile_manipulator/ee_target_6D/point` - 轮臂MPC当前时刻的手臂末端笛卡尔期望

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 保存双臂末端笛卡尔期望，根据控制模式，未使能时候可不参考

**注意事项：** 在 point 后需要加上索引，目前支持 point0，point1，分别是左臂右臂

**字段说明：**

```yaml
Float64MultiArray data    # 数据格式(共6自由度)：[末端位姿(x, y, z, yaw, pitch, roll)]
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim #一个维度描述数组，每个元素描述一个维度
    string label #维度的名称
    uint32 size #维度的大小
    uint32 stride #步长
  uint32 data_offset #数据在 data数组中的起始偏移量
float64[] data
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

# 存储最新的数据
left_arm_data = None
right_arm_data = None

def ee_target_6d_callback(msg, arm_name):
    """手臂末端6D目标回调函数"""
    global left_arm_data, right_arm_data
    
    data = msg.data
    
    if len(data) != 6:
        print(f"警告: {arm_name}数据长度不正确，期望6，实际{len(data)}")
        return
    
    # 解析数据: [x, y, z, yaw, pitch, roll]
    x = data[0]
    y = data[1]
    z = data[2]
    yaw = data[3]
    pitch = data[4]
    roll = data[5]
    
    # 保存数据
    if arm_name == "左臂":
        left_arm_data = data
    else:
        right_arm_data = data
    
    print("=" * 60)
    print(f"{arm_name}末端6D目标 (ee_target_6D):")
    print(f"  位置 (x, y, z): [{x:.4f}, {y:.4f}, {z:.4f}]")
    print(f"  姿态 (yaw, pitch, roll): [{yaw:.4f}, {pitch:.4f}, {roll:.4f}]")
    print(f"  姿态 (度): [{yaw*180/3.14159:.2f}°, {pitch*180/3.14159:.2f}°, {roll*180/3.14159:.2f}°]")

def left_arm_callback(msg):
    """左臂回调函数"""
    ee_target_6d_callback(msg, "左臂")

def right_arm_callback(msg):
    """右臂回调函数"""
    ee_target_6d_callback(msg, "右臂")

# 初始化ROS节点
rospy.init_node('sub_ee_target_6d', anonymous=True)

# 创建订阅者 - 左臂 (point0)
left_arm_sub = rospy.Subscriber(
    '/mobile_manipulator/ee_target_6D/point0', 
    Float64MultiArray, 
    left_arm_callback
)

# 创建订阅者 - 右臂 (point1)
right_arm_sub = rospy.Subscriber(
    '/mobile_manipulator/ee_target_6D/point1', 
    Float64MultiArray, 
    right_arm_callback
)

print("开始订阅手臂末端6D目标话题...")
print("  - /mobile_manipulator/ee_target_6D/point0 (左臂)")
print("  - /mobile_manipulator/ee_target_6D/point1 (右臂)")

# 保持节点运行
rospy.spin()
```



------

#### 11. `/mobile_manipulator_wbc_observation` - 机器人收到的原生传感器数据

**消息类型：** `std_msgs::Float64MultiArray`

**功能描述：** 机器人收到的原生传感器数据

**注意事项**：在 /mobile_manipulator_wbc_observation 后加上/state或者/input，分别表示状态和输入

**字段说明：**

```yaml
Float64 time               # MPC 模块内部时间
int64   mode               # MPC 的步态，不是人形，无效，不需要注意
Float64MultiArray state    # 数据格式(共3+4+14自由度)：[base位姿(x, y, yaw)(3), 下肢关节位置(4), 上肢关节位置(14)]
Float64MultiArray input    # 数据格式(共3+4+14自由度)：[base速度(x, y, yaw)(3), 下肢关节速度(4), 上肢关节速度(14)]
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

# 存储最新的数据
state_data = None
input_data = None

def wbc_observation_state_callback(msg):
    """WBC观测状态回调函数"""
    global state_data
    data = msg.data
    state_data = data
    
    if len(data) != 21:
        print(f"警告: state数据长度不正确，期望21，实际{len(data)}")
        return
    
    # 解析数据
    base_pose = data[0:3]      # base位姿(x, y, yaw) - 3个
    leg_joints = data[3:7]    # 下肢关节位置 - 4个
    arm_joints = data[7:21]   # 上肢关节位置 - 14个
    
    print("=" * 60)
    print("WBC观测状态 (mobile_manipulator_wbc_observation/state):")
    print(f"  底盘位姿 (x, y, yaw): [{base_pose[0]:.4f}, {base_pose[1]:.4f}, {base_pose[2]:.4f}]")
    print(f"  下肢关节位置 (4个): {[f'{x:.4f}' for x in leg_joints]}")
    print(f"  上肢关节位置 (14个): {[f'{x:.4f}' for x in arm_joints]}")

def wbc_observation_input_callback(msg):
    """WBC观测输入回调函数"""
    global input_data
    data = msg.data
    input_data = data
    
    if len(data) != 21:
        print(f"警告: input数据长度不正确，期望21，实际{len(data)}")
        return
    
    # 解析数据
    base_vel = data[0:3]       # base速度(x, y, yaw) - 3个
    leg_joint_vel = data[3:7]  # 下肢关节速度 - 4个
    arm_joint_vel = data[7:21] # 上肢关节速度 - 14个
    
    print("=" * 60)
    print("WBC观测输入 (mobile_manipulator_wbc_observation/input):")
    print(f"  底盘速度 (x, y, yaw): [{base_vel[0]:.4f}, {base_vel[1]:.4f}, {base_vel[2]:.4f}]")
    print(f"  下肢关节速度 (4个): {[f'{x:.4f}' for x in leg_joint_vel]}")
    print(f"  上肢关节速度 (14个): {[f'{x:.4f}' for x in arm_joint_vel]}")

# 初始化ROS节点
rospy.init_node('sub_wbc_observation', anonymous=True)

# 创建订阅者 - state
wbc_state_sub = rospy.Subscriber(
    '/mobile_manipulator_wbc_observation/state', 
    Float64MultiArray, 
    wbc_observation_state_callback
)

# 创建订阅者 - input
wbc_input_sub = rospy.Subscriber(
    '/mobile_manipulator_wbc_observation/input', 
    Float64MultiArray, 
    wbc_observation_input_callback
)

print("开始订阅WBC观测话题...")
print("  - /mobile_manipulator_wbc_observation/state (状态)")
print("  - /mobile_manipulator_wbc_observation/input (输入)")

# 保持节点运行
rospy.spin()
```



------

#### 12. `/mobile_manipulator_mpc_observation` - 输入到轮臂MPC的原生传感器数据(经过滤波)

**消息类型：** `ocs2_msgs/mpc_observation`

**功能描述：** 输入到轮臂MPC的原生传感器数据，使用 kinemicLimitFilter 进行时域滤波，二阶连续

**字段说明：**

```yaml
Float64 time               # MPC 模块内部时间
ocs2_msgs/mpc_state state      
	float32[] value        # 数据格式(共3+4+14自由度)：[base位姿(x, y, yaw)(3), 下肢关节位置(4), 上肢关节位置(14)]
	
ocs2_msgs/mpc_input input
	float32[] value        # 数据格式(共3+4+14自由度)：[base速度(x, y, yaw)(3), 下肢关节速度(4), 上肢关节速度(14)]
int8 mode                  # MPC 的步态，不是人形，无效，不需要注意
```

使用示例

```python
#!/usr/bin/env python
import rospy
from ocs2_msgs.msg import mpc_observation

def mpc_observation_callback(msg):
    """MPC观测回调函数"""
    time = msg.time
    mode = msg.mode
    state = msg.state.value
    input_data = msg.input.value
    
    # 检查数据长度
    if len(state) != 21:
        print(f"警告: state数据长度不正确，期望21，实际{len(state)}")
        return
    
    if len(input_data) != 21:
        print(f"警告: input数据长度不正确，期望21，实际{len(input_data)}")
        return
    
    # 解析state数据
    base_pose = state[0:3]      # base位姿(x, y, yaw) - 3个
    leg_joints = state[3:7]     # 下肢关节位置 - 4个
    arm_joints = state[7:21]    # 上肢关节位置 - 14个
    
    # 解析input数据
    base_vel = input_data[0:3]       # base速度(x, y, yaw) - 3个
    leg_joint_vel = input_data[3:7]   # 下肢关节速度 - 4个
    arm_joint_vel = input_data[7:21]  # 上肢关节速度 - 14个
    
    print("=" * 60)
    print(f"MPC观测 (mobile_manipulator_mpc_observation) - 时间: {time:.4f}s, 模式: {mode}")
    print("-" * 60)
    print("状态 (state):")
    print(f"  底盘位姿 (x, y, yaw): [{base_pose[0]:.4f}, {base_pose[1]:.4f}, {base_pose[2]:.4f}]")
    print(f"  下肢关节位置 (4个): {[f'{x:.4f}' for x in leg_joints]}")
    print(f"  上肢关节位置 (14个): {[f'{x:.4f}' for x in arm_joints]}")
    print("-" * 60)
    print("输入 (input):")
    print(f"  底盘速度 (x, y, yaw): [{base_vel[0]:.4f}, {base_vel[1]:.4f}, {base_vel[2]:.4f}]")
    print(f"  下肢关节速度 (4个): {[f'{x:.4f}' for x in leg_joint_vel]}")
    print(f"  上肢关节速度 (14个): {[f'{x:.4f}' for x in arm_joint_vel]}")

# 初始化ROS节点
rospy.init_node('sub_mpc_observation', anonymous=True)

# 创建订阅者
mpc_observation_sub = rospy.Subscriber(
    '/mobile_manipulator_mpc_observation', 
    mpc_observation, 
    mpc_observation_callback
)

print("开始订阅 /mobile_manipulator_mpc_observation 话题...")
print("(输入到轮臂MPC的原生传感器数据，经过滤波)")

# 保持节点运行
rospy.spin()
```



------

#### 13. `/humanoid_wheel/optimizedState_mrt` - MPC输出的原生MRT的state期望

**消息类型：** `std_msgs/Float64MultiArray`

**功能描述：** MPC 计算，并通过 MRT 模块插值后输出的state期望

**字段说明：**

```yaml
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim #一个维度描述数组，每个元素描述一个维度
    string label #维度的名称
    uint32 size #维度的大小
    uint32 stride #步长
  uint32 data_offset #数据在 data数组中的起始偏移量
float64[] data
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def optimized_state_mrt_callback(msg):
    """MPC优化状态MRT回调函数"""
    data = msg.data
    
    if len(data) != 21:
        print(f"警告: 数据长度不正确，期望21，实际{len(data)}")
        return
    
    # 解析数据
    base_pose = data[0:3]      # base位姿(x, y, yaw) - 3个
    leg_joints = data[3:7]     # 下肢关节位置 - 4个
    arm_joints = data[7:21]     # 上肢关节位置 - 14个
    
    print("=" * 60)
    print("MPC优化状态MRT (humanoid_wheel/optimizedState_mrt):")
    print("(MPC计算并通过MRT模块插值后输出的state期望)")
    print(f"  底盘位姿 (x, y, yaw): [{base_pose[0]:.4f}, {base_pose[1]:.4f}, {base_pose[2]:.4f}]")
    print(f"  下肢关节位置 (4个): {[f'{x:.4f}' for x in leg_joints]}")
    print(f"  上肢关节位置 (14个): {[f'{x:.4f}' for x in arm_joints]}")

# 初始化ROS节点
rospy.init_node('sub_optimized_state_mrt', anonymous=True)

# 创建订阅者
optimized_state_mrt_sub = rospy.Subscriber(
    '/humanoid_wheel/optimizedState_mrt', 
    Float64MultiArray, 
    optimized_state_mrt_callback
)

print("开始订阅 /humanoid_wheel/optimizedState_mrt 话题...")
print("(MPC输出的原生MRT的state期望)")

# 保持节点运行
rospy.spin()
```



------

#### 14. `/humanoid_wheel/optimizedInput_mrt` - MPC输出的原生MRT的input期望

**消息类型：** `std_msgs/Float64MultiArray`

**功能描述：** MPC 计算，并通过 MRT 模块插值后输出的input期望

**字段说明：**

```yaml
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim #一个维度描述数组，每个元素描述一个维度
    string label #维度的名称
    uint32 size #维度的大小
    uint32 stride #步长
  uint32 data_offset #数据在 data数组中的起始偏移量
float64[] data
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def optimized_input_mrt_callback(msg):
    """MPC优化输入MRT回调函数"""
    data = msg.data
    
    if len(data) != 21:
        print(f"警告: 数据长度不正确，期望21，实际{len(data)}")
        return
    
    # 解析数据
    base_vel = data[0:3]       # base速度(x, y, yaw) - 3个
    leg_joint_vel = data[3:7]  # 下肢关节速度 - 4个
    arm_joint_vel = data[7:21]  # 上肢关节速度 - 14个
    
    print("=" * 60)
    print("MPC优化输入MRT (humanoid_wheel/optimizedInput_mrt):")
    print("(MPC计算并通过MRT模块插值后输出的input期望)")
    print(f"  底盘速度 (x, y, yaw): [{base_vel[0]:.4f}, {base_vel[1]:.4f}, {base_vel[2]:.4f}]")
    print(f"  下肢关节速度 (4个): {[f'{x:.4f}' for x in leg_joint_vel]}")
    print(f"  上肢关节速度 (14个): {[f'{x:.4f}' for x in arm_joint_vel]}")

# 初始化ROS节点
rospy.init_node('sub_optimized_input_mrt', anonymous=True)

# 创建订阅者
optimized_input_mrt_sub = rospy.Subscriber(
    '/humanoid_wheel/optimizedInput_mrt', 
    Float64MultiArray, 
    optimized_input_mrt_callback
)

print("开始订阅 /humanoid_wheel/optimizedInput_mrt 话题...")
print("(MPC输出的原生MRT的input期望)")

# 保持节点运行
rospy.spin()
```



------

#### 15. `/humanoid_wheel/optimizedState_mrt_kinemicLimit` - 经过运动学限制滤波的MRT的state期望

**消息类型：** `std_msgs/Float64MultiArray`

**功能描述：** MPC 计算，并通过 MRT 模块插值后输出的state期望，使用 KinemicLimitFilter 进行滤波，保证二阶连续

**字段说明：**

```yaml
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim #一个维度描述数组，每个元素描述一个维度
    string label #维度的名称
    uint32 size #维度的大小
    uint32 stride #步长
  uint32 data_offset #数据在 data数组中的起始偏移量
float64[] data
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def optimized_state_mrt_kinemic_limit_callback(msg):
    """MPC优化状态MRT运动学限制滤波回调函数"""
    data = msg.data
    
    if len(data) != 21:
        print(f"警告: 数据长度不正确，期望21，实际{len(data)}")
        return
    
    # 解析数据
    base_pose = data[0:3]      # base位姿(x, y, yaw) - 3个
    leg_joints = data[3:7]     # 下肢关节位置 - 4个
    arm_joints = data[7:21]    # 上肢关节位置 - 14个
    
    print("=" * 60)
    print("MPC优化状态MRT运动学限制滤波 (humanoid_wheel/optimizedState_mrt_kinemicLimit):")
    print("(经过运动学限制滤波的MRT的state期望，保证二阶连续)")
    print(f"  底盘位姿 (x, y, yaw): [{base_pose[0]:.4f}, {base_pose[1]:.4f}, {base_pose[2]:.4f}]")
    print(f"  下肢关节位置 (4个): {[f'{x:.4f}' for x in leg_joints]}")
    print(f"  上肢关节位置 (14个): {[f'{x:.4f}' for x in arm_joints]}")

# 初始化ROS节点
rospy.init_node('sub_optimized_state_mrt_kinemic_limit', anonymous=True)

# 创建订阅者
optimized_state_mrt_kinemic_limit_sub = rospy.Subscriber(
    '/humanoid_wheel/optimizedState_mrt_kinemicLimit', 
    Float64MultiArray, 
    optimized_state_mrt_kinemic_limit_callback
)

print("开始订阅 /humanoid_wheel/optimizedState_mrt_kinemicLimit 话题...")
print("(经过运动学限制滤波的MRT的state期望)")

# 保持节点运行
rospy.spin()
```



------

#### 16. `/humanoid_wheel/optimizedInput_mrt_kinemicLimit` - 经过运动学限制滤波的MRT的input期望

**消息类型：** `std_msgs/Float64MultiArray`

**功能描述：** MPC 计算，并通过 MRT 模块插值后输出的input期望，使用 KinemicLimitFilter 进行滤波，保证二阶连续

**字段说明：**

```yaml
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim #一个维度描述数组，每个元素描述一个维度
    string label #维度的名称
    uint32 size #维度的大小
    uint32 stride #步长
  uint32 data_offset #数据在 data数组中的起始偏移量
float64[] data]
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def optimized_input_mrt_kinemic_limit_callback(msg):
    """MPC优化输入MRT运动学限制滤波回调函数"""
    data = msg.data
    
    if len(data) != 21:
        print(f"警告: 数据长度不正确，期望21，实际{len(data)}")
        return
    
    # 解析数据
    base_vel = data[0:3]       # base速度(x, y, yaw) - 3个
    leg_joint_vel = data[3:7]  # 下肢关节速度 - 4个
    arm_joint_vel = data[7:21]  # 上肢关节速度 - 14个
    
    print("=" * 60)
    print("MPC优化输入MRT运动学限制滤波 (humanoid_wheel/optimizedInput_mrt_kinemicLimit):")
    print("(经过运动学限制滤波的MRT的input期望，保证二阶连续)")
    print(f"  底盘速度 (x, y, yaw): [{base_vel[0]:.4f}, {base_vel[1]:.4f}, {base_vel[2]:.4f}]")
    print(f"  下肢关节速度 (4个): {[f'{x:.4f}' for x in leg_joint_vel]}")
    print(f"  上肢关节速度 (14个): {[f'{x:.4f}' for x in arm_joint_vel]}")

# 初始化ROS节点
rospy.init_node('sub_optimized_input_mrt_kinemic_limit', anonymous=True)

# 创建订阅者
optimized_input_mrt_kinemic_limit_sub = rospy.Subscriber(
    '/humanoid_wheel/optimizedInput_mrt_kinemicLimit', 
    Float64MultiArray, 
    optimized_input_mrt_kinemic_limit_callback
)

print("开始订阅 /humanoid_wheel/optimizedInput_mrt_kinemicLimit 话题...")
print("(经过运动学限制滤波的MRT的input期望)")

# 保持节点运行
rospy.spin()
```



------

#### 17. `/humanoid_wheel/bodyAcc` - WBC求解的底盘的加速度

**消息类型：** `std_msgs/Float64MultiArray`

**功能描述：** WBC 模块输出的底盘加速度

**字段说明：**

```yaml
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim #一个维度描述数组，每个元素描述一个维度
    string label #维度的名称
    uint32 size #维度的大小
    uint32 stride #步长
  uint32 data_offset #数据在 data数组中的起始偏移量
float64[] data
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def body_acc_callback(msg):
    """底盘加速度回调函数"""
    data = msg.data
    
    if len(data) < 3:
        print(f"警告: 数据长度不正确，期望至少3，实际{len(data)}")
        return
    
    # 解析数据: [base加速度(x, y, yaw)(3)]
    acc_x = data[0]
    acc_y = data[1]
    acc_yaw = data[2]
    
    print("=" * 60)
    print("WBC底盘加速度 (humanoid_wheel/bodyAcc):")
    print(f"  底盘加速度 (x, y, yaw): [{acc_x:.4f}, {acc_y:.4f}, {acc_yaw:.4f}]")
    
    # 如果有额外的数据，也显示出来
    if len(data) > 3:
        print(f"  额外数据 ({len(data)-3}个): {[f'{x:.4f}' for x in data[3:]]}")

# 初始化ROS节点
rospy.init_node('sub_body_acc', anonymous=True)

# 创建订阅者
body_acc_sub = rospy.Subscriber(
    '/humanoid_wheel/bodyAcc', 
    Float64MultiArray, 
    body_acc_callback
)

print("开始订阅 /humanoid_wheel/bodyAcc 话题...")
print("(WBC求解的底盘的加速度)")

# 保持节点运行
rospy.spin()
```



------

#### 18. `/humanoid_wheel/jointAcc` - WBC求解的关节加速度(包括下肢上肢)

**消息类型：** `std_msgs/Float64MultiArray`

**功能描述：** WBC 模块输出的关节加速度

**字段说明：**

```yaml
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim #一个维度描述数组，每个元素描述一个维度
    string label #维度的名称
    uint32 size #维度的大小
    uint32 stride #步长
  uint32 data_offset #数据在 data数组中的起始偏移量
float64[] data
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def joint_acc_callback(msg):
    """关节加速度回调函数"""
    data = msg.data
    
    if len(data) != 18:
        print(f"警告: 数据长度不正确，期望18，实际{len(data)}")
        return
    
    # 解析数据: [下肢加速度(4), 上肢加速度(14)]
    leg_joint_acc = data[0:4]   # 下肢关节加速度 - 4个
    arm_joint_acc = data[4:18]  # 上肢关节加速度 - 14个
    
    print("=" * 60)
    print("WBC关节加速度 (humanoid_wheel/jointAcc):")
    print("(WBC求解的关节加速度，包括下肢和上肢)")
    print(f"  下肢关节加速度 (4个): {[f'{x:.4f}' for x in leg_joint_acc]}")
    print(f"  上肢关节加速度 (14个): {[f'{x:.4f}' for x in arm_joint_acc]}")

# 初始化ROS节点
rospy.init_node('sub_joint_acc', anonymous=True)

# 创建订阅者
joint_acc_sub = rospy.Subscriber(
    '/humanoid_wheel/jointAcc', 
    Float64MultiArray, 
    joint_acc_callback
)

print("开始订阅 /humanoid_wheel/jointAcc 话题...")
print("(WBC求解的关节加速度，包括下肢和上肢)")

# 保持节点运行
rospy.spin()
```



------

#### 19. `/humanoid_wheel/torque` - WBC求解的关节扭矩(包括下肢上肢)

**消息类型：** `std_msgs/Float64MultiArray`

**功能描述：** WBC 模块输出的关节加速度

**字段说明：**

```yaml
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim #一个维度描述数组，每个元素描述一个维度
    string label #维度的名称
    uint32 size #维度的大小
    uint32 stride #步长
  uint32 data_offset #数据在 data数组中的起始偏移量
float64[] data
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def torque_callback(msg):
    """关节扭矩回调函数"""
    data = msg.data
    
    if len(data) != 18:
        print(f"警告: 数据长度不正确，期望18，实际{len(data)}")
        return
    
    # 解析数据: [下肢扭矩(4), 上肢扭矩(14)]
    leg_joint_torque = data[0:4]   # 下肢关节扭矩 - 4个
    arm_joint_torque = data[4:18]  # 上肢关节扭矩 - 14个
    
    print("=" * 60)
    print("WBC关节扭矩 (humanoid_wheel/torque):")
    print("(WBC求解的关节扭矩，包括下肢和上肢)")
    print(f"  下肢关节扭矩 (4个): {[f'{x:.4f}' for x in leg_joint_torque]}")
    print(f"  上肢关节扭矩 (14个): {[f'{x:.4f}' for x in arm_joint_torque]}")

# 初始化ROS节点
rospy.init_node('sub_torque', anonymous=True)

# 创建订阅者
torque_sub = rospy.Subscriber(
    '/humanoid_wheel/torque', 
    Float64MultiArray, 
    torque_callback
)

print("开始订阅 /humanoid_wheel/torque 话题...")
print("(WBC求解的关节扭矩，包括下肢和上肢)")

# 保持节点运行
rospy.spin()
```



------

#### 20. `/humanoid_wheel/eePoses` - 双臂末端的世界坐标系6D位姿

**消息类型：** `std_msgs/Float64MultiArray`

**功能描述：** 双臂末端的6D位姿世界系反馈，位姿格式: [x, y, z, yaw, pitch, roll]

**字段说明：**

```yaml
std_msgs/MultiArrayLayout layout
  std_msgs/MultiArrayDimension[] dim #一个维度描述数组，每个元素描述一个维度
    string label #维度的名称
    uint32 size #维度的大小
    uint32 stride #步长
  uint32 data_offset #数据在 data数组中的起始偏移量
float64[] data
```

使用示例

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def ee_poses_callback(msg):
    """双臂末端6D位姿回调函数"""
    data = msg.data
    
    if len(data) != 12:
        print(f"警告: 数据长度不正确，期望12，实际{len(data)}")
        return
    
    # 解析数据: [左臂位姿(6), 右臂位姿(6)]
    # 每个位姿格式: [x, y, z, yaw, pitch, roll]
    left_arm_pose = data[0:6]   # 左臂位姿 - 6个
    right_arm_pose = data[6:12] # 右臂位姿 - 6个
    
    print("=" * 60)
    print("双臂末端6D位姿 (humanoid_wheel/eePoses):")
    print("(双臂末端的世界坐标系6D位姿反馈)")
    print("-" * 60)
    print("左臂位姿:")
    print(f"  位置 (x, y, z): [{left_arm_pose[0]:.4f}, {left_arm_pose[1]:.4f}, {left_arm_pose[2]:.4f}]")
    print(f"  姿态 (yaw, pitch, roll): [{left_arm_pose[3]:.4f}, {left_arm_pose[4]:.4f}, {left_arm_pose[5]:.4f}]")
    print(f"  姿态 (度): [{left_arm_pose[3]*180/3.14159:.2f}°, {left_arm_pose[4]*180/3.14159:.2f}°, {left_arm_pose[5]*180/3.14159:.2f}°]")
    print("-" * 60)
    print("右臂位姿:")
    print(f"  位置 (x, y, z): [{right_arm_pose[0]:.4f}, {right_arm_pose[1]:.4f}, {right_arm_pose[2]:.4f}]")
    print(f"  姿态 (yaw, pitch, roll): [{right_arm_pose[3]:.4f}, {right_arm_pose[4]:.4f}, {right_arm_pose[5]:.4f}]")
    print(f"  姿态 (度): [{right_arm_pose[3]*180/3.14159:.2f}°, {right_arm_pose[4]*180/3.14159:.2f}°, {right_arm_pose[5]*180/3.14159:.2f}°]")

# 初始化ROS节点
rospy.init_node('sub_ee_poses', anonymous=True)

# 创建订阅者
ee_poses_sub = rospy.Subscriber(
    '/humanoid_wheel/eePoses', 
    Float64MultiArray, 
    ee_poses_callback
)

print("开始订阅 /humanoid_wheel/eePoses 话题...")
print("(双臂末端的世界坐标系6D位姿反馈)")

# 保持节点运行
rospy.spin()
```



------

## ROS 服务接口

### 调用服务 (Called Services)

#### 1. `/enable_lb_arm_quick_mode` - 关节快速模式切换

**服务类型：** `std_srvs/SetBool`

**功能描述：**

启用或禁用手臂快速模式，

1. 启用下肢则下肢电机直接获取lb_leg_traj的指令，不从MPC获取下肢电机指令
2. 启用上肢则手臂电机直接获取kuavo_arm_traj的指令，不从MPC获取手臂电机指令

**请求字段：**

```yaml
bool data # 模式切换类型（true：开启，false：关闭）
```

**响应字段：**

```yaml
bool success    # 操作是否成功
string message  # 状态消息
```

**相关演示脚本：**

- `cmd_arm_joint_test.py`
- `cmd_leg_joint_test.py`

使用示例

```python
#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool

def call_enable_arm_quick_mode(enable):
    """调用手臂快速模式切换服务"""
    try:
        # 等待服务可用
        rospy.loginfo(f"等待服务 /enable_lb_arm_quick_mode 可用...")
        rospy.wait_for_service('/enable_lb_arm_quick_mode', timeout=5.0)
        
        # 创建服务客户端
        service_client = rospy.ServiceProxy('/enable_lb_arm_quick_mode', SetBool)
        
        # 调用服务
        rospy.loginfo(f"调用服务: {'启用' if enable else '禁用'}手臂快速模式")
        response = service_client(enable)
        
        # 处理响应
        if response.success:
            rospy.loginfo(f"✓ 成功{'启用' if enable else '禁用'}手臂快速模式")
            if response.message:
                rospy.loginfo(f"  消息: {response.message}")
            return True
        else:
            rospy.logwarn(f"✗ 服务调用失败")
            if response.message:
                rospy.logwarn(f"  消息: {response.message}")
            return False
            
    except rospy.ROSException as e:
        rospy.logerr(f"服务不可用: {e}")
        return False
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用异常: {e}")
        return False

# 初始化ROS节点
rospy.init_node('call_enable_arm_quick_mode', anonymous=True)

# 等待一下确保节点初始化完成
rospy.sleep(0.5)

# 示例：启用快速模式
print("=" * 60)
print("示例1: 启用手臂快速模式")
print("=" * 60)
call_enable_arm_quick_mode(True)

rospy.sleep(1.0)

# 示例：禁用快速模式
print("\n" + "=" * 60)
print("示例2: 禁用手臂快速模式")
print("=" * 60)
call_enable_arm_quick_mode(False)

print("\n" + "=" * 60)
print("服务调用完成")
print("=" * 60)
```



------

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

使用示例

```python
#!/usr/bin/env python
import rospy
from kuavo_msgs.srv import changeTorsoCtrlMode

# 控制模式定义
MODES = {
    0: "NoControl - 无控制",
    1: "ArmOnly - 仅控制手臂，基座固定",
    2: "BaseOnly - 仅控制基座，手臂固定",
    3: "BaseArm - 同时控制基座和手臂",
    4: "ArmEeOnly - 仅控制手臂末端"
}

def call_mpc_control(control_mode):
    """调用MPC控制模式切换服务"""
    if control_mode not in MODES:
        rospy.logerr(f"无效模式号 {control_mode}，允许值 {list(MODES.keys())}")
        return False
    
    try:
        # 等待服务可用
        rospy.loginfo(f"等待服务 /mobile_manipulator_mpc_control 可用...")
        rospy.wait_for_service('/mobile_manipulator_mpc_control', timeout=5.0)
        
        # 创建服务客户端
        service_client = rospy.ServiceProxy('/mobile_manipulator_mpc_control', changeTorsoCtrlMode)
        
        # 调用服务
        rospy.loginfo(f"调用服务: 切换到模式 {control_mode} - {MODES[control_mode]}")
        response = service_client(control_mode=control_mode)
        
        # 处理响应
        if response.result:
            rospy.loginfo(f"✓ 成功切换到模式 {control_mode}: {MODES[control_mode]}")
            if response.message:
                rospy.loginfo(f"  消息: {response.message}")
            if hasattr(response, 'mode'):
                rospy.loginfo(f"  当前模式: {response.mode}")
            return True
        else:
            rospy.logwarn(f"✗ 服务调用失败")
            if response.message:
                rospy.logwarn(f"  消息: {response.message}")
            return False
            
    except rospy.ROSException as e:
        rospy.logerr(f"服务不可用: {e}")
        return False
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用异常: {e}")
        return False

# 初始化ROS节点
rospy.init_node('call_mpc_control', anonymous=True)

# 等待一下确保节点初始化完成
rospy.sleep(0.5)

# 示例：切换到不同模式
print("=" * 60)
print("示例1: 切换到 ArmOnly 模式 (仅控制手臂)")
print("=" * 60)
call_mpc_control(1)

rospy.sleep(1.0)

print("\n" + "=" * 60)
print("示例2: 切换到 BaseArm 模式 (同时控制基座和手臂)")
print("=" * 60)
call_mpc_control(3)

rospy.sleep(1.0)

print("\n" + "=" * 60)
print("示例3: 切换到 BaseOnly 模式 (仅控制基座)")
print("=" * 60)
call_mpc_control(2)

print("\n" + "=" * 60)
print("服务调用完成")
print("=" * 60)
print("\n可用模式列表:")
for mode_id, mode_desc in MODES.items():
    print(f"  {mode_id}: {mode_desc}")
```
