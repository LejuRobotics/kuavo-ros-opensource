### 概述
- 新增功能：**从任意关节角度进入增量控制**，且增加了**暂停和恢复增量遥操**的功能，且暂停过程中可用**外部指令控制**相关关节移动，恢复后也可正常映射VR手臂进行增量控制

### 相关配置

- 修改分支：`cy/beta/feat_lb_incremental`

- commit: `1ab6eba1f292f0d140de9cddfc6a090e54d962fb`

### 使用步骤

#### 1. 轮臂 （ROBOT_VERSION = 6x）
**1.1 启动机器**
- 仿真：`roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch`
- 实物：`roslaunch humanoid_controllers load_kuavo_real_wheel.launch`

**1.2 终端使用指令控制折叠臂上升**
```
rostopic pub /lb_leg_traj sensor_msgs/JointState "header:
  stamp: {secs: 0, nsecs: 0}
name: ['joint1', 'joint2', 'joint3', 'joint4']
position: [20.0, -40.0, 20.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0]" -1
```

**1.3 启动VR节点**
```
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_incremental_ik:=true use_incremental_hand_orientation:=false reset_joint_to_default:=false ip_address:=下位机实际有线IP
```

#### 2. 人形 （ROBOT_VERSION = 4x、5x）
**2.1 启动机器**
  - 仿真：`roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch`
  - 实物：`roslaunch humanoid_controllers load_kuavo_real.launch`

**2.2 启动VR节点**
```
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch use_cpp_incremental_ik:=true use_incremental_hand_orientation:=false reset_joint_to_default:=false ip_address:=下位机实际有线IP
```

### 备注
- 轮臂默认启动时，腰部控制默认关闭（不可旋转和升高躯干）。若需开启或关闭相关功能，参照 `docs/轮臂V1.4开发文档/轮臂增量遥操作说明.md` 进行使用即可
- 遥操过程中，按下X+A进入增量遥操后，可按X+B暂停增量遥操，此时手臂和躯干均会保持当前位置不动；再次按下X+A可继续以当前姿态进行遥操
- 若要使用外部指令控制关节状态时，建议在开启VR节点且增量模式已暂停的情况下进行控制，控制完成后可直接通过手柄按键恢复增量控制
- 具体外部指令控制实现需自行设计