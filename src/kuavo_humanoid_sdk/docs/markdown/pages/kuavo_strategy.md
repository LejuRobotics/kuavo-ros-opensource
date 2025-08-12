<a id="kuavo-strategy"></a>

#### WARNING
在运行任何代码示例之前，请确保已经启动机器人， 否则 SDK 无法正常工作：

- 如果是命令行启动，则请确保类似下面的命令已经执行:
  : - 仿真模式: `roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch` (示例命令)
    - 真实机器人: `roslaunch humanoid_controllers load_kuavo_real.launch` (示例命令)
- 如果是 h12 遥控器等启动方式，也请确保已经让机器人启动(站立)

# 策略模块

## 原子技能层级

### *class* kuavo_humanoid_sdk.KuavoRobot

Bases: `RobotBase`

#### arm_fk(q: list) → Tuple[[KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)]

机器人手臂的正运动学求解

* **Parameters:**
  **q** (*list*) – 关节位置列表,单位弧度。
* **Returns:**
  左手臂和右手臂的位姿元组,
  : 如果正运动学失败则返回(None, None)。
* **Return type:**
  Tuple[[KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)]

#### WARNING
此函数需要使用 [`KuavoSDK.Options.WithIK`](api_reference.md#kuavo_humanoid_sdk.KuavoSDK.Options.WithIK) 选项初始化SDK。

#### arm_ik(left_pose: [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), right_pose: [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), left_elbow_pos_xyz: list = [0.0, 0.0, 0.0], right_elbow_pos_xyz: list = [0.0, 0.0, 0.0], arm_q0: list | None = None, params: [KuavoIKParams](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoIKParams) | None = None) → list

机器人手臂逆向运动学求解

* **Parameters:**
  * **left_pose** ([*KuavoPose*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – 左手臂目标姿态,包含xyz位置和四元数方向
  * **right_pose** ([*KuavoPose*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – 右手臂目标姿态,包含xyz位置和四元数方向
  * **left_elbow_pos_xyz** (*list*) – 左肘部位置。如果为[0.0, 0.0, 0.0],则忽略
  * **right_elbow_pos_xyz** (*list*) – 右肘部位置。如果为[0.0, 0.0, 0.0],则忽略
  * **arm_q0** (*list* *,* *optional*) – 初始关节位置,单位为弧度。如果为None,则忽略
  * **params** ([*KuavoIKParams*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoIKParams) *,* *optional*) – 

    逆向运动学参数。如果为None,则忽略，包含:
    - major_optimality_tol: 主要最优性容差
    - major_feasibility_tol: 主要可行性容差
    - minor_feasibility_tol: 次要可行性容差
    - major_iterations_limit: 主要迭代次数限制
    - oritation_constraint_tol: 方向约束容差
    - pos_constraint_tol: 位置约束容差,当pos_cost_weight==0.0时生效
    - pos_cost_weight: 位置代价权重。设为0.0可获得高精度
* **Returns:**
  关节位置列表,单位为弧度。如果计算失败返回None
* **Return type:**
  list

#### WARNING
此函数需要在初始化SDK时设置 [`KuavoSDK.Options.WithIK`](api_reference.md#kuavo_humanoid_sdk.KuavoSDK.Options.WithIK) 选项。

#### arm_reset() → bool

手臂归位

* **Returns:**
  如果手臂归位成功返回True,否则返回False。
* **Return type:**
  bool

#### change_motor_param(motor_param: list) → Tuple[bool, str]

更改电机参数

* **Parameters:**
  **motor_param** (*list*) – [`kuavo_humanoid_sdk.interfaces.data_types.KuavoMotorParam`](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoMotorParam) 对象列表,包含:
  - Kp (float): 位置控制比例增益
  - Kd (float): 速度控制微分增益
  - id (int): 电机ID
* **Returns:**
  成功标志和消息的元组
* **Return type:**
  Tuple[bool, str]

#### control_arm_joint_positions(joint_positions: list) → bool

通过关节位置角度控制手臂

* **Parameters:**
  **joint_positions** (*list*) – 手臂的目标关节位置,单位弧度。
* **Returns:**
  如果手臂控制成功返回True,否则返回False。
* **Return type:**
  bool
* **Raises:**
  * **ValueError** – 如果关节位置列表长度不正确。
  * **ValueError** – 如果关节位置超出[-π, π]范围。
  * **RuntimeError** – 如果在尝试控制手臂时机器人不在stance状态。

#### control_arm_joint_trajectory(times: list, q_frames: list) → bool

控制机器人手臂的目标轨迹。

* **Parameters:**
  * **times** (*list*) – 时间间隔列表,单位秒。
  * **q_frames** (*list*) – 关节位置列表,单位弧度。
* **Returns:**
  如果控制成功返回True,否则返回False。
* **Return type:**
  bool
* **Raises:**
  * **ValueError** – 如果times列表长度不正确。
  * **ValueError** – 如果关节位置列表长度不正确。
  * **ValueError** – 如果关节位置超出[-π, π]范围。
  * **RuntimeError** – 如果在尝试控制手臂时机器人不在stance状态。

#### WARNING
异步接口，函数在发送命令后立即返回，用户需要自行等待运动完成。

#### control_command_pose(target_pose_x: float, target_pose_y: float, target_pose_z: float, target_pose_yaw: float) → bool

在base_link坐标系下控制机器人姿态。

* **Parameters:**
  * **target_pose_x** (*float*) – 目标x位置,单位米。
  * **target_pose_y** (*float*) – 目标y位置,单位米。
  * **target_pose_z** (*float*) – 目标z位置,单位米。
  * **target_pose_yaw** (*float*) – 目标偏航角,单位弧度。
* **Returns:**
  如果命令发送成功返回True,否则返回False。
* **Return type:**
  bool
* **Raises:**
  **RuntimeError** – 如果在尝试控制姿态时机器人不在stance状态。

#### NOTE
此命令会将机器人状态改变为’command_pose’。

#### control_command_pose_world(target_pose_x: float, target_pose_y: float, target_pose_z: float, target_pose_yaw: float) → bool

在odom(世界)坐标系下控制机器人姿态。

* **Parameters:**
  * **target_pose_x** (*float*) – 目标x位置,单位米。
  * **target_pose_y** (*float*) – 目标y位置,单位米。
  * **target_pose_z** (*float*) – 目标z位置,单位米。
  * **target_pose_yaw** (*float*) – 目标偏航角,单位弧度。
* **Returns:**
  如果命令发送成功返回True,否则返回False。
* **Return type:**
  bool
* **Raises:**
  **RuntimeError** – 如果在尝试控制姿态时机器人不在stance状态。

#### NOTE
此命令会将机器人状态改变为’command_pose_world’。

#### control_head(yaw: float, pitch: float) → bool

控制机器人的头部。

* **Parameters:**
  * **yaw** (*float*) – 头部的偏航角,单位弧度,范围[-1.396, 1.396](-80到80度)。
  * **pitch** (*float*) – 头部的俯仰角,单位弧度,范围[-0.436, 0.436](-25到25度)。
* **Returns:**
  如果头部控制成功返回True,否则返回False。
* **Return type:**
  bool

#### control_robot_end_effector_pose(left_pose: [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), right_pose: [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), frame: [KuavoManipulationMpcFrame](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)) → bool

通过手臂末端执行器的位姿控制机器人手臂

* **Parameters:**
  * **left_pose** ([*KuavoPose*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – 左手臂的位姿,包含xyz和四元数。
  * **right_pose** ([*KuavoPose*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – 右手臂的位姿,包含xyz和四元数。
  * **frame** ([*KuavoManipulationMpcFrame*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)) – 手臂的坐标系。
* **Returns:**
  如果控制成功返回True,否则返回False。
* **Return type:**
  bool

#### disable_head_tracking() → bool

禁用头部跟踪。

#### enable_head_tracking(target_id: int) → bool

启用头部跟踪 April Tag

#### get_motor_param() → Tuple[bool, list]

获取电机参数

* **Returns:**
  成功标志和 [`kuavo_humanoid_sdk.interfaces.data_types.KuavoMotorParam`](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoMotorParam) 对象列表的元组
* **Return type:**
  Tuple[bool, list]

#### jump()

使机器人跳跃。

#### WARNING
此函数暂未实现，无法调用

#### manipulation_mpc_reset() → bool

重置机器人手臂。

* **Returns:**
  如果手臂重置成功返回True,否则返回False。
* **Return type:**
  bool

#### set_auto_swing_arm_mode() → bool

机器人手臂自动摆动。

* **Returns:**
  如果切换手臂自动摆动模式成功返回True,否则返回False。
* **Return type:**
  bool

#### set_external_control_arm_mode() → bool

切换手臂控制模式到外部控制模式。

* **Returns:**
  如果切换手臂控制模式到外部控制模式成功返回True,否则返回False。
* **Return type:**
  bool

#### set_fixed_arm_mode() → bool

固定/冻结机器人手臂。

* **Returns:**
  如果手臂固定/冻结成功返回True,否则返回False。
* **Return type:**
  bool

#### set_manipulation_mpc_control_flow(control_flow: [KuavoManipulationMpcControlFlow](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcControlFlow)) → bool

设置 Manipulation MPC 控制流。
:returns: 如果 Manipulation MPC 控制流设置成功返回True,否则返回False。
:rtype: bool

#### set_manipulation_mpc_frame(frame: [KuavoManipulationMpcFrame](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)) → bool

设置 Manipulation MPC 坐标系。
:returns: 如果 Manipulation MPC 坐标系设置成功返回True,否则返回False。
:rtype: bool

#### set_manipulation_mpc_mode(ctrl_mode: [KuavoManipulationMpcCtrlMode](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcCtrlMode)) → bool

设置 Manipulation MPC 模式。
:returns: 如果 Manipulation MPC 模式设置成功返回True,否则返回False。
:rtype: bool

#### squat(height: float, pitch: float = 0.0) → bool

控制机器人的蹲姿高度和俯仰角。

* **Parameters:**
  * **height** (*float*) – 相对于正常站立高度的高度偏移量,单位米,范围[-0.35, 0.0],负值表示下蹲。
  * **pitch** (*float*) – 机器人躯干的俯仰角,单位弧度,范围[-0.4, 0.4]。
* **Returns:**
  如果蹲姿控制成功返回True,否则返回False。
* **Return type:**
  bool

#### stance() → bool

使机器人进入’stance’站立模式。

* **Returns:**
  如果机器人成功进入站立模式返回 True,否则返回 False。
* **Return type:**
  bool

#### NOTE
你可以调用 [`KuavoRobotState.wait_for_stance()`](api_reference.md#kuavo_humanoid_sdk.KuavoRobotState.wait_for_stance) 来等待机器人进入 stance 模式。

#### step_by_step(target_pose: list, dt: float = 0.4, is_left_first_default: bool = True, collision_check: bool = True) → bool

单步控制机器人运动。

* **Parameters:**
  * **target_pose** (*list*) – 机器人的目标位姿[x, y, z, yaw],单位m,rad。
  * **dt** (*float*) – 每步之间的时间间隔,单位秒。默认为0.4秒。
  * **is_left_first_default** (*bool*) – 是否先迈左脚。默认为True。
  * **collision_check** (*bool*) – 是否进行碰撞检测。默认为True。
* **Returns:**
  如果运动成功返回True,否则返回False。
* **Return type:**
  bool
* **Raises:**
  * **RuntimeError** – 如果在尝试控制步态时机器人不在stance状态。
  * **ValueError** – 如果target_pose长度不为4。

#### NOTE
你可以调用 [`KuavoRobotState.wait_for_step_control()`](api_reference.md#kuavo_humanoid_sdk.KuavoRobotState.wait_for_step_control) 来等待机器人进入step-control模式。
你可以调用 [`KuavoRobotState.wait_for_stance()`](api_reference.md#kuavo_humanoid_sdk.KuavoRobotState.wait_for_stance) 来等待step-control完成。

#### trot() → bool

使机器人进入’trot’踏步模式。

* **Returns:**
  如果机器人成功进入踏步模式返回 True,否则返回 False。
* **Return type:**
  bool

#### NOTE
你可以调用 [`KuavoRobotState.wait_for_walk()`](api_reference.md#kuavo_humanoid_sdk.KuavoRobotState.wait_for_walk) 来等待机器人进入踏步模式。

#### walk(linear_x: float, linear_y: float, angular_z: float) → bool

控制机器人行走运动。

* **Parameters:**
  * **linear_x** (*float*) – x轴方向的线速度,单位m/s,范围[-0.4, 0.4]。
  * **linear_y** (*float*) – y轴方向的线速度,单位m/s,范围[-0.2, 0.2]。
  * **angular_z** (*float*) – 绕z轴的角速度,单位rad/s,范围[-0.4, 0.4]。
* **Returns:**
  如果运动控制成功返回 True,否则返回 False。
* **Return type:**
  bool

#### NOTE
你可以调用 [`KuavoRobotState.wait_for_walk()`](api_reference.md#kuavo_humanoid_sdk.KuavoRobotState.wait_for_walk) 来等待机器人进入行走模式。

### *class* kuavo_humanoid_sdk.kuavo_strategy.KuavoRobotVision(robot_type: str = 'kuavo')

Bases: `object`

Kuavo机器人视觉系统接口。

提供从不同坐标系获取AprilTag检测数据的接口。

#### *property* apriltag_data_from_base *: [AprilTagData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)*

基座坐标系中检测到的所有AprilTag（属性）

* **Type:**
  [AprilTagData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)

#### *property* apriltag_data_from_camera *: [AprilTagData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)*

相机坐标系中检测到的所有AprilTag（属性）

* **Type:**
  [AprilTagData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)

#### *property* apriltag_data_from_odom *: [AprilTagData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)*

里程计坐标系中检测到的所有AprilTag（属性）

* **Type:**
  [AprilTagData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)

#### get_data_by_id(target_id: int, data_source: str = 'base') → dict

获取指定ID的AprilTag检测数据。

* **Parameters:**
  * **target_id** (*int*) – 要检索的AprilTag ID
  * **data_source** (*str* *,* *optional*) – 数据源坐标系。可以是”base”、”camera”或”odom”。默认为”base”。
* **Returns:**
  包含位置、方向和元数据的检测数据
* **Return type:**
  dict

#### get_data_by_id_from_base(target_id: int) → dict

从基座坐标系获取AprilTag数据。

* **Parameters:**
  **target_id** (*int*) – 要检索的AprilTag ID
* **Returns:**
  包含位置、方向和元数据的检测数据。参见 `get_data_by_id()` 的返回格式说明。
* **Return type:**
  dict

#### get_data_by_id_from_camera(target_id: int) → dict

从相机坐标系获取AprilTag数据。

* **Parameters:**
  **target_id** (*int*) – 要检索的AprilTag ID
* **Returns:**
  包含位置、方向和元数据的检测数据。参见 `get_data_by_id()` 的返回格式说明。
* **Return type:**
  dict

#### get_data_by_id_from_odom(target_id: int) → dict

从里程计坐标系获取AprilTag数据。

* **Parameters:**
  **target_id** (*int*) – 要检索的AprilTag ID
* **Returns:**
  包含位置、方向和元数据的检测数据。参见 `get_data_by_id()` 的返回格式说明。
* **Return type:**
  dict

### *class* kuavo_humanoid_sdk.kuavo_strategy.KuavoRobotState(robot_type: str = 'kuavo')

Bases: `object`

#### angular_velocity() → Tuple[float, float, float]

返回 Kuavo 机器人在世界坐标系中的角速度。

* **Returns:**
  角速度 (x, y, z)。
* **Return type:**
  Tuple[float, float, float]

#### arm_control_mode() → [KuavoArmCtrlMode](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoArmCtrlMode)

获取 Kuavo 机器人手臂的当前控制模式。

* **Returns:**
  当前手臂控制模式:
  : * ArmFixed: 0 - 机器人手臂处于固定位置。
    * AutoSwing: 1 - 机器人手臂处于自动摆动模式。
    * ExternalControl: 2 - 机器人手臂由外部控制。
    * None - 机器人手臂处于未知状态。
* **Return type:**
  [KuavoArmCtrlMode](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoArmCtrlMode)

#### arm_joint_state() → [KuavoJointData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)

获取 Kuavo 机器人手臂关节的当前状态。

获取 Kuavo 机器人手臂关节的当前状态，包括:
: - 关节位置(角度)，单位为弧度
  - 关节速度，单位为弧度/秒
  - 关节扭矩/力矩，单位为牛顿米、安培
  - 关节加速度

* **Returns:**
  手臂关节数据包含:
  : * position: list[float] \* arm_dof(14)
    * velocity: list[float] \* arm_dof(14)
    * torque: list[float] \* arm_dof(14)
    * acceleration: list[float] \* arm_dof(14)
* **Return type:**
  [KuavoJointData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)

#### *property* com_height *: float*

获取机器人质心高度。

* **Returns:**
  机器人质心高度，单位为米。
* **Return type:**
  float

#### eef_state() → Tuple[[EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

获取机器人末端执行器的当前状态。

* **Returns:**
  包含左右末端执行器状态的元组。
  : 每个EndEffectorState包含:
    : - position: (float, float, float) ，XYZ位置，单位为米
      - orientation: (float, float, float, float) ，四元数方向
      - state: EndEffectorState.GraspingState ，当前抓取状态 (UNKNOWN, OPEN, CLOSED)
* **Return type:**
  Tuple[[EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

#### gait_name() → str

获取机器人的当前步态名称。

* **Returns:**
  当前步态的名称，例如 ‘trot’、’walk’、’stance’、’custom_gait’。
* **Return type:**
  str

#### head_joint_state() → [KuavoJointData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)

获取机器人头部关节的当前状态。

获取机器人头部关节的当前状态数据，包括位置、速度、扭矩和加速度值。

* **Returns:**
  包含头部关节状态的数据结构:
  : * position (list[float]): 关节位置，单位为弧度，长度=head_dof(2)
    * velocity (list[float]): 关节速度，单位为rad/s，长度=head_dof(2)
    * torque (list[float]): 关节扭矩，单位为Nm，长度=head_dof(2)
    * acceleration (list[float]): 关节加速度，单位为rad/s^2，长度=head_dof(2)

  关节顺序为 [偏航角, 俯仰角]。
* **Return type:**
  [KuavoJointData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)

#### *property* imu_data *: [KuavoImuData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoImuData)*

获取 Kuavo 机器人IMU数据。

获取机器人当前的 IMU 传感器数据，包括陀螺仪、加速度计、自由加速度和方向四元数测量值。

* **Returns:**
  IMU数据包含:
  : * gyro (`tuple` of `float`): 陀螺仪测量值 (x, y, z)，单位rad/s
    * acc (`tuple` of `float`): 加速度计测量值 (x, y, z)，单位m/s^2
    * free_acc (`tuple` of `float`): 自由加速度 (x, y, z)，单位m/s^2
    * quat (`tuple` of `float`): 方向四元数 (x, y, z, w)
* **Return type:**
  `KuavoImuData`

#### is_stance() → bool

检查机器人当前是否处于站立模式。

* **Returns:**
  如果机器人处于站立模式返回True，否则返回False。
* **Return type:**
  bool

#### is_step_control() → bool

检查机器人当前是否处于单步控制模式。

* **Returns:**
  如果机器人处于单步控制模式返回True，否则返回False。
* **Return type:**
  bool

#### is_walk() → bool

检查机器人当前是否处于行走模式。

* **Returns:**
  如果机器人处于行走模式返回True，否则返回False。
* **Return type:**
  bool

#### *property* joint_state *: [KuavoJointData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)*

获取 Kuavo 机器人关节数据。

获取机器人关节数据，包括关节位置、速度、扭矩和加速度。

数据包括:
: - 关节位置(角度)，单位为弧度
  - 关节速度，单位为弧度/秒
  - 关节扭矩/力矩，单位为牛顿米、安培
  - 关节加速度

* **Returns:**
  包含以下关节状态数据的字典:
  : * position (list[float]): 关节位置，长度 = joint_dof(28)
    * velocity (list[float]): 关节速度，长度 = joint_dof(28)
    * torque (list[float]): 关节扭矩，长度 = joint_dof(28)
    * acceleration (list[float]): 关节加速度，长度 = joint_dof(28)
* **Return type:**
  [KuavoJointData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)

#### linear_velocity() → Tuple[float, float, float]

返回 Kuavo 机器人在世界坐标系中的线速度。

* **Returns:**
  线速度 (x, y, z)，单位为m/s。
* **Return type:**
  Tuple[float, float, float]

#### manipulation_mpc_control_flow() → [KuavoManipulationMpcControlFlow](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcControlFlow)

获取 Kuavo 机器人 Manipulation MPC 的当前控制流。

* **Returns:**
  当前 Manipulation MPC 控制流。
* **Return type:**
  [KuavoManipulationMpcControlFlow](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcControlFlow)

#### manipulation_mpc_ctrl_mode() → [KuavoManipulationMpcCtrlMode](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcCtrlMode)

获取 Kuavo 机器人 Manipulation MPC 的当前控制模式。

* **Returns:**
  当前 Manipulation MPC 控制模式。
* **Return type:**
  [KuavoManipulationMpcCtrlMode](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcCtrlMode)

#### manipulation_mpc_frame() → [KuavoManipulationMpcFrame](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)

获取机器人操作MPC的当前帧。

* **Returns:**
  末端执行器 Manipulation MPC 坐标系
* **Return type:**
  [KuavoManipulationMpcFrame](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)

#### *property* odometry *: [KuavoOdometry](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoOdometry)*

获取 Kuavo 机器人里程计数据。

获取机器人当前的里程计数据，包括位置、方向、线速度和角速度测量值。

* **Returns:**
  包含以下里程计数据的字典:
  : * position (tuple): 位置 (x, y, z)，单位为米
    * orientation (tuple): 方向四元数 (x, y, z, w)
    * linear (tuple): 线速度 (x, y, z)，单位为m/s
    * angular (tuple): 角速度 (x, y, z)，单位为rad/s
* **Return type:**
  [KuavoOdometry](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoOdometry)

#### robot_orientation() → Tuple[float, float, float, float]

返回 Kuavo 机器人在世界坐标系中的方向。

* **Returns:**
  方向四元数 (x, y, z, w)。
* **Return type:**
  Tuple[float, float, float, float]

#### robot_position() → Tuple[float, float, float]

返回 Kuavo 机器人在世界坐标系中的位置。

* **Returns:**
  位置 (x, y, z)，单位为米。
* **Return type:**
  Tuple[float, float, float]

#### wait_for_stance(timeout: float = 5.0) → bool

等待机器人进入站立模式。

* **Parameters:**
  **timeout** (*float*) – 等待机器人进入站立状态的最长时间，单位为秒。
* **Returns:**
  如果机器人在指定超时时间内进入站立状态返回True，否则返回False。
* **Return type:**
  bool

#### wait_for_step_control(timeout: float = 5.0) → bool

等待机器人进入单步控制模式。

* **Parameters:**
  **timeout** (*float*) – 等待机器人进入单步控制模式的最长时间，单位为秒。
* **Returns:**
  如果机器人在指定超时时间内进入单步控制模式返回True，否则返回False。
* **Return type:**
  bool

#### wait_for_trot(timeout: float = 5.0) → bool

等待机器人进入踏步状态。

* **Parameters:**
  **timeout** (*float*) – 等待机器人进入踏步状态的最长时间，单位为秒。
* **Returns:**
  如果机器人在指定超时时间内进入踏步状态返回True，否则返回False。
* **Return type:**
  bool

#### wait_for_walk(timeout: float = 5.0) → bool

等待机器人进入行走模式。

* **Parameters:**
  **timeout** (*float*) – 等待机器人进入行走状态的最长时间，单位为秒。
* **Returns:**
  如果机器人在指定超时时间内进入行走状态返回True，否则返回False。
* **Return type:**
  bool

### *class* kuavo_humanoid_sdk.kuavo_strategy.KuavoRobotTools

Bases: `object`

机器人工具类,提供坐标系转换接口。

该类封装了不同机器人坐标系之间的坐标变换查询功能,支持多种返回数据格式。

#### get_base_to_odom(return_type: str = 'pose_quaternion') → [PoseQuaternion](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion) | [HomogeneousMatrix](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix) | None

获取从base_link到odom坐标系的变换。

* **Parameters:**
  **return_type** (*str* *,* *optional*) – 返回格式类型。与get_tf_transform相同，默认为”pose_quaternion”。
* **Returns:**
  变换数据或 None
* **Return type:**
  Union[[PoseQuaternion](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion), [HomogeneousMatrix](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix), None]

#### get_camera_to_base(return_type: str = 'homogeneous') → [PoseQuaternion](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion) | [HomogeneousMatrix](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix) | None

获取从camera_link到base_link坐标系的变换。

* **Parameters:**
  **return_type** (*str* *,* *optional*) – 返回格式类型。与get_tf_transform相同，默认为”homogeneous”。
* **Returns:**
  变换数据或None
* **Return type:**
  Union[[PoseQuaternion](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion), [HomogeneousMatrix](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix), None]

#### get_link_position(link_name: str, reference_frame: str = 'base_link') → Tuple[float, float, float] | None

获取指定机械臂关节链接的位置

* **Parameters:**
  * **link_name** (*str*) – 关节链接名称，如”zarm_l1_link”
  * **reference_frame** (*str*) – 参考坐标系，默认为base_link
* **Returns:**
  三维位置坐标(x,y,z)，失败返回None
* **Return type:**
  Tuple[float, float, float] | None

#### get_tf_transform(target_frame: str, source_frame: str, return_type: str = 'pose_quaternion') → [PoseQuaternion](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion) | [HomogeneousMatrix](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix) | None

获取指定坐标系之间的变换。

* **Parameters:**
  * **target_frame** (*str*) – 目标坐标系名称
  * **source_frame** (*str*) – 源坐标系名称
  * **return_type** (*str* *,* *optional*) – 

    返回数据格式类型。有效值:
    - ”pose_quaternion” : 四元数姿态格式,
    - ”homogeneous” : 齐次矩阵格式。默认为”pose_quaternion”。
* **Returns:**
  指定格式的变换数据,如果失败则返回None
* **Return type:**
  Union[[PoseQuaternion](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion), [HomogeneousMatrix](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix), None]
* **Raises:**
  **ValueError** – 如果提供了无效的 return_type

## 基础策略接口

### *class* kuavo_humanoid_sdk.kuavo_strategy.kuavo_strategy.KuavoRobotStrategyBase(robot: [KuavoRobot](api_reference.md#kuavo_humanoid_sdk.KuavoRobot), robot_state: [KuavoRobotState](api_reference.md#kuavo_humanoid_sdk.KuavoRobotState), robot_tools: [KuavoRobotTools](api_reference.md#kuavo_humanoid_sdk.KuavoRobotTools), robot_vision: [KuavoRobotVision](api_reference.md#kuavo_humanoid_sdk.KuavoRobotVision))

Bases: `ABC`

Kuavo机器人策略基础类，提供策略执行的抽象接口

#### *abstract* arm_move_to_target(target_info, \*\*kwargs)

手臂移动到特定的位置

* **Parameters:**
  * **target_info** – 目标信息，包含位置、姿态等
  * **arm_mode** – 手臂控制模式
  * **\*\*kwargs** – 其他参数
* **Returns:**
  是否成功移动到目标位置
* **Return type:**
  bool

#### *abstract* arm_transport_target_down(target_info, \*\*kwargs)

放下操作对象

* **Parameters:**
  * **target_info** – 目标信息，包含位置、姿态等
  * **arm_mode** – 手臂控制模式
  * **\*\*kwargs** – 其他参数
* **Returns:**
  是否成功放下操作对象
* **Return type:**
  bool

#### *abstract* arm_transport_target_up(target_info, \*\*kwargs)

提起操作对象

* **Parameters:**
  * **target_info** – 目标信息，包含位置、姿态等
  * **arm_mode** – 手臂控制模式
  * **\*\*kwargs** – 其他参数
* **Returns:**
  是否成功提起操作对象
* **Return type:**
  bool

#### *abstract* head_find_target(target_info, \*\*kwargs)

寻找特定ID的目标

* **Parameters:**
  * **target_id** – 目标的ID标识
  * **\*\*kwargs** – 其他参数
* **Returns:**
  是否成功找到目标
* **Return type:**
  bool

#### *abstract* walk_approach_target(target_info, target_distance=0.5, \*\*kwargs)

走/接近特定的目标到指定距离

* **Parameters:**
  * **target_id** – 目标的ID标识
  * **target_distance** – 与目标的期望距离(米)
  * **\*\*kwargs** – 其他参数
* **Returns:**
  是否成功接近目标
* **Return type:**
  bool

#### *abstract* walk_to_pose(target_info, target_distance=0.5, \*\*kwargs)

走到指定距离的目标位置

* **Parameters:**
  * **target_info** – 目标信息，包含位置、姿态等
  * **target_distance** – 与目标的期望距离(米)
  * **\*\*kwargs** – 其他参数
* **Returns:**
  是否成功接近目标
* **Return type:**
  bool

## 箱子抓取策略

### 箱子信息数据结构

### *class* kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy.BoxInfo(pose: [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), size: Tuple[float, float, float] = (0.3, 0.2, 0.15), mass: float = 1.0)

Bases: `object`

箱子信息数据类

描述箱子的位置、尺寸和质量信息，用于箱子抓取策略

#### pose

箱子的位姿信息

* **Type:**
  [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)

#### size

箱子的尺寸 (长, 宽, 高) 单位: 米

* **Type:**
  Tuple[float, float, float]

#### mass

箱子的质量 单位: 千克

* **Type:**
  float

#### mass *: float* *= 1.0*

#### pose *: [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)*

#### size *: Tuple[float, float, float]* *= (0.3, 0.2, 0.15)*

### 箱子抓取策略类

### *class* kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy.KuavoGraspBox(robot: [KuavoRobot](api_reference.md#kuavo_humanoid_sdk.KuavoRobot), robot_state: [KuavoRobotState](api_reference.md#kuavo_humanoid_sdk.KuavoRobotState), robot_tools: [KuavoRobotTools](api_reference.md#kuavo_humanoid_sdk.KuavoRobotTools), robot_vision: [KuavoRobotVision](api_reference.md#kuavo_humanoid_sdk.KuavoRobotVision))

Bases: [`KuavoRobotStrategyBase`](#kuavo_humanoid_sdk.kuavo_strategy.kuavo_strategy.KuavoRobotStrategyBase)

箱子抓取策略类，继承自基础策略类

#### arm_move_to_target(target_info: BoxInfo, arm_mode='manipulation_mpc', \*\*kwargs)

添加安全保护检查

#### arm_transport_target_down(target_info: BoxInfo, arm_mode='manipulation_mpc')

添加安全检查

#### arm_transport_target_up(target_info: BoxInfo, arm_mode='manipulation_mpc')

添加安全检查

#### head_find_target(target_info: [AprilTagData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData), max_search_time=None, search_pattern='rotate_head', \*\*kwargs)

使用头部旋转寻找AprilTag目标

* **Parameters:**
  * **target_info** – AprilTag的信息
  * **max_search_time** – 最大搜索时间(秒)，如果为None则使用默认值
  * **search_pattern** – 搜索模式，”rotate_head”或”rotate_body”
* **Returns:**
  是否成功找到目标
* **Return type:**
  bool

logic:
: 1. 判断目标位置是否在机器人FOV(70度视场角)内
  2. 如果不在FOV内且search_pattern为”rotate_body”，先旋转机器人身体朝向目标位置
  3. 无论如何都使用头部搜索模式尝试找到目标
  4. 找到apriltag_data_from_odom之后，马上开始头部追踪

#### walk_approach_target(target_info: [AprilTagData](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData), target_distance=0.5, approach_speed=0.15, \*\*kwargs)

走路接近AprilTag目标

* **Parameters:**
  * **target_info** – AprilTag的信息
  * **target_distance** – 与目标的期望距离(米)
  * **approach_speed** – 接近速度(米/秒)
* **Returns:**
  是否成功接近目标
* **Return type:**
  bool

#### walk_to_pose(target_info: BoxInfo, target_distance=0.5, approach_speed=0.15, \*\*kwargs)

让机器人走到指定的位姿

* **Parameters:**
  * **target_pose** – 目标位姿
  * **target_distance** – 与目标的期望距离(米)
  * **approach_speed** – 接近速度(米/秒)
* **Returns:**
  是否成功到达目标位姿
* **Return type:**
  bool

## 搬箱子示例

以下是一个使用箱子抓取策略的基本示例:

### gazebo 仿真运行

#### 准备

第一次启动 gazebo 场景前需要修改tag尺寸:

在 `/opt/ros/noetic/share/apriltag_ros/config/tags.yaml` 文件中将 tag 的 size 尺寸修改为和立方体 tag 码的尺寸一致（只需做一次）

```yaml
standalone_tags:
  [
    {id: 0, size: 0.088, name: 'tag_0'},
    {id: 1, size: 0.088, name: 'tag_1'},
    {id: 2, size: 0.088, name: 'tag_2'},
    {id: 3, size: 0.088, name: 'tag_3'},
    {id: 4, size: 0.088, name: 'tag_4'},
    {id: 5, size: 0.088, name: 'tag_5'},
    {id: 6, size: 0.088, name: 'tag_6'},
    {id: 7, size: 0.088, name: 'tag_7'},
    {id: 8, size: 0.088, name: 'tag_8'},
    {id: 9, size: 0.088, name: 'tag_9'}
  ]
```

#### 编译

首先需要编译相关功能包:

```bash
catkin build humanoid_controllers kuavo_msgs gazebo_sim ar_control
```

#### 运行

#### WARNING
在运行之前, 需要确认机器人版本 `ROBOT_VERSION=45` ，否则会机器人末端控制会有问题

启动仿真环境:

```bash
source devel/setup.bash

# 启动gazebo场景
roslaunch humanoid_controllers load_kuavo_gazebo_manipulate.launch joystick_type:=bt2pro

# 启动ar_tag转换码操作和virtual操作
roslaunch ar_control robot_strategies.launch
```

#### NOTE
每次启动gazebo场景后需要手动打光, 需要在机器人腰部位置附近给个点光源, 否则会找不到 tag, 如下图所示:

![image](../docs/images/gazebo.jpg)

### 实物运行

#### 准备

上位机需要先修改 `./src/ros_vision/detection_apriltag/apriltag_ros/config/tags.yaml` 文件, 将 tag 的 size 尺寸修改为实际大小，比如 0.1 米

```yaml
standalone_tags:
    [
        {id: 0, size: 0.1, name: 'tag_0'},
        {id: 1, size: 0.1, name: 'tag_1'},
        {id: 2, size: 0.1, name: 'tag_2'},
        {id: 3, size: 0.1, name: 'tag_3'},
        {id: 4, size: 0.1, name: 'tag_4'},
        {id: 5, size: 0.1, name: 'tag_5'},
        {id: 6, size: 0.1, name: 'tag_6'},
        {id: 7, size: 0.1, name: 'tag_7'},
        {id: 8, size: 0.1, name: 'tag_8'},
        {id: 9, size: 0.1, name: 'tag_9'}
    ]
```

#### 编译

首先需要编译相关功能包:

```bash
catkin build humanoid_controllers grab_box
```

#### 运行

#### 下位机运行

```bash
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=bt2pro
```

#### 上位机运行

```bash
cd ~/kuavo_ros_application
source devel/setup.bash
roslaunch dynamic_biped apriltag.launch
```

### 示例代码

```python
#!/usr/bin/env python3
import time
import numpy as np
from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, AprilTagData, PoseQuaternion
from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox, BoxInfo

def main():
    print("初始化机器人...")
    # 初始化机器人及相关组件
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    robot_tools = KuavoRobotTools()
    robot_vision = KuavoRobotVision()
    
    # 初始化箱子抓取策略
    grasp_strategy = KuavoGraspBox(robot, robot_state, robot_tools, robot_vision)
    
    # 创建AprilTag数据对象
    target_april_tag = AprilTagData(
        id=[2],  # AprilTag ID
        size=[0.088],  # AprilTag 标签尺寸
        pose=[PoseQuaternion(
            position=(0.0, -1.0, 0.8),  # 位置
            orientation=(0.0, 0.0, 0.0, 1.0)  # 四元数方向
        )]
    )
    
    # 创建箱子信息对象
    box_info = BoxInfo(
        pose=KuavoPose(
            position=(0.0, -1.0, 0.9),  # 箱子位置
            orientation=(0.0, 0.0, -0.7071, 0.7071)  # 箱子朝向
        ),
        size=(0.3, 0.2, 0.15),  # 箱子尺寸(长、宽、高)，单位：米
        mass=1.0  # 箱子重量，单位：千克
    )
    
    # 创建放置位置的箱子信息
    placement_box_info = BoxInfo(
        pose=KuavoPose(
            position=(0.0, 1.0, 0.8),  # 目标放置位置坐标
            orientation=(0.0, 0.0, 0.7071, 0.7071)  # 目标放置方向（四元数）
        ),
        size=(0.3, 0.2, 0.15),  # 使用相同尺寸
        mass=1.0  # 使用相同质量
    )
    
    time.sleep(1)
    # 执行完整抓取策略
    try:
        print("========== 开始执行箱子抓取策略 ==========")
        
        # 步骤1：使用头部寻找目标
        print("\n1. 寻找目标箱子...")
        grasp_strategy.robot.disable_head_tracking()
        print("✅ 已关闭头部追踪")
        
        find_success = grasp_strategy.head_find_target(
            target_april_tag, 
            max_search_time=15.0,
            search_pattern="rotate_body" #  rotate_head
        )
        
        if not find_success:
            print("❌ 寻找目标失败，无法继续执行")
            return
        
        print("✅ 已找到目标箱子")
        time.sleep(1)  # 短暂暂停
        
        # 步骤2：走路接近目标
        # for i in range(1):
        print("\n2. 走路接近目标...")
        approach_success = grasp_strategy.walk_approach_target(
            target_april_tag,
            target_distance=0.7,  # 与目标箱子保持0.7米的距离
            approach_speed=0.2    # 接近速度0.2米/秒
        )
        
        if not approach_success:
            print("❌ 接近目标失败，无法继续执行")
            return
            
        print("✅ 已成功接近目标")
        time.sleep(1)  # 短暂暂停
        
        # 步骤3：手臂移动到抓取位置
        print("\n3. 手臂移动到抓取位置...")
        move_success = grasp_strategy.arm_move_to_target(
            box_info,
            arm_mode="manipulation_mpc"
        )
        
        if not move_success:
            print("❌ 手臂移动失败，无法继续执行")
            return
            
        print("✅ 手臂已到达抓取位置")
        # grasp_strategy.robot.arm_reset()
        time.sleep(1.0)  # 短暂暂停
        
        
        # 步骤4：提起箱子
        print("\n4. 提起箱子...")
        transport_up_success = grasp_strategy.arm_transport_target_up(
            box_info,
            arm_mode="manipulation_mpc"
        )
        
        if not transport_up_success:
            print("❌ 提起箱子失败")
            return
        time.sleep(1.0)  # 短暂暂停
            
        print("✅ 成功提起箱子")
        # grasp_strategy.robot.arm_reset()
        time.sleep(1.0)  # 展示一下成功提起的状态


        # 步骤5：关闭头部追踪
        print("\n5. 关闭头部追踪...")
        grasp_strategy.robot.disable_head_tracking()
        print("✅ 已关闭头部追踪")
        time.sleep(1.0)  # 短暂暂停

        # 步骤6：移动到放置位置
        print("\n6. 移动到放置位置...")
        move_success = grasp_strategy.walk_to_pose(
            placement_box_info,
            target_distance=0.6,
            approach_speed=0.2,
        )
        if not move_success:
            print("❌ 移动到放置位置失败")
            return
        time.sleep(1.0)  # 短暂暂停

        # 步骤7：放下箱子
        print("\n7. 放下箱子...")
        transport_down_success = grasp_strategy.arm_transport_target_down(
            placement_box_info,
            arm_mode="manipulation_mpc"
        )
        
        if not transport_down_success:
            print("❌ 放下箱子失败")
            return
            
        print("✅ 成功放下箱子")
        time.sleep(1.0)  # 短暂暂停        
        print("\n========== 箱子抓取任务完成 ==========")
        
    except Exception as e:
        print(f"执行过程中发生错误: {e}")
    finally:
        # 确保机器人回到安全状态
        print("将机器人恢复到安全姿态...")
        # 这里可以添加使机器人回到默认姿态的代码

if __name__ == "__main__":
    main() 
```
