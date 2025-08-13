<a id="api-reference"></a>

#### WARNING
在运行任何代码示例之前，请确保已经启动机器人， 否则 SDK 无法正常工作：

- 如果是命令行启动，则请确保类似下面的命令已经执行:
  : - 仿真模式: `roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch` (示例命令)
    - 真实机器人: `roslaunch humanoid_controllers load_kuavo_real.launch` (示例命令)
- 如果是 h12 遥控器等启动方式，也请确保已经让机器人启动(站立)

# API 接口

### *class* kuavo_humanoid_sdk.KuavoSDK

Bases: `object`

#### *static* DisableLogging()

禁用SDK的所有日志输出。

#### *static* Init(options: int = 1, log_level: str = 'INFO') → bool

初始化SDK。

使用指定的选项和配置初始化Kuavo SDK。

* **Parameters:**
  * **options** (*int*) – SDK的配置选项。使用: [`KuavoSDK.Options`](#kuavo_humanoid_sdk.KuavoSDK.Options) 常量，默认为Options.Normal。
  * **log_level** (*str*) – 日志级别。可选值为”ERROR”、”WARN”、”INFO”、”DEBUG”，默认为”INFO”。
* **Returns:**
  初始化成功返回True,否则返回False。
* **Return type:**
  bool
* **Raises:**
  **RuntimeError** – 如果由于缺少依赖项或连接问题导致初始化失败。

#### *class* Options

Bases: `object`

#### Normal *= 1*

#### WithIK *= 2*

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
此函数需要使用 [`KuavoSDK.Options.WithIK`](#kuavo_humanoid_sdk.KuavoSDK.Options.WithIK) 选项初始化SDK。

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
此函数需要在初始化SDK时设置 [`KuavoSDK.Options.WithIK`](#kuavo_humanoid_sdk.KuavoSDK.Options.WithIK) 选项。

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

#### control_hand_wrench(left_wrench: list, right_wrench: list) → bool

控制机器人末端力/力矩

* **Parameters:**
  * **left_wrench** (*list*) – 左手臂6维力控指令 [Fx, Fy, Fz, Tx, Ty, Tz]
  * **right_wrench** (*list*) – 右手臂6维力控指令 [Fx, Fy, Fz, Tx, Ty, Tz]
    单位:
    Fx,Fy,Fz: 牛顿(N)
    Tx,Ty,Tz: 牛·米(N·m)
* **Returns:**
  控制成功返回True, 否则返回False
* **Return type:**
  bool

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

#### enable_base_pitch_limit(enable: bool) → Tuple[bool, str]

开启/关闭机器人 basePitch 限制

#### NOTE
该接口用于关闭或开启机器人 basePitch 保护功能，关闭状态下可以进行比较大幅度的前后倾动作而不会触发保护导致摔倒。

* **Parameters:**
  **enable** (*bool*) – 开启/关闭

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
你可以调用 [`KuavoRobotState.wait_for_stance()`](#kuavo_humanoid_sdk.KuavoRobotState.wait_for_stance) 来等待机器人进入 stance 模式。

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
你可以调用 [`KuavoRobotState.wait_for_step_control()`](#kuavo_humanoid_sdk.KuavoRobotState.wait_for_step_control) 来等待机器人进入step-control模式。
你可以调用 [`KuavoRobotState.wait_for_stance()`](#kuavo_humanoid_sdk.KuavoRobotState.wait_for_stance) 来等待step-control完成。

#### trot() → bool

使机器人进入’trot’踏步模式。

* **Returns:**
  如果机器人成功进入踏步模式返回 True,否则返回 False。
* **Return type:**
  bool

#### NOTE
你可以调用 [`KuavoRobotState.wait_for_walk()`](#kuavo_humanoid_sdk.KuavoRobotState.wait_for_walk) 来等待机器人进入踏步模式。

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
你可以调用 [`KuavoRobotState.wait_for_walk()`](#kuavo_humanoid_sdk.KuavoRobotState.wait_for_walk) 来等待机器人进入行走模式。

### *class* kuavo_humanoid_sdk.KuavoRobotInfo(robot_type: str = 'kuavo')

Bases: `RobotInfoBase`

#### *property* arm_joint_dof *: int*

返回 Kuavo 机器人双臂的关节数。

* **Returns:**
  双臂的关节数，例如 14。
* **Return type:**
  int

#### *property* arm_joint_names *: list*

返回 Kuavo 机器人双臂关节的名称。

* **Returns:**
  包含双臂关节名称的列表。
* **Return type:**
  list

#### *property* eef_frame_names *: Tuple[str, str]*

返回 Kuavo 机器人末端执行器坐标系的名称。

* **Returns:**
  包含末端执行器坐标系名称的元组，其中：
  > - 第一个元素是左手坐标系名称
  > - 第二个元素是右手坐标系名称

  > 例如 (“zarm_l7_link”, “zarm_r7_link”)
* **Return type:**
  Tuple[str, str]

#### *property* end_effector_type *: str*

返回 Kuavo 机器人末端执行器的类型。

* **Returns:**
  末端执行器类型，其中：
  : - `qiangnao` 表示普通灵巧手
    - `lejuclaw` 表示乐聚二指夹爪
    - `qiangnao_touch` 表示触觉灵巧手
    - …
* **Return type:**
  str

#### *property* head_joint_dof *: int*

返回 Kuavo 机器人头部的关节数。

* **Returns:**
  头部的关节数，例如 2。
* **Return type:**
  int

#### *property* head_joint_names *: list*

返回 Kuavo 机器人头部关节的名称。

* **Returns:**
  包含头部关节名称的列表。
* **Return type:**
  list

#### *property* joint_dof *: int*

返回 Kuavo 机器人的总关节数。

* **Returns:**
  总关节数，例如 28。
* **Return type:**
  int

#### *property* joint_names *: list*

返回 Kuavo 机器人所有关节的名称。

* **Returns:**
  包含所有关节名称的列表。
* **Return type:**
  list

#### *property* robot_version *: str*

返回 Kuavo 机器人的版本。

* **Returns:**
  机器人版本号，例如 “42”、”45” 等。
* **Return type:**
  str

### *class* kuavo_humanoid_sdk.KuavoRobotState(robot_type: str = 'kuavo')

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

#### pitch_limit_enabled() → bool

获取机器人 basePitch 限制状态, 如果开启则返回True，否则返回False。

* **Returns:**
  如果机器人 basePitch 限制开启返回True，否则返回False。
* **Return type:**
  bool

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

### *class* kuavo_humanoid_sdk.KuavoRobotArm

Bases: `object`

Kuavo机器人手臂控制类。

提供了控制机器人手臂的各种接口,包括关节位置控制、轨迹控制、末端执行器姿态控制等。

#### arm_fk(q: list) → Tuple[[KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)]

机器人手臂正向运动学求解

* **Parameters:**
  **q** (*list*) – 关节位置列表,单位为弧度
* **Returns:**
  左右手臂姿态的元组,
  : 如果计算失败返回(None, None)
* **Return type:**
  Tuple[[KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)]

#### WARNING
此函数需要在初始化SDK时设置 [`KuavoSDK.Options.WithIK`](#kuavo_humanoid_sdk.KuavoSDK.Options.WithIK) 选项。

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
此函数需要在初始化SDK时设置 [`KuavoSDK.Options.WithIK`](#kuavo_humanoid_sdk.KuavoSDK.Options.WithIK) 选项。

#### arm_reset() → bool

重置机器人手臂。

* **Returns:**
  重置成功返回True,否则返回False。
* **Return type:**
  bool

#### control_arm_joint_positions(joint_position: list) → bool

控制机器人手臂关节位置。

* **Parameters:**
  **joint_position** (*list*) – 关节位置列表,单位为弧度
* **Raises:**
  * **ValueError** – 如果关节位置列表长度不正确
  * **ValueError** – 如果关节位置超出[-π, π]范围
  * **RuntimeError** – 如果在控制手臂时机器人不在站立状态
* **Returns:**
  控制成功返回True,否则返回False
* **Return type:**
  bool

#### control_arm_joint_trajectory(times: list, joint_q: list) → bool

控制机器人手臂关节轨迹。

* **Parameters:**
  * **times** (*list*) – 时间间隔列表,单位为秒
  * **joint_q** (*list*) – 关节位置列表,单位为弧度
* **Raises:**
  * **ValueError** – 如果times列表长度不正确
  * **ValueError** – 如果关节位置列表长度不正确
  * **ValueError** – 如果关节位置超出[-π, π]范围
  * **RuntimeError** – 如果在控制手臂时机器人不在站立状态
* **Returns:**
  控制成功返回True,否则返回False
* **Return type:**
  bool

#### control_hand_wrench(left_wrench: list, right_wrench: list) → bool

控制机器人末端力/力矩

* **Parameters:**
  * **left_wrench** (*list*) – 左手臂6维力控指令 [Fx, Fy, Fz, Tx, Ty, Tz]
  * **right_wrench** (*list*) – 右手臂6维力控指令 [Fx, Fy, Fz, Tx, Ty, Tz]
    单位:
    Fx,Fy,Fz: 牛顿(N)
    Tx,Ty,Tz: 牛·米(N·m)
* **Returns:**
  控制成功返回True, 否则返回False
* **Return type:**
  bool

#### control_robot_end_effector_pose(left_pose: [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), right_pose: [KuavoPose](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), frame: [KuavoManipulationMpcFrame](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)) → bool

控制机器人末端执行器姿态。

* **Parameters:**
  * **left_pose** ([*KuavoPose*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – 左手臂姿态,包含xyz位置和四元数方向
  * **right_pose** ([*KuavoPose*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – 右手臂姿态,包含xyz位置和四元数方向
  * **frame** ([*KuavoManipulationMpcFrame*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)) – 末端执行器姿态的坐标系
* **Returns:**
  控制成功返回True,否则返回False
* **Return type:**
  bool

#### manipulation_mpc_reset() → bool

重置机器人 Manipulation MPC 控制器。

* **Returns:**
  重置成功返回True,否则返回False。
* **Return type:**
  bool

#### set_auto_swing_arm_mode() → bool

设置手臂自动摆动模式。

* **Returns:**
  设置成功返回True,否则返回False
* **Return type:**
  bool

#### set_external_control_arm_mode() → bool

设置手臂外部控制模式。

* **Returns:**
  设置成功返回True,否则返回False
* **Return type:**
  bool

#### set_fixed_arm_mode() → bool

固定/冻结机器人手臂。

* **Returns:**
  固定/冻结成功返回True,否则返回False
* **Return type:**
  bool

#### set_manipulation_mpc_control_flow(control_flow: [KuavoManipulationMpcControlFlow](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcControlFlow)) → bool

设置 Manipulation MPC 控制流。

* **Returns:**
  设置成功返回True,否则返回False
* **Return type:**
  bool

#### set_manipulation_mpc_frame(frame: [KuavoManipulationMpcFrame](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)) → bool

设置 Manipulation MPC 坐标系。

* **Returns:**
  设置成功返回True,否则返回False
* **Return type:**
  bool

#### set_manipulation_mpc_mode(ctrl_mode: [KuavoManipulationMpcCtrlMode](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcCtrlMode)) → bool

设置 Manipulation MPC 控制模式。

* **Returns:**
  设置成功返回True,否则返回False
* **Return type:**
  bool

### *class* kuavo_humanoid_sdk.KuavoRobotHead

Bases: `object`

机器人头部控制类

#### control_head(yaw: float, pitch: float) → bool

控制机器人头部。

* **Parameters:**
  * **yaw** (*float*) – 头部偏航角，单位为弧度，范围 [-1.396, 1.396] (-80度 到 80度)。
  * **pitch** (*float*) – 头部俯仰角，单位为弧度，范围 [-0.436, 0.436] (-25度 到 25度)。
* **Returns:**
  如果控制成功返回True，否则返回False。
* **Return type:**
  bool

#### disable_head_tracking() → bool

禁用头部跟踪功能。

* **Returns:**
  如果禁用成功返回True，否则返回False。
* **Return type:**
  bool

#### enable_head_tracking(target_id: int) → bool

启用头部跟踪功能，在机器人运动过程中，头部将始终追踪指定的 Apriltag ID

* **Parameters:**
  **target_id** (*int*) – 目标ID。
* **Returns:**
  如果启用成功返回True，否则返回False。
* **Return type:**
  bool

### *class* kuavo_humanoid_sdk.KuavoRobotVision(robot_type: str = 'kuavo')

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
  包含位置、方向和元数据的检测数据。参见 [`get_data_by_id()`](#kuavo_humanoid_sdk.KuavoRobotVision.get_data_by_id) 的返回格式说明。
* **Return type:**
  dict

#### get_data_by_id_from_camera(target_id: int) → dict

从相机坐标系获取AprilTag数据。

* **Parameters:**
  **target_id** (*int*) – 要检索的AprilTag ID
* **Returns:**
  包含位置、方向和元数据的检测数据。参见 [`get_data_by_id()`](#kuavo_humanoid_sdk.KuavoRobotVision.get_data_by_id) 的返回格式说明。
* **Return type:**
  dict

#### get_data_by_id_from_odom(target_id: int) → dict

从里程计坐标系获取AprilTag数据。

* **Parameters:**
  **target_id** (*int*) – 要检索的AprilTag ID
* **Returns:**
  包含位置、方向和元数据的检测数据。参见 [`get_data_by_id()`](#kuavo_humanoid_sdk.KuavoRobotVision.get_data_by_id) 的返回格式说明。
* **Return type:**
  dict

### *class* kuavo_humanoid_sdk.KuavoRobotAudio

Bases: `object`

Kuavo 机器人音频系统接口，用于控制音频播放功能。

提供音乐文件播放功能。

#### play_audio(file_name: str, volume: int = 100, speed: float = 1.0) → bool

播放指定的音频文件。

* **Parameters:**
  **file_name** (*str*) – 要播放的音频文件名
* **Returns:**
  如果播放请求成功发送返回True，否则返回False
* **Return type:**
  bool

#### stop_music()

停止当前正在播放的音频。

#### text_to_speech(text: str, volume: float = 0.5) → bool

将指定文本合成并播放。

* **Parameters:**
  **text** (*str*) – 要播放的文本
* **Returns:**
  如果播放请求成功发送返回True，否则返回False
* **Return type:**
  bool

### *class* kuavo_humanoid_sdk.KuavoRobotTools

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

#### get_link_pose(link_name: str, reference_frame: str = 'base_link')

获取指定机械臂关节链接的位置

* **Parameters:**
  * **link_name** (*str*) – 关节链接名称，如”zarm_l1_link”
  * **reference_frame** (*str*) – 参考坐标系，默认为base_link
* **Returns:**
  三维位置坐标(x,y,z)，失败返回None
* **Return type:**
  Tuple[float, float, float] | None

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

### *class* kuavo_humanoid_sdk.DexterousHand

Bases: `EndEffector`

普通灵巧手控制类

#### control(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

控制灵巧手的位置。

* **Parameters:**
  * **target_positions** (*list*) – 所有手指关节的目标位置列表，长度必须为12（每只手6个手指关节），范围 => [0.0 ~ 100.0]
  * **target_velocities** (*list* *,* *optional*) – 不支持。默认为None。
  * **target_torques** (*list* *,* *optional*) – 不支持。默认为None。
* **Returns:**
  如果控制成功返回 True，否则返回 False。
* **Return type:**
  bool

#### NOTE
target_velocities 和 target_torques 参数暂不支持。

#### control_left(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

控制左手灵巧手。

* **Parameters:**
  * **target_positions** (*list*) – 左手关节的目标位置 [0 ~ 100]，长度必须为6
  * **target_velocities** (*list* *,* *optional*) – 不支持。默认为None。
  * **target_torques** (*list* *,* *optional*) – 不支持。默认为None。
* **Returns:**
  如果控制成功返回True，否则返回False。
* **Return type:**
  bool
* **Raises:**
  **ValueError** – 如果目标位置长度与关节数不匹配或值超出[0,100]范围

#### NOTE
target_velocities 和 target_torques 参数不支持。

#### control_right(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

控制右手灵巧手。

* **Parameters:**
  * **target_positions** (*list*) – 右手关节的目标位置 [0 ~ 100]，长度必须为6
  * **target_velocities** (*list* *,* *optional*) – 不支持。默认为None。
  * **target_torques** (*list* *,* *optional*) – 不支持。默认为None。
* **Returns:**
  如果控制成功返回True，否则返回False。
* **Return type:**
  bool
* **Raises:**
  **ValueError** – 如果目标位置长度与关节数不匹配或值超出[0,100]范围

#### NOTE
target_velocities 和 target_torques 参数暂不支持。

#### get_effort() → Tuple[list, list]

获取灵巧手的力。

* **Returns:**
  灵巧手的力。
* **Return type:**
  Tuple[list, list]

#### NOTE
每个手指的范围为0 ~ 100。表示最大电机电流的分数，绝对数值。
最大电机电流为600mA，换句话说，100。

#### get_gesture_names() → list

获取所有手势的名称。

* **Returns:**
  手势名称列表。
  : 例如：[‘fist’, ‘ok’, ‘thumbs_up’, ‘666’, ‘number_1’, ‘number_2’, ‘number_3’, … ], 如果没有手势则返回 None。
* **Return type:**
  list

#### get_grasping_state() → Tuple[[GraspingState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState), [GraspingState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState)]

获取灵巧手的抓取状态。

#### NOTE
该功能尚未实现。

* **Returns:**
  灵巧手的抓取状态。
* **Return type:**
  Tuple[[EndEffectorState.GraspingState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState), [EndEffectorState.GraspingState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState)]

#### get_position() → Tuple[list, list]

获取灵巧手的位置。

* **Returns:**
  灵巧手的位置。
* **Return type:**
  Tuple[list, list]

#### get_state() → Tuple[[EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

获取灵巧手的状态。

* **Returns:**
  灵巧手的状态。
* **Return type:**
  Tuple[[EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

#### get_velocity() → Tuple[list, list]

获取灵巧手的速度。

* **Returns:**
  灵巧手的速度。
* **Return type:**
  Tuple[list, list]

#### make_gesture(l_gesture_name: str, r_gesture_name: str) → bool

为双手做预定义的手势。

* **Parameters:**
  * **l_gesture_name** (*str*) – 左手手势的名称。None表示跳过左手。
  * **r_gesture_name** (*str*) – 右手手势的名称。None表示跳过右手。
* **Returns:**
  如果手势命令发送成功返回True，否则返回False。
* **Return type:**
  bool

#### NOTE
手势示例：’fist’、’ok’、’thumbs_up’、’666’等…

#### open(side: [EndEffectorSide](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) = EndEffectorSide.BOTH) → bool

通过将所有关节位置设置为 0 来张开灵巧手。

* **Parameters:**
  **side** ([*EndEffectorSide*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) *,* *optional*) – 

  要打开的手。默认为 `EndEffectorSide.BOTH`。

  可以是 `EndEffectorSide.LEFT`、`EndEffectorSide.RIGHT` 或 `EndEffectorSide.BOTH`。
* **Returns:**
  如果打开命令发送成功返回True，否则返回False。
* **Return type:**
  bool

### *class* kuavo_humanoid_sdk.TouchDexterousHand

Bases: [`DexterousHand`](#kuavo_humanoid_sdk.DexterousHand)

触觉灵巧手控制类，继承自普通灵巧手控制类，可调用普通灵巧手控制类中的所有方法

#### get_touch_state() → Tuple[[KuavoDexHandTouchState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState), [KuavoDexHandTouchState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState)]

获取灵巧手的触觉状态。

#### WARNING
该功能仅在触觉灵巧手上可用。

* **Returns:**
  Tuple[KuavoDexHandTouchState, KuavoDexHandTouchState]

### *class* kuavo_humanoid_sdk.LejuClaw

Bases: `EndEffector`

#### close(side: [EndEffectorSide](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) = EndEffectorSide.BOTH) → bool

控制二指夹爪闭合/抓取。

#### NOTE
控制二指夹爪闭合

调用此函数后，可以调用 [`LejuClaw.wait_for_finish()`](#kuavo_humanoid_sdk.LejuClaw.wait_for_finish) 等待二指夹爪到达目标位置

* **Parameters:**
  **side** ([*EndEffectorSide*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) *,* *optional*) – 要控制的二指夹爪侧(左/右或左右)，默认为 `EndEffectorState.BOTH`
* **Returns:**
  如果二指夹爪闭合命令发送成功返回 True, 否则返回 False。
* **Return type:**
  bool

#### WARNING
二指夹爪的闭合范围为 [0, 100]，其中 0 表示完全闭合，100 表示完全打开。

#### control(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

控制机器人夹爪抓取。

* **Parameters:**
  * **target_positions** (*list*) – 夹爪的目标位置。
  * **target_velocities** (*list* *,* *optional*) – 夹爪的目标速度。如果为None，将使用默认值[90, 90]。
  * **target_torques** (*list* *,* *optional*) – 夹爪的目标扭矩。如果为None，将使用默认值[1.0, 1.0]。

#### NOTE
target_positions、target_velocities 和 target_torques 必须是长度为\`self.joint_count()\`的列表。

调用此函数后，可以调用 [`LejuClaw.wait_for_finish()`](#kuavo_humanoid_sdk.LejuClaw.wait_for_finish) 等待夹爪到达目标位置。

#### WARNING
如果夹爪仍在执行上一个命令（运动未结束），这个请求可能会被丢弃。

* **Returns:**
  如果夹爪成功发送命令返回True，否则返回False。
* **Return type:**
  bool

#### control_left(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

控制机器人左夹爪抓取。

* **Parameters:**
  * **target_positions** (*list*) – 左夹爪的目标位置。
  * **target_velocities** (*list* *,* *optional*) – 左夹爪的目标速度。如果为None，将使用默认值[90, 90]。
  * **target_torques** (*list* *,* *optional*) – 左夹爪的目标扭矩。如果为None，将使用默认值[1.0, 1.0]。

#### NOTE
target_positions、target_velocities 和 target_torques 必须是长度为\`self.joint_count()/2\`的列表。

调用此函数后，可以调用 [`LejuClaw.wait_for_finish()`](#kuavo_humanoid_sdk.LejuClaw.wait_for_finish) 等待夹爪到达目标位置。

#### WARNING
如果夹爪仍在执行上一个命令（运动未结束），这个请求可能会被丢弃。

* **Returns:**
  如果夹爪成功发送命令返回True，否则返回False。
* **Return type:**
  bool

#### control_right(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

控制机器人右夹爪抓取。

* **Parameters:**
  * **target_positions** (*list*) – 右夹爪的目标位置。
  * **target_velocities** (*list* *,* *optional*) – 右夹爪的目标速度。如果为None，将使用默认值[90, 90]。
  * **target_torques** (*list* *,* *optional*) – 右夹爪的目标扭矩。如果为None，将使用默认值[1.0, 1.0]。
* **Returns:**
  如果夹爪成功发送命令返回True，否则返回False。
* **Return type:**
  bool

#### NOTE
target_positions、target_velocities 和 target_torques 必须是长度为\`self.joint_count()/2\`的列表。

调用此函数后，可以调用 [`LejuClaw.wait_for_finish()`](#kuavo_humanoid_sdk.LejuClaw.wait_for_finish) 等待夹爪到达目标位置。

#### WARNING
If the claws are still in motion from a previous command, this request may be dropped.

#### get_effort() → Tuple[list, list]

获取夹爪的力。

* **Returns:**
  夹爪的力。
* **Return type:**
  Tuple[list, list]

#### get_grasping_state() → Tuple[[GraspingState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState), [GraspingState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState)]

获取夹爪的抓取状态。

* **Returns:**
  夹爪的抓取状态。
* **Return type:**
  Tuple[[EndEffectorState.GraspingState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState), [EndEffectorState.GraspingState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState)]

#### get_position() → Tuple[list, list]

获取夹爪的位置。

* **Returns:**
  夹爪的位置，范围 [0.0, 100.0]。
* **Return type:**
  Tuple[list, list]

#### get_state() → Tuple[[EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

获取夹爪的状态。

* **Returns:**
  夹爪的状态。
  : - position: 夹爪的位置，范围 [0.0, 100.0]。
    - velocity: 夹爪的速度。
    - effort: 夹爪的力。
    - state: 夹爪的抓取状态。
* **Return type:**
  Tuple[[EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

#### get_velocity() → Tuple[list, list]

获取夹爪的速度。

* **Returns:**
  夹爪的速度。
* **Return type:**
  Tuple[list, list]

#### open(side: [EndEffectorSide](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) = EndEffectorSide.BOTH) → bool

控制夹爪释放/打开。

#### NOTE
控制夹爪打开。

调用此函数后，可以调用 [`LejuClaw.wait_for_finish()`](#kuavo_humanoid_sdk.LejuClaw.wait_for_finish) 等待夹爪到达目标位置。

* **Parameters:**
  **side** ([*EndEffectorSide*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) *,* *optional*) – 要控制的夹爪侧(左/右或左右)，默认为 `EndEffectorSide.BOTH` 。
* **Returns:**
  如果夹爪成功发送命令返回True，否则返回False。
* **Return type:**
  bool

#### wait_for_finish(side: [EndEffectorSide](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) = EndEffectorSide.BOTH, timeout: float = 2.5)

等待夹爪运动完成。

* **Parameters:**
  * **side** ([*EndEffectorSide*](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) *,* *optional*) – 要等待的夹爪侧(左/右或左右)，默认为 `EndEffectorSide.BOTH` 。
  * **timeout** (*float* *,* *optional*) – 等待超时时间，默认为 2.5 秒。
* **Returns:**
  如果运动在超时前完成返回True，否则返回False。
* **Return type:**
  bool

### *class* kuavo_humanoid_sdk.KuavoRobotObservation(robot_type: str = 'kuavo')

Bases: `object`

用于访问机器人观测数据的类。

该类提供了一个高级接口来访问机器人的观测数据，包括关节命令、速度命令和姿态命令。

#### *property* arm_position_command *: list*

获取手臂关节的位置控制命令

* **Returns:**
  手臂关节(索引12-25)的位置命令，单位为弧度。
* **Return type:**
  list

#### *property* cmd_pose *: [KuavoTwist](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoTwist)*

获取当前 cmd_pose 姿态控制命令

* **Returns:**
  包含线性姿态命令(m)和角度姿态命令(rad)的对象。
* **Return type:**
  [KuavoTwist](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoTwist)

#### *property* cmd_vel *: [KuavoTwist](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoTwist)*

获取当前 cmd_vel 速度控制命令

* **Returns:**
  包含线速度(m/s)和角速度(rad/s)命令的对象。
* **Return type:**
  [KuavoTwist](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoTwist)

#### *property* head_position_command *: list*

获取头部关节的位置控制命令

* **Returns:**
  头部关节(索引26-27)的位置命令，单位为弧度。
* **Return type:**
  list

#### *property* joint_command *: [KuavoJointCommand](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointCommand)*

获取当前关节控制命令

* **Returns:**
  包含所有机器人关节的位置、速度和力矩命令的对象。
* **Return type:**
  [KuavoJointCommand](data_types.md#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointCommand)
