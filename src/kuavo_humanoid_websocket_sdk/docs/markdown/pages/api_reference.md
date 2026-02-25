<a id="api-reference"></a>

#### WARNING
Before running any code examples, make sure to start the robot first by executing either:

- For simulation: `roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch` (Example command)
- For real robot: `roslaunch humanoid_controllers load_kuavo_real.launch` (Example command)

#### NOTE
If using websocket mode, you need to start the rosbridge server on robot first: `roslaunch rosbridge_server rosbridge_websocket.launch` (Example command)

# API Reference

### *class* kuavo_humanoid_sdk.KuavoSDK

Bases: `object`

#### *static* DisableLogging()

Disable all logging output from the SDK.

#### *static* Init(options: int = 1, log_level: str = 'INFO', websocket_mode: bool = False, websocket_host='127.0.0.1', websocket_port=9090, websocket_timeout=5) → bool

Initialize the SDK.

Initializes the Kuavo SDK with specified options and configuration.

* **Parameters:**
  * **options** (*int*) – Configuration options for the SDK. Use KuavoSDK.Options constants.
    Defaults to Options.Normal.
  * **log_level** (*str*) – Logging level to use. Options are “ERROR”, “WARN”, “INFO”, “DEBUG”.
    Defaults to “INFO”.
  * **websocket_mode** (*bool*) – Whether to use websocket mode for ROS communication.
    If True, connects to ROS through websocket instead of direct connection.
    Defaults to False.
  * **websocket_host** (*str*) – Host address of the rosbridge websocket server.
    Only used when websocket_mode is True. Defaults to “127.0.0.1”.
  * **websocket_port** (*int*) – Port number of the rosbridge websocket server.
    Only used when websocket_mode is True. Defaults to 9090.
  * **websocket_timeout** (*int*) – Timeout in seconds for the websocket connection.
    Defaults to 5.
* **Returns:**
  True if initialization succeeds, False otherwise.
* **Return type:**
  bool
* **Raises:**
  **RuntimeError** – If initialization fails due to missing dependencies or
      connection issues.

#### *class* Options

Bases: `object`

#### Normal *= 1*

#### WithIK *= 2*

### *class* kuavo_humanoid_sdk.KuavoRobot

Bases: `RobotBase`

#### arm_fk(q: list) → Tuple[[KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)]

Forward kinematics for the robot arm.

* **Parameters:**
  **q** (*list*) – List of joint positions in radians.
* **Returns:**
  Tuple of poses for the robot left arm and right arm,
  : or (None, None) if forward kinematics failed.
* **Return type:**
  Tuple[[KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)]

#### WARNING
This function requires initializing the SDK with the [`KuavoSDK.Options.WithIK`](#kuavo_humanoid_sdk.KuavoSDK.Options.WithIK).

#### arm_ik(left_pose: [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), right_pose: [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), left_elbow_pos_xyz: list = [0.0, 0.0, 0.0], right_elbow_pos_xyz: list = [0.0, 0.0, 0.0], arm_q0: list | None = None, params: [KuavoIKParams](#kuavo_humanoid_sdk.interfaces.data_types.KuavoIKParams) | None = None) → list

Inverse kinematics for the robot arm.

* **Parameters:**
  * **left_pose** ([*KuavoPose*](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – Pose of the robot left arm, xyz and quat.
  * **right_pose** ([*KuavoPose*](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – Pose of the robot right arm, xyz and quat.
  * **left_elbow_pos_xyz** (*list*) – Position of the robot left elbow. If [0.0, 0.0, 0.0], will be ignored.
  * **right_elbow_pos_xyz** (*list*) – Position of the robot right elbow. If [0.0, 0.0, 0.0], will be ignored.
  * **arm_q0** (*list* *,* *optional*) – Initial joint positions in radians. If None, will be ignored.
  * **params** ([*KuavoIKParams*](#kuavo_humanoid_sdk.interfaces.data_types.KuavoIKParams) *,* *optional*) – Parameters for the inverse kinematics. If None, will be ignored.
    Contains:
    - major_optimality_tol: Major optimality tolerance
    - major_feasibility_tol: Major feasibility tolerance
    - minor_feasibility_tol: Minor feasibility tolerance
    - major_iterations_limit: Major iterations limit
    - oritation_constraint_tol: Orientation constraint tolerance
    - pos_constraint_tol: Position constraint tolerance, works when pos_cost_weight==0.0
    - pos_cost_weight: Position cost weight. Set to 0.0 for high accuracy
* **Returns:**
  List of joint positions in radians, or None if inverse kinematics failed.
* **Return type:**
  list

#### WARNING
This function requires initializing the SDK with the [`KuavoSDK.Options.WithIK`](#kuavo_humanoid_sdk.KuavoSDK.Options.WithIK).

#### arm_reset() → bool

Reset the robot arm.

* **Returns:**
  True if the arm is reset successfully, False otherwise.
* **Return type:**
  bool

#### control_arm_joint_positions(joint_positions: list) → bool

Control the position of the arm.

* **Parameters:**
  **joint_positions** (*list*) – The target position of the arm in radians.
* **Returns:**
  True if the arm is controlled successfully, False otherwise.
* **Return type:**
  bool
* **Raises:**
  * **ValueError** – If the joint position list is not of the correct length.
  * **ValueError** – If the joint position is outside the range of [-π, π].
  * **RuntimeError** – If the robot is not in stance state when trying to control the arm.

#### control_arm_joint_trajectory(times: list, q_frames: list) → bool

Control the target poses of the robot arm.

* **Parameters:**
  * **times** (*list*) – List of time intervals in seconds.
  * **q_frames** (*list*) – List of joint positions in radians.
* **Returns:**
  True if the control was successful, False otherwise.
* **Return type:**
  bool
* **Raises:**
  * **ValueError** – If the times list is not of the correct length.
  * **ValueError** – If the joint position list is not of the correct length.
  * **ValueError** – If the joint position is outside the range of [-π, π].
  * **RuntimeError** – If the robot is not in stance state when trying to control the arm.

#### WARNING
This is an asynchronous interface. The function returns immediately after sending the command.
Users need to wait for the motion to complete on their own.

#### control_command_pose(target_pose_x: float, target_pose_y: float, target_pose_z: float, target_pose_yaw: float) → bool

Control robot pose in base_link frame.

* **Parameters:**
  * **target_pose_x** (*float*) – The target x position in meters.
  * **target_pose_y** (*float*) – The target y position in meters.
  * **target_pose_z** (*float*) – The target z position in meters.
  * **target_pose_yaw** (*float*) – The target yaw angle in radians.
* **Returns:**
  True if the command was sent successfully, False otherwise.
* **Return type:**
  bool
* **Raises:**
  **RuntimeError** – If the robot is not in stance state when trying to control pose.

#### NOTE
This command changes the robot state to ‘command_pose’.

#### control_command_pose_world(target_pose_x: float, target_pose_y: float, target_pose_z: float, target_pose_yaw: float) → bool

Control robot pose in odom (world) frame.

* **Parameters:**
  * **target_pose_x** (*float*) – The target x position in meters.
  * **target_pose_y** (*float*) – The target y position in meters.
  * **target_pose_z** (*float*) – The target z position in meters.
  * **target_pose_yaw** (*float*) – The target yaw angle in radians.
* **Returns:**
  True if the command was sent successfully, False otherwise.
* **Return type:**
  bool
* **Raises:**
  **RuntimeError** – If the robot is not in stance state when trying to control pose.

#### NOTE
This command changes the robot state to ‘command_pose_world’.

#### control_head(yaw: float, pitch: float) → bool

Control the head of the robot.

* **Parameters:**
  * **yaw** (*float*) – The yaw angle of the head in radians, range [-1.396, 1.396] (-80 to 80 degrees).
  * **pitch** (*float*) – The pitch angle of the head in radians, range [-0.436, 0.436] (-25 to 25 degrees).
* **Returns:**
  True if the head is controlled successfully, False otherwise.
* **Return type:**
  bool

#### control_robot_end_effector_pose(left_pose: [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), right_pose: [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), frame: [KuavoManipulationMpcFrame](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)) → bool

Control the end effector pose of the robot arm.

* **Parameters:**
  * **left_pose** ([*KuavoPose*](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – Pose of the robot left arm, xyz and quat.
  * **right_pose** ([*KuavoPose*](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – Pose of the robot right arm, xyz and quat.
  * **frame** ([*KuavoManipulationMpcFrame*](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)) – Frame of the robot arm.
* **Returns:**
  True if the end effector pose is controlled successfully, False otherwise.
* **Return type:**
  bool

#### disable_head_tracking() → bool

Disable the head tracking.

#### enable_head_tracking(target_id: int) → bool

Enable the head tracking.

#### jump()

Jump the robot.

#### manipulation_mpc_reset() → bool

Reset the robot arm.

* **Returns:**
  True if the arm is reset successfully, False otherwise.
* **Return type:**
  bool

#### set_auto_swing_arm_mode() → bool

Swing the robot arm.

* **Returns:**
  True if the arm is swinging successfully, False otherwise.
* **Return type:**
  bool

#### set_external_control_arm_mode() → bool

External control the robot arm.

* **Returns:**
  True if the arm is external controlled successfully, False otherwise.
* **Return type:**
  bool

#### set_fixed_arm_mode() → bool

Freezes the robot arm.

* **Returns:**
  True if the arm is frozen successfully, False otherwise.
* **Return type:**
  bool

#### set_manipulation_mpc_control_flow(control_flow: [KuavoManipulationMpcControlFlow](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcControlFlow)) → bool

Set the manipulation mpc control flow.
:returns: True if the manipulation mpc control flow is set successfully, False otherwise.
:rtype: bool

#### set_manipulation_mpc_frame(frame: [KuavoManipulationMpcFrame](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)) → bool

Set the manipulation mpc frame.
:returns: True if the manipulation mpc frame is set successfully, False otherwise.
:rtype: bool

#### set_manipulation_mpc_mode(ctrl_mode: [KuavoManipulationMpcCtrlMode](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcCtrlMode)) → bool

Set the manipulation mpc mode.
:returns: True if the manipulation mpc mode is set successfully, False otherwise.
:rtype: bool

#### squat(height: float, pitch: float = 0.0) → bool

Control the robot’s squat height and pitch.
:param height: The height offset from normal standing height in meters, range [-0.35, 0.0],Negative values indicate squatting down.
:type height: float
:param pitch: The pitch angle of the robot’s torso in radians, range [-0.4, 0.4].
:type pitch: float

* **Returns:**
  True if the squat is controlled successfully, False otherwise.
* **Return type:**
  bool

#### stance() → bool

Put the robot into ‘stance’ mode.

* **Returns:**
  True if the robot is put into stance mode successfully, False otherwise.
* **Return type:**
  bool

#### NOTE
You can call [`KuavoRobotState.wait_for_stance()`](#kuavo_humanoid_sdk.KuavoRobotState.wait_for_stance) to wait until the robot enters stance mode.

#### step_by_step(target_pose: list, dt: float = 0.4, is_left_first_default: bool = True, collision_check: bool = True) → bool

Control the robot’s motion by step.

* **Parameters:**
  * **target_pose** (*list*) – The target position of the robot in [x, y, z, yaw] m, rad.
  * **dt** (*float*) – The time interval between each step in seconds. Defaults to 0.4s.
  * **is_left_first_default** (*bool*) – Whether to start with the left foot. Defaults to True.
  * **collision_check** (*bool*) – Whether to check for collisions. Defaults to True.
* **Returns:**
  True if motion is successful, False otherwise.
* **Return type:**
  bool
* **Raises:**
  * **RuntimeError** – If the robot is not in stance state when trying to control step motion.
  * **ValueError** – If target_pose length is not 4.

#### NOTE
You can call [`KuavoRobotState.wait_for_step_control()`](#kuavo_humanoid_sdk.KuavoRobotState.wait_for_step_control) to wait until the robot enters step-control mode.
You can call [`KuavoRobotState.wait_for_stance()`](#kuavo_humanoid_sdk.KuavoRobotState.wait_for_stance) to wait the step-control finish.

#### trot() → bool

Put the robot into ‘trot’ mode.

* **Returns:**
  True if the robot is put into trot mode successfully, False otherwise.
* **Return type:**
  bool

#### NOTE
You can call [`KuavoRobotState.wait_for_walk()`](#kuavo_humanoid_sdk.KuavoRobotState.wait_for_walk) to wait until the robot enters trot mode.

#### walk(linear_x: float, linear_y: float, angular_z: float) → bool

Control the robot’s motion.

* **Parameters:**
  * **linear_x** (*float*) – The linear velocity along the x-axis in m/s, range [-0.4, 0.4].
  * **linear_y** (*float*) – The linear velocity along the y-axis in m/s, range [-0.2, 0.2].
  * **angular_z** (*float*) – The angular velocity around the z-axis in rad/s, range [-0.4, 0.4].
* **Returns:**
  True if the motion is controlled successfully, False otherwise.
* **Return type:**
  bool

#### NOTE
You can call [`KuavoRobotState.wait_for_walk()`](#kuavo_humanoid_sdk.KuavoRobotState.wait_for_walk) to wait until the robot enters walk mode.

### *class* kuavo_humanoid_sdk.KuavoRobotInfo(robot_type: str = 'kuavo')

Bases: `RobotInfoBase`

#### *property* arm_joint_dof *: int*

Return the number of joints in the double-arm.

* **Returns:**
  Number of joints in double-arm, e.g. 14
* **Return type:**
  int

#### *property* arm_joint_names *: list*

Return the names of joints in the double-arm.

* **Returns:**
  A list containing the names of joints in the double-arm.
* **Return type:**
  list

#### *property* eef_frame_names *: Tuple[str, str]*

Returns the names of the end effector frames.

* **Returns:**
  A tuple containing the end effector frame names, where:
  - First element is the left hand frame name
  - Second element is the right hand frame name
  e.g. (“zarm_l7_link”, “zarm_r7_link”)
* **Return type:**
  Tuple[str, str]

#### *property* end_effector_type *: str*

Return the type of the end effector.

* **Returns:**
  The end effector type, where:
  : - ”qiangnao” means “dexteroushand”
    - ”lejuclaw” means “lejuclaw”
    - ”qiangnao_touch” means “touchdexteroushand”
    - …
* **Return type:**
  str

#### *property* head_joint_dof *: int*

Return the number of joints in the head.

* **Returns:**
  Number of joints in head, e.g. 2
* **Return type:**
  int

#### *property* head_joint_names *: list*

Return the names of joints in the head.

* **Returns:**
  A list containing the names of joints in the head.
* **Return type:**
  list

#### *property* joint_dof *: int*

Return the total number of joints in the robot.

* **Returns:**
  Total number of joints, e.g. 28
* **Return type:**
  int

#### *property* joint_names *: list*

Return the names of all joints in the robot.

* **Returns:**
  A list containing the names of all robot joints.
* **Return type:**
  list

#### *property* robot_version *: str*

Return the version of the robot.

* **Returns:**
  The robot version, e.g. “42”, “43”…
* **Return type:**
  str

### *class* kuavo_humanoid_sdk.KuavoRobotState(robot_type: str = 'kuavo')

Bases: `object`

#### angular_velocity() → Tuple[float, float, float]

Returns the robot’s angular velocity in world coordinates.

* **Returns:**
  Angular velocity (x, y, z).
* **Return type:**
  Tuple[float, float, float]

#### arm_control_mode() → [KuavoArmCtrlMode](#kuavo_humanoid_sdk.interfaces.data_types.KuavoArmCtrlMode)

Get the current control mode of the robot arm.

* **Returns:**
  Current arm control mode:
  : ArmFixed: 0 - The robot arm is in a fixed position.
    AutoSwing: 1 - The robot arm is in automatic swing mode.
    ExternalControl: 2 - The robot arm is controlled externally.
    or None.
* **Return type:**
  [KuavoArmCtrlMode](#kuavo_humanoid_sdk.interfaces.data_types.KuavoArmCtrlMode)

#### arm_joint_state() → [KuavoJointData](#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)

Get the current state of the robot arm joints.

Get the current state of the robot arm joints, including:
: - Joint positions (angles) in radians
  - Joint velocities in radians/second
  - Joint torques/efforts in Newton-meters, A
  - Joint acceleration

* **Returns:**
  Arm joint data containing:
  : position: list[float] \* arm_dof(14)
    velocity: list[float] \* arm_dof(14)
    torque: list[float]   \* arm_dof(14)
    acceleration: list[float] \* arm_dof(14)
* **Return type:**
  [KuavoJointData](#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)

#### *property* com_height *: float*

Get the height of the robot’s center of mass.

* **Returns:**
  The height of the robot’s center of mass in meters.
* **Return type:**
  float

#### eef_state() → Tuple[[EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

Get the current state of the robot’s end effectors.

* **Returns:**
  A tuple containing the state of the left and right end effectors.
  : Each EndEffectorState contains:
    : - position: (float, float, float) - XYZ position in meters
      - orientation: (float, float, float, float) - Quaternion orientation
      - state: EndEffectorState.GraspingState - Current grasping state (UNKNOWN, OPEN, CLOSED)
* **Return type:**
  Tuple[[EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

#### gait_name() → str

Get the current gait name of the robot.

* **Returns:**
  The name of the current gait, e.g. ‘trot’, ‘walk’, ‘stance’, ‘custom_gait’.
* **Return type:**
  str

#### head_joint_state() → [KuavoJointData](#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)

Get the current state of the robot head joints.

Gets the current state data for the robot’s head joints, including position,
velocity, torque and acceleration values.

* **Returns:**
  A data structure containing the head joint states:
  : position (list[float]): Joint positions in radians, length=head_dof(2)
    velocity (list[float]): Joint velocities in rad/s, length=head_dof(2)
    torque (list[float]): Joint torques in Nm, length=head_dof(2)
    acceleration (list[float]): Joint accelerations in rad/s^2, length=head_dof(2)

  The joint order is [yaw, pitch].
* **Return type:**
  [KuavoJointData](#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)

#### *property* imu_data *: [KuavoImuData](#kuavo_humanoid_sdk.interfaces.data_types.KuavoImuData)*

Get Robot IMU Data.

Gets the current IMU sensor data from the robot, including gyroscope, accelerometer,
free acceleration and orientation quaternion measurements.

* **Returns:**
  IMU data containing:
  : * gyro (`tuple` of `float`): Gyroscope measurements (x, y, z) in rad/s
    * acc (`tuple` of `float`): Accelerometer measurements (x, y, z) in m/s^2
    * free_acc (`tuple` of `float`): Free acceleration (x, y, z) in m/s^2
    * quat (`tuple` of `float`): Orientation quaternion (x, y, z, w)
* **Return type:**
  `KuavoImuData`

#### is_stance() → bool

Check if the robot is currently in stance mode.

* **Returns:**
  True if robot is in stance mode, False otherwise.
* **Return type:**
  bool

#### is_step_control() → bool

Check if the robot is currently in step control mode.

* **Returns:**
  True if robot is in step control mode, False otherwise.
* **Return type:**
  bool

#### is_walk() → bool

Check if the robot is currently in walk mode.

* **Returns:**
  True if robot is in walk mode, False otherwise.
* **Return type:**
  bool

#### *property* joint_state *: [KuavoJointData](#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)*

Get Robot Joint Data.

Get Robot Joint Data, including joint positions, velocities, torques and accelerations.

The data includes:
: - Joint positions (angles) in radians
  - Joint velocities in radians/second
  - Joint torques/efforts in Newton-meters, A
  - Joint acceleration

* **Returns:**
  A dictionary containing joint state data with the following keys:
  : position (list[float]): Joint positions, length = joint_dof(28)
    velocity (list[float]): Joint velocities, length = joint_dof(28)
    torque (list[float]): Joint torques, length = joint_dof(28)
    acceleration (list[float]): Joint accelerations, length = joint_dof(28)
* **Return type:**
  [KuavoJointData](#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData)

#### linear_velocity() → Tuple[float, float, float]

Returns the robot’s linear velocity in world coordinates.

* **Returns:**
  Linear velocity (x, y, z) in m/s.
* **Return type:**
  Tuple[float, float, float]

#### manipulation_mpc_control_flow() → [KuavoManipulationMpcControlFlow](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcControlFlow)

Get the current control flow of the robot manipulation.

* **Returns:**
  Current manipulation control flow.
* **Return type:**
  [KuavoManipulationMpcControlFlow](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcControlFlow)

#### manipulation_mpc_ctrl_mode() → [KuavoManipulationMpcCtrlMode](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcCtrlMode)

Get the current control mode of the robot manipulation MPC.

* **Returns:**
  Current manipulation MPC control mode.
* **Return type:**
  [KuavoManipulationMpcCtrlMode](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcCtrlMode)

#### manipulation_mpc_frame() → [KuavoManipulationMpcFrame](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)

Get the current frame of the robot manipulation MPC.

* **Returns:**
  Current manipulation MPC frame.
* **Return type:**
  [KuavoManipulationMpcFrame](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)

#### *property* odometry *: [KuavoOdometry](#kuavo_humanoid_sdk.interfaces.data_types.KuavoOdometry)*

Get Robot Odometry Data.

Gets the current odometry data from the robot, including position, orientation,
linear velocity and angular velocity measurements.

* **Returns:**
  A dictionary containing odometry data with the following keys:
  : position (tuple): Position (x, y, z) in meters
    orientation (tuple): Orientation as quaternion (x, y, z, w)
    linear (tuple): Linear velocity (x, y, z) in m/s
    angular (tuple): Angular velocity (x, y, z) in rad/s
* **Return type:**
  [KuavoOdometry](#kuavo_humanoid_sdk.interfaces.data_types.KuavoOdometry)

#### robot_orientation() → Tuple[float, float, float, float]

Returns the robot’s orientation in world coordinates.

* **Returns:**
  Orientation as quaternion (x, y, z, w).
* **Return type:**
  Tuple[float, float, float, float]

#### robot_position() → Tuple[float, float, float]

Returns the robot’s position in world coordinates.

* **Returns:**
  Position (x, y, z) in meters.
* **Return type:**
  Tuple[float, float, float]

#### wait_for_stance(timeout: float = 5.0) → bool

Wait for the robot to enter stance state.

* **Parameters:**
  **timeout** (*float*) – The maximum time to wait for the robot to enter stance state in seconds.
* **Returns:**
  True if the robot enters stance state within the specified timeout, False otherwise.
* **Return type:**
  bool

#### wait_for_step_control(timeout: float = 5.0) → bool

Wait for the robot to enter step control state.

* **Parameters:**
  **timeout** (*float*) – The maximum time to wait for the robot to enter step control state in seconds.
* **Returns:**
  True if the robot enters step control state within the specified timeout, False otherwise.
* **Return type:**
  bool

#### wait_for_trot(timeout: float = 5.0) → bool

Wait for the robot to enter trot state.

* **Parameters:**
  **timeout** (*float*) – The maximum time to wait for the robot to enter trot state in seconds.
* **Returns:**
  True if the robot enters trot state within the specified timeout, False otherwise.
* **Return type:**
  bool

#### wait_for_walk(timeout: float = 5.0) → bool

Wait for the robot to enter walk state.

* **Parameters:**
  **timeout** (*float*) – The maximum time to wait for the robot to enter walk state in seconds.
* **Returns:**
  True if the robot enters walk state within the specified timeout, False otherwise.
* **Return type:**
  bool

### *class* kuavo_humanoid_sdk.KuavoRobotArm

Bases: `object`

#### arm_fk(q: list) → Tuple[[KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)]

Forward kinematics for the robot arm.

* **Parameters:**
  **q** (*list*) – List of joint positions in radians.
* **Returns:**
  Tuple of poses for the robot left arm and right arm,
  : or (None, None) if forward kinematics failed.
* **Return type:**
  Tuple[[KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)]

#### WARNING
This function requires initializing the SDK with the [`KuavoSDK.Options.WithIK`](#kuavo_humanoid_sdk.KuavoSDK.Options.WithIK).

#### arm_ik(left_pose: [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), right_pose: [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), left_elbow_pos_xyz: list = [0.0, 0.0, 0.0], right_elbow_pos_xyz: list = [0.0, 0.0, 0.0], arm_q0: list | None = None, params: [KuavoIKParams](#kuavo_humanoid_sdk.interfaces.data_types.KuavoIKParams) | None = None) → list

Inverse kinematics for the robot arm.

* **Parameters:**
  * **left_pose** ([*KuavoPose*](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – Pose of the robot left arm, xyz and quat.
  * **right_pose** ([*KuavoPose*](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose)) – Pose of the robot right arm, xyz and quat.
  * **left_elbow_pos_xyz** (*list*) – Position of the robot left elbow. If [0.0, 0.0, 0.0], will be ignored.
  * **right_elbow_pos_xyz** (*list*) – Position of the robot right elbow. If [0.0, 0.0, 0.0], will be ignored.
  * **arm_q0** (*list* *,* *optional*) – Initial joint positions in radians. If None, will be ignored.
  * **params** ([*KuavoIKParams*](#kuavo_humanoid_sdk.interfaces.data_types.KuavoIKParams) *,* *optional*) – Parameters for the inverse kinematics. If None, will be ignored.
    Contains:
    - major_optimality_tol: Major optimality tolerance
    - major_feasibility_tol: Major feasibility tolerance
    - minor_feasibility_tol: Minor feasibility tolerance
    - major_iterations_limit: Major iterations limit
    - oritation_constraint_tol: Orientation constraint tolerance
    - pos_constraint_tol: Position constraint tolerance, works when pos_cost_weight==0.0
    - pos_cost_weight: Position cost weight. Set to 0.0 for high accuracy
* **Returns:**
  List of joint positions in radians, or None if inverse kinematics failed.
* **Return type:**
  list

#### WARNING
This function requires initializing the SDK with the [`KuavoSDK.Options.WithIK`](#kuavo_humanoid_sdk.KuavoSDK.Options.WithIK).

#### arm_reset() → bool

#### control_arm_joint_positions(joint_position: list) → bool

Control the position of the robot arm joint.
:param joint_position: List of joint positions in radians
:type joint_position: list

* **Raises:**
  * **ValueError** – If the joint position list is not of the correct length.
  * **ValueError** – If the joint position is outside the range of [-π, π].
  * **RuntimeError** – If the robot is not in stance state when trying to control the arm.
* **Returns:**
  True if the control was successful, False otherwise.

#### control_arm_joint_trajectory(times: list, joint_q: list) → bool

Control the target poses of the robot arm.
:param times: List of time intervals in seconds
:type times: list
:param joint_q: List of joint positions in radians
:type joint_q: list

* **Raises:**
  * **ValueError** – If the times list is not of the correct length.
  * **ValueError** – If the joint position list is not of the correct length.
  * **ValueError** – If the joint position is outside the range of [-π, π].
  * **RuntimeError** – If the robot is not in stance state when trying to control the arm.
* **Returns:**
  True if the control was successful, False otherwise.
* **Return type:**
  bool

#### control_robot_end_effector_pose(left_pose: [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), right_pose: [KuavoPose](#kuavo_humanoid_sdk.interfaces.data_types.KuavoPose), frame: [KuavoManipulationMpcFrame](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)) → bool

Control the end effector pose of the robot arm.
:param left_pose: Pose of the robot left arm, xyz and quat.
:type left_pose: KuavoPose
:param right_pose: Pose of the robot right arm, xyz and quat.
:type right_pose: KuavoPose
:param frame: Frame of the robot end effector pose.
:type frame: KuavoManipulationMpcFrame

* **Returns:**
  True if the control was successful, False otherwise.
* **Return type:**
  bool

#### manipulation_mpc_reset() → bool

#### set_auto_swing_arm_mode() → bool

Swing the robot arm.
:returns: True if the arm is swinging successfully, False otherwise.
:rtype: bool

#### set_external_control_arm_mode() → bool

External control the robot arm.
:returns: True if the arm is external controlled successfully, False otherwise.
:rtype: bool

#### set_fixed_arm_mode() → bool

Freezes the robot arm.
:returns: True if the arm is frozen successfully, False otherwise.
:rtype: bool

#### set_manipulation_mpc_control_flow(control_flow: [KuavoManipulationMpcControlFlow](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcControlFlow)) → bool

Set the manipulation mpc control flow.
:returns: True if the manipulation mpc control flow is set successfully, False otherwise.
:rtype: bool

#### set_manipulation_mpc_frame(frame: [KuavoManipulationMpcFrame](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame)) → bool

Set the manipulation mpc frame.
:returns: True if the manipulation mpc frame is set successfully, False otherwise.
:rtype: bool

#### set_manipulation_mpc_mode(ctrl_mode: [KuavoManipulationMpcCtrlMode](#kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcCtrlMode)) → bool

Set the manipulation mpc mode.
:returns: True if the manipulation mpc mode is set successfully, False otherwise.
:rtype: bool

### *class* kuavo_humanoid_sdk.KuavoRobotHead

Bases: `object`

#### control_head(yaw: float, pitch: float) → bool

Control the head of the robot.
:param yaw: The yaw angle of the head in radians, range [-1.396, 1.396] (-80 to 80 degrees).
:type yaw: float
:param pitch: The pitch angle of the head in radians, range [-0.436, 0.436] (-25 to 25 degrees).
:type pitch: float

* **Returns:**
  True if the head is controlled successfully, False otherwise.
* **Return type:**
  bool

#### disable_head_tracking() → bool

Disable the head tracking.

#### enable_head_tracking(target_id: int) → bool

Enable the head tracking.

### *class* kuavo_humanoid_sdk.KuavoRobotVision(robot_type: str = 'kuavo')

Bases: `object`

Vision system interface for Kuavo humanoid robot.

Provides access to AprilTag detection data from different coordinate frames.

#### *property* apriltag_data_from_base *: [AprilTagData](#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)*

All detected AprilTags in camera frame (property)

* **Type:**
  [AprilTagData](#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)

#### *property* apriltag_data_from_camera *: [AprilTagData](#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)*

All detected AprilTags in camera frame (property)

* **Type:**
  [AprilTagData](#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)

#### *property* apriltag_data_from_odom *: [AprilTagData](#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)*

All detected AprilTags in camera frame (property)

* **Type:**
  [AprilTagData](#kuavo_humanoid_sdk.interfaces.data_types.AprilTagData)

#### get_data_by_id(target_id: int, data_source: str = 'base') → dict

Get AprilTag detection data for a specific ID from specified source.

* **Parameters:**
  * **target_id** (*int*) – AprilTag ID to retrieve
  * **data_source** (*str* *,* *optional*) – Data source frame. Can be “base”, “camera”,
    or “odom”. Defaults to “base”.
* **Returns:**
  Detection data containing position, orientation and metadata
* **Return type:**
  dict

#### get_data_by_id_from_base(target_id: int) → dict

Get AprilTag data from base coordinate frame.

* **Parameters:**
  **target_id** (*int*) – AprilTag ID to retrieve
* **Returns:**
  See get_data_by_id() for return format
* **Return type:**
  dict

#### get_data_by_id_from_camera(target_id: int) → dict

Get AprilTag data from camera coordinate frame.

* **Parameters:**
  **target_id** (*int*) – AprilTag ID to retrieve
* **Returns:**
  See get_data_by_id() for return format
* **Return type:**
  dict

#### get_data_by_id_from_odom(target_id: int) → dict

Get AprilTag data from odom coordinate frame.

* **Parameters:**
  **target_id** (*int*) – AprilTag ID to retrieve
* **Returns:**
  See get_data_by_id() for return format
* **Return type:**
  dict

### *class* kuavo_humanoid_sdk.KuavoRobotAudio

Bases: `object`

Audio system interface for controlling audio playback functionality of Kuavo humanoid robot.

Provides functionality to play music files.

#### play_audio(file_name: str, volume: float = 0.5, speed: float = 1.0) → bool

Play the specified audio file.

* **Parameters:**
  **file_name** (*str*) – Name of the audio file to play
* **Returns:**
  True if the play request was successfully sent, False otherwise
* **Return type:**
  bool

#### stop_music()

Stop the currently playing audio.

#### text_to_speech(text: str, volume: float = 0.5) → bool

Synthesize and play the specified text.

* **Parameters:**
  **text** (*str*) – Text to be played
* **Returns:**
  True if the play request was successfully sent, False otherwise
* **Return type:**
  bool

### *class* kuavo_humanoid_sdk.KuavoRobotTools

Bases: `object`

Robot tools class providing coordinate frame transformation interfaces.

This class encapsulates coordinate transformation queries between different robot frames,
supporting multiple return data formats.

#### get_base_to_odom(return_type: str = 'pose_quaternion') → [PoseQuaternion](#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion) | [HomogeneousMatrix](#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix) | None

Get transformation from base_link to odom frame.

* **Parameters:**
  **return_type** (*str* *,* *optional*) – Return format type. Same as get_tf_transform.
  Defaults to “pose_quaternion”.
* **Returns:**
  Transformation data or None
* **Return type:**
  Union[[PoseQuaternion](#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion), [HomogeneousMatrix](#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix), None]

#### get_camera_to_base(return_type: str = 'homogeneous') → [PoseQuaternion](#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion) | [HomogeneousMatrix](#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix) | None

Get transformation from camera_link to base_link frame.

* **Parameters:**
  **return_type** (*str* *,* *optional*) – Return format type. Same as get_tf_transform.
  Defaults to “homogeneous”.
* **Returns:**
  Transformation data or None
* **Return type:**
  Union[[PoseQuaternion](#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion), [HomogeneousMatrix](#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix), None]

#### get_tf_transform(target_frame: str, source_frame: str, return_type: str = 'pose_quaternion') → [PoseQuaternion](#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion) | [HomogeneousMatrix](#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix) | None

Get transformation between specified coordinate frames.

* **Parameters:**
  * **target_frame** (*str*) – Name of target coordinate frame
  * **source_frame** (*str*) – Name of source coordinate frame
  * **return_type** (*str* *,* *optional*) – Return data format type. Valid values:
    “pose_quaternion” - quaternion pose format,
    “homogeneous” - homogeneous matrix format. Defaults to “pose_quaternion”.
* **Returns:**
  Transformation data in specified format, or None if failed
* **Return type:**
  Union[[PoseQuaternion](#kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion), [HomogeneousMatrix](#kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix), None]
* **Raises:**
  **ValueError** – If invalid return_type is provided

### *class* kuavo_humanoid_sdk.DexterousHand

Bases: `EndEffector`

#### control(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

Set the position of the hand.

* **Parameters:**
  * **target_positions** (*list*) – List of target positions for all joints, length must be 12 (6 joints for each hand),
    range => [0.0 ~ 100.0]
  * **target_velocities** (*list* *,* *optional*) – Not supported. Defaults to None.
  * **target_torques** (*list* *,* *optional*) – Not supported. Defaults to None.
* **Returns:**
  True if control successful, False otherwise.
* **Return type:**
  bool

#### NOTE
target_velocities and target_torques are not supported.

#### control_left(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

Control the left dexterous hand.

* **Parameters:**
  * **target_positions** (*list*) – Target positions for left hand joints [0 ~ 100], length must be 6
  * **target_velocities** (*list* *,* *optional*) – Not supported. Defaults to None.
  * **target_torques** (*list* *,* *optional*) – Not supported. Defaults to None.
* **Returns:**
  True if control successful, False otherwise.
* **Return type:**
  bool
* **Raises:**
  **ValueError** – If target positions length doesn’t match joint count or values outside [0,100] range

#### NOTE
target_velocities and target_torques are not supported.

#### control_right(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

Control the right dexterous hand.

* **Parameters:**
  * **target_positions** (*list*) – Target positions for right hand joints [0 ~ 100], length must be 6
  * **target_velocities** (*list* *,* *optional*) – Not supported. Defaults to None.
  * **target_torques** (*list* *,* *optional*) – Not supported. Defaults to None.
* **Returns:**
  True if control successful, False otherwise.
* **Return type:**
  bool
* **Raises:**
  **ValueError** – If target positions length doesn’t match joint count or values outside [0,100] range

#### NOTE
target_velocities and target_torques are not supported.

#### get_effort() → Tuple[list, list]

Get the effort of the dexterous hand.

* **Returns:**
  The effort of the dexterous hand.
* **Return type:**
  Tuple[list, list]

#### NOTE
0 ~ 100 for each finger. Fraction of max motor current, absolute number.
The max motor current is 600mA, in a word, 100.

#### get_gesture_names() → list

Get the names of all gestures.

* **Returns:**
  List of gesture names.
  : e.g.: [‘fist’, ‘ok’, ‘thumbs_up’, ‘666’, ‘number_1’, ‘number_2’, ‘number_3’, … ],
    None if no gestures.
* **Return type:**
  list

#### get_grasping_state() → Tuple[[GraspingState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState), [GraspingState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState)]

Get the grasping state of the dexterous hand.

#### NOTE
The grasping state is not implemented yet.

* **Returns:**
  The grasping state of the dexterous hand.
* **Return type:**
  Tuple[[EndEffectorState.GraspingState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState), [EndEffectorState.GraspingState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState)]

#### get_position() → Tuple[list, list]

Get the position of the dexterous hand.

* **Returns:**
  The position of the dexterous hand.
* **Return type:**
  Tuple[list, list]

#### get_state() → Tuple[[EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

Get the state of the dexterous hand.

* **Returns:**
  The state of the dexterous hand.
* **Return type:**
  Tuple[[EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

#### get_velocity() → Tuple[list, list]

Get the velocity of the dexterous hand.

* **Returns:**
  The velocity of the dexterous hand.
* **Return type:**
  Tuple[list, list]

#### make_gesture(l_gesture_name: str, r_gesture_name: str) → bool

Make predefined gestures for both hands.

* **Parameters:**
  * **l_gesture_name** (*str*) – Name of gesture for left hand. None to skip left hand.
  * **r_gesture_name** (*str*) – Name of gesture for right hand. None to skip right hand.
* **Returns:**
  True if gesture command sent successfully, False otherwise.
* **Return type:**
  bool

#### NOTE
gestures e.g.: ‘fist’, ‘ok’, ‘thumbs_up’, ‘666’…

#### open(side: [EndEffectorSide](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) = EndEffectorSide.BOTH) → bool

Open the dexterous hand(s) by setting all joint positions to 0.

* **Parameters:**
  **side** ([*EndEffectorSide*](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) *,* *optional*) – Which hand(s) to open. Defaults to EndEffectorSide.BOTH.
  Can be LEFT, RIGHT, or BOTH.
* **Returns:**
  True if open command sent successfully, False otherwise.
* **Return type:**
  bool

### *class* kuavo_humanoid_sdk.TouchDexterousHand

Bases: [`DexterousHand`](#kuavo_humanoid_sdk.DexterousHand)

#### get_touch_state() → Tuple[[KuavoDexHandTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState), [KuavoDexHandTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState)]

Get the touch state of the dexterous hand.

* **Returns:**
  Tuple[KuavoDexHandTouchState, KuavoDexHandTouchState]

### *class* kuavo_humanoid_sdk.LejuClaw

Bases: `EndEffector`

#### close(side: [EndEffectorSide](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) = EndEffectorSide.BOTH) → bool

Control the claws to close/grip.

#### NOTE
Control the claws to close.
After calling this function, you can call wait_for_finish() to wait until the claws reach the target position.

* **Parameters:**
  **side** ([*EndEffectorSide*](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) *,* *optional*) – The side to control. Defaults to EndEffectorSide.BOTH.
* **Returns:**
  True if the claw is successfully gripped, False otherwise.
* **Return type:**
  bool

#### control(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

Control the claws to grip.

* **Parameters:**
  * **target_positions** (*list*) – The target positions of the claws.
  * **target_velocities** (*list* *,* *optional*) – The target velocities of the claws. If None, default value [90, 90] will be used.
  * **target_torques** (*list* *,* *optional*) – The target torques of the claws. If None, default value [1.0, 1.0] will be used.

#### NOTE
The target_positions, target_velocities,  target_torques must be a list of length self.joint_count().
After calling this function, you can call wait_for_finish() to wait until the claws reach the target position.

#### WARNING
If the claws are still in motion from a previous command, this request may be dropped.

* **Returns:**
  True if the claws are successfully gripped, False otherwise
* **Return type:**
  bool

#### control_left(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

Control the left claw to grip.

* **Parameters:**
  * **target_positions** (*list*) – The target position of the left claw.
  * **target_velocities** (*list* *,* *optional*) – The target velocity of the left claw. If None, default value 90 will be used.
  * **target_torques** (*list* *,* *optional*) – The target torque of the left claw. If None, default value 1.0 will be used.

#### NOTE
The target_positions, target_velocities, target_torques must be a list of length self.joint_count()/2.
After calling this function, you can call wait_for_finish() to wait until the claws reach the target position.

#### WARNING
If the claws are still in motion from a previous command, this request may be dropped.

* **Returns:**
  True if the claw is successfully gripped, False otherwise.
* **Return type:**
  bool

#### control_right(target_positions: list, target_velocities: list | None = None, target_torques: list | None = None) → bool

Control the right claw to grip.

* **Parameters:**
  * **target_positions** (*list*) – The target position of the right claw.
  * **target_velocities** (*list* *,* *optional*) – The target velocity of the right claw. If None, default value 90 will be used.
  * **target_torques** (*list* *,* *optional*) – The target torque of the right claw. If None, default value 1.0 will be used.
* **Returns:**
  True if the claw is successfully gripped, False otherwise.
* **Return type:**
  bool

#### NOTE
The target_positions, target_velocities, target_torques must be a list of length self.joint_count()/2.
After calling this function, you can call wait_for_finish() to wait until the claws reach the target position.

#### WARNING
If the claws are still in motion from a previous command, this request may be dropped.

#### get_effort() → Tuple[list, list]

Get the effort of the claws.

* **Returns:**
  The effort of the claws.
* **Return type:**
  Tuple[list, list]

#### get_grasping_state() → Tuple[[GraspingState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState), [GraspingState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState)]

Get the grasping state of the claws.

* **Returns:**
  The grasping state of the claws.
* **Return type:**
  Tuple[[EndEffectorState.GraspingState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState), [EndEffectorState.GraspingState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState)]

#### get_position() → Tuple[list, list]

Get the position of the claws.

* **Returns:**
  The position of the claws, range [0.0, 100.0].
* **Return type:**
  Tuple[list, list]

#### get_state() → Tuple[[EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

Get the state of the claws.

* **Returns:**
  The state of the claws.
  : - position: The position of the claws, range [0.0, 100.0].
    - velocity: The velocity of the claws.
    - effort: The effort of the claws.
    - state: The grasping state of the claws.
* **Return type:**
  Tuple[[EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState), [EndEffectorState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState)]

#### get_velocity() → Tuple[list, list]

Get the velocity of the claws.

* **Returns:**
  The velocity of the claws.
* **Return type:**
  Tuple[list, list]

#### open(side: [EndEffectorSide](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) = EndEffectorSide.BOTH) → bool

Control the claws to release/open.

#### NOTE
Control the claws to open.
After calling this function, you can call wait_for_finish() to wait until the claws reach the target position.

* **Parameters:**
  **side** ([*EndEffectorSide*](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) *,* *optional*) – The side to control. Defaults to EndEffectorSide.BOTH.
* **Returns:**
  True if the claw is successfully released, False otherwise.
* **Return type:**
  bool

#### wait_for_finish(side: [EndEffectorSide](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) = EndEffectorSide.BOTH, timeout: float = 2.5)

Wait for the claw motion to finish.

* **Parameters:**
  * **side** ([*EndEffectorSide*](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide) *,* *optional*) – The side of the claw to wait for. Defaults to EndEffectorSide.BOTH.
  * **timeout** (*float* *,* *optional*) – The timeout duration in seconds. Defaults to 2.5.
* **Returns:**
  True if motion completed before timeout, False otherwise.
* **Return type:**
  bool

### *class* kuavo_humanoid_sdk.KuavoRobotObservation(robot_type: str = 'kuavo')

Bases: `object`

Class for accessing robot observation data.

This class provides a high-level interface to access robot observation data,
including joint commands, velocity commands, and pose commands.

#### \_obs_core

The core observation object that handles ROS communication.

#### *property* arm_position_command *: list*

Get the position commands for the arm joints.

* **Returns:**
  Position commands for arm joints (indices 12-25) in radians.
* **Return type:**
  list

#### *property* cmd_pose *: [KuavoTwist](#kuavo_humanoid_sdk.interfaces.data_types.KuavoTwist)*

Get the current pose command.

* **Returns:**
  Object containing linear pose commands (m) and angular pose commands (rad).
* **Return type:**
  [KuavoTwist](#kuavo_humanoid_sdk.interfaces.data_types.KuavoTwist)

#### *property* cmd_vel *: [KuavoTwist](#kuavo_humanoid_sdk.interfaces.data_types.KuavoTwist)*

Get the current velocity command.

* **Returns:**
  Object containing linear velocity (m/s) and angular velocity (rad/s) commands.
* **Return type:**
  [KuavoTwist](#kuavo_humanoid_sdk.interfaces.data_types.KuavoTwist)

#### *property* head_position_command *: list*

Get the position commands for the head joints.

* **Returns:**
  Position commands for head joints (indices 26-27) in radians.
* **Return type:**
  list

#### *property* joint_command *: [KuavoJointCommand](#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointCommand)*

Get the current joint command.

* **Returns:**
  Object containing position, velocity, and torque commands
  : for all robot joints.
* **Return type:**
  [KuavoJointCommand](#kuavo_humanoid_sdk.interfaces.data_types.KuavoJointCommand)

### *class* kuavo_humanoid_sdk.interfaces.data_types.AprilTagData(id: list, size: list, pose: list)

Represents detected AprilTag information with pose estimation.

#### id

List of detected AprilTag IDs (integers)

* **Type:**
  list

#### size

List of tag physical sizes in meters (floats)

* **Type:**
  list

#### pose

List of PoseQuaternion objects representing tag poses

* **Type:**
  list

### *class* kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide(value)

Enum class representing the sides of the end effector.

#### LEFT

The left side of the end effector (value: ‘left’)

#### RIGHT

The right side of the end effector (value: ‘right’)

#### BOTH

Both sides of the end effector (value: ‘both’)

### *class* kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState(position: list, velocity: list, effort: list, state: [GraspingState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState))

Data class representing the state of the end effector.

* **Parameters:**
  * **position** (*list*) – float, Position of the end effector, range: [0, 100]
  * **velocity** (*list*) – float, …
  * **effort** (*list*) – float, …

#### *class* GraspingState(value)

Enum class representing the grasping states of the end effector.

#### ERROR

Error state (value: -1)

#### UNKNOWN

Unknown state (value: 0)

#### REACHED

Target position reached (value: 1)

#### MOVING

Moving to target position (value: 2)

#### GRABBED

Object successfully grasped (value: 3)

### *class* kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix(matrix: ndarray)

4x4 homogeneous transformation matrix for 3D transformations.

Represents both rotation and translation in 3D space. Can be used for
coordinate frame transformations and pose composition.

#### matrix

4x4 numpy array of shape (4, 4) containing:

```default
[[R, t],
 [0, 1]]
```

* **Type:**
  np.ndarray

### where R is 3x3 rotation matrix and t is 3x1 translation

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoArmCtrlMode(value)

Enum class representing the control modes for the Kuavo robot arm.

#### ArmFixed

The robot arm is fixed in position (value: 0)

#### AutoSwing

The robot arm is in automatic swinging mode (value: 1)

#### ExternalControl

The robot arm is controlled by external commands (value: 2)

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState(data: Tuple[[KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState)])

Data class representing the touch state of the dexterous hand.

#### *class* KuavoTouchState(normal_force1: int, normal_force2: int, normal_force3: int, tangential_force1: int, tangential_force2: int, tangential_force3: int, tangential_direction1: int, tangential_direction2: int, tangential_direction3: int, self_proximity1: int, self_proximity2: int, mutual_proximity: int, status: int)

Data class representing the touch state of the dexterous hand.

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoIKParams(major_optimality_tol: float = 0.001, major_feasibility_tol: float = 0.001, minor_feasibility_tol: float = 0.001, major_iterations_limit: float = 100, oritation_constraint_tol: float = 0.001, pos_constraint_tol: float = 0.001, pos_cost_weight: float = 0.0)

Data class representing the parameters for the IK node.

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoImuData(gyro: Tuple[float, float, float], acc: Tuple[float, float, float], free_acc: Tuple[float, float, float], quat: Tuple[float, float, float, float])

Data class representing IMU (Inertial Measurement Unit) data from the robot.

* **Parameters:**
  * **gyro** (*Tuple* *[**float* *,* *float* *,* *float* *]*) – Angular velocity around x, y, z axes in rad/s
  * **acc** (*Tuple* *[**float* *,* *float* *,* *float* *]*) – Linear acceleration in x, y, z axes in m/s^2
  * **free_acc** (*Tuple* *[**float* *,* *float* *,* *float* *]*) – Free acceleration (gravity compensated) in x, y, z axes in m/s^2
  * **quat** (*Tuple* *[**float* *,* *float* *,* *float* *,* *float* *]*) – Orientation quaternion (x, y, z, w)

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoJointCommand(joint_q: list, joint_v: list, tau: list, tau_max: list, tau_ratio: list, joint_kp: list, joint_kd: list, control_modes: list)

Data class representing joint command for the robot.

* **Parameters:**
  * **joint_q** (*list*) – List of joint position commands in radians
  * **joint_v** (*list*) – List of joint velocity commands in radians/second
  * **tau** (*list*) – List of joint torque/effort commands in Newton-meters or Amperes
  * **tau_max** (*list*) – List of maximum allowable torques for each joint
  * **tau_ratio** (*list*) – List of torque ratios (actual/maximum) for each joint
  * **joint_kp** (*list*) – List of position control proportional gains
  * **joint_kd** (*list*) – List of velocity control derivative gains
  * **control_modes** (*list*) – List of control mode integers for each joint

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData(position: list, velocity: list, torque: list, acceleration: list)

Data class representing joint states of the robot.

* **Parameters:**
  * **position** (*list*) – List of joint positions (angles) in radians
  * **velocity** (*list*) – List of joint velocities in radians/second
  * **torque** (*list*) – List of joint torques/efforts in Newton-meters or Amperes
  * **acceleration** (*list*) – List of joint accelerations in radians/second^2

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcControlFlow(value)

Enum class representing the control data flow for the Kuavo robot manipulation.

#### ThroughFullBodyMpc

Control data flows through full-body MPC before entering WBC (value: 0)

#### DirectToWbc

Control data flows directly to WBC without full-body MPC (value: 1)

#### Error

Invalid control path (value: -1)

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcCtrlMode(value)

Enum class representing the control mode for the Kuavo robot manipulation MPC.

#### NoControl

No control (value: 0)

#### ArmOnly

Only control the arm (value: 1)

#### BaseOnly

Only control the base (value: 2)

#### BaseArm

Control both the base and the arm (value: 3)

#### ERROR

Error state (value: -1)

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame(value)

Enum class representing the manipulation mpc frame for the Kuavo robot end effector.

#### KeepCurrentFrame

Keep the current frame (value: 0)

#### WorldFrame

World frame (value: 1)

#### LocalFrame

Local frame (value: 2)

#### VRFrame

VR frame (value: 3)

#### ManipulationWorldFrame

Manipulation world frame (value: 4)

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoOdometry(position: Tuple[float, float, float], orientation: Tuple[float, float, float, float], linear: Tuple[float, float, float], angular: Tuple[float, float, float])

Data class representing odometry data from the robot.

* **Parameters:**
  * **position** (*Tuple* *[**float* *,* *float* *,* *float* *]*) – Robot position (x, y, z) in world coordinates in meters
  * **orientation** (*Tuple* *[**float* *,* *float* *,* *float* *,* *float* *]*) – Robot orientation as quaternion (x, y, z, w)
  * **linear** (*Tuple* *[**float* *,* *float* *,* *float* *]*) – Linear velocity (x, y, z) in world coordinates in m/s
  * **angular** (*Tuple* *[**float* *,* *float* *,* *float* *]*) – Angular velocity (x, y, z) in world coordinates in rad/s

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoPose(position: Tuple[float, float, float], orientation: Tuple[float, float, float, float])

Data class representing the pose of the robot.

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoTwist(linear: Tuple[float, float, float], angular: Tuple[float, float, float])

Data class representing twist (velocity) data for the robot.

* **Parameters:**
  * **linear** (*Tuple* *[**float* *,* *float* *,* *float* *]*) – Linear velocity (x, y, z)
  * **angular** (*Tuple* *[**float* *,* *float* *,* *float* *]*) – Angular velocity (x, y, z)

### *class* kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion(position: Tuple[float, float, float], orientation: Tuple[float, float, float, float])

3D pose representation using position and quaternion orientation.

Provides a singularity-free orientation representation. Commonly used
in robotics for smooth interpolation between orientations.

#### position

XYZ coordinates in meters

* **Type:**
  Tuple[float, float, float]

#### orientation

Unit quaternion in
(x, y, z, w) format following ROS convention

* **Type:**
  Tuple[float, float, float, float]
