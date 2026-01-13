<a id="data-types"></a>

# 数据类型

### *class* kuavo_humanoid_sdk.interfaces.data_types.AprilTagData(id: list, size: list, pose: list)

表示检测到的AprilTag信息及姿态估计

#### id *: list*

检测到的AprilTag ID列表（整数）

#### pose *: list*

表示标签姿态的PoseQuaternion对象列表

#### size *: list*

标签物理尺寸列表，单位为米（浮点数）

### *class* kuavo_humanoid_sdk.interfaces.data_types.AprilTagDetection(position: [Point](#kuavo_humanoid_sdk.interfaces.data_types.AprilTagDetection.Point), orientation: [Quaternion](#kuavo_humanoid_sdk.interfaces.data_types.AprilTagDetection.Quaternion))

表示AprilTag检测结果的数据类

#### *class* Point(x: float, y: float, z: float)

#### x *: float*

#### y *: float*

#### z *: float*

#### *class* Quaternion(x: float, y: float, z: float, w: float)

#### w *: float*

#### x *: float*

#### y *: float*

#### z *: float*

#### orientation *: [Quaternion](#kuavo_humanoid_sdk.interfaces.data_types.AprilTagDetection.Quaternion)*

#### position *: [Point](#kuavo_humanoid_sdk.interfaces.data_types.AprilTagDetection.Point)*

### *class* kuavo_humanoid_sdk.interfaces.data_types.EndEffectorSide(value)

表示末端执行器类型的枚举类。

#### BOTH *= 'both'*

左右末端执行器

#### LEFT *= 'left'*

左末端执行器

#### RIGHT *= 'right'*

右末端执行器

### *class* kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState(position: list, velocity: list, effort: list, state: [GraspingState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState))

表示末端执行器状态的数据类。

参数:
: position (list): 浮点数，末端执行器位置，范围: [0, 100]
  <br/>
  velocity (list): 浮点数，速度
  <br/>
  effort (list): 浮点数，力矩
  <br/>
  state (GraspingState): 夹爪抓取状态

#### *class* GraspingState(value)

表示末端执行器抓取状态的枚举类。

#### ERROR *= -1*

错误状态

#### GRABBED *= 3*

成功抓取物体

#### MOVING *= 1*

正在移动到目标位置

#### REACHED *= 2*

到达目标位置

#### UNKNOWN *= 0*

未知状态

#### effort *: list*

#### position *: list*

#### state *: [GraspingState](#kuavo_humanoid_sdk.interfaces.data_types.EndEffectorState.GraspingState)*

#### velocity *: list*

### *class* kuavo_humanoid_sdk.interfaces.data_types.HomogeneousMatrix(matrix: ndarray)

用于3D变换的4x4齐次变换矩阵。

表示3D空间中的旋转和平移。可用于坐标系变换和姿态合成。

属性:
: matrix (np.ndarray): 形状为(4, 4)的4x4 numpy数组，包含:
  <br/>
  ```default
  [[R, t],
   [0, 1]]
  ```
  <br/>
  其中R是3x3旋转矩阵，t是3x1平移向量

#### matrix *: ndarray*

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoArmCtrlMode(value)

表示Kuavo机器人手臂控制模式的枚举类。

#### ArmFixed *= 0*

手臂固定(冻结)模式

#### AutoSwing *= 1*

自动摆臂模式

#### ExternalControl *= 2*

手臂由外部命令控制

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState(data: Tuple[[KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState)])

表示灵巧手触觉状态的数据类。

#### *class* KuavoTouchState(normal_force1: int, normal_force2: int, normal_force3: int, tangential_force1: int, tangential_force2: int, tangential_force3: int, tangential_direction1: int, tangential_direction2: int, tangential_direction3: int, self_proximity1: int, self_proximity2: int, mutual_proximity: int, status: int)

表示灵巧手触觉状态的数据类

#### mutual_proximity *: int*

互电容接近传感器

#### normal_force1 *: int*

法向力1

#### normal_force2 *: int*

法向力2

#### normal_force3 *: int*

法向力3

#### self_proximity1 *: int*

自电容接近传感器1

#### self_proximity2 *: int*

自电容接近传感器2

#### status *: int*

传感器状态

#### tangential_direction1 *: int*

切向力方向1

#### tangential_direction2 *: int*

切向力方向2

#### tangential_direction3 *: int*

切向力方向3

#### tangential_force1 *: int*

切向力1

#### tangential_force2 *: int*

切向力2

#### tangential_force3 *: int*

切向力3

#### data *: Tuple[[KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState), [KuavoTouchState](#kuavo_humanoid_sdk.interfaces.data_types.KuavoDexHandTouchState.KuavoTouchState)]*

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoIKParams(major_optimality_tol: float = 0.001, major_feasibility_tol: float = 0.001, minor_feasibility_tol: float = 0.001, major_iterations_limit: float = 100, oritation_constraint_tol: float = 0.001, pos_constraint_tol: float = 0.001, pos_cost_weight: float = 0.0)

表示IK节点参数的数据类。

#### major_feasibility_tol *: float* *= 0.001*

#### major_iterations_limit *: float* *= 100*

#### major_optimality_tol *: float* *= 0.001*

#### minor_feasibility_tol *: float* *= 0.001*

#### oritation_constraint_tol *: float* *= 0.001*

#### pos_constraint_tol *: float* *= 0.001*

#### pos_cost_weight *: float* *= 0.0*

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoImuData(gyro: Tuple[float, float, float], acc: Tuple[float, float, float], free_acc: Tuple[float, float, float], quat: Tuple[float, float, float, float])

表示机器人IMU（惯性测量单元）数据的数据类

#### acc *: Tuple[float, float, float]*

x、y、z轴的线性加速度，单位为米/秒²

#### free_acc *: Tuple[float, float, float]*

自由加速度（重力补偿），x、y、z轴，单位为米/秒²

#### gyro *: Tuple[float, float, float]*

绕x、y、z轴的角速度，单位为弧度/秒

#### quat *: Tuple[float, float, float, float]*

方向四元数 (x, y, z, w)

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoJointCommand(joint_q: list, joint_v: list, tau: list, tau_max: list, tau_ratio: list, joint_kp: list, joint_kd: list, control_modes: list)

表示机器人关节命令的数据类

#### control_modes *: list*

每个关节的控制模式整数列表

#### joint_kd *: list*

速度控制微分增益列表

#### joint_kp *: list*

位置控制比例增益列表

#### joint_q *: list*

关节位置命令列表，单位为弧度

#### joint_v *: list*

关节速度命令列表，单位为弧度/秒

#### tau *: list*

关节扭矩/力矩命令列表，单位为牛顿·米或安培

#### tau_max *: list*

每个关节的最大允许扭矩列表

#### tau_ratio *: list*

每个关节的扭矩比率（实际/最大）列表

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoJointData(position: list, velocity: list, torque: list, acceleration: list)

表示机器人关节状态的数据类

#### acceleration *: list*

关节加速度列表，单位为弧度/秒²

#### position *: list*

关节位置（角度）列表，单位为弧度

#### torque *: list*

关节扭矩/力矩列表，单位为牛顿·米或安培

#### velocity *: list*

关节速度列表，单位为弧度/秒

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcControlFlow(value)

表示Kuavo机器人 Manipulation MPC 控制数据流的枚举类

#### DirectToWbc *= 1*

控制数据直接流向WBC，不经过全身MPC

#### Error *= -1*

无效的控制路径

#### ThroughFullBodyMpc *= 0*

控制数据通过全身MPC后进入WBC

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcCtrlMode(value)

表示Kuavo机器人 Manipulation MPC 控制模式的枚举类

#### ArmOnly *= 1*

仅控制手臂

#### BaseArm *= 3*

同时控制底座和手臂

#### BaseOnly *= 2*

仅控制底座

#### ERROR *= -1*

错误状态

#### NoControl *= 0*

无控制

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoManipulationMpcFrame(value)

表示Kuavo机器人末端执行器 Manipulation MPC 坐标系的枚举类

#### ERROR *= -1*

错误状态

#### KeepCurrentFrame *= 0*

保持当前坐标系

#### LocalFrame *= 2*

本地坐标系

#### ManipulationWorldFrame *= 4*

操作世界坐标系

#### VRFrame *= 3*

VR坐标系

#### WorldFrame *= 1*

世界坐标系

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoMotorParam(Kp: float, Kd: float, id: int)

表示机器人电机参数的数据类

#### Kd *: float*

速度控制微分增益

#### Kp *: float*

位置控制比例增益

#### id *: int*

电机ID

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoOdometry(position: Tuple[float, float, float], orientation: Tuple[float, float, float, float], linear: Tuple[float, float, float], angular: Tuple[float, float, float])

表示机器人里程计数据的数据类

#### angular *: Tuple[float, float, float]*

世界坐标系中的角速度 (x, y, z)，单位为弧度/秒

#### linear *: Tuple[float, float, float]*

世界坐标系中的线性速度 (x, y, z)，单位为米/秒

#### orientation *: Tuple[float, float, float, float]*

机器人方向四元数 (x, y, z, w)

#### position *: Tuple[float, float, float]*

机器人在世界坐标系中的位置 (x, y, z)，单位为米

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoPose(position: Tuple[float, float, float], orientation: Tuple[float, float, float, float])

表示机器人姿态的数据类。

#### orientation *: Tuple[float, float, float, float]*

#### position *: Tuple[float, float, float]*

### *class* kuavo_humanoid_sdk.interfaces.data_types.KuavoTwist(linear: Tuple[float, float, float], angular: Tuple[float, float, float])

表示机器人扭转（速度）数据的数据类

#### angular *: Tuple[float, float, float]*

角速度 (x, y, z)，单位为弧度/秒

#### linear *: Tuple[float, float, float]*

线性速度 (x, y, z)，单位为米/秒

### *class* kuavo_humanoid_sdk.interfaces.data_types.PoseQuaternion(position: Tuple[float, float, float], orientation: Tuple[float, float, float, float])

使用位置和四元数方向的3D姿态表示

#### orientation *: Tuple[float, float, float, float]*

单位四元数，采用(x, y, z, w)格式，遵循ROS约定

#### position *: Tuple[float, float, float]*

XYZ坐标，单位为米
