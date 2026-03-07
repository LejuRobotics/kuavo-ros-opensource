## 键盘控制机器人运动 参数说明

  - 关于启动`load_kuavo_mujoco_sim.launch`及`load_kuavo_real.launch`时，参数`joystick_type`的选择说明

### 对于仿真
  - **参数的选择**：`load_kuavo_mujoco_sim.launch` 文件启动时参数 `joystick_type` 可选择 `h12` `bt2` `bt2pro` `sim` 分别对应h12遥控器，bt2遥控器，bt2pro遥控器和键盘控制。
    - 这里默认参数为`sim`，如果直接运行会弹出新终端用来从键盘输入运动控制指令。
    - 如果启动该launch文件时`joystick_type`参数选择了`h12`或`bt2`，可以新开一个终端，依次执行`cd kuavo-ros-control`，`sudo su`,`source devel/setup.bash`,`python3 src/kuavo_sdk/scripts/keyboard_control/robot_keyboard_control.py`，在此终端中也可以用键盘输入运动控制指令，此时遥控器和键盘均奏效。
    - 如果启动该launch文件时`joystick_type`参数选择了`bt2pro`，就不能使用键盘控制，因为json配置文件中按键映射不一致会导致键盘控制实失效，只能用遥控器控制。

### 对于实机
  - **参数的选择**： `load_kuavo_real.launch` 文件启动时参数 `joystick_type` 可选择 `h12` `bt2` `bt2pro` `sim` 分别对应h12遥控器，bt2遥控器，bt2pro遥控器和键盘控制。
    - 这里默认参数为`bt2`，如果启动该launch文件时`joystick_type`参数选择了`h12`或`bt2`，可以新开一个终端，依次执行`cd kuavo-ros-control`，`sudo su`,`source devel/setup.bash`,`python3 src/kuavo_sdk/scripts/keyboard_control/robot_keyboard_control.py`，在此终端中也可以用键盘输入运动控制指令，此时遥控器和键盘均奏效。
    - 如果启动launch文件时`joystick_type`参数选择了`sim`，会弹出新终端用来从键盘输入运动控制指令，非可视化界面不会弹出。
    - 如果启动该launch文件时`joystick_type`参数选择了`bt2pro`，就不能使用键盘控制，因为json配置文件中按键映射不一致会导致键盘控制实失效，只能用遥控器控制。
  - **特别**：
    - 当`joystick_type`参数不是`sim`时，不论使用上述哪种方式启动键盘控制，都不要在机器人身上插着北通手柄的接收器。
    - 当`joystick_type`参数是`sim`时，需要确保没有接入北通手柄且 H12 的服务处于关闭状态，关闭 H12 服务的方式为 `systemctl stop ocs2_h12pro_monitor.service`。
    - 使用 H12 时，下位机不可以连接北通接收器；使用北通手柄时，需要将 H12 的服务关闭。

---

## 键盘控制机器人手臂移动 程序说明
本文档分析文件：`src/kuavo_sdk/scripts/keyboard_control/arm_keyboard_control.py`。

## 1. 程序目标

该脚本是一个 ROS 键盘遥操作节点，用于：
- 按键调整单侧手臂末端位置/姿态目标。
- 调用 IK 服务将末端目标转为双臂 14 关节角（只更新当前控制侧，另一侧保持当前值）。
- 发布关节轨迹目标到手臂控制话题。

## 2. 启动与版本分支（以 `my_robot_version=52` 为例）

### 2.1 入口流程

1. `main` 读取参数 `robot_version`（`get_version_parameter()`）。
2. 校验版本系列是否在 `[42,45,47,49,52]`。
3. 创建 `KeyBoardArmController(...)` 并执行 `run()`。

关键点：
- 代码按 `version % 100000` 判断系列，因此类似 `100052` 也会走 52 分支。

### 2.2 v52 专用参数

当版本为 52 时，构造函数写入如下机器人几何和索引参数：
- `robot_zero_x = -0.003`
- `robot_zero_y = -0.2527`
- `robot_zero_z = -0.3144`
- `robot_upper_arm = 0.2837`
- `robot_lower_arm = 0.4251`
- `robot_shoulder_height = 0.3944`
- `joint_data_header, joint_data_footer = 13, 27`

这些参数的实际工程含义如下：
- 坐标系基准是机器人躯干坐标系 `base_link`，为右手系：机器人前方为 `+X`，左侧为 `+Y`，上方为 `+Z`。
- `robot_zero_x / robot_zero_y / robot_zero_z`：当手臂关节角全零、手臂自然下垂时，右臂末端执行器在 `base_link` 下的位置。
- `robot_upper_arm / robot_lower_arm`：以肘关节为分界的大臂和小臂长度，用于后续把手臂近似成二连杆来估计末端自然姿态（主要用于自适应 pitch 计算）。
- `robot_shoulder_height`：肩部坐标系相对躯干坐标系在 `Z` 方向的高度差（肩高）。
- `robot_zero_y` 也等价于肩宽相关参数（在该脚本的几何计算里承担肩部横向偏置量）。
- `joint_data_header, joint_data_footer = 13, 27`：用于从 ROS 话题中截取双臂关节数据的标志位；即读取 `joint_q[13:27]` 这 14 个值，分别对应双臂 14 个关节（7 左 + 7 右）。

在代码中的作用：
- 上述几何参数主要参与“按位置键后自适应重算 yaw/pitch”的几何关系计算。
- `13:27` 同时用于 FK 初始化输入和 IK 初值更新数据来源。

## 3. ROS 接口依赖

### 3.1 服务

- `/ik/fk_srv`（`fkSrv`）
  - 用于初始化：将当前 14 关节角转换为左右手末端位姿。
- `/ik/two_arm_hand_pose_cmd_srv`（`twoArmHandPoseCmdSrv`）
  - 用于每次按键后的 IK 求解。
- `/arm_traj_change_mode`（`changeArmCtrlMode`）
  - 运行前切换到模式 `2`，退出时切回模式 `1`。

### 3.2 订阅

- `/sensors_data_raw`（`sensorsData`）
  - 获取当前关节角；首帧触发 FK 初始化，后续仅刷新当前控制手。

### 3.3 发布

- `kuavo_arm_target_poses`（`armTargetPoses`）
  - 发布目标轨迹：`times=[time_gap]`，`values=[14关节角(度)]`。

## 4. 核心数据结构与状态

- `self.which_hand`：当前控制手（默认右手）。
- `self.eef_target_xyz` / `self.eef_target_ypr`：当前控制手末端目标（位置+自适应姿态）。
- `self.eef_angle_manual`：手动姿态增量（yaw/pitch/roll）。
- `self.control_rpy_flag`：是否启用手动姿态叠加模式（`g` 切换）。
- `self.current_joint_values`：当前 14 关节角，作为 IK 初值。
- `self._flag_pose_inited`：是否完成首帧 FK 初始化。

## 5. 初始化阶段（首帧 FK）

在首次收到 `/sensors_data_raw` 后：
1. 取 `joint_q[13:27]`（v52）。
2. 调用 FK 获取左右手当前 `pos_xyz + quat_xyzw`。
3. 四元数转欧拉角，填充 `l/r_eef_target_xyz` 和 `l/r_eef_target_ypr`。
4. 默认将“当前控制目标”指向右手（即启动即右手控制）。

这样能保证首次按键时，目标是从“当前真实姿态”增量变化，不会突跳到固定姿态。

## 6. 键盘交互逻辑

## 6.1 位置键

- `w/s`：X 正/负
- `a/d`：Y 负/正
- `q/e`：Z 正/负

每次位置变化后，会自动重算姿态：
- `yaw`：基于目标点与肩部零点的几何关系（左右手公式不同）。
- `pitch`：由 `eff_orientation_pitch()` 根据二连杆几何估计。
- `roll`：固定置 0。

### 6.1.1 自适应 pitch 计算（`eff_orientation_pitch`）

该函数把手臂在一个平面内近似为二连杆（大臂 `L1`、小臂 `L2`），根据目标点求“自然跟随”的末端 pitch。实现流程如下：

1. 计算目标点到肩部平面坐标  
在调用处先构造：
- `pos_x`：肩部到末端的平面径向距离（由 `x/y` 合成）。
- `pos_y`：末端相对肩高的竖直差（`z - robot_shoulder_height`）。

2. 用余弦定理求肘关节角 `theta2`  
`D = (pos_x^2 + pos_y^2 - L1^2 - L2^2) / (2*L1*L2)`  
然后将 `D` 裁剪到 `[-1, 1]`，避免数值误差导致 `arccos` 非法。  
`theta2 = arccos(D)`，代码里固定取正支（肘上构型）。

3. 求肩关节角 `theta1`  
`theta1 = atan2(pos_y, pos_x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))`

4. 计算连杆末端朝向 `phi`  
`phi = theta1 + theta2`

5. 映射到控制器使用的 pitch 定义  
函数最终返回：`pitch = -(phi + 1.57)`。  
这里的 `+1.57`（约 `pi/2`）是坐标系/末端姿态零位补偿，负号用于对齐该控制链路中的 pitch 正方向约定。

工程上可以理解为：位置键改变末端目标点后，程序用二连杆几何自动给出一个“较自然、不突兀”的俯仰角，减少纯位置控制下末端姿态失配。

## 6.2 姿态键（手动增量）

- `u/o`：roll +/−
- `i/k`：pitch +/−
- `j/l`：yaw +/−

注意：这些键只改 `eef_angle_manual`，只有在 `g` 打开“手动姿态模式”时才参与 IK。

## 6.3 模式/对象切换

- `g`：自动姿态模式 与 手动姿态模式切换。
- `n`：左右手切换。
  - 切换时会保存当前手的目标状态，再加载另一手目标状态。

## 7. IK 求解与发布链路

每次有效按键后执行 `update_response()`：
1. 将“自适应姿态 + 可选手动姿态增量”组合为最终旋转矩阵。
2. 旋转矩阵转四元数。
3. 调用 IK 服务，输入：
   - 左右手目标位姿（当前控制手使用新目标，另一手沿用当前关节对应状态）
   - `joint_angles_as_q0=True`（当前关节角作为初值）
4. IK 成功后得到 14 关节角（弧度）。
5. 转换为角度（degree），发布到 `kuavo_arm_target_poses`。

发布时间参数：
- `times=[time_gap]`，在 `main` 中默认 `time_gap=1.0` 秒。

## 8. 运行控制与退出

`run()` 主循环：
1. 等待 FK 初始化完成。
2. 切控制模式 `set_arm_control_mode(2)`。
3. 循环读键并触发 `update_response()`。
4. `Ctrl-C` 退出后停止线程并切回 `set_arm_control_mode(1)`。
5. `finally` 恢复终端 `termios` 设置。

## 9. 以 v52 看“从按键到动作”的最短路径

1. 按键 `w`（例如）。
2. `eef_target_xyz[0] += 0.03`（`main` 默认步长）。
3. 基于 v52 机械参数重算 `yaw/pitch`。
4. 组合姿态并求 IK。
5. 发布 14 关节角到 `kuavo_arm_target_poses`。
6. 下游控制器按 `time_gap` 执行插补。

## 10. 代码行为特征（阅读时容易忽略）

- 姿态手动键始终可改缓存，但仅在 `g` 打开后生效。
- 每次 IK 都是双臂输入，但只替换当前控制臂，另一侧保持现状。
- 回调里非当前控制臂的 `current_joint_values` 被“冻结”而非实时刷新；并且在 `ik_one_hand` 中，最终发布值由“当前控制臂 IK 解 + 非当前控制臂冻结值”拼接得到。因此在不切换控制臂的连续操作中，`armTargetPoses` 里的非当前控制臂角度是保持不变的，这可避免该侧因反馈噪声进入发布链路而产生抖动或慢漂。
- 脚本内部角度单位混用：IK/FK 用弧度，发布 `armTargetPoses` 前转度。
