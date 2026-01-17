# VR Rosbag 回放工具使用说明

## 概述

`vr_rosbag_playback.py` 是一个专门用于回放VR录制的rosbag文件的交互式工具。该工具支持在Mujoco仿真环境和机器人真机上回放VR控制的上肢动作，提供了丰富的配置选项和智能的环境检测功能。

## 使用方法

### 1. 启动程序

```bash
cd /root/kuavo_ws/vr_rosbag
python3 vr_rosbag_playback.py
```

或者直接运行（如果已设置执行权限）：

```bash
./vr_rosbag_playback.py
```

### 2. 选择Bag文件

程序会首先询问bag文件的来源：

#### 选项1：从默认目录选择
- **默认路径**：脚本所在目录（当前路径：`/root/kuavo_ws/vr_rosbag/`）
- **智能文件检测**：
  - 如果当前目录下有直接的`.bag`文件，会优先显示这些文件供选择
  - 如果当前目录下有按日期组织的文件夹（格式：`YYYY-MM-DD`），也会显示这些文件夹
  - 如果两者都存在，会询问您选择哪种方式：
    - "当前目录下的bag文件" - 直接选择当前目录中的`.bag`文件
    - "日期文件夹中的bag文件" - 进入日期文件夹选择
- 选择后，显示对应的`.bag`文件列表
- 选择要回放的bag文件

#### 选项2：输入自定义路径
- 输入bag文件的完整路径
- 如果文件不存在，可以重新输入（输入 `q` 退出）
- 支持 `~` 符号表示用户主目录
- 如果文件不是`.bag`格式，会提示确认是否继续

### 3. 查看Bag文件信息

选择文件后，程序会自动解析并显示：
- **文件路径**：bag文件的完整路径
- **录制时长**：bag文件的总时长（秒）
- **消息总数**：所有话题的消息总数
- **文件大小**：bag文件的大小
- **话题列表**：所有话题的详细信息（话题名称、消息数量、消息类型）

### 4. 选择回放目标

#### Mujoco仿真
- 程序会检查Mujoco相关节点和话题：
  - 节点：`/mujoco_simulator`、`/humanoid_mpc_observation`
  - 话题：`/kuavo_arm_traj`、`/control_robot_hand_position`
- 如果未检测到环境，会提示并询问是否继续
- **建议启动命令**：`roslaunch humanoid_controllers load_kuavo_mujoco_sim_with_vr.launch`

#### 机器人真机
- 程序会检查真机控制相关节点和话题：
  - 节点：`/humanoid_mpc_observation`、`/humanoid_quest_control_with_arm`
  - 话题：`/kuavo_arm_traj`、`/control_robot_hand_position`
- **重要**：会要求确认机器人是否处于安全状态
- **建议启动命令**：`roslaunch humanoid_controllers load_kuavo_real_wheel_vr.launch`
- **安全警告**：真机回放前请确保机器人周围有足够空间，随时准备按 Ctrl+C 停止

### 5. 选择回放模式

程序提供四种回放模式，每种模式适用于不同的场景：

#### 模式1：VR原始数据回放（回放 /leju_quest_bone_poses 和 /quest_joystick_data）

**适用场景**：
- 需要重新计算逆运动学（IK）
- 机器人配置与录制时不同
- 需要保留完整的VR原始数据信息

**数据流**：
```
1. 回放 /leju_quest_bone_poses → IK节点接收 → 计算逆运动学 → 发布 /kuavo_arm_traj
2. 回放 /quest_joystick_data → IK节点接收 → 处理手柄数据 → 控制手部/头部
3. /kuavo_arm_traj → humanoid_quest_control_with_arm 节点 → 转换为 /humanoid_mpc_target_arm
4. /humanoid_mpc_target_arm → MPC节点 → 执行机器人动作
```

**必需节点**：
- IK节点：`ik_ros_uni` 或 `ik_ros_uni_cpp_node` 或 `quest3_node`
- `humanoid_quest_control_with_arm` 节点（将 `/kuavo_arm_traj` 转换为 MPC 目标轨迹）
- MPC节点：`humanoid_sqp_mpc`

**程序检查项**：
- ✅ IK节点运行状态
- ✅ VR原始数据话题订阅情况
- ✅ IK输出话题（`/kuavo_arm_traj`、`/control_robot_hand_position`）订阅情况
- ✅ `humanoid_quest_control_with_arm` 节点是否订阅 `/kuavo_arm_traj`

**注意事项**：
- 如果回放时只有手指动，上肢不动，可能的原因：
  1. IK节点第二阶段求解失败率过高（检查IK节点日志）
  2. `humanoid_quest_control_with_arm` 节点未订阅 `/kuavo_arm_traj`
  3. 手臂控制模式未设置为外部控制模式 (2)
- 如果看到 `[TorsoIK] 第二阶段IK求解失败` 错误：
  - IK成功率低于30%时，大部分VR数据无法转换为有效的手臂轨迹
  - 可能原因：VR数据超出机器人工作空间，或机器人当前姿态不匹配
  - 建议：检查IK节点日志，调整IK参数或机器人初始姿态

#### 模式2：VR上肢动作（仅回放VR控制的上肢相关话题）

**适用场景**：
- 需要精确复现录制时的动作
- 机器人配置与录制时相同
- 希望回放更稳定、延迟更低

**高保真回放模式**（如果bag文件包含以下话题）：
- `/humanoid_mpc_target_arm` - MPC手臂目标轨迹（MPC实际执行的目标轨迹）
- `/control_robot_hand_position` - 手部位置控制

**数据流**：
```
直接回放 /humanoid_mpc_target_arm → MPC节点 → 执行机器人动作
```
这是最接近录制时实际动作的回放方式。

**备选方案**（如果缺少优先话题）：
程序会自动从以下话题中选择可用的：
- `/kuavo_arm_traj` - 手臂轨迹
- `/humanoid_mpc_target_arm` - MPC目标轨迹
- `/humanoid_mpc_arm_commanded` - MPC命令轨迹
- `/kuavo_arm_traj_filtered` - 滤波后的轨迹
- `/kuavo_arm_traj_origin` - 原始轨迹
- `/control_robot_hand_position` - 手部控制
- `/robot_head_motion_data` - 头部运动数据
- `/drake_ik/eef_pose` - 末端执行器位置
- `/drake_ik/ik_solve_error` - IK解算误差
- `/humanoid_ee_State` - 末端执行器状态
- `/modeSchedule` - 模式调度

**特点**：
- 自动排除 `/humanoid/mpc/arm_control_mode` 话题（避免模式切换问题）
- 如果检测到 `/robot_head_motion_data`，可选择排除（避免警告信息）
- 如果回放 `/kuavo_arm_traj`，需要 `humanoid_quest_control_with_arm` 节点运行

#### 模式3：自定义选择话题
- 显示bag文件中所有话题的复选框
- 可以自由选择要回放的话题
- 不选择则回放所有话题
- 适用于需要精确控制回放内容的场景

#### 模式4：回放所有话题
- 回放bag文件中的所有话题
- 适用于需要完整复现录制场景的情况
- **注意**：可能会回放控制模式话题，导致控制模式被切换

### 6. 配置回放参数

#### 回放速度
- `0.25x` - 慢速（适合仔细观察）
- `0.5x` - 半速
- `1.0x` - 正常速度（默认）
- `2.0x` - 倍速
- `4.0x` - 快速
- `自定义` - 输入任意速度倍数（如 `0.75`, `1.5`）

#### 循环播放
- 选择是否循环播放
- 循环模式下会显示每次循环的开始标记：`━━━ 第 N 次循环开始 ━━━`
- 程序会通过时间检测自动识别循环完成
- 适合需要重复观察动作的场景

#### 起始时间
- 输入从第几秒开始播放（留空则从开始）
- 范围：`0` 到 `bag文件时长`
- 支持小数（如 `10.5`）

#### 播放时长
- 输入播放多少秒（留空则播放到结束）
- 用于只回放bag文件的某一段
- 支持小数（如 `30.5`）

### 7. 环境检查

程序会自动检查以下内容，确保回放环境准备就绪：

#### 7.1 手臂控制节点检查

检查以下节点是否运行：
- `humanoid_quest_control_with_arm` - VR控制节点（**重要**）
  - 负责将 `/kuavo_arm_traj` 转换为 `/humanoid_mpc_target_arm`
  - 如果未运行，需要直接回放 `/humanoid_mpc_target_arm` 话题
- `humanoid_Arm_time_target_control` - 手臂时间目标控制
- `humanoid_plan_arm_trajectory_node` - 手臂轨迹规划节点

#### 7.2 IK节点检查（VR原始数据回放模式）

检查以下IK节点是否运行：
- `ik_ros_uni` - Python IK节点
- `ik_ros_uni_cpp_node` - C++ IK节点
- `quest3_node` - Quest3节点

如果未检测到IK节点，程序会警告并询问是否继续。

#### 7.3 话题订阅者检查

检查以下关键话题是否有订阅者：
- `/kuavo_arm_traj` - 手臂轨迹控制（必需）
- `/humanoid_mpc_target_arm` - MPC手臂目标轨迹（**重要！MPC执行动作需要**）
- `/control_robot_hand_position` - 手部位置控制（必需）
- `/robot_head_motion_data` - 头部运动数据

**注意**：
- 如果话题没有订阅者，回放的数据可能不会被处理！
- 程序会显示每个话题的订阅者列表（最多显示前2-3个）
- 特别检查 `humanoid_quest_control_with_arm` 是否订阅了 `/kuavo_arm_traj`

#### 7.4 手臂控制模式检查

检查 `/humanoid/mpc/arm_control_mode` 话题是否有数据：
- 如果话题存在且有数据，显示当前控制模式
- 如果无法获取，提示可能需要手动设置

### 8. 设置手臂控制模式

**重要**：手臂控制模式必须设置为外部控制模式 (2)，否则手臂不会执行动作。

#### 8.1 自动设置流程

程序会按以下顺序尝试设置控制模式：

1. **Python rospy方式**（优先）：
   - 使用 `rospy.ServiceProxy` 调用 `/humanoid_change_arm_ctrl_mode` 服务
   - 设置 `control_mode=2`（外部控制模式）
   - 如果服务调用失败，尝试通过话题发布控制模式

2. **命令行方式**（备选）：
   - 使用 `rosservice call` 调用服务
   - 如果服务不可用，使用 `rostopic pub` 发布控制模式到 `/humanoid/mpc/arm_control_mode`

#### 8.2 控制模式话题处理

- 如果检测到将回放 `/humanoid/mpc/arm_control_mode` 话题，程序会提示：
  - 这可能导致控制模式在回放过程中被切换
  - 建议排除此话题，改为手动设置
- 可以选择排除控制模式话题，避免回放过程中的模式切换问题

#### 8.3 手动设置（如果自动设置失败）

如果自动设置失败，可以手动运行：

```bash
python3 /home/lab/set_arm_mode.py
```

或者：

```bash
rostopic pub -1 /humanoid/mpc/arm_control_mode std_msgs/Float64MultiArray 'data: [2.0, 2.0]'
```

#### 8.4 常见问题

如果看到警告：`armControlMode_ 1 != EXTERN_CONTROL`
- 说明手臂控制模式未设置为外部控制模式
- 解决方案：
  1. 确保回放 `/humanoid/mpc/arm_control_mode` 话题（如果bag文件中有）
  2. 或者手动切换控制模式（如果bag文件中没有该话题）

### 9. 回放配置确认

在开始回放前，程序会显示完整的回放配置信息：

- **文件路径**：要回放的bag文件路径
- **回放目标**：Mujoco仿真 或 机器人真机
- **回放速度**：回放速度倍数
- **循环播放**：是/否
- **起始时间**：从第几秒开始（如果设置）
- **播放时长**：播放多少秒（如果设置）
- **回放话题数量**：选择的话题数量
- **关键话题检查**：
  - ✓ 包含手臂轨迹（`/kuavo_arm_traj`）
  - ✓ 包含MPC目标轨迹（`/humanoid_mpc_target_arm`）**重要！**
  - ✓ 包含手部位置（`/control_robot_hand_position`）
  - ✓ 包含头部运动（`/robot_head_motion_data`）

**警告提示**：
- 如果未包含 `/humanoid_mpc_target_arm` 话题，会显示警告：
  - MPC需要此话题来接收目标轨迹，否则会出现 `target not updated` 警告
  - 建议：在VR上肢动作模式下，确保包含此话题

### 10. 开始回放

确认所有配置后，程序会：
1. 显示完整的回放配置信息
2. 显示实际执行的 `rosbag play` 命令（用于调试）
3. 开始回放并显示操作提示

### 11. 回放控制

在回放过程中，可以使用以下快捷键：

- **空格键** - 暂停/继续播放
- **Q键** - 退出播放
- **Ctrl+C** - 强制停止（不推荐，可能导致终端状态异常）

**循环模式**：
- 如果启用了循环播放，程序会显示每次循环的开始标记
- 格式：`━━━ 第 N 次循环开始 ━━━`
- 程序通过时间检测自动识别循环完成（每0.3秒检查一次）

## 两种回放模式对比

### VR原始数据回放 vs VR上肢动作回放

| 特性 | VR原始数据回放 | VR上肢动作回放 |
|------|---------------|----------------|
| **数据层级** | 原始输入数据 | 最终执行轨迹 |
| **依赖节点** | IK节点 + 转换节点 + MPC | 仅MPC节点 |
| **处理延迟** | 较高（需IK计算） | 较低（直接执行） |
| **动作保真度** | 可能因IK差异有偏差 | 最接近录制时动作 |
| **适用场景** | 需要重新计算IK、机器人配置改变 | 精确复现动作、快速回放 |
| **稳定性** | 依赖多个节点 | 更稳定 |
| **推荐度** | 特定场景使用 | ⭐ 推荐 |

### 选择建议

**选择VR原始数据回放，如果**：
- 需要重新计算IK
- 机器人配置与录制时不同
- 想保留完整的VR数据信息

**选择VR上肢动作回放，如果**：
- 需要精确复现录制时的动作
- 希望回放更稳定、延迟更低
- 机器人配置与录制时相同
- 仅需MPC节点运行


