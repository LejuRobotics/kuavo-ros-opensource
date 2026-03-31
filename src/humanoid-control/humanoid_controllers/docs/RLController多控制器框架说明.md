# RLControllerBase 多控制器框架说明

## 1. 框架概述

`RLControllerBase` 是一个基于继承的多控制器框架，用于管理不同类型的强化学习（RL）控制器。该框架提供了统一的接口、线程安全的推理机制、传感器数据处理和控制器切换功能。

### 1.1 核心组件

- **RLControllerBase**: 控制器基类，提供统一接口和基础功能
- **RLControllerManager**: 控制器管理器，负责加载、切换和管理多个控制器
- **具体控制器实现**: 如 `FallStandController`（倒地起身）、`AmpWalkController`（行走控制）

### 1.2 主要特性

- ✅ **线程安全**: 推理线程与主控制循环分离，使用互斥锁保护共享数据
- ✅ **统一接口**: 所有控制器继承自 `RLControllerBase`，提供一致的 API
- ✅ **动态切换**: 支持运行时在不同控制器之间切换
- ✅ **配置驱动**: 通过 YAML 配置文件加载和管理控制器
- ✅ **传感器预处理**: 统一的 IMU 滤波和传感器数据处理
- ✅ **ROS 服务**: 提供标准化的 ROS 服务接口

## 2. 框架架构

### 2.1 类继承关系

```
RLControllerBase (基类)
├── FallStandController (倒地起身控制器)
└── AmpWalkController (AMP行走控制器)
```

**文件结构与主要 RL 相关文件**（相对 `src/humanoid-control/humanoid_controllers`）：
```
├── include/
│   └── humanoid_controllers/
│       └── rl/
│           ├── rl_controller_types.h   # 定义 RL 控制器类型（MPC/AMP/FALL_STAND 等）和控制器状态枚举，提供字符串↔枚举转换
│           ├── RLControllerBase.h      # RL 控制器抽象基类声明：统一接口、推理线程、IMU 滤波、线程安全数据访问等
│           ├── RLControllerManager.h   # 控制器管理器声明：加载/存储/切换多个 RLControllerBase 派生类，声明相关 ROS 服务
│           ├── FallStandController.h   # 倒地起身控制器声明：状态机（FALL_DOWN/READY/STAND_UP/STANDING）、轨迹数据结构、服务接口等
│           └── AmpWalkController.h     # AMP 行走控制器声明：行走命令结构、观测构建接口、RL 推理与关节命令接口
├── src/
│   └── rl/
│       ├── RLControllerBase.cpp        # RLControllerBase 实现：推理线程循环、IMU 滤波、clip 裁剪、传感器/状态/动作的线程安全封装
│       ├── RLControllerManager.cpp     # RLControllerManager 实现：从 YAML 加载控制器、创建具体控制器、提供 switch/get_list 等 ROS 服务
│       ├── FallStandController.cpp     # 倒地起身控制器实现：加载模型与 CSV 轨迹、插值起身关节、RL 推理、服务回调、观测构建等
│       └── AmpWalkController.cpp       # AMP 行走控制器实现：加载 RL 配置与 OpenVINO 模型、gait 指令接收、观测拼接、RL 推理与力/位控映射
├── config/
│   └── kuavo_v<版本号>/                      # 针对不同版本机器人（如 kuavo_v14）的配置目录
│       ├── rl_controllers.yaml         # RL 控制器列表配置：定义各控制器 name/type/config_file/enabled/class 等
│       └── rl/                         # 本版本下所有 RL 控制器的参数与轨迹文件
│           ├── amp_param.info          # AMP 行走控制器的 RL 配置（网络模型、观测布局、动作缩放、步态周期等，文件名随版本略有不同）
│           ├── fall_stand_param.info   # 倒地起身控制器的 RL/轨迹配置（模型文件名、轨迹 CSV、obs 配置、起身插值参数等）
│           └── skw_rl_param.info       # 其他 RL 控制器（如单腿/特殊步态）配置示例（不同版本可能仅包含子集）
└── src/                                # humanoid_controllers 顶层源码目录
    └── humanoidController.cpp          # 主控制器：整合 MPC+WBC 与 RLControllerManager，
                                        # 在 update() 中根据当前模式选择走「MPC+WBC」还是调用当前 RL 控制器的 update() 生成 jointCmd，
                                        # 并通过 startMPCRLInterpolation/updateMPCRLInterpolation 做 MPC⇄RL 平滑切换与倒地自动切换
```

### 2.2 与 RL / 控制器切换相关的 ROS 接口

- **控制器级服务（由每个 RL 控制器自动提供，命名空间为 `/humanoid_controllers/{controller_name}`）**
  - `/humanoid_controllers/{controller_name}/reload` (`std_srvs/Trigger`)  
    - 重新加载该控制器的配置文件（内部调用派生类的 `loadConfig`），常用于在线调参后热更新。
  - `/humanoid_controllers/{controller_name}/isActive` (`std_srvs/Trigger`)  
    - 返回当前控制器是否处于 **RUNNING** 状态（即是否被 `RLControllerManager` 选为当前控制器并允许推理）。
  - `/humanoid_controllers/{controller_name}/getState` (`std_srvs/Trigger`)  
    - 返回控制器内部状态枚举 `ControllerState` 的整数值（INITIALIZING/RUNNING/PAUSED/ERROR/STOPPED）。
  - `/humanoid_controllers/{controller_name}/getType` (`std_srvs/Trigger`)  
    - 返回该控制器的 `RLControllerType` 枚举值（MPC/AMP_CONTROLLER/FALL_STAND_CONTROLLER 等）的整数编码。
  - `/humanoid_controllers/{controller_name}/reset` (`std_srvs/Trigger`)  
    - 调用派生类的 `reset()`，用于清理内部状态（相位、轨迹时间步、动作缓存等）。

- **控制器管理与切换服务（由 `RLControllerManager` 提供，命名空间 `humanoid_controller`）**
  - `/humanoid_controller/switch_controller` (`kuavo_msgs/switchController`)  
    - 根据请求中的 `controller_name` 在 **行走控制器列表** 中切换当前控制器：  
      - `"mpc"` 或空字符串：切回 MPC 基础控制；  
      - 其他名称（如 `"amp_controller"`、`"depth_loco_controller"`、`"fall_stand"` 等）：切换到对应 RL 控制器（若已加载且启用）。
  - `/humanoid_controller/get_controller_list` (`kuavo_msgs/getControllerList`)  
    - 返回当前可用的行走控制器名称列表（包含 `"mpc"`），以及当前激活控制器名称与索引。
  - `/humanoid_controller/switch_to_next_controller` (`kuavo_msgs/switchToNextController`)  
    - 在行走控制器列表中按顺序循环切换（`mpc → 第一个 RL → ... → mpc`），适合作为手柄或键盘「一键切换模式」接口。

- **Depth Loco 控制器补充说明**
  - `DepthWalkController` 属于 `DEPTH_LOCO_CONTROLLER`，适合走楼梯斜坡这类需要地形感知的行走场景。
  - 该控制器除了常规机器人状态与 `/cmd_vel` 外，还依赖深度历史输入话题 `/camera/depth/depth_history_array`。
  - 切换到 `depth_loco_controller` 前会先检查该话题是否已发布且能收到消息；如果没有，系统会拒绝切换。
  - 从 `depth_loco_controller` 退出时，会优先恢复到进入前的控制器；如果没有历史记录，则回到 `amp_controller`。
  - 在 `kuavo_v54` 中，可通过 [`rl_controllers.yaml`](/home/lab/kuavo-ros-control/src/humanoid-control/humanoid_controllers/config/kuavo_v54/rl_controllers.yaml) 启用 `depth_loco_controller`，其模型配置位于 [`depth_loco_param.info`](/home/lab/kuavo-ros-control/src/humanoid-control/humanoid_controllers/config/kuavo_v54/rl/depth_loco_param.info)。

- **倒地 / 起身相关服务**
  - `/humanoid_controller/set_fall_down_state` (`std_srvs/SetBool`，在 `humanoidController` 中实现)  
    - 将内部的 `fall_down_state_` 设为 FALL_DOWN 或 STANDING；当设置为 **倒地** 时，若存在 `FALL_STAND_CONTROLLER`，会自动通过 `RLControllerManager` 切换过去。
  - `/humanoid_controller/trigger_fall_stand_up` (`std_srvs/Trigger`，在 `FallStandController` 中实现)  
    - 触发倒地起身流程：关闭自动步态服务、重置轨迹时间步、计算当前与轨迹参考 yaw 差，并进入起身状态机的 READY/STAND_UP 阶段。

- **监控与调试相关话题**
  - `/humanoid_controller/is_rl_controller_` (`std_msgs/Float64` / 通过 `TopicLogger` 发布)  
    - 实时发布当前是否处于 RL 控制模式（由 `!controller_manager_->isBaseControllerActive()` 决定），便于监控 MPC↔RL 模式切换。
  - `/rl_controller/actions`、`/rl_controller/singleInputData`、`/rl_controller/InputData/<key>` 等（通过 `TopicLogger` 发布）  
    - 由基类和各 RL 控制器在推理线程中发布：  
      - `actions`：当前网络输出动作；  
      - `singleInputData`：拼接后的单帧观测向量；  
      - `InputData/<key>`：按配置分块的观测子向量（例如 `joint_pos`、`base_ang_vel` 等），用于可视化和调试 RL 观测。
  - `/humanoid_controller/resetting_mpc_state_` (`std_msgs/Float64` / 通过 `TopicLogger` 发布)  
    - 实时发布 MPC 重置状态，用于监控从 RL 切换到 MPC 时的重置过程：  
      - `0` (`NOMAL`)：正常状态，MPC 正常运行；  
      - `1` (`RESET_INITIAL_POLICY`)：重置 MPC 状态1，等待初始策略；  
      - `2` (`RESET_BASE`)：重置 MPC 状态2，更新躯干位置（插值阶段）。  
    - 当从 RL 切回 MPC 时，状态会依次经历 `RESET_INITIAL_POLICY` → `RESET_BASE` → `NOMAL`，便于监控 MPC 重置进度。
  - 
 - **遥控器集成**
   - 目前北通遥控器集成了多控制器切换、倒地起身等功能
     - 

### 2.3 核心类说明

#### RLControllerBase

**职责**:
- 管理推理线程的生命周期（创建、启动、暂停、停止）
- 提供线程安全的传感器数据和状态存储
- 实现通用的 IMU 滤波预处理
- 提供标准的 ROS 服务接口（reload, isActive, getState, getType, reset）
- 定义控制器状态机（INITIALIZING, RUNNING, PAUSED, ERROR, STOPPED）

**关键成员变量**:
```cpp
// 推理相关
std::thread inference_thread_;              // 推理线程
double inference_frequency_{100.0};        // 推理频率（Hz）
Eigen::VectorXd networkInputDataRL_;        // 网络输入数据
Eigen::VectorXd actions_;                  // 当前动作（线程安全）

// 传感器数据（线程安全）
mutable std::recursive_mutex sensor_data_mtx_;
SensorData stored_sensor_data_;
Eigen::VectorXd stored_measured_state_;
std::atomic<bool> sensor_data_updated_;

// RL 控制参数
Eigen::VectorXd defalutJointPosRL_;        // 默认关节位置
Eigen::VectorXd jointKpRL_, jointKdRL_;    // PD 参数
Eigen::VectorXd torqueLimitsRL_;           // 力矩限制
// ... 更多参数
```

**关键方法**:
```cpp
// 必须实现的虚函数
virtual bool initialize() = 0;
virtual bool loadConfig(const std::string& config_file) = 0;
virtual bool updateImpl(...) = 0;
virtual void reset() = 0;

// 可选的虚函数（有默认实现）
virtual void preprocessSensorData(SensorData& sensor_data);
virtual void updateObservation(const Eigen::VectorXd& state_est, const SensorData& sensor_data);
virtual bool inference(const Eigen::VectorXd& observation, Eigen::VectorXd& action);
virtual void actionToJointCmd(...);
virtual bool shouldRunInference() const;
virtual bool isReadyToExit() const;
```

#### RLControllerManager

**职责**:
- 管理多个控制器的生命周期
- 从 YAML 配置文件加载控制器
- 提供控制器切换功能
- 提供 ROS 服务接口（switch_controller, get_controller_list, switch_to_next_controller）

**关键方法**:
```cpp
bool addController(const std::string& name, std::unique_ptr<RLControllerBase> controller);
bool switchController(const std::string& name);
bool loadControllersFromConfig(const std::string& config_file, ...);
RLControllerBase* getCurrentController();
```

### 2.4 工作流程

```
┌─────────────────┐
│ humanoidController │
│   (主控制循环)    │
└────────┬─────────┘
         │
         │ update()
         ▼
┌─────────────────┐
│ RLControllerBase │
│  - 存储传感器数据 │
│  - 调用 updateImpl() │
└────────┬─────────┘
         │
         │ updateImpl()
         ▼
┌─────────────────┐
│ 具体控制器实现   │
│ (FallStand/Amp) │
│  - 计算关节命令  │
└─────────────────┘

┌─────────────────┐
│ 推理线程         │
│ (独立线程)       │
│  - updateObservation() │
│  - inference()   │
│  - 更新 actions_ │
└─────────────────┘
```

### 2.5 系统整体控制框架（含 MPC）

从更高一层看，当前控制系统可以概括为一条「**传感器 / 状态估计 → 规划（MPC 或 RL）→ WBC/关节命令 → 执行器**」的链路，其核心模块如下：

- **humanoidController（主控制节点）**  
  - 负责整个控制循环（`update()`）、话题/服务接口、键盘/遥控指令处理等。  
  - 持有并驱动：状态估计器、MPC 接口、WBC、RL 控制器管理器等。
- **状态估计层（State Estimation）**  
  - 通过 `sensors_data_buffer_ptr_` 获取原始 `SensorData`，调用 `stateEstimate_` 估计刚体状态 `measuredRbdState_` 和观测 `currentObservation_`。  
  - 为 MPC 和 RL 提供统一的状态输入（包括质心状态、接触力估计、脚端位置等）。
- **MPC 层（基于 ocs2）**  
  - 由 `mrtRosInterface_` 与 MPC 节点交互，更新最优策略 `policy` 和目标轨迹 `mpc_current_target_trajectories_`。  
  - 在 **MPC 模式** 下，`humanoidController::update()` 使用 MPC 的 `optimizedState_mrt/optimizedInput_mrt` 作为参考。
- **WBC 层（Whole-Body Control）**  
  - 使用 `WeightedWbc` / `StandUpWbc` 将质心/摆动脚目标转化为关节空间的 `posDes/velDes/torque`。  
  - `SafetyChecker` 在此层之后做安全检查，失败时会触发倒地逻辑并切换到 RL 倒地起身控制器。
- **RL 控制层（本文件的主角）**  
  - 由 `RLControllerManager` + 若干 `RLControllerBase` 派生类构成，例如 `AmpWalkController`、`FallStandController`。  
  - 在 **RL 模式** 下，当前 RL 控制器直接在 `update()` 中生成完整的 `kuavo_msgs::jointCmd`，替代「MPC+WBC」链路的输出。

可以将两条控制路径抽象成：

```
传感器 & 状态估计
        │
        ├─► MPC + WBC + SafetyChecker ─► jointCmd （MPC 模式 / 基础控制）
        │
        └─► RLControllerManager → 当前 RLControllerBase ─► jointCmd （RL 模式）
```

两条路径共用同一套：**状态估计、硬件接口、jointCmd 发布与安全保护框架**，区别只在于「谁来产生主要的关节命令」。

### 2.6 MPC 与 RL 控制器的关系与切换逻辑

- **MPC 作为「BASE 控制器」**  
  - 在 `rl_controller_types.h` 中，`RLControllerType::MPC` 表示基础控制器；在 `RLControllerManager` 中，`current_controller_name_` 为空时即视为 MPC 模式。  
  - `walk_controllers_` 的第 0 个元素固定为 `"mpc"`，对应「基础行走控制」，其余元素为各 RL 行走控制器名称。  
  - `humanoidController::update()` 中通过  
    - `is_rl_controller_ = !controller_manager_->isBaseControllerActive();`  
    判断当前是否处于 RL 模式。

- **MPC 流程 vs RL 流程**  
  - 在 `update()` 中，经过状态估计和接触检测之后，会根据 `is_rl_controller_` 和插值状态决定当前是否走 MPC 流：  
    - 当 `mpc_flow && !is_fall_stand_controller_active` 时：  
      使用 **MPC + WBC** 生成 `jointCmdMsg`。  
    - 否则：  
      - 若当前控制器是 RL 控制器，则调用  
        `current_controller_ptr_->update(time, getRobotSensorData(), getRobotState(), jointCmdMsg);`  
        由 RL 控制器直接生成 `jointCmdMsg`（之后 humanoidController 只补齐头部关节等通用部分）。  
      - 同一 `jointCmdMsg` 话题最终送往硬件/仿真，两种模式对下层是透明的。

- **MPC ⇨ RL：带插值的平滑切换**  
  - 切换时机：外部通过 `/humanoid_controller/switch_controller` 或内部事件（如倒地）将 `RLControllerManager` 从 BASE 切换到某个 RL 控制器，此时 `is_rl_controller_` 由 `false` 变为 `true`。  
  - 切换模式由 RL 控制器的配置文件中的 `use_interpolate_from_mpc` 参数决定：
    - 若 `use_interpolate_from_mpc == true`（默认推荐），则在第一次进入 RL 模式时执行：  
      - 从当前 RL 控制器读取 `getDefaultJointPos()` 和 `getDefaultBaseHeightControl()`，作为 RL 模式下的期望姿态。  
      - 通过 `startMPCRLInterpolation()` / `updateMPCRLInterpolation()` 使用 **MPC** 在一段时间内插值 torso 高度和手臂位置，使机器人从 MPC 当前状态平滑过渡到 RL 默认站姿。  
      - 在插值期间：`mpc_flow` 仍为 `true`，MPC 继续运行，但其目标会被「插值结果」覆盖；插值完成后才完全切到 RL 控制器输出。
    - 若 `use_interpolate_from_mpc == false`，则跳过插值，下一周期直接进入 RL 控制路径（适合仿真或对姿态跳变不敏感的场景）。

- **RL ⇨ MPC：回到基础控制**  
  - 当 RL 控制结束或被切回 MPC 时（`last_is_rl_controller_ && !is_rl_controller_`）：  
    - 设置 `reset_mpc_ = true` 并记录当前状态、手臂位置，随后在 MPC 分支中重置 MPC：  
      - 暂停/恢复 MPC 节点、清空旧策略、以当前状态作为新的初始状态重建策略。  
      - 重置手臂滤波器，使 MPC 输出从当前实际姿态平滑延续，避免 RL → MPC 时出现大的关节跳变。  
  - MPC 重置完毕后，`resetting_mpc_state_` 从 `RESET_INITIAL_POLICY` / `RESET_BASE` 进入 `NOMAL`，系统回到纯 MPC+WBC 控制。

- **与倒地起身控制（FallStandController）的协同**  
  - 在以下场景会切换到 `FALL_STAND_CONTROLLER`：  
    - `/humanoid_controller/set_fall_down_state` 服务被设置为 `true`；  
    - `SafetyChecker` 检测到严重安全问题（例如中心高度/接触力异常）并将 `fall_down_state_` 置为倒地。  
  - 切换逻辑：  
    - `RLControllerManager` 切到 `RLControllerType::FALL_STAND_CONTROLLER`，`humanoidController` 暂停 MPC，之后 `update()` 直接调用倒地起身控制器的 `update()` 生成关节命令。  
    - 当 `FallStandController::isReadyToExit()` 返回 `true`（站起完成），`humanoidController` 会自动调用 `switchToBaseController()` 切回 MPC 模式，并将 `fall_down_state_` 置为 STANDING。

通过上述机制，**MPC 始终作为系统的「基准 / 安全 fallback 控制器」存在，RL 控制器作为按需启用的「插件式高层控制」**：  
- RL 激活时可以完全接管 jointCmd 生成；  
- 切入/切出过程由 MPC 保证姿态与高度的平滑过渡；  
- 任何异常（例如倒地）仍可回退到专门的 RL 起身控制或基础 MPC 控制，保证整体系统的鲁棒性。

## 3. 如何添加新控制器

### 3.1 步骤概览

1. 在 `rl_controller_types.h` 中添加控制器类型枚举
2. 创建控制器类（继承 `RLControllerBase`）
3. 实现必要的虚函数
4. 在 `RLControllerManager` 中注册控制器创建逻辑
5. 创建配置文件

### 3.2 详细步骤

#### 步骤 1: 添加控制器类型枚举

编辑 `src/humanoid-control/humanoid_controllers/include/humanoid_controllers/rl/rl_controller_types.h`:

```cpp
enum class RLControllerType
{
  MPC = 0,
  AMP_CONTROLLER,
  FALL_STAND_CONTROLLER,
  YOUR_NEW_CONTROLLER,  // 添加新类型
};

// 添加字符串转换函数
inline bool stringToRLControllerType(const std::string& type_str, RLControllerType& type)
{
  // ... 现有代码 ...
  else if (type_str == "YOUR_NEW_CONTROLLER")
  {
    type = RLControllerType::YOUR_NEW_CONTROLLER;
    return true;
  }
  // ...
}
```

#### 步骤 2: 创建控制器头文件

创建 `src/humanoid-control/humanoid_controllers/include/humanoid_controllers/rl/YourNewController.h`:

```cpp
#pragma once

#include "humanoid_controllers/rl/RLControllerBase.h"
#include <openvino/openvino.hpp>  // 如果需要神经网络推理

namespace humanoid_controller
{
  class YourNewController : public RLControllerBase
  {
  public:
    YourNewController(const std::string& name, 
                     const std::string& config_file,
                     ros::NodeHandle& nh,
                     ocs2::humanoid::TopicLogger* ros_logger = nullptr);
    
    virtual ~YourNewController();

    // 必须实现的虚函数
    bool initialize() override;
    bool loadConfig(const std::string& config_file) override;
    void reset() override;
    bool isReadyToExit() const override;

  protected:
    // 必须实现的虚函数
    bool updateImpl(const ros::Time& time, 
                    const SensorData& sensor_data,
                    const Eigen::VectorXd& measuredRbdState,
                    kuavo_msgs::jointCmd& joint_cmd) override;

    // 可选：重写传感器预处理
    void preprocessSensorData(SensorData& sensor_data) override;

    // 可选：重写观测更新
    void updateObservation(const Eigen::VectorXd& state_est, 
                          const SensorData& sensor_data) override;

    // 可选：重写推理逻辑
    bool inference(const Eigen::VectorXd& observation, 
                  Eigen::VectorXd& action) override;

    // 可选：重写动作到关节命令的转换
    void actionToJointCmd(const Eigen::VectorXd& actuation, 
                         const Eigen::VectorXd& measuredRbdState,
                         kuavo_msgs::jointCmd& joint_cmd) override;

    // 可选：重写推理条件检查
    bool shouldRunInference() const override;

  private:
    // 控制器特定的成员变量
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
    
    // 其他私有成员...
  };
}
```

#### 步骤 3: 实现控制器

创建 `src/humanoid-control/humanoid_controllers/src/rl/YourNewController.cpp`:

```cpp
#include "humanoid_controllers/rl/YourNewController.h"
#include <ros/ros.h>
#include <ocs2_core/misc/LoadData.h>

namespace humanoid_controller
{
  YourNewController::YourNewController(const std::string& name, 
                                       const std::string& config_file,
                                       ros::NodeHandle& nh,
                                       ocs2::humanoid::TopicLogger* ros_logger)
    : RLControllerBase(name, RLControllerType::YOUR_NEW_CONTROLLER, config_file, nh, ros_logger)
  {
    // 构造函数：基类会自动调用 initializeServices()
  }

  YourNewController::~YourNewController()
  {
    stop();  // 停止推理线程
  }

  bool YourNewController::initialize()
  {
    // 1. 加载配置
    if (!loadConfig(config_file_))
    {
      ROS_ERROR("[%s] Failed to load config file: %s", name_.c_str(), config_file_.c_str());
      return false;
    }

    // 2. 加载模型（如果需要）
    try
    {
      compiled_model_ = core_.compile_model(network_model_file_, "CPU");
      infer_request_ = compiled_model_.create_infer_request();
      ROS_INFO("[%s] Model loaded successfully", name_.c_str());
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[%s] Failed to load model: %s", name_.c_str(), e.what());
      return false;
    }

    // 3. 初始化 RL 滤波器（如果需要）
    loadRLFilterParams(config_file_);

    // 4. 初始化其他数据
    // ...

    initialized_ = true;
    
    // 5. 启动推理线程（可选，如果需要在初始化时启动）
    // start();  // 通常由 RLControllerManager 管理

    ROS_INFO("[%s] Controller initialized successfully", name_.c_str());
    return true;
  }

  bool YourNewController::loadConfig(const std::string& config_file)
  {
    ROS_INFO("[%s] Loading config from: %s", name_.c_str(), config_file.c_str());
    
    try
    {
      // 加载模型文件路径
      std::string network_model_root_path, network_model_file_name;
      nh_.getParam("/network_model_file", network_model_root_path);
      loadData::loadCppDataType(config_file, "networkModelFile", network_model_file_name);
      network_model_file_ = network_model_root_path + network_model_file_name;

      // 加载基本参数
      loadData::loadCppDataType(config_file, "inferenceFrequency", inference_frequency_);
      loadData::loadCppDataType(config_file, "numSingleObs", numSingleObsRL_);
      loadData::loadCppDataType(config_file, "frameStack", frameStackRL_);

      // 加载 RL 控制参数
      loadData::loadEigenMatrix(config_file, "defaultJointState", defalutJointPosRL_);
      loadData::loadEigenMatrix(config_file, "defaultBaseState", defaultBaseStateRL_);
      loadData::loadEigenMatrix(config_file, "JointControlMode", JointControlModeRL_);
      loadData::loadEigenMatrix(config_file, "jointKp", jointKpRL_);
      loadData::loadEigenMatrix(config_file, "jointKd", jointKdRL_);
      loadData::loadEigenMatrix(config_file, "torqueLimits", torqueLimitsRL_);
      loadData::loadEigenMatrix(config_file, "actionScaleTest", actionScaleTestRL_);

      // 设置初始状态
      initialStateRL_.resize(12 + defalutJointPosRL_.size());
      initialStateRL_ << defaultBaseStateRL_, defalutJointPosRL_;

      // 加载其他参数
      loadData::loadCppDataType(config_file, "actionScale", actionScaleRL_);
      loadData::loadCppDataType(config_file, "clipActions", clipActionsRL_);
      loadData::loadCppDataType(config_file, "withArm", withArmRL_);

      // 从 ROS 参数获取
      nh_.getParam("/is_real", is_real_);
      nh_.getParam("/is_roban", is_roban_);

      // 初始化观测数据
      singleInputDataRL_.resize(numSingleObsRL_);
      networkInputDataRL_.resize(numSingleObsRL_ * frameStackRL_);
      actions_.resize(num_actions_);
      singleInputDataRL_.setZero();
      networkInputDataRL_.setZero();
      actions_.setZero();

      ROS_INFO("[%s] Config loaded successfully", name_.c_str());
      return true;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[%s] Failed to load config: %s", name_.c_str(), e.what());
      return false;
    }
  }

  void YourNewController::reset()
  {
    // 重置控制器状态
    actions_.setZero();
    // 重置其他状态...
    ROS_INFO("[%s] Controller reset", name_.c_str());
  }

  bool YourNewController::isReadyToExit() const
  {
    // 定义何时控制器完成任务
    return false;  // 或根据实际逻辑返回
  }

  bool YourNewController::updateImpl(const ros::Time& time, 
                                     const SensorData& sensor_data,
                                     const Eigen::VectorXd& measuredRbdState,
                                     kuavo_msgs::jointCmd& joint_cmd)
  {
    if (!initialized_)
    {
      ROS_ERROR("[%s] Controller not initialized", name_.c_str());
      return false;
    }

    // 1. 获取当前动作（线程安全）
    Eigen::VectorXd local_action = getCurrentAction();

    // 2. 计算执行器输出
    Eigen::VectorXd actuation = updateRLcmd(measuredRbdState);

    // 3. 转换为关节命令
    actionToJointCmd(actuation, measuredRbdState, joint_cmd);
    joint_cmd.header.stamp = time;

    return true;
  }

  void YourNewController::updateObservation(const Eigen::VectorXd& state_est, 
                                            const SensorData& sensor_data)
  {
    // 1. 提取状态数据
    const Eigen::Vector3d baseEuler(state_est(2), state_est(1), state_est(0));
    const Eigen::Vector3d baseAngVel(state_est(6 + jointNum_ + waistNum_ + jointArmNum_),
                                      state_est(6 + jointNum_ + waistNum_ + jointArmNum_ + 1),
                                      state_est(6 + jointNum_ + waistNum_ + jointArmNum_ + 2));
    // ...

    // 2. 构建观测数据
    Eigen::VectorXd jointPos = sensor_data.jointPos_ - defalutJointPosRL_;
    Eigen::VectorXd jointVel = sensor_data.jointVel_;
    // ...

    // 3. 填充 singleInputDataRL_ 和 networkInputDataRL_
    // （根据配置的 singleInputData 结构）
    // ...

    // 4. 更新 input_deque（如果使用帧堆叠）
    input_deque.push_back(singleInputDataRL_);
    input_deque.pop_front();
    for (int i = 0; i < frameStackRL_; ++i)
    {
      networkInputDataRL_.segment(i * numSingleObsRL_, numSingleObsRL_) = input_deque[i];
    }
  }

  bool YourNewController::inference(const Eigen::VectorXd& observation, 
                                     Eigen::VectorXd& action)
  {
    try
    {
      // 创建推理请求
      infer_request_ = compiled_model_.create_infer_request();
      const auto input_port = compiled_model_.input();

      // 检查输入维度
      const auto expected_input_shape = input_port.get_shape();
      const size_t expected_input_length = expected_input_shape[1];
      const size_t actual_input_length = networkInputDataRL_.size();

      if (actual_input_length != expected_input_length)
      {
        ROS_ERROR_THROTTLE(1.0, "[%s] Input size mismatch: %ld vs %ld", 
                          name_.c_str(), actual_input_length, expected_input_length);
        action = Eigen::VectorXd::Zero(num_actions_);
        return false;
      }

      // 准备输入数据
      Eigen::VectorXf float_network_input = networkInputDataRL_.cast<float>();
      ov::Tensor input_tensor(input_port.get_element_type(), 
                             input_port.get_shape(), 
                             float_network_input.data());
      infer_request_.set_input_tensor(input_tensor);

      // 执行推理
      infer_request_.start_async();
      infer_request_.wait();

      // 获取输出
      const auto output_tensor = infer_request_.get_output_tensor();
      const size_t output_buf_length = output_tensor.get_size();
      const auto output_buf = output_tensor.data<float>();

      // 检查输出维度
      const size_t expected_output_length = withArmRL_ ? 
        jointNum_ + waistNum_ + jointArmNum_ : jointNum_ + waistNum_;
      if (output_buf_length != expected_output_length)
      {
        ROS_ERROR_THROTTLE(1.0, "[%s] Output size mismatch: %ld vs %ld", 
                          name_.c_str(), output_buf_length, expected_output_length);
        action = Eigen::VectorXd::Zero(num_actions_);
        return false;
      }

      // 复制输出
      action.resize(output_buf_length);
      for (int i = 0; i < output_buf_length; ++i)
      {
        action[i] = output_buf[i];
      }

      // 裁剪动作
      clip(action, clipActionsRL_);

      return true;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] Inference failed: %s", name_.c_str(), e.what());
      action = Eigen::VectorXd::Zero(num_actions_);
      return false;
    }
  }

  void YourNewController::actionToJointCmd(const Eigen::VectorXd& actuation, 
                                           const Eigen::VectorXd& measuredRbdState,
                                           kuavo_msgs::jointCmd& joint_cmd)
  {
    // 参考 FallStandController 或 AmpWalkController 的实现
    // 根据 is_real_ 和 JointControlModeRL_ 设置关节命令
    // ...
  }

  // 实现其他辅助方法...
  Eigen::VectorXd YourNewController::updateRLcmd(const Eigen::VectorXd& state)
  {
    // 计算执行器输出（参考 FallStandController::updateRLcmd）
    // ...
  }
}
```

#### 步骤 4: 在 RLControllerManager 中注册

编辑 `src/humanoid-control/humanoid_controllers/src/rl/RLControllerManager.cpp`:

```cpp
// 1. 添加头文件
#include "humanoid_controllers/rl/YourNewController.h"

// 2. 在 loadControllersFromConfig 中添加创建逻辑
bool RLControllerManager::loadControllersFromConfig(...)
{
  // ...
  // 根据类型创建控制器
  std::unique_ptr<RLControllerBase> controller;
  if (type == RLControllerType::FALL_STAND_CONTROLLER)
  {
    controller = std::make_unique<FallStandController>(name, config_file_abs, nh, ros_logger);
  }
  else if (type == RLControllerType::AMP_CONTROLLER)
  {
    controller = std::make_unique<AmpWalkController>(name, config_file_abs, nh, ros_logger);
  }
  else if (type == RLControllerType::YOUR_NEW_CONTROLLER)  // 添加这里
  {
    controller = std::make_unique<YourNewController>(name, config_file_abs, nh, ros_logger);
  }
  else
  {
    ROS_WARN("[RLControllerManager] Controller type '%s' not yet implemented, skipping", type_str.c_str());
    continue;
  }
  // ...
}
```

#### 步骤 5: 更新 CMakeLists.txt

在 `src/humanoid-control/humanoid_controllers/CMakeLists.txt` 中添加新文件:

```cmake
# 头文件
set(HEADERS
  # ... 现有文件 ...
  include/humanoid_controllers/rl/YourNewController.h
)

# 源文件
set(SOURCES
  # ... 现有文件 ...
  src/rl/YourNewController.cpp
)
```

#### 步骤 6: 创建配置文件

创建控制器配置文件（例如 `config/your_new_controller.info`）:

```info
# 模型文件
networkModelFile "your_model.xml"

# 推理频率
inferenceFrequency 100.0

# 观测参数
numSingleObs 48
frameStack 1

# 动作参数
actionScale 0.25
clipActions 1.0
withArm true

# 默认状态
defaultJointState [0.0, 0.0, ...]  # 关节数量
defaultBaseState [0.0, 0.0, ...]  # 12维

# 控制参数
JointControlMode [0, 0, ...]  # 关节数量
jointKp [50.0, 50.0, ...]     # 关节数量
jointKd [10.0, 10.0, ...]     # 关节数量
torqueLimits [100.0, 100.0, ...]  # 关节数量
actionScaleTest [1.0, 1.0, ...]   # 关节数量

# IMU 滤波参数（可选）
accFilterCutoffFreq [50.0, 50.0, 50.0]
freeAccFilterCutoffFreq [50.0, 50.0, 50.0]
gyroFilterCutoffFreq [50.0, 50.0, 50.0]
accFilterState [1.0, 1.0, 1.0]
freeAccFilterState [1.0, 1.0, 1.0]
gyroFilterState [1.0, 1.0, 1.0]

# 单输入数据配置（可选）
singleInputData {
  base_ang_vel {
    startIdx 0
    numIdx 3
    obsScales 1.0
  }
  joint_pos {
    startIdx 0
    numIdx 21
    obsScales 1.0
  }
  # ... 更多观测项 ...
}
```

#### 步骤 7: 添加到控制器列表配置

在 YAML 配置文件中添加控制器（例如 `config/controllers.yaml`）:

```yaml
controllers:
  - name: "your_new_controller"
    type: "YOUR_NEW_CONTROLLER"
    config_file: "your_new_controller.info"
    enabled: true
    class: "BASE_CONTROLLER"  # 可选
```

### 3.3 实现要点

#### 线程安全

- 使用 `getCurrentAction()` 获取动作（已加锁）
- 使用 `getRobotSensorData()` 获取传感器数据（已加锁）
- 在 `updateImpl()` 中不要直接访问 `actions_`，使用 `getCurrentAction()`

#### 推理线程

- 推理在独立线程中运行（`inferenceThreadFunc()`）
- 推理频率由 `inference_frequency_` 控制
- 只有在 `shouldRunInference()` 返回 `true` 时才执行推理
- 推理结果通过 `setCurrentAction()` 更新（线程安全）

#### 传感器预处理

- 基类已实现 IMU 滤波（`preprocessSensorData()`）
- 派生类可以重写以添加特定处理
- 记得先调用 `RLControllerBase::preprocessSensorData(sensor_data)`

#### 观测数据构建

- 使用 `singleInputDataRL_` 存储单帧观测
- 使用 `networkInputDataRL_` 存储堆叠后的观测（用于推理）
- 使用 `input_deque` 管理历史帧（如果 `frameStackRL_ > 1`）

#### 关节命令生成

- 参考 `FallStandController::actionToJointCmd()` 或 `AmpWalkController::actionToJointCmd()`
- 根据 `is_real_` 区分仿真和真实机器人
- 根据 `JointControlModeRL_` 设置控制模式
- 注意处理腰部关节的位置（`is_roban_` 时需调整）

## 4. 控制器状态管理

### 4.1 控制器状态

```cpp
enum class ControllerState
{
  INITIALIZING = 0,  // 初始化中
  RUNNING,           // 运行中（推理线程执行推理）
  PAUSED,            // 暂停（推理线程不执行推理）
  ERROR,             // 错误状态
  STOPPED,           // 停止（推理线程退出）
};
```

### 4.2 状态转换

```
INITIALIZING → (initialize()) → PAUSED
PAUSED → (resume()) → RUNNING
RUNNING → (pause()) → PAUSED
PAUSED/RUNNING → (stop()) → STOPPED
```

### 4.3 控制器切换流程

```
当前控制器 (RUNNING)
  ↓ pause()
当前控制器 (PAUSED)
  ↓
新控制器 (PAUSED)
  ↓ resume()
新控制器 (RUNNING)
```

## 5. ROS 服务接口

### 5.1 控制器级别的服务

每个控制器自动提供以下服务（命名空间为 `/{controller_name}/`）:

- `/humanoid_controllers/{controller_name}/reload`: 重新加载配置
- `/humanoid_controllers/{controller_name}/isActive`: 检查是否激活
- `/humanoid_controllers/{controller_name}/getState`: 获取状态
- `/humanoid_controllers/{controller_name}/getType`: 获取类型
- `/humanoid_controllers/{controller_name}/reset`: 重置控制器

### 5.2 管理器级别的服务

- `/humanoid_controller/switch_controller`: 切换到指定控制器
- `/humanoid_controller/get_controller_list`: 获取控制器列表
- `/humanoid_controller/switch_to_next_controller`: 切换到下一个控制器

## 6. 配置文件格式

### 6.1 控制器列表配置（YAML）

```yaml
controllers:
  - name: "fall_stand"
    type: "FALL_STAND_CONTROLLER"
    config_file: "fall_stand.info"
    enabled: true
    class: "FALL_STAND_CONTROLLER"
  
  - name: "amp_walk"
    type: "AMP_CONTROLLER"
    config_file: "amp_walk.info"
    enabled: true
    class: "BASE_CONTROLLER"
```

### 6.2 控制器参数配置（.info 文件）

使用 `ocs2_core/misc/LoadData.h` 的格式，支持:
- `loadCppDataType()`: 加载基本类型（int, double, string, bool）
- `loadEigenMatrix()`: 加载 Eigen 矩阵/向量
- `loadPtreeValue()`: 加载嵌套结构

## 7. 最佳实践

### 7.1 代码组织

- 将控制器特定的逻辑放在派生类中
- 复用基类提供的通用功能（滤波、线程管理等）
- 保持接口一致性

### 7.2 错误处理

- 在 `initialize()` 和 `loadConfig()` 中检查所有必要的参数
- 使用 `ROS_ERROR` 和 `ROS_WARN` 记录错误
- 推理失败时返回零动作

### 7.3 性能优化

- 合理设置推理频率（`inference_frequency_`）
- 避免在 `updateImpl()` 中进行耗时操作
- 使用 `ROS_ERROR_THROTTLE` 限制日志输出频率

### 7.4 调试

- 使用 `ros_logger_` 发布调试数据
- 检查推理线程是否正常运行
- 验证传感器数据是否正确更新

## 8. 示例：完整控制器实现

参考以下文件了解完整实现:
- `FallStandController`: 倒地起身控制器（包含状态机、轨迹跟踪）
- `AmpWalkController`: AMP 行走控制器（包含 gait receiver、相位管理）

## 9. 常见问题

### Q: 推理线程不执行推理？

A: 检查:
1. `shouldRunInference()` 是否返回 `true`
2. `state_` 是否为 `RUNNING`
3. `sensor_data_updated_` 是否为 `true`

### Q: 动作更新不及时？

A: 检查推理频率是否足够高，或考虑在 `updateImpl()` 中直接调用推理（不推荐）

### Q: 传感器数据不正确？

A: 检查 `preprocessSensorData()` 是否正确实现，特别是 `is_roban_` 时的关节顺序调整

### Q: 如何添加自定义 ROS 服务？

A: 在派生类的 `initialize()` 中添加服务:
```cpp
custom_srv_ = nh_.advertiseService("/your_service", 
                                   &YourController::callback, this);
```

## 10. 总结

`RLControllerBase` 框架提供了一个灵活、可扩展的多控制器架构。通过继承基类并实现必要的虚函数，可以快速添加新的控制器。框架自动处理线程管理、数据同步和状态管理，让开发者专注于控制器特定的逻辑实现。
