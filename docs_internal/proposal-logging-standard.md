# 日志输出标准设计方案

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-03-05 | 初稿 |

---

## 1. 概述

### 1.1 背景与目的

**背景：**

当前 kuavo-ros-control 项目的日志输出存在以下问题：
- 日志系统碎片化，混用 ROS 日志、MELO、Boost.Log、std::cout 等多种方式
- 日志格式不统一，各模块输出风格各异，难以解析和检索
- 全英文输出，集成部门和项目部门在功能测试、部署和问题排查时阅读困难
- 缺乏统一的日志编写规范文档，开发人员各自为政

**目的：**

- 设计统一的日志输出标准形式，规范日志编写方式
- INFO 级别日志采用中文输出，便于集成和项目部门快速理解系统状态和定位问题
- 提供清晰的日志编写指南，降低开发人员的学习成本
- 重构现有日志输出，按新标准整理内容
- 在 1.5.0 版本机器人软件中正式交付

---

## 2. 现状分析

当前系统日志输出到以下位置：

| 输出方式 | 输出位置 | 说明 |
|----------|----------|------|
| stdout/stderr | `~/.ros/stdout/{timestamp}/stdout.log` | 通过 `start_node.sh` 脚本重定向，所有节点合并输出 |
| ROS 日志 | `~/.ros/log/` | ROS 日志系统，按节点分文件存储 |
| 终端显示 | 控制台 | `output="screen"` 的节点同时输出到终端 |

### 2.1 关键日志来源

核心日志来源：

```
核心 Nodelet
├── HumanoidControllerNodelet   # 控制核心（humanoid_controllers）
├── HardwareNodeletCXX          # 硬件节点（hardware_node）
└── EkfNodelet                  # 状态估计（humanoid_estimation）

重要独立节点
├── humanoid_sqp_mpc            # 步态 MPC 求解器（humanoid_interface_ros）
├── mobile_manipulator_ctrl     # 运动学 MPC（mobile_manipulator_controllers）
└── motion_capture_ik           # 遥操作 IK（motion_capture_ik）

硬件抽象层（被 HardwareNodelet 调用）
└── hardware_plant
    └── lib/                    # 硬件底层模块库
        ├── canbus_sdk
        ├── dexhand_sdk
        └── ...
```

**核心改造目标：** `HumanoidControllerNodelet` + `HardwareNodeletCXX`，即控制器模块与硬件节点模块。

### 2.2 各层日志现状

| 层级 | 模块 | 日志方式 | 现状 |
|------|------|----------|------|
| 控制核心 | HumanoidControllerNodelet | ROS + cout 混用 | 格式不统一，缺少中文 |
| MPC 求解 | humanoid_sqp_mpc | ROS + cout 混用 | 格式不统一 |
| 硬件节点 | HardwareNodeletCXX | ROS + cout 混用 | 混合调用 hardware_plant |
| 硬件抽象 | hardware_plant | cout/cerr | 格式各异 |
| 底层模块 | canbus_sdk 等 | cout/cerr | 部分已有完善日志 |

### 2.3 存在的问题

**1. 日志方式碎片化**

同一文件混用多种日志方式：

```cpp
// humanoidController.cpp
ROS_INFO("[JoyAutoGait] stop: success: %s", node.c_str());
std::cout << "[keyboard command]: " << Walk_Command << std::endl;
std::cerr << "reset MPC " << std::endl;
```

**2. 日志格式不统一**

```cpp
// 有模块前缀
ROS_INFO("[JoyAutoGait] stop: success: %s", node.c_str());

// 无模块前缀
std::cout << "publish stop message" << std::endl;
```

**3. 中文提示不足**

各层日志均以英文为主，集成和项目部门难以快速理解系统状态。底层驱动库（如 canbus_sdk）虽有较完善的日志结构，但缺少中文提示。

**4. 滥用日志级别**

调试信息使用 INFO 级别输出，导致日志量过大，关键信息被淹没：

```cpp
// 应使用 DEBUG 级别或不打印
ROS_INFO("joint pos: %f, vel: %f", pos, vel);  // 高频循环中输出
ROS_INFO("entering function xyz");              // 调试用途
```

### 2.4 改进需求

1. **统一日志方式** - 收敛日志接口，消除碎片化
2. **统一日志格式** - 统一模块前缀、消息格式
3. **中文支持** - 关键信息使用中文，便于非研发人员阅读
4. **规范日志级别** - 明确各级别使用场景，避免滥用

---

## 3. 设计目标

### 3.1 核心目标

1. **统一性** - 统一日志接口，消除碎片化
2. **可读性** - 关键日志使用中文输出，便于非研发人员快速理解
3. **规范性** - 统一日志格式，包含时间戳、模块名、级别、消息内容
4. **高性能** - 支持异步日志，不阻塞主线程

### 3.2 约束条件

1. **技术选型** - ROS 模块使用 `leju_log` package（支持 ROS 日志和 spdlog 两种后端），非 ROS 模块基于标准库 fprintf
2. **性能** - 日志操作不影响控制循环实时性，避免高频循环中字符串拼接
3. **范围** - 优先改造核心模块（HumanoidControllerNodelet、HardwareNodeletCXX）

---

## 4. 日志规范

### 4.1 级别规范

| 级别 | 说明 | 使用场景 |
|------|------|----------|
| DEBUG | 调试信息 | 关节位置/速度/力矩值、MPC 求解过程、传感器原始数据 |
| INFO | 运行状态 | 系统启动/停止、步态模式切换、硬件连接成功、控制器初始化完成 |
| WARN | 异常警告 | 关节力矩接近限值、通信延迟、IMU 数据抖动、电池电量低 |
| ERROR | 操作失败 | CAN 通信失败、电机使能失败、传感器读取超时、急停触发、硬件故障 |

### 4.2 格式规范

#### 4.2.1 标准格式定义

**标准日志格式：**

```
[时间戳] [级别] [模块名] 消息内容
```

**格式示例：**

```
[14:30:25.123][INFO][HumanoidController] 系统初始化完成
[14:30:25.456][WARN][IMU] IMU 数据延迟超过阈值: 50ms
[14:30:25.789][ERROR][MotorDriver] 电机通信失败，ID: 3
```

#### 4.2.2 模块命名规范

**命名规则：**

1. 使用大驼峰命名法（PascalCase）或全大写（UPPER_CASE）
2. 简洁明了，长度控制在 15 字符以内
3. 能够唯一标识日志来源模块

### 4.3 性能注意事项

#### 4.3.1 高频循环中的日志

控制循环通常以 500Hz~1kHz 运行，在循环内打印日志会严重影响实时性。

**原则：**

1. **禁止在控制循环内使用 INFO/WARN/ERROR 级别日志**
2. **DEBUG 级别日志仅在调试时启用**，生产环境关闭
3. **状态变化时才打印**，而非每个循环都打印

**错误示例：**

```cpp
void controlLoop() {  // 1kHz 循环
    // ❌ 每次循环都打印，1秒产生1000条日志
    LEJU_INFO(LOG_TAG, "关节位置: {}", joint_pos);
}
```

**正确示例：**

```cpp
void controlLoop() {
    // ✅ 仅状态变化时打印
    if (gait_mode != last_gait_mode) {
        LEJU_INFO(LOG_TAG, "步态切换: {} -> {}", last_gait_mode, gait_mode);
        last_gait_mode = gait_mode;
    }

    // ✅ DEBUG 级别，生产环境可关闭
    LEJU_DEBUG(LOG_TAG, "关节位置: {}", joint_pos);
}
```

---

## 5. 改造方案

### 5.1 技术方案选型

**ROS 模块：** 新建 `leju_log` package，提供两种后端封装（ROS 日志、spdlog），统一日志接口。

**非 ROS 模块：** 基于标准库 fprintf 实现，轻量无依赖，各模块独立维护日志头文件。

### 5.2 改造范围

改造涵盖 ROS 模块和非 ROS 模块两类，共计 14 个核心模块，均为 P0 优先级，需在 1.5.0 版本前完成。

**ROS 模块（当前 ROS + cout 混用）：**

- HumanoidControllerNodelet（humanoid_controllers）
- HardwareNodeletCXX（hardware_node）
- humanoid_sqp_mpc（humanoid_interface_ros）

**非 ROS 模块（当前 cout/cerr）：**

- hardware_plant 及其 lib 子模块：
  - hardware_plant
  - canbus_sdk
  - dexhand_sdk
  - EC_Master
  - hipnuc_imu
  - LEJU_CAN_API
  - leju_claw_driver
  - motorevo_controller
  - ruiwo_controller
  - ruiwo_controller_cxx
  - xsens_ros_mti_driver

### 5.3 文件结构

**ROS 模块：** 统一使用 `leju_log` package，无需单独创建日志头文件。

**非 ROS 模块：** 各模块在自己的 include 目录下创建独立的日志头文件，文件命名采用 `xxx_log.h` 格式。

```
# ROS 模块（统一使用 leju_log package）
leju_log/include/leju_log/leju_ros_log.h
leju_log/include/leju_log/leju_spdlog.h

# 非 ROS 模块（hardware_plant 及其子模块）
hardware_plant/include/hardware_plant_log.h
hardware_plant/lib/canbus_sdk/include/canbus_sdk/canbus_log.h
hardware_plant/lib/dexhand_sdk/include/dexhand_sdk/dexhand_log.h
hardware_plant/lib/EC_Master/include/EC_Master/ecmaster_log.h
hardware_plant/lib/hipnuc_imu/include/hipnuc_imu/imu_log.h
hardware_plant/lib/LEJU_CAN_API/include/LEJU_CAN_API/lejucan_log.h
hardware_plant/lib/leju_claw_driver/include/leju_claw_driver/lejuclaw_log.h
hardware_plant/lib/motorevo_controller/include/motorevo_controller/motorevo_log.h
hardware_plant/lib/ruiwo_controller/include/ruiwo_controller/ruiwo_log.h
hardware_plant/lib/ruiwo_controller_cxx/include/ruiwo_controller_cxx/ruiwocxx_log.h
hardware_plant/lib/xsens_ros_mti_driver/include/xsens_ros_mti_driver/xsens_log.h
```

### 5.4 改造内容

**ROS 模块（HumanoidControllerNodelet、HardwareNodeletCXX、humanoid_sqp_mpc）：**

| 改造项 | 改造内容 |
|--------|----------|
| cout/printf/cerr/ROS_* 替换 | 替换为 LEJU_LOG_*（基于 spdlog） |
| 格式统一 | 添加模块前缀 |
| 中文化 | INFO/WARN/ERROR 中文化 |

**非 ROS 模块（hardware_plant 及其 lib 子模块）：**

| 改造项 | 改造内容 |
|--------|----------|
| cout/printf/cerr 替换 | 替换为各模块日志宏（如 `RWLOG_*`、`LOG_*`） |
| 格式统一 | 添加模块前缀和时间戳 |
| 中文化 | INFO/WARN/ERROR 中文化 |

### 5.5 兼容性策略

1. **渐进式迁移** - 新代码使用新日志宏，旧代码逐步替换
2. **头文件兼容** - 新日志宏可与现有 ROS_INFO 等宏共存，不影响未改造代码

### 5.6 leju_log package

`leju_log` 是一个独立的 ROS package，提供统一的日志接口，支持两种后端：

| 后端 | 优点 | 适用场景 |
|------|------|----------|
| ROS 日志 | 支持 rqt 工具、rosbag 记录、团队熟悉 | 常规开发调试 |
| spdlog | 高性能异步、fmt 格式化、灵活配置 | 高频日志、性能敏感场景 |

**目录结构：**

```
leju_log/
├── CMakeLists.txt
├── package.xml
├── include/leju_log/
│   ├── leju_ros_log.h     # ROS 日志封装
│   └── leju_spdlog.h      # spdlog 封装
└── src/
    └── leju_spdlog.cpp    # spdlog 后端实现
```

**ROS 后端（leju_ros_log.h）：**

```cpp
#pragma once
#include <ros/ros.h>

#define LEJU_ROS_DEBUG(tag, fmt, ...) ROS_DEBUG("[%s] " fmt, tag, ##__VA_ARGS__)
#define LEJU_ROS_INFO(tag, fmt, ...)  ROS_INFO("[%s] " fmt, tag, ##__VA_ARGS__)
#define LEJU_ROS_WARN(tag, fmt, ...)  ROS_WARN("[%s] " fmt, tag, ##__VA_ARGS__)
#define LEJU_ROS_ERROR(tag, fmt, ...) ROS_ERROR("[%s] " fmt, tag, ##__VA_ARGS__)
```

**spdlog 后端（leju_spdlog.h）：**

```cpp
#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace leju_log {
    inline std::shared_ptr<spdlog::logger> get_or_create(const char* name) {
        auto logger = spdlog::get(name);
        if (!logger) {
            logger = spdlog::stdout_color_mt(name);
            logger->set_pattern("[%H:%M:%S.%e][%l][%n] %v");
        }
        return logger;
    }
}

// 宏传入名称参数，首次调用自动创建 logger，静态缓存
#define LEJU_DEBUG(name, ...) do { static auto logger = leju_log::get_or_create(name); if (logger) logger->debug(__VA_ARGS__); } while(0)
#define LEJU_INFO(name, ...)  do { static auto logger = leju_log::get_or_create(name); if (logger) logger->info(__VA_ARGS__); } while(0)
#define LEJU_WARN(name, ...)  do { static auto logger = leju_log::get_or_create(name); if (logger) logger->warn(__VA_ARGS__); } while(0)
#define LEJU_ERROR(name, ...) do { static auto logger = leju_log::get_or_create(name); if (logger) logger->error(__VA_ARGS__); } while(0)
```

**ROS 后端使用方式：**

```cpp
// humanoid_controller.h - 头文件定义常量
namespace humanoid_ctrl {
    constexpr const char* LOG_TAG = "HumanoidController";
}

// 任意源文件中使用
#include "humanoid_controller.h"
#include <leju_log/leju_ros_log.h>

LEJU_ROS_INFO(humanoid_ctrl::LOG_TAG, "系统初始化完成");
LEJU_ROS_WARN(humanoid_ctrl::LOG_TAG, "MPC 求解超时: %dms", solve_time_ms);
LEJU_ROS_ERROR(humanoid_ctrl::LOG_TAG, "步态切换失败，当前状态: %s", current_state);
```

**spdlog 后端使用方式：**

```cpp
// humanoid_controller.h - 头文件定义常量
namespace humanoid_ctrl {
    constexpr const char* LOG_NAME = "HumanoidController";
}

// 任意源文件中使用
#include "humanoid_controller.h"
#include <leju_log/leju_spdlog.h>

LEJU_INFO(humanoid_ctrl::LOG_NAME, "系统初始化完成");
LEJU_WARN(humanoid_ctrl::LOG_NAME, "MPC 求解超时: {}ms", solve_time_ms);
LEJU_ERROR(humanoid_ctrl::LOG_NAME, "步态切换失败，当前状态: {}", current_state);
```

### 5.7 ROS 模块改造

humanoid_controllers、hardware_node、humanoid_sqp_mpc 为 ROS 模块，统一使用 `leju_log` package。

**改造步骤：**

1. 在 package.xml 中添加 `<depend>leju_log</depend>`
2. 在 CMakeLists.txt 中添加 `find_package(leju_log REQUIRED)`
3. 在代码中 include `<leju_log/leju_log.h>`
4. 在初始化时调用 `leju_log::init("模块名")`
5. 将 cout/cerr/ROS_* 替换为 LEJU_LOG_* 宏

### 5.8 hardware_plant 改造

hardware_plant 及其 lib 子模块为非 ROS 模块，不依赖 ROS 日志系统，基于标准库 fprintf 实现日志接口。

hardware_plant 本身当前使用 `cout`/`cerr`/`printf` 输出日志，需在 `hardware_plant/include/` 下新建 `hardware_plant_log.h`。

**设计要点：**

1. **轻量实现** - 使用 `static inline` 函数 + 宏封装，无动态内存分配，1024字节固定缓冲区（超长日志自动截断）
2. **输出流分离** - INFO/DEBUG 输出到 stdout，WARN/ERROR 输出到 stderr
3. **保留颜色支持** - 提供带颜色的宏（SUCCESS/WARNING/FAILURE），便于终端快速识别关键信息
4. **模块前缀** - 每个子模块使用独立前缀，如 `[RUIWO]`、`[CANBUS_SDK]` 等

**以 RUIWO 电机模块的日志头文件示例：**

```cpp
#pragma once

#define RUIWO_PRINT_LOG
#ifdef RUIWO_PRINT_LOG

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <sys/time.h>

static inline void __ruiwo_log(FILE* stream, const char* level, const char* tag, const char* fmt, ...) {
    char buffer[1024] = {0};
    int offset = 0;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm* tm_info = localtime(&tv.tv_sec);
    offset += snprintf(buffer + offset, sizeof(buffer) - offset,
        "[%02d:%02d:%02d.%03ld][%s][%s] ",
        tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
        tv.tv_usec / 1000, level, tag);

    va_list args;
    va_start(args, fmt);
    offset += vsnprintf(buffer + offset, sizeof(buffer) - offset, fmt, args);
    va_end(args);

    fprintf(stream, "%s\n", buffer);
}

#define RWLOG_TAG "RUIWO"

// 基础日志宏
#define RWLOG_I(fmt, ...)  __ruiwo_log(stdout, "INFO", RWLOG_TAG, fmt, ##__VA_ARGS__)
#define RWLOG_D(fmt, ...) __ruiwo_log(stdout, "DEBUG", RWLOG_TAG, fmt, ##__VA_ARGS__)
#define RWLOG_W(fmt, ...)  __ruiwo_log(stderr, "WARN", RWLOG_TAG, fmt, ##__VA_ARGS__)
#define RWLOG_E(fmt, ...) __ruiwo_log(stderr, "ERROR", RWLOG_TAG, fmt, ##__VA_ARGS__)

// 带颜色的宏
#define RWLOG_SUCCESS(fmt, ...) RWLOG_I("\033[32m" fmt "\033[0m", ##__VA_ARGS__)
#define RWLOG_WARNING(fmt, ...) RWLOG_W("\033[33m" fmt "\033[0m", ##__VA_ARGS__)
#define RWLOG_FAILURE(fmt, ...) RWLOG_E("\033[31m" fmt "\033[0m", ##__VA_ARGS__)

#else

#define RWLOG_I(...)
#define RWLOG_D(...)
#define RWLOG_W(...)
#define RWLOG_E(...)
#define RWLOG_SUCCESS(...)
#define RWLOG_WARNING(...)
#define RWLOG_FAILURE(...)

#endif // RUIWO_PRINT_LOG
```

各模块使用独立的函数名、宏前缀和 TAG，保持与现有实现一致。已有日志实现的模块（如 motorevo_controller 使用 `RWLOG_*`、canbus_sdk 使用 `LOG_*`）沿用原有命名；新增日志的模块参照上述示例，以模块名为基础定义。

### 5.9 日志级别配置

#### 5.9.1 ROS 模块（leju_log）

**spdlog 后端：** 支持运行时动态调整日志级别。

```cpp
// 全局设置
spdlog::set_level(spdlog::level::debug);

// 单个 logger 设置
auto logger = spdlog::get("HumanoidController");
if (logger) {
    logger->set_level(spdlog::level::warn);
}
```

**ROS 后端：** 通过 ROS 日志级别配置，支持 `rqt_logger_level` 动态调整。

```bash
# 命令行设置
rosservice call /node_name/set_logger_level "logger: 'ros' level: 'debug'"
```

#### 5.9.2 非 ROS 模块（hardware_plant 及 lib）

**当前限制：** **基于 fprintf 的日志实现不支持运行时级别调节，仅支持编译时开关。**

```cpp
// 编译时开关：注释掉此行可关闭该模块所有日志
#define RUIWO_PRINT_LOG
```

**设计权衡：**

| 方案 | 优点 | 缺点 |
|------|------|------|
| **fprintf（当前方案）** | 零依赖、轻量、编译简单、与现有代码风格一致 | 不支持运行时级别调节 |
| spdlog | 功能丰富、支持运行时调节 | 引入第三方依赖、增加编译复杂度、底层模块需保持简单 |

**选择 fprintf 的原因：**

1. **零依赖** - hardware_plant 及其 lib 子模块是底层硬件抽象层，需保持最小依赖，便于独立编译和移植
2. **编译简单** - 无需配置 spdlog 的 CMake 依赖链，避免跨模块依赖问题
3. **与现有实现一致** - 部分模块（如 canbus_sdk）已有成熟的 fprintf 日志实现，统一风格降低改造成本
4. **实际需求有限** - 底层模块日志主要用于启动阶段和故障诊断，运行时动态调级的需求较弱

**当前方案的缺点：**

1. **不支持运行时级别调节** - 只能通过编译时宏开关控制，无法在程序运行时动态调整日志级别
