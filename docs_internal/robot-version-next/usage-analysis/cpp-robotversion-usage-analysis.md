# C++ 旧 RobotVersion 类使用分析

**目标**：梳理旧 `RobotVersion` 类在 C++ 代码中的使用情况，评估迁移到新 `robot_version` 包的难度。

---

## 一、旧类定义

项目中存在 **两个独立的 RobotVersion 定义**，迁移时需同步替换：

### 1. 主定义：`kuavo_common/common/common.h`

```cpp
class RobotVersion {
    uint16_t Major, Minor, Patch;
public:
    RobotVersion(uint16_t major, uint16_t minor, uint16_t patch = 0);
    static RobotVersion create(int big_number);   // 45 → Major=4, Minor=5
    static bool is_valid(int big_number);
    std::string to_string() const;                 // → "45"
    bool start_with(uint16_t major) const;         // start_with(4) → true for 45
    bool start_with(uint16_t major, uint16_t minor) const;
    uint32_t version_number() const;
    std::string version_name() const;              // → "4.5.0"
    uint16_t major() const;
    uint16_t minor() const;
    uint16_t patch() const;
    bool operator==(const RobotVersion &other) const;
    bool operator!=(const RobotVersion &other) const;
    bool operator<(const RobotVersion &other) const;
    friend std::ostream &operator<<(std::ostream &os, const RobotVersion &version);
};

// 宏定义
#define IS_KUAVO_LEGGED(rb)      (rb.major() == 4 || rb.major() == 5)
#define IS_KUAVO4_LEGGED(rb)     (rb.major() == 4)
#define IS_KUAVO5_LEGGED(rb)     (rb.major() == 5)
#define IS_KUAVO4PRO_LEGGED(rb)  (rb.major() == 4 && (rb.minor() >= 5))
#define IS_ROBAN_LEGGED(rb)      (rb.major() == 1)
#define IS_ROBAN2_LEGGED(rb)     (rb.major() == 1)
```

### 2. 精简定义：`humanoid_interface_drake/common/common.h`

```cpp
struct RobotVersion {
    u_int8_t Major;
    u_int8_t Minor;
    RobotVersion(u_int8_t major, u_int8_t minor) : Major(major), Minor(minor) {}
    std::string versionName() { return std::to_string(Major) + "." + std::to_string(Minor); }
    int versionInt() { return Major * 10 + Minor; }
};
```

---

## 二、旧 API → 新 API 映射

| 旧方法 | 语义 | 新替代 |
|--------|------|--------|
| `RobotVersion::create(int)` | 从整数 45 创建 | `robot_version::RobotVersion(45)` |
| `RobotVersion(4, 5)` | 从 major/minor 创建 | `robot_version::RobotVersion(45)` |
| `rv.major()` | 获取主版本号 | 用宏 `IS_KUAVO4(rv)` 等替代 |
| `rv.minor()` | 获取次版本号 | 用具体宏或 `rv.product()` |
| `rv.to_string()` | 转字符串 "45"（用于日志） | `rv.raw()`（输出新格式字符串） |
| `rv.to_string()` | 转字符串 "45"（用于条件判断） | 用宏替代，如 `IS_ROBAN2V1(rv)` |
| `rv.start_with(1)` | 判断 Roban 系列 | `IS_ROBAN(rv)` |
| `rv.start_with(4)` | 判断 Kuavo4 系列 | `IS_KUAVO4(rv)` |
| `rv.start_with(1, 5)` | 判断 Roban 1.5 | `IS_ROBAN(rv) && rv.product() == "2v1"` (具体看语义) |
| `rv.version_number()` | 版本整数 | **删除，新包不提供此接口** |
| `rv.version_name()` | "4.5.0" | **删除**，日志用 `rv.raw()` |
| `IS_KUAVO_LEGGED(rv)` | Kuavo 双足 | `IS_KUAVO_BIPED(rv)` |
| `IS_KUAVO4_LEGGED(rv)` | Kuavo4 双足 | `IS_KUAVO4(rv)` |
| `IS_KUAVO5_LEGGED(rv)` | Kuavo5 双足 | `IS_KUAVO5(rv)` |
| `IS_KUAVO4PRO_LEGGED(rv)` | 4Pro 双足 | `IS_KUAVO4PRO(rv)` |
| `IS_ROBAN_LEGGED(rv)` | Roban | `IS_ROBAN(rv)` |
| `IS_ROBAN2_LEGGED(rv)` | Roban2 | `IS_ROBAN(rv)` |

> **原则：**
> - 不提供 `to_legacy_version()` 风格的转换函数
> - 旧版本号 int → 新格式的映射仅在构造函数内部使用（兼容旧输入）
> - 旧文件名场景（如 `TotalMassV45`）通过 `ModelInfo.legacy_id` 字段获取（如 `rv.legacy_id()` → `"45"`）
> - 业务代码中的版本判断一律使用宏，日志输出使用 `rv.raw()`

---

## 三、文件分类（共 50 个文件）

排除：protobuf 自动生成文件（`robot_info.pb.cc/h`）、旧类定义文件（`common.h/common.cpp`）、设计文档。

### A. 简单（Simple）— 32 个文件

只创建 + 传递 RobotVersion，无 `major()/minor()` 调用，无条件分支。迁移为机械替换。

典型模式：
```cpp
// 旧
RobotVersion rb_version(3, 4);
int rb_version_int;
nh.getParam("/robot_version", rb_version_int);
rb_version = RobotVersion::create(rb_version_int);
SomeInterface interface(..., rb_version);

// 新
std::string rb_version_str;
nh.getParam("/robot_version", rb_version_str);
robot_version::RobotVersion rb_version(rb_version_str);
SomeInterface interface(..., rb_version);
```

> **注意**：必须改用 `std::string` 读取 `/robot_version` 参数。原因：
> - 旧格式：参数服务器中存的是 int `45`，`getParam(key, int)` 可以读取
> - 新格式：参数服务器中存的是 string `"kuavo-4pro-biped-revo1hand"`，`getParam(key, int)` **会失败**
> - ROS 的 `getParam(key, std::string)` 无论参数实际是 int 还是 string 都能正确取到字符串（`"45"` 或 `"kuavo-4pro-biped-revo1hand"`）
> - 然后交给 `robot_version::RobotVersion` 构造函数内部的 `isdigit()` 判断走旧/新路径

#### humanoid_interface_ros（12 个文件）

| # | 文件 | 使用方式 |
|---|------|----------|
| 1 | `humanoid_interface_ros/src/HumanoidDdpMpcNode.cpp` | create(int) → 传给 HumanoidInterface |
| 2 | `humanoid_interface_ros/src/HumanoidSqpMpcNode.cpp` | create(int) → 传给 HumanoidInterface |
| 3 | `humanoid_interface_ros/src/HumanoidDummyNode.cpp` | create(int) → 传给 HumanoidInterface |
| 4 | `humanoid_interface_ros/src/HumanoidHandCommandNode.cpp` | create(int) → 传给 HumanoidInterfaceDrake |
| 5 | `humanoid_interface_ros/src/HumanoidVRHandCommandNode.cpp` | create(int) → 传给 HumanoidInterfaceDrake |
| 6 | `humanoid_interface_ros/src/HumanoidJoyCommandNode.cpp` | create(int) → 传给 HumanoidInterfaceDrake |
| 7 | `humanoid_interface_ros/src/HumanoidJoyCommandNodeWithArm.cpp` | create(int) → 传给 HumanoidInterfaceDrake |
| 8 | `humanoid_interface_ros/src/HumanoidPoseCommandNode.cpp` | create(int) → 传给 HumanoidInterfaceDrake |
| 9 | `humanoid_interface_ros/src/HumanoidPoseCommandNodeWithArm.cpp` | create(int) → 传给 HumanoidInterfaceDrake |
| 10 | `humanoid_interface_ros/src/newTargetPublisher/HumanoidAutoGaitJoyCommandNodeWithVel.cpp` | create(int) → 传给 HumanoidInterfaceDrake |
| 11 | `humanoid_interface_ros/src/newTargetPublisher/HumanoidJoyCommandNodeWithVel.cpp` | create(int) → 传给 HumanoidInterfaceDrake |
| 12 | `humanoid_interface_ros/src/QuestControlFSMNode.cpp` | create(int) → 传给 HumanoidInterfaceDrake，额外保存 `robot_version_int_` |

#### humanoid_controllers（3 个文件）

| # | 文件 | 使用方式 |
|---|------|----------|
| 13 | `humanoid_controllers/include/humanoid_controllers/humanoidController.h` | 声明 `setupHumanoidInterface(..., RobotVersion)` 虚方法参数 |
| 14 | `humanoid_controllers/src/humanoidController.cpp` | 实现 setupHumanoidInterface，传给 HumanoidInterface |
| 15 | `humanoid_controllers/src/TargetTrajectoriesPublisher.cpp` | create(int) → 传给 HumanoidInterfaceDrake |

#### humanoid_interface（3 个文件）

| # | 文件 | 使用方式 |
|---|------|----------|
| 16 | `humanoid_interface/include/humanoid_interface/HumanoidInterface.h` | 构造函数参数 + 成员变量 `rb_version_` |
| 17 | `humanoid_interface/src/HumanoidInterface.cpp` | 存储 rb_version_，传给 HumanoidInterfaceDrake 和 SwitchedModelReferenceManager |
| 18 | `humanoid_interface/include/humanoid_interface/reference_manager/SwitchedModelReferenceManager.h` | 构造函数参数声明 |

#### humanoid_interface_drake（4 个文件）

| # | 文件 | 使用方式 |
|---|------|----------|
| 19 | `humanoid_interface_drake/include/humanoid_interface_drake/common/common.h` | **独立定义的精简 RobotVersion struct**，需删除 |
| 20 | `humanoid_interface_drake/include/humanoid_interface_drake/humanoid_interface_drake.h` | getInstance/getInstancePtr 参数 + 成员变量 |
| 21 | `humanoid_interface_drake/src/humanoid_interface_drake.cpp` | 构造存储，传给 KuavoCommon |
| 22 | `humanoid_interface_drake/test/test_humanoid_interface_drake.cpp` | 创建测试对象 |

#### hardware 测试 / 示例（7 个文件）

| # | 文件 | 使用方式 |
|---|------|----------|
| 23 | `hardware_node/src/tests/ankleTest/ankle_solver_v17_test.cpp` | 创建+传递 |
| 24 | `hardware_node/src/tests/motorTest/motor_follow_test.cpp` | 创建+传递 |
| 25 | `hardware_node/src/tests/roban_arm_with_revo2_test.cc` | 创建+传递 |
| 26 | `hardware_node/src/tests/hardwareSelfCheck.cpp` | 创建+传递 |
| 27 | `hardware_node/src/tests/hardwareTest.cc` | 创建+传递 |
| 28 | `hardware_node/src/tests/dexhand_controller_test.cc` | 创建+传递 |
| 29 | `hardware_node/src/tests/dexhand/dexhand_controller_test.cc` | 创建+传递 |

#### 其他（3 个文件）

| # | 文件 | 使用方式 |
|---|------|----------|
| 30 | `hardware_plant/examples/hw_wrapped/wrapped/src/hw_wrapped_impl.cc` | 创建+传递到 HardwareParam |
| 31 | `hardware_plant/examples/quick_start/quick_start_example.cpp` | 创建+传递到 HardwareParam |
| 32 | `humanoid_interface/test/AnymalFactoryFunctions.cpp` | 硬编码 `RobotVersion(4, 5)` 传给构造 |

---

### B. 中等（Medium）— 10 个文件

用到 `to_string()` 或简单的版本创建+存储，无复杂分支逻辑。

| # | 文件 | 使用方式 |
|---|------|----------|
| 33 | `kuavo_common/include/kuavo_common/kuavo_common.h` | 声明工厂方法参数 + 成员变量 `rb_version_` |
| 34 | `kuavo_common/tests/kuavo_common_test.cpp` | `RobotVersion(4, 2)` + `to_string()` |
| 35 | `hardware_plant/src/hardware_plant.h` | 成员变量声明，HardwareParam 中默认 `RobotVersion(4, 2)` |
| 36 | `hardware_plant/src/tests/hardware_plant_test.cpp` | `create(int)` + `to_string()` 日志 |
| 37 | `hardware_plant/src/tests/motor_test.cpp` | `create(int)` + 存入 HardwareParam |
| 38 | `hardware_plant/src/tests/wheel_arm_ec_test.cpp` | `create(int)` + `to_string()` 日志 |
| 39 | `humanoid_controllers/src/playBackNodelet.cpp` | `create(int)` → 传给 HumanoidInterface + HumanoidInterfaceDrake |
| 40 | `humanoid_interface_drake/src/humanoid_interface_drake.cpp` | 构造 + 传给 KuavoCommon |
| 41 | `manipulation_nodes/motion_capture_ik/src/ik_ros_uni_cpp_node.cpp` | 读取 int 版本号用于路径拼接（未用 RobotVersion 类） |
| 42 | `manipulation_nodes/motion_capture_ik/src/wheel_ik_ros_uni_cpp_node.cpp` | 同上 |

---

### C. 复杂（Complex）— 8 个文件

有版本分支逻辑，需手工审查每个条件的语义并替换为新宏。

#### 1. `kuavo_common/src/common/kuavo_settings.cpp`

```cpp
// 旧：rb_version.major() > 1 && rb_version.major() < 2（疑似 bug，不可能同时满足）
// 语义：判断 Roban 系列
// 新：IS_ROBAN(rv)
```

方法签名需改：`getEcmasterType(RobotVersion)` → `getEcmasterType(robot_version::RobotVersion)`

#### 2. `kuavo_common/src/kuavo_common.cpp`

```cpp
// 旧
std::string effective_version = rb_version.to_string();
if (rb_version.to_string() == "15") { /* 特殊处理 */ }

// 新：用宏判断语义，用语义字段拼路径
if (IS_ROBAN2V1(rv)) { /* Roban 2v1 的特殊处理 */ }
std::string model = rv.model_name();   // "biped_s14" — 来自内置映射表
std::string config = rv.config_name(); // "kuavo_v14" — 来自内置映射表
```

> 注意：`effective_version` 如果是用于拼接配置文件路径，应改为使用 `robot_version` 包提供的语义字段（`model_name()`、`config_name()` 等），这些字段由 内置映射表定义，而非依赖旧版本号字符串。

#### 3. `hardware_plant/src/actuators_interface.cpp`

```cpp
// 旧：直接读环境变量比较字符串
std::string robot_version = getRobotVersionFromEnv();
bool is_robot_version_53 = (robot_version == "53");
bool is_robot_version_54 = (robot_version == "54");

// 新：使用 RobotVersion 对象
robot_version::RobotVersion rv(std::getenv("ROBOT_VERSION"));
bool is_robot_version_53 = IS_KUAVO5V3(rv);
bool is_robot_version_54 = IS_KUAVO5V4(rv);
```

#### 4. `hardware_node/src/hardware_node.cc`

```cpp
// 旧
hardware_param_.robot_version.start_with(1)  // 判断 Roban

// 新
IS_ROBAN(hardware_param_.robot_version)
```

3 处使用，全部为 `start_with(1)` 即 Roban 判断。

#### 5. `hardware_plant/src/hardware_plant.cc`（未在搜索列表中但有使用）

```cpp
// 旧
rb_version_.start_with(1)                    // Roban 判断
rb_version_.start_with(1, 5)                 // Roban 1.5 具体型号
rb_version_.start_with(1, 6)                 // Roban 1.6 具体型号
rb_version_.minor() <= 6                     // minor 版本比较

// 新
IS_ROBAN(rb_version_)
// start_with(1, 5) / start_with(1, 6) 需要确认对应的具体产品型号后使用宏
```

4 处使用，需逐一确认语义。

#### 6. `humanoid_controllers/src/rl/VMPController.cpp`

```cpp
// 旧
RobotVersion robot_version = RobotVersion::create(robot_version_int);
if (!IS_KUAVO4PRO_LEGGED(robot_version))
    ROS_WARN("... %s ...", robot_version.to_string().c_str());
ROS_INFO("... %s ...", robot_version.to_string().c_str());

// 新
robot_version::RobotVersion robot_version(robot_version_int);
if (!IS_KUAVO4PRO(robot_version))
    ROS_WARN("... %s ...", robot_version.raw().c_str());
ROS_INFO("... %s ...", robot_version.raw().c_str());
```

#### 7. `humanoid_interface/src/reference_manager/SwitchedModelReferenceManager.cpp`

```cpp
// 旧：直接用 int 比较
int robot_version = 0;
nodeHandle_.getParam("/robot_version", robot_version);
if (robot_version < 30) {  // 判断 Roban（版本号 < 30）
    ...
}

// 新：改用 string 读取，构造函数自动处理 int/string 两种格式
std::string rb_version_str;
nodeHandle_.getParam("/robot_version", rb_version_str);
robot_version::RobotVersion rv(rb_version_str);
if (IS_ROBAN(rv)) { ... }
```

#### 8. `manipulation_nodes/motion_capture_ik/src/main_node.cpp`

```cpp
// 需确认具体使用方式
```

#### 9. `demo/grab_box/src/grab_box_demo.cpp`

```cpp
// 通过 HumanoidInterfaceDrake 间接使用，本身无版本逻辑
// 迁移时只需更新 include 和创建方式
```

---

## 四、迁移策略

### 可脚本化的部分（~80%）

以下替换可用脚本批量完成：

```
1.  #include "kuavo_common/common/common.h"  →  #include <robot_version/robot_version.h>
2.  RobotVersion::create(xxx)               →  robot_version::RobotVersion(xxx)
3.  RobotVersion(M, N)                      →  robot_version::RobotVersion(M*10+N)  [需手动确认每个]
4.  IS_KUAVO_LEGGED(rv)                     →  IS_KUAVO_BIPED(rv)
5.  IS_KUAVO4_LEGGED(rv)                    →  IS_KUAVO4(rv)
6.  IS_KUAVO5_LEGGED(rv)                    →  IS_KUAVO5(rv)
7.  IS_KUAVO4PRO_LEGGED(rv)                →  IS_KUAVO4PRO(rv)
8.  IS_ROBAN_LEGGED(rv)                     →  IS_ROBAN(rv)
9.  IS_ROBAN2_LEGGED(rv)                    →  IS_ROBAN(rv)
10. nh.getParam("/robot_version", int变量)   →  nh.getParam("/robot_version", string变量)
```

> **关键变更：getParam 必须改用 string 类型读取**
>
> 旧代码普遍使用 `int` 读取 `/robot_version` 参数：
> ```cpp
> int rb_version_int;
> nh.getParam("/robot_version", rb_version_int);
> ```
> 新格式为字符串（如 `"kuavo-4pro-biped-revo1hand"`），`getParam(key, int)` 会失败。
> 必须改为：
> ```cpp
> std::string rb_version_str;
> nh.getParam("/robot_version", rb_version_str);
> ```
> ROS 的 `getParam(key, std::string)` 无论参数实际是 int `45` 还是 string `"kuavo-4pro-biped-revo1hand"` 都能正确取到字符串。
> 然后交给 `robot_version::RobotVersion` 构造函数内部的 `isdigit()` 自动判断走旧/新路径。

### 需手工审查的部分（~20%，8 个文件）

- `kuavo_settings.cpp` — major() 比较逻辑（含疑似 bug）
- `kuavo_common.cpp` — to_string() == "15" 特殊处理
- `actuators_interface.cpp` — 环境变量字符串比较
- `hardware_node.cc` — start_with(1) × 3 处
- `hardware_plant.cc` — start_with(1), start_with(1,5), start_with(1,6), minor() <= 6
- `VMPController.cpp` — 宏 + to_string()
- `SwitchedModelReferenceManager.cpp` — int < 30 比较
- `humanoid_interface_drake/common/common.h` — 删除独立定义，统一使用新包

### 特殊注意事项

1. **humanoid_interface_drake 有独立的 RobotVersion struct**，必须删除并改用新包
2. **actuators_interface.cpp 直接读环境变量**而非通过 RobotVersion 类，需改为使用新 API
3. **kuavo_settings.cpp 的 `major() > 1 && major() < 2`** 看起来是 bug（不可能同时成立），迁移时需确认原意
4. **QuestControlFSMNode.cpp** 额外保存了 `robot_version_int_` 成员，需确认后续是否仍需要
5. **所有默认值 `RobotVersion(3, 4)`** 出现在 ~12 个节点中，迁移后需统一默认值策略

---

## 五、汇总

| 分类 | 文件数 | 迁移方式 |
|------|:------:|----------|
| 简单（创建+传递） | 32 | 脚本批量替换 |
| 中等（to_string/存储） | 10 | 脚本替换 + 少量手动 |
| 复杂（版本分支逻辑） | 8 | 手工逐文件审查 |
| **合计** | **50** | |

| 删除项 | 说明 |
|--------|------|
| `kuavo_common/common/common.h` 中 RobotVersion 类 + 宏定义 | 被新包替代 |
| `kuavo_common/common/common.cpp` 中实现 | 被新包替代 |
| `humanoid_interface_drake/common/common.h` 整个文件 | 独立精简定义，被新包替代 |
