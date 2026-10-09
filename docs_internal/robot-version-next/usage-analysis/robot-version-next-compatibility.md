# ROBOT_VERSION 新旧格式兼容方案

**前置文档**：
- [新格式定义](robot-version-next-design-scheme1.md)
- [robot_version 包规格](robot-version-next-package.md)
- [现有使用场景详细梳理](robot-version-usage-analysis.md)

---

## 核心问题

`ROBOT_VERSION` 从 `45` 变成 `kuavo-4pro-biped-revo1hand` 后，现有代码中哪些写法会崩、怎么兼容、最终改成什么样。

���面按使用场景逐一说明。每个场景先概述现有用法和涉及范围，再说明兼容方案。详细文件清单见 [使用场景梳理](robot-version-usage-analysis.md)。

---

## 影响范围

| 类别 | 文件数 |
|------|:------:|
| Launch | ~55 |
| C++ | ~87 |
| Python | ~54 |
| Shell | ~22 |
| Docker / Service / CI | 8 |
| CMake（死代码） | 1 |

**33 个唯一版本号**：11, 13, 14, 16, 17, 40-49, 50-55, 60-63, 100045, 100049, 200049, 300049, 400049

---

## 场景 1：资源路径拼接

### 现有用法

版本号直接拼进文件路径，是最大的使用场景（~60%），四种语言均存在。三种前缀模式：

| 前缀 | 用途 | 目录数 |
|------|------|:------:|
| `biped_s${V}/` | 模型目录（URDF、mesh、xml 场景） | 30 |
| `kuavo_v${V}/` | 配置目录（kuavo.json、MPC、RL 参数） | 32 |
| `kuavo_s${V}/` | 轮式专用配置（仅 v60-v63） | 4 |

主要路径模式：

| 路径模式 | 使用位置 |
|---------|---------|
| `models/biped_s${V}/urdf/biped_s${V}.urdf` | Launch(30+), C++, Python |
| `models/biped_s${V}/urdf/drake/biped_v3_arm.urdf` | Python（IK/规划） |
| `models/biped_s${V}/meshes/${filename}` | C++（碰撞检测） |
| `config/kuavo_v${V}/kuavo.json` | Launch, Shell, C++, Python |
| `config/kuavo_v${V}/mpc/task.info` | Launch |
| `config/kuavo_v${V}/rl/*.info` | Launch, C++ |
| `config/kuavo_s${V}/task.info` | Launch（轮式） |
| `TotalMassV${V}`, `.urdf_md5_v${V}` | Shell（运行时数据） |
| `/var/ocs2/kuavo_v${V}` | Launch, Shell（运行时缓存） |

涉及 ~55 个 Launch、~17 个 C++、~37 个 Python、~5 个 Shell 文件。

### 会崩的写法

版本号直接拼进路径，字符串一换就找不到目录了：

```
biped_s45/urdf/biped_s45.urdf        ✓ 存在
biped_skuavo-4pro-biped-revo1hand/   ✗ 不存在
```

这是影响面最大的场景，涉及 ~130 处，四种语言都有。

### 兼容方式

资源目录不改名。`robot_version` 包提供映射函数，输入任意格式，输出目录名：

```
输入 "45"                        → model_name() = "biped_s45",  config_name() = "kuavo_v45"
输入 "kuavo-4pro-biped-revo1hand" → model_name() = "biped_s45",  config_name() = "kuavo_v45"
```

### 各语言改法

**Launch**（改动量最大，~55 个文件）：

```xml
<!-- 旧 -->
<arg name="urdfFile" value="$(find kuavo_assets)/models/biped_s$(arg robot_version)/urdf/biped_s$(arg robot_version).urdf"/>

<!-- 新：顶部定义一次，后面用 $(arg) 引用 -->
<arg name="model_name" default="$(eval __import__('robot_version').model_name())"/>
<arg name="urdfFile" value="$(find kuavo_assets)/models/$(arg model_name)/urdf/$(arg model_name).urdf"/>
```

**C++**（~15 个文件直接拼路径）：

```cpp
// 旧
std::string urdf = kuavo_asset_path + "/models/biped_s" + robot_version + "/urdf/...";

// 新
auto rv = robot_version::RobotVersion(robot_version_str);
std::string urdf = kuavo_asset_path + "/models/" + rv.model_name() + "/urdf/...";
```

**Python**（~35 个文件）：

```python
# 旧
urdf_path = f"{kuavo_assets_path}/models/biped_s{robot_version}/urdf/..."

# 新
from robot_version import RobotVersion
rv = RobotVersion(robot_version)
urdf_path = f"{kuavo_assets_path}/models/{rv.model_name()}/urdf/..."
```

**Shell**（~5 个文件）：

```bash
# 旧
config_file="kuavo_v${ROBOT_VERSION}/kuavo.json"

# 新
CONFIG_NAME=$(python3 -c "from robot_version import RobotVersion; print(RobotVersion.from_env().config_name())")
config_file="${CONFIG_NAME}/kuavo.json"
```

### 运行时数据文件

`TotalMassV45`、`.urdf_md5_v45` 等运行时数据文件统一改用 `robot_id` 作为后缀。已部署的旧文件通过符号链接兼容：

```bash
# 统一用 robot_id
ROBOT_ID=$(python3 -c "from robot_version import RobotVersion; print(RobotVersion.from_env().robot_id)")
MASS_FILE="TotalMassV${ROBOT_ID}"   # → TotalMassVkuavo-4pro-biped-revo1hand

# 旧机器人上已有 TotalMassV45，建符号链接：
# TotalMassVkuavo-4pro-biped-revo1hand → TotalMassV45（迁移脚本一次性执行）
```

### kuavo_s 特殊前缀

轮式版本（60-63）的配置目录当前用 `kuavo_s${V}` 前缀。在 Phase 0 前置清理中统一重命名为 `kuavo_v${V}`，之后所有版本统一使用 `config_name()` 一个函数。

---

## 场景 2：版本判断 / 条件分支

### 现有用法

用数值比较或字符串匹配走不同代码分支，是第二大使用场景（~25%）。核心判断维度：

**Launch 中的数值比较**：

| 表达式 | 语义 | 涉及文件 |
|--------|------|---------|
| `>= 40` | 是 Kuavo？ | load_kuavo_real, gazebo-sim 等 |
| `>= 50` | 是 Kuavo 5？ | mobile_manipulator_controller |
| `>= 30` | 支持 mobile manipulator？ | load_kuavo_real, mujoco_sim, isaac_sim |
| `== 15` | v15→v14 别名 | robot_version_manager + 5 个 launch |
| `== 60` / `60<=V%100<70` | 轮式？ | load_kuavo_real_wheel, gazebo-sim, mujoco |
| `!= 53` | 排除 v53 音乐 | load_kuavo_real |
| `52<=V%100<=59` | LED Strip 新版本 | set_led_mode |
| 链式 `>=50/>=40/>=10` | 分层配置选择 | mobile_manipulator_controller (3 个) |

**C++ 版本判断方式**：

| 方式 | 示例 |
|------|------|
| `RobotVersion::create(int)` + 宏（`IS_KUAVO_LEGGED` 等） | humanoidController, VMPController |
| `nh.getParam(int)` + 整数比较 | AmpWalkController, SwitchedModelReferenceManager |
| `std::getenv` + `stoi` + 整数比较 | motor_follow_test, ankle_solver_v17_test |
| `std::getenv` + 字符串比较（`== "53"`） | actuators_interface（EtherCAT 配置） |
| `std::getenv` + 首字符（`[0] == '1'`） | ruiwo_actuator（Roban 检测） |
| `start_with(1)` / `start_with(1, 5)` | hardware_node, hardware_plant |
| `major() <= 2` / `!= 1` / `== 1` | HumanoidJoyCommandNodeWithVel（多处） |
| `robot_version == 40` / `== 45` | isaac_sim_system（6 处精确匹配） |
| `int / 10` / `% 10` 整数运算 | AmpWalkController, DepthWalkController |

**Python 主版本提取模式**（6 个 SDK 核心文件）：

```python
robot_version_major = (int(robot_version) // 10) % 10
if robot_version_major == 1:   # Roban → 4-DOF 臂, 有腰
elif robot_version_major == 4: # Kuavo4 → 7-DOF 臂, 无腰
elif robot_version_major == 5: # Kuavo5 → 7-DOF 臂, 有腰
```

**Shell 正则/前缀判断**（~12 个文件）：`^1[0-9]$`（Roban）、`^5[0-9]$`（Kuavo5）、`^6[0-9]$`（轮式）、`${V:0:2}`（前缀提取）。

### 会崩的写法

所有数值比较和数字正则都会失败：

```python
int("kuavo-4pro-biped-revo1hand")  # ValueError
```

```cpp
std::stoi("kuavo-4pro-biped-revo1hand")  // std::invalid_argument
```

```bash
[[ "kuavo-4pro-biped-revo1hand" =~ ^1[0-9]$ ]]  # 永远 false
```

### 兼容方式

`RobotVersion` 类把旧的数字编码翻译成语义字段。无论输入 `45` 还是 `kuavo-4pro-biped-revo1hand`，都能回答同样的问题：

| 问题 | 旧写法 | 新写法 | 两种输入结果一致 |
|------|--------|--------|:---:|
| 是 Kuavo 吗？ | `major() >= 4` / `>= 40` | `IS_KUAVO(rv)` / `rv.is_kuavo` | ✓ |
| 是 Roban 吗？ | `major() == 1` / `[0]=='1'` / `^1[0-9]$` | `IS_ROBAN(rv)` / `rv.is_roban` | ✓ |
| 是轮式吗？ | `major() == 6` / `60<=v%100<70` | `IS_KUAVO_WHEELED(rv)` / `rv.is_wheeled` | ✓ |
| 是 Kuavo 5 吗？ | `major() == 5` / `>= 50` | `IS_KUAVO5(rv)` / `rv.product[0]=='5'` | ✓ |
| 是 4 Pro 吗？ | `major()==4 && minor()>=5` | `IS_KUAVO4PRO(rv)` | ✓ |
| 有灵巧手吗？ | `== "49"` / `== "45"` | `HAS_REVO1HAND(rv)` | ✓ |
| 有夹爪吗？ | `== "47"` | `HAS_LEJUCLAW(rv)` | ✓ |
| 是展厅版吗？ | `>= 100000` / `% 10000` | `HAS_DUMMY(rv)` | ✓ |
| 是 v53 T26 电机？ | `== "53"` | `IS_KUAVO5V3(rv)` | ✓ |

### 各语言改法

**Launch**：

```xml
<!-- 旧 -->
<group if="$(eval arg('robot_version') >= 40)">

<!-- 新：用布尔 arg（在 robot_version_manager.launch 中定义） -->
<group if="$(arg is_kuavo)">
```

**C++**：

```cpp
// 旧
int rv; nh.getParam("/robot_version", rv);
RobotVersion rb = RobotVersion::create(rv);
if (IS_KUAVO_LEGGED(rb)) { ... }
if (rb.major() == 1) { is_roban = true; }

// 新
std::string rv_str; nh.getParam("/robot_version", rv_str);
robot_version::RobotVersion rv(rv_str);
if (IS_KUAVO_BIPED(rv)) { ... }
if (IS_ROBAN(rv)) { is_roban = true; }
```

**Python**：

```python
# 旧
robot_version_major = (int(robot_version) // 10) % 10
if robot_version_major == 1:  # Roban
    arm_dof = 4
elif robot_version_major == 4:  # Kuavo 4
    arm_dof = 7

# 新
from robot_version import RobotVersion
rv = RobotVersion(robot_version)
if rv.is_roban:
    arm_dof = 4
elif rv.product[0] == '4':
    arm_dof = 7
```

**Shell**：

```bash
# 旧
if [[ "$ROBOT_VERSION" =~ ^1[0-9]$ ]]; then echo "Roban"; fi

# 新
BRAND=$(python3 -c "from robot_version import RobotVersion; print(RobotVersion.from_env().brand)")
if [[ "$BRAND" == "roban" ]]; then echo "Roban"; fi
```

### v53 音乐排除

`load_kuavo_real.launch` 中排除 v53（与上位机冲突）。迁移后用 product 判断：
```xml
<!-- 旧 -->
<group if="$(eval arg('robot_version') != 53)">
<!-- �� -->
<group unless="$(eval __import__('robot_version').RobotVersion.from_env().product == '5V3')">
```

### LED 版本范围（52-59）

`set_led_mode.launch` 中 v52-v59 使用新 LED Strip 服务。迁移后需要在 Launch 中用 product 字段判断。

### 版本兼容回退（motor_follow_test.cpp）

找不到当前版本配置时回退到最近已知版本。迁移后用 `config_name()` 查找，回退逻辑不变。

### 硬编码版本的 Launch 文件

部分 Launch 文件硬编码了特定版本（如 `load_kuavo_mujoco_sim_fixed.launch` 固定 v46、`humanoid_arm_trajectort_rviz.launch` 默认 v42），过渡期不影响，最终态改为新格式字符串。

### Protobuf 定义

`robot_info.pb.cc/h` 中有 protobuf 自动生成的 `robot_version` 字段处理代码，不需要手动迁移。

### CMake 死代码

`hardware_node/CMakeLists.txt` 中的 `ROBOT_VERSION_INT` 编译宏定义，整个代码库无 C++ 文件使用，直接删除。

---

## 场景 3：ROS 参数服务器

### 现有用法

`robot_version_manager.launch` 写入 `/robot_version` 为 int，~20 个 C++ 文件和 ~2 个 Python 文件读取。

| 语言 | 读取方式 | 文件数 |
|------|---------|:------:|
| C++ | `nh.param("/robot_version", int_var, 45)` | ~12（humanoid_interface_ros 节点） |
| C++ | `controllerNh_.getParam("/robot_version", int)` | ~5（控制器） |
| C++ | `ros::param::get("/robot_version", int)` | ~3（其他） |
| Python | `rospy.get_param('/robot_version')` | 1（SDK param.py） |
| Python | `roslibpy.Param(client, 'robot_version').get()` | 1（WebSocket SDK） |

所有 C++ 读取方都假设 `/robot_version` 是 **int 类型**。

### 会崩的写法

所有 C++ 代码用 `int` 类型读取 `/robot_version`：

```cpp
int rv;
nh.getParam("/robot_version", rv);  // 参数是 string 时：读取失败，rv 保持默认值
```

### 兼容方式

**写入端**（`robot_version_manager.launch`）：

```xml
<!-- /robot_version 保持 int，旧代码零改动 -->
<param name="robot_version" value="$(eval __import__('robot_version').legacy_int())"/>

<!-- 新增 /robot_version_next string，新代码读这个 -->
<param name="robot_version_next" type="string"
       value="$(eval __import__('robot_version').robot_id())"/>
```

`legacy_int()` 返回旧数字（`45`），新格式专属机器人返回 `0`。

**读取端**：

旧代码不用改，继续读 `/robot_version` int。新代码改为读 `/robot_version_next` string：

```cpp
// 旧代码（不用改）
int rv; nh.getParam("/robot_version", rv);

// 新代码
std::string rv_str;
nh.param<std::string>("/robot_version_next", rv_str, "");
robot_version::RobotVersion rv(rv_str);
```

### 新增语义参数

在 `robot_version_manager.launch` 中额外发布语义参数，供下游 Launch 和节点使用：

```
/robot_version          int      45（保持不动，旧代码继续读；新格式输入时通过映射表填数字，无旧编号填 0）
/robot_version_next     string   "kuavo-4pro-biped-revo1hand"（新代码读这个）
/rv/brand               string   "kuavo"
/rv/product             string   "4pro"
/rv/locomotion          string   "biped"
```

---

## 场景 4：环境变量直接读取

### 现有用法

部分代码绕过 ROS 参数服务器，直接读取 `ROBOT_VERSION` 环境变量。

| 语言 | 文件数 | 典型场景 |
|------|:------:|---------|
| C++ | ~8 | 硬件驱动（actuators_interface, ruiwo_actuator）、碰撞检测、测试程序 |
| Python | ~20 | 标定工具（joint_cali/ 8 个）、IK 脚本、h12pro 回调、setup 工具 |
| Shell | ~15 | setup 脚本、stress_test、deploy_autostart、update_mass |

特殊情况：`Hardware_tool.py` 解析 `~/.bashrc` 中的 `export ROBOT_VERSION=` 行而非直接读环境变量。

### 会崩的写法

```cpp
int v = std::stoi(std::getenv("ROBOT_VERSION"));    // 抛异常
```

```python
major = (int(os.environ.get('ROBOT_VERSION')) // 10) % 10  # ValueError
```

```cpp
bool is_roban = (std::string(env)[0] == '1');  // 'k' != '1'，逻辑错误但不崩
```

### 兼容方式

统一改为通过 `RobotVersion` 构造函数读取，它接受任意格式：

**C++**（~8 个文件）：

```cpp
// 旧
const char* env = std::getenv("ROBOT_VERSION");
bool is_roban = (std::string(env)[0] == '1');

// 新
auto rv = robot_version::RobotVersion::from_env("45");
bool is_roban = IS_ROBAN(rv);
```

**Python**（~20 个文件）：

```python
# 旧
robot_version = os.environ.get('ROBOT_VERSION', '45')
urdf_path = f"models/biped_s{robot_version}/urdf/..."

# 新
from robot_version import RobotVersion
rv = RobotVersion.from_env('45')
urdf_path = f"models/{rv.model_name()}/urdf/..."
```

### Hardware_tool.py 特殊情况

该工具解析 `~/.bashrc` 中的 `export ROBOT_VERSION=` 行来获取版本号。迁移后需要适配两种格式：

```python
# 解析出的值可能是 "45" 或 "kuavo-4pro-biped-revo1hand"
# 直接传给 RobotVersion 构造函数即可，它自动识别
rv = RobotVersion(parsed_value)
```

---

## 场景 5：Docker / systemd Service / CI

### 现有用法

| 类别 | 文件 | 内容 |
|------|------|------|
| Docker | `Dockerfile` | `echo 'export ROBOT_VERSION=42' >> /root/.bashrc` |
| Docker | `run.sh` / `run_with_gpu.sh` | `-e ROBOT_VERSION=42` |
| Service | `roban_joy_monitor.service` | `Environment=ROBOT_VERSION=14` |
| Service | `ocs2_h12pro_*.service` | `Environment=ROBOT_VERSION=40` |
| Service | `websocket_start.service` | `Environment=ROBOT_VERSION=42` |
| CI | `.gitlab-ci.yml` | `export ROBOT_VERSION=34`（Roban）/ `=40`（Kuavo） |

Service 文件中的值会被 `deploy_autostart.sh` / `websocket_deploy_script.sh` 通过 `sed` 替换为实际值。

### 会崩吗？

不会崩。这些文件中的 `ROBOT_VERSION=42` 是固定默认值，且会被部署脚本通过 `sed` 替换为实际值。只要 `sed` 替换的值是合法的（旧数字或新字符串都行），下游代码就能处理。

### 兼容方式

**过渡期**：保持旧数字不动，什么都不用改。

**最终态**：默认值改为新格式字符串，可读性更好。

```ini
# 旧
Environment=ROBOT_VERSION=42

# 最终（可选，不急）
Environment=ROBOT_VERSION=kuavo-4V2-biped-none
```

**CI 配置**：同理，`export ROBOT_VERSION=40` 过渡期不用动。最终可改为新格式，并增加新格式测试用例。

---

## 场景 6：特殊情况

以下是散布在代码库中的各种特殊处理逻辑，迁移时需要逐一对应。

### v15→v14 别名

6 个 Launch 文件和 2 个脚本有 `if v==15 then 用 v14 资源` 的特殊处理（目录 `biped_s15` 不存在）。

迁移后映射表直接处理：
```
legacy_map: 15 → "roban-2v1-biped-none"
model_map:  "roban-2v1-biped" → ModelInfo("biped_s14", "kuavo_v14", "14")
```
`model_name()` 对输入 `15` 直接返回 `biped_s14`，所有 `if v==15` 分支可以删除。

### 展厅版（100045, 100049, 200049, 300049, 400049���

旧代码用 `% 10000` 或 `Patch` 字段处理。新格式直接用 `end_effector` 字段区分：

| 旧版本号 | 新格式 | 区分点 |
|---------|--------|--------|
| 45 | `kuavo-4pro-biped-revo1hand` | end_effector=revo1hand |
| 100045 | `kuavo-4pro-biped-dummy` | end_effector=dummy |
| 47 | `kuavo-4pro-biped-lejuclaw` | end_effector=lejuclaw |

判断展厅版：`HAS_DUMMY(rv)` 替代 `version >= 100000`。

### G12 轮臂模式

`MobileManipulatorJoyCommandNode.cpp` 同时读 ROS 参数和环境变量检测 `v>=60 && joystick==h12`。迁移后统一用 `IS_KUAVO_WHEELED(rv)`。

---

## 过渡期行为总结

在迁移过程中（Phase 1~4），代码库中会同时存在旧写法和新写法。`RobotVersion` 构造函数的双格式兼容保证了这一点：

| 环境变量值 | 旧代码（未迁移） | 新代码（已迁移） |
|-----------|:---:|:---:|
| `45`（旧数字） | 正常 | 正常（构造函数 isdigit→查映射表） |
| `kuavo-4pro-biped-revo1hand`（新字符串） | **崩溃** | 正常（构造函数直接解析） |

因此过渡期的约束是：**环境变量必须保持旧数字格式，直到所有读取方都迁移到新 API**。`robot_version_manager.launch` 作为中央枢纽最先迁移（Phase 2），它内部通过 `robot_version` 包将任意格式转换为语义字段，下游 Launch 文件通过 `$(arg model_name)` 等引用，不直接接触原始版本号。

C++ 和 Python 节点的迁移（Phase 4）可以逐个进行。只要某个节点还在用 `getParam(int)` 或 `int(os.environ.get(...))` 读取，那该环境就不能切换到新格式字符串。全部迁移完成后，新格式字符串才能安全使用。

---

## 一句话总结

旧格式输入 → `RobotVersion` 构造函数查映射表 → 得到和新格式输入完全一致的语义字段。

不需要入口预处理脚本，不需要环境变量转换，不需要在两套格式之间同步状态。`RobotVersion` 构造函数就是唯一的兼容桥梁。
