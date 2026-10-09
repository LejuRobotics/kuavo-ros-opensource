# robot_version 包规格说明

## 1. 定位

独立、原子的 ROS package，提供 `ROBOT_VERSION` 的解析、映射和语义查询。

**核心原则**：

- **独立性**：不依赖项目中任何其他包，其他包单向依赖它
- **零 ROS 依赖**：纯 C++ / Python 库。`ROBOT_VERSION` 本身就是环境变量，`std::getenv()` / `os.environ.get()` 即可读取，不经过 ROS 参数服务器
- **双格式兼容**：同一接口接受旧格式（数字 `45`）和新格式（字符串 `kuavo-4pro-biped-revo1hand`），构造函数内部自动判断
- **映射表编译时嵌入**：硬编码在 `.h` 和 `.py` 中，不依赖外部 YAML 文件，零运行时文件查找开销
- **严格模式**：未知数字、非法字符串、空值一律抛异常，不静默通过
- **只做解析和映射**：不含业务逻辑（`has_waist`、`arm_dof` 等），调用方自行根据字段判断

---

## 2. 目录结构

```
src/robot_version/
├── CMakeLists.txt                         # catkin 包，无外部依赖
├── package.xml
├── setup.py                               # catkin_python_setup() 所需
├── include/robot_version/
│   └── robot_version.h                    # C++ 头文件：enum、类、宏、映射表
├── src/
│   └── robot_version.cpp                  # C++ 实现
├── python/robot_version/
│   ├── __init__.py                        # 顶层函数（供 Launch $(eval) 调用）
│   └── core.py                            # Python 实现（类、enum、映射表）
└── test/
    ├── test_robot_version.cpp             # C++ 单元测试
    ├── test_robot_version.py              # Python 单元测试
    └── test_mapping.py                    # 映射表 C++/Python 一致性测试
```

---

## 3. Enum 定义

有限闭合集合使用 enum，开放集合使用 string。

### 使用 enum 的字段

| 字段 | enum 名 | 值 |
|------|---------|------|
| brand | `Brand` | `Kuavo`, `Roban` |
| locomotion | `Locomotion` | `Biped`, `Wheeled` |
| end_effector | `EndEffectorType` | `None`, `Lejuclaw`, `Revo1hand`, `Revo2hand`, `LinkhandO6`, `Revo1touch`, `Dummy` |
| camera | `CameraModel` | `None`, `D435`, `D405`, `Gemini330`, `Gemini335L` |

### 使用 string 的字段

| 字段 | 原因 |
|------|------|
| `product` | 型号持续新增（`4pro`、`5V2`、`5V3`...），enum 维护成本过高 |
| `tags` | 开放集合，描述性标签 |

### C++

```cpp
namespace robot_version {

enum class Brand { Kuavo, Roban };
enum class Locomotion { Biped, Wheeled };
enum class EndEffectorType {
    None, Lejuclaw, Revo1hand, Revo2hand, LinkhandO6, Revo1touch, Dummy
};
enum class CameraModel {
    None, D435, D405, Gemini330, Gemini335L
};

}  // namespace robot_version
```

### Python

使用 `enum.Enum` + **字符串值**，不用 `IntEnum`。`.value` 直接是格式字符串中的值，解析时 `Brand("kuavo")` 即可构造。

```python
class Brand(enum.Enum):
    Kuavo = "kuavo"
    Roban = "roban"

class Locomotion(enum.Enum):
    Biped = "biped"
    Wheeled = "wheeled"

class EndEffectorType(enum.Enum):
    NoneType = "none"
    Lejuclaw = "lejuclaw"
    Revo1hand = "revo1hand"
    Revo2hand = "revo2hand"
    LinkhandO6 = "linkhandO6"
    Revo1touch = "revo1touch"
    Dummy = "dummy"

class CameraModel(enum.Enum):
    NoneType = "none"
    D435 = "D435"
    D405 = "D405"
    Gemini330 = "Gemini330"
    Gemini335L = "Gemini335L"
```

---

## 4. RobotVersion 类

基于参考实现 `robot_version_next_scheme1.hpp` / `.py`，补充兼容层。

### 4.1 构造 / 工厂方法

构造函数内部自动判断输入类型：全数字走旧格式查映射表，否则走新格式正则解析。调用方无需关心格式。

```cpp
// C++
class RobotVersion {
public:
    explicit RobotVersion(const std::string& input);   // "45" 或 "kuavo-4pro-biped-revo1hand" 均可
    static RobotVersion from_env(const std::string& default_val = "");
};
```

```python
# Python
class RobotVersion:
    def __init__(self, version_input):   # int 45、str "45"、str "kuavo-4pro-biped-revo1hand" 均可
        raw = str(version_input).strip()
        if raw.isdigit():
            # 查 _LEGACY_MAP → 得到新格式字符串 → 正则解析
        else:
            # 直接正则解析

    @staticmethod
    def from_env(default: str = "") -> "RobotVersion": ...
```

处理流程：

| 输入 | isdigit | 处理 | 结果 |
|------|---------|------|------|
| `45` (int) | `"45"` → True | 查映射表 → `"kuavo-4pro-biped-revo1hand"` → 正则解析 | brand=kuavo, product=4pro |
| `"kuavo-4pro-biped-revo1hand+upper_nuci7"` | False | 直接正则解析 | brand=kuavo, product=4pro, tags={upper_nuci7} |
| `99` (未知数字) | `"99"` → True | 查映射表无结果 | 抛异常 |
| `"hello"` (非法) | False | 正则不匹配 | 抛异常 |

### 4.2 语义字段（enum 返回值）

```cpp
// C++
Brand brand() const;
Locomotion locomotion() const;
EndEffectorType left_end() const;
EndEffectorType right_end() const;
CameraModel cam_head() const;
CameraModel cam_waist() const;
CameraModel cam_left_wrist() const;
CameraModel cam_right_wrist() const;
```

```python
# Python（property）
rv.brand        # Brand.Kuavo
rv.locomotion   # Locomotion.Biped
rv.left_end     # "revo1hand"
rv.right_end    # "revo1hand"
```

### 4.3 String 字段

```cpp
// C++
const std::string& product() const;          // "4pro" | "5V2" | "2v1"
const std::string& end_effector_str() const; // 原始字符串 "revo1hand" | "revo1hand_lejuclaw"
const std::string& raw() const;              // 新格式完整字符串
const std::set<std::string>& tags() const;
```

### 4.4 语义路径字段（来自内置映射表）

同一 base model 下不同末端执行器变体共享相同的路径字段。索引键为 `brand-product-locomotion`（不含 end_effector）。

```cpp
// C++
std::string model_name() const;   // "biped_s45"  — 模型目录名
std::string config_name() const;  // "kuavo_v45"  — 配置目录名
const std::string& robot_id() const;  // "kuavo-4pro-biped-revo1hand" — 运行时数据文件后缀等
```

`legacy_int()` 返回旧版本号整数（如 `45`），用于填充 `/robot_version` int 参数保持旧代码兼容；新格式专属机器人返回 `0`。不提供 `to_legacy_version()` 等其他旧格式接口。运行时数据文件统一用 `robot_id` 命名，旧文件通过符号链接兼容。

### 4.5 标签查询

```cpp
// C++
bool has_tag(const std::string& tag) const;
```

末端执行器判断不提供类方法，统一通过宏（见第 6 节）。

### 4.6 硬件信息（从 tags 提取）

```cpp
// C++
std::string get_upper_computer() const;   // 遍历 tags 找 "upper_" 前缀 → "nuci7"
std::string get_lower_computer() const;   // 遍历 tags 找 "lower_" 前缀 → "amr7000"
```

### 4.7 `__str__` / `operator<<`

输出 `raw()` 值（新格式完整字符串），日志直接可用。不需要额外的 `to_string()` 方法。

---

## 5. 映射表

映射表硬编码在 C++ 和 Python 中，新增机型时两处各加一行。

### 5.1 Legacy 兼容表（旧 int → 新格式字符串）

仅在构造函数内部使用，将旧数字转为新格式字符串后再走统一解析路径。

```cpp
static const std::map<int, std::string>& legacy_map() {
    static const std::map<int, std::string> m = {
        // kuavo-4 系列
        {40,     "kuavo-4-biped-none"},
        {42,     "kuavo-4V2-biped-none"},
        // kuavo-4pro 系列（同 base model，不同末端）
        {45,     "kuavo-4pro-biped-revo1hand"},
        {47,     "kuavo-4pro-biped-lejuclaw"},
        {100045, "kuavo-4pro-biped-dummy"},
        // kuavo-4proEDU 系列
        {49,     "kuavo-4proEDU-biped-revo1hand"},
        {100049, "kuavo-4proEDU-biped-dummy"},
        // kuavo-5 系列
        {50,     "kuavo-5-biped-none"},
        {52,     "kuavo-5V2-biped-none"},
        // kuavo-5-wheeled 系列
        {60,     "kuavo-5-wheeled-none"},
        // roban 系列
        {14,     "roban-2v1-biped-none"},
        // ... 实际需覆盖全部 33 个版本号
    };
    return m;
}
```

> **注意**：参考实现仅覆盖 ~12 个条目。实际代码库中有 33 个唯一版本号（含 11, 13, 16, 17, 41, 43, 46, 48, 51, 53, 54, 55, 61, 62, 63, 200049, 300049, 400049 等），必须在实现时全部补齐。详见 [使用场景梳理](robot-version-usage-analysis.md) 中的版本号全集。

### 5.2 模型信息表（base_key → 语义路径字段）

索引键 = `brand-product-locomotion`，同一 base model 下不同末端共享相同的路径字段。

```cpp
struct ModelInfo {
    const char* model_name;    // 模型目录名: "biped_s45"
    const char* config_name;   // 配置目录名: "kuavo_v45"
    // legacy_id 已移除，运行时数据文件统一用 robot_id 命名
};

static const std::map<std::string, ModelInfo>& model_map() {
    static const std::map<std::string, ModelInfo> m = {
        {"kuavo-4-biped",       {"biped_s40", "kuavo_v40", "40"}},
        {"kuavo-4V2-biped",     {"biped_s42", "kuavo_v42", "42"}},
        {"kuavo-4pro-biped",    {"biped_s45", "kuavo_v45", "45"}},
        {"kuavo-4proEDU-biped", {"biped_s49", "kuavo_v49", "49"}},
        {"kuavo-5-biped",       {"biped_s50", "kuavo_v50", "50"}},
        {"kuavo-5V2-biped",     {"biped_s52", "kuavo_v52", "52"}},
        {"kuavo-5-wheeled",     {"biped_s60", "kuavo_v60", "60"}},
        {"roban-2v1-biped",     {"biped_s14", "kuavo_v14", "14"}},
        // ... 同样需覆盖全部 base model
    };
    return m;
}
```

### 5.3 映射表查找逻辑

`model_name()` / `config_name()` 的查找过程：

1. 从 `robot_id`（如 `kuavo-4pro-biped-revo1hand`）截取前三段 → `kuavo-4pro-biped`
2. 用该 base_key 在 `model_map()` 中查找
3. 找到 → 返回对应字段；未找到 → 返回空字符串

---

## 6. 兼容宏（C++）

宏名相比旧版的变化：去掉 `LEGGED` 后缀（locomotion 已由独立字段表达），品牌+运动方式组合改用 `BIPED`。删除与 `IS_ROBAN` 重复的 `IS_ROBAN2_LEGGED`。

```cpp
// 品牌判断
#define IS_KUAVO(rv)            (rv.brand() == robot_version::Brand::Kuavo)
#define IS_ROBAN(rv)            (rv.brand() == robot_version::Brand::Roban)

// 品牌 + 运动方式
#define IS_KUAVO_BIPED(rv)      (IS_KUAVO(rv) && rv.locomotion() == robot_version::Locomotion::Biped)
#define IS_KUAVO_WHEELED(rv)    (IS_KUAVO(rv) && rv.locomotion() == robot_version::Locomotion::Wheeled)
#define IS_ROBAN_BIPED(rv)      (IS_ROBAN(rv) && rv.locomotion() == robot_version::Locomotion::Biped)

// 产品系列（带品牌限定）
#define IS_KUAVO4(rv)           (IS_KUAVO(rv) && rv.product()[0] == '4')
#define IS_KUAVO5(rv)           (IS_KUAVO(rv) && rv.product()[0] == '5')
#define IS_KUAVO4PRO(rv)        (IS_KUAVO(rv) && rv.product().find("4pro") == 0)

// 具体产品型号（带品牌限定）
#define IS_KUAVO4V2(rv)         (IS_KUAVO(rv) && rv.product() == "4V2")
#define IS_KUAVO4PRO_EDU(rv)    (IS_KUAVO(rv) && rv.product() == "4proEDU")
#define IS_KUAVO5V2(rv)         (IS_KUAVO(rv) && rv.product() == "5V2")
#define IS_KUAVO5V3(rv)         (IS_KUAVO(rv) && rv.product() == "5V3")
#define IS_KUAVO5V4(rv)         (IS_KUAVO(rv) && rv.product() == "5V4")
#define IS_ROBAN2V1(rv)         (IS_ROBAN(rv) && rv.product() == "2v1")

// 末端执行器
#define HAS_REVO1HAND(rv)       (rv.left_end() == robot_version::EndEffectorType::Revo1hand || rv.right_end() == robot_version::EndEffectorType::Revo1hand)
#define HAS_REVO2HAND(rv)       (rv.left_end() == robot_version::EndEffectorType::Revo2hand || rv.right_end() == robot_version::EndEffectorType::Revo2hand)
#define HAS_LINKHANDO6(rv)      (rv.left_end() == robot_version::EndEffectorType::LinkhandO6 || rv.right_end() == robot_version::EndEffectorType::LinkhandO6)
#define HAS_REVO1TOUCH(rv)      (rv.left_end() == robot_version::EndEffectorType::Revo1touch || rv.right_end() == robot_version::EndEffectorType::Revo1touch)
#define HAS_LEJUCLAW(rv)        (rv.left_end() == robot_version::EndEffectorType::Lejuclaw || rv.right_end() == robot_version::EndEffectorType::Lejuclaw)
#define HAS_DUMMY(rv)           (rv.left_end() == robot_version::EndEffectorType::Dummy || rv.right_end() == robot_version::EndEffectorType::Dummy)
#define IS_ASYMMETRIC_END(rv)   (rv.left_end() != rv.right_end())
```

旧宏 → 新宏对照：

| 旧宏 | 新宏 |
|------|------|
| `IS_KUAVO(rb)` | `IS_KUAVO(rv)` |
| `IS_ROBAN(rb)` | `IS_ROBAN(rv)` |
| `IS_KUAVO_LEGGED(rb)` | `IS_KUAVO_BIPED(rv)` |
| `IS_KUAVO_WHEELED(rb)` | `IS_KUAVO_WHEELED(rv)` |
| `IS_ROBAN_LEGGED(rb)` | `IS_ROBAN_BIPED(rv)` |
| `IS_ROBAN2_LEGGED(rb)` | 删除 |
| `IS_KUAVO4_LEGGED(rb)` | `IS_KUAVO4(rv)` |
| `IS_KUAVO5_LEGGED(rb)` | `IS_KUAVO5(rv)` |
| `IS_KUAVO4PRO_LEGGED(rb)` | `IS_KUAVO4PRO(rv)` |

业务代码中的版本判断一律使用宏，不允许直接写 `rv.product() == "xxx"` 魔法字符串。

---

## 7. Launch 文件接口

### 7.1 Python 顶层函数

`__init__.py` 导出以下函数，供 Launch 文件通过 `$(eval __import__('robot_version').xxx())` 调用：

```python
def model_name() -> str:
    """模型资源目录名。旧格式 45 → 'biped_s45'，新格式直传。"""

def config_name() -> str:
    """配置目录名。旧格式 45 → 'kuavo_v45'，新格式直传。"""

def urdf_name() -> str:
    """URDF 文件名（不带后缀）。旧格式 45 → 'biped_s45'，新格式直传。"""

def is_kuavo() -> bool: ...
def is_roban() -> bool: ...
def is_kuavo5() -> bool: ...
def is_biped() -> bool: ...
def is_wheeled() -> bool: ...
```

行为规则：读取 `ROBOT_VERSION` 环境变量，旧格式通过映射表转换，新格式直接返回。

### 7.2 Launch 文件使用方式

```xml
<launch>
  <!-- 顶部定义一次，__import__ 只出现在这里 -->
  <arg name="model_name"  default="$(eval __import__('robot_version').model_name())"/>
  <arg name="config_name" default="$(eval __import__('robot_version').config_name())"/>
  <arg name="urdf_name"   default="$(eval __import__('robot_version').urdf_name())"/>
  <arg name="is_kuavo"    default="$(eval __import__('robot_version').is_kuavo())"/>

  <!-- 后续全部用 $(arg) 引用 -->
  <arg name="urdfFile"
       default="$(find kuavo_assets)/models/$(arg model_name)/urdf/$(arg urdf_name).urdf"/>
  <arg name="taskFile"
       default="$(find humanoid_controllers)/config/$(arg config_name)/mpc/task.info"/>

  <!-- 条件分支用布尔 arg -->
  <group if="$(arg is_kuavo)">
      <node pkg="humanoid_controllers" type="upper_computer_service.py" ... />
  </group>
</launch>
```

### 7.3 路径替换规则

| 旧模式 | 新模式 |
|--------|--------|
| `biped_s$(arg robot_version)` | `$(arg model_name)` |
| `kuavo_v$(arg robot_version)` | `$(arg config_name)` |
| `kuavo_s$(arg robot_version)` | `$(arg config_name)`（目录需重命名 `kuavo_s` → `kuavo_v`） |
| `s$(arg robot_version)_collision_config` | `$(arg collision_config_name)` |

### 7.4 条件分支替换规则

| 旧表达式 | 新表达式 |
|---------|---------|
| `arg('robot_version') >= 40` | `arg('is_kuavo')` |
| `arg('robot_version') >= 30` | `arg('is_kuavo')` |
| `arg('robot_version') >= 50` | `arg('is_kuavo5')` |
| `arg('robot_version') == 60` / `60<=V%100<70` | `arg('is_wheeled')` |
| `arg('robot_version') == 15` 的 if/unless 对 | 删除（映射表自动处理） |

### 7.5 设计要点

- 命名用 `_name` 不用 `_dir`：返回的是名字不是目录路径
- 文件后缀在 Launch 文件中拼接：包只返回不带后缀的名字
- 用户可覆盖：`roslaunch xxx.launch model_name:=custom_model`

---

## 8. Shell 脚本接口

Shell 脚本中统一用 `python3 -c` 调用 `robot_version` 模块，不维护独立的 bash 解析逻辑。

```bash
# 获取字段
BRAND=$(python3 -c "from robot_version import RobotVersion; print(RobotVersion.from_env().brand)")
PRODUCT=$(python3 -c "from robot_version import RobotVersion; print(RobotVersion.from_env().product)")
LOCOMOTION=$(python3 -c "from robot_version import RobotVersion; print(RobotVersion.from_env().locomotion)")

# 获取路径字段
ROBOT_ID=$(python3 -c "from robot_version import RobotVersion; print(RobotVersion.from_env().robot_id)")
MASS_FILE="TotalMassV${ROBOT_ID}"

# 判断
[[ "$BRAND" == "roban" ]]           # 替代 ^1[0-9]$
[[ "$PRODUCT" == "4V2" ]]           # 替代 == "42"
[[ "$PRODUCT" == 5* ]]              # 替代 ^5[0-9]$
[[ "$LOCOMOTION" == "wheeled" ]]    # 替代 ^6[0-9]$ / == "60"
```

这些脚本非高频调用，python 启动开销可忽略。

---

## 9. ROS 参数服务器变更

`/robot_version` **int 保持不动**，新增 `/robot_version_next` **string**。

**写入**（`robot_version_manager.launch`）：
```xml
<!-- /robot_version 保持 int，旧代码零改动 -->
<param name="robot_version" value="$(eval __import__('robot_version').legacy_int())"/>

<!-- 新增 /robot_version_next string，新代码读这个 -->
<param name="robot_version_next" type="string"
       value="$(eval __import__('robot_version').robot_id())"/>
```

`legacy_int()` 旧格式输入时返回原始数字（`45`），新格式输入时通过映射表反查，无旧编号的新机器人返回 `0`。

**旧代码读取**（不用改）：
```cpp
int rv;
nh.getParam("/robot_version", rv);  // 继续正常工作
```

**新代码读取**：
```cpp
// C++
std::string rv_str;
nh.param<std::string>("/robot_version_next", rv_str, "");
robot_version::RobotVersion rv(rv_str);
```

```python
# Python
from robot_version import RobotVersion
rv = RobotVersion(rospy.get_param('/robot_version_next'))
```

---

## 10. 与旧 RobotVersion 类的关系

| | 旧类（kuavo_common） | 新类（robot_version 包） |
|---|---|---|
| 位置 | `kuavo_common/common/common.h` | `robot_version/robot_version.h` |
| 输入 | 仅数字 `int` | 数字或新格式字符串 |
| 字段 | `Major`, `Minor`, `Patch` | enum 语义字段 + string product |
| 判断方式 | 数字编码（`major()==4`） | 语义直接（`brand()==Kuavo`） |
| 过渡期 | 保留不动 | 新代码使用，逐步替代旧类 |

**两套旧定义均需最终删除**：
1. `src/kuavo_common/include/kuavo_common/common/common.h` — 主定义 + 9 个宏
2. `src/humanoid-control/humanoid_interface_drake/include/.../common.h` — 精简版 struct

迁移期间可在同一文件中混用新旧 API。新类不提供 `legacy()` 方法返回旧对象——如果需要旧类，说明该文件还没迁移。

---

## 11. SDK 分发

`kuavo-humanoid-sdk` 打包发布时，直接将 `robot_version` 的 Python 模块拷贝到 SDK 内部。映射表硬编码在 `.py` 中，不依赖外部文件，无需 pip 依赖。
