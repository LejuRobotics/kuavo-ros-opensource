# ROBOT_VERSION_NEXT 设计方案：纯标签模式

## 概述

沿用 `ROBOT_VERSION` 环境变量，采用人类可读的字符串格式，解决旧版本号不可读、难解析、难扩展的问题。

**格式**：`<brand>-<product>-<locomotion>[-<end_effector>[-<camera>]][+tags...]`

```bash
# 最简格式（end_effector 缺省为 none, camera 缺省为 none_none_none_none）
export ROBOT_VERSION="kuavo-4pro-biped"

# 指定 end_effector，camera 仍缺省为 none_none_none_none
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand"

# 指定 camera（必须完整 4 段格式）
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-Gemini330_none_none_none"
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-Gemini330_D435_none_none"
```

- `-` 分隔 ID 组成部分，影响资源文件查找
- `+` 分隔描述性标签，仅作元数据

---

## 形式化定义（BNF）

```bnf
<robot>        ::= <robot_id> <tags>

<robot_id>     ::= <brand> "-" <product> "-" <locomotion> [<end_effector_part> [<camera_part>]]

<brand>        ::= "kuavo" | "roban"

<product>      ::= [a-zA-Z0-9]+

<locomotion>   ::= "biped" | "wheeled"

<end_effector_part> ::= "-" <end_effector>
<end_effector> ::= <single_end> | <single_end> "_" <single_end>
<single_end>   ::= "none" | "lejuclaw" | "revo1hand" | "revo2hand" | "linkhandO6" | "revo1touch" | "dummy"

<camera_part>  ::= "-" <camera>
<camera>       ::= <cam_head> "_" <cam_waist> "_" <cam_left_wrist> "_" <cam_right_wrist>
<cam_head>     ::= "none" | <camera_model>
<cam_waist>    ::= "none" | <camera_model>
<cam_left_wrist>  ::= "none" | <camera_model>
<cam_right_wrist> ::= "none" | <camera_model>
<camera_model>    ::= "D435" | "D405" | "Gemini330" | "Gemini335L"

<tags>         ::= "" | "+" <tag> <tags>
<tag>          ::= [a-zA-Z0-9_]+
```

---

## 正则表达式

```regex
^(kuavo|roban)-([a-z0-9A-Z]+)-(biped|wheeled)(?:-((none|lejuclaw|revo1hand|revo2hand|linkhandO6|revo1touch|dummy)(_(none|lejuclaw|revo1hand|revo2hand|linkhandO6|revo1touch|dummy))?))?(?:-((none|D435|D405|Gemini330|Gemini335L)_(none|D435|D405|Gemini330|Gemini335L)_(none|D435|D405|Gemini330|Gemini335L)_(none|D435|D405|Gemini330|Gemini335L)))?((?:\+[a-zA-Z0-9_]+)*)$
```

---

## 结构图

```
# 最简格式（end_effector 和 camera 缺省）
export ROBOT_VERSION="kuavo-4pro-biped"
                      │     │    │
                      │     │    └── locomotion: biped
                      │     └── product: 4pro
                      └── brand: kuavo

                      └──────── robot_id ────────┘
                      (缺省: end_effector=none, camera=none_none_none_none)


# 指定 end_effector，camera 缺省
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand"
                      │     │    │     │
                      │     │    │     └── end_effector: revo1hand
                      │     │    └── locomotion: biped
                      │     └── product: 4pro
                      └── brand: kuavo

                      └─────────── robot_id ───────────┘
                      (缺省: camera=none_none_none_none)


# 完整相机格式
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-Gemini330_D435_none_none+upper_nuci7"
                      │     │    │     │         │                       │
                      │     │    │     │         │                       └── tags: {upper_nuci7}
                      │     │    │     │         └── camera: Gemini330_D435_none_none
                      │     │    │     └── end_effector: revo1hand
                      │     │    └── locomotion: biped
                      │     └── product: 4pro
                      └── brand: kuavo

                      └─────────────────── robot_id ───────────────────┘
```

---

## 分隔符语义

| 分隔符 | 用途 | 影响 ID | 示例 |
|-------|------|--------|------|
| `-` | 分隔 ID 组成部分 | ✅ 是 | `kuavo-4pro-biped-revo1hand-Gemini330_D435_none_none` |
| `+` | 分隔描述性标签 | ❌ 否 | `+upper_nuci7+lower_amr7000` |
| `_` | 分隔复合值（左_右 或 head_waist_leftwrist_rightwrist） | ✅ 是 | `revo1hand_lejuclaw`, `Gemini330_D435_none_none` |

---

## 格式规则

| 组成部分 | 分隔符 | 必选/可选 | 影响 ID | 说明 |
|---------|--------|----------|--------|------|
| `brand` | `-` | 必选 | ✅ | 品牌：`kuavo`、`roban` |
| `product` | `-` | 必选 | ✅ | 产品型号 |
| `locomotion` | `-` | 必选 | ✅ | 运动方式：`biped`（双足）、`wheeled`（轮式） |
| `end_effector` | `-` | 可选 | ✅ | 末端执行器，缺省为 `none` |
| `camera` | `-` | 可选 | ✅ | 相机配置，缺省为 `none_none_none_none` |
| `tag` | `+` | 可选 | ❌ | 描述性标签，可多个 |

---

## 品牌（brand）

| 品牌 | 说明 |
|------|------|
| `kuavo` | Kuavo 系列 |
| `roban` | Roban 系列 |

---

## 产品型号（product）

### Kuavo 系列

| 型号 | 说明 |
|------|------|
| `4` | Kuavo 4 |
| `4pro` | Kuavo 4 Pro |
| `4proV1` | Kuavo 4 Pro V1 |
| `4proV2` | Kuavo 4 Pro V2 |
| `4proEDU` | Kuavo 4 Pro EDU |
| `5` | Kuavo 5 |
| `5V1` | Kuavo 5 V1 |
| `5V2` | Kuavo 5 V2 |

### Roban 系列

| 型号 | 说明 |
|------|------|
| `2v0` | Roban 2.0 |
| `2v1` | Roban 2.1 |
| `2v2` | Roban 2.2 |

---

## 运动方式（locomotion）

| 值 | 说明 |
|----|------|
| `biped` | 双足机器人 |
| `wheeled` | 轮式机器人 |

---

## 末端执行器（end_effector）

### 单一值（左右相同）

| 值 | 说明 |
|----|------|
| `none` | 无末端 |
| `lejuclaw` | 夹爪 |
| `revo1hand` | 灵巧手 RevO 1代 |
| `revo2hand` | 灵巧手 RevO 2代 |
| `linkhandO6` | 灵巧手 LinkHand O6 |
| `revo1touch` | 灵巧手 RevO 1代触觉版 |
| `dummy` | 塑料假手（装饰性） |

### 复合值（左右不同）

格式：`<左手>_<右手>`

| 值 | 说明 |
|----|------|
| `revo1hand_lejuclaw` | 左灵巧手 RevO 1代 + 右夹爪 |
| `lejuclaw_revo1hand` | 左夹爪 + 右灵巧手 RevO 1代 |
| `revo1hand_revo2hand` | 左灵巧手 RevO 1代 + 右灵巧手 RevO 2代 |
| `linkhandO6_revo1touch` | 左 LinkHand O6 + 右 RevO 1代触觉版 |
| `none_revo1hand` | 左无 + 右灵巧手 RevO 1代 |
| `revo1hand_none` | 左灵巧手 RevO 1代 + 右无 |
| `none_lejuclaw` | 左无 + 右夹爪 |
| `lejuclaw_none` | 左夹爪 + 右无 |

---

## 相机配置（camera）

**格式**：`<head>_<waist>_<leftwrist>_<rightwrist>`

四个位置的相机型号，用 `_` 分隔，无相机用 `none` 表示。

### 位置说明

| 位置 | 说明 |
|------|------|
| `head` | 头部相机 |
| `waist` | 腰部相机 |
| `leftwrist` | 左手腕相机 |
| `rightwrist` | 右手腕相机 |

### 相机型号

| 型号 | 品牌 | 说明 |
|------|------|------|
| `none` | - | 该位置无相机 |
| `D435` | Intel RealSense | RealSense D435 |
| `D405` | Intel RealSense | RealSense D405 |
| `Gemini330` | Orbbec | 奥比中光 Gemini 330 |
| `Gemini335L` | Orbbec | 奥比中光 Gemini 335L |

### 配置示例

| 相机配置 | 说明 |
|---------|------|
| `none_none_none_none` | 无任何相机 |
| `Gemini330_none_none_none` | 仅头部 Gemini330 |
| `Gemini330_D435_none_none` | 头部 Gemini330 + 腰部 D435 |
| `Gemini330_none_D405_D435` | 头部 Gemini330 + 左腕 D405 + 右腕 D435 |
| `none_none_D405_none` | 仅左腕 D405 |

---

## 描述性标签（tags）

描述性标签不影响 robot_id，仅作为元数据。

### 上下位机硬件版本

用于标识不同计算平台组合，影响驱动适配和部署方式。

**上位机**：
| 标签 | 说明 |
|------|------|
| `none` | 无上位机（缺省，可不写） |
| `upper_nuci7` | Intel NUC i7 |

**下位机**：
| 标签 | 说明 |
|------|------|
| `none` | 无下位机（缺省，可不写） |
| `lower_nuci9` | Intel NUC i9 |
| `lower_amr7000` | AMR7000 |
| `lower_rk3588` | RK3588 |

**示例**：
```bash
# 无上位机、无下位机（缺省，无需标签）
kuavo-4pro-biped-revo1hand

# NUC i7 上位机 + AMR7000 下位机
+upper_nuci7+lower_amr7000

# RK3588 下位机
+lower_rk3588

# 仅上位机
kuavo-4pro-biped-revo1hand+upper_nuci7

# 仅下位机
kuavo-4pro-biped-revo1hand+lower_amr7000
```

### 结构特性

| 标签 | 说明 |
|------|------|
| `shortarm` | 短手臂 |
| `longarm` | 长手臂 |


## 使用示例

### 基础用法

```bash
# Kuavo 系列 - 双足机器人（最简格式）
export ROBOT_VERSION="kuavo-4pro-biped"                                    # Kuavo 4 Pro 双足（无末端、无相机）

# 指定 end_effector
export ROBOT_VERSION="kuavo-4pro-biped-lejuclaw"                           # Kuavo 4 Pro 双足 夹爪（无相机）
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand"                         # Kuavo 4 Pro 双足 RevO 1代灵巧手（无相机）

# 指定相机（必须完整 4 段）
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-Gemini330_none_none_none"   # Kuavo 4 Pro 双足 RevO 1代 头部相机

# Kuavo 系列 - 轮式机器人
export ROBOT_VERSION="kuavo-5-wheeled"                                    # Kuavo 5 轮式（无末端、无相机）
export ROBOT_VERSION="kuavo-5-wheeled-lejuclaw"                            # Kuavo 5 轮式 夹爪（无相机）
export ROBOT_VERSION="kuavo-5-wheeled-lejuclaw-Gemini330_D435_none_none"   # Kuavo 5 轮式 夹爪 头腰相机

# Roban 系列
export ROBOT_VERSION="roban-2v1-biped"                                  # Roban 2.1 双足（无末端、无相机）
export ROBOT_VERSION="roban-2v1-biped-lejuclaw"                          # Roban 2.1 双足 夹爪（无相机）
export ROBOT_VERSION="roban-2v2-biped-revo1hand-Gemini330_none_none_none"    # Roban 2.2 双足 RevO 1代灵巧手 头部相机
```

### 左右手非对称末端执行器

```bash
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand_lejuclaw"                 # 左灵巧手 RevO 1代 + 右夹爪（无相机）
export ROBOT_VERSION="kuavo-4pro-biped-lejuclaw_revo1hand-Gemini330_none_none_none"   # 左夹爪 + 右灵巧手 RevO 1代 头部相机
export ROBOT_VERSION="kuavo-4pro-biped-none_revo1hand-none_none_D405_none"      # 左无 + 右灵巧手 RevO 1代 仅左腕相机
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand_revo2hand"                # 左 RevO 1代 + 右 RevO 2代（无相机）
export ROBOT_VERSION="kuavo-4pro-biped-linkhandO6_revo1touch"              # 左 LinkHand O6 + 右 RevO 1代触觉版（无相机）
```

### 相机配置

```bash
# 最简格式（无末端、无相机 - 缺省值）
export ROBOT_VERSION="kuavo-4pro-biped"

# 指定末端但无相机（camera 缺省为 none_none_none_none）
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand"

# 单相机（头部）
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-Gemini330_none_none_none"

# 多相机（头 + 腰）
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-Gemini330_D435_none_none"

# 多相机（头 + 腰 + 左腕）
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-Gemini330_D435_D405_none"

# 多相机（头 + 左右腕）
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-Gemini330_none_D405_D435"

# 仅手腕相机
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-none_none_D405_D435"
```

### 上下位机硬件版本

```bash
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-Gemini330_none_none_none+upper_nuci7+lower_amr7000"  # 头部相机 + 上下位机
```

### 产品版本标记

```bash
export ROBOT_VERSION="kuavo-4pro-biped-dummy"                              # 展厅版（塑料假手，无相机）
export ROBOT_VERSION="kuavo-4proEDU-biped-revo1hand"                       # 教育版 RevO 1代（无相机）
export ROBOT_VERSION="kuavo-4proEDU-biped-revo2hand-Gemini330_none_none_none"  # 教育版 RevO 2代 有相机
```

### 完整示例

```bash
# 完整配置示例
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand_lejuclaw-Gemini330_D435+upper_nuci7+lower_amr7000"

# 最简配置示例
export ROBOT_VERSION="kuavo-4pro-biped"                                    # 最简格式，无末端、无相机

# 相机配置示例
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-Gemini330_none_none_none" # 仅头部相机
export ROBOT_VERSION="kuavo-4pro-biped-revo1hand-none_none_none_none"      # 无相机（显式）
```

---

## 与旧版本号的映射

| 旧格式（数字） | 新格式 | 说明 |
|--------------|--------|------|
| 40 | `kuavo-4-biped` | Kuavo 4 双足 第1版（无末端、无相机） |
| 42 | `kuavo-4V2-biped-revo1hand` | Kuavo 4 双足 第2版（短臂、RevO 1代灵巧手、无相机） |
| 45 | `kuavo-4pro-biped-revo1hand` | Kuavo 4 Pro 双足 RevO 1代灵巧手（无相机） |
| 47 | `kuavo-4pro-biped-lejuclaw` | Kuavo 4 Pro 双足 夹爪（无相机） |
| 49 | `kuavo-4proEDU-biped-revo1hand` | Kuavo 4 Pro EDU 双足 RevO 1代灵巧手（无相机） |
| 50 | `kuavo-5-biped` | Kuavo 5 双足 第1版（无末端、无相机） |
| 52 | `kuavo-5V2-biped` | Kuavo 5 双足 第2版（无末端、无相机） |
| 60 | `kuavo-5-wheeled` | Kuavo 5 轮式 第1版（无末端、无相机） |
| 14 | `roban-2v1-biped` | Roban 2.1 双足 第1版（无末端、无相机） |
| 100045 | `kuavo-4pro-biped-dummy` | Kuavo 4 Pro 双足 展厅版（塑料假手、无相机） |
| 100049 | `kuavo-4proEDU-biped-dummy` | Kuavo 4 Pro EDU 双足 展厅版（塑料假手、无相机） |
| 200049 | `kuavo-4proEDU-biped-claw` | Kuavo 4 Pro EDU 双足 claw 版（无相机） |

---

## 现有使用场景

本节列举当前代码中 ROBOT_VERSION（整数格式）的使用方式。

### 1. 资源文件查找

#### 1.1 URDF 模型文件

**路径模式**：
- `models/biped_s${VERSION}/urdf/biped_s${VERSION}.urdf` - 主 URDF 模型
- `models/biped_s${VERSION}/urdf/biped_s${VERSION}_gazebo.urdf` - Gazebo 仿真
- `models/biped_s${VERSION}/urdf/drake/biped_v3_arm.urdf` - Drake IK

**业务场景**: 根据 ROBOT_VERSION 定位并加载对应版本的 URDF 模型文件。不同版本（45、49、60 等）有不同的模型文件目录。

**Python**：
```python
robot_version = os.environ.get('ROBOT_VERSION', '45')
urdf_path = f"{kuavo_assets_path}/models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf"
```

**C++**：
```cpp
const char* robot_version = std::getenv("ROBOT_VERSION");
std::string urdf_path = kuavo_asset_path + "/models/biped_s" + robot_version + "/urdf/biped_s" + robot_version + ".urdf";
```

**Launch**：
```xml
<arg name="urdfFile" value="$(find kuavo_assets)/models/biped_s$(arg robot_version)/urdf/biped_s$(arg robot_version).urdf"/>
```

#### 1.2 配置文件加载

**业务场景**: 根据 ROBOT_VERSION 加载对应版本的控制参数、碰撞检测配置等。不同版本的机器人关节参数、控制参数不同。

**路径模式**：
- `kuavo_assets/config/kuavo_v${VERSION}/kuavo.json` - 机器人主配置
- `humanoid_controllers/config/kuavo_v${VERSION}/mpc/task.info` - MPC 参数
- `humanoid_controllers/config/kuavo_v${VERSION}/rl/*.info` - RL 参数
- `kuavo_arm_collision_check/config/s${VERSION}_collision_config.yaml` - 碰撞检测

**Python**：
```python
config_path = f"{kuavo_assets_path}/config/kuavo_v{robot_version}/kuavo.json"
```

**Shell**：
```bash
sed -i '...' ~/kuavo-ros-opensource/src/kuavo_assets/config/kuavo_v${ROBOT_VERSION}/kuavo.json
```

### 2. 版本特性判断

不同版本的机器人具有不同的硬件特性和能力，代码需要根据版本执行不同的逻辑。

#### 2.1 Python 中的版本判断

**业务场景**: 根据 ROBOT_VERSION 判断机器人型号、系列，执行版本特定的逻辑。例如：
- Kuavo 4 系列和 Kuavo 5 系列使用不同的控制参数
- 4 Pro EDU 版本有特殊的标定流程
- 展厅版（100045）使用假手，跳过灵巧手初始化

```python
robot_version = int(os.environ.get("ROBOT_VERSION", "45"))

# 系列判断（范围比较）
if robot_version >= 40 and robot_version < 50:
    # Kuavo 4 系列
elif robot_version >= 50 and robot_version < 60:
    # Kuavo 5 系列
elif robot_version >= 10 and robot_version < 20:
    # Roban 系列

# 精确版本判断
if robot_version == 49:
    # 4 Pro EDU 特殊处理

# 主版本提取
robot_major = (robot_version // 10) % 10  # 45 -> 4

# 展厅版兼容（模运算）
if (robot_version % 10000) == 45:
    # 兼容 45 和 100045
```

#### 2.2 C++ 中的版本判断

**业务场景**: C++ 代码在编译时或运行时根据 ROBOT_VERSION 选择不同的代码分支。例如：
- 编译时：不同版本包含不同的头文件或链接不同的库
- 运行时：根据版本初始化不同的控制器参数

```cpp
// 编译时宏
#if ROBOT_VERSION_INT >= 40
    // Kuavo 系列
#endif

// 运行时判断
int robot_version = std::stoi(std::getenv("ROBOT_VERSION"));
if (robot_version < 30) {
    is_roban_version_ = true;
}
```

#### 2.3 Launch 文件中的判断

```xml
<group if="$(eval arg('robot_version') >= 40)">
    <!-- Kuavo 系列配置 -->
</group>

<arg name="taskFile" value="
    '$(find humanoid_controllers)/config/kuavo5/task.info' if arg('robot_version') >= 50 else
    '$(find humanoid_controllers)/config/kuavo/task.info'  if arg('robot_version') >= 40 else
    '$(find humanoid_controllers)/config/roban2/task.info'
"/>
```

#### 2.4 Shell 脚本中的判断

```bash
# 整数范围比较
if [[ "$ROBOT_VERSION" -gt 50 ]] && [[ "$ROBOT_VERSION" -lt 60 ]]; then
    echo "Kuavo 5 系列"
fi

# 字符串精确比较
if [ "${ROBOT_VERSION}" = "60" ]; then
    echo "轮式机器人"
fi

# 前缀判断
if [[ "$ROBOT_VERSION" == 1* ]]; then
    robot_name="ROBAN"
else
    robot_name="KUAVO"
fi
```

### 3. 目录结构

```
src/kuavo_assets/
├── models/
│   ├── biped_s14/       # Roban 2.1
│   ├── biped_s40/       # Kuavo 4
│   ├── biped_s42/       # Kuavo 4 V2 (短臂)
│   ├── biped_s45/       # Kuavo 4 Pro 灵巧手
│   ├── biped_s47/       # Kuavo 4 Pro 夹爪
│   ├── biped_s49/       # Kuavo 4 Pro EDU
│   ├── biped_s50/       # Kuavo 5
│   ├── biped_s52/       # Kuavo 5 V2
│   ├── biped_s60/       # Kuavo 5 轮式
│   ├── biped_s100045/   # Kuavo 4 Pro 展厅版
│   └── biped_s100049/   # Kuavo 4 Pro EDU 展厅版
└── config/
    ├── kuavo_v14/
    ├── kuavo_v45/
    ├── kuavo_v49/
    └── ...
```

---

## Next 版本使用方式

本节说明在新格式下，上述场景应该如何实现。

### 1. 资源文件查找

#### 1.1 URDF 模型文件

**路径模式**：
- `models/${ROBOT_ID}/urdf/${ROBOT_ID}.urdf`
- `models/${ROBOT_ID}/urdf/${ROBOT_ID}_gazebo.urdf`
- `models/${ROBOT_ID}/urdf/drake/biped_v3_arm.urdf`

**Python**：
```python
from robot_version import RobotVersion

rv = RobotVersion.from_env("kuavo-4pro-biped-revo1hand-Gemini330")  # 简写，等价于 Gemini330_none_none_none
urdf_path = f"{kuavo_assets_path}/models/{rv.robot_id}/urdf/{rv.robot_id}.urdf"
```

**C++**：
```cpp
#include "robot_version.hpp"

auto rv = RobotVersion::from_env("kuavo-4pro-biped-revo1hand-Gemini330");  // 简写，等价于 Gemini330_none_none_none
std::string urdf_path = kuavo_asset_path + "/models/" + rv.robot_id() + "/urdf/" + rv.robot_id() + ".urdf";
```

**Launch**：
```xml
<arg name="robot_id" default="$(optenv ROBOT_ID kuavo-4pro-biped-revo1hand-Gemini330)"/>  <!-- 简写，等价于 Gemini330_none_none_none -->
<arg name="urdfFile" value="$(find kuavo_assets)/models/$(arg robot_id)/urdf/$(arg robot_id).urdf"/>
```

#### 1.2 配置文件

**Python**：
```python
rv = RobotVersion.from_env("kuavo-4pro-biped-revo1hand-Gemini330")  # 简写，等价于 Gemini330_none_none_none
config_path = f"{kuavo_assets_path}/config/{rv.robot_id}/kuavo.json"
```

**Shell**：
```bash
ROBOT_ID="${ROBOT_VERSION%%+*}"  # 去掉标签部分
sed -i '...' ~/kuavo-ros-opensource/src/kuavo_assets/config/${ROBOT_ID}/kuavo.json
```

### 2. 版本判断逻辑

#### 2.1 Python 中的判断

```python
from robot_version import RobotVersion

rv = RobotVersion.from_env("kuavo-4pro-biped-revo1hand-Gemini330")  # 简写，等价于 Gemini330_none_none_none

# 品牌判断
if rv.is_kuavo:
    # Kuavo 系列
elif rv.is_roban:
    # Roban 系列

# 型号判断
if rv.product.startswith("4"):
    # Kuavo 4 系列
elif rv.product.startswith("5"):
    # Kuavo 5 系列

# 运动方式判断
if rv.is_biped:
    # 双足机器人
elif rv.is_wheeled:
    # 轮式机器人

# 末端类型判断（通过 left_end / right_end 判断）
if rv.left_end == "revo1hand" or rv.right_end == "revo1hand":
    # 有灵巧手
if rv.left_end == "lejuclaw" or rv.right_end == "lejuclaw":
    # 有夹爪
if rv.left_end == "dummy" or rv.right_end == "dummy":
    # 展厅版（塑料假手）

# 精确判断
if rv.robot_id == "kuavo-4proEDU-biped-revo1hand-Gemini330":  # 简写，等价于 Gemini330_none_none_none
    # 4 Pro EDU 特殊处理

# 相机信息
head_camera = rv.get_camera("head")      # "Gemini330" 或 "none"
waist_camera = rv.get_camera("waist")    # "D435" 或 "none"
left_wrist_camera = rv.get_camera("leftwrist")   # "D405" 或 "none"
right_wrist_camera = rv.get_camera("rightwrist") # "D435" 或 "none"
has_any_camera = rv.has_camera()         # True 或 False

# 硬件信息
upper = rv.get_upper_computer()     # "nuci7" 或 None
```

#### 2.2 C++ 中的判断

```cpp
#include "robot_version.hpp"

auto rv = RobotVersion::from_env("kuavo-4pro-biped-revo1hand-Gemini330");  // 简写，等价于 Gemini330_none_none_none

// 品牌判断
if (rv.is_kuavo()) { ... }
if (rv.is_roban()) { ... }

// 运动方式判断
if (rv.is_biped()) { ... }
if (rv.is_wheeled()) { ... }

// 末端类型判断（通过宏）
if (HAS_REVO1HAND(rv)) { ... }
if (HAS_LEJUCLAW(rv)) { ... }
if (HAS_DUMMY(rv)) { ... }

// 相机信息
std::string head_camera = rv.get_camera("head");
std::string waist_camera = rv.get_camera("waist");
bool has_camera = rv.has_camera();

// 硬件信息
std::string upper = rv.get_upper_computer();
```

#### 2.3 Launch 文件中的判断

```xml
<arg name="robot_version" default="$(optenv ROBOT_VERSION kuavo-4pro-biped-revo1hand-Gemini330)"/>  <!-- 简写，等价于 Gemini330_none_none_none -->

<!-- 品牌判断 -->
<group if="$(eval 'kuavo' in arg('robot_version'))">
    <!-- Kuavo 系列配置 -->
</group>

<!-- 运动方式判断 -->
<group if="$(eval '-biped-' in arg('robot_version'))">
    <!-- 双足机器人配置 -->
</group>
<group if="$(eval '-wheeled-' in arg('robot_version'))">
    <!-- 轮式机器人配置 -->
</group>

<!-- 末端类型判断 -->
<group if="$(eval '-revo1hand' in arg('robot_version'))">
    <!-- 灵巧手配置 -->
</group>

<!-- 相机判断：无相机判断（支持简写 -none 或 -none_none_none_none） -->
<group if="$(eval not ('-none-' in arg('robot_version') or arg('robot_version').endswith('-none')))">
    <!-- 有相机的配置 -->
</group>
<group if="$(eval 'Gemini330' in arg('robot_version'))">
    <!-- Gemini330 相机配置 -->
</group>
```

#### 2.4 Shell 脚本中的判断

```bash
ROBOT_VERSION="${ROBOT_VERSION:-kuavo-4pro-biped-revo1hand-Gemini330}"  # 简写，等价于 Gemini330_none_none_none"

# 品牌判断
if [[ "$ROBOT_VERSION" == kuavo-* ]]; then
    echo "Kuavo 系列"
elif [[ "$ROBOT_VERSION" == roban-* ]]; then
    echo "Roban 系列"
fi

# 运动方式判断
if [[ "$ROBOT_VERSION" == *-biped-* ]]; then
    echo "双足机器人"
elif [[ "$ROBOT_VERSION" == *-wheeled-* ]]; then
    echo "轮式机器人"
fi

# 末端类型判断
if [[ "$ROBOT_VERSION" == *-revo1hand* ]]; then
    echo "有灵巧手"
elif [[ "$ROBOT_VERSION" == *-lejuclaw* ]]; then
    echo "有夹爪"
elif [[ "$ROBOT_VERSION" == *-dummy* ]]; then
    echo "展厅版（塑料假手）"
elif [[ "$ROBOT_VERSION" == *-none* ]]; then
    echo "无末端"
fi

# 相机判断（支持简写）
if [[ "$ROBOT_VERSION" == *-none ]] || [[ "$ROBOT_VERSION" == *-none_*_none ]]; then
    echo "无相机"
else
    echo "有相机配置"
fi

# 提取各字段
ROBOT_ID="${ROBOT_VERSION%%+*}"          # 去掉标签
BRAND="${ROBOT_ID%%-*}"                   # kuavo
PRODUCT=$(echo "$ROBOT_ID" | cut -d'-' -f2) # 4pro
CAMERA=$(echo "$ROBOT_ID" | cut -d'-' -f5) # Gemini330（=Gemini330_none_none_none）
```

### 3. 目录结构（Next 版本）

```
src/kuavo_assets/
├── models/
│   ├── roban-2v1-biped-none-none/                      # 无末端，无相机（简写）
│   ├── kuavo-4-biped-none-none/                        # 无末端，无相机（简写）
│   ├── kuavo-4V2-biped-revo1hand-none/                 # 灵巧手，无相机（简写）
│   ├── kuavo-4pro-biped-revo1hand-Gemini330/           # 头部相机（简写）
│   ├── kuavo-4pro-biped-lejuclaw-none/                  # 夹爪，无相机（简写）
│   ├── kuavo-4pro-biped-dummy-none/                    # 展厅版，无相机（简写）
│   ├── kuavo-4proEDU-biped-revo1hand-Gemini330/        # 教育版，头部相机（简写）
│   ├── kuavo-4proEDU-biped-dummy-none/                 # 教育版展厅，无相机（简写）
│   ├── kuavo-5-biped-none/                             # 无末端，无相机（简写）
│   ├── kuavo-5V2-biped-none/                           # 无末端，无相机（简写）
│   └── kuavo-5-wheeled-none-none/                      # 轮式无末端，无相机（简写）
└── config/
    ├── roban-2v1-biped-none-none/                      # 使用简写作为目录名
    ├── kuavo-4pro-biped-revo1hand-Gemini330/           # 使用简写作为目录名
    ├── kuavo-4proEDU-biped-revo1hand-Gemini330/        # 使用简写作为目录名
    └── ...
```

### 4. CMake 使用方式

```cmake
# 读取环境变量
if(DEFINED ENV{ROBOT_VERSION})
    set(ROBOT_VERSION $ENV{ROBOT_VERSION})
else()
    set(ROBOT_VERSION "kuavo-4pro-biped-revo1hand-Gemini330")  # 简写，等价于 Gemini330_none_none_none
endif()

# 提取 robot_id（去掉标签）
string(REGEX REPLACE "\\+.*" "" ROBOT_ID ${ROBOT_VERSION})

# 传递给 C++ 代码
add_compile_definitions(ROBOT_VERSION="${ROBOT_VERSION}")
add_compile_definitions(ROBOT_ID="${ROBOT_ID}")
```

### 5. 对比总结

| 场景 | 旧格式 | 新格式 |
|------|--------|--------|
| 环境变量 | `ROBOT_VERSION=45` | `ROBOT_VERSION=kuavo-4pro-biped` / `kuavo-4pro-biped-revo1hand-Gemini330_none_none_none` |
| 资源目录 | `biped_s45` | `kuavo-4pro-biped` / `kuavo-4pro-biped-revo1hand-Gemini330_none_none_none` |
| 配置目录 | `kuavo_v45` | `kuavo-4pro-biped` / `kuavo-4pro-biped-revo1hand-Gemini330_none_none_none` |
| 品牌判断 | `version >= 40` | `rv.is_kuavo` 或 `kuavo-*` |
| 系列判断 | `version >= 40 && version < 50` | `rv.product.startswith("4")` |
| 运动方式 | 无法直接判断 | `rv.is_biped` 或 `*-biped-*` |
| 末端类型 | 无法直接判断 | `HAS_REVO1HAND(rv)` 或 `*-revo1hand*` |
| 相机信息 | 无法直接判断 | `rv.get_camera("head")` 或 `*-Gemini330_*_*_*` |
| 展厅版 | `version >= 100000` | `HAS_DUMMY(rv)` 或 `*-dummy*` |

---
