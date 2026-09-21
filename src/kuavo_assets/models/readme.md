# kuavo 各版本末端执行器与相机配置对照

汇总各版本 kuavo 机器人的**末端执行器类型**、**末端坐标系 y 方向偏移**、**末端关节在仿真中能否运动**，以及**头部 / 手腕 / 腰部相机**配置。

- 适用包：`kuavo_assets`
- 模型目录：`models/biped_s${ROBOT_VERSION}/`
- 配置目录：`config/kuavo_v${ROBOT_VERSION}/kuavo.json`
- 运行时版本号由环境变量 `ROBOT_VERSION` 决定

---

## 1. 全版本对照表

> 每列的判定依据见 [第 3 节](#3-列含义与判定依据)。

### #4代

| 版本号 | 末端执行器 | 末端坐标在y方向偏移 | 末端执行器关节运动 | 头部相机 | 手腕相机 | 备注说明 |
|---|---|---|---|---|---|---|
| biped40 | 强脑手 | 是 | 否 | 有 | 无 | |
| biped41 | 强脑手 | 是 | 否 | 有 | 无 | |
| biped42 | 强脑手 | 是 | 否 | 有 | 无 | |
| biped43 | 强脑手 | 是 | 否 | 有 | 无 | |

### #4pro

| 版本号 | 末端执行器 | 末端坐标在y方向偏移 | 末端执行器关节运动 | 头部相机 | 手腕相机 | 备注说明 |
|---|---|---|---|---|---|---|
| biped45 | 强脑手 | 是 | 否 | 有 | 有 | |
| biped46 | 强脑手 | 是 | 否 | 有 | 无 | 头部和躯干与 s45 不同 |
| biped47 | 二指夹爪 | 是 | 是 | 有 | 无 | 实机启动为 lejuclaw |
| biped48 | 强脑手 | 是 | 是 | 有 | 无 | 灵巧手能够活动 |
| biped49 | 强脑手 | 是 | 是 | 有 | 有 | 增加手腕相机 |
| biped_s100045 | 强脑手 | 是 | 否 | 有 | 无 | 小版本，主要是末端不同 |
| biped_s100049 | 强脑手 | 是 | 否 | 有 | 无 | |
| biped_s200049 | 二指夹爪 | 否 | 是 | 有 | 有 | |
| biped_s300049 | 灵心巧手L6 | 是 | 是 | 有 | 有 | |
| biped_s400049 | 灵心巧手o6 | 是 | 是 | 有 | 有 | |

### #5代人形

| 版本号 | 末端执行器 | 末端坐标在y方向偏移 | 末端执行器关节运动 | 头部相机 | 手腕相机 | 备注说明 |
|---|---|---|---|---|---|---|
| biped50 | 强脑手 | 是 | 否 | 有 | 无 | 躯干不同，相机安装有误 |
| biped51 | 强脑手 | 是 | 否 | 有 | 无 | 加了腰部相机 |
| biped52 | 强脑手 | 否 | 否 | 有 | 无 | 躯干做了改进 |
| biped53 | 强脑手 | 否 | 否 | 有 | 无 | |
| biped54 | 强脑手 | 否 | 否 | 有 | 无 | |
| biped55 | 强脑手 | 否 | 否 | 有 | 无 | |
| biped56 | 强脑手 | 否 | 否 | 有 | 无 | |
| biped_s200053 | 二指夹爪 | 否 | 是 | 有 | 有 | |
| biped_s300053 | 强脑手 | 是 | 是 | 有 | 有 | |

### #5W

| 版本号 | 末端执行器 | 末端坐标在y方向偏移 | 末端执行器关节运动 | 头部相机 | 手腕相机 | 备注说明 |
|---|---|---|---|---|---|---|
| biped60 | 强脑手 | 否 | 否 | 有 | 无 | |
| biped61 | 强脑手 | 否 | 否 | 有 | 无 | 改进了底盘和支腿 |
| biped62 | 强脑手 | 否 | 否 | 有 | 无 | 底盘不同 |
| biped63 | 强脑手 | 否 | 否 | 有 | 无 | 底盘不同 |
| biped_s200062 | 二指夹爪 | 否 | 是 | 有 | 有 | |
| biped_s300062 | 强脑手 | 是 | 是 | 有 | 有 | |

### 遗留

| 版本号 | 末端执行器 | 末端坐标在y方向偏移 | 末端执行器关节运动 | 头部相机 | 手腕相机 | 备注说明 |
|---|---|---|---|---|---|---|
| biped70 | — | — | — | — | — | 只有一堆 mesh，没有完整文件 |

---

## 2. 末端执行器类型

| 类型 | 代码标识 | 控制器 | 关节数 | 说明 |
|---|---|---|---|---|
| 强脑手 | `qiangnao` | `DexhandController` | 12（6 关节 × 2 手） | 灵巧手，逐关节位置控制 |
| 灵心巧手 | `linker_hand` | `DexhandController`（`is_linker_hand=true`） | 12 | 复用同一控制器，仅映射表不同 |
| 二指夹爪 | `lejuclaw` | `LejuClawController` | 2 | 自研夹爪，**不走 `endEffectorCommand()`** |
| 触觉灵巧手 | `qiangnao_touch` | `DexhandController`（touch 模式） | 12 | 固定 485 协议 |
| 气泵 | `qibeng` | — | — | |
| — | `none` | 无 | 0 | 无末端 |

### 2.1 运行时判定链路

判断"实机实际装的是哪个末端"的唯一可靠依据：

```
kuavo.json : EndEffectorType
  → KuavoSettings::loadHardwareSettings()      // 字符串 → EndEffectorType 枚举
  → hardware_settings.end_effector_type
  → HardwarePlant::initEndEffector()           // 各分支互斥
  → HardwareNode::init()
       nh_.setParam("end_effector_type", ...)
       nh_.setParam("end_effector_joints_num", ...)
```

`end_effector_joints_num` 是运行时可直接观测的量，可反查实际末端：

| 观测值 | 对应末端 |
|---|---|
| `12` | `qiangnao` / `linker_hand` / `qiangnao_touch` |
| `2` | `lejuclaw` |
| `0` | `none`（未设置） |

```bash
rosparam get /end_effector_type
rosparam get /end_effector_joints_num
```

> ⚠️ **不要只看 `kuavo.json`。** 其中声明的类型可能滞后于实际装配。例如 `config/kuavo_v47/kuavo.json` 写的是 `["qiangnao","qiangnao"]`，但 s47 实机启动的是 `lejuclaw`。实际末端以 `~/.config/lejuconfig/canbus_device_cofig.yaml` 里的 CAN 设备类型为准（由 `tools/check_tool/canbus_config.sh` 写入）。

### 2.2 切换末端类型

```bash
# 方式一：菜单式配置（会同时改写 kuavo.json 与相关 launch 的 ee_type 默认值）
bash ~/kuavo-ros-control/tools/setup-kuavo-ros-control.sh
#   → setup_end_effector():  1)灵巧手  2)二指夹爪  3)触觉灵巧手  4)没有灵巧手

# 方式二：直接改类型（支持 qiangnao / lejuclaw / none / qiangnao_touch / qibeng / linker_hand）
bash ~/kuavo-ros-control/tools/check_tool/change_EndEffectorType.sh

# 方式三：CAN 设备类型（决定实机挂的是什么硬件）
bash ~/kuavo-ros-control/tools/check_tool/canbus_config.sh
#   → 末端菜单: revo1_hand(灵巧手) / revo2_hand(灵巧手) / lejuclaw(自研夹爪) / none(没有末端)
```

---

## 3. 列含义与判定依据

### 3.1 末端坐标在y方向偏移

**判定源：主 urdf 中 `zarm_l7_end_effector_joint` 的 `<origin xyz>`**

```
models/biped_s${N}/urdf/biped_s${N}.urdf
```

- 左手 y 分量为 `-0.03` → **是**（右手为镜像值 `+0.03`）
- 左手 y 分量为 `0.0` / `-0.0` → **否**

各版本实测值：

| 值 | 版本 |
|---|---|
| `0 -0.03 -0.17` → **是** | s40, s41, s42, s43, s45, s46, s47, s48, s49, s50, s51, s100045, s100049, s300049, s400049 |
| `0 -0.0 -0.17` → **否** | s200049 |
| `0 0.0 -0.17` → **否** | s52, s53, s54, s55, s56, s200053, s200062, s300053, s300062 |
| `0 0.0 -0.17` → **否** | s60, s61, s62, s63 |

> **`kuavo.json` 里没有任何"末端 y 偏移"的键。** 容易混淆的键有两类，都不是这一列：
>
> - `eef_z_offset` —— 是 **z** 方向（所有在用版本均为 `-0.0`；仅遗留版本 v13/v14/v16 为 `-0.17`、v17 为 `-0.2175`）
> - `drake_velocity_ik_bounds` 块下的 `left_y_upper_offset` / `right_y_lower_offset` —— 是 **Drake 速度 IK 的关节空间限位**，与末端安装偏置无关。在用版本（v45/v47–v49/v52–v56/v100045/v100049/v200049/v200053/v300049/v300053/v400049）一律为 `0.5 / -0.5`；v40–v43、v46、v50、v51、v60–v63、v200062、v300062、v70 **无此块**。

### 3.2 末端执行器关节运动

表示**仿真（MuJoCo）时末端关节能否运动**。

**判定源：`models/biped_s${N}/xml/biped_s${N}.xml` 中手部 actuator 的数量**

- actuator 数 `0` → **否**（模型里没有可驱动的末端关节）
- actuator 数 `> 0` → **是**

两种驱动写法：

```xml
<!-- 腱驱动夹爪：1 个 actuator 经 tendon 带动整只手指 -->
<general biastype="affine" name="l_fingers_actuator" tendon="l_claw_split"/>

<!-- 灵巧手：每个关节一个 position motor -->
<position gear="1" joint="l_thumbCMC" name="l_thumbCMC_motor"
          ctrllimited="true" ctrlrange="0 1.5708"/>
```

> `是` **不等于**该版本实机装了可动末端。例如 s47 的 xml 有 2 个腱驱动 actuator（仿真里夹爪能开合），而 s40–s43 的强脑手在 xml 里**完全没有** actuator 与手部关节。

### 3.3 头部相机 / 手腕相机 / 腰部相机

**判定源：主 urdf 中的相机 `<link>` 及其父 link**

| 相机 | 典型 link 名 | 挂载父 link |
|---|---|---|
| 头部相机 | `camera` / `camera_base` / `head_camera` / `head_camera_base` | `zhead_2_link` |
| 手腕相机 | `l/r_hand_camera` | `l/r_hand_tripod`，或 `zarm_l7_link` / `zarm_r7_link` |
| 手腕相机（二指夹爪） | `l/r_d405_camera_connect` | `l/r_twofinger_base` |
| 手腕相机（s300053/s300062） | `l_camera_link` / `r_camera_link_connect` | `zarm_l7_link` / `zarm_r7_link` |
| 腰部相机 | `waist_camera` / `waist_camera_base` | `base_link` 或 `waist_yaw_link` |

> 手腕相机**只存在于 urdf**，mujoco xml 中没有对应 link。

### 3.4 证据明细

| 模型 | xml 手部 actuator | xml 手部关节 | urdf 手部关节 | 末端类型 | 腰部相机 |
|---|---|---|---|---|---|
| s40 – s43 | 0 | 0 | 0 | 无 | 无 |
| s45 | 0 | 0 | 4（仅 `l/r_hand_tripod`、`l/r_hand_camera`） | 无 | **有**（`torso-camera`、`waist_camera` ← `base_link`） |
| s46 | 0 | 0 | 0 | 无 | 无 |
| s47 | **2** | 16 | 18 | 腱驱动夹爪 | 无 |
| s48 | **20** | 20 | 22 | 强脑灵巧手 10/手 | 无 |
| s49 | **20** | 20 | 26 | 强脑灵巧手 10/手 | 无 |
| s50 | 0 | 0 | 0 | 无 | **有**（`waist_camera` ← `base_link`） |
| s51 | 0 | 0 | 0 | 无 | **有**（`waist_camera` ← `base_link`） |
| s52 – s56 | 0 | 0 | 0 | 无 | **有**（`waist_camera` ← `waist_yaw_link`） |
| s54 | **20** | 20 | **0**（手仅在 xml 中） | 强脑灵巧手 10/手 | **有**（同 s52–s56） |
| s60 | 0 | 0 | 0 | 无 | 无（仅 `waist_link`、`waist_yaw_link`，无相机） |
| s61 – s63 | 0 | 0 | 0 | 无 | 无（同上） |
| s100045 / s100049 | 0 | 0 | 0 | 无 | 无 |
| s200049 | **2** | 16 | 18 | 腱驱动夹爪 | 无 |
| s200053 | **2** | 20 | 22 | 腱驱动夹爪 | **有**（`waist_camera` ← `waist_yaw_link`） |
| s200062 | **2** | 20 | 22 | 腱驱动夹爪 | 无 |
| s300049 | **22** | 42 | 44 | 灵心巧手（3-DOF 拇指） | 无 |
| s300053 | **20** | 20 | 22 | 强脑灵巧手 10/手 | **有**（`waist_camera` ← `waist_yaw_link`） |
| s300062 | **20** | 20 | 22 | 强脑灵巧手 10/手 | 无 |
| s400049 | **42** | 42 | 28 | 两套并存（20 强脑 + 22 灵心） | 无 |
| s70 | — | — | — | 无 urdf/xml，仅 65 个 STL | — |

补充说明：

- **`s54` 的手部几何只在 xml 中**：`models/biped_s54/xml/biped_s54.xml` 有 20 个手部关节 / 20 个 actuator；其 urdf 仅含 4 个足部关节（`l/r_foot_heel_joint`、`l/r_foot_toe_joint`）。`xml/scene.xml` 通过 `<include file="biped_s54.xml"/>` 引入。
- **`s300049` 的 xml 含两套命名并存的手部关节**（共 42 个，如 `l_thumbCMC` 与 `l_thumb_cmc_yaw` / `l_thumb_cmc_pitch` / `l_thumb_ip` 同时存在），但只有 **22** 个 actuator（灵心风格），强脑风格的那批关节在仿真中**不可驱动**。
- **`s400049` 的 42 个手部关节全部有 actuator**（20 强脑风格 + 22 灵心风格，两套并存）。
- **`s200062` 的 urdf 与 xml 对末端形态的表述不一致**：urdf 有 22 个手部关节（与 s300062 相同），xml 却只有 2 个腱驱动 actuator。实机末端须以 2.1 节的运行时链路为准。
- **`s200053` / `s200062` 的 xml 是腱驱动二指夹爪**，但对应 `kuavo.json` 的 `EndEffectorType` 写的是 `["qiangnao","qiangnao"]`。
- **`v51` / `v60` / `v61` 的 `EndEffectorType` 是大写 `["None","None"]`。** `kuavo_settings.cpp` 的映射表只有小写 `"none"`，`std::map::operator[]` 对未命中键会**静默插入默认值**（枚举值 0 = `none`），因此大写形式等价于 `none` 且**不报错**。
- **s47 与 s200049 的二指夹爪直接挂在 `zarm_l7_link` 上**（经 `l_f_bar-1_joint` → `l_f_bar-2` → `l_f_fingers`），**不经过** `zarm_l7_end_effector`。因此这两个机型的 `zarm_l7_end_effector_joint` y 偏移可以不同（`-0.03` vs `-0.0`）而不影响夹爪实际安装位置。
- **s49 起头部相机朝向翻转**：s40–s48 的 `camera` rpy 为 `0 0.4887 0`，s49 及之后为 `3.14159 0.3840 0`（roll 翻转 π，pitch 改为 0.3840）。
- **s60 → s61 头部相机原点变化**：s60 的 `camera_base` origin 为 `0.099478 0 0.032897`（rpy pitch 0.35805），s61 及之后为 `0.099619 0 0.032844`（pitch 0.3406）。
- **s50 / s51 头部相机原点差异**（对应表中 s50 备注「相机安装有误」）：s49 与 s50 的头部相机 origin 完全相同（`camera` = `0.102420738776518 -0.0475000000000003 0.126192926717169`，rpy `3.14159 0.38397 0`；`camera_base` = `0.0932759773834307 0 0.145322661773667`），而 s51 只有 `camera_base`，origin 为 `0.119717018973535 0.0237514612535074 0.0262044145991832`（rpy `0 0.48022 0`）。
- **s50 / s51 的腰部相机均存在**（对应表中 s51 备注「加了腰部相机」）：两版 `waist_camera` 的父 link 都是 `base_link`；s50 另有 `torso-camera`，s51 无。

---

## 4. 自查方法

```bash
cd ~/kuavo-ros-control/src/kuavo_assets

# 1) 末端坐标 y 偏移（左手）
grep -A6 'joint name="zarm_l7_end_effector_joint"' \
     models/biped_s${ROBOT_VERSION}/urdf/biped_s${ROBOT_VERSION}.urdf | grep origin

# 2) 仿真末端关节能否运动
grep -cE 'fingers_actuator|_motor"' models/biped_s${ROBOT_VERSION}/xml/biped_s${ROBOT_VERSION}.xml

# 3) 相机 link 及父 link
grep -nE '<link name="[^"]*camera[^"]*"' models/biped_s${ROBOT_VERSION}/urdf/biped_s${ROBOT_VERSION}.urdf

# 4) kuavo.json 声明的末端类型
python3 -c "import json;print(json.load(open('config/kuavo_v${ROBOT_VERSION}/kuavo.json'))['EndEffectorType'])"

# 5) 实机实际末端（运行时）
rosparam get /end_effector_type
rosparam get /end_effector_joints_num

# 6) 实机 CAN 设备类型（真正的权威来源）
cat ~/.config/lejuconfig/canbus_device_cofig.yaml
```
