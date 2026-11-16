# DH 夹爪控制 Demo（独立 `/dh_claw_*` 话题）

当 `kuavo.json` 中配置：

```json
"EndEffectorType": ["dhclaw", "dhclaw"]
```

整机 plant 会启动 `DhClawController`，对外使用独立 dh_claw 接口：

| 接口 | 名称 |
|------|------|
| command topic | `/dh_claw_command` |
| state topic | `/dh_claw_state` |
| control service | `/control_robot_dh_claw` |

**不要**与整机同时再启动独立的 `dhclaw_ros_node`（会抢总线/话题）。  
底层 CAN 驱动与独立调试工具见：`src/kuavo-ros-control-lejulib/hardware_plant/lib/dhclaw_driver/README.md`。

## 1. `/dh_claw_state` 状态说明

话题类型：`kuavo_msgs/dhClawState`  
`state[0]` = 左爪，`state[1]` = 右爪（消息常量，**不是** DH 寄存器原始值）。

| ROS `state` | 常量 | 含义 |
|-------------|------|------|
| `-1` | `kError` | 错误 |
| `0` | `kUnknown` | 未知 / 初始化默认 |
| `1` | `kMoving` | 运动中 |
| `2` | `kReached` | 到达目标位置 |
| `3` | `kGrabbed` | 夹住物体 |
| `4` | `kDropped` | 物体掉落 |

DH 硬件寄存器 `0x0201`（夹持状态）原始含义 → ROS 映射：

| DH 寄存器值 | 硬件含义 | → ROS `state` |
|-------------|---------|---------------|
| `0` | 运动中 | `1` (`kMoving`) |
| `1` | 到达位置 | `2` (`kReached`) |
| `2` | 夹住物体 | `3` (`kGrabbed`) |
| `3` | 物体掉落 | `4` (`kDropped`) |
| 读失败/其它 | — | `0` (`kUnknown`) |

示例：`state: [2, 2]` 表示左右都是 **到位**。

`data` 字段：

| 字段 | 是否周期读硬件 | 说明 |
|------|----------------|------|
| `position` | **是** | 每次发 state 时 CAN 读位置反馈（0~100） |
| `state` | **是** | 每次发 state 时 CAN 读夹持状态（再映射到 ROS 码） |
| `velocity` | **否** | **不是**实时速度反馈；为最近一次成功下发指令的速度 % 缓存 |
| `effort` | **否** | **不是**实时力/力矩反馈；为最近一次成功下发指令的力 % 缓存 |

> **重要（dhclaw）**：当前驱动/控制器**没有**按固定频率订阅或查询 DH 夹爪的速度、力反馈寄存器。  
> `/dh_claw_state` 里的 `velocity` / `effort` 仅作 **命令回显（last command）**，不能当作闭环力控或真实力矩监测。  
> 若从未发过指令，多为默认值（速度约 50、力约 0 或上次缓存）。  
> 真机判断到位/夹住请看 `state` 与 `position`，不要依赖 `velocity`/`effort` 变化。

### 发布与查询频率（整机）

- `hardware_node` 对 **dhclaw** 按 **50 Hz** 发布 `/dh_claw_state`（**不再**跟 `dt_≈500 Hz`）
- 原因：DH 是同步 CAN 读寄存器；与手臂共 bus 时 500Hz 会打满发送队列：`MESSAGE_QUEUE_FULL` / `read(0x0202): no reply`（CAN ID 如 `0x011`）
- controller 内 position/grip 再做 **≥20 ms** 限频缓存；定时器即使更快也只是重复发缓存
- 每次真实刷新会 CAN 查询：**位置 + 夹持状态**；**不会**额外查询速度/力反馈
- 运动执行中 `move_paw` 内部约 **20 ms** 强制刷新缓存（仅执行阶段）
- 独立节点 `dhclaw_ros_node` 为 **5 Hz**（少占 CAN）；整机已启动时不要并用
- 可视化节点（如 `HumanoidVisualizer`）**不应**再往 `/dh_claw_state` 回放 command；真反馈只由 `hardware_node` 发
- 单侧安装时：**不再**把在线侧镜像到离线侧。离线侧 `position` 固定为 0、`state=0`（`kUnknown`）；若看到左右相同，说明双侧都在线且位置碰巧一致，或右爪未成功 open/calibrate（查启动日志 `running with LEFT claw only`）
> `header.frame_id` 为 `"dh_claw"`

### 控制路径说明

| 路径 | 接口 | 说明 |
|------|------|------|
| Service | `/control_robot_dh_claw` | `dh_claw_client.py` 等；**不会**自动往 `/dh_claw_command` 再发一份 |
| Topic | `/dh_claw_command` | `dh_claw_pub.py`、VR/遥操连续流 |

两者最终都进 plant，但互不镜像；盯 command 话题时要用 pub 脚本，不能期望 client 去更新它。

## 2. 量纲（DH 百分比）

| 字段 | 范围 | 说明 |
|------|------|------|
| `position` | 0 ~ 100 | 开合位置（0 开、100 合）；plant 会四舍五入到整数 % |
| `velocity` | 1 ~ 100 | **DH 速度 %**（整数寄存器） |
| `effort` | 20 ~ 100 | **DH 力 %**（`>=20` 时直接按力百分比下发，**不是 Nm/A**） |

> 兼容说明：若 `effort < 20`，plant 仍会按 leju 电流风格做 `effort * 25` 映射。  
> 本目录 demo **统一用整数百分比**，避免小数误解；也避免走 leju 电流路径。

本目录默认参数：

| 脚本 | 打开 | 闭合 | 速度 | 力 |
|------|------|------|------|----|
| `dh_claw_client.py` / `dh_claw_pub.py` | 10 | 90 | **100** | **100** |
| `dh_claw_cylinder_pub.py` | 10 | 90 | 50 | 50 |

## 3. 脚本说明

| 脚本 | 用途 |
|------|------|
| `dh_claw_client.py` | service 开合循环（推荐真机验证） |
| `dh_claw_pub.py` | topic 开合循环 |
| `dh_claw_cylinder_pub.py` | 手臂 IK + 夹爪抓取流程示例 |

## 4. 启动

整机已 load 且 `/end_effector_type` 为 `dhclaw` 后：

```bash
cd ~/dh/kuavo-ros-control   # 按本机工作空间路径调整
source /opt/ros/noetic/setup.bash
source devel/setup.bash
# 如 master 不在本机：
# export ROS_MASTER_URI=http://kuavo_master:11311

# 推荐：service
python3 src/demo/control_dhclaw/dh_claw_client.py

# 或 topic
python3 src/demo/control_dhclaw/dh_claw_pub.py
```

快速单次：

```bash
rostopic pub -1 /dh_claw_command kuavo_msgs/dhClawCommand \
"{data: {name: ['left_claw','right_claw'], position: [20, 20], velocity: [50, 50], effort: [50, 50]}}"

rostopic echo /dh_claw_state
```

底层成功下发时可见类似日志：

```text
[DhClaw] 设置位置 位置 = 50%, 速度 = 50%, 力 = 50%
```

## 5. 单侧夹爪

plant 允许只装左或只装右：

- 有一侧 open + calibrate 成功即可整机继续（例如 `left=1 right=0`）
- 未装/无应答一侧会 disable；日志中的 `calibrate failed ... disable this side` 为**预期提示**，不是整机失败
- 两侧都失败才会 `initEndEffector` 失败
- demo 仍发左右双槽指令；缺失一侧会被忽略

## 6. 抓取圆柱任务（可选）

```bash
python3 src/demo/control_dhclaw/dh_claw_cylinder_pub.py
```

手臂目标点需按现场标定调整；夹爪开合使用 DH 速度/力百分比（默认 50/50）。
