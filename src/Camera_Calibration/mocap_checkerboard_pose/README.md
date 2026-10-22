# mocap_checkerboard_pose

通过 Motive 或青瞳动捕获取 **checkerboard 相对 l_shoulder / torso** 的实测 6D 位姿，并可选写入 URDF `checkerboard_joint`。

## 目录

```
mocap_checkerboard_pose/
├── config/bodies.yaml
├── record_mocap_poses.py      # ROS 采集 → CSV
├── process_mocap_poses.py     # CSV → 滤波 → JSON
├── apply_checkerboard_to_urdf.py  # JSON → URDF checkerboard_joint
├── start_record.sh            # 一键采集 + 处理
├── mocap_pose_utils.py
└── README.md

# 采集输出（与本目录同级，带时间戳）
├── mocap_poses_YYYYMMDD_HHMMSS.csv
└── checkerboard_relative_poses_YYYYMMDD_HHMMSS.json
```

## 操作步骤

### 0. 头部联合标定入口（推荐）

若目标是完成“动捕确定棋盘 URDF 位姿 + 头部相机采集 + 头部零位优化”，直接运行：

```bash
cd /path/to/kuavo-ros-control
bash src/Camera_Calibration/mocap_checkerboard_pose/run_interactive_head_calibration.sh
```

完整模式依次执行：

```text
检查 checkerboard / torso / l_shoulder 动捕数据
        ↓
静止采集并生成 checkerboard_relative_poses.json
        ↓
把 checkerboard_in_l_shoulder 写入当前机型 URDF
        ↓
头部逐姿态稳定后，每个姿态采一帧棋盘观测
        ↓
优化 zhead_1_joint / zhead_2_joint，自动生成报告和误差图
        ↓
write_zero.py --parts head --dry-run，人工确认后可正式写零
```

也可从 `mocap_joint_calib/run_interactive_calibration.sh` 的菜单 `10` 进入同一流程。脚本支持
`ROBOT_VERSION=45/52/56/62/63`，并为每次头部采集创建独立的时间戳 CSV 目录，避免误用旧数据。

如果希望手臂纯动捕标定与头部棋盘数据只运动一轮，选择该脚本的菜单 `11`。它仍先执行本目录的
棋盘动捕和 URDF 写入，然后启动头部 `capture_to_csv`，由双臂采集器在每个有效静止点同时触发
一张头部棋盘观测。9 个头部样本与 9 个手臂有效样本严格一一对应，双臂与头部随后分别优化，
不会把两类残差混进同一个求解器。

脚本不会自动启动运控、相机驱动或动捕接收节点。运行前需保证 `/sensors_data_raw`、头部图像与
`camera_info`、三个动捕刚体话题持续发布；S45 还需保证 180° 旋转后的头部图像和 `camera_info`
话题已启动。多机型选择会自动切换 URDF 和相机话题，动捕刚体的工装偏移由
`config/bodies.yaml` 及当前机型定义；更换工装或改变刚体安装方式后必须重新测量偏移。

> 头部标定配置的统一 FK 根是 `zarm_l1_ref_link`，所以联合流程固定使用
> `checkerboard_in_l_shoulder` / `--mode left_shoulder`。`checkerboard_in_torso` 仅供其他分析使用，
> 不能直接替换到当前头部标定链中。

### 1. Motive 侧

| 步骤 | 操作 | 完成标志 |
|------|------|----------|
| 1 | 创建刚体：`checkerboard`、`torso`、`l_shoulder` | 3 个刚体均为 Tracked |
| 2 | 开启 **NatNet Streaming** | Streaming Active |
| 3 | Live 模式，3 刚体连续稳定 ≥ 5s | 不全绿时不要采集 |

### 2. ROS 接收

**修改 IP**：编辑 [`optitrack_data_receive.launch`](../../automatic_test/optitrack_data_receive/launch/optitrack_data_receive.launch) 中的 `mocap_server_ip`（默认 `192.168.8.217`）。

**终端 1**：

```bash
cd /root/kuavo_ws
source devel/setup.bash
roslaunch optitrack_data_receive optitrack_data_receive.launch
```

临时指定 IP（不改 launch 文件）：

```bash
roslaunch optitrack_data_receive optitrack_data_receive.launch mocap_server_ip:=192.168.10.14
```

**验证**（另开终端）：

```bash
source /root/kuavo_ws/devel/setup.bash
rostopic hz /checkerboard_pose /torso_pose /l_shoulder_pose
rostopic echo /checkerboard_pose -n 1
```

### 3. 采集 + 处理

前置：对应动捕系统已开流、ROS 话题正常、机器人与标定板 **完全静止**。

**终端 2（推荐一键）**：

```bash
cd /root/kuavo_ws
source devel/setup.bash

# Motive
bash src/Camera_Calibration/mocap_checkerboard_pose/start_record.sh motive 45

# 青瞳
bash src/Camera_Calibration/mocap_checkerboard_pose/start_record.sh qingtong 45
```

不传动捕源时，使用 `config/bodies.yaml` 中的 `mocap_source`；默认是 `motive`，青瞳现场请显式传 `qingtong`。

输出文件在 `mocap_checkerboard_pose/` 目录下，例如：

- `mocap_poses_20250623_153045.csv`
- `checkerboard_relative_poses_20250623_153045.json`

**或分步手动**（在 `mocap_checkerboard_pose/` 目录下）：

```bash
cd /root/kuavo_ws/src/Camera_Calibration/mocap_checkerboard_pose
source /root/kuavo_ws/devel/setup.bash

# 采集（约 12s 自动结束）
python3 record_mocap_poses.py \
  --mocap-source qingtong \
  --duration 10 --warmup 2 \
  --output mocap_poses.csv

# 离线处理
python3 process_mocap_poses.py \
  --robot-version 45 \
  --input mocap_poses.csv \
  --output checkerboard_relative_poses.json
```

**采集时序**：

| 阶段 | 默认 | 行为 |
|------|------|------|
| 等待就绪 | ≤30s | checkerboard、torso、l_shoulder 均有效 |
| 预热 | 2s | 不写 CSV |
| 正式采集 | 10s | 写 CSV，到点自动退出 |

### 4. 停止

| 组件 | 停止方式 |
|------|----------|
| 采集脚本 | `--duration` 到点自动退出 |
| ROS 接收 | 终端 1 `Ctrl+C` |
| Motive | 全部完成后可关 Streaming |

### 5. 结果验收

JSON 中 `statistics`：平移 std **< 1 mm**、旋转 std **< 0.1°** 为稳定。

## 输出说明

- **CSV**：3 刚体 6D 位姿，位置 mm、四元数 xyzw
- **JSON** 关键字段：
  - `checkerboard_in_l_shoulder` / `checkerboard_in_torso`
  - `xyz`：米；`rpy`：弧度（ZYX）

## 工装 → link 偏移

动捕跟踪的是**工装刚体**，`bodies.yaml` 中 `link_offset_mm` 为工装原点在对应 **link 系**下的位置 (mm)。  
`process_mocap_poses.py` 会先修正为 link 位姿再算相对关系：

```
p_link = p_tooling - R @ link_offset_mm
```

当前偏移：

| 刚体 | link_offset_mm |
|------|----------------|
| checkerboard | [-203.93, -153.93, 11.50] |
| l_shoulder (S45) | [0, -68.50, 105.00] |
| l_shoulder (52/56/62/63) | [0, -53.50, 105.00] |
| torso | [120.02, -50.82, 6.0] |

`checkerboard` 和 torso 在 `config/bodies.yaml` 中维护；左肩会由 `--robot-version` 选择固定值。CSV 仍保存
**原始动捕工装位姿**；偏移仅在离线处理时应用。

### 6. 写入 URDF checkerboard_joint

使用 [`apply_checkerboard_to_urdf.py`](apply_checkerboard_to_urdf.py) 将 JSON 写入对应机型 URDF。脚本默认读取 `ROBOT_VERSION`（45→S45、52→默认 biped、56→S56、62/63→S62），也可用 `--urdf` 显式覆盖：

**模式 1 — 左肩（parent = `zarm_l1_ref_link`）**  
使用 JSON 中 `checkerboard_in_l_shoulder`：

```bash
cd src/Camera_Calibration/mocap_checkerboard_pose
export ROBOT_VERSION=45

python3 apply_checkerboard_to_urdf.py \
  --json checkerboard_relative_poses.json \
  --mode left_shoulder
```

**模式 2 — 腰部（parent = `waist_yaw_link`）**  
使用 JSON 中 `checkerboard_in_torso`：

```bash
python3 apply_checkerboard_to_urdf.py \
  --json checkerboard_relative_poses.json \
  --mode waist
```

写入前会自动备份所选 URDF 为 `<原文件>.bak.<时间戳>`。
若需输出到新文件而不覆盖：

```bash
python3 apply_checkerboard_to_urdf.py \
  --json checkerboard_relative_poses.json \
  --mode left_shoulder \
  --output ../biped_v3_arm_s62_mocap.urdf
```

## 坐标系说明

- 测量在 Motive 全局系 `mocap_frame` 下
- 世界系会在计算相对位姿时消去，但各动捕刚体自身坐标轴与对应 URDF link **无自动姿态对齐**；
  建刚体时仍需分别对齐 `checkerboard_link`、torso link 和 `zarm_l1_ref_link`

## 依赖

- ROS Noetic/Melodic + `optitrack_data_receive`
- Python3：`numpy`、`pyyaml`、`rospy`
