# mocap_checkerboard_pose

通过 Motive 动捕获取 **checkerboard 相对 l_shoulder / torso** 的实测 6D 位姿，并可选写入 URDF `checkerboard_joint`。

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

前置：Motive 开流、ROS 话题正常、机器人与标定板 **完全静止**。

**终端 2（推荐一键）**：

```bash
cd /root/kuavo_ws
source devel/setup.bash
bash src/Camera_Calibration/mocap_checkerboard_pose/start_record.sh
```

输出文件在 `mocap_checkerboard_pose/` 目录下，例如：

- `mocap_poses_20250623_153045.csv`
- `checkerboard_relative_poses_20250623_153045.json`

**或分步手动**（在 `mocap_checkerboard_pose/` 目录下）：

```bash
cd /root/kuavo_ws/src/Camera_Calibration/mocap_checkerboard_pose
source /root/kuavo_ws/devel/setup.bash

# 采集（约 12s 自动结束）
python3 record_mocap_poses.py \
  --duration 10 --warmup 2 \
  --output mocap_poses.csv

# 离线处理
python3 process_mocap_poses.py \
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

当前偏移（可在 `config/bodies.yaml` 修改）：

| 刚体 | link_offset_mm |
|------|----------------|
| checkerboard | [-153.93, -203.93, 11.50] |
| l_shoulder | [0, 30.25, 105.00] |
| torso | [122.02, -45.82, 2.50] |

CSV 仍保存**原始动捕工装位姿**；偏移仅在离线处理时应用。

### 6. 写入 URDF checkerboard_joint

使用 [`apply_checkerboard_to_urdf.py`](apply_checkerboard_to_urdf.py) 将 JSON 写入 [`biped_v3_arm_s62.urdf`](../biped_v3_arm_s62.urdf)：

**模式 1 — 左肩（parent = `zarm_l1_ref_link`）**  
使用 JSON 中 `checkerboard_in_l_shoulder`：

```bash
cd src/Camera_Calibration/mocap_checkerboard_pose

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

写入前会自动备份原 URDF 为 `biped_v3_arm_s62.urdf.bak.<时间戳>`。  
若需输出到新文件而不覆盖：

```bash
python3 apply_checkerboard_to_urdf.py \
  --json checkerboard_relative_poses.json \
  --mode left_shoulder \
  --output ../biped_v3_arm_s62_mocap.urdf
```

## 坐标系说明

- 测量在 Motive 全局系 `mocap_frame` 下
- 与 URDF `zarm_l1_ref_link` **无自动对齐**

## 依赖

- ROS Noetic/Melodic + `optitrack_data_receive`
- Python3：`numpy`、`pyyaml`、`rospy`
