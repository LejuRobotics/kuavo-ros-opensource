# hand_accuracy_test

实机末端 **l_hand / r_hand** 动捕精度测试：双臂同步执行 teach JSON，每点静止 5s 后采集 Motive 与 TF，经 `link_offset` 修正后对比左右末端在 `waist_yaw_link`（动捕 `torso`）系下的位置误差。

## 目录

```
hand_accuracy_test/
├── config/hand_accuracy.yaml    # teach 路径、偏移、TF、时序
├── run_hand_accuracy_test.py    # 主控：下发 + 采集 + 对比 + 报告
└── README.md

# 测试输出（本目录，带时间戳）
├── hand_accuracy_report_YYYYMMDD_HHMMSS.json
└── hand_accuracy_report_YYYYMMDD_HHMMSS.csv
```

## 对比约定

| 项目 | 说明 |
|------|------|
| 动捕 `torso` | 对应机器人 `waist_yaw_link` |
| 动捕位姿 | 工装刚体，需减 `link_offset_mm` 得到 link 位姿 |
| TF | `waist_yaw_link` → `zarm_l7_end_effector` / `zarm_r7_end_effector` |
| 坐标对齐 | 可选 `mocap_frame_align`：Motive torso 系 → URDF waist 系 |
| 误差 | \(\|p_\text{mocap\_hand\_in\_torso} - p_\text{tf\_ee\_in\_waist}\|\)（mm） |

## 前置条件

1. **Motive**：创建并跟踪刚体 `l_hand`、`r_hand`、`torso`，开启 NatNet Streaming。
2. **ROS 动捕接收**（终端 1）：

```bash
cd /root/kuavo_ws
source devel/setup.bash
roslaunch optitrack_data_receive optitrack_data_receive.launch
# 或指定 IP：
# roslaunch optitrack_data_receive optitrack_data_receive.launch mocap_server_ip:=192.168.10.14
```

验证话题：

```bash
rostopic hz /l_hand_pose /r_hand_pose /torso_pose
```

3. **机器人**：已运行，`/sensors_data_raw` 有效，`/tf` 含 `waist_yaw_link`、`zarm_l7_end_effector`、`zarm_r7_end_effector`。
4. **Teach 文件**：默认使用 `teach_capture_output/teach_left_joint_test.json` 与 `teach_right_joint_test.json`（可在 yaml 中修改）。

## 运行

**终端 2**：

```bash
cd /root/kuavo_ws
source devel/setup.bash
python3 src/Camera_Calibration/hand_accuracy_test/run_hand_accuracy_test.py
```

自定义配置：

```bash
cd /root/kuavo_ws
source devel/setup.bash
python3 src/Camera_Calibration/hand_accuracy_test/run_hand_accuracy_test.py \
  --config src/Camera_Calibration/hand_accuracy_test/config/hand_accuracy.yaml
```

## 流程说明

1. 等待 `/sensors_data_raw` 有效，切换 wheel62 外控 + quick mode + 轨迹插补。
2. 双臂同步插值到 teach 各关键点（`move_duration=2s`）。
3. 每点到位后 **静止 5s**（`hold_sec`），再在 **1s** 窗口内采集动捕；同时读取 TF。
4. 动捕：扣 `link_offset_mm` → 算 `l_hand`/`r_hand` 相对 `torso` 位姿 → 3σ 滤波取均值。
5. 与 TF 平移对比，输出 JSON / CSV / 终端摘要（默认告警阈值 5 mm）。

## 配置要点（`config/hand_accuracy.yaml`）

| 字段 | 默认 | 说明 |
|------|------|------|
| `hold_sec` | 5.0 | 到位后静止时长 |
| `mocap_collect_sec` | 1.0 | 静止后动捕采集窗口 |
| `skip_first_last` | false | true 时跳过首尾零位点 |
| `warn_threshold_mm` | 5.0 | 终端摘要 WARN 阈值 |
| `link_offset_mm` | 见 yaml | l_hand / r_hand / torso 工装偏移（**URDF link 系** [x,y,z] mm） |
| `mocap_frame_align.enabled` | true | 对比前将 hand_in_torso 变到 URDF parent 系 |
| `mocap_frame_align.axis_flip` | `[1,-1,1]` | Y 轴镜像时启用；对齐 Motive 后改 `enabled: false` |

## 坐标系对齐（重要）

Motive Streaming 设为 **Z-up** 后，刚体局部轴仍须与 URDF link 一致。若报告出现 **dy≈±500 mm、dx/dz 正常**，说明 `torso` 刚体 Y 轴与 `waist_yaw_link` 反向，不是 `link_offset_mm` 的 xyz 顺序写错。

**根治（推荐）**：在 Motive 中编辑 `torso` 刚体，使局部轴与 `waist_yaw_link` 一致（X 前 / Y 左 / Z 上，按机器人 URDF），然后 `mocap_frame_align.enabled: false`。

**临时软件修正**：`hand_accuracy.yaml` 中：

```yaml
mocap_frame_align:
  enabled: true
  axis_flip: [1, -1, 1]   # Y 轴取反
```

也可用 `rpy_rad: [roll, pitch, yaw]`（弧度）代替 `axis_flip`。

**验证 offset 是否正确**：将三个 `link_offset_mm` 暂设为 `[0,0,0]` 再跑；若 Y 仍镜像 → 坐标系问题；若多轴出现 ~50–120 mm 常值偏差 → 重测 offset。

## 报告示例

终端摘要：

```
点       左范数(mm)    右范数(mm)     状态
T1           2.341        3.102       OK
...
左臂: mean=2.500 max=4.100 std=0.800 mm
右臂: mean=2.800 max=4.500 std=0.900 mm
```

JSON 含完整元数据、每点动捕/TF/分轴误差；CSV 便于 Excel 分析。

## 绘制精度柱形图

根据测试报告绘制左右手相对腰部的绝对位置精度（\(\|p_\mathrm{mocap}-p_\mathrm{TF}\|\)）：

```bash
python3 src/Camera_Calibration/hand_accuracy_test/plot_hand_accuracy_report.py \
  --json src/Camera_Calibration/hand_accuracy_test/hand_accuracy_report_YYYYMMDD_HHMMSS.json
```

省略 `--json` 时自动使用本目录最新 `hand_accuracy_report_*.json`。  
输出默认：同目录 `hand_accuracy_report_*_accuracy_bars.png`（上路点柱形图 + 中平均误差 + 下分轴均值）。

## 与现有模块关系

| 模块 | 关系 |
|------|------|
| `mocap_checkerboard_pose/mocap_pose_utils.py` | import 偏移修正、相对位姿、3σ 滤波 |
| `demos/kuavo_both_arms/both_arms_table_publisher.py` | 复用双臂下发与控制模式逻辑 |
| `run_chessboard_calibration.sh` | 不修改；本测试为独立流程 |

## 常见问题

- **动捕帧过少**：检查 Motive 刚体是否全绿、`rostopic hz` 是否有数据。
- **TF 查询失败**：确认 `robot_state_publisher` / 轮臂 TF 树已发布 `waist_yaw_link` 与末端 link。
- **手臂不跟轨迹**：确认外控模式、`/enable_lb_arm_quick_mode`、`/enable_arm_traj_interpolator` 已使能。
- **Y 向误差 ~500 mm、X/Z 正常**：启用 `mocap_frame_align.axis_flip: [1,-1,1]`，或在 Motive 重对齐 `torso` 刚体轴向。
- **link_offset 怎么填**：在 **URDF link 坐标系**下测量工装偏移，顺序始终 [x,y,z]，与 Motive 界面 Y-up/Z-up 无关。
