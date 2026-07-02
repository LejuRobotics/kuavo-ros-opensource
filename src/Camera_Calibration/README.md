## Camera_Calibration（Kuavo 相机/关节零点联合标定）

本目录提供一套面向 Kuavo 实机的 **棋盘格相机标定 + 关节零点修正** 工具链，围绕 3 个 demo：

- 头部相机（`kuavo_head_demo`）
- 左腕相机（`kuavo_left_wrist`）
- 右腕相机（`kuavo_right_wrist`）

典型目标是：采集多姿态下的棋盘格观测 → 优化得到每个关节的 `bias`（以及必要时的相机/外参/URDF 修正）→ 将 `bias` 写回零点文件，使得下一轮标定时误差显著变小。

### 机型与 URDF（52 / 62）

当前支持两种实机布局，**命令不变**，由环境变量 `ROBOT_VERSION` 自动分流（`run_chessboard_calibration.sh`、`auto_camera_calib_and_apply_zero.py` 及各 demo launch 行为一致）：

| `ROBOT_VERSION` | 布局 | 标定用 URDF | 标定后 URDF | 零点回写脚本 |
|-----------------|------|-------------|-------------|--------------|
| `52`（默认） | 人形 biped52 | `biped_v3_arm.urdf` | `biped_v3_arm_calibrated.urdf` | `apply_zero_deltas_to_arms_zero.py`（14 维 Ruiwo + EC） |
| `62` / `63` | 轮臂 wheel62 | `biped_v3_arm_s62.urdf` | `biped_v3_arm_s62_calibrated.urdf` | `apply_zero_deltas_to_arms_zero_wheel62.py`（16 维 Motorevo） |

未设置 `ROBOT_VERSION` 时按人形（52）处理。需要时可显式覆盖：`--robot_layout biped52` 或 `--robot_layout wheel62`。

---

## 目录结构（你真正需要看的部分）

- **`demos/`**
  - 每个 demo 一个 `*.launch`，负责接入 `robot_calibration` 的 capture/optimize 流程，并对齐 Kuavo 的话题/坐标系约定。
  - 入口 README：
    - `demos/kuavo_head_demo/README.md`
    - `demos/kuavo_left_wrist/README.md`
    - `demos/kuavo_right_wrist/README.md`

- **`run_chessboard_calibration.sh`**
  - 三个 demo 的统一入口脚本，支持一次性跑完采集/优化（以及项目里约定的并行优化模式）。
  - 你日常“跑一遍标定”的第一选择。

- **`auto_camera_calib_and_apply_zero.py`**
  - 一键流程（采集 → 优化 → 打印修正量 → 交互确认写入零点）。
  - 适合“限位标零后，插入一次相机标定”那种工作流。

- **`apply_zero_deltas_to_arms_zero.py`** / **`apply_zero_deltas_to_arms_zero_wheel62.py`**
  - 按机型把 `output/**/calibration.yaml` 的 joint bias 写入 `~/.config/lejuconfig/arms_zero.yaml`：
    - 52：`apply_zero_deltas_to_arms_zero.py`（14 维 Ruiwo + EC `offset.csv`）
    - 62/63：`apply_zero_deltas_to_arms_zero_wheel62.py`（16 维 Motorevo，不写 EC）
  - 一键脚本 `auto_camera_calib_and_apply_zero.py` 会按 `ROBOT_VERSION` 自动选用对应脚本。

- **`plot_board_error_from_csv.py`**
  - 离线验证脚本：基于 `output_csv/**` 的采集 CSV，把 **标定前 URDF** 与 **优化后 URDF/YAML** 的棋盘位姿误差画出来。

- **`chessboard_pose_logger/`**
  - 单独的棋盘位姿记录工具（可用于标定前/后对比、排错）。

- **`output_csv/`（运行时生成）**
  - capture 阶段产生的 `joints.csv`、`features.csv`、以及优化用到的样本列表等。

- **`output/`（运行时生成）**
  - optimize 阶段的结果与报告：`calibration.yaml`、`calibrated.urdf`、误差图、`optimization_metrics.md` 等。

- **`robot_calibration-ros1/`**
  - 上游 `robot_calibration` 源码（ROS1）。通常你不需要改这里；主要用于本目录 demo 的依赖。

---

## 快速开始（推荐）

### 前置条件

- **`ROBOT_VERSION` 与实机一致**：人形设 `52`，轮臂设 `62` 或 `63`（见上文机型表）。
- **机器人控制已启动**：确保 Kuavo 实机相关 launch 已运行，能够下发手臂轨迹并且相机话题在发布。
- **棋盘格可见**：相机画面能稳定看到棋盘格，且光照/反光不过曝。
- **三路 demo 的话题/`frame_id` 对齐**：各 demo README 里有对应话题约定。

### 一键采集（3 个 demo）

在仓库根目录执行：

```bash
bash src/Camera_Calibration/run_chessboard_calibration.sh capture
```

采集完成后会在 `src/Camera_Calibration/output_csv/` 下看到对应 demo 的 CSV 目录。

### 一键优化（3 个 demo）

```bash
bash src/Camera_Calibration/run_chessboard_calibration.sh optimize
```

优化完成后会在 `src/Camera_Calibration/output/<demo>/` 下看到：

- `calibration.yaml`：关节 `bias`（单位 rad）
- `calibrated*.urdf`：优化后的 URDF（若该 demo 配置优化了 free_frames/free_params）
- `optimization_metrics.md`：优化指标与摘要（保留）
- `board_pose_error_pre_post_vs_urdf.png`：优化前后误差对比图（用于快速判断效果）

### 一键标定 + 写零点（推荐）

采集、优化、打印修正量、确认后写零点，**52/62 共用同一条命令**：

```bash
python3 src/Camera_Calibration/auto_camera_calib_and_apply_zero.py
```

### 仅将优化结果写入零点

也可单独执行（脚本随 `ROBOT_VERSION` 自动选择；人形示例）：

```bash
python3 src/Camera_Calibration/apply_zero_deltas_to_arms_zero.py --dry-run   # 先核对
python3 src/Camera_Calibration/apply_zero_deltas_to_arms_zero.py             # 确认后写入
```

轮臂（62/63）对应 `apply_zero_deltas_to_arms_zero_wheel62.py`，参数相同。

脚本会自动备份 `~/.config/lejuconfig/arms_zero.yaml`；人形若有 EC 改动还会备份 `offset.csv`。

---

## 关键约定（必须读）

### `calibration.yaml` 的含义

本项目约定 `output/**/calibration.yaml` 里每个 `*_joint` 的值是 **bias（rad）**，并且在误差计算/FK 中按下面方式使用：

- `q_used = q_reported + bias`

（这与 `plot_board_error_from_csv.py` 的实现一致。）

### 零点写入规则（脚本已固化）

- **人形 52（Ruiwo + EC）**
  - `arms_zero.yaml`：`q_reported = signed_raw_pos - zero_offset` → 写入 `zero_offset += (-bias)`
  - `offset.csv`（deg）：`q_deg = raw_deg - offset_deg` → 仅更新 `zarm_l1_joint` / `zarm_r1_joint`

- **轮臂 62/63（Motorevo）**
  - `arms_zero.yaml` 16 维槽位：`[zarm_l1..l7, zarm_r1..r7, zhead_1, zhead_2]`，同样 `new_zero = old_zero - bias`
  - 不写 `offset.csv`

---

## 验证与排错

### 如何确认“写入零点有效”

- 重新跑一轮 optimize：
  - 若写入正确，新的 `calibration.yaml` 里对应关节 bias 应显著变小（趋近 0），误差图也会明显下降。

### 常见问题

- **手臂 bias 全是 0**
  - 先检查 `output/kuavo_left_wrist/calibration.yaml`、`output/kuavo_right_wrist/calibration.yaml` 是否确实全为 0；
  - 若是，说明优化阶段没有“看到”足够的有效观测/激励（采集姿态太少、棋盘检测失败、话题对不上、样本被剔除等）。

- **后台 roslaunch 没关干净**
  - 现象：新的 launch 起不来、端口被占、topic 混在一起。
  - 处理：先确认 `rosmaster/roslaunch` 是否仍在运行，再手动 kill 对应 PID。

---

## 进一步阅读（按优先级）

- `run_chessboard_calibration.sh`：你日常最常用的一键入口
- `auto_camera_calib_and_apply_zero.py`：一键“采集+优化+写零点”的闭环入口
- `apply_zero_deltas_to_arms_zero.py` / `apply_zero_deltas_to_arms_zero_wheel62.py`：零点写入与映射打印（排查方向问题就看它）
- `plot_board_error_from_csv.py`：离线验证“优化前/后误差”

