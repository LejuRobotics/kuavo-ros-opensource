## Camera_Calibration（Kuavo 相机/关节零点联合标定）

本目录提供一套面向 Kuavo 实机的 **棋盘格相机标定 + 关节零点修正** 工具链，围绕 3 个 demo：

- 头部相机（`kuavo_head_demo`）
- 左腕相机（`kuavo_left_wrist`）
- 右腕相机（`kuavo_right_wrist`）

典型目标是：采集多姿态下的棋盘格观测 → 优化得到每个关节的 `bias`（以及必要时的相机/外参/URDF 修正）→ 将 `bias` 写回零点文件，使得下一轮标定时误差显著变小。

---

## 目录结构（你真正需要看的部分）

- **ROS 包 `kuavo_camera_calibration`**（本目录根下的 `package.xml` + `CMakeLists.txt`）
  - **`launch/`**：`kuavo_head_demo.launch`、`kuavo_right_wrist_demo.launch`、`kuavo_left_wrist_demo.launch`
  - 启动示例：`roslaunch kuavo_camera_calibration kuavo_head_demo.launch`
  - URDF / yaml / 默认输出路径在 launch 内通过 `$(find kuavo_camera_calibration)` 解析

- **`demos/`**
  - 各 demo 的 yaml 配置与轨迹下发脚本；说明见各子目录 README。

- **`run_chessboard_calibration.sh`**
  - 三个 demo 的统一入口脚本，支持一次性跑完采集/优化（以及项目里约定的并行优化模式）。
  - 你日常“跑一遍标定”的第一选择。

- **`auto_camera_calib_and_apply_zero.py`**
  - 一键流程（采集 → 优化 → 打印修正量 → 交互确认写入零点）。
  - 适合“限位标零后，插入一次相机标定”那种工作流。

- **`apply_zero_deltas_to_arms_zero.py`**
  - 把 `output/**/calibration.yaml` 里的 joint bias 合并后，写入：
    - Ruiwo：`~/.config/lejuconfig/arms_zero.yaml`（弧度，`arms_zero_position` 14 维）
    - EC：`~/.config/lejuconfig/offset.csv`（角度，固定更新 `zarm_l1_joint`/`zarm_r1_joint`）

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

- **编译 ROS 包**（launch/URDF 路径通过 `$(find kuavo_camera_calibration)` 解析，勿写死本机用户目录）：

```bash
catkin build kuavo_camera_calibration robot_calibration robot_calibration_msgs
source devel/setup.bash
```

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

### 一键采集 + 优化 + 写零点（QA 手眼/相机标定闭环）

```bash
python3 src/Camera_Calibration/auto_camera_calib_and_apply_zero.py
```

内部调用 `run_chessboard_calibration.sh`，launch 使用 `roslaunch kuavo_camera_calibration <demo>.launch`。

优化完成后会在 `src/Camera_Calibration/output/<demo>/` 下看到：

- `calibration.yaml`：关节 `bias`（单位 rad）
- `calibrated*.urdf`：优化后的 URDF（若该 demo 配置优化了 free_frames/free_params）
- `optimization_metrics.md`：优化指标与摘要（保留）
- `board_pose_error_pre_post_vs_urdf.png`：优化前后误差对比图（用于快速判断效果）

### 将优化结果写入零点（让下次误差变小）

```bash
python3 src/Camera_Calibration/apply_zero_deltas_to_arms_zero.py
```

建议第一次先 dry-run 核对映射/方向：

```bash
python3 src/Camera_Calibration/apply_zero_deltas_to_arms_zero.py --dry-run
```

脚本会自动备份：

- `~/.config/lejuconfig/arms_zero.yaml.bak.<timestamp>`
- `~/.config/lejuconfig/offset.csv.bak.<timestamp>`（如果有 EC 改动）

---

## 关键约定（必须读）

### `calibration.yaml` 的含义

本项目约定 `output/**/calibration.yaml` 里每个 `*_joint` 的值是 **bias（rad）**，并且在误差计算/FK 中按下面方式使用：

- `q_used = q_reported + bias`

（这与 `plot_board_error_from_csv.py` 的实现一致。）

### 零点写入规则（脚本已固化）

- **Ruiwo（`arms_zero.yaml`）**
  - 驱动发布角度（见 `ruiwo_actuator.cpp::set_joint_state`）：`q_reported = signed_raw_pos - zero_offset`
  - 因此要让上报角增加 `bias`：统一写入 `zero_offset += (-bias)`（不再按 negtive 分支）
  - `negtive_address` 仅影响 raw 取符号，不影响“减零点”的方向

- **EC（`offset.csv`，单位 deg）**
  - 驱动发布角度：`q_deg = raw_deg - offset_deg`
  - 因此要让上报角增加 `bias_deg`：`offset_deg += (-bias_deg)`

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
- `apply_zero_deltas_to_arms_zero.py`：零点写入规则与映射打印（排查方向问题就看它）
- `plot_board_error_from_csv.py`：离线验证“优化前/后误差”

