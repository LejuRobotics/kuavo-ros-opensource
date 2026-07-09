## kuavo_head_demo

### 入口

本 demo **只有一个 launch**：

- `src/Camera_Calibration/demos/kuavo_head_demo/kuavo_head_demo.launch`

它是整个链路里的 “头部相机” 部分：负责采集头部相机的棋盘角点观测与关节角，并输出优化后的 `calibration.yaml`（头部相关 bias）。

### 常用用法

launch 会自动解析 `camera_calib_root` 与 `csv_dir`（绝对路径）；**机型**按环境变量 `ROBOT_VERSION` 自动选择 URDF（52→biped52，62/63→wheel62，与 `run_chessboard_calibration.sh` 一致）。仅在需要时可显式覆盖：`robot_layout:=biped52` 或 `robot_layout:=wheel62`。

- **采集（capture_to_csv，手动触发）**：

```bash
roslaunch src/Camera_Calibration/demos/kuavo_head_demo/kuavo_head_demo.launch \
  do_capture_to_csv:=true do_optimize_from_csv:=false do_calibrate_manual:=false
```

- **采集（推荐：一键脚本自动下发 + 自动触发采样）**：

```bash
bash src/Camera_Calibration/run_chessboard_calibration.sh capture
```

- **优化（optimize_from_csv）**：

```bash
roslaunch src/Camera_Calibration/demos/kuavo_head_demo/kuavo_head_demo.launch \
  do_capture_to_csv:=false do_optimize_from_csv:=true do_calibrate_manual:=false
```

> 优化产物默认写到：`src/Camera_Calibration/output/kuavo_head/`（包括 `calibration.yaml`、误差图、`optimization_metrics.md` 等）。


### 关键约定

- 公共 URDF：`biped_v3_arm.urdf`（biped52）或 `biped_v3_arm_s62.urdf`（wheel62）
- 输出：
  - CSV：`src/Camera_Calibration/output_csv/kuavo_head/`
  - 图/报告：`src/Camera_Calibration/output/kuavo_head/`

### 从优化结果写入零点

头部 demo 的 `calibration.yaml` 会被顶层脚本统一合并并写入零点文件：

```bash
python3 src/Camera_Calibration/apply_zero_deltas_to_arms_zero.py --dry-run
python3 src/Camera_Calibration/apply_zero_deltas_to_arms_zero.py
```

整体流程与目录说明请看：`src/Camera_Calibration/README.md`。

