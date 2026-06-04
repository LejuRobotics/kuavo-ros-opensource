## kuavo_head_demo

### 入口

本 demo **只有一个 launch**：

- `src/Camera_Calibration/demos/kuavo_head_demo/kuavo_head_demo.launch`

它是整个链路里的 “头部相机” 部分：负责采集头部相机的棋盘角点观测与关节角，并输出优化后的 `calibration.yaml`（头部相关 bias）。

### 常用用法

在仓库根目录执行：

- **采集（capture_to_csv，手动触发）**：

```bash
roslaunch src/Camera_Calibration/demos/kuavo_head_demo/kuavo_head_demo.launch \
  do_capture_to_csv:=true do_optimize_from_csv:=false do_calibrate_manual:=false \
  csv_dir:=src/Camera_Calibration/output_csv/kuavo_head
```

- **优化（optimize_from_csv）**：

```bash
roslaunch src/Camera_Calibration/demos/kuavo_head_demo/kuavo_head_demo.launch \
  do_capture_to_csv:=false do_optimize_from_csv:=true do_calibrate_manual:=false \
  csv_dir:=src/Camera_Calibration/output_csv/kuavo_head
```

### 关键约定

- 公共 URDF：`src/Camera_Calibration/biped_v3_arm.urdf`
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

