## kuavo_right_wrist

### 入口

本 demo **只有一个 launch**：

- `src/Camera_Calibration/demos/kuavo_right_wrist/kuavo_right_wrist_demo.launch`

它是整个链路里的 “右腕相机” 部分：负责把 **右腕相机的棋盘角点观测** 与 **机器人关节角** 采集成 CSV，并调用优化器输出 `calibration.yaml`。

### 常用用法

- **采集（capture_to_csv，手动触发）**：

```bash
roslaunch /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/demos/kuavo_right_wrist/kuavo_right_wrist_demo.launch \
  do_capture_to_csv:=true do_optimize_from_csv:=false \
  csv_dir:=/home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output_csv/kuavo_right_wrist
```

- **采集（推荐：一键脚本自动下发 + 自动触发采样）**：

```bash
bash /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/run_chessboard_calibration.sh capture
```

- **优化（optimize_from_csv）**：

```bash
roslaunch /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/demos/kuavo_right_wrist/kuavo_right_wrist_demo.launch \
  do_capture_to_csv:=false do_optimize_from_csv:=true \
  csv_dir:=/home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output_csv/kuavo_right_wrist
```

> 优化产物默认写到：`src/Camera_Calibration/output/kuavo_right_wrist/`（包括 `calibration.yaml`、误差图、`optimization_metrics.md` 等）。

### 关键约定

- 公共 URDF：`src/Camera_Calibration/biped_v3_arm.urdf`（已包含 `right_wrist_camera_base` + `right_wrist_camera_color_optical_frame`）
- 右手自动下发脚本：`src/Camera_Calibration/demos/kuavo_right_wrist/right_wrist_table_publisher.py`
  - 会读取 `src/Camera_Calibration/teach_capture_output/teach_right_joint.json`
  - 会在发布 `/kuavo_arm_traj` 前尝试切到 **external_control(2)** 并调用 **/enable_wbc_arm_trajectory_control**
- 当前在线话题对齐（2D 棋盘角点）：
  - `/right_wrist_camera/color/image_raw`
  - `/right_wrist_camera/color/camera_info`
  - `frame_id="right_wrist_camera_color_optical_frame"`

### 从优化结果写入零点

右腕 demo 的 `calibration.yaml` 会被顶层脚本统一合并并写入零点文件：

```bash
python3 src/Camera_Calibration/apply_zero_deltas_to_arms_zero.py --dry-run
python3 src/Camera_Calibration/apply_zero_deltas_to_arms_zero.py
```

整体流程与目录说明请看：`src/Camera_Calibration/README.md`。

