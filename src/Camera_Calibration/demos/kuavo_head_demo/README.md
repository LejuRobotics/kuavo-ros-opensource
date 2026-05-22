## kuavo_head_demo

### 入口

本 demo 的 launch 文件（ROS 标准路径）：

- `kuavo_camera_calibration/launch/kuavo_head_demo.launch`

负责头部相机棋盘格采集与 `calibration.yaml` 优化输出。

### 前置

```bash
catkin build kuavo_camera_calibration robot_calibration robot_calibration_msgs
source devel/setup.bash
```

### 常用用法

- **采集（capture_to_csv）**：

```bash
roslaunch kuavo_camera_calibration kuavo_head_demo.launch \
  do_capture_to_csv:=true do_optimize_from_csv:=false do_calibrate_manual:=false
```

- **优化（optimize_from_csv）**：

```bash
roslaunch kuavo_camera_calibration kuavo_head_demo.launch \
  do_capture_to_csv:=false do_optimize_from_csv:=true do_calibrate_manual:=false
```

- **推荐：统一入口脚本**（头 + 左右腕）：

```bash
bash src/Camera_Calibration/run_chessboard_calibration.sh capture
bash src/Camera_Calibration/run_chessboard_calibration.sh optimize
```

配置文件位于 `kuavo_camera_calibration/demos/kuavo_head_demo/*.yaml`；URDF 与输出目录由 launch 内 `$(find kuavo_camera_calibration)` 解析。

### 关键约定

- 公共 URDF：包内 `biped_v3_arm.urdf`
- CSV：`output_csv/kuavo_head/`（相对包根，运行时可由 `csv_dir` 覆盖）
- 报告：`output/kuavo_head/`

### 从优化结果写入零点

```bash
python3 src/Camera_Calibration/apply_zero_deltas_to_arms_zero.py --dry-run
python3 src/Camera_Calibration/apply_zero_deltas_to_arms_zero.py
```

整体说明：`src/Camera_Calibration/README.md`。
