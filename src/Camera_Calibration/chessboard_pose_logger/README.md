## chessboard_pose_logger

### 作用

订阅相机图像与 `camera_info`，用 OpenCV 识别棋盘（`findChessboardCorners + solvePnP`），把 **棋盘相对相机** 的位姿 \(T\_{cam}^{board}\) 持续追加写入一个 CSV。

它不参与 `robot_calibration` 的优化，只用于：

- 标定前/后快速对比（同一相机、同一棋盘、同一位置，位姿是否更稳定/更接近）
- 排查“是不是根本没识别到棋盘 / `camera_info` 不对 / frame_id 不对”
- 生成独立的位姿日志，供你用自定义脚本做统计分析

### 启动

标定前（使用在线 `camera_info`）：

```bash
bash src/Camera_Calibration/chessboard_pose_logger/start_capture.sh --camera left_wrist --mode pre
```

标定后（使用你校准后的内参 YAML 覆盖）：

```bash
bash src/Camera_Calibration/chessboard_pose_logger/start_capture.sh --camera left_wrist --mode post --intrinsics_yaml /abs/path/to/intrinsics.yaml
```

> 提示：`--camera` 只是选择预设话题组合（见 `start_capture.sh`），并不“自动发现相机”。

### 输出

默认输出目录：

- `src/Camera_Calibration/output_csv/chessboard_pose/<camera>/`

文件名：

- `board_pose_<pre|post>_<timestamp>.csv`

CSV 中的 `t_x,t_y,t_z` 与 `q_x,q_y,q_z,q_w` 表示 **棋盘坐标系在相机坐标系下** 的位姿（与 `solvePnP` 输出一致）。

### 常见问题

- **CSV 一直不增长 / 没有文件**
  - 检查图像话题是否有数据、棋盘是否在画面内
  - 检查 `camera_info` 是否在发布（且内参合理）

- **位姿跳变很大**
  - 典型原因是角点检测不稳定（反光、运动模糊、棋盘太小/太远）
  - 或者 `camera_info` 与当前图像不匹配（分辨率/裁剪不一致）

---

更多整体流程请看顶层文档：`src/Camera_Calibration/README.md`。

