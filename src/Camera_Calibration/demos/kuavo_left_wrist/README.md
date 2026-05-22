## kuavo_left_wrist

### 入口

- `kuavo_camera_calibration/launch/kuavo_left_wrist_demo.launch`

左腕相机棋盘格采集与关节零点优化。

### 前置

```bash
catkin build kuavo_camera_calibration robot_calibration robot_calibration_msgs
source devel/setup.bash
```

### 常用用法

- **采集**：

```bash
roslaunch kuavo_camera_calibration kuavo_left_wrist_demo.launch \
  do_capture_to_csv:=true do_optimize_from_csv:=false
```

- **推荐一键脚本**：

```bash
bash src/Camera_Calibration/run_chessboard_calibration.sh capture
```

- **优化**：

```bash
roslaunch kuavo_camera_calibration kuavo_left_wrist_demo.launch \
  do_capture_to_csv:=false do_optimize_from_csv:=true
```

### 关键约定

- URDF：包内 `biped_v3_arm.urdf`
- 轨迹下发：`demos/kuavo_left_wrist/left_wrist_table_publisher.py`
- 话题：`/left_wrist_camera/color/image_raw`，`frame_id=left_wrist_camera_color_optical_frame`

### 写入零点

见 `src/Camera_Calibration/README.md`。
