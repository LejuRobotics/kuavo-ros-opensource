# s56 标定适配 — 修改总结

> 分支: `xiezhicong/dev/add-s56-calib`
> 基础分支: `dev`
> 日期: 2026-07-10

## 问题

`ROBOT_VERSION=56` 在标定脚本中未被识别，回退到 `biped52` → 加载 `biped_v3_arm.urdf`（s52 参数）。s56 和 s52 的关节位置、轴方向、相机安装位姿均有差异，导致 FK 计算错误 → Ceres 优化失败 → calibration.yaml 全 0。

## 修改清单

### 1. 新建 `src/Camera_Calibration/biped_v3_arm_s56.urdf` (771行)

基于 s56 原 URDF (`kuavo_assets/models/biped_s56/urdf/biped_s56.urdf`) 的关节参数，融合 v3_arm.urdf 的标定辅助结构。

**保留 s56 原参数**（与 s52/v3_arm 不同的部分）：

| 参数 | s52 (v3_arm) | s56 (v3_arm_s56) |
|------|-------------|-----------------|
| 肩宽 zarm_l1 origin Y | 0.1737 | **0.2527** |
| zhead_1 origin Z | 0.3715 | **0.49535** |
| zhead_2 origin Z | 0.138 | **0** |
| camera_base origin | (0.120, 0.024, 0.026) | **(0.094, 0.048, 0.050)** |
| camera_base rpy pitch | 0.48 | **0.34** |
| zarm_l4 origin Z | -0.1797 | **-0.2837** |
| zarm_l5 origin Z | -0.1195 | **-0.1201** |
| zarm_r4 origin Z | -0.1797 | **-0.2837** |
| zarm_r5 origin Z | -0.1195 | **-0.1201** |
| 各关节 limit / effort / velocity | s52 值 | **s56 值** |
| 各连杆 inertial (质心/质量/惯量) | s52 值 | **s56 值** |

**与 v3_arm.urdf 保持一致的方向约定**：

| 关节 | axis | 原因 |
|------|------|------|
| `zhead_2_joint` | **0 -1 0** | 与 v3_arm 对齐传感器方向约定（s56 原始为 0 1 0） |

**新增标定结构**（s56 原始 URDF 中不存在）：

- `zarm_l1_ref_link` + `zarm_l1_ref_joint` — FK 根坐标系（位于肩部位置 -0.003, 0.2527, 0.283）
- `zarm_l1_joint` 改为挂在 `zarm_l1_ref_link` 下（origin 改为 0 0 0），左臂其余关节不变
- `head_camera_color_optical_frame_0` + `head_camera_color_optical_frame` — 头部相机光学坐标系链
- `l_hand_tripod` + `l_hand_camera` + `left_wrist_camera_color_optical_frame` — 左手腕相机
- `r_hand_tripod` + `r_hand_camera` + `right_wrist_camera_color_optical_frame` — 右手腕相机
- `checkerboard_link` + `checkerboard_joint` — 棋盘格名义位置（**需实测后更新**）

### 2. 修改 `run_chessboard_calibration.sh`

- 第 16 行: usage 文本添加 `biped56`
- 第 22 行: `--robot_layout` 参数新增 `biped56`
- 第 69 行: resolve_robot_layout 的 case 新增 `biped56`
- 第 75 行: ROBOT_VERSION=56 → `biped56`
- 第 88-95 行: 新增 biped56 分支，加载 `biped_v3_arm_s56.urdf` / `biped_v3_arm_s56_calibrated.urdf`

### 3. 修改 `auto_camera_calib_and_apply_zero.py`

- 第 34 行: `_resolve_robot_layout` 新增 `biped56`
- 第 39 行: ROBOT_VERSION=56 → `biped56`
- 第 86-91 行: argparse choices 新增 `biped56`

### 4. 修改 3 个 launch 文件

- `demos/kuavo_head_demo/kuavo_head_demo.launch` 第 7-10 行
- `demos/kuavo_right_wrist/kuavo_right_wrist_demo.launch` 第 7-10 行
- `demos/kuavo_left_wrist/kuavo_left_wrist_demo.launch` 第 7-10 行

均新增 `biped56` 识别逻辑，robot_layout=biped56 时加载 `biped_v3_arm_s56.urdf`。

## 实机上还需完成的事项

1. **棋盘格位置实测**: `checkerboard_joint` 的 origin 目前沿用 s52 值，需要在 s56 实机上用尺子量出棋盘格实际位置后更新
2. **`zhead_2_joint` 方向实机验证**: 在实机上做单关节转动测试，确认 `axis="0 -1 0"` 与传感器正方向一致
3. **`ROBOT_VERSION=56` 运行测试**: `python3 src/Camera_Calibration/auto_camera_calib_and_apply_zero.py`，确认 calibration.yaml 不再全 0
