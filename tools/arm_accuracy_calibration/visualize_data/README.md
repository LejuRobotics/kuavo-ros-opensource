# URDF 标定可视化工具说明

## 目录结构

```
arm_accuracy_calibration/          # 工具根目录
├── visualize_data/                # 本工具目录
│   ├── one_key_run.sh             # 一键运行脚本（推荐入口）
│   ├── update_urdf.py             # 将标定结果写入 URDF
│   ├── plt_fk_optimized_result.py # 调用 FK 服务对比标定前后结果
│   ├── print_T_from_csv.py        # 查看单个变换矩阵 CSV
│   └── config/
│       └── biped_v3_arm_custom.urdf  # 标定写入的目标 URDF（自定义版本）
└── calibration_output/            # 与 visualize_data/ 同级，FK 对比数据默认读取位置
    └── raw_data_motion/
        └── joint_0/
            ├── raw_sensor.csv     # 左臂关节角（弧度）
            └── raw_poses.csv      # 动捕刚体位姿
```

## 快速开始（推荐）

直接运行一键脚本：

```bash
bash tools/arm_accuracy_calibration/visualize_data/one_key_run.sh
```

脚本依次执行：
1. 将标定结果写入 `config/biped_v3_arm_custom.urdf`
2. 后台启动 `humanoid_controllers` 主节点（提供 FK 服务）
3. 等待 `/ik/fk_srv_with_refer_frame` 服务就绪（超时 10s 自动退出）
4. 运行 `plt_fk_optimized_result.py` 输出对比图

## 分步使用

### 1. 仅更新 URDF

```bash
python3 tools/arm_accuracy_calibration/visualize_data/update_urdf.py \
  --data-dir /path/to/fitted_results \
  --update-urdf tools/arm_accuracy_calibration/visualize_data/config/biped_v3_arm_custom.urdf
```

加 `--auto-update` 表示仅验证通过后才更新：

```bash
python3 tools/arm_accuracy_calibration/visualize_data/update_urdf.py \
  --data-dir /path/to/fitted_results \
  --update-urdf tools/arm_accuracy_calibration/visualize_data/config/biped_v3_arm_custom.urdf \
  --auto-update
```

不传 `--update-urdf` 时，`--auto-update` 默认写入 `config/biped_v3_arm_custom.urdf`。

### 2. 仅校验数据目录

```bash
python3 tools/arm_accuracy_calibration/visualize_data/update_urdf.py \
  --data-dir /path/to/fitted_results \
  --validate
```

### 3. 仅预览参数（不改文件）

```bash
python3 tools/arm_accuracy_calibration/visualize_data/update_urdf.py \
  --data-dir /path/to/fitted_results
```

### 4. FK 对比可视化

```bash
python3 tools/arm_accuracy_calibration/visualize_data/plt_fk_optimized_result.py \
  [--base-frame zarm_l1_ref_link] \
  [--end-frame zarm_l6_link] \
  [--hand-side 0] \
  [--sensor-csv /path/to/raw_sensor.csv] \
  [--poses-csv  /path/to/raw_poses.csv] \
  [--urdf-path  config/biped_v3_arm_custom.urdf] \
  [--save-png   result.png]
```

**CSV 数据默认读取路径**（不传参数时自动使用）：

```
calibration_output/raw_data_motion/joint_0/raw_sensor.csv
calibration_output/raw_data_motion/joint_0/raw_poses.csv
```

> 如果该路径下没有文件，脚本会报错并提示正确的放置位置。
> 可通过 `--sensor-csv` / `--poses-csv` 手动指定其他路径。

FK 对比逻辑：
- **fk_before**：`raw_sensor.csv` 均值关节角 + 默认 URDF → 标定前正运动学结果
- **fk_after**：全零关节角 + `config/biped_v3_arm_custom.urdf` → 标定后正运动学结果
- **mocap**：硬编码的动捕参考位置（肩部坐标系，单位 m）

### 5. 查看单个矩阵

```bash
python3 tools/arm_accuracy_calibration/visualize_data/print_T_from_csv.py \
  /path/to/T_joint2_in_joint1_fitted.csv
```

## 数据文件要求

### fitted_results 目录结构

```
fitted_results/
├── joint_0/（或 joint_-1_test*）
│   ├── T_joint2_in_joint1_fitted.csv   # 必需
│   ├── T_joint5_in_joint4_fitted.csv   # 必需
│   ├── T_joint6_in_joint5_fitted.csv   # 必需
│   ├── T_joint3_in_joint2_fitted.csv   # 新格式（二选一）
│   ├── T_joint4_in_joint3_fitted.csv   # 新格式（二选一）
│   ├── T_joint3_in_joint1_fitted.csv   # 旧格式（二选一）
│   └── T_joint4_in_joint1_fitted.csv   # 旧格式（二选一）
├── joint_1/circle_fit_result.csv
├── joint_2/circle_fit_result.csv
├── joint_3/circle_fit_result.csv
├── joint_4/circle_fit_result.csv
└── joint_5/circle_fit_result.csv
```

### FK 对比 CSV 格式

路径：`calibration_output/raw_data_motion/joint_0/`

`raw_sensor.csv`：

| timestamp | left_arm_1 | left_arm_2 | ... | left_arm_7 |
|-----------|-----------|-----------|-----|-----------|
| ...       | rad       | rad       | ... | rad       |

`raw_poses.csv`：

| timestamp | joint_1_px | joint_1_py | joint_1_pz | joint_1_qx | ... | belly_qw |
|-----------|-----------|-----------|-----------|-----------|-----|---------|
| ...       | mm        | mm        | mm        | -         | ... | -       |

## 注意事项

- 平移单位：`joint_0` CSV 为 mm，写 URDF 时自动转换为 m
- 更新 URDF 前建议先备份 `config/biped_v3_arm_custom.urdf`
- 运行 `plt_fk_optimized_result.py` 前需确保 ROS FK 服务已启动
- FK 对比数据默认从 `calibration_output/raw_data_motion/joint_0/` 读取，文件不存在时脚本会报错提示正确路径

## 依赖

- Python 3
- `numpy`、`scipy`
- ROS（运行 FK 对比时需要）
- `kuavo_msgs`（FK 服务消息类型）
