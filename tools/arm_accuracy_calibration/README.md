# Arm Accuracy Calibration 手臂精度标定

刚体位姿标定模块：从动捕刚体位姿解算各 joint 相对变换（T 矩阵），经拟合得到标定结果。流程分为**模块一（数据采集）→ 模块二（离线解算 T）→ 模块三（拟合并保存）→ 模块四（DH 参数）**。

## 快速开始（推荐）

目标：在 `tools/arm_accuracy_calibration/calibration_output/` 下生成拟合后的 T 以及 DH 参数。

```bash
cd tools/arm_accuracy_calibration

# 模块一（需要 ROS + 动捕）：分别采集 joint_0..7 的 raw_poses.csv（建议每个关节都采一份）
./run_arm_calibration.sh --build     # 可选：先编译并启动 motioncapture，再运行模块一（默认 joint_2）
python3 run_data_collection.py --joint 0
python3 run_data_collection.py --joint 1
python3 run_data_collection.py --joint 2
python3 run_data_collection.py --joint 3
python3 run_data_collection.py --joint 4
python3 run_data_collection.py --joint 5
python3 run_data_collection.py --joint 6
python3 run_data_collection.py --joint 7

# 模块二（三、四无需 ROS）：离线解算 T
python3 run_batch_process.py

# 模块三：拟合并保存
python3 run_fit_save.py

# 模块四：用 joint_0 的拟合 T 解 DH 参数
python3 run_dh_from_fitted.py
```

## 功能特点

- 直接使用动捕刚体位姿（位置 + 四元数），无需手动标定 marker 点
- 支持多刚体阵列配置（config.yaml），灵活扩展
- 模块一：关节运动控制 + 动捕/传感器数据记录（双 CSV）
- 模块二：离线解算 T 矩阵（joint_0 不乘配置偏移，joint_1～7 乘配置偏移）
- 模块三：joint_0 做 T 最小二乘拟合，joint_1～7 做圆拟合，输出拟合 T、圆心、半径、轴线
- 模块四：读取模块三 joint_0 的拟合 T，转到 DH 系后反解 DH 参数，打印并保存

## 目录结构

```
arm_accuracy_calibration/
├── config/
│   └── config.yaml              # 阵列配置（刚体到 joint 的 4x4 变换，单位 mm）
├── functions/
│   ├── __init__.py
│   ├── config_loader.py         # 配置加载
│   ├── rigid_body_store.py     # 刚体位姿存储与回调
│   ├── frame_solver.py         # 单阵列解算（坐标变换）
│   ├── batch_solver.py         # 批量解算
│   ├── fit_matrix_ls.py        # T 矩阵最小二乘拟合
│   ├── fit_rotation_axis_ls.py # 圆/旋转轴拟合
│   ├── cmd_arm_joint.py        # 单关节往复运动（被模块一调用）
│   └── ...
├── calibration_output/         # 工作区：三模块的 CSV/结果统一放此目录下
│   ├── raw_data_motion/        # 模块一输出
│   │   └── joint_<N>/          # N=0..7，每目录下 raw_poses.csv、raw_sensor.csv 等
│   ├── matrix_data_motion/     # 模块二输出
│   │   └── joint_<k>/          # 解算得到的 T 矩阵 CSV
│   └── fitted_results/         # 模块三输出
│       └── joint_<k>/          # 拟合后的 T、圆拟合结果等
├── run_data_collection.py      # 模块一：数据采集
├── run_batch_process.py        # 模块二：离线批量解算
├── run_fit_save.py             # 模块三：拟合并保存
├── run_fit_save.sh             # 模块三入口脚本（可选）
├── run_dh_from_fitted.py       # 模块四：从 joint_0 拟合 T 解 DH 参数
├── run_arm_calibration.sh      # 一键启动：编译、动捕、模块一
└── README.md
```

## 依赖要求

- Python 3.x
- ROS（模块一需要；模块二、三无需）
- numpy、matplotlib、PyYAML、pandas、scipy（模块二、三）
- kuavo_msgs（项目自定义消息）

### Python 依赖安装（模块二/三/四）

在能联网/已配置 pip 源的环境中，可直接安装：

```bash
python3 -m pip install -U numpy pandas scipy pyyaml matplotlib
```

无图形界面（无 X11/Wayland）环境建议设置 matplotlib 后端，避免弹窗报错：

```bash
export MPLBACKEND=Agg
```

---

## 运行

### 模块一：数据采集（run_data_collection）

在 ROS 与动捕已启动的前提下，运行后进行关节运动控制并记录动捕 + 传感器数据。默认写 `calibration_output/raw_data_motion/joint_<N>/`，同一目录下生成两个 CSV（见下方「CSV 文件与表头」）。

```bash
cd tools/arm_accuracy_calibration
# 默认：关节 2，0°~100° 往复，数据写 calibration_output/raw_data_motion/joint_2/
python3 run_data_collection.py
# 关节 0：全关节保持 0°，数据写 calibration_output/raw_data_motion/joint_0/
python3 run_data_collection.py --joint 0
# 指定工作区或输出目录
python3 run_data_collection.py --workspace my_workspace
python3 run_data_collection.py --joint 3 --start -90 --end 90 --output_dir /path/to/out
```

或使用一键脚本（编译、source、启动 motioncapture、再运行模块一）：

```bash
./run_arm_calibration.sh          # 不编译
./run_arm_calibration.sh --build  # 先编译再运行
```

- 未收到的话题对应列用 **NaN** 填充，保证每行列数一致。
- 模块二只读取 **raw_poses.csv**（动捕），不读 raw_sensor.csv。

#### 参数说明（模块一）

- **--joint**：采集哪一个关节的数据目录（0..7），写入 `calibration_output/raw_data_motion/joint_<joint>/`
- **--start / --end**：该关节往复运动的起止角（deg）
- **--workspace**：工作区根目录（默认当前目录下的 `calibration_output/`；当你在别处运行脚本时可用它定位输出）
- **--output_dir**：显式指定输出目录（优先级通常高于 workspace，若两者同时给出以脚本实现为准）

### 模块二：离线批量解算（run_batch_process）

读取 `calibration_output/raw_data_motion/joint_0`～`joint_7` 下的 **raw_poses.csv**，筛除动捕 pos 任一分量 > 9999 的行后，解算 T 矩阵并写入 `calibration_output/matrix_data_motion/joint_<k>/`。**无需 ROS**。

```bash
cd tools/arm_accuracy_calibration
python3 run_batch_process.py
# 指定工作区
python3 run_batch_process.py --workspace my_workspace
```

- **joint_0**：解算 j2-in-j1、j3-in-j2、j4-in-j3，保存三个 CSV（见下方表头）。
- **joint_1..7**：每个目录解算一对 T（joint_1→j2-in-j1，joint_2→j3-in-j2，joint_3→j4-in-j3，joint_4..7 均为 j4-in-j3）。
- 结束后会弹窗绘制各 T 的平移 tx/ty/tz 随时间曲线。

#### 参数说明（模块二）

- **--workspace**：工作区根目录（用于定位 `calibration_output/`）

### 模块三：拟合并保存（run_fit_save）

读取 **calibration_output/matrix_data_motion** 下 CSV，joint_0 做 T 最小二乘拟合，joint_1～7 做圆拟合，结果写入 **calibration_output/fitted_results/**。**无需 ROS**。

```bash
cd tools/arm_accuracy_calibration
python3 run_fit_save.py
# 或指定工作区 / 单独覆盖数据目录、输出目录
./run_fit_save.sh
python3 run_fit_save.py --workspace my_workspace
./run_fit_save.sh --data_dir calibration_output/matrix_data_motion --output_dir calibration_output/fitted_results --quiet
```

可选参数：`--workspace`（工作区根目录）、`--data_dir`、`--output_dir`、`--quiet`。（模块二已做数据筛选，模块三不再做过滤。）

### 模块四：从拟合 T 解 DH 参数（run_dh_from_fitted）

读取 **calibration_output/fitted_results/joint_0/** 下三个 `*_fitted.csv`（模块三输出），将关节系 T 用 `config/dh_reference_frames.yaml` 转到 DH 系，再反解 DH 参数 [a, alpha, d, theta]，终端打印并保存到工作区根目录 **calibration_output/dh_params.csv**。**无需 ROS**。

```bash
cd tools/arm_accuracy_calibration
python3 run_dh_from_fitted.py
# 指定工作区或输入/输出目录
python3 run_dh_from_fitted.py --workspace my_workspace
python3 run_dh_from_fitted.py --input_dir calibration_output/fitted_results/joint_0 --output_dir calibration_output --quiet
```

可选参数：`--workspace`、`--input_dir`、`--output_dir`、`--quiet`。依赖 **config/dh_reference_frames.yaml**（T_dh_i_to_J_i 矩阵）。

#### 参数说明（模块四）

- **--input_dir**：模块三产物目录（默认 `calibration_output/fitted_results/joint_0`）
- **--output_dir**：DH 参数输出目录（默认 `calibration_output/`，输出 `dh_params.csv`）
- **config/dh_reference_frames.yaml**：DH 参考系到关节系的固定变换（必须存在且与机器人结构一致）

---

## CSV 文件与表头汇总

以下为各模块涉及的全部 CSV 文件及其**完整表头**（列顺序一致，未列出处表示与上文相同或仅一行数据）。

### 模块一输出（`calibration_output/raw_data_motion/joint_<N>/`）

同一目录下两个文件，行数一一对应（同一时刻一行）。

**1. raw_poses.csv**（仅动捕刚体位姿）

| 表头（列顺序） |
|----------------|
| timestamp |
| joint_1_px, joint_1_py, joint_1_pz, joint_1_qx, joint_1_qy, joint_1_qz, joint_1_qw |
| joint_2_px, joint_2_py, joint_2_pz, joint_2_qx, joint_2_qy, joint_2_qz, joint_2_qw |
| joint_3_px, joint_3_py, joint_3_pz, joint_3_qx, joint_3_qy, joint_3_qz, joint_3_qw |
| joint_4_px, joint_4_py, joint_4_pz, joint_4_qx, joint_4_qy, joint_4_qz, joint_4_qw |
| belly_px, belly_py, belly_pz, belly_qx, belly_qy, belly_qz, belly_qw |

**2. raw_sensor.csv**（仅传感器左臂关节角）

| 表头（列顺序） |
|----------------|
| timestamp, left_arm_1, left_arm_2, left_arm_3, left_arm_4, left_arm_5, left_arm_6, left_arm_7 |

- 数据来源：刚体列来自话题 `joint_1`～`joint_4`、`belly_pose`（PoseStamped）；`left_arm_1..7` 来自 `/sensors_data_raw` 的 `joint_q[13:20]`（单位与话题一致）。

---

### 模块二输出（`calibration_output/matrix_data_motion/joint_<k>/`）

**3. T_joint2_in_joint1.csv**（joint_0 下；joint_1 下为同一表头格式）

| 表头（列顺序） |
|----------------|
| timestamp, m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33 |

**4. T_joint3_in_joint2.csv**（joint_0、joint_2 下）

表头同 **3**：`timestamp, m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33`

**5. T_joint4_in_joint3.csv**（joint_0、joint_3～7 下）

表头同 **3**：`timestamp, m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33`

- 矩阵为 4×4 齐次变换，行优先；平移分量 m03, m13, m23 单位 mm。

---

### 模块三输出（`calibration_output/fitted_results/joint_<k>/`）

**6. \<原名\>_fitted.csv**（joint_0 下，如 T_joint2_in_joint1_fitted.csv）

仅一行数据，无 timestamp，表头为 4×4 矩阵行优先：

| 表头（列顺序） |
|----------------|
| m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33 |

**7. circle_fit_result.csv**（joint_1～7 下，每目录一个）

仅一行数据，表头：

| 表头（列顺序） |
|----------------|
| m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33 |
| center_x_mm, center_y_mm, center_z_mm, radius_mm |
| axis_x, axis_y, axis_z |

- 前 16 列为由圆心+轴线构造的 4×4 齐次矩阵（行优先）；圆心、半径单位 mm；轴线为单位向量。

---

## 数据流与配置

### 数据流

```
模块一：动捕话题 + /sensors_data_raw
  → raw_data_motion/joint_<N>/raw_poses.csv + raw_sensor.csv

模块二：raw_poses.csv（筛除 pos>9999）
  → solve_joints_in_frame（joint_0 不乘配置，joint_1～7 乘配置）
  → calibration_output/matrix_data_motion/joint_<k>/*.csv（T 矩阵，带 timestamp）

模块三：calibration_output/matrix_data_motion 下 CSV
  → joint_0：最小二乘拟合 T → *_fitted.csv
  → joint_1～7：圆拟合 → circle_fit_result.csv（T + 圆心 + 半径 + 轴线）
  → calibration_output/fitted_results/joint_<k>/
```

### 配置文件 config/config.yaml

- **arrays**：每个阵列一项，包含 `joint` 与 `T_rigid_body_to_joint`（4×4，单位 mm）。
- 模块二解算时，joint_0 不使用该变换（相当于单位阵），joint_1～7 使用该变换。

---

## 注意事项

1. **模块一**：需先启动 ROS master 与动捕（如用 `run_arm_calibration.sh` 会启动 motioncapture）；话题为 `joint_1`～`joint_4`、`belly_pose`、`/sensors_data_raw`。
2. **无图形界面**：模块一、二结束时的 matplotlib 弹窗可能报错。建议 `export MPLBACKEND=Agg` 关闭交互后端；即使报错，CSV 通常已写入，优先检查输出目录是否生成。
3. **单位**：位置/平移/圆心/半径为 mm；角度为 deg；轴线为单位向量无单位。

## 故障排查

- **超时未收到话题数据**：检查动捕与 `/sensors_data_raw` 是否发布；话题名是否与代码一致。
- **模块二无有效数据**：确认 raw_poses.csv 存在且非空；检查是否整行被筛除（pos>9999 或 nan）。
- **模块三拟合报错**：确认 calibration_output/matrix_data_motion 下对应 joint 目录存在且 CSV 有数据行（非仅表头）。
