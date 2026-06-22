# 四点循环运动与动捕记录

本目录为**四点循环**独立模块，与上层手臂标定数据采集（`run_data_collection` 等）分开，避免混用。

## 文件说明

- **four_points_cycle.py**：四点循环运动与动捕/传感器记录主脚本；每轮循环单独一个 CSV + 每轮 4 张位置图
- **run_four_points_cycle.sh**：一键启动（动捕 SDK + 四点脚本）
- **four_points_to_joint1.py**：离线脚本，读某轮 CSV（如 `output/cycle_0/data.csv`），按点分组并转到 joint_1 系，保存 4 个 CSV
- **output/**：默认输出根目录，其下子文件夹 **cycles/** 内按轮次生成 `cycle_0/`、`cycle_1/`、...，每轮内含 `data.csv` 与 `point_0.png`..`point_3.png`

## 用法

```bash
cd tools/arm_accuracy_calibration/four_points_cycle
./run_four_points_cycle.sh [--build] [--mocap-sdk=optitrack|nokov] [-- 脚本参数...]
```

- 每轮循环的四个点数据写入 `output/cycles/cycle_N/data.csv`（N=0,1,...），有几次循环就生成几个子文件夹。
- 每轮结束后自动根据该轮 CSV 绘制 4 张位置图（8 刚体 X/Y/Z vs 时间），保存为 `output/cycles/cycle_N/point_0.png`..`point_3.png`。
- 可通过 `--output-dir` 指定输出根目录。

## 转到 joint_1 系（离线）

对某一轮的数据做转换时，指定该轮的 CSV：

```bash
python3 four_points_to_joint1.py -i output/cycles/cycle_0/data.csv -o output/cycles/cycle_0
```

生成 `output/cycles/cycle_0/point_0_in_joint1.csv` .. `point_3_in_joint1.csv`。
