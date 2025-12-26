# 7 自由度旋量标定（Python）

基于等距离约束的 POE 标定，纯位置场景（无需姿态），支持 7 自由度。

## 目录结构
```
calibration_python/
├── function/                 # 数学工具
│   └── screw_utils.py
├── calibrate_screw.py        # 联合标定入口（旋量+Pe0）
├── calibrate_pe0.py          # 仅标定 Pe0
├── config/                   # 理论参数、初始工具矩阵
└── input/output/             # 数据输入与结果输出
```

核心函数对照：
- `exp_se3_r / jacobi_pos / forward_kinematics` 等：对应 MATLAB function 目录。
- `calibrate_robot_screw()`：对应 MATLAB 主标定流程。

## 环境依赖
```
python3.8+
pip install numpy matplotlib
```

## 数据准备
- 旋量初值：`config/theory_kesi.txt`（6×7，米）
- 工具矩阵：`config/target_flange_matrix.txt`（4×4，取第4列为 Pe0，米）
- 关节角：`input/calibration_joints.txt`（N×7，度）
- 末端位置：`input/calibration_points.txt`（N×3，毫米）

## 快速运行
联合标定（旋量 + Pe0）：
```
cd tools/calibration_python
python3 calibrate_screw.py
```
仅标定 Pe0：
```
cd tools/calibration_python
python3 calibrate_pe0.py
```

## 输出
- `output/iden_kesi.txt`：标定旋量（6×7）
- `output/iden_pe0.txt`：标定工具平移（4×1）
- `output/calibration_errors.txt` / `calibration_errors.png`：误差统计与曲线

## 算法要点
- 优化变量：42（旋量增量）+3（Pe0）= 45 维
- 约束：任意两位形的实测距离² = 理论距离²（等距离约束）
- 求解：正则化伪逆 `(JᵀJ + λI)⁻¹ Jᵀ e`，默认 λ=1e-6

## 参数可调
在 `calibrate_screw.py` 末尾：
- `num_train / num_test`：训练/测试集划分
- `lambda_reg`：正则化系数
- `max_iter` / `tol`：迭代上限与收敛阈值

## 使用提示
- 单位：关节角输入度，内部自动转弧度；位置输入毫米，内部转米。
- 7 自由度自适应：关节数取 `kesi_init.shape[1]`。
- 稳定性：数据噪声大时可增大 λ；收敛慢可略减小 λ。

## 与 MATLAB 版本差异
- 支持 7 自由度；正则化已实现。
- 直接联合优化 45 维（未分阶段）。
- 模块化 Python 结构，便于集成与复现。
