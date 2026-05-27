
============================================================
相机标定优化指标报告
  demo: right_wrist
  时间: 2026-04-23T11:26:58+08:00
  CSV 目录: src/Camera_Calibration/output_csv/kuavo_right_wrist
  结果目录: src/Camera_Calibration/output/kuavo_right_wrist
============================================================

---------- optimize_from_csv ----------
（本脚本不再落盘 roslaunch 完整日志；请直接看终端输出或 /root/.ros/log）


---------- plot_board_error_from_csv (right_wrist, 2026-04-23T11:26:58+08:00) ----------
离群点阈值（与 launch 一致）: pos=0.1 m, rot=10.0 deg
[plot] 使用 optimize_from_csv 写入的样本列表（与优化阶段一致），忽略 --reject_outliers_* 的二次筛选
| sample | pos_err_pre(m) | pos_err_post(m) | rot_err_pre(deg) | rot_err_post(deg) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 0.005582 | 0.006089 | 0.3555 | 0.3575 |
| 1 | 0.005837 | 0.006615 | 0.8208 | 0.8329 |
| 2 | 0.002779 | 0.002584 | 0.8382 | 0.8008 |
| 3 | 0.003847 | 0.003530 | 0.6588 | 0.6858 |
| 4 | 0.003453 | 0.003331 | 0.7057 | 0.6287 |
| 5 | 0.008409 | 0.008021 | 0.6031 | 0.5392 |
| 6 | 0.006897 | 0.007161 | 0.2491 | 0.2415 |
| 7 | 0.008239 | 0.008389 | 0.6568 | 0.6926 |
| 8 | 0.004984 | 0.004598 | 1.2174 | 1.1756 |
| mean | 0.005559 | 0.005591 | 0.6784 | 0.6616 |
Wrote src/Camera_Calibration/output/kuavo_right_wrist/board_pose_error_pre_post_vs_urdf.png
Wrote src/Camera_Calibration/output/kuavo_right_wrist/board_pose_bars_vs_urdf.png
[summary] 平均误差下降（与输出图片一致，基于最终用于绘图的样本）
  position: mean 0.005559 -> 0.005591 m, drop -0.000032 m (-0.58%)
  rotation: mean 0.6784 -> 0.6616 deg, drop 0.0168 deg (2.47%)
