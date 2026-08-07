
============================================================
相机标定优化指标报告
  demo: right_wrist
  时间: 2026-07-18T23:08:19+08:00
  CSV 目录: /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output_csv/kuavo_right_wrist
  结果目录: /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output/kuavo_right_wrist
============================================================

---------- optimize_from_csv ----------
（本脚本不再落盘 roslaunch 完整日志；请直接看终端输出或 /root/.ros/log）


---------- plot_board_error_from_csv (right_wrist, 2026-07-18T23:08:19+08:00) ----------
离群点阈值（与 launch 一致）: pos=0.32 m, rot=120 deg
[plot] 使用 optimize_from_csv 写入的样本列表（与优化阶段一致），忽略 --reject_outliers_* 的二次筛选
| sample | pos_err_pre(m) | pos_err_post(m) | rot_err_pre(deg) | rot_err_post(deg) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 0.017928 | 0.015478 | 1.4686 | 1.2285 |
| 1 | 0.017699 | 0.010646 | 2.1694 | 2.1201 |
| 2 | 0.012664 | 0.011544 | 2.5962 | 2.8828 |
| 3 | 0.018615 | 0.013313 | 1.7563 | 1.6057 |
| 4 | 0.020006 | 0.014338 | 2.3362 | 2.1395 |
| 5 | 0.014535 | 0.010432 | 2.4029 | 2.1702 |
| 6 | 0.016183 | 0.010977 | 2.0296 | 1.7770 |
| 7 | 0.011543 | 0.007044 | 2.7254 | 2.5808 |
| 8 | 0.013845 | 0.008813 | 2.8560 | 2.7306 |
| mean | 0.015891 | 0.011398 | 2.2601 | 2.1372 |
Wrote /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output/kuavo_right_wrist/board_pose_error_pre_post_vs_urdf.png
Wrote /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output/kuavo_right_wrist/board_pose_bars_vs_urdf.png
[summary] 平均误差下降（与输出图片一致，基于最终用于绘图的样本）
  position: mean 0.015891 -> 0.011398 m, drop 0.004493 m (28.27%)
  rotation: mean 2.2601 -> 2.1372 deg, drop 0.1228 deg (5.43%)
