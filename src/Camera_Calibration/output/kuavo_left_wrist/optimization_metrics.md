
============================================================
相机标定优化指标报告
  demo: left_wrist
  时间: 2026-07-18T23:08:22+08:00
  CSV 目录: /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output_csv/kuavo_left_wrist
  结果目录: /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output/kuavo_left_wrist
============================================================

---------- optimize_from_csv ----------
（本脚本不再落盘 roslaunch 完整日志；请直接看终端输出或 /root/.ros/log）


---------- plot_board_error_from_csv (left_wrist, 2026-07-18T23:08:22+08:00) ----------
离群点阈值（与 launch 一致）: pos=0.32 m, rot=120 deg
[plot] 使用 optimize_from_csv 写入的样本列表（与优化阶段一致），忽略 --reject_outliers_* 的二次筛选
| sample | pos_err_pre(m) | pos_err_post(m) | rot_err_pre(deg) | rot_err_post(deg) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 0.024015 | 0.020769 | 1.4233 | 1.6073 |
| 1 | 0.021053 | 0.014305 | 2.0337 | 2.0799 |
| 2 | 0.018401 | 0.013422 | 3.2990 | 3.4264 |
| 3 | 0.022513 | 0.016409 | 1.1943 | 1.2754 |
| 4 | 0.021920 | 0.012425 | 2.4602 | 2.2953 |
| 5 | 0.020012 | 0.009205 | 2.8106 | 2.7315 |
| 6 | 0.018756 | 0.012323 | 1.9712 | 1.9850 |
| 7 | 0.014672 | 0.005607 | 2.5860 | 2.5548 |
| 8 | 0.032198 | 0.023514 | 3.8454 | 3.7434 |
| mean | 0.021504 | 0.014220 | 2.4026 | 2.4110 |
Wrote /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output/kuavo_left_wrist/board_pose_error_pre_post_vs_urdf.png
Wrote /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output/kuavo_left_wrist/board_pose_bars_vs_urdf.png
[summary] 平均误差下降（与输出图片一致，基于最终用于绘图的样本）
  position: mean 0.021504 -> 0.014220 m, drop 0.007284 m (33.87%)
  rotation: mean 2.4026 -> 2.4110 deg, drop -0.0084 deg (-0.35%)
