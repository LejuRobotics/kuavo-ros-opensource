
============================================================
相机标定优化指标报告
  demo: left_wrist
  时间: 2026-04-23T11:27:01+08:00
  CSV 目录: /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output_csv/kuavo_left_wrist
  结果目录: /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_left_wrist
============================================================

---------- optimize_from_csv ----------
（本脚本不再落盘 roslaunch 完整日志；请直接看终端输出或 /root/.ros/log）


---------- plot_board_error_from_csv (left_wrist, 2026-04-23T11:27:01+08:00) ----------
离群点阈值（与 launch 一致）: pos=0.1 m, rot=10.0 deg
[plot] 使用 optimize_from_csv 写入的样本列表（与优化阶段一致），忽略 --reject_outliers_* 的二次筛选
| sample | pos_err_pre(m) | pos_err_post(m) | rot_err_pre(deg) | rot_err_post(deg) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 0.003749 | 0.005752 | 0.9148 | 0.9753 |
| 1 | 0.001924 | 0.000919 | 0.2080 | 0.2701 |
| 2 | 0.008905 | 0.007619 | 0.8702 | 0.9123 |
| 3 | 0.002014 | 0.002829 | 0.5313 | 0.5739 |
| 4 | 0.006227 | 0.005958 | 0.5668 | 0.4653 |
| 5 | 0.003922 | 0.003990 | 0.3405 | 0.3968 |
| 6 | 0.004759 | 0.004199 | 0.8920 | 0.9043 |
| 7 | 0.007609 | 0.005050 | 0.3890 | 0.3435 |
| 8 | 0.009968 | 0.009233 | 1.1950 | 1.1479 |
| mean | 0.005453 | 0.005061 | 0.6564 | 0.6655 |
Wrote /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_left_wrist/board_pose_error_pre_post_vs_urdf.png
Wrote /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_left_wrist/board_pose_bars_vs_urdf.png
[summary] 平均误差下降（与输出图片一致，基于最终用于绘图的样本）
  position: mean 0.005453 -> 0.005061 m, drop 0.000392 m (7.19%)
  rotation: mean 0.6564 -> 0.6655 deg, drop -0.0091 deg (-1.38%)
