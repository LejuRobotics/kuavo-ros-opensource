
============================================================
相机标定优化指标报告
  demo: head
  时间: 2026-04-24T15:50:13+08:00
  CSV 目录: /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output_csv/kuavo_head
  结果目录: /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_head
============================================================

---------- optimize_from_csv ----------
（本脚本不再落盘 roslaunch 完整日志；请直接看终端输出或 /root/.ros/log）


---------- plot_board_error_from_csv (head, 2026-04-24T15:50:13+08:00) ----------
离群点阈值（与 launch 一致）: pos=1 m, rot=100.0 deg
[plot] 使用 optimize_from_csv 写入的样本列表（与优化阶段一致），忽略 --reject_outliers_* 的二次筛选
| sample | pos_err_pre(m) | pos_err_post(m) | rot_err_pre(deg) | rot_err_post(deg) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 0.009355 | 0.009287 | 0.9692 | 0.9786 |
| 1 | 0.010017 | 0.009571 | 0.9952 | 0.9612 |
| 2 | 0.012005 | 0.011510 | 1.3595 | 1.3813 |
| 3 | 0.011944 | 0.011301 | 1.3372 | 1.3543 |
| 4 | 0.010598 | 0.010131 | 1.1049 | 1.0371 |
| 5 | 0.009503 | 0.009399 | 0.8916 | 0.9078 |
| 6 | 0.011683 | 0.011523 | 1.1900 | 1.2297 |
| 7 | 0.011956 | 0.011510 | 1.2647 | 1.2936 |
| 8 | 0.010804 | 0.010442 | 0.8713 | 0.8681 |
| mean | 0.010874 | 0.010519 | 1.1093 | 1.1124 |
Wrote /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_head/board_pose_error_pre_post_vs_urdf.png
Wrote /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_head/board_pose_bars_vs_urdf.png
[summary] 平均误差下降（与输出图片一致，基于最终用于绘图的样本）
  position: mean 0.010874 -> 0.010519 m, drop 0.000355 m (3.26%)
  rotation: mean 1.1093 -> 1.1124 deg, drop -0.0031 deg (-0.28%)
