
============================================================
相机标定优化指标报告
  demo: right_wrist
  时间: 2026-04-24T15:50:16+08:00
  CSV 目录: /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output_csv/kuavo_right_wrist
  结果目录: /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_right_wrist
============================================================

---------- optimize_from_csv ----------
（本脚本不再落盘 roslaunch 完整日志；请直接看终端输出或 /root/.ros/log）


---------- plot_board_error_from_csv (right_wrist, 2026-04-24T15:50:16+08:00) ----------
离群点阈值（与 launch 一致）: pos=0.1 m, rot=10.0 deg
[plot] 使用 optimize_from_csv 写入的样本列表（与优化阶段一致），忽略 --reject_outliers_* 的二次筛选
| sample | pos_err_pre(m) | pos_err_post(m) | rot_err_pre(deg) | rot_err_post(deg) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 0.065275 | 0.003655 | 5.3103 | 0.2033 |
| 1 | 0.060663 | 0.006666 | 5.1433 | 0.5430 |
| 2 | 0.041793 | 0.003241 | 4.2854 | 0.6918 |
| 3 | 0.061546 | 0.002105 | 5.4891 | 0.5201 |
| 4 | 0.053559 | 0.003387 | 4.8513 | 0.5308 |
| 5 | 0.041915 | 0.007530 | 4.2484 | 0.4702 |
| 6 | 0.056459 | 0.008037 | 5.4527 | 0.3521 |
| 7 | 0.036388 | 0.009223 | 3.7196 | 0.6067 |
| 8 | 0.027087 | 0.005158 | 3.1855 | 1.0925 |
| mean | 0.049410 | 0.005445 | 4.6317 | 0.5567 |
Wrote /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_right_wrist/board_pose_error_pre_post_vs_urdf.png
Wrote /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_right_wrist/board_pose_bars_vs_urdf.png
[summary] 平均误差下降（与输出图片一致，基于最终用于绘图的样本）
  position: mean 0.049410 -> 0.005445 m, drop 0.043965 m (88.98%)
  rotation: mean 4.6317 -> 0.5567 deg, drop 4.0750 deg (87.98%)
