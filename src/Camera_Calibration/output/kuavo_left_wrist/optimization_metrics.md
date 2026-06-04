
============================================================
相机标定优化指标报告
  demo: left_wrist
  时间: 2026-04-24T15:50:19+08:00
  CSV 目录: /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output_csv/kuavo_left_wrist
  结果目录: /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_left_wrist
============================================================

---------- optimize_from_csv ----------
（本脚本不再落盘 roslaunch 完整日志；请直接看终端输出或 /root/.ros/log）


---------- plot_board_error_from_csv (left_wrist, 2026-04-24T15:50:19+08:00) ----------
离群点阈值（与 launch 一致）: pos=0.1 m, rot=10.0 deg
[plot] 使用 optimize_from_csv 写入的样本列表（与优化阶段一致），忽略 --reject_outliers_* 的二次筛选
| sample | pos_err_pre(m) | pos_err_post(m) | rot_err_pre(deg) | rot_err_post(deg) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 0.031782 | 0.007515 | 3.7163 | 0.9730 |
| 1 | 0.028068 | 0.001000 | 2.8428 | 0.2833 |
| 2 | 0.021228 | 0.007708 | 2.5464 | 0.7348 |
| 3 | 0.028064 | 0.002443 | 3.0355 | 0.4451 |
| 4 | 0.031267 | 0.005339 | 2.5383 | 0.6378 |
| 5 | 0.035279 | 0.004461 | 3.5462 | 0.3961 |
| 6 | 0.029752 | 0.003500 | 3.6033 | 0.9742 |
| 7 | 0.012673 | 0.003862 | 2.0530 | 0.8912 |
| 8 | 0.020584 | 0.009081 | 2.2359 | 1.1084 |
| mean | 0.026522 | 0.004990 | 2.9020 | 0.7160 |
Wrote /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_left_wrist/board_pose_error_pre_post_vs_urdf.png
Wrote /home/lab/guofucheng/kuavo-ros-control/src/Camera_Calibration/output/kuavo_left_wrist/board_pose_bars_vs_urdf.png
[summary] 平均误差下降（与输出图片一致，基于最终用于绘图的样本）
  position: mean 0.026522 -> 0.004990 m, drop 0.021532 m (81.19%)
  rotation: mean 2.9020 -> 0.7160 deg, drop 2.1860 deg (75.33%)
