
============================================================
相机标定优化指标报告
  demo: head
  时间: 2026-07-18T23:08:17+08:00
  CSV 目录: /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output_csv/kuavo_head
  结果目录: /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output/kuavo_head
============================================================

---------- optimize_from_csv ----------
（本脚本不再落盘 roslaunch 完整日志；请直接看终端输出或 /root/.ros/log）


---------- plot_board_error_from_csv (head, 2026-07-18T23:08:17+08:00) ----------
离群点阈值（与 launch 一致）: pos=1 m, rot=200.0 deg
[plot] 使用 optimize_from_csv 写入的样本列表（与优化阶段一致），忽略 --reject_outliers_* 的二次筛选
| sample | pos_err_pre(m) | pos_err_post(m) | rot_err_pre(deg) | rot_err_post(deg) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 0.017666 | 0.008082 | 2.8339 | 2.5560 |
| 1 | 0.014322 | 0.004254 | 2.9359 | 2.6742 |
| 2 | 0.023002 | 0.012502 | 3.2858 | 3.0652 |
| 3 | 0.022845 | 0.012370 | 3.0061 | 2.7847 |
| 4 | 0.015945 | 0.006887 | 2.7062 | 2.4330 |
| 5 | 0.014487 | 0.004387 | 2.6980 | 2.4239 |
| 6 | 0.023861 | 0.014333 | 2.9628 | 2.7374 |
| 7 | 0.022219 | 0.011785 | 2.8096 | 2.5724 |
| 8 | 0.013926 | 0.004532 | 2.8160 | 2.5535 |
| mean | 0.018697 | 0.008792 | 2.8949 | 2.6445 |
Wrote /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output/kuavo_head/board_pose_error_pre_post_vs_urdf.png
Wrote /media/xiezhicong/新加卷1/kuavo-ros-control/src/Camera_Calibration/output/kuavo_head/board_pose_bars_vs_urdf.png
[summary] 平均误差下降（与输出图片一致，基于最终用于绘图的样本）
  position: mean 0.018697 -> 0.008792 m, drop 0.009905 m (52.97%)
  rotation: mean 2.8949 -> 2.6445 deg, drop 0.2504 deg (8.65%)
