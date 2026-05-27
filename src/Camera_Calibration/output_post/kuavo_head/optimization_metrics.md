
============================================================
相机标定优化指标报告
  demo: head
  时间: 2026-04-23T11:26:55+08:00
  CSV 目录: src/Camera_Calibration/output_csv/kuavo_head
  结果目录: src/Camera_Calibration/output/kuavo_head
============================================================

---------- optimize_from_csv ----------
（本脚本不再落盘 roslaunch 完整日志；请直接看终端输出或 /root/.ros/log）


---------- plot_board_error_from_csv (head, 2026-04-23T11:26:55+08:00) ----------
离群点阈值（与 launch 一致）: pos=1 m, rot=100.0 deg
[plot] 使用 optimize_from_csv 写入的样本列表（与优化阶段一致），忽略 --reject_outliers_* 的二次筛选
| sample | pos_err_pre(m) | pos_err_post(m) | rot_err_pre(deg) | rot_err_post(deg) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 0.008660 | 0.008512 | 0.9161 | 0.9530 |
| 1 | 0.009428 | 0.009371 | 0.8915 | 0.8839 |
| 2 | 0.011782 | 0.011424 | 1.3563 | 1.3726 |
| 3 | 0.011858 | 0.011496 | 1.1942 | 1.2197 |
| 4 | 0.010762 | 0.010689 | 0.9847 | 0.9631 |
| 5 | 0.008556 | 0.008551 | 0.7651 | 0.7814 |
| 6 | 0.010793 | 0.010566 | 1.2215 | 1.2489 |
| 7 | 0.011561 | 0.011215 | 1.1856 | 1.2079 |
| 8 | 0.009879 | 0.009823 | 0.7873 | 0.7821 |
| mean | 0.010364 | 0.010183 | 1.0336 | 1.0458 |
Wrote src/Camera_Calibration/output/kuavo_head/board_pose_error_pre_post_vs_urdf.png
Wrote src/Camera_Calibration/output/kuavo_head/board_pose_bars_vs_urdf.png
[summary] 平均误差下降（与输出图片一致，基于最终用于绘图的样本）
  position: mean 0.010364 -> 0.010183 m, drop 0.000181 m (1.75%)
  rotation: mean 1.0336 -> 1.0458 deg, drop -0.0123 deg (-1.19%)
