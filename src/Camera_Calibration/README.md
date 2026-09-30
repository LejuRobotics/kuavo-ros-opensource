# Camera_Calibration：Kuavo 头部与双臂标定

本目录包含两套标定流程：一套使用动捕测量棋盘和手腕，求解头部、双臂关节零位；另一套使用头部、左右腕相机观察棋盘，分别采集和优化。两套流程的观测来源、输出和写零脚本不同，先按任务选择入口。

| 要做的事 | 入口 | 适用机型 |
|---|---|---|
| 头部与双臂动捕零位标定、独立精度验证 | [`mocap_joint_calib/run_interactive_calibration.sh`](mocap_joint_calib/run_interactive_calibration.sh) | S45、S52、S56、S62、S63 |
| 只做头部：动捕测棋盘位置、头部相机采集、优化与验证 | [`mocap_checkerboard_pose/run_interactive_head_calibration.sh`](mocap_checkerboard_pose/run_interactive_head_calibration.sh) | S45、S52、S56、S62、S63 |
| 头部、左右腕三路相机棋盘标定 | [`run_chessboard_calibration.sh`](run_chessboard_calibration.sh) | S45、S52、S56、S62、S63 |

三路相机流程另有 [`auto_camera_calib_and_apply_zero.py`](auto_camera_calib_and_apply_zero.py) 串联采集、优化和写零；该包装脚本当前只分流 S52、S56、S62/63，**不支持 S45**。53/55 也未接入上述一键入口；不要通过改写 `ROBOT_VERSION` 假装成其他实机版本。

## 开始前确认

1. 在标定终端核对 `ROBOT_VERSION`、ROS 参数 `/robot_version` 和实际机器人型号。动捕一键流程检测到两者冲突会停止；三路棋盘脚本在未识别版本时可能回退到 S52，因此运行前应显式确认版本。
2. 按当前机型的操作规范启动运控，并确认 `/sensors_data_raw` 持续发布。涉及头部的流程还需启动头部相机；S45 使用旋转 180° 后的图像和 `camera_info` 话题。三路棋盘流程还需核对左右腕相机画面与话题对应，确保棋盘角点可见。
3. 动捕流程需先启动 Motive/OptiTrack 或青瞳/VRPN 接收节点，并确认所需刚体持续发布。头部棋盘测量需要 `checkerboard`、`torso`、`l_shoulder`；双臂采集还需要 `l_hand`、`r_hand`。更换工装后先核对配置中的刚体到 URDF link 偏移。
4. 每次驱动头部或双臂前，确认运动范围无人、急停可用，并先检查 teach 轨迹。出现异常运动时先停止控制程序，再按急停，必要时关闭主电源。

常用核对命令：

```bash
echo "${ROBOT_VERSION:-未设置}"
rosparam get /robot_version
rostopic hz /sensors_data_raw
# Motive：按实际流程检查 /checkerboard_pose、/torso_pose、/l_shoulder_pose、/l_hand_pose、/r_hand_pose
# 青瞳：检查 /vrpn_client_node/<刚体名>/pose
```

脚本不会替现场人员启动运控、相机驱动或动捕接收节点。ROS 环境和工作空间需要已构建；动捕交互入口会加载 `/opt/ros/noetic/setup.bash` 与当前工作空间的 `devel/setup.bash`。具体节点、话题和准备步骤见下方各流程文档。

## 推荐流程：头部与双臂动捕标定

在仓库根目录运行：

```bash
bash src/Camera_Calibration/mocap_joint_calib/run_interactive_calibration.sh
```

脚本会确认机型、手腕工装（`legacy` 或 `new`）和动捕源（Motive 或青瞳），再显示以下主菜单：

| 菜单 | 用途 | 写零 |
|---|---|---|
| 1 | 完整零位标定：先测棋盘、采头部，再独立采双臂；分别优化、出图、确认写零 | 可选，头部与双臂串行写入 |
| 2 | 完整精度验证：先头部棋盘，再末端动捕 | 不写零 |
| 3 / 4 | 仅头部标定 / 仅头部精度验证 | 仅 3 可选写零 |
| 5 / 6 | 仅双臂标定 / 仅末端精度验证 | 仅 5 可选写零 |
| 7 | 单步工具：只运动检查、采集、优化、绘图、写零预览或单项验证 | 按所选步骤 |

完整标定中的头部、双臂是两段独立运动：头部采集时不下发双臂轨迹，双臂采集时不下发头部轨迹。头部棋盘位姿在涉及头部的主流程中重新测量，不能复用机器人移动或运控重启前的旧基准。采集完成后，脚本按当前机型、工装和动捕源核对数据来源，分别生成优化结果和误差图；正式写零前会先展示 dry-run 计划，再要求人工确认。

支持机型与标定模型：

| `ROBOT_VERSION` | 布局 / FK 根 | 标定 URDF |
|---|---|---|
| 45 | `biped45` / `base_link` | `biped_v3_arm_s45.urdf` |
| 52 | `biped52` / `waist_yaw_link` | `biped_v3_arm.urdf` |
| 56 | `biped56` / `waist_yaw_link` | `biped_v3_arm_s56.urdf` |
| 62、63 | `wheel62` / `waist_yaw_link` | `biped_v3_arm_s62.urdf` |

关节索引由当前版本的 `src/kuavo_assets/config/kuavo_v${ROBOT_VERSION}/kuavo.json` 推导。S45 无腰关节；第一次在某台实机上运行前，应检查 `/sensors_data_raw` 的 `joint_q` 长度及双臂、头部索引是否与机型配置一致。

详细操作、输出、单步工具和现场确认点见 [`mocap_joint_calib/README.md`](mocap_joint_calib/README.md)。只做头部时，也可使用 [`mocap_checkerboard_pose/README.md`](mocap_checkerboard_pose/README.md) 中的独立入口。

## 原有流程：三路相机棋盘标定

此流程分别使用头部、右腕和左腕相机观察棋盘。`capture` 采集 CSV；`optimize` 读取这些 CSV，输出每路关节偏差、标定 URDF 和误差图；`test` 用独立 teach 姿态采样并绘制测试图。`move` 只执行轨迹，不采集。示例：

```bash
# 先设置与实机一致的 ROBOT_VERSION；也可显式传 --robot_layout。
bash src/Camera_Calibration/run_chessboard_calibration.sh capture --demo all
bash src/Camera_Calibration/run_chessboard_calibration.sh optimize --demo all
bash src/Camera_Calibration/run_chessboard_calibration.sh test --demo all
```

可用 `--demo head|left_wrist|right_wrist|all` 只运行所需相机。S45 的头部图像使用旋转 180° 的话题；各相机的棋盘尺寸、图像话题和 `camera_info` 约定见 [`demos/kuavo_head_demo/README.md`](demos/kuavo_head_demo/README.md)、[`demos/kuavo_left_wrist/README.md`](demos/kuavo_left_wrist/README.md) 和 [`demos/kuavo_right_wrist/README.md`](demos/kuavo_right_wrist/README.md)。

S52、S56、S62/63 也可使用下列包装入口，完成三路采集、优化、dry-run 展示，并由操作者决定是否写零：

```bash
python3 src/Camera_Calibration/auto_camera_calib_and_apply_zero.py
```

## 结果、验证与写零

- 动捕双臂采集生成 `mocap_joint_calib/capture_*.json`，离线优化在 `mocap_joint_calib/output/` 下生成 `calibration.yaml` 和误差图；末端精度验证在 `hand_accuracy_test/` 下生成带时间戳的报告。现场生成的 capture、优化结果和报告不作为仓库基准文件提交。
- 三路棋盘采集数据位于 `output_csv/`，优化和测试结果位于 `output/` 对应 demo 目录。`calibration.yaml` 中的关节值是弧度制 bias，FK 比较时使用 `q_used = q_reported + bias`。
- 写零后需要停止并重新启动运控，使新零点生效；随后使用独立测试姿态复测，不用参与本次优化的采集数据替代验证。更换工装、动捕坐标补偿、URDF 相机模型或机器人位置后，旧结果不能直接与新结果横向比较。

动捕流程的 [`write_zero.py`](mocap_joint_calib/write_zero.py) **独立调用时默认正式写入**，并按电机类型修改 `arms_zero.yaml` 或 `offset.csv`。必须先显式传 `--dry-run` 核对机型、关节、槽位和 `old -> new`，确认后才按其文档执行正式写入；脚本会备份原零点文件。同一份 bias 不要重复写入。三路棋盘流程使用 [`apply_zero_deltas_to_arms_zero.py`](apply_zero_deltas_to_arms_zero.py)（S45/52/56）或 [`apply_zero_deltas_to_arms_zero_wheel62.py`](apply_zero_deltas_to_arms_zero_wheel62.py)（S62/63），也应先 dry-run。

头部优化当前固定 `camera_base` 外参，只估计头部关节零位。S45 倒装相机、S56/S62 相机链或棋盘固定位置变化后，应重新采集并检查优化前后及独立验证的棋盘误差，避免把模型误差解释成关节零位偏差。
