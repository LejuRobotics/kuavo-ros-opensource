# Mocap 拟合、验证与模型替换

以下命令在 `kuavo-ros-control` 根目录执行。`ROBOT_VERSION` 选择配置及机型，
操作模式由命令参数决定，不按 capture 文件名判断。

| 参数 | 操作 | 是否询问替换运行模型 |
| --- | --- | --- |
| 无 `--validate-urdf`、无 `--output-urdf` | 25 点拟合、5 点验证、30 点最终拟合，输出 JSON | 否 |
| `--output-urdf` | 拟合并额外生成候选 `urdf_final.urdf` | 成功后询问 `yes/no` |
| `--validate-urdf PATH` | 用指定模型验证输入数据，默认冻结已保存的 CTB/ETK 角度 | 否，不写输出 |

拟合并生成候选：

```bash
ROBOT_VERSION=63 bash src/Camera_Calibration/mocap_kin_calib/mocap_optimize.sh \
  src/Camera_Calibration/mocap_kin_calib/capture_fit.json \
  --output-urdf --force-output
```

`--force-output` 只允许覆盖输出目录里已有的候选 URDF，不代替最后的 `yes`。
25/30 点拟合必须收敛，5 点验证必须通过；最终每个 Ei 的平移向量范数必须
不大于 15 mm，旋转向量范数必须不大于 10°。失败时不生成或覆盖最终输出，
也不询问替换。JSON-only 成功时仍会覆盖输出目录中的两个 JSON。

候选导出后先检查替换清单。只有输入 `yes` 才执行替换；`no`、直接回车、
输入结束或中断都会保留运行模型，候选结果仍保留在输出目录。

S62/S63 拟合及替换目标均为 S63 模型；这不会改变实际控制 launch 的机型选择。
S63 的 `yes` 同步 `src/kuavo_assets/models/biped_s63/urdf` 下的整个模型族：

- 主 `biped_s63.urdf` 使用本次候选。
- `biped_s63_gazebo.urdf`、Drake 的 `biped_v3_arm.urdf`、
  `biped_v3_upper_body.urdf`、`biped_v3_full.urdf` 使用相同的 14 个最终关节 origin，
  保留各自其余模型结构。Full/upper_body 的躯干 `base_link` 对应主模型的
  `waist_yaw_link`，核对的是躯干到 J7 的手臂链。
- `drake/Larm.urdf`、`drake/Rarm.urdf` 从候选重建为标准 S63 单臂模型；
  文件名保留，关节名使用 `zarm_l1_joint` 至 `zarm_l7_joint` 及对应右臂命名。
- Gazebo xacro 无独立手臂几何，通过 include 使用更新后的 Gazebo URDF。

应用前完整备份目录到同级 `urdf.bak.<时间戳>`。所有模型先生成并检查，
写入过程中失败会回滚本次已替换文件。首次应用还保存
`urdf/biped_s63.nominal.urdf`，以后默认拟合始终使用这个原始主模型；该文件不会
随下一次应用覆盖。没有原始备份的已标定模型不能再次作为拟合基准，需用
`--urdf` 显式提供原始模型。原始模型有独立设计变更时应重新核对基准，不能
把旧备份当成新设计。

独立数据验证：

```bash
ROBOT_VERSION=63 bash src/Camera_Calibration/mocap_kin_calib/mocap_optimize.sh \
  src/Camera_Calibration/mocap_kin_calib/capture_vali.json \
  --validate-urdf src/Camera_Calibration/mocap_kin_calib/output_s63/urdf_final.urdf
```

`--validate-urdf` 不能与输出/覆盖开关混用。应用后也可把验证路径换成运行时的
`src/kuavo_assets/models/biped_s63/urdf/biped_s63.urdf`。
文件替换不控制机器人；已运行的 IK/MPC 进程需要重新加载模型后才使用新几何。
