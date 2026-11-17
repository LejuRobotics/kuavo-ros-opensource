# Mocap 拟合、验证与模型替换

以下命令在 `kuavo-ros-control` 根目录执行。`ROBOT_VERSION` 选择配置及机型，
操作模式由命令参数决定，不按 capture 文件名判断。

支持 `53/55/56` 和 `62/63` 系完整内部版本号，例如 `300062`、`200063`、`210062`；
环境变量填 `500063`，不是显示名称 `63.5`。62/63 系共用 `calib_s63.yaml`，
但采集反馈、拟合基准、原始备份和运行模型均按完整版本分别选择，不再将 62 路由到 63。
对应的 `kuavo.json` 和主 URDF 必须已存在；缺失资产不会回退到基础版本。
工装安装基准仍须与配置的 CTB/ETK 一致。

采集使用本包入口，复用 `mocap_joint_calib` 的动作、采样、工装预处理和 JSON 输出，
不修改其源码。本包自行读取 YAML、选择完整版本模型，并把轨迹路径转成绝对路径。
只在本次调用中接管原采集器的配置加载和模型默认值函数，不读取其默认 YAML，
也不依赖 `_ROBOT_CALIB_DEFAULTS`。同事修改这些默认配置不会影响本入口；
若修改录点接口、配置字段或 JSON 格式，仍需核对兼容性。
原 CLI 参数继续可用，显式 `--config`、`--urdf`、`--kuavo-json` 等优先：

```bash
ROBOT_VERSION=500063 python3 src/Camera_Calibration/mocap_kin_calib/record_joint_poses.py \
  --fixture new \
  --output /实际路径/capture_fit.json
```

采集会按原流程控制手臂；显式 `--config`、`--urdf` 优先。采集 JSON 保留完整版本号，
优化时环境变量必须与其一致。拟合命令中的 `63` 可换成同一个完整版本号。
62/63 系默认输出分别为 `output_s62`、`output_s63`、`output_s500063` 等；自定义 YAML 输出目录保持不变。

版本和路径的来源：

| 内容 | 决定方式 |
| --- | --- |
| 采集版本 | 显式 `--robot-version` 优先，否则读取 `ROBOT_VERSION`；完整编号写入 JSON 元数据 |
| 优化版本 | `ROBOT_VERSION` 与 JSON 中的版本必须一致；直接运行 Python 且未设置环境变量时，依次取 JSON 版本、YAML 版本 |
| 默认配置 | 62/63 系都用本包 `calib_s63.yaml`；这个文件名不代表实际版本是 63 |
| 拟合基准 | 当前版本自己的主 URDF；有 `biped_s${V}.nominal.urdf` 时用该原始备份；显式 `--urdf` 优先 |
| 默认输出 | 本包 `output_s${V}/`，包含候选 URDF 和偏置 JSON；自定义 `kinematics.output_dir` 按 YAML 目录解析 |
| 最后写回 | 输入 `yes` 后，只同步 `kuavo_assets/models/biped_s${V}/urdf/` |
| 验证模型 | 由 `--validate-urdf` 的路径明确指定，不随版本参数自动替换该路径 |

## 给测试的 5W 命令

测试机必须取得包含本次适配的提交。以下以开发分支 `gqx/feat/mocap-calib-5w-variants`
发布后为例；合入其他分支后改用对应分支。新克隆后需先进入仓库目录：

```bash
git clone https://www.lejuhub.com/highlydynamic/kuavo-ros-control.git
cd kuavo-ros-control
git switch gqx/feat/mocap-calib-5w-variants
catkin build humanoid_controllers robot_calibration optitrack_data_receive
source devel/setup.bash
```

所有终端都在仓库根目录操作。动捕接收和机器人启动命令沿用原流程；
先确认接收 launch 的动捕服务器 IP，再分别运行：

```bash
# 终端一：动捕接收
source devel/setup.bash
roslaunch optitrack_data_receive optitrack_data_receive.launch
```

```bash
# 终端二：机器人；填写这台实机的完整版本号
source devel/setup.bash
export ROBOT_VERSION=62
roslaunch humanoid_controllers load_kuavo_real_wheel.launch
```

终端三使用相同完整版本；`--fixture new` 对应测试流程的新工装。
录拟合姿态前，确认本包 `config/calib_s63.yaml` 的 `calibration.teach_left/teach_right`
分别为 `../mocap_kin_calib/config/teach_left_joint_fit.json` 和对应的 `teach_right_joint_fit.json`。
每次录制会控制机器人按轨迹运动：

```bash
source devel/setup.bash
export ROBOT_VERSION=62
python3 src/Camera_Calibration/mocap_kin_calib/record_joint_poses.py \
  --fixture new \
  --output src/Camera_Calibration/mocap_kin_calib/capture_fit.json
```

录独立验证姿态前，将同一 YAML 的 `teach_left/teach_right` 分别改为
`../mocap_kin_calib/config/teach_left_joint_validation.json` 和对应的 `teach_right_joint_validation.json`。
下次录拟合姿态前改回 `*_fit.json`；文件名 `capture_vali.json` 本身不会切换运动轨迹。

```bash
python3 src/Camera_Calibration/mocap_kin_calib/record_joint_poses.py \
  --fixture new \
  --output src/Camera_Calibration/mocap_kin_calib/capture_vali.json
```

终端四的版本与采集一致。无需重复指定默认 `calib_s63.yaml`：

```bash
source devel/setup.bash
export ROBOT_VERSION=62
bash src/Camera_Calibration/mocap_kin_calib/mocap_optimize.sh \
  src/Camera_Calibration/mocap_kin_calib/capture_fit.json \
  --output-urdf --force-output

bash src/Camera_Calibration/mocap_kin_calib/mocap_optimize.sh \
  src/Camera_Calibration/mocap_kin_calib/capture_vali.json \
  --validate-urdf "src/Camera_Calibration/mocap_kin_calib/output_s${ROBOT_VERSION}/urdf_final.urdf" \
  --no-freeze-frame-angles
```

这里保留原测试命令的 `--no-freeze-frame-angles`：验证时重新估计共享 CTB/ETK 旋转，
URDF 几何和 CTB/ETK 平移不变。只有共享 CTB（schema 1）候选可去掉该参数，
连同拟合时保存的 CTB/ETK 旋转一起固定验证。手动选择逐点 CTB（schema 2）的候选没有可冻结的共享 CTB；
若需受限逐点验证，设置 `kinematics.validation.qingtong: true`，其行为见下方模式表。
版本 `62` 的候选在 `output_s62`，不能继续写死为 `output_s63`。
对小版本，在每个相关终端把 `62` 换成相同完整编号（例如 `500063`）。


## 一键采集、拟合和验证

由测试人员手动启动机器人控制、OptiTrack 或青瞳 VRPN，并加载 ROS 环境。
一键脚本先选择采集设备，再进入“开始拟合采点？”，直接订阅已启动的话题，
不包含 OptiTrack/VRPN 启动步骤。准备好后运行：

```bash
bash src/Camera_Calibration/mocap_kin_calib/mocap_optimize_fpf.sh
```

脚本默认 `ROBOT_VERSION=62`；已设置的环境变量优先，例如
`ROBOT_VERSION=500063 bash .../mocap_optimize_fpf.sh`。默认使用 `calib_s63.yaml`，
S53/S55/S56 使用 `calib_s56.yaml`，也可用 `--config PATH` 指定话题、CTB/ETK 和阈值配置。
62/63 系及其完整内部版本号共用参数配置，但采集模型、拟合基准和替换目录都按实际 `ROBOT_VERSION` 选择：
`62` 使用 `biped_s62/urdf/biped_s62.urdf`，`500063` 使用 `biped_s500063/urdf/biped_s500063.urdf`。

例如以下命令均使用 `calib_s63.yaml` 中的三个 Ei 权重和数据质量阈值，
分别读取自己的 `kuavo_v${ROBOT_VERSION}/kuavo.json` 和主 URDF，
默认分别输出到 `output_s300062`、`output_s200063`、`output_s210062`：

```bash
ROBOT_VERSION=300062 bash src/Camera_Calibration/mocap_kin_calib/mocap_optimize_fpf.sh
ROBOT_VERSION=200063 bash src/Camera_Calibration/mocap_kin_calib/mocap_optimize_fpf.sh
ROBOT_VERSION=210062 bash src/Camera_Calibration/mocap_kin_calib/mocap_optimize_fpf.sh
```

流程依次询问：

1. **选择采集设备**：`1` 为青瞳，`2` 为动捕 OptiTrack。青瞳将本次拟合和验证的四个话题改为
   `/vrpn_client_node/l_hand/pose`、`/vrpn_client_node/r_hand/pose`、
   `/vrpn_client_node/torso/pose`、`/vrpn_client_node/l_shoulder/pose`（`PoseStamped`）；
   OptiTrack 保留 YAML 中的原话题。只修改本次运行配置，原 YAML 不变。
2. **开始拟合采点？** `y` 执行 `mocap_kin_calib/record_joint_poses.py --fixture new`，
   自动使用 `teach_left_joint_fit.json` / `teach_right_joint_fit.json`，完整采集 30 点后保存 `capture_fit.json`。
3. 检查 torso—shoulder 数据质量：左右链各取一次原始位置均值；任一点距均值 >8 mm 退出，
   >3 mm 则同步剔除该 pose，最多剔除 3 个，超过则退出。删点后不重算均值，打印原始编号及偏差。
4. **开始验证采点？** `y` 自动切到 `teach_left_joint_validation.json` /
   `teach_right_joint_validation.json`，当前轨迹为 10 个采集点，保存 `capture_vali.json`。
5. **开始拟合并导出候选？** `y` 按质量筛选后的实际点数执行拟合、留出验收和最终拟合，
   更新输出目录中的 `urdf_final.urdf`、`kinematic_offsets.json`、`frame_transforms.json`。
6. **开始验证候选 URDF？** `y` 使用独立验证文件和候选 URDF，默认重拟合共享 CTB/ETK 旋转。
7. 两次验收通过后，才询问是否替换运行模型；只有输入 `yes` 才备份并安装。

设备选择仅控制采集话题和来源标记，不决定 CTB 拟合模式；拟合及验证模式只读取
YAML 配置，默认均使用共享 CTB。采集器显式接收所选的 `--mocap-source qingtong/motive`。

两次采点提示也可输入 `r` 复用已有完整文件；`n`、回车、EOF 或中断会停止流程。
新采集先写入本次运行目录，点数、索引、观测和质量检查通过后才替换目标 capture，
不完整或质量不合格的采集不会覆盖已有完整文件；失败的新数据保留在运行目录。
teach 路径只写入本次运行配置，原 YAML 不变。

稳定性诊断不需要 shoulder 的真实 URDF 变换。对每点、每只手计算
`T_torso_shoulder = T_torso_hand @ inverse(T_shoulder_hand)`，左右测量链分别用
原始完整 30 点的位置计算一次算术均值，再计算每点距该均值的三维欧氏距离。
任一链上任一点 **超过 8 mm** 就报数据质量错误并退出，不通过删点规避这道门限。
否则，将任一链上 **超过 3 mm** 的 pose 加入剔除集合；两条链取并集，
同一 pose 同步剔除、数量只算一次。需要剔除超过 3 个则退出。
**删点后不重算均值，不迭代筛选**；保留点的偏差仍相对原始 30 点均值。
恰好 8 mm 不触发硬退出，但会剔除；恰好 3 mm 保留。
最大两两位置差和 SO(3) 旋转角差只作诊断，不参与退出或剔除，也不切换 CTB 模式。
两两差与距均值偏差含义不同：例如两点各距均值 3 mm、方向相反，两两差可达 6 mm。
两个 YAML 均可配置：

```yaml
workflow:
  shoulder_stability:
    max_translation_mm: 8.0
    exclude_translation_mm: 3.0
    max_excluded_poses: 3
    max_rotation_deg: 1.0
```

位置指标针对记录时的参考原点，旋转指标不依赖平移偏移。这些质量门限不代表
绝对测量精度。四通道用于质量检查，不增加拟合残差，也不要求 shoulder 外参。
缺失或无效观测会报错。`stability.json` 使用 schema 3，明确标记
`quality_metric: distance_from_original_mean`，保存两条链的原始均值、逐点有符号 XYZ 偏差、
距均值距离、需要剔除和实际剔除的编号。日志区分超过 8 mm 与超限点数超过 3 个。

原始 capture 不删点、不重编号。原始 25 FIT / 5 VALIDATION 分组中被选中的坏点均剔除，
不把留出点补入拟合集；最终拟合也只用保留点。例如剔除 FIT 的 1 点和 VALIDATION 的 2 点，
流程变为 24 FIT → 3 VALIDATION → 27 FINAL。日志显示实际编号和点数，两个 JSON 及候选
URDF 元数据保存 `data_selection`（采集文件哈希、剔除编号、实际分组、质量指标、原始均值和阈值）。
一键入口在独立验证采集前检查，独立拟合入口在求解前执行同样规则。
另外采集的独立验证文件仍使用全部验证 pose，不按拟合数据的编号剔除。

下文的 25/5/30 表示筛选前的分组；若发生剔除，以日志和导出元数据中的实际点数为准。

| 默认模式 | 25/30 点拟合 | 5 点留出验收 | 独立候选验证 |
| --- | --- | --- | --- |
| 共享 CTB | 一组 CTB 旋转 + 共享 ETK/Ei | 所有拟合参数冻结 | 几何冻结，重拟合共享 CTB/ETK |

一键脚本内部统一使用 `--validate-urdf ... --no-freeze-frame-angles`；不对外提供
freeze/no-freeze 开关，也不改写 `kinematics.ctb.mode` 或 `kinematics.validation.qingtong`。
独立验证报告的是重拟合参考角度后的误差，不是完全冻结参数的预测误差。
共享 CTB 不施加逐点 3° 上限。手动逐点模式的配置见下文。

默认输出 `output_s${ROBOT_VERSION}`（例如 `output_s62`、`output_s500063`），日志、阶段配置和 `stability.json` 保存在其
`fpf_<时间戳>_*` 子目录。质量失败时也保存 `stability.json`，包含逐 pose 最大差及对应 pose、
超限点对、退出原因或剔除结果。可自定义输入、输出路径以复用历史采集：

```bash
bash src/Camera_Calibration/mocap_kin_calib/mocap_optimize_fpf.sh \
  --fit-capture /path/to/capture_fit.json \
  --validation-capture /path/to/capture_vali.json \
  --output-dir /path/to/results
```

## 独立拟合和验证接口

| 参数 | 操作 | 是否询问替换运行模型 |
| --- | --- | --- |
| 无 `--validate-urdf`、无 `--output-urdf` | 25 点拟合、5 点验证、30 点最终拟合，输出 JSON | 否 |
| `--output-urdf` | 拟合并额外生成候选 `urdf_final.urdf` | 成功后询问 `yes/no` |
| `--validate-urdf PATH` | 固定指定模型的机械几何；由 `validation.qingtong` 选择参考角验证方式 | 否，不写输出 |

独立拟合默认 `kinematics.ctb.mode: shared`，使用 93 参数共享 CTB 模型。
5 点验收完全沿用 25 点拟合参数，不重拟合参考角度；通过后再进行 30 点最终拟合。

仅显式设置 `kinematics.ctb.mode: per_sample` 时启用逐点模式。CTB 平移由 YAML 固定，
每个采集点单独拟合 CTB 旋转，同一点左右手共用。
只用 `kinematics.ctb.rotation_bound_deg` 控制角度上限，默认 `3.0`：
每个 CTB 相对单位阵 `I` 的总旋转角不得超过 3°，不是三个轴各自允许 ±3°。
逐点模式不包含共享 CTB 旋转，也不叠加 `initial_rotvec_deg`。
左右 ETK 的平移继续固定，ETK 旋转和 14 个 Ei 跨点共享；不引入 shoulder
外参或 shoulder 残差。Ei 正则权重与共享模式相同，见下文。N 点联合拟合共有 `90 + 3N` 个参数，
25 点为 165 个，30 点为 180 个。

25 点拟合后，ETK/Ei 固定为 25 点拟合结果，仅为 5 个留出点各自拟合 CTB，
仍相对 `I` 受 3° 总旋转角上限约束。验收使用调整参考角度后的条件误差，
不属于完全冻结参考角度的预测误差。验证通过才对全部 30 点联合拟合，
逐点 CTB 仍受相同的角度上限约束。

逐点模式的 `frame_transforms.json` 使用 schema 2 保存 CTB 旋转、采集文件哈希及
共享 ETK；仍支持读取旧 schema 1 的共享 CTB/ETK 元数据。
共享模式输出 schema 1。
逐点 CTB 不写入 URDF 关节几何，候选 URDF 仅将 Ei 合入 14 个手臂关节 origin，
CTB/ETK 作为验证元数据保留。

拟合并生成候选：

```bash
ROBOT_VERSION=63 bash src/Camera_Calibration/mocap_kin_calib/mocap_optimize.sh \
  src/Camera_Calibration/mocap_kin_calib/capture_fit.json \
  --output-urdf --force-output
```

`--force-output` 只允许覆盖输出目录里已有的候选 URDF，不代替最后的 `yes`。
`--output-urdf --no-install` 仅导出候选、跳过安装询问；一键脚本用它把安装推迟到独立验证之后。
25/30 点拟合必须收敛，5 点验证必须通过；最终每个 Ei 的平移和旋转向量范数必须
满足 `mocap_optimize_1010.py` 中的 `EI_OUTPUT_*` 上限。失败时不生成或覆盖最终输出，
也不询问替换。JSON-only 成功时仍会覆盖输出目录中的两个 JSON。

### Ei 偏置正则权重

在 `config/calib_s56.yaml` 或 `config/calib_s63.yaml` 中修改：

```yaml
kinematics:
  optimizer:
    EI_POSITION_WEIGHT: 100.0
    EI_THETA_WEIGHT: 0.0001
    EI_OTHER_ROTATION_WEIGHT: 1.0
```

对每个关节，令 `t` 为 Ei 平移（mm），`r` 为 Ei 旋转向量（deg），`a` 为
URDF 中该关节的单位局部轴，`theta = dot(r, a)`、`r_other = r - theta*a`。
目标函数在观测残差平方和之外追加
`w_position*||t||² + w_theta*theta² + w_other*||r_other||²`，整个最小二乘目标乘 `1/2`。
实现中残差乘 `sqrt(weight)`；每个关节只追加一次，CTB/ETK 不加这三项正则。
权重必须有限且非负，`0` 关闭对应项。增大位置权重抑制 xyz 偏置，减小 theta
权重允许更大的轴向旋转；有非轴向旋转时，theta 不等同于独立电机零点。

三个权重同时设为 `0` 可关闭这部分正则；旧 YAML 缺少这些键时同样按 `0` 处理。
旧 `ei.translation_prior_mm` / `rotation_prior_deg` 仍兼容并叠加，默认均为 `0`。
求解器数值缩放保持内部默认平移 `2 mm`、旋转 `1 deg`，旧的
`translation_scale_mm` / `rotation_scale_deg` 配置仍可覆盖数值缩放。
一键流程原样传递三个权重到阶段 YAML，拟合日志打印生效值。
权重不改变 25/5/30 分组、验证误差计算、验收阈值和输出上限。

候选导出后先检查替换清单。只有输入 `yes` 才执行替换；`no`、直接回车、
输入结束或中断都会保留运行模型，候选结果仍保留在输出目录。

62/63 系使用自身主 URDF 的原始基准拟合。`yes` 同步当前完整版本的
`src/kuavo_assets/models/biped_s${ROBOT_VERSION}/urdf` 模型族。
例如 `500063` 只更新 `biped_s500063/urdf`，保留其 O6 手及相机结构。
以下文件名中的 `${V}` 表示完整版本号：

- 主 `biped_s${V}.urdf` 使用本次候选。
- `biped_s${V}_gazebo.urdf`、Drake 的 `biped_v3_arm.urdf`、`biped_v3_full.urdf`，
  以及目录中已有的 `biped_v3_upper_body.urdf`、`biped_v3_arm_only.urdf`，
  使用相同的 14 个最终关节 origin，保留各自其余结构、惯性、限位和 mesh。
  Upper_body 兼容 `base_link` 或 `waist_yaw_link` 躯干根；核对躯干到 J7 的链。
- `drake/Larm.urdf`、`drake/Rarm.urdf` 从该版本候选重建为标准单臂模型；
  文件名保留，关节名使用 `zarm_l1_joint` 至 `zarm_l7_joint` 及对应右臂命名。
- Gazebo xacro 无独立手臂几何，通过 include 使用更新后的 Gazebo URDF。

应用前完整备份目录到同级 `urdf.bak.<时间戳>`。所有模型先生成并检查，
写入过程中失败会回滚本次已替换文件。首次应用还保存
`urdf/biped_s${V}.nominal.urdf`，以后默认拟合始终使用这个原始主模型；该文件不会
随下一次应用覆盖。没有原始备份的已标定模型不能再次作为拟合基准，需用
`--urdf` 显式提供原始模型。原始模型有独立设计变更时应重新核对基准，不能
把旧备份当成新设计。

独立数据验证时，`kinematics.validation.qingtong` 默认 `false`，保留原共享
CTB/ETK 逻辑：默认 `--freeze-frame-angles` 冻结旧 schema 1 保存的共享角度；
`--no-freeze-frame-angles`（同 `--fit-frame-angles`）重新拟合共享 CTB 和左右
ETK 共 9 个角度参数，不施加逐点 CTB 的 3° 上限。

青瞳数据验证需将对应 YAML 中的 `kinematics.validation.qingtong` 改为 `true`，
再执行：

```bash
ROBOT_VERSION=63 bash src/Camera_Calibration/mocap_kin_calib/mocap_optimize.sh \
  src/Camera_Calibration/mocap_kin_calib/capture_vali.json \
  --validate-urdf src/Camera_Calibration/mocap_kin_calib/output_s63/urdf_final.urdf
```

`--validate-urdf` 不能与输出/覆盖开关混用。应用后也可把验证路径换成运行时的
`src/kuavo_assets/models/biped_s63/urdf/biped_s63.urdf`。
`qingtong: true` 时，两种开关都重新拟合当前数据的逐点 CTB，相对 `I` 受
`ctb.rotation_bound_deg` 限制。默认 `--freeze-frame-angles` 仅冻结保存的共享
ETK；`--no-freeze-frame-angles` 还会重新拟合共享 ETK。两者都属于调整参考角度
后的条件误差。元数据中的逐点 CTB 和采集哈希用于记录拟合来源，不作为新验证
数据的固定 CTB；旧 schema 1 仍可提供共享 ETK。

schema 2 没有单一共享 CTB，因此 `qingtong: false` 配合默认冻结模式会报错，
不会取均值或选某一点代替。可选择 `qingtong: true` 做青瞳逐点验证，或保留
`qingtong: false` 并追加 `--no-freeze-frame-angles`，从 `I` 开始重新拟合共享
CTB，同时以保存的 ETK 为初值拟合共享 ETK。

独立接口的 `validation.qingtong` 只控制 `--validate-urdf`，训练模式由 `ctb.mode`
控制；一键流程同时设置这两个值。所有独立验证模式都固定指定 URDF 的机械几何。
文件替换不控制机器人；已运行的 IK/MPC 进程需要重新加载模型后才使用新几何。
