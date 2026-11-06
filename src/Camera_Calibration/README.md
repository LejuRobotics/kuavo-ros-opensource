# Camera_Calibration：Kuavo 头部与双臂标定

编译标定包时，在仓库根目录运行：

```bash
catkin build robot_calibration
```

配置阶段找不到 Ceres 或标定所需的七个 ROS 包时，会尝试从已配置的 apt 软件源安装，需要可用的软件源和网络，以及 root 或免交互 sudo 权限。安装失败时可运行 `tools/install_robot_calibration_deps.sh` 安装依赖后重新编译。其他依赖仍需提前安装。

本目录包含两套标定流程：一套使用动捕测量棋盘和手腕，求解头部、双臂关节零位；另一套使用头部、左右腕相机观察棋盘，分别采集和优化。两套流程的观测来源、输出和写零脚本不同，先按任务选择入口。

| 要做的事 | 入口 | 适用机型 |
|---|---|---|
| 头部与双臂动捕零位标定、独立精度验证 | [`mocap_joint_calib/run_interactive_calibration.sh`](mocap_joint_calib/run_interactive_calibration.sh) | S45、S49 系列、S52、S53、S55、S56、S62、S63 及对应小版本 |
| 只做头部：动捕测棋盘位置、头部相机采集、优化与验证 | [`mocap_checkerboard_pose/run_interactive_head_calibration.sh`](mocap_checkerboard_pose/run_interactive_head_calibration.sh) | S45、S49 系列、S52、S53、S55、S56、S62、S63 及对应小版本 |
| 头部、左右腕三路相机棋盘标定 | [`run_chessboard_calibration.sh`](run_chessboard_calibration.sh) | 头部：全部机型；左右腕：S52、S53、S55、S56、S62、S63 及对应小版本（S45/S49 系列仅 `--demo head`） |

小版本（内部小版本号）指 `ROBOT_VERSION` 按 `%100` 折回后才是基础型号的版本号，例如 `100049`＝49.1、`200049`＝49.2、`200053`、`400055`、`500062`、`400063`。当前接入的小版本：

| 基础型号 | 小版本 | 布局 / 标定 URDF |
|---|---|---|
| 49 | `100049`、`200049`、`300049`、`400049` | `biped45` / `biped_v3_arm_s49.urdf` |
| 53 | `200053`、`300053`、`400053`、`500053` | `biped56` / `biped_v3_arm_s53.urdf` |
| 55 | `200055`、`300055`、`400055`、`500055` | `biped56` / `biped_v3_arm_s55.urdf` |
| 62 | `200062`、`300062`、`400062`、`500062`、`600062` | `wheel62` / `biped_v3_arm_s62.urdf` |
| 63 | `200063`、`300063`、`400063`、`500063` | `wheel62` / `biped_v3_arm_s63.urdf` |

S49 系列与 S45 共用 `biped45` 布局、14 维零点槽位与同一套关节索引，标定流程一致，但**标定模型、采集配置和输出各自独立**：`biped_v3_arm_s49.urdf`（由 `biped_v3_arm_s45.urdf` 逐字复制，仅 `<robot name>` 不同）、`calib_s49.yaml`/`calib_qingtong_s49.yaml`、输出 `biped_v3_arm_s49_calibrated.urdf`，因此标 49 不会覆盖 45 的结果。但**头部相机话题不共用**：S45 的**机器人模型** `kuavo_assets/models/biped_s45/urdf/biped_s45.urdf` 里 `camera` 的 rpy 是 `0 0.48869 0`（未含翻转），驱动因此发布 180° 旋转后的图像与 `head_camera_color_optical_frame_rot180`；S49 系列模型已是 `3.14159 0.38397 0`，用原始话题与 `head_camera_color_optical_frame`。注意**机器人模型和标定 URDF 是两回事**：`biped_v3_arm_s45.urdf` 里的 `camera_joint` 已按实物写入翻转（`3.14159 0.38392 0`），并与 `biped_s100049` 的相机朝向只差 0.003°，所以标定 URDF 里两个帧都提供，不能拿它的 rpy 去反推要不要旋图。两个判定不能混用（见下方"机型判定"）。

S53/S55/S56 共用 `biped56` 布局和关节索引，头部安装高度（`zhead_1_joint` 相对 `waist_yaw_link` 的 Z 坐标）统一为 `0.49575 m`，标定模板与输出各自独立。S63 使用独立的 `biped_v3_arm_s63.urdf` 及 `biped_v3_arm_s63_calibrated.urdf`，不覆盖 S62 输出。模板仍沿用现有手臂几何，小版本的实机差异需要单独核对。

布局/URDF/FK 根/零点脚本/标定配置这些"按机型定值"的东西只在两个文件里维护：Python 侧 [`robot_versions.py`](robot_versions.py)，Shell 侧 [`robot_versions.sh`](robot_versions.sh)（同一张表的两份镜像）。各入口脚本都查这张表，新增小版本时只要基础型号已存在，无需再改脚本。

三路相机流程另有 [`auto_camera_calib_and_apply_zero.py`](auto_camera_calib_and_apply_zero.py) 串联采集、优化和写零，支持 S52、S53、S55、S56、S62/63 及其小版本；**S45 与 S49 系列不支持**（基础型号 45/49 的模型没有可用的左右腕相机帧 `l/r_hand_tripod`、`l/r_hand_camera`；49 系列用的是 `l_camera_link`/`camera_l_base`/`camera_r_base`），包装脚本会直接报错并提示改用 `run_chessboard_calibration.sh --demo head`。不要通过改写 `ROBOT_VERSION` 假装成其他实机版本。


### 运动学标定的 210062 / 310062 资源

仓库包含 `210062` 和 `310062` 各自的主 URDF 及 `kuavo.json`。运动学标定使用完整版本号选择模型、保存采集元数据并隔离优化输出。采集也支持通过 `--urdf` 和 `--kuavo-json` 指定文件；FPF 和默认优化流程使用完整版本对应的主 URDF 路径。

实机使用前，仍需核对模型、关节索引和配置与实际机器人一致；离线测试不能代替实机验证。

## 开始前确认

1. 在标定终端核对 `ROBOT_VERSION`、ROS 参数 `/robot_version` 和实际机器人型号。动捕一键流程检测到两者冲突会停止；三路棋盘脚本遇到机型表里没有的版本会打印 `[WARN]` 后回退 S52，也不会替你校验实机，因此运行前应显式确认版本。
2. 按当前机型的操作规范启动运控，并确认 `/sensors_data_raw` 持续发布。涉及头部的流程还需启动头部相机；**S45** 使用旋转 180° 后的图像和 `camera_info` 话题（`/head_camera/color/image_raw_rotate_180`、`camera_info_rotate_180`），**S49 系列及 S52 以上**使用原始话题。三路棋盘流程还需核对左右腕相机画面与话题对应，确保棋盘角点可见。

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

支持机型与标定模型（45 与 49 系列布局相同但模型文件分开，标定结果互不覆盖）：

| `ROBOT_VERSION`（基础型号） | 布局 / FK 根 | 标定 URDF | 零点文件 |
|---|---|---|---|
| 45 | `biped45` / `base_link` | `biped_v3_arm_s45.urdf` | `arms_zero.yaml` 14 维 + `offset.csv`（EC l1/r1） |
| 49（S49 系列） | `biped45` / `base_link` | `biped_v3_arm_s49.urdf` | `arms_zero.yaml` 14 维 + `offset.csv`（EC l1/r1） |
| 52 | `biped52` / `waist_yaw_link` | `biped_v3_arm.urdf` | `arms_zero.yaml` 14 维 + `offset.csv`（EC l1/r1） |
| 53 | `biped56` / `waist_yaw_link` | `biped_v3_arm_s53.urdf` | `arms_zero.yaml` 16 维（Ruiwo） |
| 55 | `biped56` / `waist_yaw_link` | `biped_v3_arm_s55.urdf` | `arms_zero.yaml` 16 维（Ruiwo） |
| 56 | `biped56` / `waist_yaw_link` | `biped_v3_arm_s56.urdf` | `arms_zero.yaml` 16 维（Ruiwo） |
| 62 | `wheel62` / `waist_yaw_link` | `biped_v3_arm_s62.urdf` | `arms_zero.yaml` 16 维（Motorevo，专用脚本） |
| 63 | `wheel62` / `waist_yaw_link` | `biped_v3_arm_s63.urdf` | `arms_zero.yaml` 16 维（Motorevo，专用脚本） |

关节索引由当前版本的 `src/kuavo_assets/config/kuavo_v${ROBOT_VERSION}/kuavo.json` 推导。S45 与 S49 系列无腰关节；第一次在某台实机上运行前，应检查 `/sensors_data_raw` 的 `joint_q` 长度及双臂、头部索引是否与机型配置一致。

### 机型判定：布局按 `%100` 折回，头部相机只看基础版本

各脚本统一先把 `ROBOT_VERSION` 按 `%100` 折回基础型号再查机型表（`100049→49`、`200053→53`、`400055→55`、`500062→62`、`400063→63`），与 `humanoid_controllers/launch/robot_version_manager.launch` 的折法一致；机型表就是 [`robot_versions.py`](robot_versions.py) / [`robot_versions.sh`](robot_versions.sh)，所以新增小版本一般不需要改脚本。但两件事的判定依据不同，不能混用：

| 判定 | 依据 | 影响 |
|---|---|---|
| 布局、关节索引、FK 根、控制模式、零点脚本 | 折回后的**基础型号**（45 与 49 都→`biped45`；53/55 与 56 都→`biped56`；62 与 63 都→`wheel62`） | 45/49 臂索引 `[12,19)`/`[19,26)`、头索引 `[26,28)`、FK 根 `base_link`；62/63 臂索引 `[4,11)`/`[11,18)`、头索引 `[18,20)` |
| 标定 URDF 与输出 | 折回后的基础型号各用自身文件 | 53/55/56、62/63 分别保存 `*_calibrated.urdf` |
| 头部相机 rot180 话题与虚拟帧 | 基础型号是否**等于 45** | 仅 45 用 `*_rotate_180` 与 `head_camera_color_optical_frame_rot180` |

这里有两个容易混淆的文件，判据必须落在"话题与帧成对切换"上，而不是某个 URDF 的 rpy：

| 文件 | `camera`/`camera_base` rpy | 含义 |
|---|---|---|
| 机器人模型 `models/biped_s45/urdf/biped_s45.urdf` | `0 0.48869 0` | 未含翻转 → 驱动发 `*_rotate_180` 话题 + `head_camera_color_optical_frame_rot180` |
| 机器人模型 `models/biped_s100049/urdf/biped_s100049.urdf` | `3.14159 0.38397 0` | 翻转已进模型 → 原始话题 + `head_camera_color_optical_frame` |
| 标定 URDF `biped_v3_arm_s45.urdf` / `biped_v3_arm_s49.urdf`（内容一致，仅机型名不同） | `camera_joint` = `3.14159 0.38392 0` | **已按实物写入翻转**，与 `biped_s100049` 的相机朝向只差 0.003°；并同时提供 `head_camera_color_optical_frame` 与 `..._rot180` 两个帧 |
| 标定 URDF `biped_v3_arm.urdf` / `_s56` / `_s62` | `0 0.48022 0` / `0 0.34022 0` / `0 0.3406 0` | 无翻转，且只提供 `head_camera_color_optical_frame` |

因此："S45 用旋转话题" 的依据是**机器人模型未含翻转**（决定驱动发布哪个话题/帧），不是标定 URDF 缺翻转；标定 URDF 两帧齐全，是为了让两套约定都能接。成对关系必须保持：**旋转图 ↔ `_rot180` 帧、原图 ↔ 裸 optical 帧**；主点近似在图像中心时这两对在几何上等价，混用则等于多转了 180°。把 rot180 判成 `layout == 'biped45'` 会把 49 系列误判为倒装，这是必须避免的。

读取 `kuavo_v${ROBOT_VERSION}/kuavo.json`、`biped_s${ROBOT_VERSION}` 这类**资源路径始终用原始内部版本号**，只有查布局/URDF/配置表时才用折回后的基础型号。


详细操作、输出、单步工具和现场确认点见 [`mocap_joint_calib/README.md`](mocap_joint_calib/README.md)。只做头部时，也可使用 [`mocap_checkerboard_pose/README.md`](mocap_checkerboard_pose/README.md) 中的独立入口。

## 原有流程：三路相机棋盘标定

此流程分别使用头部、右腕和左腕相机观察棋盘。`capture` 采集 CSV；`optimize` 读取这些 CSV，输出每路关节偏差、标定 URDF 和误差图；`test` 用独立 teach 姿态采样并绘制测试图。`move` 只执行轨迹，不采集。示例：

```bash
# 先设置与实机一致的 ROBOT_VERSION；也可显式传 --robot_layout。
bash src/Camera_Calibration/run_chessboard_calibration.sh capture --demo all
bash src/Camera_Calibration/run_chessboard_calibration.sh optimize --demo all
bash src/Camera_Calibration/run_chessboard_calibration.sh test --demo all
```

可用 `--demo head|left_wrist|right_wrist|all` 只运行所需相机。S45 的头部图像使用旋转 180° 的话题，S49 系列及 S52 以上用原始话题（该话题切换由 `robot_versions.sh` 的 `rv_head_rot180` 统一给出）；各相机的棋盘尺寸、图像话题和 `camera_info` 约定见 [`demos/kuavo_head_demo/README.md`](demos/kuavo_head_demo/README.md)、[`demos/kuavo_left_wrist/README.md`](demos/kuavo_left_wrist/README.md) 和 [`demos/kuavo_right_wrist/README.md`](demos/kuavo_right_wrist/README.md)。**S45 与 S49 系列只支持 `--demo head`**（`biped45` 布局的腕部相机帧不匹配）：s45 的腕相机帧是 `l/r_hand_tripod`、`l/r_hand_camera`，而 100049 用 `l_camera_link`/`camera_l_base`/`l_d405_camera`、200049 用 `camera_l_base`/`camera_r_base`，且 s45 无腰，腕 demo 的 `joint_q` 索引没有对应分支。脚本按布局显式拒绝，不会静默走错分支。S52、S53、S55、S56、S62、S63 的腕部三路相机均支持。


S52、S53、S55、S56、S62/63 及其小版本也可使用下列包装入口，完成三路采集、优化、dry-run 展示，并由操作者决定是否写零（布局与零点脚本由机型表决定，`ROBOT_VERSION` 留空时按 `biped52` 处理并告警）：

```bash
python3 src/Camera_Calibration/auto_camera_calib_and_apply_zero.py
```

## 结果、验证与写零

- 动捕双臂采集生成 `mocap_joint_calib/capture_*.json`，离线优化在 `mocap_joint_calib/output/` 下生成 `calibration.yaml` 和误差图；末端精度验证在 `hand_accuracy_test/` 下生成带时间戳的报告。现场生成的 capture、优化结果和报告不作为仓库基准文件提交。
- 三路棋盘采集数据位于 `output_csv/`，优化和测试结果位于 `output/` 对应 demo 目录。`calibration.yaml` 中的关节值是弧度制 bias，FK 比较时使用 `q_used = q_reported + bias`。
- 写零后需要停止并重新启动运控，使新零点生效；随后使用独立测试姿态复测，不用参与本次优化的采集数据替代验证。更换工装、动捕坐标补偿、URDF 相机模型或机器人位置后，旧结果不能直接与新结果横向比较。

动捕流程的 [`write_zero.py`](mocap_joint_calib/write_zero.py) **独立调用时默认正式写入**，并按电机类型修改 `arms_zero.yaml` 或 `offset.csv`。必须先显式传 `--dry-run` 核对机型、关节、槽位和 `old -> new`，确认后才按其文档执行正式写入；脚本会备份原零点文件。同一份 bias 不要重复写入。三路棋盘流程按基础型号选择零点脚本（机型表的 `apply_script` 字段）：[`apply_zero_deltas_to_arms_zero.py`](apply_zero_deltas_to_arms_zero.py) 负责 45/49/52/53/55/56 及对应小版本，[`apply_zero_deltas_to_arms_zero_wheel62.py`](apply_zero_deltas_to_arms_zero_wheel62.py) 负责 62/63 及对应小版本；两个脚本都会拒绝其它基础型号并提示正确的脚本，也都应先 dry-run。

零点槽位：45/49/52 为 14 维 `[zarm_l2..l7, zarm_r2..r7, zhead_1, zhead_2]`，其中 `zarm_l1`/`zarm_r1` 属 EC 电机，写入 `offset.csv`；53/55/56 为 16 维 `[zarm_l1..l7, zarm_r1..r7, zhead_1, zhead_2]`，手臂关节全在 `arms_zero.yaml`；62/63 同样是 16 维但走 Motorevo 驱动。EC 槽位（`zarm_l1`/`zarm_r1` 在 `offset.csv` 的下标）按各机型 `kuavo.json` 的 `MOTORS_TYPE` 推导：45 与 49 系列为 `12/13`，52 为 `13/14`；不要使用固定常量。


头部优化当前固定 `camera_base` 外参，只估计头部关节零位。S45 倒装相机、S56/S62 相机链（含 S53/S55、S62/S63 小版本）或棋盘固定位置变化后，应重新采集并检查优化前后及独立验证的棋盘误差，避免把模型误差解释成关节零位偏差。
