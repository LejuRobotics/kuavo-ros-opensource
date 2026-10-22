# mocap_joint_calib：纯动捕关节零位标定

使用光学动捕测得的 `l_hand`、`r_hand` 相对 `torso` 的 6DOF 位姿，与 URDF 正向运动学结果比较，通过 Ceres 求解左右臂关节零位偏差。采集端支持：

- Motive / OptiTrack：通过 `optitrack_data_receive` 接收刚体位姿；
- 青瞳 / Avatar：通过 `vrpn_client_ros` 接收刚体位姿。

## 目录说明

```text
mocap_joint_calib/
├── config/
│   ├── calib_s45.yaml              # S45 + Motive
│   ├── calib_qingtong_s45.yaml     # S45 + 青瞳
│   ├── calib.yaml                  # S52/S56/S62/S63 + Motive
│   └── calib_qingtong.yaml         # S52/S56/S62/S63 + 青瞳
├── run_interactive_calibration.sh  # 一键交互入口（推荐）
├── record_joint_poses.py           # 运动、采集并生成 capture JSON
├── mocap_optimize.sh               # Ceres 离线优化入口
├── mocap_optimize.launch
├── mocap_calibration_s45.yaml      # S45 优化模型与 14 个自由参数
├── mocap_calibration.yaml          # wheel62 优化模型
├── write_zero.py                   # 将 bias 写入零点文件
├── scripts/
│   ├── plot_mocap_error.py         # 优化前后误差图
│   ├── compare_fk_mocap_table.py   # 逐点误差表
│   └── regression_test.py          # 合成数据回归测试
└── output/
    ├── calibration.yaml            # 优化输出，单位 rad
    └── mocap_pose_*.png            # 误差图
```

## 一键交互标定（推荐）

### 1. 一键脚本负责什么

主菜单流程 `1. 完整零点标定`（先头部，后末端）会依次执行：

```text
动捕测量棋盘位姿并写入当前机型 URDF（不驱动机构）
        ↓
头部段：只驱动头部走完整条 teach 轨迹，逐点触发棋盘采样
        ↓
手臂段：可选的只运动检查（只驱动双臂），然后只驱动双臂走完整条
        teach 轨迹，逐点采集手腕动捕
        ↓
头部：Ceres 优化 + 误差报告图 + write_zero.py --dry-run 预览 → 人工确认后正式写零
        ↓
手臂：Ceres 优化 + 误差报告图 + write_zero.py --dry-run 预览 → 人工确认后正式写零
        ↓
人工重启运控
        ↓
如需验证，再走菜单 2（完整精度验证）：先头部棋盘、后末端动捕
```

两路写零**严格串行**：Ruiwo 电机构型下 `--parts head` 与 `--parts arm` 写的是同一个
`<config-root>/arms_zero.yaml`，只是槽位不同，并行会互相覆盖。

只做其中一半时用流程 3（头部零点标定）或流程 5（手臂零点标定）；只想复现某一步用菜单 `7` 单步工具。
流程 1 的头部段与流程 3 是**同一个采集入口**，手臂段与流程 5 是同一个入口，只是串在一起执行。

脚本不会自动启动机器人运控、Motive、青瞳接收节点，也不会自动重启运控。运行一键脚本前，
这些数据源必须由现场人员启动。脚本只检查每个必需话题是否能在超时内收到有效消息；为了排除
偶发帧或断流，正式采集前仍建议使用 `rostopic hz` 观察一段时间。

### 2. 支持的机型映射

| `ROBOT_VERSION` | 布局 | FK 根节点 | URDF | 采集配置 |
|---|---|---|---|---|
| 45 | `biped45` | `base_link` | `biped_v3_arm_s45.urdf` | `calib_s45.yaml` / `calib_qingtong_s45.yaml` |
| 52 | `biped52` | `waist_yaw_link` | `biped_v3_arm.urdf` | `calib.yaml` / `calib_qingtong.yaml` |
| 56 | `biped56` | `waist_yaw_link` | `biped_v3_arm_s56.urdf` | `calib.yaml` / `calib_qingtong.yaml` |
| 62、63 | `wheel62` | `waist_yaw_link` | `biped_v3_arm_s62.urdf` | `calib.yaml` / `calib_qingtong.yaml` |

关节总数及双臂、头部、腰部索引由对应的
`src/kuavo_assets/config/kuavo_v${ROBOT_VERSION}/kuavo.json` 自动推导。YAML 中保留的索引是
未自动识别机型时的回退值，不应代替实机 `kuavo.json`。

一键脚本按以下优先级识别机器人版本：

1. 当前终端的 `ROBOT_VERSION`；
2. ROS 参数 `/robot_version`；
3. 两者都没有有效值时，才提示人工选择。

若环境变量与 ROS 参数不一致，脚本会停止，禁止在版本不确定时继续标定。可先检查：

```bash
echo "${ROBOT_VERSION:-未设置}"
rosparam get /robot_version
```

**S45 专属约定**（无腰部关节）：左臂 `joint_q` 为 `[12, 19)`、右臂为 `[19, 26)`，
标定关节为 `zarm_l1..l7` / `zarm_r1..r7`。上车前必须确认实际 `/sensors_data_raw` 的 `joint_q`
长度和索引与配置一致——索引错误会让优化器用错误关节角求解，得到看似正常但不可用的零位偏差。

### 3. 启动前准备

先按当前机型的正常实机流程启动运控。仓库中的常用入口为：

```bash
# S45 / S52 / S56
roslaunch humanoid_controllers load_kuavo_real.launch

# S62 / S63 轮臂
roslaunch humanoid_controllers load_kuavo_real_wheel.launch
```

现场若有专用启动参数或上位机流程，以当前机器人操作规范为准。不要为了本标定重复执行关节限位
标定或改变控制配置。

再启动一种动捕接收方式：

```bash
# Motive / OptiTrack，IP 按现场修改
roslaunch optitrack_data_receive optitrack_data_receive.launch \
  mocap_server_ip:=192.168.x.x

# 或青瞳 / VRPN，IP 按现场修改
roslaunch vrpn_client_ros sample.launch server:=10.10.30.252
```

采集阶段要求 `l_hand`、`r_hand`、`torso`、`l_shoulder` 四个刚体均有数据。
即使 `l_shoulder` 当前只用于辅助记录，它仍在采集器的完整快照检查中，因此不能缺失或停止发布。
若走涉及头部的流程（`1` 完整零点标定、`2` 完整精度验证、`3` 头部零点标定、`4` 头部精度验证），
还必须启动头部相机；S45 需要先启动 180° 旋转节点，使
`/head_camera/color/image_raw_rotate_180` 和 `/head_camera/color/camera_info_rotate_180` 有数据，
其他机型使用原始头部图像和 `camera_info` 话题。

建议在第三个终端确认数据持续更新：

```bash
# 机器人关节数据
rostopic hz /sensors_data_raw

# Motive
rostopic hz /l_hand_pose /r_hand_pose /torso_pose /l_shoulder_pose

# 青瞳
rostopic hz \
  /vrpn_client_node/l_hand/pose \
  /vrpn_client_node/r_hand/pose \
  /vrpn_client_node/torso/pose \
  /vrpn_client_node/l_shoulder/pose
```

### 4. 运行一键流程

在工作空间根目录执行：

```bash
cd /path/to/kuavo-ros-control
bash src/Camera_Calibration/mocap_joint_calib/run_interactive_calibration.sh
```

脚本会自动加载 `/opt/ros/noetic/setup.bash` 和当前工作空间的 `devel/setup.bash`，然后提示选择
Motive 或青瞳。首次在某台机器人运行时，推荐选择菜单 `1`，并在手臂段提示“是否先执行只运动
安全检查（驱动双臂，不采集）”时选择 `y`。

主菜单只有 6 条端到端流程，加上单步工具和退出：

| 选项 | 流程 | 是否驱动机构 | 是否写零 |
|---|---|---:|---:|
| 1 | 完整零点标定（先头部，后末端）：头部采集 + 手臂采集（两段独立）+ 优化 + 误差报告 + 写零 | 头部+双臂（分时） | 两路分别人工确认后写入 |
| 2 | 完整精度验证（先头部，后末端）：头部棋盘位姿 + 验证 + 精度报告 + 精度结果 | 头部+双臂 | 否 |
| 3 | 头部零点标定：棋盘位姿 + 头部采集 + 优化 + 误差报告 + 写零 | 仅头部 | 人工确认后写入 |
| 4 | 头部精度验证：棋盘位姿 + 验证 + 精度报告 + 精度结果 | 仅头部 | 否 |
| 5 | 手臂零点标定：采集 + 优化 + 误差报告 + 写零 | 双臂 | 人工确认后写入 |
| 6 | 末端精度验证：验证 + 精度报告 + 精度结果 | 双臂 | 否 |
| 7 | 单步工具（子菜单，见 4.1） | 视具体单步而定 | 视具体单步而定 |
| 0 | 退出 | 否 | 否 |

表中"优化 + 误差报告"和"精度报告 + 精度结果"都是**自动执行、不询问**：流程 1/3/5 采集成功后
直接优化并画优化前后误差图；流程 2/4/6 验证完成后直接生成报告与精度图并打印结果。
流程 1/3/5 都会**重新采集**；只想复用已有结果或只补做某一步，请走菜单 `7`。

流程 1 的采集是**先头部、后手臂的两段独立采集**，两段各自走自己的完整 teach 轨迹：

1. **头部段**：只驱动头部逐点走完 9 个头部 teach 点。每点在 `hold_sec`（默认 3.0s）静止后
   发布一帧 `/head_keyframe_flag` 触发棋盘采样；全部触发后头部回零，随后发布
   `/head_keyframe_done` 结束头部采集会话，让 `capture_to_csv` 落盘。
   这一段走的是 `capture_head_only`，与流程 3 完全相同——**双臂全程收不到任何指令**。
2. **手臂段**：头部会话已经结束、不再被下发，随后只驱动双臂逐点走完 11 个手臂 teach 点
   （跳过首尾零位得到 9 个有效点），每个有效点在 `hold_sec`（默认 3.0s）静止后采集
   该点的手腕动捕与关节。这一段走 `record_joint_poses.py --no-capture-head`，
   与流程 5 完全相同——**头部一次都不动**。

之所以可以这样拆：头部棋盘标定链（`camera_to_base`、`tag_to_base`）的公共根为
`zarm_l1_ref_link`——它固定在腰部、不随手臂关节转动，链条中不含任何手臂关节；头部采集的
`/joint_states` 也只含 `zhead_1_joint`/`zhead_2_joint`。因此头部棋盘观测与手臂姿态在数学上无关，
拆成两段不会损失精度，却彻底消除了手臂运动遮挡头部相机对棋盘的观测的可能。

头部段采集结束会校验头部 CSV 必须完整包含 `sample_id=0..8`，否则拒绝进入头部优化，
且不会回退使用旧 CSV。

流程 3（头部零点标定）走的**纯头部采集**与流程 1 的头部段是同一个入口：
只启动头部棋盘链采集，按同一套 `sample_id=0..8` 契约走完整条头部 teach 轨迹，
双臂既不参与采集也不被下发指令。
流程 1/3 产出的会话里都没有手臂 capture，`head_zero_pipeline` 也只需要头部 CSV 目录；
棋盘位姿的前提校验由头部子脚本在优化阶段读取会话标记时完成，重复校验不在这里再做一遍。
流程 3 采集前不再单独跑运动检查（该检查只属于还会驱动双臂的流程 1），
现场确认点由头部采集自己的提示与确认承担。

流程 1 的双臂和头部写零彼此独立：每一路只有在该路优化与 dry-run 成功后才会询问是否写入，
选择 `y` 后还需按安全提示输入 `WRITE`。跳过、取消或写入失败一路时，脚本仍会继续另一路。
两路写零**严格串行**执行（先头部、后手臂）——Ruiwo 电机构型下两路写的是同一个
`arms_zero.yaml`，只是槽位不同，并行会互相覆盖。任一路正式写入后，都必须在两路确认结束后
停止并重新启动运控，再做精度验证。

流程 2 的完整精度验证在写零并重启运控之后使用，**先头部、后末端串行执行**（不再沿用旧菜单 13
的 `&`/`wait` 并行实现，便于操作者逐路盯住运动）：头部用棋盘、手腕用动捕，两路各自预检、
各自记录失败，一路取消或失败仍继续另一路。测试数据只用于独立误差评价和画图，不会再次参与
优化，避免用测试集重新拟合标定参数。若确实需要两路同时驱动，走菜单 `7` 单步工具的
「头部+手腕并行精度验证」（需两人盯守）。

流程 2 的头部段与流程 4 一样，以**棋盘位姿写入**为第一步。棋盘位姿写在 URDF 里，是头部标定与
头部验证共同的比较基准，只能靠动捕实测，无法从精度报告反推。该步只做「动捕测量 + 写 URDF」，
不驱动任何机构，人员无需离场；它被取消或棋盘动捕未通过时只跳过头部段，末端段照常执行。

**只要流程涉及头部（1/2/3/4）就一律重测**，不做任何跳过——写零后要重启运控，重启后机器人底座
姿态与位置可能变化，旧基准不再成立；操作者也可能手动重启运控或挪动机器人，脚本无从感知，
复用旧基准会让头部结果整体偏移。因此 `ensure_head_board` 不维护“本轮是否已写入过”这类进程内
状态，进入流程 1/2/3/4 时无条件执行 `--mode board`。代价是每次多几十秒的动捕测量。

**单步工具 10/11 不做这一步**，若从单步工具直接进头部验证，请先走单步 9 完成棋盘位姿写入。

#### 4.1 单步工具子菜单（菜单 `7`）

每一步都不会自动衔接下一步，适合排查单个环节或复用已有结果：

| 单步选项 | 功能 |
|---|---|
| 1 | 只运动检查（驱动双臂，不采集） |
| 2 | 仅采集双臂数据 |
| 3 | 优化最新且与当前机型匹配的 capture（含误差图） |
| 4 | 绘制优化前后误差图（不重新优化） |
| 5 | 写零 dry-run |
| 6 | 正式写零（含 dry-run 和二次确认） |
| 7 | 末端精度验证（含精度报告图） |
| 8 | 绘制最新且与当前机型匹配的精度报告 |
| 9 | 头部联合标定（棋盘位姿 → 相机采集 → 优化/写零） |
| 10 | 头部写零后独立验证（含测试图片） |
| 11 | 头部+手腕并行精度验证（两路同时驱动，需两人盯守） |
| 0 | 返回主菜单 |

两个已知限制：

- **头部误差图没有单独补画的入口。** 头部画图目前融合在 optimize/test 内部
  （`plot_board_error_from_csv.py` 没有独立入口），因此单步工具里没有“仅重画头部误差图”；
  头部误差图会随单步 9（头部联合标定）和单步 10（头部验证）一起生成。
- 单步 11 的并行验证沿用了旧的并发行为实现，手腕测试被主动跳过时会被报告为失败，
  属于已知缺陷；需要逐路确认时请改用主菜单 2 或 6。

### 5. 现场确认点

运行完整流程时，应在以下位置人工判断，而不是连续按回车。
注意**询问点已经收敛**：采集成功后自动优化出图、验证完成后自动出报告出图，
都不再询问；需要人工介入的只剩"每次驱动机构前"和"是否正式写零"：

- 只运动检查（仅流程 1 的手臂段有这一步）：人员退出机械臂运动范围，急停可用；逐点确认无碰撞、无异常抖动；
  该检查**只驱动双臂**（提示为“是否先执行只运动安全检查（驱动双臂，不采集）”）——此时头部已经采集完毕、
  不再被下发；流程 3 只驱动头部、不设单独的运动检查，
  采集前由头部采集自己的"人员远离机构、确认急停可用"提示与确认把关；
- 棋盘位姿：棋盘在底座静止、双臂零位状态下测量；**流程 1/2/3/4 每次进入都重测并写回 URDF**，
  不存在“本轮已写入所以跳过”的情况，重复测量属预期行为；
  该步不驱动机构，人员无需离场；
- 采集：终端没有关节索引警告，左右手有效帧充足，位置和姿态标准差无明显异常；
  流程 1 分两段，头部段必须走完整条轨迹（9 点全部触发采样）后才进入手臂段；
  流程 3 只有头部段，同样要求 9 点全部触发；
  头部段和手臂段现在都在终端打印自己的阶段标题（`[1/4 头部采集]` / `[2/4 手臂采集]` 等），
  两段之间没有任何重叠下发，不会再出现"头臂同时运动"的日志；
- 优化：左右臂或头部优化后误差整体下降，bias 数值合理，不存在单点支配结果；
  优化与画图在采集结束后**自动执行**，不再询问，请注意看终端输出与
  `output/`、`output/kuavo_head/` 下的图；
- dry-run：机器人版本、关节名、motor index、零点文件和 `old -> new` 方向正确；
- 正式写零：**这是流程里唯一的询问点**。先输入 `y`，再按提示输入 `WRITE`，避免误操作；
  不想写零点就答 `n`，优化结果与图仍然保留，可在单步工具中补做；
- 写零后：停止并重新启动运控，新零点才会生效；动捕接收节点可以保持运行；
  两路零点写入是串行的，任一路写入后都必须重启运控，再做下一步验证；
- 精度验证：机器人重启且关节数据恢复后再继续；流程 2 的头部段会先测量并写入棋盘位姿
  （同样要求棋盘静止、双臂零位），随后才驱动头部按测试轨迹验证，最后驱动双臂；
  验证完成后的报告读取与精度图绘制**自动执行**，不再询问；
  逐路确认，使用独立验证轨迹检查泛化效果。

同一份优化结果只允许正式写入一次。一键脚本会记录本次优化的机型、capture 和
`calibration.yaml` 校验值，写入后归档标记，防止误用旧结果或重复扣除同一组 bias。

### 6. 输出文件和可视化

| 位置 | 产物 |
|---|---|
| `mocap_joint_calib/capture_*.json` | 原始关节与动捕采集结果 |
| `mocap_joint_calib/output/calibration.yaml` | 优化得到的关节 bias，单位 rad |
| `mocap_joint_calib/output/mocap_pose_*.png` | 优化前后误差图（pre/post × 左右手 + summary） |
| `hand_accuracy_test/hand_accuracy_report_*` | 末端精度报告：`.json`、`_raw_tooling.csv`、`_waist.csv`、`_accuracy_bars.png` |
| `output_csv/kuavo_head_sessions/<version>/session_*/` | 头部会话：`joints.csv`、`camera_intrinsics.csv`、`features.csv`、`.head_capture_session.json`（URDF 哈希及来源） |

因此，走流程 1/3/5 采集成功后会自动生成“优化前后误差图”；走流程 2/4/6 验证完成后会自动生成
“精度报告图”，两者都不再询问。只有在**采集/验证本身被取消或失败**时对应图才不会生成，
此时可用菜单 `7` 单步工具的 4（重画手臂误差图）或 8（重画末端精度报告图）单独补画；
头部误差图没有独立的补画入口，需重跑单步 9 或单步 10（见 4.1）。

### 7. 常见中断和报错

- `无法连接 ROS master`：先启动 `roscore`/运控，并确认当前终端的 `ROS_MASTER_URI`；
- `/sensors_data_raw` 超时：运控未启动、话题名不对，或 `ROBOT_VERSION` 对应的 `joint_q` 长度不匹配；
- 动捕话题超时：Motive 检查 NatNet Streaming、网络与刚体名称，青瞳检查 Avatar VRPN 服务、服务器 IP
  和 `/vrpn_client_node/<name>/pose`；用 `rostopic hz` 确认话题持续更新，不只是存在；
- `ROBOT_VERSION` 与 `/robot_version` 冲突：修正或清除过期环境变量，再重新运行；
- 找不到“最新匹配 capture”，或优化提示 URDF 与 capture 不匹配：现有文件的机型、URDF 或 FK 根元数据
  与本次选择不一致，应重新采集；S45 必须满足 `fk_root=base_link`、`--layout biped45`、
  URDF 为 `biped_v3_arm_s45.urdf`；
- `record_joint_poses.py: unrecognized arguments`：交互脚本和采集脚本来自不同版本，应同步更新二者；
- 拒绝正式写零：先在同一次交互流程中完成优化，确保来源标记和 `calibration.yaml` 校验值一致；
- `write_zero.py` 找不到配置文件：默认零点目录为 `~/.config/lejuconfig`，机器人使用其他目录时显式传入
  `--config-root`：

  ```bash
  python3 src/Camera_Calibration/mocap_joint_calib/write_zero.py \
    --robot-version 45 --parts arm \
    --calibration-yaml src/Camera_Calibration/mocap_joint_calib/output/calibration.yaml \
    --config-root /实际/lejuconfig/路径 \
    --dry-run
  ```
- 写零后姿态异常：立即停止运控并按急停，使用 `~/.config/lejuconfig` 下对应的 `.bak.*` 恢复。

在正常流程中可用 `Ctrl+C` 终止。若机器人已经出现异常运动，优先使用控制器停止、急停和主电源，
不要依赖终端脚本退出完成安全制动。

## 手动分步命令（排查用）

一键流程已经覆盖下面每一步；这里保留以 S45、双臂 14 关节为例的手动命令，
便于排查单个环节或在不走交互菜单时复现某一步。

### 1. 实机与动捕准备

#### 安全要求

- 确认实机为 S45、`ROBOT_VERSION=45`，并记录当前分支和 commit；
- 首次运行先使用 `--motion-only` 检查所有 teach 点位，人员退出机械臂运动范围；
- 发现异常运动时，先停止控制程序，再按急停，必要时关闭主电源；不要用手扶住抖动的机器人；
- 正常停止可使用控制器停止按钮、终端 `x` + Enter 或 `Ctrl+C`；
- 写零前必须先执行 `--dry-run`，写入后重启控制程序才会生效。

#### 刚体要求

在 Motive 或 Avatar 中创建并持续跟踪：

- `l_hand`：左手末端刚体；
- `r_hand`：右手末端刚体；
- `torso`：躯干刚体，对应 S45 的 `base_link`；
- `l_shoulder`：当前配置中的辅助记录刚体。

刚体名称必须与所选 YAML 一致。手部和 torso 刚体坐标轴应按 URDF 坐标约定建立，工装发生拆装后必须重新确认 `link_offset_mm`。

本模块使用旋转残差，因此 `mocap_frame_align` 必须是合法旋转，即旋转矩阵行列式为 `+1`。`axis_flip: [1, -1, 1]` 是镜像，不可用于 6DOF 姿态对齐；需要调整轴向时，优先在 Motive / Avatar 中重新建立刚体坐标系，或使用合法的 `rpy_rad` 旋转。

动捕接收的启动命令与话题检查见一键部分 **3. 启动前准备**。手动流程只是在已 source 工作空间的终端里固定机型，并用对应的采集配置：

```bash
source devel/setup.bash
export ROBOT_VERSION=45
```

| 动捕系统 | 采集配置 |
|---|---|
| Motive / OptiTrack（NatNet Streaming） | `config/calib_s45.yaml` |
| 青瞳 / Avatar（VRPN） | `config/calib_qingtong_s45.yaml` |

### 2. 检查运动轨迹

首次上车或更换 teach 文件后，先只运动、不采集（青瞳把 `--config` 换成 `config/calib_qingtong_s45.yaml`，下同）：

```bash
python3 src/Camera_Calibration/mocap_joint_calib/record_joint_poses.py \
  --robot-version 45 \
  --config src/Camera_Calibration/mocap_joint_calib/config/calib_s45.yaml \
  --no-capture-head \
  --motion-only
```

逐点确认机械臂不碰撞、不接近奇异位形，左右臂实际关节角与下发角度基本一致。

### 3. 采集标定数据

```bash
python3 src/Camera_Calibration/mocap_joint_calib/record_joint_poses.py \
  --robot-version 45 \
  --config src/Camera_Calibration/mocap_joint_calib/config/calib_s45.yaml \
  --no-capture-head
```

脚本按 teach 轨迹逐点运动，到位并保持静止后采集完整 `joint_q` 和动捕位姿，输出：

```text
src/Camera_Calibration/mocap_joint_calib/capture_YYYYMMDD_HHMMSS.json
```

采集结束后至少检查：

- 没有 `joint_q` 索引布局警告；
- 左右手的 `valid_frames` 足够；
- `pos_std_mm`、`rot_std_deg` 没有明显异常；
- 每个有效样本都包含 `l_hand_in_torso` 和 `r_hand_in_torso`。

### 4. 离线优化

显式指定本次采集文件，避免误用目录中其他 capture：

```bash
bash src/Camera_Calibration/mocap_joint_calib/mocap_optimize.sh \
  src/Camera_Calibration/mocap_joint_calib/capture_YYYYMMDD_HHMMSS.json \
  --layout biped45
```

优化使用 `base_link -> zarm_l7_link / zarm_r7_link` 两条 KDL 链，以位置和旋转共 6 维残差求解双臂 14 个 bias，结果输出到：

```text
src/Camera_Calibration/mocap_joint_calib/output/calibration.yaml
```

检查终端中的 Ceres initial/final cost、解是否可用以及 14 个 bias。不要仅以生成了 YAML 作为成功依据；明显过大的 bias、cost 不下降或残差异常时禁止写零。

### 5. 检查优化效果

生成优化前后误差图：

```bash
python3 src/Camera_Calibration/mocap_joint_calib/scripts/plot_mocap_error.py \
  --capture src/Camera_Calibration/mocap_joint_calib/capture_YYYYMMDD_HHMMSS.json \
  --layout biped45 \
  --calibration_yaml src/Camera_Calibration/mocap_joint_calib/output/calibration.yaml
```

也可以输出逐点误差表：

```bash
python3 src/Camera_Calibration/mocap_joint_calib/scripts/compare_fk_mocap_table.py \
  --capture src/Camera_Calibration/mocap_joint_calib/capture_YYYYMMDD_HHMMSS.json \
  --calibration_yaml src/Camera_Calibration/mocap_joint_calib/output/calibration.yaml
```

重点检查优化后的位置和姿态误差是否整体下降，是否存在单个样本明显偏离，以及左右臂是否都得到改善。

### 6. 使用 write_zero.py 写入零点

`write_zero.py` 根据 `kuavo.json` 中的电机类型和关节数量确定零点槽位：

- Ruiwo 类电机写入 `~/.config/lejuconfig/arms_zero.yaml`，单位 rad；
- EC 类电机写入 `~/.config/lejuconfig/offset.csv`，单位 deg；
- 写入前自动生成带时间戳的 `.bak.*` 备份；
- 不调用 ROS service，不会在运行中直接改变 hardware node 的内存零点。

#### 6.1 必须先 dry-run

显式指定机型和本次优化结果：

```bash
python3 src/Camera_Calibration/mocap_joint_calib/write_zero.py \
  --robot-version 45 \
  --parts arm \
  --calibration-yaml src/Camera_Calibration/mocap_joint_calib/output/calibration.yaml \
  --dry-run
```

逐项核对输出中的：

- `robot_version` 是否为 `45`；
- `kuavo_json` 和 `config_root` 是否指向当前机器人；
- 关节名、motor index、文件类型和 slot 是否正确；
- `old -> new` 的方向和变化量是否符合预期。

#### 6.2 确认后正式写入

确认 dry-run 无误后，去掉 `--dry-run`：

```bash
python3 src/Camera_Calibration/mocap_joint_calib/write_zero.py \
  --robot-version 45 \
  --parts arm \
  --calibration-yaml src/Camera_Calibration/mocap_joint_calib/output/calibration.yaml
```

记录脚本打印的 `backup:` 和 `written:` 路径。随后停止当前控制程序并重新启动，使新零点生效。重启后先在安全姿态低风险验证；如姿态或运动异常，立即停止控制程序并按急停，使用对应 `.bak.*` 文件恢复原零点后再排查。

> 不要对同一份 bias 重复执行正式写入。`write_zero.py` 的语义是在当前零点基础上再次扣除 bias，重复执行会重复修正。

## 历史数据与机型复测

本次调整了手腕/躯干工装偏移、动捕坐标补偿、S45/S56/S62 头部相机模型，以及头部优化的自由参数。
旧 capture、头部 CSV 和精度报告对应的模型与补偿可能不同，应按当时配置留存，不能直接与新采集结果横向比较；
更换工装或重新启动、移动机器人后需重新测量棋盘位姿并采集独立验证数据。
头部优化当前只估计关节零位，`camera_base` 外参固定为 URDF 值；若 S45 倒装相机或 S56 的实际安装
位姿与 URDF 不符，关节 bias 可能吸收外参误差，须通过独立棋盘点位复测确认。
提交到 `dev` 前，应在 MR 附上当前提交的 S45 倒装相机及 S56 实机复测结果（至少包含头部优化前后
board error、所用机型/工装/动捕源，以及旧 CSV 是否作废的结论）。

### 坐标系和可观测性

- S45 的动捕 `torso` 对应 URDF `base_link`；
- 优化观测为 `l_hand_in_torso`、`r_hand_in_torso`；
- `link_offset_mm` 将动捕工装原点换算到对应 link 原点；
- 手部刚体轴与 URDF link 轴的固定偏差会污染腕部关节 bias，安装后必须核对轴向；
- 肘部伸直、腕部姿态单一时，相邻或共轴关节可能不可区分，teach 轨迹应覆盖肩、肘、腕的不同姿态；
- 没有头部刚体时，头部关节不可观测，S45 示例始终使用 `--parts arm`。

### 依赖

- ROS 1；
- Ceres、KDL、`robot_calibration`；
- Python 3、NumPy、PyYAML；
- Motive：`optitrack_data_receive`；
- 青瞳：`vrpn_client_ros`。
