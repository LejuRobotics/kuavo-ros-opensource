# 拳师手套与 Quest3 全身速度 IK

融合模式下，Quest3 控制双臂、躯干和底盘，拳师手套给 SG100 提供手指目标。入口为 `quest_velocity_ik.launch`。指定 `hand_control_source:=qianshi` 后默认走增量速度 IK，也可显式选 `teleop_mode:=absolute`。

使用机器人实际的 `robot_version`，其 `kuavo.json` 需配置左右 SG100。当前入口使用一套 Quest 控制器与 Python monitor。IP 按 Quest3 实际地址填写。

```bash
source devel/setup.bash
bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel quest_velocity_ik.launch \
  robot_version:=400062 ip_address:=192.168.5.178 \
  hand_control_source:=qianshi teleop_mode:=incremental qianshi_backend:=linux_sdk
```

用 `qianshi_hand_side:=left` 只启动左手套，`qianshi_hand_side:=right` 只启动右手套，默认 `both` 启动双手套。把该参数追加到上方命令。单手模式只控制所选侧的 SG100 手指，Quest3 的双臂和躯干控制照常。用 Quest 手柄完成 A 准备、双前扳机启动和后续模式切换。

SDK 版中，两只手套共用一个 USB 接收板。默认左手帧 ID 为 12，右手为 2。双手模式要求同一口收到两个目标 ID 的 CRC 有效帧，单手模式只要求所选侧的 ID。用 `qianshi_left_id`、`qianshi_right_id` 修改左右手帧 ID。

| 参数 | 用途 |
| --- | --- |
| `qianshi_hand_side` | `left`、`right` 或 `both` |
| `qianshi_auto_detect` | 默认 `true`。断线后重新枚举和验证串口 |
| `qianshi_right_port` | 共享接收板的首选路径，留空自动搜索。多个接收器均匹配时可指定路径消歧 |
| `qianshi_detected_ports` | 用逗号分隔的候选白名单，留空扫描所有符合条件的候选 |
| `qianshi_baud` | 默认 `921600` |
| `qianshi_sdk_python` | SDK Python 解释器路径，留空由桥接节点解析 |

需要固定串口时，使用 `qianshi_auto_detect:=false qianshi_right_port:=/dev/sg100_glove_receiver`，该口仍需通过所选侧的 ID 验证。自动扫描会跳过 SG100 机手、电源板、遥控器和 CAN 系统串口。若系统别名误指向手套接收板，请先纠正别名。SDK 环境和手动只读扫描命令见 [SG100 SDK 安装说明](../../manipulation_nodes/sg100_glove_bridge/README.md)。

## Windows UDP 启动

Windows 模式不搜索串口、不要求接收板或 Linux SDK。专用入口默认双手、手势开启，左右五指映射均为 `[A,B,C,D,E]`，分别对应拇指、食指、中指、无名指、小指；F 为手腕。专用入口默认绝对式全身 IK，适用于手柄固定手背、无需持续扣扳机的位置跟随；躯干控制保持开启。若需要增量式，显式追加 `teleop_mode:=incremental`，并持续按住对应手柄的侧握键（grip）才会更新该侧位置。

```bash
source devel/setup.bash
bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel quest_windows_udp.launch \
  robot_version:=400062 ip_address:=192.168.5.178
```

`robot_version` 使用实际机型；环境已正确设置 `ROBOT_VERSION` 时可省略。`ip_address` 是 Quest 头显 IP。Windows 的目的 IP 填运行此 launch 的 Linux 主机可达地址。`qianshi_hand_side:=left/right` 仍可选择单手，此时用物理手柄按键启动与切换。

| Windows 数据输出 | 默认目的 UDP 端口 | launch 参数 |
| --- | ---: | --- |
| 右手 IKHand 和右手 SendDof | 18002 | `qianshi_udp_right_port` |
| 左手 IKHand 和左手 SendDof | 18003 | `qianshi_udp_left_port` |

需要同时开启左右手的 IKHand 和 SendDof 共四路输出。同一只手的两种数据填相同端口；接收器按报文头自动区分，分别跟踪序号。旧的 `qianshi_udp_*_ik_port` / `qianshi_udp_*_dof_port` 参数已替换为上表两个参数；18012/18013 不再监听。单手模式只绑定所选侧的一个端口。

通用 `quest_velocity_ik.launch` 仍默认 Linux SDK，Windows 后端可通过 `hand_control_source:=qianshi qianshi_backend:=windows_udp` 选择。通用入口的五指映射默认空列表，需显式传入 `qianshi_udp_left_module_map:='[A,B,C,D,E]' qianshi_udp_right_module_map:='[A,B,C,D,E]'` 才会输出手势位姿。

更换手套或发送端模块布局时，使用 `qianshi_udp_diagnostics:=true` 逐指检查 A～F 并覆盖相应模块映射。协议、冻结恢复和诊断说明见 [Windows UDP 说明](../../manipulation_nodes/sg100_glove_bridge/WINDOWS_UDP.md)。

## 手柄安装与末端标定

全身绝对式入口通过 `quest_controller_calibration_file` 加载 `wheel_arm_abs_vel/config/quest_controller_calibration.yaml`。初始配置关闭；只接受与实际 400062 SG100 URDF 哈希一致的 schema 2 标定。普通 wheel 的 schema 1 旋转不能直接使用。增量模式暂不接受此标定，应使用 `teleop_mode:=absolute`。

默认标定控制器到机器人末端的固定旋转，不假定安装距离为 10 cm，不施加平移补偿。只有独立测量沿掌背法向的安装距离后，才显式提供 `--outward-distance` 启用该平移补偿。400062 SG100 模型的手指沿末端 -Z，左掌心为 -Y，右掌心为 +Y。平移在原始手柄旋转下计算；姿态校正只作用于控制器输入。原始骨骼保留在 `/leju_quest_bone_poses_raw`，再次标定时不会重复补偿。

用 `quest_left_palm_to_controller_offset` / `quest_right_palm_to_controller_offset` 可手动覆盖对应平移向量（米）。默认空列表表示继承标定；无启用标定时为零。显式 `[0,0,0]` 关闭该侧平移补偿。

### 随运控自动录制（常用）

按平时启动运控和本页的 Quest 手套遥操入口。运控自带的 rosbag 会在 `~/.ros` 记录：

- `/leju_quest_bone_poses_raw`：未补偿的肩肘掌位姿，供重复标定。
- `/leju_quest_bone_poses`、`/mm/teleop_tcp_target` 与左右实际末端 FK：检查输入、目标、执行之间的偏差。
- `/quest3/controller_calibration_status`、`/quest3/controller_calibration_mapper_status`：接收端实际平移及 mapper 的模型标定状态。
- `/glove_gesture/*`、左右 `/qianshi/glove_*/fingertips` 和 `/quest_joystick_data`：区分原始识别与最终合成按键。

机器人不必进入手臂跟随状态即可采集标定。保持实际安装，按掌心向上、相对、向下各稳定至少15秒；完成后按正常流程退出并等待 bag 封包。只启动运控而没有开启 Quest 接收，不会产生标定姿态。

若手背安装使掌心向上时的手柄追踪不稳，改录 **掌心相对 → 掌心向下**，每段稳定至少 15 秒，手指仍朝前、手腕不折弯。计算时必须显式指定 `--pose-order facing,down`。固定偏置只能补偿固定安装关系，不能修复遮挡导致的动态定位漂移。

### 独立录制：无需启动运控

保持手柄的实际安装方式，打开 Quest 追踪应用。在已 source ROS 和本仓库 devel 的终端执行：

```bash
bash src/wheel_arm_abs_vel/scripts/record_quest_calibration.sh 192.168.5.31
```

如果已有遥操的 Quest 接收节点运行，请先结束该接收任务，或只用 rosbag 订阅已有姿态话题；脚本会拒绝启动第二个接收器。

入口使用独立 ROS master（11329），仅启动 VR 接收和 rosbag；没有运控、IK 或手套驱动。按“掌心向上 → 掌心相对 → 掌心向下”，双臂向前伸直、手指朝前、手腕不折弯，每段保持至少 15 秒。完成后 Ctrl+C 正常封包，文件位于当前用户 `~/.ros/quest_mount_calibration_*.bag`。

### 离线计算并写入

从仓库根目录执行。不提供 bag 文件时，默认从 `~/.ros` 选择最新已封包的录制；同一录制的完整分片一起读取。新录制仍有 `.active` 时会拒绝读取半成品。也可显式传入某一个 bag，或用 `--bag-dir` 指定其他目录：

```bash
python3 src/manipulation_nodes/noitom_hi5_hand_udp_python/scripts/calibrate_quest_controller.py \
  --eef-convention sg100_400062 \
  --model src/kuavo_assets/models/biped_s400062/urdf/biped_s400062.urdf \
  --calibrate-orientation \
  --report /tmp/quest_mount_report.json --apply
```

以上命令默认三姿态。新录的双姿态 bag 在命令中增加 `--pose-order facing,down`，仍要求每段至少 10 秒且通过全部原有质量门限。若从旧三姿态 bag 中排除向上段，必须同时指定相对、向下的两个确切区间，例如 `--pose-order facing,down --intervals 22:34 40:52`（相对整次录制开始的秒数，仅为示例）。不能让程序从未知安装旋转自行猜测语义标签；存在多个候选时会拒绝写入。

通过全部质量检查后，脚本备份并原子更新全身入口的配置，下次重启遥操生效。当前翻掌 bag 无法唯一确定安装距离：跨姿态的手腕位移与安装偏置会混淆，同轴转动也无法辨识任意三维平移。未知距离不会写入 ROS YAML 的 `known_distance_m` 字段，平移状态为 `not_identifiable_from_recording`；运行时零偏移表示关闭补偿，不是测得零距离。明确测量后可传 `--outward-distance <米数>`，此值记录为用户先验。若拒绝标定，报告会给出稳定区间、残差与原因，生效配置不变。旧 bag 只有 `/leju_quest_bone_poses` 时，必须确认当时没有位置补偿，再加 `--topic /leju_quest_bone_poses --allow-uncompensated-legacy-topic`。

末端位置仍使用模型真实臂长和旋转工具偏移；VR 推断的虚拟肘不等于人体实测肘。标定不会通过放大机器人臂长强行拉直。


## 交互流程

以下为双手模式的手套手势流程。单手模式使用物理手柄按键，仍可输出所选侧的五指位姿供检查。

1. 双手同时做拇指与小指捏合，保持约 2 秒进入交互。手指目标冻结，位姿继续输入手势识别。Windows 模式继续缓存新的 IKHand 并输出新鲜 SendDof；尚未收到首帧 IKHand 的手暂不输出目标，退出交互后恢复最新目标。
2. 右手拇指捏食指相当于 A，保持到原控制器完成启动准备。若控制器已通过原入口启动，直接进行下一步。
3. 双手握拳给出双前扳机请求。全身 IK 继续检查实时 Quest 输入、机器人状态及执行确认。跟随生效后自动退出交互，恢复手指映射。若未启动成功，松开双手后可重试。
4. 左手拇指捏食指、中指分别等效 X、Y，右手分别等效 A、B。后续 X+A 和 X+B 仍走全身 FSM 的准备与恢复、保持与恢复流程。每次启动只接受一次成功的握拳启动。 手套停机：左手拇指同时捏食指和中指（X+Y），右手松开 A/B，连续保持至少 0.2 秒。若同时识别出 A/B，则按 X+A/X+B 处理，松开 A/B 后重新计算停机保持时间。物理手柄 X+Y 仍即时触发停车；手套不会补齐单个物理按键形成停车组合。

全身实机运控入口 `load_kuavo_real_velocity_ik.launch` 默认 `sg100_kp:=0.1 sg100_kd:=0.002`，与普通 wheel 手套融合实际使用的驱动增益一致。需要覆盖时把参数加在运控启动命令上；驱动初始化时缓存增益，修改后须重启运控，单独重启遥操不会更新已缓存的值。

融合入口使用运控内已有的 SG100 驱动，并关闭原 Quest 的 SG100 扳机发布和 mapper bridge。仅收包时可设 `qianshi_publish_real:=false`。用 `qianshi_glove_gesture_enable:=false` 可关闭手套按键映射并使用物理手柄。

## CPU 分区

使用 `platform_roslaunch.sh` 和相同的 `KUAVO_RUNTIME_PROFILE` 配置运行运控和遥操。Linux 接收器和 SDK 共用核池，从当前允许的后台 CPU 中选择最多两个物理核心。候选排除隔离核、WBC、驱动、IK、mapper、sidecar 及其 SMT 同胞。不足两核时用一个，无可用核时报错。Windows UDP 与手势节点留在普通后台分区。`quest_windows_udp.launch` 同样经过 `platform_roslaunch.sh` 的全身分区入口，保留 IK、mapper、WBC 和驱动的核心分配。

在 255H 默认布局且各核均可用时，WBC 为 2/3，驱动保留 6/7，IK 为 5，mapper 为 4，sidecar 为 14/15，手套 SDK 选择 12/13。AGX 按拓扑与 cpuset 选择。启动日志的 `glove SDK and receiver CPUs` 显示生效值。

以下命令只读检查当前主机分区。

```bash
/usr/bin/python3 src/wheel_arm_abs_vel/scripts/runtime_partition.py --check
```
