# 拳师手套 + Quest3 + SG100 全身遥操使用手册

更新日期：2026-11-23。个人手势标定章节对应本地新增功能；网络地址与机器人部署信息保留此前记录，使用前按现场确认。

本文用于 `kuavo-ros-control` **的全身绝对式 IK 版本**，分支为 `czb/beta/glove-vel-ik`，机器人型号为 `400062`。主流程采用 **Windows UDP 手套输入 + Quest3 手背固定手柄 + SG100 双手**。

机器人工作区为 `/home/lab/lihengyu/kuavo-ros-control`。下文命令除特别说明外，都在机器人上执行。普通 `kuavo-ros-control-wheel` 的旧 launch 和标定配置不能直接替换本文的全身入口。

## 1. 当前配置与地址

| 项目 | 当前值 / 含义 |
| --- | --- |
| 机器人 SSH 地址 | `lab@10.10.31.123` |
| 机器人在 dynlab111 有线网段的地址 | `192.168.5.11`；当前同一网卡另有 `192.168.5.127` |
| Quest 头显 IP | 最近实际启动使用 `192.168.5.31`；头显换网后需重新确认 |
| Windows UDP 目的 IP | 当前可填写机器人 `192.168.5.11`，不是头显 IP |
| 机器人型号 | `400062` |
| 灵巧手类型 | `heiman`，即当前 SG100 |
| 遥操入口 | `wheel_arm_abs_vel/quest_windows_udp.launch` |
| 遥操模式 | 默认 `absolute`，绝对式跟随 |
| 手套模式 | 默认双手、手势识别开启 |
| 当前安装标定 | 已启用；左右距离均为用户指定的 **0.15 m**，同时启用已验证的姿态校正 |
| 标定配置提交 | `339fb2a887`：配置：启用手背安装十五厘米偏移及已验证姿态标定 |

**15 cm 是指定的安装距离，不是从 bag 测得的真值。** 脚本本身不再默认假设 10 cm 或 15 cm；当前 15 cm 来自已经保存的这份配置。

网络地址变化时，在机器人上查看：

```bash
ip -4 -brief address
```

`ip_address:=...` 填 **Quest 头显地址**；Windows 软件的 UDP 目标地址填 **机器人地址**。不要把路由器网关 `192.168.5.1` 当作机器人地址。

## 2. 日常启动：两个终端

### 2.1 启动前准备

1. 按现场流程给机器人、两只 SG100 和手套供电。
2. 戴好 Quest，打开原来的 Quest 遥操应用，使用两个物理手柄的追踪数据。
3. 保持手柄与手套安装牢固。当前安装为手背固定，不额外启用旧的 180° 翻转修正。
4. Windows 软件开启下一节中的四路输出。
5. 运控与遥操各使用一个终端；不要同时运行另一套旧遥操或第二个 Quest 接收器。

在两个终端分别 SSH 登录：

```bash
ssh lab@10.10.31.123
```

使用本机平时运行运控的账号和权限。如果现场流程需要 `sudo -s`，先切换，再执行下面的 `cd`、`source` 和 `export`。两个终端保持一致的 ROS 环境与机器人型号。个人手势配置保存在当前仓库内，切换账号不会改变默认目录；需保证文件权限，见第 8.4 节。

### 2.2 终端一：运控

```bash
cd /home/lab/lihengyu/kuavo-ros-control
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROBOT_VERSION=400062

bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel load_kuavo_real_velocity_ik.launch \
  robot_version:=400062 ee_type:=heiman
```

等待运控和 SG100 驱动完成初始化，保持这个终端运行，再启动终端二。

- `ee_type:=heiman` 明确选中当前 SG100，避免机器配置自动识别到其他末端。
- 这个入口默认锁住底盘：`base_locked:=true`。若本次确实要使用底盘遥操，在**运控命令**追加 `base_locked:=false`，并重启运控后生效。
- SG100 当前默认增益为 `sg100_kp:=0.1 sg100_kd:=0.002`。通常不必重复填写；修改增益需要重启运控。

### 2.3 终端二：Quest + 手套融合遥操

```bash
cd /home/lab/lihengyu/kuavo-ros-control
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROBOT_VERSION=400062

bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel quest_windows_udp.launch \
  robot_version:=400062 ip_address:=192.168.5.31
```

这里默认已经包含 Windows UDP、双手、手势开启、五指模块映射和绝对式模式。更新到包含个人标定功能的版本后，合格的个人手势配置也会在本次启动自动加载，确认方式见第 8.4 节。`ip_address` 填 Quest 头显 IP；显式写型号是为了让两个终端一致。

此入口默认开启躯干跟随。若本次只需要固定躯干，给**遥操命令**追加 `control_torso:=false`；头部跟随可用 `enable_head_control:=false` 关闭。

**运行 launch 后还需要第 4 节的手势启动流程，机器人不会仅因为收到 UDP 就进入跟随。**

### 2.4 不想在脚本前写 `bash`

脚本具有可执行权限时，可以直接执行，其他准备步骤不变：

```bash
./src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel quest_windows_udp.launch \
  robot_version:=400062 ip_address:=192.168.5.31
```

省略的是 `bash`，不是整个 `platform_roslaunch.sh`。该脚本处理本机运行环境和 CPU 分区，两个终端都沿用它。若原环境已配置 `KUAVO_RUNTIME_PROFILE`，两边使用相同配置，不要单独取消分区。

## 3. Windows 软件的四路输出

Windows 软件的四项输出都开启，目标 IP 填机器人 `192.168.5.11`：

| 软件输出 | 目的 UDP 端口 | 用途 |
| --- | ---: | --- |
| 右手 IKHand / IK UDP | `18002` | 右 SG100 手指关节控制 |
| 右手 SendDof / DoF UDP | `18002` | 右手指尖位姿和手势识别 |
| 左手 IKHand / IK UDP | `18003` | 左 SG100 手指关节控制 |
| 左手 SendDof / DoF UDP | `18003` | 左手指尖位姿和手势识别 |

**同一只手的 IK 和 DoF 共用端口是正确配置。** Linux 接收器通过报文内容区分两种数据。当前入口不使用旧的 `18012/18013` 端口。

- 只有 IKHand：手指可能能动，但无法完成手势识别。
- 只有 SendDof：可能能识别手势，但没有完整手指关节控制输入。
- 当前 A/B/C/D/E 模块依次对应拇指、食指、中指、无名指、小指，F 为手腕。更换模块排列后必须重新核对，不能照搬该顺序。
- Windows 模式不在机器人端搜索手套串口，不需要机器人安装 Linux 手套 SDK。手套采集及接收板连接由 Windows 软件侧完成。

## 4. 首次启动与手势操作

### 4.1 每次启动遥操进程后的第一次跟随

1. **进入交互模式**：双手分别做“拇指捏小指”，同时保持约 **2 秒**。
2. 两手松开刚才的捏合，再做下一手势。在交互模式中，SG100 手指保持进入前的目标，不再跟随手套动作。
3. **准备机器人**：右手拇指捏食指，相当于按 **A**，保持到机器人准备完成，再松开。不要只快速点一下。
4. **首次开启跟随**：双手握拳，相当于同时扣动两个前扳机，等待真正进入跟随。
5. 跟随成功后自动退出交互模式，手指恢复映射；之后按正常动作遥操。

双拳启动仅在当前遥操进程第一次成功开启时使用。如果启动失败，两手松开后可重试；第一次已经成功后，恢复跟随使用 **X+A** 或对应的保持恢复流程，不再重复双拳启动。

可以用掌心相对的自然前伸姿势稳定等待输入，但不需要为了启动强行把手臂完全伸直。终端打印“识别到双拳”不等于执行已确认；要同时确认自动退出交互以及跟随状态。

### 4.2 基础按键映射

除进入 / 退出交互的手势外，以下按键合成只在**交互模式内**生效：

| 手套动作 | 等效按键 |
| --- | --- |
| 左手拇指捏食指 | X |
| 左手拇指捏中指 | Y |
| 右手拇指捏食指 | A |
| 右手拇指捏中指 | B |
| 双手握拳 | 双前扳机，仅用于首次成功启动 |
| 双手各自拇指捏小指约 2 秒 | 进入 / 退出交互模式 |

再次切换交互前先松开上一轮捏合。手势还需要新鲜的 Quest 控制器输入；只有手套 DoF 数据或识别日志，而没有 Quest 输入，不能保证机器人响应。

### 4.3 跟随中的切换、保持与停机

使用下面组合前，先双手拇指捏小指进入交互，再松开：

| 操作 | 手套动作 | 机器人行为 |
| --- | --- | --- |
| X+A | 左拇指捏食指，同时右拇指捏食指 | 跟随时返回准备姿态；准备完成后再次使用，恢复跟随；固定保持时使用，则返回准备姿态 |
| X+B | 左拇指捏食指，同时右拇指捏中指 | 已成功启动的跟随与固定保持之间切换；不是首次启动，也不是停机 |
| X+Y | 左拇指同时捏食指和中指，右手 A/B 松开，连续保持至少 0.2 秒 | 发送软件停机信号，相关控制节点进入停止/退出流程 |

做 X+A / X+B 时，左手只捏食指，中指松开；组合完成后左右手都松开，再发下一次请求。若左手仍维持 X+Y，右手 A/B 松开后会重新计停机保持时间。

**进入交互只冻结手指，不会自动暂停手臂。** 需要手臂固定保持时使用 X+B；需要返回准备姿态时使用 X+A。恢复真实跟随后自动退出交互，手指恢复控制。

X+Y 注意事项：

- 必须处于交互模式，且右手不要同时触发 A 或 B。
- 右手手套或物理手柄的 A/B 按下时，会抑制手套 X+Y，以避免将 X+A/X+B 误判为停机；松开右手后重新计算 0.2 秒。
- 物理手柄完整按下 X+Y 不受这段手套保持时间约束。
- 这不是 Linux 关机或切断机器人供电。正常停机后，再按现场流程退出剩余终端和断电；紧急情况使用现场物理急停。

### 4.4 单手模式

在遥操启动命令中追加一个参数：

```text
qianshi_hand_side:=left
```

或：

```text
qianshi_hand_side:=right
```

只接收所选手套并控制对应灵巧手；Quest 的双臂输入链仍然存在。单手模式不启动双手手势识别，使用物理 Quest 手柄进行 A 准备、双前扳机首次启动和后续组合键操作。

## 5. 常用参数速查

除注明外，以下参数加在遥操 launch 命令末尾。

| 参数 | 本流程默认值 | 何时修改 |
| --- | --- | --- |
| `ip_address` | 必填 | Quest 的实际 IP |
| `glove_ik_freshness_timeout` | `0.5` 秒 | Windows 专用入口的每手 IK 新鲜度，允许 `(0, 0.5]`；超过时不恢复旧目标或刷新失联侧 |
| `robot_version` | 从环境读取；本文显式填 `400062` | 必须与运控机型一致 |
| `teleop_mode` | `absolute` | 本手册和当前启用的标定使用绝对式，不切换增量式 |
| `qianshi_hand_side` | `both` | `left` / `right` 单手接收 |
| `qianshi_glove_gesture_enable` | `true` | 设 `false` 改用物理手柄按键 |
| `glove_gesture_calibration_enable` | `true` | 默认自动加载个人手势阈值；`false` 恢复原 launch 阈值 |
| `glove_gesture_calibration_file` | `src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/active.yaml` | 选择另一配置或跨执行账号加载；使用绝对路径 |
| `qianshi_udp_bind_address` | `0.0.0.0` | 通常不用修改 |
| `qianshi_udp_right_port` | `18002` | 与 Windows 右手两路目标端口一致 |
| `qianshi_udp_left_port` | `18003` | 与 Windows 左手两路目标端口一致 |
| `qianshi_udp_diagnostics` | `false` | 设 `true` 查看两种报文和指尖模块诊断 |
| `qianshi_publish_real` | `true` | 设 `false` 关闭手套到 SG100 的真机手指命令；不等于关闭手臂遥操 |
| `control_torso` | `true` | 设 `false` 初始固定躯干 |
| `enable_head_control` | `true` | 设 `false` 关闭头部跟随 |
| `quest_controller_calibration_file` | `wheel_arm_abs_vel/config/quest_controller_calibration.yaml` | 选择另一份全身标定配置 |
| `quest_left_palm_to_controller_offset` / `quest_right_palm_to_controller_offset` | `[]` | 空列表继承标定；显式 `[0,0,0]` 仅关闭该侧平移，保留姿态校正 |
| `base_locked` | `true`，属于**运控**参数 | 需要底盘移动时在运控命令设 `false` |
| `ee_type` | 自动识别；本文运控显式 `heiman` | 本机使用 SG100 |

左右偏移向量是局部坐标中的三维量，不要把 15 cm 直接填成世界坐标的 `[0,0,0.15]`。

## 6. Linux SDK 与自动搜索接收板（可选）

只有把手套接收板连接到机器人、改走 Linux SDK 时才使用本节。Windows UDP 正常工作时无需执行串口识别。

两只手套共用 **一个 USB 接收板、一个串口**。默认左手数据帧 ID 为 `12`，右手为 `2`；不是两块接收板。

替换终端二的命令：

```bash
bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh \
  wheel_arm_abs_vel quest_velocity_ik.launch \
  robot_version:=400062 ip_address:=192.168.5.31 \
  hand_control_source:=qianshi qianshi_backend:=linux_sdk \
  teleop_mode:=absolute \
  qianshi_hand_side:=both qianshi_auto_detect:=true \
  qianshi_left_id:=12 qianshi_right_id:=2
```

此通用入口在手套模式下默认增量式，因此这里必须显式写 `teleop_mode:=absolute`，才能沿用当前全身标定。

| 搜索参数 | 默认值 / 用法 |
| --- | --- |
| `qianshi_auto_detect` | `true`，枚举候选并核对有效帧 ID |
| `qianshi_hand_side` | `both`；单手只要求相应 ID |
| `qianshi_left_id` / `qianshi_right_id` | `12` / `2`；设备 ID 不同才修改 |
| `qianshi_baud` | `921600` |
| `qianshi_right_port` | 留空自动找；指定时作为共享接收板路径，不表示只供右手使用 |
| `qianshi_detected_ports` | 留空；需要限制范围时填写逗号分隔的候选串口路径 |
| `qianshi_sdk_python` | 留空自动查找 SDK 环境；已有独立环境时指定其 Python 路径 |

双手模式必须在同一串口收到左右两个 ID 的有效帧才通过，不能因发现一个串口就认为双手接入成功。自动识别会避开已识别的机器人手端、电源板、遥控器等系统串口。

固定接收板路径时追加：

```text
qianshi_auto_detect:=false qianshi_right_port:=/dev/sg100_glove_receiver
```

路径必须真实存在，固定路径仍会核对目标 ID。SDK 使用独立 Python 环境；首次安装参见仓库内 `src/manipulation_nodes/sg100_glove_bridge/README.md` 的 SDK 环境章节。

## 7. Quest 末端安装：录包与重新标定

### 7.1 当前已生效的配置

配置文件：

```text
src/wheel_arm_abs_vel/config/quest_controller_calibration.yaml
```

当前保存的是 `400062 + SG100` 对应的全身标定，左右距离 **0.15 m**，来自指定先验，姿态来源于已经通过检查的“掌心相对、掌心向下”数据。**安装不变时日常启动直接加载，不必每次重新标定。**

更换安装位置、角度或松动后应重新录制。普通 wheel 的旧标定文件不能直接用于这个全身入口；当前启用标定也不适用于增量模式。

### 7.2 随运控录制：推荐日常流程

1. 按第 2 节启动运控和 Quest 遥操，使头显已经持续输出姿态。
2. 不必进入手臂跟随状态即可录制。保持安装不变、双臂前伸、手指朝前、手腕不折弯。
3. **双手掌心相对，稳定保持至少 15 秒。**
4. 转为 **双手掌心向下，稳定保持至少 15 秒。**
5. 按正常流程结束本次录制，并等待 `.bag.active` 封包成 `.bag` 后再计算。

运控自带录包会记录所需话题，文件默认在运行账号的 `~/.ros`，不是工作区的 `./ros`。切换运行账号时注意 `~` 的实际位置；当前机器 `/home/lab/.ros` 可访问这组录包。

本安装掌心向上容易遮挡手柄，因此推荐这两段流程。只有向上追踪确实可靠时，才改录“向上 → 相对 → 向下”，并在计算时声明三姿态顺序。

录包包括原始 VR 骨骼 `/leju_quest_bone_poses_raw`、处理后骨骼、末端目标与实际 FK、标定状态、手套手势和合成按键。只有运控而没有 Quest 接收节点，不会有有效标定骨骼。

### 7.3 独立录制：不启动运控

先结束其他 Quest 接收器，加载本工作区环境后执行：

```bash
bash src/wheel_arm_abs_vel/scripts/record_quest_calibration.sh 192.168.5.31
```

该入口只启动 VR 接收和 rosbag，使用独立 ROS master `11329`，不启动运控、IK 或手套驱动。按第 7.2 节做两段动作，完成后 Ctrl+C，等待正常封包。文件仍位于当前账号 `~/.ros`。

若提示已有 Quest 接收器，不要再启动第二份；使用已有运控录包或先结束原接收任务。

### 7.4 读取最新完整 bag，沿用 15 cm 并应用

在机器人工作区根目录、已经 source 环境的终端执行：

```bash
python3 src/manipulation_nodes/noitom_hi5_hand_udp_python/scripts/calibrate_quest_controller.py \
  --pose-order facing,down \
  --eef-convention sg100_400062 \
  --model src/kuavo_assets/models/biped_s400062/urdf/biped_s400062.urdf \
  --calibrate-orientation \
  --outward-distance 0.15 \
  --output /tmp/quest_controller_calibration.candidate.yaml \
  --report /tmp/quest_mount_report.json \
  --apply
```

- **没有填写 bag 文件名**：默认读取 `~/.ros` 最新已完成的一次录制，并合并其完整分片，分片文件名中的时间戳可以不同。
- `--pose-order facing,down` 明确本次记录的姿态顺序，不会让程序猜掌心朝向。
- `--outward-distance 0.15` 表示本次继续使用指定的 15 cm，不表示程序从录包计算出了该距离。
- 去掉 `--apply` 只生成候选和报告，不修改启动使用的配置。
- 通过质量检查后，脚本自动备份并更新配置；**下次重启遥操后加载，不会热更新当前进程。**

如果录包由其他账号生成，可追加 `--bag-dir /home/lab/.ros`。显式传一个 `.bag` 文件只会读这一文件，不会自动加上兄弟分片；常规使用保持默认目录选择即可。

### 7.5 距离未知、姿态不足或分段歧义

**距离未知：**去掉 `--outward-distance 0.15`，保留 `--calibrate-orientation`。结果仅校正姿态、不施加平移。若同时使用 `--apply`，会关闭旧配置中的 15 cm 平移补偿，**不会自动继承旧距离**。

当前翻掌 bag 不能可靠独立确定任意三维安装距离：翻掌过程的手腕平移与安装偏置会混淆。要测距离，需要明确的固定参考点/测量约束；要恢复一般三维偏移，还需要非平行旋转轴的数据。不能把拟合出一个数字当作已测得真值。

**三姿态录制：**使用 `--pose-order up,facing,down`，按向上、相对、向下顺序各保持至少 15 秒。若向上段追踪不可靠，不强行纳入。

**有多段候选或从三姿态中排除向上段：**先看报告中的时间窗，再用 `--intervals` 指定相对整次录制开始的秒数：

```text
--pose-order facing,down --intervals 相对段起始秒:相对段结束秒 向下段起始秒:向下段结束秒
```

这些时间要按本次 bag 填写，不复用上一次的秒数。两个区间必须依次排列、互不重叠，且各自至少有 10 秒合格数据。

双手都需要通过追踪、连续性、稳定性和跨姿态一致性检查。脚本会搜索稳定子窗口，但不会跨无效追踪或断流拼接。`--min-hold 3` 只用于诊断，不能用于应用配置。记录期间尽量保持身体朝向和手的位置稳定。

### 7.6 确认加载与回退

重启遥操后，在另一个已 source 的终端只读查看：

```bash
rosparam get /quest3/controller_calibration/enabled
rosparam get /quest3/controller_calibration/metadata/distance_source
rostopic echo -n 1 /quest3/controller_calibration_status
rostopic echo -n 1 /quest3/controller_calibration_mapper_status
```

当前 15 cm 配置还可查询：

```bash
rosparam get /quest3/controller_calibration/metadata/known_distance_m
```

应为 `0.15`。仅姿态配置会省略这个数值参数，并在状态中标记平移未标定；不把零当成测量结果。要判断实际是否用了平移，查看接收端状态中的 `effective_palm_to_controller_offset`，因为手动 launch 参数可以覆盖配置。

临时关闭左右平移、保留姿态校正时，在遥操启动命令追加：

```text
quest_left_palm_to_controller_offset:='[0,0,0]' quest_right_palm_to_controller_offset:='[0,0,0]'
```

全部回退可恢复自己选定的 `quest_controller_calibration.yaml.before-*` 备份，或把标定配置改成以下内容，再重启遥操：

```yaml
quest3:
  controller_calibration:
    enabled: false
```

## 8. 个人手势距离标定与下次启动自动应用

### 8.1 标定什么、什么时候生效

**标定通过后，默认自动应用到下一次遥操启动，不需要手动填阈值或修改日常启动命令。** 标定结果默认保存在当前仓库内，标定脚本和遥操启动统一从该目录保存、读取：

```text
src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/active.yaml
```

这里标定的是左右手各自的食指/中指/小指捏合，以及握拳的按下、释放距离，用于适配操作者与穿戴状态。第 7 节是 Quest 末端安装姿态/位置标定，两者独立；本节不会修改当前 15 cm 安装补偿，也不会改变手指关节映射或手势按键含义。

自动生效需要满足：

- 三轮采集完成并通过质量验证；没有指定 `--no-activate`。
- 下次启动的是包含本功能的全身 IK 工作区，双手手势识别开启。
- 使用默认保存目录，或通过 `glove_gesture_calibration_file` 指向实际文件。
- 标定和遥操使用同一份仓库；Windows UDP / Linux SDK 后端、五指模块映射或左右板卡 ID 与标定时一致。执行账号必须具有相应文件读写权限。

配置只在启动时读取，不会热更新已经运行的遥操。**标定通过后无需重新编译；重新启动遥操即可加载。** 未标定过时，默认文件缺失会沿用原 launch 阈值。

本功能目前实现于本地 `/home/chenzibo/kuavo_ws/kuavo-ros-control` 的 `czb/beta/glove-vel-ik` 分支；本次未部署机器人。机器人要使用它，需要先更新对应代码。个人配置现在位于本仓库内，但当前工作区通过本机 `.git/info/exclude` 忽略生成数据，**仅提交或拉取代码不会同步本地个人标定结果**；应在实际接收机上标定，或把经过验证且输入配置一致的文件复制过去并显式指定路径。

本机完整目录为：

```text
/home/chenzibo/kuavo_ws/kuavo-ros-control/src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/
```

目录中的 `active.yaml` 用于启动自动读取，`profiles/` 保存具名配置，`recordings/` 保存原始数据和断点，`reports/` 保存质量报告。以下涉及 `src/...` 的文件命令在仓库根目录执行。默认目录由包位置计算，无需每次传 `--directory`；文件夹内的 README 说明参与代码管理，当前工作区的个人生成数据由本地忽略规则排除，不纳入代码提交。

旧 `~/.ros/glove_gesture_calibration/` 不再自动读取。本次已复制旧的三份原始记录到新目录，原文件保留作备份；最新的 `张三_20261123_154123_481071.json` 包含 9/33 段，可用新路径断点继续。还没有通过完整标定生成的 `active.yaml`，它会在完整验证通过后自动创建。Quest 安装偏置的 bag 路径仍为 `~/.ros`，本次不改变。

### 8.2 开始标定（本地 Windows UDP）

先正常退出运控、遥操、MuJoCo 和其他手套接收程序；手套供电、Windows 发送软件保持运行。无需启动运控、Quest 应用或额外的 `roscore`。

```bash
cd /home/chenzibo/kuavo_ws/kuavo-ros-control
source /opt/ros/noetic/setup.bash
# 首次安装本功能或更新脚本安装配置后执行；以后每次标定不必再编译
catkin build sg100_glove_bridge
source devel/setup.bash
rosrun sg100_glove_bridge calibrate_glove_gestures.py --profile 张三
```

Windows 的目标 IP 填**当前执行标定的接收电脑地址**；左手目标端口 **18003**，右手 **18002**。同一只手 IKHand 与 SendDof 共用端口；标定必须收到双手 **SendDof**，只有 IKHand 不够。默认`模块顺序为拇指到小指 A B C D E`。若日常配置不同，用 `--left-port`、`--right-port`、`--left-module-map`、`--right-module-map` 设置同样的输入。

脚本使用独立本机 ROS master（11339），只启动采集后端，不发真机手指或遥操按键命令。遇到现有接收进程、端口占用或其他节点混入时会报错退出。

### 8.3 按提示做三轮动作

前两轮拟合，第三轮独立验证。每个动作前先按提示双手张开、按回车，记录至少 1 秒有效释放状态；再摆好目标动作、按回车，记录至少 4 秒有效保持数据。出现间隔超限时自动清空双手当前窗口并重新计时，整个步骤最多额外等待 10 秒，始终保持到完成提示。每轮按提示依次做：

| 动作 | 要点 |
| --- | --- |
| 双手张开、自然放松、四指伸直并拢 | 分别采集；并拢时拇指分开，作为非目标动作 |
| 双手分别拇指捏食指 | X+A，其余手指松开 |
| 双手分别拇指捏中指 | 食指、小指松开 |
| 双手分别拇指捏小指 | 交互手势，食指、中指松开 |
| 双手握拳 | 验证首次启动用的双拳识别 |
| 仅左手握拳、仅右手握拳 | 分别采集，另一只手张开，检查单拳误触 |
| X+B | 左拇指捏食指，右拇指捏中指 |
| X+Y | 左拇指同时捏食指和中指，左无名指/小指伸开，右手张开 |

这段流程在独立采集环境中进行，X+Y 不会发送停机命令。短暂间隔超限会自动丢弃当前双手窗口并重新采集，保持当前动作直到完成提示。持续异常达到总时限后可手动重试；目标重试复用已合格的释放段，无需反复张手。输入 `q` 或 Ctrl+C 会保存已完成动作并退出。需要更长保持可加 `--hold-seconds 6`。当前标定要求双手，不能只录单手。

采集结束后自动计算左右独立阈值，检查目标与非目标距离能否分开，并回放真实识别逻辑检查误触、释放和断流。如果失败，会输出报告位置，**不替换旧 active**。重新穿戴、检查映射并按报告重录，不要把“采集结束”当成“标定通过”。

Linux SDK 使用同一流程、单个共享接收板，改用：

```bash
rosrun sg100_glove_bridge calibrate_glove_gestures.py --profile 张三 \
  --backend linux_sdk --serial-port auto --left-id 12 --right-id 2
```

### 8.4 下次启动与确认加载

成功时终端会显示“标定通过”和“下次遥操将自动加载”的完整路径。之后按第 2 节正常启动运控和遥操，无需增加标定参数；仍按第 4 节进入交互、A 准备、双拳首次开启跟随。

遥操启动日志应显示 `已加载个人手势标定 张三`。在同一 ROS 环境执行：

```bash
rostopic echo -n 1 /glove_gesture/calibration_status
```

该话题是包含 JSON 文本的 String 消息，检查：

| 字段 | 预期含义 |
| --- | --- |
| `profile_loaded: true` | 已加载个人配置 |
| `profile_name` | 本次操作者名字 |
| `source` | 实际加载的 YAML 完整路径 |
| `thresholds` | 左右手本次生效的按下/释放阈值 |
| `reason: default_profile_missing` | 默认文件不存在，当前沿用 launch 阈值 |
| `reason: calibration_disabled` | 显式关闭了个人标定加载 |

默认路径根据 `sg100_glove_bridge` 包的位置确定，与当前终端目录及 HOME 无关。使用 `sudo -s` 后只要 source 的仍是本仓库，就读取同一个文件；需注意文件权限。若要显式选择文件，可在第 2.3 节遥操命令末尾追加：

```text
glove_gesture_calibration_file:=/home/chenzibo/kuavo_ws/kuavo-ros-control/src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/active.yaml
```

该例路径用于本地；在机器人上应填写机器人实际存在的文件路径。

### 8.5 多人切换、暂不应用与回退

```bash
# 列出已保存的个人配置
rosrun sg100_glove_bridge calibrate_glove_gestures.py --list
# 选择张三，下次启动自动加载；不会改变当前运行的遥操
rosrun sg100_glove_bridge calibrate_glove_gestures.py --activate 张三
# 为李四采集并保存，暂不替换默认配置
rosrun sg100_glove_bridge calibrate_glove_gestures.py --profile 李四 --no-activate
```

保存目录中，`profiles/姓名.yaml` 为具名配置，`recordings/*.json` 为原始采集，`reports/*.json` 为验证结果；替换已有 YAML 前保存 `原文件名.before-时间戳` 备份。指定 `--directory` 只改变保存目录，不会改变日常 launch 的默认读取路径，需同时用上述 launch 参数指向新的 `active.yaml`。

临时恢复原 launch 距离阈值，可在遥操命令末尾追加：

```text
glove_gesture_calibration_enable:=false
```

这只关闭个人配置加载，不关闭手势识别。现存配置损坏或输入身份不匹配时，手势节点会报错并使所属 launch 退出；选择匹配的个人配置或重新标定后再启动，不会静默忽略坏配置。

更多端口、模块映射、离线重算和独立 Linux 入口参数见[个人手势标定详细说明](../src/manipulation_nodes/sg100_glove_bridge/docs/personal_gesture_calibration.md)。普通遥操 bag 仍用于第 7 节的 Quest 安装标定；本节用带动作标签的采集 JSON。

### 8.6 标定中断后继续（断点恢复）

每完成一个目标动作，立即保存原始采集 JSON，终端显示已保存的 `N/33` 和文件路径；手动退出也保留已完成动作。网络抖动时，脚本只丢弃当前不连续窗口，不删除已保存阶段；并且总等待时间固定，不会无限自动重来。目标动作默认有效保持 4 秒、总等待上限约 14 秒，释放段约 11 秒。

继续时使用终端打印的实际 JSON 文件路径：

```bash
rosrun sg100_glove_bridge calibrate_glove_gestures.py --profile 张三 \
  --resume-recording src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/recordings/张三_实际时间.json
```

恢复前严格检查操作者、后端/映射/ID、已有数据质量及连续阶段顺序，从下一未完成动作继续；保留原记录，写入新的记录文件。Linux SDK 恢复需带上原来的 `--backend linux_sdk` 和对应板卡参数。恢复时保持同一操作者、手套穿戴和模块安装；若重新穿戴或调整模块，应重新标定，不能拼接旧数据。所有 33 段完整验证通过后才自动更新 active。`--from-recording` 用于完整数据离线重算，不能代替断点继续。

本机实测 Windows Wi-Fi 发送存在源帧连续但到达批量延迟的情况。建议发送端改用有线与接收端做对照；自动寻找连续窗口只改善标定流程，不会消除实际遥操链路的网络延迟。

## 9. 运行检查与常见问题

以下为只读检查；在遥操运行、同一 ROS 环境的终端执行，`rostopic hz` 用 Ctrl+C 结束：

```bash
rostopic hz /leju_quest_bone_poses_raw
rostopic hz /sg100_sim_hand_command
rostopic hz /sg100_sim_left_hand_command
rostopic hz /qianshi/glove_right/fingertips
rostopic hz /qianshi/glove_left/fingertips
rostopic echo -n 1 /glove_gesture/interaction_mode
rostopic echo -n 1 /glove_gesture/teleop_active
# 以下话题需要已更新到个人手势标定版本，且双手手势节点运行
rostopic echo -n 1 /glove_gesture/calibration_status
```

状态的含义：

- `/glove_gesture/interaction_mode=true` 表示交互开启。
- `/glove_gesture/teleop_active=true` 表示真实跟随已经确认。
- 跟随中刚进入交互时，两者可以同时为 true，因为交互只冻结手指。
- X+A 返回准备或 X+B 固定保持后，跟随状态为 false；恢复跟随为 true 时自动退出交互。

首次成功还可看到日志：`[FixedStart] First arm transition completed; live IK following confirmed`。停机链路收到命令时可看到：`[IkRos] 收到停止机器人信号，正在退出程序...`。

| 现象 | 优先检查 |
| --- | --- |
| 手指能动，但没有手势 | Windows 是否两侧都发送 SendDof；左右 fingertips 是否持续更新；是否使用带默认五指映射的 `quest_windows_udp.launch` |
| 标定通过但下次未加载 | 查看 calibration_status；确认 source 的仓库、文件路径和权限、手势开启、未加 --no-activate；本地文件不会随 Git 自动同步到机器人 |
| 手势配置损坏或后端/模块映射/ID 不匹配 | 选择当前输入对应的配置或重新标定；查看启动报错，不要只重复重启 |
| 标定 master 端口占用 | 新版不会把正常退出后的 TCP TIME_WAIT 当作已有 master；若确有监听进程，正常退出旧标定，或使用空闲的 `--master-port 11340`，不改变 UDP 端口 |
| 标定一直提示重录 | 查看左右帧数/Hz/有效跨度/最大到达间隔；新版标定保留批次内全部真实 DoF，要求至少 20 Hz、到达间隔不超过 0.3 秒，最多额外等待 10 秒。旧进程需退出重启；持续超限检查网络和发送端 |
| 标定报告不通过 | 检查双手 DoF、模块映射、动作与穿戴；失败保留旧 active，按报告重录 |
| 手势有识别日志，但机器人没响应 | 是否已进入交互；Quest 骨骼/手柄输入是否新鲜；首次是否先 A 准备再双拳；是否真的获得跟随确认 |
| 进入交互后手指不动 | 这是冻结手指的预期行为；退出交互或成功恢复跟随后才恢复手指控制 |
| 两手都没数据 | Windows 目的 IP 应为机器人；端口为右 18002、左 18003；检查软件输出与网络 |
| 只有一只手工作 | 检查 hand_side、对应侧 IK/DoF 两路、左右端口；Linux SDK 还要检查目标手套 ID |
| X+Y 不停机 | 先进入交互；右手 A/B 都松开；左 X+Y 持续至少 0.2 秒；确认 Quest 输入正常 |
| 双拳在第二次不再启动 | 首次成功后使用 X+A 恢复；X+B 只用于已有会话的保持/恢复 |
| 末端姿态或位置不符 | 检查安装是否变化、当前标定是否加载、是否被显式偏移参数覆盖；15 cm 仅为当前指定值 |
| 掌心向上定位漂移 | 不把遮挡数据用于标定；固定偏置不能修复动态丢跟踪，需调整可见性或安装后重新标定 |
| 人伸直，机器人仍略弯 | Quest 推断肘点不等于真实人体肘；检查目标与实际 FK，不能直接放大臂长或取消限制 |
| 找不到包 / launch | 确认在 `kuavo-ros-control` 正确分支，source 的是这个工作区，而非 wheel 仓库 |
| 独立录包没有数据 | Quest 应用和两个手柄是否在线；是否存在另一份接收器；只启动运控不能代替 Quest 接收 |

## 10. 更新后编译

已编译正常的机器人日常启动不必重复编译。更新了 C++、消息或构建配置后，在已安装依赖的工作区执行：

```bash
cd /home/lab/lihengyu/kuavo-ros-control
source /opt/ros/noetic/setup.bash
catkin build wheel_arm_abs_vel sg100_glove_bridge noitom_hi5_hand_udp_python -j 6
source devel/setup.bash
```

不要加 `--no-deps` 跳过需要的依赖。Linux SDK 的独立 Python 环境需要另行安装，catkin 不会替代 SDK 环境安装；Windows UDP 路径不需要该 SDK。

只修改标定 YAML 时，不需要重新编译 C++，重新启动遥操即可。若改的是 SG100 驱动增益或运控参数，则重启运控。

## 11. 对照来源

本手册按当前分支实际 launch、私有全身 FSM、手套输入合成逻辑和标定脚本核对；操作检查没有启动或移动机器人。原机器人流程基于提交 `339fb2a887`；个人手势标定章节依据本地 `czb/beta/glove-vel-ik` 新增实现核对，未声称机器人已部署该功能。

- [全身手套参数说明](../src/wheel_arm_abs_vel/docs/qianshi_glove.md)
- [Windows UDP 协议说明](../src/manipulation_nodes/sg100_glove_bridge/WINDOWS_UDP.md)
- [运控入口](../src/wheel_arm_abs_vel/launch/load_kuavo_real_velocity_ik.launch)
- [Windows 融合入口](../src/wheel_arm_abs_vel/launch/quest_windows_udp.launch)
- [手套按键合成](../src/wheel_arm_abs_vel/scripts/glove_quest_input.py)
- [全身私有状态机](../src/wheel_arm_abs_vel/src/QuestAbsoluteControlFSMNode.cpp)
- [个人手势标定说明](../src/manipulation_nodes/sg100_glove_bridge/docs/personal_gesture_calibration.md)
- [个人手势标定工具](../src/manipulation_nodes/sg100_glove_bridge/scripts/calibrate_glove_gestures.py)
- [Quest 安装标定脚本](../src/manipulation_nodes/noitom_hi5_hand_udp_python/scripts/calibrate_quest_controller.py)

当前新标定已完成离线质量与加载检查；更新安装补偿后的实际姿态、伸直程度仍以现场复测为准。


## Windows 专用入口的故障保护

当前维护入口为 `quest_windows_udp.launch`，只接受绝对式、Windows UDP 与手套输入组合。启动前检查机型和末端安装标定；随仓库的已启用配置属于 400062。其他机型需要通过 `quest_controller_calibration_file` 显式选择对应配置，未做安装标定时可使用含 `quest3: {controller_calibration: {enabled: false}}` 的独立 YAML。型号不匹配会在节点启动前报错，不自动套用 400062 的姿态/15 cm 偏移。

每只手独立检查 IK 更新时间。一侧失联后另一侧不能持续刷新其旧命令；退出交互只恢复仍新鲜的目标。单手握拳会屏蔽该侧食指/中指捏合位，以避免误成 X+Y；真正的 X+Y 仍需左无名指/小指伸开、右手张开，保持原来的持续时间要求。

这些保护由专用入口设置的参数与子进程环境共同启用。退出后即使 ROS master 留有参数，也不会让其他启动入口继承本次策略。按当前使用要求，不增加 UDP 来源校验。
