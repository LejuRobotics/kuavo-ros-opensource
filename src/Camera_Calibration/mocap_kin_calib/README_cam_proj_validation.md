# 头部相机 URDF 投影验证（S56 / S62 / S63）

`cam_proj_validation.py` 直接用 Python 运行，无需新增 catkin 编译。
订阅实际关节反馈、头部相机图像/内参/静态 TF，使用指定 URDF 做 FK，显示双臂网格轮廓。
它仅发布叠图，不发运动命令、不写零点、不替换运控模型。

投影模型默认按 `--robot-version` 选择（未指定版本时读取 `ROBOT_VERSION`，再默认 `56`）：
版本 `62`、`63` 均使用 `src/kuavo_assets/models/biped_s63/urdf/biped_s63.urdf`，
其他版本仍使用本目录的 `output/urdf_final.urdf`。显式传入 `--urdf` 时优先使用该候选模型。
模型路径选择不会改变反馈版本：例如 `63` 仍读取 `kuavo_v63/kuavo.json` 的关节配置。

`--robot-version` 决定如何读取关节反馈；`--urdf` 决定拿哪个模型画投影，两者是独立参数。
下面的启动示例省略 `--urdf`，以便按版本使用默认模型：

| 投影参数 | 实际加载的模型 |
| --- | --- |
| `--robot-version 56` | 本目录 `output/urdf_final.urdf` |
| `--robot-version 62` | `src/kuavo_assets/models/biped_s63/urdf/biped_s63.urdf` |
| `--robot-version 63` | `src/kuavo_assets/models/biped_s63/urdf/biped_s63.urdf` |
| `--robot-version 63 --urdf /实际路径/候选.urdf` | 显式指定的候选文件 |

因此，若保留旧命令中的 `--urdf .../output/urdf_final.urdf`，即使版本改成 `63`，
程序也仍加载那个文件；它不会把文件内容自动换成 S63。验证某次优化输出时，应显式填写那次的完整候选 URDF。


| 角色  | 机内有线地址 | 用户 / 工作空间 |
| --- | --- | --- |
| 下位机 / 运控 | `192.168.26.1` | `lab` / `/home/lab/guoqingxuan/kuavo-ros-control` |
| 上位机 / 相机 | `192.168.26.12` | `leju_kuavo` / `/home/leju_kuavo/kuavo_ros_application` |

### 终端 1：原有运控，按现场允许的标定流程启动

此命令会接管电机，只在现场允许运行机器人时使用；本次远程准备没有执行它。

```bash
ssh lab@10.10.31.33
cd /home/lab/guoqingxuan/kuavo-ros-control
source devel/setup.bash
export ROBOT_VERSION=56
roslaunch humanoid_controllers load_kuavo_real.launch
```

上面是 S56 示例。本仓库的 S62/S63 轮臂入口为：

```bash
export ROBOT_VERSION=63  # S62 则填 62
roslaunch humanoid_controllers load_kuavo_real_wheel.launch
```

本仓库的 `load_kuavo_real.launch` 默认 `robot_type=2`，不会仅因 `ROBOT_VERSION=62/63`
自动切换到轮臂控制器。现场若使用其他分支提供的统一入口，应核对该工作区实际的 launch；
`rospack find humanoid_controllers` 可查看当前 ROS 环境选中的包路径。
原运控已经运行并提供 `/sensors_data_raw` 时，直接复用，无需为投影重新启动。

不要与 `leju_hardware` 等另一套硬件控制重复启动。标定完成后继续保留此终端即可；
投影无需再次优化、写零，或将候选 URDF 替换进运控。

### 终端 2：仅启动现有头部相机驱动

```bash
ssh -J lab@10.10.31.33 leju_kuavo@192.168.26.12
cd /home/leju_kuavo/kuavo_ros_application
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.26.1:11311
export ROS_IP=192.168.26.12
rosparam set /head_camera/out/compressed/jpeg_quality 95
roslaunch orbbec_camera gemini_330_series.launch \
  camera_name:=head_camera serial_number:=CP2H16300043 \
  color_width:=1280 color_height:=720 color_fps:=15 \
  color_rotation:=180 enable_hardware_reset:=false
```

这是 `kuavo_camera cameras.launch` 原本包含的现成驱动入口。直接使用它可指定较低帧率，
不需要新写相机代码，也不启动腕相机、运动控制或 ARControl。保留默认深度配置以保持原相机 TF 约定。
序列号对应本次实机头部相机；换机器人时使用该机器自己的头部相机序列号。

现有 `roslaunch kuavo_camera cameras.launch has_head:=true head_camera_type:=orbbec_camera
has_left_wrist:=false has_right_wrist:=false rviz:=false` 同样兼容，但它将彩色帧率写死为 60。
1280×720 RGB 原始图像在 15 fps 约 41 MB/s，在 60 fps 约 166 MB/s；较低采集帧率能减轻网络压力。
当前实机 launch 已附带 `republish_color`，输出 `/head_camera/out/compressed`。
2026-10-08 已在上位机补装缺失的 `ros-noetic-compressed-image-transport`，并单独恢复了该转发节点；
没有重启相机或运控。下位机有线网口当天只协商到 100 Mbps，所以下文改用压缩话题。
JPEG 质量设为 95，图像仍是 1280×720，保留原 header，内参和像素坐标约定不变；
压缩是有损编码，保存的 `raw.png` 是解码后的图像，不等于原始 RGB 字节。

换上位机且缺插件时，仅需一次安装 `sudo apt-get install --no-install-recommends ros-noetic-compressed-image-transport`。
若驱动已在运行、先前的转发节点因缺插件退出，可在上位机同样的 ROS 环境中单独恢复它：

```bash
rosrun image_transport republish raw in:=color/image_raw compressed \
  __ns:=/head_camera __name:=republish_color
```

已有 `/head_camera/out/compressed` 发布者时不要重复执行。投影脚本已有压缩订阅支持，无需编译。

### 终端 3：投影窗口显示在当前电脑

```bash
ssh -XC lab@10.10.31.33
cd /home/lab/guoqingxuan/kuavo-ros-control
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.26.1:11311
export ROS_IP=192.168.26.1
python3 src/Camera_Calibration/mocap_kin_calib/cam_proj_validation.py \
  --robot-version 56 \
  --sync-mode receipt \
  --compressed \
  --image-topic /head_camera/out/compressed \
  --display-width 640 \
  --fps 15 \
  --mask-color 255 190 0
```

62/63 使用默认 S63 模型时，将上面的 `--robot-version 56` 改成实际反馈版本 `--robot-version 62` 或 `--robot-version 63`，不加 `--urdf`。
验证某轮优化结果时，再添加 `--urdf /那一轮实际的完整候选文件.urdf`。
不需要手填外参，不需要先做 head 棋盘标定。`DISPLAY` 由 SSH 自动设置，别手动改成 `:0`。
本机当前上下位机时钟未同步，因此上述命令显式使用接收时间匹配；请在每个 pose 停稳后观察和截图。
画面会标注 `RECEIPT / static poses`。该模式不保证运动中的图像与关节采样时刻一致，详见下文。

- 黄色填充/边界为双臂网格；蓝色叉号为 `zarm_l7_link` / `zarm_r7_link` 原点。
  S56 已过滤附带手掌、手指，仅保留至第 7 关节腕壳；S62/S63 的手部网格过滤尚未适配。
  所有版本都不使用向下延伸 0.17 m 的 `*_end_effector` 标记。
- `s` 保存，`q` / Esc 退出。无有效反馈或缺其他输入时显示原图与 `NO OVERLAY`。
- 脚本默认窗口宽度最多 960 像素；上述命令显式改为 640 以减轻 X11 流量。
  投影计算、保存和发布仍为原图尺寸。
- 保存路径：`output/cam_proj_validation/<时间>/`，含 `raw.png`、`overlay.png`、`metadata.json`。
  元数据记录 URDF SHA256、实际角度、图像/关节时间戳、内外参、安装变换来源、像素旋转及网格路径。
- 只发布叠图而不显示窗口时加 `--no-display`，输出 `/cam_proj_validation/image_overlay`。

### 终端 4：按需使用已有程序摆姿，会驱动关节

只在现场允许运动时使用。先让头部和手臂进入合适视野，停稳后看未参与拟合的 pose。
若需要低头，S56 头部 pitch 模型限位为 ±30°，不要直接发送 45°。现场确认后可使用已有接口：

```bash
# 在 79、source devel/setup.bash 后执行；单位为度。
rostopic pub -1 /robot_head_motion_data kuavo_msgs/robotHeadMotionData \
  '{joint_data: [0.0, 20.0]}'
```

手臂可沿用已有手动摆姿方法；也可复用原采集器的轨迹：

```bash
python3 src/Camera_Calibration/mocap_joint_calib/record_joint_poses.py \
  --config src/Camera_Calibration/mocap_kin_calib/config/calib_s56.yaml \
  --robot-version 56 --motion-only --no-capture-head
```

`--config` 应为此前实际使用的配置。`--motion-only` 会切换模式、对齐起点、走轨迹并回零，
不是只读；`--no-capture-head` 仅阻止该回放器下发头部轨迹。已有轨迹不保证每个 pose 都在视野内。

## 上机前只读自检与常见问题

在终端 3 的投影命令末尾加 `--check`，最多等待 8 秒后打印检查结果并退出，不弹窗口：

```bash
python3 src/Camera_Calibration/mocap_kin_calib/cam_proj_validation.py \
  --robot-version 56 \
  --sync-mode receipt \
  --compressed --image-topic /head_camera/out/compressed \
  --check
```

所有项 `OK` 时退出码为 0，否则为 2。检查项包括图像、内参、固定变换、关节反馈、时间匹配和像素旋转。
只开相机时，关节反馈/时间匹配未通过是预期结果，不会因此启动电机。

`--sync-mode receipt` 在投影进程内以两路消息的单调接收时刻匹配，仍要求差值不超过
`--max-skew 0.1` 秒，且两路数据均新鲜；不会调整系统时钟、重写消息或放宽到几十秒。
画面和保存的 metadata 会保留原始 header 差、接收差及数据年龄。
`--check` 的 `receipt_match: OK` 仅表示接收时间匹配可用，`capture_time_alignment`
仍是 `unverified_receipt`；不能据此判断采集时钟已同步。此模式只用于停稳 pose 验证。
两台机完成时间同步后，可改用 `--sync-mode header`（脚本默认值）按原始采集时间戳匹配。

| 提示 | 检查内容 |
| --- | --- |
| Waiting for fresh camera images | 上位机驱动、ROS master/IP、所选图像话题；上述命令使用 `/head_camera/out/compressed` |
| Waiting for joint feedback | 原运控是否发布 `/sensors_data_raw`，不是 `/sensor_data_raw` |
| joint_q must contain 29 / 20 finite radians | S56 应收到完整 29 项，S62/S63 应收到完整 20 项；不要自行截取双臂/头部数组 |
| No fixed /tf_static path | 是否收到头部驱动静态 TF、frame 名是否与默认 head_camera 一致 |
| Image/joint stamp mismatch | 检查两台机时钟；停稳 pose 可显式用 `--sync-mode receipt`，不要直接放大允许时差 |
| Image/joint receipt mismatch / Waiting for fresh joint feedback | 检查数据是否持续到达、传输积压；接收时间匹配也保留 0.1 秒差值与新鲜度限制 |
| Camera anchor absent | 候选 URDF 是否保留完整头部链；纯 arm-only 模型不能直接使用 |
| No desktop display | 从当前桌面使用 `ssh -XC`，或用 `--no-display` |

队列只保留最新图像，网格与畸变映射缓存一次。`--fps` 限制处理频率，不会降低驱动发送频率；
因此采集帧率在终端 2 设置。降低 `--display-width` 可减轻显示流量，不改变投影/截图分辨率。

## 计算、复用与固定变换来源

S56 `/sensors_data_raw` 的 `joint_data.joint_q` 是 29 个弧度值，S62/S63 是 20 个弧度值。脚本复用
`mocap_joint_calib/write_zero.py` 的纯关节映射函数、`plot_board_error_from_csv.py` 的 FK 基础函数，
以及棋盘记录器的 CameraInfo 读取函数；不会调用这些模块的运动或写零入口。

| 关节 | S56 零基索引 | S62/S63 零基索引 |
| --- | --- | --- |
| 左臂 7 轴 | `[13:20]` | `[4:11]` |
| 右臂 7 轴 | `[20:27]` | `[11:18]` |
| 头部 yaw / pitch | `[27]` / `[28]` | `[18]` / `[19]` |
| 腰部 yaw | `[12]` | 不参与投影 FK |

S56 保持从 `base_link` 计算；S62/S63 从双臂与头部共同的 `waist_yaw_link` 计算。
轮臂仍校验完整 20 项反馈，但投影 FK 仅需要双臂 14 轴和头部 2 轴，
无需补齐 `knee_joint`、`leg_joint`、`waist_pitch_joint`、`waist_yaw_joint`。
这四轴位于共同根节点之前，其变换在相机与手臂的相对位姿中抵消，没有将它们假设为零。
metadata 中 `fk_root` 表示 FK 根坐标系，`T_root_optical` 表示该根到光学系的变换；
仅 S56 等以 `base_link` 为根时额外保留原有 `T_base_optical` 字段。
FK 每关节使用
`origin @ Rot(axis, q_measured)`，两侧手臂和头部均从候选 URDF 计算，不消费旧模型的动态 `/tf`。

当前投影仍统一使用 S56 实机的 `+3.7°` 头部 yaw 补偿；这不是 S62/S63 的标定结果，
轮臂做精度判断前需按本机零位确认该补偿。关节映射修复不改变这项补偿。

固定链由三段自动组成：

```text
head_camera_base --候选 URDF 固定偏移--> head_camera_depth
                 --安装连接约定--> head_camera_link
                 --驱动 /tf_static--> head_camera_color_optical_frame
```

上位机现有 `dynamic_biped/launch/kuavo5_sensor_robot_enable.launch:50` 将
`head_camera_depth` 与相机驱动根坐标系 `camera_link` 按单位变换连接。
Orbbec 驱动使用 `camera_name:=head_camera` 时，驱动根坐标系名为 `head_camera_link`。
轮臂的独立依据见仓库 [搬箱子案例使用教程：TF 树报错解决](../../demo/pickbox/搬箱子案例使用教程.md#七-tf-树报错解决)，
其中明确配置 `0 0 0 0 0 0 head_camera_depth head_camera_link`。
脚本在内部为 S56/S62/S63 复用这一名义安装约定，不额外发布 TF，也不启动整套含 ARControl 的 launch。
若实际已有安装 TF，优先采用它；候选 URDF 的固定几何仍使用候选值。
单位变换只连接上述两个安装锚点，不会清零 URDF 中的相机安装偏移；
驱动到彩色光学系的变换仍读取本机 `/tf_static`，缺失时继续报错。
该约定不是本机实测外参，保存的 `extrinsic_source` 会标明 `nominal`。

```text
T_root_optical = FK_candidate(head_camera_base) × T_anchor_optical
p_optical = inverse(T_root_optical) × FK_candidate(arm_link) × p_mesh_in_link
```

网格应用各自 `scale` 和 `visual origin`，按相机内参/畸变投到图像中。25.2 万真实三角面
中的附带手部已去除，当前约 23.7 万三角面生成双臂覆盖 mask，保留凹形和孔洞，不使用凸包替代。
没有对躯干或现实物体做深度遮挡；
双臂重叠处显示覆盖区域，不区分前后手臂。

S56 的 `l_hand_pitch_nohand.STL` / `r_hand_pitch_nohand.STL` 实际包含手掌和手指，
不能仅凭 nohand 文件名判断几何范围。投影现在仅选择 `zarm_[lr]1_link` 到 `zarm_[lr]7_link`，
并对这两个已核验 SHA256 的 S56 网格保留腕壳的完整 CAD 连通零件，去除其下方手部零件。
原 STL、候选 URDF 和 FK 均不改；网格若更换版本，会提示重新核验，避免套用错误分界。
保存的 metadata 包含 `mesh_filters` 和 `endpoint_frames`，可追溯显示范围。

本次 `output/urdf_final.urdf` 相对标准 `kuavo_assets/models/biped_s56/urdf/biped_s56.urdf`
仅 14 个手臂关节 origin 不同，保留头部与相机安装链。头部/相机参数没有参与手臂优化。
`mocap_kin_calib/biped_v3_arm.urdf` 则是纯臂模型；不同 Drake full 模型的头部几何也不能直接混用。
未来优化输出是否完整应核对优化日志 `model URDF` 和实际 link，不能只根据文件名判断。

### 已验证的图像旋转约定

本机 Gemini335L（SN `CP2H16300043`、FW `1.4.60`、SDK `1.10.22`）使用 MJPG→ROS rgb8。
2026-09-30 显式设置、回读 SDK 的 0°/180°，再分别启动 ROS 驱动并采图：两组 ROS 图像同向，
K、D、TF 完全相同；图像直接相关系数约 0.970，额外旋转 180° 后约 0.003。

因此默认 `--image-rotation 0`，**不根据驱动 `color_rotation=180` 再翻转投影**。
若将来更换固件/驱动路径并确认图像确实额外旋转，可显式用 `--image-rotation 180`；
它只旋投影像素和 TCP 标记，保留输入图像、FK、K/D 不变。
证据保存在 `output/cam_proj_readiness_20260930/`，包括两组实际图像和 `verified_projection_policy.json`。

### 是否需要额外棋盘标定

无需先做 head 棋盘标定才能使用上述名义安装链。现有
`demos/kuavo_head_demo/kuavo_head_chessboard_calibration.yaml` 只优化两个头关节零位，
相机安装六项全部固定；它不会自动求出未知安装外参。
[头部相机投影计算报告](https://bcn9fa1lvktb.feishu.cn/wiki/SOxcwbfY5iOsbhk5Awoc8oMbnce)
使用已有 TF 链并手工补过 28°，不能直接将该补偿套到 S56。

投影仍包含名义安装/头部零位的不确定性。对比原始与优化 URDF 时保持相同的相机参数与头部零位，
在多个未拟合 pose 停稳观察；整体偏差不能直接归因于手臂优化。
