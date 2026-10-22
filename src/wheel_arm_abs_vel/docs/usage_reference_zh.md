# 绝对式轮臂 IK：参数、控制逻辑与维护参考

> 2026-10-22 更新：本文对应 Draft MR !4139 候选分支，源码基线 `b26d5d18e`（包含躯干启动授权撤销修复）。日常启动见[快速使用文档](usage_quickstart_zh.md)；历史 QA 记录见[QA 更新清单](qa_updates_20261022.md)。

## 1. 版本与范围

本文描述 `codex/abs-pico-startup-20261021` 的独立绝对式链路，仍为草稿，尚未合入 dev。第8—11节按当前候选源码刷新；参数表列出顶层声明，内联参数摘录不展开 include，也不替代运行时命名空间和条件。增量式入口、参数和控制语义不由本文替换。

### 1.1 部署位置与容器边界

真机在机器人宿主机直接安装依赖、编译和启动，不使用容器。电脑端仿真在 Docker 容器内完成拉取、依赖安装、编译、控制器与输入启动；宿主机只准备 Docker、图形授权和进入容器。完整执行顺序见第一份文档；检查、自动选型和验证边界见本文件第12—15节。

默认容器为 Ubuntu 20.04，使用普通 sim 用户和容器内 sudo。Linux host 网络用于头显 UDP 广播，不代表网络隔离；private IPC 隔离共享内存，控制器与输入必须进入同一个容器。源码/build/devel 在 home volume，系统依赖在容器可写层；stop/start 保留，删除容器后不能仅靠 volume 恢复依赖。软件渲染通过 X11 显示在电脑桌面，无机器人设备映射。

第一份完整展示拉取、依赖、构建、Docker和启动命令；不调用文档配套辅助脚本。复制整个代码块执行，不使用额外外层圆括号。

## 2. 运行链路

Quest/Pico 数据 → 输入有效性与手柄状态 → 绝对坐标映射 → Pinocchio 速度 IK → 正式参考与所有权 → 执行约束 → WBC/RNEA → 仿真或硬件反馈。末端夹合走独立输出路径，因此“夹爪可动”不能证明手臂已经取得跟随权限。

正式参考含底盘 3 维、腰部 4 维、双臂 14 维；固定底盘时仍要区分整体参考与 18 维腰臂控制。默认共享内存键为 344421/344422，控制器和输入必须一致。先启动控制器，再启动一种输入。同一 IPC 命名空间不能同时运行多套使用相同键的链路。

## 3. 输入切换、HOLD 与恢复

手臂模式为固定 0、复位 1、跟随 2。X+A 在 1 与 2 间切换，0 下允许切回复位；X+B 只在 2 与 0 间切换，1 下禁止。输入恢复、连续性恢复、操作确认和执行权取得是不同条件。`confirm_ready` 表示具备确认条件，不表示确认已经被接受。长时间丢失可以锁存 HOLD；不能仅靠新帧到来判断已经重新跟随。

默认连续性时间为 grace 60 ms、brake 150 ms、lost 500 ms；恢复窗口 100 ms 且至少 3 帧。Quest/Pico 默认 hold_on_input_timeout=true；具体有效行为还受状态机和有效性条件影响。绝对任务误差驱动速度参考，参考积分与平滑不等于输入映射就是纯增量。本版本已合入 X+B 恢复时保留身体标定与映射锚点的修正；首次跟随或显式重新标定仍需建立对应映射。恢复时参考从当前状态平滑输出，不能据此误判为输入重新采用纯增量映射。

## 4. Pico 与官方文档对应关系

参考同版本仓库的 `src/manipulation_nodes/pico-body-tracking-server/README.md`。官方页面：[Pico 全身遥操作](https://www.lejuhub.com/highlydynamic/kuavo-ros-control/-/tree/dev/src/manipulation_nodes/pico-body-tracking-server)。

两条链路共享追踪器佩戴、LejuVRTeleopRobot、全身校准、服务器选择与连接步骤。官方入口使用其专属 Pico 状态机；绝对入口使用 `monitor_pico.py` 将 full_body 与手柄数据转换为 Quest 兼容话题，不执行官方 `vr_command`。因此启动顺序可对齐，解锁和控制范围按键不能直接搬用。

绝对接收器需要完整骨骼（包含 SPINE3），不能只提供 upper_body。其输出包括 `/leju_quest_bone_poses`、`/quest_joystick_data`，连接状态写入 ROS 参数 `/quest3/connected`。Pico primary/secondary 映射为 X/Y/A/B，grip/trigger 数值保留，协议未提供的触摸状态为 false。Quest/Pico 输入端 `enable_head_control` 均默认 false，机器端独立头控默认 `vr_follow`；显式 `track_hand` 才跟手。Pico 的 Neck/Head 映射到输入槽 24/25。

## 5. 关键参数速查

| 参数 | 默认及含义 |
| --- | --- |
| robot_version | 读取启动进程ROBOT_VERSION；显式robot_version优先，资产与配置按机型选择；当前候选支持范围见第13.3节 |
| ee_type | auto，读取实际机型 kuavo.json 的 EndEffectorType |
| dynamics_backend | 实机和仿真入口均为 rnea |
| rnea_mode | full |
| base_locked | 实机 true；设为 false 后，在初始化、反馈、使能和摇杆回中条件满足时可驾驶，不依赖手臂 ACTIVE |
| reference_transport | shm；上下游必须一致 |
| reference_shm_key / owner_shm_key | 344421 / 344422 |
| require_ik_hard_isolation | false；根据机器平台与实际 CPU 配置检查 |
| visualize_humanoid | 仿真入口默认 false；第一份文档显式 true |
| hold_on_input_timeout | Quest/Pico 均为 true |
| enable_terminal_start | 输入端 false；默认在第一个控制器终端按 o；true 仅开启输入端备用确认 |
| enable_head_control | false；头控由独立head_control_node负责 |
| hand_position_cutoff_hz / hand_orientation_cutoff_hz | 10.0 / 8.0 Hz；实际配置同时受 filter_tuning_enabled 控制 |
| Pico robot_ip / broadcast_ip | 默认均留空；接收器内置多网卡广播，App 搜索 KUAVO；robot_ip/broadcast_ip 成对覆盖 |
| Quest ip_address | 本文必须填写Quest头显IPv4地址，直接连接；不得留空进入自动发现 |

## 6. 参数生效与维护

参数表按文件列出，同名项可能出现在多个入口，不能把某文件中的参数直接传给另一个未声明它的入口。`value` 声明通常是内部固定值，不等价于可自由覆盖的 `default` 参数。YAML 原值不一定是运行最终值，例如 operating_temporal 配置可能被 launch 覆盖。调参前停止运行、保存配置及日志，再核对 rosparam 中的最终值。

平台绑核由 `platform_roslaunch.sh` → `runtime_partition.py` 与运行脚本按实际 CPU 拓扑选择。通用 x86 默认配置同样包含固定核号且要求内核隔离，并不是自动适配所有桌面。普通桌面可视仿真使用源码已有的 `KUAVO_RUNTIME_PARTITION=0`；该选项不作为实机部署办法。不要把 AAEON 或 AGX 的 CPU 编号照搬到其他电脑；预检失败应检查拓扑和预留核，不能为通过启动任意解除保护。

绝对式内置调度配置为 IK FIFO 70、Mapper FIFO 35 / nice -10；实际落地受权限和运行环境影响。共享启动器对已识别的增量式 Quest 入口保持 dev 的 IK FIFO 40、Mapper nice -5，不注入绝对式 Drake 路径或执行绝对式机型解析。显式 `KUAVO_RUNTIME_PROFILE` 仍优先；无需在日常命令重复设置这些默认值。

## 7. 常见问题

| 现象 | 检查顺序 |
| --- | --- |
| 首次资产库编译提示没有目标 | 先编译 kuavo_msgs 完成 Catkin prebuild，再执行资产指定目标 |
| pip 无法卸载 PyYAML | 使用第一份文档第3.2节的固定pip和--ignore-installed安装块，随后 pip check |
| 只有夹合，手臂 HOLD | 检查身体骨骼有效性、freshness、模式、confirm_ready、确认事件和 owner |
| 输入恢复仍 HOLD | 查看是否长丢失锁存、稳定恢复窗口和重新确认；不要只观察 connected |
| Pico 发现不到服务器 | 检查电脑网卡、同一广播域、AP 客户端隔离、UDP 8443 广播、UDP 12345 接收及重复接收器 |
| 仿真没有窗口 | 检查图形会话 DISPLAY、XAUTHORITY、OpenGL；不能用 headless 结果替代可视验收 |
| 找到旧工作区节点 | 检查 rospack 路径及预检 LINK_PATH_PASS，使用干净 shell 和正确 devel/setup.bash |

### 7.1 躯干与按键

绝对式通过胸部追踪控制躯干高度、俯仰和转腰，默认映射比例分别为 0.5、1.0、0.7。启动后首次有效跟随默认开启躯干控制；Quest 触摸 X+Y 后按 B、Pico RT+Y 可关闭／重开躯干，双臂继续跟随。X+B 仅保持／恢复双臂关节，不撤销躯干权限。X+A 复位、故障或输入失效撤销躯干权限，恢复后需用躯干组合键重新开启；默认开启只在本次 Mapper 启动后的首次跟随应用一次。base_locked 仍只锁底盘。enable_body_mirroring=false 关闭胸部映射，不替代躯干权限开关。

Pico 单独长按 X/A 至少 0.5 秒切换左右拇指，扳机须均小于 0.5，且已收到控制器 observation；X+A/X+B/X+Y 组合优先。原始触摸字段仍为 false，不支持裸手和腰部触摸组合替代。默认在第一个控制器终端按 `o`；输入终端辅助监听默认关闭，仅 `enable_terminal_start:=true` 时启用，与长按 A 调用同一初始化服务。无 TTY 时该辅助节点提示退出。

### 7.2 权限撤销与首次默认开启

首次有效跟随开启后立即消费默认开启标记。`reset()` 同时关闭躯干并取消尚未消费的标记；X+A 准备、停止、故障、无效手柄输入、已建立输入流过期都会走撤销路径，发生在首次跟随前也不例外。恢复 ready 或重新请求双臂跟随不重新授予躯干权限，必须用 Quest/Pico 躯干组合键开启。

启动时尚无首包、初始化和首次跟随握手尚未完成属于等待，保持关闭但保留一次默认开启。它与真正的 reset 分开处理。X+B 单独固定双臂仍保留躯干权限和独立底盘通道；保持的是双臂关节，躯干移动时手臂随躯干整体移动。

### 7.3 底盘、头控与 Pico 查找

底盘不动先检查 `base_locked`、初始化、底盘硬件急停/ON、本地使能、摇杆回中、输入和反馈时效，再对照 `[BaseGate]` 原因。此前现场确认为急停/ON 状态，不据此断言定位丢失。

默认 `head_control_mode:=vr_follow` 跟随 VR 头部；`track_hand` 为显式选择。Chest→Head 相对姿态受上游骨骼影响的现象仍未关闭。

Pico App 继续通过“查找机器人”发现 `KUAVO`：输入进程在未收到头显姿态前即启动发现线程，按网卡广播机器人 IP 和末端类型（UDP 8443），约每秒一次；UDP 12345 接收线程独立工作。移出接收线程不取消自动发现，也不要求改用手填 IP。实际无线网络和 App 兼容性仍需真头显复测。

## 8. 全部 launch 参数声明

### 8.1 bypass_communication_profile.launch

| 参数 | 声明类型 | 源码值 | 说明 |
| --- | --- | --- | --- |
| full_rate_audit | default | false |  |
| second_round_enabled | default | true |  |

### 8.2 control.launch

| 参数 | 声明类型 | 源码值 | 说明 |
| --- | --- | --- | --- |
| require_ik_hard_isolation | default | $(optenv KUAVO_REQUIRE_IK_HARD_ISOLATION false) |  |
| arm_driver_reserved_cpus | default | $(optenv KUAVO_ARM_DRIVER_RESERVED_CPUS 7) |  |
| reference_transport | default | shm |  |
| reference_shm_key | default | 344421 |  |
| owner_shm_key | default | 344422 |  |
| controller_type | default | legacy |  |
| real | default | true |  |
| robot_version | default | $(eval optenv('ROBOT_VERSION', 'UNCONFIGURED_ROBOT_VERSION' if arg('controller_type') == 'wheel_arm_abs_vel' else '40')) |  |
| dynamics_backend | default | $(eval 'rnea' if arg('real') else 'wbc') |  |

### 8.3 dynamics_backend.launch

| 参数 | 声明类型 | 源码值 | 说明 |
| --- | --- | --- | --- |
| filter_tuning_enabled | default | $(optenv KUAVO_ABS_FILTER_TUNING true) |  |
| dynamics_backend | default | wbc |  |
| rnea_mode | default | full |  |
| rnea_shadow | default | false |  |

### 8.4 j3_overshoot_profile.launch

| 参数 | 声明类型 | 源码值 | 说明 |
| --- | --- | --- | --- |
| enabled | default | false |  |
| kd | default | 18.0 |  |

### 8.5 load_kuavo_absolute_sim_base.launch

| 参数 | 声明类型 | 源码值 | 说明 |
| --- | --- | --- | --- |
| filter_tuning_enabled | default | $(optenv KUAVO_ABS_FILTER_TUNING true) |  |
| require_ik_hard_isolation | default | $(optenv KUAVO_REQUIRE_IK_HARD_ISOLATION false) |  |
| arm_driver_reserved_cpus | default | $(optenv KUAVO_ARM_DRIVER_RESERVED_CPUS 7) |  |
| reference_transport | default | shm |  |
| reference_shm_key | default | 344421 |  |
| owner_shm_key | default | 344422 |  |
| simulation_motor_servo | default | true |  |
| dynamics_backend | default | wbc |  |
| rnea_mode | default | full |  |
| rnea_shadow | default | false |  |
| output_system_info | default | true |  |
| with_monitor | default | true |  |
| with_arm_trajectory | default | true |  |
| robot_type | default | 1 |  |
| nodelet_robot_type | default | 1 |  |
| wheel_control_type | default | 1 |  |
| use_sqp | default | true | Use SQP if true, DDP if false |
| rviz | default | true |  |
| multiplot | default | false |  |
| robot_version | default | $(optenv ROBOT_VERSION UNCONFIGURED_ROBOT_VERSION) |  |
| use_vr_control | default | true |  |
| use_whole_body_ik_reference | default | true |  |
| initialize_whole_body_ik_natural_pose | default | $(arg use_whole_body_ik_reference) |  |
| urdfFile | value | $(find kuavo_assets)/models/biped_s$(arg robot_version)/urdf/biped_s$(arg robot_version).urdf |  |
| taskFile | value | $(find humanoid_wheel_interface)/config/kuavo_s$(arg robot_version)/task.info |  |
| libFolder | value | /var/ocs2/kuavo_s$(arg robot_version) |  |
| use_external_mpc | default | false |  |
| wbc_frequency | default | 500 |  |
| sensor_frequency | default | 500 |  |
| use_joystick | default | true |  |
| with_estimation | default | true |  |
| use_joint_filter | default | false |  |
| estimator_type | default | 1 |  |
| wbc_only | default | false |  |
| play_back | default | false |  |
| real | default | false |  |
| joystick_type | default | sim |  |
| sim_input_mode | default | keyboard |  |
| arm_move_spd | default | 1.2 |  |
| start_way | default | manual |  |
| use_estimator_contact | default | false |  |
| visualize_humanoid | default | true |  |
| coredump | default | false |  |
| with_coredump | value | --with-coredump |  |
| with_coredump | value |  |  |
| with_ik | default | $(eval int(arg('robot_version')) != 63) |  |
| with_mm_ik | default | false |  |
| use_shm_communication | value | false |  |
| teach_pendant | default | 0 |  |
| pure_sim | default | true |  |
| absolute_launch_contract | value | $(eval {'supported': 'wheel_arm_abs_vel_s62'}['supported' if (int(arg('robot_version')) in (62, 63, 400062) and (int(arg('robot_version')) != 63 or not arg('with_ik')) and int(arg('robot_type')) == 1 and int(arg('nodelet_robot_type')) == 1 and int(arg('wheel_control_type')) != 0 and int(arg('estimator_type')) == 1 and arg('use_whole_body_ik_reference') and not arg('use_external_mpc') and not arg('real')) else 'Private absolute sim requires S62/S63/400062 (S63 with_ik=false) wheel mode, measured estimator, formal reference and no external MPC']) |  |
| head_control_mode | default | vr_follow | vr_follow / track_hand / fixed |
| head_track_target | default | both_hands_midpoint | head_control_mode:=track_hand 时：both_hands_midpoint / left_hand / right_hand / active_hand |

### 8.6 load_kuavo_mujoco_velocity_ik.launch

| 参数 | 声明类型 | 源码值 | 说明 |
| --- | --- | --- | --- |
| filter_tuning_enabled | default | $(optenv KUAVO_ABS_FILTER_TUNING true) |  |
| require_ik_hard_isolation | default | $(optenv KUAVO_REQUIRE_IK_HARD_ISOLATION false) |  |
| arm_driver_reserved_cpus | default | $(optenv KUAVO_ARM_DRIVER_RESERVED_CPUS 7) |  |
| reference_transport | default | shm |  |
| reference_shm_key | default | 344421 |  |
| owner_shm_key | default | 344422 |  |
| head_control_mode | default | vr_follow | vr_follow / track_hand / fixed |
| head_track_target | default | both_hands_midpoint | head_control_mode:=track_hand 时：both_hands_midpoint / left_hand / right_hand / active_hand |
| robot_version | default | $(optenv ROBOT_VERSION UNCONFIGURED_ROBOT_VERSION) |  |
| simulation_motor_servo | default | true |  |
| dynamics_backend | default | rnea |  |
| rnea_mode | default | full |  |
| visualize_humanoid | default | false |  |

### 8.7 load_kuavo_real_velocity_ik.launch

| 参数 | 声明类型 | 源码值 | 说明 |
| --- | --- | --- | --- |
| filter_tuning_enabled | default | $(optenv KUAVO_ABS_FILTER_TUNING true) |  |
| ros_ip | default | 192.168.26.1 | ROS address reachable by chassis; auto preserves caller environment |
| require_ik_hard_isolation | default | $(optenv KUAVO_REQUIRE_IK_HARD_ISOLATION false) |  |
| arm_driver_reserved_cpus | default | $(optenv KUAVO_ARM_DRIVER_RESERVED_CPUS 7) |  |
| j3_overshoot_profile | default | true |  |
| j3_kd | default | 18.0 |  |
| communication_full_rate_audit | default | false |  |
| communication_second_round_enabled | default | true |  |
| reference_transport | default | shm |  |
| reference_shm_key | default | 344421 |  |
| owner_shm_key | default | 344422 |  |
| base_locked | default | true | False enables joystick chassis commands in initialized idle hold or with a healthy active absolute owner |
| arm_inertia_feedforward_scale | default | 1.0 |  |
| dynamics_backend | default | rnea |  |
| rnea_mode | default | full |  |
| rnea_shadow | default | false |  |
| real | default | true |  |
| cali | default | false |  |
| cali_arm | default | false |  |
| cali_leg | default | false |  |
| output_system_info | default | true |  |
| vr_remote_control | default | true |  |
| robot_type | default | 1 |  |
| nodelet_robot_type | default | 1 |  |
| wheel_control_type | default | 1 |  |
| use_sqp | default | true | Compatibility argument; this fixed IK entry starts no MPC optimizer |
| rviz | default | false |  |
| multiplot | default | false |  |
| robot_version | default | $(optenv ROBOT_VERSION UNCONFIGURED_ROBOT_VERSION) |  |
| use_vr_control | default | true |  |
| urdfFile | value | $(find kuavo_assets)/models/biped_s$(arg robot_version)/urdf/biped_s$(arg robot_version).urdf |  |
| taskFile | value | $(find humanoid_wheel_interface)/config/kuavo_s$(arg robot_version)/task.info |  |
| libFolder | value | /var/ocs2/kuavo_s$(arg robot_version) |  |
| rvizconfig | default | $(find humanoid_wheel_interface_ros)/rviz/mobile_manipulator.rviz |  |
| use_external_mpc | default | false |  |
| wbc_frequency | default | 500 |  |
| sensor_frequency | default | 500 |  |
| use_joystick | default | false |  |
| with_estimation | default | true |  |
| use_joint_filter | default | false |  |
| estimator_type | default | 1 |  |
| wbc_only | default | false |  |
| play_back | default | false |  |
| joystick_type | default | bt2 |  |
| sim_input_mode | default | keyboard |  |
| arm_move_spd | default | 1.2 |  |
| start_way | default | manual |  |
| use_estimator_contact | default | false |  |
| visualize_humanoid | default | true |  |
| ruiwo_cxx_sdk | default | true |  |
| only_half_up_body | default | false |  |
| only_half_up_body | default | false |  |
| coredump | default | false |  |
| with_coredump | value | --with-coredump |  |
| with_coredump | value |  |  |
| with_ik | default | $(eval int(arg('robot_version')) != 63) |  |
| with_mm_ik | default | false |  |
| with_led | default | true |  |
| led_for_state | default | true | 是否启动启动状态灯光监控（启动中绿灯闪烁→准备站立绿灯常亮→站立完成白灯常亮→跌倒红灯闪烁） |
| use_external_odom_tf | default | $(arg real) |  |
| enable_base_emergency_stop | default | true |  |
| with_modbus_io_bridge | default | true |  |
| relay_port | default | /dev/kuavo_relay |  |
| pressure_port | default | /dev/kuavo_pressure |  |
| pressure_publish_rate | default | 1.0 |  |
| teach_pendant | default | 0 |  |
| lejuclaw_high_freq_switch | default | false | Use high-frequency continuous claw control by default; a live /quest_joystick_data stream also enables it. |
| lejuclaw_vr_stream_timeout_ms | default | 1000 | Timeout in ms before the Quest joystick stream is considered inactive. |
| use_wheel_bridge | default | true |  |
| use_wheel_bridge | default | false |  |
| music | default | enable |  |
| sg100_cmd_filter_alpha | default | 0.2 |  |
| use_custom_arm_prepare | default | false |  |
| absolute_launch_contract | value | $(eval {'supported': 'wheel_arm_abs_vel_s62'}['supported' if (int(arg('robot_version')) in (62, 63, 400062) and (int(arg('robot_version')) != 63 or not arg('with_ik')) and int(arg('robot_type')) == 1 and int(arg('nodelet_robot_type')) == 1 and int(arg('wheel_control_type')) != 0 and int(arg('estimator_type')) == 1 and not arg('use_external_mpc') and arg('real')) else 'Private absolute real requires S62/S63/400062 (S63 with_ik=false) wheel mode, measured estimator and no external MPC']) |  |
| head_control_mode | default | vr_follow | vr_follow / track_hand / fixed |
| head_track_target | default | both_hands_midpoint | head_control_mode:=track_hand 时：both_hands_midpoint / left_hand / right_hand / active_hand |

### 8.8 pico_input.launch

| 参数 | 声明类型 | 源码值 | 说明 |
| --- | --- | --- | --- |
| port | default | 12345 |  |
| peer_ip | default |  |  |
| advertise_ip | default |  |  |
| broadcast_ip | default |  |  |
| calibration_file | default | $(find wheel_arm_abs_vel)/config/pico_quest_input.yaml |  |
| enable_head_control | default | false |  |

### 8.9 pico_velocity_ik.launch

| 参数 | 声明类型 | 源码值 | 说明 |
| --- | --- | --- | --- |
| filter_tuning_enabled | default | $(optenv KUAVO_ABS_FILTER_TUNING true) |  |
| ip_address | default |  |  |
| robot_ip | default |  |  |
| broadcast_ip | default |  |  |
| require_ik_hard_isolation | default | $(optenv KUAVO_REQUIRE_IK_HARD_ISOLATION false) |  |
| arm_driver_reserved_cpus | default | $(optenv KUAVO_ARM_DRIVER_RESERVED_CPUS 7) |  |
| j3_overshoot_profile | default | true |  |
| j3_kd | default | 18.0 |  |
| communication_full_rate_audit | default | false |  |
| communication_second_round_enabled | default | true |  |
| legacy_pico_profile | default | false |  |
| reference_transport | default | shm |  |
| reference_shm_key | default | 344421 |  |
| owner_shm_key | default | 344422 |  |
| robot_version | default | $(optenv ROBOT_VERSION UNCONFIGURED_ROBOT_VERSION) |  |
| input_source | value | pico |  |
| hold_on_input_timeout | default | true |  |
| pico_port | default | 12345 |  |
| pico_peer_ip | default | $(arg ip_address) |  |
| pico_advertise_ip | default | $(arg robot_ip) |  |
| pico_broadcast_ip | default | $(arg broadcast_ip) |  |
| pico_calibration_file | default | $(find wheel_arm_abs_vel)/config/pico_quest_input.yaml |  |
| execution_governor_enabled | default | true |  |
| wrist_envelope_enabled | default | $(arg filter_tuning_enabled) |  |
| vr_kalman_flattening_enabled | default | false |  |
| vr_kalman_enabled | default | true |  |
| continuous_reference_enabled | default | true |  |
| hand_reference_frequency | default | 12.0 |  |
| orientation_reference_mode | default | fixed_bandwidth |  |
| hand_position_cutoff_hz | default | 10.0 |  |
| hand_orientation_cutoff_hz | default | 8.0 |  |
| joint_jerk_enabled | default | true |  |
| adaptive_reference_enabled | default | true |  |
| innovation_reference_enabled | default | true |  |
| torso_reference_frequency | default | 2.5 |  |
| operating_temporal_config | default | $(find wheel_arm_abs_vel)/config/operating_temporal_experiment.yaml |  |
| operating_envelope_enabled | default | true |  |
| temporal_cost_enabled | default | true |  |
| operating_diagnostics_enabled | default | true |  |
| velocity_tcp_geometry_file | default |  |  |
| enable_vr_ik_debug | default | false |  |
| wheel_ik | default | true |  |
| enable_ik_velocity_limit | default | false |  |
| enable_velocity_ik_takeover | default | true |  |
| whole_body_shadow_isolate_position_ik | default | false |  |
| enable_position_ik_status_log | default | false |  |
| velocity_ik_feasibility_mode | default | active |  |
| velocity_ik_nominal_arm_reach | default | 0.70937088 |  |
| velocity_ik_selected_reach_reserve | default | 0.03 |  |
| velocity_ik_joint_safe_range_ratios | default | [0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8] |  |
| velocity_ik_joint_safe_center | default | [0.2617993878, 0.0, 0.0, -0.5235987756, 0.0, 0.0, 0.0, 0.2617993878, 0.0, 0.0, -0.5235987756, 0.0, 0.0, 0.0] |  |
| tracking_source_mode | default | controller |  |
| publish_tracking_mapping_debug | default | $(arg enable_vr_ik_debug) |  |
| record_task07_debug_bag | default | false |  |
| task07_debug_bag_dir | default | $(env HOME)/.ros/task07 |  |
| version | default | 4 |  |
| ctrl_arm_idx | default | 2 |  |
| ik_type_idx | default | 0 |  |
| control_torso | default | true |  |
| dexhand_front_trigger | default | true | Both front triggers independently command six dexhand channels, 0=open and 100=closed. |
| control_finger_type | default | 0 |  |
| predict_gesture | default | false |  |
| ee_type | default | auto | Read EndEffectorType from this robot version kuavo.json; optional explicit override |
| sg100_enabled | default | true | Allow the shared end-effector resolver to start SG100 only for heiman; false explicitly disables it |
| enable_head_control | default | false |  |
| enable_terminal_start | default | false |  |
| hand_reference_mode | default | thumb_index |  |
| use_arm_collision | default | false |  |
| use_two_stage_ik | default | false |  |
| use_step_turning | default | false |  |
| use_cpp_monitor | default | false |  |
| use_cpp_incremental_ik | default | false |  |
| enable_vr_stop_robot | default | true |  |
| enable_absolute_target_alignment | default | $(arg wheel_ik) |  |
| enable_absolute_start_gap_check | default | false |  |
| absolute_enable_max_position_error | default | 0.30 |  |
| absolute_enable_max_orientation_error | default | 0.35 |  |
| absolute_enable_max_vr_age | default | 0.25 |  |
| absolute_enable_max_joint_state_age | default | 0.10 |  |
| absolute_enable_mode_timeout | default | 3.0 |  |
| absolute_enable_service_timeout | default | 2.0 |  |
| absolute_enable_exit_mode | default | 0 |  |
| enable_absolute_discontinuity_guard | default | true |  |
| absolute_max_tcp_position_step | default | 0.15 |  |
| absolute_max_tcp_orientation_step | default | 1.0472 |  |
| absolute_max_joint_step | default | 0.25 |  |
| absolute_ik_trust_region_step | default | 0.25 |  |
| absolute_hard_recovery_successes | default | 3 |  |
| absolute_hard_target_max_step | default | 0.25 |  |
| absolute_hard_recovery_max_offset | default | 0.25 |  |
| absolute_hard_recovery_stability_step | default | 0.05 |  |
| absolute_execution_max_joint_velocity | default | 1.5 |  |
| absolute_execution_max_joint_acceleration | default | 6.0 |  |
| absolute_execution_min_joint_limit_margin | default | 0.03 |  |
| tracking_rearm_stable_frames | default | 3 |  |
| single_hand_mode | default | false |  |
| use_arm_only_ik | default | false |  |
| init_control_waist_enable | default | true |  |
| enable_timestamp_log | default | false |  |
| use_pico | default | false |  |
| enable_videostream | default | false |  |
| camera_publisher_name | default | /camera/color |  |
| task_space_acc_limit | default | 500.0 |  |
| task_space_jerk_limit | default | 3000.0 |  |
| joint_space_acc_limit | default | 500.0 |  |
| joint_space_jerk_limit | default | 3000.0 |  |
| max_joint_velocity | default | 30.0 |  |
| delta_scale_x | default | 1.26 |  |
| delta_scale_y | default | 1.26 |  |
| delta_scale_z | default | 1.26 |  |
| delta_scale_roll | default | 1.0 |  |
| delta_scale_pitch | default | 1.0 |  |
| delta_scale_yaw | default | 1.0 |  |
| max_pos_diff | default | 0.45 |  |
| arm_move_threshold | default | 0.002 |  |
| reset_joint_to_default | default | true |  |
| pos_vel_limit | default | 30.0 |  |
| fhan_r | default | 3000.0 |  |
| fhan_kh0 | default | 3.0 |  |
| fhan_r_joint | default | 6000.0 |  |
| fhan_kh0_joint | default | 3.0 |  |
| ik_ros_fhan_r | default | 3000.0 |  |
| ik_ros_fhan_kh0 | default | 4.0 |  |
| ik_ros_fhan_r_joint | default | 3000.0 |  |
| ik_ros_fhan_kh0_joint | default | 4.0 |  |
| ik_ros_max_joint_velocity | default | 300.0 |  |
| left_hand_quat_offset_roll | default | 0.0 |  |
| left_hand_quat_offset_pitch | default | 0.0 |  |
| left_hand_quat_offset_yaw | default | 0.0 |  |
| right_hand_quat_offset_roll | default | 0.0 |  |
| right_hand_quat_offset_pitch | default | 0.0 |  |
| right_hand_quat_offset_yaw | default | 0.0 |  |
| sphere_radius_limit | default | 0.6 |  |
| min_reachable_distance | default | 0.08 |  |
| elbow_min_distance | default | 0.16 |  |
| elbow_max_distance | default | 0.24 |  |
| use_incremental_hand_orientation | default | false |  |
| limit_shoulder_pitch_upper | default | false |  |
| shoulder_pitch_upper_limit | default | 0.3 |  |
| box_min_bound | default | [0.00, -0.6, -0.19] |  |
| box_max_bound | default | [0.4, 0.6, 0.4] |  |
| chest_offset_y_ax | default | 0.05 |  |
| left_center | default | [0, 0.02, 0] |  |
| right_center | default | [0, -0.02, 0] |  |
| default_left_hand_pos_on_exit | default | [0.00, 0.293, -0.16] |  |
| default_right_hand_pos_on_exit | default | [0.00, -0.293, -0.16] |  |
| hand_changing_mode_threshold | default | 1.20 |  |
| zyx_limits_final | default | [1.4923, 1.4923, 1.4923] |  |
| zyx_limits_ee | default | [1.4923, 1.4923, 1.4923] |  |
| zyx_limits_link4 | default | [1.5708, 0.6, 0.6] |  |

### 8.10 quest_velocity_ik.launch

| 参数 | 声明类型 | 源码值 | 说明 |
| --- | --- | --- | --- |
| filter_tuning_enabled | default | $(optenv KUAVO_ABS_FILTER_TUNING true) |  |
| require_ik_hard_isolation | default | $(optenv KUAVO_REQUIRE_IK_HARD_ISOLATION false) |  |
| arm_driver_reserved_cpus | default | $(optenv KUAVO_ARM_DRIVER_RESERVED_CPUS 7) |  |
| j3_overshoot_profile | default | true |  |
| j3_kd | default | 18.0 |  |
| communication_full_rate_audit | default | false |  |
| communication_second_round_enabled | default | true |  |
| legacy_pico_profile | default | false |  |
| reference_transport | default | shm |  |
| reference_shm_key | default | 344421 |  |
| owner_shm_key | default | 344422 |  |
| robot_version | default | $(optenv ROBOT_VERSION UNCONFIGURED_ROBOT_VERSION) |  |
| input_source | default | quest |  |
| hold_on_input_timeout | default | true |  |
| pico_port | default | 12345 |  |
| pico_peer_ip | default |  |  |
| pico_advertise_ip | default |  |  |
| pico_broadcast_ip | default |  |  |
| pico_calibration_file | default | $(find wheel_arm_abs_vel)/config/pico_quest_input.yaml |  |
| execution_governor_enabled | default | true |  |
| wrist_envelope_enabled | default | $(arg filter_tuning_enabled) |  |
| vr_kalman_flattening_enabled | default | false |  |
| vr_kalman_enabled | default | true |  |
| continuous_reference_enabled | default | true |  |
| hand_reference_frequency | default | 12.0 |  |
| orientation_reference_mode | default | fixed_bandwidth |  |
| hand_position_cutoff_hz | default | 10.0 |  |
| hand_orientation_cutoff_hz | default | 8.0 |  |
| joint_jerk_enabled | default | true |  |
| adaptive_reference_enabled | default | true |  |
| innovation_reference_enabled | default | true |  |
| torso_reference_frequency | default | 2.5 |  |
| operating_temporal_config | default | $(find wheel_arm_abs_vel)/config/operating_temporal_experiment.yaml |  |
| operating_envelope_enabled | default | true |  |
| temporal_cost_enabled | default | true |  |
| operating_diagnostics_enabled | default | true |  |
| velocity_tcp_geometry_file | default |  |  |
| enable_vr_ik_debug | default | false |  |
| wheel_ik | default | true |  |
| enable_ik_velocity_limit | default | false |  |
| enable_velocity_ik_takeover | default | true |  |
| whole_body_shadow_isolate_position_ik | default | false |  |
| enable_position_ik_status_log | default | false |  |
| velocity_ik_feasibility_mode | default | active |  |
| velocity_ik_nominal_arm_reach | default | 0.70937088 |  |
| velocity_ik_selected_reach_reserve | default | 0.03 |  |
| velocity_ik_joint_safe_range_ratios | default | [0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8] |  |
| velocity_ik_joint_safe_center | default | [0.2617993878, 0.0, 0.0, -0.5235987756, 0.0, 0.0, 0.0, 0.2617993878, 0.0, 0.0, -0.5235987756, 0.0, 0.0, 0.0] |  |
| tracking_source_mode | default | controller |  |
| publish_tracking_mapping_debug | default | $(arg enable_vr_ik_debug) |  |
| record_task07_debug_bag | default | false |  |
| task07_debug_bag_dir | default | $(env HOME)/.ros/task07 |  |
| version | default | 4 |  |
| ctrl_arm_idx | default | 2 |  |
| ik_type_idx | default | 0 |  |
| ip_address | default |  |  |
| control_torso | default | true |  |
| dexhand_front_trigger | default | true | Both front triggers independently command six dexhand channels, 0=open and 100=closed. |
| control_finger_type | default | 0 |  |
| predict_gesture | default | false |  |
| ee_type | default | auto | Read EndEffectorType from this robot version kuavo.json; optional explicit override |
| sg100_enabled | default | true | Allow the shared end-effector resolver to start SG100 only for heiman; false explicitly disables it |
| enable_head_control | default | false |  |
| enable_terminal_start | default | false |  |
| hand_reference_mode | default | thumb_index |  |
| use_arm_collision | default | false |  |
| use_two_stage_ik | default | false |  |
| use_step_turning | default | false |  |
| use_cpp_monitor | default | false |  |
| use_cpp_incremental_ik | default | false |  |
| enable_vr_stop_robot | default | true |  |
| enable_absolute_target_alignment | default | $(arg wheel_ik) |  |
| enable_absolute_start_gap_check | default | false |  |
| absolute_enable_max_position_error | default | 0.30 |  |
| absolute_enable_max_orientation_error | default | 0.35 |  |
| absolute_enable_max_vr_age | default | 0.25 |  |
| absolute_enable_max_joint_state_age | default | 0.10 |  |
| absolute_enable_mode_timeout | default | 3.0 |  |
| absolute_enable_service_timeout | default | 2.0 |  |
| absolute_enable_exit_mode | default | 0 |  |
| enable_absolute_discontinuity_guard | default | true |  |
| absolute_max_tcp_position_step | default | 0.15 |  |
| absolute_max_tcp_orientation_step | default | 1.0472 |  |
| absolute_max_joint_step | default | 0.25 |  |
| absolute_ik_trust_region_step | default | 0.25 |  |
| absolute_hard_recovery_successes | default | 3 |  |
| absolute_hard_target_max_step | default | 0.25 |  |
| absolute_hard_recovery_max_offset | default | 0.25 |  |
| absolute_hard_recovery_stability_step | default | 0.05 |  |
| absolute_execution_max_joint_velocity | default | 1.5 |  |
| absolute_execution_max_joint_acceleration | default | 6.0 |  |
| absolute_execution_min_joint_limit_margin | default | 0.03 |  |
| tracking_rearm_stable_frames | default | 3 |  |
| single_hand_mode | default | false |  |
| use_arm_only_ik | default | false |  |
| init_control_waist_enable | default | true |  |
| enable_timestamp_log | default | false |  |
| use_pico | default | false |  |
| enable_videostream | default | false |  |
| camera_publisher_name | default | /camera/color |  |
| task_space_acc_limit | default | 500.0 |  |
| task_space_jerk_limit | default | 3000.0 |  |
| joint_space_acc_limit | default | 500.0 |  |
| joint_space_jerk_limit | default | 3000.0 |  |
| max_joint_velocity | default | 30.0 |  |
| delta_scale_x | default | 1.26 |  |
| delta_scale_y | default | 1.26 |  |
| delta_scale_z | default | 1.26 |  |
| delta_scale_roll | default | 1.0 |  |
| delta_scale_pitch | default | 1.0 |  |
| delta_scale_yaw | default | 1.0 |  |
| max_pos_diff | default | 0.45 |  |
| arm_move_threshold | default | 0.002 |  |
| reset_joint_to_default | default | true |  |
| pos_vel_limit | default | 30.0 |  |
| fhan_r | default | 3000.0 |  |
| fhan_kh0 | default | 3.0 |  |
| fhan_r_joint | default | 6000.0 |  |
| fhan_kh0_joint | default | 3.0 |  |
| ik_ros_fhan_r | default | 3000.0 |  |
| ik_ros_fhan_kh0 | default | 4.0 |  |
| ik_ros_fhan_r_joint | default | 3000.0 |  |
| ik_ros_fhan_kh0_joint | default | 4.0 |  |
| ik_ros_max_joint_velocity | default | 300.0 |  |
| left_hand_quat_offset_roll | default | 0.0 |  |
| left_hand_quat_offset_pitch | default | 0.0 |  |
| left_hand_quat_offset_yaw | default | 0.0 |  |
| right_hand_quat_offset_roll | default | 0.0 |  |
| right_hand_quat_offset_pitch | default | 0.0 |  |
| right_hand_quat_offset_yaw | default | 0.0 |  |
| sphere_radius_limit | default | 0.6 |  |
| min_reachable_distance | default | 0.08 |  |
| elbow_min_distance | default | 0.16 |  |
| elbow_max_distance | default | 0.24 |  |
| use_incremental_hand_orientation | default | false |  |
| limit_shoulder_pitch_upper | default | false |  |
| shoulder_pitch_upper_limit | default | 0.3 |  |
| box_min_bound | default | [0.00, -0.6, -0.19] |  |
| box_max_bound | default | [0.4, 0.6, 0.4] |  |
| chest_offset_y_ax | default | 0.05 |  |
| left_center | default | [0, 0.02, 0] |  |
| right_center | default | [0, -0.02, 0] |  |
| default_left_hand_pos_on_exit | default | [0.00, 0.293, -0.16] |  |
| default_right_hand_pos_on_exit | default | [0.00, -0.293, -0.16] |  |
| hand_changing_mode_threshold | default | 1.20 |  |
| zyx_limits_final | default | [1.4923, 1.4923, 1.4923] |  |
| zyx_limits_ee | default | [1.4923, 1.4923, 1.4923] |  |
| zyx_limits_link4 | default | [1.5708, 0.6, 0.6] |  |
| absolute_launch_contract | value | $(eval {'supported': 'wheel_arm_abs_vel_s62'}['supported' if (int(arg('robot_version')) in (62, 63, 400062) and arg('wheel_ik') and arg('enable_velocity_ik_takeover') and not arg('use_cpp_incremental_ik') and not arg('use_arm_only_ik') and not arg('legacy_pico_profile')) else 'Private absolute input requires S62/S63/400062 wheel_ik=true takeover=true; incremental/arm-only/legacy profiles are unsupported']) |  |

### 8.11 sg100_hand.launch

| 参数 | 声明类型 | 源码值 | 说明 |
| --- | --- | --- | --- |
| left_port | default | /dev/lefthand |  |
| right_port | default | /dev/righthand |  |
| baudrate | default | 1000000 |  |
| start_driver | default | true |  |
| start_mapper_bridge | default | false |  |

## 9. YAML 配置完整原文

### 9.1 execution_governor.yaml

```yaml
# Phase 1. Candidate defaults, NOT a validated hardware stability envelope.
# No change to maximum speed/acceleration or normal target feedforward.
# Left/right shoulders are resolved by joint NAME, not wire-array offsets.
execution_governor:
  enabled: true
  zarm_l3_joint:
    error_deadband: 0.20
    error_full: 0.50
    prediction_time: 0.04
    penalty_ratio: 4.0
    smoothing_ratio: 0.25
    attack_time: 0.01
    release_time: 0.04
  zarm_r3_joint:
    error_deadband: 0.20
    error_full: 0.50
    prediction_time: 0.04
    penalty_ratio: 4.0
    smoothing_ratio: 0.25
    attack_time: 0.01
    release_time: 0.04
```

### 9.2 fixed_base_18d.yaml

```yaml
# Shared by manual Quest and real-packet MuJoCo replay. Simulation validation
# only; this profile is NOT approval to use the controller on hardware.
enable_base_motion: false
enable_root_translation_intent: false
enable_root_yaw_intent: false
enable_continuous_recruitment: false
enable_natural_hang_bootstrap: false
enable_body_mirroring: true
body_tracking_frame: quest_tracking_robot_axes

# Legacy publishers retain differentiated FF. Estimated Quest packets use
# causal skeleton filtering plus consistent continuous task references.
continuous_reference_enabled: true
# Fixed -3dB hand cutoffs; innovation/direct_task remain explicit rollback modes.
orientation_reference_mode: fixed_bandwidth
hand_position_cutoff_hz: 10.0
hand_orientation_cutoff_hz: 8.0
hand_reference_frequency: 12.0
torso_reference_frequency: 2.5
# Rotation only; position and elbow retain hand_reference_frequency.
hand_orientation_reference_frequency: 6.0
# New continuous reference: fast motion plus bounded observation correction.
# Set enabled:false to recover the previous (46373205) reference generator.
innovation_reference:
  enabled: true
  position_frequency: 12.0
  position_damping: 1.0
  position_speed: 3.0
  position_acceleration: 36.0
  position_jerk: 3600.0
  orientation_frequency: 3.0 # independent correction ceiling; not a speed cap
  orientation_damping: 1.0
  orientation_speed: 9.0
  orientation_acceleration: 108.0
  orientation_jerk: 10800.0
# Latency alignment and time WITHOUT a new valid observation are separate.
reference_prediction:
  min_hold: 0.040
  max_hold: 0.060
  decay: 0.030
  max_alignment_age: 0.060
# Task-space bandwidth scheduling, not a deadband or joint speed limit.
# Position/elbow scales: m, m/s; orientation scales: rad, rad/s.
adaptive_reference:
  enabled: true
  min_frequency: 1.0
  speed_time_constant: 0.020
  attack_time_constant: 0.010
  release_time_constant: 0.200
  bumpless_enabled: true
  # Only acceleration changes induced by frequency/guide modulation;
  # not global Cartesian jerk limits or joint speed limits.
  position_parameter_jerk: 300.0
  orientation_parameter_jerk: 300.0
  active_parameter_jerk_multiplier: 8.0
  position_error_start: 0.004
  position_error_full: 0.012
  position_speed_start: 0.030
  position_speed_full: 0.120
  orientation_error_start: 0.00872664625997165
  orientation_error_full: 0.026179938779915
  orientation_speed_start: 0.05235987755983
  orientation_speed_full: 0.261799387799149
hand_position_gain: 12.0
hand_orientation_gain: 8.0
feedforward_filter_time_constant: 0.010
hand_position_weight: 12.0
hand_orientation_weight: 6.0
elbow_position_weight: 0.15
primary_regularization_weight: 0.0001
primary_regularization_max_weight: 0.002
torso_height_gain: 0.50
torso_pitch_gain: 1.0
torso_yaw_gain: 0.7
torso_height_tracking_gain: 4.0
torso_pitch_tracking_gain: 5.0
torso_yaw_tracking_gain: 8.0
torso_intent_weight: 5.0

# Separate operating ability, fixed drive cost and explicit low ready pose.
# A constrained human target remains visible as a residual, never redefined.
# Waist order: knee, leg, waist_pitch, waist_yaw.
lower_joint_effort_multiplier: [6.0, 3.0, 2.0, 1.0]
lower_joint_cost_velocity_scale: [1.0, 1.0, 1.0, 1.0]
lower_joint_velocity_cap: [0.30, 0.45, 0.80, 2.00]
lower_joint_acceleration_cap: [1.00, 1.50, 3.00, 8.00]
preparation_lower_position: [0.0, 0.0, 0.0, 0.0]
preparation_lower_position_tolerance: 0.008
preparation_lower_measured_position_tolerance: 0.015
preparation_velocity_tolerance: 0.030
preparation_settle_cycles: 50
# First acquisition only: independent of PREPARE completion and fault recovery.
startup_readiness:
  window: 0.080
  median_window: 0.010
  arm_position: 0.002
  lower_position: 0.001
  arm_speed: 0.050
  lower_speed: 0.030
  arm_raw_speed: 0.150
  lower_raw_speed: 0.100
  raw_dwell: 0.010
  extreme_speed: 0.300
# [x,z,pitch,yaw] offsets from FK of low ready q; m,m,rad,rad.
# Bending may lower Chest below ready height. Not a certified hardware envelope.
torso_workspace_lower_offset: [-0.18, -0.22, -0.5235987755982988, -1.5707963267948966]
torso_workspace_upper_offset: [0.40, 0.34, 0.8726646259971648, 1.5707963267948966]

# Optional absolute-only wrist commissioning profile. Disabling restores the
# joint_* limits loaded above; stopping and outward bounds stay consistent.
wrist_envelope:
  enabled: true
  j5: {velocity: 4.5, acceleration: 45.0, jerk: 4000.0}
  j6: {velocity: 5.5, acceleration: 55.0, jerk: 6000.0}
  j7: {velocity: 5.5, acceleration: 55.0, jerk: 6000.0}
```

### 9.3 j3_overshoot_profile.yaml

```yaml
# Startup-only S62/S63 candidate. Disable via j3_overshoot_profile:=false on BOTH launches.
# Braking is transmitted in WholeBodyIkReference; no independent owner constant.
kp: 250.0
j3_kd: 18.0
joint_dynamic_overrides:
  zarm_l3_joint: {velocity: 2.2, acceleration: 18.0, braking: 18.0, outward: 18.0, jerk: 2000.0}
  zarm_r3_joint: {velocity: 2.2, acceleration: 18.0, braking: 18.0, outward: 18.0, jerk: 2000.0}
```

### 9.4 operating_temporal_experiment.yaml

```yaml
# EXPERIMENT ONLY. Load into the IK node's PRIVATE namespace after
# fixed_base_18d.yaml. This file does not start a node or enable takeover.
# B0: both false; B1: envelope true; B2: temporal true; B3: both true.
# Always select before starting a stopped test. No ACTIVE parameter switching.
operating_envelope:
  enabled: false
  # Layout v1: base xyz(3), knee/leg/waist_pitch/yaw(4), left arm(7), right arm(7).
  # Extra comfort reserve = position [rad] + time [s] * |next dq|.
  extra_position: [0, 0, 0, 0.015, 0.015, 0.015, 0.02, 0.025, 0.025, 0.025, 0.025, 0.035, 0.035, 0.035, 0.025, 0.025, 0.025, 0.025, 0.035, 0.035, 0.035]
  extra_time: [0, 0, 0, 0.03, 0.03, 0.03, 0.03, 0.04, 0.04, 0.04, 0.04, 0.05, 0.05, 0.05, 0.04, 0.04, 0.04, 0.04, 0.05, 0.05, 0.05]
temporal_cost:
  enabled: false
  # Exploratory small weights, NOT hardware-approved tuning.
  acceleration_weight: [0, 0, 0, 0.000001, 0.000001, 0.000003, 0.00001, 0.001, 0.001, 0.001, 0.001, 0.0005, 0.0005, 0.0005, 0.001, 0.001, 0.001, 0.001, 0.0005, 0.0005, 0.0005]
  delta_acceleration_weight: [0, 0, 0, 0.000001, 0.000001, 0.000001, 0.000003, 0.0003, 0.0003, 0.0003, 0.0003, 0.0001, 0.0001, 0.0001, 0.0003, 0.0003, 0.0003, 0.0003, 0.0001, 0.0001, 0.0001]
  acceleration_scale: [1, 1, 1, 1, 1.5, 3, 8, 50, 50, 50, 50, 100, 100, 100, 50, 50, 50, 50, 100, 100, 100]
  # Delta acceleration scale [rad/s^2 per sample], NOT jerk [rad/s^3].
  delta_acceleration_scale: [1, 1, 1, 1, 1.5, 3, 8, 50, 50, 50, 50, 100, 100, 100, 50, 50, 50, 50, 100, 100, 100]
operating_diagnostics:
  enabled: true
  stride: 1  # 500 Hz needed for spectral analysis; monitor async drops.

# QP-internal discrete jerk bound; original safety bounds have priority.
joint_jerk:
  enabled: true
  limits:
    zarm_l1_joint: 25000.0
    zarm_l2_joint: 25000.0
    zarm_l3_joint: 25000.0
    zarm_l4_joint: 25000.0
    zarm_l5_joint: 50000.0
    zarm_l6_joint: 50000.0
    zarm_l7_joint: 50000.0
    zarm_r1_joint: 25000.0
    zarm_r2_joint: 25000.0
    zarm_r3_joint: 25000.0
    zarm_r4_joint: 25000.0
    zarm_r5_joint: 50000.0
    zarm_r6_joint: 50000.0
    zarm_r7_joint: 50000.0
```

### 9.5 pico_quest_input.yaml

```yaml
# PICO SDK full_body only. Feet, legs and appended contact-state item are ignored.
# Input contract: controller / LEGACY_PALM_6D. No hold or confidence gating.
# Local hand correction [x,y,z metres, roll,pitch,yaw radians], applied before
# inverse Quest palm axes. Zero is beta's hand-frame convention, not a measured
# Pico controller-to-Quest physical calibration.
left_hand_offset: [0, 0, 0, 0, 0, 0]
right_hand_offset: [0, 0, 0, 0, 0, 0]
```

### 9.6 rnea.yaml

```yaml
# Free-space feedforward, frozen base. These are simulation commissioning defaults,
# not motor current limits or permission to run a new robot without validation.
mode: full
acceleration_gain: 0.25
velocity_cutoff_hz: 60.0
sensor_timeout: 0.05
max_period: 0.05
dynamic_ramp_seconds: 0.2
torque_limit_ratio: 0.8  # fraction of task.info torqueLimitsTask, reserve PD margin
torque_rate_lower: 3000.0  # Nm/s (joint-side)
torque_rate_arm: 1000.0
acceleration_limit_lower: 20.0  # rad/s^2 AFTER acceleration_gain
acceleration_limit_arm: 100.0
jerk_limit_lower: 200.0  # rad/s^3, dynamic feedforward only
jerk_limit_arm: 500.0
# Optional 18-element overrides: torque_rate, acceleration_limit, jerk_limit.

# Canonical driven order: knee, leg, waist_pitch, waist_yaw,
# L1..L7, R1..R7. Values are AFTER acceleration_gain (0.25).
# Match IK 25000/25000/2000/25000/4000/6000/6000 times gain;
# J3 assumes the default j3_overshoot_profile. Explicit overrides remain allowed.
jerk_limit: [200.0, 200.0, 200.0, 200.0,
             6250.0, 6250.0, 500.0, 6250.0, 1000.0, 1500.0, 1500.0,
             6250.0, 6250.0, 500.0, 6250.0, 1000.0, 1500.0, 1500.0]
```

### 9.7 teleop_continuity.yaml

```yaml
# Stop robot before changing. No runtime parameter switching.
continuity:
  grace: 0.060
  brake: 0.150
  lost: 0.500
  recovery: 0.100
  recovery_frames: 3
acquisition:
  joint_speed: 0.80           # initial rad/s budget, released smoothly in 1 s
  linear_speed: 0.40         # m/s, catch-up FEEDBACK task request, NOT world TCP cap
  angular_speed: 1.20        # rad/s, catch-up FEEDBACK task request; live FF stays independent
  acceleration: 3.00        # initial rad/s^2 budget; hard-domain braking has priority
  position_tolerance: 0.03   # m; both reference and measured FK
  rotation_tolerance: 0.15   # rad
  settle: 0.10              # diagnostic initial convergence; does NOT gate release or remove feedback cap
  release: 1.0
```

### 9.8 vr_kalman.yaml

```yaml
# Fixed-noise, variable-dt skeleton CV. No static-intent detector.
# q: continuous white-acceleration spectral density, NOT acceleration caps.
# Light input profile: Q x approximately (1 / 0.75)^4. This shortens the
# locally linear CV observer pole-decay time by about 25% at 72/120 Hz.
# Pose and velocity remain one joint estimate; R, resets, and downstream
# reference / feedforward filters and all control limits are unchanged.
vr_kalman_enabled: true
vr_kalman_position_sigma: 0.001  # m
vr_kalman_position_q: 0.63      # m^2/s^3; previous 0.2
vr_kalman_rotation_sigma: 0.003 # rad
vr_kalman_rotation_q: 5.7       # rad^2/s^3; chest; previous 1.8
vr_kalman_hand_rotation_q: 1.42 # palms only; both arms; previous 0.45

# Optional candidate, NOT default: meets 10-Hz gain <=1.03 and <1-ms
# additional phase delay over 0.1..5 Hz at 72/90/120 Hz. It passes MORE
# 30-Hz noise (roughly 9..24% amplitude); it is not a broadband denoiser.
# Keep disabled until input-noise replay/real-machine A/B supports the tradeoff.
vr_kalman_flattening:
  enabled: false
  position_q: 0.7875
  rotation_q: 7.125
  hand_rotation_q: 1.775
  position_velocity_gain: 0.425
  rotation_velocity_gain: 0.425
  hand_velocity_gain: 0.65
```

### 9.9 rnea_baseline.yaml

```yaml
# Free-space feedforward, frozen base. These are simulation commissioning defaults,
# not motor current limits or permission to run a new robot without validation.
mode: full
acceleration_gain: 0.25
velocity_cutoff_hz: 120.0
sensor_timeout: 0.05
max_period: 0.05
dynamic_ramp_seconds: 0.2
torque_limit_ratio: 0.8  # fraction of task.info torqueLimitsTask, reserve PD margin
torque_rate_lower: 3000.0  # Nm/s (joint-side)
torque_rate_arm: 1000.0
acceleration_limit_lower: 20.0  # rad/s^2 AFTER acceleration_gain
acceleration_limit_arm: 100.0
jerk_limit_lower: 200.0  # rad/s^3, dynamic feedforward only
jerk_limit_arm: 500.0
# Optional 18-element overrides: torque_rate, acceleration_limit, jerk_limit.

jerk_limit: [200.0, 200.0, 200.0, 200.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0]
```

### 9.10 simulation_servo.yaml

```yaml
# SI joint-side simulation gains; hardware PDO gains have different units.
# Order: lower 4, left arm 7, right arm 7, head 2. XML force limits remain active.
kp: [3000, 3000, 1000, 500, 270, 210, 256, 180, 120, 120, 120, 270, 210, 256, 180, 120, 120, 120, 10, 10]
kd: [150, 150, 80, 30, 20, 10, 10, 10, 6, 6, 6, 20, 10, 10, 10, 6, 6, 6, 1, 1]
```

## 10. JSON 配置完整原文

### 10.1 model_s400062.json

```json
{
  "velocity_ik_urdf": "biped_s400062/urdf/biped_s400062.urdf",
  "velocity_ik_fixed_joints": [
    "wheel_left_front_joint",
    "wheel_right_front_joint",
    "wheel_left_behind_joint",
    "wheel_right_behind_joint",
    "zhead_1_joint",
    "zhead_2_joint",
    "l_thumb_j1",
    "l_thumb_j2",
    "l_thumb_j3",
    "l_index_j1",
    "l_index_j2",
    "l_index_j3",
    "l_middle_j1",
    "l_middle_j2",
    "l_little_j1",
    "l_little_j2",
    "l_little_j3",
    "r_thumb_j1",
    "r_thumb_j2",
    "r_thumb_j3",
    "r_index_j1",
    "r_index_j2",
    "r_index_j3",
    "r_middle_j1",
    "r_middle_j2",
    "r_little_j1",
    "r_little_j2",
    "r_little_j3"
  ],
  "velocity_ik_position_limits": {
    "knee_joint": [
      0.0,
      1.448
    ],
    "zarm_l1_joint": [
      -2.4,
      0.6
    ],
    "zarm_r1_joint": [
      -2.4,
      0.6
    ],
    "zarm_l2_joint": [
      -0.349065850398866,
      1.46
    ],
    "zarm_r2_joint": [
      -1.46,
      0.349065850398866
    ],
    "zarm_l3_joint": [
      -1.5707963267949,
      0.46
    ],
    "zarm_r3_joint": [
      -0.46,
      1.5707963267949
    ]
  }
}
```

### 10.2 model_s62.json

```json
{
  "velocity_ik_urdf": "biped_s62/urdf/biped_s62.urdf",
  "velocity_ik_fixed_joints": [
    "wheel_left_front_joint",
    "wheel_right_front_joint",
    "wheel_left_behind_joint",
    "wheel_right_behind_joint",
    "zhead_1_joint",
    "zhead_2_joint"
  ],
  "velocity_ik_position_limits": {
    "knee_joint": [
      0.0,
      1.448
    ],
    "zarm_l1_joint": [
      -2.4,
      0.6
    ],
    "zarm_r1_joint": [
      -2.4,
      0.6
    ],
    "zarm_l2_joint": [
      -0.349065850398866,
      1.46
    ],
    "zarm_r2_joint": [
      -1.46,
      0.349065850398866
    ],
    "zarm_l3_joint": [
      -1.5707963267949,
      0.46
    ],
    "zarm_r3_joint": [
      -0.46,
      1.5707963267949
    ]
  }
}
```

### 10.3 model_s63.json

```json
{
  "velocity_ik_urdf": "biped_s63/urdf/biped_s63.urdf",
  "velocity_ik_fixed_joints": [
    "wheel_left_front_joint",
    "wheel_right_front_joint",
    "wheel_left_behind_joint",
    "wheel_right_behind_joint",
    "zhead_1_joint",
    "zhead_2_joint"
  ],
  "velocity_ik_position_limits": {
    "zarm_l1_joint": [
      -2.4,
      0.6
    ],
    "zarm_l2_joint": [
      -0.349065850398866,
      1.46
    ],
    "zarm_l3_joint": [
      -1.48352986419518,
      0.46
    ],
    "zarm_r1_joint": [
      -2.4,
      0.6
    ],
    "zarm_r2_joint": [
      -1.46,
      0.349065850398866
    ],
    "zarm_r3_joint": [
      -0.46,
      1.48352986419518
    ]
  }
}
```

### 10.4 runtime_partition.json

```json
{
  "version": 1,
  "wbc_cpus": [
    2,
    3
  ],
  "driver_reserved_cpus": [
    5,
    6,
    7
  ],
  "ik_cpus": [
    14
  ],
  "mapper_cpus": [
    4
  ],
  "background_cpus": [
    0,
    1,
    8,
    9,
    10,
    11,
    12,
    13
  ],
  "ik_fifo_priority": 70,
  "mapper_fifo_priority": 35,
  "mapper_nice": -10,
  "python_switch_interval_sec": 0.001
}
```

### 10.5 runtime_partition_aaeon_255h.json

```json
{
  "version": 2,
  "platform": "aaeon_255h",
  "wbc_cpus": [2, 3],
  "driver_reserved_cpus": [6, 7],
  "ik_cpus": [5],
  "mapper_cpus": [4],
  "sidecar_cpus": [14, 15],
  "background_cpus": [0, 1, 8, 9, 10, 11, 12, 13],
  "ik_fifo_priority": 70,
  "mapper_fifo_priority": 35,
  "mapper_nice": -10,
  "python_switch_interval_sec": 0.001
}
```

## 11. launch 内联 ROS 参数

### 11.1 bypass_communication_profile.launch

```xml
<param name="/vr_ik/visualization_rate_hz" value="$(eval 0.0 if arg('full_rate_audit') else 30.0)" type="double"/>
<param name="/vr_ik/execution_owner_mirror_stride" value="$(eval 1 if arg('full_rate_audit') else 5)" type="int"/>
<param name="/absolute_velocity_ik/diagnostic_rate_hz" value="$(eval 0.0 if arg('full_rate_audit') else 100.0)" type="double"/>
<param name="/absolute_velocity_ik/formal_reference_mirror_rate_hz" value="$(eval 0.0 if arg('full_rate_audit') else 100.0)" type="double"/>
<param name="/monitor_quest3/observation_tf_rate_hz" value="$(eval 0.0 if arg('full_rate_audit') else 30.0)" type="double"/>
<param name="/ik_ros_uni/noncontrol_publish_rate_hz" value="$(eval 0.0 if arg('full_rate_audit') else 100.0)" type="double"/>
<param name="/ik_ros_uni/noncontrol_publish_coalesce" value="$(eval not arg('full_rate_audit'))" type="bool"/>
<param name="/absolute_velocity_ik/communication_full_rate_audit" value="$(arg full_rate_audit)" type="bool"/>
<param name="/absolute_velocity_ik/operating_diagnostics/stride" value="$(eval 1 if (arg('full_rate_audit') or not arg('second_round_enabled')) else 5)" type="int"/>
<param name="/absolute_velocity_ik/runtime_probe_detail_stride" value="$(eval 1 if (arg('full_rate_audit') or not arg('second_round_enabled')) else 10)" type="int"/>
```

### 11.2 control.launch

```xml

```

### 11.3 dynamics_backend.launch

```xml
<param name="/dynamics_backend" value="$(arg dynamics_backend)" type="str"/>
<param name="/dynamics/rnea_shadow" value="$(arg rnea_shadow)" type="bool"/>
<param name="/dynamics/rnea/mode" value="$(arg rnea_mode)" type="str"/>
```

### 11.4 j3_overshoot_profile.launch

```xml
<param name="/j3_overshoot_profile/enabled" value="$(arg enabled)" type="bool"/>
<param name="/j3_overshoot_profile/j3_kd" value="$(arg kd)" type="double"/>
```

### 11.5 load_kuavo_absolute_sim_base.launch

```xml
<param name="/whole_body_velocity_ik/reference/transport" value="$(arg reference_transport)" />
<param name="/whole_body_velocity_ik/reference/shm_reference_key" value="$(arg reference_shm_key)" />
<param name="/whole_body_velocity_ik/reference/shm_owner_key" value="$(arg owner_shm_key)" />
<param name="/controller_type" value="wheel_arm_abs_vel" />
<param name="/cmd_truncation_enable" value="true" />
<param name="/absolute_simulation/motor_servo" value="$(arg simulation_motor_servo)" />
<param name="use_vr_control" value="$(arg use_vr_control)" />
<param name="/vr_ik/use_velocity_ik_reference" value="$(arg use_whole_body_ik_reference)" type="bool" />
<param name="/vr_ik/velocity_ik_reference_topic" value="/whole_body_velocity_ik/reference" type="string" />
<param name="/vr_ik/velocity_ik_reference_timeout_sec" value="0.05" type="double" />
<param name="/vr_ik/wbc_arm_ddq_feedforward_gain" value="0.25" type="double" />
<param name="gaitCommandFile" value="$(find humanoid_controllers)/config/kuavo_v$(arg robot_version)/command/gait.info" />
<param name="referenceFile" value="$(find humanoid_controllers)/config/kuavo_v$(arg robot_version)/command/reference.info" />
<param name="multiplot" value="$(arg multiplot)" />
<param name="wbc_only" value="$(arg wbc_only)" />
<param name="use_external_mpc" value="$(arg use_external_mpc)" />
<param name="wbc_frequency" value="$(arg wbc_frequency)" />
<param name="sensor_frequency" value="$(arg sensor_frequency)" />
<param name="with_estimation" value="$(arg with_estimation)" />
<param name="use_joint_filter" value="$(arg use_joint_filter)" />
<param name="estimator_type" value="$(arg estimator_type)" />
<param name="play_back" value="$(arg play_back)" />
<param name="real" value="$(arg real)" />
<param name="joystick_type" value="$(arg joystick_type)" />
<param name="arm_move_spd" value="$(arg arm_move_spd)" />
<param name="start_way" value="$(arg start_way)" />
<param name="use_estimator_contact" value="$(arg use_estimator_contact)" />
<param name="build_cppad_state" value="0" />
<param name="visualize_humanoid" value="$(arg visualize_humanoid)" />
<param name="enable_pull_up_protect" value="true" />
<param name="pull_up_force_threshold" value="0.70" />
<param name="robot_type" value="$(arg robot_type)" />
<param name="nodelet_robot_type" value="$(arg nodelet_robot_type)" />
<param name="wheel_control_type" value="$(arg wheel_control_type)" />
<param name="use_shm_communication" value="$(arg use_shm_communication)" />
<param name="taskFile" value="$(arg taskFile)" />
<param name="urdfFile" value="$(arg urdfFile)" />
<param name="libFolder" value="$(arg libFolder)" />
<param name="robot_description" textfile="$(arg urdfFile)" />
<param name="pure_sim" value="$(arg pure_sim)" />
<param name="teach_pendant" value="$(arg teach_pendant)" />
<param name="/mpc/schedule/enable" value="false" />
<param name="/mpc/schedule/start_time" value="-1.0" />
<param name="/timeout_warning_ms" value="20" />
<param name="/nodelet_manager/max_log_size_gb" value="34.0" />
<param name="/nodelet_manager/log_delete_limit_gb" value="34.0" />
<param name="num_worker_threads" type="int" value="10" />
<param name="depth_frequency" value="60.0" />
<param name="legged_robot_scene_param" value="$(find kuavo_assets)/models/biped_s$(arg robot_version)/xml/scene.xml" />
<param unless="$(arg real)" name="/joy_node/last_state" value="stance" />
```

### 11.6 load_kuavo_mujoco_velocity_ik.launch

```xml

```

### 11.7 load_kuavo_real_velocity_ik.launch

```xml
<param name="/controller_type" value="wheel_arm_abs_vel" />
<param name="/cmd_truncation_enable" value="true" />
<param name="/whole_body_velocity_ik/reference/transport" value="$(arg reference_transport)" />
<param name="/whole_body_velocity_ik/reference/shm_reference_key" value="$(arg reference_shm_key)" />
<param name="/whole_body_velocity_ik/reference/shm_owner_key" value="$(arg owner_shm_key)" />
<param name="/vr_ik/enable_base_control" value="true" type="bool" />
<param name="/vr_ik/arm_inertia_feedforward_scale" value="$(arg arm_inertia_feedforward_scale)" type="double" />
<param name="/base_locked" value="$(arg base_locked)" type="bool" />
<param name="use_vr_control" value="$(arg use_vr_control)" />
<param name="real" type="bool" value="$(arg real)" />
<param name="cali" type="bool" value="$(arg cali)" />
<param name="cali_arm" type="bool" value="$(arg cali_arm)" />
<param name="cali_leg" type="bool" value="$(arg cali_leg)" />
<param name="multiplot" value="$(arg multiplot)" />
<param name="wbc_only" value="$(arg wbc_only)" />
<param name="use_external_mpc" value="$(arg use_external_mpc)" />
<param name="wbc_frequency" value="$(arg wbc_frequency)" />
<param name="sensor_frequency" value="$(arg sensor_frequency)" />
<param name="with_estimation" value="$(arg with_estimation)" />
<param name="use_joint_filter" value="$(arg use_joint_filter)" />
<param name="estimator_type" value="$(arg estimator_type)" />
<param name="play_back" value="$(arg play_back)" />
<param name="joystick_type" value="$(arg joystick_type)" />
<param name="arm_move_spd" value="$(arg arm_move_spd)" />
<param name="start_way" value="$(arg start_way)" />
<param name="use_estimator_contact" value="$(arg use_estimator_contact)" />
<param name="only_half_up_body" value="$(arg only_half_up_body)" />
<param name="build_cppad_state" value="0" />
<param name="visualize_humanoid" value="$(arg visualize_humanoid)" />
<param name="enable_pull_up_protect" value="true" />
<param name="pull_up_force_threshold" value="0.70" />
<param name="enable_base_emergency_stop" value="$(arg enable_base_emergency_stop)" />
<param name="robot_type" value="$(arg robot_type)" />
<param name="nodelet_robot_type" value="$(arg nodelet_robot_type)" />
<param name="wheel_control_type" value="$(arg wheel_control_type)" />
<param name="ruiwo_cxx_sdk" value="$(arg ruiwo_cxx_sdk)" />
<param name="taskFile" value="$(arg taskFile)" />
<param name="urdfFile" value="$(arg urdfFile)" />
<param name="use_external_odom_tf" value="$(arg use_external_odom_tf)" />
<param name="libFolder" value="$(arg libFolder)" />
<param name="gaitCommandFile" value="$(find humanoid_controllers)/config/kuavo_v$(arg robot_version)/command/gait.info" />
<param name="referenceFile" value="$(find humanoid_controllers)/config/kuavo_v$(arg robot_version)/command/reference.info" />
<param name="robot_description" textfile="$(arg urdfFile)" />
<param name="teach_pendant" value="$(arg teach_pendant)" />
<param name="/mpc/schedule/enable" value="false" />
<param name="/mpc/schedule/start_time" value="-1.0" />
<param name="/timeout_warning_ms" value="20" />
<param name="/nodelet_manager/max_log_size_gb" value="34.0" />
<param name="/nodelet_manager/log_delete_limit_gb" value="34.0" />
<param name="num_worker_threads" type="int" value="10" />
<param name="hardware/is_ready" value="0" />
<param name="hardware/ready_to_start" value="0" />
<param name="lejuclaw_high_freq_switch" value="$(arg lejuclaw_high_freq_switch)" />
<param name="lejuclaw_vr_stream_timeout_ms" value="$(arg lejuclaw_vr_stream_timeout_ms)" type="int" />
<param name="joystick_type" value="h12" />
<param name="start_way" value="auto" />
<param name="relay_port" value="$(arg relay_port)" />
<param name="pressure_port" value="$(arg pressure_port)" />
<param name="pressure_publish_rate" value="$(arg pressure_publish_rate)" />
<param name="audio_path" value="/home/lab/.config/lejuconfig/music" type="string" />
<param name="/vr_ik/use_velocity_ik_reference" value="true" type="bool" />
<param name="/vr_ik/velocity_ik_reference_topic" value="/whole_body_velocity_ik/reference" type="string" />
<param name="/vr_ik/velocity_ik_reference_timeout_sec" value="0.05" type="double" />
<param name="/vr_ik/wbc_arm_ddq_feedforward_gain" value="0.25" type="double" />
<param name="/pinocchio_deployment/BASE_CONTROL" value="$(eval 'LOCKED' if arg('base_locked') else 'VR_INCREMENTAL_JOYSTICK')" type="str" />
<param name="/pinocchio_deployment/LEGACY_NORMAL_QUICK_SWITCH" value="DISABLED" type="str" />
<param name="/pinocchio_deployment/IK_PIPELINE" value="PINOCCHIO_VELOCITY" type="str" />
<param name="use_custom_arm_prepare" type="bool" value="$(arg use_custom_arm_prepare)" />
<param name="sg100_left_port" value="/dev/lefthand" />
<param name="sg100_right_port" value="/dev/righthand" />
<param name="sg100_baudrate" value="1000000" />
<param name="sg100_control_rate" value="100.0" />
<param name="sg100_publish_joint_states" value="true" />
<param name="sg100_cmd_filter_alpha" value="$(arg sg100_cmd_filter_alpha)" />
<param name="sg100_debug_state" value="false" />
<param name="sg100_output_limit" value="0.5" />
```

### 11.8 pico_input.launch

```xml
<param name="enable_head_control" value="$(arg enable_head_control)"/>
<param name="port" value="$(arg port)"/>
<param name="peer_ip" value="$(arg peer_ip)"/>
<param name="advertise_ip" value="$(arg advertise_ip)"/>
<param name="broadcast_ip" value="$(arg broadcast_ip)"/>
```

### 11.9 pico_velocity_ik.launch

```xml

```

### 11.10 quest_velocity_ik.launch

```xml
<param name="/whole_body_velocity_ik/reference/transport" value="$(arg reference_transport)" />
<param name="/whole_body_velocity_ik/reference/shm_reference_key" value="$(arg reference_shm_key)" />
<param name="/whole_body_velocity_ik/reference/shm_owner_key" value="$(arg owner_shm_key)" />
<param name="/vr_input_source" value="$(eval {'quest': 'quest', 'pico': 'pico'}[arg('input_source')])" />
<param name="/vr_ik/enable_solve_loop_timing_log" value="true" />
<param name="/vr_ik/enable_lock_wait_timing_log" value="true" />
<param name="/vr_ik/enable_arm_traj_receive_timing_log" value="true" />
<param name="/vr_ik/enable_arm_traj_shadow_publish" value="true" />
<param name="/vr_ik/enable_ik_velocity_limit_shadow_publish" value="true" />
<param name="/use_cpp_incremental_ik" value="$(arg use_cpp_incremental_ik)" type="bool" />
<param name="/enable_vr_stop_robot" value="$(arg enable_vr_stop_robot)" type="bool" />
<param name="/vr_ik/enable_absolute_mode_handshake" value="$(eval arg('enable_absolute_target_alignment') and not arg('use_cpp_incremental_ik'))" type="bool" />
<param name="/vr_cmd_vel/linear_scale_x" value="0.4" />
<param name="/vr_cmd_vel/linear_scale_y" value="0.4" />
<param name="/vr_cmd_vel/angular_scale_z" value="0.45" />
<param name="/single_hand_mode" value="$(arg single_hand_mode)" type="bool" />
<param name="/enable_timestamp_log" value="$(arg enable_timestamp_log)" type="bool" />
<param name="/quest3/limit_shoulder_pitch_upper" value="$(arg limit_shoulder_pitch_upper)" type="bool" />
<param name="/quest3/shoulder_pitch_upper_limit" value="$(arg shoulder_pitch_upper_limit)" type="double" />
<param name="/quest3/use_step_turning" value="$(arg use_step_turning)" />
<param name="control_torso" value="$(arg control_torso)" type="bool" />
<param name="/wheel_ik" value="$(arg wheel_ik)" type="bool" />
<param name="/vr_ik/enable_velocity_ik_takeover" value="$(arg enable_velocity_ik_takeover)" type="bool" />
<param name="/reset_joint_to_default" value="$(arg reset_joint_to_default)" type="bool" />
<param name="/use_pico" value="$(arg use_pico)" type="bool" />
<param name="continuity/hold_on_timeout" value="$(arg hold_on_input_timeout)" type="bool" />
<param name="robot_version" value="$(arg robot_version)" type="int" />
<param name="ctrl_arm_idx" value="$(arg ctrl_arm_idx)" type="int" />
<param name="solver_rate_hz" value="500.0" type="double" />
<param name="target_topic" value="/mm/teleop_tcp_target" type="string" />
<param name="enable_takeover" value="$(arg enable_velocity_ik_takeover)" type="bool" />
<param name="output_prefix" value="/whole_body_velocity_ik/diagnostics" type="string" />
<param name="formal_reference_topic" value="/whole_body_velocity_ik/reference" type="string" />
<param name="command_topic" value="/kuavo_arm_traj" type="string" />
<param name="command_frame_id" value="absolute_velocity_ik_v1" type="string" />
<param name="require_external_arm_mode" value="$(arg enable_velocity_ik_takeover)" type="bool" />
<param name="strict_target_frame" value="true" type="bool" />
<param name="tracking_source_mode" value="$(arg tracking_source_mode)" type="string" />
<param name="publish_input_debug" value="$(arg enable_vr_ik_debug)" type="bool" />
<param name="feasibility_mode" value="$(arg velocity_ik_feasibility_mode)" type="string" />
<param name="enable_singularity_escape_constraints" value="false" type="bool" />
<param name="root_translation_deadband" value="0.035" type="double" />
<param name="root_translation_full_distance" value="0.10" type="double" />
<param name="root_translation_coherence_start" value="0.03" type="double" />
<param name="root_translation_coherence_full" value="0.12" type="double" />
<param name="root_translation_active_confidence" value="0.20" type="double" />
<param name="root_translation_release_error" value="0.01" type="double" />
<param name="base_intent_gain" value="2.0" type="double" />
<param name="base_intent_weight" value="5.0" type="double" />
<param name="body_torso_deadband" value="0.015" type="double" />
<param name="body_torso_full_error" value="0.15" type="double" />
<param name="cascade_stage_dwell_time" value="0.10" type="double" />
<param name="cascade_stage_admission_threshold" value="0.15" type="double" />
<param name="cascade_stage_release_dwell_time" value="0.60" type="double" />
<param name="cascade_stage_release_threshold" value="0.05" type="double" />
<param name="natural_hang_arm_threshold" value="0.08" type="double" />
<param name="cascade_minimum_active_recruitment" value="0.20" type="double" />
<param name="arm_reference_tracking_qp_limit" value="0.06" type="double" />
<param name="lower_reference_tracking_qp_limit" value="0.08" type="double" />
<param name="reference_tracking_recovery_arm_limit" value="0.07" type="double" />
<param name="reference_tracking_recovery_lower_limit" value="0.05" type="double" />
<param name="reference_tracking_recovery_dwell" value="0.40" type="double" />
<param name="orientation_tracking_full_error" value="0.06" type="double" />
<param name="orientation_tracking_zero_error" value="0.16" type="double" />
<param name="orientation_joint_margin_zero" value="0.02" type="double" />
<param name="orientation_joint_margin_full" value="0.15" type="double" />
<param name="secondary_correction_regularization" value="1e-6" type="double" />
<param name="lower_directional_effectiveness_start" value="0.02" type="double" />
<param name="lower_directional_effectiveness_full" value="0.15" type="double" />
<param name="base_admission_dwell_time" value="0.40" type="double" />
<param name="base_admission_on_threshold" value="0.10" type="double" />
<param name="base_admission_off_threshold" value="0.03" type="double" />
<param name="preparation_measured_position_tolerance" value="$(eval 0.15 if int(arg('robot_version')) == 400062 else 0.06)" type="double" />
<param name="translation_singularity_activation_sigma" value="0.060" type="double" />
<param name="translation_singularity_safe_sigma" value="0.035" type="double" />
<param name="translation_singularity_escape_sigma" value="0.015" type="double" />
<param name="translation_singularity_gain" value="3.0" type="double" />
<param name="singularity_gradient_step" value="0.002" type="double" />
<param name="singularity_gradient_min_norm" value="0.001" type="double" />
<param name="natural_escape_speed" value="0.40" type="double" />
<param name="nominal_arm_reach" value="$(arg velocity_ik_nominal_arm_reach)" type="double" />
<param name="selected_reach_reserve" value="$(arg velocity_ik_selected_reach_reserve)" type="double" />
<param name="use_legacy_shoulder_pitch_cap" value="$(arg limit_shoulder_pitch_upper)" type="bool" />
<param name="legacy_shoulder_pitch_upper" value="$(arg shoulder_pitch_upper_limit)" type="double" />
<param name="wrist_envelope/enabled" value="$(arg wrist_envelope_enabled)" type="bool" />
<param name="continuous_reference_enabled" value="$(arg continuous_reference_enabled)" />
<param name="adaptive_reference/enabled" value="$(arg adaptive_reference_enabled)" />
<param name="innovation_reference/enabled" value="$(arg innovation_reference_enabled)" />
<param name="hand_reference_frequency" value="$(arg hand_reference_frequency)" />
<param name="torso_reference_frequency" value="$(arg torso_reference_frequency)" />
<param name="orientation_reference_mode" value="$(arg orientation_reference_mode)" />
<param name="hand_position_cutoff_hz" value="$(arg hand_position_cutoff_hz)" />
<param name="hand_orientation_cutoff_hz" value="$(arg hand_orientation_cutoff_hz)" />
<param name="joint_jerk/enabled" value="$(arg joint_jerk_enabled)" type="bool" />
<param name="execution_governor/enabled" value="$(arg execution_governor_enabled)" type="bool" />
<param name="operating_envelope/enabled" value="$(arg operating_envelope_enabled)" type="bool" />
<param name="temporal_cost/enabled" value="$(arg temporal_cost_enabled)" type="bool" />
<param name="operating_diagnostics/enabled" value="$(arg operating_diagnostics_enabled)" type="bool" />
<param name="use_compact_ik_status" value="true" type="bool" />
<param name="noncontrol_publish_async" value="true" type="bool" />
<param name="dexhand_front_trigger" value="$(arg dexhand_front_trigger)" type="bool" />
<param name="vr_kalman_flattening/enabled" value="$(eval arg('filter_tuning_enabled') and arg('vr_kalman_flattening_enabled'))" type="bool" />
<param name="vr_kalman_enabled" value="$(arg vr_kalman_enabled)" />
<param name="use_arm_collision" value="$(arg use_arm_collision)" />
<param name="enable_ik_velocity_limit" value="$(arg enable_ik_velocity_limit)" type="bool" />
<param name="enable_velocity_ik_takeover" value="$(arg enable_velocity_ik_takeover)" type="bool" />
<param name="sg100_enabled" value="$(arg sg100_enabled)" type="bool" />
<param name="whole_body_shadow_isolate_position_ik" value="$(arg whole_body_shadow_isolate_position_ik)" type="bool" />
<param name="enable_position_ik_status_log" value="$(arg enable_position_ik_status_log)" type="bool" />
<param name="publish_tracking_mapping_debug" value="$(arg publish_tracking_mapping_debug)" type="bool" />
<param name="tracking_source_mode" value="$(arg tracking_source_mode)" type="string" />
<param name="enable_absolute_target_alignment" value="$(arg enable_absolute_target_alignment)" type="bool" />
<param name="enable_absolute_start_gap_check" value="$(arg enable_absolute_start_gap_check)" type="bool" />
<param name="absolute_enable_max_position_error" value="$(arg absolute_enable_max_position_error)" type="double" />
<param name="absolute_enable_max_orientation_error" value="$(arg absolute_enable_max_orientation_error)" type="double" />
<param name="absolute_enable_max_vr_age" value="$(arg absolute_enable_max_vr_age)" type="double" />
<param name="absolute_enable_max_joint_state_age" value="$(arg absolute_enable_max_joint_state_age)" type="double" />
<param name="absolute_enable_mode_timeout" value="$(arg absolute_enable_mode_timeout)" type="double" />
<param name="absolute_enable_service_timeout" value="$(arg absolute_enable_service_timeout)" type="double" />
<param name="absolute_enable_exit_mode" value="$(arg absolute_enable_exit_mode)" type="int" />
<param name="enable_absolute_discontinuity_guard" value="$(arg enable_absolute_discontinuity_guard)" type="bool" />
<param name="absolute_max_tcp_position_step" value="$(arg absolute_max_tcp_position_step)" type="double" />
<param name="absolute_max_tcp_orientation_step" value="$(arg absolute_max_tcp_orientation_step)" type="double" />
<param name="absolute_max_joint_step" value="$(arg absolute_max_joint_step)" type="double" />
<param name="absolute_ik_trust_region_step" value="$(arg absolute_ik_trust_region_step)" type="double" />
<param name="absolute_hard_recovery_successes" value="$(arg absolute_hard_recovery_successes)" type="int" />
<param name="absolute_hard_target_max_step" value="$(arg absolute_hard_target_max_step)" type="double" />
<param name="absolute_hard_recovery_max_offset" value="$(arg absolute_hard_recovery_max_offset)" type="double" />
<param name="absolute_hard_recovery_stability_step" value="$(arg absolute_hard_recovery_stability_step)" type="double" />
<param name="absolute_execution_max_joint_velocity" value="$(arg absolute_execution_max_joint_velocity)" type="double" />
<param name="absolute_execution_max_joint_acceleration" value="$(arg absolute_execution_max_joint_acceleration)" type="double" />
<param name="absolute_execution_min_joint_limit_margin" value="$(arg absolute_execution_min_joint_limit_margin)" type="double" />
<param name="tracking_rearm_stable_frames" value="$(arg tracking_rearm_stable_frames)" type="int" />
<param name="velocity_tcp_forearm_length" value="0.45" type="double" />
<param name="velocity_tcp_geometry_id" value="V$(arg robot_version)_CONFIGURED_FULL_FOREARM_0450_UNCALIBRATED_V1" type="str" />
<param name="robot_version" value="$(arg robot_version)" type="int" />
<param name="robot_type" value="1" type="int" />
<param name="referenceFile" value="$(find humanoid_controllers)/config/kuavo_v$(arg robot_version)/command/reference.info" type="str" />
<param name="gaitCommandFile" value="$(find humanoid_controllers)/config/kuavo_v$(arg robot_version)/command/gait.info" type="str" />
<param name="wheel_ik" value="$(arg wheel_ik)" />
<param name="use_pico" value="$(arg use_pico)" />
<param name="control_finger_tf_enabled" value="$(arg predict_gesture)" type="bool" />
<param name="enable_head_control" value="$(arg enable_head_control)" />
<param name="enable_head_control" value="$(arg enable_head_control)" />
```

### 11.11 sg100_hand.launch

```xml

```

## 12. 完整检查与离线回归

### 12.1 核心依赖检查

已有环境通过此检查后可保留核心安装，首次缺依赖仍执行第一份第3节。检查不启动控制器。

```bash
bash --noprofile --norc -e <<'ABS_STEP_07'
  set -e
  cd ~/kuavo-ros-control-absolute
  source /opt/ros/noetic/setup.bash
  dependency_check_dir=$(mktemp -d /tmp/kuavo-absolute-deps.XXXXXX)
  cat > "$dependency_check_dir/CMakeLists.txt" <<'CMAKE_CHECK'
cmake_minimum_required(VERSION 3.16)
project(absolute_dependency_check LANGUAGES C CXX)
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs
  geometry_msgs sensor_msgs nav_msgs trajectory_msgs tf tf2_msgs
  message_generation nodelet pluginlib)
include(CheckIncludeFileCXX)
check_include_file_cxx("nlohmann/json.hpp" HAVE_NLOHMANN_JSON)
if(NOT HAVE_NLOHMANN_JSON)
  message(FATAL_ERROR "Install nlohmann-json3-dev")
endif()
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(drake CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glfw3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
find_library(UDEV_LIBRARY NAMES udev)
if(NOT UDEV_LIBRARY)
  message(FATAL_ERROR "libudev development library is missing")
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
  find_package(OpenVINO REQUIRED)
endif()
CMAKE_CHECK
  cmake -S "$dependency_check_dir" -B "$dependency_check_dir/build" \
    -Ddrake_DIR=/opt/drake/lib/cmake/drake
  printf '依赖检查目录：%s\n' "$dependency_check_dir"
ABS_STEP_07
```

### 12.2 运行前检查

首次编译或升级后执行全部代码，看到PREFLIGHT_PASS再启动。检查Python、launch、可执行文件和动态库，不运行控制器。

```bash
set -e
cd ~/kuavo-ros-control-absolute &&
sudo --preserve-env=ROBOT_VERSION bash --noprofile --norc <<'PRECHECK'
set -e
source devel/setup.bash
robot_model=$(/usr/bin/python3 src/wheel_arm_abs_vel/scripts/absolute_robot_version.py)
export ROBOT_VERSION="$robot_model"
export PYTHONDONTWRITEBYTECODE=1
export LD_LIBRARY_PATH="/opt/drake/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if ! /usr/bin/python3 -c 'import pydrake' >/dev/null 2>&1; then
  export PYTHONPATH="/opt/drake/lib/python3.8/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi

rospack find wheel_arm_abs_vel
rospack find kuavo_assets
if python3 src/wheel_arm_abs_vel/scripts/runtime_partition.py --check; then
  echo RUNTIME_PARTITION_PASS
else
  if test -f /.dockerenv; then
    echo "普通桌面仿真按第一份命令关闭平台CPU分区。"
  else
    echo "实机 CPU 分区不匹配，请按第二份文档修复。"; exit 1
  fi
fi
assets_dir=$(rospack find kuavo_assets)
python3 src/wheel_arm_abs_vel/scripts/absolute_end_effector.py \
  --config "$assets_dir/config/kuavo_v$robot_model/kuavo.json"

python3 - <<'PY_CHECK'
import importlib
import os
from pathlib import Path
import subprocess
import sys
import roslaunch
import roslib.packages
import rospkg

rp = rospkg.RosPack()
package = Path(rp.get_path("wheel_arm_abs_vel")).resolve()
expected = (Path.cwd() / "src/wheel_arm_abs_vel").resolve()
assert package == expected, "加载到其他工作区: " + str(package)
sys.path.insert(0, str(package / "scripts"))
for name in ("numpy", "scipy", "yaml", "pydrake", "netifaces",
             "google.protobuf", "psutil", "zmq",
             "wheel_arm_abs_vel.msg", "ik_ros_velocity",
             "monitor_quest3_velocity", "monitor_pico", "robot_state_server_velocity"):
    importlib.import_module(name)
    print("IMPORT_PASS", name)

for entry in ("load_kuavo_real_velocity_ik.launch",
              "quest_velocity_ik.launch", "pico_velocity_ik.launch",
              "load_kuavo_mujoco_velocity_ik.launch"):
    config = roslaunch.config.load_config_default(
        [(str(package / "launch" / entry),
          ["robot_version:=" + os.environ["ROBOT_VERSION"]])],
        None, verbose=False)
    for node in config.nodes:
        if node.package in ("kuavo_audio_player", "kuavo_audio_receiver", "kuavo_led_controller") or node.type == "start_modbus_io_bridge.sh":
            continue  # 辅助功能不在本文验收范围
        found = roslib.packages.find_node(node.package, node.type, rospack=rp)
        assert found, "缺少节点: " + node.package + "/" + node.type
    print("LAUNCH_AND_EXECUTABLES_PASS", entry, len(config.nodes))

libraries = (
    "wheel_arm_abs_vel/pinocchio_velocity_ik_node",
    "wheel_arm_abs_vel/absolute_mujoco_sim",
    "wheel_arm_abs_vel/fixed_ik_reference_services_node",
    "wheel_arm_abs_vel/humanoid_quest_absolute_control",
    "libwheel_absolute_controllers.so",
    "libnodelet_wheel_absolute_controller.so",
    "libwheel_absolute_dynamics.so",
    "libnodelet_hardware.so",
    "libhumanoid_wheel_wbc.so",
)
library_dirs = [Path("devel/lib")] + [
    Path(v) for v in os.environ.get("LD_LIBRARY_PATH", "").split(":") if v]
for item in libraries:
    candidates = [folder / item for folder in library_dirs]
    path = next((v for v in candidates if v.is_file()), None)
    assert path is not None, "缺少编译产物: " + item
    result = subprocess.run(["ldd", str(path)], text=True, capture_output=True)
    output = result.stdout + result.stderr
    assert result.returncode == 0 and "not found" not in output, output
    print("LINK_PATH_PASS", item, str(path.resolve()))
print("PREFLIGHT_PASS")
PY_CHECK
PRECHECK
```

### 12.3 离线回归

修改源码后在停止控制的工作区执行；机型由启动进程的ROBOT_VERSION环境读取：

```bash
bash --noprofile --norc -e <<'ABS_STEP_11'
  set -e
cd ~/kuavo-ros-control-absolute
source devel/setup.bash
python3 -B src/wheel_arm_abs_vel/test/absolute_sim_source_test.py
python3 -B src/wheel_arm_abs_vel/test/pico_discovery_test.py
python3 -B src/wheel_arm_abs_vel/test/pico_input_parity_test.py
python3 -B src/wheel_arm_abs_vel/test/terminal_initial_start_test.py
python3 -B src/wheel_arm_abs_vel/test/runtime_topology_test.py
python3 -B src/wheel_arm_abs_vel/test/runtime_partition_test.py
python3 -B src/wheel_arm_abs_vel/test/absolute_robot_version_test.py
python3 -B src/wheel_arm_abs_vel/test/absolute_end_effector_test.py
ABS_STEP_11
```


```bash
bash --noprofile --norc -e <<'ABS_STEP_12'
  set -e
  cd ~/kuavo-ros-control-absolute
  source devel/setup.bash
  robot_model=$(/usr/bin/python3 src/wheel_arm_abs_vel/scripts/absolute_robot_version.py)
  export ROBOT_VERSION="$robot_model"
  catkin profile set velocity-ik
  catkin build wheel_arm_abs_vel --no-deps -j4 -p2 --no-status --make-args tests
  cd build/velocity-ik/wheel_arm_abs_vel
  ctest --output-on-failure
ABS_STEP_12
```

## 13. 启动时读取 kuavo.json 与 CPU 适配

### 13.1 读取时机和机型来源

遵循humanoid-control的启动链：显式robot_version参数优先，否则读取进程环境ROBOT_VERSION。Shell负责加载持久环境；启动程序不再自行解析~/.bashrc或/etc/environment。显式参数有冲突、变量非法或缺失时，绝对式入口报错；不采用通用人形入口的默认版本40。

机型确定后导出到启动子进程，launch通过optenv读取，并传给已有robot_version_manager.launch。该管理器按版本选择kuavo_assets/models/biped_s编号中的URDF、kuavo_assets/config/kuavo_v编号/kuavo.json，以及对应控制配置，并把JSON加载为kuavo_configuration参数。末端解析读取同一机型JSON；CPU检测仍独立进行。程序不通过CPU或资产目录列表推测物理机型。

普通用户通过sudo启动时显式保留ROBOT_VERSION；不再绕去读取root的.bashrc。第一份已写好sudo --preserve-env=ROBOT_VERSION，因此在部署正确的机器上日常无需手填机型。

### 13.2 字段与职责

| 配置来源 | 作用 | 注意 |
| --- | --- | --- |
| ROBOT_VERSION / robot_version | 确定读取哪个机型目录 | 是机器身份，不由CPU类型猜测 |
| kuavo.json 的 EndEffectorType | 选择左右末端 | 左右类型、驱动和实际设备必须匹配 |
| kuavo.json 及配套模型配置 | 模型路径、运动学与几何 | 新自动模型契约合入后按其字段读取，不手工复制另一机型数据 |
| /proc、sysfs、可用CPU集合与平台配置 | CPU分区选择与校验 | CPU型号和拓扑独立检测，不是kuavo.json决定CPU型号 |
| 机型标定与设备配置 | 零位和硬件通信 | 自动选型不能替代机器标定 |

### 13.3 当前候选入口与范围

第一份直接使用当前草稿分支，绝对式入口保持 wheel_arm_abs_vel；尚未合入 dev。候选源码当前仍保留62、63、400062的控制兼容检查。自动读取版本不等于支持任意机型；更广泛的模型契约适配须在合并时核对，不能仅凭存在kuavo.json认定支持。

### 13.4 一次配置与每次读取

首次部署把正确ROBOT_VERSION导出到当前Shell，并保存在实际启动用户的Shell配置中。新交互终端加载该环境；非交互服务须在服务环境中配置同一变量。配置文件变更后重新加载Shell环境，不能期待程序回读.bashrc。

每次启动依据环境变量选择资产和kuavo.json，不缓存末端或CPU计算结果。配置变更下次启动生效，不支持运行中热切换。控制器与Pico/Quest终端必须加载相同机型环境。

### 13.5 仅缺少ROBOT_VERSION的新环境：一次性配置

已有机器人或已配置的仿真容器跳过本节，不重复写机型。正常启动会自动读取现有环境。只有全新环境没有机型变量时，执行下面代码；在容器内配置仅作用于该容器的sim用户。

若 Shell 文件已有赋值但当前变量为空，重新打开正常交互终端或修正原有赋值，不追加重复配置。新环境可在当前终端显式导出已确认的实际机型，例如以下仅用于 S62 仿真；S63 或 400062 使用对应编号。需要持久化时在实际启动用户的 Shell/服务配置中维护同一变量，无需交互选择。

```bash
export ROBOT_VERSION=62
python3 src/wheel_arm_abs_vel/scripts/absolute_robot_version.py
```

## 14. 依赖、网络与维护说明

### 14.1 首次安装

第一份中调用的kuavo_environment_setup.sh和install_deps_packages.sh均来自已经拉取的源仓库，不是文档外部工具。已显示具体调用的安装函数和额外APT/Python包；不执行安装脚本完整main。安装函数可能修改系统软件源，必须在控制已安全停止的机器上执行。

Drake固定1.19.0、Pinocchio沿用项目版本2.6.21。系统Python3.8、Pillow9.5.0及固定numpy-quaternion用于本版兼容。APT出现Drake签名过期时，执行以下代码刷新签名，不跳过校验：

```bash
bash --noprofile --norc -e <<'ABS_STEP_06'
  set -e
  key_file=$(mktemp /tmp/kuavo-drake-key.XXXXXX)
  curl -fsSL https://drake-apt.csail.mit.edu/drake.asc -o "$key_file"
  test "$(gpg --show-keys --with-colons "$key_file" | awk -F: '$1=="fpr" {print $10; exit}')" = B8305A890EB713000ABBC32382FAAFE5AFEF06D7
  if test -f /etc/apt/trusted.gpg.d/drake.gpg; then
    sudo cp -an /etc/apt/trusted.gpg.d/drake.gpg /etc/apt/trusted.gpg.d/drake.gpg.before-absolute
  fi
  sudo gpg --batch --yes --dearmor -o /etc/apt/trusted.gpg.d/drake.gpg "$key_file"
  sudo chmod 644 /etc/apt/trusted.gpg.d/drake.gpg
  sudo apt-get update
ABS_STEP_06
```

### 14.2 Pico自动发现网络

默认无需填写头显IP、本机IP或广播地址。参考人形Pico的RobotInfoBroadcaster，绝对式接收器启动时内置枚举启用且支持广播的IPv4网卡，在每个有效地址所属子网上发送机器人信息；多个网卡无需交互选择。跳过回环、链路本地、关闭接口和不支持广播的地址。广播地址按实际掩码计算，例如10.10.30.176/23对应10.10.31.255。

广播内容保持data.robot_name、data.robot_ip和data.eef_type，UDP8443每秒发送一次；UDP12345接收头显数据。各网卡发送失败独立处理，不阻止其他网卡广播。启动日志打印发现地址列表。没有有效网卡时告警，连接网络后重启输入节点；当前在启动时读取接口，不支持运行中自动刷新网络地址。

需要限定网络时，在Pico启动行追加robot_ip:=本机局域网地址 broadcast_ip:=该网段广播地址，两者必须一起提供。robot_ip是电脑地址，不是Pico头显地址。默认会在所有有效广播接口上发布发现信息；需要限制广播范围时使用上述显式配置。

Pico需full_body、手柄数据和包含SPINE3的身体数据，与电脑同一广播域；AP不能隔离客户端，防火墙允许上述UDP端口。接收器绑定首个有效头显，勿同时启动另一个Pico/GMR接收器。网卡发现不会改变控制模式或发出机器人动作。

### 14.2.1 Quest指定IP连接

Quest与Pico使用不同的连接方式。第一份真机和仿真Quest命令都显式传ip_address。本次 Quest IP 为 192.168.5.24，命令已填入；网络变化时按实际地址修改，不得留空。无需填写机器人IP，也不启动Quest自动广播发现。

源码monitor_quest3_velocity.py收到有效IP后执行setup_socket，默认头显端口10019，再发送初始握手；缺少IP时才进入broadcast_robot_info_and_wait_for_quest3。源码仍保留这条自动发现功能，但本文不使用。不需要另传一个不存在的关闭广播参数。网络检查用Quest当前显示的IP，换网络后重新填写。

### 14.3 最简启动与默认参数覆盖

第一份采用逐行交互命令：进入目录，sudo进入root交互Shell，加载系统ROS和工作区环境，然后执行仓库启动入口。sudo保留ROBOT_VERSION，仿真还保留图形变量。sudo行执行后，剩余命令应在root子Shell内执行；不要把这些交互步骤原样当成无人值守脚本。

机型、kuavo.json、末端和CPU按已有配置自动读取。Drake安装路径兼容处理在入口内，不用重复手写。以下覆盖参数追加到对应启动命令末尾，同一参数只写一次；完整参数见第8节。

| 项目 | 默认行为 | 修改方式 |
| --- | --- | --- |
| robot_version | 读取进程ROBOT_VERSION | 控制器和输入均追加robot_version:=62，通常无需覆盖 |
| 真机dynamics_backend | rnea | 仅真机控制器追加dynamics_backend:=wbc；输入入口不声明此参数 |
| 仿真dynamics_backend | rnea | 仅仿真控制器追加dynamics_backend:=wbc等已验证选项；输入入口不传此参数 |
| base_locked | true，底盘锁定 | 仅真机控制器末尾追加base_locked:=false；当前仿真顶层入口未声明该参数 |
| ros_ip | 真机默认192.168.26.1 | 控制器追加ros_ip:=本机实际底盘通信地址 |
| ee_type | auto，读取机型JSON | 输入入口追加ee_type:=实际末端类型，通常不覆盖 |
| enable_head_control | false | 由控制器的 head_control_node 负责，默认 vr_follow；此兼容参数不重新启用接收器头控 |
| enable_terminal_start | false；默认第一个控制器终端按 o | 仅需输入端备用监听时追加 enable_terminal_start:=true |
| Quest ip_address | 源码空值会自动发现，本文不采用 | 必须在Quest输入命令填写ip_address:=头显IPv4地址 |
| Pico robot_ip / broadcast_ip | 留空，自动网卡广播 | Pico输入成对追加robot_ip:=本机地址 broadcast_ip:=广播地址 |
| visualize_humanoid | 仿真false | 文档显式传visualize_humanoid:=true，确保窗口可见 |
| KUAVO_RUNTIME_PARTITION | 启用平台CPU策略 | 仅电脑仿真export KUAVO_RUNTIME_PARTITION=0 |

真机需已有正确机器身份、驱动和底盘网络配置。终端o调用真实初始化服务，按现场提示扶稳后确认；仿真不调用硬件初始化。X+Y 保留软件停止；X+B 仅固定双臂，躯干与底盘权限独立。躯干组合键可以关闭躯干并让双臂继续跟随。启动前确认没有另一套控制链路。

### 14.4 容器

Dockerfile和docker build/run/exec命令都在第一份展开，不依赖外部Dockerfile。采用host网络以接收局域网广播，private IPC，并添加SYS_NICE和rtprio=99以允许Mapper设置线程优先级；两个终端进入同一容器。没有映射机器人设备。

源码和编译产物位于home卷，系统依赖位于容器可写层。保留容器并stop/start；删除容器后仅靠home卷不能恢复全部环境。容器不挂载文档目录或候选补丁，直接在容器内拉取第2节指定的草稿分支。

### 14.5 版本更新

更新前安全停止控制，核对当前分支为 `codex/abs-pico-startup-20261021` 且没有待处理修改；只执行快进拉取。已有部署使用其他测试分支时先核对其同步关系，不能照抄切换或覆盖。Catkin也可能修改已跟踪的profile，先核对保存，不使用reset或清空目录。更新后重编译并预检。

### 14.6 绝对式仿真退出与图形兼容

仿真启动命令不变，内部改用绝对式专用进程absolute_mujoco_sim。Ctrl+C、关闭MuJoCo窗口或SIGTERM都会结束仿真并带动整个仿真启动组退出；请先在输入终端Ctrl+C，再结束控制器。日志应出现[AbsoluteSimulation] clean exit和控制线程join信息。ROS会把required节点正常结束显示为“has died / process has finished cleanly”，这不等于崩溃，应检查子进程退出状态和后续日志。

KUAVO_SIM_X11_NO_SHM默认1，使本仿真使用普通X11图像传输，以兼容private IPC容器。无需额外启动参数；若宿主图形环境确实需要系统默认MIT-SHM，在启动控制器的root终端先执行export KUAVO_SIM_X11_NO_SHM=0。

构建会在build目录生成绝对式私有MuJoCo编译单元，公共mujoco_cpp文件及库不变。如果报告MuJoCo lifecycle anchor changed，表示上游相关源码变化，需要维护者复核适配，不能跳过此检查。

## 15. 验证边界

2026-10-22 本次权限修复后的源码 `b26d5d18e` 已通过 Release 构建及62/62 CTest；6种 CPU 拓扑下增量式调度配置与当时 dev 一致。本次权限修复已补测首次开启前撤销、首次开启后关闭/复位、恢复 ready 和显式重开，并验证正常首次跟随仍默认开启；Pico 另补测未收到姿态前发送发现包。上述属于软件回归，不能替代真实 App 无线发现和机器人动作验收。

此前 #4486/#4452 候选完成 Quest 10 项、Pico 16 项合成输入仿真；底盘项验证指令通道，不等于硬件位移验收。AGX 卡顿改善、Pico 真头显和完整版本稳定性仍需现场复测。#4485/#4478 暂不处理；手臂偏斜及头部耦合不列为本次已修复。

以下表格保留较早首装和仿真退出验证记录；49/49是历史版本结果，不是当前回归数量。

本轮验证对象为MR !4139候选，尚未合入dev。测试容器为全新Ubuntu20.04环境，重新拉取仓库、完整安装依赖，并从空build/devel开始编译；使用yyb已有SSH权限拉取候选分支，不能据此声称未合并的dev已包含这些修改。

| 项目 | 本轮结果 | 边界 |
| --- | --- | --- |
| 依赖首装 | 通过，pip check无依赖冲突 | Ubuntu20.04 x86_64隔离容器 |
| 完整编译 | 62个目标包成功，0失败 | 14个包有编译警告 |
| 启动预检 | Python、launch、节点和动态库检查通过 | 不代表硬件驱动已验收 |
| 回归测试 | 49/49通过 | S63仿真条件不一致修复后重跑通过 |
| 可视仿真 | S62、S63显示窗口并发布反馈 | 本轮未验证实际FOLLOW轨迹 |
| Quest指定IP | 模拟握手、姿态和按键通过，无发现广播 | 使用本机协议模拟器，非真实头显 |
| Pico自动发现 | 自动广播网卡地址并接收模拟数据 | 非真实头显无线网络稳定性测试 |
| 仿真退出 | S62 Ctrl+C、SIGTERM、关闭窗口及S63 Ctrl+C均正常回收 | 没有再出现terminate、LCM悬空线程、段错误或强制杀进程 |
| 真机 | 本轮未启动 | 真机动作和长时间稳定性仍需现场验收 |

无头显数据时的头部TF等待、模型/依赖警告属于日志中的非致命提示；短时测试订阅者退出附近观察到XMLRPC连接拒绝，未影响反馈或退出结果，因此不宣称所有日志零告警。source_relative_age表示相对积压估计，不是单向无线延迟，不能放宽保护阈值或删除HOLD掩盖输入异常。

复验详情见同目录validation_20261021.md。
