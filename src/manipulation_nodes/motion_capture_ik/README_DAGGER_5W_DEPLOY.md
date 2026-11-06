# 5W Quest3 与 LeTools DAgger 配套运行

本功能在轮臂增量遥操基础上增加候选模式。设置 `dagger_candidate_only:=true` 后，下位机生成手臂、末端和可选腰部候选，由边侧 LeTools-Learning 的 `dagger` 任务仲裁、渐变并统一下发。普通遥操使用 `dagger_candidate_only:=false`。

以下命令以 v63、单 Quest、Leju claw 为例，仓库路径和 Quest IP 按现场修改。当前候选求解使用四维下身加十四维手臂的 18 维模型，不支持 `dual_vr`、arm-only IK 或 `single_hand_mode` 摇杆控躯干入口。边侧的 `which_arm: left/right/both` 用于选择介入侧，与 KRC 的 `single_hand_mode` 不是同一个设置。

## 编译

按照仓库 [readme.md](../../../readme.md#编译) 中的教程编译。

## 启动

两机 ROS 终端使用同一个 `ROS_MASTER_URI`，各自的 `ROS_IP` 必须可被对端访问，并同步时钟。切换到 root 后也要确认网络配置；具体网络及边侧环境安装见 LeTools-Learning 的 `README_DAGGER.md`。

先确认运控机型配置中的 `EndEffectorType` 与实际末端一致。下面 VR 命令中的 `ee_type:=lejuclaw` 只选择 VR 侧末端类型，不能代替运控的硬件配置。

终端 1 启动运控，并按现场流程完成机器人初始化。示例显式关闭独立头控，避免依赖本机修改过的 launch 默认值：

```bash
sudo su
cd /home/lab/kuavo-ros-control-150
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real_wheel.launch with_head_control_node:=false
```

终端 2 启动候选节点，并完成现场所需的 Quest/人体标定。先退出普通 VR launch，同一时间只运行一套 VR 输入链：

```bash
sudo su
cd /home/lab/kuavo-ros-control-150
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch \
  ip_address:=10.42.0.12 \
  use_cpp_monitor:=false \
  use_cpp_incremental_ik:=true \
  wheel_ik:=true \
  use_arm_only_ik:=false \
  single_hand_mode:=false \
  use_incremental_hand_orientation:=true \
  reset_joint_to_default:=false \
  arm_traj_smooth_enable:=false \
  ee_type:=lejuclaw \
  dagger_candidate_only:=true
```

此命令的位置和手腕姿态均采用增量映射。候选模式下 `use_incremental_hand_orientation` 默认 true，普通遥操默认 false；命令行显式设置优先。旧命令若写了 `use_incremental_hand_orientation:=false`，需改为 true 或移除覆盖值。只改 ROS 参数不会更新已经启动的 IK 节点，需重启 VR launch。

边侧启动与任务匹配的模型服务，并在 LeTools 菜单选择 `dagger`。推理、接管渐变、回切和录包配置见边侧 `README_DAGGER.md`。腰部介入还要求边侧开启 `5w_wholebody: true`、`dagger_waist_on: true`，模型布局匹配，且至少一个下身关节未锁定。

## 操作与执行模式

候选模式不执行普通 VR 的准备归零，不需要 `X+A` 开启真实控制模式。有新鲜反馈时，未按 grip 也会持续发布候选，空闲侧同步实测位置；按住边侧配置控制侧的 grip 请求接管，松开后由边侧等待新模型动作并平滑回切。

`X+A/X+B` 不再切换机器人真实控制模式。`X+Y` 在 `enable_vr_stop_robot=true` 时仍通过 `/stop_robot` 发送停止请求。候选模式不直接下发手臂、末端或腰部运动目标；停止消息及独立头控不属于这些运动候选，头控由运控 launch 的 `with_head_control_node` 单独决定。

候选 IK 节点启动和退出不设置/恢复 VR 增益、加速度任务、快速模式、插补器或 SHM 链路。边侧沿用已有流程启用插补、选择 `/kuavo_arm_traj` transport 并设置 `quickMode=2`。当候选参数为 true、控制器首次进入 DAgger 上肢快模式时，使用本机新鲜实测关节清除旧目标，再进入 WBC 外部控制模式 2。退出时释放到 WBC 模式 0；软暂停使释放失败时仍关闭上肢快模式，并返回失败供调用方处理。

这里的 `quickMode` 和 WBC 控制模式是两套状态。上述切换发生在任务执行初始化/退出，不在每次 grip 接管时重复。快速模式入口检查的是控制器本机接收缓存的时间，不拿硬件 `sensor_time` 与 ROS 时钟比较。

每次接管按侧重建实测锚点，并清理对应的旧 IK/点优化历史及四元数恢复状态。持续遥操仍使用 KRC 的 IK 和发布处理，关闭 `arm_traj_smooth_enable` 不等于关闭全部滤波。当前候选模式把左右第 7 腕关节限制为 URDF 与 `[-0.60, 0.60] rad` 的交集。

## 配套协议

| 话题 | 类型与内容 |
|---|---|
| `/dagger/vr/arm_candidate` | `std_msgs/String`，JSON `schema_version=1`；左 7 维、右 7 维位置 rad 和速度 rad/s，以及同帧接管状态和 `waist` 扩展 |
| `/dagger/vr/eef_action` | `sensor_msgs/JointState.position=[左 trigger,右 trigger]`，范围 `[0,1]` |
| `/dagger/vr/left_intervention`、`/dagger/vr/right_intervention` | `std_msgs/Bool`；grip 达到 0.70 按下，降到 0.30 释放，随 joystick 输入持续更新 |
| `/dagger/vr/body_pose` | `geometry_msgs/PoseStamped`；适配器从 Chest（索引 23）生成，KRC 本地用于腰部计算 |
| `/dagger/vr/waist_config` | `std_msgs/String`；腰部介入开启时，由边侧自动发布锁存配置 |

手臂 JSON 包含 `schema_version/session/sequence/stamp_ns/position/velocity`、`left_requested/right_requested`、`left_ready/right_ready`、`left_generation/right_generation` 及 `waist` 对象。手臂数组始终为 14 维，不包含夹爪或下身。当前边侧使用这一 JSON 协议，不接收旧的 `/dagger/vr/arm_action` JointState 手臂协议。

节点重启改变 `session`，每个发布帧递增 `sequence`；grip 状态变化或输入失效会撤销 ready 并更新代次。完成当前代次的实测重锚和有效 IK 后才 ready；对应侧首个 ready 帧使用当前实测位置和零速度。未接管侧不沿用上一轮 VR 缓存。`ready=true` 只表示候选准备好，实际接管还取决于边侧收到新鲜请求、候选和末端输入后的仲裁。

手臂 `stamp_ns` 是候选发布时的 ROS 时间，不是硬件执行时间。KRC 另行检查原始关节反馈消息的时间戳、骨骼接收和介入心跳，手臂侧上限固定为 0.2 秒；反馈过期时跳过候选发布，骨骼/心跳过期时撤销对应请求与就绪。边侧还检查候选及信号年龄，调大边侧超时不会放宽下位机固定的手臂检查。

腰部扩展保持 JSON schema 1，包含 `config_session/requested/ready/generation/stamp_ns/position`，位置顺序为 `[knee,leg,waist_pitch,waist_yaw]`，单位 rad。边侧配置包含 `schema_version=1`、`session/which_arm/lower/upper/lock_mask/lock_position/max_speed/action_timeout/signal_timeout`。KRC 确认配置会话，在有效控制侧 grip 按住时建立身体增量锚点并求解；输入、反馈或求解失效时撤销腰部 ready。

腰部 `stamp_ns` 取本次依赖的身体位姿与传感器反馈中较早的时间，未就绪时为 0。身体位姿时间戳由输入适配器在回调时生成；重复发布腰部候选不会刷新旧输入年龄。腰部的动作/信号超时使用边侧配置，默认 0.2 秒；关节限位、锁定和速度限制来自边侧配置，并与 KRC URDF 限位结合。腰部介入使用身体位姿与 grip，不使用普通 VR 的 X+摇杆开关。

机器人执行话题仍遵循原有单位：`/kuavo_arm_traj` 为角度和角速度，`/lb_leg_traj` 为角度。候选和执行话题不可混用。

## 检查与排查

逐条执行；频率检查用 Ctrl+C 结束后再运行下一条：

```bash
rosparam get /quest3/dagger_candidate_only
rosparam get /quest3/connected
rostopic type /dagger/vr/arm_candidate
rostopic hz /dagger/vr/arm_candidate
rostopic echo -n 1 /dagger/vr/arm_candidate
rostopic hz /dagger/vr/eef_action
rostopic echo -n 1 /dagger/vr/left_intervention
rostopic echo -n 1 /dagger/vr/right_intervention
```

未按 grip 时 `requested=false, ready=false` 正常。边侧启动最多等待 3 秒获得合法、新鲜的手臂候选，不要求提前按住 grip。腰部开启时还会等待配置会话确认；检查 `waist.config_session` 与本轮 `/dagger/vr/waist_config` 中的 `session` 一致，再按有效 grip 检查腰部 ready。

| 现象 | 检查内容 |
|---|---|
| 编译时报 build/devel 权限错误 | 按仓库要求进入 root，使用目标仓库的默认构建目录 |
| 没有合法手臂候选 | 确认候选节点已重新编译并重启；检查 Quest 标定、关节反馈和时间戳，以及 ROS 网络 |
| `DAgger requires fresh locally received arm feedback` | 控制器拒绝上肢快模式；检查运控传感器缓存与本机接收年龄，确认新 `humanoid_controllers` 已编译并重启 |
| 手腕仍像绝对式 | 检查命令行是否还覆盖为 `use_incremental_hand_orientation:=false`，改正后重启 VR |
| 按住 grip 但 ready 持续为 false | 检查反馈/骨骼/请求是否新鲜、重锚和 IK 是否成功；未按 grip 时 false 属于正常状态 |
| 腰部不响应 | 检查边侧全身/腰部开关、锁定关节、配置会话、有效控制侧 grip、`body_pose` 和反馈新鲜度 |

边侧启动检查会确认同一个候选发布节点订阅左右介入请求，且没有普通 `humanoid_quest_control_with_arm` 同时运行。IK 在两种模式下都可能注册执行话题 publisher，不能单看 publisher 名单认定候选正在驱动机器人，应核对实际流量和控制服务调用。

普通遥操使用同一个 launch，将 `dagger_candidate_only` 省略或设为 false。普通遥操的准备归零、腰部开关、快速模式与头控配置按现场原设置使用；不要直接把候选示例的所有参数视为普通遥操的默认推荐。
