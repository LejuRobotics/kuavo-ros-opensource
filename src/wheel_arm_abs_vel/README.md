# Wheel-arm absolute velocity IK

This additive package contains the S62, S63 and 400062 absolute teleoperation mode. Its source
was extracted from a145c974e3b475d0740acd24cd36346b1723573b onto official dev
9ca0b7b880188faa1b686b48e906c032c36124d7. The platform adaptation is confined to this package. Shared hardware and legacy
incremental/humanoid sources are unchanged from the target dev branch.

## 使用与启动 / Getting started

完整中文使用指南见 [研扬 / AGX 自动识别与启动](docs/platform_runtime_adaptation.md)。
包含5.19可复制的两终端命令、机型一次性配置、14/16核研扬与12核AGX分配、
只读检查、Pico/可见仿真、常见报错、停止与回退。

已保存正确机型并source工作区后，两端都使用专属wrapper，无需每次传版本/核数：

```bash
bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh wheel_arm_abs_vel load_kuavo_real_velocity_ik.launch dynamics_backend:=rnea vr_remote_control:=true base_locked:=false
bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh wheel_arm_abs_vel quest_velocity_ik.launch ip_address:=192.168.5.21
```

上面是两个终端分别执行的命令，Quest IP沿用5.19当前记录，其他现场替换为实际地址。
机型来自显式参数、环境或当前用户已保存配置，不根据CPU猜测，也不再默认62。
实机准备环境、root权限和工作区路径请先按完整指南操作。

底盘默认锁定；上述 `base_locked:=false` 仅在ACTIVE跟随时启用左摇杆平移（各轴0.40 m/s）和右摇杆转向（0.45 rad/s）。暂停、失去正式参考或指令超时0.20秒即清零；详细操作和底盘网段说明见完整指南。

新版输入操作说明见 [Quest / Pico 按键、头控与终端 o](docs/pico_quest_controls.md)。

## Selection

Existing launch commands keep their existing behavior. The new selector defaults
to the original wheel controller:

    roslaunch wheel_arm_abs_vel control.launch

Select the new controller explicitly:

    bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh wheel_arm_abs_vel control.launch controller_type:=wheel_arm_abs_vel

The direct load_kuavo_real_velocity_ik.launch and load_kuavo_mujoco_velocity_ik.launch
entries also explicitly select this mode. The new plugin rejects a missing
selector, a non-wheel controller, an incompatible estimator, or external MPC.
S62, S63 and 400062 have separate private full-model contracts. The absolute wrapper resolves the saved robot identity; missing configuration is an error, not an implicit S62 selection.

The controller and VR input are separate processes. Input entries are
wheel_arm_abs_vel/quest_velocity_ik.launch and wheel_arm_abs_vel/pico_velocity_ik.launch.
Keep the existing input IP and robot settings appropriate to the deployment;
these commands are not hardware qualification. Do not run legacy and absolute
command writers together.

The real:=false option selects the existing visible MuJoCo infrastructure and
the WBC backend. Real mode defaults to RNEA. RNEA is not the default for the
existing torque-only MuJoCo actuator model. Backend selection is startup-only.

## Isolation and shared safety

- Controller/nodelet, Quest FSM, Mapper, Pinocchio/ProxQP solver, reference owner,
  messages, configs, and scheduling helpers live under this package.
- Eight new ROS messages use the wheel_arm_abs_vel package namespace. Existing
  messages, services, protobuf fields, raw VR packets, and kuavo_msgs/jointCmd
  are unchanged. Old launch graphs never load the new plugin.
- HardwareNodelet is the original dev plugin. Its command conversion,
  cmdTruncation, JointProtectionAction, actuator drivers, stop handling, and
  hardware startup remain authoritative. The new launch explicitly enables the
  original cmd_truncation_enable switch; the legacy launch is unchanged.
- AbsoluteWbcBase derives from the original WbcBase. Torque constraints,
  lower-body and arm-error protection, and task settings call the original
  implementation; the new dynamics/reference terms are separate.
- A new controller instance latches measured posture with zero desired velocity
  before ownership. The reference manager admits fresh, valid atomic q/dq/ddq
  transactions and owns bounded stopping after source/lease loss. Release retains
  the stopped endpoint, without re-entering a legacy trajectory writer.
- Stop and controller teardown notify the existing hardware lifecycle. This
  package adds no motor driver or copy of the hardware safety layer.
- The version-specific full-model reductions and guarded IK intervals are in
  config/model_s62.json, config/model_s63.json and config/model_s400062.json; baseline assets and URDFs are unchanged.

The raw donor's shared RT changes and copied AbsoluteHardwareNode are excluded.
Consequently deferred hardware initialization and donor-only hardware telemetry
are not part of this refactor. Real hardware follows official dev startup.
No claim of identical hardware timing or physical qualification is implied.

## Build and validation

Build in the project's provisioned remote ROS Noetic environment with the same
Drake, Pinocchio and runtime libraries used by the existing controllers. The new
package uses Catkin and contains its own bundled ProxSuite headers.

Do not blindly execute repository-wide ALL targets: the unchanged kuavo_assets
ALL target includes installation/configuration hooks. The validation procedure
builds the kuavo_assets library target explicitly, then the audited old/new
dependency closure. It does not change those hooks or install dependencies.

CTest includes model alignment, QP regression, freshness, bounded stopping,
reference continuity, RNEA and owner lifecycle tests. Python regressions cover
Mapper transactions, held endpoints, input gating and scheduling. Offline ROS
smoke tests use isolated loopback ROS and synthetic feedback. Build, unit tests
and offline smoke results do not qualify real actuators or a physical closed loop.

## S62/S63/400062 on ARM / AGX and AAEON

Core Ultra 7 255H selects the audited `config/runtime_partition_aaeon_255h.json`:
WBC 2/3 (P), IK 5 (P)/FIFO40, Mapper 4 (P), driver reservation 6/7,
sidecars 14/15 (LP-E), background 0/1/8/9/10/11/12/13. The CPU model is read
from `/proc/cpuinfo`; the P/E/LP-E mapping is explicitly configured from the
AAEON .19 CPUID audit. Other x86 models keep `config/runtime_partition.json`.
`KUAVO_RUNTIME_PROFILE` explicitly overrides automatic selection. Re-audit the
CPU map after BIOS core changes; old-kernel frequency/socket fields are unreliable.

AAEON `isolcpus=2,3,7,14,15` accepts CPU5 soft isolation with WARN;
`isolcpus=2,3,5,7` uses hard isolation with INFO. AGX keeps WBC2, IK3/FIFO40,
Mapper4, driver7, background0/1/5/6/8/9/10/11 and prefers sidecars10/11.
Stock AGX `isolcpus=2,7` starts in soft mode; `isolcpus=2,3,7` uses hard mode.
`require_ik_hard_isolation:=true` explicitly opts into rejecting soft isolation.

Use the parent wrapper for both controller and input launches on 255H and AGX:

```bash
bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh wheel_arm_abs_vel load_kuavo_real_velocity_ik.launch
```

It fences newly launched background processes away from control/IK/Mapper cores.
Noncontrol/marker/probe workers use the sidecar pool; the standalone state server
uses a sidecar prefix, and its embedded Python-monitor publisher fences only its
own thread. Existing services and IRQs are outside this launch-local policy.
Legacy/unrelated launches are passed through. No shared C++ scheduling is changed.

All three supported versions require an explicit private model contract and their
own full URDF. IK, Mapper and the private controller reject missing, legacy Drake
arm or cross-version model paths. 400062 uses its independent biped_s400062 model,
including hand joints fixed by model_s400062.json; it is not mapped to S62.

Read-only preflight and offline topology tests:

```bash
python3 src/wheel_arm_abs_vel/scripts/runtime_partition.py --check
python3 -B src/wheel_arm_abs_vel/test/runtime_topology_test.py
python3 -B src/wheel_arm_abs_vel/test/runtime_partition_test.py
```

See [platform runtime adaptation](docs/platform_runtime_adaptation.md) for
the complete CPU tables, automatic model selection and startup procedure,
validation requirements and 500 Hz acceptance boundaries. Offline tests do not
qualify AGX real-time performance. Explicit custom ARM profiles must preserve
WBC2, driver7, distinct roles and the cgroup ceiling. The existing
`KUAVO_RUNTIME_PARTITION=0` bypass is for isolated offline testing.

## S63 full model

Save ROBOT_VERSION=63 once for the actual launch user, then use the private wrapper on both terminals. For an intentional temporary override, pass robot_version:=63 on both the control and Quest/Pico input launches.
S63 selects kuavo_assets/models/biped_s63/urdf/biped_s63.urdf for the IK,
Mapper neutral geometry and WBC/RNEA. config/model_s63.json locks the four
wheels and two head joints, matching the S63 WBC task.info reduction (21D
including world x/y/yaw, 18 controlled joints). S63 rejects missing model
contracts, S62 model paths and Drake arm-only URDFs. Existing S62 model and J3 values are unchanged. S63 now enables the same J3
dynamic/gain profile (kd=18); physical qualification remains separate. S63 also defaults with_ik=false and rejects with_ik=true:
that optional legacy arms_ik_node service loads a separate Drake upper-body
model and is not part of this absolute chain. S62 keeps that service's original
default. Assets and shared hardware sources are unchanged; hardware telemetry,
command snapshots and legacy incremental scheduling remain the target dev behavior.

S63 operating caps were reviewed against S63's own historical teleop limits,
intersected with its complete URDF. Only the six narrower shoulder/upper-arm
intervals need explicit caps; knee uses the S63 full-URDF upper limit
1.44862327915529 (not S62's 1.448 cap). The historical arm-only URDF is
comparison evidence in tests, never a production S63 model source.
The Mapper's 0.45 m human forearm retarget setting remains an explicit,
uncalibrated convention, not a robot link length or hardware calibration.

Regression tests cover full-URDF FK/Jacobians, IK/WBC joint order and FK,
Mapper shoulder/TCP geometry, S63 RNEA and operating-limit intersection.
These offline tests do not qualify real hardware or target-host RT timing.

Runtime CPU partitioning uses the selected private platform profile and must match
the host's online/isolated topology and cgroup cpuset. Offline tests may set
KUAVO_RUNTIME_PARTITION=0; deployment must validate its own topology.

## 400062 startup

Save ROBOT_VERSION=400062 once for the actual launch user. After building and sourcing this workspace, use the same two-terminal workflow as S62/S63. The following explicit arguments are optional temporary overrides, not required on every start:

Start the controller:

    bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh wheel_arm_abs_vel control.launch controller_type:=wheel_arm_abs_vel robot_version:=400062

In a separate terminal, select one input source with the deployment IP settings:

    bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh wheel_arm_abs_vel quest_velocity_ik.launch robot_version:=400062 ip_address:=192.168.5.21
    bash src/wheel_arm_abs_vel/scripts/platform_roslaunch.sh wheel_arm_abs_vel pico_velocity_ik.launch robot_version:=400062

Replace the example Quest/Pico network settings with the actual deployment addresses; see the Chinese guide. Use only one input entry. For MuJoCo, add real:=false to the controller command.
Both controller and input must use the same full robot version. Version 400062
loads its own URDF/configuration and defaults to the configured heiman end
effector. Its 22 finger joints are fixed in the IK model, matching the existing
400062 dynamics task file; fingers are controlled separately through SG100.
The physical IK remains fixed-base 18-axis with a 21-D reference interface.
The S62/S63 J3 gain override stays disabled for 400062; existing 400062 gains
and hardware protection remain authoritative.

Model alignment and launch parsing are offline checks. Physical teleoperation,
hand-driver connectivity and actuator stability require deployment validation.

## 黑漫手独立服务控制

400062 机器端启动后，仿真和实物的 SG100HandBridge 都提供独立手势控制，
无需启动 Quest/Pico 节点。仿真需已开始运行，整机控制不得处于禁用状态。

    rosservice call /sg100/switch_gesture "id: 1
    hand_side: 2"
    rosservice call /sg100/step_gesture "direction: 1
    hand_side: 0"

hand_side 为 0=左手、1=右手、2=双手。服务会选择服务控制并执行目标手势，
默认使用曲线进度 0.0，对应 YAML 第一列 `[0, value]` 的松开扳机姿态；
第二列 `[100, value]` 对应完全捏住扳机。
可在机器启动前设置 /sg100/service_progress（范围 0～1，默认 0）。
单手请求不会给另一只尚未通过服务控制的手发送新目标。
查询当前手势仍使用 /sg100/get_gestures。

服务控制下，VR 扳机和断流事件不会覆盖服务目标。恢复 VR 扳机控制需显式调用：

    rosservice call /sg100/use_vr_control "data: true"

此时要求有效扳机输入，恢复原有 500 ms 断流保护。
/sg100/set_enabled 的人工禁用对两种控制方式都有效；切换手势不自动解除禁用。
服务接收成功表示目标已接受，实际到位请查看 /sg100_hand_state。

全身 IK 的 `quest_velocity_ik.launch` / `pico_velocity_ik.launch` 在 heiman 机型默认启用摸键切换：
摸 X+A 持续 300 ms 切下一个，摸 X+B 持续 300 ms 切上一个；松开后才能再次切换。
Quest 触摸 X+Y 再按 B 属于躯干组合，期间不执行末端手势步进。
首次有效遥操默认进入躯干模式；Quest 用上述组合、Pico 用 Y+RT 切换躯干／手臂模式。X+A 在两种模式下均执行腰部、折叠臂与双臂全身复位，恢复跟随后保留所选模式。Pico 建议先按 Y 再捏 RT，避免先单独捏 RT 控制夹爪。操作、松开后的夹爪接管及复位行为见 [Quest/Pico 操作说明](docs/pico_quest_controls.md)。
同时按下组合键不触发手势切换，避免与手臂解锁冲突。
VR 使用 `/sg100/vr_step_gesture`（同 SG100StepGesture 类型），要求新鲜扳机输入，
切换双手手势并接管 VR 控制；松开扳机使用 YAML 第一列，捏住扳机向第二列插值。
普通 `/sg100/step_gesture` 和 `/sg100/switch_gesture` 仍独立执行服务目标。
切入 VR 会结束上一轮服务控制，清除左右手的服务激活状态，并以当前 VR 手势
作为后续服务步进的起点。再切回服务时，只有新请求指定的手会激活；
`/sg100/use_vr_control false` 本身不会恢复历史服务目标。

## 头部控制

轮臂机器启动时默认拉起 `head_control_node`。它发布 `/robot_head_motion_data`，Quest 节点不再发头部指令。仿真轮臂、实机轮臂和全身 IK 的入口参数相同：

| 参数 | 默认 | 取值 |
| --- | --- | --- |
| head_control_mode | vr_follow | vr_follow / track_hand / fixed |
| head_track_target | both_hands_midpoint | 仅 track_hand：both_hands_midpoint / left_hand / right_hand / active_hand |

`vr_follow` 跟随 Quest 骨骼 `Chest` 相对 `Head` 的姿态。只启动机器、不启动 VR 时没有这组 TF，节点不发布头角，头保持不动。`fixed` 平滑回到正前方（yaw = 0，pitch = 0）。这两个模式不使用 `head_track_target`。

`track_hand` 看机器人自己的手，不依赖 VR。手和头都在 `waist_yaw_link` 下取值：

| head_track_target | 行为 |
| --- | --- |
| both_hands_midpoint | 跟踪双手连线。`config.json` 的 `hand_bias` 默认 0.5，即正中间；0 看左手，1 看右手。一只手更活跃时再按 `velocity_bias` 偏向它 |
| left_hand | 固定跟踪左手 |
| right_hand | 固定跟踪右手 |
| active_hand | 跟踪正在移动的手 |

目标在头部后方时保持当前头角，回到前方至少 0.03 m 后继续跟随。

```bash
roslaunch wheel_arm_abs_vel load_kuavo_mujoco_velocity_ik.launch robot_version:=400062 head_control_mode:=track_hand head_track_target:=both_hands_midpoint
```

实机把入口换成 `load_kuavo_real_velocity_ik.launch`。普通轮臂仿真、实机分别是 `load_kuavo_mujoco_sim_wheel.launch`、`load_kuavo_real_wheel.launch`。不需要这个节点时传 `with_head_control_node:=false`。PICO 轮臂启动会关掉它，头改由 `pico_head_control_node` 发布。

运行中切换：

```bash
rosservice call /head_control_node/set_head_control_mode "{mode: track_hand, fixed_hand: both_hands_midpoint}"
rosservice call /head_control_node/set_head_control_mode "{mode: track_hand, fixed_hand: right_hand}"
rosservice call /head_control_node/set_head_control_mode "{mode: left_hand, fixed_hand: ''}"
rosservice call /head_control_node/set_head_control_mode "{mode: fixed, fixed_hand: ''}"
rosservice call /head_control_node/set_head_control_mode "{mode: vr_follow, fixed_hand: ''}"
```

`mode` 可以是 `vr_follow`、`fixed`、`track_hand`，也可以直接写注视目标。`mode` 为 `track_hand` 时，`fixed_hand` 才是注视目标；留空则保持当前目标。`vr_follow` 和 `fixed` 忽略 `fixed_hand`。非空但无法识别的 `fixed_hand` 会让这次调用失败。返回的 `current_mode` 是实际生效的名字：`vr_follow`、`fixed`、`both_hands_midpoint`、`left_hand`、`right_hand`、`active_hand`。

跟手模式下按 X+A 把胳膊收回时，头会先切到 `fixed`；再按一次恢复刚才的跟手目标。

不启动 VR、只看头跟双手中点时，机器用上面的 `track_hand` 起来之后再跑：

```bash
roslaunch wheel_arm_abs_vel quest_velocity_ik.launch robot_version:=400062
roslaunch noitom_hi5_hand_udp_python test_head_midpoint_arm_traj.launch
```

全身 IK 需要 `quest_velocity_ik.launch` 里的 `absolute_velocity_ik` 才会动手。普通轮臂不用这条，测试会改发 `/kuavo_arm_traj`。

角度限制、平滑和换手阈值在 `src/manipulation_nodes/noitom_hi5_hand_udp_python/scripts/config.json` 的 `head_control`。launch 传入的模式优先于配置文件。
