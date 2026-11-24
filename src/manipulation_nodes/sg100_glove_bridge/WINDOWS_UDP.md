# 拳师 Windows UDP 与全身 IK

全身入口、必要参数、手势流程和 CPU 分配见 [全身手套遥操说明](../../wheel_arm_abs_vel/docs/qianshi_glove.md)。`wheel_arm_abs_vel/quest_windows_udp.launch` 默认启用绝对式全身 IK、Windows 双手、手势和已标定的 `[A,B,C,D,E]` 五指映射。手柄固定手背时无需持续扣扳机；增量式需显式传入 `teleop_mode:=incremental`，并按住对应手柄侧握键（grip）控制位置。机器人型号由实际 `ROBOT_VERSION` 或 `robot_version` 指定。

## 数据与端口

| Windows 输出 | 目的端口 | launch 参数 |
| --- | ---: | --- |
| 右手 IKHand + SendDof | 18002 | `qianshi_udp_right_port` |
| 左手 IKHand + SendDof | 18003 | `qianshi_udp_left_port` |

四路输出同时打开，同一只手共用一个目的端口。目的 IP 为 Linux 接收主机的可达地址；launch 的 `ip_address` 是 Quest 头显地址。接收绑定地址 `qianshi_udp_bind_address` 默认为 `0.0.0.0`。单手模式只监听所选侧端口。Windows 模式不执行串口识别，也不启动 Linux SDK。

旧版四端口参数 `qianshi_udp_*_ik_port`、`qianshi_udp_*_dof_port` 已替换为上表参数，18012/18013 不再监听。启用的左右端口必须不同；端口被占用时节点退出并使 launch 结束。

IKHand 是完整 UTF-8 `<SG100>frame time q0 ... q10</SG100>` 后接实际 CRLF，恰好 13 个数。帧号接受整数和全零小数形式，例如 `4255.00000000`。SendDof 是 200 字节小端 `<IIIId42fq`，Magic=`0x4C313047`、Version=`4`、Flags=`0`；XYZ 单位为米，四元数顺序 WXYZ。

接收器按报文头分流，各手、各协议独立跟踪序号。非法、重复和乱序报文会丢弃。`qianshi_udp_restart_timeout` 默认 2 秒，重启恢复需要新序列的递增确认。每轮有界读取，IKHand 与 SendDof 分别只发布本轮最新有效帧。

## 五指映射与手势

当前手套的 A/B/C/D/E 对应拇指/食指/中指/无名指/小指，F 对应手腕。专用入口使用该映射；通用 `quest_velocity_ik.launch` 的模块列表默认为空，空列表不发布对应侧 fingertips，因此无法识别该侧手势。更换设备或模块配置后应重新核对，可用 `qianshi_udp_diagnostics:=true` 查看各模块 XYZ 与变化量。

独立诊断命令（已有 ROS master，不启动驱动）：

```bash
rosrun sg100_glove_bridge sg100_windows_udp_node.py \
  _right_port:=18002 _left_port:=18003 _publish_real:=false _diagnostics:=true
```

此命令仍发布仿真手指话题。逐指移动确认映射后，通过 `qianshi_udp_left_module_map`、`qianshi_udp_right_module_map` 覆盖五个模块，顺序为拇指、食指、中指、无名指、小指。手势阈值沿用 40/60 mm 捏合迟滞，未在此同步中调整。

## 手指输出与冻结

IKHand 通过 `/sg100_sim_hand_command`（右）与 `/sg100_sim_left_hand_command`（左）输出 11 维关节弧度，真机话题为 `/sg100_hand_command`。本分支使用全身驱动契约：`source_id=1`，收到数据的手启用掩码 `0x07FF`；未收到数据或未选择的手用零掩码禁用，其 11 维零数组仅用于通过消息长度检查。

桥接订阅 `/glove_gesture/interaction_mode`。进入交互时保持此前最后目标，继续接收新的 IKHand 并缓存，SendDof 仍实时发布到 `/qianshi/glove_left/fingertips` 和 `/qianshi/glove_right/fingertips`。若某手还没有可冻结目标，交互期间暂不输出该手的首个 IKHand 目标。退出交互立即输出最新缓存目标，即使此时发送端暂时停发也可恢复。SendDof 断流后不重复发布旧位姿，手势超时继续有效。

`qianshi_publish_real:=false` 关闭真机指令发布，`qianshi_emit_gesture_poses:=false` 关闭手势位姿。单手模式及 `qianshi_glove_gesture_enable:=false` 使用物理手柄按键。

## 离线验证

在此工作区 source 后运行，各测试独立进程执行：

```bash
source devel/setup.bash
python3 src/manipulation_nodes/sg100_glove_bridge/test/test_windows_udp.py
python3 src/wheel_arm_abs_vel/test/qianshi_whole_body_test.py
python3 src/wheel_arm_abs_vel/test/runtime_topology_test.py
```

协议测试使用 ROS 替身和本机 UDP 回环。launch 测试只解析节点及参数；CPU 测试使用模拟拓扑和受控子进程。这些验证不启动真实硬件，不替代全身 IK 真机验收。
