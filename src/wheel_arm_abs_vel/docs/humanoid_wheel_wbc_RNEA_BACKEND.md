# 可选 RNEA 前馈后端（底盘冻结，500 Hz）

## 架构与边界

新轮臂绝对入口默认 `dynamics_backend:=rnea`；该模式不构造或调用 WBC/QP。
两种后端共用现有 IK、最终参考检查、接管/回位/失联状态机、关节指令
发布与硬件保护。准备动作也经过同一个后端接口。不提供在线热切换或
异常自动回退 WBC。RNEA 模式不提供 WBC 接触力服务。

RNEA 使用同一 URDF/关节模型、独立 Pinocchio Data，按关节名核对标量
关节及 q/v 索引。当前只支持 world XY/yaw + 18 个驱动关节；基座 q/v/a
置零，假定静止底盘与平整地面。保留躯干—双臂惯性耦合。不实现接触力控、
移动底盘补偿、倾覆稳定性约束、摩擦辨识，也不更改电机控制模式或 PD。

`gravity`: g(q_measured)

`bias`: RNEA(q_measured, v_filtered, 0)

`full`: RNEA(q_measured, v_filtered, a_admitted)

位置/速度命令保持上游最终参考不变，没有新增软件 PD。加速度系数只作用
于 ddq，不缩小重力。只有 q/dq 与原子参考一致的 ddq 才能被接纳；准备动作
没有可信 ddq 时使用 bias，不对分段线性轨迹或测量速度差分。

速度一阶低通、加速度幅度/jerk 限制、启动动态项 0.2 s 淡入、关节前馈
力矩幅度/变化率限制均只作用于补偿，不降低位置轨迹速度。首条有效力矩
以限幅后的重力初始化；这不是在线后端切换，不能保证任意旧控制器到本后端
的力矩无跳变。参考失效后惯性补偿有界撤出，仍保留可用的重力/速度补偿。

默认前馈限幅为 task.info `torqueLimitsTask` 的 80%，不能当作电机总力矩或
电流上限：硬件 PD 会叠加力矩。配置在 wheel_arm_abs_vel/config/rnea.yaml。
这些是仿真验证起点，不是已验证真机参数。需检查硬件层摩擦/转子惯量补偿
是否重复，以及真实末端负载与模型是否一致。

测量缺失/过期、无效输入输出、异常周期触发锁存故障，通过现有 `/stop_robot`
非阻塞请求停止，并禁止继续发布关节指令；不突然发送零力矩、不复用无限期旧解。
重启前须排查原因。正常 ddq 暂时无效不会触发这种停止。
持续力矩裁剪会记录并告警，不因高速跟踪误差新增 HOLD。

反馈新鲜度使用与测量快照一起保存的 steady_clock 接收时间，不将 MuJoCo
物理时间 sensor_time 与墙上时钟直接相减。共享内存重复读取相同发布标记
不刷新接收时间；ROS 不完整关节数组不进入快照。该检查是本地传输新鲜度，
不是传感器内部采样延迟的估计。

## 专用轮臂绝对入口

此后端只由新 `WheelAbsoluteControllerNodelet` 加载。原有
`humanoid_wheel_wbc` 库、轮臂增量入口和人形入口保持 dev 原样；
不能把新参数传给旧 `load_kuavo_mujoco_sim_wheel.launch` 来启用此功能。

控制器使用 `wheel_arm_abs_vel/load_kuavo_mujoco_velocity_ik.launch`
或 `load_kuavo_real_velocity_ik.launch`，输入使用
`wheel_arm_abs_vel/quest_velocity_ik.launch` 或
`pico_velocity_ik.launch`。新真机入口默认选择 RNEA，MuJoCo 入口默认 WBC（原仿真仅接收力矩）；
`dynamics_backend:=wbc` 选择专用的 `AbsoluteWeightedWbc`。
仿真必须显示窗口；软件构建和离线回放不代表实机或仿真验收。

影子比较：`dynamics_backend:=wbc rnea_shadow:=true`。只有 WBC 输出到执行层；
RNEA 输出仅记录。影子后端使用 WBC 本周期已接纳、已缩放的同一份 ddq，
便于对照，不能将影子模式总耗时视为独立 RNEA 性能。独立 RNEA 的 gain
使用 `/dynamics/rnea/acceleration_gain`；WBC 的原有 gain 参数不变。

## 诊断和验收

`/dynamics/backend_id`：0=WBC，1=RNEA。
`/dynamics/compute_ms`：实际执行后端耗时，不包括影子计算。
`raw_torque` / `applied_torque` / `admitted_acceleration`：原始补偿、最终补偿、
接纳的加速度（后者尚未经过 RNEA 内部 jerk/ramp）。
`acceleration_valid` / `torque_limited` / `rate_limited` / `saturation_seconds`：
参考有效性与保护状态。
`shadow_valid` / `shadow_rnea_torque` / `shadow_difference`：影子结果。
上述短名均在 `/dynamics/` 下。

WBC 的原有 bodyAcc/jointAcc 诊断只在 WBC 模式发布，RNEA 不伪造 QP 解。
历史 `/whole_body_velocity_ik/diagnostics/wbc_applied` 名称保留兼容，其实际语义
为最终动力学后端消费参考；分析时结合 backend_id。

离线测试使用实际 s62 URDF 与任务文件：验证 g、RNEA=M*a+nle、无额外 PD、
基座冻结、关节映射拒绝、无效状态/参考、限幅/变化率、ddq 开关抖动与 12000
周期有限输出。通过该包的 CTest `rnea_backend_test` 执行；验证记录由运行环境单独保存。
容器耗时只供软件比较，不代表真机实时性能。

后续仿真按 gravity → bias → full，静态 → 单臂 → 双臂 → 胸臂协同验证。
比较跟踪误差、力矩变化率、裁剪比例、状态切换峰值及完整周期耗时。
两种后端的力矩不要求相等：WBC 还有误差反馈、软接触变量和解耦模型。
