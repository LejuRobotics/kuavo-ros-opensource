# Kuavo SDK vs WebSocket SDK 差异报告

> 生成日期: 2026-06-11 | 更新: 2026-06-11 (完成 22 文件移植)
>
> 对比目录:
> - **D1 (SDK)**: `src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/kuavo/` (43 个文件, ROS rospy 原生通信)
> - **D2 (WebSocket SDK)**: `src/kuavo_humanoid_websocket_sdk/kuavo_humanoid_sdk/kuavo/` (**52 个文件**, 已全部移植, roslibpy + WebSocket 通信, 0 个 rospy 引用)

> **移植状态**: ✅ 22 个 D1 独有文件已全部移植到 D2, D2 现在有 52 个 Python 文件, 零 rospy/rospkg 引用。

---

## 一、总体架构差异

### 1. 通信层完全不同

| 层面 | D1 (SDK) | D2 (WebSocket SDK) |
|------|----------|---------------------|
| 通信库 | `rospy` (原生 ROS) | `roslibpy` (通过 WebSocket) |
| 消息格式 | 强类型 ROS Message 对象 (`msg.field`) | 动态 dict (`msg['field']`) |
| 服务调用 | `rospy.ServiceProxy` 阻塞式 | `roslibpy.Service.call()` 异步式 |
| 参数获取 | `rospy.get_param()` | `roslibpy.Param` |
| 订阅器 | `rospy.Subscriber` | `roslibpy.Topic.subscribe()` |
| 节点管理 | `rospy.init_node()` | 无需初始化 ROS 节点 |

### 2. 文件覆盖度差异

**D1 独有（22 个文件，未移植到 WebSocket SDK）**:

| 文件 | 功能 |
|------|------|
| `core/llm_doubao.py` + `core/llm_doubao_lib/*` | 豆包大模型对话 |
| `core/llm_rtasr_lib/*` | 实时语音识别 & TTS |
| `core/microphone.py` | 麦克风控制 |
| `core/navigation.py` | 导航核心 |
| `core/model_utils/model_utils.py` | 模型工具 |
| `core/ros/camera.py` | 相机 ROS 接口 |
| `core/ros/microphone.py` | 麦克风 ROS 接口 |
| `core/ros/navigation.py` | 导航 ROS 接口 |
| `core/sdk_deprecated.py` | 废弃 API 装饰器 |
| `demo_climbstair.py` | 爬楼演示 |
| `logger_client.py` | 日志客户端 |
| `robot_blockly.py` | Blockly 编程接口 |
| `robot_climbstair.py` | 爬楼控制 |
| `robot_llm.py` | 大模型接口 |
| `robot_microphone.py` | 麦克风接口 |
| `robot_navigation.py` | 导航接口 |
| `robot_speech.py` | 语音接口 |

**D2 独有: 无** — D2 是 D1 的子集。

---

## 二、逐文件逻辑差异（共同文件 25 个）

### 完全一致（仅有 docstring 语言差异）— 4 个文件

| 文件 | 差异程度 |
|------|----------|
| `core/dex_hand_control.py` | 字节一致 |
| `core/leju_claw_control.py` | 字节一致 |
| `core/ros/sat_utils.py` | 字节一致 |
| `core/ros/observation.py` | 仅消息访问方式 (attr vs dict)，逻辑一致 |
| `robot_audio.py` | 仅 docstring 中/英文差异 |
| `robot_controller.py` | 仅 docstring 中/英文差异 |
| `robot_vision.py` | 仅 core 类名差异 |
| `wheel_arm.py` | 仅空白行差异 |

---

### 🔴 `core/core.py` — 差异最大文件

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **Gait-Compatible 状态** | 无 | 新增 `_GAIT_COMPATIBLE_STATES = {'command_pose': 'stance', 'command_pose_world': 'stance'}`，允许命令位姿状态与 stance 共存 |
| **线程安全** | 无 | 新增 `self._state_lock = threading.RLock()` |
| **safe_to_stance / safe_to_trot** | 不存在 | **新增**线程安全包装方法 |
| **is_arm_collision_mode** | 不存在 | **新增**方法，委托给 `self._control.is_arm_collision_mode()` |
| **_do_gait_transition** | 不存在 | **新增**，在独立线程中执行步态切换并加锁 |
| **_on_enter_stance** | 发送命令后立即返回 | 发送后**等待最多 2.0s**（最小 1.0s 稳定时间）确认机器人进入 stance |
| **_on_enter_trot** | 检查 `_is_mpc_mode()`，非 MPC 模式返回 False | **无 MPC 检查**，直接调用 `robot_trot()` |
| **_is_mpc_mode** | **存在** | **不存在**（整个方法缺失） |
| **MPC 模式门控** | 10 个方法（squat, step_control, command_pose 等）有 `if not self._is_mpc_mode(): return False` 检查 | **全部无条件执行** |
| **碰撞检测处理** | 抛 `RuntimeError` | 记录 `SDKLogger.error` 并返回 False |
| **stance 状态检查** | `control_robot_arm_joint_positions` 中 stance 检查**被注释掉** | **强制校验**，非 stance 抛 `RuntimeError` |
| **arm_ctrl_mode 设置** | 服务成功后**手动设置** `self._rb_state._arm_ctrl_mode` | 依赖状态观察更新，**持续重试循环** |
| **_humanoid_gait_changed** | **同步直接调用**状态转换（阻塞回调线程） | 检查 `_GAIT_COMPATIBLE_STATES` 后**在 daemon 线程中异步执行** |
| **control_robot_arm_target_poses** | **存在** | **缺失** |

---

### 🔴 `core/ros/control.py` — 第二大差异文件

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **connect() 方法** | 实际检查发布者订阅者数量 (`get_num_connections()`) | **全部直接返回 True**，无连接检查 |
| **initialize** | **并行多线程**连接检查 | **串行**连接检查 |
| **control_robot_arm_target_poses** | **存在** | **缺失** |
| **control_robot_waist 签名** | `control_robot_waist(self, target_pos: list)` | `control_robot_waist(self, yaw: float)` — **API 不兼容** |
| **enable/disable_head_tracking** | **存在** | **缺失** |
| **arm_ik 高精度模式** | 有约束模式分支，`constraint_mode == 6` 时调用专门的高精度 IK 服务 | **无高精度模式**，所有情况调用标准 IK |
| **pub_dexhand_command** | 正常 | 引用了 `dexhandCommand.POSITION_CONTROL` 但 **`dexhandCommand` 从未导入！** — 🐛 **运行时 Bug** |
| **torso/wheel 发布者** | `__init__` 中创建一次 | **每次调用重新创建** `WebSocketKuavoSDK` 和 `roslibpy.Topic` — 资源浪费 |
| **WheelArmROSControl** | `control_wheel_arm_joint_positions(positions)` — 无 duration 参数，但 service proxy 被注释掉（**不可用**） | `control_wheel_arm_joint_positions(positions, duration=5.0)` — 有 duration 参数，功能完整 |
| **arm_ctrl_mode 响应字段** | `resp.mode` | `response.get('control_mode', 0)` — **字段名不同**，可能是 Bug |
| **KuavoRobotControl.__new__** | 惰性初始化 ROS 节点 | 不初始化节点 |
| **get_motor_param 返回类型** | 返回 `KuavoMotorParam` 对象 | 返回 `type('KuavoMotorParam', (), {...})` 动态匿名类型 |

---

### 🔴 `core/ros/state.py` — 第三大差异文件

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **pitch_limit_enabled** | **存在** | **缺失** |
| **_srv_get_pitch_limit_status** | **存在** | **缺失** |
| **_srv_get_dexhand_gesture_state** | **存在** | **缺失** |
| **共享内存通信** | 支持 `/sensors_data_raw_shm` | **不支持**，仅 `/sensors_data_raw` |
| **Wheel-Arm MPC 话题** | 双足和轮臂均用 `/humanoid_mpc_observation` | 轮臂专用 `/mobile_manipulator_mpc_observation`，双足用 `/humanoid_mpc_observation` |
| **Wheel-Arm 检测** | `kuavo_ros_param.is_wheel_arm_robot()` | `roslibpy.Param(...).get()` 比较 `robot_type == 1` |
| **null 安全** | 简单 `is None` 检查，可能在轮臂场景下 `AttributeError` | 多处 `hasattr` + `is None` 双重保护，更安全 |
| **manipulation_mpc 检查** | 检查 `/enable_manipulation_mpc` 参数和 wheel-arm 类型，不符合时返回 `ERROR` | **跳过所有检查**，直接调用服务 |

---

### 🟡 `robot.py` — 重要差异

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **Init 签名** | `Init(options, log_level)` | `Init(options, log_level, websocket_mode, websocket_host, websocket_port, websocket_timeout)` — 新增 4 个 WebSocket 参数 |
| **Init 流程** | 直接创建 `KuavoROSEnv` → `Init` → `kuavo_core.initialize()` | 设定 `WebSocketKuavoSDK` 配置 → 嵌套 `launch_robot()` 函数（检测启动状态、交互式询问、调用 `robot_start()`/`robot_stand()`） → 然后 `initialize()` |
| **StopRobot** | **不存在** | **新增** `StopRobot()` 方法 |
| **__init__** | 不初始化 ROS env | 额外调用 `KuavoROSEnvWebsocket().Init()` |
| **stance()** | `self._kuavo_core.to_stance()` | `self._kuavo_core.safe_to_stance()` |
| **trot()** | `self._kuavo_core.to_trot()` | `self._kuavo_core.safe_to_trot()` |
| **jump()** | `raise NotImplementedError` | `pass` (静默忽略) |
| **control_head** | 内联实现全量 yaw/pitch 检查逻辑 + logging | 委托给 `self._robot_head.control_head()` |
| **head_tracking** | 直接调用 `self._kuavo_core.enable/disable_head_tracking` | 委托给 `self._robot_head` |
| **control_waist(yaw)** | **不存在** | **新增** `control_waist(self, yaw: float)` |
| **control_waist_pos** | `self._robot_waist.control_waist(joint_positions)` | `self._robot_waist.control_waist_pos(joint_positions=joint_positions)` — 方法名不同 |
| **control_command_pose_world_stamped** | 接受 `TwistStamped` 类型，调用 `control_command_pose_world_stamped` | 接受 dict 或 TwistStamped，提取 x/y/z/yaw 后调用 `control_command_pose_world` — 不同底层方法 |
| **Arm 方法委托** | 大部分直接委托 `self._kuavo_core.*` | 大部分委托 `self._robot_arm.*` |
| **control_arm_joint_positions** | 校验长度 → 按 URDF 限幅裁剪 → 调用 core | 仅校验长度 → 直接委托 robot_arm（裁剪在 arm 层处理） |
| **control_arm_joint_trajectory** | 校验 + 限幅 + 弧度转角度 → 调用 core | 直接委托 robot_arm，不做转换 |
| **control_arm_target_poses** | **存在**（含 `@sdk_deprecated`） | **不存在** |
| **is_arm_collision_mode** | **不存在** | **新增** |
| **arm_fk** | 校验 `q` 长度 → 失败转 `(None, None)` | 直接委托 `self._robot_arm.arm_fk(q)` |

---

### 🟡 `robot_arm.py`

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **弃用装饰器** | 整个类和所有方法标记 `@sdk_deprecated` | 无弃用标记 |
| **control_arm_target_poses** | **存在** | **不存在** |
| **is_arm_collision_mode** | **不存在** | **新增** |
| **arm_ik_free 重复定义** | 🐛 **同一方法定义了两次**（行 217 和行 323），第二次覆盖第一次 — Bug | 仅定义一次 |

---

### 🟡 `robot_waist.py` — API 完全不同

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **control_waist 签名** | `control_waist(self, target_pos: list)` — 接受 list | `control_waist(self, yaw: float)` — 接受单个 float |
| **control_waist 类型检查** | 无 | `isinstance(yaw, (int, float))` → `TypeError` |
| **control_waist 日志** | `_send_log()` 发送开始/完成日志 | 无日志 |
| **_send_log 辅助方法** | **存在** | **不存在** |
| **WAIST_LIMIT_DEG** | 不存在（硬编码 `180`） | 类属性 `self.WAIST_LIMIT_DEG = 180` |
| **getCurrentWaistPos** | **存在**（但有 bug：return 语句缺失） | **不存在** |
| **control_waist_pos** | **不存在** | **新增**，接受 `joint_positions: list`，提取第一个元素作 yaw 后转发完整 list |

---

### 🟡 `robot_head.py`

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **弃用装饰器** | 类和所有方法标记 `@sdk_deprecated` | 无 |
| **_send_log** | 有 | 无 |
| **control_head 日志** | 详细的开始/限制超限/完成日志 | 仅 `SDKLogger.warn` |
| **pitch 限幅方式** | **条件式**：仅在超限时裁剪，容差 ±0.001 | **无条件式**：每次调用都 `min(limit, max(-limit, val))` |
| **pitch 限制精度** | `± math.pi/7.2 ± 0.001` | `± math.pi/7.2`（无容差） |

---

### 🟡 `robot_info.py`

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **参数后端** | `RosParameter` | `RosParamWebsocket` |
| **URDF 缓存** | `__init__` 中缓存 `_humanoid_description` 和 `_arm_joint_limits` | **不缓存** |
| **get_arm_joint_limits** | 返回缓存副本（O(1)） | **每次调用重新解析 URDF XML**（O(n) 开销） |
| **_load_arm_joint_limits_from_urdf** | **存在**（__init__ 中调用一次） | **不存在**（逻辑内联在 get_arm_joint_limits 中） |
| **版本打印** | `__init__` 中打印 "当前为轮臂模型"/"当前为双足模型" | 不打印 |

---

### 🟡 `dexterous_hand.py`

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **get_touch_state** | 🐛 **Bug**: 返回 `self._rb_state.eef_state.state`（抓取状态，而非触觉状态） | ✅ **修复**: 返回 `self._rb_state.dexhand_touch_state` |
| **get_dexhand_gesture_state** | **存在** | **缺失** |
| **make_gesture_sync** | **存在**（两阶段等待循环，5.0s 超时） | **缺失** |
| **time import** | 存在 | 不存在（未使用） |

---

### 🟡 `core/ros/tools.py`

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **轮臂坐标帧映射** | 存在（被注释掉），可将 `base_link→waist_yaw_link`, `odom→base_link` | **不存在** |
| **四元数库** | `tf.transformations.quaternion_matrix` — `[x, y, z, w]` 顺序 | `transforms3d.quaternions.quat2mat` — `[w, x, y, z]` 顺序 — **约定不同** |
| **get_link_pose** | **存在** | **缺失** |
| **参数名** | `time=0.0`（遮蔽模块名） | `time_=0.0`（正确） |
| **服务等待** | `rospy.wait_for_service` 阻塞式 | 直接创建 `roslibpy.Service` 非阻塞式 |

---

### 🟡 `core/ros/vision.py`

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **_get_data_by_id** | 返回**所有**匹配 ID 的检测结果 | 返回**第一个**匹配 ID 的检测结果 |
| **_get_data_by_id 错误处理** | 无效 data_source 静默返回 None | 无效 data_source **抛出 ValueError** |
| **_tf_callback** | 不存在 | **存在**但为死代码（订阅了空回调，填充的 `_transforms` 从未被使用） |
| **未使用导入** | 无 | `numpy` 和 `transforms3d` 已导入但未使用 |
| **size 字段访问** | `detection.size`（属性） | `detection.get('size')`（dict，防缺失） |
| **AprilTag 位姿构造** | 直接赋值 ROS msg 对象 | 显式构建 `AprilTagDetection.Point/Quaternion` + `float()` 转换 |

---

### 🟢 `core/audio.py` — 微小差异

仅底层实现类不同：`Audio()` vs `AudioWebsocket()`。包装类逻辑一致。

---

### 🟢 `core/ros/audio.py` — 微小差异

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **audio_data_publisher** | 有（`Int16MultiArray`） | **无** |
| **publish_audio_chunk** | **存在**（实时音频流 + 增益放大） | **缺失** |

---

### 🟢 `core/ros_env.py`

| 差异项 | D1 | D2 |
|--------|-----|-----|
| **类名** | `KuavoROSEnv` | `KuavoROSEnvWebsocket` |
| **模型路径** | `rospy.get_param('/modelPath')` | `os.environ.get('KUAVO_MODEL_PATH')` |
| **连通性检查** | `rospy.get_master().getPid()` | `self.websocket.client.is_connected` |
| **robot_type 默认值** | `0` | `2` (双足) |
| **check_rosnode_exists** | `subprocess` + `rosnode list` | `roslibpy.Service` + `rosapi/Nodes` |
| **launch_ik_node** | `subprocess` + rosparam | `roslibpy` + 环境变量 `ROBOT_VERSION` |

---

### 🟢 `core/ros/controller.py`

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **类型定义来源** | 从 `kuavo_humanoid_sdk.interfaces.Controller` 导入 | 本地 `@dataclass` 重新定义 |
| **控制器轮询** | `rospy.Timer` 每 1.0s 查询 | 按需查询（缓存为空时才查） |
| **轮臂检测** | `kuavo_ros_param.is_wheel_arm_robot()` | 本地 `_is_wheel_arm_robot()` 方法 |
| **响应字段** | `resp.current_controller` | `response.get('current_controller', '')` |

---

### 🟢 `core/ros/param.py`

| 差异项 | D1 (SDK) | D2 (WebSocket SDK) |
|--------|----------|---------------------|
| **waist_dof 默认值** | `None` | `0` |
| **robot_type / is_legged / is_wheel_arm** | **存在** | **缺失** |
| **joint_names 包含 waist** | ✅ 包含 | ❌ **不包含** waist joints |
| **joint_names robot_version==5** | waist 单独追加 | waist joint names **折叠进 leg_link_names** |
| **end_frames_names** | 根据 `robot_version_major` 和 `arm_dof` 动态选择 | **硬编码固定值** |
| **make_robot_param 缓存 humanoid_description** | ✅ 缓存 | ❌ **不缓存** |
| **make_robot_param None 校验** | 被注释掉 | **激活**（除 end_effector_type 外均抛异常） |
| **模块级实例** | `kuavo_ros_param = RosParameter()` | **无** |

---

### 🟢 `robot_state.py`

仅 core 类不同（`KuavoRobotStateCore` vs `KuavoRobotStateCoreWebsocket`），以及 `pitch_limit_enabled` 属性在 D2 中缺失。

---

### 🟢 `robot_observation.py` / `robot_tool.py`

仅底层 core 类不同，逻辑完全一致。

---

### 🟢 `robot_wheel_control.py` / `leju_claw.py`

逻辑一致。仅有 docstring 纠错（D1 说 "ArmFixed(0)" 但代码用的是 `AutoSwing`，D2 修正了 docstring）。

---

## 三、关键发现总结

### 🐛 已发现的 Bug

| 位置 | 描述 | 严重性 |
|------|------|--------|
| D1 `robot_arm.py:217,323` | `arm_ik_free()` 重复定义，第二个覆盖第一个 | 中 |
| D1 `dexterous_hand.py` | `TouchDexterousHand.get_touch_state()` 返回抓取状态而非触觉状态 | **高** |
| D1 `robot_waist.py` | `getCurrentWaistPos` 有 return 语句缺失 | 中 |
| D1 `robot_wheel_control.py` | docstring 写 "ArmFixed(0)" 但代码用 `AutoSwing` | 低 |
| D2 `core/ros/control.py` | `pub_dexhand_command` 引用未导入的 `dexhandCommand` — 会导致 `NameError` | **高** |
| D2 `core/ros/control.py` | arm_ctrl_mode 读取 `control_mode` 字段但 ROS 服务可能返回 `mode` | 中 |
| D2 `core/ros/control.py` | torso/wheel 发布者每次调用重新创建 WebSocket 连接 | 中（性能） |

### ⚠️ D2 缺失的关键功能（相比 D1）

1. **头跟踪** (`enable_head_tracking` / `disable_head_tracking`) — `control.py` 和 `state.py` 中均缺失
2. **高精度 IK** (`constraint_mode == 6` 分支) — `control.py` 中完全缺失
3. **触觉手势同步** (`make_gesture_sync` / `get_dexhand_gesture_state`) — `dexterous_hand.py` 中缺失
4. **MPC 模式检测** (`_is_mpc_mode`) — `core.py` 中整个方法缺失
5. **实时音频流** (`publish_audio_chunk`) — `audio.py` 中缺失
6. **轮臂坐标帧映射** — `tools.py` 中缺失
7. **共享内存通信支持** — `state.py` 中缺失
8. **LLM / 麦克风 / 导航 / 语音 / Blockly / 爬楼** — 整个模块未移植

### ✅ D2 引入的改进（相比 D1）

1. **线程安全** — `core.py` 中 `_state_lock` 和 `safe_*` 方法
2. **Gait-Compatible 状态** — 允许 command_pose 与 stance 共存
3. **stance 确认等待** — 最多 2s 等待机器人确认 stance 状态
4. **更安全的 null 防护** — `state.py` 中多处 `hasattr` 检查
5. **stance 状态强制校验** — arm joint 操作强制要求 stance 状态
6. **触觉状态修复** — `get_touch_state` 返回正确的触觉状态
7. **交互式启动流程** — `robot.py` 中的 `launch_robot()` 交互式引导
8. **更快的启动** — 无 ROS 节点初始化开销
