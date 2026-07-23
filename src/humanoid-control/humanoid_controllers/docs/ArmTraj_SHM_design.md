# ArmTraj SHM 设计文档（人形 / 轮臂统一）

> 关联：轮臂 MR [!3723](https://www.lejuhub.com/highlydynamic/kuavo-ros-control/-/merge_requests/3723) / Issue [#3198](https://www.lejuhub.com/highlydynamic/kuavodevlab/-/issues/3198)  
> 本设计在 !3723 落地后，将通路扩展到双足人形，并抽成可复用组件统一维护。
>
> 飞书版：https://bcn9fa1lvktb.feishu.cn/docx/QcRhdrIwho5enLxogdTcE7uonxs

## 1. 背景与目标

### 1.1 问题

增量 VR（mode2）期间，IK → WBC 手臂轨迹若走 `/kuavo_arm_traj` ROS TCP：

- 长连接 / 调度抖动导致轨迹到达抖动
- WBC 与 IK 同机时不必要的序列化与拷贝

轮臂 !3723 已用 SysV 共享内存（`ArmTrajShmManager`）+ `SetIncrementalArmTrajLink` 解决轮臂侧。

### 1.2 本改动目标

| 目标 | 说明 |
|------|------|
| 功能对齐轮臂 | 双足增量 VR 同样 IK→WBC 走 SHM，topic 仍发供 MPC |
| 统一维护 | 人形 / 轮臂共用 `ArmTrajReceiver`（WBC）与 `ArmTrajWriter`（VR） |
| 生命周期清晰 | SHM 链路由 **mode2 进/出** 显式切，不做 stale 自动回退 |
| 可验证 | 增量测试脚本直读 SHM + HTML 报告，不抢先 setLink |

## 2. 与轮臂 MR !3723 的关系

### 2.1 继承（功能一致性）

| 能力 | !3723（轮臂） | 本 MR |
|------|----------------|-------|
| SysV SHM `key=343434`，布局 `ArmTrajShmData` | ✓ | ✓ 同布局 |
| Writer 写 rad；ROS topic 仍发 deg 供 MPC | ✓ | ✓ |
| WBC `TRANSPORT_SHM` 时退订 `/kuavo_arm_traj` ingest | ✓ | ✓（Receiver） |
| `SetIncrementalArmTrajLink`（NONE / SHM / KUAVO_ARM_TRAJ） | ✓ | ✓ |
| 诊断话题 `/ik_debug/arm_traj_receive/{using_shm,transport}` | ✓ | ✓ |

### 2.2 本 MR 相对 !3723 的必要改动

| 改动 | 必要性 |
|------|--------|
| **双足接入**（`Quest3IkIncrementalROS` + `humanoidController`） | !3723 只覆盖轮臂；双足增量同样存在 TCP jitter，需同源通路 |
| **`ArmTrajReceiver` 统一 WBC 读端** | 避免人形 / 轮臂各写一套 SHM+退订逻辑；退订必须在独立 CallbackQueue spinner，否则 service 线程 shutdown 订阅会闪退 |
| **`ArmTrajWriter` 统一 VR 写端** | 双足 / 轮臂曾复制近百行 `setIncrementalArmTrajLink`；抽组件后只差 service 路径 |
| **去掉 POSIX named sem** | `/dev/shm/sem.*` 曾出现 0 字节坏文件 → `sem_wait` SIGBUS，拖垮 IK / nodelet；改为 **seqlock**（写端 seq=0→写→发 seq） |
| **Writer `invalidate()`** | 进 SHM 前清残留 seq，避免 Receiver 把旧帧当新数据误报 stale |
| **SHM 生命周期 = mode2**（方案 A） | 禁止 startup 预链；测试侧不抢先 `set TRANSPORT_SHM` |
| **stale 不自动 fallback** | !3723 在 0.3s 无更新时自动 `transport=NONE` 恢复 topic。实机/仿真进退 mode 时易误触发，且与「禁止擅自 fallback」冲突。现：**只 ERROR_THROTTLE + hold last**，退链只由 mode2 退出 / `shutdown` 显式发起 |

> 功能一致性指：**增量 mode2 期间 IK→WBC 轨迹通路等价（SHM + 退订 ROS ingest + topic 仍发）**。  
> 不一致处仅在 **退出 / 异常策略**：本 MR 不做 stale 自动回退，由上层 mode 显式管理。

## 3. 架构总览

```
                    mode2 进入                          mode2 退出
                         │                                  │
                         ▼                                  ▼
              ArmTrajWriter.setTransport(SHM)    setTransport(NONE)
                         │                                  │
         ┌───────────────┼───────────────┐                  │
         │ 1) Writer 就绪 / invalidate      │                  │
         │ 2) call WBC set_link(SHM)     │                  │
         └───────────────┬───────────────┘                  │
                         ▼                                  ▼
              ArmTrajReceiver.setLink(SHM)        setLink(NONE)
              - 退订 /kuavo_arm_traj               - 恢复订阅
              - 只 ingest SHM                      - ingest ROS

  IK publish ──writeIfActive──► SysV SHM ──shmThread──► TrajSink
       │                         (rad)                    │
       └──► /kuavo_arm_traj (deg) ──► MPC / 其它节点      ▼
                                                    arm_joint_trajectory_
```

### 3.1 组件职责

| 组件 | 包 | 职责 | 不含 |
|------|-----|------|------|
| `ArmTrajShmManager` | `kuavo_common` | SHM attach / Writer seqlock 写 / Reader 双读快照 / invalidate | ROS、mode 策略 |
| `ArmTrajReceiver` | `humanoid_controllers` | 读 SHM 或 ROS；service 切 transport；独立 spinner 退订 | mode2 |
| `ArmTrajWriter` | `motion_capture_ik` | Writer 生命周期；调 WBC setLink；可选侧车 service；`writeIfActive` | mode2（由 VR 回调调用） |

### 3.2 人形 / 轮臂接线差异（仅路径）

| | 人形（双足） | 轮臂 |
|--|-------------|------|
| Receiver service | `/humanoid_controller/set_incremental_arm_traj_link` | `/humanoid_wheel/set_incremental_arm_traj_link` |
| Writer 侧车 | `/quest3_ik/set_incremental_arm_traj_link` | `/wheel_ik/set_incremental_arm_traj_link` |
| Receiver 宿主 | `humanoidController` | `ControlDataManager` |
| Writer 宿主 | `Quest3IkIncrementalROS` | `WheelQuest3IkIncrementalROS` |
| 默认 transport | `KUAVO_ARM_TRAJ` | `NONE`（仍订阅 ROS，ingest 语义与 KUAVO_ARM_TRAJ 相同：吃 topic） |

## 4. SHM 协议

### 4.1 布局（Python / C++ 均 360 字节，`position` offset=24）

```text
seq:u64 | stamp_nsec:u64 | num_joints:u32 | valid:bool | pad3 | position[14] | velocity[14] | effort[14]
```

- 单位：**弧度**
- `MAX_ARM_JOINTS = 14`
- `SHM_KEY = 343434`

### 4.2 写协议（seqlock）

1. `seq = 0`（Reader 丢弃半写）
2. 写 stamp / joints / pos / vel / effort，`valid=true`
3. `seq = ++write_seq`

### 4.3 读协议

- 双读 `seq`；`seq==0` 或与 `last_read_seq` 相同则无更新
- **不使用** POSIX named semaphore

### 4.4 invalidate

Writer 在切到 SHM 前（或已有 Writer 再进 SHM）清零 payload + `seq=0`，避免旧 seq 被当成新帧。

## 5. Transport 状态机

```text
                  setLink(SHM) / mode2 enter
    NONE/KUAVO_ARM_TRAJ ─────────────────────► SHM
         ▲                                      │
         │         setLink(NONE|KUAVO_ARM_TRAJ) │
         └──────────────────────────────────────┘
                      mode2 exit / Writer.shutdown
```

- **SHM**：只 ingest 共享内存；异步退订 `/kuavo_arm_traj`
- **NONE / KUAVO_ARM_TRAJ**：ingest ROS topic（当前实现二者 ingest 行为相同，消息文案不同）
- **stale**：`age > arm_traj_shm_stale_sec`（默认 0.3，可被 `/vr_ik/arm_traj_shm_stale_timeout_sec` 覆盖）→ 仅打 ERROR，**不改 transport**

## 6. 线程与安全注意

| 点 | 说明 |
|----|------|
| Receiver 退订 | 必须 `CallbackQueue::addCallback` 到 traj spinner；禁止在 service 回调里直接 `Subscriber::shutdown` |
| Writer 并发 | `setTransport`（mode 回调）与 `writeIfActive`（发布线程）共享 `shm_`；当前无锁（!3723 同源竞态）。已知债：可加 mutex，本次未改 |
| shutdown 顺序 | VR 析构先 `arm_traj_writer_.shutdown()` → `setTransport(NONE)` 再释放 Writer |

## 7. 验证与工具

| 工具 | 路径 | 作用 |
|------|------|------|
| 增量平滑测试 | `scripts/increment_test/test_incremental_ik_smoothness.py` | 进 mode2 画圆；**不**抢先 setLink |
| SHM 只读轮询 | `scripts/increment_test/arm_traj_shm_reader.py` | ctypes 对齐布局，与 C++ Reader 并存 |
| 对比图 / HTML | `scripts/increment_test/plot_arm_traj_sensor_cmd.py` | 录 SHM + 中间话题，出报告 |

通过判据（建议）：

1. mode2 期间 `/ik_debug/arm_traj_receive/using_shm ≈ 1`
2. 测试输出 `shm_stats.updates > 0`
3. 退 mode2 后 `using_shm ≈ 0`，无闪退
4. 进退 mode **无** SHM stale 误报刷屏（偶发网络/卡顿告警除外）
5. 手臂跟随画圆；MPC 所需 `/kuavo_arm_traj` 仍有发布

## 8. 文件索引

| 文件 | 角色 |
|------|------|
| `kuavo_common/.../arm_traj_shm.{h,cpp}` | SHM 底层 |
| `humanoid_controllers/.../ArmTrajReceiver.{h,cpp}` | WBC 统一读端 |
| `motion_capture_ik/.../ArmTrajWriter.{h,cpp}` | VR 统一写端 |
| `humanoidController.cpp` | 人形挂 Receiver |
| `ControlDataManager.cpp` | 轮臂挂 Receiver |
| `Quest3IkIncrementalROS.*` | 人形挂 Writer + mode2 钩子 |
| `WheelQuest3IkIncremental*.*` | 轮臂挂 Writer + mode2 钩子 |

## 9. 已知限制

1. Writer `writeIfActive` / `setTransport` 无互斥（见 §6）
2. Receiver 上 `TRANSPORT_NONE` 与 `KUAVO_ARM_TRAJ` ingest 行为相同，语义略糊但 mode2 退出正确
3. 与 !3723 文档/测试脚本中「stale→自动恢复 topic」描述不一致——以本设计 §2.2 为准
4. 未覆盖：实物长时间遥操压测、多 Writer 争用同一 SHM key（设计上单 Writer）

## 10. 更新记录

| 日期 | 改动 | 原因 |
|------|------|------|
| 2026-07-23 | 初稿：统一 Receiver/Writer、mode2 生命周期、禁 stale fallback、去 sem | 双足对齐轮臂并降低维护成本 |
