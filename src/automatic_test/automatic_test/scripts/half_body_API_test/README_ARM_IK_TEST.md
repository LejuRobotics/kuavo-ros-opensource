## 半身手臂 IK 采集与回放说明

### 脚本概述
- `record_ik_pose.py`：订阅 `/sensors_data_raw` 以保证关节数据有效，同时通过 TF 从 `base_link` → `zarm_l7_end_effector`/`zarm_r7_end_effector` 读取左右手末端位姿，按 JSON Lines 写入样本文件，字段包含：`index`、`q_arm`、`left`、`right`。
- `replay_ik_pose.py`：读取样本（支持“采集文件”或“回放清单”），切外部控制模式(2)，按样本调用 IK 服务；成功则下发 14 关节（度）到 `kuavo_arm_target_poses`，失败则下发全 0。可选 TF 误差评估，执行结束写入汇总行。

### 默认文件位置
- 采集输出（默认）：
  - `${ROS_PACKAGE_PATH}/automatic_test/scripts/half_body_API_test/actions/ik_pose_samples.jsonl`
- 回放清单（默认）：
  - 输入（采集模式下为输入、回放清单模式下为输入+输出）：
    - `${ROS_PACKAGE_PATH}/automatic_test/scripts/half_body_API_test/actions/ik_pose_samples.jsonl`
  - 输出（采集模式下生成的回放清单）：
    - `${ROS_PACKAGE_PATH}/automatic_test/scripts/half_body_API_test/actions/ik_pose_replay_list.jsonl`

说明：当检测到输入 JSONL 含 `ik_success` 字段时，视为“回放清单”，程序会原地更新该文件（输出路径与输入绝对路径一致）。

### 先决条件
- 已启动 ROS Master。
- TF 树可查询到 `base_link` 到左右末端的变换：`zarm_l7_end_effector`、`zarm_r7_end_effector`。
- IK/FK 节点（至少 IK）正常：`/ik/two_arm_hand_pose_cmd_srv`。
- 手臂控制模式服务：`/arm_traj_change_mode`；轨迹执行侧订阅 `kuavo_arm_target_poses`。
- 话题 `/sensors_data_raw` 正在发布（用于关节数据有效性与对齐）。

### 采集：record_ik_pose.py
功能：以 10Hz（默认）采集 1 秒 N 次（默认 10）TF 位姿样本，写入 JSONL。

运行示例：
```bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch
rosrun automatic_test record_ik_pose.py

# 自定义参数
rosrun automatic_test record_ik_pose.py \
  --out $(rospack find automatic_test)/scripts/half_body_API_test/actions/ik_pose_samples.jsonl \
  --samples 20 \
  --rate 20 \
  --base_frame base_link \
  --right_ee_frame zarm_r7_end_effector \
  --left_ee_frame zarm_l7_end_effector
```

JSONL 单行样例：
```json
{"index": 0, "q_arm": [..14个弧度..], "left": {"pos_xyz": [x,y,z], "quat_xyzw": [x,y,z,w]}, "right": {"pos_xyz": [x,y,z], "quat_xyzw": [x,y,z,w]}}
```

说明：
- `index` 自增；若文件已存在，会在末尾继续编号。
- `q_arm` 单位弧度；位姿单位米/四元数。

### 回放：replay_ik_pose.py
两种输入模式：
- 采集文件（不含 `ik_success`）：按 `--ratio` 抽样生成“回放清单”到 `--out`，并逐条执行。
- 回放清单（含 `ik_success`）：直接使用该文件并原地更新（输出路径=输入路径）。

运行示例：
```bash
# 1) 使用采集文件按 30% 抽样并执行，结果写到回放清单
rosrun automatic_test replay_ik_pose.py \
  --file $(rospack find automatic_test)/scripts/half_body_API_test/actions/ik_pose_samples.jsonl \
  --ratio 0.3 \
  --out $(rospack find automatic_test)/scripts/half_body_API_test/actions/ik_pose_replay_list.jsonl \
  --sleep 10

# 2) 直接回放清单，原地更新 ik_success/误差/汇总
rosrun automatic_test replay_ik_pose.py \
  --file $(rospack find automatic_test)/scripts/half_body_API_test/actions/ik_pose_replay_list.jsonl \
  --sleep 10
```

关键参数：
- `--sleep`：每次运动的总等待时间（秒）。其中 `kuavo_arm_target_poses.times = [sleep - wait_time]`，`wait_time` 用于收敛与评估。
- `--wait_time`：在总等待时间中预留的收敛/评估时间（秒，默认 4.0）。
- `--pos_err_thresh`、`--ori_err_thresh_deg`：到达阈值（米/度）。`--check_ori` 开启后同时评估姿态误差。
- `--ik_req_rate`、`--reach_req_rate`：总体判定阈值（IK 成功率、到达成功率），用于最终汇总。

执行流程：
1) 切手臂模式到 2（外部控制）。
2) 逐条样本：
   - 调用 `/ik/two_arm_hand_pose_cmd_srv`。成功：发布返回的 14 关节（单位“度”）到 `kuavo_arm_target_poses`；失败：发布全 0。
   - 等待 `--sleep` 秒（其中 `sleep - wait_time` 为到位时间，额外 `wait_time` 秒留给稳定与评估）。
   - 若为回放清单：将 `ik_success` 原地更新为 `true`。若启用评估，增加 `*_pos_err`、`*_ori_err_deg`、`*_in_limit` 字段。
3) 切回模式 1。
4) 在输出文件首行插入汇总 JSON（包含总数、IK 成功率、到达成功率与是否通过）。

回放清单单行样例（执行后）：
```json
{"index": 12, "ik_success": true, "left": {..}, "right": {..}, "right_pos_err": 0.008, "right_ori_err_deg": 3.2, "right_in_limit": true, "left_pos_err": 0.007, "left_ori_err_deg": 2.8, "left_in_limit": true}
```

注意事项：
- 若输入为回放清单，则输出与输入绝对路径一致（原地更新）。
- 采集依赖 TF；若 TF 瞬时不可用会跳过该样本。
- 真实设备务必确认限位与安全；外部控制模式下谨慎操作。


### 输出产物字段说明

- 采集样本（JSONL 每行一个样本）
```json
{"index": 0, "q_arm": [..14个弧度..],
 "left":  {"pos_xyz": [x,y,z], "quat_xyzw": [x,y,z,w]},
 "right": {"pos_xyz": [x,y,z], "quat_xyzw": [x,y,z,w]}}
```
- 字段含义：
  - **index**: 样本序号（自增，文件存在时继续累加）。
  - **q_arm**: 手臂14个关节角，单位弧度，顺序与 `/sensors_data_raw` 中 12~25 对齐。
  - **left/right.pos_xyz**: 末端位置（m），基于 `base_frame`（默认 `base_link`）。
  - **left/right.quat_xyzw**: 末端姿态四元数（x,y,z,w）。

- 回放清单（JSONL，每行一个待测样本，执行后会追加结果字段并原地更新）
```json
{"index": 12,
 "ik_success": true,
 "left":  {"pos_xyz": [...], "quat_xyzw": [...]},
 "right": {"pos_xyz": [...], "quat_xyzw": [...]},
 "right_pos_err": 0.008, "right_ori_err_deg": 3.2, "right_in_limit": true,
 "left_pos_err": 0.007,  "left_ori_err_deg": 2.8,  "left_in_limit": true}
```
- 结果字段：
  - **ik_success**: IK 是否成功（成功后置 true）。
  - ***_pos_err**: TF 实测与样本目标位置误差（m）。
  - ***_ori_err_deg**: TF 实测与样本目标姿态角度误差（deg）。
  - ***_in_limit**: 是否满足到达阈值（位置 ≤ `--pos_err_thresh`，且在 `--check_ori` 下姿态 ≤ `--ori_err_thresh_deg`）。

- 汇总行（执行结束写入文件首行）
```json
{"summary": {"total": N, "ik_success": K,
 "ik_success_rate": 0.9,
 "reach_success": R, "reach_total": K,
 "reach_success_rate": 0.9,
 "ik_req_rate": 0.9, "reach_req_rate": 0.9,
 "test_pass": true}}
```
- 说明：
  - 仅在 IK 成功样本中统计到达率（reach_total = ik_success）。
  - `test_pass = (ik_success_rate ≥ ik_req_rate) 且 (reach_success_rate ≥ reach_req_rate)`。


### 回放参数说明

- 基本参数
  - **--file**: 输入 JSONL 文件
    - 采集文件模式：不含 `ik_success`，将按比例抽样后生成回放清单。
    - 回放清单模式：含 `ik_success`，原地更新（输出=输入绝对路径）。
  - **--out**: 抽样生成的回放清单输出路径（仅在“采集文件模式”生效）。
  - **--ratio**: 抽样比例 (0,1]，如 0.3 表示随机取 30% 样本。
  - **--index**: 指定单条索引执行（优先于 `--ratio`，越界将报错退出）。
  - **--sleep**: 每条运动总等待时间（s）。内部下发 `times = [sleep - 4.0]`，预留 4s 稳定与评估。

- TF/阈值相关
  - **--base_frame**: 基坐标系（默认 `base_link`）。
  - **--right_ee_frame / --left_ee_frame**: 右/左末端坐标系名（默认 `zarm_r7_end_effector`/`zarm_l7_end_effector`）。
  - **--pos_err_thresh**: 位置误差阈值（m），用于 `*_in_limit` 判定。
  - **--ori_err_thresh_deg**: 姿态误差阈值（deg），用于 `*_in_limit` 判定（需 `--check_ori` 开启）。
  - **--check_ori**: 开启姿态误差判定（默认仅判定位置误差）。

- 通过判定阈值
  - **--ik_req_rate**: IK 成功率门限（默认 0.90）。
  - **--reach_req_rate**: 到达成功率门限（默认 0.90，统计对象为 IK 成功样本）。


