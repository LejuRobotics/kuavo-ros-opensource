# 关节扭矩对照表（Kuavo）

本文给出各关节的"名称→数组下标→扭矩限制"对照关系，适用于 Kuavo 系列：版本 4x 与 10004x。

**注意：关节扭矩的限制是绝对值。**

## 腿+手臂+头

### robot_state.proto

- `robot_state.proto` 中 `JointState` 使用 `repeated double` 字段承载位置/速度/力矩，不改变顺序；发送顺序与 `/sensors_data_raw.joint_data` 完全一致（下标即数组索引 0..N-1）。手腕关节为 `zarm_l7_joint` 和 `zarm_r7_joint`。
- 结构定义：

```proto
message JointState {
  repeated double position = 1;
  repeated double velocity = 2;
  repeated double torque = 3;
}
```

### 全身关节下标对照表

| 关节名称 | Kuavo(4x/1004x) 索引 | 关节扭矩上限(\|τ\|max) |
|:---|:---:|:---:|
| leg_l1_joint | 0 | 88.9 |
| leg_l2_joint | 1 | 15.54 |
| leg_l3_joint | 2 | 50.4 |
| leg_l4_joint | 3 | 94.5 |
| leg_l5_joint | 4 | 18.9 |
| leg_l6_joint | 5 | 18.9 |
| leg_l7_joint | 6 | 88.9 |
| leg_r2_joint | 7 | 15.54 |
| leg_r3_joint | 8 | 50.4 |
| leg_r4_joint | 9 | 94.5 |
| leg_r5_joint | 10 | 18.9 |
| leg_r6_joint | 11 | 18.9 |
| zarm_l1_joint | 12 | 46.67 |
| zarm_l2_joint | 13 | 52.5 |
| zarm_l3_joint | 14 | 39.9 |
| zarm_l4_joint | 15 | 52.5 |
| zarm_l5_joint | 16 | 9.87 |
| zarm_l6_joint | 17 | 9.87 |
| zarm_l7_joint | 18 | 9.87 |
| zarm_r1_joint | 19 | 46.67 |
| zarm_r2_joint | 20 | 52.5 |
| zarm_r3_joint | 21 | 39.9 |
| zarm_r4_joint | 22 | 52.5 |
| zarm_r5_joint | 23 | 9.87 |
| zarm_r6_joint | 24 | 9.87 |
| zarm_r7_joint | 25 | 9.87 |
| zhead_1_link | 26 | 140 |
| zhead_2_link | 27 | 140 |


## 末端执行器

### robot_state.proto

- `robot_state.proto` 中 `EndEffectorState` 夹爪和灵巧手会分别发送不同长度的数据。
- 结构定义：

```proto
message EndEffectorState {
  // none: 0
  // lejuclaw: 夹爪 ==> X2
  // qiangnao, revo2, qiangnao_touch: 灵巧手: ==> X12
  repeated double position = 1;
  repeated double velocity = 2;
  repeated double effort = 3;
}
```

### lejuclaw（2 通道）

| 通道下标 | 名称 | 关节扭矩上限(\|τ\|max) |
|:---:|:---|:---:|
| 0 | lejuclaw_left | 3 |
| 1 | lejuclaw_right | 3 |

### 灵巧手（qiangnao / qiangnao_touch / revo2，12 通道）

| 通道下标 | 名称 | 关节扭矩上限(\|τ\|max) |
|:---:|:---|:---:|
| 0 | l_thumb | 100 |
| 1 | l_thumb_aux | 100 |
| 2 | l_index | 100 |
| 3 | l_middle | 100 |
| 4 | l_ring | 100 |
| 5 | l_pinky | 100 |
| 6 | r_thumb | 100 |
| 7 | r_thumb_aux | 100 |
| 8 | r_index | 100 |
| 9 | r_middle | 100 |
| 10 | r_ring | 100 |
| 11 | r_pinky | 100 |
