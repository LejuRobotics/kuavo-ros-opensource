# 手臂力反馈数据协议文档

## 数据来源
- **ROS 话题**: `/state_estimate/arm_contact_force`
- **消息类型**: `std_msgs/Float64MultiArray`
- **数据格式**: `[左末端Fx, Fy, Fz, 右末端Fx, Fy, Fz]` (6个double值，单位：N)
- **说明**: 只有力数据，没有力矩数据

## Protobuf 数据结构

### ArmForceFeedback 消息
```protobuf
message ArmForceFeedback {
  repeated double max_force = 1;  // 各手末端 xyz 中最大的力，归一化到 0-255
}
```

**数组索引说明**:
- `max_force[0]`: 左手末端最大力（归一化值 0-255）
- `max_force[1]`: 右手末端最大力（归一化值 0-255）
- 可扩展：未来可添加更多手臂数据

### RobotState 扩展
```protobuf
message RobotState {
  Header header = 1;
  EndEffectorState ee_state = 2;
  JointState joint_state = 3;
  ItemMassForceResponse item_mass_force_response = 4;
  optional ArmForceFeedback arm_force_feedback = 5;  // 可选字段：末端力反馈数据
}
```

## 数据处理说明

### 机器人端处理
1. 接收 6 个力数据：`[左手末端Fx, Fy, Fz, 右手末端Fx, Fy, Fz]`
2. 计算每臂 xyz 中最大的力（取绝对值）：
   - `left_max_force = max(|Fx|, |Fy|, |Fz|)`
   - `right_max_force = max(|Fx|, |Fy|, |Fz|)`
3. 归一化到 0-255（线性映射）：
   - `normalized_force = (max_force / 19.6) * 255.0`
   - 19.6N ≈ 2kg 物体重量
   - 超过 19.6N 的力会被限制在 255
4. 存入数组：`max_force = [left_normalized, right_normalized]`

### VR 端使用
- `arm_force_feedback.max_force[0]`: 左手末端振动强度（0-255）
- `arm_force_feedback.max_force[1]`: 右手末端振动强度（0-255）
- 可直接映射到手柄震动：
  - 0：无震动
  - 255：最大震动（约等于 2kg 物体重量对应的力）
- 该字段为可选字段，使用时需检查 `HasField('arm_force_feedback')`

## 数据示例

### 输入数据（/state_estimate/arm_contact_force）
```
[5.0, -2.5, 8.0, 3.2, -1.0, 4.5]
```
- 左手末端：Fx=5.0N, Fy=-2.5N, Fz=8.0N
- 右手末端：Fx=3.2N, Fy=-1.0N, Fz=4.5N

### 输出数据（ArmForceFeedback.max_force）
```
max_force = [104.08, 58.52]
```
- `max_force[0] = 104.08`: 左手末端 max(|5.0|, |2.5|, |8.0|) = 8.0N → (8.0/19.6)*255 ≈ 104.08
- `max_force[1] = 58.52`: 右手末端 max(|3.2|, |1.0|, |4.5|) = 4.5N → (4.5/19.6)*255 ≈ 58.52


