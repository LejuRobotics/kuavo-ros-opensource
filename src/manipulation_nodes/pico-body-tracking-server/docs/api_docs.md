# 数据格式约定文档

## Socket 数据格式

### 手柄数据

- `controllers.left.primaryButton`: 主按钮 (X)
- `controllers.left.secondaryButton`: 副按钮 (Y)
- `controllers.left.gripButton`: 握把按钮
- `controllers.left.triggerButton`: 扳机按钮
- `controllers.left.gripValue`: 握把值 (0.0 到 1.0)
- `controllers.left.triggerValue`: 扳机值 (0.0 到 1.0)
- `controllers.left.thumbstick`: 摇杆位置 [x, y] (-1.0 到 1.0)
- `controllers.right.primaryButton`: 主按钮 (A)
- `controllers.right.secondaryButton`: 副按钮 (B)
- `controllers.right.gripButton`: 握把按钮
- `controllers.right.triggerButton`: 扳机按钮
- `controllers.right.gripValue`: 握把值 (0.0 到 1.0)
- `controllers.right.triggerValue`: 扳机值 (0.0 到 1.0)
- `controllers.right.thumbstick`: 摇杆位置 [x, y] (-1.0 到 1.0)

## ROS 数据格式

### topics

#### PICO 手柄数据

话题名称: `/pico/joy`

话题描述: 发布 PICO 左右手柄数据

数据类型: `kuavo_msgs/JoySticks`

| 字段 | 类型 | 描述 |
|------|------|------|
| left_x | float32 | 左手柄摇杆X轴位置 |
| left_y | float32 | 左手柄摇杆Y轴位置 |
| left_trigger | float32 | 左手柄扳机值 (0.0 到 1.0) |
| left_grip | float32 | 左手柄握把值 (0.0 到 1.0) |
| left_first_button_pressed | bool | 左手柄主按钮按下状态 (X按键) |
| left_second_button_pressed | bool | 左手柄副按钮按下状态 (Y按键) |
| left_first_button_touched | bool | pico不支持这个参数 |
| left_second_button_touched | bool | pico不支持这个参数 |
| right_x | float32 | 右手柄摇杆X轴位置 |
| right_y | float32 | 右手柄摇杆Y轴位置 |
| right_trigger | float32 | 右手柄扳机值 (0.0 到 1.0) |
| right_grip | float32 | 右手柄握把值 (0.0 到 1.0) |
| right_first_button_pressed | bool | 右手柄主按钮按下状态 (A按键) |
| right_second_button_pressed | bool | 右手柄副按钮按下状态 (B按键) |
| right_first_button_touched | bool | pico不支持这个参数 |
| right_second_button_touched | bool | pico不支持这个参数 |


  ![PICO手柄](../assets/imgs/pico-joy.png)