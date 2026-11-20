# 底盘接管状态
# 话题: /odom/chassis_takeover_state
# 发布方: 底盘 (底盘驱动 / auto_wheel_bridge_node 或 urobot 底盘服务)
# 订阅方: 运控 (humanoidControllerWheelWbc / MobileManipulatorReferenceManager MPC)
#
# header.frame_id 固定为 "odom", 与里程计使用同一识别头;
# header.stamp 为本帧发布时间。
std_msgs/Header header

# 控制权状态
uint8 STATE_AUTO     = 0   # 自动: 未接管, 底盘正常执行运控下发的指令
uint8 STATE_TAKEOVER = 1   # 接管: 外部节点或人工拥有底盘控制权, 运控不应再驱动底盘
uint8 state

# 接管来源, 仅 STATE_TAKEOVER 时有效; STATE_AUTO 时必须为 SOURCE_NONE
# 未显式赋值时默认为 0 (SOURCE_NONE)
uint8 SOURCE_NONE       = 0
uint8 SOURCE_MANUAL     = 1   # 人工推车 / 底盘手动模式
uint8 SOURCE_JOYSTICK   = 2   # 手柄 / 遥控器
uint8 SOURCE_NAVIGATION = 3   # 导航 / 外部规划器
uint8 SOURCE_VR         = 4   # VR / 遥操作
uint8 SOURCE_EXTERNAL   = 5   # 其它外部节点
uint8 source

# 本次状态切换的时间戳
# STATE_TAKEOVER: 进入接管的时刻; STATE_AUTO: 释放接管、回到自动的时刻
# 心跳消息中该值保持不变, 订阅方据此检测状态边沿
time state_change_time
