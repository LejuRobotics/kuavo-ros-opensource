# 最后真实下发到夹爪电机的 PTM 指令（run_ptm_mode / controlPTM 实际参数）
std_msgs/Header header

kuavo_msgs/endEffectorData data # position 为逻辑行程百分比 5~95，速度 %，力矩 A
int32[] kp # 位置环比例增益（安全 PTM 时为 0）
int32[] kd # 位置环微分增益（安全 PTM 时保留）
bool zero_kp # true 表示断流看门狗等安全 PTM
