# 电池ID (0:左电池/BAT1, 1:右电池/BAT2)
uint8 battery_id

# 电压 (单位: mV，已从SDK的10mV转换)
uint32 voltage

# 电流 (单位: mA，已从SDK的10mA转换，正数表示充电，负数表示放电)
int32 current

# 剩余容量 (单位: mAh，已从SDK的10mAh转换)
uint32 remaining_capacity

# 满充容量 (单位: mAh，已从SDK的10mAh转换)
uint32 full_capacity

# 剩余电量百分比 (0-100)
uint8 percentage

# 放电循环次数
uint16 cycle_count

# 保护状态标志位 (包含cell1-16和cell17-33的均衡状态)
uint32 protection_flags

# 温度传感器数据 (6个NTC, 单位: 摄氏度)
int16[6] temperatures

# 时间戳
time timestamp
