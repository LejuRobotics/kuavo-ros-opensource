# 电源板系统状态（对应协议 0x01 系统状态读取应答 System_Runtime_Status_t，共7字节）
time timestamp

# Param1 原始字节: 中断保护状态和电池通信状态
# bit0: 急停中断标志 stop_int_flag（1=中断, 0=正常）
# bit1: RF遥控中断标志 rf_int_flag（1=中断, 0=正常）
# bit2: 控制板类型 board_type（0=足式电源板, 1=轮式电源板）
# bit3: 保留
# bit4: 理想二极管失效标志 ideal_diode_fail_flag（1=失效, 0=有效）
# bit5: 总过流保护标志 cur_ov_protection_flag（1=保护, 0=正常）
# bit6: BAT1通信正常标志 bat1_comm_ok_flag（1=通信正常, 0=通信失败）
# bit7: BAT2通信正常标志 bat2_comm_ok_flag（1=通信正常, 0=通信失败）
uint8 status_byte1

# Param2 原始字节: 电池和电源状态
# bit0: BAT1存在标志 bat1_exists_flag（1=存在, 0=不存在）
# bit1: BAT2存在标志 bat2_exists_flag（1=存在, 0=不存在）
# bit2: BAT1低电量标志 bat1_low_power_flag（1=低电, 0=正常）
# bit3: BAT2低电量标志 bat2_low_power_flag（1=低电, 0=正常）
# bit4: 充电状态标志 charging_flag（1=充电中, 0=未充电）
# bit5: 12V电源3故障标志 stat3_12v_Fail_flag（1=故障, 0=正常）
# bit6: 19V电源1故障标志 stat1_19v_Fail_flag（1=故障, 0=正常）
# bit7: 24V电源故障标志 stat_24v_Fail_flag（1=故障, 0=正常）
uint8 status_byte2

# Param3 原始字节: 输出控制状态
# bit0: 保留
# bit1: ARM输出使能状态（1=启用, 0=禁用）
# bit2: LEG输出使能状态（1=启用, 0=禁用）
# bit3: 19V电源输出使能状态（1=启用, 0=禁用）
# bit4: 12V电源1输出使能状态（1=启用, 0=禁用）
# bit5: 12V电源2输出使能状态（1=启用, 0=禁用）
# bit6: 12V电源3输出使能状态（1=启用, 0=禁用）
# bit7: 24V电源输出使能状态（1=启用, 0=禁用）
uint8 status_byte3

# Param4: NTC温度（uint8, 单位 °C）
uint8 ntc_temperature

# Param5: 充电电压（uint8, 单位 V，未使用）
uint8 charge_voltage

# Param6: BAT1电池测量电压（uint8, 单位 V）
uint8 bat1_voltage

# Param7: BAT2电池测量电压（uint8, 单位 V）
uint8 bat2_voltage

# ===== Param1 拆解标志 =====
bool stop_int               # 急停中断
bool rf_int                 # RF遥控中断
bool board_is_wheel         # 控制板类型: true=轮式电源板, false=足式电源板
bool ideal_diode_fail       # 理想二极管失效
bool cur_ov_protection      # 总过流保护
bool bat1_comm_ok           # BAT1通信正常
bool bat2_comm_ok           # BAT2通信正常

# ===== Param2 拆解标志 =====
bool bat1_exists            # BAT1存在
bool bat2_exists            # BAT2存在
bool bat1_low_power         # BAT1低电量
bool bat2_low_power         # BAT2低电量
bool charging               # 充电中
bool fail_12v               # 12V电源3故障
bool fail_19v               # 19V电源1故障
bool fail_24v               # 24V电源故障

# ===== Param3 拆解标志 =====
bool arm_en                 # ARM输出使能
bool leg_en                 # LEG输出使能
bool out_19v_en             # 19V电源输出使能
bool out1_12v_en            # 12V电源1输出使能
bool out2_12v_en            # 12V电源2输出使能
bool out3_12v_en            # 12V电源3输出使能
bool out_24v_en             # 24V电源输出使能
