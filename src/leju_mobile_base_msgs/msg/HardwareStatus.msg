#Robot Hardware Status

Header header

# ========= 电池 =========
float32 battery_soc            # 电量 [%]
bool   low_battery_alarm       # 低电量报警
float32 battery_temperature    # 电池温度 (℃)
float32 battery_health         # 健康度 [%]

# ========= 驱动器 =========
uint8  driver_status             # 0 NORMAL / 1 ERROR
string[] driver_fault_code       # 驱动器错误码
string[] driver_fault_message    # 驱动器错误信息

# ========= IMU =========
uint8  imu_status                # 0 NORMAL / 1 ERROR
string[] imu_fault_code          # IMU 错误码
string[] imu_fault_message       # IMU 错误信息

# ========= 里程计 =========
float32 all_distance             # 总里程 
uint8  odom_status               # 0 NORMAL / 1 ERROR
string[] odom_fault_code         # 里程计错误码
string[] odom_fault_message      # 里程计错误信息

# ========= LASER =========
uint8  laser_status                # 0 NORMAL / 1 ERROR
string[] laser_fault_code          # LASER 错误码
string[] laser_fault_message       # LASER 错误信息

# ========= CAMERA =========
uint8  camera_status                # 0 NORMAL / 1 ERROR
string[] camera_fault_code          # CAMERA 错误码
string[] camera_fault_message       # CAMERA 错误信息

# ========= ROBOT =========
uint8  robot_status                # 0 NORMAL / 1 ERROR
string[] robot_fault_code          # ROBOT 错误码
string[] robot_fault_message       # ROBOT 错误信息
