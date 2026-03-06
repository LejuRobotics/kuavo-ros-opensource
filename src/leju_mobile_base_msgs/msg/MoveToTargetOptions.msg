# MoveToTargetOptions.msg
# 可选的导航参数

# 避障设置
# 如果avoid_enabled为false，则完全关闭避障
bool avoid_enabled
float32 avoid_distance

# 速度参数
float32 linear_velocity
float32 angular_velocity

# 精度阈值
float32 position_threshold  # 位置阈值（米）
float32 angle_threshold     # 角度阈值（弧度）

# 运动限制
bool allow_rotation         # 是否允许旋转
