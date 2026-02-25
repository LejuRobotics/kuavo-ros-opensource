# Robot battery information

Header header

# Battery voltage (V)
float32 voltage

# Battery current (A)
float32 current

# Battery charge percentage (0-100)
float32 percentage

# Battery temperature (Celsius)
float32 temperature

# Battery status
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_CHARGING = 1
uint8 STATUS_DISCHARGING = 2
uint8 STATUS_NOT_CHARGING = 3
uint8 STATUS_FULL = 4
uint8 status

# Estimated remaining time (seconds)
# -1 if unknown
int32 time_remaining

# Battery health (0-100)
uint8 health
