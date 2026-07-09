# Battery Management System (BMS) information

Header header

# Total pack voltage (V)
float32 pack_voltage

# Total pack current (A)
float32 pack_current

# State of Charge (0-100%)
float32 soc

# State of Health (0-100%)
float32 soh

# Number of cells in the pack
uint8 cell_count

# Individual cell voltages (V)
float32[] cell_voltages

# Individual cell temperatures (Celsius)
float32[] cell_temperatures

# Maximum cell voltage (V)
float32 max_cell_voltage

# Minimum cell voltage (V)
float32 min_cell_voltage

# Maximum cell temperature (Celsius)
float32 max_cell_temp

# Minimum cell temperature (Celsius)
float32 min_cell_temp

# Number of charge cycles
uint32 cycle_count

# BMS status flags
uint32 status_flags

# BMS error codes
uint32 error_code
