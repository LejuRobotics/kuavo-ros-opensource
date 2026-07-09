# Navigation mode switching goal message

# Navigation mode
uint8 MODE_NORMAL = 0       # Normal navigation mode
uint8 MODE_CAREFUL = 1      # Careful mode - slower, more conservative
uint8 MODE_FAST = 2         # Fast mode - faster navigation
uint8 MODE_TIGHT_SPACE = 3  # Tight space mode - for narrow areas
uint8 MODE_DOCK = 4         # Docking mode - for precise positioning
uint8 navigation_mode

# Mode-specific parameters
# For CAREFUL mode: obstacle avoidance sensitivity (0-100)
# For FAST mode: speed multiplier (0-100)
# For TIGHT_SPACE mode: reduced footprint percentage (0-100)
# For DOCK mode: docking precision level (0-100)
uint8 mode_parameter

# Whether to persist this mode across navigation goals
bool persist_mode
