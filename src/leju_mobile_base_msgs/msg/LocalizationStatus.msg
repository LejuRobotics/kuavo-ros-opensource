# Localization status reported by the chassis
# Published on /move_base/localization_status.

Header header

# Current localization state
uint8 STATUS_UNKNOWN = 0          # State is unknown or has not been reported yet
uint8 STATUS_DISABLED = 1         # Localization module is stopped
uint8 STATUS_INITIALIZING = 2     # Localization is starting or converging
uint8 STATUS_LOCALIZED = 3        # Localization is valid and available for navigation
uint8 STATUS_LOST = 4             # Localization was valid but is now lost
uint8 STATUS_ERROR = 5            # Localization module encountered an error
uint8 status

# Localization confidence in the range [0.0, 1.0]; -1.0 means unavailable
float32 confidence

# Identifier of the map currently used for localization; empty if unavailable
string map_id

# Error information; error_code is 0 and error_msg is empty when there is no error
uint32 error_code
string error_msg
