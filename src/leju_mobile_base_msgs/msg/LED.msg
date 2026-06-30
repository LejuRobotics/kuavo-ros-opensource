# RGB LED strip color control command

# LED mode
uint8 MODE_OFF = 0
uint8 MODE_SOLID = 1
uint8 MODE_BLINK = 2
uint8 MODE_BREATH = 3
uint8 MODE_RAINBOW = 4
uint8 MODE_CHASE = 5
uint8 mode

# RGB color values (0-255)
uint8 red
uint8 green
uint8 blue

# Brightness (0-100%)
uint8 brightness

# Effect speed (0-100, only applicable for animated modes)
uint8 speed

# Blink frequency in Hz (only for MODE_BLINK)
float32 frequency
