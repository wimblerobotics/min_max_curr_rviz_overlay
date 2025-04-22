# MinMaxCurr.msg
# Defines the data structure for the Min/Max/Current value display overlay

# Title to display (optional, defaults to topic name if empty)
string title

# Minimum value for the bar range
float32 min

# Maximum value for the bar range
float32 max

# Current value to display
float32 current

# Color for the bar fill (RGBA)
# If alpha is near zero, a default green-yellow-red gradient based on percentage is used.
std_msgs/ColorRGBA current_color

# --- Critical Value Indication ---

# Value threshold for critical state
float32 critical_value

# If true, 'current' < 'critical_value' is critical.
# If false, 'current' > 'critical_value' is critical.
bool critical_if_under true

# Future
string critical_python_function

# How to indicate a critical state
uint8 ANIMATION_NONE=0
uint8 ANIMATION_COLORIZE=1 # Fill background with critical_color
uint8 ANIMATION_FLASH=2    # Flash background with critical_color
uint8 critical_animation_type 0

# Color to use for critical animation (RGBA)
std_msgs/ColorRGBA critical_color

# Number of decimal places to show for min/max/current values
uint8 precision 1

# Future
bool compact
