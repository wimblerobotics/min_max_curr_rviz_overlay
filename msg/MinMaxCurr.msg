# Animation Type Constants
uint8 ANIMATION_NONE=0
uint8 ANIMATION_COLORIZE=1
uint8 ANIMATION_FLASH=2

float32 min
float32 max
float32 current
float32 critical_value
bool critical_if_under
uint8 critical_animation_type
string critical_python_function
string title
bool compact
std_msgs/ColorRGBA current_color
std_msgs/ColorRGBA critical_color

# Field for selecting animation type
