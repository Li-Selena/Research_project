std_msgs/Header header

uint8 IDLE=0
uint8 ACTIVE=1
uint8 REACHED=2
uint8 FAULT=3

uint32 command_id
uint8 mode
uint8 state
offset_caster_interfaces/MotionCommand active_command
geometry_msgs/Twist output_twist
float64 position_error
float64 yaw_error
string detail
