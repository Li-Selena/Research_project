std_msgs/Header header

uint8 STOP=0
uint8 WORLD_VELOCITY_FIXED_YAW=1
uint8 WORLD_VELOCITY_DYNAMIC_YAW=2
uint8 BODY_VELOCITY_FIXED_YAW=3
uint8 BODY_VELOCITY_DYNAMIC_YAW=4
uint8 WORLD_POSITION_FIXED_YAW=5
uint8 WORLD_POSITION_DYNAMIC_YAW=6

uint32 command_id
uint8 mode
float64 x
float64 y
float64 target_yaw
float64 yaw_rate
