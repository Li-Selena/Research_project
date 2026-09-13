# Target timestamp and configured FLU output frame, normally arm_base_link.
std_msgs/Header header
# valid=false means x/y/z are zero and distance=-1.
bool valid
# predicted=true marks a short target-loss prediction without a new measurement.
bool predicted
# FLU coordinates and Euclidean distance in metres.
float32 x
float32 y
float32 z
float32 distance
