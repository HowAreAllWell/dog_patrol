uint8 WAITING_TARGET=0
uint8 APPROACHING=1
uint8 ARRIVED=2
uint8 HOLDING=3
uint8 TRACKING=4
uint8 BLOCKED=5

std_msgs/Header header
uint32 target_id
uint8 status
float32 distance_to_target
bool distance_valid
string detail
