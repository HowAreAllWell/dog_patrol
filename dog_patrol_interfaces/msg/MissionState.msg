uint8 STARTUP=0
uint8 PATROL=1
uint8 CONFIRM_TARGET=2
uint8 APPROACH_TARGET=3
uint8 VERIFY_IDENTITY=4
uint8 TRACK_INTRUDER=5

std_msgs/Header header
uint32 state_seq
uint8 state
uint32 target_id
bool blocked
string detail
