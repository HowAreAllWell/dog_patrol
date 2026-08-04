uint8 PASSED=0
uint8 NOT_PASSED=1
uint8 ERROR=2
uint8 CANCELLED=3

std_msgs/Header header
uint32 observed_state_seq
uint32 target_id
uint8 result
string provider
string detail
