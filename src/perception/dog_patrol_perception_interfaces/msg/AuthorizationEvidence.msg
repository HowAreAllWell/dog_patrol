uint8 PASSED=0
uint8 NOT_PASSED=1
uint8 ERROR=2
uint8 CANCELLED=3

uint8 INITIAL_FACE=0
uint8 DUAL_FIRST=1
uint8 DUAL_SECOND=2

std_msgs/Header header
uint32 observed_state_seq
uint32 target_id
uint8 stage
uint8 result
string provider
string detail

# Voice providers echo the commanded round. Face providers may leave this zero.
uint32 round_number
