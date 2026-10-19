uint8 INITIAL_FACE=0
uint8 DUAL_FIRST=1
uint8 DUAL_SECOND=2
uint8 CANCEL=3

std_msgs/Header header
uint32 observed_state_seq
uint32 target_id
uint8 stage

# One-based voice round; zero preserves legacy first/second-stage commands.
uint32 round_number
