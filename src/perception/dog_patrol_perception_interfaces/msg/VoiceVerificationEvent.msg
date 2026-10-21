# Lifecycle points used to retain full-scene evidence photographs.
uint8 WAIT_STARTED=0
uint8 PASSED=1
uint8 NOT_PASSED=2

# Actual window/result time, before hardware cleanup; photo names use the
# selected camera frame's source timestamp instead.
std_msgs/Header header
uint32 observed_state_seq
uint32 target_id
uint32 round_number
uint8 event

# Business score from distinct matching microphone channels at the decision:
# 0 -> 0%, 1 -> 50%, 2 -> 75%, 3 -> 90%, 4..6 -> 100%.
# WAIT_STARTED has no measurement (confidence_valid=false), not a measured zero.
bool confidence_valid
uint8 vote_count
uint8 confidence_percent
uint8[] matching_channels
