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
