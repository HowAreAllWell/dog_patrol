# Perception-internal capability health associated with one mission STARTUP.
uint8 NOT_READY=0
uint8 READY=1
uint8 ERROR=2

std_msgs/Header header
string capability
uint8 status
string diagnostic
uint32 observed_startup_state_seq
