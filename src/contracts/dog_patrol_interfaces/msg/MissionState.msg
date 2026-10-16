uint8 STARTUP=0
uint8 PATROL=1
uint8 CONFIRM_TARGET=2
uint8 APPROACH_TARGET=3
uint8 VERIFY_IDENTITY=4
uint8 TRACK_INTRUDER=5
uint8 RECOVER_PATROL=6
uint8 READY=7

std_msgs/Header header
uint32 state_seq
uint8 state
uint32 target_id
string detail

# Successfully verified target, not the target released by navigation recovery.
# Set to the current target only when VERIFY_IDENTITY accepts AUTHORIZED.
# Retained in RECOVER_PATROL, PAUSED+READY and the following PATROL; cleared
# for a new target task or session. TARGET_LOST and EXECUTION_ERROR never add
# a handled target. In PAUSED+READY and pause-recovery RECOVER_PATROL, target_id
# is zero and this field may retain the last successfully verified target.
uint32 handled_target_id
