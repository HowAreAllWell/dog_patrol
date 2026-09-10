uint8 STARTUP=0
uint8 PATROL=1
uint8 CONFIRM_TARGET=2
uint8 APPROACH_TARGET=3
uint8 VERIFY_IDENTITY=4
uint8 TRACK_INTRUDER=5
uint8 RECOVER_PATROL=6

std_msgs/Header header
uint32 state_seq
uint8 state
uint32 target_id
string detail

# Business disposition, not the target released by navigation recovery.
# Nonzero only after AUTHORIZED or final TARGET_LOST in TRACK_INTRUDER.
# Retained in RECOVER_PATROL and the following PATROL; cleared for a new
# target task or session. Failures before completion must leave this zero.
uint32 handled_target_id
