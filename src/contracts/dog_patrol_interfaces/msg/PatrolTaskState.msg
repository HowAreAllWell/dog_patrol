uint8 TASK_NO_TASK=0
uint8 TASK_RUNNING=1
uint8 TASK_PAUSED=2
uint8 TASK_RETURNING=3

uint8 TASK_OUTCOME_NONE=0
uint8 TASK_OUTCOME_COMPLETED=1
uint8 TASK_OUTCOME_RETURNED=2
uint8 TASK_OUTCOME_TERMINATED=3
uint8 TASK_OUTCOME_FAILED=4

uint8 NAVIGATION_STOP=0
uint8 NAVIGATION_WAYPOINT=1
uint8 NAVIGATION_MISSION=2

std_msgs/Header header
uint32 state_seq
uint8 task_state
uint8 last_task_outcome
uint8 navigation_mode
bool home_pose_valid
geometry_msgs/PoseStamped home_pose
string detail
