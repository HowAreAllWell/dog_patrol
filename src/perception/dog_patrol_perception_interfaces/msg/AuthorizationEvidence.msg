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

# Face providers return identity metadata and the selected tracking-crop photo.
# Voice providers leave these fields empty/zero.
string person_id
float32 similarity
# Face similarity state: "measured", "no_face_detected",
# "measurement_error", or "not_applicable".
string similarity_status
string image_url
