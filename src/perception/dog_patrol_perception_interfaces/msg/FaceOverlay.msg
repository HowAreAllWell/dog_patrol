# Lightweight per-frame face detection box for overlay rendering only.
# Published by the face provider whenever a face is detected in a tracked
# target crop; consumed by the tracking visualizer adapter to draw the face
# box on the shared tracking overlay canvas. This is NOT an authorization
# decision; see AuthorizationEvidence for pass/fail results.
#
# QoS: best-effort, volatile, KEEP_LAST(1) -- last-writer-wins stream.
# The stamp/frame metadata identifies the camera crop that produced this box;
# consumers compare it with the frame being rendered, not wall-clock receipt.

std_msgs/Header header
uint32 observed_state_seq
uint32 target_id
uint8 stage
uint32 source_frame_number
bool source_frame_number_available

# Face bounding box in the source (full camera frame) coordinate system.
# width/height < 0 means no face in the latest crop (clear the overlay).
int32 bbox_x
int32 bbox_y
int32 bbox_width
int32 bbox_height

# Mirrors AuthorizationEvidence.result codes for coloring:
# PASSED=0, NOT_PASSED=1, ERROR=2, CANCELLED=3.
uint8 PASSED=0
uint8 NOT_PASSED=1
uint8 ERROR=2
uint8 CANCELLED=3
uint8 result
