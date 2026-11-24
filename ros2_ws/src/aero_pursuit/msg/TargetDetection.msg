# Visual tracker output for each processed frame (dataset / analysis).
# Bounding box in pixels of the original RGB image (independent of the debug scale).
std_msgs/Header header      # frame_id: camera optical frame

uint8 INACTIVE=0
uint8 LOST=1
uint8 CONFIRMING=2
uint8 CONFIRMED=3
uint8 status
float32 score

int32 x
int32 y
int32 width
int32 height
uint32 image_width
uint32 image_height

bool has_depth              # Valid depth points in the ROI
float32 depth               # [m] (NaN without depth)
bool depth_beyond_range     # Whole ROI beyond the depth range: target farther than depth_max_range
float32 depth_max_range     # [m] maximum range of the depth camera (sensor far clip)
float32 depth_valid_fraction  # Fraction of the ROI pixels with depth within range
float32 azimuth             # [rad] offset around Z of camera_link (FLU): + target to the left
float32 elevation           # [rad] offset around Y of camera_link (FLU): + target above
float32 processing_time_ms  # TrackerVit inference time for the frame
string verifier             # TrackVerifier: "" (off) | ok | suspect:<check> | failed:<check>
float32 appearance          # Similarity of the ROI with the initial template [0-1] (NaN if not computed)
