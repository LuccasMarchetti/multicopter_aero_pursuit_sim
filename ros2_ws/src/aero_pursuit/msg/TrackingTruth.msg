# Visual tracker x ground truth (/acquisition/truth), on every tracker frame.
# Measures the perception quality independently of the control: is the tracker on the
# right target? With swarms/decoys, `tracked_identity` reveals identity switches.
std_msgs/Header header

string target_frame               # Target designated by the scenario
bool target_visible               # Target projection inside the image and in front of the camera
int32 gt_x                        # Projected target box (pixels of the original image)
int32 gt_y
int32 gt_width
int32 gt_height
float32 target_distance           # [m]
uint8 tracker_status              # TargetDetection.status
float32 iou                       # Tracker box x projected target box (0 without tracking)
float32 center_error_px           # Distance between the centers [px] (NaN without tracking)
string tracked_identity           # Vehicle with the largest overlap with the tracker box ("" none)
bool on_target                    # tracked_identity == target_frame
uint32 visible_count              # Vehicles visible in the image (target + decoys)
uint32 acquisitions               # Automatic tracker initializations in this run
uint32 identity_switches          # tracked_identity changes between vehicles in this run
