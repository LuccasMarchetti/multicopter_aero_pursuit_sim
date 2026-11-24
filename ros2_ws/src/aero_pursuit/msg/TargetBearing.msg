# Line of sight to the target when there is no valid depth (fallback).
std_msgs/Header header
geometry_msgs/Vector3 direction   # Unit vector in the header frame
float32 azimuth                   # [rad] offset in camera_link (FLU): + target to the left
float32 elevation                 # [rad] offset in camera_link (FLU): + target above
float32 score                     # Tracker confidence
float32 angular_size              # [rad] angular size of the ROI: 2·atan(sqrt(w h) / (2 sqrt(fx fy))); NaN if unknown
float32 range_lower_bound         # [m] target beyond the depth camera range: distance > this value; NaN if unknown
