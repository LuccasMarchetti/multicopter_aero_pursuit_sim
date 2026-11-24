std_msgs/Header header

uint8 IDLE=0
uint8 TRACKING=1
uint8 COASTING=2
uint8 LOST=3
uint8 DISABLED=4
uint8 APPROACH=5          # No depth: approach guided by the line of sight

uint8 mode
string mode_name
string controller
string yaw_mode
string vertical_mode     # los | match_altitude | center_image
float32 target_age      # Age of the last target estimate [s] (-1 if there was never one)
float32 bearing_age     # Age of the last line of sight [s] (-1 if there was never one)
float32 range           # Distance to the target [m]
float32 range_error     # Distance - safety distance [m]
float32 command_speed   # Norm of the commanded velocity [m/s]
string output            # What goes to the PX4: trajectory | attitude | thrust_torque | motors
# Own motion (PX4) and the target's estimated from it (v_target = v_own + estimated v_rel), ENU
float32[3] chaser_velocity   # [m/s] (NaN unknown)
float32 chaser_yaw_rate      # Body z rate (FLU, + counterclockwise) [rad/s]
float32[3] target_velocity   # [m/s] (NaN without a recent 3D estimate)
float32 target_turn_rate     # Course rate of the target horizontal velocity (+ left) [rad/s]
