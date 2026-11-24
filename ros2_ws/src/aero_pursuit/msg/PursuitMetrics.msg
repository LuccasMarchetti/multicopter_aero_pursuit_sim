# Pursuit performance metrics (published every active cycle of
# pursuit_controller_node; recorded in the rosbag by session_recorder).
# Totals reset at every (re)engagement in TRACKING.
std_msgs/Header header

string controller
uint8 mode                  # Igual a PursuitStatus.mode

# --- Instantaneous ---
float32 position_error      # |p_perseguidor - p_standoff| [m]
float32 velocity_error      # |v_ref - v_alvo| [m/s]
float32 range_error         # |r| - d [m]
float32 los_error           # Pursuer heading x LOS azimuth [rad] (NaN without a valid heading)
float32 camera_off_axis     # Target offset from the optical axis (vision) [rad] (NaN without vision)
float32 image_azimuth       # Estimate in a body-fixed camera image (TF mount x odometry), + left [rad]
float32 image_elevation     # Same, + up [rad] (NaN without odometry)
float32 acceleration        # |a| comandada [m/s²]
float32 deadband_factor     # 1 = full actuation, 0 = paused by the deadband
float32 thrust_ratio        # Required / maximum x500 thrust (differential flatness)
float32 tilt                # Required tilt [rad]
float32 solve_time_us       # Control law computation time [µs]

# --- Totals since engagement ---
float32 rmse_position       # [m]
float32 rmse_velocity       # [m/s]
float32 rmse_los            # [rad]
float32 rmse_image          # Image off-axis sqrt(az² + el²) of image_azimuth/elevation [rad]
float32 max_position_error  # [m]
float32 effort_l2           # ∫|a|² dt [m²/s³]
float32 effort_l1           # ∫|a| dt [m/s]
float32 engaged_time        # [s]
