# Gazebo free camera (/spectator/command), "Gazebo" view of the GCS.
string FREE='free'       # Pose absoluta (position + yaw/pitch)
string FOLLOW='follow'   # Behind the target, in its heading (distance, elevation, relative azimuth)
string ORBIT='orbit'    # Rotates around the target (orbit_rate)
string LOOK='look'      # Fixed position looking at the target

string mode
string target                     # Frame seguido (follow/orbit/look)
geometry_msgs/Point position      # free/look: position in the world (ENU)
float64 yaw                       # free: ENU heading [rad]
float64 pitch                     # free: + looks down [rad]
float64 distance                  # follow/orbit [m]
float64 azimuth                   # follow/orbit: angle around the target [rad]
float64 elevation                 # follow/orbit: + above the target [rad]
float64 orbit_rate                # orbit [rad/s]
