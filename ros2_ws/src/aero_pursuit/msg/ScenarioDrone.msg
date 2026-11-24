# State of a drone controlled by the scenario (ScenarioStatus.drones).
string name                       # Model in Gazebo = TF frame = topic namespace
string model                      # Catalog key (x3, x4, tiltrotor, x500_depth)
string role                       # target | pursuer
float32 radius                    # Bounding radius [m] (box projection in the ground truth)
string leg                        # Current leg of the plan
uint32 legs_started
uint32 leg_iteration              # Current lap of the leg
bool in_transit                   # Going to the start of the leg
bool finished                     # Plan completed
geometry_msgs/Point position      # Ground truth (mundo, ENU)
geometry_msgs/Vector3 velocity
geometry_msgs/Point reference     # Current reference (NaN if velocity only)
