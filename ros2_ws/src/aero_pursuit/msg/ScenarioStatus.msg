# scenario_runner state (/scenario/status), published at 5 Hz and on every transition.
std_msgs/Header header

uint8 IDLE=0          # No scenario loaded
uint8 LOADED=1        # Carregado (drones inseridos), aguardando start
uint8 STAGING=2       # Taking everyone to the start positions
uint8 RUNNING=3       # Running the plans
uint8 PAUSED=4
uint8 FINISHED=5      # All repetitions completed
uint8 ERROR=6

uint8 state
string state_name
string scenario                   # Scenario name
string variant                    # Variant label (studies: controller, detection...)
string run_id                     # <scenario>[_<variant>]_r<k>: identifies the run in the dataset
uint32 run_index                  # 1..repeat
uint32 repeat
float64 elapsed                   # RUNNING time [simulation s]
float64 duration                  # 0: until the plans end
bool engaged                      # Pursuit enabled by the scenario
string pursuer_target             # Target frame of the main pursuer
string detection                  # vision | omniscient
string acquisition                # auto | manual (target_acquisition follows this field)
bool reacquire
string message                    # Last event / error
aero_pursuit/ScenarioDrone[] drones
