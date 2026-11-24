#!/bin/bash
# Runs a scenario of <repo>/scenarios from scratch (headless simulation through sim_manager).
#   ./run_scenario.sh figure8_basic [launch args]      e.g. record:=false
#   GUI=1 ./run_scenario.sh chained_laps               Gazebo with a window
# In a simulation already open, prefer the GCS (Scenarios tab) or:
#   ros2 service call /scenario_runner/command aero_pursuit/srv/ScenarioCommand "{command: start, path: figure8_basic.yaml}"
set -e
WS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ $# -ge 1 ] || { sed -n 2,6p "$0" | sed 's/^# \{0,1\}//'; exit 1; }
export SCENARIO=$1
shift
export SKIP_PX4_BUILD=${SKIP_PX4_BUILD:-1} FOXGLOVE=${FOXGLOVE:-0}
[ -n "${GUI:-}" ] || export GZ_HEADLESS=1
"$WS_PATH/sim_manager.sh" stop > /dev/null 2>&1 || true
exec "$WS_PATH/sim_manager.sh" start record:=true joystick:=false "$@"
