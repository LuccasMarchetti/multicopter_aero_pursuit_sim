#!/bin/bash
# Runs the Raspberry Pi study from this checkout: Gazebo + PX4 here (AERO_PURSUER_HOST=remote, the
# pursuer nodes are not started here), the pursuer container already running on the Pi
# (docker/pursuer.sh run <pi>). Detached; log in datasets/study_pi.log.
#   ./ros2_ws/run_study_pi.sh [study.yaml]      (default scenarios/studies/yaw_camera_speed_pi.yaml)
# The container is restarted first (PI_HOST, default luccas@192.168.3.9; PI_HOST= skips it): the
# offboard node keeps the state of the previous PX4, and with a new simulation it never took off
# again (the runs started with the pursuer on the ground).
REPO=$(cd "$(dirname "$0")/.." && pwd)
STUDY=${1:-scenarios/studies/yaw_camera_speed_pi.yaml}
PI_HOST=${PI_HOST-luccas@192.168.3.9}
source /opt/ros/jazzy/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
cd "$REPO"
mkdir -p datasets
if [ -n "$PI_HOST" ]; then
  ssh "$PI_HOST" 'docker restart aero-pursuer >/dev/null' || { echo "restarting aero-pursuer on $PI_HOST failed" >&2; exit 1; }
  sleep 8
fi
AERO_PURSUER_HOST=remote SKIP_PX4_BUILD=1 nohup python3 ros2_ws/run_study.py "$STUDY" --tag pi \
  > datasets/study_pi.log 2>&1 < /dev/null &
echo "study started (pid $!): tail -f $REPO/datasets/study_pi.log"
