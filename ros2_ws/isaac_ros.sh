#!/usr/bin/env bash
# ROS side of the Isaac Sim simulation (isaac_world.launch.xml), run from THIS workspace
# (ros2_ws/install), like sim_manager.sh does for Gazebo: the same `aero build` updates both.
# Called by the GCS (sim_isaac.py) where the GCS runs: natively on the host or inside the GCS
# container (docker mode). Isaac itself (isaac/aero_world.py) runs on the host (isaac/host.sh world).
#
#   isaac_ros.sh start [launch args]    detached; log in $AERO_ISAAC_STATE/logs/ros.log
#   isaac_ros.sh stop
#   isaac_ros.sh status
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE="${AERO_ISAAC_STATE:-$HOME/.local/state/aero/isaac}"
LOG="$STATE/logs/ros.log"
PIDFILE="$STATE/run/ros.pid"
mkdir -p "$STATE/logs" "$STATE/run"

# Same DDS as the Isaac world (isaac/run_world.sh): domain 42, shared-memory profile, no
# discovery range override.
export ROS_DOMAIN_ID="${AERO_ISAAC_DOMAIN:-42}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/../isaac/fastdds_shm.xml"
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY

running() { [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; }

stop() {
  if running; then
    local pid; pid=$(cat "$PIDFILE")
    kill -INT -- "-$pid" 2>/dev/null || true          # the whole launch (own session/group)
    for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -KILL -- "-$pid" 2>/dev/null || true
  fi
  rm -f "$PIDFILE"
  # Leftovers of a launch started elsewhere (old aero-isaac container, terminal).
  pkill -f '__node:=aero_isaac_container' 2>/dev/null || true
}

case "${1:-status}" in
  start)
    shift
    stop
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
    cd "$HOME"
    setsid nohup ros2 launch aero_pursuit isaac_world.launch.xml "$@" > "$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDFILE"
    echo "Isaac ROS side started from $WS/install (log: $LOG)"
    ;;
  stop) stop; echo "Isaac ROS side stopped" ;;
  status) if running; then echo "running (pid $(cat "$PIDFILE"))"; else echo "stopped"; fi ;;
  *) sed -n '2,10p' "$0"; exit 1 ;;
esac
