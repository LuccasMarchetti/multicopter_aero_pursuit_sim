#!/usr/bin/env bash
# Isaac Sim side of this computer (docs/ISAAC_SIM.md): the Isaac world (aero_world.py: Isaac Sim +
# Pegasus + PX4 SITL) on the host. The ROS side (isaac_world.launch.xml) runs where the GCS runs,
# from the workspace `aero build` compiles (ros2_ws/isaac_ros.sh), like the Gazebo launch.
# Called by the GCS (sim_isaac.py) directly when it runs natively, or through
# isaac/host_agent.py when it runs in a container; also by `aero isaac ...`.
#
#   host.sh world [args]                       Isaac world only (what the GCS uses)
#   host.sh up [world args] [-- launch args]   world + ROS side from ros2_ws/install (native install)
#   host.sh ros [launch args]                  ROS side only (ros2_ws/isaac_ros.sh)
#   host.sh down                               stop both sides
#   host.sh status                             processes, GPU, real-time factor
#   host.sh px4 <command...>                   PX4 shell command (e.g. commander status)
#   host.sh agent start|stop|status            host agent for a GCS in a container
#
# State (logs, agent socket, video sockets shared with the containers): AERO_ISAAC_STATE,
# default ~/.local/state/aero/isaac.
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO="$(dirname "$HERE")"
STAGE="${AERO_ISAAC_STATE:-$HOME/.local/state/aero/isaac}"   # logs/, run/
DOMAIN="${ROS_DOMAIN_ID:-42}"
ROS_SIDE="$REPO/ros2_ws/isaac_ros.sh"
AGENT_SOCK="$STAGE/run/host.sock"
AGENT_PID="$STAGE/run/host_agent.pid"
PX4_BUILD="$HOME/PX4-Autopilot/build/px4_sitl_default"
mkdir -p "$STAGE/logs" "$STAGE/run" "$REPO/datasets"


start_world() {
  cd "$STAGE"
  ROS_DOMAIN_ID=$DOMAIN setsid nohup "$HERE/run_world.sh" "$@" > logs/world.log 2>&1 < /dev/null &
  echo "Isaac world starting (log: $STAGE/logs/world.log)"
}

start_ros() {
  AERO_ISAAC_STATE="$STAGE" bash "$ROS_SIDE" start "$@"
}

stop_all() {
  AERO_ISAAC_STATE="$STAGE" bash "$ROS_SIDE" stop >/dev/null 2>&1 || true
  docker rm -f aero-isaac >/dev/null 2>&1 || true   # container of the old layout, if any
  bash "$HERE/kill_world.sh"
  echo "Isaac world and ROS side stopped"
}

stop_agent() {
  if [ -f "$AGENT_PID" ]; then kill "$(cat "$AGENT_PID")" 2>/dev/null || true; rm -f "$AGENT_PID"; fi
  rm -f "$AGENT_SOCK"
}

cmd="${1:-}"; shift || true
case "$cmd" in
  up)
    world_args=(); ros_args=()
    while [ $# -gt 0 ] && [ "$1" != "--" ]; do world_args+=("$1"); shift; done
    [ "${1:-}" = "--" ] && shift; ros_args=("$@")
    start_world "${world_args[@]}"
    start_ros "${ros_args[@]}"
    ;;
  world) start_world "$@" ;;
  ros) start_ros "$@" ;;
  down) stop_all ;;
  status)
    pgrep -af '[a]ero_world.py|[p]x4_sitl_default/bin/px4' | cut -c1-120 || true
    AERO_ISAAC_STATE="$STAGE" bash "$ROS_SIDE" status
    nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader
    grep -a 'RTF' "$STAGE/logs/world.log" 2>/dev/null | tail -1 || true
    ;;
  build)
    echo "The ROS side of Isaac runs from the workspace (ros2_ws/install): build it with \`aero build\`." >&2
    echo "(The aero-isaac image is not used any more.)" >&2
    exit 1
    ;;
  px4)
    [ $# -gt 0 ] || { echo "usage: $0 px4 <command> [args]"; exit 1; }
    pgrep -f "px4_sitl_default/bin/[p]x4" >/dev/null || { echo "PX4 is not running" >&2; exit 1; }
    mod="$1"; shift
    cd "$PX4_BUILD" && exec "./bin/px4-$mod" --instance 0 "$@"
    ;;
  agent)
    case "${1:-status}" in
      start)
        stop_agent
        setsid nohup python3 "$HERE/host_agent.py" "$AGENT_SOCK" > "$STAGE/logs/host_agent.log" 2>&1 < /dev/null &
        echo $! > "$AGENT_PID"
        echo "host agent: $AGENT_SOCK"
        ;;
      stop) stop_agent ;;
      *) if [ -f "$AGENT_PID" ] && kill -0 "$(cat "$AGENT_PID")" 2>/dev/null; then
           echo "agent: running (pid $(cat "$AGENT_PID"))"; else echo "agent: stopped"; fi ;;
    esac
    ;;
  *) sed -n '2,17p' "$0"; exit 1 ;;
esac
