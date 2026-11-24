#!/bin/bash
# Simulation manager that does not tie the processes to a terminal (alternative to
# start_simulation.sh/tmux). Each process runs in the background, in its own process group,
# with its output in a separate log file:
#
#   <repo>/logs/<date_time>/{gazebo_px4,xrce_agent,foxglove,aero_pursuit}.log
#   <repo>/logs/<date_time>/ros/        (per-process ROS 2 logs, ROS_LOG_DIR)
#   <repo>/logs/latest -> current run
#
# Usage:
#   ./sim_manager.sh start [launch args]       e.g. start record:=false
#   ./sim_manager.sh stop                      SIGINT -> SIGTERM -> SIGKILL (only the registered groups)
#   ./sim_manager.sh status
#   ./sim_manager.sh logs                      list the logs of the current run
#   ./sim_manager.sh logs <process> [filter]   tail -f (e.g. logs aero_pursuit pursuit_controller_node)
#   ./sim_manager.sh px4 "<command>"           send a command to the PX4 console (e.g. "commander status")
#   ./sim_manager.sh view                      read-only tmux: one pane per log (closing it does not stop the simulation)
#
# Variables: SCENARIO (file in <repo>/scenarios, run at start), WORLD_NAME (lawn), LOG_LEVEL (warn),
#            XRCE_VERBOSE (3), SKIP_PX4_BUILD, GZ_SIM_VERBOSE, GZ_RENDER_API, GZ_HEADLESS,
#            FOXGLOVE=0 (no bridge), FOXGLOVE_ALL_TOPICS=1 (raw images), AERO_BASE_DRONES (target
#            drones of the base world, default "X4:x4:5:-2:1").

set -u
WS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Logs e datasets ficam na raiz do repositório (vale para qualquer pasta de clone).
REPO_ROOT="$(dirname "$WS_PATH")"
LOG_ROOT=${LOG_ROOT:-$REPO_ROOT/logs}
WORLD_NAME=${WORLD_NAME:-lawn}
LOG_LEVEL=${LOG_LEVEL:-warn}
XRCE_VERBOSE=${XRCE_VERBOSE:-3}
PROCS=(gazebo_px4 xrce_agent foxglove aero_pursuit)

# shellcheck source=scenario_env.sh
source "$WS_PATH/scenario_env.sh"

run_dir() {
  readlink -f "$LOG_ROOT/latest" 2>/dev/null
}

# latest -> <dir>. Se latest for um diretório comum (criado por versões antigas do
# launch_gz_and_drones.sh), o `ln -sfn` criaria o link DENTRO dele e os .pid iriam
# para lugares diferentes: o stop não achava o px4_stdin_holder e o `sleep` ficava órfão.
link_latest() {  # link_latest <dir>
  local latest="$LOG_ROOT/latest"
  if [ -d "$latest" ] && [ ! -L "$latest" ]; then
    mv "$latest" "$LOG_ROOT/legacy_$(date -r "$latest" +%Y%m%d_%H%M%S)"
  fi
  ln -sfn "$(basename "$1")" "$latest"   # relativo: vale no host e no contêiner
}

group_alive() {  # group_alive <pgid>: algum processo do grupo ainda existe
  kill -0 -- "-$1" 2>/dev/null
}

is_running() {  # is_running <processo>
  local pidfile
  pidfile="$(run_dir)/$1.pid"
  [ -f "$pidfile" ] && group_alive "$(cat "$pidfile")"
}

spawn() {  # spawn <processo> <comando> [stdin]
  local name=$1 cmd=$2 input=${3:-/dev/null} dir
  dir=$(run_dir)
  # setsid: grupo próprio => stop sinaliza o processo e todos os seus filhos.
  setsid bash -c "$cmd" > "$dir/$name.log" 2>&1 < "$input" &
  echo $! > "$dir/$name.pid"
  echo "  $name (pid $!) -> $dir/$name.log"
}

cmd_start() {
  local p
  for p in "${PROCS[@]}"; do
    if is_running "$p"; then
      echo "Simulation already running ($p). Use '$0 stop' first." >&2
      exit 1
    fi
  done
  if [ ! -f "$WS_PATH/install/setup.bash" ]; then
    echo "Workspace not built: run 'colcon build' in $WS_PATH" >&2
    exit 1
  fi

  local dir="$LOG_ROOT/$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$dir/ros"
  link_latest "$dir"
  # Own gz-transport partition per computer (see launch_gz_and_drones.sh).
  export ROS_LOG_DIR="$dir/ros" LOG_DIR="$dir" GZ_PARTITION="${AERO_GZ_PARTITION:-$(hostname)}"
  # GZ_VERBOSE=1 exportado ligava o modo debug do gz-transport (flood no gazebo_px4.log).
  export SKIP_PX4_BUILD="${SKIP_PX4_BUILD:-}" GZ_SIM_VERBOSE="${GZ_SIM_VERBOSE:-${GZ_VERBOSE:-1}}"
  unset GZ_VERBOSE
  export GZ_RENDER_API="${GZ_RENDER_API:-opengl}" GZ_HEADLESS="${GZ_HEADLESS:-}"
  local setup="source $WS_PATH/install/setup.bash"
  local scenario_args="scenario_dir:=$REPO_ROOT/scenarios"
  scenario_env "$REPO_ROOT" "$WS_PATH" || exit 1
  [ -n "${SCENARIO_PATH:-}" ] && scenario_args+=" scenario:=$SCENARIO_PATH"

  # Console do PX4 por FIFO, mantido aberto por um `sleep` (sem EOF no stdin).
  mkfifo "$dir/px4.stdin"
  setsid sleep infinity > "$dir/px4.stdin" 2> /dev/null < /dev/null &
  echo $! > "$dir/px4_stdin_holder.pid"

  echo "Starting the simulation ($WORLD_NAME, logs in $dir):"
  spawn xrce_agent "MicroXRCEAgent udp4 -p 8888 -v $XRCE_VERBOSE"
  # Fila curta e sem imagens cruas (config/foxglove_bridge.yaml). FOXGLOVE=0 não o
  # inicia (a GCS web dispensa o Foxglove).
  if [ "${FOXGLOVE:-1}" = "1" ]; then
    local fox_args="--params-file $WS_PATH/src/aero_pursuit/config/foxglove_bridge.yaml"
    [ "${FOXGLOVE_ALL_TOPICS:-0}" = "1" ] && fox_args+=" -p topic_whitelist:=\"['.*']\""
    spawn foxglove "$setup && ros2 run foxglove_bridge foxglove_bridge \
--ros-args $fox_args --log-level $LOG_LEVEL"
  fi
  spawn gazebo_px4 "cd $WS_PATH && ./launch_gz_and_drones.sh $WORLD_NAME" "$dir/px4.stdin"
  spawn aero_pursuit "$setup && ros2 launch aero_pursuit aero_pursuit_sim.launch.xml \
world_name:=$WORLD_NAME log_level:=$LOG_LEVEL dataset_dir:=$REPO_ROOT/datasets $scenario_args $*"
  echo "Follow it with: $0 logs <process> | $0 view | $0 status"
}

stop_group() {  # stop_group <pidfile>
  local pid
  [ -f "$1" ] || return 0
  pid=$(cat "$1")
  # O grupo inteiro é acompanhado: o líder (bash -c) pode sair antes dos filhos,
  # que ficariam órfãos se só o líder fosse verificado.
  group_alive "$pid" || return 0
  kill -INT -- "-$pid" 2>/dev/null
  for _ in $(seq 1 20); do
    group_alive "$pid" || return 0
    sleep 0.5
  done
  kill -TERM -- "-$pid" 2>/dev/null
  for _ in $(seq 1 6); do
    group_alive "$pid" || return 0
    sleep 0.5
  done
  echo "  forcing the shutdown of group $pid (SIGKILL)"
  kill -KILL -- "-$pid" 2>/dev/null
}

cmd_stop() {
  local dir p
  dir=$(run_dir)
  [ -n "$dir" ] || { echo "No run recorded."; return 0; }
  # Ordem inversa: o aero_pursuit fecha o rosbag antes de o Gazebo sumir.
  for p in aero_pursuit foxglove gazebo_px4 xrce_agent; do
    if is_running "$p"; then
      echo "Stopping $p..."
      stop_group "$dir/$p.pid"
    fi
  done
  stop_group "$dir/px4_stdin_holder.pid"
  echo "Simulation stopped. Logs: $dir"
}

cmd_status() {
  local dir p
  dir=$(run_dir)
  [ -n "$dir" ] || { echo "No run recorded."; return 0; }
  echo "Run: $dir"
  for p in "${PROCS[@]}"; do
    if is_running "$p"; then
      printf "  %-13s running (pid %s, log %s)\n" "$p" "$(cat "$dir/$p.pid")" \
        "$(du -h "$dir/$p.log" 2>/dev/null | cut -f1)"
    else
      printf "  %-13s stopped\n" "$p"
    fi
  done
}

cmd_logs() {
  local dir
  dir=$(run_dir)
  [ -n "$dir" ] || { echo "No run recorded."; return 1; }
  if [ $# -eq 0 ]; then
    ls -la "$dir"/*.log
    echo "Logs ROS 2 por processo: $dir/ros"
    return 0
  fi
  local file="$dir/$1.log"
  [ -f "$file" ] || { echo "Log inexistente: $file" >&2; return 1; }
  if [ $# -ge 2 ]; then
    tail -n 200 -f "$file" | grep --line-buffered -- "$2"
  else
    tail -n 200 -f "$file"
  fi
}

cmd_px4() {
  local fifo
  fifo="$(run_dir)/px4.stdin"
  [ -p "$fifo" ] && is_running gazebo_px4 || { echo "PX4 is not running." >&2; return 1; }
  echo "$*" > "$fifo"
  echo "Sent to the PX4: $* (output: $0 logs gazebo_px4)"
}

cmd_view() {
  local dir session=sim_logs p
  dir=$(run_dir)
  [ -n "$dir" ] || { echo "No run recorded."; return 1; }
  command -v tmux &> /dev/null || { echo "tmux not installed." >&2; return 1; }
  tmux kill-session -t $session 2>/dev/null
  tmux new-session -d -s $session -n logs "tail -n 100 -f $dir/${PROCS[0]}.log"
  for p in "${PROCS[@]:1}"; do
    tmux split-window -t $session:logs "tail -n 100 -f $dir/$p.log"
    tmux select-layout -t $session:logs tiled
  done
  tmux attach-session -t $session
}

case "${1:-}" in
  start) shift; cmd_start "$@" ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  logs) shift; cmd_logs "$@" ;;
  px4) shift; cmd_px4 "$@" ;;
  view) cmd_view ;;
  *) sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
