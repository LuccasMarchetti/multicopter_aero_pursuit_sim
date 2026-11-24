#!/bin/bash
# Sessão tmux completa: Gazebo + PX4 SITL, MicroXRCEAgent, Foxglove Bridge e o
# container do aero_pursuit (bridge, visão, estimação, controle, gravação).
#
# Uso: ./start_simulation.sh [argumentos extras do launch]
#   ./start_simulation.sh yaw_mode:=hold
#   ./start_simulation.sh training:=true evader_policy:=random
#   ./start_simulation.sh record:=false
#   SCENARIO=figure8_basic ./start_simulation.sh   (cenário de <repo>/scenarios, inicia sozinho)
# Variáveis: WORLD_NAME (padrão lawn), SKIP_PX4_BUILD=1, LOG_LEVEL (padrão warn),
#   XRCE_VERBOSE (0-6, padrão 3 = avisos), GZ_SIM_VERBOSE, GZ_RENDER_API, GZ_HEADLESS,
#   FOXGLOVE_ALL_TOPICS=1 (anuncia também as imagens cruas no Foxglove),
# Encerrar: fechar a sessão (prefixo + :kill-session) ou ./stop_simulation.sh
# Alternativa sem tmux, com um arquivo de log por processo: ./sim_manager.sh

SESSION="px4_ros2_sim"
WS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_NAME=${WORLD_NAME:-lawn}
LAUNCH_ARGS="$*"
LOG_LEVEL=${LOG_LEVEL:-warn}
XRCE_VERBOSE=${XRCE_VERBOSE:-3}
# Bridge com fila curta e sem imagens cruas (config/foxglove_bridge.yaml).
FOXGLOVE_ARGS="--params-file $WS_PATH/src/aero_pursuit/config/foxglove_bridge.yaml"
[ "${FOXGLOVE_ALL_TOPICS:-0}" = "1" ] && FOXGLOVE_ARGS+=" -p topic_whitelist:=\"['.*']\""

if ! command -v tmux &> /dev/null; then
  echo "tmux is not installed. Installing..."
  sudo apt update && sudo apt install -y tmux
fi
if [ ! -f "$WS_PATH/install/setup.bash" ]; then
  echo "Workspace not built: run 'colcon build' in $WS_PATH" >&2
  exit 1
fi

# Encerramento: stop_simulation.sh (sinal gracioso -> espera -> SIGKILL). A sessão só é
# fechada depois, para o SIGHUP não interromper a limpeza do launch_gz_and_drones.sh.
CLEANUP="$WS_PATH/stop_simulation.sh"

echo "[0/2] Cleaning old processes..."
"$CLEANUP"
tmux kill-session -t $SESSION 2>/dev/null

# Logs em logs/<data_hora>, com logs/latest -> execução atual (mesmo layout do
# sim_manager.sh; sem isso o launch_gz_and_drones.sh criava latest como pasta comum).
REPO_ROOT="$(dirname "$WS_PATH")"
LOG_ROOT=${LOG_ROOT:-$REPO_ROOT/logs}
LOG_DIR="$LOG_ROOT/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
if [ -d "$LOG_ROOT/latest" ] && [ ! -L "$LOG_ROOT/latest" ]; then
  mv "$LOG_ROOT/latest" "$LOG_ROOT/legacy_$(date -r "$LOG_ROOT/latest" +%Y%m%d_%H%M%S)"
fi
ln -sfn "$(basename "$LOG_DIR")" "$LOG_ROOT/latest"
export LOG_DIR

export GZ_PARTITION="${AERO_GZ_PARTITION:-$(hostname)}"   # own gz-transport partition (see launch_gz_and_drones.sh)
export SKIP_PX4_BUILD GZ_SIM_VERBOSE GZ_RENDER_API GZ_HEADLESS
# GZ_VERBOSE=1 no ambiente liga o modo debug do gz-transport (flood): repassado como
# verbosidade do Gazebo pelo launch_gz_and_drones.sh, que remove a variável.
[ -n "$GZ_VERBOSE" ] && export GZ_SIM_VERBOSE=${GZ_SIM_VERBOSE:-$GZ_VERBOSE}
unset GZ_VERBOSE

tmux new-session -d -s $SESSION -n 'Simulation' -c "$WS_PATH"
# setsid -f: a limpeza sobrevive ao fim do servidor do tmux (que mata os próprios jobs
# ao sair) e não o bloqueia enquanto espera os processos.
tmux set-hook -t $SESSION session-closed "run-shell -b 'setsid -f $CLEANUP > /dev/null 2>&1'"

# Serviços de fundo
tmux new-window -d -t $SESSION -n 'Services' -c "$WS_PATH"
tmux send-keys -t $SESSION:Services "MicroXRCEAgent udp4 -p 8888 -v $XRCE_VERBOSE" C-m
tmux split-window -v -t $SESSION:Services -c "$WS_PATH"
tmux send-keys -t $SESSION:Services.1 "source $WS_PATH/install/setup.bash && \
  ros2 run foxglove_bridge foxglove_bridge --ros-args $FOXGLOVE_ARGS --log-level $LOG_LEVEL" C-m

# O ros_gz_bridge roda como componente no container do aero_pursuit
# (config/gz_bridge_<mundo>.yaml), com comunicação intra-processo para as imagens.

# shellcheck source=scenario_env.sh
source "$WS_PATH/scenario_env.sh"
scenario_env "$REPO_ROOT" "$WS_PATH" || exit 1
SCENARIO_ARGS="scenario_dir:=$REPO_ROOT/scenarios"
[ -n "$SCENARIO_PATH" ] && SCENARIO_ARGS+=" scenario:=$SCENARIO_PATH"

AERO_CAMERA_MOUNT=${AERO_CAMERA_MOUNT:-fixed}   # fixed (x500_depth) | gimbal (x500_gimbal_depth)
echo "[1/2] Starting PX4 SITL + Gazebo..."
tmux send-keys -t $SESSION:Simulation "AERO_CAMERA_MOUNT=$AERO_CAMERA_MOUNT ./launch_gz_and_drones.sh $WORLD_NAME" C-m

echo "[2/2] Running aero_pursuit ($LAUNCH_ARGS)..."
tmux split-window -v -t $SESSION:Simulation -c "$WS_PATH"
tmux send-keys -t $SESSION:Simulation.1 "source $WS_PATH/install/setup.bash && \
  ros2 launch aero_pursuit aero_pursuit_sim.launch.xml world_name:=$WORLD_NAME \
  log_level:=$LOG_LEVEL dataset_dir:=$REPO_ROOT/datasets $SCENARIO_ARGS $LAUNCH_ARGS" C-m

tmux select-layout -t $SESSION:Simulation even-vertical
tmux attach-session -t $SESSION:Simulation
