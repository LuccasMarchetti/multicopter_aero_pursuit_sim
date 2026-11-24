#!/bin/bash
# Gazebo + PX4 SITL (pursuer x500_gimbal_depth: camera on a pitch gimbal) + target drones (default: X4).
# Uso: ./launch_gz_and_drones.sh [mundo]   (padrão: lawn)
#   SKIP_PX4_BUILD=1  pula o `make px4_sitl`
#   PX4_DIR=...       caminho do PX4-Autopilot (padrão: ~/PX4-Autopilot)
#   GZ_RENDER_API=opengl|vulkan   backend do OGRE2 (padrão: opengl)
#   GZ_HEADLESS=1     servidor sem GUI, sensores renderizados na GPU via EGL
#                     (automático em sessões SSH)
#   GZ_DRI_PRIME=1    notebooks híbridos: força a GPU dedicada (Mesa)
#   GZ_SIM_VERBOSE=0..4   verbosidade do Gazebo (padrão: 1 = só erros; GZ_VERBOSE é aceito
#                     por compatibilidade, mas NÃO é repassado: ver abaixo)
#   GZ_TRANSPORT_VERBOSE=1   liga o modo debug do gz-transport (discovery, muito verboso)
#   PX4_QUIET=1       silencia o console do PX4 (log em $LOG_DIR/px4.log)
#   KILL_STALE=0     não encerra px4/gz sim de execuções anteriores (padrão: encerra)
#   GZ_STOP_TIMEOUT=s espera pelo encerramento gracioso antes do SIGKILL (padrão: 5)
#   AERO_BASE_DRONES="X4:x4:5:-2:1 ..."  target drones of the base world (default: only the X4).
#   AERO_SCENERY=none|forest_edge    cenografia (config/scenery/<nome>.json; também no Isaac Sim).
#   AERO_REPLAY=1    mundo sem gravidade para o replay de sessão (GCS: poses gravadas teleportadas).
#
# Saída: o log completo do Gazebo e do spawn vai para $LOG_DIR/gazebo.log; no terminal
# aparecem só as linhas de erro do Gazebo e o console do PX4.
#
# O ros_gz_bridge e as TFs da câmera são iniciados pelo aero_pursuit_sim.launch.xml.
# As TFs dos modelos vêm do PosePublisher anexado por gz_pose_tf.sh (padrão) ou do
# nó gazebo_tf_bridge (tf_source:=node); os nomes de modelo abaixo devem coincidir
# com config/gz_bridge_<mundo>_tf.yaml e `gazebo_tf_bridge.models`.

WORLD_NAME=${1:-"lawn"}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_SDF="$PX4_DIR/Tools/simulation/gz/worlds/$WORLD_NAME.sdf"
GZ_STOP_TIMEOUT=${GZ_STOP_TIMEOUT:-5}

if [ ! -f "$WORLD_SDF" ]; then
  echo "World not found: $WORLD_SDF" >&2
  exit 1
fi
# Session replay (GCS, world_replay node): the models are teleported to their recorded poses every
# frame, and Gazebo keeps a body's velocity across a teleport, so gravity would accelerate them
# between frames. The replay world has no gravity (nothing else moves them: PX4 is off).
if [ "${AERO_REPLAY:-}" = "1" ]; then
  replay_sdf="${XDG_RUNTIME_DIR:-/tmp}/aero_${WORLD_NAME}_replay.sdf"
  sed 's|<gravity>[^<]*</gravity>|<gravity>0 0 0</gravity>|' "$WORLD_SDF" > "$replay_sdf" && WORLD_SDF="$replay_sdf"
fi
# Scenery (trees around the flight area, config/scenery/<name>.json, the same in Isaac Sim): a
# copy of the world with the models included; the world keeps its name.
if [ -n "${AERO_SCENERY:-}" ] && [ "$AERO_SCENERY" != "none" ]; then
  scenery_file="$WS_DIR/src/aero_pursuit/config/scenery/$AERO_SCENERY.json"
  scenery_sdf="${XDG_RUNTIME_DIR:-/tmp}/aero_${WORLD_NAME}_${AERO_SCENERY}.sdf"
  if [ -f "$scenery_file" ] && python3 "$WS_DIR/tools/scenery_world.py" "$WORLD_SDF" "$scenery_file" "$scenery_sdf"; then
    WORLD_SDF="$scenery_sdf"
  else
    echo "Warning: scenery '$AERO_SCENERY' unavailable ($scenery_file); world without it." >&2
  fi
fi
if [ ! -f "$WS_DIR/src/aero_pursuit/config/gz_bridge_$WORLD_NAME.yaml" ]; then
  echo "Warning: no config/gz_bridge_$WORLD_NAME.yaml in aero_pursuit (the bridge will not start)." >&2
fi

# --- Encerramento ----------------------------------------------------------------
# Lista todos os descendentes de <pid> (gz sim e seus forks, PX4, gz_pose_tf.sh e os
# subshells dele, `gz service`...), exceto o próprio subshell que faz a consulta.
descendants() {
  ps -e -o pid= -o ppid= | awk -v root="$1" -v self="$BASHPID" '
    { parent[$1] = $2 }
    END {
      for (p in parent) {
        if (p == self) continue
        q = parent[p]
        while (q > 1) {
          if (q == self) break
          if (q == root) { print p; break }
          q = parent[q]
        }
      }
    }'
}

alive() {  # alive <pid...>: imprime os que ainda existem
  local p
  for p in "$@"; do kill -0 "$p" 2>/dev/null && echo "$p"; done
}

# SIGTERM -> espera GZ_STOP_TIMEOUT -> SIGKILL. O servidor do Gazebo pode travar no
# próprio encerramento (SIGTERM/SIGINT ficam bloqueados na thread principal e o
# processo nunca sai), por isso o SIGKILL é obrigatório e não pode ser interrompido.
terminate() {  # terminate <pid...>
  [ $# -eq 0 ] && return 0
  kill -TERM "$@" 2>/dev/null
  local left=("$@") _
  for _ in $(seq 1 $((GZ_STOP_TIMEOUT * 4))); do
    mapfile -t left < <(alive "${left[@]}")
    [ ${#left[@]} -eq 0 ] && return 0
    sleep 0.25
  done
  echo "Forcing the shutdown (SIGKILL): ${left[*]}" >&2
  kill -KILL "${left[@]}" 2>/dev/null
}

cleanup() {
  # Ctrl+C repetido, SIGTERM ou o fechamento do tmux (SIGHUP) durante a limpeza não
  # podem abortá-la: era isso que deixava o gz sim órfão.
  trap '' INT TERM HUP
  trap - EXIT
  echo "Stopping Gazebo/PX4..." >&2
  local px4_pids=() pids p
  # PX4 primeiro, com SIGINT (encerramento limpo dos módulos e do log).
  for p in $(descendants $$); do
    [ "$(cat "/proc/$p/comm" 2>/dev/null)" = px4 ] && px4_pids+=("$p")
  done
  if [ ${#px4_pids[@]} -gt 0 ]; then
    kill -INT "${px4_pids[@]}" 2>/dev/null
    for _ in $(seq 1 8); do
      [ -z "$(alive "${px4_pids[@]}")" ] && break
      sleep 0.25
    done
  fi
  mapfile -t pids < <(descendants $$)
  terminate "${pids[@]}"
  # Netos que já tinham sido reparentados no meio do caminho.
  mapfile -t pids < <(descendants $$)
  [ ${#pids[@]} -gt 0 ] && kill -KILL "${pids[@]}" 2>/dev/null
  wait 2>/dev/null
  echo "Encerrado." >&2
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

# Restos de uma execução anterior (ex.: terminal fechado com SIGKILL) quebram a nova:
# o PX4 se conecta ao primeiro mundo que achar no gz-transport e recusa subir se outra
# instância segura o lock. Encerra-os antes de começar.
STALE=()
[ "${KILL_STALE:-1}" = "1" ] && mapfile -t STALE < <({ pgrep -x px4; pgrep -f '^(ruby[^ ]* [^ ]*/)?gz sim'; } | sort -u)
if [ ${#STALE[@]} -gt 0 ]; then
  echo "Warning: stopping processes of a previous simulation:" >&2
  ps -o pid=,etime=,cmd= -p "$(IFS=,; echo "${STALE[*]}")" | cut -c1-120 >&2
  terminate "${STALE[@]}"
fi

cd "$PX4_DIR" || exit 1
if [ -z "$SKIP_PX4_BUILD" ]; then
  make px4_sitl || exit 1
fi

# Plugins e recursos do Gazebo do PX4
source "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models

# Gazebo reachable over Tailscale (optional, e.g. a remote Gazebo GUI). Each computer has its own
# gz-transport partition (default: its hostname): two GCSs on the same network would otherwise mix
# their worlds (same world and model names: the PX4 received the IMU of both and never flew).
if command -v tailscale &> /dev/null && TS_IP=$(tailscale ip -4 2>/dev/null) && [ -n "$TS_IP" ]; then
  export GZ_IP=$TS_IP
fi
export GZ_PARTITION="${AERO_GZ_PARTITION:-$(hostname)}"

# --- Verbosidade ---------------------------------------------------------------
# A variável de ambiente GZ_VERBOSE=1 NÃO é a verbosidade do Gazebo: o gz-transport a
# lê e entra em modo debug, imprimindo cada anúncio/conexão de tópico e serviço
# ("Advertise options", "Process UUID", "Connection callback"...) em TODOS os
# processos gz (gz sim, gz service, gz_bridge do PX4). Era a origem do flood.
GZ_SIM_VERBOSE=${GZ_SIM_VERBOSE:-${GZ_VERBOSE:-1}}
unset GZ_VERBOSE
if [ "${GZ_TRANSPORT_VERBOSE:-0}" = "1" ]; then
  export GZ_VERBOSE=1
fi

# --- Renderização na GPU -------------------------------------------------------
# Os sensores (câmera RGB e profundidade) são renderizados pelo sistema Sensors
# com OGRE2 (server.config do PX4). O OGRE2 usa a GPU desde que o driver OpenGL /
# Vulkan seja de hardware; com llvmpipe (software) tudo roda na CPU.
GZ_RENDER_API=${GZ_RENDER_API:-opengl}
LOG_DIR=${LOG_DIR:-$(dirname "$WS_DIR")/logs/latest}
mkdir -p "$LOG_DIR"
GZ_LOG="$LOG_DIR/gazebo.log"
: > "$GZ_LOG"
if [ -n "$GZ_DRI_PRIME" ]; then
  export DRI_PRIME=1
fi
if [ -n "$LIBGL_ALWAYS_SOFTWARE" ] && [ "$LIBGL_ALWAYS_SOFTWARE" != "0" ]; then
  echo "Warning: LIBGL_ALWAYS_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE forces CPU rendering." >&2
fi
if command -v glxinfo &> /dev/null && [ -n "$DISPLAY" ]; then
  RENDERER=$(glxinfo -B 2>/dev/null | grep -m1 "OpenGL renderer" | cut -d: -f2-)
  echo "OpenGL renderer:${RENDERER:- desconhecido}"
  if echo "$RENDERER" | grep -qi "llvmpipe\|softpipe\|software"; then
    echo "Warning: software renderer; the cameras will render on the CPU." >&2
  fi
fi
# O RenderSystem_Vulkan do gz_ogre_next_vendor (Jazzy) pode vir com símbolos do
# glslang não resolvidos; nesse caso o Gazebo aborta, então volta ao OpenGL.
if [ "$GZ_RENDER_API" = "vulkan" ]; then
  VK_PLUGIN=$(find /opt/ros/*/opt/gz_ogre_next_vendor /usr/lib -name "RenderSystem_Vulkan.so*" 2>/dev/null | head -n1)
  if [ -z "$VK_PLUGIN" ] || ldd -r "$VK_PLUGIN" 2>&1 | grep -q "undefined symbol"; then
    echo "Warning: OGRE2 Vulkan backend unavailable/broken ($VK_PLUGIN); using OpenGL." >&2
    GZ_RENDER_API=opengl
  fi
fi
GZ_ARGS=(-r -v "$GZ_SIM_VERBOSE" --render-engine ogre2)
if [ "$GZ_RENDER_API" = "vulkan" ]; then
  GZ_ARGS+=(--render-engine-api-backend vulkan)
fi

# Log completo em $GZ_LOG; no terminal, só erros (sem os códigos de cor) e falhas.
show_errors() {
  tee -a "$GZ_LOG" | sed -u 's/\x1b\[[0-9;]*m//g' | \
    grep --line-buffered -E '\[Err\]|\[Error\]|Error |[Ee]rror:|Segmentation|Aborted|terminate called' | \
    sed -u 's/^/[gazebo] /' >&2
}

if [ -n "$SSH_CLIENT" ] || [ -n "$GZ_HEADLESS" ] || { [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; }; then
  echo "Gazebo headless (server + EGL rendering on the GPU, API: $GZ_RENDER_API), log: $GZ_LOG"
  gz sim -s --headless-rendering "${GZ_ARGS[@]}" "$WORLD_SDF" > >(show_errors) 2>&1 &
else
  echo "Gazebo (OGRE2, API: $GZ_RENDER_API), log: $GZ_LOG"
  gz sim "${GZ_ARGS[@]}" "$WORLD_SDF" > >(show_errors) 2>&1 &
fi
GZ_PID=$!

# Aguarda o serviço de criação de entidades antes de inserir os modelos.
ready=
for _ in $(seq 1 120); do
  if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "Error: Gazebo exited during start-up (see $GZ_LOG)." >&2
    exit 1
  fi
  gz service -l 2>/dev/null | grep -q "^/world/$WORLD_NAME/create$" && { ready=1; break; }
  sleep 0.5
done
if [ -z "$ready" ]; then
  echo "Error: /world/$WORLD_NAME/create did not appear in 60 s (see $GZ_LOG)." >&2
  exit 1
fi

MODELS_DIR="$WS_DIR/src/aero_pursuit/gz_models"
spawn() {  # spawn <pasta do modelo> <nome> <x> <y> <z>
  if gz service -s "/world/$WORLD_NAME/create" \
      --reqtype gz.msgs.EntityFactory \
      --reptype gz.msgs.Boolean \
      --timeout 10000 \
      --req "sdf_filename: \"$MODELS_DIR/$1/model.sdf\", name: \"$2\", pose: {position: {x: $3, y: $4, z: $5}}" \
      2>&1 | tee -a "$GZ_LOG" | grep -q "data: true"; then
    echo "Model $2 inserted."
  else
    echo "Error: could not insert model $2 (see $GZ_LOG)." >&2
  fi
}
# Base drones: "<name>:<model folder>:<x>:<y>:<z>" separated by spaces. Default: only the X4;
# the others (X3, Tiltrotor, X5) can be added live in the GCS Drones tab.
AERO_BASE_DRONES=${AERO_BASE_DRONES:-"X4:x4:5:-2:1"}
BASE_NAMES=()
for entry in $AERO_BASE_DRONES; do
  IFS=: read -r b_name b_model b_x b_y b_z <<< "$entry"
  spawn "$b_model" "$b_name" "$b_x" "$b_y" "$b_z"
  BASE_NAMES+=("$b_name")
done

# Os controladores de velocidade dos alvos (MulticopterVelocityControl) começam
# desligados: sem o enable, o Twist do joystick/política do evasor é ignorado. Repete
# algumas vezes porque a primeira mensagem pode chegar antes da descoberta do gz-transport.
# Ligar/desligar em execução: /<ns>/enable (std_msgs/Bool) pelo bridge ou pela GCS.
(for _ in 1 2 3; do
  for ns in "${BASE_NAMES[@]}"; do
    gz topic -t "/$ns/enable" -m gz.msgs.Boolean -p "data: true" >> "$GZ_LOG" 2>&1
  done
  sleep 1
done) &

# TFs nativas: PosePublisher em cada modelo (o x500_depth_0 surge com o PX4).
"$WS_DIR/gz_pose_tf.sh" "$WORLD_NAME" x500_depth_0 "${BASE_NAMES[@]}" > "$LOG_DIR/gz_pose_tf.log" 2>&1 &

# PX4 SITL com câmera de profundidade (detecta o mundo já em execução).
# Roda em segundo plano + `wait`: o `wait` é interrompido pelos sinais tratados, então
# o cleanup começa na hora (com o PX4 em primeiro plano, o bash só tratava o SIGTERM
# depois que o PX4 saísse). `0<&0` mantém o stdin (terminal ou FIFO do sim_manager)
# no `cat`, que repassa os comandos digitados ao console do PX4.
export PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_x500_depth PX4_GZ_WORLD=$WORLD_NAME
# Pursuer: x500_gimbal_depth (camera on a pitch gimbal; a "fixed" camera is the gimbal held still,
# gimbal_node). Inserted here under the x500_depth name (same frames, topics and bridge), and the
# PX4 attaches to it instead of spawning its own model (same airframe 4002).
gimbal_sdf="<sdf version=\"1.9\"><include><uri>file://$MODELS_DIR/x500_gimbal_depth/model.sdf</uri></include></sdf>"
if gz service -s "/world/$WORLD_NAME/create" --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 10000 \
    --req "name: \"x500_depth_0\", allow_renaming: false, sdf: '$gimbal_sdf'" \
    2>&1 | tee -a "$GZ_LOG" | grep -q "data: true"; then
  echo "Pursuer x500_gimbal_depth inserted as x500_depth_0."
  export PX4_GZ_MODEL_NAME=x500_depth_0
  sleep 1
else
  echo "Error: could not insert x500_gimbal_depth (see $GZ_LOG)." >&2
  exit 1
fi
if [ -n "$PX4_QUIET" ]; then
  (sleep 4; echo "param set NAV_DLL_ACT 0"; exec cat) 0<&0 | \
    ./build/px4_sitl_default/bin/px4 > "$LOG_DIR/px4.log" 2>&1 &
else
  (sleep 4; echo "param set NAV_DLL_ACT 0"; exec cat) 0<&0 | \
    ./build/px4_sitl_default/bin/px4 &
fi
PX4_PID=$!

wait "$PX4_PID"
echo "PX4 exited (code $?)." >&2
