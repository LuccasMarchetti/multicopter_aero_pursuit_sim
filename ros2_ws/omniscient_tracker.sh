#!/bin/bash
# Alterna a fonte de medida do estimador em execução:
#   ./omniscient_tracker.sh [frame_alvo] [true|false]
#   ./omniscient_tracker.sh X4 true     # ground truth (TF do Gazebo) do X4
#   ./omniscient_tracker.sh X4 false    # volta para a visão (TrackerVit)
# Equivale a `ros2 param set /target_estimator_node ...` (também aceito via /target_tf_cmd).

TARGET_FRAME="${1:-X4}"
ENABLE_TF_TRACKING="${2:-true}"
NODE=/target_estimator_node
WS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "$ENABLE_TF_TRACKING" in
  true|false) ;;
  *) echo "Uso: $0 [frame_alvo] [true|false]" >&2; exit 1 ;;
esac

source "$WS_PATH/install/setup.bash"

echo "Target Frame:        ${TARGET_FRAME}"
echo "Enable TF Tracking:  ${ENABLE_TF_TRACKING}"

# O frame primeiro: ao habilitar, o filtro já reinicia no alvo correto.
ros2 param set $NODE default_target_frame "$TARGET_FRAME" && \
  ros2 param set $NODE use_tf_tracking "$ENABLE_TF_TRACKING"
