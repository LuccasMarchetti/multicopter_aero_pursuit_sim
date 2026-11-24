#!/bin/bash
# Anexa o sistema gz::sim::systems::PosePublisher aos modelos em execução, para que
# o ros_gz_bridge publique as TFs `<mundo> -> <modelo>` diretamente no /tf
# (config/gz_bridge_<mundo>_tf.yaml), sem o nó gazebo_tf_bridge.
#
# Uso: ./gz_pose_tf.sh [mundo] [modelo ...]
#   ./gz_pose_tf.sh lawn                       # modelos padrão (GZ_TF_MODELS)
#   ./gz_pose_tf.sh lawn x500_depth_0 X4
# Variáveis: GZ_TF_MODELS, GZ_TF_RATE (Hz, padrão 50), GZ_TF_TIMEOUT (s, padrão 180).
#
# Espera cada modelo existir (o x500_depth_0 só aparece quando o PX4 sobe) e é
# idempotente: se o tópico de pose do modelo já existe, não anexa de novo.

WORLD=${1:-lawn}
shift
MODELS=("$@")
if [ ${#MODELS[@]} -eq 0 ]; then
  read -r -a MODELS <<< "${GZ_TF_MODELS:-x500_depth_0 X3 X4 Tiltrotor X5}"
fi
RATE=${GZ_TF_RATE:-50}
TIMEOUT=${GZ_TF_TIMEOUT:-180}

PLUGIN_XML="<publish_link_pose>false</publish_link_pose>\
<publish_nested_model_pose>false</publish_nested_model_pose>\
<publish_model_pose>true</publish_model_pose>\
<publish_sensor_pose>false</publish_sensor_pose>\
<publish_collision_pose>false</publish_collision_pose>\
<publish_visual_pose>false</publish_visual_pose>\
<use_pose_vector_msg>true</use_pose_vector_msg>\
<update_frequency>$RATE</update_frequency>"

model_id() {
  gz model -m "$1" 2>/dev/null | grep -oP 'Model: \[\K[0-9]+' | head -n1
}

attach() {  # attach <modelo>
  local model=$1 id deadline=$((SECONDS + TIMEOUT))
  # `gz topic -l` também lista tópicos que só têm assinantes (ex.: o gz_tf_bridge,
  # se o launch subir antes); por isso confere se há publisher de fato.
  if gz topic -i -t "/world/$WORLD/model/$model/pose" 2>/dev/null | grep -q '^Publishers'; then
    echo "[gz_pose_tf] $model: already publishes its pose."
    return 0
  fi
  until id=$(model_id "$model") && [ -n "$id" ]; do
    if [ $SECONDS -ge $deadline ]; then
      echo "[gz_pose_tf] $model: not found after ${TIMEOUT}s." >&2
      return 1
    fi
    sleep 1
  done
  if gz service -s "/world/$WORLD/entity/system/add" \
      --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 5000 \
      --req "entity: {id: $id}, plugins: [{name: \"gz::sim::systems::PosePublisher\", \
filename: \"gz-sim-pose-publisher-system\", innerxml: \"$PLUGIN_XML\"}]" 2>/dev/null | grep -q true; then
    echo "[gz_pose_tf] $model (id $id): PosePublisher anexado ($RATE Hz)."
  else
    echo "[gz_pose_tf] $model: could not attach the PosePublisher." >&2
    return 1
  fi
}

for m in "${MODELS[@]}"; do
  attach "$m" &
done
wait
