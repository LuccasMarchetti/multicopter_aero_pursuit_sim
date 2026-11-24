#!/bin/bash
# Encerra todos os processos da simulação (tmux do start_simulation.sh ou restos de
# execuções anteriores): sinal gracioso -> espera -> SIGKILL no que sobrar.
#
# Uso: ./stop_simulation.sh            (STOP_TIMEOUT=s, padrão 10)
#      DRY_RUN=1 ./stop_simulation.sh  só lista o que seria encerrado
#
# Padrões exatos: um padrão amplo como "px4" mataria qualquer processo com PX4-Autopilot
# (ou px4_msgs) na linha de comando. O "[x]" nos padrões -f impede o pkill de casar com o shell que
# o executa. O joy_node é o executável game_controller_node.

STOP_TIMEOUT=${STOP_TIMEOUT:-10}

# modo do pgrep;padrão;sinal gracioso
# Os padrões -f são ancorados no executável (^...): sem isso, qualquer comando que só
# cite o nome (vim launch_gz_and_drones.sh, git diff, o editor com foxglove_bridge.yaml
# aberto, o shell de quem chamou este script) também seria encerrado.
PATTERNS=(
  # faz a própria limpeza (PX4 -> gz -> SIGKILL)
  "-f;^[^ ]*bash [^ ]*launch_gz_and_drones\.sh( |$);TERM"
  # ros2 launch repassa SIGINT aos nós
  "-f;^[^ ]*python[0-9.]* [^ ]*ros2 launch aero_pursuit aero_pursuit_sim;INT"
  "-x;px4;INT"
  "-f;^(ruby[^ ]* [^ ]*/)?gz sim;TERM"
  "-x;MicroXRCEAgent;TERM"
  "-x;foxglove_bridge;TERM"
  "-f;^[^ ]*python[0-9.]* [^ ]*ros2 run foxglove_bridge;TERM"
  "-f;^([^ ]*gdbserver [^ ]+ )?[^ ]*/component_container_mt .*__node:=aero_pursuit_container;TERM"
  "-f;^[^ ]*/(game_controller_node|joy_node) .*__node:=joy_node;TERM"
  "-f;^[^ ]*/static_transform_publisher .*__node:=camera_(link|optical)_tf;TERM"
)

pids_of() {  # pids_of <modo> <padrão>
  pgrep "$1" "$2" | grep -vx "$$"
}

all_pids() {
  local entry mode pat
  for entry in "${PATTERNS[@]}"; do
    IFS=';' read -r mode pat _ <<< "$entry"
    pids_of "$mode" "$pat"
  done | sort -u
}

if [ -n "$DRY_RUN" ]; then
  left=$(all_pids)
  [ -n "$left" ] && ps -o pid=,etime=,cmd= -p "$(echo $left | tr ' ' ',')" | cut -c1-120
  exit 0
fi

for entry in "${PATTERNS[@]}"; do
  IFS=';' read -r mode pat sig <<< "$entry"
  pids=$(pids_of "$mode" "$pat")
  [ -n "$pids" ] && kill "-$sig" $pids 2>/dev/null
done

for _ in $(seq 1 $((STOP_TIMEOUT * 4))); do
  [ -z "$(all_pids)" ] && exit 0
  sleep 0.25
done

left=$(all_pids)
if [ -n "$left" ]; then
  echo "Forcing the shutdown (SIGKILL):" >&2
  ps -o pid=,cmd= -p "$(echo $left | tr ' ' ',')" | cut -c1-120 >&2
  kill -KILL $left 2>/dev/null
fi
exit 0
