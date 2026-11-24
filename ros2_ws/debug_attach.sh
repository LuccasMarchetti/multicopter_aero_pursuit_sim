#!/bin/bash
# Live debugging of the aero_pursuit C++ nodes (gdb).
#
#   ./debug_attach.sh build            build with symbols (RelWithDebInfo)
#   ./debug_attach.sh attach [pattern] attach gdb to the running process
#                                      (default: aero_pursuit_container)
#   ./debug_attach.sh server           how to start the container under gdbserver
#   ./debug_attach.sh core             open the container's latest core dump
#
# The nodes run as components in a single process (component_container_mt): attaching to it
# reaches all of them (vision_tracker, estimator, controller...).
# Breakpoints by function, e.g. `break aero_pursuit::PursuitControllerNode::runController`.
# In VS Code: Run and Debug -> "aero_pursuit: attach to the container" (.vscode/launch.json).

WS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ptrace_check() {
  local scope
  scope=$(cat /proc/sys/kernel/yama/ptrace_scope 2>/dev/null || echo 0)
  if [ "$scope" != "0" ]; then
    echo "kernel.yama.ptrace_scope=$scope: gdb can only attach to child processes."
    echo "Allow it until the next boot with:  sudo sysctl -w kernel.yama.ptrace_scope=0"
    echo "(or use '$0 server', which does not need to attach)."
    return 1
  fi
}

case "${1:-}" in
  build)
    cd "$WS_PATH" && colcon build --packages-select aero_pursuit \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    echo "Note: the type stays in the CMake cache; go back with -DCMAKE_BUILD_TYPE=Release."
    ;;
  attach)
    pattern=${2:-aero_pursuit_container}
    pid=$(pgrep -f -- "__node:=$pattern" | head -n1)
    [ -z "$pid" ] && pid=$(pgrep -f -- "$pattern" | grep -v $$ | head -n1)
    if [ -z "$pid" ]; then
      echo "Process '$pattern' not found (is the simulation running?)." >&2
      exit 1
    fi
    ptrace_check || exit 1
    echo "Attaching to PID $pid ($(tr '\0' ' ' < /proc/"$pid"/cmdline | cut -c1-80))..."
    source "$WS_PATH/install/setup.bash"
    # SIGINT/SIGTERM repassados: Ctrl-C no launch continua encerrando normalmente.
    exec gdb -p "$pid" \
      -ex "set pagination off" \
      -ex "handle SIGINT nostop pass" -ex "handle SIGTERM nostop pass" \
      -ex "handle SIGPIPE nostop noprint pass" \
      -ex "echo \\n(gdb attached: 'c' continues; Ctrl-C in gdb pauses; 'thread apply all bt')\\n"
    ;;
  server)
    cat <<MSG
Start the container under gdbserver (nothing runs until the debugger connects):
  ./start_simulation.sh debug:=true            # port 3000 (gdb_port:=N for another)
Connect from VS Code ("aero_pursuit: gdbserver :3000") or in a terminal:
  source install/setup.bash
  gdb /opt/ros/jazzy/lib/rclcpp_components/component_container_mt \\
      -ex "target remote localhost:3000" -ex c
MSG
    ;;
  core)
    command -v coredumpctl &> /dev/null || { echo "coredumpctl unavailable."; exit 1; }
    source "$WS_PATH/install/setup.bash"
    exec coredumpctl debug component_container_mt
    ;;
  *)
    sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
