#!/bin/bash
# gdb com o ambiente do workspace (usado como miDebuggerPath no VS Code):
# os nós precisam do LD_LIBRARY_PATH/AMENT_PREFIX_PATH de install/setup.bash.
WS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS_PATH/install/setup.bash"
exec gdb "$@"
