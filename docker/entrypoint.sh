#!/bin/bash
# Container entry point. Usage (via docker/aero.sh or docker compose run):
#   gcs [args]          web ground station (default; it starts the simulation)
#   build [packages]    colcon build of the workspace (incremental, in a volume)
#   sim [args]          full simulation in the background (sim_manager.sh start)
#   scenario <file>     runs a scenario from scenarios/ (headless)
#   study <file>        runs a study (scenarios/studies/*.yaml)
#   test                aero_pursuit tests
#   shell | <command>   interactive bash or any command
set -e
REPO=${AERO_REPO:-/home/aero/aero_pursuit}
WS=$REPO/ros2_ws
set +u
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Named volumes are created by root: hand them to the user (host UID) the first time.
for d in "$WS/build" "$WS/install" "$WS/log" "$HOME/.gz" "$HOME/.config/aero_gcs"; do
  mkdir -p "$d" 2> /dev/null || true
  [ -w "$d" ] || sudo chown "$(id -u):$(id -g)" "$d"
done

ensure_built() {
  if [ ! -f "$WS/src/px4_msgs/package.xml" ]; then
    echo "px4_msgs missing: run 'git submodule update --init' on the host." >&2
    exit 1
  fi
  if ! ls "$WS"/src/aero_pursuit/models/*.onnx > /dev/null 2>&1; then
    bash "$WS/src/aero_pursuit/models/download_models.sh"
  fi
  # Volumes built under another repository path (e.g. before the px4_docker -> aero_pursuit
  # rename): CMake refuses the old caches and setup.bash points to the old prefix. Start over.
  local cache="$WS/build/aero_pursuit/CMakeCache.txt"
  if [ -f "$cache" ] && ! grep -q "^CMAKE_HOME_DIRECTORY:INTERNAL=$WS/src/aero_pursuit$" "$cache"; then
    echo "Workspace built in another path: cleaning build/ and install/ of the volumes..."
    find "$WS/build" "$WS/install" -mindepth 1 -delete 2>/dev/null || true
  fi
  if [ ! -f "$WS/install/setup.bash" ]; then
    echo "First run: building the workspace (a few minutes)..."
    build
  fi
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
}

build() {
  (cd "$WS" && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release "$@")
}

cmd=${1:-gcs}
shift || true
case "$cmd" in
  build) build ${1:+--packages-select "$@"} ;;
  gcs) ensure_built; exec "$WS/gcs.sh" "$@" ;;
  sim) ensure_built; exec "$WS/sim_manager.sh" start "$@" ;;
  scenario) ensure_built; "$WS/run_scenario.sh" "$@"; exec tail -F "$REPO/logs/latest/aero_pursuit.log" ;;
  study) ensure_built; exec python3 "$WS/run_study.py" "$@" ;;
  test) ensure_built; cd "$WS" && colcon test --packages-select aero_pursuit aero_pursuit_gcs && exec colcon test-result --verbose ;;
  shell) ensure_built; exec bash ;;
  *) ensure_built; exec "$cmd" "$@" ;;
esac
