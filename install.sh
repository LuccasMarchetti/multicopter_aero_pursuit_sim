#!/bin/bash
# Full project installation on a new machine (Ubuntu 24.04 + ROS 2 Jazzy).
#
#   git clone --recursive git@github.com:LuccasMarchetti/aero-pursuit.git ~/aero_pursuit
#   cd ~/aero_pursuit && ./install.sh
#
# Steps (idempotent: whatever is already installed is skipped):
#   ros      ROS 2 Jazzy (desktop) + ros_gz (Gazebo Harmonic), colcon, rosdep
#   apt      GStreamer (VA-API/x264/x265/JPEG), drivers VA, aiohttp, tmux, openssl...
#   px4      PX4-Autopilot pinned at $PX4_REF in $PX4_DIR + make px4_sitl
#   xrce     Micro-XRCE-DDS-Agent (ponte PX4 <-> ROS 2)
#   opencv   OpenCV >= 4.9 in /usr/local (cv::TrackerVit; ros2_ws/install_opencv.sh)
#   acados   MPC solvers (acados + CasADi code generator) in ~/.local/share/acados, no root
#            (ros2_ws/install_acados.sh; ACADOS_ROOT changes the place)
#   ws       px4_msgs submodule, tracker ONNX model, rosdep and colcon build
#   gcs      (optional, --gcs-service) web GCS as a user service (starts at login)
#
# Uso:
#   ./install.sh                  everything (except the GCS service)
#   ./install.sh --check          diagnostics only (installs nothing)
#   ./install.sh --dry-run        shows the commands without running them
#   ./install.sh --only ws,gcs    only the listed steps
#   ./install.sh --skip px4       pula etapas
#   ./install.sh --gcs-service    includes the GCS service
# Variables: PX4_DIR (~/PX4-Autopilot), PX4_REF (tested commit), ROS_DISTRO (jazzy),
#   OPENCV_VERSION (4.9.0), XRCE_REF (v2.4.3), JOBS (cores), REBUILD_PX4=1.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$REPO/ros2_ws"
ROS_DISTRO=${ROS_DISTRO:-jazzy}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
UDEV_RULE=/etc/udev/rules.d/99-aero-gamepad.rules
UDEV_RULE_TEXT='SUBSYSTEM=="input", ENV{ID_INPUT_JOYSTICK}=="1", GROUP="plugdev", MODE="0660"'
PX4_REF=${PX4_REF:-ff5b9484369b714763db9638517c08df0c242237}   # v1.18.0-beta1-315 (testado)
XRCE_REF=${XRCE_REF:-v2.4.3}
OPENCV_VERSION=${OPENCV_VERSION:-4.9.0}
JOBS=${JOBS:-$(nproc)}

ALL_STEPS=(ros apt px4 xrce opencv acados ws gcs)
STEPS=(ros apt px4 xrce opencv acados ws)
MODE=install
while [ $# -gt 0 ]; do
  case "$1" in
    --check) MODE=check ;;
    --dry-run) MODE=dry ;;
    --only) IFS=, read -r -a STEPS <<< "$2"; shift ;;
    --skip) IFS=, read -r -a SKIP <<< "$2"; shift
            for s in "${SKIP[@]}"; do STEPS=("${STEPS[@]/$s}"); done ;;
    --gcs-service) STEPS+=(gcs) ;;
    -h|--help) sed -n 2,26p "$0"; exit 0 ;;
    *) echo "Unknown option: $1 (see --help)" >&2; exit 2 ;;
  esac
  shift
done

c_ok=$'\e[32m'; c_warn=$'\e[33m'; c_err=$'\e[31m'; c_dim=$'\e[2m'; c_off=$'\e[0m'
FAILS=0
ok()   { echo "  ${c_ok}✔${c_off} $*"; }
warn() { echo "  ${c_warn}!${c_off} $*"; }
bad()  { echo "  ${c_err}✘${c_off} $*"; FAILS=$((FAILS + 1)); }
step() { echo; echo "== $* =="; }
run() {  # runs (or only shows, with --dry-run)
  echo "  ${c_dim}\$ $*${c_off}"
  [ "$MODE" = dry ] || "$@"
}
want() { [[ " ${STEPS[*]} " == *" $1 "* ]]; }
have_pkg() { dpkg -s "$1" >/dev/null 2>&1; }
missing_pkgs() { local p; for p in "$@"; do have_pkg "$p" || echo "$p"; done; }

APT_PKGS=(
  build-essential cmake git curl wget pkg-config tmux openssl iproute2 gdb
  python3-aiohttp python3-colcon-common-extensions python3-rosdep python3-vcstool
  python3-venv   # acados code generator (ros2_ws/install_acados.sh)
  libeigen3-dev
  python3-yaml python3-numpy python3-matplotlib
  # GStreamer: VA-API (vah264enc/vah265enc in plugins-bad), x264 (ugly), x265 (bad), JPEG (good)
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
  gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools
  # VA-API drivers (GPU encoding): older AMD/Intel (mesa) and recent Intel
  mesa-va-drivers intel-media-va-driver vainfo
  # OpenCL for GPU inference (dnn_target: opencl): Mesa ICD (rusticl) + diagnostics
  ocl-icd-libopencl1 mesa-opencl-icd clinfo
)
ROS_PKGS=(
  python3-rosdep python3-colcon-common-extensions
  "ros-$ROS_DISTRO-desktop" "ros-$ROS_DISTRO-ros-gz" "ros-$ROS_DISTRO-joy"
  "ros-$ROS_DISTRO-rosbag2-storage-mcap" "ros-$ROS_DISTRO-tf2-eigen"
  "ros-$ROS_DISTRO-foxglove-bridge"
)

opencv_version() {
  local f=/usr/local/lib/cmake/opencv4/OpenCVConfig-version.cmake
  [ -f "$f" ] && sed -nE 's/.*OpenCV_VERSION ([0-9.]+).*/\1/p' /usr/local/lib/cmake/opencv4/OpenCVConfig.cmake 2>/dev/null | head -1
}

version_ge() { [ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -1)" = "$2" ]; }

# -- diagnostics -----------------------------------------------------------------
check() {
  step "Diagnostics"
  local os; os=$(. /etc/os-release && echo "$ID $VERSION_ID")
  [ "$os" = "ubuntu 24.04" ] && ok "Ubuntu 24.04" || warn "System $os (tested on Ubuntu 24.04)"
  [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] && ok "ROS 2 $ROS_DISTRO" || bad "ROS 2 $ROS_DISTRO ausente (etapa ros)"
  if command -v clinfo >/dev/null && [ "$(clinfo -l 2>/dev/null | grep -c Device)" -gt 0 ]; then
    ok "OpenCL: $(clinfo -l 2>/dev/null | grep -m1 Device | sed 's/.*Device #[0-9]*: //')"
  else
    warn "No OpenCL device (dnn_target: opencl falls back to the CPU; the CPU is the default)"
  fi
  local miss; miss=$(missing_pkgs "${ROS_PKGS[@]}" "${APT_PKGS[@]}" | tr '\n' ' ')
  [ -z "$miss" ] && ok "apt packages" || bad "apt packages missing: $miss"
  if [ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    ok "PX4 SITL built in $PX4_DIR ($(git -C "$PX4_DIR" rev-parse --short HEAD 2>/dev/null))"
  else
    bad "PX4 SITL not built in $PX4_DIR (step px4)"
  fi
  command -v MicroXRCEAgent >/dev/null && ok "MicroXRCEAgent" || bad "MicroXRCEAgent ausente (etapa xrce)"
  local cv; cv=$(opencv_version || true)
  if [ -n "$cv" ] && version_ge "$cv" 4.9; then ok "OpenCV $cv in /usr/local"; else bad "OpenCV >= 4.9 missing in /usr/local (step opencv)"; fi
  [ -f "${ACADOS_ROOT:-$HOME/.local/share/acados}/lib/libacados.so" ] && ok "acados" || bad "acados ausente (etapa acados)"
  [ -f "$WS/src/px4_msgs/package.xml" ] && ok "px4_msgs" || bad "px4_msgs vazio (git submodule update --init)"
  ls "$WS"/src/aero_pursuit/models/*.onnx >/dev/null 2>&1 && ok "TrackerVit ONNX model" || bad "ONNX model missing (step ws)"
  [ -f "$WS/install/setup.bash" ] && ok "workspace built" || bad "workspace not built (step ws)"
  # Video encoders available for the GCS streamer.
  if command -v gst-inspect-1.0 >/dev/null; then
    local e found=()
    for e in vah264enc vah265enc x264enc x265enc jpegenc; do gst-inspect-1.0 "$e" >/dev/null 2>&1 && found+=("$e"); done
    if [[ " ${found[*]} " == *" vah264enc "* ]]; then ok "video on the GPU (VA-API): ${found[*]}"
    elif [ ${#found[@]} -gt 0 ]; then warn "no VA-API: video encoded on the CPU (${found[*]})"
    else bad "nenhum codificador GStreamer (etapa apt)"; fi
  fi
  command -v cloudflared >/dev/null || [ -x "$HOME/.local/bin/cloudflared" ] && ok "cloudflared" \
    || warn "cloudflared missing (the GCS downloads it on the first tunnel use)"
  local js; js=$(ls /dev/input/by-id/*-event-joystick 2>/dev/null | head -1)
  if [ -z "$js" ]; then warn "no USB gamepad connected (local joystick; the browser one does not need it)"
  elif [ -r "$js" ]; then ok "gamepad readable: $(basename "$js")"
  else warn "gamepad without read permission ($(basename "$js")): the apt step installs the udev rule"; fi
  systemctl --user is-enabled aero-gcs.service >/dev/null 2>&1 && ok "aero-gcs service (starts at login)" \
    || warn "GCS without a service (./ros2_ws/gcs.sh, or ./install.sh --only gcs)"
}

# -- etapas ----------------------------------------------------------------------
step_ros() {
  step "ROS 2 $ROS_DISTRO + Gazebo (ros_gz)"
  if [ ! -f "/etc/apt/sources.list.d/ros2.list" ] && ! ls /etc/apt/sources.list.d/ 2>/dev/null | grep -q ros2; then
    run sudo apt-get update
    run sudo apt-get install -y software-properties-common curl
    run sudo add-apt-repository -y universe
    # Official package that installs the ROS 2 key and repository.
    local ver; ver=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -oP '"tag_name": "\K[^"]+')
    run curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
    run sudo apt-get install -y /tmp/ros2-apt-source.deb
  fi
  local miss; mapfile -t miss < <(missing_pkgs "${ROS_PKGS[@]}")
  if [ ${#miss[@]} -gt 0 ]; then
    run sudo apt-get update
    run sudo apt-get install -y "${miss[@]}"
  else
    ok "ROS packages already installed"
  fi
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then run sudo rosdep init; fi
  run rosdep update
}

step_apt() {
  step "System dependencies (GStreamer, VA-API, Python...)"
  local miss; mapfile -t miss < <(missing_pkgs "${APT_PKGS[@]}")
  if [ ${#miss[@]} -gt 0 ]; then
    run sudo apt-get update
    run sudo apt-get install -y "${miss[@]}"
  else
    ok "already installed"
  fi
  # VA-API needs access to /dev/dri (render/video group).
  if ! id -nG | grep -qwE 'render'; then
    warn "user not in the 'render' group: adding it (takes effect at the next login)"
    run sudo usermod -aG render,video "$USER"
  fi
  # USB gamepad on the server (joystick:=local): udev only grants /dev/input/event* to
  # whoever is at the physical seat (uaccess). Over SSH, or with the login screen
  # open, SDL cannot open the controller. The rule grants only joysticks to the plugdev group
  # (not the whole input group, which would also give access to keyboards).
  if [ "$(cat "$UDEV_RULE" 2>/dev/null)" != "$UDEV_RULE_TEXT" ]; then
    run sudo bash -c "printf '%s\\n' '$UDEV_RULE_TEXT' > '$UDEV_RULE' && udevadm control --reload && udevadm trigger --subsystem-match=input"
  fi
  id -nG | grep -qw plugdev || run sudo usermod -aG plugdev "$USER"
}

step_px4() {
  step "PX4-Autopilot ($PX4_REF) in $PX4_DIR"
  if [ ! -d "$PX4_DIR/.git" ]; then
    run git clone https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
  fi
  if [ "$(git -C "$PX4_DIR" rev-parse HEAD 2>/dev/null)" != "$PX4_REF" ]; then
    run git -C "$PX4_DIR" fetch --tags origin
    run git -C "$PX4_DIR" checkout "$PX4_REF"
  fi
  if [ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] && [ -z "${REBUILD_PX4:-}" ] &&
     [ "$(git -C "$PX4_DIR" rev-parse HEAD 2>/dev/null)" = "$PX4_REF" ]; then
    ok "PX4 SITL already built at this commit (REBUILD_PX4=1 forces it)"
    return
  fi
  run git -C "$PX4_DIR" submodule update --init --recursive
  # SITL build dependencies; Gazebo comes from ros_gz (Jazzy vendor).
  run bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx --no-sim-tools
  # With ROS sourced, PX4's CMake finds gz-sim/gz-transport from ros_gz.
  run bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && make -C '$PX4_DIR' px4_sitl -j$JOBS"
}

step_xrce() {
  step "Micro-XRCE-DDS-Agent ($XRCE_REF)"
  if command -v MicroXRCEAgent >/dev/null; then ok "already installed"; return; fi
  local src="$HOME/Micro-XRCE-DDS-Agent"
  [ -d "$src/.git" ] || run git clone -b "$XRCE_REF" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$src"
  run cmake -S "$src" -B "$src/build" -DCMAKE_BUILD_TYPE=Release
  run cmake --build "$src/build" -j"$JOBS"
  run sudo cmake --install "$src/build"
  run sudo ldconfig /usr/local/lib/
}

step_opencv() {
  step "OpenCV $OPENCV_VERSION (/usr/local)"
  local cv; cv=$(opencv_version || true)
  if [ -n "$cv" ] && version_ge "$cv" 4.9; then ok "OpenCV $cv already installed"; return; fi
  run env OPENCV_VERSION="$OPENCV_VERSION" bash "$WS/install_opencv.sh"
}

step_acados() {
  step "acados (MPC solvers)"
  run bash "$WS/install_acados.sh"   # idempotent
}

step_ws() {
  step "Workspace ROS ($WS)"
  if [ -d "$REPO/.git" ]; then
    run git -C "$REPO" submodule update --init --recursive
  elif [ ! -f "$WS/src/px4_msgs/package.xml" ]; then
    run git clone https://github.com/PX4/px4_msgs.git "$WS/src/px4_msgs"
    run git -C "$WS/src/px4_msgs" checkout f719702a6ae14ec73d22e4ec99193687c5ed788c
  fi
  run bash "$WS/src/aero_pursuit/models/download_models.sh"   # idempotent: fetches only missing models
  # shellcheck disable=SC1090
  [ "$MODE" = dry ] || { set +u; source "/opt/ros/$ROS_DISTRO/setup.bash"; set -u; }   # setup.bash is not compatible with set -u
  run rosdep install --from-paths "$WS/src" --ignore-src -r -y --rosdistro "$ROS_DISTRO"
  run bash -c "cd '$WS' && source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
}

step_gcs() {
  step "Web GCS as a user service"
  run "$WS/gcs.sh" install-service
}

# -- principal -------------------------------------------------------------------
if [ "$MODE" = check ]; then check; [ "$FAILS" -eq 0 ]; exit; fi
if [ "$(id -u)" = 0 ]; then echo "Run as a regular user (the script uses sudo when needed)." >&2; exit 1; fi
echo "Repository: $REPO"
echo "Etapas: ${STEPS[*]}  (modo: $MODE)"
[ "$REPO" = "$HOME/aero_pursuit" ] || warn "clone outside ~/aero_pursuit: it works, but the launch datasets default is ~/aero_pursuit/datasets"
for s in "${ALL_STEPS[@]}"; do
  want "$s" && "step_$s"
done
check
echo
echo "Done. Start the GCS with:  $WS/gcs.sh   and open http://localhost:8080"
echo "(the simulation is started by the GCS itself, Simulation tab)"
