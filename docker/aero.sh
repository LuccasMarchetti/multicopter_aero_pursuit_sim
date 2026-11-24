#!/bin/bash
# Shortcut to the project container (docker compose with the host UID/GID/GPU).
#
#   ./docker/aero.sh image                 builds the image (~30-50 min the first time)
#   ./docker/aero.sh                       GCS at http://localhost:8080 (Ctrl+C stops everything)
#   ./docker/aero.sh build [packages]      colcon build (after editing C++/messages)
#   ./docker/aero.sh scenario flood_decoys headless simulation running a scenario
#   ./docker/aero.sh study scenarios/studies/controllers_vs_trajectories.yaml
#   ./docker/aero.sh test | shell | <comando>
#   GUI=1 ./docker/aero.sh                 Gazebo window on the host (X11)
#   ./docker/aero.sh down                  removes the container (volumes stay; 'down -v' deletes them)
#   ./docker/aero.sh up | stop             GCS in the background as aero-gcs (compose.gcs.yaml; this is
#                                          what `aero gcs start|stop` runs in docker mode)
#
# The first run also prepares the host (Linux with apt), asking for the sudo password when needed:
# installs Docker Engine + Compose v2 + buildx if missing, starts the daemon and puts the user in
# the docker group (no sudo needed afterwards). AERO_SKIP_DOCKER_SETUP=1 skips this step.
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker_ready() {  # engine + compose v2 + buildx installed and the daemon reachable by this user
  docker info > /dev/null 2>&1 && docker compose version > /dev/null 2>&1 &&
    docker buildx version > /dev/null 2>&1
}

ensure_docker() {
  docker_ready && return 0
  if ! command -v apt-get > /dev/null; then
    echo "Docker incompleto. Instale Docker Engine, Compose v2 e buildx:" \
      "https://docs.docker.com/engine/install/" >&2
    return 0  # the checks below give the specific error
  fi
  local missing=()
  command -v docker > /dev/null || missing+=("Docker Engine")
  docker compose version > /dev/null 2>&1 || missing+=("Compose v2")
  docker buildx version > /dev/null 2>&1 || missing+=("buildx")
  if [ ${#missing[@]} -gt 0 ]; then
    # Docker from Docker's repository (docker-ce) has its own plugins; mixing it with Ubuntu's
    # docker.io packages conflicts.
    local pkgs=(docker.io docker-compose-v2 docker-buildx)
    dpkg-query -W -f='${Status}' docker-ce 2> /dev/null | grep -q 'ok installed' &&
      pkgs=(docker-compose-plugin docker-buildx-plugin)
    echo ">>> Docker: missing ${missing[*]}. Installing: ${pkgs[*]} (asks for the sudo password)"
    if ! { sudo apt-get update -qq && sudo apt-get install -y "${pkgs[@]}"; }; then
      echo ">>> Packages unavailable on this system: using the official installer (get.docker.com)"
      curl -fsSL https://get.docker.com | sudo sh
    fi
  fi
  if ! sudo docker info > /dev/null 2>&1; then
    echo ">>> Starting the Docker service"
    sudo systemctl enable --now docker 2> /dev/null || sudo service docker start
  fi
  if [ "$(id -u)" != 0 ] && ! docker info > /dev/null 2>&1; then
    if ! id -nG "$(id -un)" | tr ' ' '\n' | grep -qx docker; then
      echo ">>> Adding $(id -un) to the docker group (for the next sessions without sudo)"
      sudo usermod -aG docker "$(id -un)"
    fi
    # This session does not have the new group yet: run again inside it.
    if [ -z "${AERO_IN_DOCKER_GROUP:-}" ] && command -v sg > /dev/null; then
      export AERO_IN_DOCKER_GROUP=1
      exec sg docker -c "$(printf '%q ' "$0" "$@")"
    fi
  fi
}
[ -n "${AERO_SKIP_DOCKER_SETUP:-}" ] || ensure_docker "$@"
# Container user = host user, so files written in the mounted repository stay theirs.
# Under sudo (user outside the docker group) the real user is SUDO_UID/SUDO_GID; never root.
AERO_UID=$(id -u) AERO_GID=$(id -g)
if [ "$AERO_UID" = 0 ]; then
  AERO_UID=${SUDO_UID:-1000} AERO_GID=${SUDO_GID:-1000}
  [ "$AERO_UID" = 0 ] && AERO_UID=1000 AERO_GID=1000
  echo "Warning: running as root; container user with UID $AERO_UID." \
    "Better join the docker group: sudo usermod -aG docker \$USER (and open another session)." >&2
fi
export AERO_UID AERO_GID
RENDER_GID=$(getent group render | cut -d: -f3 || true)
VIDEO_GID=$(getent group video | cut -d: -f3 || true)
export RENDER_GID=${RENDER_GID:-109} VIDEO_GID=${VIDEO_GID:-44}
FILES=(-f "$DIR/compose.yaml")
mesa_gpu() {  # a render device Mesa can drive (Intel/AMD), not only NVIDIA ones
  local d
  for d in /sys/class/drm/renderD*; do
    [ -e "$d" ] && [ "$(basename "$(readlink -f "$d/device/driver")")" != nvidia ] && return 0
  done
  return 1
}
# NVIDIA in the container only with the NVIDIA container runtime (a GPU alone is not enough).
if command -v nvidia-smi > /dev/null && nvidia-smi -L > /dev/null 2>&1 &&
  docker info --format '{{json .Runtimes}}' 2> /dev/null | grep -q nvidia; then
  FILES+=(-f "$DIR/compose.nvidia.yaml")
elif ! mesa_gpu && "$DIR/nvidia_host.sh" available; then
  # NVIDIA GPU without the NVIDIA container runtime: driver devices and libraries mounted by hand.
  FILES+=(-f "$("$DIR/nvidia_host.sh" compose "${AERO_ISAAC_STATE:-$HOME/.local/state/aero/isaac}/../compose.nvidia-host.yaml")")
fi
if [ -n "${GUI:-}" ]; then
  export GZ_HEADLESS=0
  command -v xhost > /dev/null && xhost +si:localuser:"$(id -un)" > /dev/null
fi
# px4_msgs submodule (mounted from the host together with the code).
[ -f "$DIR/../ros2_ws/src/px4_msgs/package.xml" ] || git -C "$DIR/.." submodule update --init --recursive

# Compose v2: the docker plugin (`docker compose`) or the standalone v2 binary (`docker-compose`).
# Without it, `docker compose -f ...` fails with "unknown shorthand flag: 'f' in -f".
if docker compose version > /dev/null 2>&1; then
  COMPOSE=(docker compose)
elif command -v docker-compose > /dev/null &&
  [ "$(docker-compose version --short 2>/dev/null | sed 's/^v//' | cut -d. -f1)" -ge 2 ] 2>/dev/null; then
  COMPOSE=(docker-compose)
else
  cat >&2 <<'EOF'
Docker Compose v2 not found (needed by this project).
  Ubuntu with the docker.io package:   sudo apt install docker-compose-v2
  Docker from the official repository: sudo apt install docker-compose-plugin
Then check with: docker compose version
EOF
  exit 1
fi
compose() { "${COMPOSE[@]}" "${FILES[@]}" "$@"; }
case "${1:-gcs}" in
  image) shift; compose build "$@" ;;
  down) shift; compose down "$@" ;;
  up)
    mkdir -p "${AERO_ISAAC_STATE:-$HOME/.local/state/aero/isaac}/run" "${AERO_GCS_CONFIG:-$HOME/.config/aero_gcs}"
    FILES+=(-f "$DIR/compose.gcs.yaml"); compose up -d --no-build aero ;;
  stop) FILES+=(-f "$DIR/compose.gcs.yaml"); compose rm -sf aero ;;
  gcs) shift || true; compose run --rm --name aero_pursuit aero gcs "$@" ;;
  *) compose run --rm aero "$@" ;;
esac
