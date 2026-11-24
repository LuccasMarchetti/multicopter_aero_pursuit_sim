#!/usr/bin/env bash
# One-time install on the Isaac Sim computer (Ubuntu 22.04, NVIDIA driver >= 550, Docker):
#   - Isaac Sim 5.1 in ~/isaacsim51 (copied from ISAACSIM_SRC when it is not there yet)
#   - Pegasus Simulator v5.1.0 in ~/PegasusSimulator, installed in the Python of Isaac
#   - PX4-Autopilot at the commit of docker/Dockerfile, px4_sitl_default (MAVLink HIL for Pegasus)
#   - ~/isaac_env.sh (Isaac paths + internal ROS 2 Humble libraries of the bridge)
# Needs sudo once for the PX4 build tools (ninja, ccache, astyle). Idempotent.
set -euo pipefail
ISAACSIM_SRC="${ISAACSIM_SRC:-}"   # an existing Isaac Sim 5.1 folder to copy, when ~/isaacsim51 is missing
PX4_REF=ff5b9484369b714763db9638517c08df0c242237
PX4_TAG=v1.18.0-beta1
PEGASUS_REF=v5.1.0

if [ ! -x "$HOME/isaacsim51/python.sh" ]; then
  [ -n "$ISAACSIM_SRC" ] || { echo "Isaac Sim 5.1 not found in ~/isaacsim51: extract the NVIDIA workstation package there or run with ISAACSIM_SRC=<folder of an installation>" >&2; exit 1; }
  echo "== Isaac Sim: copiando $ISAACSIM_SRC"
  cp -a "$ISAACSIM_SRC" "$HOME/isaacsim51" 2>/dev/null || true   # LICENSE files without read access are skipped
fi

cat > "$HOME/isaac_env.sh" <<'ENV'
# Isaac Sim 5.1 + Pegasus (aero_pursuit). Internal ROS 2 Jazzy libraries of the Isaac bridge:
# same Fast DDS (2.14) as the aero-isaac container, so the raw camera images go through shared
# memory (Humble's Fast DDS 2.6 cannot share memory with it and grows without bound over UDP).
# Isaac Python is 3.11: do not source /opt/ros/humble in the same shell.
export ISAACSIM_PATH=$HOME/isaacsim51
export ISAACSIM_PYTHON=$ISAACSIM_PATH/python.sh
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ISAACSIM_PATH/exts/isaacsim.ros2.bridge/jazzy/lib
# Shared lab machine: own domain, discovery on this host only (run_world.sh and the container
# also load isaac/fastdds_shm.xml).
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV
source "$HOME/isaac_env.sh"

if [ ! -d "$HOME/PegasusSimulator" ]; then
  echo "== Pegasus $PEGASUS_REF"
  git clone -q --depth 1 --branch "$PEGASUS_REF" https://github.com/PegasusSimulator/PegasusSimulator.git "$HOME/PegasusSimulator"
fi
"$ISAACSIM_PYTHON" -m pip show pegasus-simulator >/dev/null 2>&1 || \
  (cd "$HOME/PegasusSimulator/extensions" && "$ISAACSIM_PYTHON" -m pip install -q --editable pegasus.simulator)

for p in ninja-build ccache astyle; do
  dpkg -s "$p" >/dev/null 2>&1 || NEED_APT="${NEED_APT:-} $p"
done
if [ -n "${NEED_APT:-}" ]; then
  echo "== apt:$NEED_APT"; sudo apt-get install -y -qq $NEED_APT
fi
# Larger UDP socket buffers for DDS (images fall back to UDP when shared memory is unavailable).
if [ "$(sysctl -n net.core.rmem_max)" -lt 8388608 ]; then
  printf 'net.core.rmem_max=8388608\nnet.core.wmem_max=8388608\n' | sudo tee /etc/sysctl.d/60-aero-dds.conf >/dev/null
  sudo sysctl -q --system
fi
pip3 install --user -q kconfiglib jinja2 pyros-genmsg jsonschema future pyyaml numpy packaging toml pyserial psutil lark empy==3.3.4

if [ ! -x "$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4" ]; then
  echo "== PX4 $PX4_TAG ($PX4_REF)"
  mkdir -p "$HOME/PX4-Autopilot" && cd "$HOME/PX4-Autopilot"
  [ -d .git ] || { git init -q . && git remote add origin https://github.com/PX4/PX4-Autopilot.git; }
  git fetch -q --depth 1 origin "$PX4_REF" && git checkout -q FETCH_HEAD && git tag -f "$PX4_TAG"
  git submodule update --init --recursive --depth 1 --jobs 8
  make px4_sitl_default
fi
echo "== ready: Isaac $(cut -d+ -f1 "$HOME/isaacsim51/VERSION"), Pegasus $PEGASUS_REF, PX4 $PX4_TAG"
