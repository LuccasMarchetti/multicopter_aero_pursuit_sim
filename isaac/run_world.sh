#!/usr/bin/env bash
# Starts isaac/aero_world.py with the Python of Isaac Sim (host side; isaac/host.sh up|world).
# Arguments go to aero_world.py (--help). Environment: ~/isaac_env.sh (isaac/setup_host.sh).
set -eo pipefail
source "$HOME/isaac_env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Same Fast DDS profile as the aero-isaac container (large shared-memory segments for the raw
# camera images, loopback-only UDP discovery): fastdds_shm.xml.
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/fastdds_shm.xml"
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY
bash "$HERE/kill_world.sh" >/dev/null 2>&1 || true
# Shared machine: a memory cap so a runaway never reaches the kernel OOM killer of everyone.
MEM_MAX="${AERO_ISAAC_MEM_MAX:-48G}"
if systemd-run --user --scope -q true 2>/dev/null; then
  exec systemd-run --user --scope -q -p MemoryMax="$MEM_MAX" -p MemorySwapMax=0 \
    "$ISAACSIM_PYTHON" "$HERE/aero_world.py" "$@"
fi
exec "$ISAACSIM_PYTHON" "$HERE/aero_world.py" "$@"
