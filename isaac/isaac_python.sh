#!/usr/bin/env bash
# Runs a script with the Python of Isaac Sim (environment: ~/isaac_env.sh, isaac/setup_host.sh).
set -eo pipefail
source "$HOME/isaac_env.sh"
exec "$ISAACSIM_PYTHON" "$@"
