#!/usr/bin/env bash
# Stops aero_world.py and the PX4 SITL it launched (Pegasus leaves PX4 running if Isaac dies).
pkill -f "[a]ero_world.py" || true
pkill -f "PX4-Autopilot/build/px4_sitl_default/bin/[p]x4" || true
for _ in 1 2 3 4 5; do
  pgrep -f "[a]ero_world.py|px4_sitl_default/bin/[p]x4" >/dev/null || exit 0
  sleep 1
done
pkill -9 -f "[a]ero_world.py" || true
pkill -9 -f "PX4-Autopilot/build/px4_sitl_default/bin/[p]x4" || true
