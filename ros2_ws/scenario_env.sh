#!/bin/bash
# Library of the simulation scripts: resolves $SCENARIO into an absolute path and validates it.
#   SCENARIO_PATH       absolute path of the scenario (empty without SCENARIO)

scenario_env() {  # scenario_env <repo root> <workspace>
  local repo=$1 ws=$2 tool
  SCENARIO_PATH=""
  [ -n "${SCENARIO:-}" ] || return 0
  if [ -f "$SCENARIO" ]; then
    SCENARIO_PATH=$(readlink -f "$SCENARIO")
  elif [ -f "$repo/scenarios/$SCENARIO" ]; then
    SCENARIO_PATH="$repo/scenarios/$SCENARIO"
  elif [ -f "$repo/scenarios/$SCENARIO.yaml" ]; then
    SCENARIO_PATH="$repo/scenarios/$SCENARIO.yaml"
  else
    echo "Scenario not found: $SCENARIO" >&2
    return 1
  fi
  tool="$ws/install/aero_pursuit/lib/aero_pursuit/scenario_tool"
  if ! "$tool" validate "$SCENARIO_PATH" > /dev/null; then
    "$tool" validate "$SCENARIO_PATH" >&2
    return 1
  fi
  export SCENARIO_PATH
  echo "Scenario: $SCENARIO_PATH"
}
