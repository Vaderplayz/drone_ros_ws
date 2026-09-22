#!/usr/bin/env bash
# Start the conservative real-hardware planner and fail-closed spatial guard.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"

exec env \
  ROS_WS="${ROS_WS}" \
  USE_SIM_TIME=false \
  PLANNER_NODE=local_planner_mode_a \
  PLANNER_PARAMS_FILE="${ROS_WS}/src/obs_avoid/config/local_planner_mode_a_real_safe.yaml" \
  GUARD_PARAMS_FILE="${ROS_WS}/src/obs_avoid/config/spatial_command_guard_real.yaml" \
  "${ROS_WS}/src/obs_avoid/scripts/start_flight_mode.sh"
