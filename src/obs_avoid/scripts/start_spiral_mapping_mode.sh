#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
ENABLE_MISSION_OBS_AVOID="${ENABLE_MISSION_OBS_AVOID:-false}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u

echo "[info] start_spiral_mapping_mode.sh launches flight mission control only."
echo "[info] it is not part of mapping pipeline startup."

if [[ "${ENABLE_MISSION_OBS_AVOID,,}" == "1" || "${ENABLE_MISSION_OBS_AVOID,,}" == "true" || "${ENABLE_MISSION_OBS_AVOID,,}" == "yes" || "${ENABLE_MISSION_OBS_AVOID,,}" == "on" ]]; then
  exec ros2 run obs_avoid spiral_mapping_mode --ros-args -p use_sim_time:="${USE_SIM_TIME}" \
    -r /mavros/setpoint_velocity/cmd_vel:=/offboard_stack/cmd_vel "$@"
fi

exec ros2 run obs_avoid spiral_mapping_mode --ros-args -p use_sim_time:="${USE_SIM_TIME}" "$@"
