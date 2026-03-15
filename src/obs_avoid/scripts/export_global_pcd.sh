#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
SERVICE_NAME="${SERVICE_NAME:-/vertical_lidar_mapper/save_pcd}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u
export COLCON_TRACE="${COLCON_TRACE:-0}"

echo "[wait] waiting for service ${SERVICE_NAME} ..."
until ros2 service list 2>/dev/null | grep -qx "${SERVICE_NAME}"; do
  sleep 1
done

echo "[run] requesting global PCD export..."
ros2 service call "${SERVICE_NAME}" std_srvs/srv/Trigger "{}"
