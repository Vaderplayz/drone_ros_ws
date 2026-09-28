#!/usr/bin/env bash
# Bench launcher for the LD19 only. It sends no flight or PX4 commands.

set -euo pipefail
# shellcheck disable=SC1090,SC1091

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
LD19_PORT="${LD19_PORT:-/dev/ldlidar_vertical}"
LD19_BAUDRATE="${LD19_BAUDRATE:-230400}"
LD19_TOPIC="${LD19_TOPIC:-/scan_vertical}"
LD19_FRAME_ID="${LD19_FRAME_ID:-lidar_vert_link}"
LD19_NODE_NAME="${LD19_NODE_NAME:-ld19_vertical}"

for setup in /opt/ros/jazzy/setup.bash "${ROS_SETUP}"; do
  if [[ ! -f "${setup}" ]]; then
    echo "[error] missing setup: ${setup}" >&2
    exit 1
  fi
done

set +u
source /opt/ros/jazzy/setup.bash
source "${ROS_SETUP}"
set -u

if ! ros2 pkg prefix ldlidar_stl_ros2 >/dev/null 2>&1; then
  echo "[error] ldlidar_stl_ros2 is not built in ${ROS_WS}" >&2
  echo "        colcon build --packages-select ldlidar_stl_ros2 --symlink-install" >&2
  exit 1
fi
if [[ ! -r "${LD19_PORT}" || ! -w "${LD19_PORT}" ]]; then
  echo "[error] ${LD19_PORT} is absent or not readable/writable by $(id -un)" >&2
  echo "        sudo ${SCRIPT_DIR}/configure_ld19_vertical_device.sh /dev/ttyUSB<N>" >&2
  exit 1
fi

echo "[run] LD19 ${LD19_PORT}@${LD19_BAUDRATE} -> ${LD19_TOPIC} (${LD19_FRAME_ID})"
exec ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
  -r __node:="${LD19_NODE_NAME}" \
  -p product_name:=LDLiDAR_LD19 \
  -p topic_name:="${LD19_TOPIC}" \
  -p frame_id:="${LD19_FRAME_ID}" \
  -p port_name:="${LD19_PORT}" \
  -p port_baudrate:="${LD19_BAUDRATE}" \
  -p laser_scan_dir:=true \
  -p enable_angle_crop_func:=false

