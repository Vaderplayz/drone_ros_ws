#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 BAG_DIRECTORY [IMAGE_TOPIC] [CAMERA_INFO_TOPIC]" >&2
  exit 2
fi

BAG_PATH="$1"
IMAGE_TOPIC="${2:-/image_raw}"
CAMERA_INFO_TOPIC="${3:-/camera_info}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-${SCRIPT_DIR}/../../../install/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS workspace setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -e "${BAG_PATH}/metadata.yaml" ]]; then
  echo "[error] rosbag directory does not contain metadata.yaml: ${BAG_PATH}" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${ROS_SETUP}"
CONFIG="$(ros2 pkg prefix apriltag_precision_landing)/share/apriltag_precision_landing/config/apriltag_precision_landing.yaml"
DETECTOR_PID=""

cleanup() {
  if [[ -n "${DETECTOR_PID}" ]] && kill -0 "${DETECTOR_PID}" 2>/dev/null; then
    kill "${DETECTOR_PID}" 2>/dev/null || true
    wait "${DETECTOR_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[run] detector in replay mode"
ros2 run apriltag_precision_landing apriltag_camera_detector_node --ros-args \
  --params-file "${CONFIG}" \
  -p input_source:=ros_topics \
  -p image_topic:="${IMAGE_TOPIC}" \
  -p camera_info_topic:="${CAMERA_INFO_TOPIC}" \
  -p publish_image_stream:=false \
  -p use_sim_time:=true &
DETECTOR_PID="$!"

sleep 1
echo "[run] replaying ${BAG_PATH}"
ros2 bag play "${BAG_PATH}" --clock

