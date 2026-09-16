#!/usr/bin/env bash
# Start the real camera, AprilTag detector, and MAVROS landing-target publisher.
# MAVROS and the ground-control supervisor are expected to be managed separately.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${ROS_WS:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
APRILTAG_SCRIPT="${APRILTAG_SCRIPT:-${ROS_WS}/src/apriltag_precision_landing/scripts/start_real_apriltag_pipeline.sh}"
STATE_ROOT="${XDG_RUNTIME_DIR:-/tmp}/camera_tag_detection_$(id -u)"

if [[ ! -x "${APRILTAG_SCRIPT}" ]]; then
  echo "[error] AprilTag launcher is missing or not executable: ${APRILTAG_SCRIPT}" >&2
  exit 1
fi
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS workspace setup is missing: ${ROS_SETUP}" >&2
  exit 1
fi

mkdir -p "${STATE_ROOT}"
exec 9>"${STATE_ROOT}/launcher.lock"
if ! flock -n 9; then
  echo "[info] camera and tag detection launcher is already running"
  exit 0
fi
printf '%s\n' "$$" >"${STATE_ROOT}/launcher.pid"

export ROS_WS ROS_SETUP
export START_MAVROS=0
export START_CAMERA=0
export START_IMAGE_VIEW=0
export START_SYSTEM_MONITOR=0
export START_GROUND_CONTROL_SUPERVISOR=0
export KILL_BEFORE_LAUNCH=0

exec "${APRILTAG_SCRIPT}"
