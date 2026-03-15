#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"

MAP_TOPIC="${MAP_TOPIC:-/map}"
POSE_TOPIC="${POSE_TOPIC:-/mavros/local_position/pose}"
RAW_GOAL_TOPIC="${RAW_GOAL_TOPIC:-/mission_goal}"
NAV_GOAL_TOPIC="${NAV_GOAL_TOPIC:-/drone_goal}"
PATH_TOPIC="${PATH_TOPIC:-/slam_nav/global_path}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u

echo "[run] slam_navigation_mode"
echo "[info] map_topic=${MAP_TOPIC}"
echo "[info] pose_topic=${POSE_TOPIC}"
echo "[info] raw_goal_topic=${RAW_GOAL_TOPIC}"
echo "[info] nav_goal_topic=${NAV_GOAL_TOPIC}"
echo "[info] path_topic=${PATH_TOPIC}"
echo "[hint] mission node should publish to ${RAW_GOAL_TOPIC} (e.g. remap /drone_goal:=/mission_goal)"

exec ros2 run obs_avoid slam_navigation_mode --ros-args \
  -p use_sim_time:="${USE_SIM_TIME}" \
  -p map_topic:="${MAP_TOPIC}" \
  -p pose_topic:="${POSE_TOPIC}" \
  -p raw_goal_topic:="${RAW_GOAL_TOPIC}" \
  -p nav_goal_topic:="${NAV_GOAL_TOPIC}" \
  -p path_topic:="${PATH_TOPIC}" "$@"
