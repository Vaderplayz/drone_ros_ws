#!/usr/bin/env bash
# Replay a mapping bag with the experimental mapper and optional slam_toolbox.
# shellcheck disable=SC1090,SC1091
set -euo pipefail

if (( $# < 1 )); then
  echo "Usage: $0 BAG_PATH [--with-slam-toolbox]" >&2
  exit 2
fi

BAG_PATH="$1"
shift
WITH_SLAM_TOOLBOX=0
if [[ "${1:-}" == "--with-slam-toolbox" ]]; then
  WITH_SLAM_TOOLBOX=1
fi

ROS_WS="${ROS_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
source /opt/ros/jazzy/setup.bash
source "${ROS_WS}/install/setup.bash"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${ROS_WS}/runtime_logs/submap_slam_comparison_${STAMP}"
mkdir -p "${OUT_DIR}"
PIDS=()

start_owned() {
  local log_file="$1"
  shift
  setsid "$@" >"${log_file}" 2>&1 &
  PIDS+=("$!")
}

cleanup() {
  set +e
  for ((i=${#PIDS[@]} - 1; i>=0; --i)); do
    kill -TERM -- "-${PIDS[$i]}" 2>/dev/null || true
  done
  wait || true
}
trap cleanup EXIT INT TERM

start_owned "${OUT_DIR}/submap_slam.log" \
  ros2 launch submap_slam_2d submap_slam_2d.launch.py use_sim_time:=true

if (( WITH_SLAM_TOOLBOX == 1 )); then
  start_owned "${OUT_DIR}/slam_toolbox.log" \
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
    slam_params_file:="${ROS_WS}/src/obs_avoid/config/slam2d_real_1lidar.yaml"
fi

start_owned "${OUT_DIR}/comparison_recorder.log" \
  ros2 run submap_slam_2d comparison_recorder.py "${OUT_DIR}"

TOPICS=(
  /clock /scan_slam /lidar/odom /mavros/local_position/odom /tf /tf_static
  /map /submap_slam/map /submap_slam/trajectory /submap_slam/corrected_pose
  /submap_slam/diagnostics
)
start_owned "${OUT_DIR}/output_bag.log" \
  ros2 bag record -o "${OUT_DIR}/comparison_output" "${TOPICS[@]}"

sleep 2
ros2 bag play "${BAG_PATH}" --clock 100
sleep 2
ros2 topic echo /submap_slam/diagnostics >"${OUT_DIR}/diagnostics.yaml" --once || true
cleanup
trap - EXIT INT TERM
echo "Replay outputs: ${OUT_DIR}"
