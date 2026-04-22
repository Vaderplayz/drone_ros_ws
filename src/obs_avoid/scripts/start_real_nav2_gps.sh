#!/usr/bin/env bash

set -euo pipefail

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[error] missing command: $1" >&2
    exit 1
  fi
}

is_true() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
      return 0
    fi
    if (( "$(date +%s)" - start_ts >= timeout_sec )); then
      echo "[error] timed out waiting for topic: ${topic}" >&2
      return 1
    fi
    sleep 1
  done
}

kill_if_running() {
  local pattern="$1"
  pgrep -af "${pattern}" >/dev/null 2>&1 || return 0
  pkill -f "${pattern}" >/dev/null 2>&1 || true
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS_DEFAULT="$(cd "${PKG_DIR}/../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
KILL_BEFORE_LAUNCH="${KILL_BEFORE_LAUNCH:-1}"
GPS_TOPIC="${GPS_TOPIC:-/mavros/global_position/global}"
GPS_FALLBACK_TO_SLAM="${GPS_FALLBACK_TO_SLAM:-1}"

MAVROS_LAUNCH_FILE="${MAVROS_LAUNCH_FILE:-px4.launch}"
FCU_URL="${FCU_URL:-serial:///dev/ttyACM0:921600}"
GCS_URL="${GCS_URL:-}"
MAVROS_RESPAWN="${MAVROS_RESPAWN:-true}"

ENABLE_RPLIDAR="${ENABLE_RPLIDAR:-0}"
RPLIDAR_SERIAL_PORT="${RPLIDAR_SERIAL_PORT:-/dev/ttyUSB0}"
RPLIDAR_BAUDRATE="${RPLIDAR_BAUDRATE:-115200}"
RPLIDAR_FRAME_ID="${RPLIDAR_FRAME_ID:-laser_frame}"
RPLIDAR_INVERTED="${RPLIDAR_INVERTED:-false}"
RPLIDAR_ANGLE_COMPENSATE="${RPLIDAR_ANGLE_COMPENSATE:-true}"

RL_PARAMS_FILE="${RL_PARAMS_FILE:-${PKG_DIR}/config/nav2_gps_dual_ekf.yaml}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${PKG_DIR}/config/nav2_gps_nav2_params.yaml}"
NAV2_USE_COMPOSITION="${NAV2_USE_COMPOSITION:-false}"
NAV2_AUTOSTART="${NAV2_AUTOSTART:-true}"

CMD_BRIDGE_INPUT_TOPIC="${CMD_BRIDGE_INPUT_TOPIC:-/cmd_vel}"
CMD_BRIDGE_OUTPUT_TOPIC="${CMD_BRIDGE_OUTPUT_TOPIC:-/planner_cmd_vel}"
CMD_BRIDGE_FRAME_ID="${CMD_BRIDGE_FRAME_ID:-map}"

MAVROS_LOG="${MAVROS_LOG:-/tmp/mavros_nav2_gps.log}"
RL_LOG="${RL_LOG:-/tmp/robot_localization_nav2_gps.log}"
NAV2_LOG="${NAV2_LOG:-/tmp/nav2_gps.log}"
CMD_BRIDGE_LOG="${CMD_BRIDGE_LOG:-/tmp/nav2_cmd_bridge.log}"
USER_CTRL_LOG="${USER_CTRL_LOG:-/tmp/user_ctrl_nav2_gps.log}"
RPLIDAR_LOG="${RPLIDAR_LOG:-/tmp/rplidar_nav2_gps.log}"
SLAM_FALLBACK_SCRIPT="${SLAM_FALLBACK_SCRIPT:-${SCRIPT_DIR}/start_real_basic_2d.sh}"

PIDS=()
NAMES=()

cleanup() {
  set +e
  local i
  for i in "${!PIDS[@]}"; do
    if kill -0 "${PIDS[$i]}" >/dev/null 2>&1; then
      echo "[stop] ${NAMES[$i]} (pid=${PIDS[$i]})"
      kill "${PIDS[$i]}" >/dev/null 2>&1 || true
    fi
  done
}

add_process() {
  local pid="$1"
  local name="$2"
  PIDS+=("${pid}")
  NAMES+=("${name}")
}

fallback_to_slam_stack() {
  if ! is_true "${GPS_FALLBACK_TO_SLAM}"; then
    echo "[error] GPS is unavailable and GPS_FALLBACK_TO_SLAM=0. Exiting." >&2
    return 1
  fi
  if [[ ! -f "${SLAM_FALLBACK_SCRIPT}" ]]; then
    echo "[error] fallback script not found: ${SLAM_FALLBACK_SCRIPT}" >&2
    return 1
  fi

  echo "[warn] GPS topic not available on ${GPS_TOPIC}. Falling back to SLAM-only stack."

  # Stop started processes before handing off to the SLAM stack launcher.
  cleanup
  trap - EXIT INT TERM

  export ROS_WS ROS_SETUP WAIT_TIMEOUT_SEC
  export MAVROS_LAUNCH_FILE FCU_URL MAVROS_RESPAWN
  export ENABLE_RPLIDAR RPLIDAR_SERIAL_PORT RPLIDAR_BAUDRATE
  export RPLIDAR_FRAME_ID RPLIDAR_INVERTED RPLIDAR_ANGLE_COMPENSATE
  export KILL_BEFORE_LAUNCH=1

  exec "${SLAM_FALLBACK_SCRIPT}"
}

kill_existing_stack() {
  kill_if_running "ros2 launch mavros ${MAVROS_LAUNCH_FILE}"
  kill_if_running "/mavros_node"
  kill_if_running "ros2 run rplidar_ros rplidar_composition"
  kill_if_running "ros2 launch obs_avoid nav2_gps_localization.launch.py"
  kill_if_running "ros2 launch nav2_bringup navigation_launch.py"
  kill_if_running "ros2 run obs_avoid nav2_cmd_vel_bridge"
  kill_if_running "ros2 run obs_avoid user_ctrl"
  kill_if_running "ros2 run obs_avoid local_planner_mode_a"
  kill_if_running "ros2 run obs_avoid local_planner_sector_mode"
  kill_if_running "ros2 run obs_avoid local_planner_hybrid_mode"
  kill_if_running "ros2 run obs_avoid slam_navigation_mode"
}

start_mavros() {
  local cmd=(ros2 launch mavros "${MAVROS_LAUNCH_FILE}"
    "fcu_url:=${FCU_URL}"
    "respawn_mavros:=${MAVROS_RESPAWN}"
    "use_sim_time:=false")
  if [[ -n "${GCS_URL}" ]]; then
    cmd+=("gcs_url:=${GCS_URL}")
  fi
  echo "[run] MAVROS -> ${MAVROS_LOG}"
  "${cmd[@]}" >"${MAVROS_LOG}" 2>&1 &
  add_process "$!" "mavros"
}

start_rplidar() {
  if ! is_true "${ENABLE_RPLIDAR}"; then
    return
  fi
  echo "[run] rplidar_composition -> ${RPLIDAR_LOG}"
  ros2 run rplidar_ros rplidar_composition --ros-args \
    -p channel_type:=serial \
    -p serial_port:="${RPLIDAR_SERIAL_PORT}" \
    -p serial_baudrate:="${RPLIDAR_BAUDRATE}" \
    -p frame_id:="${RPLIDAR_FRAME_ID}" \
    -p inverted:="${RPLIDAR_INVERTED}" \
    -p angle_compensate:="${RPLIDAR_ANGLE_COMPENSATE}" \
    -p topic_name:=scan >"${RPLIDAR_LOG}" 2>&1 &
  add_process "$!" "rplidar"
}

start_rl() {
  echo "[run] robot_localization (dual EKF + navsat) -> ${RL_LOG}"
  ros2 launch obs_avoid nav2_gps_localization.launch.py \
    use_sim_time:=false \
    params_file:="${RL_PARAMS_FILE}" >"${RL_LOG}" 2>&1 &
  add_process "$!" "robot_localization"
}

start_nav2() {
  echo "[run] nav2_bringup navigation_launch.py -> ${NAV2_LOG}"
  ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    autostart:="${NAV2_AUTOSTART}" \
    use_composition:="${NAV2_USE_COMPOSITION}" \
    params_file:="${NAV2_PARAMS_FILE}" >"${NAV2_LOG}" 2>&1 &
  add_process "$!" "nav2"
}

start_cmd_bridge() {
  echo "[run] nav2_cmd_vel_bridge -> ${CMD_BRIDGE_LOG}"
  ros2 run obs_avoid nav2_cmd_vel_bridge --ros-args \
    -p input_topic:="${CMD_BRIDGE_INPUT_TOPIC}" \
    -p output_topic:="${CMD_BRIDGE_OUTPUT_TOPIC}" \
    -p frame_id:="${CMD_BRIDGE_FRAME_ID}" >"${CMD_BRIDGE_LOG}" 2>&1 &
  add_process "$!" "nav2_cmd_bridge"
}

start_user_ctrl() {
  echo "[run] user_ctrl (PX4 OFFBOARD bridge mode) -> ${USER_CTRL_LOG}"
  ros2 run obs_avoid user_ctrl --ros-args \
    -p use_sim_time:=false \
    -p ask_goal_on_start:=false \
    -p print_input_help_on_start:=false \
    -p enable_internal_goal_nav:=false \
    -p planner_cmd_timeout_sec:=0.5 >"${USER_CTRL_LOG}" 2>&1 &
  add_process "$!" "user_ctrl"
}

main() {
  require_cmd ros2

  if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
  fi
  if [[ ! -f "${RL_PARAMS_FILE}" ]]; then
    echo "[error] robot_localization params file not found: ${RL_PARAMS_FILE}" >&2
    exit 1
  fi
  if [[ ! -f "${NAV2_PARAMS_FILE}" ]]; then
    echo "[error] nav2 params file not found: ${NAV2_PARAMS_FILE}" >&2
    exit 1
  fi

  set +u
  source "${ROS_SETUP}"
  set -u

  if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
    echo "[error] mavros package not found in current overlay." >&2
    exit 1
  fi
  if ! ros2 pkg prefix obs_avoid >/dev/null 2>&1; then
    echo "[error] obs_avoid package not found in current overlay." >&2
    exit 1
  fi
  if ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
    echo "[error] robot_localization package not found." >&2
    exit 1
  fi
  if ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
    echo "[error] nav2_bringup package not found." >&2
    exit 1
  fi
  if is_true "${ENABLE_RPLIDAR}" && ! ros2 pkg prefix rplidar_ros >/dev/null 2>&1; then
    echo "[error] ENABLE_RPLIDAR=1 but rplidar_ros package not found." >&2
    exit 1
  fi

  if [[ "${KILL_BEFORE_LAUNCH}" == "1" ]]; then
    echo "[prep] stopping stale nav2+gps stack processes (if any)"
    kill_existing_stack
    sleep 1
  fi

  trap cleanup EXIT INT TERM

  start_mavros
  echo "[wait] /mavros/state"
  wait_for_topic "/mavros/state" "${WAIT_TIMEOUT_SEC}"
  echo "[wait] /mavros/local_position/odom"
  wait_for_topic "/mavros/local_position/odom" "${WAIT_TIMEOUT_SEC}"
  echo "[wait] /mavros/imu/data"
  wait_for_topic "/mavros/imu/data" "${WAIT_TIMEOUT_SEC}"
  echo "[wait] ${GPS_TOPIC}"
  if ! wait_for_topic "${GPS_TOPIC}" "${WAIT_TIMEOUT_SEC}"; then
    fallback_to_slam_stack
  fi

  start_rplidar
  if is_true "${ENABLE_RPLIDAR}"; then
    echo "[wait] /scan"
    wait_for_topic "/scan" "${WAIT_TIMEOUT_SEC}"
  fi

  start_rl
  echo "[wait] /odometry/local"
  wait_for_topic "/odometry/local" "${WAIT_TIMEOUT_SEC}"
  echo "[wait] /odometry/gps"
  wait_for_topic "/odometry/gps" "${WAIT_TIMEOUT_SEC}"

  start_nav2
  echo "[wait] /cmd_vel"
  wait_for_topic "/cmd_vel" "${WAIT_TIMEOUT_SEC}"

  start_cmd_bridge
  start_user_ctrl

  echo "[ok] Nav2 + GPS fusion stack started"
  echo "[info] This stack uses Nav2 for global planning + obstacle avoidance (no local_planner_mode_a)."
  echo "[info] Ensure PX4 is OFFBOARD + armed (user_ctrl handles requests automatically)."
  echo "[info] logs:"
  echo "  - ${MAVROS_LOG}"
  echo "  - ${RL_LOG}"
  echo "  - ${NAV2_LOG}"
  echo "  - ${CMD_BRIDGE_LOG}"
  echo "  - ${USER_CTRL_LOG}"
  if is_true "${ENABLE_RPLIDAR}"; then
    echo "  - ${RPLIDAR_LOG}"
  fi
  echo "[check] ros2 topic hz /cmd_vel"
  echo "[check] ros2 topic hz /planner_cmd_vel"
  echo "[check] ros2 topic hz /mavros/setpoint_velocity/cmd_vel"

  wait
}

main "$@"
