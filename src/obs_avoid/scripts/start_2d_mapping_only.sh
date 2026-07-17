#!/usr/bin/env bash
# Observer-only 2D mapping. Requires the independent RF2O/PX4 pipeline to be running.
# shellcheck disable=SC1090,SC1091

set -euo pipefail

ROS_WS_DEFAULT="/home/pi5drone/drone_ros_ws"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"

SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-${ROS_WS}/src/obs_avoid/config/slam2d_real_1lidar.yaml}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan_rf2o}"
ODOM_TOPIC="${ODOM_TOPIC:-/mavros/local_position/odom}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
MAP_FRAME="${MAP_FRAME:-map}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
LIDAR_FRAME="${LIDAR_FRAME:-laser_frame}"
INPUT_WAIT_SEC="${INPUT_WAIT_SEC:-60}"
MAP_WAIT_SEC="${MAP_WAIT_SEC:-120}"
PROCESS_STOP_TIMEOUT_SEC="${PROCESS_STOP_TIMEOUT_SEC:-5}"
MAX_INPUT_AGE_SEC="${MAX_INPUT_AGE_SEC:-10}"
MAX_INPUT_STAMP_DELTA_SEC="${MAX_INPUT_STAMP_DELTA_SEC:-10}"

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${ROS_WS}/runtime_logs/mapping_2d_${RUN_STAMP}"
MASTER_LOG="${LOG_DIR}/master.log"
SLAM_LOG="${LOG_DIR}/slam_toolbox.log"
SNAPSHOT_FILE="${LOG_DIR}/system_snapshot.txt"
MAP_SAVE_DIR="${MAP_SAVE_DIR:-${ROS_WS}/maps}"

STATE_ROOT="${XDG_RUNTIME_DIR:-/tmp}/mapping_2d_only_$(id -u)"
LOCK_FILE="${STATE_ROOT}/launcher.lock"
OWNER_FILE="${STATE_ROOT}/launcher.pid"
PROCESS_FILE="${STATE_ROOT}/slam_process.tsv"
BOOT_ID="$(< /proc/sys/kernel/random/boot_id)"

SLAM_PID=""
LOCK_ACQUIRED=0
SHUTDOWN_REASON="normal exit"
CONTROL_TOPICS=(
  /cmd_vel
  /planner_cmd_vel
  /mavros/setpoint_velocity/cmd_vel
  /mavros/setpoint_raw/local
)
CONTROL_COUNTS=()

if [[ -t 1 ]]; then
  GREEN=$'\033[1;32m'
  YELLOW=$'\033[1;33m'
  RED=$'\033[1;31m'
  RESET=$'\033[0m'
else
  GREEN=""
  YELLOW=""
  RED=""
  RESET=""
fi

timestamp() {
  date '+%Y-%m-%dT%H:%M:%S%z'
}

log() {
  printf '[%s] %s\n' "$(timestamp)" "$*"
}

log_started() {
  printf '%s[%s] STARTED %s%s\n' "${GREEN}" "$(timestamp)" "$*" "${RESET}"
}

log_warning() {
  printf '%s[%s] WARNING %s%s\n' "${YELLOW}" "$(timestamp)" "$*" "${RESET}"
}

log_error() {
  printf '%s[%s] ERROR %s%s\n' "${RED}" "$(timestamp)" "$*" "${RESET}" >&2
}

publisher_count() {
  local topic="$1" info count
  info="$(ros2 topic info "${topic}" -v 2>/dev/null || true)"
  count="$(awk '/Publisher count:/{print $3; exit}' <<<"${info}")"
  printf '%s\n' "${count:-0}"
}

validate_launcher_settings() {
  local value topic frame launch_arguments
  case "${USE_SIM_TIME}" in
    true|false) ;;
    *)
      log_error "USE_SIM_TIME must be exactly true or false, got '${USE_SIM_TIME}'"
      return 1 ;;
  esac
  for value in "${INPUT_WAIT_SEC}" "${MAP_WAIT_SEC}" "${PROCESS_STOP_TIMEOUT_SEC}" \
    "${MAX_INPUT_AGE_SEC}" "${MAX_INPUT_STAMP_DELTA_SEC}"; do
    if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
      log_error "timeout and timestamp limits must be positive integer seconds, got '${value}'"
      return 1
    fi
  done
  for topic in "${SCAN_TOPIC}" "${ODOM_TOPIC}" "${MAP_TOPIC}"; do
    if [[ ! "${topic}" =~ ^/[A-Za-z0-9_/]+$ || "${topic}" == *//* ||
      "${topic}" == */ ]]; then
      log_error "invalid absolute ROS topic name '${topic}'"
      return 1
    fi
  done
  for frame in "${MAP_FRAME}" "${ODOM_FRAME}" "${BASE_FRAME}" "${LIDAR_FRAME}"; do
    if [[ ! "${frame}" =~ ^[A-Za-z][A-Za-z0-9_/]*$ || "${frame}" == *//* ||
      "${frame}" == */ ]]; then
      log_error "invalid TF frame name '${frame}'"
      return 1
    fi
  done

  if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
    log_error "slam_toolbox package is unavailable"
    return 1
  fi
  if ! ros2 pkg executables slam_toolbox 2>/dev/null | \
    grep -q '^slam_toolbox async_slam_toolbox_node$'; then
    log_error "slam_toolbox async executable is unavailable"
    return 1
  fi
  launch_arguments="$(timeout 10 ros2 launch slam_toolbox online_async_launch.py \
    --show-args 2>/dev/null || true)"
  if [[ "${launch_arguments}" != *"'slam_params_file'"* ||
    "${launch_arguments}" != *"'use_sim_time'"* ]]; then
    log_error "installed online_async_launch.py lacks required launch arguments"
    return 1
  fi
  if ! ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
    log_warning "nav2_map_server is unavailable; live mapping works but map_saver_cli will not"
  fi
  log_started "launcher inputs and installed ROS interfaces validated"
}

process_group_alive() {
  local pid="$1"
  kill -0 -- "-${pid}" 2>/dev/null || kill -0 "${pid}" 2>/dev/null
}

signal_process_group() {
  local pid="$1" signal="$2"
  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill "-${signal}" -- "-${pid}" 2>/dev/null || true
  elif kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" "${pid}" 2>/dev/null || true
  fi
}

stop_slam() {
  local deadline
  [[ -n "${SLAM_PID}" ]] || return 0
  process_group_alive "${SLAM_PID}" || return 0

  log "Stopping mapping-owned slam_toolbox process group pgid=${SLAM_PID}"
  signal_process_group "${SLAM_PID}" TERM
  deadline=$(( $(date +%s) + PROCESS_STOP_TIMEOUT_SEC ))
  while (( $(date +%s) < deadline )); do
    process_group_alive "${SLAM_PID}" || break
    sleep 0.2
  done
  if process_group_alive "${SLAM_PID}"; then
    log_warning "forcing stopped slam_toolbox process group pgid=${SLAM_PID}"
    signal_process_group "${SLAM_PID}" KILL
  fi
  wait "${SLAM_PID}" 2>/dev/null || true
}

cleanup() {
  local exit_code=$?
  set +e
  trap - EXIT INT TERM
  log "Shutdown reason: ${SHUTDOWN_REASON}; launcher_exit=${exit_code}"
  stop_slam
  if [[ "${LOCK_ACQUIRED}" == "1" ]]; then
    : >"${PROCESS_FILE}"
    : >"${OWNER_FILE}"
  fi
  log "RF2O/PX4 fusion, MAVROS, LiDAR, camera, AprilTag, and flight control were not stopped"
  exit "${exit_code}"
}

on_signal() {
  SHUTDOWN_REASON="received signal $1"
  exit 130
}

recover_previous_mapping() {
  local recorded_pid recorded_boot deadline
  [[ -s "${PROCESS_FILE}" ]] || return 0
  IFS=$'\t' read -r recorded_pid recorded_boot <"${PROCESS_FILE}" || true
  [[ "${recorded_pid:-}" =~ ^[0-9]+$ ]] || return 0
  [[ "${recorded_boot:-}" == "${BOOT_ID}" ]] || return 0
  process_group_alive "${recorded_pid}" || return 0

  log_warning "recovering slam_toolbox process group from an interrupted mapping run"
  signal_process_group "${recorded_pid}" TERM
  deadline=$(( $(date +%s) + PROCESS_STOP_TIMEOUT_SEC ))
  while (( $(date +%s) < deadline )); do
    process_group_alive "${recorded_pid}" || break
    sleep 0.2
  done
  if process_group_alive "${recorded_pid}"; then
    signal_process_group "${recorded_pid}" KILL
  fi
  log_started "previous mapping-owned process group cleared"
}

acquire_mapping_lock() {
  mkdir -p "${STATE_ROOT}"
  exec 9>"${LOCK_FILE}"
  if ! flock -n 9; then
    log_error "another 2D mapping launcher is already running"
    [[ -s "${OWNER_FILE}" ]] && log_error "launcher pid=$(<"${OWNER_FILE}")"
    return 1
  fi
  LOCK_ACQUIRED=1
  printf '%s\n' "$$" >"${OWNER_FILE}"
  recover_previous_mapping
  : >"${PROCESS_FILE}"
}

wait_for_message() {
  local topic="$1" timeout_sec="$2" reliability="$3"
  local start elapsed last_progress=-1
  start="$(date +%s)"
  while true; do
    if timeout 3 ros2 topic echo "${topic}" --once \
      --qos-reliability "${reliability}" >/dev/null 2>&1; then
      log_started "live input ${topic}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      log_error "no message on required input ${topic} after ${timeout_sec}s"
      return 1
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${topic}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

wait_for_transform() {
  local parent="$1" child="$2" timeout_sec="$3"
  local start elapsed output last_progress=-1
  start="$(date +%s)"
  while true; do
    output="$(timeout 3 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 || true)"
    if grep -q -- '- Translation:' <<<"${output}"; then
      log_started "TF ${parent} -> ${child}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      log_error "missing required TF ${parent} -> ${child} after ${timeout_sec}s"
      return 1
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for TF ${parent} -> ${child}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

capture_control_baseline() {
  local topic
  CONTROL_COUNTS=()
  for topic in "${CONTROL_TOPICS[@]}"; do
    CONTROL_COUNTS+=("$(publisher_count "${topic}")")
  done
}

verify_no_control_publishers_added() {
  local index topic current changed=0
  for index in "${!CONTROL_TOPICS[@]}"; do
    topic="${CONTROL_TOPICS[$index]}"
    current="$(publisher_count "${topic}")"
    if [[ "${current}" != "${CONTROL_COUNTS[$index]}" ]]; then
      changed=1
      log_warning "external publisher count changed on ${topic}: ${CONTROL_COUNTS[$index]} -> ${current}"
    fi
  done
  if (( changed == 0 )); then
    log_started "no flight-control or planner publishers added"
  else
    log_warning "mapping launcher still owns only slam_toolbox; inspect independently changed control publishers"
  fi
}

require_clean_mapping_namespace() {
  local nodes existing_map_tf
  nodes="$(ros2 node list 2>/dev/null || true)"
  if grep -qx '/slam_toolbox' <<<"${nodes}"; then
    log_error "an unowned /slam_toolbox node is already running"
    return 1
  fi
  if [[ "$(publisher_count "${MAP_TOPIC}")" != "0" ]]; then
    log_error "${MAP_TOPIC} already has a publisher; refusing a second map source"
    ros2 topic info "${MAP_TOPIC}" -v 2>/dev/null || true
    return 1
  fi
  existing_map_tf="$(timeout 3 ros2 run tf2_ros tf2_echo \
    "${MAP_FRAME}" "${ODOM_FRAME}" 2>&1 || true)"
  if grep -q -- '- Translation:' <<<"${existing_map_tf}"; then
    log_error "TF ${MAP_FRAME} -> ${ODOM_FRAME} already exists; refusing a second mapping authority"
    return 1
  fi
}

topic_stamp_seconds() {
  local topic="$1" reliability="$2" sample stamp
  sample="$(timeout 5 ros2 topic echo "${topic}" --once --field header \
    --qos-reliability "${reliability}" 2>/dev/null || true)"
  stamp="$(awk '/^[[:space:]]*sec:/{print $2; exit}' <<<"${sample}")"
  [[ "${stamp}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${stamp}"
}

verify_live_timestamp_domain() {
  local scan_stamp odom_stamp delta wall_time scan_age odom_age
  scan_stamp="$(topic_stamp_seconds "${SCAN_TOPIC}" best_effort)" || {
    log_error "could not read a timestamp from ${SCAN_TOPIC}"
    return 1
  }
  odom_stamp="$(topic_stamp_seconds "${ODOM_TOPIC}" best_effort)" || {
    log_error "could not read a timestamp from ${ODOM_TOPIC}"
    return 1
  }

  delta=$((scan_stamp - odom_stamp))
  (( delta < 0 )) && delta=$((-delta))
  if (( delta > MAX_INPUT_STAMP_DELTA_SEC )); then
    log_error "input clock mismatch: ${SCAN_TOPIC}=${scan_stamp}, ${ODOM_TOPIC}=${odom_stamp}, delta=${delta}s"
    return 1
  fi

  if [[ "${USE_SIM_TIME}" == "false" ]]; then
    wall_time="$(date +%s)"
    scan_age=$((wall_time - scan_stamp))
    odom_age=$((wall_time - odom_stamp))
    (( scan_age < 0 )) && scan_age=$((-scan_age))
    (( odom_age < 0 )) && odom_age=$((-odom_age))
    if (( scan_age > MAX_INPUT_AGE_SEC || odom_age > MAX_INPUT_AGE_SEC )); then
      log_error "stale/non-wall-clock inputs: scan_age=${scan_age}s odom_age=${odom_age}s"
      return 1
    fi
  fi
  log_started "live scan and odometry timestamps share one clock domain"
}

validate_slam_node() {
  local node_info
  node_info="$(timeout 5 ros2 node info /slam_toolbox 2>/dev/null || true)"
  if [[ -z "${node_info}" ]]; then
    log_warning "slam_toolbox graph introspection unavailable; /map and full TF chain already verified"
    return 0
  fi
  if ! grep -qF "${SCAN_TOPIC}:" <<<"${node_info}"; then
    log_error "slam_toolbox is not subscribed to ${SCAN_TOPIC}"
    printf '%s\n' "${node_info}"
    return 1
  fi
  for topic in "${CONTROL_TOPICS[@]}"; do
    if grep -qF "${topic}:" <<<"${node_info}"; then
      log_error "slam_toolbox unexpectedly exposes control topic ${topic}"
      return 1
    fi
  done
  log_started "slam_toolbox scan subscription verified"
}

start_slam() {
  setsid ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="${SLAM_PARAMS_FILE}" use_sim_time:="${USE_SIM_TIME}" \
    9>&- >"${SLAM_LOG}" 2>&1 &
  SLAM_PID="$!"
  printf '%s\t%s\n' "${SLAM_PID}" "${BOOT_ID}" >"${PROCESS_FILE}"
  log_started "mapping-only slam_toolbox pgid=${SLAM_PID}"
}

wait_for_map() {
  local start elapsed last_progress=-1
  start="$(date +%s)"
  while true; do
    if ! kill -0 "${SLAM_PID}" 2>/dev/null; then
      log_error "slam_toolbox exited before publishing ${MAP_TOPIC}"
      tail -n 60 "${SLAM_LOG}" 2>/dev/null || true
      return 1
    fi
    if timeout 3 ros2 topic echo "${MAP_TOPIC}" --once \
      --qos-reliability reliable --qos-durability transient_local >/dev/null 2>&1; then
      log_started "2D occupancy map ${MAP_TOPIC}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= MAP_WAIT_SEC )); then
      log_error "slam_toolbox is alive but no ${MAP_TOPIC} message arrived after ${MAP_WAIT_SEC}s"
      tail -n 60 "${SLAM_LOG}" 2>/dev/null || true
      return 1
    fi
    if (( elapsed / 5 > last_progress )); then
      last_progress=$((elapsed / 5))
      log "Waiting for ${MAP_TOPIC}; elapsed=${elapsed}s deadline=${MAP_WAIT_SEC}s"
    fi
    sleep 1
  done
}

write_snapshot() {
  {
    printf 'timestamp=%s\nworkspace=%s\nuse_sim_time=%s\n' \
      "$(timestamp)" "${ROS_WS}" "${USE_SIM_TIME}"
    printf 'scan_topic=%s\nmap_topic=%s\nframes=%s->%s->%s->%s\n' \
      "${SCAN_TOPIC}" "${MAP_TOPIC}" "${MAP_FRAME}" "${ODOM_FRAME}" \
      "${BASE_FRAME}" "${LIDAR_FRAME}"
    printf '\n-- slam node --\n'
    timeout 5 ros2 node info /slam_toolbox || true
    printf '\n-- map topic --\n'
    timeout 5 ros2 topic info "${MAP_TOPIC}" -v || true
    printf '\n-- scan topic --\n'
    timeout 5 ros2 topic info "${SCAN_TOPIC}" -v || true
  } >"${SNAPSHOT_FILE}" 2>&1
}

ready_banner() {
  printf '\n%s' "${GREEN}"
  printf '######################################################################\n'
  printf '#                                                                    #\n'
  printf '#                         2D MAP READY                               #\n'
  printf '#                                                                    #\n'
  printf '#              OBSERVER ONLY - NO FLIGHT CONTROL                     #\n'
  printf '#                                                                    #\n'
  printf '######################################################################\n'
  printf '%s\n' "${RESET}"
}

main() {
  mkdir -p "${LOG_DIR}" "${MAP_SAVE_DIR}"
  touch "${MASTER_LOG}"
  # Keep the logger alive while the launcher handles Ctrl+C and terminates its
  # separately-grouped slam_toolbox child. It exits naturally when this pipe closes.
  exec > >(trap '' INT TERM; exec tee -a "${MASTER_LOG}") 2>&1
  trap cleanup EXIT
  trap 'on_signal INT' INT
  trap 'on_signal TERM' TERM

  if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "${ROS_SETUP}" ]]; then
    log_error "ROS setup is missing"
    exit 1
  fi
  if [[ ! -f "${SLAM_PARAMS_FILE}" ]]; then
    log_error "SLAM configuration is missing: ${SLAM_PARAMS_FILE}"
    exit 1
  fi

  set +u
  source /opt/ros/jazzy/setup.bash
  source "${ROS_SETUP}"
  set -u

  for command in ros2 timeout setsid flock awk grep tee; do
    if ! command -v "${command}" >/dev/null 2>&1; then
      log_error "missing command: ${command}"
      exit 1
    fi
  done

  validate_launcher_settings

  log "Independent observer-only 2D mapping startup"
  log "This launcher will only start slam_toolbox"
  log "It will not manage RF2O/PX4 fusion, MAVROS, LiDAR, camera, AprilTag, planner, or flight control"

  acquire_mapping_lock
  require_clean_mapping_namespace
  capture_control_baseline

  if [[ "$(publisher_count "${SCAN_TOPIC}")" != "1" ]]; then
    log_error "${SCAN_TOPIC} must have exactly one publisher from the fusion pipeline"
    ros2 topic info "${SCAN_TOPIC}" -v 2>/dev/null || true
    exit 1
  fi
  if [[ "${USE_SIM_TIME}" == "true" ]]; then
    wait_for_message /clock "${INPUT_WAIT_SEC}" best_effort
  fi
  wait_for_message "${SCAN_TOPIC}" "${INPUT_WAIT_SEC}" best_effort
  wait_for_message "${ODOM_TOPIC}" "${INPUT_WAIT_SEC}" best_effort
  verify_live_timestamp_domain
  wait_for_transform "${ODOM_FRAME}" "${BASE_FRAME}" "${INPUT_WAIT_SEC}"
  wait_for_transform "${BASE_FRAME}" "${LIDAR_FRAME}" "${INPUT_WAIT_SEC}"

  start_slam
  wait_for_map
  wait_for_transform "${MAP_FRAME}" "${ODOM_FRAME}" "${INPUT_WAIT_SEC}"
  wait_for_transform "${MAP_FRAME}" "${LIDAR_FRAME}" "${INPUT_WAIT_SEC}"
  validate_slam_node
  verify_no_control_publishers_added
  write_snapshot
  ready_banner

  log "Move the vehicle manually to build the map; this process sends no vehicle commands"
  log "Save map: ros2 run nav2_map_server map_saver_cli -f ${MAP_SAVE_DIR}/indoor_map"
  log "Runtime logs: ${LOG_DIR}"

  while kill -0 "${SLAM_PID}" 2>/dev/null; do
    sleep 2
  done
  SHUTDOWN_REASON="slam_toolbox exited unexpectedly"
  log_error "${SHUTDOWN_REASON}"
  return 1
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
