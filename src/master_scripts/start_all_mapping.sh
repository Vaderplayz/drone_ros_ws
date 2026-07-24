#!/usr/bin/env bash
# Start RF2O/PX4 fusion, 2D SLAM, and vertical-lidar 3D mapping in order.
# shellcheck disable=SC1090,SC1091

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="/home/pi5drone/drone_ros_ws"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

FUSION_SCRIPT="${FUSION_SCRIPT:-${SCRIPT_DIR}/start_rf2o_px4_fusion.sh}"
MAPPING_2D_SCRIPT="${MAPPING_2D_SCRIPT:-${SCRIPT_DIR}/start_2d_mapping_only.sh}"
MAPPING_3D_SCRIPT="${MAPPING_3D_SCRIPT:-${SCRIPT_DIR}/start_real_3d_mapping_lidar2.sh}"

FUSION_SCAN_TOPIC="${FUSION_SCAN_TOPIC:-/scan_rf2o}"
FUSION_ODOM_TOPIC="${FUSION_ODOM_TOPIC:-/lidar/odom}"
FUSION_BRIDGE_DIAGNOSTICS_TOPIC="${FUSION_BRIDGE_DIAGNOSTICS_TOPIC:-/lidar_odom_px4_bridge/diagnostics}"
PX4_ODOMETRY_OUT_TOPIC="${PX4_ODOMETRY_OUT_TOPIC:-/mavros/odometry/out}"
PX4_ODOM_TOPIC="${PX4_ODOM_TOPIC:-/mavros/local_position/odom}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
GLOBAL_CLOUD_TOPIC="${GLOBAL_CLOUD_TOPIC:-/mapping/global_cloud}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
MAP_FRAME="${MAP_FRAME:-map}"

FUSION_WAIT_SEC="${FUSION_WAIT_SEC:-120}"
FUSION_BRIDGE_OUTPUT_WAIT_SEC="${FUSION_BRIDGE_OUTPUT_WAIT_SEC:-15}"
MAPPING_2D_WAIT_SEC="${MAPPING_2D_WAIT_SEC:-180}"
MAPPING_3D_WAIT_SEC="${MAPPING_3D_WAIT_SEC:-120}"
CHILD_STOP_TIMEOUT_SEC="${CHILD_STOP_TIMEOUT_SEC:-50}"
REQUIRE_PX4_BRIDGE_OUTPUT="${REQUIRE_PX4_BRIDGE_OUTPUT:-0}"
MAPPING_3D_ENABLE_MAP_REBASE="${MAPPING_3D_ENABLE_MAP_REBASE:-false}"
MAPPING_3D_ENABLE_RELATIVE_POSE_GATE="${MAPPING_3D_ENABLE_RELATIVE_POSE_GATE:-true}"

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${ROS_WS}/runtime_logs/all_mapping_${RUN_STAMP}"
MASTER_LOG="${LOG_DIR}/master.log"
STATE_ROOT="${XDG_RUNTIME_DIR:-/tmp}/all_mapping_$(id -u)"
LOCK_FILE="${STATE_ROOT}/launcher.lock"

FUSION_PID=""
MAPPING_2D_PID=""
MAPPING_3D_PID=""
SHUTDOWN_REASON="normal exit"
CLEANING_UP=0

timestamp() {
  date '+%Y-%m-%d %H:%M:%S'
}

log() {
  printf '[%s] [all-mapping] %s\n' "$(timestamp)" "$*"
}

fail() {
  log "ERROR: $*"
  return 1
}

child_alive() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

start_launcher() {
  local label="$1" script="$2"
  shift 2
  log "Starting ${label}: ${script}"
  setsid env ROS_WS="${ROS_WS}" ROS_SETUP="${ROS_SETUP}" "$@" "${script}" &
  LAST_STARTED_PID=$!
  sleep 1
  if ! child_alive "${LAST_STARTED_PID}"; then
    wait "${LAST_STARTED_PID}" 2>/dev/null || true
    fail "${label} launcher exited during startup"
  fi
}

wait_for_message() {
  local topic="$1" timeout_sec="$2" reliability="$3" owner_pid="$4"
  local start elapsed
  start="$(date +%s)"
  while true; do
    if ! child_alive "${owner_pid}"; then
      fail "launcher pid=${owner_pid} exited while waiting for ${topic}"
      return 1
    fi
    if timeout 3 ros2 topic echo "${topic}" --once \
      --qos-reliability "${reliability}" >/dev/null 2>&1; then
      log "Ready: message stream ${topic}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      fail "no message on ${topic} after ${timeout_sec}s"
      return 1
    fi
    if (( elapsed % 10 == 0 )); then
      log "Waiting for ${topic}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

wait_for_transform() {
  local parent="$1" child="$2" timeout_sec="$3" owner_pid="$4"
  local start elapsed output
  start="$(date +%s)"
  while true; do
    if ! child_alive "${owner_pid}"; then
      fail "launcher pid=${owner_pid} exited while waiting for TF ${parent} -> ${child}"
      return 1
    fi
    output="$(timeout 3 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 || true)"
    if grep -q -- '- Translation:' <<<"${output}"; then
      log "Ready: TF ${parent} -> ${child}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      fail "missing TF ${parent} -> ${child} after ${timeout_sec}s"
      return 1
    fi
    if (( elapsed % 10 == 0 )); then
      log "Waiting for TF ${parent} -> ${child}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

stop_launcher() {
  local label="$1" pid="$2" start
  [[ -n "${pid}" ]] || return 0
  if ! child_alive "${pid}"; then
    wait "${pid}" 2>/dev/null || true
    return 0
  fi

  log "Stopping ${label} launcher pid=${pid}"
  kill -TERM "${pid}" 2>/dev/null || true
  start="$(date +%s)"
  while child_alive "${pid}"; do
    if (( $(date +%s) - start >= CHILD_STOP_TIMEOUT_SEC )); then
      log "WARNING: ${label} did not finish cleanup within ${CHILD_STOP_TIMEOUT_SEC}s"
      kill -KILL "${pid}" 2>/dev/null || true
      break
    fi
    sleep 0.5
  done
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  local exit_code=$?
  (( CLEANING_UP == 0 )) || return
  CLEANING_UP=1
  set +e
  trap - EXIT INT TERM
  log "Shutdown reason: ${SHUTDOWN_REASON}; launcher_exit=${exit_code}"

  # Keep /map and odometry alive while the 3D exporter finishes.
  stop_launcher "3D mapping" "${MAPPING_3D_PID}"
  stop_launcher "2D mapping" "${MAPPING_2D_PID}"
  stop_launcher "RF2O/PX4 fusion" "${FUSION_PID}"
  log "Combined mapping stack stopped; individual autosave handlers have completed"
  exit "${exit_code}"
}

on_signal() {
  SHUTDOWN_REASON="received signal $1"
  exit 130
}

validate_environment() {
  local script command
  [[ -f /opt/ros/jazzy/setup.bash ]] || fail "missing /opt/ros/jazzy/setup.bash"
  [[ -f "${ROS_SETUP}" ]] || fail "workspace setup is missing: ${ROS_SETUP}"
  for script in "${FUSION_SCRIPT}" "${MAPPING_2D_SCRIPT}" "${MAPPING_3D_SCRIPT}"; do
    [[ -x "${script}" ]] || fail "launcher is missing or not executable: ${script}"
  done
  for command in ros2 timeout setsid flock grep tee; do
    command -v "${command}" >/dev/null 2>&1 || fail "missing command: ${command}"
  done
  if [[ "${REQUIRE_PX4_BRIDGE_OUTPUT}" != "0" &&
    "${REQUIRE_PX4_BRIDGE_OUTPUT}" != "1" ]]; then
    fail "REQUIRE_PX4_BRIDGE_OUTPUT must be 0 or 1"
  fi
}

main() {
  mkdir -p "${LOG_DIR}" "${STATE_ROOT}"
  touch "${MASTER_LOG}"
  exec > >(trap '' INT TERM; exec tee -a "${MASTER_LOG}") 2>&1
  trap cleanup EXIT
  trap 'on_signal INT' INT
  trap 'on_signal TERM' TERM

  validate_environment
  set +u
  source /opt/ros/jazzy/setup.bash
  source "${ROS_SETUP}"
  set -u

  exec 9>"${LOCK_FILE}"
  if ! flock -n 9; then
    fail "another combined mapping launcher owns ${LOCK_FILE}"
    return 1
  fi

  log "Starting complete real-drone mapping stack"
  log "MAVROS and the boot AprilTag pipeline remain externally managed"

  start_launcher "RF2O/PX4 fusion" "${FUSION_SCRIPT}" \
    CANONICAL_SCAN_TOPIC="${FUSION_SCAN_TOPIC}" \
    LIDAR_ODOM_TOPIC="${FUSION_ODOM_TOPIC}" \
    PX4_ODOMETRY_OUT_TOPIC="${PX4_ODOMETRY_OUT_TOPIC}" \
    PX4_BRIDGE_DIAGNOSTICS_TOPIC="${FUSION_BRIDGE_DIAGNOSTICS_TOPIC}" \
    PX4_BRIDGE_WAIT_SEC="${FUSION_BRIDGE_OUTPUT_WAIT_SEC}" \
    PX4_ODOM_TOPIC="${PX4_ODOM_TOPIC}" \
    ODOM_PARENT_FRAME="${ODOM_FRAME}" \
    BASE_FRAME="${BASE_FRAME}"
  FUSION_PID="${LAST_STARTED_PID}"
  wait_for_message "${FUSION_SCAN_TOPIC}" "${FUSION_WAIT_SEC}" best_effort "${FUSION_PID}"
  wait_for_message "${FUSION_ODOM_TOPIC}" "${FUSION_WAIT_SEC}" best_effort "${FUSION_PID}"
  wait_for_message "${PX4_ODOM_TOPIC}" "${FUSION_WAIT_SEC}" best_effort "${FUSION_PID}"
  wait_for_transform "${ODOM_FRAME}" "${BASE_FRAME}" "${FUSION_WAIT_SEC}" "${FUSION_PID}"
  wait_for_message "${FUSION_BRIDGE_DIAGNOSTICS_TOPIC}" "${FUSION_WAIT_SEC}" reliable "${FUSION_PID}"
  if [[ "${REQUIRE_PX4_BRIDGE_OUTPUT}" == "1" ]]; then
    wait_for_message "${PX4_ODOMETRY_OUT_TOPIC}" "${FUSION_WAIT_SEC}" reliable "${FUSION_PID}"
  else
    log "Fusion stage ready; ${PX4_ODOMETRY_OUT_TOPIC} is not required for observer-only mapping"
  fi

  start_launcher "2D mapping" "${MAPPING_2D_SCRIPT}" \
    SOURCE_SCAN_TOPIC="${FUSION_SCAN_TOPIC}" \
    ODOM_TOPIC="${PX4_ODOM_TOPIC}" \
    MAP_TOPIC="${MAP_TOPIC}" \
    MAP_FRAME="${MAP_FRAME}" \
    ODOM_FRAME="${ODOM_FRAME}" \
    BASE_FRAME="${BASE_FRAME}"
  MAPPING_2D_PID="${LAST_STARTED_PID}"
  wait_for_message "${MAP_TOPIC}" "${MAPPING_2D_WAIT_SEC}" reliable "${MAPPING_2D_PID}"
  wait_for_transform "${MAP_FRAME}" "${ODOM_FRAME}" "${MAPPING_2D_WAIT_SEC}" "${MAPPING_2D_PID}"

  start_launcher "3D mapping" "${MAPPING_3D_SCRIPT}" \
    PX4_ODOM_TOPIC="${PX4_ODOM_TOPIC}" \
    TARGET_FRAME="${MAP_FRAME}" \
    MAP_FRAME="${MAP_FRAME}" \
    ODOM_FRAME="${ODOM_FRAME}" \
    BASE_FRAME="${BASE_FRAME}" \
    GLOBAL_CLOUD_TOPIC="${GLOBAL_CLOUD_TOPIC}" \
    ENABLE_MAP_REBASE="${MAPPING_3D_ENABLE_MAP_REBASE}" \
    ENABLE_RELATIVE_POSE_GATE="${MAPPING_3D_ENABLE_RELATIVE_POSE_GATE}" \
    REQUIRE_2D_MAP=1 \
    AUTO_SAVE_3D_MAP_ON_EXIT=1
  MAPPING_3D_PID="${LAST_STARTED_PID}"
  wait_for_message "${GLOBAL_CLOUD_TOPIC}" "${MAPPING_3D_WAIT_SEC}" reliable "${MAPPING_3D_PID}"

  log "ALL READY: fusion + 2D SLAM + vertical-lidar 3D mapping"
  log "Ctrl+C saves 3D PCD/GLB first, then 2D YAML/PGM/PNG"
  log "Runtime logs: ${LOG_DIR} and each child launcher's runtime_logs directory"

  while true; do
    child_alive "${FUSION_PID}" || { SHUTDOWN_REASON="fusion launcher exited"; return 1; }
    child_alive "${MAPPING_2D_PID}" || { SHUTDOWN_REASON="2D mapping launcher exited"; return 1; }
    child_alive "${MAPPING_3D_PID}" || { SHUTDOWN_REASON="3D mapping launcher exited"; return 1; }
    sleep 2
  done
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
