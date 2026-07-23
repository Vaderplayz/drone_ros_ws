#!/usr/bin/env bash
# Independent lidar2 3D mapping using an existing MAVROS/AprilTag odometry feed.
# shellcheck disable=SC1090,SC1091

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="/home/pi5drone/drone_ros_ws"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
MAPPING_3D_SCRIPT="${MAPPING_3D_SCRIPT:-${SCRIPT_DIR}/start_real_3d_mapping_lidar2.sh}"

USE_SIM_TIME="${USE_SIM_TIME:-false}"
PX4_ODOM_TOPIC="${PX4_ODOM_TOPIC:-/mavros/local_position/odom}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
LIDAR2_FRAME_ID="${LIDAR2_FRAME_ID:-lidar_vert_link}"
LIDAR2_X="${LIDAR2_X:-0.0}"
LIDAR2_Y="${LIDAR2_Y:-0.0}"
LIDAR2_Z="${LIDAR2_Z:-0.70}"
LIDAR2_ROLL="${LIDAR2_ROLL:-1.57079632679}"
LIDAR2_PITCH="${LIDAR2_PITCH:-0.0}"
LIDAR2_YAW="${LIDAR2_YAW:-1.57079632679}"
GLOBAL_CLOUD_TOPIC="${GLOBAL_CLOUD_TOPIC:-/mapping/global_cloud}"
ENABLE_FLOOR_STABILIZATION="${ENABLE_FLOOR_STABILIZATION:-true}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-90}"
STOP_TIMEOUT_SEC="${STOP_TIMEOUT_SEC:-50}"

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${ROS_WS}/runtime_logs/independent_3d_${RUN_STAMP}"
MASTER_LOG="${LOG_DIR}/master.log"
ODOM_TF_LOG="${LOG_DIR}/odom_to_tf.log"
STATE_ROOT="${XDG_RUNTIME_DIR:-/tmp}/independent_3d_mapping_$(id -u)"
LOCK_FILE="${STATE_ROOT}/launcher.lock"

ODOM_TF_PID=""
MAPPING_3D_PID=""
EXTERNAL_ODOM_TF=0
SHUTDOWN_REASON="normal exit"
CLEANING_UP=0

timestamp() {
  date '+%Y-%m-%d %H:%M:%S'
}

log() {
  printf '[%s] [independent-3d] %s\n' "$(timestamp)" "$*"
}

fail() {
  SHUTDOWN_REASON="$*"
  log "ERROR: $*"
  return 1
}

process_alive() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

wait_for_message() {
  local topic="$1" timeout_sec="$2" owner_pid="${3:-}"
  local start elapsed last_progress=-1
  start="$(date +%s)"
  while true; do
    if [[ -n "${owner_pid}" ]] && ! process_alive "${owner_pid}"; then
      fail "process pid=${owner_pid} exited while waiting for ${topic}"
      return 1
    fi
    if timeout 3 ros2 topic echo "${topic}" --once \
      --qos-reliability best_effort >/dev/null 2>&1; then
      log "Ready: message stream ${topic}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      fail "no message on ${topic} after ${timeout_sec}s"
      return 1
    fi
    if (( elapsed / 10 > last_progress )); then
      last_progress=$((elapsed / 10))
      log "Waiting for ${topic}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

tf_available() {
  local parent="$1" child="$2" output
  output="$(timeout 3 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 || true)"
  grep -q -- '- Translation:' <<<"${output}"
}

wait_for_transform() {
  local parent="$1" child="$2" timeout_sec="$3" owner_pid="${4:-}"
  local start elapsed last_progress=-1
  start="$(date +%s)"
  while true; do
    if [[ -n "${owner_pid}" ]] && ! process_alive "${owner_pid}"; then
      fail "process pid=${owner_pid} exited while waiting for TF ${parent} -> ${child}"
      [[ -f "${ODOM_TF_LOG}" ]] && tail -n 40 "${ODOM_TF_LOG}"
      return 1
    fi
    if tf_available "${parent}" "${child}"; then
      log "Ready: TF ${parent} -> ${child}"
      return 0
    fi
    elapsed=$(( $(date +%s) - start ))
    if (( elapsed >= timeout_sec )); then
      fail "missing TF ${parent} -> ${child} after ${timeout_sec}s"
      return 1
    fi
    if (( elapsed / 10 > last_progress )); then
      last_progress=$((elapsed / 10))
      log "Waiting for TF ${parent} -> ${child}; elapsed=${elapsed}s deadline=${timeout_sec}s"
    fi
    sleep 1
  done
}

start_odom_tf_if_missing() {
  if tf_available "${ODOM_FRAME}" "${BASE_FRAME}"; then
    EXTERNAL_ODOM_TF=1
    log "Reusing existing TF ${ODOM_FRAME} -> ${BASE_FRAME}"
    return 0
  fi

  log "Starting full-pose MAVROS odometry TF bridge"
  setsid ros2 run vertical_lidar_mapper odom_to_tf_bridge_node --ros-args \
    -r __node:=independent_3d_odom_to_tf \
    -p use_sim_time:="${USE_SIM_TIME}" \
    -p odom_topic:="${PX4_ODOM_TOPIC}" \
    -p odom_frame:="${ODOM_FRAME}" \
    -p base_frame:="${BASE_FRAME}" \
    -p use_msg_frame_ids:=false >"${ODOM_TF_LOG}" 2>&1 &
  ODOM_TF_PID=$!
  wait_for_transform "${ODOM_FRAME}" "${BASE_FRAME}" "${WAIT_TIMEOUT_SEC}" "${ODOM_TF_PID}"
}

validate_lidar2_extrinsic() {
  local output translation rpy
  output="$(timeout 3 ros2 run tf2_ros tf2_echo "${BASE_FRAME}" "${LIDAR2_FRAME_ID}" 2>&1 || true)"
  translation="$(awk -F'[][]' '/^- Translation:/{print $2; exit}' <<<"${output}")"
  rpy="$(awk -F'[][]' '/^- Rotation: in RPY \(radian\)/{print $2; exit}' <<<"${output}")"
  if [[ -z "${translation}" || -z "${rpy}" ]]; then
    fail "could not inspect TF ${BASE_FRAME} -> ${LIDAR2_FRAME_ID}"
    return 1
  fi

  if ! awk \
    -v translation="${translation}" -v rpy="${rpy}" \
    -v expected_translation="${LIDAR2_X},${LIDAR2_Y},${LIDAR2_Z}" \
    -v expected_rpy="${LIDAR2_ROLL},${LIDAR2_PITCH},${LIDAR2_YAW}" '
      function abs(value) {return value < 0 ? -value : value}
      function angle_error(left, right, delta) {
        delta = left - right
        while (delta > 3.141592653589793) delta -= 6.283185307179586
        while (delta < -3.141592653589793) delta += 6.283185307179586
        return abs(delta)
      }
      BEGIN {
        split(translation, actual_t, ",")
        split(expected_translation, expected_t, ",")
        split(rpy, actual_r, ",")
        split(expected_rpy, expected_r, ",")
        valid = 1
        for (i = 1; i <= 3; i++) {
          if (abs(actual_t[i] - expected_t[i]) > 0.02) valid = 0
          if (angle_error(actual_r[i], expected_r[i]) > 0.03) valid = 0
        }
        exit(valid ? 0 : 1)
      }'
  then
    fail "TF ${BASE_FRAME} -> ${LIDAR2_FRAME_ID} does not match requested translation/RPY: [${translation}] [${rpy}]"
    return 1
  fi
  log "Verified lidar2 TF: translation=[${translation}] RPY=[${rpy}]"
}

start_3d_mapping() {
  log "Starting lidar2 mapping in ${ODOM_FRAME}; no 2D map is required"
  setsid env \
    ROS_WS="${ROS_WS}" \
    ROS_SETUP="${ROS_SETUP}" \
    USE_SIM_TIME="${USE_SIM_TIME}" \
    PX4_ODOM_TOPIC="${PX4_ODOM_TOPIC}" \
    ODOM_FRAME="${ODOM_FRAME}" \
    BASE_FRAME="${BASE_FRAME}" \
    LIDAR2_FRAME_ID="${LIDAR2_FRAME_ID}" \
    LIDAR2_X="${LIDAR2_X}" \
    LIDAR2_Y="${LIDAR2_Y}" \
    LIDAR2_Z="${LIDAR2_Z}" \
    LIDAR2_ROLL="${LIDAR2_ROLL}" \
    LIDAR2_PITCH="${LIDAR2_PITCH}" \
    LIDAR2_YAW="${LIDAR2_YAW}" \
    TARGET_FRAME="${ODOM_FRAME}" \
    REQUIRE_2D_MAP=0 \
    EXPORT_MAP2D_ON_SAVE=false \
    EXPORT_SLAM_MAP2D_ON_SAVE=false \
    EXPORT_STRUCTURAL_MESH_ON_SAVE=false \
    ENABLE_MAP_REBASE=false \
    ENABLE_RELATIVE_POSE_GATE=false \
    ENABLE_FLOOR_STABILIZATION="${ENABLE_FLOOR_STABILIZATION}" \
    "${MAPPING_3D_SCRIPT}" &
  MAPPING_3D_PID=$!
  wait_for_transform "${BASE_FRAME}" "${LIDAR2_FRAME_ID}" "${WAIT_TIMEOUT_SEC}" "${MAPPING_3D_PID}"
  validate_lidar2_extrinsic
  wait_for_message "${GLOBAL_CLOUD_TOPIC}" "${WAIT_TIMEOUT_SEC}" "${MAPPING_3D_PID}"
}

stop_process() {
  local label="$1" pid="$2" start
  [[ -n "${pid}" ]] || return 0
  if ! process_alive "${pid}"; then
    wait "${pid}" 2>/dev/null || true
    return 0
  fi

  log "Stopping ${label} pid=${pid}"
  kill -TERM "${pid}" 2>/dev/null || true
  start="$(date +%s)"
  while process_alive "${pid}"; do
    if (( $(date +%s) - start >= STOP_TIMEOUT_SEC )); then
      log "WARNING: ${label} did not stop within ${STOP_TIMEOUT_SEC}s"
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
  stop_process "3D mapping launcher" "${MAPPING_3D_PID}"
  if [[ "${EXTERNAL_ODOM_TF}" != "1" ]]; then
    stop_process "odometry TF bridge" "${ODOM_TF_PID}"
  fi
  log "Independent 3D mapping stopped; MAVROS and AprilTag were left running"
  exit "${exit_code}"
}

on_signal() {
  SHUTDOWN_REASON="received signal $1"
  exit 130
}

main() {
  mkdir -p "${LOG_DIR}" "${STATE_ROOT}"
  touch "${MASTER_LOG}"
  exec > >(trap '' INT TERM; exec tee -a "${MASTER_LOG}") 2>&1
  trap cleanup EXIT
  trap 'on_signal INT' INT
  trap 'on_signal TERM' TERM

  [[ -f /opt/ros/jazzy/setup.bash ]] || { fail "missing ROS Jazzy setup"; return 1; }
  [[ -f "${ROS_SETUP}" ]] || { fail "missing workspace setup: ${ROS_SETUP}"; return 1; }
  [[ -x "${MAPPING_3D_SCRIPT}" ]] || { fail "missing 3D launcher: ${MAPPING_3D_SCRIPT}"; return 1; }

  set +u
  source /opt/ros/jazzy/setup.bash
  source "${ROS_SETUP}"
  set -u

  for command in ros2 timeout setsid flock grep tee awk; do
    command -v "${command}" >/dev/null 2>&1 || { fail "missing command: ${command}"; return 1; }
  done
  ros2 pkg prefix vertical_lidar_mapper >/dev/null 2>&1 || {
    fail "vertical_lidar_mapper is unavailable in this overlay"
    return 1
  }

  exec 9>"${LOCK_FILE}"
  if ! flock -n 9; then
    fail "another independent 3D launcher owns ${LOCK_FILE}"
    return 1
  fi

  log "Independent 3D startup; expecting MAVROS and AprilTag to already be running"
  wait_for_message "${PX4_ODOM_TOPIC}" "${WAIT_TIMEOUT_SEC}"
  start_odom_tf_if_missing
  start_3d_mapping

  log "ALL READY: independent lidar2 3D map in frame ${ODOM_FRAME}"
  log "Live cloud: ${GLOBAL_CLOUD_TOPIC}; Ctrl+C autosaves PCD and trajectory"
  log "Runtime logs: ${LOG_DIR}"

  while true; do
    process_alive "${MAPPING_3D_PID}" || {
      SHUTDOWN_REASON="3D mapping launcher exited"
      return 1
    }
    if [[ "${EXTERNAL_ODOM_TF}" != "1" ]] && ! process_alive "${ODOM_TF_PID}"; then
      SHUTDOWN_REASON="odometry TF bridge exited"
      return 1
    fi
    sleep 2
  done
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
