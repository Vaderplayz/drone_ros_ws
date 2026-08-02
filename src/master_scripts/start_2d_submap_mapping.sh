#!/usr/bin/env bash
# Observer-only launcher for the experimental submap mapper.
# Requires /scan_slam and odom -> base_footprint to exist already.
# shellcheck disable=SC1090,SC1091,SC2317
set -euo pipefail

ROS_WS_DEFAULT="/home/pi5drone/drone_ros_ws"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
PARAMS_FILE="${PARAMS_FILE:-${ROS_WS}/src/submap_slam_2d/config/real_rf2o_submap.yaml}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan_slam}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
ODOM_TOPIC="${ODOM_TOPIC:-/mavros/local_position/odom}"
WAIT_SEC="${WAIT_SEC:-60}"
RECORD_SUBMAP_BAG="${RECORD_SUBMAP_BAG:-0}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
STOP_TIMEOUT_SEC="${STOP_TIMEOUT_SEC:-5}"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${ROS_WS}/runtime_logs/submap_slam_2d_${STAMP}"
STATE_DIR="${XDG_RUNTIME_DIR:-/tmp}/submap_slam_2d_$(id -u)"
LOCK_FILE="${STATE_DIR}/launcher.lock"
OWNER_FILE="${STATE_DIR}/launcher.pid"
PROCESS_FILE="${STATE_DIR}/owned_processes.tsv"
BOOT_ID="$(< /proc/sys/kernel/random/boot_id)"
PIDS=()
NAMES=()

timestamp() { date '+%Y-%m-%dT%H:%M:%S%z'; }
log() { printf '[%s] %s\n' "$(timestamp)" "$*" | tee -a "${LOG_DIR}/master.log"; }
publisher_count() {
  ros2 topic info "$1" -v 2>/dev/null | awk '/Publisher count:/{print $3; found=1; exit} END{if(!found) print 0}'
}
process_alive() { kill -0 -- "-$1" 2>/dev/null || kill -0 "$1" 2>/dev/null; }

recover_previous_run() {
  local pid name boot deadline
  local stale=()
  [[ -s "${PROCESS_FILE}" ]] || return 0
  while IFS=$'\t' read -r pid name boot; do
    [[ "${pid:-}" =~ ^[0-9]+$ && "${boot:-}" == "${BOOT_ID}" ]] || continue
    process_alive "${pid}" && stale+=("${pid}")
  done <"${PROCESS_FILE}"
  (( ${#stale[@]} > 0 )) || return 0
  log "Recovering ${#stale[@]} process group(s) owned by an interrupted previous run"
  for pid in "${stale[@]}"; do kill -TERM -- "-${pid}" 2>/dev/null || true; done
  deadline=$(( $(date +%s) + STOP_TIMEOUT_SEC ))
  while (( $(date +%s) < deadline )); do
    local any=0
    for pid in "${stale[@]}"; do process_alive "${pid}" && any=1; done
    (( any == 0 )) && return 0
    sleep 0.2
  done
  for pid in "${stale[@]}"; do
    if process_alive "${pid}"; then
      kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
  done
}

start_owned() {
  local name="$1" logfile="$2"
  shift 2
  setsid "$@" >"${logfile}" 2>&1 &
  local pid="$!"
  PIDS+=("${pid}")
  NAMES+=("${name}")
  printf '%s\t%s\t%s\n' "${pid}" "${name}" "${BOOT_ID}" >>"${PROCESS_FILE}"
  log "STARTED ${name} pgid=${pid}"
}

cleanup() {
  local code=$?
  set +e
  trap - EXIT INT TERM
  for ((i=${#PIDS[@]} - 1; i>=0; --i)); do
    process_alive "${PIDS[$i]}" && kill -TERM -- "-${PIDS[$i]}" 2>/dev/null
  done
  sleep "${STOP_TIMEOUT_SEC}"
  for ((i=${#PIDS[@]} - 1; i>=0; --i)); do
    process_alive "${PIDS[$i]}" && kill -KILL -- "-${PIDS[$i]}" 2>/dev/null
    wait "${PIDS[$i]}" 2>/dev/null || true
  done
  : >"${PROCESS_FILE}"
  : >"${OWNER_FILE}"
  log "Stopped only launcher-owned submap mapping processes; exit=${code}"
  exit "${code}"
}
trap cleanup EXIT INT TERM

mkdir -p "${LOG_DIR}" "${STATE_DIR}"
source /opt/ros/jazzy/setup.bash
if [[ ! -r "${ROS_SETUP}" ]]; then
  log "ERROR workspace setup missing: ${ROS_SETUP}"
  exit 1
fi
source "${ROS_SETUP}"

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  log "ERROR another submap mapping launcher is active"
  exit 1
fi
printf '%s\n' "$$" >"${OWNER_FILE}"
recover_previous_run
: >"${PROCESS_FILE}"

for command in ros2 timeout setsid flock awk tee; do
  command -v "${command}" >/dev/null || { log "ERROR missing command: ${command}"; exit 1; }
done
ros2 pkg prefix submap_slam_2d >/dev/null || { log "ERROR submap_slam_2d is not built"; exit 1; }
[[ -r "${PARAMS_FILE}" ]] || { log "ERROR config missing: ${PARAMS_FILE}"; exit 1; }

deadline=$(( $(date +%s) + WAIT_SEC ))
while [[ "$(publisher_count "${SCAN_TOPIC}")" != "1" ]]; do
  (( $(date +%s) < deadline )) || {
    log "ERROR ${SCAN_TOPIC} must have exactly one publisher"
    ros2 topic info "${SCAN_TOPIC}" -v 2>&1 | tee -a "${LOG_DIR}/master.log" || true
    exit 1
  }
  sleep 1
done
timeout "${WAIT_SEC}" ros2 topic echo "${SCAN_TOPIC}" --once \
  --qos-reliability best_effort >/dev/null || { log "ERROR no message on ${SCAN_TOPIC}"; exit 1; }
timeout "${WAIT_SEC}" ros2 run tf2_ros tf2_echo "${ODOM_FRAME}" "${BASE_FRAME}" --once \
  >"${LOG_DIR}/tf_prerequisite.log" 2>&1 || {
  log "ERROR TF ${ODOM_FRAME} -> ${BASE_FRAME} is unavailable"
  exit 1
}
log "Prerequisites ready: one ${SCAN_TOPIC} publisher and ${ODOM_FRAME} -> ${BASE_FRAME}"

start_owned submap_slam "${LOG_DIR}/submap_slam.log" \
  ros2 launch submap_slam_2d submap_slam_2d.launch.py \
    params_file:="${PARAMS_FILE}" use_sim_time:="${USE_SIM_TIME}" \
    scan_topic:="${SCAN_TOPIC}" full_odom_topic:="${ODOM_TOPIC}" \
    odom_frame:="${ODOM_FRAME}" base_frame:="${BASE_FRAME}"

if [[ "${RECORD_SUBMAP_BAG}" == "1" ]]; then
  start_owned rosbag "${LOG_DIR}/rosbag.log" ros2 bag record \
    -o "${LOG_DIR}/submap_mapping_bag" \
    /scan_slam /lidar/odom /mavros/local_position/odom /tf /tf_static /map \
    /submap_slam/map /submap_slam/trajectory /submap_slam/corrected_pose \
    /submap_slam/diagnostics
fi

deadline=$(( $(date +%s) + WAIT_SEC ))
while ! timeout 3 ros2 topic echo /submap_slam/diagnostics --once >/dev/null 2>&1; do
  process_alive "${PIDS[0]}" || { log "ERROR submap mapper exited"; exit 1; }
  (( $(date +%s) < deadline )) || { log "ERROR no submap diagnostics"; exit 1; }
done
log "READY logs=${LOG_DIR}; Ctrl-C stops only this mapper and optional bag"
while process_alive "${PIDS[0]}"; do sleep 1; done
log "ERROR submap mapper exited unexpectedly; inspect ${LOG_DIR}/submap_slam.log"
exit 1
