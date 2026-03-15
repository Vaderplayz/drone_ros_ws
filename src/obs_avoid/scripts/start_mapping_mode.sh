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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"

MAPPING_PROFILE="${MAPPING_PROFILE:-v11_full_detail}"
SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-${ROS_WS}/src/uav_stack_bringup/config/slam_mapping_fast.yaml}"
MAPPER_PARAMS_FILE_DEFAULT_LEGACY="${ROS_WS}/src/vertical_lidar_mapper/config/params.yaml"
MAPPER_PARAMS_FILE_DEFAULT_V11_CLEAN="${ROS_WS}/src/vertical_lidar_mapper/config/v11_clean.yaml"
MAPPER_PARAMS_FILE_DEFAULT_V11_DETAIL="${ROS_WS}/src/vertical_lidar_mapper/config/v11_detail.yaml"
MAPPER_PARAMS_FILE_DEFAULT_V11_FULL_DETAIL="${ROS_WS}/src/vertical_lidar_mapper/config/v11_full_detail.yaml"
MAPPER_PARAMS_FILE="${MAPPER_PARAMS_FILE:-}"

SCAN_TOPIC="${SCAN_TOPIC:-/scan_vertical}"
TARGET_FRAME="${TARGET_FRAME:-map}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"
KILL_BEFORE_LAUNCH="${KILL_BEFORE_LAUNCH:-1}"

WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
WAIT_TOPICS=(
  "/scan_horizontal"
  "/scan_vertical"
  "/mavros/local_position/odom"
)
WAIT_TF_SOURCE_NODES=(
  "/px4_odom_flatten_node"
  "/odom_to_tf_bridge"
)

SLAM_LOG="${SLAM_LOG:-/tmp/slam_mapping_fast.log}"
MAPPER_LOG="${MAPPER_LOG:-/tmp/vertical_lidar_mapper.log}"

SLAM_PID=""
MAPPER_PID=""

cleanup() {
  set +e
  if [[ -n "${MAPPER_PID}" ]] && kill -0 "${MAPPER_PID}" >/dev/null 2>&1; then
    echo "[stop] vertical_lidar_mapper (pid=${MAPPER_PID})"
    kill "${MAPPER_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${SLAM_PID}" ]] && kill -0 "${SLAM_PID}" >/dev/null 2>&1; then
    echo "[stop] slam_toolbox (pid=${SLAM_PID})"
    kill "${SLAM_PID}" >/dev/null 2>&1 || true
  fi
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

wait_for_required_topics() {
  local topic
  for topic in "${WAIT_TOPICS[@]}"; do
    echo "[wait] topic ${topic}"
    wait_for_topic "${topic}" "${WAIT_TIMEOUT_SEC}"
  done
}

wait_for_tf_source_node() {
  local start_ts
  start_ts="$(date +%s)"

  while true; do
    local node_list
    node_list="$(ros2 node list 2>/dev/null || true)"
    local node
    for node in "${WAIT_TF_SOURCE_NODES[@]}"; do
      if printf '%s\n' "${node_list}" | grep -qx "${node}"; then
        echo "[wait] found TF source node: ${node}"
        return 0
      fi
    done

    if (( "$(date +%s)" - start_ts >= WAIT_TIMEOUT_SEC )); then
      echo "[error] timed out waiting for TF source node (${WAIT_TF_SOURCE_NODES[*]})" >&2
      return 1
    fi
    sleep 1
  done
}

main() {
  require_cmd ros2

  case "${MAPPING_PROFILE}" in
    v11_clean)
      if [[ -z "${MAPPER_PARAMS_FILE}" ]]; then
        MAPPER_PARAMS_FILE="${MAPPER_PARAMS_FILE_DEFAULT_V11_CLEAN}"
      fi
      ;;
    v11_detail)
      if [[ -z "${MAPPER_PARAMS_FILE}" ]]; then
        MAPPER_PARAMS_FILE="${MAPPER_PARAMS_FILE_DEFAULT_V11_DETAIL}"
      fi
      ;;
    v11_full_detail)
      if [[ -z "${MAPPER_PARAMS_FILE}" ]]; then
        MAPPER_PARAMS_FILE="${MAPPER_PARAMS_FILE_DEFAULT_V11_FULL_DETAIL}"
      fi
      ;;
    legacy)
      if [[ -z "${MAPPER_PARAMS_FILE}" ]]; then
        MAPPER_PARAMS_FILE="${MAPPER_PARAMS_FILE_DEFAULT_LEGACY}"
      fi
      ;;
    *)
      echo "[error] invalid MAPPING_PROFILE='${MAPPING_PROFILE}'" >&2
      echo "        allowed: v11_full_detail | v11_clean | v11_detail | legacy" >&2
      exit 1
      ;;
  esac

  if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
  fi
  if [[ ! -f "${SLAM_PARAMS_FILE}" ]]; then
    echo "[error] SLAM params file not found: ${SLAM_PARAMS_FILE}" >&2
    exit 1
  fi
  if [[ ! -f "${MAPPER_PARAMS_FILE}" ]]; then
    echo "[error] mapper params file not found: ${MAPPER_PARAMS_FILE}" >&2
    exit 1
  fi

  # setup.bash may reference COLCON_TRACE while unset; temporarily disable nounset.
  set +u
  source "${ROS_SETUP}"
  set -u
  export COLCON_TRACE="${COLCON_TRACE:-0}"

  if is_true "${USE_SIM_TIME}"; then
    WAIT_TOPICS=( "/clock" "${WAIT_TOPICS[@]}" )
  fi

  if [[ "${KILL_BEFORE_LAUNCH}" == "1" ]]; then
    echo "[prep] stopping stale mapper/slam processes (if any)"
    pkill -f "/vertical_lidar_mapper/vertical_lidar_mapper_node" >/dev/null 2>&1 || true
    pkill -f "async_slam_toolbox_node" >/dev/null 2>&1 || true
    sleep 1
  fi

  trap cleanup EXIT INT TERM

  wait_for_required_topics
  wait_for_tf_source_node

  echo "[run] slam_toolbox -> ${SLAM_LOG}"
  ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="${SLAM_PARAMS_FILE}" \
    use_sim_time:="${USE_SIM_TIME}" >"${SLAM_LOG}" 2>&1 &
  SLAM_PID="$!"

  sleep 3

  echo "[run] vertical_lidar_mapper -> ${MAPPER_LOG}"
  ros2 launch vertical_lidar_mapper vertical_lidar_mapper.launch.py \
    params_file:="${MAPPER_PARAMS_FILE}" \
    scan_topic:="${SCAN_TOPIC}" \
    target_frame:="${TARGET_FRAME}" \
    enable_odom_tf_bridge:=false \
    use_sim_time:="${USE_SIM_TIME}" >"${MAPPER_LOG}" 2>&1 &
  MAPPER_PID="$!"

  echo "[ok] mapping-only mode started"
  echo "[info] mapping profile=${MAPPING_PROFILE}"
  echo "[info] mapper params=${MAPPER_PARAMS_FILE}"
  echo "[info] slam params=${SLAM_PARAMS_FILE}"
  echo "[info] no flight-control node is launched by this script"
  echo "[info] slam pid=${SLAM_PID}, mapper pid=${MAPPER_PID}"
  echo "[info] tail -f ${SLAM_LOG}"
  echo "[info] tail -f ${MAPPER_LOG}"

  wait -n "${SLAM_PID}" "${MAPPER_PID}"
  exit_code=$?
  echo "[warn] one process exited (code=${exit_code}), stopping the other."
  exit "${exit_code}"
}

main "$@"
