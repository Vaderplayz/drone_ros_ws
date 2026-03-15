#!/usr/bin/env bash

set -euo pipefail

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[error] missing command: $1" >&2
    exit 1
  fi
}

wait_for_service() {
  local service_name="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if ros2 service list 2>/dev/null | grep -qx "${service_name}"; then
      return 0
    fi
    if (( "$(date +%s)" - start_ts >= timeout_sec )); then
      echo "[error] timed out waiting for service: ${service_name}" >&2
      return 1
    fi
    sleep 1
  done
}

extract_path_from_save_response() {
  local key="$1"
  local response="$2"
  case "${key}" in
    pcd)
      printf '%s\n' "${response}" | sed -n 's/.*Saved PCD: \([^|]*\.pcd\).*/\1/p' | tail -n1 | xargs
      ;;
    map2d)
      printf '%s\n' "${response}" | sed -n 's/.*2D map: \([^|]*\.pgm\).*/\1/p' | tail -n1 | xargs
      ;;
    map2d_yaml)
      printf '%s\n' "${response}" | sed -n 's/.*yaml: \([^)]*\.yaml\).*/\1/p' | tail -n1 | xargs
      ;;
    trajectory)
      printf '%s\n' "${response}" | sed -n 's/.*trajectory: \([^ ]*\.csv\).*/\1/p' | tail -n1 | xargs
      ;;
    *)
      return 1
      ;;
  esac
}

find_latest_file() {
  local dir="$1"
  local pattern="$2"
  find "${dir}" -maxdepth 1 -type f -name "${pattern}" -printf '%T@ %p\n' 2>/dev/null \
    | sort -n \
    | tail -n1 \
    | cut -d' ' -f2-
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-${ROS_WS}/src/uav_stack_bringup/config/slam_mapping_fast.yaml}"

BAG_PATH="${1:-}"
OUTPUT_DIR="${2:-${HOME}/vertical_mapper_exports/refined}"
REFINE_PROFILE="${3:-v11_clean}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-45}"

if [[ -z "${BAG_PATH}" ]]; then
  echo "usage: $0 <bag_path> [output_dir] [refine_profile]" >&2
  echo "profiles: v11_clean | v11_balanced | legacy" >&2
  exit 1
fi

require_cmd ros2
require_cmd timeout

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -d "${BAG_PATH}" ]]; then
  echo "[error] bag path not found: ${BAG_PATH}" >&2
  exit 1
fi
if [[ ! -f "${SLAM_PARAMS_FILE}" ]]; then
  echo "[error] SLAM params file not found: ${SLAM_PARAMS_FILE}" >&2
  exit 1
fi

case "${REFINE_PROFILE}" in
  v11_clean)
    MAPPER_PARAMS_FILE="${ROS_WS}/src/vertical_lidar_mapper/config/v11_clean.yaml"
    ;;
  v11_balanced)
    MAPPER_PARAMS_FILE="${ROS_WS}/src/vertical_lidar_mapper/config/v11_balanced.yaml"
    ;;
  legacy)
    MAPPER_PARAMS_FILE="${ROS_WS}/src/vertical_lidar_mapper/config/params.yaml"
    ;;
  *)
    echo "[error] invalid refine profile: ${REFINE_PROFILE}" >&2
    echo "        allowed: v11_clean | v11_balanced | legacy" >&2
    exit 1
    ;;
esac

if [[ ! -f "${MAPPER_PARAMS_FILE}" ]]; then
  echo "[error] mapper params file not found: ${MAPPER_PARAMS_FILE}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u
export COLCON_TRACE="${COLCON_TRACE:-0}"

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${OUTPUT_DIR}/${RUN_STAMP}_${REFINE_PROFILE}"
mkdir -p "${RUN_DIR}"
mkdir -p "${HOME}/vertical_mapper_exports"

SLAM_LOG="${RUN_DIR}/offline_slam_toolbox.log"
MAPPER_LOG="${RUN_DIR}/offline_vertical_lidar_mapper.log"
BAG_LOG="${RUN_DIR}/offline_bag_play.log"
REPORT_FILE="${RUN_DIR}/refine_report_${RUN_STAMP}.txt"

SLAM_PID=""
MAPPER_PID=""

cleanup() {
  set +e
  if [[ -n "${MAPPER_PID}" ]] && kill -0 "${MAPPER_PID}" >/dev/null 2>&1; then
    kill "${MAPPER_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${SLAM_PID}" ]] && kill -0 "${SLAM_PID}" >/dev/null 2>&1; then
    kill "${SLAM_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

START_EPOCH="$(date +%s)"

echo "[run] slam_toolbox -> ${SLAM_LOG}"
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="${SLAM_PARAMS_FILE}" \
  use_sim_time:=true >"${SLAM_LOG}" 2>&1 &
SLAM_PID="$!"

sleep 2

echo "[run] vertical_lidar_mapper (${REFINE_PROFILE}) -> ${MAPPER_LOG}"
ros2 launch vertical_lidar_mapper vertical_lidar_mapper.launch.py \
  params_file:="${MAPPER_PARAMS_FILE}" \
  scan_topic:=/scan_vertical \
  target_frame:=map \
  enable_odom_tf_bridge:=false \
  use_sim_time:=true >"${MAPPER_LOG}" 2>&1 &
MAPPER_PID="$!"

wait_for_service "/vertical_lidar_mapper/save_pcd" "${WAIT_TIMEOUT_SEC}"
wait_for_service "/vertical_lidar_mapper/rebuild_global" "${WAIT_TIMEOUT_SEC}"

echo "[run] ros2 bag play ${BAG_PATH}"
ros2 bag play "${BAG_PATH}" --clock >"${BAG_LOG}" 2>&1

sleep 2

echo "[run] forcing global rebuild from keyframes"
timeout 10 ros2 service call /vertical_lidar_mapper/rebuild_global std_srvs/srv/Trigger "{}" \
  >"${RUN_DIR}/rebuild_service.txt" 2>&1 || true

echo "[run] saving map assets"
SAVE_RESPONSE="$(timeout 15 ros2 service call /vertical_lidar_mapper/save_pcd std_srvs/srv/Trigger "{}" 2>&1 || true)"
printf '%s\n' "${SAVE_RESPONSE}" >"${RUN_DIR}/save_service.txt"

EXPORT_DIR="${HOME}/vertical_mapper_exports"
PCD_PATH="$(extract_path_from_save_response pcd "${SAVE_RESPONSE}")"
MAP2D_PATH="$(extract_path_from_save_response map2d "${SAVE_RESPONSE}")"
MAP2D_YAML_PATH="$(extract_path_from_save_response map2d_yaml "${SAVE_RESPONSE}")"
TRAJECTORY_PATH="$(extract_path_from_save_response trajectory "${SAVE_RESPONSE}")"

if [[ -z "${PCD_PATH}" ]]; then
  PCD_PATH="$(find_latest_file "${EXPORT_DIR}" "vertical_global_map_*.pcd")"
fi
if [[ -z "${MAP2D_PATH}" ]]; then
  MAP2D_PATH="$(find_latest_file "${EXPORT_DIR}" "vertical_map2d_*.pgm")"
fi
if [[ -z "${MAP2D_YAML_PATH}" ]]; then
  MAP2D_YAML_PATH="$(find_latest_file "${EXPORT_DIR}" "vertical_map2d_*.yaml")"
fi
if [[ -z "${TRAJECTORY_PATH}" ]]; then
  TRAJECTORY_PATH="$(find_latest_file "${EXPORT_DIR}" "vertical_trajectory_*.csv")"
fi

REFINED_PCD="${RUN_DIR}/refined_global_map_${RUN_STAMP}.pcd"
REFINED_MAP2D_PGM="${RUN_DIR}/refined_map2d_${RUN_STAMP}.pgm"
REFINED_MAP2D_YAML="${RUN_DIR}/refined_map2d_${RUN_STAMP}.yaml"
REFINED_TRAJECTORY="${RUN_DIR}/refined_trajectory_${RUN_STAMP}.csv"

if [[ -n "${PCD_PATH}" && -f "${PCD_PATH}" ]]; then
  cp -f "${PCD_PATH}" "${REFINED_PCD}"
fi
if [[ -n "${MAP2D_PATH}" && -f "${MAP2D_PATH}" ]]; then
  cp -f "${MAP2D_PATH}" "${REFINED_MAP2D_PGM}"
fi
if [[ -n "${MAP2D_YAML_PATH}" && -f "${MAP2D_YAML_PATH}" ]]; then
  cp -f "${MAP2D_YAML_PATH}" "${REFINED_MAP2D_YAML}"
fi
if [[ -n "${TRAJECTORY_PATH}" && -f "${TRAJECTORY_PATH}" ]]; then
  cp -f "${TRAJECTORY_PATH}" "${REFINED_TRAJECTORY}"
fi

STATUS_SNAPSHOT="$(timeout 8 ros2 topic echo --once /mapping/status 2>/dev/null || true)"
printf '%s\n' "${STATUS_SNAPSHOT}" >"${RUN_DIR}/mapping_status_snapshot.txt"

END_EPOCH="$(date +%s)"
ELAPSED_SEC="$(( END_EPOCH - START_EPOCH ))"

{
  echo "offline_refine_report"
  echo "run_stamp=${RUN_STAMP}"
  echo "profile=${REFINE_PROFILE}"
  echo "bag_path=${BAG_PATH}"
  echo "duration_sec=${ELAPSED_SEC}"
  echo "slam_params=${SLAM_PARAMS_FILE}"
  echo "mapper_params=${MAPPER_PARAMS_FILE}"
  echo "run_dir=${RUN_DIR}"
  echo "refined_pcd=${REFINED_PCD}"
  echo "refined_map2d_pgm=${REFINED_MAP2D_PGM}"
  echo "refined_map2d_yaml=${REFINED_MAP2D_YAML}"
  echo "refined_trajectory=${REFINED_TRAJECTORY}"
  echo "--- save_service_response ---"
  printf '%s\n' "${SAVE_RESPONSE}"
  echo "--- mapping_status_snapshot ---"
  printf '%s\n' "${STATUS_SNAPSHOT}"
} >"${REPORT_FILE}"

echo "[ok] offline refine completed"
echo "[out] ${REFINED_PCD}"
echo "[out] ${REFINED_MAP2D_PGM}"
echo "[out] ${REFINED_MAP2D_YAML}"
echo "[out] ${REFINED_TRAJECTORY}"
echo "[out] ${REPORT_FILE}"
