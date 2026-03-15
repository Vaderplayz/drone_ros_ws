#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
EXPORT_DIR="${EXPORT_DIR:-${HOME}/vertical_mapper_exports}"
EXPORT_FIRST="${EXPORT_FIRST:-false}"

usage() {
  echo "Usage: $0 [--export-first] [--dir <pcd_dir>]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --export-first)
      EXPORT_FIRST=true
      shift
      ;;
    --dir)
      if [[ $# -lt 2 ]]; then
        usage
        exit 1
      fi
      EXPORT_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[error] unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ "${EXPORT_FIRST}" == "true" ]]; then
  if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[error] ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
  fi
  set +u
  source "${ROS_SETUP}"
  set -u
  echo "[run] requesting new PCD export..."
  ros2 service call /vertical_lidar_mapper/save_pcd std_srvs/srv/Trigger "{}" >/dev/null
fi

if [[ ! -d "${EXPORT_DIR}" ]]; then
  echo "[error] export dir not found: ${EXPORT_DIR}" >&2
  exit 1
fi

LATEST_PCD="$(find "${EXPORT_DIR}" -maxdepth 1 -type f -name '*.pcd' -printf '%T@ %p\n' | sort -nr | head -n1 | awk '{print $2}')"

if [[ -z "${LATEST_PCD}" ]]; then
  echo "[error] no .pcd files found in ${EXPORT_DIR}" >&2
  exit 1
fi

echo "[info] latest PCD: ${LATEST_PCD}"

if command -v CloudCompare >/dev/null 2>&1; then
  exec CloudCompare "${LATEST_PCD}"
fi

if command -v meshlab >/dev/null 2>&1; then
  exec meshlab "${LATEST_PCD}"
fi

if command -v pcl_viewer >/dev/null 2>&1; then
  exec pcl_viewer "${LATEST_PCD}"
fi

echo "[warn] no external viewer found (tried: pcl_viewer, CloudCompare, meshlab)." >&2
echo "[info] open this file manually: ${LATEST_PCD}"
