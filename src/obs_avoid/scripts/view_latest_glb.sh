#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DEFAULT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WS="${ROS_WS:-${ROS_WS_DEFAULT}}"
ROS_SETUP="${ROS_SETUP:-${ROS_WS}/install/setup.bash}"
EXPORT_DIR="${EXPORT_DIR:-${ROS_WS}/maps/vertical_3d}"
EXPORT_FIRST=0

usage() {
  printf 'Usage: %s [--export-first] [--dir <glb_dir>]\n' "$0"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --export-first)
      EXPORT_FIRST=1
      shift
      ;;
    --dir)
      [[ $# -ge 2 ]] || { usage >&2; exit 1; }
      EXPORT_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '[error] unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ "${EXPORT_FIRST}" == "1" ]]; then
  [[ -f "${ROS_SETUP}" ]] || {
    printf '[error] ROS setup not found: %s\n' "${ROS_SETUP}" >&2
    exit 1
  }
  set +u
  source "${ROS_SETUP}"
  set -u
  ros2 service call /vertical_lidar_mapper/save_pcd std_srvs/srv/Trigger "{}"
fi

[[ -d "${EXPORT_DIR}" ]] || {
  printf '[error] export directory not found: %s\n' "${EXPORT_DIR}" >&2
  exit 1
}

LATEST_GLB="$(find "${EXPORT_DIR}" -maxdepth 1 -type f -name '*.glb' -printf '%T@\t%p\n' | \
  sort -nr | head -n1 | cut -f2-)"
[[ -n "${LATEST_GLB}" ]] || {
  printf '[error] no GLB model found in %s\n' "${EXPORT_DIR}" >&2
  exit 1
}

printf '[info] latest structural model: %s\n' "${LATEST_GLB}"

if command -v blender >/dev/null 2>&1; then
  export VERTICAL_MAP_GLB="${LATEST_GLB}"
  exec blender --python-expr \
    "import os,bpy; bpy.ops.wm.read_factory_settings(use_empty=True); bpy.ops.import_scene.gltf(filepath=os.environ['VERTICAL_MAP_GLB'])"
fi

if command -v xdg-open >/dev/null 2>&1; then
  exec xdg-open "${LATEST_GLB}"
fi

printf '[warn] no GLB viewer found; install Blender or open the file on another computer.\n' >&2
exit 2
