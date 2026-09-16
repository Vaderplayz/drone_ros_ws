#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
PACKAGE_ROOT="$(cd "$(dirname "${SCRIPT_PATH}")/.." && pwd)"
DEFAULT_WORKSPACE="$(cd "${PACKAGE_ROOT}/../.." && pwd)"
WORKSPACE="${DRONE_ROS_WS:-${DEFAULT_WORKSPACE}}"
ROS_DISTRO_NAME="${MINI_GC_ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
WORKSPACE_SETUP="${WORKSPACE}/install/setup.bash"
PYTHON_BIN="${MINI_GC_PYTHON:-${WORKSPACE}/.venv-ground-control/bin/python}"
STATE_ROOT="${XDG_STATE_HOME:-${HOME}/.local/state}/mini-ground-control"
RUNTIME_ROOT="${XDG_RUNTIME_DIR:-/tmp}"
LOG_FILE="${STATE_ROOT}/app.log"

mkdir -p "${STATE_ROOT}"
exec >>"${LOG_FILE}" 2>&1

notify_failure() {
  if command -v notify-send >/dev/null 2>&1; then
    notify-send --urgency=critical "Mini Ground Control" "$1\nLog: ${LOG_FILE}"
  fi
  printf '[error] %s\n' "$1" >&2
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  notify_failure "ROS ${ROS_DISTRO_NAME} is not installed"
  exit 2
fi
if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  notify_failure "Workspace is not built: ${WORKSPACE}"
  exit 2
fi
if [[ ! -x "${PYTHON_BIN}" ]]; then
  notify_failure "PySide environment is missing: ${PYTHON_BIN}"
  exit 2
fi

exec 9>"${RUNTIME_ROOT}/mini-ground-control-${UID}.lock"
if ! flock -n 9; then
  if command -v notify-send >/dev/null 2>&1; then
    notify-send "Mini Ground Control" "The application is already running"
  fi
  exit 0
fi

set +u
source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"
set -u
export DRONE_ROS_WS="${WORKSPACE}"
cd "${WORKSPACE}"
printf '[%s] starting Mini Ground Control from %s\n' "$(date --iso-8601=seconds)" "${WORKSPACE}"
exec "${PYTHON_BIN}" -m mini_ground_control.main "$@"
