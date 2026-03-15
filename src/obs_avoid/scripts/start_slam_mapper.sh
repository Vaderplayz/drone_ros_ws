#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[deprecated] start_slam_mapper.sh is deprecated."
echo "[info] use the single mapping entrypoint instead:"
echo "       ./obs_avoid/scripts/start_mapping_mode.sh"

exec "${SCRIPT_DIR}/start_mapping_mode.sh" "$@"
