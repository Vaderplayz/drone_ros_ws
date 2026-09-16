#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
PACKAGE_ROOT="$(cd "$(dirname "${SCRIPT_PATH}")/.." && pwd)"
LAUNCHER_SOURCE="${PACKAGE_ROOT}/scripts/launch_desktop_app.sh"
TEMPLATE="${PACKAGE_ROOT}/desktop/mini-ground-control.desktop.in"
ICON_SOURCE="${PACKAGE_ROOT}/icons/mini-ground-control.svg"
BIN_DIR="${HOME}/.local/bin"
APP_DIR="${HOME}/.local/share/applications"
ICON_DIR="${HOME}/.local/share/icons/hicolor/scalable/apps"
EXECUTABLE="${BIN_DIR}/mini-ground-control"
DESKTOP_FILE="${APP_DIR}/mini-ground-control.desktop"

mkdir -p "${BIN_DIR}" "${APP_DIR}" "${ICON_DIR}"
chmod +x "${LAUNCHER_SOURCE}"
ln -sfn "${LAUNCHER_SOURCE}" "${EXECUTABLE}"
install -m 0644 "${ICON_SOURCE}" "${ICON_DIR}/mini-ground-control.svg"
sed "s|@EXECUTABLE@|${EXECUTABLE}|g" "${TEMPLATE}" >"${DESKTOP_FILE}.tmp"
install -m 0644 "${DESKTOP_FILE}.tmp" "${DESKTOP_FILE}"
rm -f "${DESKTOP_FILE}.tmp"

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "${APP_DIR}" >/dev/null 2>&1 || true
fi
if command -v gtk-update-icon-cache >/dev/null 2>&1; then
  gtk-update-icon-cache -f -t "${HOME}/.local/share/icons/hicolor" >/dev/null 2>&1 || true
fi

if command -v gsettings >/dev/null 2>&1; then
  favorites="$(gsettings get org.gnome.shell favorite-apps 2>/dev/null || true)"
  if [[ "${favorites}" != *"'mini-ground-control.desktop'"* ]]; then
    if [[ "${favorites}" == "[]" ]]; then
      updated="['mini-ground-control.desktop']"
    else
      updated="${favorites%]}, 'mini-ground-control.desktop']"
    fi
    gsettings set org.gnome.shell favorite-apps "${updated}"
  fi
fi

printf 'Installed Mini Ground Control\n'
printf '  launcher: %s\n' "${DESKTOP_FILE}"
printf '  log:      %s\n' "${XDG_STATE_HOME:-${HOME}/.local/state}/mini-ground-control/app.log"
