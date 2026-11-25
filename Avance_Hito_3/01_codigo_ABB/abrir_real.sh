#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
# Separado del visualizador local (dominio 43): sus sliders no interfieren.
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=44 LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/imt342-abb-real-${UID}.lock"
if ! flock -n 9; then
  echo 'ABB real ya esta abierto. Revisa sus ventanas.'
  exit 0
fi
cd "$ROOT"
exec /usr/bin/python3 abb_real.py
