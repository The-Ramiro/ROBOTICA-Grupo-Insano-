#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=45 ROS_LOCALHOST_ONLY=1 LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb
RWS_SRC="$ROOT/rws_equipo/ros2_ws/src"
export PYTHONPATH="$RWS_SRC/irb120_control:$RWS_SRC/irb120_conexion${PYTHONPATH:+:$PYTHONPATH}"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/imt342-panel-${UID}.lock"
if ! flock -n 9; then
  echo 'El panel didactico ya esta abierto.'
  exit 0
fi
exec /usr/bin/python3 "$ROOT/panel_robotica.py"
