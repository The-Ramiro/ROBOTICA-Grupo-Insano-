#!/usr/bin/env bash
# Gemelo en RViz del IRB 120 REAL: el modelo se mueve con los ejes que el controlador reporta por RWS (solo lectura).
#
# Mueves el robot con el FlexPendant; el controlador reporta los ejes; el puente los lee y publica en
# /irb120/real/joint_states; robot_state_publisher calcula las transformadas y RViz mueve el modelo.
# Este script NO envia ningun comando al robot.
#
# Uso (desde cualquier carpeta):
#   scripts/gemelo_rviz.sh --host 192.168.125.1 --user "Default User"
# Los argumentos se pasan al puente (abb_rws_joint_bridge.py); la contrasena se pide sin mostrarla.
# Para ensayar sin robot: --host 127.0.0.1 --port 8099 --user operador (con un controlador simulado).
set -euo pipefail

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
twin_dir="${ROS_TWIN_DIR:-$HOME/ros_twin}"
# RViz (Ogre/GLX) necesita Qt sobre X11; en una sesion Wayland se usa XWayland.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
py="${ROS_PY312:-$HOME/ros_py312/bin/python}"

if [[ ! -x "$py" ]]; then
  echo "Falta el entorno de Python 3.12: $py"
  echo "Crealo con: python3.12 -m venv ~/ros_py312 && ~/ros_py312/bin/pip install numpy pyyaml requests"
  exit 2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "No se encontro /opt/ros/jazzy/setup.bash"
  exit 2
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

"$py" "$here/preparar_gemelo.py" --out "$twin_dir"

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do kill "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher \
  --ros-args --params-file "$twin_dir/robot_state_publisher.yaml" \
  -r joint_states:=/irb120/real/joint_states &
pids+=($!)

rviz2 -d "$twin_dir/gemelo.rviz" &
pids+=($!)

echo "RViz y robot_state_publisher en marcha. Ctrl+C detiene todo."
"$py" "$here/abb_rws_joint_bridge.py" --rate 10 "$@"
