#!/usr/bin/env python3
"""Prepara los archivos del gemelo RViz del IRB 120 sin `colcon` ni `ros2` (que no funcionan con el Python 3.14).

Genera en --out (fuera del repositorio):
  - robot_state_publisher.yaml : parametros con el URDF del Hito 1 y las mallas como file:// absolutas
  - gemelo.rviz                : copia de la configuracion de RViz del Hito 1

No modifica nada dentro del repositorio.
"""
import argparse
from pathlib import Path
import shutil
import sys

import yaml

PROJECT = Path(__file__).resolve().parents[1]
PACKAGE = PROJECT / "ros2_ws" / "src" / "irb120_hito1"
URDF = PACKAGE / "urdf" / "irb120.urdf"
RVIZ = PACKAGE / "rviz" / "hito1.rviz"
PREFIX = "package://irb120_hito1/"


def build_urdf(text=None, package=PACKAGE):
    """URDF con las mallas apuntando a rutas file:// absolutas del paquete."""
    text = URDF.read_text(encoding="utf-8") if text is None else text
    if PREFIX not in text:
        raise ValueError("El URDF no contiene mallas package://irb120_hito1/.")
    return text.replace(PREFIX, f"file://{Path(package).resolve()}/")


def missing_meshes(urdf_text):
    """Rutas file:// del URDF que no existen en disco."""
    found, start = [], 0
    while True:
        i = urdf_text.find('filename="file://', start)
        if i < 0:
            break
        j = urdf_text.index('"', i + 10)
        path = Path(urdf_text[i + 10 + len("file://"):j])
        if not path.exists():
            found.append(str(path))
        start = j
    return found


def prepare(out):
    out = Path(out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    urdf = build_urdf()
    lost = missing_meshes(urdf)
    if lost:
        raise FileNotFoundError("Faltan mallas: " + ", ".join(lost[:3]))
    params = {"/**": {"ros__parameters": {"robot_description": urdf}}}
    (out / "robot_state_publisher.yaml").write_text(yaml.safe_dump(params, allow_unicode=True), encoding="utf-8")
    shutil.copyfile(RVIZ, out / "gemelo.rviz")
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="~/ros_twin")
    args = parser.parse_args(argv)
    try:
        out = prepare(args.out)
    except (OSError, ValueError) as exc:
        print(f"No se pudo preparar el gemelo: {exc}")
        return 2
    print(f"Gemelo preparado en {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
