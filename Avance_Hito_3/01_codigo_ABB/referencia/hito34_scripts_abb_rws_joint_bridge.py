#!/usr/bin/env python3
"""Puente de estado RWS -> ROS 2 (SOLO LECTURA): publica los seis ejes REALES del IRB 120 como sensor_msgs/JointState.

Cada ciclo hace UN GET a /rw/motionsystem/mechunits/<unidad>/jointtarget, convierte grados a radianes y publica.
No tiene suscriptores, servicios ni acciones: nada de ROS llega al controlador. Reutiliza el lector validado
(`abb_lectura_rws.py`), cuya lista de recursos permitidos impide cualquier otra ruta. No inicia RAPID, motores ni EGM.

Uso (Jazzy necesita Python 3.12; ver docs/conexion_abb/GUIA_PUENTE_ROS2.md):

    source /opt/ros/jazzy/setup.bash
    ~/ros_py312/bin/python scripts/abb_rws_joint_bridge.py --host 192.168.125.1 --user "Default User"

La contrasena se pide sin mostrarla y no se guarda. Los nombres joint_1..joint_6 son los del URDF del Hito 1.
"""

import argparse
import getpass
import importlib.util
import ipaddress
import json
import math
from pathlib import Path
import re
import sys
import time

_spec = importlib.util.spec_from_file_location("abb_lectura_rws", Path(__file__).with_name("abb_lectura_rws.py"))
abb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(abb)

JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
LIMITS_DEG = ((-165, 165), (-110, 110), (-110, 70), (-160, 160), (-120, 120), (-400, 400))
MAX_RATE_HZ = 10.0
DEFAULT_TOPIC = "/irb120/real/joint_states"


class BridgeError(RuntimeError):
    """Una lectura no se pudo completar; el puente no publica nada en ese ciclo."""


def build_base_url(host, port=80):
    """URL base. Solo IPv4 concreta; el puerto distinto de 80 se acepta unicamente en loopback (para simulacros)."""
    address = ipaddress.IPv4Address(host)
    if address.is_multicast or address.is_unspecified or str(address) == "255.255.255.255":
        raise ValueError("No se permiten direcciones multicast, broadcast ni inespecificas.")
    if not 0 < port < 65536:
        raise ValueError("Puerto fuera de rango.")
    if port != 80 and not address.is_loopback:
        raise ValueError("Solo se admite un puerto distinto de 80 en 127.0.0.1 (simulacro local).")
    return f"http://{address}" if port == 80 else f"http://{address}:{port}"


def fetch_joints_deg(session, base, mechunit="ROB_1", timeout=1.5):
    """Seis ejes en grados leidos del controlador; BridgeError si la lectura no es valida."""
    if not re.fullmatch(r"[A-Za-z0-9_-]+", mechunit):
        raise BridgeError("Nombre de unidad mecanica invalido.")
    result = abb.read_resource(session, base, f"/rw/motionsystem/mechunits/{mechunit}/jointtarget", timeout)
    status = result["http_status"]
    if status in (401, 403):
        raise BridgeError(f"HTTP {status}: autenticacion o permiso")
    if status != 200:
        raise BridgeError(f"HTTP {status} al leer jointtarget")
    axes = abb.robot_axes(result["records"])
    if axes is None:
        raise BridgeError("la respuesta no trae seis ejes finitos")
    return axes


def joint_state_fields(axes_deg):
    """(nombres, posiciones en radianes) del mensaje JointState."""
    if len(axes_deg) != 6 or not all(math.isfinite(v) for v in axes_deg):
        raise ValueError("Se necesitan seis ejes finitos.")
    return list(JOINT_NAMES), [math.radians(v) for v in axes_deg]


def outside_limits(axes_deg, margin_deg=1.0):
    """Ejes (1..6) fuera de los limites del datasheet, con un margen de tolerancia."""
    return [i + 1 for i, (v, (lo, hi)) in enumerate(zip(axes_deg, LIMITS_DEG))
            if not lo - margin_deg <= v <= hi + margin_deg]


def run(args, password, health_check=None):
    import rclpy
    import requests
    from requests.auth import HTTPDigestAuth
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    base = build_base_url(args.host, args.port)

    class Bridge(Node):
        def __init__(self):
            super().__init__("abb_rws_joint_bridge")
            self.publisher = self.create_publisher(JointState, args.topic, 10)
            self.telemetry = self.create_publisher(String, args.topic.rstrip("/") + "/rws", 10)
            self.session = requests.Session()
            self.session.trust_env = False  # nunca por un proxy del entorno
            self.session.auth = HTTPDigestAuth(args.user, password)
            self.published = 0
            self.failures = 0
            self.last_report = 0.0
            self.create_timer(1.0 / args.rate, self.tick)
            if health_check is not None:
                self.create_timer(0.5, health_check)
            self.get_logger().info(f"Leyendo {base} ({args.mechunit}) a {args.rate:g} Hz -> {args.topic}. SOLO LECTURA.")

        def tick(self):
            started = time.monotonic()
            try:
                axes = fetch_joints_deg(self.session, base, args.mechunit, timeout=min(1.5, 0.8 / args.rate + 0.5))
            except (BridgeError, requests.RequestException) as exc:
                self.failures += 1
                event = String()
                event.data = json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}",
                                         "failures": self.failures})
                self.telemetry.publish(event)
                if self.failures in (1, 10) or self.failures % 50 == 0:
                    self.get_logger().warning(f"Lectura fallida ({self.failures} seguidas): {type(exc).__name__}: {exc}")
                return
            if self.failures:
                self.get_logger().info(f"Lectura recuperada tras {self.failures} fallos.")
            self.failures = 0
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name, msg.position = joint_state_fields(axes)
            event = String()
            event.data = json.dumps({"ok": True, "seq": self.published + 1,
                                     "source": base, "mechunit": args.mechunit,
                                     "stamp": {"sec": msg.header.stamp.sec,
                                               "nanosec": msg.header.stamp.nanosec},
                                     "axes_deg": axes,
                                     "request_ms": (time.monotonic() - started) * 1000.0})
            self.telemetry.publish(event)
            self.publisher.publish(msg)
            self.published += 1
            bad = outside_limits(axes)
            if bad:
                self.get_logger().warning(f"Ejes fuera de los limites del datasheet: {bad}")
            now = time.monotonic()
            if args.print_every > 0 and now - self.last_report >= args.print_every:
                self.last_report = now
                print("ejes [deg]: " + "  ".join(f"{v:9.3f}" for v in axes), flush=True)
            if args.max_messages and self.published >= args.max_messages:
                raise SystemExit(0)

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.session.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", required=True, help="IPv4 del controlador (192.168.125.1)")
    parser.add_argument("--user", required=True, help="Usuario UAS; la contrasena se pide sin mostrarla")
    parser.add_argument("--mechunit", default="ROB_1")
    parser.add_argument("--rate", type=float, default=5.0, help=f"Lecturas por segundo (0 < f <= {MAX_RATE_HZ:g}); defecto 5")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help=f"Tema JointState (defecto {DEFAULT_TOPIC})")
    parser.add_argument("--port", type=int, default=80, help="Solo para simulacros en 127.0.0.1")
    parser.add_argument("--print-every", type=float, default=1.0, help="Segundos entre impresiones en pantalla (0 = nunca)")
    parser.add_argument("--max-messages", type=int, default=0, help="Salir tras N mensajes (0 = sin limite)")
    args = parser.parse_args(argv)
    if not (math.isfinite(args.rate) and 0 < args.rate <= MAX_RATE_HZ):
        parser.error(f"--rate debe estar entre 0 y {MAX_RATE_HZ:g} Hz.")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.mechunit):
        parser.error("--mechunit debe contener solo letras, numeros, guion o guion bajo.")
    try:
        build_base_url(args.host, args.port)
    except ValueError as exc:
        parser.error(str(exc))
    return run(args, getpass.getpass("Contraseña UAS: "))


if __name__ == "__main__":
    sys.exit(main())
