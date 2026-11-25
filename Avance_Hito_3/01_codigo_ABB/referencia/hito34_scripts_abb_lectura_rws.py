#!/usr/bin/env python3
"""Inventario local y diagnóstico de lectura RWS 1.0 (RobotWare 6).

Sin --host no hay tráfico hacia el controlador. Con --host se usan únicamente
GET sobre recursos de lectura predefinidos. No inicia RAPID, motores ni EGM.
Ejecutar contra el equipo físico solo dentro del acceso supervisado del Gate 1.
"""

import argparse
from datetime import datetime, timezone
import getpass
import ipaddress
import json
import math
from pathlib import Path
import platform
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


READ_PATHS = frozenset({
    "/rw/system", "/rw/panel/ctrlstate", "/rw/panel/opmode",
    "/rw/rapid/execution", "/rw/motionsystem/mechunits",
})


def local_inventory():
    """Lee información local; no modifica la red ni busca equipos."""
    result = {"platform": platform.platform(), "python": platform.python_version()}
    interfaces = []
    base = Path("/sys/class/net")
    if base.exists():
        for item in sorted(base.iterdir()):
            row = {"name": item.name, "wireless": (item / "wireless").exists()}
            for field in ("operstate", "carrier"):
                try:
                    row[field] = (item / field).read_text().strip()
                except OSError:
                    row[field] = None
            interfaces.append(row)
    result["interfaces"] = interfaces
    for key, command in (("addresses", ["ip", "-j", "addr"]),
                         ("routes", ["ip", "-j", "route"])):
        try:
            proc = subprocess.run(command, capture_output=True, text=True, timeout=5)
            result[key] = json.loads(proc.stdout) if proc.returncode == 0 else {
                "unverified": proc.stderr.strip()}
        except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
            result[key] = {"unverified": str(exc)}
    return result


def state_records(text):
    """Extrae registros RWS JSON o XHTML sin ejecutar enlaces ni acciones."""
    try:
        obj = json.loads(text)
    except ValueError:
        obj = None
    if obj is not None:
        records = []

        def walk(value):
            if isinstance(value, dict):
                if "_type" in value or any(f"rax_{i}" in value for i in range(1, 7)):
                    records.append(value)
                for child in value.values():
                    walk(child)
            elif isinstance(value, list):
                for child in value:
                    walk(child)

        walk(obj)
        return records
    try:
        root = ET.fromstring(text)
    except ET.ParseError:
        return []
    records = []
    for element in root.iter():
        if element.tag.split("}")[-1] != "li":
            continue
        record = {"_type": element.get("class", ""), "_title": element.get("title", "")}
        for child in element.iter():
            if child.tag.split("}")[-1] == "span" and child.get("class"):
                record[child.get("class")] = "".join(child.itertext()).strip()
        records.append(record)
    return records


def robot_axes(records):
    """Devuelve seis valores robax finitos; no sustituye datos ausentes por ceros."""
    for row in records:
        try:
            values = [float(row[f"rax_{i}"]) for i in range(1, 7)]
        except (KeyError, ValueError, TypeError):
            continue
        if all(math.isfinite(value) for value in values):
            return values
    return None


def read_resource(session, base, path, timeout):
    permitted_joint_path = re.fullmatch(
        r"/rw/motionsystem/mechunits/[A-Za-z0-9_-]+/jointtarget", path)
    if path not in READ_PATHS and not permitted_joint_path:
        raise ValueError("Recurso fuera de la lista permitida de lectura.")
    response = session.get(base + path, params={"json": "1"}, timeout=timeout,
                           allow_redirects=False)
    # No se registran cabeceras de autenticación, cookies, contraseñas ni claves.
    return {"path": path, "http_status": response.status_code,
            "records": state_records(response.text) if response.status_code == 200 else []}


def probe(session, base, mechunit="ROB_1", timeout=4.0):
    if not re.fullmatch(r"[A-Za-z0-9_-]+", mechunit):
        raise ValueError("Nombre de unidad mecánica inválido.")
    report = {"transport": "RWS 1.0 / HTTP Digest", "base_url": base,
              "mechunit": mechunit, "mode": "read_only", "movement_authorized": False,
              "state": "unverified", "resources": []}
    first = read_resource(session, base, "/rw/system", timeout)
    report["resources"].append(first)
    if first["http_status"] in (401, 403):
        report["state"] = "http_reachable_authentication_or_permission_pending"
        return report
    if first["http_status"] != 200 or not any(
        row.get("_type") == "sys-system-li" for row in first["records"]
    ):
        report["state"] = "rws_identity_not_verified"
        return report
    report["state"] = "rws_system_observed"
    for path in ("/rw/panel/ctrlstate", "/rw/panel/opmode",
                 "/rw/rapid/execution", "/rw/motionsystem/mechunits"):
        report["resources"].append(read_resource(session, base, path, timeout))
    units = report["resources"][-1]
    if units["http_status"] != 200 or not any(
        row.get("_title") == mechunit for row in units["records"]
    ):
        report["joint_snapshot"] = None
        report["note"] = "Unidad mecánica solicitada no confirmada en la respuesta."
        return report
    joints = read_resource(session, base,
                           f"/rw/motionsystem/mechunits/{mechunit}/jointtarget", timeout)
    report["resources"].append(joints)
    axes = robot_axes(joints["records"]) if joints["http_status"] == 200 else None
    report["joint_snapshot"] = None if axes is None else {
        "rax_1_to_6": axes, "unit": "deg (convención jointtarget/robax de RAPID)",
        "origin": "controller_http_response", "physical_controller_identity_confirmed": False,
        "note": "Contrastar identidad y valores con el FlexPendant; no implica movimiento ni trayectoria aceptada."}
    if axes is not None:
        report["state"] = "rws_joint_snapshot_observed"
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", help="IPv4 del controlador autorizado; omitida = solo inventario local")
    parser.add_argument("--user", help="Usuario UAS autorizado; la contraseña se pide sin mostrarla")
    parser.add_argument("--mechunit", default="ROB_1", help="Confirmar en el controlador; defecto ROB_1")
    parser.add_argument("--timeout", type=float, default=4.0)
    parser.add_argument("--output", type=Path, help="Guardar evidencia JSON sin secretos")
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0 or args.timeout > 30:
        parser.error("--timeout debe estar entre 0 y 30 segundos.")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.mechunit):
        parser.error("--mechunit debe contener solo letras, números, guion o guion bajo.")
    result = {"observed_at_utc": datetime.now(timezone.utc).isoformat(),
              "local_inventory": local_inventory(), "controller_probe": None}
    code = 0
    if args.host:
        try:
            address = ipaddress.IPv4Address(args.host)
        except ipaddress.AddressValueError:
            parser.error("--host debe ser una IPv4 concreta, sin URL, puerto ni rango.")
        if address.is_multicast or address.is_unspecified or str(address) == "255.255.255.255":
            parser.error("No se permiten direcciones multicast, broadcast ni inespecíficas.")
        try:
            import requests
            from requests.auth import HTTPDigestAuth
        except ImportError:
            parser.error("Falta requests: instalar docs/conexion_abb/requirements.txt en un entorno virtual.")
        with requests.Session() as session:
            session.trust_env = False  # No enviar tráfico del robot por un proxy del entorno.
            if args.user:
                session.auth = HTTPDigestAuth(args.user, getpass.getpass("Contraseña UAS: "))
            try:
                result["controller_probe"] = probe(session, f"http://{address}",
                                                    args.mechunit, args.timeout)
                if result["controller_probe"]["state"] != "rws_joint_snapshot_observed":
                    code = 2
            except requests.RequestException as exc:
                result["controller_probe"] = {"state": "connection_or_service_not_verified",
                                               "error_type": type(exc).__name__,
                                               "movement_authorized": False}
                code = 2
    rendered = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n")
    print(rendered)
    return code


if __name__ == "__main__":
    sys.exit(main())
