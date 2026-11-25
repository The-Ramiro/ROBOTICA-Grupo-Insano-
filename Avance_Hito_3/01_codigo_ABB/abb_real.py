#!/usr/bin/env python3
"""ABB RWS -> JointState -> RViz. Consultas GET; sin consignas al robot."""
import ipaddress
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import http.cookiejar
import xml.etree.ElementTree as ET

import numpy as np
import yaml
import rclpy
from sensor_msgs.msg import JointState
from PyQt5 import QtCore, QtWidgets
from abb_rws_joint_bridge import fetch_joints_deg, joint_state_fields, BridgeError
from abb_lectura_rws import state_records

ROOT = Path(__file__).resolve().parent
TOPIC = '/irb120/real/joint_states'


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


class Session:
    """Adaptador del lector del repositorio, Digest y cookies, sin proxy."""
    def __init__(self, base, user, password):
        auth = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        auth.add_password(None, base, user, password)
        self.opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({}), NoRedirect(),
            urllib.request.HTTPDigestAuthHandler(auth),
            urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))

    def get(self, url, params=None, timeout=0.8, allow_redirects=False):
        if params:
            url += '?' + urllib.parse.urlencode(params)
        try:
            with self.opener.open(urllib.request.Request(url, method='GET'), timeout=timeout) as r:
                status, body = r.status, r.read().decode('utf-8')
        except urllib.error.HTTPError as exc:
            status, body = exc.code, ''
            exc.close()
        return type('Response', (), {'status_code': status, 'text': body})()


class Reader(threading.Thread):
    def __init__(self, host, user, password):
        super().__init__(daemon=True)
        self.base = 'http://' + host
        self.credentials = (user, password)
        self.session = Session(self.base, user, password)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.seq = 0
        self.axes = None
        self.when = 0.0
        self.tcp = None
        self.tcp_when = 0.0
        self.error = 'Esperando la primera lectura...'
        self.tcp_error = ''
        self.tcp_quaternion = None
        self.comparison = None

    def run(self):
        next_tcp = 0.0
        while not self.stop.is_set():
            started = time.monotonic()
            try:
                axes = fetch_joints_deg(self.session, self.base, 'ROB_1', timeout=0.7)
                with self.lock:
                    self.axes, self.when = axes, time.monotonic()
                    self.seq += 1
                    self.error = ''
            except (BridgeError, OSError, ValueError) as exc:
                with self.lock:
                    self.error = f'{type(exc).__name__}: {exc}'
                self.stop.wait(0.5)
                continue
            if time.monotonic() >= next_tcp and not self.stop.is_set():
                try:
                    tcp_started = time.monotonic()
                    response = self.session.get(
                        self.base + '/rw/motionsystem/mechunits/ROB_1/robtarget',
                        params={'json': '1', 'coordinate': 'Base', 'tool': 'tool0', 'wobj': 'wobj0'},
                        timeout=0.4)
                    if response.status_code != 200:
                        raise ValueError(f'HTTP {response.status_code}')
                    values = None
                    quaternion = None
                    for row in state_records(response.text):
                        if all(k in row for k in ('x', 'y', 'z')):
                            values = [float(row[k]) for k in ('x', 'y', 'z')]
                            if all(k in row for k in ('q1', 'q2', 'q3', 'q4')):
                                raw = [float(row[k]) for k in ('q2', 'q3', 'q4', 'q1')]
                                if all(math.isfinite(v) for v in raw) and sum(v*v for v in raw) > 1e-12:
                                    quaternion = raw
                            break
                    if values is None or not all(math.isfinite(v) for v in values):
                        raise ValueError('XYZ ausentes o invalidos')
                    with self.lock:
                        self.tcp, self.tcp_when = values, time.monotonic()
                        self.tcp_error = ''
                        self.tcp_quaternion = quaternion
                        # Conservar los ejes de esta consulta, no otros posteriores.
                        self.comparison = (list(axes), list(values), self.when,
                                           (tcp_started + self.tcp_when)/2)
                except (OSError, ValueError) as exc:
                    with self.lock:
                        self.tcp_error = str(exc)
                next_tcp = time.monotonic() + 0.5
            self.stop.wait(max(0.0, 0.1 - (time.monotonic() - started)))

    def snapshot(self):
        with self.lock:
            return (self.seq, self.axes, self.when, self.error,
                    self.tcp, self.tcp_when, self.tcp_error, self.comparison)

    def orientation_snapshot(self):
        with self.lock:
            quaternion = None if self.tcp_quaternion is None else list(self.tcp_quaternion)
            angles = None if self.comparison is None else list(self.comparison[0])
            return quaternion, angles, self.tcp_when, self.tcp_error


def origin_matrix(xyz, rpy):
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = xyz
    return t


def tool_position(model, positions):
    """FK del mismo URDF que muestra RViz: brida tool0, respecto a base_link."""
    children = {j.find('child').get('link'): j for j in model.findall('joint')}
    chain, link = [], 'tool0'
    while link != 'base_link':
        j = children[link]
        chain.append(j)
        link = j.find('parent').get('link')
    t = np.eye(4)
    for j in reversed(chain):
        origin = j.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
        t = t @ origin_matrix(xyz, rpy)
        if j.get('type') == 'revolute':
            axis = np.array([float(v) for v in j.find('axis').get('xyz').split()])
            axis /= np.linalg.norm(axis)
            x, y, z = axis
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            q = positions[j.get('name')]
            rotation = np.eye(4)
            rotation[:3, :3] = np.eye(3) + math.sin(q)*skew + (1-math.cos(q))*(skew @ skew)
            t = t @ rotation
    return t[:3, 3]*1000.0


def main():
    app = QtWidgets.QApplication(sys.argv)
    dialog = QtWidgets.QDialog()
    dialog.setWindowTitle('Conectar ABB real')
    form = QtWidgets.QFormLayout(dialog)
    host = QtWidgets.QLineEdit('192.168.125.1')
    user = QtWidgets.QLineEdit('Default User')
    password = QtWidgets.QLineEdit()
    password.setEchoMode(QtWidgets.QLineEdit.Password)
    password.setPlaceholderText('Vacia: cuenta de fabrica documentada por ABB')
    form.addRow('IP del controlador', host)
    form.addRow('Usuario', user)
    form.addRow('Contrasena', password)
    form.addRow(QtWidgets.QLabel('Lee el robot y muestra su postura. Muevelo desde el FlexPendant.'))
    buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    form.addRow(buttons)
    if dialog.exec_() != QtWidgets.QDialog.Accepted:
        return 0
    address = ipaddress.IPv4Address(host.text().strip())
    if address.is_multicast or address.is_unspecified or int(address) & 255 == 255:
        raise ValueError('Usa la IP concreta del controlador')
    reader = Reader(str(address), user.text(), password.text() or 'robotics')
    password.clear()

    xml = (ROOT/'urdf/irb120.urdf').read_text().replace('package://abb_irb120_support/', ROOT.as_uri()+'/')
    model = ET.fromstring(xml)
    joint_names = [j.get('name') for j in model.findall('joint') if j.get('type') == 'revolute']
    if joint_names != list(joint_state_fields([0]*6)[0]):
        raise ValueError('Nombres/orden de ejes distintos de joint_1..joint_6')
    runtime = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home()/'.cache'))) / 'imt342_abb_real'
    runtime.mkdir(parents=True, exist_ok=True)
    params = runtime/'robot_description.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': xml}}}))
    processes, logs = [], []
    rclpy.init(args=[])
    node = rclpy.create_node('abb_real_monitor')
    pub = node.create_publisher(JointState, TOPIC, 1)
    win = QtWidgets.QWidget()
    win.setWindowTitle('ABB REAL | Seguimiento en RViz')
    win.resize(690, 650)
    layout = QtWidgets.QVBoxLayout(win)
    layout.addWidget(QtWidgets.QLabel('<h2>ABB real → RViz</h2>ROB_1 · ' + str(address)))
    status = QtWidgets.QLabel('Conectando...')
    status.setWordWrap(True)
    layout.addWidget(status)
    table = QtWidgets.QTableWidget(6, 5)
    table.setHorizontalHeaderLabels(['Eje', 'Real [°]', 'Referencia [°]', 'Ref. - real [°]', '|Error| [°]'])
    table.verticalHeader().setVisible(False)
    table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
    table.setMinimumHeight(220)
    references = []
    for i in range(6):
        for column, text in ((0, str(i+1)), (1, '—'), (3, '—'), (4, '—')):
            item = QtWidgets.QTableWidgetItem(text)
            item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
            table.setItem(i, column, item)
        reference = QtWidgets.QLineEdit()
        reference.setPlaceholderText('Sin referencia')
        references.append(reference)
        table.setCellWidget(i, 2, reference)
    layout.addWidget(table)
    reference_buttons = QtWidgets.QHBoxLayout()
    capture = QtWidgets.QPushButton('Tomar postura actual como referencia')
    clear = QtWidgets.QPushButton('Borrar referencias')
    reference_buttons.addWidget(capture)
    reference_buttons.addWidget(clear)
    layout.addLayout(reference_buttons)
    limits_button = QtWidgets.QPushButton('Configurar limites y porcentajes')
    layout.addWidget(limits_button)
    def edit_limits():
        processes.append(subprocess.Popen(['/usr/bin/python3', str(ROOT/'editar_limites.py')]))
    limits_button.clicked.connect(edit_limits)
    def capture_reference():
        _, current, stamp, failed, *_ = reader.snapshot()
        if current is not None and not failed and time.monotonic()-stamp < 1:
            for field, value in zip(references, current):
                field.setText(f'{value:.9f}')
    capture.clicked.connect(capture_reference)
    clear.clicked.connect(lambda: [field.clear() for field in references])
    actual_xyz = QtWidgets.QLabel('ABB tool0 / Base [mm]: —')
    model_xyz = QtWidgets.QLabel('Modelo tool0 / base_link [mm]: —')
    actual_xyz.setWordWrap(True)
    model_xyz.setWordWrap(True)
    layout.addWidget(actual_xyz)
    layout.addWidget(model_xyz)
    position_error = QtWidgets.QLabel('Diferencia modelo - ABB [mm]: —')
    position_error.setWordWrap(True)
    position_error.setStyleSheet('font-weight: bold; font-size: 15px')
    layout.addWidget(position_error)
    note = QtWidgets.QLabel('La referencia articular solo sirve para comparar; no mueve el robot.\nError articular: referencia - real, sin reducir vueltas de J6.\nXYZ: brida tool0 respecto a Base; sin herramienta adicional.\nModelo y ABB se leen sucesivamente: compara en reposo.\nActualizacion de ejes: hasta 10 Hz. Cerrar detiene RViz.')
    note.setWordWrap(True)
    layout.addWidget(note)
    last_seq, last_when, rate = 0, None, 0.0

    def update():
        nonlocal last_seq, last_when, rate
        seq, axes, when, error, tcp, tcp_when, tcp_error, comparison = reader.snapshot()
        now = time.monotonic()
        fresh = axes is not None and now-when < 1.0 and not error
        if fresh:
            status.setStyleSheet('color: #087d35; font-weight: bold')
            status.setText(f'EN VIVO · {rate:.1f} lecturas/s · dato hace {now-when:.2f} s')
        else:
            status.setStyleSheet('color: #b34c00; font-weight: bold')
            status.setText('SIN DATOS RECIENTES · Modelo en la ultima postura recibida.\n' + error)
        if seq != last_seq and axes is not None and fresh:
            if last_when is not None and when > last_when:
                measured = 1.0/(when-last_when)
                rate = measured if rate == 0 else 0.8*rate+0.2*measured
            last_seq, last_when = seq, when
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name, msg.position = joint_state_fields(axes)
            pub.publish(msg)
            xyz = tool_position(model, dict(zip(msg.name, msg.position)))
            model_xyz.setText('Modelo tool0 / base_link [mm]:\n' + '   '.join(f'{k}={v:.6f}' for k,v in zip('XYZ', xyz)))
        if tcp is not None:
            stale = bool(tcp_error) or now-tcp_when > 2.0
            actual_xyz.setText('ABB tool0 / Base [mm]' + (' · dato anterior' if stale else '') + ':\n' + '   '.join(f'{k}={v:.6f}' for k,v in zip('XYZ', tcp)))
        elif tcp_error:
            actual_xyz.setText('ABB XYZ no disponible: ' + tcp_error)
        capture.setEnabled(fresh)
        for i, field in enumerate(references):
            table.item(i, 1).setText(f'{axes[i]:.6f}' if fresh else '—')
            difference = None
            if fresh and field.text().strip():
                try:
                    reference = float(field.text().strip().replace(',', '.'))
                    if math.isfinite(reference):
                        difference = reference - axes[i]
                except ValueError:
                    pass
            table.item(i, 3).setText('—' if difference is None else f'{difference:+.9f}')
            table.item(i, 4).setText('—' if difference is None else f'{abs(difference):.9f}')
            field.setToolTip('Angulo de referencia en grados. No envia consignas al ABB.')
        if fresh and comparison is not None and not tcp_error and now-tcp_when < 2:
            paired_axes, paired_tcp, joints_stamp, cartesian_stamp = comparison
            names, positions = joint_state_fields(paired_axes)
            predicted = tool_position(model, dict(zip(names, positions)))
            delta = predicted - np.array(paired_tcp)
            position_error.setText('Diferencia modelo - ABB [mm]:\n' +
                '   '.join(f'Δ{k}={v:+.9f}' for k, v in zip('XYZ', delta)) +
                f'\nDistancia total: {np.linalg.norm(delta):.9f} mm · separacion de lecturas: {abs(cartesian_stamp-joints_stamp)*1000:.0f} ms')
        else:
            position_error.setText('Diferencia modelo - ABB [mm]: —\nFaltan lecturas recientes de ejes o XYZ.')
        rclpy.spin_once(node, timeout_sec=0)
        if any(p.poll() is not None for p in processes[:2]):
            app.quit()

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    try:
        for args, name in [
            (['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher', '--ros-args', '--params-file', str(params), '-r', 'joint_states:='+TOPIC], 'robot_state_publisher.log'),
            (['/opt/ros/humble/lib/rviz2/rviz2', '-d', str(ROOT/'irb120.rviz')], 'rviz.log')]:
            log = (runtime/name).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT))
        reader.start()
        timer.start(30)
        win.show()
        app.exec_()
    finally:
        timer.stop()
        reader.stop.set()
        if reader.ident is not None:
            reader.join(timeout=4)
        for p in processes:
            if p.poll() is None:
                p.terminate()
        for p in processes:
            try:
                p.wait(timeout=4)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
