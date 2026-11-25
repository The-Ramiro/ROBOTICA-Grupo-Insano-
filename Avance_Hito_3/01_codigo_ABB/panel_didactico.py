#!/usr/bin/env python3
"""Panel de exploración y trayectoria simulada. No tiene salida de movimiento físico."""
from concurrent.futures import ThreadPoolExecutor
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import yaml
import numpy as np
from PyQt5 import QtCore, QtWidgets
import rclpy
from sensor_msgs.msg import JointState
from abb_real import Reader
from limites_movimiento import FACTORY, SOURCE, load, save, effective, angle_percent
from objetivos_preview import Model, configured_envelope, prepare_target

ROOT = Path(__file__).resolve().parent


class Preview:
    def __init__(self):
        self.processes, self.logs = [], []
        rclpy.init(args=[])
        self.node = rclpy.create_node('abb_didactico_vista_previa')
        self.pub = self.node.create_publisher(JointState, '/preview/joint_states', 1)
        cache = Path.home()/'.cache/imt342_preview'
        cache.mkdir(parents=True, exist_ok=True)
        xml = (ROOT/'urdf/irb120.urdf').read_text().replace('package://abb_irb120_support/', ROOT.as_uri()+'/')
        params = cache/'robot.yaml'
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': xml}}}))
        config = yaml.safe_load((ROOT/'irb120.rviz').read_text())
        config['Visualization Manager']['Displays'][1]['Name'] = 'VISTA PREVIA · SIN ENVIO AL ROBOT'
        config['Panels'] = []
        config['Window Geometry'].update({'Width': 500, 'Height': 680, 'X': 760, 'Y': 40,
                                          'Hide Left Dock': True, 'Hide Right Dock': True})
        rviz = cache/'preview.rviz'
        rviz.write_text(yaml.safe_dump(config, allow_unicode=True))
        try:
            for cmd, name in [
                (['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher', '--ros-args', '--params-file', str(params), '-r', 'joint_states:=/preview/joint_states'], 'rsp.log'),
                (['/opt/ros/humble/lib/rviz2/rviz2', '-d', str(rviz)], 'rviz.log')]:
                stream = (cache/name).open('w')
                self.logs.append(stream)
                self.processes.append(subprocess.Popen(cmd, stdout=stream, stderr=subprocess.STDOUT))
        except Exception:
            self.close()
            raise

    def publish(self, q):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [f'joint_{i}' for i in range(1, 7)]
        msg.position = [math.radians(v) for v in q]
        self.pub.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0)

    def close(self):
        for p in self.processes:
            if p.poll() is None:
                p.terminate()
        for p in self.processes:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        for stream in self.logs:
            stream.close()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class Panel(QtWidgets.QWidget):
    def __init__(self, preview):
        super().__init__()
        self.preview, self.model = preview, Model()
        self.config = load()
        self.envelope = configured_envelope(require_floor=False)
        self.reader = None
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.version = 0
        self.lock_ui = False
        self.plan = None
        self.animation = None
        self.initial = [0., -30., 30., 0., 45., 0.]
        self.rendered = self.initial.copy()
        self.setWindowTitle('ABB · Panel didactico | Vista previa')
        self.resize(690, 710)
        self.move(70, 30)
        self.setStyleSheet('QGroupBox {font-weight:bold; margin-top:8px;} QGroupBox::title {subcontrol-origin:margin;} QPushButton {padding:7px;} QSlider::handle:horizontal {background:#217bb6; width:17px; margin:-4px 0; border-radius:5px;}')
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel('<h2>ABB IRB120 · Explorar y preparar</h2>'))
        badge = QtWidgets.QLabel('VISTA PREVIA  ·  El brazo real no recibe movimientos')
        badge.setStyleSheet('background:#ddeeff;color:#14507b;padding:8px;font-weight:bold;')
        layout.addWidget(badge)
        top = QtWidgets.QHBoxLayout()
        connect = QtWidgets.QPushButton('Conectar lectura del ABB')
        connect.clicked.connect(self.connect_reader)
        self.copy_real = QtWidgets.QPushButton('Usar postura real como inicio')
        self.copy_real.setEnabled(False)
        self.copy_real.clicked.connect(self.use_real)
        top.addWidget(connect)
        top.addWidget(self.copy_real)
        layout.addLayout(top)
        self.real_status = QtWidgets.QLabel('Inicio: postura de muestra. No es una lectura real.')
        self.real_status.setWordWrap(True)
        layout.addWidget(self.real_status)
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, 1)
        joints_tab = QtWidgets.QWidget()
        joints = QtWidgets.QVBoxLayout(joints_tab)
        intro = QtWidgets.QLabel('1 · Mueve los deslizadores o escribe un angulo. Solo cambia el modelo 3D.')
        intro.setWordWrap(True)
        joints.addWidget(intro)
        self.sliders, self.angles, self.percentages = [], [], []
        bounds = effective(self.config)
        for i, row in enumerate(bounds):
            group = QtWidgets.QGroupBox(f'J{i+1} · ABB {FACTORY[i][0]:g}° a {FACTORY[i][1]:g}°')
            line = QtWidgets.QHBoxLayout(group)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(math.ceil(row['min_deg']*1000), math.floor(row['max_deg']*1000))
            angle = QtWidgets.QDoubleSpinBox()
            angle.setRange(slider.minimum()/1000, slider.maximum()/1000)
            angle.setDecimals(3)
            angle.setSuffix(' °')
            angle.setValue(self.initial[i])
            slider.setValue(round(angle.value()*1000))
            percentage = QtWidgets.QLabel()
            percentage.setMinimumWidth(60)
            line.addWidget(slider, 1)
            line.addWidget(angle)
            line.addWidget(percentage)
            slider.valueChanged.connect(lambda value, index=i: self.slider_changed(index, value))
            angle.valueChanged.connect(lambda value, index=i: self.angle_changed(index, value))
            self.sliders.append(slider)
            self.angles.append(angle)
            self.percentages.append(percentage)
            joints.addWidget(group)
        joints.addWidget(QtWidgets.QLabel('Porcentaje del rango: 0% = minimo ABB; 100% = maximo ABB.'))
        joints_scroll = QtWidgets.QScrollArea()
        joints_scroll.setWidgetResizable(True)
        joints_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        joints_scroll.setWidget(joints_tab)
        self.tabs.addTab(joints_scroll, 'Angulos · deslizadores')
        xyz_tab = QtWidgets.QWidget()
        xyz_layout = QtWidgets.QVBoxLayout(xyz_tab)
        text = QtWidgets.QLabel('2 · Escribe el destino de la brida en milimetros, respecto a la base.\nSe conserva la orientacion de la postura de inicio.\nSe calcula una trayectoria en articulaciones: no una recta cartesiana.')
        text.setWordWrap(True)
        xyz_layout.addWidget(text)
        self.xyz = []
        form = QtWidgets.QFormLayout()
        initial_xyz = self.model.tip(self.initial, self.envelope)[:3, 3]*1000
        for key, value in zip('XYZ', initial_xyz):
            field = QtWidgets.QDoubleSpinBox()
            field.setRange(-1000, 1200)
            field.setDecimals(3)
            field.setSuffix(' mm')
            field.setValue(float(value))
            field.valueChanged.connect(self.invalidate)
            form.addRow(key, field)
            self.xyz.append(field)
        xyz_layout.addLayout(form)
        calculate = QtWidgets.QPushButton('Calcular angulos y preparar trayectoria XYZ')
        calculate.clicked.connect(lambda: self.prepare('xyz'))
        xyz_layout.addWidget(calculate)
        xyz_layout.addStretch()
        self.tabs.addTab(xyz_tab, 'Destino X · Y · Z')
        speed_tab = QtWidgets.QWidget()
        speed_layout = QtWidgets.QVBoxLayout(speed_tab)
        speed_layout.addWidget(QtWidgets.QLabel('3 · Velocidad de la trayectoria (% del maximo de ABB)'))
        speed_line = QtWidgets.QHBoxLayout()
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(0, 10000)
        self.speed = QtWidgets.QDoubleSpinBox()
        self.speed.setRange(0, 100)
        self.speed.setDecimals(2)
        self.speed.setSuffix(' %')
        self.speed.setValue(self.config['global_speed_percent'])
        self.speed_slider.setValue(round(self.speed.value()*100))
        self.speed_slider.valueChanged.connect(lambda v: self.speed.setValue(v/100))
        self.speed.valueChanged.connect(self.speed_changed)
        speed_line.addWidget(self.speed_slider, 1)
        speed_line.addWidget(self.speed)
        speed_layout.addLayout(speed_line)
        self.speed_table = QtWidgets.QTableWidget(6, 4)
        self.speed_table.setHorizontalHeaderLabels(['Eje', 'Maximo ABB\n[°/s]', 'Limite elegido\n[°/s]', '% efectivo\nABB'])
        self.speed_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.speed_table.verticalHeader().hide()
        self.speed_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        speed_layout.addWidget(self.speed_table)
        self.travel = QtWidgets.QDoubleSpinBox()
        self.travel.setRange(.01, 100)
        self.travel.setDecimals(2)
        self.travel.setSuffix(' % del rango ABB por orden')
        self.travel.setValue(min(row['max_command_travel_percent'] for row in self.config['axes']))
        self.travel.valueChanged.connect(self.travel_changed)
        speed_layout.addWidget(QtWidgets.QLabel('Desplazamiento maximo por eje en una propuesta:'))
        speed_layout.addWidget(self.travel)
        actions = QtWidgets.QHBoxLayout()
        keep = QtWidgets.QPushButton('Guardar ajustes')
        keep.clicked.connect(self.save_limits)
        advanced = QtWidgets.QPushButton('Rangos y ajustes por eje…')
        advanced.clicked.connect(self.advanced)
        reload = QtWidgets.QPushButton('Recargar ajustes')
        reload.clicked.connect(self.reload_limits)
        for b in (keep, advanced, reload): actions.addWidget(b)
        speed_layout.addLayout(actions)
        note = QtWidgets.QLabel(f'<a href="{SOURCE}">Ficha del fabricante ABB</a> · IRB120 estandar, no IRB120T.\nEl % es un limite configurado, no velocidad medida. La duracion se adapta al eje mas restrictivo.')
        note.setOpenExternalLinks(True)
        note.setWordWrap(True)
        speed_layout.addWidget(note)
        speed_scroll = QtWidgets.QScrollArea()
        speed_scroll.setWidgetResizable(True)
        speed_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        speed_scroll.setWidget(speed_tab)
        self.tabs.addTab(speed_scroll, 'Velocidad y limites')
        self.preview_xyz = QtWidgets.QLabel()
        self.preview_xyz.setWordWrap(True)
        layout.addWidget(self.preview_xyz)
        self.status = QtWidgets.QLabel('Explora una postura y prepara su trayectoria antes de reproducirla.')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        bottom = QtWidgets.QHBoxLayout()
        self.prepare_button = QtWidgets.QPushButton('Preparar trayectoria')
        self.prepare_button.clicked.connect(lambda: self.prepare('xyz' if self.tabs.currentIndex()==1 else 'joint'))
        self.play = QtWidgets.QPushButton('▶ Reproducir vista previa')
        self.play.setEnabled(False)
        self.play.clicked.connect(self.animate)
        stop = QtWidgets.QPushButton('■ Detener vista previa')
        stop.clicked.connect(self.stop_animation)
        for b in (self.prepare_button, self.play, stop): bottom.addWidget(b)
        layout.addLayout(bottom)
        self.execute = QtWidgets.QPushButton('Ejecutar en el brazo real · BLOQUEADO')
        self.execute.setEnabled(False)
        self.execute.setToolTip('Falta integrar el canal fisico ABB y validar las protecciones de la celda. No existe envio de consignas en este panel.')
        layout.addWidget(self.execute)
        warning = QtWidgets.QLabel('Altura: brida sobre el plano de la base. Piso fisico y colisiones de la celda pendientes.\nDetener vista previa solo detiene la animacion, no es una parada de emergencia del robot.')
        warning.setWordWrap(True)
        layout.addWidget(warning)
        self.refresh_goal()
        self.refresh_speed()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)

    def goal(self): return [a.value() for a in self.angles]

    def invalidate(self, *_):
        if self.lock_ui: return
        self.version += 1
        self.plan = None
        self.animation = None
        if hasattr(self, 'play'): self.play.setEnabled(False)

    def slider_changed(self, i, value):
        self.angles[i].setValue(value/1000)

    def angle_changed(self, i, value):
        blocker = QtCore.QSignalBlocker(self.sliders[i])
        self.sliders[i].setValue(round(value*1000))
        del blocker
        if not self.lock_ui:
            self.invalidate()
            self.refresh_goal()

    def refresh_goal(self):
        q = self.goal()
        for i, field in enumerate(self.percentages): field.setText(f'{angle_percent(i,q[i]):.1f}%')
        try:
            bounds = effective(self.config)
            if any(not row['min_deg'] <= v <= row['max_deg'] for v,row in zip(q,bounds)):
                raise ValueError('Destino fuera de los rangos configurados')
            z = self.model.check_height(q, self.envelope)
            xyz = self.model.tip(q, self.envelope)[:3, 3]*1000
            self.preview_xyz.setText('Destino del modelo: ' + '   '.join(f'{k} = {v:.3f} mm' for k,v in zip('XYZ',xyz)))
            self.rendered = q.copy()
        except ValueError as exc:
            self.status.setText('Postura no mostrada: '+str(exc))

    def speed_changed(self, value):
        if self.lock_ui: return
        blocker = QtCore.QSignalBlocker(self.speed_slider)
        self.speed_slider.setValue(round(value*100))
        del blocker
        self.config['global_speed_percent'] = value
        self.invalidate()
        self.refresh_speed()

    def travel_changed(self, value):
        if self.lock_ui: return
        for row in self.config['axes']: row['max_command_travel_percent'] = value
        self.invalidate()

    def refresh_speed(self):
        for i, row in enumerate(effective(self.config)):
            for col, value in enumerate((f'J{i+1}', f'{FACTORY[i][2]:g}', f"{row['speed_deg_s']:.3f}", f"{row['speed_percent']:.2f}%")):
                self.speed_table.setItem(i,col,QtWidgets.QTableWidgetItem(value))

    def save_limits(self):
        save(self.config)
        self.status.setText('Ajustes guardados. No se envio ninguna orden al robot.')

    def advanced(self):
        subprocess.Popen(['/usr/bin/python3', str(ROOT/'editar_limites.py')])

    def reload_limits(self):
        self.config = load()
        self.lock_ui = True
        self.speed.setValue(self.config['global_speed_percent'])
        self.speed_slider.setValue(round(self.speed.value()*100))
        self.travel.setValue(min(r['max_command_travel_percent'] for r in self.config['axes']))
        self.lock_ui = False
        self.invalidate()
        self.refresh_speed()
        self.refresh_goal()
        self.status.setText('Ajustes recargados. Prepara de nuevo la trayectoria.')

    def connect_reader(self):
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle('Leer ABB real (sin movimiento)')
        form = QtWidgets.QFormLayout(dialog)
        host, user, password = QtWidgets.QLineEdit('192.168.125.1'), QtWidgets.QLineEdit('Default User'), QtWidgets.QLineEdit()
        password.setEchoMode(QtWidgets.QLineEdit.Password)
        password.setPlaceholderText('Vacia: acceso de fabrica documentado por ABB')
        for key,field in (('IP',host),('Usuario',user),('Contrasena',password)): form.addRow(key,field)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok|QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if dialog.exec_() != QtWidgets.QDialog.Accepted: return
        import ipaddress
        try:
            address = ipaddress.IPv4Address(host.text().strip())
            if address.is_multicast or address.is_unspecified or int(address)&255==255: raise ValueError('IP no valida')
            if self.reader is not None:
                self.reader.stop.set()
                self.reader.join(timeout=3)
            self.reader = Reader(str(address),user.text(),password.text() or 'robotics')
            self.reader.start()
            password.clear()
        except ValueError as exc: self.status.setText(str(exc))

    def use_real(self):
        if self.reader is None: return
        _,q,stamp,error,*_ = self.reader.snapshot()
        if q is None or error or time.monotonic()-stamp>1: return
        bounds = effective(self.config)
        if any(not row['min_deg'] <= v <= row['max_deg'] for v,row in zip(q,bounds)):
            self.status.setText('La postura real queda fuera de los limites de vista previa. No se recorto ni sustituyo.')
            return
        try: self.model.check_height(q,self.envelope)
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        self.initial = list(q)
        self.set_goal(q)
        self.status.setText('Inicio copiado de la lectura real. El modelo sigue siendo una vista previa.')

    def set_goal(self,q):
        self.lock_ui = True
        for field,value in zip(self.angles,q): field.setValue(value)
        xyz = self.model.tip(q,self.envelope)[:3,3]*1000
        for field,value in zip(self.xyz,xyz): field.setValue(float(value))
        self.lock_ui = False
        self.invalidate()
        self.refresh_goal()

    def prepare(self,mode):
        if self.future is not None: return
        self.invalidate()
        self.status.setText('Calculando trayectoria de vista previa…')
        self.prepare_button.setEnabled(False)
        options = dict(envelope=self.envelope, motion_config=self.config)
        if mode=='xyz': options['xyz_m']=[f.value()/1000 for f in self.xyz]
        else: options['target_deg']=self.goal()
        # Copia para que editar la velocidad no cambie un calculo en curso.
        import copy
        self.future_version = self.version
        self.future = self.pool.submit(prepare_target,self.model,list(self.initial),**copy.deepcopy(options))

    def animate(self):
        if self.plan is None: return
        self.animation = time.monotonic()
        self.status.setText('Reproduciendo SOLO el modelo. El robot no recibe datos.')

    def stop_animation(self):
        self.animation = None
        self.status.setText('Animacion detenida. No es una orden de parada para el ABB.')

    def tick(self):
        if self.reader is not None:
            _,q,stamp,error,*_=self.reader.snapshot()
            fresh=q is not None and not error and time.monotonic()-stamp<1
            self.copy_real.setEnabled(fresh)
            self.real_status.setText(('Lectura real: '+'  '.join(f'J{i+1}={v:.2f}°' for i,v in enumerate(q))) if fresh else 'Sin lectura real reciente: '+error)
        if self.future is not None and self.future.done():
            future,self.future=self.future,None
            self.prepare_button.setEnabled(True)
            try:
                plan=future.result()
                if self.future_version!=self.version:
                    self.status.setText('Cambiaste el objetivo o los limites: prepara otra vez.')
                else:
                    if plan['mode']=='xyz_hold_orientation':
                        self.set_goal(plan['target_deg'])
                    self.plan=plan
                    self.play.setEnabled(True)
                    peaks='  '.join(f'J{i+1} {v:.2f}°/s' for i,v in enumerate(plan['peak_speed_deg_s']))
                    self.status.setText(f"Trayectoria preparada · {plan['duration_s']:.2f} s\nPicos calculados: {peaks}")
            except Exception as exc: self.status.setText('Trayectoria rechazada: '+str(exc))
        if self.animation is not None and self.plan is not None:
            elapsed=time.monotonic()-self.animation
            u=min(1.,elapsed/self.plan['duration_s'])
            blend=10*u**3-15*u**4+6*u**5
            self.rendered=(np.array(self.initial)+blend*(np.array(self.plan['target_deg'])-self.initial)).tolist()
            if u==1:
                self.animation=None
                self.status.setText('Vista previa terminada. No se ejecuto movimiento fisico.')
        self.preview.publish(self.rendered)

    def closeEvent(self,event):
        self.timer.stop()
        if self.reader is not None:
            self.reader.stop.set()
            self.reader.join(timeout=3)
        self.pool.shutdown(wait=False,cancel_futures=True)
        self.preview.close()
        event.accept()


def main():
    app=QtWidgets.QApplication(sys.argv)
    preview=Preview()
    try: window=Panel(preview)
    except Exception:
        preview.close()
        raise
    signal.signal(signal.SIGINT,lambda *_:window.close())
    window.show()
    return app.exec_()


if __name__=='__main__': sys.exit(main())
