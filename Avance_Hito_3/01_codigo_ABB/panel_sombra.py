#!/usr/bin/env python3
"""Naranja es la lectura real y blanco el destino que se puede enviar al ABB."""
import math
from copy import deepcopy
from pathlib import Path
import signal
import subprocess
import sys
from threading import Event
import time
import yaml
import numpy as np
import rclpy
from PyQt5 import QtCore, QtWidgets
from sensor_msgs.msg import JointState
from alternativas_irb120 import branches
from panel_didactico import Panel, Preview
from escena_doble import robot_xml, rviz_config, SUPPORT_TOPIC, support_marker
from ejecutar_rws import check_goal, execute_goal
from limites_movimiento import effective
from objetivos_preview import transform
from panel_verificacion import RvizPane, minimum_jacobian_singular_value, orientation_error

ROOT=Path(__file__).resolve().parent


class DualPreview(Preview):
    def __init__(self):
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from visualization_msgs.msg import MarkerArray
        self.processes,self.logs=[],[]
        rclpy.init(args=[])
        self.node=rclpy.create_node('abb_real_y_destino')
        self.pub=self.node.create_publisher(JointState,'/goal/joint_states',1)
        self.real_pub=self.node.create_publisher(JointState,'/real/joint_states',1)
        self.static=StaticTransformBroadcaster(self.node)
        transforms=[]
        for prefix in ('real','goal'):
            t=TransformStamped()
            t.header.stamp=self.node.get_clock().now().to_msg()
            t.header.frame_id='world'
            t.child_frame_id=prefix+'/base_link'
            t.transform.rotation.w=1.
            transforms.append(t)
        self.static.sendTransform(transforms)
        support_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.support_pub = self.node.create_publisher(MarkerArray, SUPPORT_TOPIC, support_qos)
        self.support_pub.publish(support_marker(ROOT/'meshes/barandas.stl'))
        self.cache=Path.home()/'.cache/imt342_real_y_destino'
        self.cache.mkdir(parents=True,exist_ok=True)
        source=(ROOT/'urdf/irb120.urdf').read_text().replace('package://abb_irb120_support/',ROOT.as_uri()+'/')
        self.params={}
        for prefix,white in (('real',False),('goal',True)):
            p=self.cache/(prefix+'.yaml')
            p.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
                'robot_description':robot_xml(source,white=white),'frame_prefix':prefix+'/'}}}))
            self.params[prefix]=p
        self.real_started=False
        config=rviz_config(yaml.safe_load((ROOT/'irb120.rviz').read_text()))
        rviz=self.cache/'real_y_destino.rviz'
        rviz.write_text(yaml.safe_dump(config,allow_unicode=True))
        try:
            self.start_rsp('goal')
            self.rviz = self.start(['/opt/ros/humble/lib/rviz2/rviz2','-d',str(rviz)],'rviz.log')
        except Exception:
            self.close()
            raise

    def start(self,cmd,name):
        log=(self.cache/name).open('w')
        self.logs.append(log)
        process = subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
        self.processes.append(process)
        return process

    def start_rsp(self,prefix):
        self.start(['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher',
                    '--ros-args','--params-file',str(self.params[prefix]),
                    '-r','__node:='+prefix+'_state_publisher',
                    '-r','joint_states:=/'+prefix+'/joint_states',
                    '-r','robot_description:=/'+prefix+'/robot_description'],prefix+'.log')

    def publish_real(self,q):
        if not self.real_started:
            self.start_rsp('real')
            self.real_started=True
        msg=JointState()
        msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.name=[f'joint_{i}' for i in range(1,7)]
        msg.position=[math.radians(v) for v in q]
        self.real_pub.publish(msg)

    def publish(self, q):
        super().publish(q)
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0)


class GoalPanel(Panel):
    motion_message = QtCore.pyqtSignal(str)

    def __init__(self,preview):
        self.real_seen=False
        self.motion=None
        self.motion_cancel=Event()
        super().__init__(preview)
        self.xyz_dirty = False
        self.pending_xyz = None
        self.commit_xyz_request = None
        self.reviewing = False
        self.ik_reference = None
        self.metrics_at = 0.
        self.jacobian_cache = None
        self.alternatives_key = None
        self.xyz_timer = QtCore.QTimer(self)
        self.xyz_timer.setSingleShot(True)
        self.xyz_timer.setInterval(350)
        self.xyz_timer.timeout.connect(lambda: self.prepare('xyz'))
        self.setWindowTitle('IMT 342 · ABB IRB 120')
        for row in self.config['axes']: row['axis_speed_percent']=100.
        self.motion_tab=self.tabs.widget(2)
        self.tabs.removeTab(2)
        self.motion_tab.hide()
        self.prepare_button.hide()
        self.play.hide()
        for button in self.findChildren(QtWidgets.QPushButton):
            if 'Detener vista previa' in button.text(): button.hide()
        for field in self.xyz:
            field.valueChanged.connect(self.xyz_edited)
            field.lineEdit().returnPressed.connect(self.commit_xyz)
            field.setToolTip('Escribe XYZ y pulsa Enter para revisar el envío al ABB.')
        for field in self.angles:
            field.valueChanged.connect(self.direct_edited)
        self.copy_real.setText('Poner sombra sobre el brazo real')
        for label in self.findChildren(QtWidgets.QLabel):
            text=label.text()
            if text.startswith('<h2>'): label.setText('<h2>ABB IRB 120</h2>')
            elif text.startswith('VISTA PREVIA'): label.setText('Naranja · ABB real    Blanco · destino')
            elif text.startswith(('1 ·', '2 ·', 'Altura:', 'Porcentaje del rango:')):
                label.hide()
        self.real_status.setText('ABB desconectado')
        self.status.clear()
        self.execute.hide()
        self.motion_message.connect(self.status.setText)
        self.integrate_layout()
        from objetivo_rviz import TipTarget
        xyz = self.model.tip(self.rendered, self.envelope)[:3, 3]
        self.tip_target = TipTarget(self.preview.node, self.pick_target, xyz)

    def integrate_layout(self):
        content = QtWidgets.QWidget()
        column = self.layout()
        content.setLayout(column)
        original = self.tabs
        sections = QtWidgets.QTabWidget()
        sections.setDocumentMode(True)
        self.preview_xyz.hide()
        control = QtWidgets.QWidget()
        controls = QtWidgets.QVBoxLayout(control)
        controls.setSpacing(8)
        connection = column.itemAt(column.indexOf(self.real_status) - 1).layout()
        column.removeItem(connection)
        column.removeWidget(self.real_status)
        controls.addLayout(connection)
        controls.addWidget(self.real_status)
        joints = QtWidgets.QGridLayout()
        joints.setVerticalSpacing(4)
        joints.setColumnStretch(1, 1)
        for index, (slider, angle, percent) in enumerate(zip(self.sliders, self.angles, self.percentages)):
            axis = QtWidgets.QLabel(f'J{index+1}')
            axis.setStyleSheet('font-weight:bold;')
            angle.setToolTip(f'Rango habilitado: {angle.minimum():g}° a {angle.maximum():g}°')
            percent.setToolTip('Posición dentro del rango del fabricante')
            percent.setMinimumWidth(44)
            percent.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            for col, widget in enumerate((axis, slider, angle, percent)):
                joints.addWidget(widget, index, col)
        controls.addLayout(joints)
        title = QtWidgets.QLabel('Destino de la brida (mm)')
        title.setStyleSheet('font-weight:bold; margin-top:6px;')
        controls.addWidget(title)
        xyz_row = QtWidgets.QHBoxLayout()
        for axis, field in zip('XYZ', self.xyz):
            field.setSuffix('')
            xyz_row.addWidget(QtWidgets.QLabel(axis))
            xyz_row.addWidget(field, 1)
        controls.addLayout(xyz_row)
        controls.addStretch()
        sections.addTab(control, 'Control')

        metrics = QtWidgets.QWidget()
        metrics_layout = QtWidgets.QVBoxLayout(metrics)
        self.joint_table = self.measurement_table(
            metrics_layout, 'Articulaciones · Δ = destino − medida',
            [f'J{i}' for i in range(1, 7)], '°')
        self.position_table = self.measurement_table(
            metrics_layout, 'Brida · Δ = destino FK − posición ABB', 'XYZ', 'mm')
        self.position_error = QtWidgets.QLabel()
        self.position_error.setWordWrap(True)
        metrics_layout.addWidget(self.position_error)
        metrics_layout.addWidget(QtWidgets.QLabel('Cinemática y límites'))
        self.math_summary = QtWidgets.QLabel()
        self.math_summary.setWordWrap(True)
        metrics_layout.addWidget(self.math_summary)
        alternatives_title = QtWidgets.QLabel('Soluciones para la misma pose')
        alternatives_title.setStyleSheet('font-weight:bold; margin-top:8px;')
        metrics_layout.addWidget(alternatives_title)
        self.alternatives_caption = QtWidgets.QLabel()
        self.alternatives_caption.setWordWrap(True)
        metrics_layout.addWidget(self.alternatives_caption)
        self.alternatives_table = QtWidgets.QTableWidget(0, 6)
        self.alternatives_table.setHorizontalHeaderLabels([f'J{i} (°)' for i in range(1, 7)])
        self.alternatives_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.alternatives_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.alternatives_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.alternatives_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.alternatives_table.currentCellChanged.connect(self.show_alternative)
        metrics_layout.addWidget(self.alternatives_table)
        self.alternative_detail = QtWidgets.QLabel()
        self.alternative_detail.setWordWrap(True)
        metrics_layout.addWidget(self.alternative_detail)
        self.sampled_orientations = QtWidgets.QLabel()
        self.sampled_orientations.setWordWrap(True)
        metrics_layout.addWidget(self.sampled_orientations)
        metrics_layout.addStretch()
        math_scroll = QtWidgets.QScrollArea(widgetResizable=True)
        math_scroll.setWidget(metrics)
        sections.addTab(math_scroll, 'Verificación')
        column.replaceWidget(original, sections)
        original.hide()
        original.deleteLater()
        self.tabs = sections

        left = QtWidgets.QScrollArea(widgetResizable=True)
        left.setWidget(content)
        left.setMinimumWidth(390)
        left.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        column.removeWidget(self.status)
        column.removeWidget(self.execute)
        sidebar = QtWidgets.QWidget()
        sidebar.setMinimumWidth(390)
        sidebar_layout = QtWidgets.QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(0, 0, 0, 5)
        sidebar_layout.addWidget(left, 1)
        sidebar_layout.addWidget(self.status)
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        split.addWidget(sidebar)
        self.rviz_pane = RvizPane(self.preview.rviz)
        split.addWidget(self.rviz_pane)
        split.setStretchFactor(1, 1)
        split.setSizes([470, 810])
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.addWidget(split)
        self.setStyleSheet('QWidget {font-size: 13px; color:#182634;} '
                           'QTabBar::tab {padding: 8px; background:#edf1f5; font-weight:bold;} '
                           'QTabBar::tab:selected {background:#d6e9ef; color:#0e5979;} '
                           'QPushButton {padding: 6px;} QGroupBox {margin-top:8px;}')
        self.execute.setStyleSheet('QPushButton {background:#0e5979;color:white;font-weight:bold;padding:9px;} '
                                   'QPushButton:disabled {background:#e2e9ee;color:#536371;}')
        self.resize(1240, 760)

    @staticmethod
    def measurement_table(layout, title, rows, unit):
        heading = QtWidgets.QLabel(title)
        heading.setStyleSheet('font-weight:bold; margin-top:4px;')
        layout.addWidget(heading)
        table = QtWidgets.QTableWidget(len(rows), 3)
        table.setHorizontalHeaderLabels((f'Medido ({unit})', f'Destino ({unit})', f'Δ ({unit})'))
        table.setVerticalHeaderLabels(list(rows))
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        table.verticalHeader().setDefaultSectionSize(23)
        table.setFixedHeight(table.horizontalHeader().sizeHint().height() + 23*len(rows) + 2)
        for row in range(len(rows)):
            for col in range(3):
                item = QtWidgets.QTableWidgetItem('—')
                item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                table.setItem(row, col, item)
        layout.addWidget(table)
        return table

    def xyz_edited(self):
        if not self.lock_ui:
            self.commit_xyz_request = None
            self.xyz_dirty = True
            self.pending_xyz = None
            self.xyz_timer.start()

    def direct_edited(self):
        if not self.lock_ui:
            self.commit_xyz_request = None
            self.xyz_timer.stop()
            self.xyz_dirty = False
            self.pending_xyz = None
            self.ik_reference = None
            self.update_xyz_fields(self.model.tip(self.rendered, self.envelope)[:3, 3])
            if self.goal() == self.rendered:
                self.status.clear()

    def update_xyz_fields(self, xyz):
        self.lock_ui = True
        try:
            for field, value in zip(self.xyz, xyz):
                field.setValue(value * 1000)
        finally:
            self.lock_ui = False

    def commit_xyz(self):
        if self.motion is not None or self.reviewing:
            return
        xyz = tuple(field.value()/1000 for field in self.xyz)
        self.xyz_timer.stop()
        self.commit_xyz_request = xyz
        self.pending_xyz = (xyz, False)

    def pick_target(self, xyz, final):
        if self.motion is not None or self.reviewing:
            return
        self.xyz_timer.stop()
        self.commit_xyz_request = tuple(xyz) if final else None
        self.xyz_dirty = True
        self.tabs.setCurrentIndex(0)
        if any(not field.minimum() <= value * 1000 <= field.maximum()
               for field, value in zip(self.xyz, xyz)) or xyz[2] < self.envelope.min_tip_z_m:
            self.pending_xyz = None
            self.invalidate()
            self.reject_marker('Punto fuera del rango XYZ o por debajo de la altura mínima.')
            return
        self.pending_xyz = (xyz, True)

    def reject_marker(self, reason):
        self.commit_xyz_request = None
        self.update_xyz_fields(self.model.tip(self.rendered, self.envelope)[:3, 3])
        self.status.setText(f'{reason} Se conserva la última postura válida.')

    def refresh_metrics(self, snapshot, fresh):
        q = self.rendered
        goal = self.model.tip(q, self.envelope)
        goal_xyz = goal[:3, 3]*1000
        measured_xyz = None
        position_error = 'XYZ del ABB pendiente de lectura.'
        key = tuple(q)
        if self.jacobian_cache is None or self.jacobian_cache[0] != key:
            sigma = minimum_jacobian_singular_value(self.model, self.envelope, q, goal)
            self.jacobian_cache = key, sigma
        values = [f'Altura de la brida · {goal_xyz[2]:.3f} mm (mín. {self.envelope.min_tip_z_m*1000:.3f} mm)',
                  f'Jacobiano escalado · σ mín. {self.jacobian_cache[1]:.6f} m/rad (escala angular 0,2 m)',
                  'Un σ mín. cercano a cero indica proximidad a una singularidad.']
        if self.ik_reference and tuple(self.ik_reference[0]) == key:
            residual = np.linalg.norm(goal[:3, 3]*1000 - self.ik_reference[1])
            values.append(f'Residual IK · ‖XYZ pedido − XYZ obtenido por FK‖ = {residual:.6f} mm')
        if fresh:
            _, measured, _, _, tcp, stamp, tcp_error, comparison = snapshot
            measured_pose = self.model.tip(measured, self.envelope)
            fk_distance = np.linalg.norm(goal_xyz - measured_pose[:3, 3]*1000)
            values.append(f'Distancia por FK · ‖FK(destino) − FK(medido)‖ = {fk_distance:.6f} mm')
            values.append(f'Diferencia de orientación por FK · {orientation_error(goal[:3, :3], measured_pose[:3, :3]):.6f}°')
            if tcp is not None and not tcp_error and comparison and time.monotonic()-stamp < 1:
                paired_q, paired_tcp, *_ = comparison
                delta_fk = self.model.tip(paired_q, self.envelope)[:3, 3]*1000 - paired_tcp
                measured_xyz = paired_tcp
                position_error = (
                    f'Distancia a la brida real · ‖Δ XYZ‖ = {np.linalg.norm(goal_xyz - paired_tcp):.6f} mm\n'
                    f'Error del modelo · ‖FK(ángulos medidos) − XYZ ABB‖ = {np.linalg.norm(delta_fk):.6f} mm')
            try:
                check_goal(self.model, self.envelope, self.config, measured, q)
                values.append('Comprobación previa · dentro de los límites del modelo')
            except ValueError as exc:
                values.append(str(exc))
        else:
            measured = [None]*6
        for table, readings, targets in (
            (self.joint_table, measured, q),
            (self.position_table, [None]*3 if measured_xyz is None else measured_xyz, goal_xyz),
        ):
            for index, (actual, target) in enumerate(zip(readings, targets)):
                row = ('—' if actual is None else f'{actual:.6f}', f'{target:.6f}',
                       '—' if actual is None else f'{target-actual:+.6f}')
                for column, value in enumerate(row):
                    table.item(index, column).setText(value)
        self.position_error.setText(position_error)
        self.math_summary.setText('\n'.join(values))
        if self.alternatives_table.isVisible():
            self.refresh_alternatives(q, goal)

    def refresh_alternatives(self, q, goal):
        xyz = goal[:3, 3]
        if self.ik_reference and self.ik_reference[0] == q:
            xyz = self.ik_reference[1]/1000
        bounds = [(limit['min_deg'], limit['max_deg']) for limit in effective(self.config)]
        key = (tuple(q), tuple(xyz), tuple(bounds))
        if key == self.alternatives_key:
            return
        self.alternatives_key = key
        rotation = goal[:3, :3]
        self.alternative_context = (xyz, rotation, q, bounds)
        self.alternatives, singular = branches(self.model, self.envelope, xyz, rotation, q, bounds)
        table = self.alternatives_table
        blocker = QtCore.QSignalBlocker(table)
        table.clearContents()
        table.setRowCount(len(self.alternatives))
        for row, angles in enumerate(self.alternatives):
            for column, value in enumerate(angles):
                item = QtWidgets.QTableWidgetItem(f'{value:.3f}')
                item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                table.setItem(row, column, item)
        del blocker
        table.setVisible(bool(self.alternatives))
        radial = math.hypot(rotation[0, 0], rotation[1, 0])
        roll = math.atan2(rotation[2, 1], rotation[2, 2]) if radial > 1e-8 else math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], radial)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0]) if radial > 1e-8 else 0.
        position = ' / '.join(f'{value*1000:.3f}' for value in xyz)
        rpy = ' / '.join(f'{math.degrees(value):.3f}°' for value in (roll, pitch, yaw))
        caption = (f'XYZ {position} mm  ·  Roll / pitch / yaw {rpy}\n'
                   f'{len(self.alternatives)} configuraciones verificadas mediante FK, límites y altura. '
                   'Ordenadas por cercanía angular a la sombra.')
        if singular:
            caption += '\nHay una familia singular. Se muestran configuraciones representativas.'
        self.alternatives_caption.setText(caption)
        if self.alternatives:
            table.selectRow(0)
            self.show_alternative(0)
        else:
            self.alternative_detail.clear()
        samples = ['<b>Mismo punto con otras orientaciones</b>',
                   'Configuraciones válidas al girar ±30° alrededor de cada eje local de la brida.']
        for index, axis in enumerate('XYZ'):
            counts = []
            for turn in (-30, 30):
                rpy = [0., 0., 0.]
                rpy[index] = math.radians(turn)
                matrix = rotation @ transform(rpy=rpy)[:3, :3]
                configs, sampled = branches(self.model, self.envelope, xyz, matrix, q, bounds)
                counts.append(f'{turn:+d}° · {len(configs)}' + ('*' if sampled else ''))
            samples.append(f'Eje local {axis} &nbsp; ' + ' &nbsp; | &nbsp; '.join(counts))
        if any('*' in line for line in samples):
            samples.append('* Familia singular con configuraciones representativas.')
        self.sampled_orientations.setText('<br>'.join(samples))

    def show_alternative(self, row, *_):
        if row < 0:
            return
        xyz, rotation, current, bounds = self.alternative_context
        q = self.alternatives[row]
        pose = self.model.tip(q, self.envelope)
        position_error = np.linalg.norm(pose[:3, 3] - xyz)*1000
        angle_error = orientation_error(pose[:3, :3], rotation)
        sigma = minimum_jacobian_singular_value(self.model, self.envelope, q, pose)
        margin, limiting_axis = min((min(v-low, high-v), i)
                                    for i, (v, (low, high)) in enumerate(zip(q, bounds), 1))
        delta = max(abs(a-b) for a, b in zip(q, current))
        self.alternative_detail.setText(
            f'Solución {row+1} · Residual XYZ {position_error:.6f} mm · Orientación {angle_error:.6f}°\n'
            f'σ mín. {sigma:.6f} m/rad · Margen mínimo {margin:.3f}° en J{limiting_axis}\n'
            f'Altura {pose[2, 3]*1000:.3f} mm · Mayor cambio desde la sombra {delta:.3f}°')

    def execute_selected_goal(self):
        if (self.motion is not None or self.future is not None or self.reader is None
                or self.tip_target.dragging or self.pending_xyz is not None):
            self.status.setText('Vista previa lista. Conecta el ABB y arranca ServidorDirecto en manual reducido.')
            return
        if self.xyz_dirty:
            self.status.setText('Elige un destino válido antes de ejecutar.')
            return
        _, q, stamp, error, *_ = self.reader.snapshot()
        if q is None or error or time.monotonic()-stamp > 1:
            self.status.setText('Espera una lectura reciente del brazo naranja.')
            return
        target = self.goal()
        if target != self.rendered:
            self.status.setText('El destino introducido no coincide con la sombra visible.')
            return
        try:
            check_goal(self.model, self.envelope, self.config, q, target)
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        axes = '   '.join(f'J{i} {a:.3f}° → {b:.3f}°' for i, (a, b) in enumerate(zip(q, target), 1))
        self.reviewing = True
        try:
            confirm = QtWidgets.QMessageBox.question(
                self, 'Ejecutar destino blanco',
                'El ABB ejecutará el destino al recibirlo. Confirma que el recorrido está despejado.\n\n' + axes,
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        finally:
            self.reviewing = False
        if confirm != QtWidgets.QMessageBox.Yes:
            return
        self.motion_cancel.clear()
        self.execute.setEnabled(False)
        self.tabs.setEnabled(False)
        self.status.setText('Verificando estado, límites y altura del recorrido…')
        self.motion=self.pool.submit(execute_goal, self.reader.base, self.reader.credentials,
                                    self.model, self.envelope, deepcopy(self.config), list(q), target,
                                    self.motion_message.emit, self.motion_cancel)

    def use_real(self):
        if self.reader is None or self.motion is not None:return
        _,q,stamp,error,*_=self.reader.snapshot()
        if q is None or error or time.monotonic()-stamp>1:return
        try:
            if any(not r['min_deg']<=v<=r['max_deg'] for v,r in zip(q,effective(self.config))):
                raise ValueError('Postura real fuera del rango permitido para el destino')
            self.model.check_height(q,self.envelope)
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        self.initial=list(q)
        self.commit_xyz_request = None
        self.xyz_timer.stop()
        self.set_goal(q)
        self.pending_xyz = None
        self.ik_reference = None
        self.xyz_dirty = False
        self.status.clear()

    def prepare(self,mode):
        if mode == 'xyz':
            self.pending_xyz = (tuple(field.value()/1000 for field in self.xyz), False)

    def start_ik(self, xyz, from_marker):
        from objetivos_preview import solve_xyz
        self.xyz_dirty = True
        self.invalidate()
        self.future_version = self.version
        self.status.clear()
        self.ik_from_marker = from_marker
        reference = list(self.rendered)
        self.requested_xyz = np.asarray(xyz)*1000
        self.future=self.pool.submit(solve_xyz,self.model,reference,xyz,self.envelope)

    def tick(self):
        committed_goal = None
        fresh=False
        snapshot=None
        if self.reader is not None:
            snapshot=self.reader.snapshot()
            _,q,stamp,error,*_=snapshot
            fresh=q is not None and not error and time.monotonic()-stamp<1
            self.copy_real.setEnabled(fresh and self.motion is None)
            if fresh:
                self.preview.publish_real(q)
                self.real_seen=True
                self.real_status.setText('ABB en vivo')
            else:
                self.real_status.setText(('NARANJA CONGELADO: ultima lectura, no posicion actual. ' if self.real_seen else 'SIN LECTURA REAL: naranja no disponible. ')+error)
        if self.future is not None and self.future.done():
            future,self.future=self.future,None
            try:
                target = [round(angle, field.decimals()) for angle, field in zip(future.result(), self.angles)]
                if self.future_version == self.version:
                    bounds=effective(self.config)
                    if any(not r['min_deg']<=v<=r['max_deg'] for v,r in zip(target,bounds)):
                        raise ValueError('XYZ requiere angulos fuera de los limites configurados')
                    self.model.check_height(target,self.envelope)
                    self.set_goal(target)
                    if not self.ik_from_marker:
                        self.update_xyz_fields(self.requested_xyz/1000)
                    self.ik_reference = (self.goal(), self.requested_xyz)
                    self.xyz_dirty = self.pending_xyz is not None or self.tip_target.dragging
                    self.status.clear()
                    if (self.commit_xyz_request is not None and not self.xyz_dirty
                            and np.allclose(self.commit_xyz_request, self.requested_xyz/1000,
                                            rtol=0., atol=1e-9)):
                        committed_goal = tuple(self.goal())
                        self.commit_xyz_request = None
            except Exception as exc:
                if self.future_version == self.version and self.pending_xyz is None:
                    self.commit_xyz_request = None
                    if self.ik_from_marker:
                        self.reject_marker(str(exc))
                    else:
                        self.status.setText('Destino rechazado: '+str(exc))
        if self.motion is not None and self.motion.done():
            motion,self.motion=self.motion,None
            self.tabs.setEnabled(True)
            try:
                result=motion.result()
                label = (f"Trayectoria completada · {result['points']} puntos" if result.get('points', 1) > 1
                         else 'Llegada confirmada')
                self.status.setText(f"{label} · orden {result['seq']} · error máximo {result['max_error_deg']:.6f}°")
            except Exception as exc:
                self.status.setText('Envío detenido. Revisa el estado del ABB antes de repetir. '+str(exc))
        if self.pending_xyz is not None and self.future is None and self.motion is None and not self.reviewing:
            (xyz, from_marker), self.pending_xyz = self.pending_xyz, None
            self.start_ik(xyz, from_marker)
        if not self.tip_target.dragging and self.future is None and self.pending_xyz is None:
            pose = self.model.tip(self.rendered, self.envelope)
            self.tip_target.set_tip(pose[:3, 3], pose[:3, :3])
        if not getattr(self, 'central_execute_state', False):
            self.execute.setEnabled(
                fresh and self.motion is None and self.future is None
                and self.pending_xyz is None and not self.tip_target.dragging
                and not self.xyz_dirty and self.goal() == self.rendered)
        if time.monotonic() >= self.metrics_at:
            self.refresh_metrics(snapshot, fresh)
            self.metrics_at = time.monotonic()+.25
        self.preview.publish(self.rendered)
        if (getattr(self, 'send_on_edit', True) and committed_goal is not None
                and committed_goal == tuple(self.goal())):
            self.execute_selected_goal()

    def closeEvent(self, event):
        if self.motion is not None and not self.motion.done():
            self.motion_cancel.set()
            self.status.setText('Hay una orden pendiente. Usa el FlexPendant si necesitas detener el robot.')
            event.ignore()
            return
        self.tip_target.close()
        super().closeEvent(event)


def main():
    app=QtWidgets.QApplication(sys.argv)
    preview=DualPreview()
    try:window=GoalPanel(preview)
    except Exception:
        preview.close()
        raise
    signal.signal(signal.SIGINT,lambda *_:window.close())
    window.show()
    return app.exec_()


if __name__=='__main__':sys.exit(main())
