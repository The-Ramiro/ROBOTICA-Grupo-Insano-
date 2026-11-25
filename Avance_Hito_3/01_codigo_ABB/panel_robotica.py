#!/usr/bin/env python3
"""Panel con ejecución explícita, pose 6D y análisis del modelo del proyecto."""
from concurrent.futures import CancelledError, ThreadPoolExecutor
from threading import Event
import signal
import sys
import time
from pathlib import Path
from datetime import datetime

import numpy as np
from PyQt5 import QtCore, QtWidgets

from calculos_robotica import (
    analyze, geometric_jacobian, quaternion_rotation, rotation_rpy, rpy_degrees, solve_pose,
)
from graficas_robotica import SeriesPlot
from limites_movimiento import effective
from panel_automatico import AutomaticPanel
from panel_poses import PosePage
from panel_sombra import DualPreview
from panel_verificacion import orientation_error
from velocidad_abb import SpeedController
from trayectorias_panel import RoutePanel
from registro_robotica import Recorder
from ventana_analisis import AnalysisWindow


class RoboticsPanel(RoutePanel, AutomaticPanel):
    def __init__(self, preview):
        self.central_execute_state = True
        self.rpy = []
        self.pending_rotation = None
        self.analysis_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='robotica-calculos')
        self.analysis_future = None
        self.analysis_result = None
        self.analysis_key = None
        self.analysis_at = 0.
        self.plan_source = None
        self.analysis_anchor = None
        self.analysis_cancel = Event()
        self.last_target_edit = time.monotonic()
        self.speed_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='abb-velocidad')
        self.speed_client = None
        self.speed_connection = None
        self.speed_future = None
        self.speed_operation = None
        self.speed_dirty = False
        self.speed_actual = None
        self.speed_read_at = 0.
        self.speed_poll_at = 0.
        super().__init__(preview, send_on_edit=False)
        self.setWindowTitle('IMT 342 · ABB IRB 120 · Robótica')
        controls = self.tabs.widget(0).layout()
        label = QtWidgets.QLabel('Orientación de la brida (°)')
        label.setStyleSheet('font-weight:bold; margin-top:4px;')
        controls.insertWidget(controls.count()-1, label)
        row = QtWidgets.QHBoxLayout()
        rotation = self.model.tip(self.rendered, self.envelope)[:3, :3]
        for name, value in zip(('Roll', 'Pitch', 'Yaw'), rpy_degrees(rotation)):
            field = QtWidgets.QDoubleSpinBox()
            field.setRange(-180., 180.)
            field.setDecimals(3)
            field.setValue(float(value))
            field.setKeyboardTracking(False)
            field.setToolTip('R = Rz(yaw) Ry(pitch) Rx(roll). Ejes de la base.')
            field.valueChanged.connect(self.orientation_edited)
            field.lineEdit().returnPressed.connect(self.commit_xyz)
            row.addWidget(QtWidgets.QLabel(name))
            row.addWidget(field, 1)
            self.rpy.append(field)
        controls.insertLayout(controls.count()-1, row)
        self.create_real_speed(controls)
        for field in self.xyz:
            field.setKeyboardTracking(False)
            field.setToolTip('XYZ en milímetros. Enter actualiza la vista previa de IK.')
        self.tip_target.enable_rotation(self.pose_target, rotation)
        self.create_manipulator(controls)
        verification = self.tabs.widget(1).widget().layout()
        self.orientation_info = QtWidgets.QLabel()
        self.orientation_info.setWordWrap(True)
        verification.insertWidget(verification.count()-1, self.orientation_info)
        self.orientation_residual = QtWidgets.QLabel()
        self.orientation_residual.setWordWrap(True)
        verification.insertWidget(verification.count()-1, self.orientation_residual)
        self.status.parentWidget().layout().addWidget(self.execute)
        self.execute.show()
        self.execute.setText('Ejecutar destino')
        self.execute.setToolTip('Envía el destino validado al ABB con la sesión automática habilitada.')
        self.execute.clicked.connect(self.execute_selected_goal)
        self.create_route_editor()
        controls.removeWidget(self.real_speed_group)
        bottom = self.status.parentWidget().layout()
        bottom.insertWidget(bottom.indexOf(self.execute), self.real_speed_group)
        self.create_kinematics()
        self.create_trajectory()
        self.create_dynamics()
        self.recorder = Recorder()
        self.math_window = AnalysisWindow(self)
        # Los widgets de cálculo conservan sus datos y conexiones, fuera del control.
        while self.tabs.count() > 2:
            widget, title = self.tabs.widget(2), self.tabs.tabText(2)
            self.tabs.removeTab(2)
            self.math_window.details.addTab(widget, title)
        self.pose_page = PosePage(self)
        self.tabs.addTab(self.pose_page, 'Poses')
        self.tabs.currentChanged.connect(self.refresh_pose_tab)
        self.analysis_menu = QtWidgets.QToolButton()
        self.analysis_menu.setText('Análisis')
        self.analysis_menu.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        menu = QtWidgets.QMenu(self.analysis_menu)
        for title, index in (('Trayectoria',0),('Singularidades',1),('Seguimiento',3),('Errores',4)):
            menu.addAction(title, lambda checked=False, page=index: self.show_analysis(page))
        self.analysis_menu.setMenu(menu)
        self.connection_controls.addWidget(self.analysis_menu)
        self.tabs.setUsesScrollButtons(True)
        self.setStyleSheet(self.styleSheet() + ' QTabBar::tab {padding:7px 5px;font-size:12px;}')

    def refresh_pose_tab(self):
        if self.tabs.currentWidget() is self.pose_page:
            self.pose_page.refresh()
            self.route_tab_changed()

    def show_analysis(self, page=0):
        self.math_window.tabs.setCurrentIndex(page)
        self.math_window.show()
        self.math_window.raise_()
        self.math_window.activateWindow()
        if page == 2:
            self.math_window.show_measured()
        elif page in (0,3):
            self.math_window.prepare_preview()
            self.math_window.tracking.refresh()
            self.math_window.trajectory.refresh()
        elif page==1 and self.math_window.result is None:
            self.math_window.calculate()
        elif page==4:
            self.math_window.errors.poll()

    def create_real_speed(self, controls):
        group = QtWidgets.QGroupBox('Velocidad del brazo real')
        self.real_speed_group = group
        layout = QtWidgets.QVBoxLayout(group)
        row = QtWidgets.QHBoxLayout()
        self.real_speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.real_speed_slider.setRange(0, 100)
        self.real_speed_value = QtWidgets.QSpinBox()
        self.real_speed_value.setRange(0, 100)
        self.real_speed_value.setSuffix(' %')
        self.real_speed_slider.valueChanged.connect(self.real_speed_value.setValue)
        self.real_speed_value.valueChanged.connect(self.real_speed_edited)
        self.apply_real_speed = QtWidgets.QPushButton('Aplicar')
        self.apply_real_speed.setToolTip('Aplica el porcentaje global entre movimientos, en modo AUTO.')
        self.apply_real_speed.clicked.connect(self.set_real_speed)
        self.apply_real_speed.setEnabled(False)
        row.addWidget(self.real_speed_slider, 1)
        row.addWidget(self.real_speed_value)
        row.addWidget(self.apply_real_speed)
        layout.addLayout(row)
        self.real_speed_label = QtWidgets.QLabel('Porcentaje ABB pendiente de conexión')
        self.real_speed_label.setWordWrap(True)
        layout.addWidget(self.real_speed_label)
        note = QtWidgets.QLabel(
            '100% = vmax de ABB para el movimiento. Cada conexión inicia al 0%; '
            'elige el porcentaje y pulsa Aplicar. La velocidad efectiva depende del recorrido.')
        note.setWordWrap(True)
        note.setStyleSheet('font-size:11px;')
        layout.addWidget(note)
        controls.insertWidget(controls.count()-1, group)

    def create_manipulator(self, controls):
        row = QtWidgets.QHBoxLayout()
        self.manipulator_mode = QtWidgets.QComboBox()
        for text, value in (('Mover', 'move'), ('Girar', 'rotate'), ('Ambos', 'both')):
            self.manipulator_mode.addItem(text, value)
        self.move_step = QtWidgets.QDoubleSpinBox()
        self.move_step.setRange(.1, 20.)
        self.move_step.setDecimals(1)
        self.move_step.setSuffix(' mm')
        self.move_step.setValue(.1)
        self.rotation_step = QtWidgets.QDoubleSpinBox()
        self.rotation_step.setRange(.1, 15.)
        self.rotation_step.setDecimals(1)
        self.rotation_step.setSuffix(' °')
        self.rotation_step.setValue(.1)
        row.addWidget(self.manipulator_mode)
        row.addWidget(QtWidgets.QLabel('Paso'))
        row.addWidget(self.move_step)
        row.addWidget(self.rotation_step)
        controls.insertLayout(controls.count()-2, row)
        for widget in (self.manipulator_mode, self.move_step, self.rotation_step):
            widget.setToolTip('Clic derecho en la esfera para avanzar por pasos precisos.')
        self.manipulator_mode.currentIndexChanged.connect(self.configure_manipulator)
        self.move_step.valueChanged.connect(self.configure_manipulator)
        self.rotation_step.valueChanged.connect(self.configure_manipulator)
        self.configure_manipulator()

    def configure_manipulator(self):
        self.tip_target.configure(
            self.manipulator_mode.currentData(), self.move_step.value(), self.rotation_step.value())
        for field in self.xyz:
            field.setSingleStep(self.move_step.value())
        for field in self.rpy:
            field.setSingleStep(self.rotation_step.value())

    def target_edited(self):
        self.last_target_edit = time.monotonic()
        self.analysis_cancel.set()

    def direct_edited(self):
        if not self.lock_ui:
            self.target_edited()
        super().direct_edited()

    def real_speed_edited(self, value):
        blocker = QtCore.QSignalBlocker(self.real_speed_slider)
        self.real_speed_slider.setValue(value)
        del blocker
        self.speed_dirty = True
        if hasattr(self,'math_window'):
            self.route_retime_preview(value)
            self.math_window.request_preview()

    def connect_reader(self):
        if self.speed_future is not None and self.speed_operation == 'apply':
            self.status.setText('Espera la confirmación de velocidad antes de cambiar la conexión.')
            return
        self.route_stop_preview()
        super().connect_reader()

    def start_automatic(self):
        if self.speed_future is not None and self.speed_operation == 'apply':
            self.status.setText('Espera la confirmación de velocidad antes de iniciar la sesión.')
            return
        super().start_automatic()

    def set_real_speed(self):
        if self.speed_client is None or self.speed_future is not None or self.motion is not None:
            return
        if self.auto_task is not None and self.auto_operation in ('start', 'connect', 'stop'):
            return
        self.speed_operation = 'apply'
        self.speed_future = self.speed_pool.submit(self.speed_client.apply, self.real_speed_value.value())
        self.apply_real_speed.setEnabled(False)
        self.status.setText('Aplicando el porcentaje global en el ABB…')

    def tick_speed(self):
        if self.speed_future is not None and self.speed_future.done():
            future, operation = self.speed_future, self.speed_operation
            self.speed_future = None
            self.speed_poll_at = time.monotonic()+2.
            try:
                result = future.result()
                self.speed_actual = result
                self.speed_read_at = time.monotonic()
                self.real_speed_label.setText(f'ABB confirmado · {result["ratio"]:g}% · {result["mode"]}')
                if operation == 'apply':
                    self.speed_dirty = False
                    self.status.setText(f'Velocidad global confirmada por ABB · {result["ratio"]:g}%')
                if not self.speed_dirty:
                    changed = self.real_speed_value.value() != round(result['ratio'])
                    blocker = QtCore.QSignalBlocker(self.real_speed_value)
                    self.real_speed_value.setValue(round(result['ratio']))
                    del blocker
                    blocker = QtCore.QSignalBlocker(self.real_speed_slider)
                    self.real_speed_slider.setValue(round(result['ratio']))
                    del blocker
                    if changed:
                        self.route_retime_preview(round(result['ratio']))
                        self.math_window.request_preview()
            except Exception as exc:
                self.speed_actual = None
                self.real_speed_label.setText('Sin lectura confirmada de velocidad ABB')
                if operation == 'apply':
                    self.status.setText('Velocidad sin confirmar · '+str(exc))
        busy = (self.motion is not None or self.stop_requested or self.reviewing
                or (self.auto_task is not None and self.auto_operation in ('start', 'connect', 'stop')))
        if self.reader is not None and self.speed_future is None and not busy:
            connection = (self.reader.base, self.reader.credentials)
            if connection != self.speed_connection:
                changing_connection = self.speed_connection is not None
                if self.speed_client is not None:
                    self.speed_pool.submit(self.speed_client.close)
                self.speed_client = SpeedController(*connection)
                self.speed_connection = connection
                if changing_connection:
                    self.speed_dirty = False
                self.speed_actual = None
                self.speed_poll_at = 0.
            if time.monotonic() >= self.speed_poll_at:
                self.speed_operation = 'read'
                self.speed_future = self.speed_pool.submit(self.speed_client.read)
                self.speed_poll_at = time.monotonic()+2.
        applying = self.speed_future is not None and self.speed_operation == 'apply'
        self.real_speed_slider.setEnabled(not busy and not applying)
        self.real_speed_value.setEnabled(not busy and not applying)
        fresh = self.speed_actual is not None and time.monotonic()-self.speed_read_at < 5.
        self.apply_real_speed.setEnabled(
            fresh and self.speed_actual['mode'] == 'AUTO' and self.speed_dirty
            and self.speed_future is None and not busy)

    def page(self, title):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        layout.setSpacing(8)
        scroll = QtWidgets.QScrollArea(widgetResizable=True)
        scroll.setWidget(widget)
        self.tabs.addTab(scroll, title)
        return layout

    @staticmethod
    def label(layout, text=''):
        widget = QtWidgets.QLabel(text)
        widget.setWordWrap(True)
        layout.addWidget(widget)
        return widget

    @staticmethod
    def table(layout, rows, columns):
        widget = QtWidgets.QTableWidget(len(rows), len(columns))
        widget.setStyleSheet('QTableWidget {font-size:11px;} QHeaderView::section {font-size:11px;}')
        widget.setVerticalHeaderLabels(list(rows))
        widget.setHorizontalHeaderLabels(columns)
        widget.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        widget.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        widget.verticalHeader().setDefaultSectionSize(25)
        widget.setFixedHeight(widget.horizontalHeader().sizeHint().height()+len(rows)*25+4)
        for row in range(len(rows)):
            for col in range(len(columns)):
                widget.setItem(row, col, QtWidgets.QTableWidgetItem('—'))
        layout.addWidget(widget)
        return widget

    @staticmethod
    def fill(table, values, digits=5):
        for row, data in enumerate(values):
            for col, value in enumerate(data):
                item = table.item(row, col)
                item.setText(f'{value:.{digits}f}')
                item.setToolTip(f'{value:.9f}')
                item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

    def create_kinematics(self):
        layout = self.page('Cinemática')
        self.label(layout, 'FK del URDF · T(base→brida). Traslación en metros.')
        self.transform_table = self.table(layout, ('x', 'y', 'z', '1'), ('nx', 'oy', 'az', 'p'))
        self.label(layout, 'Jacobiano geométrico · [v; ω] = J(q) q̇. q̇ en rad/s.')
        self.jac_table = self.table(layout, ('vx', 'vy', 'vz', 'ωx', 'ωy', 'ωz'),
                                    [f'J{i}' for i in range(1, 7)])
        self.kinematics_info = self.label(layout)
        self.label(layout, 'SVD del Jacobiano con filas angulares escaladas por 0,2 m. '
                   'κ = σmáx/σmín. Cerca de singularidad, pequeños cambios de pose pueden exigir grandes cambios articulares. '
                   'El pitch ±90° produce ambigüedad en roll/yaw por la representación Euler.')
        layout.addStretch()

    def create_trajectory(self):
        layout = self.page('LSPB')
        form = QtWidgets.QFormLayout()
        settings = (
            ('Duración mínima (s)', 0.2, 120., 4.),
            ('Mezcla tb/T (%)', 5., 50., 20.),
            ('Límite de análisis (% ABB)', .1, 100., 10.),
            ('Aceleración de diseño (°/s²)', .1, 500., 30.),
        )
        self.plan_fields = []
        for title, low, high, value in settings:
            field = QtWidgets.QDoubleSpinBox()
            field.setRange(low, high)
            field.setDecimals(2)
            field.setValue(value)
            field.setKeyboardTracking(False)
            field.valueChanged.connect(self.plan_edited)
            form.addRow(title, field)
            self.plan_fields.append(field)
        layout.addLayout(form)
        self.lspb_info = self.label(layout, 'Calculando perfil…')
        self.label(layout, 'V = Δq/(T−tb), a = V/tb. Se amplía T para respetar los límites elegidos. '
                   'ABB ejecuta MoveAbsJ con su perfil interno y la velocidad configurada en RAPID.')
        self.position_plot = SeriesPlot('Posición LSPB', '°')
        self.velocity_plot = SeriesPlot('Velocidad LSPB', '°/s')
        self.acceleration_plot = SeriesPlot('Aceleración LSPB', '°/s²')
        for plot in (self.position_plot, self.velocity_plot, self.acceleration_plot):
            layout.addWidget(plot)
        self.peaks = self.table(layout, [f'J{i}' for i in range(1, 7)],
                               ('|V| °/s', '|a| °/s²'))
        layout.addStretch()

    def create_dynamics(self):
        layout = self.page('Dinámica')
        self.label(layout, 'Estimación URDF · τ = M(q)q̈ + c(q,q̇) + g(q). '
                   'Las inercias del modelo requieren validación física, especialmente la aproximación de link_6. '
                   'Cálculo sin fricción, reductores ni herramienta adicional. '
                   'Los pares máximos admitidos por el fabricante quedan pendientes de datos.')
        self.dynamic_time = self.label(layout)
        self.time_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.time_slider.setRange(0, 0)
        self.time_slider.valueChanged.connect(self.show_dynamics)
        layout.addWidget(self.time_slider)
        self.torque_table = self.table(layout, [f'J{i}' for i in range(1, 7)],
                                      ('M q̈', 'c', 'g', 'τ (Nm)'))
        self.label(layout, 'Matriz de inercia M(q) · kg·m²')
        self.mass_table = self.table(layout, [f'J{i}' for i in range(1, 7)],
                                    [f'J{i}' for i in range(1, 7)])
        self.energy_info = self.label(layout)
        self.label(layout, 'K = ½ q̇ᵀM q̇, U = Σ m g zCOM, P = τᵀq̇. '
                   'c se calcula con las derivadas de M y los coeficientes de Christoffel.')
        self.torque_plot = SeriesPlot('Par estimado durante LSPB', 'Nm')
        layout.addWidget(self.torque_plot)
        layout.addStretch()

    def plan_edited(self):
        self.target_edited()
        self.analysis_key = None
        self.analysis_result = None
        self.analysis_at = 0.

    def orientation_edited(self):
        if not self.lock_ui:
            self.invalidate()
            self.xyz_edited()

    def xyz_edited(self):
        if not self.lock_ui:
            self.target_edited()
            self.pending_rotation = None
        super().xyz_edited()

    def sync_orientation(self):
        if len(self.rpy) != 3:
            return
        rotation = self.model.tip(self.rendered, self.envelope)[:3, :3]
        self.lock_ui = True
        try:
            for field, value in zip(self.rpy, rpy_degrees(rotation)):
                field.setValue(float(value))
        finally:
            self.lock_ui = False

    def update_xyz_fields(self, xyz):
        super().update_xyz_fields(xyz)
        self.sync_orientation()

    def set_goal(self, q):
        self.target_edited()
        super().set_goal(q)
        self.sync_orientation()

    def pose_target(self, xyz, rotation, final):
        if self.route_animation is not None:
            self.route_stop_preview()
        self.target_edited()
        self.pending_rotation = rotation.copy()
        self.lock_ui = True
        try:
            for field, value in zip(self.rpy, rpy_degrees(rotation)):
                field.setValue(float(value))
        finally:
            self.lock_ui = False
        self.pick_target(xyz, final)

    def start_ik(self, xyz, from_marker):
        self.xyz_dirty = True
        self.invalidate()
        self.future_version = self.version
        self.status.clear()
        self.ik_from_marker = from_marker
        self.requested_xyz = np.asarray(xyz)*1000
        rotation = (self.pending_rotation if from_marker and self.pending_rotation is not None
                    else rotation_rpy([field.value() for field in self.rpy]))
        self.requested_rotation = rotation.copy()
        self.pending_rotation = None
        bounds = [(row['min_deg'], row['max_deg']) for row in effective(self.config)]
        self.future = self.pool.submit(
            solve_pose, self.model, list(self.rendered), xyz, rotation, self.envelope, bounds)

    def analysis_request(self):
        try:
            start = self.current_measurement()
            source = 'Postura inicial medida del ABB'
        except ValueError:
            start = list(self.initial)
            source = 'Postura inicial de muestra · ABB sin lectura reciente'
        if (self.analysis_anchor is not None and self.analysis_anchor[1] == source
                and np.max(np.abs(np.asarray(start)-self.analysis_anchor[0])) <= .02):
            start = list(self.analysis_anchor[0])
        else:
            self.analysis_anchor = (list(start), source)
        target = list(self.rendered)
        duration, blend, speed, acceleration = [field.value() for field in self.plan_fields]
        key = (tuple(start), tuple(target), duration, blend, speed, acceleration, source)
        return key, source, (self.model, self.envelope, start, target, effective(self.config),
                             duration, blend/100, speed, acceleration)

    def display_analysis(self, data):
        path = data['path']
        self.fill(self.transform_table, data['T'])
        self.fill(self.jac_table, data['jac'])
        singular = data['singular']
        condition = '∞' if singular[-1] < 1e-10 else f'{singular[0]/singular[-1]:.4f}'
        values = ' / '.join(f'{value:.6f}' for value in singular)
        self.kinematics_info.setText(
            f'σ1…σ6 · {values}\nκ · {condition} · Rango numérico {np.sum(singular > 1e-7)}/6\n'
            f'Manipulabilidad escalada ∏σ · {np.prod(singular):.8g}')
        self.lspb_info.setText(
            f'{self.plan_source}\nT {path["duration"]:.3f} s · tb {path["blend_time"]:.3f} s\n'
            f'Altura mínima muestreada {data["minimum_height"]*1000:.3f} mm · '
            f'|q̇(0)|máx {np.max(np.abs(path["qd"][0])):.6f} rad/s · '
            f'|q̇(T)|máx {np.max(np.abs(path["qd"][-1])):.6f} rad/s')
        for plot, key in ((self.position_plot, 'q'), (self.velocity_plot, 'qd'),
                          (self.acceleration_plot, 'qdd')):
            plot.set_series(path['t'], np.degrees(path[key]))
        self.fill(self.peaks, np.degrees(np.c_[np.abs(path['velocity']), np.abs(path['acceleration'])]), 3)
        indices = data['dynamic_indices']
        self.torque_plot.set_series(path['t'][indices], data['tau'])
        self.time_slider.setRange(0, len(indices)-1)
        self.show_dynamics(self.time_slider.value())

    def show_dynamics(self, index):
        data = self.analysis_result
        if data is None:
            return
        index = min(index, len(data['dynamic_indices'])-1)
        frame = data['evaluated'][index]
        sample = data['dynamic_indices'][index]
        path = data['path']
        q = np.degrees(path['q'][sample])
        pose = self.model.tip(q, self.envelope)
        twist = geometric_jacobian(self.model, q, pose[:3, 3]) @ path['qd'][sample]
        self.fill(self.mass_table, frame['M'])
        self.fill(self.torque_table, np.c_[frame['inertia'], frame['c'], frame['g'], frame['tau']])
        self.dynamic_time.setText(f't {path["t"][sample]:.3f} s / {path["duration"]:.3f} s')
        power = frame['tau'] @ path['qd'][sample]
        self.energy_info.setText(
            f'Masa móvil del URDF {data["moving_mass"]:.3f} kg\n'
            f'K {frame["kinetic"]:.5f} J · U {frame["potential"]:.5f} J · '
            f'K+U {frame["kinetic"]+frame["potential"]:.5f} J · P {power:.5f} W\n'
            f'Rapidez lineal {np.linalg.norm(twist[:3])*1000:.3f} mm/s · '
            f'Rapidez angular {np.degrees(np.linalg.norm(twist[3:])):.3f}°/s')

    def execution_block_reason(self):
        """Una sola decisión para el botón y el clic; los GET no son movimientos."""
        if not self.auto_armed:
            return 'Conecta el ABB para habilitar la ejecución.'
        if self.motion is not None or self.reviewing or self.stop_requested:
            return 'Espera a que termine la operación actual.'
        if self.auto_task is not None and self.auto_operation != 'status':
            return 'Preparando o deteniendo la sesión del ABB.'
        if self.speed_future is not None and self.speed_operation == 'apply':
            return 'Espera la confirmación del cambio de velocidad.'
        if self.speed_dirty:
            return 'Aplica primero el porcentaje de velocidad elegido.'
        try:
            self.current_measurement()
        except ValueError as exc:
            return str(exc)
        if self.speed_actual is None or time.monotonic()-self.speed_read_at >= 5.:
            return 'Se necesita una lectura reciente de la velocidad ABB.'
        if self.speed_actual['mode'] != 'AUTO':
            return 'El controlador debe confirmar modo AUTO.'
        if self.speed_actual['ratio'] <= 0:
            return 'La velocidad ABB está en 0%; aplica un porcentaje positivo.'
        if (self.future is not None or self.pending_xyz is not None or self.xyz_dirty
                or self.tip_target.dragging or self.route_animation is not None):
            return 'Finaliza el ajuste o la vista previa de la sombra.'
        if self.uses_route():
            if not self.route_points:
                return 'Añade puntos a la trayectoria.'
            if not self.route_valid:
                return 'Revisa el aviso del recorrido antes de ejecutar.'
        elif (self.goal() != self.rendered or self.analysis_result is None
              or tuple(self.analysis_result['target']) != tuple(self.rendered)):
            return 'Espera a que terminen los cálculos del destino seleccionado.'
        return ''

    def execute_selected_goal(self):
        reason = self.execution_block_reason()
        if reason:
            self.status.setText(reason)
            return
        if self.uses_route():
            self.route_execute()
            return
        if self.speed_dirty:
            self.status.setText('Aplica primero el porcentaje de velocidad elegido.')
            return
        if self.speed_future is not None and self.speed_operation == 'apply':
            self.status.setText('Espera la confirmación de velocidad del ABB.')
            return
        if self.speed_actual is not None and self.speed_actual['ratio'] == 0:
            self.status.setText('La velocidad del ABB está en 0%. Aplica un porcentaje mayor para mover.')
            return
        if (self.analysis_result is None
                or tuple(self.analysis_result['target']) != tuple(self.rendered)):
            self.status.setText('Espera a que terminen los cálculos del destino.')
            return
        try:
            q = self.current_measurement()
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        if np.max(np.abs(np.asarray(q)-self.analysis_result['start'])) > .02:
            self.plan_edited()
            self.status.setText('Actualizando los cálculos desde la postura real.')
            return
        super().execute_selected_goal()

    def refresh_metrics(self, snapshot, fresh):
        super().refresh_metrics(snapshot, fresh)
        if self.ik_reference and self.ik_reference[0] == self.goal() and hasattr(self, 'requested_rotation'):
            rotation = self.model.tip(self.rendered, self.envelope)[:3, :3]
            self.orientation_residual.setText(
                f'Residual angular IK · {orientation_error(rotation, self.requested_rotation):.6f}°\n'
                'eR = acos((tr(Rpedido Rmodeloᵀ) − 1)/2).')
        else:
            self.orientation_residual.clear()

    def tick(self):
        super().tick()
        self.tick_speed()
        if self.reader is not None and hasattr(self.reader, 'orientation_snapshot'):
            quaternion, measured, stamp, error = self.reader.orientation_snapshot()
            if quaternion is not None and measured is not None and not error and time.monotonic()-stamp < 1.:
                rotation = quaternion_rotation(quaternion)
                desired = self.model.tip(self.rendered, self.envelope)[:3, :3]
                fk = self.model.tip(measured, self.envelope)[:3, :3]
                angles = ' / '.join(f'{v:.4f}°' for v in rpy_degrees(rotation))
                self.orientation_info.setText(
                    f'Roll / pitch / yaw ABB · {angles}\n'
                    f'Error angular destino–ABB · {orientation_error(desired, rotation):.6f}°\n'
                    f'Error angular FK(medido)–ABB · {orientation_error(fk, rotation):.6f}°')
            else:
                self.orientation_info.setText('Orientación ABB pendiente de lectura reciente.')
        else:
            self.orientation_info.setText('Orientación ABB pendiente de conexión.')
        if self.analysis_future is not None and self.analysis_future.done():
            future, self.analysis_future = self.analysis_future, None
            try:
                result = future.result()
                key, source, _ = self.analysis_request()
                if self.analysis_key == key:
                    self.analysis_result = result
                    self.plan_source = source
                    self.display_analysis(result)
            except CancelledError:
                self.analysis_result = None
                self.analysis_key = None
            except Exception as exc:
                self.analysis_result = None
                self.lspb_info.setText('Cálculo pendiente · '+str(exc))
        if (time.monotonic() >= self.analysis_at and self.analysis_future is None
                and self.future is None and not self.xyz_dirty and self.motion is None
                and not self.tip_target.dragging and self.pending_xyz is None
                and self.route_animation is None
                and time.monotonic()-self.last_target_edit > .35):
            key, source, args = self.analysis_request()
            if key != self.analysis_key:
                self.analysis_key = key
                self.analysis_result = None
                self.plan_source = source
                self.lspb_info.setText('Calculando LSPB, cinemática y dinámica…')
                self.analysis_cancel = Event()
                self.analysis_future = self.analysis_pool.submit(analyze, *args, cancelled=self.analysis_cancel)
            self.analysis_at = time.monotonic()+.5
        if self.speed_future is not None and self.speed_operation == 'apply':
            self.start_auto.setEnabled(False)
        self.tick_route()
        if self.tabs.currentWidget() is self.pose_page:
            self.pose_page.refresh()
        reason = self.execution_block_reason()
        self.execute.setEnabled(not reason)
        self.execute.setToolTip(reason or ('Envía la trayectoria al ABB.'
            if self.uses_route() else 'Envía el destino al ABB.'))
        self.recorder.capture(self)

    def closeEvent(self, event):
        self.route_stop_preview()
        if self.auto_armed or self.auto_task is not None or self.motion is not None or self.stop_requested:
            super().closeEvent(event)
            return
        self.analysis_pool.shutdown(wait=False, cancel_futures=True)
        if self.recorder.samples or self.recorder.events:
            try:
                folder = Path.home()/'abb-real'/'ordenes'
                folder.mkdir(parents=True, exist_ok=True)
                self.recorder.save(folder/('lecturas_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json'))
            except (OSError, ValueError) as exc:
                self.status.setText('Registro sin guardar: '+str(exc))
        self.math_window.shutdown()
        if self.speed_client is not None:
            self.speed_pool.submit(self.speed_client.close)
        self.speed_pool.shutdown(wait=False)
        super().closeEvent(event)


def main():
    import argparse
    import trayectorias
    parser = argparse.ArgumentParser()
    parser.add_argument('--ruta')
    parser.add_argument('--analisis', action='store_true')
    parser.add_argument('--registro-actual')
    options, _ = parser.parse_known_args()
    app = QtWidgets.QApplication(sys.argv)
    preview = DualPreview()
    try:
        window = RoboticsPanel(preview)
    except Exception:
        preview.close()
        raise
    signal.signal(signal.SIGINT, lambda *_: QtCore.QTimer.singleShot(0, window.close))
    window.show()
    if options.registro_actual:
        try:
            window.recorder = Recorder.load(options.registro_actual)
        except (ValueError, OSError) as exc:
            window.status.setText('No se restauraron las lecturas de la sesión: '+str(exc))
    if options.ruta:
        try:
            window.route_points = trayectorias.load(options.ruta)
            window.route_refresh(0)
            window.tabs.setCurrentWidget(window.route_page)
        except (ValueError, OSError) as exc:
            window.status.setText('No se abrió la trayectoria: '+str(exc))
    if options.analisis:
        QtCore.QTimer.singleShot(600, lambda: window.show_analysis(0))
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
