"""Editor de puntos y vista previa sin envío físico automático."""
import time
import numpy as np
from PyQt5 import QtCore, QtWidgets
import trayectorias as route
from limites_movimiento import effective


class RoutePanel:
    def stop_automatic(self):
        if hasattr(self, 'route_timer'):
            self.route_stop_preview()
        super().stop_automatic()

    def create_route_editor(self):
        self.route_points = []
        self.route_valid = False
        self.route_drawn_anchor = None
        self.route_animation = None
        self.route_drawing = None
        if self.preview.node is not None:
            from ruta_rviz import RouteDrawing
            self.route_drawing = RouteDrawing(self.preview.node)
        page = QtWidgets.QWidget()
        self.route_page = page
        layout = QtWidgets.QVBoxLayout(page)
        layout.setSpacing(4)
        layout.setContentsMargins(7, 7, 7, 7)
        title = QtWidgets.QLabel('Construye el recorrido con la sombra')
        title.setStyleSheet('font-weight:bold; font-size:14px;')
        layout.addWidget(title)
        intro = QtWidgets.QLabel(
            'Sombra → Añadir punto → Previsualizar → Ejecutar')
        intro.setWordWrap(True)
        layout.addWidget(intro)
        row = QtWidgets.QHBoxLayout()
        self.route_add = QtWidgets.QPushButton('+ Añadir sombra como punto')
        self.route_add.clicked.connect(self.route_add_point)
        row.addWidget(self.route_add)
        self.route_replace = QtWidgets.QPushButton('Reemplazar seleccionado')
        self.route_replace.clicked.connect(self.route_replace_point)
        row.addWidget(self.route_replace)
        layout.addLayout(row)
        self.route_table = QtWidgets.QTableWidget(0, 4)
        self.route_table.setHorizontalHeaderLabels(['Punto', 'X · mm', 'Y · mm', 'Z · mm'])
        self.route_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.route_table.verticalHeader().hide()
        self.route_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.route_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.route_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.route_table.itemSelectionChanged.connect(self.route_selection)
        self.route_table.setMinimumHeight(115)
        self.route_table.setMaximumHeight(145)
        self.route_table.verticalHeader().setDefaultSectionSize(24)
        layout.addWidget(self.route_table, 1)
        self.route_detail = QtWidgets.QLabel('Selecciona un punto para ver sus seis ángulos.')
        self.route_detail.setWordWrap(True)
        layout.addWidget(self.route_detail)
        row = QtWidgets.QHBoxLayout()
        self.route_edit_buttons = []
        for text, callback in (
            ('Ver en sombra', self.route_show_point), ('↑', lambda: self.route_move(-1)),
            ('↓', lambda: self.route_move(1)), ('Eliminar', self.route_delete_point),
            ('Vaciar', self.route_clear)):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(callback)
            row.addWidget(button)
            self.route_edit_buttons.append(button)
        layout.addLayout(row)
        row = QtWidgets.QHBoxLayout()
        self.route_preview = QtWidgets.QPushButton('▶ Previsualizar sombra')
        self.route_preview.clicked.connect(self.route_preview_clicked)
        row.addWidget(self.route_preview)
        for text, callback in (('Guardar…', self.route_save), ('Abrir…', self.route_open)):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(callback)
            row.addWidget(button)
            self.route_edit_buttons.append(button)
        layout.addLayout(row)
        self.route_info = QtWidgets.QLabel('Sin puntos · máximo 100. El inicio se toma del ABB al ejecutar.')
        self.route_info.setWordWrap(True)
        layout.addWidget(self.route_info)
        note = QtWidgets.QLabel(
            'Recorrido articular estimado, con parada en cada punto. '
            'La animación es ilustrativa; ABB planifica el movimiento real. '
            'La vista previa no comprueba obstáculos del laboratorio.')
        note.setWordWrap(True)
        note.setStyleSheet('font-size:11px;color:#555;')
        self.route_preview.setToolTip(note.text())
        note.deleteLater()
        layout.addStretch()
        self.tabs.insertTab(1, page, 'Trayectorias')
        self.route_timer = QtCore.QTimer(self)
        self.route_timer.setInterval(33)
        self.route_timer.timeout.connect(self.route_animate)
        self.route_last_drawing = 0.
        self.tabs.currentChanged.connect(self.route_tab_changed)

    def route_selection(self):
        row = self.route_table.currentRow()
        if 0 <= row < len(self.route_points):
            self.route_detail.setText('  ·  '.join(
                f'J{i} {v:.3f}°' for i, v in enumerate(self.route_points[row], 1)))

    def route_anchor(self):
        try:
            return self.current_measurement()
        except ValueError:
            return None

    def route_refresh(self, selected=None):
        self.route_table.setRowCount(len(self.route_points))
        for index, q in enumerate(self.route_points):
            xyz = self.model.tip(q, self.envelope)[:3, 3]*1000
            for column, text in enumerate([f'P{index+1}', *(f'{v:.3f}' for v in xyz)]):
                item = QtWidgets.QTableWidgetItem(text)
                item.setToolTip(' / '.join(f'J{i}: {v:.4f}°' for i, v in enumerate(q, 1)))
                self.route_table.setItem(index, column, item)
        if selected is not None and self.route_points:
            self.route_table.selectRow(min(selected, len(self.route_points)-1))
        self.route_detail.setText('Selecciona un punto para ver sus seis ángulos.')
        self.route_selection()
        self.route_draw()
        if hasattr(self,'math_window'):self.math_window.request_preview()

    def route_draw(self):
        self.route_valid = False
        anchor = self.route_anchor()
        self.route_drawn_anchor = None if anchor is None else list(anchor)
        start = anchor if anchor is not None else (self.route_points[0] if self.route_points else self.rendered)
        poses = route.samples(start, self.route_points if anchor is not None else self.route_points[1:])
        xyz = [self.model.tip(q, self.envelope)[:3, 3] for q in poses]
        vertices = [self.model.tip(q, self.envelope)[:3, 3]
                    for q in self.route_points]
        if self.route_drawing is not None and self.uses_route():
            self.route_drawing.update(xyz if self.route_points else [], vertices)
        if not self.route_points:
            self.route_info.setText('Sin puntos · máximo 100. Añade la postura de la sombra.')
            return
        try:
            self.route_preview_targets(anchor)
            self.route_valid = True
            min_z = min(float(p[2]) for p in xyz)*1000
            self.route_info.setText(
                (f'{len(self.route_points)} puntos · salida desde la posición real del ABB.\n'
                 if anchor is not None else f'{len(self.route_points)} puntos · sin lectura ABB: vista desde P1.\n') +
                f'Altura mínima muestreada {min_z:.1f} mm · informativa, sin bloqueo del tramo.')
        except ValueError as exc:
            self.route_info.setText('Revisar recorrido · '+str(exc))

    def route_add_point(self):
        if len(self.route_points) >= route.MAX_POINTS:
            self.status.setText('Máximo 100 puntos por trayectoria.')
            return
        if (self.future is not None or self.pending_xyz is not None or self.xyz_dirty
                or self.tip_target.dragging or self.goal() != self.rendered):
            self.status.setText('Termina de ajustar la sombra antes de guardar el punto.')
            return
        q = list(self.rendered)
        if self.route_points and max(abs(a-b) for a, b in zip(q, self.route_points[-1])) <= .005:
            self.status.setText('La sombra coincide con el último punto. Muévela antes de añadir otro.')
            return
        self.route_points.append(q)
        self.route_refresh(len(self.route_points)-1)

    def route_replace_point(self):
        row = self.route_table.currentRow()
        if (row >= 0 and self.future is None and not self.xyz_dirty and self.pending_xyz is None
                and not self.tip_target.dragging and self.goal() == self.rendered):
            self.route_points[row] = list(self.rendered)
            self.route_refresh(row)

    def route_show_point(self):
        row = self.route_table.currentRow()
        if row >= 0:
            self.set_goal(self.route_points[row])

    def route_move(self, direction):
        row = self.route_table.currentRow()
        other = row+direction
        if 0 <= row < len(self.route_points) and 0 <= other < len(self.route_points):
            self.route_points[row], self.route_points[other] = self.route_points[other], self.route_points[row]
            self.route_refresh(other)

    def route_delete_point(self):
        row = self.route_table.currentRow()
        if row >= 0:
            del self.route_points[row]
            self.route_refresh(row)

    def route_clear(self):
        self.route_points.clear()
        self.route_refresh()

    def route_save(self):
        if not self.route_points:
            return
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Guardar trayectoria', 'trayectoria.json', 'Trayectoria (*.json)')
        if filename:
            try:
                route.save(filename, self.route_points)
                self.status.setText('Trayectoria guardada. La velocidad se elige al ejecutar.')
            except (ValueError, OSError) as exc:
                self.status.setText(str(exc))

    def route_open(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Abrir trayectoria', '', 'Trayectoria (*.json)')
        if filename:
            try:
                values = route.load(filename)
                self.route_points = values
                self.route_refresh(0)
            except (ValueError, OSError) as exc:
                self.status.setText(str(exc))

    def route_preview_targets(self, anchor):
        if anchor is not None:
            return route.prepare(self.model, self.envelope, self.config, anchor, self.route_points)
        values = route.points(self.route_points)
        # Sin lectura no se supone una postura inicial ni se dibuja su acceso.
        bounds = effective(self.config)
        for q in values:
            if any(not r['min_deg'] <= v <= r['max_deg'] for v, r in zip(q, bounds)):
                raise ValueError('Punto fuera del rango articular configurado.')
            self.model.check_height(q, self.envelope)
        if len(values) > 1:
            route.prepare(self.model, self.envelope, self.config, values[0], values)
        return values[1:]

    def route_preview_clicked(self):
        if self.route_animation is not None:
            self.route_stop_preview()
            return
        try:
            anchor = self.route_anchor()
            targets = self.route_preview_targets(anchor)
            self.route_draw()
            from referencia_recorrido import make_reference, profile_settings
            speed,duration,blend,acceleration = profile_settings(self)
            start = anchor if anchor is not None else self.route_points[0]
            reference = make_reference(start,targets or [start],speed,duration,blend,acceleration)
            self.route_animation = np.asarray(reference['q'],dtype=float)
            self.route_reference = reference
            self.route_profile_start = list(start)
            self.route_profile_targets = [list(q) for q in (targets or [start])]
            self.route_times = np.asarray(reference['t'],dtype=float)
            self.route_started = time.monotonic()
            self.math_window.preview_profile = reference
            self.math_window.preview_requested = True
            self.math_window.trajectory.refresh()
            if hasattr(self.tip_target, 'set_enabled'):
                self.tip_target.set_enabled(False)
            self.route_frame = 0
            self.route_preview.setText('■ Parar vista previa')
            self.status.setText(f'Vista previa LSPB · {speed:g}% · duración estimada {self.route_times[-1]:.2f} s.')
            self.route_timer.start()
        except ValueError as exc:
            self.status.setText(str(exc))

    def route_retime_preview(self,speed):
        if self.route_animation is None:return
        if speed <= 0:
            self.route_stop_preview()
            self.status.setText('Vista previa detenida · velocidad 0%.')
            return
        from referencia_recorrido import rescale_reference, retime_instant
        now=time.monotonic()
        old=self.route_reference
        reference=rescale_reference(old,speed)
        self.route_started=now-float(retime_instant(old,reference,now-self.route_started))
        self.route_reference=reference
        self.route_times=np.asarray(reference['t'])
        self.math_window.preview_profile=reference
        self.math_window.preview_requested=True
        self.math_window.trajectory.refresh()
        self.status.setText(f'Vista previa LSPB · {speed:g}% · duración estimada {self.route_times[-1]:.2f} s.')

    def route_stop_preview(self):
        self.route_timer.stop()
        self.route_animation = None
        self.route_preview.setText('▶ Previsualizar sombra')
        pose = self.model.tip(self.rendered, self.envelope)
        self.tip_target.set_tip(pose[:3, 3], pose[:3, :3])
        if hasattr(self.tip_target, 'set_enabled'):
            self.tip_target.set_enabled(True)

    def route_animate(self):
        if self.route_animation is None:
            return
        elapsed = time.monotonic()-self.route_started
        self.route_frame = min(int(np.searchsorted(self.route_times,elapsed)),len(self.route_times)-1)
        pose = [float(np.interp(elapsed,self.route_times,self.route_animation[:,j])) for j in range(6)]
        self.set_goal(pose)
        if elapsed >= self.route_times[-1]:
            self.route_stop_preview()
            self.status.setText('Vista previa terminada. No se envió ningún movimiento al ABB.')

    def uses_route(self):
        page = self.tabs.currentWidget()
        poses = getattr(self, 'pose_page', None)
        return page is self.route_page or (poses is not None and page is poses and poses.is_route_point())

    def route_tab_changed(self):
        page = self.tabs.currentWidget()
        if page is not self.route_page:
            self.route_stop_preview()
        uses_route = self.uses_route()
        if uses_route:
            self.route_draw()
        elif self.route_drawing is not None:
            self.route_drawing.update([], [])
            self.route_last_drawing = time.monotonic()
        self.execute.setText('Ejecutar trayectoria' if uses_route else 'Ejecutar destino')

    def route_execute(self):
        if not self.auto_armed or self.motion is not None or self.reviewing:
            self.status.setText('Conecta el ABB y espera a que esté listo.')
            return
        if (self.route_animation is not None or self.speed_dirty
                or (self.speed_future is not None and self.speed_operation == 'apply')
                or self.future is not None or self.pending_xyz is not None
                or self.xyz_dirty or self.tip_target.dragging):
            self.status.setText('Finaliza la vista previa y aplica la velocidad antes de ejecutar.')
            return
        try:
            start = self.current_measurement()
            targets = route.prepare(self.model, self.envelope, self.config, start, self.route_points)
            self.route_draw()
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        self.tabs.setEnabled(False)
        self.reviewing = True
        from referencia_recorrido import begin_recording
        begin_recording(self,start,targets)
        self.motion = self.auto_pool.submit(self.auto_session.execute_route, start, targets)
        if hasattr(self,'recorder'):self.recorder.execution=self.motion
        self.status.setText('Revisando y enviando la secuencia completa al ABB…')

    def tick_route(self):
        busy = self.motion is not None or self.reviewing or self.stop_requested
        editing = not busy and self.route_animation is None
        for button in [self.route_add, self.route_replace, *self.route_edit_buttons]:
            button.setEnabled(editing)
        self.route_preview.setEnabled(not busy and bool(self.route_points))
        self.route_table.setEnabled(editing)
        if self.route_drawing is not None and time.monotonic()-self.route_last_drawing > 1.:
            if editing and self.route_points and self.uses_route():
                anchor = self.route_anchor()
                previous = self.route_drawn_anchor
                if ((anchor is None) != (previous is None)
                        or (anchor is not None and previous is not None
                            and max(abs(a-b) for a, b in zip(anchor, previous)) > .02)):
                    self.route_draw()
            self.route_drawing.publish()
            self.route_last_drawing = time.monotonic()
