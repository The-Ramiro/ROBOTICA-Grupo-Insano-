"""Ventana independiente para perfiles completos, singularidades y lecturas reales."""
from concurrent.futures import CancelledError, ThreadPoolExecutor
from threading import Event
import json
from pathlib import Path
import time
import numpy as np
from PyQt5 import QtCore, QtWidgets, QtGui
from analisis_recorrido import report, GEOMETRY, mobility_state
from grafica_informe import ReportPlot
from grafica_singularidad import SingularityPlot
from registro_robotica import Recorder
from panel_seguimiento import TrackingPage
from panel_perfiles import TrajectoryPage
from panel_errores import ErrorsPage


class AnalysisWindow(QtWidgets.QMainWindow):
    progress = QtCore.pyqtSignal(str)

    def __init__(self, panel):
        super().__init__(panel, QtCore.Qt.Window)
        self.panel = panel
        self.setWindowTitle('IMT 342 · Análisis del recorrido y registro ABB')
        self.resize(1140, 760)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='informe-ruta')
        self.future = None
        self.cancelled = Event()
        self.result = None
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        row = QtWidgets.QHBoxLayout()
        self.refresh = QtWidgets.QPushButton('Calcular recorrido completo')
        self.refresh.clicked.connect(self.calculate)
        row.addWidget(self.refresh)
        cancel = QtWidgets.QPushButton('Cancelar cálculo')
        cancel.clicked.connect(self.cancelled.set)
        row.addWidget(cancel)
        row.addWidget(QtWidgets.QLabel('Avisar si margen direccional ≤'))
        self.threshold = QtWidgets.QDoubleSpinBox()
        self.threshold.setDecimals(2)
        self.threshold.setRange(.01, 20.)
        self.threshold.setValue(1.)
        self.threshold.setSuffix(' %')
        self.threshold.setToolTip('100 σmín/σmáx. Umbral didáctico configurable, no límite ABB ni garantía de seguridad.')
        self.threshold.valueChanged.connect(lambda: self.show_report(self.result) if self.result is not None else None)
        row.addWidget(self.threshold)
        self.last_live_refresh = 0.
        self.status = QtWidgets.QLabel('Selecciona puntos y calcula el recorrido.')
        self.status.setWordWrap(True)
        layout.addLayout(row)
        layout.addWidget(self.status)
        self.progress.connect(self.status.setText)
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, 1)
        for i in range(row.count()):
            if row.itemAt(i).widget() is not None:row.itemAt(i).widget().hide()
        self.status.hide()
        self.preview_profile=None
        self.preview_requested=False
        self.preview_due=None
        self.report_requested=False
        self.profile=self.scroll_widget()
        self.profile.hide()
        self.trajectory=TrajectoryPage(self)
        self.tabs.addTab(self.trajectory,'Trayectoria')
        self.point_table = QtWidgets.QTableWidget(0, 7)
        self.point_table.setHorizontalHeaderLabels(['Tramo hacia', 'Inicio · s', 'Llegada · s', 'Peor instante · s', 'Margen mínimo', 'Rango / 6', 'Diagnóstico'])
        self.point_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.point_table.horizontalHeader().setStretchLastSection(True)
        self.point_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.point_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.sing_page = QtWidgets.QWidget()
        sing_layout = QtWidgets.QVBoxLayout(self.sing_page)
        self.label(sing_layout,'Singularidad del robot · rango 6: seis direcciones instantáneas; rango menor: pérdida de movilidad.')
        self.sing_views = QtWidgets.QTabWidget()
        live_page = QtWidgets.QWidget()
        live_layout = QtWidgets.QVBoxLayout(live_page)
        self.live_detail=QtWidgets.QComboBox()
        self.live_detail.addItems(['Diagnóstico global','J1–J6 · singularidad durante el recorrido'])
        self.live_detail.setCurrentIndex(0)
        live_controls=QtWidgets.QHBoxLayout();live_controls.addWidget(self.live_detail,1)
        row.removeWidget(self.threshold)
        live_controls.addWidget(QtWidgets.QLabel('Umbral de cercanía'))
        live_controls.addWidget(self.threshold);self.threshold.show()
        live_layout.addLayout(live_controls)
        self.live_sing_status = QtWidgets.QLabel('Sin lecturas actuales del ABB.')
        self.live_sing_status.setWordWrap(True)
        live_layout.addWidget(self.live_sing_status)
        self.live_body = self.scroll_widget()
        live_layout.addWidget(self.live_body, 1)
        self.sing_views.addTab(live_page, 'ABB en vivo · tiempo medido')
        planned_page = QtWidgets.QWidget()
        planned_layout = QtWidgets.QVBoxLayout(planned_page)
        self.sing_views.addTab(planned_page, 'Recorrido propuesto · estimación')
        sing_layout.addWidget(self.sing_views, 1)
        controls = QtWidgets.QHBoxLayout()
        row.removeWidget(self.refresh)
        self.refresh.setText('Actualizar recorrido');self.refresh.show()
        controls.addWidget(self.refresh)
        self.sing_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sing_slider.setRange(0, 0)
        self.sing_slider.valueChanged.connect(self.show_sing_instant)
        controls.addWidget(self.sing_slider, 1)
        show = QtWidgets.QPushButton('Ver postura en sombra')
        show.clicked.connect(self.show_sing_pose)
        controls.addWidget(show)
        planned_layout.addLayout(controls)
        self.sing_instant = QtWidgets.QLabel('Calcula primero el recorrido.')
        self.sing_instant.setWordWrap(True)
        planned_layout.addWidget(self.sing_instant)
        self.sing_instant.hide()
        self.sing_body = self.scroll_widget()
        planned_layout.addWidget(self.sing_body, 1)
        self.point_table.setMinimumHeight(160)
        self.point_table.setMaximumHeight(220)
        self.point_table.cellClicked.connect(self.select_sing_point)
        details = QtWidgets.QCheckBox('Mostrar tabla por tramo')
        details.toggled.connect(self.point_table.setVisible)
        planned_layout.addWidget(details)
        planned_layout.addWidget(self.point_table)
        self.point_table.hide()
        self.sing_views.tabBar().show()
        self.sing_views.currentChanged.connect(lambda i:self.calculate() if i==1 and self.future is None else None)
        self.tabs.addTab(self.sing_page, 'Singularidades')
        real = QtWidgets.QWidget()
        real_layout = QtWidgets.QVBoxLayout(real)
        row = QtWidgets.QHBoxLayout()
        read = QtWidgets.QPushButton('Actualizar gráficas de lecturas ABB')
        read.clicked.connect(self.show_measured)
        row.addWidget(read)
        save = QtWidgets.QPushButton('Guardar registro…')
        save.clicked.connect(self.export_record)
        row.addWidget(save)
        logs = QtWidgets.QPushButton('Abrir registro de ejecución…')
        logs.clicked.connect(self.open_log)
        row.addWidget(logs)
        real_layout.addLayout(row)
        self.real_body = self.scroll_widget()
        real_layout.addWidget(self.real_body, 1)
        self.tabs.addTab(real, 'Registro')
        self.tabs.setTabVisible(2,False)
        self.details = QtWidgets.QTabWidget(self)
        self.details.hide()  # Conservar datos internos usados por la validación del destino.
        self.tracking = TrackingPage(self)
        self.tabs.addTab(self.tracking, 'Seguimiento')
        self.errors = ErrorsPage(panel)
        self.tabs.addTab(self.errors, 'Errores')
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(150)
        self.timer.timeout.connect(self.poll)
        self.timer.start()

    def scroll_widget(self):
        scroll = QtWidgets.QScrollArea(widgetResizable=True)
        body = QtWidgets.QWidget()
        QtWidgets.QVBoxLayout(body)
        body.setMinimumWidth(1020)
        scroll.setWidget(body)
        return scroll

    def scroll_page(self, name):
        scroll = self.scroll_widget()
        self.tabs.addTab(scroll, name)
        return scroll

    @staticmethod
    def clear(scroll):
        layout = scroll.widget().layout()
        while layout.count():
            item = layout.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()
        return layout

    @staticmethod
    def label(layout, message):
        label = QtWidgets.QLabel(message)
        label.setWordWrap(True)
        layout.addWidget(label)

    def request_preview(self):
        self.preview_due=time.monotonic()+.25
        self.report_requested=True
        if self.future is not None:self.cancelled.set()

    def prepare_preview(self):
        from referencia_recorrido import make_reference, profile_settings
        p=self.panel
        if p.recorder.active:return
        self.preview_requested=True
        if p.route_animation is not None:
            self.preview_profile=p.route_reference
            self.trajectory.refresh()
            return
        start=p.route_anchor()
        targets=[list(q) for q in p.route_points] if p.uses_route() and p.route_points else [list(p.rendered)]
        if start is None:start=targets[0]
        speed,duration,blend,acceleration=profile_settings(p)
        try:self.preview_profile=make_reference(start,targets,speed,duration,blend,acceleration)
        except ValueError:self.preview_profile=None
        self.trajectory.refresh();self.tracking.refresh()

    def calculate(self):
        if self.future is not None:
            self.cancelled.set()
            self.status.setText('Cancelando cálculo anterior. Vuelve a calcular al terminar.')
            return
        p = self.panel
        targets = [list(q) for q in p.route_points] if p.uses_route() and p.route_points else [list(p.rendered)]
        start = p.route_anchor()
        if p.route_animation is not None:
            start=list(p.route_profile_start)
            targets=[list(q) for q in p.route_profile_targets]
        if not p.route_points and start is None:
            targets = [list(p.rendered)]
        from referencia_recorrido import profile_settings
        speed,duration,blend,acceleration = profile_settings(p)
        self.cancelled = Event()
        self.status.setText('Calculando LSPB y singularidades del recorrido…')
        self.future = self.pool.submit(report, p.model, p.envelope, start, targets, speed,
                                       duration, blend, acceleration, .001,
                                       self.cancelled, self.progress.emit, dynamics=False)
        self.report_requested=False
        self.refresh.setEnabled(False)

    def poll(self):
        if self.preview_due is not None and time.monotonic()>=self.preview_due and not self.panel.recorder.active:
            self.preview_due=None
            self.prepare_preview()
        if self.report_requested and self.preview_due is None and self.future is None and not self.panel.recorder.active:
            if self.panel.real_speed_value.value()>0:self.calculate()
            else:
                self.report_requested=False
                self.result=None
                self.clear(self.sing_body)
        self.trajectory.poll()
        self.tracking.poll()
        self.errors.poll()
        if self.isVisible() and time.monotonic()-self.last_live_refresh >= .5:
            self.last_live_refresh = time.monotonic()
            self.show_live_singularities()
        if self.future is None or not self.future.done():
            return
        future, self.future = self.future, None
        self.refresh.setEnabled(True)
        try:
            result = future.result()
            if not self.report_requested:
                self.result = result
                self.show_report(self.result)
        except CancelledError:
            self.status.setText('Cálculo cancelado. El control del ABB no cambió.')
        except Exception as exc:
            self.status.setText('No se pudo calcular: '+str(exc))

    def show_report(self, data):
        layout = self.clear(self.profile)
        self.profile.widget().setMinimumWidth(max(1020, len(data['rows'])*65))
        origin = 'Desde la postura medida del ABB' if data['initial_known'] else 'Sin ABB: empieza en P1; no se supone un acceso inicial'
        self.label(layout, origin+'.\nPERFIL ESTIMADO LSPB: no reproduce el planificador de MoveAbsJ ni los tiempos reales. '
                   f'Análisis al {data["speed"]:g}% de los límites articulares nominales. '
                   'Verde: aceleración · blanco: velocidad constante · rojo: desaceleración. '
                   'Mueve el cursor para leer valores. Desplaza horizontalmente para recorrer todos los puntos.')
        labels = [f'J{i}' for i in range(1, 7)]
        for title, unit, values in [('Posición articular', 'rad', data['q']),
                                     ('Velocidad articular', 'rad/s', data['qd']),
                                     ('Aceleración articular', 'rad/s²', data['qdd'])]:
            layout.addWidget(ReportPlot(title, unit, data['t'], values, labels, data['phases'], data['point_times']))
        self.point_table.setRowCount(len(data['rows']))
        for index, row in enumerate(data['rows']):
            warnings = row['rank'] < 6 or row['margin'] <= self.threshold.value()
            detail = mobility_state(row['margin'], row['rank'], self.threshold.value())
            values = [f'P{row["point"]}', f'{row["start"]:.3f}', f'{row["end"]:.3f}',
                      f'{row["critical_time"]:.3f}', f'{row["margin"]:.5f} %', str(row['rank']), detail]
            for col, text in enumerate(values):
                item = QtWidgets.QTableWidgetItem(text)
                item.setToolTip(detail)
                if warnings:
                    item.setBackground(QtGui.QColor('#ffe7cf'))
                self.point_table.setItem(index, col, item)
        flagged = [f'P{r["point"]}' for r in data['rows'] if r['rank'] < 6 or r['margin'] <= self.threshold.value()]
        self.status.setText(f'{len(data["rows"])} puntos · {data["duration"]:.3f} s estimados. '
                            +('Cercanía o singularidad hacia '+', '.join(flagged[:20]) if flagged else 'Sin cercanía detectada en las muestras evaluadas.'))
        self.show_singularities()

    def show_singularities(self):
        data = self.result
        layout = self.clear(self.sing_body)
        self.sing_body.widget().setMinimumWidth(max(1020, len(data['rows'])*65))
        self.label(layout, 'LSPB estimado, no perfil interno ABB. La malla finita y las lecturas de red pueden omitir un cruce rápido.')
        segments = [(r['start'], r['end'],
                     (('Actual' if data['initial_known'] else 'P1') if r['point'] == 1 else f"P{r['point']-1}")+
                     f" → P{r['point']}") for r in data['rows'] if not r['skipped']]
        self.planned_sing_plot = SingularityPlot(data['t'], data['sigma']/.2, np.abs(np.sin(data['q'][:,4])),
                     data['rank'], data['mobility'], segments, self.threshold.value(), 'RECORRIDO PROPUESTO · LSPB estimado')
        self.planned_sing_plot.selected.connect(self.sing_slider.setValue)
        layout.addWidget(self.planned_sing_plot)
        descriptions = {
            'wrist': ('Muñeca: ángulo entre los ejes J4 y J6', '°', '0°: los dos ejes quedan alineados; en este brazo sucede con J5 cerca de 0°.'),
            'elbow': ('Codo: desalineación entre brazo y antebrazo', '°', '0°: hombro, codo y centro de muñeca quedan sobre una recta. Se usan las dimensiones del URDF, no se supone J3=0°.'),
            'shoulder': ('Hombro: distancia del centro de muñeca al eje de la base', 'mm', '0 mm: el centro de muñeca cae sobre el eje de giro de la base.')}
        for key, name, close, aligned, unit in GEOMETRY:
            title, unit, explanation = descriptions[key]
            self.label(layout, explanation)
            layout.addWidget(ReportPlot(title, unit, data['t'], data['geometry'][key], [name],
                                       marks=data['point_times'], threshold=close, y_min=0.))
        self.sing_slider.setRange(0, len(data['t'])-1)
        self.show_sing_instant()

    def show_sing_instant(self):
        if self.result is None:
            return
        data = self.result
        i = min(self.sing_slider.value(), len(data['t'])-1)
        if hasattr(self, 'planned_sing_plot'):
            self.planned_sing_plot.cursor = i
            self.planned_sing_plot.update()
        self.sing_instant.setText(f"t = {data['t'][i]:.3f} s · hacia P{int(data['point'][i])} · " +
            mobility_state(data['mobility'][i], data['rank'][i], self.threshold.value()) +
            f" · margen {data['mobility'][i]:.5f}%\n" +
            ' · '.join(f"{name}: {data['geometry'][key][i]:.2f}{unit}"
                       for key, name, close, aligned, unit in GEOMETRY))

    def select_sing_point(self, row, column):
        if self.result is None or row >= len(self.result['rows']):
            return
        entry = self.result['rows'][row]
        instant = entry['critical_time']
        self.sing_slider.setValue(int(np.argmin(abs(self.result['t']-instant))))

    def show_live_singularities(self):
        recorder = self.panel.recorder
        rows = list(recorder.samples)
        live = False
        if self.panel.reader is not None:
            _, q, stamp, error, *_ = self.panel.reader.snapshot()
            live = q is not None and not error and time.monotonic()-stamp <= 1.
        latest = rows[-1] if rows else None
        live = live and latest is not None and time.monotonic()-recorder.zero-latest['t'] <= 1.
        if latest is not None and 'margin' in latest:
            state = mobility_state(latest['margin'], latest['rank'], self.threshold.value())
            self.live_sing_status.setText(recorder.status_text()+f" · rango {latest['rank']}/6 · margen {latest['margin']:.5f}%")
            self.live_sing_status.setToolTip(state)
            color = '#b42318' if latest['rank'] < 6 else '#a15c00' if latest['margin'] <= self.threshold.value() else '#176b36'
            self.live_sing_status.setStyleSheet(f'color:{color if live else "#666666"};font-weight:bold;')
        else:
            self.live_sing_status.setText(recorder.status_text())
        key = (recorder.session_id,len(rows),rows[-1]['t'] if rows else None,self.threshold.value(),self.live_detail.currentIndex(),recorder.state)
        if key == getattr(self, '_live_plot_key', None):return
        self._live_plot_key = key
        selected = [r for r in rows if 'margin' in r and r.get('run',1)>0]
        layout = self.clear(self.live_body)
        if not selected:
            self.label(layout, 'Todavía no hay muestras de esta ejecución. No se inventan lecturas.')
            return
        worst = min(selected, key=lambda r:r['margin'])
        origin = 0.
        t = np.array([r['t'] for r in selected])
        margins = np.array([r['margin'] for r in selected], dtype=float)
        ranks = np.array([r['rank'] for r in selected], dtype=float)
        marks, previous = [], None
        for i, r in enumerate(selected):
            if r.get('moving') and r.get('point') is not None and r['point'] != previous:
                marks.append((t[i], r['point']))
            previous = r.get('point')
            if i and t[i]-t[i-1] > .5:
                margins[i] = ranks[i] = np.nan
        sigma = np.array([r['sigma']/.2 for r in selected], dtype=float)
        wrist = np.abs(np.sin(np.radians([r['q'][4] for r in selected])))
        gaps = np.r_[False, np.diff(t) > .5]
        sigma[gaps] = wrist[gaps] = np.nan
        segments = []
        begin, point = 0, selected[0].get('point') if selected[0].get('moving') else None
        for n in range(1,len(selected)+1):
            next_point = (selected[n].get('point') if selected[n].get('moving') else None) if n < len(selected) else object()
            if next_point != point:
                end = min(n,len(selected)-1)
                segments.append((t[begin],t[end], f'Hacia P{point}' if point is not None else 'Sin tramo activo'))
                begin, point = n, next_point
        if self.live_detail.currentIndex()==0:
            self.live_body.widget().setMinimumHeight(560)
            layout.addWidget(SingularityPlot(t,sigma,wrist,ranks,margins,segments,self.threshold.value(),
                             recorder.status_text()+' · ángulos medidos + Jacobiano del modelo'))
        else:
            self.live_body.widget().setMinimumHeight(850)
            self.label(layout,'J1–J6 · bandas salmón: postura próxima o singular.')
            intervals=[];begin=None
            near=(margins<=self.threshold.value()) | (ranks<6)
            for i,flag in enumerate(near):
                if flag and begin is None:begin=t[i]
                if begin is not None and (not flag or i==len(t)-1):
                    intervals.append((begin,t[i],'decel'));begin=None
            body=QtWidgets.QWidget();body.setMinimumHeight(760);grid=QtWidgets.QGridLayout(body)
            q=np.asarray([r['q'] for r in selected],dtype=float);q[gaps]=np.nan
            for j in range(6):
                plot=ReportPlot(f'J{j+1} · ángulo durante el recorrido','°',t,q[:,j],[f'J{j+1}'],phases=intervals,marks=marks)
                plot.colors=(ReportPlot.colors[j],)
                plot.setFixedHeight(245);grid.addWidget(plot,j//2,j%2)
            layout.addWidget(body)

    def show_sing_pose(self):
        if self.result is None:
            return
        p = self.panel
        if (p.motion is not None or (p.auto_task is not None and p.auto_operation!='status')
                or p.route_animation is not None):
            self.status.setText('Finaliza el movimiento o la vista previa antes de mostrar otra postura.')
            return
        i = min(self.sing_slider.value(), len(self.result['t'])-1)
        p.set_goal(np.degrees(self.result['q'][i]).tolist())
        self.status.setText(f"Postura de t={self.result['t'][i]:.3f}s mostrada solo en la sombra; no se envió al ABB.")
        p.raise_()
        p.activateWindow()

    def show_measured(self):
        self.render_measured(self.panel.recorder)

    def render_measured(self, recorder):
        layout = self.clear(self.real_body)
        self.real_body.widget().setMinimumWidth(1020)
        try:
            data = recorder.arrays()
            self.label(layout, 'LECTURAS ABB de esta sesión. q viene del controlador; velocidad y aceleración son derivadas numéricas y contienen ruido de muestreo/red. '
                       'XYZ usa FK de q medido. Error = distancia entre FK y TCP comunicado por ABB; no mide el error físico con un sensor externo.')
            marks, previous = [], None
            for sample in data['samples']:
                point = sample.get('point')
                if point is not None and point != previous:
                    marks.append((sample['t'], point))
                previous = point
            for name, unit, values, labels in (
                ('Articulaciones medidas', 'rad', data['q'], [f'J{i}' for i in range(1, 7)]),
                ('Velocidad derivada de lecturas', 'rad/s', data['qd'], [f'J{i}' for i in range(1, 7)]),
                ('Aceleración derivada de lecturas', 'rad/s²', data['qdd'], [f'J{i}' for i in range(1, 7)]),
                ('σmín a partir de ángulos medidos', 'm/rad escalado', data['sigma'], ['σmín']),
                ('Diferencia FK–TCP ABB', 'mm', data['error'], ['Error FK–ABB'])):
                layout.addWidget(ReportPlot(name, unit, data['t'], values, labels, marks=marks))
        except ValueError as exc:
            self.label(layout, str(exc))
        self.label(layout, 'Sucesos observados: una coincidencia con una singularidad no demuestra que haya causado la parada. '
                   'Consulta el mensaje concreto del controlador o del envío.')
        self.add_events(layout, list(recorder.events))

    def add_events(self, layout, events):
        table = QtWidgets.QTableWidget(len(events), 3)
        table.setHorizontalHeaderLabels(['Fecha / tiempo', 'Origen', 'Mensaje / último tramo observado'])
        table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setStretchLastSection(True)
        table.setMinimumHeight(350)
        table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        for i, event in enumerate(events):
            for j, text in enumerate([event.get('date', event.get('time', '')), event.get('source', 'Registro'), event.get('message', '')]):
                table.setItem(i, j, QtWidgets.QTableWidgetItem(str(text)))
        layout.addWidget(table)

    def export_record(self):
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Guardar lecturas y sucesos', 'registro_abb.json', 'Registro (*.json)')
        if filename:
            try:
                self.panel.recorder.save(filename)
            except OSError as exc:
                self.status.setText(str(exc))

    def open_log(self):
        initial = getattr(self.panel.auto_session, 'log', Path.home()/'abb-real'/'ordenes')
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Registro de ejecución del panel', str(initial), 'Registro (*.jsonl *.json)')
        if not filename:
            return
        try:
            if Path(filename).suffix == '.json':
                self.render_measured(Recorder.load(filename))
                self.tabs.setCurrentIndex(2)
                return
            if Path(filename).stat().st_size > 20_000_000:
                raise ValueError('Registro demasiado grande.')
            events = []
            for line in Path(filename).read_text(encoding='utf-8').splitlines():
                obj = json.loads(line)
                if obj.get('event') == 'controller_events':
                    for row in obj.get('events', []):
                        events.append(dict(date=row.get('tstamp', ''), source='Registro ABB',
                                           message=f'{row.get("code", "")} {row.get("title", "")} · {row.get("desc", "")}'))
                if obj.get('event') in ('execution_failed', 'arrival', 'order_cancelled_after_stop', 'stop_unconfirmed', 'route_prepared'):
                    message = obj.get('message', obj.get('error', obj['event']))
                    if obj.get('progress'):
                        message += ' · '+str(obj['progress'])
                    events.append(dict(date=obj.get('time', ''), source='Envío RWS', message=message))
            self.add_events(self.clear(self.real_body), events)
            self.tabs.setCurrentIndex(2)
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self.status.setText(str(exc))

    def shutdown(self):
        self.cancelled.set()
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.close()
