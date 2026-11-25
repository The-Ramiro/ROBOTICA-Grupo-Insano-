"""Comparación temporal del perfil previsto con las lecturas del ABB."""
from bisect import bisect_right
from itertools import islice
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from errores_recorrido import error_series, joint_rates, profile_state, spatial_errors, summarize_errors
from grafica_informe import ReportPlot
from referencia_recorrido import align_start, inferred_start, observed_start


AXES = ('X', 'Y', 'Z')
JOINTS = tuple(f'J{i}' for i in range(1, 7))
VELOCITIES = ('vX', 'vY', 'vZ', 'ωX', 'ωY', 'ωZ')
METRICS = (
    ('Posición', 'mm', 2.), ('Orientación', '°', 1.),
    ('Jacobiano lineal', 'm/rad', .01), ('Jacobiano angular', 'rad/rad', .02),
    ('Velocidad lineal', 'mm/s', 20.), ('Velocidad angular', '°/s', 10.),
)


class ErrorsPage(QtWidgets.QWidget):
    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.source_key = None
        self.display_key = None
        self.reset_history()

        layout = QtWidgets.QVBoxLayout(self)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        note = QtWidgets.QLabel('Perfil LSPB y lectura ABB al mismo tiempo · Δ = previsto − ABB · Marco de la base')
        note.setWordWrap(True)
        note.setToolTip('Posición, orientación y Jacobiano ABB calculados por cinemática directa con los seis ángulos leídos por RWS.')
        layout.addWidget(note)
        self.notice = QtWidgets.QLabel()
        self.notice.setWordWrap(True)
        layout.addWidget(self.notice)
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, 1)

        summary = self.add_page('Resumen del recorrido')
        self.summary_table = self.add_table(summary, 'Picos de error por magnitud',
            [f'{name} · {unit}' for name, unit, _ in METRICS],
            ('Umbral', 'Pico', 'Instante (s)', 'Tramo previsto', 'Lecturas sobre umbral'))
        self.thresholds = []
        for index, (_, _, default) in enumerate(METRICS):
            field = QtWidgets.QDoubleSpinBox()
            field.setDecimals(4 if default < 1. else 2)
            field.setRange(0., 1000000.)
            field.setSingleStep(default/10.)
            field.setValue(default)
            field.setKeyboardTracking(False)
            field.setAlignment(QtCore.Qt.AlignCenter)
            field.setToolTip('Umbral para destacar diferencias en esta vista.')
            self.summary_table.setCellWidget(index, 0, field)
            self.thresholds.append(field)
        legend = QtWidgets.QLabel('Umbrales editables para el análisis. Los picos se calculan con todas las lecturas válidas de la sesión.')
        legend.setWordWrap(True)
        summary.addWidget(legend)
        summary.addStretch()

        pose = self.add_page('Posición y orientación')
        tables = QtWidgets.QHBoxLayout()
        x_columns, r_columns = QtWidgets.QVBoxLayout(), QtWidgets.QVBoxLayout()
        tables.addLayout(x_columns, 1)
        tables.addLayout(r_columns, 1)
        pose.addLayout(tables)
        self.position = self.add_table(x_columns, 'Posición de la brida · mm', ('X', 'Y', 'Z', '‖Δp‖'),
                                       ('Previsto', 'ABB', 'Δ'))
        self.orientation = self.add_table(r_columns, 'Vector de rotación del error · °', (*AXES, 'Ángulo total'), ('ΔR',))
        details = QtWidgets.QCheckBox('Mostrar matriz de rotación relativa')
        pose.addWidget(details)
        self.rotation = self.add_table(pose, '', AXES, AXES)
        self.rotation.setToolTip('ΔR = R blanco × (R ABB)ᵀ. La norma del vector de rotación es el ángulo entre las orientaciones.')
        self.rotation.hide()
        details.toggled.connect(self.rotation.setVisible)
        self.pose_plots = self.add_plots(pose,
            ('Error de posición', 'mm', ('ΔX', 'ΔY', 'ΔZ', 'Norma')),
            ('Error de orientación', '°', ('ΔX', 'ΔY', 'ΔZ', 'Ángulo total')),
        )

        jac = self.add_page('Jacobiano')
        self.jacobian_view = QtWidgets.QTabBar()
        self.jacobian_view.setExpanding(False)
        for label in ('Diferencia ΔJ', 'J previsto', 'J ABB'):
            self.jacobian_view.addTab(label)
        jac.addWidget(self.jacobian_view)
        self.jacobian = self.add_table(jac, 'Jacobiano geométrico 6 × 6', VELOCITIES, JOINTS)
        self.jacobian.setToolTip('Filas v en m/rad. Filas ω en rad/rad. Las columnas corresponden a J1…J6. '
                                 'La gráfica usa la norma de Frobenius de cada bloque.')
        self.jacobian_norm = QtWidgets.QLabel()
        jac.addWidget(self.jacobian_norm)
        self.jac_plots = self.add_plots(jac,
            ('Norma del error jacobiano lineal', 'm/rad', ('‖ΔJv‖',)),
            ('Norma del error jacobiano angular', 'rad/rad', ('‖ΔJω‖',)),
        )
        for plot in self.jac_plots:
            plot.y_min = 0.

        velocity = self.add_page('Velocidad espacial')
        self.velocity = self.add_table(velocity, 'Velocidad de la brida en la base · v = J(q) q̇',
                                       (*[f'{axis} · mm/s' for axis in VELOCITIES[:3]],
                                        *[f'{axis} · °/s' for axis in VELOCITIES[3:]]),
                                       ('Previsto', 'ABB', 'Δ'))
        self.velocity_status = QtWidgets.QLabel()
        self.velocity_status.setWordWrap(True)
        velocity.addWidget(self.velocity_status)
        self.velocity_plots = self.add_plots(velocity,
            ('Error de velocidad lineal', 'mm/s', ('ΔvX', 'ΔvY', 'ΔvZ', 'Norma')),
            ('Error de velocidad angular', '°/s', ('ΔωX', 'ΔωY', 'ΔωZ', 'Norma')),
        )

        self.tabs.currentChanged.connect(self.refresh)
        self.jacobian_view.currentChanged.connect(self.refresh)
        for field in self.thresholds:
            field.valueChanged.connect(self.refresh)

    def add_page(self, name):
        scroll = QtWidgets.QScrollArea(widgetResizable=True)
        body = QtWidgets.QWidget()
        body.setMinimumWidth(930)
        scroll.setWidget(body)
        self.tabs.addTab(scroll, name)
        layout = QtWidgets.QVBoxLayout(body)
        layout.setSpacing(7)
        return layout

    @staticmethod
    def add_table(layout, title, rows, columns):
        if title:
            label = QtWidgets.QLabel(title)
            label.setStyleSheet('font-weight:bold')
            layout.addWidget(label)
        table = QtWidgets.QTableWidget(len(rows), len(columns))
        table.setVerticalHeaderLabels(rows)
        table.setHorizontalHeaderLabels(columns)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Fixed)
        table.verticalHeader().setDefaultSectionSize(28)
        table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        table.setAlternatingRowColors(True)
        table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        table.setFixedHeight(30 + 28*len(rows))
        for row in range(len(rows)):
            for column in range(len(columns)):
                item = QtWidgets.QTableWidgetItem('—')
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                if columns[column] == 'Δ':
                    item.setBackground(QtGui.QColor('#e6eff8'))
                table.setItem(row, column, item)
        layout.addWidget(table)
        return table

    @staticmethod
    def add_plots(layout, *specs):
        row = QtWidgets.QHBoxLayout()
        plots = []
        for title, units, labels in specs:
            chart = ReportPlot(title, units, [0.], np.full((1, len(labels)), np.nan), labels)
            row.addWidget(chart, 1)
            plots.append(chart)
        layout.addLayout(row)
        layout.addStretch()
        return plots

    @staticmethod
    def fill(table, data):
        for row in range(table.rowCount()):
            for column in range(table.columnCount()):
                value = data[row, column] if data is not None else np.nan
                table.item(row, column).setText(f'{value:.6f}' if np.isfinite(value) else '—')

    def reset_history(self):
        self.last_sample = None
        self.last_comparison = None
        self.reference = None
        self.profile = None
        self.timestamps = []
        self.magnitudes = []
        self.curves = [[] for _ in METRICS]

    def poll(self):
        if self.isVisible():
            self.refresh()

    def refresh(self):
        recorder = self.panel.recorder
        reference = recorder.reference
        rows = recorder.samples if reference is not None else ()
        onset = observed_start(reference, rows)
        start_label = 'inicio observado'
        if onset is None:
            onset = inferred_start(reference, rows)
            start_label = 'inicio estimado'
        key = (recorder.session_id, id(reference), onset)
        if key != self.source_key:
            self.source_key = key
            self.reset_history()
            self.reference = align_start(reference, onset)
            if self.reference is not None:
                self.profile = dict(times=np.asarray(self.reference['t']), angles=np.asarray(self.reference['q']).T,
                                    rates=np.asarray(self.reference['qd']).T)
        if rows and (self.last_sample is None or rows[-1]['t'] > self.last_sample['t']):
            start = 0 if self.last_sample is None else bisect_right(rows, self.last_sample['t'], key=lambda row: row['t'])
            self.compare(islice(rows, start, start+64))
        status = 'Esperando un recorrido con perfil previsto y lecturas ABB.'
        if self.last_sample is not None:
            instant = self.last_sample['t']
            status = f'{recorder.status_text()} · t = {instant:.3f} s'
            if onset is not None:
                status += f' · {start_label} {onset:.2f} s'
            if instant < rows[-1]['t']:
                status += ' · calculando el registro'
            elif recorder.active and time.monotonic()-recorder.zero-instant > .5:
                status += ' · esperando nueva lectura ABB'
        self.status.setText(status)
        thresholds = tuple(field.value() for field in self.thresholds)
        display_key = (key, len(self.timestamps), thresholds, self.tabs.currentIndex(), self.jacobian_view.currentIndex())
        if display_key == self.display_key:
            return
        self.display_key = display_key
        self.show_summary(thresholds)
        self.display(self.last_comparison)

    def compare(self, rows):
        for row in rows:
            instant = row['t']
            white, white_rate = profile_state(self.reference, instant, **self.profile)
            abb_rate = joint_rates(self.last_sample, row)
            result = spatial_errors(self.panel.model, self.panel.envelope, white, row['q'], white_rate, abb_rate)
            series = error_series(result)
            self.timestamps.append(instant)
            self.magnitudes.append(tuple(values[-1] for values in series))
            for curve, values in zip(self.curves, series):
                curve.append(values)
            self.last_sample = row
            self.last_comparison = result

    def point_at(self, instant):
        marks = self.reference.get('marks', [])
        if instant < self.reference.get('startup_s', 0.):
            return 'Arranque'
        for t, point in marks:
            if instant <= t:
                return f'Hacia P{point}'
        return f'Después de P{marks[-1][1]}' if marks else '—'

    def show_summary(self, thresholds):
        summaries = summarize_errors(self.magnitudes, thresholds) if self.magnitudes else [None]*len(METRICS)
        flagged = []
        for index, summary in enumerate(summaries):
            values = ('—',)*4
            over = 0
            if summary is not None:
                peak, over = summary
                instant = self.timestamps[peak]
                values = (f'{self.magnitudes[peak][index]:.6f}', f'{instant:.3f}', self.point_at(instant), str(over))
            if over:
                flagged.append(METRICS[index][0].lower())
            for column, value in enumerate(values, 1):
                item = self.summary_table.item(index, column)
                item.setText(value)
                item.setBackground(QtGui.QBrush(QtGui.QColor('#fce6cc')) if over else QtGui.QBrush())
        if flagged:
            self.notice.setText('Superó el umbral durante el recorrido · '+', '.join(flagged))
            self.notice.setStyleSheet('font-weight:bold;color:#9a4e10')
        else:
            self.notice.setText('Sin diferencias sobre los umbrales en las lecturas disponibles.' if self.magnitudes else '')
            self.notice.setStyleSheet('')

    def trace(self, plot, metric):
        timestamps = np.asarray(self.timestamps)
        values = np.asarray(self.curves[metric])
        if len(timestamps):
            gaps = np.flatnonzero(np.diff(timestamps) > .5) + 1
            middle = (timestamps[gaps-1]+timestamps[gaps])/2.
            plot.t = np.insert(timestamps, gaps, middle)
            plot.values = np.insert(values, gaps, np.nan, axis=0)
            plot.marks = [(t, point) for t, point in self.reference.get('marks', []) if plot.t[0] <= t <= plot.t[-1]]
        else:
            plot.t, plot.values, plot.marks = np.array([0.]), np.full((1, len(plot.labels)), np.nan), []
        if plot.cursor is not None and plot.cursor >= len(plot.t):
            plot.cursor = None
        plot.update()

    def display(self, latest):
        section = self.tabs.currentIndex()
        if section == 1:
            position = None if latest is None else np.vstack((
                latest['position'].T, [np.nan, np.nan, np.linalg.norm(latest['position'][2])],
            ))
            self.fill(self.position, position)
            self.fill(self.orientation, None if latest is None else self.curves[1][-1][:, None])
            self.fill(self.rotation, None if latest is None else latest['rotation'])
            self.trace(self.pose_plots[0], 0)
            self.trace(self.pose_plots[1], 1)
        elif section == 2:
            view = (2, 0, 1)[self.jacobian_view.currentIndex()]
            self.fill(self.jacobian, None if latest is None else latest['jacobians'][view])
            stamp = '' if latest is None else (
                f'‖ΔJv‖ {np.linalg.norm(latest["jacobians"][2, :3]):.6f} m/rad · '
                f'‖ΔJω‖ {np.linalg.norm(latest["jacobians"][2, 3:]):.6f} rad/rad')
            self.jacobian_norm.setText(stamp or 'Filas v en m/rad · filas ω en rad/rad')
            self.trace(self.jac_plots[0], 2)
            self.trace(self.jac_plots[1], 3)
        elif section == 3:
            self.fill(self.velocity, None if latest is None else latest['velocity'].T)
            if latest is not None and np.isfinite(latest['velocity'][2]).all():
                delta = latest['velocity'][2]
                self.velocity_status.setText(f'‖Δv‖ {np.linalg.norm(delta[:3]):.6f} mm/s · '
                                             f'‖Δω‖ {np.linalg.norm(delta[3:]):.6f} °/s · ABB por diferencias de ángulos')
            else:
                self.velocity_status.setText('Para calcular Δ hacen falta dos lecturas consecutivas cercanas.')
            self.trace(self.velocity_plots[0], 4)
            self.trace(self.velocity_plots[1], 5)
