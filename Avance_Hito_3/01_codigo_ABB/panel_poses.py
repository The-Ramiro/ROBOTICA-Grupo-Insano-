"""Selector de configuraciones articulares para un destino cartesiano."""
import numpy as np
from PyQt5 import QtCore, QtWidgets

from alternativas_irb120 import branches
from analisis_recorrido import matrix_metrics
from calculos_robotica import geometric_jacobian
from limites_movimiento import effective


class PosePage(QtWidgets.QScrollArea):
    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.options = []
        self.context_key = None

        self.setWidgetResizable(True)
        body = QtWidgets.QWidget()
        self.setWidget(body)
        layout = QtWidgets.QVBoxLayout(body)
        self.point = QtWidgets.QComboBox()
        self.point.currentIndexChanged.connect(self.change_point)
        layout.addWidget(self.point)
        self.summary = QtWidgets.QLabel()
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.table = self.make_table(['Configuración', 'Recorrido (°)', 'Rango', 'Margen (%)'])
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.table.setToolTip('Recomendada: menor suma de distancias articulares a los puntos vecinos.')
        self.table.setMinimumHeight(110)
        self.table.setMaximumHeight(150)
        self.table.currentCellChanged.connect(self.show_option)
        layout.addWidget(self.table, 1)
        self.angles = QtWidgets.QLabel()
        self.angles.setWordWrap(True)
        layout.addWidget(self.angles)
        layout.addWidget(QtWidgets.QLabel('Jacobiano geométrico en la base'))
        self.matrix = self.make_table([f'J{i}' for i in range(1, 7)])
        self.matrix.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.matrix.setMinimumHeight(174)
        self.matrix.setMaximumHeight(195)
        layout.addWidget(self.matrix)
        self.select = QtWidgets.QPushButton('Seleccionar pose')
        self.select.clicked.connect(self.apply_option)
        layout.addWidget(self.select)
        layout.addStretch()

    @staticmethod
    def make_table(headers):
        table = QtWidgets.QTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        table.verticalHeader().setDefaultSectionSize(23)
        table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        return table

    def change_point(self):
        self.refresh()
        self.panel.route_tab_changed()
        self.panel.math_window.request_preview()

    def sync_points(self):
        count = len(self.panel.route_points)
        if self.point.count() == count + 1:
            return
        index = self.point.currentData()
        blocker = QtCore.QSignalBlocker(self.point)
        self.point.clear()
        self.point.addItem('Destino de Control', -1)
        for row in range(count):
            self.point.addItem(f'P{row+1} de la trayectoria', row)
        if index is None:
            index = max(0, self.panel.route_table.currentRow()) if count else -1
        self.point.setCurrentIndex(min(index + 1, count))
        del blocker

    def is_route_point(self):
        index = self.point.currentData()
        return index is not None and 0 <= index < len(self.panel.route_points)

    def can_select(self):
        p = self.panel
        return (p.tabs.isEnabled() and p.motion is None and not p.reviewing
                and not p.stop_requested and p.route_animation is None
                and p.future is None and p.pending_xyz is None and not p.xyz_dirty
                and not p.tip_target.dragging)

    def refresh(self):
        p = self.panel
        self.sync_points()
        editable = self.can_select()
        self.select.setEnabled(editable and 0 <= self.table.currentRow() < len(self.options))
        if not editable:
            return
        index = self.point.currentData()
        route = p.route_points
        anchor = p.route_anchor()
        goal = route[index] if self.is_route_point() else p.rendered
        previous = route[index-1] if index > 0 else anchor
        following = route[index+1] if 0 <= index < len(route)-1 else None
        neighbors = [np.asarray(q) for q in (previous, following) if q is not None]
        if not neighbors:
            neighbors = [np.asarray(goal)]
        bounds = [(row['min_deg'], row['max_deg']) for row in effective(p.config)]
        key = (index, tuple(goal), tuple(tuple(np.round(q, 1)) for q in neighbors), tuple(bounds))
        if key == self.context_key:
            return
        self.context_key = key
        target = p.model.tip(goal, p.envelope)
        solutions, singular_family = branches(
            p.model, p.envelope, target[:3, 3], target[:3, :3], neighbors[0], bounds)
        if (target[2, 3] >= p.envelope.min_tip_z_m
                and all(lo <= q <= hi for q, (lo, hi) in zip(goal, bounds))):
            if not any(np.max(np.abs(np.subtract(q, goal))) < .005 for q in solutions):
                solutions.append(tuple(goal))
        options = []
        for q in solutions:
            tip = p.model.tip(q, p.envelope)
            jacobian = geometric_jacobian(p.model, q, tip[:3, 3])
            sigma, condition, _, rank = matrix_metrics(jacobian)
            distance = sum(np.linalg.norm(q-neighbor) for neighbor in neighbors)
            options.append((distance, 100/condition, rank, sigma, q, jacobian))
        options.sort(key=lambda row: (row[0], -row[1]))
        self.options = options
        caption = f'{len(options)} opciones para la misma posición y orientación.'
        if singular_family:
            caption += ' Las familias singulares se muestran con ejemplos.'
        self.summary.setText(caption)

        blocker = QtCore.QSignalBlocker(self.table)
        self.table.setRowCount(len(options))
        selected = 0
        for row, (distance, margin, rank, _, q, _) in enumerate(options):
            current = np.max(np.abs(np.subtract(q, goal))) < .005
            label = 'Recomendada' if row == 0 else f'Opción {row+1}'
            if current:
                label += ' · en uso'
                selected = row
            for col, value in enumerate((label, f'{distance:.2f}', f'{rank}/6', f'{margin:.4f}')):
                self.table.setItem(row, col, QtWidgets.QTableWidgetItem(value))
        if options:
            self.table.selectRow(selected)
        del blocker
        self.show_option(self.table.currentRow())

    def show_option(self, row, *_):
        self.matrix.setRowCount(0)
        self.angles.clear()
        self.select.setEnabled(self.can_select() and 0 <= row < len(self.options))
        if not 0 <= row < len(self.options):
            return
        _, margin, rank, sigma, q, jacobian = self.options[row]
        self.angles.setText('  ·  '.join(f'J{i} {value:.3f}°' for i, value in enumerate(q, 1))
                            + f'\nRango {rank}/6 · σ mín. {sigma:.5g} · margen {margin:.4f}%')
        self.matrix.setRowCount(6)
        self.matrix.setVerticalHeaderLabels(['vx', 'vy', 'vz', 'ωx', 'ωy', 'ωz'])
        self.matrix.setToolTip('Filas lineales en m/rad y filas angulares en rad/rad. '
                               'Para rango y margen se pondera el bloque angular con 0,20 m.')
        for i, values in enumerate(jacobian):
            for j, value in enumerate(values):
                self.matrix.setItem(i, j, QtWidgets.QTableWidgetItem(f'{value:.4f}'))

    def apply_option(self):
        key = self.context_key
        self.refresh()
        row = self.table.currentRow()
        if key != self.context_key or not self.can_select() or not 0 <= row < len(self.options):
            return
        p = self.panel
        q = list(self.options[row][4])
        if self.is_route_point():
            index = self.point.currentData()
            p.route_points[index] = q
            p.set_goal(q)
            p.route_refresh(index)
            p.status.setText(f'Pose seleccionada para P{index+1}.')
        else:
            p.set_goal(q)
            p.math_window.request_preview()
            p.status.setText('Pose seleccionada para el destino.')
        self.refresh()
