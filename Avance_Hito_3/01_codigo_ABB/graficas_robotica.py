"""Gráficas Qt de las seis articulaciones, sin dependencia adicional."""
from PyQt5 import QtCore, QtGui, QtWidgets
import numpy as np


class SeriesPlot(QtWidgets.QWidget):
    colors = ('#1877b9', '#d56b18', '#2c8b54', '#a64ca6', '#b43a42', '#6b7280')

    def __init__(self, title, units):
        super().__init__()
        self.title, self.units = title, units
        self.times = self.values = None
        self.setMinimumHeight(185)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

    def set_series(self, times, values):
        self.times, self.values = np.asarray(times), np.asarray(values)
        self.update()

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor('#ffffff'))
        painter.setPen(QtGui.QColor('#253444'))
        painter.drawText(8, 17, self.title + ' (' + self.units + ')')
        if self.times is None:
            painter.drawText(12, 60, 'Calculando…')
            return
        area = QtCore.QRectF(62, 29, max(60, self.width()-80), self.height()-77)
        low, high = float(self.values.min()), float(self.values.max())
        margin = max((high-low)*.08, .05)
        low, high = low-margin, high+margin
        painter.setPen(QtGui.QColor('#dae2ea'))
        for i in range(4):
            y = area.top()+area.height()*i/3
            painter.drawLine(QtCore.QPointF(area.left(), y), QtCore.QPointF(area.right(), y))
            painter.setPen(QtGui.QColor('#536271'))
            painter.drawText(QtCore.QRectF(0, y-8, 56, 18), QtCore.Qt.AlignRight,
                             f'{high-(high-low)*i/3:.2f}')
            painter.setPen(QtGui.QColor('#dae2ea'))
        maximum_time = max(float(self.times[-1]), 1e-9)
        for index, color in enumerate(self.colors):
            path = QtGui.QPainterPath()
            for k, (t, value) in enumerate(zip(self.times, self.values[:, index])):
                p = QtCore.QPointF(area.left()+area.width()*t/maximum_time,
                                  area.bottom()-area.height()*(value-low)/(high-low))
                path.moveTo(p) if k == 0 else path.lineTo(p)
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 1.5))
            painter.drawPath(path)
            painter.drawText(65+index*(self.width()-80)//6, self.height()-5, f'J{index+1}')
        painter.setPen(QtGui.QColor('#536271'))
        painter.drawText(int(area.left()), int(area.bottom()+18), '0')
        painter.drawText(int(area.right()-55), int(area.bottom()+18), f'{maximum_time:.2f} s')
