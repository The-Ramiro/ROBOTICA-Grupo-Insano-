"""Gráfica grande con fases, puntos y lectura del tiempo con el cursor."""
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets


class ReportPlot(QtWidgets.QWidget):
    colors = ('#1877b9', '#d66a18', '#269654', '#b44ba6', '#c43b3b', '#65656d')

    def __init__(self, title, units, t, values, labels, phases=(), marks=(), threshold=None, y_min=None):
        super().__init__()
        self.title, self.units, self.t = title, units, np.asarray(t)
        self.values = np.asarray(values)
        if self.values.ndim == 1:
            self.values = self.values[:, None]
        self.labels, self.phases, self.marks, self.threshold = labels, phases, marks, threshold
        self.y_min = y_min
        self.cursor = None
        self.setMouseTracking(True)
        self.setMinimumHeight(245)
        self.setMaximumHeight(245)

    def mouseMoveEvent(self, event):
        area = QtCore.QRectF(80, 39, max(60, self.width()-105), self.height()-85)
        fraction = np.clip((event.x()-area.left())/area.width(), 0, 1)
        self.cursor = int(np.argmin(abs(self.t-(self.t[0]+fraction*(self.t[-1]-self.t[0])))))
        index = self.cursor
        self.setToolTip(f't = {self.t[index]:.4f} s\n'+'\n'.join(
            f'{label}: {self.values[index, i]:.7g} {self.units}' for i, label in enumerate(self.labels)))
        self.update()

    def leaveEvent(self, event):
        self.cursor = None
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('white'))
        p.setPen(QtGui.QColor('#243445'))
        p.drawText(12, 20, self.title+' ['+self.units+']')
        area = QtCore.QRectF(80, 39, max(60, self.width()-105), self.height()-85)
        finite = self.values[np.isfinite(self.values)]
        if not len(finite):
            p.drawText(90, 80, 'Sin datos válidos. No se sustituyen por ceros.')
            return
        low, high = float(finite.min()), float(finite.max())
        margin = max((high-low)*.08, 1e-8 if high < .01 else .05)
        low, high = low-margin, high+margin
        if self.y_min is not None:
            low = self.y_min
            high = max(high, (self.threshold or 0)*1.2, low+1e-8)
        span = max(float(self.t[-1]-self.t[0]), 1e-9)
        x = lambda t: area.left()+area.width()*(t-self.t[0])/span
        y = lambda v: area.bottom()-area.height()*(v-low)/(high-low)
        for begin, end, phase in self.phases:
            p.fillRect(QtCore.QRectF(x(begin), area.top(), max(.1, x(end)-x(begin)), area.height()),
                       QtGui.QColor('#e5f3e8' if phase == 'accel' else '#fbe9e9'))
        if self.threshold is not None:
            bottom = y(min(self.threshold, high))
            p.fillRect(QtCore.QRectF(area.left(), max(area.top(), bottom), area.width(),
                                    max(0., area.bottom()-max(area.top(), bottom))), QtGui.QColor('#fbe0d8'))
        p.setPen(QtGui.QColor('#dde3e9'))
        for i in range(5):
            yy = area.top()+i*area.height()/4
            p.drawLine(QtCore.QPointF(area.left(), yy), QtCore.QPointF(area.right(), yy))
            p.setPen(QtGui.QColor('#586572'))
            p.drawText(QtCore.QRectF(0, yy-9, 73, 20), QtCore.Qt.AlignRight,
                       f'{high-i*(high-low)/4:.4g}')
            p.setPen(QtGui.QColor('#dde3e9'))
        for t, label in self.marks:
            p.setPen(QtGui.QPen(QtGui.QColor('#b4bec9'), 1, QtCore.Qt.DashLine))
            p.drawLine(QtCore.QPointF(x(t), area.top()), QtCore.QPointF(x(t), area.bottom()))
            p.drawText(int(x(t)+3), int(area.top()-4), f'P{label}')
        p.save()
        p.setClipRect(area.adjusted(-1, -1, 1, 1))
        for index, label in enumerate(self.labels):
            path = QtGui.QPainterPath()
            begun = False
            previous = None
            for t, v in zip(self.t, self.values[:, index]):
                if not np.isfinite(v):
                    begun = False
                    previous = None
                    continue
                point = QtCore.QPointF(x(t), y(v))
                if begun and index in getattr(self,'step_series',set()):
                    path.lineTo(QtCore.QPointF(point.x(),previous.y()))
                path.lineTo(point) if begun else path.moveTo(point)
                begun = True
                previous = point
            styles=getattr(self,'styles',())
            p.setPen(QtGui.QPen(QtGui.QColor(self.colors[index % len(self.colors)]), 1.5,
                                styles[index] if index < len(styles) else QtCore.Qt.SolidLine))
            p.drawPath(path)
        if self.cursor is not None:
            p.setPen(QtGui.QPen(QtGui.QColor('#344c66'), 1, QtCore.Qt.DashLine))
            xx = x(self.t[self.cursor])
            p.drawLine(QtCore.QPointF(xx, area.top()), QtCore.QPointF(xx, area.bottom()))
        p.restore()
        for index, label in enumerate(self.labels):
            p.setPen(QtGui.QColor(self.colors[index % len(self.colors)]))
            p.drawText(int(85+index*(self.width()-95)/max(1,len(self.labels))), self.height()-5, label)
        p.setPen(QtGui.QColor('#586572'))
        for i in range(7):
            t = self.t[0]+span*i/6
            p.drawText(int(x(t)-15), int(area.bottom()+18), f'{t:.2f}')
        p.drawText(int(area.center().x()-35), self.height()-15, 'Tiempo [s]')
