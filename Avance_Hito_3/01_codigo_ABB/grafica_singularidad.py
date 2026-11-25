"""Monitor de singularidad y movilidad; sin órdenes al controlador."""
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets


class SingularityPlot(QtWidgets.QWidget):
    selected = QtCore.pyqtSignal(int)
    colors = ('#777777', '#e04848', '#2caa53', '#2495e9', '#e8a128', '#9464bd')

    def __init__(self, t, sigma, wrist, ranks, margins, segments, threshold=1., source='ESTIMACIÓN LSPB'):
        super().__init__()
        self.t, self.sigma, self.wrist, self.ranks, self.margins = [np.asarray(v, dtype=float)
            for v in (t, sigma, wrist, ranks, margins)]
        self.segments, self.threshold, self.source = segments, threshold, source
        self.cursor = len(self.t)-1
        self.setMinimumHeight(450)
        self.setMinimumWidth(1010)
        self.setMouseTracking(True)
        self.setToolTip('Rojo: σ6 de [Jv/0,20 m; Jω]. Azul: |sin(q5)|, indicio de muñeca solamente. '
                        'Rango: direcciones instantáneas independientes, no número de motores. '
                        'Banda naranja: 100 σ6/σ1 bajo el umbral configurable. No es un límite ABB.')

    def mouseMoveEvent(self, event):
        if not event.buttons() & QtCore.Qt.LeftButton:
            return
        self.select_at(event.x())

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.select_at(event.x())

    def select_at(self, position):
        f = np.clip((position-115)/max(1, self.width()-145), 0., 1.)
        self.cursor = int(np.argmin(abs(self.t-(self.t[0]+f*(self.t[-1]-self.t[0])))))
        self.selected.emit(self.cursor)
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('white'))
        i = self.cursor
        rank = int(self.ranks[i]) if np.isfinite(self.ranks[i]) else None
        near = np.isfinite(self.margins[i]) and self.margins[i] <= self.threshold
        color = '#b3261e' if rank is not None and rank < 6 else '#a66300' if near else '#287c32'
        text = ('SIN DATOS EN ESTA MUESTRA' if rank is None else
                f'SINGULAR · {rank}/6 direcciones · pierde {6-rank}' if rank < 6 else
                'CERCA DE SINGULARIDAD · rango 6/6' if near else 'REGULAR · rango 6/6')
        p.fillRect(QtCore.QRectF(0, 0, self.width(), 35), QtGui.QColor(color))
        p.setPen(QtGui.QColor('white'))
        p.drawText(QtCore.QRectF(8, 0, self.width()-16, 35), QtCore.Qt.AlignCenter,
                   f'{text}  |  σ6={self.sigma[i]:.6g}  |  margen={self.margins[i]:.5f}%  |  t={self.t[i]:.3f}s')
        p.setPen(QtGui.QColor('#243445'))
        p.drawText(12, 55, self.source+' · Singularidad y movilidad por tramo')
        p.drawText(12, 75, 'Rango 6: seis direcciones instantáneas. Rango <6: pierde direcciones; el brazo aún puede moverse.')
        upper = QtCore.QRectF(115, 108, self.width()-145, 140)
        lower = QtCore.QRectF(115, 300, self.width()-145, 90)
        span = max(self.t[-1]-self.t[0], 1e-9)
        x = lambda t: upper.left()+(t-self.t[0])/span*upper.width()
        finite = self.sigma[np.isfinite(self.sigma)]
        high = max(1.05, float(finite.max())*1.1 if len(finite) else 1.05)
        y1 = lambda v: upper.bottom()-upper.height()*v/high
        finite_ranks = self.ranks[np.isfinite(self.ranks)]
        low_rank = min(4.5, float(finite_ranks.min())-.5) if len(finite_ranks) else 4.5
        y2 = lambda v: lower.bottom()-lower.height()*(v-low_rank)/(6.5-low_rank)
        for n, (begin, end, label) in enumerate(self.segments):
            c = QtGui.QColor('#b4bec9')
            for area in (upper, lower):
                left, right = max(area.left(), x(begin)), min(area.right(), x(end))
                if right >= left:
                    p.setPen(QtGui.QPen(c, 1, QtCore.Qt.DashLine))
                    p.drawLine(QtCore.QPointF(left, area.top()), QtCore.QPointF(left, area.bottom()))
            p.setPen(c)
            if x(end)-x(begin) > 55:
                p.drawText(QtCore.QRectF(x(begin), 88, x(end)-x(begin), 18), QtCore.Qt.AlignCenter, label)
        # Resaltar los intervalos observados que caen bajo el umbral relativo.
        for n in range(len(self.t)-1):
            if np.isfinite(self.margins[n:n+2]).all() and min(self.margins[n:n+2]) <= self.threshold:
                p.fillRect(QtCore.QRectF(x(self.t[n]), upper.top(), max(1., x(self.t[n+1])-x(self.t[n])), upper.height()),
                           QtGui.QColor(255, 185, 90, 35))
        for area, ticks, yf in ((upper, np.linspace(0, high, 5), y1),
                                (lower, range(max(0, int(np.ceil(low_rank))), 7), y2)):
            for v in ticks:
                yy = yf(v)
                p.setPen(QtGui.QPen(QtGui.QColor('#dde3e9'), 1))
                p.drawLine(QtCore.QPointF(area.left(), yy), QtCore.QPointF(area.right(), yy))
                p.setPen(QtGui.QColor('#586572'))
                label = f'{v:g}' if area == upper else f'{v} / 6'
                p.drawText(QtCore.QRectF(12, yy-10, 94, 20), QtCore.Qt.AlignRight, label)
        def curve(values, yf, area, color, style, step=False):
            p.save(); p.setClipRect(area.adjusted(-2,-2,2,2))
            path, previous = QtGui.QPainterPath(), None
            for t, v in zip(self.t, values):
                if not np.isfinite(v):
                    previous = None; continue
                point = QtCore.QPointF(x(t), yf(v))
                if previous is None: path.moveTo(point)
                else:
                    if step: path.lineTo(QtCore.QPointF(point.x(), previous.y()))
                    path.lineTo(point)
                previous = point
            p.setPen(QtGui.QPen(QtGui.QColor(color), 2, style)); p.drawPath(path)
            if len(self.t) == 1 and np.isfinite(values[0]):
                p.setBrush(QtGui.QColor(color)); p.drawEllipse(QtCore.QPointF(x(self.t[0]),yf(values[0])),3.,3.)
            if step:
                for t,v in zip(self.t,values):
                    if np.isfinite(v) and v < 6:
                        p.setBrush(QtGui.QColor('#d12f2f')); p.drawEllipse(QtCore.QPointF(x(t),yf(v)),3.,3.)
            p.restore()
        curve(self.sigma,y1,upper,'#dd3030',QtCore.Qt.SolidLine)
        curve(self.wrist,y1,upper,'#168de2',QtCore.Qt.DotLine)
        curve(self.ranks,y2,lower,'#2c853a',QtCore.Qt.SolidLine,step=True)
        p.setPen(QtGui.QPen(QtGui.QColor('#d93333'),1,QtCore.Qt.DashLine))
        p.drawLine(QtCore.QPointF(upper.left(),y1(0)),QtCore.QPointF(upper.right(),y1(0)))
        for area in (upper,lower):
            p.setPen(QtGui.QPen(QtGui.QColor('#475869'),1,QtCore.Qt.DashLine))
            p.drawLine(QtCore.QPointF(x(self.t[i]),area.top()),QtCore.QPointF(x(self.t[i]),area.bottom()))
            for tick in np.linspace(self.t[0],self.t[-1],7):
                p.drawText(QtCore.QRectF(x(tick)-30,area.bottom()+3,60,18),QtCore.Qt.AlignCenter,f'{tick:.2f}')
        p.setPen(QtGui.QColor('#dd3030'));p.drawText(115,286,'━ σ6 · menor valor singular de J normalizado')
        p.setPen(QtGui.QColor('#168de2'));p.drawText(495,286,'┈ |sin(q5)| · índice de muñeca')
        p.setPen(QtGui.QColor('#a66300'));p.drawText(820,286,'Sombreado: cercanía')
        p.setPen(QtGui.QColor('#253342'))
        p.drawText(115,429,'Tiempo acumulado [s] · Curva inferior: rango numérico de J · Rojo: muestras con pérdida de rango')
        p.drawText(115,448,'σ6 → 0 indica singularidad. |sin(q5)| solo informa sobre la muñeca; no descarta codo u hombro.')
