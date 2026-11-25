"""Gráficas de dinámica 6R; trabajo numérico separado del hilo de control."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor, CancelledError
from threading import Event
import time
import numpy as np
from PyQt5 import QtCore, QtWidgets
from calculos_robotica import Dynamics
from dinamica_trayectoria import planned_dynamics, measured_dynamics, reconstruct_dynamics
from grafica_informe import ReportPlot


class DynamicsPage(QtWidgets.QWidget):
    progress = QtCore.pyqtSignal(str)

    def __init__(self, analysis):
        super().__init__()
        self.analysis, self.panel = analysis, analysis.panel
        self.dynamic = Dynamics(self.panel.model)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='dinamica-6r')
        self.future = None
        self.live_future = None
        self.live_pool = ThreadPoolExecutor(max_workers=1,thread_name_prefix='dinamica-real')
        self.live_session = None
        self.rebuilt_session=None
        self.rebuild_cancel=Event()
        self.live_error=None
        self.cancelled = Event()
        self.planned = self.report_source = None
        self.live_rows = deque(maxlen=36000)
        self.last_stamp = None
        self.next_live = 0.
        self.latest_note = 'Sin lecturas ABB.'
        root = QtWidgets.QVBoxLayout(self)
        note = QtWidgets.QLabel('Dinámica estimada · naranja: inercia · violeta: gravedad · azul: Coriolis · negro: par total [N·m].')
        note.setToolTip('τ=M(q)q̈+c(q,q̇)+g(q). Masas e inercias del URDF; sin fricción, reductores ni carga. No son pares medidos por ABB.')
        note.setWordWrap(True); root.addWidget(note)
        bar = QtWidgets.QHBoxLayout()
        self.mode = QtWidgets.QComboBox()
        self.mode.addItems(['Propuesta · LSPB estimado (fuera de sesión)','Recorrido actual · dinámica de lecturas ABB'])
        self.mode.setCurrentIndex(1)
        self.mode.currentIndexChanged.connect(self.render)
        bar.addWidget(self.mode)
        bar.addWidget(QtWidgets.QLabel('Una sesión por recorrido'))
        self.joint = QtWidgets.QComboBox(); self.joint.addItem('Todas las articulaciones',None)
        for j in range(6):self.joint.addItem(f'J{j+1}',j)
        self.joint.currentIndexChanged.connect(self.render); bar.addWidget(self.joint)
        refresh = QtWidgets.QPushButton('Recalcular propuesta')
        refresh.clicked.connect(self.recalculate);bar.addWidget(refresh)
        for i in range(bar.count()):
            if bar.itemAt(i).widget() is not None:bar.itemAt(i).widget().hide()
        root.addLayout(bar)
        self.status = QtWidgets.QLabel('Calcula un recorrido y abre esta pestaña.')
        self.status.setWordWrap(True);root.addWidget(self.status)
        self.progress.connect(self.status.setText)
        self.scroll = QtWidgets.QScrollArea(widgetResizable=True)
        body = QtWidgets.QWidget();body.setMinimumWidth(1020);QtWidgets.QGridLayout(body)
        self.scroll.setWidget(body);root.addWidget(self.scroll,1)
        self.readout = QtWidgets.QLabel('Los valores se muestran en N·m. Pasa el cursor por cada curva para consultar datos.')
        self.readout.setWordWrap(True);root.addWidget(self.readout)

    def recalculate(self):
        if self.future is not None:
            self.status.setText('Espera a que termine el cálculo actual.');return
        self.report_source = None
        self.planned = None
        self.clear_plots()
        self.analysis.calculate()

    def showEvent(self,event):
        super().showEvent(event)
        self.poll_live()
        self.render()

    def poll(self):
        self.poll_live()
        if self.future is not None and self.future.done():
            future,self.future = self.future,None
            try:
                result = future.result()
                if self.kind == 'planned':
                    self.planned = result
                self.render()
            except CancelledError:pass
            except Exception as exc:
                self.status.setText('No se pudo estimar la dinámica: '+str(exc))
        if not self.isVisible() or self.future is not None:
            return
        if self.mode.currentIndex() == 0:
            if self.analysis.future is not None:return
            data = self.analysis.result
            if data is None:
                self.status.setText('Pulsa Calcular recorrido completo para analizar la postura o trayectoria propuesta.');return
            if data is not self.report_source:
                self.report_source = data
                self.cancelled = Event();self.kind = 'planned'
                self.status.setText('Calculando dinámica de las seis articulaciones…')
                self.future = self.pool.submit(planned_dynamics,self.dynamic,data,self.cancelled,self.progress.emit)
    def poll_live(self):
        recorder=self.panel.recorder
        if self.live_session != recorder.session_id:
            self.rebuild_cancel.set();self.rebuild_cancel=Event()
            self.rebuilt_session=None;self.live_error=None
            self.live_session=recorder.session_id
            self.live_rows.clear();self.last_stamp=None
            for row in recorder.dynamic_samples:
                self.live_rows.append(dict(row,run=1,moving=True,point=row.get('point')))
            if self.live_rows:self.last_stamp=self.live_rows[-1]['t']
            if self.isVisible() and self.mode.currentIndex()==1:self.render()
        if getattr(self,'live_state',None)!=recorder.state:
            self.live_state=recorder.state
            if self.isVisible() and self.mode.currentIndex()==1:self.render()
        if self.live_future is not None and self.live_future.done():
            future,self.live_future=self.live_future,None
            try:
                result=future.result()
                if self.live_future_session==recorder.session_id:
                    frames=result if isinstance(result,list) else [result]
                    merged={r['t']:r for r in self.live_rows}
                    merged.update({r['t']:r for r in frames})
                    self.live_rows.clear();self.live_rows.extend(merged[t] for t in sorted(merged))
                    recorder.dynamic_samples.clear()
                    for frame in self.live_rows:
                        recorder.dynamic_samples.append({k:(v.tolist() if isinstance(v,np.ndarray) else v)
                            for k,v in frame.items() if k in ('t','point','moving','run','inertia','g','c','tau')})
                    self.live_error=None if self.live_rows else 'No hay suficientes lecturas continuas para estimar la dinámica.'
                    if not recorder.active:recorder.archive()
                    if self.isVisible() and self.mode.currentIndex()==1:self.render()
            except Exception as exc:
                if self.live_future_session==recorder.session_id:
                    self.live_error=str(exc)
                    if self.isVisible() and self.mode.currentIndex()==1:self.status.setText('Dinámica no disponible: '+str(exc))
        if self.live_future is not None or time.monotonic()<self.next_live:return
        self.next_live=time.monotonic()+.25
        rows=list(recorder.samples)
        if not recorder.active and len(rows)>=5 and self.rebuilt_session!=recorder.session_id:
            self.rebuilt_session=recorder.session_id
            self.live_future_session=recorder.session_id
            self.live_future=self.live_pool.submit(reconstruct_dynamics,self.dynamic,rows,self.rebuild_cancel,self.progress.emit)
            return
        if not rows or rows[-1]['t']==self.last_stamp:return
        self.last_stamp=rows[-1]['t']
        from dinamica_trayectoria import measured_derivatives
        try:measured_derivatives(rows[-30:])
        except ValueError as exc:
            self.live_error=str(exc)
            if self.isVisible() and self.mode.currentIndex()==1:self.status.setText(str(exc))
            return
        self.live_future_session=recorder.session_id
        self.live_future=self.live_pool.submit(measured_dynamics,self.dynamic,rows[-30:])

    def render(self):
        live = self.mode.currentIndex() == 1
        if live:
            rows = list(self.live_rows)
            if not rows:
                self.clear_plots()
                self.status.setText(self.live_error or 'Esperando lecturas suficientes para estimar la dinámica.');return
            t = np.array([r['t'] for r in rows])
            data = {key:np.array([r[key] for r in rows],dtype=float) for key in ('inertia','g','c','tau')}
            gaps = np.r_[False,np.diff(t) > .75]
            for values in data.values(): values[gaps] = np.nan
            marks,previous = [],None
            for stamp,r in zip(t,rows):
                if r['moving'] and r['point'] is not None and r['point'] != previous:marks.append((stamp,r['point']))
                previous = r['point']
            phases = []
            if t[0]>0.:
                t=np.r_[0.,t]
                for key in data:data[key]=np.vstack((np.full((1,6),np.nan),data[key]))
            self.status.setText(self.panel.recorder.status_text())
        else:
            if self.planned is None:
                self.clear_plots()
                self.status.setText('Esperando el cálculo dinámico del recorrido propuesto.');return
            data = self.planned;t = data['t']
            marks,phases = self.report_source['point_times'],self.report_source['phases']
            self.status.setText(f'RECORRIDO PROPUESTO · {len(t)} muestras dinámicas · tiempos LSPB estimados, no perfil interno ABB. '
                'Verde: aceleración; rojo: desaceleración. Se preservan muestras próximas a cambios de fase y a los puntos.')
        self.clear_plots()
        layout = self.scroll.widget().layout()
        joint = None
        self.scroll.widget().setMinimumHeight(760)
        axes = range(6) if joint is None else [joint]
        for n,j in enumerate(axes):
            values = np.column_stack([data[key][:,j] for key in ('inertia','g','tau','c')])
            plot = ReportPlot(f'J{j+1} · dinámica estimada', 'N·m', t, values,
                ['Inercia','Gravedad','Total','Coriolis'],phases=phases,marks=marks)
            plot.colors=('#ed8a13','#9860be','#141414','#267da8')
            if joint is not None:plot.setMinimumHeight(440);plot.setMaximumHeight(440)
            layout.addWidget(plot,n//2 if joint is None else n,n%2 if joint is None else 0)
        last = np.flatnonzero(np.all(np.isfinite(data['tau']),axis=1))
        if len(last):
            i=last[-1]
            self.readout.setText(f't={t[i]:.3f}s · Par total estimado [N·m]: '+
                ' · '.join(f'J{j+1}: {data["tau"][i,j]:.5f}' for j in range(6)))

    def clear_plots(self):
        layout = self.scroll.widget().layout()
        while layout.count():
            item=layout.takeAt(0)
            if item.widget() is not None:item.widget().deleteLater()

    def shutdown(self):
        self.cancelled.set()
        self.rebuild_cancel.set()
        self.pool.shutdown(wait=False,cancel_futures=True)
        self.live_pool.shutdown(wait=False,cancel_futures=True)
