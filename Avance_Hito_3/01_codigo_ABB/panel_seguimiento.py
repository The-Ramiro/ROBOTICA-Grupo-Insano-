"""Seis curvas: referencia completa y medición progresiva."""
import time
import numpy as np
from PyQt5 import QtCore,QtWidgets
from grafica_informe import ReportPlot
from referencia_recorrido import align_start, comparison_curves, inferred_start, observed_start, timing_caption

class TrackingPage(QtWidgets.QWidget):
    def __init__(self,analysis,trajectory=False):
        super().__init__()
        self.analysis=analysis;self.panel=analysis.panel
        self.next_refresh=0.;self.key=None;self.trajectory=trajectory
        self.cleared_session=None;self.cleared_until=None
        root=QtWidgets.QVBoxLayout(self)
        bar=QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel('Gris: LSPB estimado completo · naranja: ABB en vivo. Los tiempos pueden diferir.'))
        self.clear_orange=QtWidgets.QPushButton('Limpiar curva naranja')
        self.clear_orange.setToolTip('Oculta las lecturas mostradas en las seis gráficas; conserva los archivos del registro.')
        self.clear_orange.clicked.connect(self.clear_measured)
        bar.addWidget(self.clear_orange)
        self.variable=QtWidgets.QComboBox()
        self.variable.addItem('Posición','q');self.variable.addItem('Velocidad','qd');self.variable.addItem('Aceleración','qdd')
        self.variable.currentIndexChanged.connect(self.refresh)
        if trajectory:bar.addWidget(self.variable)
        else:self.variable.hide()
        root.addLayout(bar)
        self.status=QtWidgets.QLabel('Esperando recorrido.');root.addWidget(self.status)
        self.scroll=QtWidgets.QScrollArea(widgetResizable=True)
        body=QtWidgets.QWidget();body.setMinimumWidth(1020);body.setMinimumHeight(760)
        QtWidgets.QGridLayout(body);self.scroll.setWidget(body);root.addWidget(self.scroll,1)
        self.footer=QtWidgets.QLabel('');self.footer.setWordWrap(True);root.addWidget(self.footer)

    def poll(self):
        if self.isVisible() and time.monotonic()>=self.next_refresh:
            self.next_refresh=time.monotonic()+.5;self.refresh()

    def clear_measured(self):
        r=self.panel.recorder
        self.cleared_session=r.session_id
        self.cleared_until=r.samples[-1]['t'] if r.samples else None
        self.key=None
        self.refresh()

    def refresh(self):
        r=self.panel.recorder;rows=list(r.samples)
        if self.cleared_session==r.session_id and self.cleared_until is not None:
            rows=[row for row in rows if row['t']>self.cleared_until]
        preview=getattr(self.analysis,'preview_profile',None)
        using_preview=getattr(self.analysis,'preview_requested',False) and not r.active
        reference=preview if using_preview else r.reference or (preview if not rows else None)
        recorded_reference = reference if reference is r.reference and not using_preview else None
        onset = observed_start(recorded_reference, r.samples)
        start_label = 'inicio observado'
        if onset is None:
            onset = inferred_start(recorded_reference, r.samples)
            start_label = 'inicio estimado tras pausa de lectura'
        self.clear_orange.setEnabled(bool(rows))
        variable=self.variable.currentData()
        key=(r.session_id,len(rows),rows[-1]['t'] if rows else None,id(reference),variable,r.state,using_preview,self.cleared_until,onset,start_label)
        self.status.setText((f"Vista previa · {reference['speed']:g}% · {reference['t'][-1]:.2f} s estimados"
                            +(' · naranja: registro anterior' if rows else '') if using_preview and reference else
                            'Sin perfil previsto · selecciona una velocidad mayor que 0%.' if using_preview else
                            r.status_text()+(' · sin perfil guardado' if rows and reference is None else '')))
        timing = timing_caption(reference)
        if timing:
            percent = '' if using_preview else f" · {reference['speed']:g}%"
            self.status.setText(self.status.text()+percent+' · '+timing)
        if onset is not None:
            self.status.setText(self.status.text()+f' · {start_label} {onset:.2f} s')
        if key==self.key:return
        self.key=key
        reference = align_start(reference, onset)
        data=comparison_curves(reference,rows,variable)
        if using_preview:
            # El registro anterior no debe prolongar el eje temporal de una ruta nueva.
            end=reference['t'][-1] if reference else 0.
            keep=data['t']<=end+1e-9
            for name in ('t','expected','actual'):data[name]=data[name][keep]
        layout=self.scroll.widget().layout()
        while layout.count():
            item=layout.takeAt(0)
            if item.widget() is not None:item.widget().deleteLater()
        names={'q':('posición','°'),'qd':('velocidad','°/s'),'qdd':('aceleración','°/s²')}
        title,unit=names[variable]
        for j in range(6):
            plot=ReportPlot(f'J{j+1} · {title}',unit,data['t'],
                np.column_stack((data['expected'][:,j],data['actual'][:,j])),['Previsto','ABB'],marks=data['marks'])
            plot.colors=('#7a8088','#ea8517');plot.setFixedHeight(245)
            layout.addWidget(plot,j//2,j%2)
        if not self.trajectory and rows and reference and not using_preview:
            rt=rows[-1]['t'];q=np.asarray(rows[-1]['q'])
            desired=np.array([np.interp(rt,reference['t'],np.asarray(reference['q'])[:,j]) for j in range(6)])
            self.footer.setText(f'Δ respecto al perfil estimado · t={rt:.2f}s · '+
                ' · '.join(f'J{j+1}: {desired[j]-q[j]:+.5f}°' for j in range(6)))
        else:self.footer.setText('Curva naranja del registro anterior; sus tiempos medidos no se modifican.' if using_preview and rows else
                                 'Velocidad y aceleración ABB: derivadas de las lecturas.' if variable!='q' else '')
