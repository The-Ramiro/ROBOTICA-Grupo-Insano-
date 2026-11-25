"""Perfiles de las seis articulaciones con sus colores originales."""
import time
import numpy as np
from PyQt5 import QtWidgets
from grafica_informe import ReportPlot
from referencia_recorrido import comparison_curves, timing_caption


class TrajectoryPage(QtWidgets.QWidget):
    def __init__(self,analysis):
        super().__init__()
        self.analysis=analysis;self.panel=analysis.panel
        self.next_refresh=0.;self.key=None
        root=QtWidgets.QVBoxLayout(self)
        self.variable=QtWidgets.QTabBar()
        self.variable.addTab('Lecturas ABB en vivo')
        self.variable.setTabData(0,'actual')
        self.variable.addTab('Perfil LSPB previsto')
        self.variable.setTabData(1,'expected')
        self.variable.setExpanding(False)
        self.variable.currentChanged.connect(self.refresh)
        root.addWidget(self.variable)
        self.status=QtWidgets.QLabel();root.addWidget(self.status)
        self.scroll=QtWidgets.QScrollArea(widgetResizable=True)
        body=QtWidgets.QWidget();body.setMinimumWidth(1020);body.setMinimumHeight(760)
        QtWidgets.QVBoxLayout(body);self.scroll.setWidget(body);root.addWidget(self.scroll,1)

    def poll(self):
        if self.isVisible() and time.monotonic()>=self.next_refresh:
            self.next_refresh=time.monotonic()+.5;self.refresh()

    def refresh(self):
        r=self.panel.recorder;rows=list(r.samples)
        source=self.variable.tabData(self.variable.currentIndex())
        preview=getattr(self.analysis,'preview_profile',None)
        using_preview=(source=='expected' and getattr(self.analysis,'preview_requested',False) and not r.active)
        ref=preview if using_preview else r.reference or (preview if not rows else None)
        caption=(f"Vista previa · {ref['speed']:g}% · LSPB estimado" if source=='expected' and ref is preview and preview else
                 r.status_text()+(' · velocidad y aceleración derivadas' if source=='actual' else ' · LSPB estimado'))
        self.status.setText(caption)
        timing = timing_caption(ref) if source=='expected' else ''
        if timing:
            percent = '' if ref is preview else f" · {ref['speed']:g}%"
            self.status.setText(caption+percent+' · '+timing)
        if using_preview and ref is None:
            self.status.setText('Sin perfil previsto · selecciona una velocidad mayor que 0%.')
        key=(r.session_id,len(rows),rows[-1]['t'] if rows else None,id(ref),source,r.state)
        if key==self.key:return
        self.key=key
        layout=self.scroll.widget().layout()
        while layout.count():
            item=layout.takeAt(0)
            if item.widget() is not None:item.widget().deleteLater()
        for variable,title,unit in (('q','Posición articular','rad'),('qd','Velocidad articular','rad/s'),('qdd','Aceleración articular','rad/s²')):
            data=comparison_curves(ref,rows if source=='actual' else [],variable)
            plot=ReportPlot(title,unit,data['t'],np.radians(data[source]),[f'J{j+1}' for j in range(6)],
                phases=ref.get('phases',[]) if ref and source=='expected' else [],marks=data['marks'])
            plot.setFixedHeight(245);layout.addWidget(plot)
