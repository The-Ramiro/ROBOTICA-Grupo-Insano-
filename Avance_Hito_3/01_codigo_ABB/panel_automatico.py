#!/usr/bin/env python3
"""Control automático explícitamente habilitado por sesión."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
import sys
import time

from PyQt5 import QtCore, QtWidgets

from control_automatico import AutomaticSession
from ejecutar_rws import check_goal
from panel_sombra import GoalPanel
from conexion_dialogo import connection_settings
from abb_real import Reader


class AutomaticPanel(GoalPanel):
    def __init__(self, preview, *, send_on_edit=True):
        self.send_on_edit = send_on_edit
        self.auto_armed = False
        self.auto_session = None
        self.auto_task = None
        self.auto_operation = None
        self.stop_requested = False
        self.closing = False
        self.auto_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='abb-automatico')
        self.next_status = 0.
        self.execution_fault = None
        self.cleanup_note = ''
        super().__init__(preview)
        self.setWindowTitle('IMT 342 · ABB IRB 120 · AUTOMÁTICO')
        layout = self.status.parentWidget().layout()
        controls = QtWidgets.QHBoxLayout()
        self.connection_controls = controls
        self.start_auto = QtWidgets.QPushButton('Conectar')
        self.start_auto.clicked.connect(self.connect_reader)
        self.stop_auto = QtWidgets.QPushButton('Detener')
        self.stop_auto.setEnabled(False)
        self.stop_auto.clicked.connect(self.stop_automatic)
        controls.addWidget(self.start_auto, 1)
        controls.addWidget(self.stop_auto)
        layout.insertLayout(0, controls)
        self.auto_label = QtWidgets.QLabel('Automático deshabilitado')
        self.auto_label.setWordWrap(True)
        layout.insertWidget(1, self.auto_label)
        self.failure_box = QtWidgets.QFrame()
        failure_layout = QtWidgets.QVBoxLayout(self.failure_box)
        self.failure_notice = QtWidgets.QLabel()
        self.failure_notice.setWordWrap(True)
        self.failure_notice.setStyleSheet('background:#fff0ed;color:#8b1919;font-weight:bold;padding:7px;')
        failure_layout.addWidget(self.failure_notice)
        buttons = QtWidgets.QHBoxLayout()
        details = QtWidgets.QPushButton('Detalle del fallo')
        details.clicked.connect(self.show_failure_details)
        buttons.addWidget(details)
        acknowledge = QtWidgets.QPushButton('Entendido')
        acknowledge.clicked.connect(self.acknowledge_failure)
        buttons.addWidget(acknowledge)
        failure_layout.addLayout(buttons)
        layout.insertWidget(2, self.failure_box)
        self.failure_box.hide()
        for button in self.findChildren(QtWidgets.QPushButton):
            if button.text() == 'Conectar lectura del ABB':
                button.hide()
        if self.send_on_edit:
            for slider in self.sliders:
                slider.sliderReleased.connect(self.execute_selected_goal)
            for field in self.angles:
                field.lineEdit().returnPressed.connect(self.execute_selected_goal)
        for field in self.xyz:
            field.setToolTip('Enter envía el destino de IK cuando la sesión automática está habilitada.')

    def connect_reader(self):
        if self.auto_armed or self.auto_task is not None or self.motion is not None:
            self.status.setText('Detén la sesión automática antes de cambiar la conexión.')
            return
        host, user = '192.168.125.1', 'Default User'
        if self.reader is not None:
            host = self.reader.base.removeprefix('http://')
            user = self.reader.credentials[0]
        settings = connection_settings(self, host, user)
        if settings is None:
            return
        host, user, password, filename = settings
        if self.reader is not None:
            self.reader.stop.set()
        previous = self.auto_session
        self.reader = Reader(host, user, password)
        self.reader.start()
        self.motion_cancel.clear()
        self.commit_xyz_request = None
        self.pending_xyz = None
        self.xyz_timer.stop()
        self.invalidate()
        self.auto_session = AutomaticSession(
            self.reader.base, self.reader.credentials, self.model, self.envelope,
            deepcopy(self.config), self.motion_cancel, self.motion_message.emit)
        session = self.auto_session
        def prepare():
            if previous is not None:
                previous.close()
            return session.connect(filename)
        self.auto_operation = 'connect'
        self.auto_task = self.auto_pool.submit(prepare)
        self.start_auto.setEnabled(False)
        self.stop_auto.setEnabled(True)
        self.tabs.setEnabled(False)
        self.reviewing = True
        self.auto_label.setText('Conectando y preparando el controlador…')

    def current_measurement(self):
        if self.reader is None:
            raise ValueError('Conecta primero la lectura del ABB.')
        _, q, stamp, error, *_ = self.reader.snapshot()
        if q is None or error or time.monotonic()-stamp > 1.:
            raise ValueError('Se necesita una lectura reciente del ABB.')
        return list(q)

    def start_automatic(self):
        self.connect_reader()

    def failure_text(self):
        data = self.execution_fault or {}
        point, confirmed, total = data.get('point'), data.get('confirmed_point'), data.get('total')
        heading = 'Ejecución incompleta' if data.get('during_motion') else 'Sesión interrumpida'
        if data.get('route') is False:
            heading += ' · destino de Control'
        else:
            if point is not None:
                heading += f' · hacia P{point}'+(f' de {total}' if total else '')
            if confirmed is not None:
                heading += f' · último confirmado: P{confirmed}' if confirmed else ' · ningún punto nuevo confirmado'
        return heading+'\n'+data.get('message', 'Resultado sin confirmar.')+('\n'+self.cleanup_note if self.cleanup_note else '')

    def report_failure(self, error, *, during_motion=False):
        if self.execution_fault is not None:
            return
        details = getattr(self.auto_session, 'last_failure', None) or getattr(error, 'details', {})
        self.execution_fault = dict(details, during_motion=during_motion)
        self.execution_fault.setdefault('message', str(error))
        self.execution_fault.setdefault('time', time.strftime('%Y-%m-%d %H:%M:%S'))
        self.cleanup_note = 'Se solicita Stop. No se reanuda ni se reintenta automáticamente.'
        self.failure_notice.setText(self.failure_text())
        self.failure_box.show()

    def acknowledge_failure(self):
        self.execution_fault = None
        self.cleanup_note = ''
        self.failure_box.hide()
        self.status.setText('Aviso confirmado. Revisa la causa y pulsa Conectar para preparar una nueva ejecución.')

    def show_failure_details(self):
        self.failure_dialog = QtWidgets.QMessageBox(self)
        self.failure_dialog.setWindowTitle('Detalle de la ejecución interrumpida')
        self.failure_dialog.setIcon(QtWidgets.QMessageBox.Warning)
        self.failure_dialog.setText(self.failure_text())
        self.failure_dialog.setDetailedText(json.dumps(self.execution_fault, indent=2, ensure_ascii=False))
        self.failure_dialog.show()

    def stop_automatic(self):
        self.auto_armed = False
        self.motion_cancel.set()
        self.stop_requested = self.auto_session is not None
        self.commit_xyz_request = None
        self.pending_xyz = None
        self.xyz_timer.stop()
        self.invalidate()
        self.start_auto.setEnabled(False)
        self.stop_auto.setEnabled(False)
        self.auto_label.setText('Deteniendo sesión…')

    def execute_selected_goal(self):
        if not self.auto_armed:
            self.status.setText('Pulsa Conectar para preparar el ABB antes de ejecutar.')
            return
        if (self.motion is not None or self.future is not None or self.xyz_dirty
                or self.pending_xyz is not None or self.tip_target.dragging or self.reviewing):
            return
        try:
            start = self.current_measurement()
            target = list(self.goal())
            if target != self.rendered:
                raise ValueError('El destino y la sombra visible difieren.')
            if max(abs(a-b) for a, b in zip(start, target)) <= .005:
                return
            check_goal(self.model, self.envelope, self.config, start, target)
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        self.tabs.setEnabled(False)
        self.reviewing = True
        from referencia_recorrido import begin_recording
        begin_recording(self,start,[target])
        self.motion = self.auto_pool.submit(self.auto_session.execute, start, target)
        if hasattr(self,'recorder'):self.recorder.execution=self.motion
        self.status.setText('Verificando destino y publicando orden automática…')

    def tick(self):
        if self.auto_task is not None and self.auto_task.done():
            task, operation = self.auto_task, self.auto_operation
            self.auto_task = None
            try:
                result = task.result()
                if operation in ('start', 'connect') and not self.stop_requested:
                    self.auto_armed = True
                    self.set_goal(result)
                    self.xyz_dirty = False
                    self.auto_label.setText('Conectado · AUTO listo')
                    self.next_status = time.monotonic()+1.
                    self.execution_fault = None
                    self.cleanup_note = ''
                    self.failure_box.hide()
                elif operation == 'stop':
                    self.stop_requested = False
                    self.auto_label.setText('Sesión detenida')
                    self.status.setText('RAPID detenido por la PC.')
                    if self.execution_fault is not None:
                        self.cleanup_note = 'RAPID detenido y postura estable confirmados.'
            except Exception as exc:
                self.auto_armed = False
                self.motion_cancel.set()
                self.auto_label.setText('Automático deshabilitado')
                if operation == 'stop' and self.execution_fault is not None:
                    prefix = 'RAPID detenido; limpieza pendiente: ' if getattr(self.auto_session, 'stop_verified', False) else 'Stop sin confirmar: '
                    self.cleanup_note = prefix+str(exc)
                elif operation in ('start', 'connect') and self.execution_fault is not None:
                    self.cleanup_note = 'Nueva conexión no completada: '+str(exc)
                else:
                    self.status.setText(str(exc))
                if operation == 'status':
                    self.report_failure(exc)
                    self.stop_requested = True
                if operation in ('start', 'connect', 'stop'):
                    self.stop_requested = False
            if operation in ('start', 'connect', 'stop'):
                self.tabs.setEnabled(True)
                self.reviewing = False
        if self.motion is not None and self.motion.done():
            if self.motion.exception() is not None:
                self.report_failure(self.motion.exception(), during_motion=True)
                self.auto_armed = False
                self.motion_cancel.set()
                self.stop_requested = True
                self.auto_label.setText('Orden interrumpida · revisa el ABB antes de iniciar otra sesión')
            self.reviewing = False
        super().tick()
        if self.stop_requested and self.auto_task is None and self.motion is None:
            self.auto_operation = 'stop'
            self.auto_task = self.auto_pool.submit(self.auto_session.stop)
        if (self.auto_armed and self.auto_task is None and self.motion is None
                and time.monotonic() >= self.next_status):
            try:
                self.current_measurement()
            except ValueError as exc:
                self.stop_automatic()
                self.status.setText(str(exc))
            else:
                self.auto_operation = 'status'
                self.auto_task = self.auto_pool.submit(self.auto_session.ready)
                self.next_status = time.monotonic() + 1.
        busy = self.auto_task is not None or self.motion is not None or self.stop_requested
        self.start_auto.setEnabled(not self.auto_armed and not busy)
        self.stop_auto.setEnabled(self.auto_armed or busy)
        if self.closing and not busy:
            QtCore.QTimer.singleShot(0, self.close)
            self.closing = False
        if self.execution_fault is not None:
            message = self.failure_text()
            self.failure_notice.setText(message)
            self.auto_label.setText('Ejecución interrumpida · revisa el detalle antes de volver a conectar')
            self.status.setText(message)

    def closeEvent(self, event):
        if self.auto_armed or self.auto_task is not None or self.motion is not None or self.stop_requested:
            self.closing = True
            self.stop_automatic()
            event.ignore()
            return
        self.auto_pool.shutdown(wait=False)
        if self.auto_session is not None:
            self.auto_session.close()
        super().closeEvent(event)


def main():
    from panel_robotica import main as robotics_main
    return robotics_main()


if __name__ == '__main__':
    sys.exit(main())
