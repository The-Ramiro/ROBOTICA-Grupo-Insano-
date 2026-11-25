"""Sesión RWS automática. Todas las operaciones se serializan desde el panel."""
from datetime import datetime
import json
import math
from pathlib import Path
import time

import requests
from requests.auth import HTTPDigestAuth

from ejecutar_rws import check_goal, POSTURE_TOL_DEG, TCP_PATH
from limites_movimiento import effective
from fallos_ejecucion import ExecutionFault, ProgressWatchdog, failure_message
from irb120_control.rws_j6 import (
    Client, literal, measured_target, parse_target, same_target, single, source_fingerprint, error_details,
)
from irb120_control.rws_mover import FORM, logout

MODULE = 'IMT342_Automatico'
ROUTINE = 'ServidorAutomatico'
TASK = '/rw/rapid/tasks/T_ROB1'
SYMBOL = f'/rw/rapid/symbol/data/RAPID/T_ROB1/{MODULE}/'
MODULE_PATH = f'/rw/rapid/modules/{MODULE}'
SYMBOLS = ('autoInicio', 'autoDestino', 'autoOrden', 'autoHecho',
           'autoEstado', 'autoActivo', 'autoFin', 'autoSesion',
           'autoCantidad', 'autoRuta', 'autoTramo')
SYMBOLS += ('autoUltimo', 'autoErrorEje', 'autoErrorDelta', 'autoErrorRAPID')


class AutomaticSession:
    def __init__(self, base, credentials, model, envelope, config, cancelled, notify):
        self.base = base
        self.model, self.envelope, self.config = model, envelope, config
        self.cancelled, self.notify = cancelled, notify
        self.session = requests.Session()
        self.session.trust_env = False
        self.session.auth = HTTPDigestAuth(*credentials)
        self.client = Client(self.session, base, timeout=1.5)
        self.client.extra_reads.update(
            {MODULE_PATH, TASK+'/pcp', TASK+'/program/builderror', TCP_PATH, '/rw/panel/speedratio', '/rw/elog/0'}
            | {SYMBOL+name for name in SYMBOLS})
        self.session_number = None
        self.mastership = False
        self.progress = {'point': None, 'total': 0, 'state': 'idle'}
        self.last_failure = None
        self.stop_verified = False
        folder = Path.home()/'abb-real'/'ordenes'
        folder.mkdir(parents=True, exist_ok=True)
        self.log = folder/datetime.now().strftime('automatico_%Y%m%d_%H%M%S_%f.jsonl')

    def record(self, event, **values):
        with self.log.open('a', encoding='utf-8') as output:
            output.write(json.dumps({'time': datetime.now().isoformat(), 'event': event, **values})+'\n')

    def post(self, path, action, data=None, *, stopping=False):
        if self.cancelled.is_set() and not stopping:
            raise ValueError('Sesión deshabilitada. Iníciala de nuevo para enviar órdenes.')
        self.record('post_attempt', path=path, action=action, data=data)
        response = self.session.post(
            self.base+path, params={'action': action}, data=data, headers=FORM,
            timeout=2., allow_redirects=False)
        self.record('post_response', path=path, action=action, http=response.status_code)
        if response.status_code not in (200, 204):
            detail = error_details(response.text)
            raise ValueError(f'El controlador rechazó {action} (HTTP {response.status_code}). {detail}')

    def acquire(self, *, stopping=False):
        self.post('/rw/mastership/rapid', 'request', stopping=stopping)
        self.mastership = True

    def release(self):
        if self.mastership:
            self.post('/rw/mastership/rapid', 'release', stopping=True)
            self.mastership = False

    def states(self):
        return (
            single(self.client.read('/rw/panel/opmode'), 'opmode'),
            single(self.client.read('/rw/panel/ctrlstate'), 'ctrlstate'),
            single(self.client.read('/rw/rapid/execution'), 'ctrlexecstate'),
        )

    def number(self, name):
        return float(single(self.client.read(SYMBOL+name), 'value'))

    def prepare_pointer(self, *, stopping=False, take_over=False):
        pointer = next((r for r in self.client.read(TASK+'/pcp')
                        if r.get('_title') == 'progpointer'), {})
        if pointer.get('executiontype') == 'userroutine':
            if pointer.get('modulemame') != MODULE and not take_over:
                raise ValueError('Hay otra rutina de usuario seleccionada. Revisa el ABB.')
            self.post(TASK, 'abortexeclevel', stopping=stopping)
            pointer = next((r for r in self.client.read(TASK+'/pcp')
                            if r.get('_title') == 'progpointer'), {})
        # Tras EXIT o un reinicio puede no existir un contexto al que volver.
        # En ese caso se selecciona la rutina normal, sin iniciar RAPID.
        userlevel = 'true' if pointer.get('routinename') and pointer.get('executiontype') else 'false'
        self.post(TASK+'/pcp', 'set-pp-routine',
                  {'module': MODULE, 'routine': ROUTINE, 'userlevel': userlevel}, stopping=stopping)

    def joints(self):
        try:
            return measured_target(self.client.read(TASK+'/motion', resource='jointtarget'))
        except ValueError as exc:
            if '-1073436650' in str(exc):
                raise ValueError(
                    'ABB: sistema mecánico no preparado. El encargado debe revisar '
                    'la sincronización y los contadores de vueltas en el FlexPendant. '
                    'No se envió ningún destino.') from exc
            raise

    def wait_stopped(self):
        deadline = time.monotonic()+5.
        while single(self.client.read('/rw/rapid/execution'), 'ctrlexecstate') != 'stopped':
            if time.monotonic() >= deadline:
                raise ValueError('El ABB no confirmó RAPID detenido.')
            time.sleep(.1)
        previous, _ = self.joints()
        while True:
            time.sleep(.15)
            current, _ = self.joints()
            if max(abs(a-b) for a, b in zip(previous, current)) <= POSTURE_TOL_DEG:
                return
            if time.monotonic() >= deadline:
                raise ValueError('El brazo sigue cambiando de postura.')
            previous = current

    def verify_module(self):
        self.client.verify_identity()
        modules = self.client.read('/rw/rapid/modules', task='T_ROB1')
        if MODULE not in {row.get('name') for row in modules}:
            raise ValueError('Carga IMT342_Automatico.mod en T_ROB1 y comprueba RAPID.')
        rows = self.client.read(MODULE_PATH, resource='module-text', task='T_ROB1')
        self.client.resolve_module_text(rows, MODULE)
        expected = Path(__file__).with_name('rapid')/f'{MODULE}.mod'
        if source_fingerprint(single(rows, 'module-text')) != source_fingerprint(expected.read_text(encoding='ascii')):
            raise ValueError('El módulo automático cargado difiere de la versión revisada.')

    def connect(self, module_file):
        selected = Path(module_file)
        expected = Path(__file__).with_name('rapid')/f'{MODULE}.mod'
        raw = selected.read_bytes()
        if len(raw) > 100_000:
            raise ValueError('El módulo seleccionado es demasiado grande.')
        source = raw.decode('ascii')
        if source_fingerprint(source) != source_fingerprint(expected.read_text(encoding='ascii')):
            raise ValueError('Selecciona IMT342_Automatico.mod de esta versión del panel.')
        self.client.verify_identity()
        mode, motors, execution = self.states()
        if mode != 'AUTO' or motors not in ('motoroff', 'motoron'):
            raise ValueError(f'Prepara AUTO y resuelve la parada del controlador ({mode}/{motors}).')
        if execution == 'running':
            self.notify('Deteniendo RAPID para preparar la conexión…')
            self.post('/rw/rapid/execution', 'stop', {'stopmode': 'stop', 'usetsp': 'alltsk'})
            deadline = time.monotonic()+5.
            while self.states()[2] != 'stopped':
                if self.cancelled.is_set() or time.monotonic() >= deadline:
                    raise ValueError('No se confirmó RAPID detenido.')
                time.sleep(.1)
        elif execution != 'stopped':
            raise ValueError('Estado RAPID no reconocido para cargar el módulo.')
        self.wait_stopped()
        self.acquire()
        try:
            if self.states()[2] != 'stopped':
                raise ValueError('RAPID cambió de estado durante la preparación.')
            pointer = next((r for r in self.client.read(TASK+'/pcp')
                            if r.get('_title') == 'progpointer'), {})
            if pointer.get('executiontype') == 'userroutine':
                self.post(TASK, 'abortexeclevel')
            modules = self.client.read('/rw/rapid/modules', task='T_ROB1')
            replacing = MODULE in {row.get('name') for row in modules}
            matches = False
            if replacing:
                rows = self.client.read(MODULE_PATH, resource='module-text', task='T_ROB1')
                self.client.resolve_module_text(rows, MODULE)
                old_source = single(rows, 'module-text')
                matches = source_fingerprint(old_source) == source_fingerprint(source)
                if not matches:
                    backup = self.log.with_suffix('.anterior.mod')
                    backup.write_text(old_source, encoding='utf-8')
                    self.record('module_backup', file=str(backup))
            if not matches:
                import hashlib
                name = f'{MODULE}_{hashlib.sha256(raw).hexdigest()[:12]}.mod'
                self.notify('Subiendo y cargando el módulo RAPID…')
                response = self.session.put(
                    self.base+'/fileservice/$HOME/'+name, data=raw,
                    headers={'Content-Type': 'application/octet-stream'}, timeout=5., allow_redirects=False)
                if response.status_code not in (200, 201, 204):
                    raise ValueError(f'RobotWare rechazó el archivo (HTTP {response.status_code}).')
                check = self.session.get(self.base+'/fileservice/$HOME/'+name, timeout=3., allow_redirects=False)
                if check.status_code != 200 or check.content != raw:
                    raise ValueError('El archivo subido no coincide con el seleccionado.')
                self.post(TASK, 'loadmod',
                          {'modulepath': '$HOME/'+name, 'replace': 'true' if replacing else 'false'})
            self.post(TASK, 'build')
            errors = self.client.read(TASK+'/program/builderror', start=1, limit=30)
            if errors:
                details = ' / '.join(
                    f'{r.get("ModuleName", "")} fila {r.get("row", "")}: {r.get("error", "")}' for r in errors)
                raise ValueError('RAPID contiene errores de compilación. '+details)
            self.verify_module()
            self.prepare_pointer(take_over=True)
        finally:
            self.release()
        try:
            self.reconcile_stopped_order(take_over=True)
        finally:
            self.release()
        self.notify('Módulo comprobado. Preparando recepción automática…')
        return self.start()

    def ready(self, *, idle=True):
        if self.cancelled.is_set():
            raise ValueError('Sesión deshabilitada.')
        state = self.states()
        if state != ('AUTO', 'motoron', 'running'):
            raise ValueError(f'Sesión interrumpida por el controlador ({"/".join(state)}).')
        pointer = next((r for r in self.client.read(TASK+'/pcp')
                        if r.get('_title') == 'progpointer'), {})
        permitted_routines = (ROUTINE,) if idle else (
            ROUTINE, 'RevisarYPosicionar', 'Coincide', 'CoincideInicio',
            'DentroDeLimites', 'SinEjesExternos', 'GuardarDiferencia')
        if pointer.get('modulemame') != MODULE or pointer.get('routinename') not in permitted_routines:
            raise ValueError('El puntero salió de ServidorAutomatico.')
        if self.number('autoSesion') != self.session_number:
            raise ValueError('RAPID se reinició. Inicia una nueva sesión desde el panel.')
        if self.number('autoActivo') != 1 or self.number('autoFin') != 0:
            raise ValueError('ServidorAutomatico terminó o dejó de aceptar objetivos.')
        if idle and self.number('autoOrden') != self.number('autoHecho'):
            raise ValueError('Hay una orden pendiente. Comprueba su resultado antes de iniciar otra sesión.')
        return state

    def start(self):
        start_attempted = False
        try:
            self.verify_module()
            mode, motors, execution = self.states()
            if mode != 'AUTO':
                raise ValueError('El encargado debe seleccionar AUTO y confirmar el cambio en el FlexPendant.')
            if motors not in ('motoroff', 'motoron') or execution != 'stopped':
                raise ValueError(f'Arranque requiere RAPID detenido y motoroff/motoron ({motors}/{execution}).')
            if self.number('autoOrden') != self.number('autoHecho'):
                raise ValueError('Existe una orden anterior pendiente. Comprueba su resultado en el ABB.')
            q, _ = self.joints()
            bounds = effective(self.config)
            if any(not r['min_deg'] <= a <= r['max_deg'] for a, r in zip(q, bounds)):
                raise ValueError('La postura inicial está fuera de los límites configurados.')
            self.model.check_height(q, self.envelope)
            z = float(single(self.client.read(TCP_PATH, coordinate='Base', tool='tool0', wobj='wobj0'), 'z'))
            if abs(z - self.model.tip(q, self.envelope)[2, 3]*1000) > 10.:
                raise ValueError('La altura ABB y la del modelo difieren más de 10 mm.')
            self.acquire()
            if self.states() != (mode, motors, execution):
                raise ValueError('El estado del ABB cambió durante la preparación.')
            self.prepare_pointer()
            # No heredar el porcentaje de la version lenta al cambiar a vmax.
            # La recepcion puede arrancar, pero Ejecutar exige un ajuste positivo.
            self.post('/rw/panel/speedratio', 'setspeedratio', {'speed-ratio': '0'})
            ratio = float(single(self.client.read('/rw/panel/speedratio'), 'speedratio'))
            if not math.isfinite(ratio) or abs(ratio) > .01:
                raise ValueError('No se confirmó velocidad inicial 0%. RAPID no se inició.')
            self.record('startup_speed_confirmed', ratio_percent=ratio)
            if motors != 'motoron':
                self.post('/rw/panel/ctrlstate', 'setctrlstate', {'ctrl-state': 'motoron'})
            deadline = time.monotonic() + 5.
            while self.states() != ('AUTO', 'motoron', 'stopped'):
                if time.monotonic() >= deadline or self.cancelled.is_set():
                    raise ValueError('No se confirmó Motors On para iniciar la sesión.')
                time.sleep(.1)
            previous_session = self.number('autoSesion')
            start_attempted = True
            self.post('/rw/rapid/execution', 'start', {
                'regain': 'continue', 'execmode': 'continue', 'cycle': 'once',
                'condition': 'none', 'stopatbp': 'enabled', 'alltaskbytsp': 'false',
            })
            deadline = time.monotonic() + 5.
            while True:
                self.session_number = self.number('autoSesion')
                if self.session_number != previous_session:
                    self.ready()
                    break
                if time.monotonic() >= deadline or self.cancelled.is_set():
                    raise ValueError('No se confirmó el arranque de ServidorAutomatico.')
                time.sleep(.1)
            self.record('session_ready', session=self.session_number)
            self.release()
            return list(self.joints()[0])
        except Exception:
            if start_attempted:
                self.attempt_stop()
            raise
        finally:
            self.release()

    def execute(self, confirmed_start, target):
        self.last_failure = None
        self.progress = dict(point=1, total=1, point_map=[1], confirmed_point=0, state='preparing')
        return self._execute(confirmed_start, [target], route=False)

    def execute_route(self, confirmed_start, targets):
        from trayectorias import prepare, points, ROBOT_BUFFER
        # Validar la lista completa antes de publicar el primer bloque.
        original = points(targets)
        targets = prepare(self.model, self.envelope, self.config, confirmed_start, original)
        skipped = len(original)-len(targets)
        self.last_failure = None
        result, errors = None, []
        percent = float(single(self.client.read('/rw/panel/speedratio'), 'speedratio'))
        total = len(original)
        confirmed = skipped
        for offset in range(0, len(targets), ROBOT_BUFFER):
            if self.cancelled.is_set():
                raise ValueError('Secuencia cancelada antes del siguiente bloque.')
            if float(single(self.client.read('/rw/panel/speedratio'), 'speedratio')) != percent:
                self.attempt_stop()
                raise ValueError('La velocidad cambió entre bloques. Secuencia detenida.')
            point_map = list(range(offset+skipped+1, min(offset+ROBOT_BUFFER, len(targets))+skipped+1))
            self.progress = dict(point=point_map[0], total=total, offset=offset, state='preparing',
                                 confirmed_point=confirmed, point_map=point_map)
            start = confirmed_start if offset == 0 else list(self.joints()[0])
            result = self._execute(start, targets[offset:offset+ROBOT_BUFFER], route=True)
            errors.append(result['max_error_deg'])
            confirmed = point_map[-1]
        self.progress = {**self.progress, 'point':total, 'total':total, 'confirmed_point':total, 'state':'completed'}
        return dict(seq=result['seq'], max_error_deg=max(errors), points=total)

    def command_snapshot(self):
        values = {key: self.number(key) for key in (
            'autoOrden', 'autoHecho', 'autoEstado', 'autoTramo', 'autoUltimo',
            'autoErrorEje', 'autoErrorDelta', 'autoErrorRAPID')}
        mapping = self.progress.get('point_map', [1])
        local, completed = int(values['autoTramo']), int(values['autoUltimo'])
        point = mapping[local-1] if 1 <= local <= len(mapping) else self.progress.get('point')
        confirmed = mapping[completed-1] if 1 <= completed <= len(mapping) else self.progress.get('confirmed_point', 0)
        return dict(values, point=point, confirmed_point=confirmed, total=self.progress.get('total', 1))

    def rejected(self, snapshot):
        code = int(snapshot['autoEstado'])
        return ExecutionFault(failure_message(code, snapshot['point'], int(snapshot['autoErrorEje']),
            snapshot['autoErrorDelta'], int(snapshot['autoErrorRAPID'])), kind='rapid_rejected', **snapshot)

    def _execute(self, confirmed_start, targets, *, route):
        from trayectorias import prepare, ROBOT_BUFFER
        published = False
        snapshot = None
        event_baseline = None
        try:
            self.ready()
            try:
                event_baseline = {str(row.get('_title')) for row in self.client.read('/rw/elog/0', lang='en', order='lifo')}
            except Exception:
                pass  # El registro auxiliar no impide leer y supervisar el movimiento.
            self.acquire()
            self.ready()
            current, external = self.joints()
            if max(abs(a-b) for a, b in zip(current, confirmed_start)) > POSTURE_TOL_DEG:
                raise ValueError('La postura real cambió mientras se calculaba el destino.')
            z = float(single(self.client.read(TCP_PATH, coordinate='Base', tool='tool0', wobj='wobj0'), 'z'))
            if route:
                targets = prepare(self.model, self.envelope, self.config, current, targets, z)
                if len(targets) > ROBOT_BUFFER:
                    raise ValueError('El bloque excede la capacidad del receptor RAPID.')
            else:
                check_goal(self.model, self.envelope, self.config, current, targets[0], z)
            target = list(targets[-1])
            previous = self.number('autoOrden')
            if not previous.is_integer() or not 0 <= previous < 1_000_000:
                raise ValueError('Secuencia RAPID inválida.')
            payloads = {'autoInicio': [current, external], 'autoDestino': [target, external]}
            for name, payload in payloads.items():
                self.post(SYMBOL+name, 'set', {'value': literal(payload)})
                readback = parse_target(single(self.client.read(SYMBOL+name), 'value'))
                if not same_target(payload, readback, tolerance=.005):
                    raise ValueError(f'Lectura de vuelta incorrecta de {name}.')
            if route:
                array = [[list(q), list(external)] for q in targets]
                array += [[list(target), list(external)] for _ in range(ROBOT_BUFFER-len(targets))]
                self.post(SYMBOL+'autoRuta', 'set', {'value': json.dumps(array)})
                readback = json.loads(single(self.client.read(SYMBOL+'autoRuta'), 'value'))
                if (not isinstance(readback, list) or len(readback) != ROBOT_BUFFER
                        or any(not same_target(expected, parse_target(json.dumps(actual)), tolerance=.005)
                               for expected, actual in zip(array, readback))):
                    raise ValueError('Lectura de vuelta incorrecta de la lista de puntos.')
            count = len(targets) if route else 0
            self.post(SYMBOL+'autoCantidad', 'set', {'value': str(count)})
            if self.number('autoCantidad') != count:
                raise ValueError('Cantidad de puntos sin confirmar.')
            self.ready()
            measured, _ = self.joints()
            if max(abs(a-b) for a, b in zip(current, measured)) > POSTURE_TOL_DEG:
                raise ValueError('El ABB cambió de postura antes de publicar la orden.')
            if self.number('autoOrden') != previous:
                raise ValueError('Otra aplicación publicó una orden.')
            speed_ratio = float(single(self.client.read('/rw/panel/speedratio'), 'speedratio'))
            if not math.isfinite(speed_ratio) or not 0 < speed_ratio <= 100:
                raise ValueError('La velocidad global del ABB debe ser mayor que 0% para ejecutar.')
            self.record('speed_confirmed', ratio_percent=speed_ratio)
            self.record('route_prepared' if route else 'goal_prepared',
                        start=current, targets=targets, speed_percent=speed_ratio)
            seq = int(previous) + 1
            published = True
            self.post(SYMBOL+'autoOrden', 'set', {'value': str(seq)})
            self.release()
            self.notify(f'ABB ejecutando trayectoria de {len(targets)} puntos…' if route
                        else 'ABB moviéndose hacia el destino de IK…')
            # Margen conservador de vigilancia; no predice la velocidad de MoveAbsJ/vmax.
            travel = sum(max(abs(a-b) for a, b in zip(aq, bq))
                         for aq, bq in zip([current]+targets[:-1], targets))
            timeout = max(180., 2*travel/(5*speed_ratio/100)+30.)
            deadline = time.monotonic() + timeout
            watchdog = ProgressWatchdog(timeout_s=min(60., 12./math.sqrt(speed_ratio/100.)))
            last_point = None
            while time.monotonic() < deadline:
                # Leer primero el resultado propio: puede estar disponible aunque
                # RAPID ya se haya detenido o haya terminado una rutina por error.
                snapshot = self.command_snapshot()
                self.progress = {**self.progress, 'point': snapshot['point'],
                                 'confirmed_point': snapshot['confirmed_point'], 'state': 'moving'}
                if (snapshot['autoOrden'] == seq and snapshot['autoTramo'] >= 1
                        and (snapshot['autoEstado'] == 1 or
                             (snapshot['autoHecho'] == seq and snapshot['autoEstado'] == 2))):
                    reference = list(targets[max(0,min(len(targets)-1,int(snapshot['autoTramo'])-1))])
                    if self.progress.get('goal_deg') != reference or self.progress.get('goal_seq') != seq:
                        self.progress = {**self.progress, 'goal_deg':reference, 'goal_seq':seq,
                                         'goal_observed_at':time.monotonic()}
                        self.record('target_observed', seq=seq, point=snapshot['point'], target_deg=reference)
                if snapshot['autoOrden'] != seq:
                    raise ExecutionFault('Otra aplicación cambió la orden activa.', kind='external_change', **snapshot)
                if snapshot['autoEstado'] < 0:
                    raise self.rejected(snapshot)
                actual_ratio = float(single(self.client.read('/rw/panel/speedratio'), 'speedratio'))
                if not math.isfinite(actual_ratio) or abs(actual_ratio-speed_ratio) > .01:
                    raise ValueError('El porcentaje ABB cambió durante la ejecución. Secuencia detenida.')
                if snapshot['autoHecho'] == seq:
                    if snapshot['autoEstado'] != 2:
                        raise self.rejected(snapshot)
                    measured, _ = self.joints()
                    error = max(abs(a-b) for a, b in zip(measured, target))
                    self.record('arrival', seq=seq, measured=measured, max_error_deg=error)
                    if error > .1:
                        raise ValueError(f'Error de llegada de {error:.6f}° mayor de 0,1°.')
                    self.progress = {**self.progress, 'confirmed_point': snapshot['point'], 'state': 'completed'}
                    return {'seq': seq, 'max_error_deg': error, 'points': len(targets)}
                self.ready(idle=False)
                measured, _ = self.joints()
                index = max(0, min(len(targets)-1, int(snapshot['autoTramo'])-1))
                remaining = max(abs(a-b) for a, b in zip(measured, targets[index]))
                if watchdog.observe(time.monotonic(), snapshot['point'], snapshot['confirmed_point'], remaining):
                    destination = f'P{snapshot["point"]}' if route else 'el destino'
                    raise ExecutionFault(f'Sin avance confirmado hacia {destination} durante '
                        f'{watchdog.timeout:.0f} s; se solicita Stop.', kind='no_progress',
                        measured=measured, remaining_deg=remaining, **snapshot)
                if snapshot['point'] != last_point:
                    self.record('route_progress', snapshot=snapshot, measured=measured)
                    last_point = snapshot['point']
                if route:
                    self.notify(f'ABB · hacia P{snapshot["point"]}/{snapshot["total"]} · '
                                f'último confirmado P{snapshot["confirmed_point"]} · {speed_ratio:g}%')
                else:
                    self.notify(f'ABB · hacia el destino de Control · {speed_ratio:g}%')
                time.sleep(.2)
            raise ExecutionFault('Tiempo total agotado sin confirmar la llegada.', kind='timeout', **(snapshot or {}))
        except Exception as exc:
            stop_accepted = None
            if published:
                stop_accepted = self.attempt_stop()
            if published and not isinstance(exc, requests.RequestException):
                try:
                    snapshot = self.command_snapshot()
                    if snapshot['autoEstado'] < 0 and not isinstance(exc, ExecutionFault):
                        exc = self.rejected(snapshot)
                except Exception:
                    pass
            details = getattr(exc, 'details', {})
            message = str(exc)
            kind = details.get('kind', 'communication' if isinstance(exc, requests.RequestException) else 'state_or_validation')
            if kind == 'communication':
                message = 'Comunicación interrumpida con el ABB. La llegada no está confirmada. '+str(exc)
            self.last_failure = {**(snapshot or {}), **details}
            self.last_failure.update(message=message, kind=kind, published=published, route=route,
                progress=dict(self.progress), log=str(self.log), stop_request_accepted=stop_accepted)
            self.record('execution_failed', message=message, progress=self.progress, detail=self.last_failure)
            self.progress = {**self.progress, 'state': 'failed', 'message': message,
                'point': self.last_failure.get('point', self.progress.get('point')),
                'confirmed_point': self.last_failure.get('confirmed_point', self.progress.get('confirmed_point', 0))}
            try:
                events = self.client.read('/rw/elog/0', lang='en', order='lifo')
                relevant = [{k: row[k] for k in ('_title', 'code', 'title', 'msgtype', 'tstamp', 'desc', 'actions') if k in row}
                            for row in events if event_baseline is None or str(row.get('_title')) not in event_baseline][:16]
                self.record('controller_events', events=relevant,
                            note=('Eventos nuevos desde la orden; incluyen paradas informativas.' if event_baseline is not None
                                  else 'Sin referencia inicial: eventos recientes, no necesariamente causa de este fallo.'))
            except Exception as diagnostic_error:
                self.record('diagnostic_unavailable', message=str(diagnostic_error))
            raise
        finally:
            self.release()

    def attempt_stop(self):
        try:
            self.post('/rw/rapid/execution', 'stop',
                      {'stopmode': 'stop', 'usetsp': 'alltsk'}, stopping=True)
            self.record('stop_requested')
            return True
        except Exception as exc:
            self.record('stop_unconfirmed', error=str(exc))
            self.notify('Parada por red sin confirmar. Usa Stop o la parada física del ABB.')
            return False

    def stop(self):
        self.cancelled.set()
        self.stop_verified = False
        try:
            self.post('/rw/rapid/execution', 'stop',
                      {'stopmode': 'stop', 'usetsp': 'alltsk'}, stopping=True)
            self.wait_stopped()
            self.stop_verified = True
            self.reconcile_stopped_order()
        finally:
            self.release()

    def reconcile_stopped_order(self, *, take_over=False):
        self.verify_module()
        mode, _, execution = self.states()
        if mode != 'AUTO' or execution != 'stopped':
            return
        previous = self.number('autoOrden')
        if previous == self.number('autoHecho'):
            return
        pointer = next((r for r in self.client.read(TASK+'/pcp')
                        if r.get('_title') == 'progpointer'), {})
        if pointer.get('modulemame') != MODULE and not take_over:
            raise ValueError('Orden pendiente en otro contexto. Revisa el programa del ABB.')
        if not previous.is_integer() or not 0 < previous < 1_000_000:
            raise ValueError('Secuencia pendiente inválida.')
        self.acquire(stopping=True)
        mode, _, execution = self.states()
        if (mode, execution) != ('AUTO', 'stopped'):
            raise ValueError('El estado cambió durante la cancelación.')
        current, _ = self.joints()
        deadline = time.monotonic()+3.
        while True:
            time.sleep(.15)
            measured, _ = self.joints()
            if max(abs(a-b) for a, b in zip(current, measured)) <= POSTURE_TOL_DEG:
                break
            if time.monotonic() >= deadline:
                raise ValueError('El brazo todavía cambia de postura. Comprueba la parada física.')
            current = measured
        if self.number('autoOrden') != previous or self.states()[2] != 'stopped':
            raise ValueError('La orden o RAPID cambiaron durante la cancelación.')
        self.prepare_pointer(stopping=True, take_over=take_over)
        for name, value in (('autoFin', 1), ('autoActivo', 0), ('autoEstado', -1)):
            self.post(SYMBOL+name, 'set', {'value': str(value)}, stopping=True)
        # Confirmar la cancelación al final del protocolo, sin publicar otra orden.
        self.post(SYMBOL+'autoHecho', 'set', {'value': str(int(previous))}, stopping=True)
        if self.number('autoHecho') != previous or self.number('autoEstado') != -1:
            raise ValueError('Cancelación sin lectura de vuelta confirmada.')
        self.record('order_cancelled_after_stop', seq=int(previous), measured_deg=measured)

    def close(self):
        try:
            logout(self.client)
        finally:
            self.session.close()
