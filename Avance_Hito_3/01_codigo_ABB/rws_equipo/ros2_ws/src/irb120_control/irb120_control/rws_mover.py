"""Mover el IRB 120 desde la PC mediante RWS y el módulo LabJointRws v2 (ServidorPC).

La PC no arranca RAPID, no enciende motores ni cambia el modo. El operador deja
ServidorPC en ejecución desde el FlexPendant (manual reducido, habilitador
presionado) y la PC solo escribe objetivos articulares absolutos en grados:

    pcTarget  <- objetivo J1..J6 (lectura de vuelta comprobada)
    pcSeq     <- pcSeq + 1      (ordena ejecutar el objetivo ya escrito)
    rbCodigo  -> resultado del robot (2 = llegada; negativo = rechazo)
    rbSeq     -> copia de pcSeq cuando el robot terminó o rechazó

En manual, ABB exige el permiso remoto RMMP para escribir variables PERS: el
FlexPendant muestra una solicitud que el operador debe aceptar. El permiso se
mantiene consultando /users/rmmp/poll periódicamente.
"""
import argparse
from datetime import datetime, timezone
import getpass
import json
import math
from pathlib import Path
import shlex
import threading
import time

from irb120_conexion.rutas import PROJECT
from irb120_conexion.comunicacion.lectura_rws import error_details, state_records
from irb120_conexion.nodos.lectura_articular.puente_rws import build_base_url
from irb120_control.rws_j6 import (Client, Refused, EMPTY, LIMITS, SYMBOL, SERVER_CANDIDATE,
    SERVER_MODULE_SHA256, SERVER_AUTO_MODULE_SHA256, ORIGINAL_NORMALIZED_SHA256, J6_MODULE_SHA256,
    measured_target, module_label, parse_target, literal, same_target, single, source_fingerprint)

BASE_SYMBOL = '/rw/rapid/symbol/data/RAPID/T_ROB1/LabJointRws/'
NUM_SYMBOLS = ('pcSeq', 'pcFin', 'rbSeq', 'rbCodigo', 'rbActivo')
RMMP = '/users/rmmp'
RMMP_POLL = '/users/rmmp/poll'
MAX_STEP_DEG = 10.0      # igual a maxPasoDeg del módulo RAPID
MARGIN_DEG = 2.0         # igual a margenDeg del módulo RAPID
# La postura se relee entre el cálculo del tramo y su envío: el ruido de medida y el
# error residual de llegada (centésimas de grado) no deben rechazar un «j6 +5» con paso 5.
STEP_TOL_DEG = 0.05
AUTO_OFF = 'CONST bool permitirAuto := FALSE;'
AUTO_ON = 'CONST bool permitirAuto := TRUE;'
CODES = {0: 'listo', 1: 'moviendo', 2: 'llegada', -1: 'objetivo vacío o no válido',
         -2: 'fuera de límites con margen de 2°', -3: 'algún eje cambia más de 10° en una orden',
         -4: 'modo no permitido por el módulo (¿AUTO sin permitirAuto?)'}
FORM = {'Content-Type': 'application/x-www-form-urlencoded'}


def check_target(target, current, step):
    """Comprobación local (además de la del robot): límites con margen y paso máximo."""
    target = parse_target(literal(target))
    for index, (q, (lo, hi)) in enumerate(zip(target[0], LIMITS), start=1):
        if not lo + MARGIN_DEG <= q <= hi - MARGIN_DEG:
            raise Refused(f'J{index}={q:.3f}° fuera de [{lo + MARGIN_DEG}, {hi - MARGIN_DEG}].')
    worst = max(abs(a - b) for a, b in zip(target[0], current[0]))
    if worst > step + STEP_TOL_DEG:
        raise Refused(f'Cambio de {worst:.3f}° mayor que el paso permitido de {step}°.')
    return target


def split_path(current, goal, step):
    """Divide un recorrido articular en tramos rectos de como máximo `step` grados por eje."""
    if not 0 < step <= MAX_STEP_DEG:
        raise Refused(f'--paso debe estar entre 0 y {MAX_STEP_DEG}°.')
    goal = parse_target(literal(goal))
    for index, (q, (lo, hi)) in enumerate(zip(goal[0], LIMITS), start=1):
        if not lo + MARGIN_DEG <= q <= hi - MARGIN_DEG:
            raise Refused(f'Destino J{index}={q:.3f}° fuera de [{lo + MARGIN_DEG}, {hi - MARGIN_DEG}].')
    worst = max(abs(a - b) for a, b in zip(goal[0], current[0]))
    # Observado en el IRC5 (16/11/2026): un «j6 +5» medido como 5,000004° se partía en
    # dos órdenes. Se tolera el ruido de medida sin superar nunca el máximo del robot.
    tolerance = min(STEP_TOL_DEG, MAX_STEP_DEG - step)
    count = max(1, math.ceil((worst - tolerance) / step - 1e-9))
    return [[[c + (g - c) * k / count for c, g in zip(current[0], goal[0])], [9e9] * 6]
            for k in range(1, count + 1)]


class Rmmp:
    """Solicita y mantiene RMMP. El sondeo usa una sesión paralela con las mismas cookies,
    renovada cada 30 s, porque el ABB revoca el permiso si se pierde esa conexión."""

    def __init__(self, client, session_factory):
        self.client, self.factory = client, session_factory
        self.poll_session = None
        self.poll_started = 0.
        self.granted = False
        self.last_poll = 0.

    def _status(self, session):
        response = session.get(self.client.base + RMMP_POLL, params={'json': '1'},
                               timeout=self.client.timeout, allow_redirects=False)
        if response.status_code != 200:
            raise Refused(f'RMMP poll: HTTP {response.status_code}; {error_details(response.text)}')
        return single(state_records(response.text), 'status')

    def _new_session(self):
        session = self.factory()
        session.trust_env = False
        session.auth = self.client.session.auth
        for cookie in self.client.session.cookies:
            session.cookies.set_cookie(cookie)
        return session

    def request(self, wait=90., sleep=time.sleep):
        response = self.client.session.post(self.client.base + RMMP, params={'json': '1'},
                                            data={'privilege': 'modify'}, headers=FORM,
                                            timeout=self.client.timeout, allow_redirects=False)
        if response.status_code not in (200, 201, 202, 204):
            raise Refused(f'Solicitud RMMP: HTTP {response.status_code}; {error_details(response.text)}')
        print('>>> En el FlexPendant aparece una solicitud de acceso remoto: el operador debe ACEPTARLA.',
              flush=True)
        started = time.monotonic()
        while time.monotonic() - started < wait:
            status = self._status(self.client.session)
            if status == 'GRANTED':
                self.granted = True
                self.keep(force=True)
                print('RMMP concedido: la PC puede escribir objetivos en manual.', flush=True)
                return
            if status != 'PENDING':
                raise Refused(f'RMMP no concedido (estado {status}). El operador lo rechazó o caducó.')
            sleep(.25)
        raise Refused('RMMP no concedido a tiempo. Repetir y aceptar en el FlexPendant.')

    def keep(self, force=False):
        if not self.granted:
            return
        now = time.monotonic()
        if not force and now - self.last_poll < 2.:
            return
        old = None
        if self.poll_session is None or now - self.poll_started > 30.:
            old, self.poll_session, self.poll_started = self.poll_session, self._new_session(), now
        status = self._status(self.poll_session)
        self.last_poll = now
        if old is not None:
            try:
                old.close()
            except Exception:
                pass
        if status != 'GRANTED':
            self.granted = False
            raise Refused(f'Se perdió RMMP (estado {status}). Pedirlo de nuevo con el operador.')

    def close(self):
        if self.poll_session is not None:
            try:
                self.poll_session.close()
            except Exception:
                pass


class Mover(Client):
    def __init__(self, session, base, session_factory=None, timeout=3., log=None, sleep=time.sleep):
        super().__init__(session, base, timeout)
        self.extra_reads = {BASE_SYMBOL + name for name in NUM_SYMBOLS} | {RMMP_POLL}
        self.rmmp = Rmmp(self, session_factory or (lambda: type(session)()))
        self.log, self.sleep = log, sleep
        self.module_version = 'no_leido'
        # Una sola operación HTTP a la vez: requests.Session no es segura entre hilos.
        self.lock = threading.RLock()
        self.keeper_error = None
        self.on_joints = None

    def start_keeper(self, period=2.):
        """Mantiene RMMP mientras el usuario escribe en el prompt (sin sondeo el ABB lo revoca)."""
        stop = threading.Event()

        def run():
            while not stop.wait(period):
                if self.lock.acquire(blocking=False):
                    try:
                        self.rmmp.keep()
                    except Exception as exc:  # se informa en la siguiente orden
                        self.keeper_error = str(exc)
                    finally:
                        self.lock.release()
        threading.Thread(target=run, daemon=True, name='rmmp-keeper').start()
        return stop

    def record(self, **row):
        row = {'at_utc': datetime.now(timezone.utc).isoformat(), **row}
        if self.log is not None:
            with self.log.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(row, ensure_ascii=False) + '\n')

    # ---------- lecturas ----------
    def num(self, name):
        value = single(self.read(BASE_SYMBOL + name), 'value')
        try:
            return float(value)
        except ValueError as exc:
            raise Refused(f'{name} no es numérico: {value}') from exc

    def joints(self):
        target = measured_target(self.read('/rw/rapid/tasks/T_ROB1/motion', resource='jointtarget'))
        if self.on_joints is not None:  # p. ej. el panel refresca RViz durante un movimiento
            self.on_joints(list(target[0]))
        return target

    def status(self):
        state = {'opmode': single(self.read('/rw/panel/opmode'), 'opmode'),
                 'ctrlstate': single(self.read('/rw/panel/ctrlstate'), 'ctrlstate'),
                 'execution': single(self.read('/rw/rapid/execution'), 'ctrlexecstate'),
                 'joints_deg': self.joints()[0], 'module': self.module_version}
        if self.module_version.startswith('servidor_v2'):
            for name in NUM_SYMBOLS:
                state[name] = self.num(name)
        return state

    # ---------- escrituras ----------
    def write(self, name, value):
        path = SYMBOL if name == 'pcTarget' else BASE_SYMBOL + name
        if name not in ('pcTarget', 'pcSeq', 'pcFin'):
            raise Refused('Solo se escriben pcTarget, pcSeq y pcFin.')
        self.rmmp.keep()
        self.write_events.append({'at_utc': datetime.now(timezone.utc).isoformat(), 'symbol': name,
                                  'value': value, 'result': 'pending_or_unknown'})
        response = self.session.post(self.base + path, params={'action': 'set'}, data={'value': value},
                                     headers=FORM, timeout=self.timeout, allow_redirects=False)
        event = self.write_events[-1]
        event['http_status'] = response.status_code
        event['result'] = 'accepted' if response.status_code in (200, 204) else 'rejected'
        self.record(event='write', symbol=name, value=value, http_status=response.status_code)
        if response.status_code not in (200, 204):
            hint = ''
            if response.status_code == 403:
                hint = ' En manual hace falta RMMP aceptado en el FlexPendant.'
                if self.rmmp.granted:
                    # Observado en el IRC5 (16/11/2026): tras detener RAPID el sondeo seguía en GRANTED
                    # pero la escritura daba 403. Se descarta el permiso para pedirlo de nuevo.
                    self.rmmp.granted = False
                    hint = (' El controlador ya no acepta el permiso RMMP: repetir la orden y '
                            'pulsar «Conceder» otra vez en el FlexPendant.')
            raise Refused(f'POST {name}: HTTP {response.status_code}; {error_details(response.text)}.{hint}')

    def ensure_rmmp(self, state):
        if state['opmode'] in ('MANR', 'MANF') and not self.rmmp.granted:
            self.rmmp.request(sleep=self.sleep)

    # ---------- protocolo ----------
    def ready(self, state):
        problems = []
        if state.get('module') not in ('servidor_v2', 'servidor_v2_auto'):
            problems.append(f"módulo {state.get('module')}: ejecutar 'instalar' (AUTO, motores apagados, RAPID detenido)")
        if state['opmode'] == 'MANF':
            problems.append('modo manual 100%: usar manual reducido')
        elif state['opmode'] == 'AUTO' and state.get('module') != 'servidor_v2_auto':
            problems.append('modo AUTO: el módulo solo acepta manual reducido (permitirAuto := FALSE)')
        if state['ctrlstate'] != 'motoron':
            problems.append(f"motores {state['ctrlstate']}: mantener el habilitador (manual) o Motors On (AUTO)")
        if state['execution'] != 'running':
            problems.append('RAPID detenido: PP to Routine → ServidorPC, Start y pulsar «Iniciar»')
        if state.get('rbActivo') != 1:
            problems.append('ServidorPC no está activo (rbActivo=0): arrancarlo y pulsar «Iniciar» en el FlexPendant')
        if state.get('pcFin', 0) != 0:
            problems.append('pcFin=1: el servidor terminó; arrancarlo de nuevo')
        if state.get('pcSeq') is not None and state.get('pcSeq') != state.get('rbSeq'):
            problems.append('hay una orden en curso (pcSeq≠rbSeq): esperar su respuesta')
        return problems

    def send(self, target, step=MAX_STEP_DEG, timeout=120., echo=print):
        state = self.status()
        problems = self.ready(state)
        if problems:
            raise Refused('No listo: ' + '; '.join(problems))
        current = [state['joints_deg'], [9e9] * 6]
        target = check_target(target, current, step)
        self.ensure_rmmp(state)
        self.write('pcTarget', literal(target))
        readback = self.get_target()
        if not same_target(target, readback):
            raise Refused('La lectura de vuelta de pcTarget no coincide; no se ordena el movimiento.')
        seq = int(state['pcSeq']) + 1
        self.record(event='order', seq=seq, from_deg=current[0], target_deg=target[0])
        self.write('pcSeq', str(seq))
        started = time.monotonic(); warned = False
        while time.monotonic() - started < timeout:
            self.rmmp.keep()
            q = self.joints()[0]
            done = self.num('rbSeq') == seq
            code = int(self.num('rbCodigo'))
            error = max(abs(a - b) for a, b in zip(q, target[0]))
            echo('  ' + ' '.join(f'J{i}={v:8.3f}' for i, v in enumerate(q, 1)) +
                 f' | falta {error:7.3f}° | robot: {CODES.get(code, code)}')
            if done:
                self.record(event='result', seq=seq, code=code, joints_deg=q, max_error_deg=error)
                if code != 2:
                    raise Refused(f'El robot rechazó la orden {seq}: código {code} ({CODES.get(code, "?")}).')
                return {'seq': seq, 'code': code, 'joints_deg': q, 'max_error_deg': error}
            execution = single(self.read('/rw/rapid/execution'), 'ctrlexecstate')
            if execution != 'running' and not warned:
                warned = True
                echo('  AVISO: RAPID se detuvo (¿se soltó el habilitador?). Al pulsar Start de nuevo la orden continúa.')
            self.sleep(.25)
        raise Refused(f'Sin respuesta del robot a la orden {seq} en {timeout:.0f} s. '
                      'No enviar otra: revisar FlexPendant y ejecutar «estado».')

    def move_to(self, goal, step, timeout=120., echo=print):
        current = self.joints()
        legs = split_path(current, goal, step)
        echo(f'Recorrido dividido en {len(legs)} orden(es) de como máximo {step}° por eje.')
        result = None
        for index, leg in enumerate(legs, 1):
            echo(f'Orden {index}/{len(legs)} → ' + ', '.join(f'{v:.3f}' for v in leg[0]))
            result = self.send(leg, step=step, timeout=timeout, echo=echo)
        return result

    def finish(self):
        state = self.status()
        self.ensure_rmmp(state)
        self.write('pcFin', '1')

    def connect(self):
        """Lee identidad y versión del módulo una vez por sesión."""
        identity = self.verify_identity()
        self.module_version = module_label(source_fingerprint(self.module_source()))
        return identity

    def install(self, directory, auto=False):
        """auto=True instala la misma v2 con permitirAuto := TRUE (motores ON sin habilitador,
        solo con autorización del responsable). Se transfiere por RWS para que la huella sea
        exacta: editar la línea en el FlexPendant puede reformatearla."""
        path, sha = SERVER_CANDIDATE, SERVER_MODULE_SHA256
        if auto:
            source = SERVER_CANDIDATE.read_text(encoding='utf-8')
            if source.count(AUTO_OFF) != 1:
                raise Refused('El candidato no contiene exactamente una línea permitirAuto := FALSE.')
            directory.mkdir(parents=True, exist_ok=True)
            path, sha = directory / 'LabJointRws_auto.mod', SERVER_AUTO_MODULE_SHA256
            path.write_text(source.replace(AUTO_OFF, AUTO_ON), encoding='utf-8')
        return self.update_module(directory, candidate_path=path, candidate_sha=sha,
                                  accepted=(ORIGINAL_NORMALIZED_SHA256, J6_MODULE_SHA256,
                                            SERVER_MODULE_SHA256, SERVER_AUTO_MODULE_SHA256))


def logout(mover):
    """Cierra la sesión RWS: RW6 admite pocas sesiones simultáneas y tarda en caducarlas."""
    try:
        mover.session.get(mover.base + '/logout', timeout=2., allow_redirects=False)
    except Exception:
        pass


def print_status(state):
    print(f"Modo: {state['opmode']} | Motores: {state['ctrlstate']} | RAPID: {state['execution']} | "
          f"Módulo: {state.get('module')}")
    print('Ejes (°): ' + ', '.join(f'J{i}={v:.3f}' for i, v in enumerate(state['joints_deg'], 1)))
    if 'rbActivo' in state:
        code = int(state['rbCodigo'])
        print(f"ServidorPC activo: {'SI' if state['rbActivo'] == 1 else 'NO'} | pcSeq={state['pcSeq']:.0f} "
              f"rbSeq={state['rbSeq']:.0f} | último resultado: {CODES.get(code, code)}")


def parse_goal(words, current):
    """abs a b c d e f  |  j<N> <delta>  |  j<N>= <valor>"""
    goal = [list(current[0]), [9e9] * 6]
    if words[0] == 'abs':
        if len(words) != 7:
            raise Refused('Uso: abs J1 J2 J3 J4 J5 J6 (grados)')
        goal[0] = [float(w) for w in words[1:]]
        return goal
    if len(words) == 2 and words[0].lower().startswith('j'):
        name = words[0].lower()
        absolute = name.endswith('=')
        index = int(name.strip('j=')) - 1
        if not 0 <= index < 6:
            raise Refused('Eje entre J1 y J6.')
        value = float(words[1])
        goal[0][index] = value if absolute else goal[0][index] + value
        return goal
    raise Refused('Orden no reconocida.')


HELP = """Órdenes:
  estado                 modo, motores, RAPID y ángulos
  j2 +5 / j6 -10         mover un eje de forma relativa (grados)
  j3= 20                 llevar un eje a un valor absoluto
  abs 0 -10 20 0 30 0    llevar los seis ejes a valores absolutos
  paso 5                 cambiar el tamaño máximo de cada orden (0 a 10°)
  fin                    terminar ServidorPC en el robot (pcFin=1)
  salir                  cerrar este programa (el robot sigue esperando)
Ctrl+C no detiene el robot: para eso, soltar el habilitador o pulsar Stop/emergencia."""


def interactive(mover, step, timeout):
    print(HELP)
    with mover.lock:
        state = mover.status()
        print_status(state)
        # Pedir RMMP ahora, con el robot quieto, y no en medio de la primera orden.
        mover.ensure_rmmp(state)
    stop = mover.start_keeper()
    try:
        return _interactive(mover, step, timeout)
    finally:
        stop.set()


def _interactive(mover, step, timeout):
    while True:
        try:
            line = input(f'irb120 (paso {step}°)> ').strip()
        except EOFError:
            return 0
        if not line:
            continue
        try:
            words = shlex.split(line)
        except ValueError as exc:
            print('NO ENVIADO:', exc)
            continue
        if mover.keeper_error:
            print('AVISO:', mover.keeper_error, '— la siguiente escritura volverá a pedir RMMP.')
            mover.keeper_error = None
        try:
            mover.lock.acquire()
            if words[0] in ('salir', 'exit', 'q'):
                return 0
            if words[0] in ('ayuda', 'help', '?'):
                print(HELP)
            elif words[0] == 'estado':
                state = mover.status()
                print_status(state)
                for problem in mover.ready(state):
                    print('FALTA:', problem)
            elif words[0] == 'paso':
                value = float(words[1])
                if not 0 < value <= MAX_STEP_DEG:
                    raise Refused('El paso debe estar entre 0 y 10°.')
                step = value
            elif words[0] == 'fin':
                mover.finish(); print('pcFin=1 enviado: ServidorPC termina en el robot.')
            else:
                current = mover.joints()
                goal = parse_goal(words, current)
                legs = len(split_path(current, goal, step))
                if legs > 1 or words[0] == 'abs':
                    # Un «abs» en radianes o un destino lejano moverían varios ejes sin avisar.
                    shown = ', '.join(f'J{i}={v:.2f}' for i, v in enumerate(goal[0], 1))
                    if input(f'{legs} orden(es) hasta {shown} (grados). ¿Confirmar? [s/N] ').strip().lower() != 's':
                        print('Cancelado: no se envió nada.')
                        continue
                result = mover.move_to(goal, step, timeout)
                print(f"LLEGADA confirmada por el robot (orden {result['seq']}), error máximo "
                      f"{result['max_error_deg']:.3f}°.")
        except (Refused, ValueError, IndexError) as exc:
            print('NO ENVIADO / DETENIDO:', exc)
        finally:
            mover.lock.release()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('estado', 'instalar', 'mover', 'interactivo', 'terminar'))
    parser.add_argument('valores', nargs='*', help='mover: "abs J1..J6", "j2 +5" o "j3= 20"')
    parser.add_argument('--host', default='192.168.125.1')
    parser.add_argument('--user', default='Default User')
    parser.add_argument('--http-port', type=int, default=80, help='Distinto de 80 solo en 127.0.0.1')
    parser.add_argument('--paso', type=float, default=5., help='Máximo cambio por eje en cada orden (≤10°)')
    parser.add_argument('--timeout', type=float, default=120., help='Segundos de espera por orden')
    parser.add_argument('--auto', action='store_true',
                        help='instalar: variante con permitirAuto := TRUE (modo AUTO, motores ON sin habilitador)')
    parser.add_argument('--log-dir', type=Path, default=PROJECT / 'salidas/control_articular/rws_mover')
    args = parser.parse_args(argv)
    if not 0 < args.paso <= MAX_STEP_DEG:
        parser.error('--paso debe estar entre 0 y 10')
    if args.action == 'mover' and not args.valores:
        parser.error('mover necesita valores, por ejemplo: mover j6 +5')
    try:
        base = build_base_url(args.host, args.http_port)
    except ValueError as exc:
        parser.error(str(exc))
    import requests
    from requests.auth import HTTPDigestAuth
    args.log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%fZ')
    log = args.log_dir / f'{args.action}_{stamp}.jsonl'
    password = getpass.getpass(f'Contraseña RWS de "{args.user}" (no se guarda): ')
    mover = None
    try:
        with requests.Session() as session:
            session.trust_env = False
            session.auth = HTTPDigestAuth(args.user, password)
            mover = Mover(session, base, session_factory=requests.Session, log=log)
            identity = mover.connect()
            print(f"Controlador {identity['name']} | RobotWare {identity['rwversionname']} | "
                  f"módulo {mover.module_version}")
            if args.action == 'estado':
                state = mover.status()
                print_status(state)
                problems = mover.ready(state)
                for problem in problems:
                    print('FALTA:', problem)
                if not problems:
                    print('LISTO para recibir órdenes.')
                return 0
            if args.action == 'instalar':
                report = mover.install(args.log_dir, auto=args.auto)
                report['write_events'] = mover.write_events
                path = args.log_dir / f'instalar_{stamp}.json'
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
                print(json.dumps({k: v for k, v in report.items() if k != 'write_events'},
                                 ensure_ascii=False, indent=2))
                print('Informe:', path)
                if report['passed']:
                    print(('Módulo v2 (variante AUTO) instalado. ' if args.auto else 'Módulo v2 instalado. ') +
                          'En el FlexPendant: Editor de programas → Verificar programa. '
                          'El puntero pudo restablecerse: NO pulsar Start sobre main.')
                return 0 if report['passed'] else 2
            if args.action == 'terminar':
                mover.finish(); print('pcFin=1 enviado.')
                return 0
            if args.action == 'mover':
                goal = parse_goal(args.valores, mover.joints())
                result = mover.move_to(goal, args.paso, args.timeout)
                print(f"LLEGADA confirmada (orden {result['seq']}), error máximo {result['max_error_deg']:.3f}°.")
                return 0
            return interactive(mover, args.paso, args.timeout)
    except (Refused, ValueError, requests.RequestException, OSError) as exc:
        print('DETENIDO:', exc)
        if mover is not None and mover.write_events:
            print('Registro:', log)
        return 2
    except KeyboardInterrupt:
        print('\nInterrumpido en la PC. Esto NO detiene el robot: el operador suelta el habilitador o pulsa Stop.')
        return 130
    finally:
        if mover is not None:
            mover.rmmp.close()
            logout(mover)


if __name__ == '__main__':
    raise SystemExit(main())
