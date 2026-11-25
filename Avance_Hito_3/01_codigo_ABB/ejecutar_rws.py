"""Envia un destino articular a ServidorDirecto mediante RWS."""
from datetime import datetime
import json
import math
from pathlib import Path
import time

from limites_movimiento import effective

MODULE = 'IMT342_DestinoDirecto'
ROUTINE = 'ServidorDirecto'
TASK_PATH = '/rw/rapid/tasks/T_ROB1'
MODULE_PATH = f'/rw/rapid/modules/{MODULE}'
SYMBOL = f'/rw/rapid/symbol/data/RAPID/T_ROB1/{MODULE}/'
TCP_PATH = '/rw/motionsystem/mechunits/ROB_1/robtarget'
POSTURE_TOL_DEG = 0.02


def flange_radii(model, offset):
    """Cota de la distancia a la punta desde el pivote de cada eje."""
    by_child = {joint.find('child').get('link'): joint for joint in model.joints}
    link, distance = 'tool0', math.hypot(*offset)
    radii = {}
    while link != 'base_link':
        joint = by_child[link]
        if joint.get('type') == 'revolute':
            radii[joint.get('name')] = distance
        origin = joint.find('origin')
        if origin is not None:
            distance += math.hypot(*map(float, origin.get('xyz', '0 0 0').split()))
        link = joint.find('parent').get('link')
    return [radii[f'joint_{i}'] for i in range(1, 7)]


def check_goal(model, envelope, config, start, target, real_z_mm=None):
    """Verifica rangos y extremos; la cota conservadora del tramo es informativa."""
    bounds = effective(config)
    if len(start) != 6 or len(target) != 6 or any(
        not math.isfinite(q) or not row['min_deg'] <= q <= row['max_deg']
        for pose in (start, target) for q, row in zip(pose, bounds)
    ):
        raise ValueError('Postura o destino fuera del rango articular configurado.')
    delta = [abs(a-b) for a, b in zip(start, target)]
    if max(delta) <= .005:
        raise ValueError('El destino ya coincide con la postura actual.')

    radii = flange_radii(model, envelope.tcp_offset_m)
    # Cambiar un eje altera la altura como máximo su radio por el ángulo
    # recorrido. Sumar estas cotas cubre cualquier orden de avance de los ejes.
    # El giro del eje vertical fijo de la base no cambia Z.
    base = next(joint for joint in model.joints if joint.get('name') == 'joint_1')
    if (base.find('parent').get('link') == 'base_link'
            and list(map(float, base.find('origin').get('rpy').split())) == [0., 0., 0.]
            and list(map(float, base.find('axis').get('xyz').split())) == [0., 0., 1.]):
        radii[0] = 0.
    z0 = float(model.tip(start, envelope)[2, 3])
    z1 = float(model.tip(target, envelope)[2, 3])
    real_z = z0 if real_z_mm is None else float(real_z_mm) / 1000
    if not math.isfinite(real_z) or abs(real_z-z0) > .01:
        raise ValueError('La altura real de la brida y la del modelo difieren más de 10 mm.')
    if min(real_z, z0, z1) < envelope.min_tip_z_m:
        raise ValueError('Un punto del movimiento está por debajo de la altura mínima configurada.')
    lower_z = min(real_z, z0, z1) - sum(
        radius * math.radians(change + POSTURE_TOL_DEG) for radius, change in zip(radii, delta))
    # Por indicación del operador, esta estimación no bloquea el movimiento.
    # No confundir una cota baja con un choque confirmado.
    return {'lower_z_bound_m': lower_z,
            'height_warning': lower_z < envelope.min_tip_z_m}


def execute_goal(base, credentials, model, envelope, config, confirmed_start, target, notify, cancelled):
    from requests import Session
    from requests.auth import HTTPDigestAuth
    from irb120_control.rws_j6 import Client, literal, measured_target, parse_target, same_target, single, source_fingerprint
    from irb120_control.rws_mover import FORM, RMMP_POLL, Rmmp, logout

    expected = Path(__file__).with_name('rapid') / f'{MODULE}.mod'
    logs = Path.home() / 'abb-real' / 'ordenes'
    logs.mkdir(parents=True, exist_ok=True)
    log = logs / datetime.now().strftime('directo_%Y%m%d_%H%M%S_%f.jsonl')

    def record(**values):
        with log.open('a', encoding='utf-8') as output:
            output.write(json.dumps({'time': datetime.now().isoformat(), **values})+'\n')

    with Session() as session:
        session.trust_env = False
        session.auth = HTTPDigestAuth(*credentials)
        client = Client(session, base)
        names = ('directoInicio', 'directoDestino', 'directoOrden', 'directoHecho',
                 'directoEstado', 'directoActivo', 'directoFin')
        client.extra_reads.update({MODULE_PATH, TASK_PATH+'/pcp', TCP_PATH, RMMP_POLL}
                                  | {SYMBOL+name for name in names})
        rmmp = Rmmp(client, Session)

        def phase(require_idle):
            mode = single(client.read('/rw/panel/opmode'), 'opmode')
            motors = single(client.read('/rw/panel/ctrlstate'), 'ctrlstate')
            rapid = single(client.read('/rw/rapid/execution'), 'ctrlexecstate')
            if (mode, motors, rapid) != ('MANR', 'motoron', 'running'):
                raise ValueError(f'ServidorDirecto requiere manual reducido, motores ON y RAPID en ejecución ({mode}/{motors}/{rapid}).')
            pointer = next((row for row in client.read(TASK_PATH+'/pcp') if row.get('_title') == 'progpointer'), {})
            if (pointer.get('modulemame'), pointer.get('routinename')) != (MODULE, ROUTINE):
                raise ValueError('Arranca IMT342_DestinoDirecto / ServidorDirecto desde el FlexPendant.')
            if require_idle and (read_num('directoActivo') != 1 or read_num('directoFin') != 0
                                 or read_num('directoOrden') != read_num('directoHecho')):
                raise ValueError('ServidorDirecto no está listo o tiene una orden pendiente.')

        def read_num(name):
            return float(single(client.read(SYMBOL+name), 'value'))

        def joints():
            return measured_target(client.read(TASK_PATH+'/motion', resource='jointtarget'))

        def checked_start():
            z = single(client.read(TCP_PATH, coordinate='Base', tool='tool0', wobj='wobj0'), 'z')
            start, external = joints()
            if any(abs(a-b) > POSTURE_TOL_DEG for a, b in zip(start, confirmed_start)):
                raise ValueError('La postura cambió desde la confirmación. Sincroniza la sombra y vuelve a ejecutar.')
            check_goal(model, envelope, config, start, target, z)
            return start, external

        def write(name, value):
            if cancelled.is_set():
                raise ValueError('Orden cancelada antes de publicarla.')
            rmmp.keep()
            record(event='write_attempt', symbol=name, value=value)
            response = session.post(base+SYMBOL+name, params={'action': 'set'}, data={'value': value},
                                    headers=FORM, timeout=3, allow_redirects=False)
            record(event='write_response', symbol=name, http=response.status_code)
            if response.status_code not in (200, 204):
                raise ValueError(f'Error RWS al escribir {name} (HTTP {response.status_code}).')

        try:
            client.verify_identity()
            modules = client.read('/rw/rapid/modules', task='T_ROB1')
            if MODULE not in {row.get('name') for row in modules}:
                raise ValueError('Carga IMT342_DestinoDirecto.mod en T_ROB1 y comprueba el programa.')
            records = client.read(MODULE_PATH, resource='module-text', task='T_ROB1')
            client.resolve_module_text(records, MODULE)
            if source_fingerprint(single(records, 'module-text')) != source_fingerprint(expected.read_text(encoding='ascii')):
                raise ValueError('El código de ServidorDirecto difiere del módulo revisado.')
            phase(require_idle=True)
            checked_start()
            notify('Concede el acceso remoto en el FlexPendant.')
            rmmp.request(wait=45.)
            phase(require_idle=True)
            start, external = checked_start()
            previous = read_num('directoOrden')
            if not 0 <= previous < 1_000_000 or not previous.is_integer():
                raise ValueError('Secuencia RAPID inválida.')
            targets = {'directoInicio': [start, external], 'directoDestino': [target, external]}
            for name, payload in targets.items():
                write(name, literal(payload))
            phase(require_idle=True)
            if read_num('directoOrden') != previous:
                raise ValueError('Otra aplicación publicó una orden. Revisa el ABB.')
            checked_start()
            for name, payload in targets.items():
                readback = parse_target(single(client.read(SYMBOL+name), 'value'))
                if not same_target(payload, readback, tolerance=.005):
                    raise ValueError(f'{name} no coincide con la lectura de vuelta.')
            seq = int(previous)+1
            write('directoOrden', str(seq))
            notify('Orden publicada. Esperando que el ABB confirme la llegada.')

            deadline = time.monotonic()+180
            while time.monotonic() < deadline:
                done = read_num('directoHecho')
                code = read_num('directoEstado')
                if done == seq:
                    if code != 2:
                        raise ValueError(f'El ABB canceló o rechazó la orden (estado {code}).')
                    measured, _ = joints()
                    error = max(abs(a-b) for a, b in zip(measured, target))
                    record(event='result', seq=seq, code=code, measured_deg=measured, max_error_deg=error)
                    return {'seq': seq, 'max_error_deg': error}
                if cancelled.is_set() or single(client.read('/rw/rapid/execution'), 'ctrlexecstate') != 'running':
                    raise ValueError('Orden ya publicada. Comprueba su estado en el FlexPendant antes de repetir.')
                rmmp.keep()
                time.sleep(.35)
            raise ValueError('Sin confirmación de llegada. Revisa el estado de la orden en el FlexPendant.')
        finally:
            rmmp.close()
            logout(client)
