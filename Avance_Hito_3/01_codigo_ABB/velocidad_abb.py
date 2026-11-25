"""Porcentaje global real de RobotWare mediante RWS en modo AUTO."""
from datetime import datetime
import json
import math
from pathlib import Path

import requests
from requests.auth import HTTPDigestAuth

from control_automatico import MODULE, ROUTINE, SYMBOL, TASK
from irb120_control.rws_j6 import Client, single
from irb120_control.rws_mover import FORM, logout

RESOURCE = '/rw/panel/speedratio'


class SpeedController:
    def __init__(self, base, credentials):
        self.base = base
        self.session = requests.Session()
        self.session.trust_env = False
        self.session.auth = HTTPDigestAuth(*credentials)
        self.client = Client(self.session, base, timeout=1.5)
        self.client.extra_reads.update(
            {RESOURCE, TASK+'/pcp', SYMBOL+'autoOrden', SYMBOL+'autoHecho', SYMBOL+'autoActivo'})

    def read(self):
        self.client.verify_identity()
        value = float(single(self.client.read(RESOURCE), 'speedratio'))
        if not math.isfinite(value) or not 0 <= value <= 100:
            raise ValueError('Porcentaje de velocidad ABB inválido.')
        return dict(ratio=value,
                    mode=single(self.client.read('/rw/panel/opmode'), 'opmode'),
                    execution=single(self.client.read('/rw/rapid/execution'), 'ctrlexecstate'))

    def post(self, path, action, data=None):
        result = self.session.post(
            self.base+path, params={'action': action}, data=data, headers=FORM,
            timeout=2., allow_redirects=False)
        if result.status_code not in (200, 204):
            raise ValueError(f'RobotWare rechazó {action} (HTTP {result.status_code}).')

    def apply(self, value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 100:
            raise ValueError('La velocidad global debe estar entre 0 y 100%.')
        self.client.verify_identity()
        acquired = False
        folder = Path.home()/'abb-real'/'ordenes'
        folder.mkdir(parents=True, exist_ok=True)
        log = folder/datetime.now().strftime('velocidad_%Y%m%d_%H%M%S_%f.json')
        record = {'time': datetime.now().isoformat(), 'requested_percent': value, 'confirmed': False}
        try:
            if self.read()['mode'] != 'AUTO':
                raise ValueError('RobotWare permite ajustar la velocidad desde la PC en modo AUTO.')
            self.post('/rw/mastership/rapid', 'request')
            acquired = True
            state = self.read()
            record['before_percent'] = state['ratio']
            if state['mode'] != 'AUTO':
                raise ValueError('RobotWare permite ajustar la velocidad desde la PC en modo AUTO.')
            if state['execution'] == 'running':
                pointer = next((r for r in self.client.read(TASK+'/pcp')
                                if r.get('_title') == 'progpointer'), {})
                if (pointer.get('modulemame'), pointer.get('routinename')) != (MODULE, ROUTINE):
                    raise ValueError('Detén el otro programa antes de ajustar la velocidad desde este panel.')
                sequence = float(single(self.client.read(SYMBOL+'autoOrden'), 'value'))
                done = float(single(self.client.read(SYMBOL+'autoHecho'), 'value'))
                active = float(single(self.client.read(SYMBOL+'autoActivo'), 'value'))
                if sequence != done or active != 1:
                    raise ValueError('Espera a que termine el movimiento para ajustar la velocidad.')
            elif state['execution'] != 'stopped':
                raise ValueError('Estado RAPID no reconocido para ajustar velocidad.')
            log.write_text(json.dumps(record), encoding='utf-8')
            self.post(RESOURCE, 'setspeedratio', {'speed-ratio': f'{value:g}'})
            result = self.read()
            record['actual_percent'] = result['ratio']
            if abs(result['ratio']-value) > .01:
                raise ValueError('La lectura de velocidad del ABB difiere del valor solicitado.')
            record['confirmed'] = True
            return result
        except Exception as exc:
            record['error'] = str(exc)
            raise
        finally:
            log.write_text(json.dumps(record), encoding='utf-8')
            if acquired:
                self.post('/rw/mastership/rapid', 'release')

    def close(self):
        logout(self.client)
        self.session.close()
