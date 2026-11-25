"""Pruebas de protocolo con RWS simulado. No acceden a un controlador real."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
from threading import Event
import unittest
from unittest.mock import patch

import requests

ROOT = Path(__file__).resolve().parent
RWS = ROOT/'rws_equipo'/'ros2_ws'/'src'
for package in ('irb120_control', 'irb120_conexion'):
    sys.path.insert(0, str(RWS/package))

from control_automatico import AutomaticSession, MODULE, MODULE_PATH, ROUTINE, SYMBOL, TASK, TCP_PATH
from irb120_control.rws_j6 import literal, parse_target
from limites_movimiento import load
from objetivos_preview import Model, configured_envelope


class FakeRws:
    def __init__(self, model, envelope):
        self.mode, self.motors, self.execution = 'AUTO', 'motoroff', 'stopped'
        self.module, self.routine = 'MainModule', 'main'
        self.execution_type = 'normal'
        self.q = [0., -30., 30., 0., 45., 0.]
        self.model, self.envelope = model, envelope
        self.values = {
            'autoOrden': 0., 'autoHecho': 0., 'autoActivo': 0.,
            'autoFin': 0., 'autoEstado': 0., 'autoSesion': 0.,
            'autoCantidad': 0., 'autoTramo': 0., 'autoRuta': '[]',
            'autoUltimo': 0., 'autoErrorEje': 0., 'autoErrorDelta': 0., 'autoErrorRAPID': 0.,
        }
        self.source = (ROOT/'rapid'/f'{MODULE}.mod').read_text(encoding='ascii')
        self.writes = []
        self.lose_commit_response = False
        self.bad_readback = False
        self.closed = False
        self.speed_ratio = 100.
        self.module_loaded = True
        self.files = {}

    def post(self, url, params, data, **_):
        self.writes.append((url, params['action'], deepcopy(data)))
        action = params['action']
        if action == 'set-pp-routine':
            if not self.execution_type and data['userlevel'] == 'true':
                raise ValueError('SYS_CTRL_E_EXEC_STATE')
            self.module, self.routine = data['module'], data['routine']
            self.execution_type = 'userroutine' if data['userlevel'] == 'true' else 'normal'
        if action == 'abortexeclevel':
            self.module, self.routine, self.execution_type = 'MainModule', 'main', 'normal'
        if action == 'loadmod':
            self.source = self.files[data['modulepath'].split('/')[-1]].decode('ascii').replace('\r\n', '\n')
            self.module_loaded = True
        if action == 'setspeedratio':
            self.speed_ratio = float(data['speed-ratio'])
        if action == 'setctrlstate':
            self.motors = data['ctrl-state']
        if action == 'start':
            self.execution = 'running'
            self.values['autoSesion'] += 1
            self.values['autoActivo'] = 1
            self.values['autoFin'] = 0.
            self.values['autoEstado'] = 0.
            self.values['autoHecho'] = self.values['autoOrden']
        if action == 'stop':
            self.execution = 'stopped'
        if action == 'set':
            name = url.split('/')[-1]
            self.values[name] = data['value']
            if name == 'autoOrden':
                if self.lose_commit_response:
                    raise requests.Timeout('Respuesta de publicación perdida')
                self.values['autoOrden'] = float(data['value'])
                self.values['autoHecho'] = float(data['value'])
                self.values['autoEstado'] = 2.
                self.values['autoUltimo'] = self.values['autoTramo'] = float(self.values['autoCantidad']) or 1.
                self.q = parse_target(self.values['autoDestino'])[0]
        return type('Response', (), {'status_code': 204})()

    def close(self):
        self.closed = True

    def put(self, url, data, **_):
        self.files[url.split('/')[-1]] = data
        self.writes.append((url, 'PUT', None))
        return type('Response', (), {'status_code': 201})()

    def get(self, url, **_):
        content = self.files.get(url.split('/')[-1])
        return type('Response', (), {'status_code': 200 if content is not None else 404,
                                     'content': content})()


class FakeClient:
    def __init__(self, session, base, **_):
        self.session, self.base = session, base
        self.extra_reads = set()

    def read(self, path, **_):
        rws = self.session
        states = {
            '/rw/panel/opmode': [{'opmode': rws.mode}],
            '/rw/panel/ctrlstate': [{'ctrlstate': rws.motors}],
            '/rw/rapid/execution': [{'ctrlexecstate': rws.execution}],
            '/rw/panel/speedratio': [{'speedratio': rws.speed_ratio}],
            TASK+'/pcp': [{'_title': 'progpointer', 'modulemame': rws.module,
                          'routinename': rws.routine, 'executiontype': rws.execution_type}],
            '/rw/rapid/modules': [{'name': MODULE}] if rws.module_loaded else [],
            TASK+'/program/builderror': [],
            MODULE_PATH: [{'module-text': rws.source}],
            TASK+'/motion': [rws.q, [9e9]*6],
            TCP_PATH: [{'z': rws.model.tip(rws.q, rws.envelope)[2, 3]*1000}],
        }
        if path.startswith(SYMBOL):
            name = path[len(SYMBOL):]
            value = rws.values[name]
            if getattr(rws, 'bad_route_readback', False) and name == 'autoRuta':
                value = '[]'
            if rws.bad_readback and name == 'autoDestino':
                value = literal([[0.]*6, [9e9]*6])
            return [{'value': str(value)}]
        return states[path]

    def verify_identity(self):
        pass

    def resolve_module_text(self, records, module):
        pass


class AutomaticProtocolTest(unittest.TestCase):
    def test_pointer_without_context_uses_normal_routine_without_start(self):
        self.fake.module = self.fake.routine = self.fake.execution_type = None
        self.control.prepare_pointer(take_over=True)
        writes = [(action, data) for _, action, data in self.fake.writes]
        self.assertEqual(writes, [('set-pp-routine', {'module': MODULE, 'routine': ROUTINE, 'userlevel': 'false'})])
        self.assertEqual(self.fake.execution, 'stopped')
        self.assertEqual(self.fake.motors, 'motoroff')

    @classmethod
    def setUpClass(cls):
        cls.model = Model()
        cls.envelope = configured_envelope(require_floor=False)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.fake = FakeRws(self.model, self.envelope)
        for replacement in (
            patch('control_automatico.requests.Session', return_value=self.fake),
            patch('control_automatico.Client', FakeClient),
            patch('control_automatico.measured_target', side_effect=lambda rows: rows),
            patch('control_automatico.logout'),
            patch('control_automatico.Path.home', return_value=Path(self.temp.name)),
        ):
            replacement.start()
            self.addCleanup(replacement.stop)
        self.cancelled = Event()
        self.control = AutomaticSession(
            'http://abb-simulado', ('usuario', 'clave-simulada'),
            self.model, self.envelope, load(), self.cancelled, lambda _: None)

    def test_manual_and_protective_stop_produce_no_writes(self):
        for mode, motors in (('MANR', 'motoron'), ('AUTO', 'guardstop'), ('AUTO', 'emergencystop')):
            self.fake.mode, self.fake.motors = mode, motors
            with self.assertRaises(ValueError):
                self.control.start()
            self.assertEqual(self.fake.writes, [])

    def test_modified_module_and_pending_order_produce_no_writes(self):
        self.fake.source = self.fake.source.replace('objetivo,vmax,fine', 'objetivo,v100,fine')
        with self.assertRaises(ValueError):
            self.control.start()
        self.assertEqual(self.fake.writes, [])
        self.fake.source = (ROOT/'rapid'/f'{MODULE}.mod').read_text(encoding='ascii')
        self.fake.values['autoOrden'] = 7.
        with self.assertRaises(ValueError):
            self.control.start()
        self.assertEqual(self.fake.writes, [])

    def test_start_arms_without_a_motion_command(self):
        before = list(self.fake.q)
        self.assertEqual(self.control.start(), before)
        self.assertEqual(self.fake.q, before)
        self.assertTrue(self.fake.values['autoActivo'])
        self.assertFalse(any(action == 'set' for _, action, _ in self.fake.writes))
        self.assertEqual(self.fake.writes[-1][1], 'release')

    def test_mechanical_not_ready_does_not_enable_motors_or_start(self):
        read = self.control.client.read
        def mechanical_error(path, **params):
            if path == TASK+'/motion':
                raise ValueError('HTTP 400; code=-1073436650')
            return read(path, **params)
        with patch.object(self.control.client, 'read', side_effect=mechanical_error):
            with self.assertRaisesRegex(ValueError, 'contadores de vueltas'):
                self.control.connect(ROOT/'rapid'/f'{MODULE}.mod')
        self.assertEqual(self.fake.writes, [])

    def test_one_goal_is_read_back_then_published_once(self):
        start = self.control.start()
        self.fake.speed_ratio = 25.
        target = start.copy()
        target[0] += 1.
        result = self.control.execute(start, target)
        self.assertEqual(result['max_error_deg'], 0.)
        self.assertEqual(self.fake.q, target)
        self.assertEqual(self.control.progress['goal_deg'], target)
        self.assertEqual(self.control.progress['goal_seq'], result['seq'])
        goals = [(url, data) for url, action, data in self.fake.writes if action == 'set']
        self.assertEqual([url.split('/')[-1] for url, _ in goals],
                         ['autoInicio', 'autoDestino', 'autoCantidad', 'autoOrden'])
        self.assertEqual(self.fake.writes[-1][1], 'release')

    def test_start_with_motors_already_on_avoids_redundant_enable(self):
        self.fake.motors = 'motoron'
        self.control.start()
        self.assertFalse(any(action == 'setctrlstate' for _, action, _ in self.fake.writes))
        self.assertEqual(self.fake.execution, 'running')

    def test_start_resets_inherited_full_speed_before_enable_and_start(self):
        self.fake.speed_ratio = 100.
        self.control.start()
        self.assertEqual(self.fake.speed_ratio, 0.)
        actions = [action for _, action, _ in self.fake.writes]
        self.assertLess(actions.index('setspeedratio'), actions.index('setctrlstate'))
        self.assertLess(actions.index('setspeedratio'), actions.index('start'))

    def test_unconfirmed_speed_reset_never_starts_rapid_or_enables_motors(self):
        post = self.fake.post
        def reject_reset(url, params, data, **kwargs):
            result = post(url, params, data, **kwargs)
            if params['action'] == 'setspeedratio':
                self.fake.speed_ratio = 100.
            return result
        with patch.object(self.fake, 'post', side_effect=reject_reset):
            with self.assertRaisesRegex(ValueError, 'velocidad inicial 0%'):
                self.control.start()
        self.assertFalse(any(action in ('start', 'setctrlstate')
                             for _, action, _ in self.fake.writes))

    def test_bad_readback_never_publishes_sequence(self):
        start = self.control.start()
        self.fake.speed_ratio = 25.
        self.fake.bad_readback = True
        target = start.copy()
        target[0] += 1.
        with self.assertRaises(ValueError):
            self.control.execute(start, target)
        self.assertFalse(any(url.endswith('autoOrden') for url, _, _ in self.fake.writes))

    def test_ambiguous_commit_stops_and_never_retries(self):
        start = self.control.start()
        self.fake.speed_ratio = 25.
        self.fake.lose_commit_response = True
        target = start.copy()
        target[0] += 1.
        with self.assertRaises(requests.Timeout):
            self.control.execute(start, target)
        self.assertEqual(sum(url.endswith('autoOrden') for url, _, _ in self.fake.writes), 1)
        self.assertEqual(self.fake.execution, 'stopped')

    def test_new_rapid_session_and_mode_change_revoke_readiness(self):
        self.control.start()
        self.fake.values['autoSesion'] += 1.
        with self.assertRaises(ValueError):
            self.control.ready()
        self.fake.values['autoSesion'] -= 1.
        self.fake.mode = 'MANR'
        with self.assertRaises(ValueError):
            self.control.ready()

    def test_cancelled_session_never_posts_a_goal(self):
        start = self.control.start()
        self.fake.speed_ratio = 25.
        self.cancelled.set()
        target = start.copy()
        target[0] += 1.
        with self.assertRaises(ValueError):
            self.control.execute(start, target)
        self.assertFalse(any(action == 'set' for _, action, _ in self.fake.writes))

    def test_zero_speed_never_publishes_motion_sequence(self):
        start = self.control.start()
        self.fake.speed_ratio = 25.
        self.fake.speed_ratio = 0.
        target = start.copy()
        target[0] += 1.
        with self.assertRaises(ValueError):
            self.control.execute(start, target)
        self.assertFalse(any(url.endswith('autoOrden') for url, _, _ in self.fake.writes))

    def test_route_is_uploaded_and_read_back_before_one_commit(self):
        start = self.control.start()
        self.fake.speed_ratio = 10.
        a, b = list(start), list(start)
        a[0] += 1.
        b[0] += 2.
        result = self.control.execute_route(start, [a, b])
        import json
        array = json.loads(self.fake.values['autoRuta'])
        self.assertEqual(len(array), 20)
        self.assertEqual([v[0] for v in array[:2]], [a, b])
        self.assertEqual(float(self.fake.values['autoCantidad']), 2.)
        self.assertEqual(result['points'], 2)
        self.assertEqual(sum(url.endswith('autoOrden') for url, _, _ in self.fake.writes), 1)

    def test_invalid_later_point_rejects_entire_route_before_commit(self):
        start = self.control.start()
        self.fake.speed_ratio = 10.
        a, b = list(start), list(start)
        a[0] += 1.
        b[0] = 999.
        with self.assertRaisesRegex(ValueError, 'P2'):
            self.control.execute_route(start, [a, b])
        self.assertFalse(any(action == 'set' for _, action, _ in self.fake.writes))

    def test_bad_route_readback_never_publishes_sequence(self):
        start = self.control.start()
        self.fake.speed_ratio = 10.
        target = list(start)
        target[0] += 1.
        self.fake.bad_route_readback = True
        with self.assertRaisesRegex(ValueError, 'lista de puntos'):
            self.control.execute_route(start, [target])
        self.assertFalse(any(url.endswith('autoOrden') for url, _, _ in self.fake.writes))

    def test_single_goal_clears_previous_route_count(self):
        start = self.control.start()
        self.fake.speed_ratio = 10.
        self.fake.values['autoCantidad'] = 7.
        target = list(start)
        target[0] += 1.
        self.control.execute(start, target)
        self.assertEqual(float(self.fake.values['autoCantidad']), 0.)

    def test_fault_at_fourth_point_is_read_even_if_rapid_stopped(self):
        start = self.control.start()
        self.fake.speed_ratio = 10.
        targets = []
        for index in range(1, 6):
            q = list(start)
            q[0] += index
            targets.append(q)
        post = self.fake.post
        def reject_fourth(url, params, data, **kwargs):
            result = post(url, params, data, **kwargs)
            if url.endswith('autoOrden'):
                self.fake.execution = 'stopped'
                self.fake.values.update(autoHecho=0., autoEstado=-3., autoTramo=4., autoUltimo=3.,
                                        autoErrorEje=5., autoErrorDelta=.073)
            return result
        with patch.object(self.fake, 'post', side_effect=reject_fourth):
            with self.assertRaisesRegex(ValueError, 'P4.*J5'):
                self.control.execute_route(start, targets)
        self.assertEqual(self.control.last_failure['confirmed_point'], 3)
        self.assertEqual(self.control.last_failure['point'], 4)
        self.assertEqual(self.control.last_failure['kind'], 'rapid_rejected')
        self.assertEqual(sum(url.endswith('autoOrden') for url, _, _ in self.fake.writes), 1)

    def test_skipped_start_keeps_original_point_numbers(self):
        start = self.control.start()
        self.fake.speed_ratio = 10.
        a, b = list(start), list(start)
        a[0] += 1.
        b[0] += 2.
        result = self.control.execute_route(start, [start, a, b])
        self.assertEqual(result['points'], 3)
        self.assertEqual(self.control.progress['confirmed_point'], 3)

    def test_hundred_points_use_five_confirmed_blocks(self):
        start = self.control.start()
        self.fake.speed_ratio = 10.
        targets = []
        for index in range(1, 101):
            q = list(start)
            q[0] += index*.1
            targets.append(q)
        result = self.control.execute_route(start, targets)
        self.assertEqual(result['points'], 100)
        self.assertEqual(sum(url.endswith('autoOrden') for url, _, _ in self.fake.writes), 5)
        self.assertEqual(self.fake.q, targets[-1])

    def test_external_speed_change_stops_the_route(self):
        start = self.control.start()
        self.fake.speed_ratio = 10.
        target = list(start)
        target[0] += 1.
        post = self.fake.post
        def change_override(url, params, data, **kwargs):
            result = post(url, params, data, **kwargs)
            if url.endswith('autoOrden'):
                self.fake.speed_ratio = 20.
            return result
        with patch.object(self.fake, 'post', side_effect=change_override):
            with self.assertRaisesRegex(ValueError, 'porcentaje ABB cambió'):
                self.control.execute_route(start, [target])
        self.assertEqual(self.fake.execution, 'stopped')

    def test_lost_route_commit_response_never_republishes(self):
        start = self.control.start()
        self.fake.speed_ratio = 10.
        target = list(start)
        target[0] += 1.
        self.fake.lose_commit_response = True
        with self.assertRaises(requests.Timeout):
            self.control.execute_route(start, [target])
        self.assertEqual(sum(url.endswith('autoOrden') for url, _, _ in self.fake.writes), 1)
        self.assertEqual(self.fake.execution, 'stopped')

    def test_stop_cancels_pending_order_without_republishing(self):
        self.control.start()
        self.fake.values['autoOrden'] = 1.
        self.fake.values['autoHecho'] = 0.
        self.fake.values['autoEstado'] = 1.
        before = list(self.fake.q)
        self.control.stop()
        self.assertEqual(self.fake.execution, 'stopped')
        self.assertEqual(float(self.fake.values['autoHecho']), 1.)
        self.assertEqual(float(self.fake.values['autoEstado']), -1.)
        self.assertEqual(float(self.fake.values['autoActivo']), 0.)
        self.assertFalse(any(url.endswith('autoOrden') for url, _, _ in self.fake.writes))
        self.cancelled.clear()
        self.control.start()
        self.assertEqual(self.fake.q, before)
        self.assertTrue(any(action == 'abortexeclevel' for _, action, _ in self.fake.writes))

    def test_stop_does_not_acknowledge_order_from_another_program_context(self):
        self.control.start()
        self.fake.values['autoOrden'] = 1.
        self.fake.module = 'OtroModulo'
        with self.assertRaises(ValueError):
            self.control.stop()
        self.assertEqual(float(self.fake.values['autoHecho']), 0.)

    def test_connect_uploads_missing_module_and_starts_without_motion(self):
        self.fake.module_loaded = False
        before = list(self.fake.q)
        self.control.connect(ROOT/'rapid'/f'{MODULE}.mod')
        self.assertTrue(self.fake.module_loaded)
        self.assertEqual(self.fake.q, before)
        self.assertEqual(self.fake.execution, 'running')
        self.assertEqual(sum(action == 'loadmod' for _, action, _ in self.fake.writes), 1)
        self.assertFalse(any(url.endswith('autoOrden') for url, _, _ in self.fake.writes))

    def test_connect_reuses_matching_module_and_replaces_selected_context(self):
        self.fake.execution, self.fake.execution_type = 'running', 'userroutine'
        self.fake.module, self.fake.routine = 'OtroModulo', 'OtraRutina'
        self.control.connect(ROOT/'rapid'/f'{MODULE}.mod')
        self.assertEqual((self.fake.module, self.fake.routine), (MODULE, ROUTINE))
        self.assertFalse(any(action == 'loadmod' for _, action, _ in self.fake.writes))
        self.assertTrue(any(action == 'stop' for _, action, _ in self.fake.writes))

    def test_connect_backs_up_changed_module_before_replacement(self):
        self.fake.source = self.fake.source.replace('objetivo,vmax,fine', 'objetivo,v100,fine')
        previous = self.fake.source
        self.control.connect(ROOT/'rapid'/f'{MODULE}.mod')
        self.assertEqual(self.control.log.with_suffix('.anterior.mod').read_text(encoding='utf-8'), previous)
        self.assertEqual(self.fake.source, (ROOT/'rapid'/f'{MODULE}.mod').read_text(encoding='ascii'))

    def test_connect_rejects_incompatible_file_before_writing(self):
        selected = Path(self.temp.name)/'otro.mod'
        selected.write_text('MODULE Otro\nENDMODULE', encoding='ascii')
        with self.assertRaises(ValueError):
            self.control.connect(selected)
        self.assertEqual(self.fake.writes, [])
