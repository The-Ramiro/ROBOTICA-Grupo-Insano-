from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from test_control_automatico import FakeClient, FakeRws
from objetivos_preview import Model, configured_envelope
from velocidad_abb import MODULE, RESOURCE, ROUTINE, SpeedController


class FakeSpeedRws(FakeRws):
    ratio = 100.
    ignore_speed_write = False

    def post(self, url, params, data, **kwargs):
        if params['action'] == 'setspeedratio':
            self.writes.append((url, params['action'], data))
            if not self.ignore_speed_write:
                self.ratio = float(data['speed-ratio'])
            return type('Response', (), {'status_code': 204})()
        return super().post(url, params, data, **kwargs)


class FakeSpeedClient(FakeClient):
    def read(self, path, **params):
        if path == RESOURCE:
            return [{'speedratio': str(self.session.ratio)}]
        return super().read(path, **params)


class SpeedControlTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.fake = FakeSpeedRws(Model(), configured_envelope(require_floor=False))
        for replacement in (
            patch('velocidad_abb.requests.Session', return_value=self.fake),
            patch('velocidad_abb.Client', FakeSpeedClient),
            patch('velocidad_abb.logout'),
            patch('velocidad_abb.Path.home', return_value=Path(self.temp.name)),
        ):
            replacement.start()
            self.addCleanup(replacement.stop)
        self.controller = SpeedController('http://abb-simulado', ('usuario', 'clave-simulada'))

    def speed_writes(self):
        return [row for row in self.fake.writes if row[1] == 'setspeedratio']

    def test_read_is_read_only(self):
        self.assertEqual(self.controller.read()['ratio'], 100.)
        self.assertEqual(self.fake.writes, [])

    def test_stopped_controller_accepts_and_confirms_global_ratio(self):
        result = self.controller.apply(25)
        self.assertEqual(result['ratio'], 25.)
        self.assertEqual(self.speed_writes()[0][2], {'speed-ratio': '25'})
        self.assertEqual(self.fake.writes[-1][1], 'release')

    def test_invalid_values_do_not_write(self):
        for value in (-1, 101, float('nan'), True, '50'):
            with self.assertRaises(ValueError):
                self.controller.apply(value)
        self.assertEqual(self.fake.writes, [])

    def test_manual_and_foreign_program_do_not_change_speed(self):
        self.fake.mode = 'MANR'
        with self.assertRaises(ValueError):
            self.controller.apply(20)
        self.assertEqual(self.fake.writes, [])
        self.fake.mode, self.fake.execution = 'AUTO', 'running'
        with self.assertRaises(ValueError):
            self.controller.apply(20)
        self.assertEqual(self.speed_writes(), [])

    def test_own_server_must_be_idle(self):
        self.fake.execution = 'running'
        self.fake.module, self.fake.routine = MODULE, ROUTINE
        self.fake.values['autoActivo'] = 1.
        self.fake.values['autoOrden'] = 1.
        with self.assertRaises(ValueError):
            self.controller.apply(30)
        self.assertEqual(self.speed_writes(), [])
        self.fake.values['autoHecho'] = 1.
        self.assertEqual(self.controller.apply(30)['ratio'], 30.)

    def test_failed_readback_is_not_retried(self):
        self.fake.ignore_speed_write = True
        with self.assertRaises(ValueError):
            self.controller.apply(40)
        self.assertEqual(len(self.speed_writes()), 1)
        self.assertEqual(self.fake.writes[-1][1], 'release')
