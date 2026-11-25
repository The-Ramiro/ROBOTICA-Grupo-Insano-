import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
import trayectorias as route
from objetivos_preview import Model, configured_envelope
from limites_movimiento import load
from test_control_automatico import ROOT, MODULE
from irb120_control.rws_j6 import source_fingerprint


class RouteTest(unittest.TestCase):
    def test_conservative_height_bound_warns_without_rejecting_route(self):
        from ejecutar_rws import check_goal
        model, envelope, config = Model(), configured_envelope(require_floor=False), load()
        start = [0., -30., 30., 0., 45., 0.]
        target = [0., 60., -50., 0., 45., 0.]
        report = check_goal(model, envelope, config, start, target)
        self.assertTrue(report['height_warning'])
        self.assertLess(report['lower_z_bound_m'], 0.)
        self.assertEqual(route.prepare(model, envelope, config, start, [target]), [target])

    def test_file_roundtrip_and_invalid_formats(self):
        values = [[0., -30., 30., 0., 45., 0.], [1., -30., 30., 0., 45., 0.]]
        with tempfile.TemporaryDirectory() as folder:
            filename = Path(folder)/'ruta.json'
            route.save(filename, values)
            self.assertEqual(route.load(filename), values)
            obj = json.loads(filename.read_text())
            obj['unit'] = 'rad'
            filename.write_text(json.dumps(obj))
            with self.assertRaises(ValueError):
                route.load(filename)

    def test_finite_six_axis_points_and_capacity(self):
        for value in ([], [[0.]*5], [[True]*6], [[float('nan')]*6], [[0.]*6]*101):
            with self.assertRaises(ValueError):
                route.points(value)

    def test_prepare_skips_initial_pose_and_rejects_later_duplicate(self):
        model, envelope, config = Model(), configured_envelope(require_floor=False), load()
        start = [0., -30., 30., 0., 45., 0.]
        target = [1., -30., 30., 0., 45., 0.]
        self.assertEqual(route.prepare(model, envelope, config, start, [start, target]), [target])
        with self.assertRaisesRegex(ValueError, 'P2'):
            route.prepare(model, envelope, config, start, [target, target])

    def test_samples_reach_points_without_joint_overshoot(self):
        initial, target = [0.]*6, [10., -5., 1., 0., 0., 20.]
        path = route.samples(initial, [target])
        np.testing.assert_allclose(path[0], initial)
        np.testing.assert_allclose(path[-1], target)
        self.assertTrue(np.all(path >= np.minimum(initial, target)-1e-9))
        self.assertTrue(np.all(path <= np.maximum(initial, target)+1e-9))

    def test_module_fingerprint_accepts_array_data_but_not_changed_code(self):
        import re
        source = (ROOT/'rapid'/f'{MODULE}.mod').read_text(encoding='ascii')
        replaced = re.sub(r'(PERS jointtarget autoRuta\{20\} := )[^;]+;',
                          lambda m: m[1]+json.dumps([[[0.]*6,[9e9]*6]]*20)+';', source)
        self.assertEqual(source_fingerprint(source), source_fingerprint(replaced))
        self.assertNotEqual(source_fingerprint(source), source_fingerprint(source.replace('objetivo,vmax', 'objetivo,v100')))
