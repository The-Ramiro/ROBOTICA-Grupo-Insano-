from concurrent.futures import CancelledError
from threading import Event
import unittest
import numpy as np
from objetivos_preview import Model, configured_envelope
from analisis_recorrido import report, singular_metrics, geometry_metrics, matrix_metrics, mobility_state
from registro_robotica import Recorder


class ReportTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model, cls.envelope = Model(), configured_envelope(require_floor=False)

    def test_full_route_times_and_no_assumed_start(self):
        points = [[0., -30., 30., 0., 45., 0.], [5., -30., 30., 0., 45., 0.], [10., -30., 30., 0., 45., 0.]]
        data = report(self.model, self.envelope, None, points, dynamics=False)
        np.testing.assert_allclose(np.degrees(data['q'][0]), points[0])
        np.testing.assert_allclose(np.degrees(data['q'][-1]), points[-1])
        self.assertTrue(np.all(np.diff(data['t']) > 0))
        self.assertEqual(len(data['rows']), 3)
        self.assertFalse(data['initial_known'])
        self.assertTrue(data['rows'][0]['skipped'])
        self.assertEqual(len(data['phases']), 4)

    def test_wrist_singularity_is_localized_inside_segment(self):
        a, b = [0., -30., 30., 0., -10., 0.], [0., -30., 30., 0., 10., 0.]
        data = report(self.model, self.envelope, a, [b], dynamics=False)
        index = int(np.argmin(data['sigma']))
        self.assertLess(data['sigma'][index], 1e-7)
        self.assertLess(index, len(data['t'])-1)
        self.assertGreater(index, 0)
        self.assertTrue(data['rows'][0]['near'])
        self.assertEqual(int(data['rank'][index]), 5)
        self.assertEqual(data['rows'][0]['rank'], 5)
        self.assertLess(data['rows'][0]['margin'], 1e-6)
        self.assertGreater(data['rows'][0]['critical_time'], 0.)
        self.assertIn('Singular', mobility_state(data['mobility'][index], data['rank'][index]))
        self.assertEqual(data['rank'][0], 6)
        self.assertEqual(data['rank'][-1], 6)
        warning = next(w for w in data['rows'][0]['warnings'] if w['name'] == 'Muñeca')
        self.assertGreater(warning['first'], 0.)
        self.assertLess(warning['first'], warning['time'])
        self.assertLess(warning['value'], 1e-6)

    def test_geometric_cases_match_the_urdf_not_a_zero_elbow_assumption(self):
        import math
        q = [0., -30., 0., 0., 30., 0.]
        normal = geometry_metrics(self.model.frames(q))
        self.assertAlmostEqual(normal['wrist'], 30., places=6)
        self.assertGreater(normal['elbow'], 70.)
        q[2] = -math.degrees(math.atan2(.302, .07))
        straight = geometry_metrics(self.model.frames(q))
        self.assertLess(straight['elbow'], 1e-5)
        self.assertLess(singular_metrics(self.model, self.envelope, q)[0], 1e-7)
        q = [0., -math.degrees(math.atan2(.302, .27+.07)), 0., 0., 30., 0.]
        shoulder = geometry_metrics(self.model.frames(q))
        self.assertLess(shoulder['shoulder'], 1e-7)
        self.assertLess(singular_metrics(self.model, self.envelope, q)[0], 1e-7)

    def test_cancellation_terminates_report(self):
        cancelled = Event()
        cancelled.set()
        with self.assertRaises(CancelledError):
            report(self.model, self.envelope, None, [[0.]*6], cancelled=cancelled)

    def test_rank_loss_can_exceed_one_and_nearness_is_not_rank_loss(self):
        jac = np.diag([1., 1., 1., 5., 0., 0.])
        sigma, condition, _, rank = matrix_metrics(jac)
        self.assertEqual(rank, 4)
        self.assertEqual(sigma, 0.)
        self.assertIn('4/6', mobility_state(100./condition, rank))
        jac = np.diag([1., 1., 1., 5., 5., .005])
        sigma, condition, _, rank = matrix_metrics(jac)
        self.assertEqual(rank, 6)
        self.assertIn('Cerca', mobility_state(100./condition, rank))

    def test_no_samples_are_not_fabricated(self):
        recorder = Recorder()
        with self.assertRaises(ValueError):
            recorder.arrays()
        recorder.samples.append(dict(t=0., q=[0.]*6, xyz=[0.]*3, sigma=0., fk_tcp_error_mm=None))
        data = recorder.arrays()
        self.assertTrue(np.all(np.isnan(data['qd'])))
        self.assertTrue(np.all(np.isnan(data['qdd'])))
        self.assertTrue(np.all(np.isnan(data['error'])))

    def test_nonuniform_measured_velocity_and_missing_tcp(self):
        recorder = Recorder()
        for t in [0., .1, .23, .4]:
            recorder.samples.append(dict(t=t, q=[t*10]*6, xyz=[0.]*3, sigma=.01, fk_tcp_error_mm=None))
        data = recorder.arrays()
        np.testing.assert_allclose(data['qd'], np.radians(10.), atol=1e-9)
        self.assertTrue(np.all(np.isnan(data['error'])))

    def test_execution_session_resets_then_freezes_on_completion(self):
        from types import SimpleNamespace
        from concurrent.futures import Future
        import time
        from unittest.mock import Mock
        from unittest.mock import patch
        clock=[100.]
        with patch('registro_robotica.time.monotonic',side_effect=lambda:clock[0]):
            reader=Mock()
            p=SimpleNamespace(status=SimpleNamespace(text=lambda:''),reader=reader,
                model=self.model,envelope=self.envelope,motion=Future(),
                auto_session=SimpleNamespace(progress={'point':2}))
            recorder=Recorder();recorder.archive=Mock()
            def sample(seq):
                clock[0]+=.05
                reader.snapshot.return_value=(seq,[0.]*6,clock[0],'',None,None,'',None)
                recorder.capture(p)
            sample(0)
            self.assertEqual(len(recorder.samples),0)
            recorder.begin_execution(p.motion)
            session=recorder.session_id
            sample(1)
            self.assertEqual(recorder.samples[-1]['rank'],5)
            self.assertTrue(recorder.active)
            p.motion.set_result(None)
            sample(2);sample(3)
            self.assertFalse(recorder.active)
            count=len(recorder.samples)
            sample(4)
            self.assertEqual(len(recorder.samples),count)
            self.assertEqual(recorder.state,'completed')
            p.motion=Future();recorder.begin_execution(p.motion)
            self.assertNotEqual(recorder.session_id,session)
            self.assertEqual(len(recorder.samples),0)
            self.assertEqual(len(recorder.dynamic_samples),0)
            sample(5)
            self.assertLess(recorder.samples[-1]['t'],.1)
            self.assertEqual(recorder.samples[-1]['run'],1)
