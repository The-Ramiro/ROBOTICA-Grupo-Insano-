import unittest
from unittest.mock import patch
import numpy as np
from referencia_recorrido import make_reference,comparison_curves

class ReferenceTest(unittest.TestCase):
    @patch('referencia_recorrido.load_calibration', return_value=None)
    def test_override_scales_time_velocity_and_acceleration_even_on_short_moves(self, _calibration):
        fast=make_reference([0.]*6,[[1.]*6],speed=100.)
        slow=make_reference([0.]*6,[[1.]*6],speed=25.)
        lead=fast.get('startup_s',0.);tail=fast.get('completion_s',0.)
        moving=np.asarray(fast['t'])[2:-1] if lead else np.asarray(fast['t'])
        slow_moving=np.asarray(slow['t'])[2:-1] if lead else np.asarray(slow['t'])
        np.testing.assert_allclose(slow_moving-lead,(moving-lead)*4.)
        self.assertAlmostEqual(slow['t'][-1]-lead-tail,(fast['t'][-1]-lead-tail)*4.)
        np.testing.assert_allclose(slow['q'],fast['q'])
        np.testing.assert_allclose(slow['qd'],np.asarray(fast['qd'])*.25)
        np.testing.assert_allclose(slow['qdd'],np.asarray(fast['qdd'])*.25**2)
        with self.assertRaises(ValueError):make_reference([0.]*6,[[1.]*6],speed=0.)

    def test_preview_uses_selected_percent_execution_uses_confirmed_percent(self):
        from types import SimpleNamespace
        from referencia_recorrido import profile_settings
        p=SimpleNamespace(real_speed_value=SimpleNamespace(value=lambda:25.),speed_actual={'ratio':50.})
        self.assertEqual(profile_settings(p)[0],25.)
        self.assertEqual(profile_settings(p,confirmed=True)[0],50.)

    def test_retime_does_not_change_saved_reference_and_matches_new_profile(self):
        from referencia_recorrido import rescale_reference
        old=make_reference([0.]*6,[[10.]*6],speed=50.)
        for speed in (25., 75., 100.):
            new=rescale_reference(old,speed)
            expected=make_reference([0.]*6,[[10.]*6],speed=speed)
            self.assertEqual(old['speed'],50.)
            for key in ('t','q','qd','qdd','marks'):
                np.testing.assert_allclose(new[key],expected[key])

    def test_full_reference_exists_before_measurements_and_measured_future_is_blank(self):
        ref=make_reference([0.]*6,[[10.]*6,[20.]*6],speed=10.)
        self.assertGreater(ref['t'][-1],0.)
        np.testing.assert_allclose(ref['q'][-1],[20.]*6)
        data=comparison_curves(ref,[])
        self.assertTrue(np.isfinite(data['expected']).all())
        self.assertTrue(np.isnan(data['actual']).all())
        rows=[dict(t=0.,q=[0.]*6),dict(t=.1,q=[1.]*6)]
        data=comparison_curves(ref,rows)
        self.assertTrue(np.isnan(data['actual'][data['t']>.1]).all())
        self.assertTrue(np.isfinite(data['expected'][data['t']>.1]).all())
        self.assertEqual(len(ref['marks']),2)

    def test_unknown_reference_is_not_reconstructed_from_real_data(self):
        data=comparison_curves(None,[dict(t=0.,q=[4.]*6)])
        self.assertTrue(np.isnan(data['expected']).all())
        np.testing.assert_allclose(data['actual'],[[4.]*6])

    def test_tracking_aligns_only_the_initial_wait_with_a_fresh_bracket(self):
        from referencia_recorrido import align_start, observed_start
        reference = make_reference([0.]*6, [[10.]*6, [20.]*6], speed=60.)
        rows = [dict(t=1.3, q=[.001]*6), dict(t=1.5, q=[.6]*6)]
        aligned = align_start(reference, observed_start(reference, rows))
        self.assertAlmostEqual(aligned['startup_s'], 1.4)
        np.testing.assert_allclose(np.diff(aligned['t'][1:]), np.diff(reference['t'][1:]))
        self.assertEqual(aligned['q'], reference['q'])
        self.assertNotEqual(aligned['marks'], reference['marks'])
        self.assertIs(align_start(reference, observed_start(reference, rows[1:])), reference)
        self.assertIsNone(observed_start(reference, [rows[0], dict(t=2.0, q=[.6]*6)]))

    def test_tracking_estimates_high_speed_start_across_a_telemetry_gap(self):
        from referencia_recorrido import inferred_start, observed_start
        reference = make_reference([0.]*6, [[40.]*6], speed=60.)
        start = 1.2
        rows = [dict(t=.1, q=[0.]*6)]
        for fraction in (.07, .15, .22):
            planned = np.interp(40*fraction, np.asarray(reference['q'])[:, 0], reference['t'])
            rows.append(dict(t=start+planned-reference['startup_s'], q=[40*fraction]*6))
        self.assertIsNone(observed_start(reference, rows))
        self.assertAlmostEqual(inferred_start(reference, rows), start)
        self.assertIsNone(inferred_start(dict(reference, speed=40.), rows))
        self.assertIsNone(inferred_start(reference, rows[:-1]))
        rows[-1]['q'][0] += 3.
        self.assertIsNone(inferred_start(reference, rows))

    def test_profile_is_saved_with_session_and_independent_of_edited_targets(self):
        from registro_robotica import Recorder
        from tempfile import TemporaryDirectory
        from pathlib import Path
        targets=[[10.]*6]
        reference=make_reference([0.]*6,targets)
        targets[0][0]=99.
        self.assertEqual(reference['q'][-1][0],10.)
        recorder=Recorder();recorder.reference=reference
        with TemporaryDirectory() as folder:
            path=Path(folder)/'session.json';recorder.save(path)
            restored=Recorder.load(path)
        np.testing.assert_allclose(restored.reference['q'],reference['q'])
