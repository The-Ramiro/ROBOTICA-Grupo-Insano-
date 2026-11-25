import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
import unittest
import numpy as np
from calibracion_tiempos import MODEL_PATH,load_calibration,fit_sessions
from referencia_recorrido import make_reference

class TimeCalibrationTest(unittest.TestCase):
    def test_motion_time_is_predicted_within_five_percent_of_recorded_runs(self):
        model=json.loads(MODEL_PATH.read_text(encoding='utf-8'))
        for session in model['sessions']:
            rows=[r for r in model['observations'] if r['session']==session['id']]
            ref=make_reference(rows[0]['start'],[r['target'] for r in rows],session['speed'])
            self.assertTrue(ref['calibrated'])
            observed = rows[-1]['end_s']-session['startup_s']
            predicted = ref['marks'][-1][0]-ref['startup_s']
            self.assertLess(abs(predicted-observed), observed*.05)
            np.testing.assert_allclose(ref['q'][-1],rows[-1]['target'])
            np.testing.assert_allclose(ref['qd'][-1],np.zeros(6))
        for result in model['validation_leave_one_session_out']:
            # La diferencia entre repeticiones también incluye latencia y confirmación.
            self.assertLess(abs(result['error_s']),.10*result['measured_s'])

    def test_report_and_preview_have_identical_time_and_angle_samples(self):
        from objetivos_preview import Model,configured_envelope
        from analisis_recorrido import report
        model=load_calibration();session=model['sessions'][0]
        rows=[r for r in model['observations'] if r['session']==session['id']]
        start=rows[0]['start'];targets=[r['target'] for r in rows]
        ref=make_reference(start,targets,50.)
        data=report(Model(),configured_envelope(require_floor=False),start,targets,50.,dynamics=False)
        np.testing.assert_allclose(data['t'],ref['t'])
        np.testing.assert_allclose(np.degrees(data['q']),ref['q'])
        np.testing.assert_allclose(np.degrees(data['qd']),ref['qd'])
        self.assertGreater(ref['startup_s'],0.)
        self.assertEqual(ref['t'][0],0.)
        self.assertEqual(ref['t'][1],ref['startup_s'])

    def test_missing_or_invalid_model_falls_back_to_nominal_profile(self):
        with TemporaryDirectory() as folder:
            path=Path(folder)/'model.json'
            with patch('calibracion_tiempos.MODEL_PATH',path):
                self.assertIsNone(load_calibration())
                self.assertFalse(make_reference([0.]*6,[[10.]*6]).get('calibrated',False))
                path.write_text('{invalid')
                self.assertIsNone(load_calibration())
        with self.assertRaises(ValueError):fit_sessions([])
