import unittest
import numpy as np
from errores_recorrido import error_series, joint_rates, profile_state, spatial_errors, summarize_errors, tracking_arrays
from objetivos_preview import Model, configured_envelope
from referencia_recorrido import make_reference


class TrackingErrorsTest(unittest.TestCase):
    def test_destination_steps_and_signed_remaining_error(self):
        rows=[dict(t=0.,q=[0.]*6,goal_q=[10.]*6,goal_xyz=[100.,0.,0.],tcp_xyz=[0.,0.,0.]),
              dict(t=.1,q=[10.]*6,goal_q=[10.]*6,goal_xyz=[100.,0.,0.],tcp_xyz=[100.,0.,0.]),
              dict(t=.2,q=[10.]*6,goal_q=[20.]*6,goal_xyz=[200.,0.,0.],tcp_xyz=[100.,0.,0.])]
        data=tracking_arrays(rows)
        np.testing.assert_allclose(data['angular'][:,0],[10.,0.,10.])
        np.testing.assert_allclose(data['distance'],[100.,0.,100.])

    def test_old_records_do_not_invent_reference_or_measured_tcp(self):
        data=tracking_arrays([dict(t=0.,q=[0.]*6,xyz=[1.,2.,3.])])
        self.assertTrue(np.isnan(data['goal']).all())
        self.assertTrue(np.isnan(data['tcp']).all())
        self.assertTrue(np.isnan(data['distance']).all())

    def test_network_gap_breaks_curves(self):
        row=dict(q=[0.]*6,goal_q=[1.]*6,goal_xyz=[1.]*3,tcp_xyz=[0.]*3)
        data=tracking_arrays([dict(row,t=0.),dict(row,t=1.)])
        self.assertTrue(np.isnan(data['angular'][1]).all())

    def test_spatial_error_for_same_pose_uses_two_distinct_rates(self):
        model = Model()
        envelope = configured_envelope(require_floor=False)
        q = [10., -20., 30., 40., 50., 60.]
        result = spatial_errors(model, envelope, q, q, [1.]*6, [0.]*6)
        np.testing.assert_array_equal(result['position'][2], 0.)
        np.testing.assert_allclose(result['orientation'], 0., atol=1e-12)
        np.testing.assert_array_equal(result['jacobians'][2], 0.)
        jacobian = result['jacobians'][0]
        expected = jacobian @ np.full(6, np.pi/180.)
        np.testing.assert_allclose(result['velocity'][2], np.r_[expected[:3]*1000., np.degrees(expected[3:])])

    def test_spatial_error_is_signed_in_base_and_does_not_invent_missing_rate(self):
        model = Model()
        envelope = configured_envelope(require_floor=False)
        abb, white = [[0., -20., 30., 40., 50., 60.], [10., -20., 30., 40., 50., 60.]]
        result = spatial_errors(model, envelope, white, abb, [0.]*6)
        reverse = spatial_errors(model, envelope, abb, white)
        np.testing.assert_allclose(result['orientation'], [0., 0., 10.], atol=1e-10)
        np.testing.assert_allclose(result['jacobians'][2], -reverse['jacobians'][2])
        np.testing.assert_allclose(result['position'][2], -reverse['position'][2])
        self.assertTrue(np.isnan(result['velocity'][2]).all())

    def test_rate_from_consecutive_samples_and_profile_at_same_time(self):
        before = dict(t=1., q=[0.]*6)
        np.testing.assert_allclose(joint_rates(before, dict(t=1.2, q=[2.]*6)), [10.]*6)
        self.assertIsNone(joint_rates(before, dict(t=1.6, q=[2.]*6)))
        self.assertIsNone(joint_rates(before, before))
        reference = make_reference([0.]*6, [[10.]*6], speed=60.)
        instant = reference['t'][-2]
        q, qd = profile_state(reference, instant)
        np.testing.assert_allclose(q, reference['q'][-2])
        np.testing.assert_allclose(qd, reference['qd'][-2])

    def test_summary_retains_peak_and_counts_only_available_readings(self):
        values = [[1., np.nan, 0.], [4., np.nan, 2.], [3., np.nan, 3.], [0., np.nan, 0.]]
        self.assertEqual(summarize_errors(values, [2., 1., 3.]), [(1, 2), None, (2, 0)])

    def test_plot_metrics_keep_units_and_preserve_missing_velocity(self):
        model = Model()
        envelope = configured_envelope(require_floor=False)
        q = [0., -30., 30., 0., 45., 0.]
        position, orientation, linear_j, angular_j, linear_v, angular_v = error_series(
            spatial_errors(model, envelope, q, q, [0.]*6))
        for values in (position, orientation, linear_j, angular_j):
            np.testing.assert_allclose(values, 0., atol=1e-12)
        self.assertTrue(np.isnan(linear_v).all())
        self.assertTrue(np.isnan(angular_v).all())
