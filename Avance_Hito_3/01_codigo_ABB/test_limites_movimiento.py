import unittest
from limites_movimiento import defaults, effective, validate, angle_percent, ProposalRejected
from objetivos_preview import Model, HeightEnvelope, prepare_target


class LimitsTests(unittest.TestCase):
    def test_speed_percent_conversion_and_asymmetric_range(self):
        c = defaults()
        c['global_speed_percent'] = 10
        c['axes'][0]['axis_speed_percent'] = 50
        rows = effective(c)
        self.assertEqual(rows[0]['speed_deg_s'], 12.5)
        self.assertEqual(rows[5]['speed_deg_s'], 42)
        self.assertEqual(angle_percent(2, -110), 0)
        self.assertEqual(angle_percent(2, 70), 100)
        self.assertAlmostEqual(angle_percent(2, 0), 61.11111111)

    def test_cannot_exceed_factory_or_enable_hardware(self):
        for key, value in (('global_speed_percent', 101), ('global_speed_percent', float('nan')),
                           ('physical_execution_allowed', True)):
            c = defaults()
            c[key] = value
            with self.assertRaises(ProposalRejected):
                validate(c)
        c = defaults()
        c['axes'][0]['max_deg'] = 166
        with self.assertRaises(ProposalRejected):
            validate(c)

    def test_planner_obeys_speed_and_command_travel(self):
        model = Model()
        envelope = HeightEnvelope(.15, -.01, (0, 0, 0))
        q = [10, -20, 30, 15, 40, 5]
        c = defaults()
        c['global_speed_percent'] = .1
        target = [10.5, -20, 30, 15, 40, 5]
        p = prepare_target(model, q, envelope=envelope, target_deg=target, motion_config=c)
        self.assertLessEqual(p['peak_speed_deg_s'][0], .25+1e-10)
        c['axes'][0]['max_command_travel_percent'] = .01
        with self.assertRaises(ProposalRejected):
            prepare_target(model, q, envelope=envelope, target_deg=target, motion_config=c)
        c = defaults()
        c['global_speed_percent'] = 0
        with self.assertRaises(ProposalRejected):
            prepare_target(model, q, envelope=envelope, target_deg=target, motion_config=c)


if __name__ == '__main__':
    unittest.main()
