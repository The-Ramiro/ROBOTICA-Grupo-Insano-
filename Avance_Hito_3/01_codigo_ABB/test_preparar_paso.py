import math
import unittest
from preparar_paso import prepare_step, send_to_robot, ProposalRejected


class PreviewGuards(unittest.TestCase):
    def test_rejects_invalid_and_stale_commands(self):
        cases = [([0]*5, 1, .5, 10), ([0]*6, 0, .5, 10),
                 ([0]*6, True, .5, 10), ([0]*6, 1, 90, 10),
                 ([0]*6, 1, math.nan, 10), ([0]*6, 1, .5, 9),
                 ([0]*6, 1, .5, 11), ([math.inf]+[0]*5, 1, .5, 10),
                 ([164]+[0]*5, 1, .5, 10), ([0]*6, 1, '0.5', 10)]
        for q, axis, delta, stamp in cases:
            with self.subTest(q=q, axis=axis, delta=delta, stamp=stamp):
                with self.assertRaises(ProposalRejected):
                    prepare_step(q, axis, delta, stamp, now=10)

    def test_one_axis_and_no_wrap(self):
        q = [0, 0, 0, 0, 0, 350]
        proposal = prepare_step(q, 6, .5, 10, now=10)
        self.assertEqual(proposal['target_deg'], [0, 0, 0, 0, 0, 350.5])
        self.assertEqual(proposal['samples'][0]['positions_deg'], q)
        for sample in proposal['samples']:
            self.assertEqual(sample['positions_deg'][:5], q[:5])
            self.assertTrue(350 <= sample['positions_deg'][5] <= 350.5)
        self.assertLessEqual(proposal['peak_speed_deg_s'], 1)
        self.assertLessEqual(proposal['peak_acceleration_deg_s2'], 2)
        self.assertFalse(proposal['hardware_execution_allowed'])
        self.assertFalse(proposal['collision_checked'])

    def test_physical_output_is_blocked(self):
        with self.assertRaises(ProposalRejected):
            send_to_robot({})


if __name__ == '__main__':
    unittest.main()
