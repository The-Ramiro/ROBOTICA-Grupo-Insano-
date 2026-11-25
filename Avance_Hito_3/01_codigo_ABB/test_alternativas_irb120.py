import unittest

import numpy as np

from alternativas_irb120 import branches
from limites_movimiento import effective, load
from objetivos_preview import Model, configured_envelope


class IRB120BranchesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = Model()
        cls.envelope = configured_envelope(require_floor=False)
        cls.bounds = [(r['min_deg'], r['max_deg']) for r in effective(load())]

    def solutions(self, angles, bounds=None):
        pose = self.model.tip(angles, self.envelope)
        solutions, singular = branches(
            self.model, self.envelope, pose[:3, 3], pose[:3, :3], angles,
            self.bounds if bounds is None else bounds)
        for solution in solutions:
            np.testing.assert_allclose(
                self.model.tip(solution, self.envelope), pose, atol=2e-5, rtol=0)
        return solutions, singular

    def test_equivalent_turns_follow_configured_joint_limit(self):
        angles = (0., -30., 30., 0., 45., 0.)
        solutions, singular = self.solutions(angles)
        self.assertFalse(singular)
        self.assertEqual(solutions[0], angles)
        equating_turns = {q[5] for q in solutions if q[:5] == angles[:5]}
        self.assertEqual(equating_turns, {-360., 0., 360.})
        smaller = self.bounds[:5] + [(-180., 180.)]
        limited, _ = self.solutions(angles, smaller)
        self.assertTrue(all(-180 <= q[5] <= 180 for q in limited))

    def test_wrist_singularity_keeps_equivalent_roll_splits(self):
        angles = (40., -49., 27., 38., 0., 350.)
        solutions, singular = self.solutions(angles)
        self.assertTrue(singular)
        self.assertEqual(solutions[0], angles)
        self.assertTrue(any(q[3] != angles[3] and q[4] == 0. for q in solutions))

    def test_distinct_elbow_branches_reproduce_same_pose(self):
        angles = (30., 10., -70., 60., 25., 0.)
        solutions, singular = self.solutions(angles)
        self.assertFalse(singular)
        self.assertIn(angles, solutions)
        self.assertTrue(any(q[2] != angles[2] for q in solutions))
        self.assertTrue(any(q[0] != angles[0] for q in solutions))
        self.assertTrue(any(q[4] < 0 for q in solutions))
