import unittest
import numpy as np
from objetivos_preview import Model, HeightEnvelope, prepare_target, rotation_vector, ProposalRejected


class GoalPreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = Model()
        cls.q = [10., -20., 30., 15., 40., 5.]
        # Geometría sintética de prueba, NO parámetros para el laboratorio.
        cls.envelope = HeightEnvelope(.15, -.01, (0., 0., 0.))

    def test_xyz_preserves_orientation_and_reaches_point(self):
        initial = self.model.tip(self.q, self.envelope)
        goal = initial[:3, 3]+[.002, 0, .001]
        p = prepare_target(self.model, self.q, envelope=self.envelope, xyz_m=list(goal))
        final = self.model.tip(p['target_deg'], self.envelope)
        self.assertLess(np.linalg.norm(final[:3, 3]-goal), 1e-5)
        self.assertLess(np.linalg.norm(rotation_vector(initial[:3, :3]@final[:3, :3].T)), 1e-4)
        self.assertFalse(p['hardware_execution_allowed'])

    def test_height_rejects_low_tip(self):
        with self.assertRaises(ProposalRejected):
            prepare_target(self.model, self.q,
                           envelope=HeightEnvelope(.9, -.01, (0., 0., 0.)), target_deg=self.q)

    def test_rejects_floor_penetration_of_robot_body(self):
        with self.assertRaises(ProposalRejected):
            prepare_target(self.model, self.q,
                           envelope=HeightEnvelope(.4, .1, (0., 0., 0.)), target_deg=self.q)

    def test_checks_intermediate_path_not_only_destination(self):
        class ObstacleOnPath:
            def check_height(self, q, envelope):
                if 10.1 < q[0] < 10.4:
                    raise ProposalRejected('Altura intermedia insuficiente')
                return .5
        with self.assertRaises(ProposalRejected):
            prepare_target(ObstacleOnPath(), self.q, envelope=self.envelope,
                           target_deg=[10.5, -20, 30, 15, 40, 5])

    def test_unknown_height_and_ambiguous_modes_are_rejected(self):
        with self.assertRaises(ProposalRejected):
            prepare_target(self.model, self.q, envelope=HeightEnvelope(None, -.01, (0, 0, 0)), target_deg=self.q)
        with self.assertRaises(ProposalRejected):
            prepare_target(self.model, self.q, envelope=self.envelope,
                           target_deg=self.q, xyz_m=[.2, 0, .4])

    def test_preview_without_floor_still_enforces_tip_height(self):
        p = prepare_target(self.model, self.q,
                           envelope=HeightEnvelope(0., None, (0., 0., 0.)), target_deg=self.q)
        self.assertFalse(p['hardware_execution_allowed'])
        self.assertFalse(p['continuous_collision_checked'])
        with self.assertRaises(ProposalRejected):
            prepare_target(self.model, self.q,
                           envelope=HeightEnvelope(.9, None, (0., 0., 0.)), target_deg=self.q)


if __name__ == '__main__':
    unittest.main()
