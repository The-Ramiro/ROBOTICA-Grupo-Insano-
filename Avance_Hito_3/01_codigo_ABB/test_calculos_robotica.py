import unittest
from concurrent.futures import CancelledError
from threading import Event

import numpy as np

from calculos_robotica import (
    Dynamics, analyze, geometric_jacobian, lspb, quaternion_rotation, rotation_quaternion,
    rotation_rpy, rpy_degrees, solve_pose,
)
from limites_movimiento import effective, load
from objetivos_preview import Model, configured_envelope, rotation_vector


class RoboticsMathTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = Model()
        cls.envelope = configured_envelope(require_floor=False)
        cls.q = np.array([15., -20., 25., 40., 35., -10.])

    def test_rpy_and_quaternions_at_euler_singularities(self):
        for rpy in ((25, -80, 165), (120, 90, -30), (-45, -90, 78), (179, .1, -179)):
            rotation = rotation_rpy(rpy)
            np.testing.assert_allclose(rotation_rpy(rpy_degrees(rotation)), rotation, atol=1e-12)
            np.testing.assert_allclose(
                quaternion_rotation(rotation_quaternion(rotation)), rotation, atol=1e-12)

    def test_pose_ik_reaches_changed_orientation(self):
        target = self.q + np.array([1., 2., 3., 4., 5., 6.])
        pose = self.model.tip(target, self.envelope)
        bounds = [(r['min_deg'], r['max_deg']) for r in effective(load())]
        solved = solve_pose(self.model, self.q, pose[:3, 3], pose[:3, :3], self.envelope, bounds)
        actual = self.model.tip(solved, self.envelope)
        np.testing.assert_allclose(actual, pose, atol=2e-5, rtol=0)

    def test_geometric_jacobian_matches_fk_derivatives(self):
        pose = self.model.tip(self.q, self.envelope)
        jac = geometric_jacobian(self.model, self.q, pose[:3, 3])
        h = 1e-6
        for axis in range(6):
            delta = np.eye(6)[axis]*np.degrees(h)
            before = self.model.tip(self.q-delta, self.envelope)
            after = self.model.tip(self.q+delta, self.envelope)
            numeric = np.r_[(after[:3, 3]-before[:3, 3])/(2*h),
                            rotation_vector(after[:3, :3] @ before[:3, :3].T)/(2*h)]
            np.testing.assert_allclose(jac[:, axis], numeric, atol=2e-7, rtol=0)

    def test_lspb_endpoints_rest_and_bounds(self):
        target = np.array([.1, -.3, .4, -.5, .8, -1.])
        limits, acceleration = np.ones(6)*.2, .3
        plan = lspb(np.zeros(6), target, .2, .2, limits, acceleration)
        np.testing.assert_allclose(plan['q'][0], 0., atol=1e-12)
        np.testing.assert_allclose(plan['q'][-1], target, atol=1e-12)
        np.testing.assert_allclose(plan['qd'][[0, -1]], 0., atol=1e-12)
        self.assertTrue(np.all(np.abs(plan['qd']) <= limits+1e-12))
        self.assertTrue(np.all(np.abs(plan['qdd']) <= acceleration+1e-12))

    def test_mass_is_positive_and_gravity_is_potential_gradient(self):
        dynamics = Dynamics(self.model)
        mass, gravity, _ = dynamics.matrices(self.q)
        np.testing.assert_allclose(mass, mass.T, atol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(mass).min(), 0.)
        h = 1e-5
        gradient = []
        for axis in range(6):
            offset = np.eye(6)[axis]*np.degrees(h)
            gradient.append((dynamics.matrices(self.q+offset)[2]
                             - dynamics.matrices(self.q-offset)[2])/(2*h))
        np.testing.assert_allclose(gravity, gradient, atol=2e-7)

    def test_coriolis_satisfies_kinetic_energy_identity(self):
        dynamics = Dynamics(self.model)
        velocity = np.array([.1, -.2, .3, -.4, .5, -.6])
        result = dynamics.evaluate(self.q, velocity, np.zeros(6))
        h = 1e-5
        offset = np.degrees(velocity*h)
        derivative = (dynamics.matrices(self.q+offset)[0]
                      - dynamics.matrices(self.q-offset)[0])/(2*h)
        self.assertAlmostEqual(float(velocity @ result['c']),
                               float(.5*velocity @ derivative @ velocity), places=8)
        static = dynamics.evaluate(self.q, np.zeros(6), np.zeros(6))
        np.testing.assert_allclose(static['tau'], static['g'], atol=1e-12)

    def test_obsolete_analysis_can_be_cancelled(self):
        cancelled = Event()
        cancelled.set()
        with self.assertRaises(CancelledError):
            analyze(self.model, self.envelope, self.q, self.q+1., effective(load()),
                    4., .2, 10., 30., cancelled=cancelled)
