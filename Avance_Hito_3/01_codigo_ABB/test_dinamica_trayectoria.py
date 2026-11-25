import unittest
import numpy as np
from objetivos_preview import Model
from calculos_robotica import Dynamics
from dinamica_trayectoria import measured_derivatives, measured_dynamics, planned_dynamics,reconstruct_dynamics


class DynamicsTrajectoryTest(unittest.TestCase):
    def test_derivatives_and_reconstruction_at_four_hz(self):
        times=np.arange(12)*.25
        rows=[dict(t=float(t),q=np.degrees(np.ones(6)*(.1*t+.02*t*t)).tolist(),run=1,point=1,moving=True)
              for t in times]
        qd,qdd=measured_derivatives(rows)
        np.testing.assert_allclose(qd,.1+.04*times[-1],atol=1e-9)
        np.testing.assert_allclose(qdd,.04,atol=1e-9)
        frames=reconstruct_dynamics(Dynamics(Model()),rows)
        self.assertGreater(len(frames),5)
        self.assertEqual(frames[-1]['t'],times[-1])
        np.testing.assert_allclose(frames[-1]['tau'],frames[-1]['inertia']+frames[-1]['c']+frames[-1]['g'])

    def test_causal_derivatives_on_nonuniform_samples_have_correct_units(self):
        velocity=np.arange(1.,7.)*.1
        acceleration=np.arange(1.,7.)*-.2
        times=np.array([0.,.07,.13,.21,.29,.38,.47])
        rows=[dict(t=t,q=np.degrees(velocity*t+.5*acceleration*t*t).tolist()) for t in times]
        qd,qdd=measured_derivatives(rows)
        np.testing.assert_allclose(qd,velocity+acceleration*times[-1],atol=1e-10)
        np.testing.assert_allclose(qdd,acceleration,atol=1e-10)

    def test_gap_or_too_few_samples_never_fabricates_acceleration(self):
        rows=[dict(t=t,q=[0.]*6) for t in [0.,.01,.02,.03,.04,.4]]
        with self.assertRaises(ValueError):measured_derivatives(rows)
        with self.assertRaises(ValueError):measured_derivatives(rows[:3])

    def test_static_six_joint_torque_equals_gravity(self):
        dynamic=Dynamics(Model())
        q=[0.,-30.,30.,0.,45.,0.]
        rows=[dict(t=t,q=q,run=2,point=3,moving=True) for t in np.linspace(0.,.5,9)]
        frame=measured_dynamics(dynamic,rows)
        np.testing.assert_allclose(frame['inertia'],0.,atol=1e-9)
        np.testing.assert_allclose(frame['c'],0.,atol=1e-9)
        np.testing.assert_allclose(frame['tau'],frame['g'],atol=1e-9)
        self.assertEqual(frame['tau'].shape,(6,))
        self.assertEqual(frame['run'],2)

    def test_planned_components_sum_to_total_for_all_six_axes(self):
        dynamic=Dynamics(Model())
        data=dict(t=np.array([0.,.5,1.]),q=np.radians([[0,-30,30,0,45,0]]*3),
            qd=np.array([[.1]*6]*3),qdd=np.array([[.2]*6]*3),
            point_times=[(1.,1)],phases=[(0.,.2,'accel'),(.8,1.,'decel')])
        out=planned_dynamics(dynamic,data)
        np.testing.assert_allclose(out['tau'],out['inertia']+out['c']+out['g'],atol=1e-12)
        self.assertEqual(out['tau'].shape,(3,6))
