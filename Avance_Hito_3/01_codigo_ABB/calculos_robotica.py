"""Pose, Jacobiano, LSPB y dinámica estimada a partir del URDF del proyecto."""
import math
from concurrent.futures import CancelledError

import numpy as np

from alternativas_irb120 import branches
from objetivos_preview import origin, rotation_vector, transform
from preparar_paso import ProposalRejected
from limites_movimiento import FACTORY


def rpy_degrees(rotation):
    cosine = math.hypot(rotation[0, 0], rotation[1, 0])
    pitch = math.atan2(-rotation[2, 0], cosine)
    if cosine > 1e-8:
        roll, yaw = math.atan2(rotation[2, 1], rotation[2, 2]), math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll, yaw = math.atan2(-rotation[1, 2], rotation[1, 1]), 0.
    return np.degrees([roll, pitch, yaw])


def rotation_rpy(degrees):
    values = np.asarray(degrees, dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError('Roll, pitch y yaw deben ser tres números finitos.')
    return transform(rpy=np.radians(values))[:3, :3]


def quaternion_rotation(values):
    q = np.asarray(values, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-8:
        raise ValueError('Cuaternión de orientación inválido.')
    x, y, z, w = q/np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def rotation_quaternion(r):
    k = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[1, 0]+r[0, 1], r[2, 0]+r[0, 2], r[2, 1]-r[1, 2]],
        [r[1, 0]+r[0, 1], r[1, 1]-r[0, 0]-r[2, 2], r[2, 1]+r[1, 2], r[0, 2]-r[2, 0]],
        [r[2, 0]+r[0, 2], r[2, 1]+r[1, 2], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)],
    ])/3
    q = np.linalg.eigh(k)[1][:, -1]
    return q if q[3] >= 0 else -q


def solve_pose(model, reference, xyz, rotation, envelope, bounds):
    xyz, rotation = np.asarray(xyz, dtype=float), np.asarray(rotation, dtype=float)
    if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
        raise ProposalRejected('XYZ debe contener tres números finitos.')
    if (rotation.shape != (3, 3) or not np.all(np.isfinite(rotation))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(rotation), 1.)):
        raise ProposalRejected('La orientación solicitada debe ser una rotación válida.')
    solutions, _ = branches(model, envelope, xyz, rotation, reference, bounds)
    if not solutions:
        raise ProposalRejected('Pose fuera de alcance para la orientación, altura o límites elegidos.')
    return list(solutions[0])


def geometric_jacobian(model, q, point, link='tool0', frames=None):
    frames = model.frames(q) if frames is None else frames
    joints = {joint.find('child').get('link'): joint for joint in model.joints}
    jac = np.zeros((6, 6))
    while link != 'base_link':
        joint = joints[link]
        parent = joint.find('parent').get('link')
        if joint.get('type') == 'revolute':
            pose = frames[parent] @ origin(joint.find('origin'))
            axis = pose[:3, :3] @ np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            axis /= np.linalg.norm(axis)
            index = int(joint.get('name').split('_')[-1])-1
            jac[:3, index] = np.cross(axis, point-pose[:3, 3])
            jac[3:, index] = axis
        link = parent
    return jac


def lspb(start, target, duration, blend, vmax, amax, count=181):
    """Mismas mezclas parabólicas que hito2/irb120/lspb.py. Ángulos en rad."""
    start, target, vmax = (np.asarray(v, dtype=float) for v in (start, target, vmax))
    if (start.shape != (6,) or target.shape != (6,) or vmax.shape != (6,)
            or not all(np.all(np.isfinite(v)) for v in (start, target, vmax))):
        raise ValueError('LSPB requiere seis ángulos y seis límites de velocidad finitos.')
    if not all(math.isfinite(v) for v in (duration, blend, amax)) or duration <= 0 or not 0 < blend <= .5 or amax <= 0 or np.any(vmax <= 0):
        raise ValueError('Duración, velocidad y aceleración deben ser positivas. La mezcla debe estar entre 0 y 50%.')
    delta = target-start
    time_velocity = np.max(np.abs(delta)/vmax)/(1-blend)
    time_acceleration = math.sqrt(np.max(np.abs(delta))/(amax*blend*(1-blend)))
    total = max(duration, float(time_velocity), time_acceleration)
    tb = total*blend
    velocity, acceleration = delta/(total-tb), delta/((total-tb)*tb)
    times = np.unique(np.r_[np.linspace(0, total, count), tb, total-tb])
    s, sd, sdd = [], [], []
    for t in times:
        if t < tb:
            s.append(start+.5*acceleration*t*t)
            sd.append(acceleration*t)
            sdd.append(acceleration)
        elif t <= total-tb:
            s.append(start+velocity*(t-.5*tb))
            sd.append(velocity)
            sdd.append(np.zeros(6))
        else:
            remaining = total-t
            s.append(target-.5*acceleration*remaining*remaining)
            sd.append(acceleration*remaining)
            sdd.append(-acceleration)
    return dict(t=times, q=np.array(s), qd=np.array(sd), qdd=np.array(sdd),
                duration=total, blend_time=tb, velocity=velocity, acceleration=acceleration)


class Dynamics:
    """τ=M q̈+C q̇+g, estimado con masas/inercias del URDF, sin fricción ni motor."""
    def __init__(self, model):
        self.model = model
        self.links = []
        for link in model.xml.findall('link'):
            inertial = link.find('inertial')
            if inertial is None or link.get('name') == 'base_link':
                continue
            mass = float(inertial.find('mass').get('value'))
            data = inertial.find('inertia')
            inertia = np.array([[float(data.get('i'+a+b if a <= b else 'i'+b+a))
                                 for b in 'xyz'] for a in 'xyz'])
            if mass <= 0 or np.min(np.linalg.eigvalsh(inertia)) <= 0:
                raise ValueError('Masa o tensor de inercia inválido en '+link.get('name'))
            self.links.append((link.get('name'), mass, origin(inertial.find('origin')), inertia))

    def matrices(self, q):
        frames = self.model.frames(q)
        mass_matrix, gravity, potential = np.zeros((6, 6)), np.zeros(6), 0.
        for link, mass, offset, inertia in self.links:
            com = frames[link] @ offset
            jac = geometric_jacobian(self.model, q, com[:3, 3], link, frames)
            inertia_world = com[:3, :3] @ inertia @ com[:3, :3].T
            mass_matrix += mass*jac[:3].T @ jac[:3] + jac[3:].T @ inertia_world @ jac[3:]
            gravity += mass*9.81*jac[2]
            potential += mass*9.81*com[2, 3]
        return mass_matrix, gravity, potential

    def evaluate(self, q, qd, qdd):
        mass, gravity, potential = self.matrices(q)
        h = 1e-5
        derivative = []
        for index in range(6):
            offset = np.eye(6)[index]*math.degrees(h)
            derivative.append((self.matrices(np.asarray(q)+offset)[0]
                               - self.matrices(np.asarray(q)-offset)[0])/(2*h))
        derivative = np.asarray(derivative)
        coriolis = (np.einsum('kij,j,k->i', derivative, qd, qd)
                    - .5*np.einsum('ijk,j,k->i', derivative, qd, qd))
        inertia = mass @ qdd
        return dict(M=mass, g=gravity, c=coriolis, inertia=inertia,
                    tau=inertia+coriolis+gravity, kinetic=.5*qd @ mass @ qd, potential=potential)


def analyze(model, envelope, start, target, bounds, duration, blend, speed_percent, acceleration, *, cancelled=None):
    vmax = np.radians([row[2] for row in FACTORY])*speed_percent/100
    path = lspb(np.radians(start), np.radians(target), duration, blend, vmax, math.radians(acceleration))
    positions = np.degrees(path['q'])
    height = []
    for q in positions:
        if cancelled is not None and cancelled.is_set():
            raise CancelledError()
        height.append(model.tip(q, envelope)[2, 3])
    tip = model.tip(target, envelope)
    jac = geometric_jacobian(model, target, tip[:3, 3])
    scaled = jac.copy()
    scaled[3:] *= .2
    singular = np.linalg.svd(scaled, compute_uv=False)
    dynamics = Dynamics(model)
    snapshot = dynamics.evaluate(target, np.zeros(6), np.zeros(6))
    indices = np.unique(np.r_[0, np.linspace(0, len(positions)-1, 25, dtype=int), len(positions)-1])
    evaluated = []
    for i in indices:
        if cancelled is not None and cancelled.is_set():
            raise CancelledError()
        evaluated.append(dynamics.evaluate(positions[i], path['qd'][i], path['qdd'][i]))
    return dict(path=path, start=np.array(start), target=np.array(target), jac=jac,
                singular=singular, T=tip, snapshot=snapshot, dynamic_indices=indices,
                evaluated=evaluated,
                tau=np.array([row['tau'] for row in evaluated]),
                kinetic=np.array([row['kinetic'] for row in evaluated]),
                potential=np.array([row['potential'] for row in evaluated]),
                minimum_height=float(min(height)), moving_mass=sum(row[1] for row in dynamics.links))
