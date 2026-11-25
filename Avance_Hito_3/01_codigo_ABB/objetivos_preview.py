"""Objetivos articulares o XYZ y filtro de altura. Exclusivamente vista previa.

No conecta al controlador. La herramienta y el piso requieren medidas reales.
El muestreo de altura no constituye detección continua de colisiones.
"""
from dataclasses import dataclass
import itertools
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
from preparar_paso import ProposalRejected, finite_number, model_limits
from limites_movimiento import load as load_motion_limits, effective as effective_limits

ROOT = Path(__file__).resolve().parent


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = xyz
    return t


def origin(element):
    if element is None:
        return np.eye(4)
    return transform(*[[float(v) for v in element.get(k, '0 0 0').split()] for k in ('xyz', 'rpy')])


def numbers(values, length, name):
    if not isinstance(values, (list, tuple, np.ndarray)) or len(values) != length:
        raise ProposalRejected(f'{name}: se requieren {length} numeros')
    return np.array([finite_number(v, name) for v in values])


@dataclass(frozen=True)
class HeightEnvelope:
    # Todas las alturas en el marco base_link, en metros. No hay altura predeterminada.
    min_tip_z_m: float
    floor_z_m: float
    tcp_offset_m: tuple

    def validate(self):
        low = finite_number(self.min_tip_z_m, 'Altura minima de punta')
        numbers(self.tcp_offset_m, 3, 'Desplazamiento de herramienta')
        if self.floor_z_m is None:
            return  # Solo vista previa: no hay comprobacion del piso sin medida.
        floor = finite_number(self.floor_z_m, 'Altura del piso')
        if low <= floor:
            raise ProposalRejected('La altura minima de la punta debe estar por encima del piso')


def configured_envelope(path=None, *, require_floor=True):
    config = json.loads(Path(path or ROOT/'configuracion_control.json').read_text(encoding='utf-8'))
    if config.get('frame') != 'base_link' or config.get('tool') != 'tool0':
        raise ProposalRejected('Marco o herramienta distintos de la configuracion prevista')
    if config.get('min_tip_z_m') is None:
        raise ProposalRejected('Falta identificar la altura del punto de giro indicado por el usuario')
    if config.get('floor_z_m') is None and require_floor:
        raise ProposalRejected('Altura de brida guardada; falta definir el plano del piso respecto a la base')
    envelope = HeightEnvelope(config['min_tip_z_m'], config['floor_z_m'], tuple(config['tcp_offset_m']))
    envelope.validate()
    return envelope


class Model:
    def __init__(self):
        self.xml = ET.parse(ROOT/'urdf/irb120.urdf').getroot()
        self.joints = self.xml.findall('joint')
        self.boxes = []
        # Cajas conservadoras de las mallas del modelo, no del entorno del laboratorio.
        for link in self.xml.findall('link'):
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    continue
                path = ROOT/mesh.get('filename').replace('package://abb_irb120_support/', '')
                raw = path.read_bytes()
                count = struct.unpack_from('<I', raw, 80)[0]
                if 84+50*count != len(raw):
                    raise ProposalRejected('Formato STL no admitido por la vista previa')
                dtype = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
                vertices = np.frombuffer(raw, dtype=dtype, count=count, offset=84)['v'].reshape(-1, 3).astype(float)
                scale = np.array([float(v) for v in mesh.get('scale', '1 1 1').split()])
                vertices *= scale
                lo, hi = vertices.min(axis=0), vertices.max(axis=0)
                corners = np.array(list(itertools.product(*zip(lo, hi))))
                self.boxes.append((link.get('name'), origin(visual.find('origin')), corners))

    def frames(self, q_deg):
        q = dict(zip([f'joint_{i}' for i in range(1, 7)], np.radians(numbers(q_deg, 6, 'Angulos'))))
        frames = {'base_link': np.eye(4)}
        pending = list(self.joints)
        while pending:
            ready = [j for j in pending if j.find('parent').get('link') in frames]
            if not ready:
                # world->base_link queda fuera: el marco pedido es base_link.
                break
            for j in ready:
                parent, child = j.find('parent').get('link'), j.find('child').get('link')
                t = frames[parent] @ origin(j.find('origin'))
                if j.get('type') == 'revolute':
                    axis = np.array([float(v) for v in j.find('axis').get('xyz').split()])
                    axis /= np.linalg.norm(axis)
                    x, y, z = axis
                    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                    a = q[j.get('name')]
                    turn = np.eye(4)
                    turn[:3, :3] = np.eye(3)+math.sin(a)*skew+(1-math.cos(a))*(skew@skew)
                    t = t@turn
                frames[child] = t
                pending.remove(j)
        return frames

    def tip(self, q_deg, envelope):
        return self.frames(q_deg)['tool0'] @ transform(envelope.tcp_offset_m)

    def check_height(self, q_deg, envelope):
        frames = self.frames(q_deg)
        tip_z = (frames['tool0'] @ transform(envelope.tcp_offset_m))[2, 3]
        if tip_z < envelope.min_tip_z_m:
            raise ProposalRejected('La punta baja del minimo configurado')
        if envelope.floor_z_m is None:
            return float(tip_z)
        for link, offset, corners in self.boxes:
            pose = frames[link] @ offset
            z = (corners @ pose[:3, :3].T + pose[:3, 3])[:, 2].min()
            if z < envelope.floor_z_m-1e-8:
                raise ProposalRejected(f'El modelo de {link} invade el plano del piso')
        return float(tip_z)


def rotation_vector(matrix):
    angle = math.acos(float(np.clip((np.trace(matrix)-1)/2, -1, 1)))
    skew = np.array([matrix[2, 1]-matrix[1, 2], matrix[0, 2]-matrix[2, 0], matrix[1, 0]-matrix[0, 1]])
    if angle < 1e-7:
        return skew/2
    if math.pi-angle < 1e-5:
        _, vectors = np.linalg.eigh((matrix+matrix.T)/2)
        return angle*vectors[:, -1]
    return angle*skew/(2*math.sin(angle))


def solve_xyz(model, start_deg, xyz_m, envelope):
    """IK numérica manteniendo la orientación actual; rechaza si no converge."""
    target = numbers(xyz_m, 3, 'XYZ')
    initial = numbers(start_deg, 6, 'Angulos')
    goal_rotation = model.tip(initial, envelope)[:3, :3]
    bounds = np.radians(np.array(model_limits()) + np.array([2, -2]))
    q = np.radians(initial)
    def residual(q_rad):
        pose = model.tip(np.degrees(q_rad), envelope)
        return np.r_[target-pose[:3, 3], .2*rotation_vector(goal_rotation@pose[:3, :3].T)]
    for _ in range(180):
        error = residual(q)
        if np.linalg.norm(error[:3]) < 1e-5 and np.linalg.norm(error[3:]) < 2e-5:
            return list(np.degrees(q))
        h = 1e-5
        jac = np.column_stack([(residual(q+np.eye(6)[i]*h)-error)/h for i in range(6)])
        step = -jac.T @ np.linalg.solve(jac@jac.T+1e-5*np.eye(6), error)
        step *= min(1, .05/max(float(np.linalg.norm(step)), 1e-12))
        old = float(np.linalg.norm(error))
        for factor in (1, .5, .2, .05):
            trial = np.clip(q+factor*step, bounds[:, 0], bounds[:, 1])
            if np.linalg.norm(residual(trial)) < old:
                q = trial
                break
        else:
            break
    raise ProposalRejected('No se encontro una postura XYZ cercana conservando la orientacion')


def prepare_target(model, start_deg, *, envelope, target_deg=None, xyz_m=None, motion_config=None):
    envelope.validate()
    if (target_deg is None) == (xyz_m is None):
        raise ProposalRejected('Elegir exactamente un modo: seis angulos o XYZ')
    start = numbers(start_deg, 6, 'Angulos actuales')
    target = numbers(target_deg, 6, 'Angulos destino') if target_deg is not None else np.array(solve_xyz(model, start, xyz_m, envelope))
    policy = load_motion_limits() if motion_config is None else motion_config
    bounds = effective_limits(policy)
    for q in (start, target):
        for value, row in zip(q, bounds):
            if not row['min_deg'] <= value <= row['max_deg']:
                raise ProposalRejected('Angulos fuera del margen de vista previa')
    delta = target-start
    duration = .5
    for change, row in zip(delta, bounds):
        if abs(change) > row['max_command_delta_deg']+1e-10:
            raise ProposalRejected('Recorrido por orden supera el porcentaje configurado')
        if abs(change) > 1e-10:
            if row['speed_deg_s'] == 0:
                raise ProposalRejected('Un eje que debe moverse tiene velocidad configurada en 0%')
            duration = max(duration, 1.875*abs(change)/row['speed_deg_s'],
                           math.sqrt(10/math.sqrt(3)*abs(change)/policy['preview_acceleration_deg_s2']))
    count = max(1, math.ceil(duration/.02))
    if count > 50000:
        raise ProposalRejected('Propuesta demasiado larga')
    min_z = math.inf
    path = []
    for k in range(count+1):
        u = k/count
        q = start+(10*u**3-15*u**4+6*u**5)*delta
        min_z = min(min_z, model.check_height(q, envelope))
        path.append(q.tolist())
    return {'kind': 'preview_only', 'hardware_execution_allowed': False,
            'continuous_collision_checked': False, 'tool_geometry_verified': False,
            'mode': 'joint' if xyz_m is None else 'xyz_hold_orientation',
            'target_deg': target.tolist(), 'duration_s': duration,
            'limits_used': bounds,
            'peak_speed_deg_s': (1.875*np.abs(delta)/duration).tolist(),
            'minimum_tip_height_m_sampled': min_z, 'positions_deg': path,
            'note': 'Filtro muestreado del modelo; falta validar herramienta, entorno y protecciones fisicas.'}
