"""Ramas geométricas para el URDF del ABB IRB 120 con muñeca X-Y-X."""
from itertools import product
import math

import numpy as np

from objetivos_preview import origin, rotation_vector, transform
from preparar_paso import ProposalRejected


def equivalent_angles(angle, limits):
    lower, upper = limits
    start = math.ceil((lower - angle - 1e-8)/360)
    end = math.floor((upper - angle + 1e-8)/360)
    return [min(upper, max(lower, angle + 360*turn)) for turn in range(start, end + 1)]


def wrist_angles(rotation, bounds, reference):
    bend = math.acos(float(np.clip(rotation[0, 0], -1., 1.)))
    if bend < 1e-7:
        # J4 + J6 queda determinado. Se representan la postura próxima y los extremos.
        total = math.degrees(math.atan2(rotation[2, 1], rotation[1, 1]))
        (j4_min, j4_max), _, (j6_min, j6_max) = bounds
        for angle in equivalent_angles(total, (j4_min + j6_min, j4_max + j6_max)):
            low, high = max(j4_min, angle - j6_max), min(j4_max, angle - j6_min)
            nearest = (reference[3] + angle - reference[5])/2
            for roll in (float(np.clip(nearest, low, high)), low, high):
                yield roll, 0., angle - roll
    elif math.pi - bend > 1e-7:
        # La otra singularidad X-Y-X, J5 = ±180°, queda fuera del IRB 120.
        roll = math.atan2(rotation[1, 0], -rotation[2, 0])
        end = math.atan2(rotation[0, 1], rotation[0, 2])
        yield from (tuple(map(math.degrees, wrist)) for wrist in (
            (roll, bend, end), (roll + math.pi, -bend, end + math.pi)))


def branches(model, envelope, xyz, rotation, reference, bounds):
    """Devuelve ángulos verificados con FK y si se muestreó una familia singular."""
    joint = {j.get('name'): origin(j.find('origin')) for j in model.joints}
    flange_tool = joint['link_6-tool0']
    flange_rotation = rotation @ flange_tool[:3, :3].T
    center = (np.asarray(xyz) - rotation @ np.asarray(envelope.tcp_offset_m)
              - flange_rotation @ (flange_tool[:3, 3] + joint['joint_6'][:3, 3]))
    radius = math.hypot(*center[:2])
    z = center[2] - joint['joint_2'][2, 3]
    upper_arm = joint['joint_3'][2, 3]
    forearm_x, forearm_z = joint['joint_5'][0, 3], joint['joint_4'][2, 3]
    forearm = math.hypot(forearm_x, forearm_z)
    offset = math.atan2(forearm_x, forearm_z)
    elbow_cos = (radius**2 + z**2 - upper_arm**2 - forearm**2)/(2*upper_arm*forearm)
    if abs(elbow_cos) > 1. + 1e-9:
        return [], False
    elbow = math.acos(float(np.clip(elbow_cos, -1., 1.)))
    singular = radius < 1e-7
    if singular:
        lower, higher = bounds[0]
        shoulders = [(math.radians(a), 0.) for a in np.linspace(lower, higher, 9)]
        shoulders.append((math.radians(reference[0]), 0.))
    else:
        heading = math.atan2(center[1], center[0])
        shoulders = ((heading, radius), (heading + math.pi, -radius))
    candidates = []
    for (j1, radial), bend in product(shoulders, (elbow, -elbow)):
        j2 = math.atan2(radial, z) - math.atan2(forearm*math.sin(bend), upper_arm + forearm*math.cos(bend))
        j3 = bend - offset
        first = tuple(equivalent_angles(math.degrees(q), limits)
                      for q, limits in zip((j1, j2, j3), bounds))
        if not all(first):
            continue
        arm = transform(rpy=(0., 0., j1))[:3, :3] @ transform(rpy=(0., j2 + j3, 0.))[:3, :3]
        wrist = arm.T @ flange_rotation
        singular |= abs(wrist[0, 0] - 1.) < 1e-14
        for j4, j5, j6 in wrist_angles(wrist, bounds[3:], reference):
            for angles in product(*first, *(equivalent_angles(q, limits)
                                            for q, limits in zip((j4, j5, j6), bounds[3:]))):
                q = tuple(round(angle, 3) for angle in angles)
                if any(not low <= v <= high for v, (low, high) in zip(q, bounds)):
                    continue
                if any(max(abs(a-b) for a, b in zip(q, old)) < .005 for old in candidates):
                    continue
                pose = model.tip(q, envelope)
                if (np.linalg.norm(pose[:3, 3] - xyz) > 0.00002
                        or np.linalg.norm(rotation_vector(pose[:3, :3] @ rotation.T)) > math.radians(.01)):
                    continue
                try:
                    model.check_height(q, envelope)
                except ProposalRejected:
                    continue
                candidates.append(q)
    candidates.sort(key=lambda q: float(np.linalg.norm(np.asarray(q) - reference)))
    return candidates, singular
