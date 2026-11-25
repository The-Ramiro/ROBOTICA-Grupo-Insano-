"""Lista de destinos articulares y recorrido estimado de la brida. Sin red."""
import json
from pathlib import Path

import numpy as np
from ejecutar_rws import check_goal

MAX_POINTS = 100
ROBOT_BUFFER = 20


def points(values):
    if not isinstance(values, (list, tuple)) or not 1 <= len(values) <= MAX_POINTS:
        raise ValueError(f'La trayectoria debe tener entre 1 y {MAX_POINTS} puntos.')
    result = []
    for index, value in enumerate(values, 1):
        if (not isinstance(value, (list, tuple)) or len(value) != 6
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not np.isfinite(v) for v in value)):
            raise ValueError(f'P{index}: se requieren seis ángulos finitos en grados.')
        result.append([float(v) for v in value])
    return result


def prepare(model, envelope, config, start, values, real_z_mm=None):
    targets = points(values)
    previous = list(start)
    result = []
    for index, target in enumerate(targets, 1):
        if max(abs(a-b) for a, b in zip(previous, target)) <= .005:
            if index == 1:
                continue  # P1 puede coincidir con la postura de partida.
            raise ValueError(f'P{index}: coincide con el punto anterior.')
        try:
            check_goal(model, envelope, config, previous, target,
                       real_z_mm if not result else None)
        except ValueError as exc:
            raise ValueError(f'Tramo hacia P{index}: {exc}') from exc
        result.append(target)
        previous = target
    if not result:
        raise ValueError('Añade un punto diferente de la postura de partida.')
    return result


def samples(start, targets):
    """Interpolación articular suave para dibujo/animación; ABB planifica el real."""
    poses = [np.asarray(start, dtype=float)]
    for target in targets:
        initial, final = poses[-1], np.asarray(target, dtype=float)
        count = max(16, min(120, int(np.max(np.abs(final-initial))*2)+1))
        for u in np.linspace(0., 1., count)[1:]:
            blend = 10*u**3-15*u**4+6*u**5
            poses.append(initial+(final-initial)*blend)
    return np.asarray(poses)


def save(filename, values):
    payload = dict(format='imt342-ruta-articular-v1', robot='ABB IRB120',
                   tool='tool0', unit='deg', points=points(values))
    Path(filename).write_text(json.dumps(payload, indent=2), encoding='utf-8')


def load(filename):
    path = Path(filename)
    if path.stat().st_size > 100_000:
        raise ValueError('Archivo de trayectoria demasiado grande.')
    obj = json.loads(path.read_text(encoding='utf-8'))
    if (not isinstance(obj, dict) or obj.get('format') != 'imt342-ruta-articular-v1'
            or obj.get('robot') != 'ABB IRB120' or obj.get('tool') != 'tool0'
            or obj.get('unit') != 'deg'):
        raise ValueError('El archivo no es una trayectoria articular compatible.')
    return points(obj.get('points'))
