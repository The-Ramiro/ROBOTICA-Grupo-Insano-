"""Informe muestreado de toda una ruta. LSPB estimado, no perfil interno ABB."""
from concurrent.futures import CancelledError
import math
import numpy as np
from calculos_robotica import geometric_jacobian, Dynamics
from referencia_recorrido import profile_segment
from calibracion_tiempos import load_calibration, timing_adjustments
from objetivos_preview import transform

GEOMETRY = (('wrist', 'Muñeca', 5., .1, '°'),
            ('elbow', 'Codo', 5., .1, '°'),
            ('shoulder', 'Hombro', 10., .1, 'mm'))


def mobility_state(margin, rank, threshold=1.):
    """Margen = 100 sigma_min/sigma_max, no probabilidad ni % de velocidad."""
    if rank < 6:
        return f'Singular (criterio numérico) · {rank}/6 direcciones independientes'
    if margin <= threshold:
        return 'Cerca de singularidad · 6/6, con una dirección difícil de producir'
    return 'Sin cercanía detectada con el umbral elegido · 6/6'


def geometry_metrics(frames):
    """Geometría del IRB120: desviación de alineación y radio de muñeca."""
    def angle(a, b):
        cosine = abs(float(np.dot(a, b)))/(np.linalg.norm(a)*np.linalg.norm(b))
        return math.degrees(math.acos(float(np.clip(cosine, 0., 1.))))
    wrist = frames['link_5'][:3, 3]
    upper = frames['link_3'][:3, 3]-frames['link_2'][:3, 3]
    forearm = wrist-frames['link_3'][:3, 3]
    offset = wrist-frames['link_1'][:3, 3]
    axis = frames['link_1'][:3, 2]
    radial = offset-axis*np.dot(offset, axis)
    return dict(wrist=angle(frames['link_4'][:3, 0], frames['link_6'][:3, 0]),
                elbow=angle(upper, forearm), shoulder=float(np.linalg.norm(radial)*1000))


def singular_metrics(model, envelope, q):
    frames = model.frames(q)
    tip = (frames['tool0'] @ transform(envelope.tcp_offset_m))[:3, 3]
    jac = geometric_jacobian(model, q, tip, frames=frames)
    return matrix_metrics(jac)


def matrix_metrics(jac):
    jac = jac.copy()
    jac[3:] *= .2
    s = np.linalg.svd(jac, compute_uv=False)
    return float(s[-1]), float(s[0]/max(s[-1], 1e-12)), float(np.prod(s)), int(np.sum(s > max(1e-7, s[0]*1e-7)))


def report(model, envelope, start, targets, speed=10., duration=1., blend=.2,
           acceleration=30., threshold=.001, cancelled=None, progress=None, dynamics=True):
    if not targets:
        raise ValueError('Añade al menos un punto.')
    if not 0 < speed <= 100 or not 0 < threshold <= 1:
        raise ValueError('Porcentaje o umbral de análisis inválido.')
    def check():
        if cancelled is not None and cancelled.is_set():
            raise CancelledError()
    previous = list(targets[0] if start is None else start)
    initial_known = start is not None
    chunks, phases, rows, point_times = [], [], [], []
    offset = 0.
    calibrated=load_calibration() is not None and any(np.max(np.abs(np.asarray(target)-previous))>.005 for target in targets)
    _, lead, tail = timing_adjustments(speed, load_calibration() if calibrated else None)
    if calibrated:
        offset=lead
        chunks.append(dict(t=np.array([0.,offset]),q=np.radians([previous,previous]),
                           qd=np.zeros((2,6)),qdd=np.zeros((2,6)),point=np.zeros(2)))
    for index, target in enumerate(targets, 1):
        check()
        if np.max(np.abs(np.asarray(target)-previous)) <= .005:
            point_times.append((offset, index))
            rows.append(dict(point=index, start=offset, end=offset, duration=0.,
                             sigma=singular_metrics(model, envelope, target)[0], skipped=True,
                             geometry=geometry_metrics(model.frames(target))))
            previous = list(target)
            continue
        path = profile_segment(previous,target,speed,duration,blend,acceleration)
        times = path['t']+offset
        chunks.append(dict(t=times, q=path['q'], qd=path['qd'], qdd=path['qdd'],
                           point=np.full(len(times), index)))
        phases.extend([(offset, offset+path['blend_time'], 'accel'),
                       (offset+path['duration']-path['blend_time'], offset+path['duration'], 'decel')])
        rows.append(dict(point=index, start=offset, end=float(times[-1]), duration=path['duration'],
                         max_v=float(np.max(np.abs(path['qd']))),
                         max_a=float(np.max(np.abs(path['qdd']))), skipped=False))
        offset = float(times[-1])
        point_times.append((offset, index))
        previous = list(target)
    if tail > 0:
        chunks.append(dict(t=np.array([offset,offset+tail]),q=np.radians([previous,previous]),
                           qd=np.zeros((2,6)),qdd=np.zeros((2,6)),point=np.full(2,len(targets))))
        offset+=tail
    if chunks:
        data = {k: np.concatenate([c[k][(0 if i == 0 else 1):] for i, c in enumerate(chunks)])
                for k in ('t', 'q', 'qd', 'qdd', 'point')}
    else:
        data = dict(t=np.array([0.]), q=np.radians([previous]), qd=np.zeros((1, 6)),
                    qdd=np.zeros((1, 6)), point=np.array([len(targets)]))
    xyz, sigma, condition, manip, ranks, twists, links = [], [], [], [], [], [], []
    tool_offset = transform(envelope.tcp_offset_m)
    geometries = {key: [] for key, *_ in GEOMETRY}
    for index, q in enumerate(np.degrees(data['q'])):
        check()
        frames = model.frames(q)
        metrics = geometry_metrics(frames)
        for key in geometries:
            geometries[key].append(metrics[key])
        point = (frames['tool0'] @ tool_offset)[:3, 3]
        xyz.append(point)
        links.append([frames[f'link_{i}'][:3, 3] for i in range(1, 7)])
        jac = geometric_jacobian(model, q, point, frames=frames)
        s, c, m, rank = matrix_metrics(jac)
        twists.append(jac @ data['qd'][index])
        sigma.append(s); condition.append(c); manip.append(m); ranks.append(rank)
        if progress is not None and index % 80 == 0:
            progress(f'Jacobiano y recorrido: {index+1}/{len(data["t"])} muestras')
    xyz = np.asarray(xyz)
    links = np.asarray(links)
    link_v = np.gradient(links, data['t'], axis=0) if len(links) > 1 else np.zeros_like(links)
    link_a = np.gradient(link_v, data['t'], axis=0) if len(links) > 2 else np.zeros_like(links)
    sigma = np.asarray(sigma)
    mobility = 100./np.asarray(condition)
    geometries = {key: np.asarray(values) for key, values in geometries.items()}
    for row in rows:
        selection = (data['t'] >= row['start']-1e-9) & (data['t'] <= row['end']+1e-9)
        row['warnings'] = []
        for key, name, close, aligned, unit in GEOMETRY:
            if row['skipped']:
                minimum, instant, first = row['geometry'][key], row['start'], row['start']
            else:
                indices = np.flatnonzero(selection)
                minimum_index = indices[int(np.argmin(geometries[key][indices]))]
                minimum = float(geometries[key][minimum_index])
                instant = float(data['t'][minimum_index])
                near_indices = indices[geometries[key][indices] <= close]
                first = float(data['t'][near_indices[0]]) if len(near_indices) else None
            if minimum <= close:
                row['warnings'].append(dict(name=name, value=minimum, unit=unit, time=instant,
                                             first=first, state='Alineado' if minimum <= aligned else 'Cerca'))
        if np.any(selection):
            row['sigma'] = float(sigma[selection].min())
            row['near'] = row['sigma'] < threshold
            indices = np.flatnonzero(selection)
            critical = indices[np.argmin(mobility[indices])]
            row.update(margin=float(mobility[critical]), rank=int(ranks[critical]),
                       critical_time=float(data['t'][critical]))
        else:
            row['near'] = row['sigma'] < threshold
            s, c, _, rank = singular_metrics(model, envelope, targets[row['point']-1])
            row.update(margin=100./c, rank=rank, critical_time=row['start'])
    # Dinámica en una malla acotada: evita miles de diferencias de M(q).
    indices = np.unique(np.linspace(0, len(xyz)-1, min(25, len(xyz)), dtype=int))
    tau, energy = [], []
    if dynamics:
        dynamic = Dynamics(model)
        for n, index in enumerate(indices):
            check()
            if progress is not None:
                progress(f'Dinámica estimada: {n+1}/{len(indices)} muestras')
            frame = dynamic.evaluate(np.degrees(data['q'][index]), data['qd'][index], data['qdd'][index])
            tau.append(frame['tau'])
            energy.append([frame['kinetic'], frame['potential'], frame['tau'] @ data['qd'][index]])
    return dict(**data, xyz=xyz, twist=np.asarray(twists), links=links, link_v=link_v, link_a=link_a,
                sigma=sigma, condition=np.asarray(condition), mobility=mobility,
                manipulability=np.asarray(manip), rank=np.asarray(ranks), rows=rows,
                phases=phases, point_times=point_times, threshold=threshold,
                initial_known=initial_known, speed=speed, dynamic_t=data['t'][indices],
                tau=np.asarray(tau), energy=np.asarray(energy), duration=offset, geometry=geometries)
