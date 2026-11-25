"""Comparación geométrica y registro de errores del ABB."""
import numpy as np

from calculos_robotica import geometric_jacobian
from objetivos_preview import rotation_vector, transform


def joint_rates(previous, current, *, max_gap=.5):
    """Deriva grados por segundo usando dos lecturas consecutivas."""
    if previous is None:
        return None
    elapsed = current['t'] - previous['t']
    if not 1e-6 < elapsed <= max_gap:
        return None
    return (np.asarray(current['q']) - previous['q']) / elapsed


def profile_state(reference, instant, *, times=None, angles=None, rates=None):
    """Devuelve posición y velocidad articular previstas al tiempo de la lectura ABB."""
    times = np.asarray(reference['t']) if times is None else times
    angles = np.asarray(reference['q']).T if angles is None else angles
    rates = np.asarray(reference['qd']).T if rates is None else rates
    return ([np.interp(instant, times, axis) for axis in angles],
            [np.interp(instant, times, axis, left=0., right=0.) for axis in rates])


def spatial_errors(model, envelope, white_q, abb_q, white_rate=None, abb_rate=None):
    """Δ = blanco − ABB. Todas las magnitudes se expresan en la base del robot."""
    offset = transform(envelope.tcp_offset_m)

    def evaluate(angles, rates):
        frames = model.frames(angles)
        tip = frames['tool0'] @ offset
        jacobian = geometric_jacobian(model, angles, tip[:3, 3], frames=frames)
        twist = np.full(6, np.nan) if rates is None else jacobian @ np.radians(rates)
        return tip, jacobian, np.r_[twist[:3]*1000., np.degrees(twist[3:])]

    white_tip, white_j, white_twist = evaluate(white_q, white_rate)
    abb_tip, abb_j, abb_twist = evaluate(abb_q, abb_rate)
    xyz = np.stack((white_tip[:3, 3], abb_tip[:3, 3])) * 1000.
    relative_rotation = white_tip[:3, :3] @ abb_tip[:3, :3].T
    orientation = np.degrees(rotation_vector(relative_rotation))
    return dict(position=np.vstack((xyz, xyz[0]-xyz[1])), orientation=orientation,
                rotation=relative_rotation, jacobians=np.stack((white_j, abb_j, white_j-abb_j)),
                velocity=np.stack((white_twist, abb_twist, white_twist-abb_twist)))


def error_series(comparison):
    """Componentes de cada gráfica y magnitud sin mezclar unidades."""
    def vector(values):
        return np.r_[values, np.linalg.norm(values)]

    jacobian, twist = comparison['jacobians'][2], comparison['velocity'][2]
    return (vector(comparison['position'][2]), vector(comparison['orientation']),
            [np.linalg.norm(jacobian[:3])], [np.linalg.norm(jacobian[3:])],
            vector(twist[:3]), vector(twist[3:]))


def summarize_errors(magnitudes, thresholds):
    """Picos y cantidad de lecturas sobre cada umbral, excluyendo datos ausentes."""
    summaries = []
    for values, threshold in zip(np.asarray(magnitudes).T, thresholds):
        valid = np.isfinite(values)
        if not valid.any():
            summaries.append(None)
            continue
        peak = int(np.argmax(np.where(valid, values, -np.inf)))
        summaries.append((peak, int(np.count_nonzero(valid & (values > threshold)))))
    return summaries


def tracking_arrays(rows):
    if not rows:
        raise ValueError('Todavía no hay lecturas registradas del ABB.')
    t=np.array([r['t'] for r in rows]);t-=t[0]
    def values(key,size):
        return np.array([r[key] if r.get(key) is not None else [np.nan]*size for r in rows],dtype=float)
    q,goal=values('q',6),values('goal_q',6)
    tcp,xyz=values('tcp_xyz',3),values('goal_xyz',3)
    angular=goal-q
    cartesian=xyz-tcp
    consistency=np.array([r.get('fk_tcp_error_mm') if r.get('fk_tcp_error_mm') is not None else np.nan for r in rows])
    for i in np.flatnonzero(np.diff(t) > .5)+1:
        for data in (q,goal,tcp,xyz,angular,cartesian,consistency):data[i]=np.nan
    marks=[];previous=None
    for stamp,row in zip(t,rows):
        if row.get('moving') and row.get('point') is not None and row['point'] != previous:
            marks.append((stamp,row['point']))
        previous=row.get('point')
    return dict(t=t,q=q,goal=goal,tcp=tcp,xyz=xyz,angular=angular,cartesian=cartesian,
                distance=np.linalg.norm(cartesian,axis=1),consistency=consistency,marks=marks)
