"""Dinámica 6R del URDF: estimación, nunca lectura de par del controlador."""
from concurrent.futures import CancelledError
import numpy as np


def measured_derivatives(rows, window=.5):
    if len(rows) < 5:
        raise ValueError('Esperando al menos cinco lecturas para estimar velocidad y aceleración.')
    stamps=np.asarray([r['t'] for r in rows])
    if np.any(np.diff(stamps)<=0):raise ValueError('Tiempos de lectura no crecientes.')
    cadence=float(np.median(np.diff(stamps)[-10:]))
    gap_limit=min(.5,max(.25,2*cadence))
    gaps=np.flatnonzero(np.diff(stamps)>gap_limit)
    if len(gaps):rows=rows[int(gaps[-1])+1:]
    if len(rows)<5:raise ValueError('Esperando cinco lecturas continuas después del hueco de comunicación.')
    end = rows[-1]['t']
    window=max(window,min(1.5,5*cadence))
    rows = [r for r in rows if end-r['t'] <= window+1e-9]
    t = np.asarray([r['t'] for r in rows])
    if len(t) < 5 or t[-1]-t[0] < .15 or np.any(np.diff(t) <= 0) or np.any(np.diff(t) > gap_limit):
        raise ValueError('Lecturas insuficientes o discontinuas para estimar la dinámica.')
    x = t-t[-1]
    design = np.column_stack((np.ones(len(x)), x, .5*x*x))
    if np.linalg.cond(design) > 1e6:
        raise ValueError('Ventana de lectura mal condicionada para calcular derivadas.')
    # Ajuste causal cuadrático: reduce ruido sin utilizar lecturas futuras.
    fit = np.linalg.lstsq(design, np.radians([r['q'] for r in rows]), rcond=None)[0]
    return fit[1], fit[2]


def reconstruct_dynamics(dynamic,rows,cancelled=None,progress=None):
    indices=set(np.linspace(4,len(rows)-1,min(120,max(0,len(rows)-4)),dtype=int))
    for i in range(4,len(rows)):
        if rows[i].get('point')!=rows[i-1].get('point'):indices.add(i)
    frames=[]
    for n,i in enumerate(sorted(indices)):
        if cancelled is not None and cancelled.is_set():raise CancelledError()
        try:frame=measured_dynamics(dynamic,rows[max(0,i-29):i+1])
        except ValueError:continue
        frames.append(frame)
        if progress is not None and n%10==0:progress(f'Reconstruyendo dinámica: {n+1}/{len(indices)}')
    return frames


def measured_dynamics(dynamic, rows):
    qd, qdd = measured_derivatives(rows)
    row = rows[-1]
    return dict(t=row['t'], run=row.get('run', 0), point=row.get('point'),
                moving=row.get('moving', False), qd=qd, qdd=qdd,
                **dynamic.evaluate(row['q'], qd, qdd))


def planned_dynamics(dynamic, data, cancelled=None, progress=None):
    times = data['t']
    indices = set(np.linspace(0, len(times)-1, min(140,len(times)), dtype=int))
    for stamp, _ in data['point_times']:
        indices.add(int(np.argmin(abs(times-stamp))))
    for begin, end, _ in data['phases']:
        for stamp in (begin,end):
            i = int(np.argmin(abs(times-stamp)))
            indices.update((max(0,i-1),i,min(len(times)-1,i+1)))
    indices = sorted(indices)
    result = {key:[] for key in ('inertia','g','c','tau')}
    for n,i in enumerate(indices):
        if cancelled is not None and cancelled.is_set():
            raise CancelledError()
        frame = dynamic.evaluate(np.degrees(data['q'][i]), data['qd'][i], data['qdd'][i])
        for key in result:
            result[key].append(frame[key])
        if progress is not None and n % 10 == 0:
            progress(f'Dinámica 6R: {n+1}/{len(indices)} muestras')
    return dict(t=times[indices], **{key:np.asarray(v) for key,v in result.items()})
