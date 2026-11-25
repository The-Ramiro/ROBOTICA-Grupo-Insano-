"""Perfil previsto para comparar curvas; no es el planificador interno ABB."""
import math
import numpy as np
from calculos_robotica import lspb
from limites_movimiento import FACTORY
from calibracion_tiempos import load_calibration, duration100, timing_adjustments


def profile_segment(start,target,speed,duration,blend,acceleration):
    if not math.isfinite(speed) or not 0 < speed <= 100:
        raise ValueError('Selecciona una velocidad mayor que 0% para previsualizar el recorrido.')
    scale=speed/100.
    model=load_calibration()
    factor, _, _ = timing_adjustments(speed, model)
    scale /= factor
    if model is not None:
        duration=duration100(start,target,model)
        blend=model['blend']
        delta=float(np.max(np.abs(np.radians(target)-np.radians(start))))
        acceleration=max(acceleration,math.degrees(delta/(duration**2*blend*(1-blend)))*1.001)
    path=lspb(np.radians(start),np.radians(target),duration,blend,
              np.radians([r[2] for r in FACTORY]),math.radians(acceleration),count=65)
    # El override reduce el ritmo completo, también en las rampas y tramos cortos.
    path['t']=path['t']/scale
    path['duration']/=scale
    path['blend_time']/=scale
    path['qd']=path['qd']*scale
    path['qdd']=path['qdd']*scale**2
    return path


def make_reference(start,targets,speed=10.,duration=1.,blend=.2,acceleration=30.):
    if start is None or not targets:return None
    if not math.isfinite(speed) or not 0 < speed <= 100:
        raise ValueError('Selecciona una velocidad mayor que 0% para previsualizar el recorrido.')
    previous=np.asarray(start,dtype=float)
    chunks={k:[] for k in ('t','q','qd','qdd')};marks=[];phases=[];offset=0.
    model=load_calibration()
    moving=any(np.max(np.abs(np.asarray(target)-previous))>.005 for target in targets)
    _, lead, tail = timing_adjustments(speed, model if moving else None)
    if model is not None and moving:
        offset=lead
        chunks=dict(t=[0.,offset],q=[previous.tolist(),previous.tolist()],qd=[[0.]*6,[0.]*6],qdd=[[0.]*6,[0.]*6])
    for point,target in enumerate(targets,1):
        if np.max(np.abs(previous-target))<=.005:
            marks.append((offset,point));previous=np.asarray(target,dtype=float);continue
        path=profile_segment(previous,target,speed,duration,blend,acceleration)
        cut=0 if not chunks['t'] else 1
        chunks['t'].extend((path['t'][cut:]+offset).tolist())
        for key in ('q','qd','qdd'):chunks[key].extend(np.degrees(path[key][cut:]).tolist())
        phases.extend(((offset,offset+path['blend_time'],'accel'),
                       (offset+path['duration']-path['blend_time'],offset+path['duration'],'decel')))
        offset+=path['duration'];marks.append((offset,point));previous=np.asarray(target,dtype=float)
    if not chunks['t']:
        chunks=dict(t=[0.],q=[previous.tolist()],qd=[[0.]*6],qdd=[[0.]*6])
    if tail > 0:
        chunks['t'].append(offset+tail);chunks['q'].append(previous.tolist())
        chunks['qd'].append([0.]*6);chunks['qdd'].append([0.]*6)
    metadata=dict(calibrated=True,calibration_sessions=len(model['sessions']),
                  calibrated_speed_range=model['speed_range'],startup_s=lead,completion_s=tail,
                  timing_by_speed=model.get('timing_by_speed', [])) if model else {}
    return dict(**chunks,marks=marks,phases=phases,source='LSPB · tiempo calibrado' if model else 'LSPB estimado',speed=speed,**metadata)


def rescale_reference(reference,speed):
    if not math.isfinite(speed) or not 0 < speed <= 100:
        raise ValueError('La velocidad debe ser mayor que 0% y hasta 100%.')
    if reference['t'][-1] == 0:
        return dict(reference, speed=speed)
    old_speed = reference['speed']
    timing = dict(reference, startup_s=reference.get('startup_s', 0.))
    old_factor, old_lead, old_tail = timing_adjustments(old_speed, timing)
    factor, lead, tail = timing_adjustments(speed, timing)
    scale = old_speed/speed * factor/old_factor
    duration = (reference['t'][-1] - old_lead - old_tail)*scale + lead + tail
    updated = dict(reference, speed=speed, startup_s=lead, completion_s=tail, t=[0., duration])
    def convert(t):
        return float(retime_instant(reference, updated, t))
    return dict(updated, t=retime_instant(reference, updated, reference['t']).tolist(),
                qd=(np.asarray(reference['qd'])/scale).tolist(),
                qdd=(np.asarray(reference['qdd'])/scale**2).tolist(),
                marks=[(convert(t), label) for t, label in reference['marks']],
                phases=[(convert(a), convert(b), label) for a,b,label in reference['phases']])


def retime_instant(previous, updated, elapsed):
    """Conserva la fase de la animación al cambiar el porcentaje."""
    def boundaries(ref):
        total = ref['t'][-1]
        return [0., ref.get('startup_s', 0.), total-ref.get('completion_s', 0.), total]
    old, new = boundaries(previous), boundaries(updated)
    keep = np.r_[True, np.diff(old) > 0.]
    return np.interp(elapsed, np.asarray(old)[keep], np.asarray(new)[keep])


def timing_caption(reference):
    if not reference or not reference.get('calibrated'):
        return ''
    speed = reference['speed']
    span = reference.get('calibrated_speed_range')
    if span is None:
        return 'tiempo calibrado'
    if not span[0] <= speed <= span[1]:
        return 'tiempo extrapolado'
    knots = reference.get('timing_by_speed', [])
    if knots and not any(math.isclose(speed, row[0], abs_tol=1e-6) for row in knots):
        return 'tiempo interpolado'
    return 'tiempo calibrado'


def observed_start(reference, rows):
    """Acota el arranque con una lectura en reposo y otra en movimiento."""
    if not reference or not reference.get('startup_s'):
        return None
    times = reference['t']
    if len(times) < 3 or times[0] != 0. or not math.isclose(times[1], reference['startup_s']):
        return None
    last_rest = None
    initial = np.asarray(reference['q'][0])
    for row in rows:
        delta = np.max(np.abs(row['q']-initial))
        if delta <= .05:
            last_rest = row['t']
        elif delta > .3:
            if last_rest is not None and 0. < row['t']-last_rest <= .5:
                return (last_rest+row['t'])/2.
            return None
    return None


def inferred_start(reference, rows):
    """Estima el arranque si la telemetría se cortó durante el primer tramo."""
    if not reference or reference['speed'] < 60 or not reference.get('startup_s'):
        return None
    lead = reference['startup_s']
    end = next((instant for instant, _ in reference['marks'] if instant > lead), None)
    if end is None:
        return None
    times = np.asarray(reference['t'])
    segment = (times >= lead) & (times <= end + 1e-9)
    elapsed = times[segment] - lead
    q = np.asarray(reference['q'])[segment]
    if len(q) < 3 or not math.isclose(elapsed[0], 0., abs_tol=1e-9):
        return None
    direction = q[-1] - q[0]
    length_squared = direction @ direction
    if length_squared < 1.:
        return None
    progress = (q-q[0]) @ direction / length_squared
    last_rest = None
    first_moving = None
    estimates = []
    for row in rows:
        delta = np.asarray(row['q']) - q[0]
        distance = np.max(np.abs(delta))
        if first_moving is None:
            if distance <= .05:
                last_rest = row['t']
            if distance > .3:
                first_moving = row['t']
        fraction = delta @ direction / length_squared
        if fraction > .30:
            break
        if .04 <= fraction and np.max(np.abs(delta-fraction*direction)) <= .5:
            estimates.append(row['t'] - np.interp(fraction, progress, elapsed))
    if (last_rest is None or first_moving is None or first_moving-last_rest <= .5
            or len(estimates) < 3 or np.ptp(estimates) > .15):
        return None
    onset = float(np.median(estimates))
    return onset if last_rest < onset < first_moving else None


def align_start(reference, onset):
    """Cambia solo la espera inicial de una copia para Seguimiento."""
    if onset is None:
        return reference
    shift = onset-reference['startup_s']
    return dict(reference, startup_s=onset, t=[0., *(t+shift for t in reference['t'][1:])],
                marks=[(t+shift, point) for t, point in reference.get('marks', [])],
                phases=[(a+shift, b+shift, phase) for a, b, phase in reference.get('phases', [])])


def profile_settings(panel,confirmed=False):
    fields=getattr(panel,'plan_fields',None)
    duration,blend,percent,acceleration=[f.value() for f in fields] if fields else (1.,20.,10.,30.)
    actual=getattr(panel,'speed_actual',None) or {}
    widget=getattr(panel,'real_speed_value',None)
    speed=actual.get('ratio',percent) if confirmed else (widget.value() if widget is not None else percent)
    return speed,duration,blend/100.,acceleration


def begin_recording(panel,start,targets):
    recorder=getattr(panel,'recorder',None)
    if recorder is None:return
    speed,duration,blend,acceleration=profile_settings(panel,confirmed=True)
    try:reference=make_reference(start,targets,speed,duration,blend,acceleration)
    except ValueError:reference=None
    if hasattr(panel,'math_window'):
        panel.math_window.preview_profile=None
        panel.math_window.preview_requested=False
        panel.math_window.preview_due=None
        panel.math_window.report_requested=False
    recorder.begin_execution(None,reference=reference)


def comparison_curves(reference,rows,variable='q'):
    rt=np.asarray([r['t'] for r in rows],dtype=float)
    measured=np.asarray([r['q'] for r in rows],dtype=float).reshape(-1,6)
    if variable in ('qd','qdd'):
        measured=np.gradient(measured,rt,axis=0) if len(rt)>1 else np.full_like(measured,np.nan)
    if variable=='qdd':
        measured=np.gradient(measured,rt,axis=0) if len(rt)>2 else np.full_like(measured,np.nan)
    for i in np.flatnonzero(np.diff(rt)>.5)+1:measured[max(0,i-1):i+1]=np.nan
    pt=np.asarray(reference['t'],dtype=float) if reference else np.array([],dtype=float)
    t=np.unique(np.r_[0.,pt,rt])
    expected=np.full((len(t),6),np.nan);actual=np.full_like(expected,np.nan)
    for j in range(6):
        if reference:expected[:,j]=np.interp(t,pt,np.asarray(reference[variable])[:,j])
        if len(rt):actual[:,j]=np.interp(t,rt,measured[:,j],left=np.nan,right=np.nan)
    return dict(t=t,expected=expected,actual=actual,marks=reference.get('marks',[]) if reference else [])
