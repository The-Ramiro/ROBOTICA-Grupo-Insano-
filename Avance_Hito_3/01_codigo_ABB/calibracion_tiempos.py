"""Calibración empírica del TIEMPO de la vista previa; no modifica el ABB."""
import json
import math
from pathlib import Path
import numpy as np
from limites_movimiento import FACTORY

MODEL_PATH=Path(__file__).with_name('calibracion_tiempos.json')
_cached_key=None
_cached=None

def load_calibration():
    global _cached_key,_cached
    try:
        key=(str(MODEL_PATH),MODEL_PATH.stat().st_mtime_ns)
        if key!=_cached_key:
            data=json.loads(MODEL_PATH.read_text(encoding='utf-8'))
            values=[*data['coefficients'],data['startup_s'],data['blend']]
            if data['schema']!=1 or not np.isfinite(values).all() or min(values)<0 or not 0<data['blend']<=.5:
                raise ValueError('Calibración temporal inválida.')
            knots = data.get('timing_by_speed', [])
            if knots:
                table = np.asarray(knots, dtype=float)
                if (table.ndim != 2 or table.shape[1] != 4 or not np.isfinite(table).all()
                        or not 0 < table[0, 0] <= table[-1, 0] <= 100
                        or np.any(np.diff(table[:, 0]) <= 0) or np.any(table[:, 1:] <= 0)):
                    raise ValueError('Ajuste por porcentaje inválido.')
            _cached,_cached_key=data,key
        return _cached
    except (OSError,ValueError,KeyError,TypeError):return None

def features(start,target):
    delta=np.abs(np.asarray(target,dtype=float)-np.asarray(start,dtype=float))
    travel=float(np.max(delta))
    return np.array([1-math.exp(-travel/5.),float(np.max(delta/np.array([a[2] for a in FACTORY])))])

def duration100(start,target,model=None):
    model=load_calibration() if model is None else model
    if model is None:return None
    return max(.001,float(features(start,target)@np.asarray(model['coefficients'])))

def timing_adjustments(speed, model):
    """Factor de movimiento, espera inicial y confirmación final."""
    if model is None:
        return 1., 0., 0.
    knots = model.get('timing_by_speed', [])
    if not knots:
        return 1., float(model['startup_s']), float(model.get('completion_s', 0.))
    percentages, factors, starts, finishes = np.asarray(knots, dtype=float).T
    return tuple(float(np.interp(speed, percentages, values)) for values in (factors, starts, finishes))

def observations(record, *, arrival_tolerance_deg=.05):
    """Solo recorridos completos, metas alcanzadas en orden y señales medidas."""
    d=record['data'];ref=d['reference'];rows=d['samples']
    if d.get('state')!='completed' or len(rows)<10:raise ValueError('Registro incompleto.')
    speed=float(ref['speed'])
    if not 0<speed<=100:raise ValueError('Porcentaje no válido.')
    t=np.array([r['t'] for r in rows]);q=np.array([r['q'] for r in rows])
    if np.any(np.diff(t)<=0):raise ValueError('Tiempos no crecientes.')
    previous=np.array(ref['q'][0],dtype=float);cursor=0;previous_end=None;result=[];startup=None
    for expected,point in ref['marks']:
        target=np.array([np.interp(expected,ref['t'],np.asarray(ref['q'])[:,j]) for j in range(6)])
        delta=target-previous
        if np.max(np.abs(delta))<.15:
            previous=target;continue
        available=np.arange(cursor,len(t))
        hit=available[np.max(abs(q[available]-target),axis=1)<=arrival_tolerance_deg]
        if not len(hit):raise ValueError('Meta no confirmada por las muestras.')
        end=int(hit[0])
        movement=np.arange(cursor,end+1)
        movement=movement[np.max(abs(q[movement]-previous),axis=1)>.3]
        if not len(movement):raise ValueError('Tramo sin movimiento observado.')
        first=int(movement[0]);motion_start=(t[max(cursor,first-1)]+t[first])/2.
        if startup is None:startup=float(motion_start)
        begin=motion_start if previous_end is None else previous_end
        seconds=float(t[end]-begin)
        if seconds<=.15:raise ValueError('Tramo sin resolución temporal suficiente.')
        result.append(dict(session=d['session_id'],point=int(point),speed=speed,
                           start=previous.tolist(),target=target.tolist(),seconds=seconds,
                           normalized_s=seconds*speed/100.,begin_s=float(begin),end_s=float(t[end]),
                           x=features(previous,target).tolist()))
        previous_end=float(t[end]);cursor=end;previous=target
    if not result:raise ValueError('Sin tramos utilizables.')
    return result,float(startup),float(t[-1])

def coefficients(rows):
    x=np.array([r['x'] for r in rows]);y=np.array([r['normalized_s'] for r in rows])
    choices=[np.maximum(0.,np.linalg.lstsq(x,y,rcond=None)[0])]
    for j in range(2):
        c=np.zeros(2);c[j]=max(0.,float(x[:,j]@y/(x[:,j]@x[:,j])));choices.append(c)
    return min(choices,key=lambda c:float(np.mean((x@c-y)**2)))

def fit_timing_by_speed(rows, sessions, coef):
    timing = []
    for speed in sorted({session['speed'] for session in sessions}):
        matched = [session for session in sessions if session['speed'] == speed]
        segments = [row for row in rows if row['speed'] == speed]
        measured = sum(row['normalized_s'] for row in segments)
        nominal = sum(np.asarray(row['x']) @ coef for row in segments)
        timing.append([speed, measured/float(nominal),
                       float(np.median([s['startup_s'] for s in matched])),
                       float(np.median([s['completion_s'] for s in matched]))])
    return timing

def fit_sessions(records):
    rows=[];starts=[];sessions=[]
    for record in records:
        try:part,start,total=observations(record)
        except (ValueError,KeyError,TypeError):continue
        rows.extend(part);starts.append(start)
        sessions.append(dict(id=record['data']['session_id'],file=record['file'],speed=part[0]['speed'],
                             startup_s=start,completion_s=max(0.,total-part[-1]['end_s']),measured_s=total))
    if len(sessions)<2 or len(rows)<6:raise ValueError('Se requieren dos sesiones completas y seis tramos.')
    coef=coefficients(rows);startup=float(np.median(starts));validation=[]
    repeated_speeds = all(sum(s['speed'] == row['speed'] for s in sessions) >= 2 for row in rows)
    timing = fit_timing_by_speed(rows, sessions, coef) if repeated_speeds else []
    for session in sessions:
        train=[r for r in rows if r['session']!=session['id']]
        test=[r for r in rows if r['session']==session['id']]
        c = coefficients(train)
        training_sessions = [s for s in sessions if s['id'] != session['id']]
        lead = float(np.median([s['startup_s'] for s in training_sessions]))
        tail = float(np.median([s['completion_s'] for s in training_sessions]))
        factor = 1.
        if repeated_speeds:
            adjustment = fit_timing_by_speed(train, training_sessions, c)
            factor, lead, tail = timing_adjustments(session['speed'], {'timing_by_speed': adjustment})
        predicted = lead + sum(float(np.array(r['x']) @ c)*factor*100/r['speed'] for r in test) + tail
        validation.append(dict(session=session['id'],speed=session['speed'],measured_s=session['measured_s'],
                               predicted_s=predicted,error_s=predicted-session['measured_s']))
    return dict(schema=1,robot='IRB120-3/0.6',method='duration100 = c0*(1-exp(-max_delta_deg/5)) + c1*max(delta_deg/vmax_deg_s)',
                coefficients=coef.tolist(),startup_s=startup,completion_s=float(np.median([s['completion_s'] for s in sessions])),blend=.08,
                timing_by_speed=timing,
                speed_range=[min(r['speed'] for r in rows),max(r['speed'] for r in rows)],
                sessions=sessions,observations=rows,validation_leave_one_session_out=validation,
                note='Solo estimación gráfica de tiempos. Mezcla LSPB 8% asumida, no ajustada a errores angulares. Escala temporal inversa al porcentaje; fuera del rango observado es extrapolación. No es el planificador ABB.')
