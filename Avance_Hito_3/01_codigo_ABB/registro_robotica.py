"""Registro de lecturas observadas; nunca sustituye lecturas ausentes por ceros."""
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import time
from uuid import uuid4
import numpy as np
from analisis_recorrido import singular_metrics


class Recorder:
    def __init__(self):
        self.samples = deque(maxlen=36000)
        self.events = deque(maxlen=5000)
        self.last_key = None
        self.last_message = ''
        self.last_reader_error = ''
        self.zero = time.monotonic()
        self.execution = None
        self.run = 0
        self.was_moving = False
        self.session_id = uuid4().hex
        self.state = 'waiting'
        self.active = False
        self.done_seen = None
        self.dynamic_samples = deque(maxlen=36000)
        self.archive_path = None
        self.reference = None

    def begin_execution(self, future, reference=None):
        if self.samples:
            self.archive()
        self.samples.clear();self.events.clear();self.dynamic_samples.clear()
        self.session_id=uuid4().hex
        self.zero=time.monotonic()
        self.execution=future;self.run=1;self.active=True;self.state='recording'
        self.last_key=None;self.done_seen=None;self.archive_path=None
        self.last_message='';self.last_reader_error=''
        self.reference=reference
        self.event('Nuevo recorrido: tiempo cero y gráficas reiniciadas.')

    def finish(self, confirmed=True):
        self.active=False
        failed=self.execution is not None and (self.execution.cancelled() or self.execution.exception() is not None)
        self.state='interrupted' if failed else 'completed'
        self.event('Recorrido interrumpido; registro retenido.' if failed else 'Recorrido finalizado; registro retenido.')
        if not confirmed:self.event('No se obtuvo una lectura final reciente; se conserva la última disponible.')
        self.archive()

    def status_text(self):
        return {'waiting':'Esperando recorrido', 'recording':'Recorrido en curso',
                'completed':'Recorrido finalizado · sesión retenida',
                'interrupted':'Recorrido interrumpido · sesión retenida'}.get(self.state,'Sesión retenida')

    def archive(self):
        if not self.samples:return
        folder=Path.home()/'abb-real'/'ordenes'
        try:
            folder.mkdir(parents=True,exist_ok=True)
            if self.archive_path is None:
                self.archive_path=folder/f'recorrido_{datetime.now():%Y%m%d_%H%M%S}_{self.session_id[:8]}.json'
            self.save(self.archive_path)
        except (ValueError,OSError) as exc:
            self.events.append(dict(date=datetime.now().isoformat(),source='Registro',message='No se pudo guardar: '+str(exc)))

    def save(self, filename):
        Path(filename).write_text(json.dumps(dict(format='imt342-registro-v1',
            samples=list(self.samples), events=list(self.events), session_id=self.session_id,
            state=self.state, dynamics=list(self.dynamic_samples), reference=self.reference), indent=2, allow_nan=False), encoding='utf-8')

    @classmethod
    def load(cls, filename):
        path = Path(filename)
        if path.stat().st_size > 30_000_000:
            raise ValueError('Registro demasiado grande.')
        obj = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(obj, dict) or obj.get('format') != 'imt342-registro-v1':
            raise ValueError('No es un registro de lecturas ABB compatible.')
        result = cls()
        reference=obj.get('reference')
        if reference is not None:
            t=np.asarray(reference['t'],dtype=float)
            if (t.ndim!=1 or not 1<=len(t)<=20000 or not np.isfinite(t).all()
                    or np.any(np.diff(t)<=0) or t[0]<0
                    or any(np.asarray(reference[k]).shape!=(len(t),6) or not np.isfinite(reference[k]).all()
                           for k in ('q','qd','qdd'))):
                raise ValueError('Perfil de referencia inválido.')
            result.reference=reference
        for row in obj.get('samples', [])[-36000:]:
            if (len(row['q']) != 6 or len(row['xyz']) != 3
                    or not np.all(np.isfinite([row['t'], row['sigma'], *row['q'], *row['xyz']]))
                    or (row.get('fk_tcp_error_mm') is not None and not np.isfinite(row['fk_tcp_error_mm']))):
                raise ValueError('Muestra de registro inválida.')
            if 'margin' in row and (not np.isfinite(row['margin']) or not 0 <= row['margin'] <= 100
                                    or row.get('rank') not in range(7)
                                    or not isinstance(row.get('run'), int) or row['run'] < 0):
                raise ValueError('Diagnóstico de singularidad inválido.')
            for key,size in (('goal_q',6),('goal_xyz',3),('tcp_xyz',3)):
                if row.get(key) is not None and (len(row[key])!=size or not np.isfinite(row[key]).all()):
                    raise ValueError('Referencia de seguimiento inválida.')
            result.samples.append(row)
        for row in obj.get('events', [])[-5000:]:
            result.events.append(dict(date=str(row.get('date', '')), source=str(row.get('source', '')),
                                      message=str(row.get('message', ''))))
        if result.samples:
            # Un registro restaurado no es una lectura actual. Dejar un hueco
            # al reconectar, conservando el eje temporal y los datos anteriores.
            result.zero = time.monotonic()-result.samples[-1]['t']-2.
            result.run = max(r.get('run',0) for r in result.samples)
            result.state = 'interrupted' if obj.get('state') == 'interrupted' else 'completed'
            result.session_id = str(obj.get('session_id') or result.session_id)
        for row in obj.get('dynamics', [])[-36000:]:
            if not np.isfinite(row['t']) or any(np.asarray(row[key]).shape!=(6,) or not np.isfinite(row[key]).all()
                                               for key in ('inertia','g','c','tau')):
                raise ValueError('Muestra dinámica inválida.')
            result.dynamic_samples.append(row)
        return result

    def event(self, message, source='Panel'):
        self.events.append(dict(time=time.monotonic()-self.zero,
                                date=datetime.now().isoformat(timespec='milliseconds'),
                                source=source, message=message))

    def capture(self, panel):
        if self.active and self.execution is not None and self.execution.done():
            if self.done_seen is None:self.done_seen=time.monotonic()
            elif time.monotonic()-self.done_seen>1.5:
                self.finish(confirmed=False)
        if not self.active:return
        message = panel.status.text()
        if message and message != self.last_message:
            self.event(message)
            self.last_message = message
        if panel.reader is None:
            return
        seq, q, stamp, error, tcp, tcp_stamp, tcp_error, comparison = panel.reader.snapshot()
        if error and error != self.last_reader_error:
            self.event(error, 'Lectura ABB')
        self.last_reader_error = error
        key = (id(panel.reader), seq)
        if key == self.last_key or q is None or error or time.monotonic()-stamp > 1.:
            return
        if self.done_seen is not None and stamp<self.done_seen:return
        instant=max(0.,stamp-self.zero)
        if self.samples and instant<=self.samples[-1]['t']:return
        self.last_key = key
        progress = dict(getattr(panel.auto_session, 'progress', {}))
        point = progress.get('point')
        pose = panel.model.tip(q, panel.envelope)
        sigma, condition, _, rank = singular_metrics(panel.model, panel.envelope, q)
        motion = getattr(panel, 'motion', None)
        sample_run = 1
        mismatch = None
        if comparison is not None:
            angles, position, angle_stamp, position_stamp = comparison
            if abs(angle_stamp-position_stamp) <= .1:
                mismatch = float(np.linalg.norm(panel.model.tip(angles, panel.envelope)[:3, 3]*1000-position))
        reference = progress.get('goal_deg')
        if reference is not None and (progress.get('state') not in ('moving','completed')
                                      or progress.get('goal_observed_at',0.) < self.zero
                                      or stamp < progress.get('goal_observed_at', float('inf'))):
            reference = None
        goal_xyz = (panel.model.tip(reference,panel.envelope)[:3,3]*1000).tolist() if reference is not None else None
        actual_xyz = np.asarray(tcp).tolist() if (tcp is not None and not tcp_error
            and tcp_stamp is not None and abs(stamp-tcp_stamp) <= .1) else None
        self.samples.append(dict(t=instant, q=list(q), xyz=(pose[:3, 3]*1000).tolist(),
                                 sigma=sigma, margin=100./condition, rank=rank, run=sample_run,
                                 moving=motion is not None and not motion.done(),
                                 goal_q=reference, goal_xyz=goal_xyz, tcp_xyz=actual_xyz,
                                 point=point, fk_tcp_error_mm=mismatch))
        if self.done_seen is not None:self.finish()

    def arrays(self):
        rows = list(self.samples)
        if not rows:
            raise ValueError('Todavía no hay lecturas del ABB en esta sesión.')
        t = np.array([r['t'] for r in rows])
        q = np.radians([r['q'] for r in rows])
        # La derivación numérica requiere tiempos estrictamente crecientes.
        keep = np.r_[True, np.diff(t) > 1e-6]
        t, q = t[keep], q[keep]
        rows = [r for r, use in zip(rows, keep) if use]
        qd = np.gradient(q, t, axis=0) if len(t) > 1 else np.full_like(q, np.nan)
        qdd = np.gradient(qd, t, axis=0) if len(t) > 2 else np.full_like(q, np.nan)
        for index in np.flatnonzero(np.diff(t) > .5):
            qd[max(0, index-1):index+3] = np.nan
            qdd[max(0, index-1):index+3] = np.nan
        return dict(t=t, q=q, qd=qd, qdd=qdd, xyz=np.array([r['xyz'] for r in rows]),
                    sigma=np.array([r['sigma'] for r in rows]),
                    error=np.array([np.nan if r['fk_tcp_error_mm'] is None else r['fk_tcp_error_mm'] for r in rows]),
                    events=list(self.events), samples=rows)
