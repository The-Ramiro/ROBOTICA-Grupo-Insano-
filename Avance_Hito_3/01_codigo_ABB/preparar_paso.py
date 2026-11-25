"""Prepara propuestas sin transmitirlas. No implementa movimiento físico.

Los límites de demostración NO son una autorización para usarlos en el robot.
La celda, herramienta, carga, colisiones y parada deben revisarse aparte.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET


class ProposalRejected(ValueError):
    pass


def finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProposalRejected(f'{name}: se requiere un numero, sin texto ni booleanos')
    value = float(value)
    if not math.isfinite(value):
        raise ProposalRejected(f'{name}: valor no finito')
    return value


@dataclass(frozen=True)
class PreviewLimits:
    """Valores de demostración exclusivos para preparar una vista previa."""
    max_step_deg: float = 1.0
    max_speed_deg_s: float = 1.0
    max_acceleration_deg_s2: float = 2.0
    max_sample_age_s: float = 0.5
    joint_margin_deg: float = 2.0
    period_s: float = 0.02

    def validate(self):
        for name, value in vars(self).items():
            if finite_number(value, name) <= 0:
                raise ProposalRejected(f'{name}: debe ser positivo')
        if self.period_s > 0.1 or self.period_s < 0.001:
            raise ProposalRejected('Periodo de vista previa fuera de rango')


def model_limits(path=None):
    path = path or Path(__file__).with_name('urdf')/'irb120.urdf'
    joints = [j for j in ET.parse(path).getroot().findall('joint') if j.get('type') == 'revolute']
    if [j.get('name') for j in joints] != [f'joint_{i}' for i in range(1, 7)]:
        raise ProposalRejected('Orden del modelo inesperado')
    return [(math.degrees(float(j.find('limit').get('lower'))),
             math.degrees(float(j.find('limit').get('upper')))) for j in joints]


def prepare_step(axes_deg, axis, delta_deg, measured_at, *, now=None, limits=None):
    """Un eje, límites articulares y quintica suave; NO valida colisiones.

    measured_at y now usan time.monotonic() de este mismo proceso. Nunca deben
    importarse timestamps monotónicos de otra computadora ni renovar muestras viejas.
    """
    limits = limits or PreviewLimits()
    limits.validate()
    now = time.monotonic() if now is None else finite_number(now, 'now')
    measured_at = finite_number(measured_at, 'measured_at')
    if not 0 <= now-measured_at <= limits.max_sample_age_s:
        raise ProposalRejected('Lectura vencida o timestamp futuro: obtener otra lectura')
    if isinstance(axis, bool) or not isinstance(axis, int) or not 1 <= axis <= 6:
        raise ProposalRejected('Elegir un eje de 1 a 6')
    if not isinstance(axes_deg, (list, tuple)) or len(axes_deg) != 6:
        raise ProposalRejected('Se requieren seis angulos medidos')
    start = [finite_number(v, f'J{i+1}') for i, v in enumerate(axes_deg)]
    delta = finite_number(delta_deg, 'paso')
    if not 0 < abs(delta) <= limits.max_step_deg:
        raise ProposalRejected('Paso nulo o superior al limite de vista previa')
    target = start.copy()
    target[axis-1] += delta  # Sin wrap: conservar las vueltas de J6.
    for i, (lo, hi) in enumerate(model_limits()):
        for q in (start[i], target[i]):
            if not lo+limits.joint_margin_deg <= q <= hi-limits.joint_margin_deg:
                raise ProposalRejected(f'J{i+1} fuera del margen del modelo')
    # s(u)=10u³-15u⁴+6u⁵. Max s'=1.875; max |s''|=10/sqrt(3).
    duration = max(0.5, 1.875*abs(delta)/limits.max_speed_deg_s,
                   math.sqrt((10/math.sqrt(3))*abs(delta)/limits.max_acceleration_deg_s2))
    count = math.ceil(duration/limits.period_s)
    duration = count*limits.period_s
    samples = []
    for k in range(count+1):
        u = k/count
        blend = 10*u**3-15*u**4+6*u**5
        q = start.copy()
        q[axis-1] += delta*blend
        samples.append({'time_s': k*limits.period_s, 'positions_deg': q})
    return {
        'kind': 'preview_only', 'hardware_execution_allowed': False,
        'collision_checked': False, 'lab_limits_approved': False,
        'start_deg': start, 'target_deg': target, 'axis': axis,
        'duration_s': duration,
        'peak_speed_deg_s': 1.875*abs(delta)/duration,
        'peak_acceleration_deg_s2': (10/math.sqrt(3))*abs(delta)/duration**2,
        'samples': samples,
        'note': 'Propuesta de simulacion. No demuestra ausencia de colisiones ni seguridad fisica.'}


def send_to_robot(*args, **kwargs):
    """Bloqueo explícito: falta integrar y revisar el canal físico de ABB."""
    raise ProposalRejected('Envio fisico no implementado: esta herramienta solo prepara vistas previas')
