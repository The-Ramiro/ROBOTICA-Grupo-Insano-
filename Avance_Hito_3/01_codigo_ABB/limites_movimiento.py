"""Límites configurables de vista previa, acotados por la ficha ABB y el URDF."""
import json
from pathlib import Path
from preparar_paso import finite_number, ProposalRejected, model_limits

ROOT = Path(__file__).resolve().parent
SOURCE = 'https://library.e.abb.com/public/7aa0711a20fa41c49a8fdf3bbc3d5bb0/IRB120-Rev.J-ROBO149EN_D.pdf'
# IRB120-3/0.6 estándar, no IRB120T. J6: rango estándar, sin ampliarlo.
FACTORY = ((-165., 165., 250.), (-110., 110., 250.), (-110., 70., 250.),
           (-160., 160., 320.), (-120., 120., 320.), (-400., 400., 420.))


def defaults():
    return {'schema': 1, 'robot': 'IRB120-3/0.6', 'source': SOURCE,
            'global_speed_percent': 1., 'preview_acceleration_deg_s2': 2.,
            'joint_margin_deg': 2., 'physical_execution_allowed': False,
            'axes': [{'min_deg': lo, 'max_deg': hi, 'axis_speed_percent': 100.,
                      'max_command_travel_percent': 1.} for lo, hi, _ in FACTORY]}


def percentage(value, name, positive=False):
    value = finite_number(value, name)
    if not (0 < value <= 100 if positive else 0 <= value <= 100):
        raise ProposalRejected(name+': porcentaje fuera de rango 0..100')
    return value


def validate(config):
    if config.get('robot') != 'IRB120-3/0.6':
        raise ProposalRejected('Modelo distinto del IRB120 estandar')
    percentage(config.get('global_speed_percent'), 'Velocidad global')
    acceleration = finite_number(config.get('preview_acceleration_deg_s2'), 'Aceleracion')
    if not 0 < acceleration <= 100:
        raise ProposalRejected('Aceleracion de vista previa fuera de rango')
    margin = finite_number(config.get('joint_margin_deg'), 'Margen articular')
    if not 0 <= margin <= 10:
        raise ProposalRejected('Margen articular fuera de rango')
    if config.get('physical_execution_allowed') is not False:
        raise ProposalRejected('Este perfil no habilita movimiento fisico')
    axes = config.get('axes')
    if not isinstance(axes, list) or len(axes) != 6:
        raise ProposalRejected('Faltan los seis ejes')
    for i, (row, (lo, hi, _)) in enumerate(zip(axes, FACTORY)):
        lower = finite_number(row.get('min_deg'), 'Minimo')
        upper = finite_number(row.get('max_deg'), 'Maximo')
        if not lo <= lower < upper <= hi or upper-lower <= 2*margin:
            raise ProposalRejected(f'J{i+1}: rango fuera de ABB o menor que el margen')
        percentage(row.get('axis_speed_percent'), f'J{i+1} velocidad')
        percentage(row.get('max_command_travel_percent'), f'J{i+1} recorrido por orden', positive=True)
    return config


def load(path=None):
    path = Path(path or ROOT/'parametros_movimiento.json')
    return validate(json.loads(path.read_text(encoding='utf-8')) if path.exists() else defaults())


def save(config, path=None):
    validate(config)
    path = Path(path or ROOT/'parametros_movimiento.json')
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(config, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    temporary.replace(path)


def effective(config):
    validate(config)
    result = []
    for row, (lo, hi, speed), (urdf_lo, urdf_hi) in zip(config['axes'], FACTORY, model_limits()):
        pct = config['global_speed_percent']*row['axis_speed_percent']/100.
        low = max(row['min_deg'], urdf_lo)+config['joint_margin_deg']
        high = min(row['max_deg'], urdf_hi)-config['joint_margin_deg']
        if low >= high:
            raise ProposalRejected('Rango efectivo vacio')
        result.append({'min_deg': low, 'max_deg': high,
                       'speed_deg_s': speed*pct/100., 'speed_percent': pct,
                       'range_percent': 100*(row['max_deg']-row['min_deg'])/(hi-lo),
                       'max_command_delta_deg': (hi-lo)*row['max_command_travel_percent']/100.})
    return result


def angle_percent(axis_index, angle):
    """0%=mínimo ABB; 100%=máximo ABB. No es porcentaje de velocidad."""
    lo, hi, _ = FACTORY[axis_index]
    return 100*(finite_number(angle, 'Angulo')-lo)/(hi-lo)
