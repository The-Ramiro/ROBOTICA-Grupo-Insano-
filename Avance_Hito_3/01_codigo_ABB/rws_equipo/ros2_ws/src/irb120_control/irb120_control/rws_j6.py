"""Diagnóstico RWS y preparación de ensayo puntual J6; no arranca RAPID ni motores."""
import argparse
from datetime import datetime, timezone
import getpass
import hashlib
import json
import math
from pathlib import Path
import re
import time
from urllib.parse import unquote

from irb120_conexion.rutas import PROJECT
from irb120_conexion.comunicacion.lectura_rws import state_records, error_details
from irb120_conexion.nodos.lectura_articular.puente_rws import build_base_url

MODULE = 'LabJointRws'
TASK = 'T_ROB1'
SYMBOL = '/rw/rapid/symbol/data/RAPID/T_ROB1/LabJointRws/pcTarget'
MODULE_PATH = '/rw/rapid/modules/LabJointRws'
MAIN_MODULE_PATH = '/rw/rapid/modules/Module1'
MASTERSHIP = '/rw/mastership/rapid'
CANDIDATE = Path(__file__).resolve().parents[1] / 'rapid/rws_j6/LabJointRws.mod'
ORIGINAL_MODULE_SHA256 = 'fbf5527946c8e2e9b3d627628326a495c8c614b8ea25b069a2ec067da797e1ec'
ORIGINAL_NORMALIZED_SHA256 = 'e39d1f16392c93096d19754bc826da61664632b6acf8a4cab06bf8e4e9fdc55c'
J6_MODULE_SHA256 = 'cb07b7fba9a6f869ed87f21a77cdfb70ea454232132a3e2973daaffb34d2fdf0'
# Versión 2 (servidor de objetivos, rws_mover.py). Revisada el 11/11/2026.
SERVER_CANDIDATE = Path(__file__).resolve().parents[1] / 'rapid/rws_mover/LabJointRws.mod'
SERVER_MODULE_SHA256 = '1a7ab00bf5d43fe285d4b2b0c333d04e8d07a7c5bb322b919eb68aff95f77789'
# Misma versión con permitirAuto := TRUE (plan B autorizado por el responsable).
SERVER_AUTO_MODULE_SHA256 = '4f387764e894dcfe3aa94fcffcb6c22a5abf70bf9c438b7c797f9b8fb5ddbc26'
EMPTY = [[9e9] * 6, [9e9] * 6]
LIMITS = ((-165,165),(-110,110),(-110,70),(-160,160),(-120,120),(-400,400))


class Refused(ValueError):
    pass


def single(records, key):
    values = [r[key] for r in records if key in r]
    if len(values) != 1:
        raise Refused('Falta un valor único para ' + key)
    return values[0]


def parse_target(text, allow_empty=False):
    try:
        value = json.loads(text, parse_constant=lambda x: None)
        if not isinstance(value,list) or len(value)!=2 or any(not isinstance(v,list) or len(v)!=6 for v in value):
            raise ValueError('Formato jointtarget')
        if any(type(v) not in (float,int) or not math.isfinite(v) for group in value for v in group):
            raise ValueError('Valor no finito o no numérico')
        if allow_empty and all(v>=8e9 for group in value for v in group):
            return [list(group) for group in EMPTY]
        for q,(lo,hi) in zip(value[0],LIMITS):
            if not lo<=q<=hi:raise ValueError('Eje fuera de límites nominales')
        if not all(v>=8e9 for v in value[1]):
            raise ValueError('Ensayo no preparado para ejes externos activos')
        return [[float(v) for v in value[0]], [9e9]*6]
    except (ValueError,TypeError) as exc:
        raise Refused('jointtarget inválido: '+str(exc)) from exc


def measured_target(records):
    rows=[r for r in records if r.get('_type')=='rapid-jointtarget']
    if len(rows)!=1:raise Refused('Falta el jointtarget de la tarea.')
    try:
        value=[[float(rows[0][f'j{i}']) for i in range(1,7)],
               [float(rows[0][f'ej{i}']) for i in range(1,7)]]
    except (KeyError,ValueError,TypeError) as exc:
        raise Refused('No llegaron los doce campos del jointtarget.') from exc
    return parse_target(json.dumps(value))


def make_j6_target(current, delta):
    current=parse_target(json.dumps(current))
    if not math.isfinite(delta) or abs(delta)>.1+1e-12:
        raise Refused('La primera prueba J6 limita el incremento a ±0.1°.')
    target=[list(v) for v in current]
    target[0][5]+=delta
    if not -398<=target[0][5]<=398:
        raise Refused('J6 queda fuera de los límites nominales con margen de 2°.')
    return target


def same_target(left,right,tolerance=1e-4):
    return all(abs(a-b)<=tolerance for a,b in zip(left[0],right[0])) and all(
        a>=8e9 and b>=8e9 for a,b in zip(left[1],right[1]))


def literal(target):
    return '['+','.join('['+','.join(format(v,'.15g') for v in group)+']' for group in target)+']'


_PERS_VALUE=re.compile(r'^([ \t]*PERS[ \t]+(jointtarget|num)[ \t]+\w+(?:\{(\d+)\})?[ \t]*:=[ \t]*)[^;]*;',flags=re.M)
_PERS_INITIAL={'jointtarget':'[[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]]','num':'0'}


def source_fingerprint(source):
    # El editor ABB puede normalizar CRLF; el resto del código debe coincidir.
    # Observado en RW6.12 (16/11/2026): el texto que devuelve el controlador lleva
    # el valor ACTUAL de cada PERS en su declaración (p. ej. un pcTarget pendiente,
    # con 9E+09 en vez de 9E9). Ese valor es dato, no código: se restituye el
    # inicial revisado antes de calcular la huella. Los datos se validan aparte
    # (pcTarget vacío, pcSeq == rbSeq, etc.).
    source=source.replace('\r\n','\n').replace('\r','\n').strip()
    source=_PERS_VALUE.sub(lambda m:m.group(1)+(
        '['+','.join([_PERS_INITIAL[m.group(2)]]*int(m.group(3)))+']'
        if m.group(3) else _PERS_INITIAL[m.group(2)])+';',source)
    return hashlib.sha256(source.encode()).hexdigest()


def module_label(digest):
    return {J6_MODULE_SHA256:'J6_revisado',ORIGINAL_NORMALIZED_SHA256:'original_v5',
            SERVER_MODULE_SHA256:'servidor_v2',SERVER_AUTO_MODULE_SHA256:'servidor_v2_auto'}.get(digest,'no_verificado')


def diagnose_inspection(inspection, historical=False):
    """Interpreta lecturas; nunca certifica PP, permisos locales ni movimiento seguro."""
    if not isinstance(inspection,dict):
        raise Refused('El archivo no contiene una inspección RWS reconocida.')
    resources=inspection.get('resources')
    if not isinstance(resources,dict):
        raise Refused('El archivo no contiene una inspección RWS reconocida.')
    issues=[]
    def value(label,key):
        entry=resources.get(label,{})
        if not entry.get('ok'):
            issues.append(f'NO_LEIDO_{label.upper()}: '+entry.get('error','Falta lectura.'))
            return None
        try:
            return single(entry.get('records',[]),key)
        except Refused as exc:
            issues.append(f'NO_LEIDO_{label.upper()}: '+str(exc))
            return None
    name=value('system','name');version=value('system','rwversionname')
    identity_ok=name=='120-101516' and version=='6.12.01.00'
    if not identity_ok:issues.append('IDENTIDAD_NO_VERIFICADA: comprobar controlador y RobotWare antes de continuar.')
    mode=value('mode','opmode');state=value('controller','ctrlstate')
    execution=value('execution','ctrlexecstate')
    if mode=='AUTO':
        issues.append('MODO_AUTO: falta pasar localmente a manual reducido; ProbarObjetivo y ServidorPC retornan sin mover en AUTO.')
    elif mode!='MANR':
        issues.append('MODO_NO_MANUAL_REDUCIDO: se necesita MANR para la rutina local de esta prueba.')
    if state=='motoroff':
        motor_note='Motores apagados: permite revisar menús; el movimiento requiere habilitación local del operador.'
    elif state=='motoron':
        motor_note='Motores encendidos: no escribir ni limpiar objetivos en este estado.'
    else:
        motor_note='Estado de parada/protección o estado desconocido: revisar con el responsable.'
        issues.append('ESTADO_CONTROLADOR: '+str(state)+'. No preparar movimiento; revisar el estado local.')
    if execution!='stopped':
        issues.append('RAPID_NO_DETENIDO: confirmar el estado y detener localmente antes de esta preparación.')
    target_text=value('target','value');target=None;pending=None
    if target_text is not None:
        try:
            target=parse_target(target_text,allow_empty=True)
            pending=target!=EMPTY
            if pending:
                issues.append('OBJETIVO_PENDIENTE: revisar y limpiar con motores apagados/RAPID detenido antes del ensayo local sin movimiento.')
        except Refused as exc:
            issues.append('OBJETIVO_NO_VALIDO: '+str(exc))
    source=value('module','module-text')
    digest=source_fingerprint(source) if isinstance(source,str) else None
    module_version=module_label(digest)
    if module_version not in ('J6_revisado','servidor_v2','servidor_v2_auto'):
        issues.append('MODULO_NO_REVISADO: confirmar el módulo vigente; instalar la v2 con rws_mover instalar.')
    snapshot_ok=identity_ok and mode=='MANR' and state in ('motoroff','motoron') and execution=='stopped' and pending is False and module_version in ('J6_revisado','servidor_v2','servidor_v2_auto')
    return {'action':'diagnose_read_only','observed_at_utc':inspection.get('observed_at_utc'),
            'base_url':inspection.get('base_url'),'historical_report':historical,
            'requests':'none_historical_file' if historical else 'GET_only',
            'identity':{'name':name,'rwversionname':version},'identity_verified':identity_ok,
            'mode':mode,'ctrlstate':state,'execution':execution,'motor_note':motor_note,
            'module_version':module_version,'module_normalized_sha256':digest,
            'target_pending':pending,'target_joints_deg':target[0] if pending else None,
            'issues':issues,'local_navigation_prerequisites_observed':snapshot_ok,
            'local_execution_verified':False,'motion_authorized_by_diagnostic':False,
            'motion_command_sent':False,
            'next_step':('Lecturas compatibles con revisar Debug en manual. Seleccionar MostrarObjetivo desde el inicio '
                         'y comprobar su ejecución local con el operador; esta lectura no verifica puntero, permisos ni arranque.'
                         if snapshot_ok else 'Resolver los puntos indicados antes de preparar otro objetivo. '
                         'Este diagnóstico no cambia modo, motores, variables ni puntero.')}


def print_diagnosis(report):
    print('DIAGNOSTICO DE SOLO LECTURA' + (' · ARCHIVO HISTORICO, NO ES ESTADO ACTUAL' if report['historical_report'] else ''))
    print('Lectura UTC:',report.get('observed_at_utc'))
    print(f"Modo: {report['mode']} | Controlador: {report['ctrlstate']} | RAPID: {report['execution']}")
    print('Módulo:',report['module_version'],'| Objetivo pendiente:',
          'SI' if report['target_pending'] is True else 'NO' if report['target_pending'] is False else 'DESCONOCIDO')
    print(report['motor_note'])
    for issue in report['issues']:print('COMPROBAR:',issue)
    print('Siguiente paso:',report['next_step'])
    print('Este resultado no autoriza ni verifica movimiento físico.')


class Client:
    def __init__(self, session, base, timeout=3.):
        self.session, self.base, self.timeout = session, base, timeout
        self.write_events=[]
        self.extra_reads=set()

    def read(self, path, **params):
        allowed = {'/rw/system', '/rw/panel/ctrlstate', '/rw/panel/opmode',
                   '/rw/rapid/execution', '/rw/rapid/tasks/T_ROB1/motion',
                   '/rw/rapid/modules', MODULE_PATH, MAIN_MODULE_PATH, SYMBOL}
        if path not in allowed | self.extra_reads:
            raise Refused('Recurso fuera de la lista de diagnóstico.')
        response = self.session.get(self.base + path, params={'json': '1', **params},
                                    timeout=self.timeout, allow_redirects=False)
        if response.status_code != 200:
            detail = error_details(response.text)
            raise Refused(f'GET {path}: HTTP {response.status_code}; {detail}')
        return state_records(response.text)

    def inspect(self):
        started = time.monotonic()
        report = {'observed_at_utc': datetime.now(timezone.utc).isoformat(),
                  'base_url': self.base, 'route': 'RWS_RAPID_not_EGM',
                  'requests': 'GET_only', 'motion_command_sent': False,
                  'resources': {}}
        for label, path, params in (
            ('system', '/rw/system', {}),
            ('controller', '/rw/panel/ctrlstate', {}),
            ('mode', '/rw/panel/opmode', {}),
            ('execution', '/rw/rapid/execution', {}),
            ('joints', '/rw/rapid/tasks/T_ROB1/motion', {'resource': 'jointtarget'}),
            ('modules', '/rw/rapid/modules', {'task': TASK}),
            ('target', SYMBOL, {}),
            ('module', MODULE_PATH, {'resource': 'module-text', 'task': TASK}),
        ):
            # No se registran contraseña, cabeceras ni cookies.
            try:
                records = self.read(path, **params)
                if label == 'module':
                    self.resolve_module_text(records)
                report['resources'][label] = {'ok': True, 'records': records}
            except Refused as exc:
                report['resources'][label] = {'ok': False, 'error': str(exc)}
                if 'HTTP 401' in str(exc) or 'HTTP 403' in str(exc):
                    break
        report['elapsed_s'] = time.monotonic() - started
        return report

    def resolve_module_text(self, records, module=MODULE):
        if module not in (MODULE, 'Module1', 'IMT342_DestinoDirecto', 'IMT342_Automatico'):
            raise Refused('Módulo fuera de la revisión puntual.')
        for row in records:
            if 'module-text' in row:
                return
            if 'file-path' not in row:
                continue
            path = unquote(row['file-path']).strip('"')
            # Solo el temporal del módulo indicado por este GET; nunca seguir
            # URLs, rutas arbitrarias ni enlaces suministrados por otro servidor.
            if not re.fullmatch(r'/hd0a/TEMP[A-Za-z0-9_.-]+', path):
                raise Refused('Ruta de temporal no reconocida; exportar el módulo para revisión.')
            candidates = ('/fileservice' + path, '/fileservice/' + path)
            for resource in candidates:
                response = self.session.get(self.base + resource, timeout=self.timeout,
                                            allow_redirects=False)
                if response.status_code == 404:
                    continue
                if response.status_code != 200:
                    raise Refused(f'GET del temporal RAPID: HTTP {response.status_code}; '
                                  f'{error_details(response.text)}')
                raw = response.content
                if len(raw) > 100000:
                    raise Refused('Módulo demasiado grande para esta inspección puntual.')
                try:
                    source = raw.decode('utf-8-sig')
                except UnicodeDecodeError:
                    source = raw.decode('latin-1')
                if not re.search(r'\bMODULE\s+' + module + r'\b', source, flags=re.I):
                    raise Refused('El temporal no contiene el módulo esperado: '+module)
                row['module-text'] = source
                return
            raise Refused('No se pudo leer el temporal del módulo; no se habilita ninguna escritura.')
        raise Refused('La respuesta del módulo no contiene su texto ni un temporal reconocido.')

    def module_source(self, module=MODULE):
        if module not in (MODULE, 'Module1'):
            raise Refused('Módulo fuera de la revisión puntual.')
        path=MODULE_PATH if module==MODULE else MAIN_MODULE_PATH
        records=self.read(path,resource='module-text',task=TASK)
        self.resolve_module_text(records,module)
        return single(records,'module-text')

    def assert_stopped_off(self):
        state=single(self.read('/rw/panel/ctrlstate'),'ctrlstate')
        mode=single(self.read('/rw/panel/opmode'),'opmode')
        execution=single(self.read('/rw/rapid/execution'),'ctrlexecstate')
        if state!='motoroff' or execution!='stopped' or mode not in ('AUTO','MANR'):
            raise Refused(f'No se escribe: estado={state}, modo={mode}, RAPID={execution}. '
                          'Se requieren motores apagados y RAPID detenido, sin cambiar el modo desde la PC.')
        return {'ctrlstate':state,'opmode':mode,'ctrlexecstate':execution}

    def get_target(self):
        return parse_target(single(self.read(SYMBOL),'value'),allow_empty=True)

    def set_target(self,target):
        # Escritura de objetivo separada de la actualización explícita del módulo.
        target=parse_target(literal(target),allow_empty=True)
        self.assert_stopped_off()
        self.write_events.append({'at_utc':datetime.now(timezone.utc).isoformat(),
                                  'target':target,'result':'pending_or_unknown'})
        response=self.session.post(self.base+SYMBOL,params={'action':'set'},data={'value':literal(target)},
                                   timeout=self.timeout,allow_redirects=False)
        self.write_events[-1]['http_status']=response.status_code
        self.write_events[-1]['result']='accepted' if response.status_code==204 else 'rejected'
        if response.status_code!=204:
            raise Refused(f'POST pcTarget: HTTP {response.status_code}; {error_details(response.text)}. '
                          'No se reintenta ni se cambia el modo para obtener permisos.')

    def verify_identity(self):
        identity=next((r for r in self.read('/rw/system') if r.get('_type')=='sys-system-li'),{})
        if identity.get('name')!='120-101516' or identity.get('rwversionname')!='6.12.01.00':
            raise Refused('Identidad/RobotWare no coinciden con el controlador revisado.')
        return {'name':identity['name'],'rwversionname':identity['rwversionname']}

    def stable_target(self):
        self.assert_stopped_off()
        first=measured_target(self.read('/rw/rapid/tasks/T_ROB1/motion',resource='jointtarget'))
        time.sleep(.2)
        second=measured_target(self.read('/rw/rapid/tasks/T_ROB1/motion',resource='jointtarget'))
        if not same_target(first,second,tolerance=.005):
            raise Refused('La postura cambió entre las dos lecturas; no se escribe.')
        return second

    def reception_test(self):
        identity=self.verify_identity()
        digest=self.reviewed_source()
        if self.get_target()!=EMPTY:
            raise Refused('Ya hay un objetivo pendiente. No se sobrescribe; revisar con el operador.')
        current=self.stable_target()
        report={'observed_at_utc':datetime.now(timezone.utc).isoformat(),'identity':identity,
                'action':'write_read_clear_same_measured_angles','module_sha256':digest,
                'motion_command_sent':False,'initial_joints_deg':current[0],
                'target_joints_deg':current[0],'readback_verified':False,'cleared':False}
        # Escribir los ángulos actuales no ejecuta ProbarObjetivo. El operador mantiene
        # RAPID detenido durante toda esta prueba y no pulsa Start.
        self.set_target(current)
        actual=self.get_target()
        report['readback_joints_deg']=actual[0]
        report['readback_verified']=same_target(current,actual)
        report['readback_errors_deg']=[a-b for a,b in zip(actual[0],current[0])]
        # Limpiar incluso si la comparación falla, siempre que siga detenido/apagado.
        self.set_target(EMPTY)
        report['cleared']=self.get_target()==EMPTY
        after=self.stable_target()
        report['after_joints_deg']=after[0]
        report['no_displacement_observed']=same_target(current,after,tolerance=.005)
        report['final_state']=self.assert_stopped_off()
        report['passed']=all(report[k] for k in ('readback_verified','cleared','no_displacement_observed'))
        return report

    def reviewed_source(self,j6_only=False):
        source=self.module_source()
        digest=source_fingerprint(source)
        if j6_only and digest!=J6_MODULE_SHA256:
            raise Refused('No se prepara movimiento: falta la versión J6 revisada con vLabJ6 '
                          'y conservación de J1–J5. La versión original usa v5. '
                          'Usar actualizar_modulo y verificar el programa en el FlexPendant.')
        if digest not in (ORIGINAL_NORMALIZED_SHA256,J6_MODULE_SHA256,SERVER_MODULE_SHA256,SERVER_AUTO_MODULE_SHA256):
            raise Refused('El módulo cambió desde su revisión. Ejecutar inspeccionar antes de escribir.')
        return hashlib.sha256(source.encode()).hexdigest()

    def update_module(self,directory,candidate_path=None,candidate_sha=J6_MODULE_SHA256,accepted=None):
        """Solo sustituye LabJointRws conocido; nunca carga programas ni arranca tareas."""
        candidate=(candidate_path or CANDIDATE).read_text(encoding='utf-8')
        accepted=set(accepted or (ORIGINAL_NORMALIZED_SHA256,J6_MODULE_SHA256))|{candidate_sha}
        if source_fingerprint(candidate)!=candidate_sha:
            raise Refused('El candidato local cambió; requiere una nueva revisión.')
        identity=self.verify_identity()
        state=self.assert_stopped_off()
        if state['opmode']!='AUTO':
            raise Refused('Esta transferencia requiere AUTO, motores apagados y RAPID detenido. '
                          'No se solicita RMMP ni se cambia modo desde Linux.')
        if self.get_target()!=EMPTY:
            raise Refused('pcTarget no está vacío; no se edita el módulo con un objetivo pendiente.')
        before=self.module_source()
        digest=source_fingerprint(before)
        if digest not in accepted:
            raise Refused('El módulo del ABB difiere de las versiones revisadas. Solo inspeccionar.')
        if digest==candidate_sha:
            return {'action':'update_LabJointRws','passed':True,'already_installed':True,
                    'motion_command_sent':False,'program_verification_required':True,
                    'note':'Ya coincide; no se hizo POST. Verificar programa localmente antes de mover.'}
        modules_before=self.read('/rw/rapid/modules',task=TASK)
        names_before=sorted(single([row],'name') for row in modules_before)
        if MODULE not in names_before or 'Module1' not in names_before:
            raise Refused('No se encontraron LabJointRws y Module1; no se transfiere.')
        main_before=self.module_source('Module1')
        main_digest=source_fingerprint(main_before)
        initial=self.stable_target()
        directory.mkdir(parents=True,exist_ok=True)
        stamp=datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%fZ')
        backup=directory/('respaldo_modulo_'+stamp)
        backup.mkdir()
        for name,source in ((MODULE,before),('Module1',main_before),('LabJointRws_candidato',candidate)):
            (backup/(name+'.mod')).write_text(source,encoding='utf-8')
        journal=backup/'transferencia.json'
        report={'action':'update_LabJointRws','identity':identity,'backup_directory':str(backup.resolve()),
                'motion_command_sent':False,'program_pointer_may_reset':True,
                'program_verification_required':True,'before_sha256':digest,
                'candidate_sha256':candidate_sha,'module1_before_sha256':main_digest,
                'initial_joints_deg':initial[0],'passed':False}
        def record(phase):
            report['phase']=phase
            report['write_events']=list(self.write_events)
            journal.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        def post(path,params,data,expected):
            self.write_events.append({'at_utc':datetime.now(timezone.utc).isoformat(),
                                      'resource':path,'action':params['action'],'result':'pending_or_unknown'})
            record('request_pending_or_unknown')
            response=self.session.post(self.base+path,params=params,data=data,
                                       headers={'Content-Type':'application/x-www-form-urlencoded'},
                                       timeout=self.timeout,allow_redirects=False)
            event=self.write_events[-1]
            event.update(http_status=response.status_code,
                         result='accepted' if response.status_code==expected else 'rejected')
            detail=error_details(response.text)
            # RW6.12 observado: la edición puede liberar mastership internamente.
            # Solo esta respuesta exacta de release confirma que ya no lo poseemos;
            # otros 403 siguen siendo errores, incluido request/set-module-text.
            if (path==MASTERSHIP and params['action']=='release' and response.status_code==403 and
                    str(detail.get('code'))=='-1073445880' and
                    'The resource is not held by caller' in detail.get('message','')):
                event['result']='already_not_held'
                report['mastership_release']='already_not_held_by_this_session'
                record('permission_already_released')
                return
            record('response_received')
            if response.status_code!=expected:
                raise Refused(f"{params['action']}: HTTP {response.status_code}; {detail}. "
                              'Sin reintento automático. Revisar el informe y el módulo del ABB.')
        record('backed_up_no_write')
        print('Respaldo previo:',backup.resolve(),flush=True)
        acquisition_event=len(self.write_events)
        # Solicitud únicamente del dominio RAPID. Incluso un timeout de adquisición
        # puede haber concedido mastership: intentar liberarlo en finally.
        try:
            post(MASTERSHIP,{'action':'request'},{},204)
            state=self.assert_stopped_off()
            if state['opmode']!='AUTO' or self.get_target()!=EMPTY:
                raise Refused('Cambió el modo o pcTarget; transferencia cancelada.')
            if source_fingerprint(self.module_source())!=digest or source_fingerprint(self.module_source('Module1'))!=main_digest:
                raise Refused('El programa cambió después del respaldo; no se sustituye.')
            post(MODULE_PATH,{'task':TASK,'action':'set-module-text'},{'text':candidate},200)
            report['readback_sha256']=source_fingerprint(self.module_source())
            report['module1_after_sha256']=source_fingerprint(self.module_source('Module1'))
            report['module_list_unchanged']=names_before==sorted(single([r],'name') for r in self.read('/rw/rapid/modules',task=TASK))
            report['target_empty']=self.get_target()==EMPTY
            report['final_state']=self.assert_stopped_off()
            after=self.stable_target()
            report['after_joints_deg']=after[0]
            report['no_displacement_observed']=same_target(initial,after,tolerance=.005)
            report['passed']=(report['readback_sha256']==candidate_sha and
                              report['module1_after_sha256']==main_digest and report['module_list_unchanged'] and
                              report['target_empty'] and report['no_displacement_observed'])
            if not report['passed']:
                raise Refused('La verificación posterior no pasó. No arrancar RAPID; inspeccionar. Sin restauración automática.')
        except BaseException:
            report['passed']=False
            record('interrupted_inspect_before_any_start')
            raise
        finally:
            # No modifica motores, ejecución ni puntero; solo libera este permiso.
            try:
                if self.write_events[acquisition_event]['result']!='rejected':
                    post(MASTERSHIP,{'action':'release'},{},204)
            except BaseException:
                report['passed']=False
                record('release_failed_inspect_session')
                raise
            record('finished' if report['passed'] else 'interrupted_inspect_before_any_start')
        return report

    def prepare_j6(self,delta=.1):
        if delta==0 or not math.isfinite(delta) or abs(delta)>.1:
            raise Refused('Preparar admite un incremento no nulo de hasta ±0.1°. Para cero usar comprobar_recepcion.')
        identity=self.verify_identity()
        digest=self.reviewed_source(j6_only=True)
        if self.get_target()!=EMPTY:
            raise Refused('Hay un objetivo pendiente; no se sobrescribe ni se acumulan incrementos.')
        current=self.stable_target()
        target=make_j6_target(current,delta)
        self.set_target(target)
        readback=self.get_target()
        if not same_target(target,readback):
            self.set_target(EMPTY)
            raise Refused('La lectura de vuelta no coincide; se solicitó vaciar pcTarget. Inspeccionar.')
        self.assert_stopped_off()
        return {'schema':1,'action':'prepare_J6_only','observed_at_utc':datetime.now(timezone.utc).isoformat(),
                'identity':identity,'module_sha256':digest,'base_url':self.base,
                'initial_joints_deg':current[0],'target_joints_deg':target[0],
                'delta_deg':delta,'readback_joints_deg':readback[0],'readback_verified':True,
                'target_pending':True,'motion_command_sent':False,'passed':True,
                'note':'Solo pcTarget. El operador ejecuta LabJointRws/ProbarObjetivo en manual reducido. '
                       'Si cancela o cambia de postura, limpiar y preparar de nuevo.'}

    def clear(self):
        self.verify_identity();self.reviewed_source();self.set_target(EMPTY)
        cleared=self.get_target()==EMPTY
        return {'action':'clear_pcTarget','cleared':cleared,'passed':cleared,'motion_command_sent':False}

    def observe(self,plan,duration,samples_path):
        if plan.get('schema')!=1 or plan.get('action')!='prepare_J6_only' or plan.get('base_url')!=self.base:
            raise Refused('El archivo no es un plan J6 de este controlador.')
        initial=parse_target(json.dumps([plan.get('initial_joints_deg'),[9e9]*6]))
        expected=make_j6_target(initial,plan.get('delta_deg'))
        target=parse_target(json.dumps([plan.get('target_joints_deg'),[9e9]*6]))
        if not same_target(expected,target) or abs(plan['delta_deg'])<.03:
            raise Refused('El plan no coincide con un ensayo J6 observable de al menos 0.03°.')
        identity=self.verify_identity()
        self.reviewed_source(j6_only=True)
        started=time.monotonic();samples=[];stable=0;departed=False;reason='Tiempo agotado sin llegada comprobada.'
        with samples_path.open('x') as stream:
            while time.monotonic()-started<duration:
                q=measured_target(self.read('/rw/rapid/tasks/T_ROB1/motion',resource='jointtarget'))[0]
                others=max(abs(a-b) for a,b in zip(q[:5],initial[0][:5]))
                delta=q[5]-initial[0][5];error=q[5]-target[0][5]
                row={'t_s':time.monotonic()-started,'joints_deg':q,'delta_j6_deg':delta,
                     'error_j6_deg':error,'max_other_joint_change_deg':others}
                samples.append(row);stream.write(json.dumps(row)+'\n');stream.flush()
                print(f'J6={q[5]:.5f}° | objetivo={target[0][5]:.5f}° | error={error:+.5f}° '
                      f'| cambio máximo J1–J5={others:.5f}°',flush=True)
                if others>.02 or abs(delta)>abs(plan['delta_deg'])+.02:
                    reason='ALERTA: recorrido fuera de la prueba. Solicitar parada al operador; este monitor solo lee.'
                    break
                departed=departed or abs(delta)>.03
                stable=stable+1 if departed and abs(error)<=.02 else 0
                if stable>=3:
                    reason='Tres lecturas de llegada dentro de 0.02°; contrastar con FlexPendant.'
                    break
                time.sleep(.2)
        return {'action':'observe_J6_only','identity':identity,'motion_command_sent':False,
                'passed':stable>=3,'reason':reason,'samples':len(samples),
                'final_sample':samples[-1] if samples else None,
                'samples_path':str(samples_path),'note':'No calcula velocidad/aceleración física fiable a 5 Hz ni ordena parada.'}


def save_inspection(report, directory):
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%fZ')
    resources = report['resources']
    module = next((r['module-text'] for r in resources.get('module', {}).get('records', [])
                   if 'module-text' in r), None)
    if module is not None:
        path = directory / ('LabJointRws_' + stamp + '.mod')
        path.write_text(module, encoding='utf-8')
        report['module_sha256'] = hashlib.sha256(module.encode()).hexdigest()
        report['module_source_path'] = str(path.resolve())
    path = directory / ('inspeccion_' + stamp + '.json')
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('inspeccionar','diagnosticar','actualizar_modulo','comprobar_recepcion','preparar','limpiar','observar'))
    parser.add_argument('--host', default='192.168.125.1')
    parser.add_argument('--user', default='Default User')
    parser.add_argument('--http-port', type=int, default=80, help='Puerto distinto de 80 solo en loopback para pruebas')
    parser.add_argument('--log-dir', type=Path, default=PROJECT / 'salidas/control_articular/rws_j6')
    parser.add_argument('--delta',type=float,default=.1,help='Incremento relativo de J6 para preparar; máximo ±0.1°')
    parser.add_argument('--plan',type=Path,help='Informe preparar_*.json para observar la llegada')
    parser.add_argument('--duracion',type=float,default=60.,help='Segundos de observación, de 5 a 120')
    parser.add_argument('--informe',type=Path,help='Solo diagnosticar: analizar una inspección guardada sin conexión ni contraseña')
    args = parser.parse_args(argv)
    if args.informe and args.action!='diagnosticar':parser.error('--informe solo se admite con diagnosticar')
    if args.action=='observar' and args.plan is None:parser.error('observar requiere --plan preparar_*.json')
    if not math.isfinite(args.duracion) or not 5<=args.duracion<=120:parser.error('--duracion debe estar entre 5 y 120 s')
    try:
        base = build_base_url(args.host, args.http_port)
    except ValueError as exc:
        parser.error(str(exc))
    import requests
    from requests.auth import HTTPDigestAuth
    client=None
    try:
        if args.action=='diagnosticar' and args.informe:
            report=diagnose_inspection(json.loads(args.informe.read_text()),historical=True)
            print_diagnosis(report)
            return 0 if report['local_navigation_prerequisites_observed'] else 2
        purpose=('solo GET' if args.action in ('inspeccionar','diagnosticar','observar') else
                 'editar solo LabJointRws; sin arranque' if args.action=='actualizar_modulo' else 'solo pcTarget; sin arranque')
        password = getpass.getpass(f'Contraseña UAS ({purpose}, no se guarda): ')
        with requests.Session() as session:
            session.trust_env = False
            session.auth = HTTPDigestAuth(args.user, password)
            client=Client(session,base)
            if args.action not in ('inspeccionar','diagnosticar'):
                args.log_dir.mkdir(parents=True,exist_ok=True)
                stamp=datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%fZ')
                if args.action=='comprobar_recepcion':report=client.reception_test()
                elif args.action=='actualizar_modulo':report=client.update_module(args.log_dir)
                elif args.action=='preparar':report=client.prepare_j6(args.delta)
                elif args.action=='limpiar':report=client.clear()
                else:report=client.observe(json.loads(args.plan.read_text()),args.duracion,
                                           args.log_dir/('muestras_'+stamp+'.jsonl'))
                report['write_events']=client.write_events
                path=args.log_dir/(args.action+'_'+stamp+'.json')
                path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
                print(json.dumps(report,ensure_ascii=False,indent=2))
                print('Informe:',path)
                print('La PC no arrancó RAPID, no activó motores ni cambió el modo.')
                if report.get('program_verification_required'):
                    print('La edición puede restablecer el puntero. Verificar programa en FlexPendant, sin Start. '
                          'Aún NO se ha comprobado la compilación ni se ha preparado un giro.')
                if report.get('target_pending'):print('pcTarget queda PENDIENTE. Para cancelar: rws_j6 limpiar con motores apagados y RAPID detenido.')
                return 0 if report['passed'] else 2
            report = client.inspect()
        path = save_inspection(report, args.log_dir)
        if args.action=='diagnosticar':
            diagnosis=diagnose_inspection(report)
            diagnosis['inspection_path']=str(path.resolve())
            diagnostic_path=path.with_name(path.name.replace('inspeccion_','diagnostico_',1))
            diagnostic_path.write_text(json.dumps(diagnosis,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print_diagnosis(diagnosis)
            print('Informe:',diagnostic_path)
            print('Inspección completa:',path)
            return 0 if diagnosis['local_navigation_prerequisites_observed'] else 2
        for label, entry in report['resources'].items():
            if label == 'module':
                print('Módulo:', report.get('module_source_path', entry.get('error', 'Sin texto en la respuesta')))
            elif label == 'system' and entry['ok']:
                identity = next((r for r in entry['records'] if r.get('_type') == 'sys-system-li'), {})
                print('Controlador:', identity.get('name'), '| RobotWare:', identity.get('rwversionname'))
            elif label == 'modules' and entry['ok']:
                print('Módulos:', ', '.join(r.get('name', '?') for r in entry['records']))
            else:
                print(label + ':', json.dumps(entry, ensure_ascii=False))
        print('Informe:', path)
        print('SOLO LECTURA: no se escribieron variables, motores, puntero ni ejecución RAPID.')
        return 0 if all(r['ok'] for r in report['resources'].values()) else 2
    except (ValueError, requests.RequestException, OSError, TypeError, KeyboardInterrupt, EOFError) as exc:
        print('Prueba interrumpida:', str(exc))
        events=client.write_events if client else []
        target_possible=any(e.get('target') is not None and e['result']!='rejected' for e in events)
        edit_possible=any(e.get('action')=='set-module-text' and e['result']!='rejected' for e in events)
        if client and client.write_events:
            args.log_dir.mkdir(parents=True,exist_ok=True)
            path=args.log_dir/('interrumpido_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%fZ')+'.json')
            path.write_text(json.dumps({'action':args.action,'error':str(exc),'write_events':client.write_events,
                'motion_command_sent':False,'target_may_be_pending':target_possible,
                'module_may_have_changed':edit_possible},ensure_ascii=False,indent=2)+'\n')
            print('Registro:',path)
        if target_possible:
            print('No repetir automáticamente ni pulsar Start. Inspeccionar pcTarget: un POST con timeout '
                  'puede haberse aplicado; el objetivo podría seguir pendiente.')
        if args.action=='actualizar_modulo':
            if edit_possible:
                print('La edición puede haberse aplicado y restablecido el puntero. No pulsar Start. '
                      'Ejecutar inspeccionar y revisar el respaldo; no se restaura automáticamente.')
            else:
                print('No se registró ninguna edición aceptada o incierta del módulo. No se preparó un giro. '
                      'Revisar la causa antes de volver a intentarlo.')
        return 130 if isinstance(exc,(KeyboardInterrupt,EOFError)) else 2


if __name__ == '__main__':
    raise SystemExit(main())
