"""Diagnóstico de ejecución y vigilancia de avance; no envía órdenes."""
import math


class ExecutionFault(ValueError):
    def __init__(self, message, **details):
        super().__init__(message)
        self.details = dict(message=message, **details)


class ProgressWatchdog:
    def __init__(self, timeout_s=12., movement_deg=.002):
        self.timeout = float(timeout_s)
        self.movement = float(movement_deg)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('Tiempo de vigilancia inválido.')
        self.point = self.completed = self.best = self.last = None

    def observe(self, now, point, completed, error_deg):
        if not math.isfinite(error_deg):
            raise ValueError('Error de postura no finito.')
        changed = point != self.point or completed != self.completed
        if self.last is None or changed or error_deg <= self.best-self.movement:
            self.last, self.best = now, error_deg
            self.point, self.completed = point, completed
        return now-self.last >= self.timeout


def failure_message(code, point=None, axis=0, delta=0., rapid_error=0):
    reasons = {-1: 'Orden cancelada', -2: 'Punto fuera de límites o ejes externos no admitidos',
               -3: 'La postura medida no coincide con el inicio del tramo',
               -4: 'El ABB dejó de estar en modo automático', -5: 'Altura mínima de un punto',
               -6: 'Ejes externos no admitidos', -7: 'Error de ejecución RAPID',
               -8: 'Cantidad de puntos inválida', -9: 'No se confirmó una llegada estable al punto'}
    message = reasons.get(code, 'El ABB rechazó la orden')+f' (código {code})'
    if point is not None:
        message += f' · hacia P{point}'
    if axis:
        message += f' · J{axis}: diferencia {delta:.6f}°'
    if rapid_error:
        message += f' · error RAPID {rapid_error}'
    return message
