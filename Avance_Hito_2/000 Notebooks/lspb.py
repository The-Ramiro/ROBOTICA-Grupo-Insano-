"""Paso 5 del Hito 2: trayectoria LSPB (Linear Segment with Parabolic Blends) en espacio articular.

Ecuaciones y notacion de clase (IMT-342, W09-S01, diapositivas 8 y 9; hoja de trabajo LSPB W09-S01):

    Aceleracion       0 <= t <= tc            q = qi + q̈c t^2 / 2         q̇ = q̈c t          q̈ = q̈c
    Vel. constante    tc < t <= tf - tc       q = qi + q̇c (t - tc / 2)     q̇ = q̇c = q̈c tc    q̈ = 0
    Desaceleracion    tf - tc < t <= tf       q = qf - q̈c (tf - t)^2 / 2   q̇ = q̈c (tf - t)   q̈ = -q̈c

    tc = tf / 2 - (1/2) sqrt(tf^2 - 4 (qf - qi) / q̈c)        tiempo de mezcla
    |q̈c| >= 4 |qf - qi| / tf^2                                 aceleracion minima (factibilidad)

La clase trata una junta. Para mover las 6 juntas a la vez (que todas lleguen juntas a q_f) se usa la extension
estandar de Siciliano et al. (2009, sec. 4.2.1): mismo tf y mismo tc para todas; cada junta ajusta su q̈c.
"""

import numpy as np

import irb120_modelo as m

# --------------------------------------------------------------------------- supuestos de limites
# vmax: datasheet ABB por eje, al 25 % (override de velocidad conservador para pruebas de laboratorio).
# amax: ABB NO publica aceleraciones articulares. Supuesto declarado: cada eje llega a su vmax en T_RAMPA_S.
ESCALA_VEL = 0.25
T_RAMPA_S = 0.25
VMAX = ESCALA_VEL * m.VMAX_RAD_S               # rad/s
AMAX = VMAX / T_RAMPA_S                        # rad/s^2
PASO_TF_S = 0.01                               # tf se redondea hacia arriba a centesimas de segundo


# --------------------------------------------------------------------------- formulas de clase (una junta)
def aceleracion_minima(qi, qf, tf):
    """|q̈c|_min = 4 |qf - qi| / tf^2."""
    return 4.0 * np.abs(np.asarray(qf) - np.asarray(qi)) / tf**2


def tiempo_mezcla(qi, qf, tf, qdd_c):
    """tc = tf/2 - 1/2 sqrt(tf^2 - 4 (qf - qi) / q̈c)  (q̈c con el mismo signo que qf - qi)."""
    D = qf - qi
    if D == 0:
        return 0.0
    radicando = tf**2 - 4.0 * D / qdd_c
    if radicando < -1e-12:
        raise ValueError("q̈c menor que la aceleracion minima: no existe perfil LSPB con ese tf.")
    return tf / 2 - 0.5 * np.sqrt(max(radicando, 0.0))


def tiempo_minimo(D, vmax, amax):
    """Menor tf de una junta que se mueve |D| sin pasar vmax ni amax.

    Si |D| >= vmax^2 / amax el perfil es trapezoidal (llega a vmax): tf = |D|/vmax + vmax/amax.
    Si no, es triangular (no llega a vmax):                          tf = 2 sqrt(|D| / amax).
    """
    D = abs(D)
    if D == 0:
        return 0.0
    return D / vmax + vmax / amax if D >= vmax**2 / amax else 2.0 * np.sqrt(D / amax)


def _tf_elegido(tf, tf_min, paso):
    """t_f por defecto (tf_min redondeado hacia arriba a `paso`) o el dado, que debe ser >= tf_min."""
    if tf is None:
        return float(np.ceil(tf_min / paso - 1e-9) * paso)
    if tf < tf_min - 1e-12:
        raise ValueError(f"t_f = {tf:.4f} s es menor que el mínimo {tf_min:.4f} s con estos v_max, a_max y t_c. "
                         f"Use t_f >= {tf_min:.4f} s, o None para el mínimo.")
    return float(tf)


# --------------------------------------------------------------------------- planificacion sincronizada
class LSPB:
    """Perfil LSPB sincronizado de q_i a q_f: mismo tf y tc para las 6 juntas."""

    def __init__(self, q_i, q_f, vmax=VMAX, amax=AMAX, tf=None, tc=None):
        self.q_i = np.asarray(q_i, dtype=float)
        self.q_f = np.asarray(q_f, dtype=float)
        self.D = self.q_f - self.q_i
        self.vmax, self.amax = np.asarray(vmax, float), np.asarray(amax, float)
        self.tf_min = np.array([tiempo_minimo(d, v, a) for d, v, a in zip(self.D, self.vmax, self.amax)])
        self.critica = int(np.argmax(self.tf_min))                          # junta que fija el tiempo
        k = self.critica
        if tc is None:
            # t_c por la fórmula de clase con la aceleración máxima de la junta crítica
            tf_min = self.tf_min.max()
            self.tf = _tf_elegido(tf, tf_min, PASO_TF_S)
            self.tc = tiempo_mezcla(0.0, abs(self.D[k]), self.tf, self.amax[k])
        else:
            # t_c dado: el t_f mínimo que cumple v_max y a_max en todas las juntas es
            # max_j (t_c + max(|D_j| / v_max_j, |D_j| / (t_c a_max_j))), y además t_f >= 2 t_c
            self.tc = float(tc)
            tf_min = max(self.tc + np.max(np.maximum(np.abs(self.D) / self.vmax,
                                                    np.abs(self.D) / (self.tc * self.amax))), 2 * self.tc)
            self.tf = _tf_elegido(tf, tf_min, PASO_TF_S)
        self.qd_c = self.D / (self.tf - self.tc)                            # q̇c [rad/s]
        self.qdd_c = self.qd_c / self.tc                                    # q̈c [rad/s^2]
        if np.any(np.abs(self.qd_c) > self.vmax + 1e-9) or np.any(np.abs(self.qdd_c) > self.amax + 1e-9):
            raise ValueError("el perfil LSPB viola v_max o a_max en alguna junta.")

    def fase(self, t):
        return "aceleración" if t <= self.tc else ("vel. constante" if t <= self.tf - self.tc else "desaceleración")

    def evaluar(self, t):
        """q, q̇, q̈ (vectores de 6) en el instante t, con las ecuaciones de clase por tramos."""
        t = float(np.clip(t, 0.0, self.tf))
        tc, tf, a, v = self.tc, self.tf, self.qdd_c, self.qd_c
        if t <= tc:
            return self.q_i + 0.5 * a * t**2, a * t, a.copy()
        if t <= tf - tc:
            return self.q_i + v * (t - tc / 2), v.copy(), np.zeros(6)
        return self.q_f - 0.5 * a * (tf - t) ** 2, a * (tf - t), -a

    def muestrear(self, n=1001):
        t = np.linspace(0.0, self.tf, n)
        q, qd, qdd = (np.array(x) for x in zip(*(self.evaluar(ti) for ti in t)))
        return t, q, qd, qdd
