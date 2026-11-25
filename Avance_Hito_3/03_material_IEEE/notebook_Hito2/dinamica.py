"""Paso 7 del Hito 2: dinamica reducida 1R evaluada sobre la trayectoria LSPB.

Modelo de clase (IMT-342, W09-S02, diapositivas 4 y 5; Guia oficial Hito 2, sec. 3.4):

    tau = J q̈ + m g lc sin(q)        q medido desde la vertical hacia abajo,  J = Ic + m lc^2 (Steiner)

En 1R la matriz de Coriolis es nula (W09-S02, diapositiva 6), asi que tau = inercial + gravitacional.

Aplicacion al IRB 120: la junta 3 (codo) moviendo el antebrazo (codo -> centro de muneca W). Los ejes 2 y 3 son
paralelos y horizontales, asi que el angulo absoluto del antebrazo respecto a la vertical es q2 + q3 + constante:

    theta = q2 + q3 - delta - pi/2        (theta = 0: antebrazo colgando hacia abajo; positivo en el sentido de q3)

ABB no publica masas ni inercias por eslabon (W09-S01, handoff de dinamica: no derivar los 6 GDL sin esos datos).
Por eso la masa es la del modelo ilustrativo de clase (m = 1 kg, W09-S02 torque_python) y el antebrazo se modela
como barra uniforme; la geometria si es la del robot. tau es lineal en m: para otra masa se multiplica.
"""

import numpy as np

import pieper as pp

G = 9.81                             # m/s^2
M_KG = 1.0                           # masa ilustrativa del modelo 1R de clase (W09-S02, torque_python)
L_M = pp.L3                          # 0.310 m: codo -> centro de muneca (tabla DH, Bahani et al. 2023)
LC_M = L_M / 2                       # supuesto: barra uniforme, centro de masa a la mitad
IC = M_KG * L_M**2 / 12              # inercia de una barra uniforme respecto a su centro de masa
J_KGM2 = IC + M_KG * LC_M**2         # Steiner (W09-S02, diap. 4): J = Ic + m lc^2 = m L^2 / 3


def tau_1r(q, qdd, J=J_KGM2, m=M_KG, lc=LC_M, g=G):
    """(tau_inercial, tau_gravedad, tau_total) del modelo 1R: tau = J q̈ + m g lc sin q."""
    q, qdd = np.asarray(q, float), np.asarray(qdd, float)
    t_in = J * qdd
    t_g = m * g * lc * np.sin(q)
    return t_in, t_g, t_in + t_g


def angulo_antebrazo(q):
    """theta del antebrazo desde la vertical hacia abajo, positivo en el sentido de q3 (q puede ser (..., 6))."""
    q = np.asarray(q, float)
    return q[..., 1] + q[..., 2] - pp.DELTA - np.pi / 2


def tau_gravedad_exacta(q, m=M_KG, lc=LC_M, g=G):
    """Momento del peso del antebrazo respecto al eje del codo z2, calculado con la FK 3D (validacion del 1R).

    tau = z2 · ((p_c - o2) x (0, 0, m g)),  con p_c a lc del eje del codo sobre la recta codo -> W.
    """
    import irb120_modelo as m_
    T = m_.marcos(q)
    o2, z2, w = T[2][:3, 3], T[2][:3, 2], T[4][:3, 3]
    p_c = o2 + lc * (w - o2) / np.linalg.norm(w - o2)
    return float(z2 @ np.cross(p_c - o2, np.array([0.0, 0.0, m * g])))
