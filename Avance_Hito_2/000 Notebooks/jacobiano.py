"""Paso 6 del Hito 2: Jacobiano geometrico del ABB IRB 120 y analisis de singularidades por rango.

Jacobiano geometrico (Siciliano et al., 2009, sec. 3.1.3, ec. 3.30), juntas de revolucion:

    J_i = [ z_{i-1} x (p - o_{i-1}) ;  z_{i-1} ]        [v; w] = J(q) q̇

z_{i-1} y o_{i-1} son el eje z y el origen del marco DH i-1 (salen de la FK); p es el punto de referencia
(el TCP, o el centro de muneca W para desacoplar brazo y muneca, sec. 3.3.1).

Singularidad = perdida de rango de J (sec. 3.3). El rango se mide con la descomposicion en valores singulares
J = U S V^T (sec. 3.9 y apendice A): rango numerico = cantidad de sigma_i > TOL_RANGO.
"""

import numpy as np

import irb120_modelo as m
import pieper as pp

TOL_RANGO = 1e-6        # sigma_i por debajo de esto cuenta como cero
PASO_FD = 1e-6          # paso de las diferencias finitas centradas [rad]


def jacobiano(q, punto="tcp"):
    """J(q) 6x6 con columnas [z_{i-1} x (p - o_{i-1}); z_{i-1}]. punto = "tcp" o "muneca" (centro W)."""
    T = m.marcos(q)                                   # [T_0^0, T_0^1, ..., T_0^6, T_0^TCP]
    p = T[-1][:3, 3] if punto == "tcp" else T[4][:3, 3]   # W = origen del marco 4 (ejes 4, 5, 6 se cortan ahi)
    J = np.zeros((6, 6))
    for i in range(6):
        z, o = T[i][:3, 2], T[i][:3, 3]
        J[:3, i] = np.cross(z, p - o)
        J[3:, i] = z
    return J


def _vee(S):
    """Vector w de una matriz antisimetrica S = [w]x."""
    return np.array([S[2, 1] - S[1, 2], S[0, 2] - S[2, 0], S[1, 0] - S[0, 1]]) / 2.0


def jacobiano_numerico(q, h=PASO_FD):
    """J por diferencias finitas centradas de la FK, independiente de la formula z x (p - o).

    Columna lineal:  (p(q + h e_i) - p(q - h e_i)) / 2h
    Columna angular: vee( (R(q + h e_i) - R(q - h e_i)) / 2h · R(q)^T ),  porque Ṙ R^T = [w]x (Siciliano, sec. 3.1.1).
    """
    q = np.asarray(q, float)
    R0 = m.fk(q)[:3, :3]
    J = np.zeros((6, 6))
    for i in range(6):
        e = np.zeros(6)
        e[i] = h
        Tp, Tm = m.fk(q + e), m.fk(q - e)
        J[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * h)
        J[3:, i] = _vee((Tp[:3, :3] - Tm[:3, :3]) / (2 * h) @ R0.T)
    return J


def analisis_rango(J, tol=TOL_RANGO):
    """SVD de J: valores singulares, rango numerico, condicion y direcciones mas debiles (u_6, v_6)."""
    U, s, Vt = np.linalg.svd(J)
    rango = int(np.sum(s > tol))
    return dict(
        sigma=s,
        rango=rango,
        singular=rango < 6,
        kappa=s[0] / s[-1] if s[-1] > 0 else np.inf,
        u_debil=U[:, -1],          # twist cartesiano mas dificil (perdido si sigma_6 = 0)
        v_debil=Vt[-1],            # movimiento articular que menos mueve al TCP (nulo si sigma_6 = 0)
        det=float(np.linalg.det(J)),
    )


def radio_muneca(q):
    """r_W: distancia horizontal con signo de W al eje 1 (en la direccion de q1)."""
    pw = m.marcos(q)[4][:3, 3]
    return pw[0] * np.cos(q[0]) + pw[1] * np.sin(q[0])


def det_factorizado(q):
    """det J = det J11 (brazo) · det J22 (muneca), con J referido a W (triangular por bloques, Siciliano 3.3.1).

    det J11 = L2 L3 cos(q3 - delta) r_W     -> codo estirado (q3 = delta - 90°) u hombro (r_W = 0)
    det J22 = -sin q5                        -> muneca (q5 = 0: ejes 4 y 6 colineales)
    """
    q = np.asarray(q, float)
    brazo = pp.L2 * pp.L3 * np.cos(q[2] - pp.DELTA) * radio_muneca(q)
    muneca = -np.sin(q[4])
    return brazo, muneca
