"""Paso 2 del Hito 2: cinematica inversa analitica del ABB IRB 120 por el metodo de Pieper.

El IRB 120 tiene muneca esferica: los ejes 4, 5 y 6 se cortan en un punto W (a4 = a5 = d5 = 0).
Por el teorema de Pieper el problema se desacopla:

1. Posicion (brazo, q1 q2 q3): el centro de muneca W solo depende de q1, q2, q3.
       p_W = p_d - (d6 + TCP) z_d
   - q1 tiene 2 opciones: atan2(y, x) (brazo de frente) o atan2(y, x) + pi (brazo de espaldas).
   - q3 sale de la ley de cosenos en el plano del brazo: 2 opciones (codo arriba / codo abajo,
     segun quede el codo por encima o por debajo de la recta hombro-muneca).
   - q2 sale de la geometria del triangulo hombro-codo-muneca.
2. Orientacion (muneca, q4 q5 q6):
       R_3^6 = (R_0^3)^T R_d = Rz(q4) Ry(-q5) Rz(q6 + offset6)   (offset6 = 180 deg, tabla DH)
   es una descomposicion de Euler ZYZ con 2 opciones (q5 > 0 / q5 < 0).

En total 2 x 2 x 2 = 8 ramas. Todo es cerrado (atan2, sqrt y ley de cosenos): sin solvers numericos.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

import irb120_modelo as m

D1 = m.DH[0, 0]                      # 0.290 m, altura del hombro
L2 = m.DH[1, 1]                      # 0.270 m, brazo
A3 = m.DH[2, 1]                      # 0.070 m, desfase del codo
D4 = m.DH[3, 0]                      # 0.302 m, antebrazo
D6 = m.DH[5, 0]                      # 0.072 m, muneca -> brida
L3 = np.hypot(D4, A3)                # 0.3100 m, codo -> centro de muneca
DELTA = np.arctan2(A3, D4)           # 13.05 deg, inclinacion fija del antebrazo
TOL_SINGULAR = 1e-9                  # |sin q5| por debajo de esto: muneca singular
TOL_POS_M = 1e-6                     # criterio de aceptacion FK: 1 micrometro
TOL_ROT_RAD = 1e-6                   # y 1 microrradian


def envolver(angulo):
    """Lleva un angulo a (-pi, pi]."""
    return (angulo + np.pi) % (2 * np.pi) - np.pi


def centro_muneca(T_d):
    """p_W = p_d - (d6 + TCP) z_d: se retrocede desde el TCP por el eje de la herramienta."""
    return T_d[:3, 3] - (D6 + m.TCP_M) * T_d[:3, 2]


def soluciones_brazo(p_w):
    """Las 4 combinaciones (brazo, codo) -> (q1, q2, q3) o None si W no es alcanzable."""
    x, y, z = p_w
    salida = []
    for brazo, q1, r in (("frente", np.arctan2(y, x), np.hypot(x, y)),
                         ("espaldas", np.arctan2(y, x) + np.pi, -np.hypot(x, y))):
        s = z - D1                                   # altura de W sobre el hombro
        dist2 = r * r + s * s                        # |S W|^2
        k = (L2 * L2 + L3 * L3 - dist2) / (2 * L2 * L3)   # = sin(q3 - delta)  (ley de cosenos)
        if abs(k) > 1.0:                             # W fuera del alcance: ninguna rama de codo es real
            salida += [(brazo, "arriba", None, k), (brazo, "abajo", None, k)]
            continue
        ramas = {}
        for signo in (1.0, -1.0):                    # las dos raices de la ley de cosenos
            q3 = DELTA + np.arctan2(k, signo * np.sqrt(1.0 - k * k))
            phi = q3 - DELTA + np.pi / 2             # angulo codo entre brazo y antebrazo
            q2 = np.arctan2(r, s) - np.arctan2(L3 * np.sin(phi), L2 + L3 * np.cos(phi))
            # Nombre geometrico: el codo E = L2 (sin q2, cos q2) queda por encima o por debajo de la recta S-W.
            e_r, e_s = L2 * np.sin(q2), L2 * np.cos(q2)
            codo = "arriba" if (r * e_s - s * e_r) / r > 0 else "abajo"
            ramas[codo] = (envolver(q1), envolver(q2), envolver(q3))
        for codo in ("arriba", "abajo"):
            salida.append((brazo, codo, ramas[codo], k))
    return salida


def rotacion_0_3(q1, q2, q3):
    T = np.eye(4)
    for q, (d, a, alpha, offset) in zip((q1, q2, q3), m.DH[:3]):
        T = T @ m.dh_matrix(q + offset, d, a, alpha)
    return T[:3, :3]


def soluciones_muneca(R36, q4_ref=0.0):
    """Las 2 soluciones (muneca, q4, q5, q6, singular) de R36 = Rz(q4) Ry(-q5) Rz(q6 + offset6).

    offset6 es el offset de theta de la junta 6 en la tabla DH (180 deg en el marco tool0 de ABB); se lee de m.DH
    en cada llamada para que la IK siga a la FK si se cambia.
    """
    off6 = m.DH[5, 3]
    s_beta = np.hypot(R36[0, 2], R36[1, 2])
    c_beta = R36[2, 2]
    if s_beta < TOL_SINGULAR:
        # q5 = 0 (o 180): z3 y z5 alineados, solo q4 + q6 esta determinado. Se fija q4 = q4_ref.
        suma = np.arctan2(R36[1, 0], R36[0, 0])      # = q4 + q6 + offset6  si c_beta = +1
        q5 = 0.0 if c_beta > 0 else np.pi
        q6 = envolver(suma - q4_ref - off6) if c_beta > 0 else envolver(q4_ref - np.arctan2(-R36[1, 0], -R36[0, 0]) - off6)
        return [("q5+", q4_ref, q5, q6, True), ("q5-", envolver(q4_ref + np.pi), -q5, envolver(q6 + np.pi), True)]
    salida = []
    for muneca, signo in (("q5+", -1.0), ("q5-", 1.0)):     # q5 = -beta
        beta = np.arctan2(signo * s_beta, c_beta)
        alpha = np.arctan2(signo * R36[1, 2], signo * R36[0, 2])
        gamma = np.arctan2(signo * R36[2, 1], -signo * R36[2, 0])
        salida.append((muneca, envolver(alpha), envolver(-beta), envolver(gamma - off6), False))
    return salida


@dataclass
class Candidato:
    """Una de las 8 ramas de Pieper."""

    rama: int
    brazo: str
    codo: str
    muneca: str
    q: np.ndarray | None              # None si la rama no es real
    k_codo: float                     # sin(q3 - delta); |k| > 1 -> fuera de alcance
    muneca_singular: bool = False
    e_pos_m: float = np.nan
    e_rot_rad: float = np.nan
    fuera_de_limites: list = field(default_factory=list)

    @property
    def real(self):
        return self.q is not None

    @property
    def valida_fk(self):
        return self.real and self.e_pos_m < TOL_POS_M and self.e_rot_rad < TOL_ROT_RAD

    @property
    def admisible(self):
        return self.valida_fk and not self.fuera_de_limites


def errores_pose(T_d, T):
    """(error de posicion [m], error de orientacion [rad]) entre dos poses."""
    e_pos = np.linalg.norm(T[:3, 3] - T_d[:3, 3])
    R_rel = T_d[:3, :3].T @ T[:3, :3]
    seno = np.linalg.norm([R_rel[2, 1] - R_rel[1, 2], R_rel[0, 2] - R_rel[2, 0], R_rel[1, 0] - R_rel[0, 1]]) / 2
    e_rot = np.arctan2(seno, (np.trace(R_rel) - 1) / 2)   # angulo de R_d^T R (distancia geodesica)
    return float(e_pos), float(e_rot)


def pieper(T_d, q4_ref=0.0):
    """Enumera SIEMPRE las 8 ramas de Pieper para la pose del TCP T_d y valida cada una por FK."""
    candidatos = []
    for brazo, codo, q123, k in soluciones_brazo(centro_muneca(T_d)):
        if q123 is None:
            for muneca in ("q5+", "q5-"):
                candidatos.append(Candidato(len(candidatos) + 1, brazo, codo, muneca, None, k))
            continue
        R36 = rotacion_0_3(*q123).T @ T_d[:3, :3]
        for muneca, q4, q5, q6, singular in soluciones_muneca(R36, q4_ref):
            q = np.array([*q123, q4, q5, q6])
            c = Candidato(len(candidatos) + 1, brazo, codo, muneca, q, k, singular)
            c.e_pos_m, c.e_rot_rad = errores_pose(T_d, m.fk(q))
            c.fuera_de_limites = [i + 1 for i in range(6)
                                  if not m.LIMITES_RAD[i, 0] - 1e-9 <= q[i] <= m.LIMITES_RAD[i, 1] + 1e-9]
            candidatos.append(c)
    return candidatos
