"""Paso 1 del Hito 2: modelo geometrico del ABB IRB 120, pose inicial q_i y poses objetivo T_d.

Todo esta en unidades SI: longitudes en m, angulos en rad (en grados solo para mostrarlos).

Fuentes de los datos (la guia prohibe inventarlos):
- Tabla DH clasica: Bahani et al. (2023), Tabla I; coincide con las cotas del datasheet ABB.
- Limites y velocidades por eje: ABB, Product specification IRB 120 (3HAC035960).
- TCP: punto sobre el eje z6 de la brida (tool0), a 0.150 m de ella.
"""

import numpy as np

# --------------------------------------------------------------------------- tabla DH
# Convencion DH clasica: A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i), con theta_i = q_i + offset_i.
# Columnas: d [m], a [m], alpha [rad], offset de theta [rad].
DH = np.array(
    [
        [0.290, 0.000, -np.pi / 2, 0.0],         # 1: base -> hombro
        [0.000, 0.270, 0.0, -np.pi / 2],         # 2: brazo (offset -90 deg: q2 = 0 deja el brazo vertical)
        [0.000, 0.070, -np.pi / 2, 0.0],         # 3: codo
        [0.302, 0.000, np.pi / 2, 0.0],          # 4: antebrazo hasta el centro de muneca
        [0.000, 0.000, -np.pi / 2, 0.0],         # 5: muneca
        [0.072, 0.000, 0.0, np.pi],              # 6: brida (tool0); offset 180 deg para el marco ABB
    ]
)

TCP_M = 0.150  # distancia brida -> TCP sobre z6

# Limites articulares [deg] y velocidades maximas [deg/s] del datasheet ABB.
LIMITES_DEG = np.array([[-165, 165], [-110, 110], [-110, 70], [-160, 160], [-120, 120], [-400, 400]], dtype=float)
LIMITES_RAD = np.deg2rad(LIMITES_DEG)
VMAX_DEG_S = np.array([250, 250, 250, 320, 320, 420], dtype=float)
VMAX_RAD_S = np.deg2rad(VMAX_DEG_S)

# --------------------------------------------------------------------------- configuracion inicial
# q_i: posicion de calibracion ABB (todos los ejes en 0 deg), la unica postura de referencia del fabricante.
Q_I = np.array([0.0, 0.0, -np.pi / 2, 0.0, 0.0, 0.0])

# --------------------------------------------------------------------------- poses objetivo
# Cada pose es (x, y, z [m], roll, pitch, yaw [deg]) del TCP en el marco base, con R = Rz(yaw) Ry(pitch) Rx(roll).
# EJEMPLO: reemplazar por las poses que entregue el docente. De cada una saldra q_f con Pieper (paso 2).
POSES_OBJETIVO = {
    "T_d1": (0.25, -0.10, 0.12, 180.0, 0.0, 0.0),  # herramienta apuntando verticalmente hacia la mesa (-Z0)
}


# --------------------------------------------------------------------------- funciones
def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def dh_matrix(theta, d, a, alpha):
    """Matriz homogenea A_i de la convencion DH clasica."""
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def marcos(q):
    """Lista [T_0^0, T_0^1, ..., T_0^6, T_0^TCP] para la configuracion articular q [rad]."""
    q = np.asarray(q, dtype=float)
    if q.shape != (6,):
        raise ValueError("q debe tener 6 angulos articulares.")
    T = np.eye(4)
    salida = [T.copy()]
    for qi, (d, a, alpha, offset) in zip(q, DH):
        T = T @ dh_matrix(qi + offset, d, a, alpha)
        salida.append(T.copy())
    T_tcp = T.copy()
    T_tcp[:3, 3] += TCP_M * T[:3, 2]  # traslacion pura sobre z6
    salida.append(T_tcp)
    return salida


def fk(q):
    """Cinematica directa al TCP: T_0^TCP(q)."""
    return marcos(q)[-1]


def pose_xyz_rpy(x, y, z, roll_deg, pitch_deg, yaw_deg):
    """Matriz homogenea 4x4 desde posicion [m] y roll-pitch-yaw [deg]."""
    T = np.eye(4)
    T[:3, :3] = rot_z(np.deg2rad(yaw_deg)) @ rot_y(np.deg2rad(pitch_deg)) @ rot_x(np.deg2rad(roll_deg))
    T[:3, 3] = (x, y, z)
    return T


def rpy_deg(R):
    """(roll, pitch, yaw) [deg] de una matriz de rotacion R = Rz Ry Rx."""
    pitch = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.rad2deg([roll, pitch, yaw])


def es_rotacion(R, tol=1e-9):
    """R es una rotacion valida: ortonormal y det = +1."""
    return np.allclose(R.T @ R, np.eye(3), atol=tol) and abs(np.linalg.det(R) - 1.0) < tol


def dentro_de_limites(q):
    q = np.asarray(q, dtype=float)
    return bool(np.all(q >= LIMITES_RAD[:, 0]) and np.all(q <= LIMITES_RAD[:, 1]))
