"""Plantilla de seleccion de la rama de Pieper: por que se descarta o se elige cada una, con un puntaje.

Etapa 1 - Filtros obligatorios (pasa / no pasa). Si una rama falla uno, se DESCARTA y su puntaje es 0:
    F1 real          el centro de muneca esta al alcance del brazo (|k| <= 1)
    F2 FK            T(q) reproduce T_d: e_p < 1 um y e_R < 1 urad
    F3 limites       todas las juntas dentro del rango del datasheet ABB

Etapa 2 - Indices de calidad, cada uno normalizado entre 0 (peor) y 1 (mejor):
    I_lim   margen a los limites: la junta mas cercana a su tope, en fraccion de la mitad de su rango.
            1 = todas en el centro de su rango, 0 = alguna en el tope.
    I_sing  distancia a la singularidad de muneca: |sin q5|. 1 = q5 = +-90 deg, 0 = q5 = 0 (ejes 4 y 6 alineados).
    I_esp   espacio libre: altura minima del codo y del centro de muneca sobre la mesa (z = 0),
            dividida por Z_LIBRE_M y saturada en 1. Evita posturas que rozan la mesa o la base.
    I_mov   movimiento desde q_ref: 1 - ||W (q - q_ref)|| / ||W pi||. 1 = no se mueve, 0 = todas las
            juntas giran 180 deg. W pesa mas el brazo (1-3) que la muneca (4-6) porque mueve mas masa.

Puntaje = 100 * sum(PESOS[i] * I_i)  ->  entre 0 y 100. Se elige la rama admisible de mayor puntaje.
"""

import numpy as np

import irb120_modelo as m
import pieper as pp

PESOS = {"I_lim": 0.25, "I_sing": 0.25, "I_esp": 0.15, "I_mov": 0.35}   # suman 1
W_JUNTAS = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])                      # peso por junta en I_mov
Z_LIBRE_M = 0.20                                                          # altura que se considera "libre"


def representante_cercano(q, q_ref):
    """Para juntas con rango mayor a 360 deg (q6 = +-400 deg) elige q + 2 pi k dentro de limites, el mas cercano a q_ref."""
    q = np.array(q, dtype=float)
    for i, (lo, hi) in enumerate(m.LIMITES_RAD):
        if hi - lo > 2 * np.pi:
            opciones = [q[i] + 2 * np.pi * k for k in range(-2, 3) if lo <= q[i] + 2 * np.pi * k <= hi]
            q[i] = min(opciones, key=lambda v: abs(v - q_ref[i]))
    return q


def indices(q, q_ref):
    lo, hi = m.LIMITES_RAD[:, 0], m.LIMITES_RAD[:, 1]
    margen = np.minimum(q - lo, hi - q) / ((hi - lo) / 2)
    F = m.marcos(q)
    z_min = min(F[2][2, 3], F[4][2, 3])           # codo (marco 2) y centro de muneca (marco 4)
    return {
        "I_lim": float(np.clip(margen.min(), 0, 1)),
        "I_sing": float(abs(np.sin(q[4]))),
        "I_esp": float(np.clip(z_min / Z_LIBRE_M, 0, 1)),
        "I_mov": float(np.clip(1 - np.linalg.norm(W_JUNTAS * (q - q_ref)) / np.linalg.norm(W_JUNTAS * np.pi), 0, 1)),
    }


def razones_descarte(c):
    """Texto con cada filtro que falla y su valor numerico."""
    if not c.real:
        return [f"F1: no existe, |k| = {abs(c.k_codo):.3f} > 1 (centro de muneca fuera de alcance)"]
    razones = []
    if not c.valida_fk:
        razones.append(f"F2: no reproduce T_d (e_p = {c.e_pos_m:.1e} m, e_R = {c.e_rot_rad:.1e} rad)")
    for j in c.fuera_de_limites:
        lo, hi = m.LIMITES_DEG[j - 1]
        razones.append(f"F3: q{j} = {c.q[j - 1]:.4f} rad fuera de [{np.deg2rad(lo):.4f}, {np.deg2rad(hi):.4f}] rad")
    return razones


def evaluar(candidatos, q_ref=None):
    """Evalua las 8 ramas. Devuelve (filas, rama elegida o None)."""
    q_ref = m.Q_I if q_ref is None else np.asarray(q_ref, dtype=float)
    filas = []
    for c in candidatos:
        fila = {"rama": c.rama, "nombre": f"{c.brazo}-{c.codo}-{c.muneca}", "q": None, "indices": None,
                "puntaje": 0.0, "estado": "descartada", "razones": razones_descarte(c)}
        if c.real:
            q = representante_cercano(c.q, q_ref)
            fila["q"] = q
            fila["indices"] = indices(q, q_ref)
            if c.admisible:
                fila["puntaje"] = 100 * sum(PESOS[k] * v for k, v in fila["indices"].items())
                fila["estado"] = "admisible"
        filas.append(fila)
    admisibles = [f for f in filas if f["estado"] == "admisible"]
    elegida = max(admisibles, key=lambda f: f["puntaje"]) if admisibles else None
    if elegida:
        elegida["estado"] = "ELEGIDA"
        for f in admisibles:
            if f is not elegida:
                peor = min(PESOS, key=lambda k: f["indices"][k] - elegida["indices"][k])
                f["razones"] = [f"admisible, pero {f['puntaje']:.1f} < {elegida['puntaje']:.1f} puntos "
                                f"(pierde sobre todo en {peor}: {f['indices'][peor]:.2f} vs {elegida['indices'][peor]:.2f})"]
        elegida["razones"] = ["mayor puntaje entre las admisibles"]
    return filas, elegida


def imprimir(nombre, filas):
    print(f"\n{nombre}")
    cab = f"{'rama':>4} {'configuración':<22} {'F1':>3} {'F2':>3} {'F3':>3}  {'I_lim':>5} {'I_sing':>6} {'I_esp':>5} {'I_mov':>5}  {'puntaje':>7}  {'estado':<10}"
    print(cab); print("-" * len(cab))
    for f in filas:
        r = f["razones"]
        f1 = "no" if any(x.startswith("F1") for x in r) else "sí"
        f2 = "—" if f1 == "no" else ("no" if any(x.startswith("F2") for x in r) else "sí")
        f3 = "—" if f1 == "no" else ("no" if any(x.startswith("F3") for x in r) else "sí")
        ind = "  ".join(f"{f['indices'][k]:>{len(k)}.2f}" for k in PESOS) if f["indices"] else f"{'—':^27}"
        print(f"{f['rama']:>4} {f['nombre']:<22} {f1:>3} {f2:>3} {f3:>3}  {ind}  {f['puntaje']:7.1f}  {f['estado']:<10}")
    print("\nMotivo de cada decisión:")
    for f in filas:
        print(f"  rama {f['rama']}: " + "; ".join(f["razones"]))
