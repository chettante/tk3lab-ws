import numpy as np

A, B, H = 5, 5, 5   # semi-lunghezza (y: da -A a +A), larghezza (x), quota massima [m]
P0 = np.array([-1.0, 0.0, 0.025])   # punto di partenza e di arrivo [m]


def time_law(t, T, ramp_fraction=0.15):
    """Legge temporale: accelerazione liscia, poi velocita' di crociera.

    Il drone parte da fermo, raggiunge la massima velocita' lungo la
    lunghezza d'arco dopo ``ramp_fraction * T`` e la mantiene fino a ``T``.
    Di conseguenza a ``t = T`` la velocita' e' ancora non nulla: chi vuole
    fermarsi esattamente nel punto finale deve aggiungere una decelerazione.
    """
    if T <= 0:
        raise ValueError("T deve essere maggiore di zero")
    if not 0 < ramp_fraction < 1:
        raise ValueError("ramp_fraction deve essere compresa tra 0 e 1")

    t = np.clip(t, 0.0, T)
    ramp_time = ramp_fraction * T

    # L'area sotto s_dot deve essere 1: meta' della rampa ha velocita'
    # media pari a meta' della velocita' di crociera.
    cruise_sd = 1.0 / (T - 0.5 * ramp_time)
    u = t / ramp_time
    in_ramp = t < ramp_time

    ramp_s = cruise_sd * ramp_time * (u**3 - 0.5 * u**4)
    ramp_sd = cruise_sd * (3.0 * u**2 - 2.0 * u**3)
    ramp_sdd = cruise_sd * (6.0 * u - 6.0 * u**2) / ramp_time

    cruise_s = 0.5 * cruise_sd * ramp_time + cruise_sd * (t - ramp_time)
    s = np.where(in_ramp, ramp_s, cruise_s)
    sd = np.where(in_ramp, ramp_sd, cruise_sd)
    sdd = np.where(in_ramp, ramp_sdd, 0.0)
    return s, sd, sdd


def path(s):
    """Geometria dell'otto e sue derivate rispetto a s."""
    c1, c2 = 2*np.pi, 4*np.pi
    f = np.array([
        (B/2) * np.sin(c2*s),
        -A * np.cos(c1*s),
        16*H * s**2 * (1 - s)**2,
    ])
    df = np.array([                       # df/ds
        (B/2) * c2 * np.cos(c2*s),
        A * c1 * np.sin(c1*s),
        32*H * s * (1 - s) * (1 - 2*s),
    ])
    ddf = np.array([                      # d²f/ds²
        -(B/2) * c2**2 * np.sin(c2*s),
        A * c1**2 * np.cos(c1*s),
        32*H * (1 - 6*s + 6*s**2),
    ])
    return f, df, ddf

# Tabella della lunghezza d'arco: rende costante la velocita' reale del drone
# nella fase di crociera anche nelle parti curve dell'otto.
_ARC_S = np.linspace(0.0, 1.0, 20_001)
_, _ARC_DF, _ = path(_ARC_S)
_ARC_SEGMENT = np.linalg.norm(_ARC_DF, axis=0)
_ARC_LENGTH = np.concatenate((
    [0.0],
    np.cumsum(0.5 * (_ARC_SEGMENT[1:] + _ARC_SEGMENT[:-1]) * np.diff(_ARC_S)),
))
_PATH_LENGTH = _ARC_LENGTH[-1]


def generate_trajectory(t, T, p0=P0):
    """
    Setpoint dell'otto al tempo t (0 <= t <= T), traslato in modo che
    parta e arrivi in p0. t può essere uno scalare o un array.
    Restituisce pos, vel, acc (ciascuno con 3 righe: x, y, z).
    """
    arc_fraction, arc_fraction_dot, arc_fraction_ddot = time_law(t, T)
    distance = arc_fraction * _PATH_LENGTH
    s = np.interp(distance, _ARC_LENGTH, _ARC_S)
    f, df, ddf = path(s)

    path_speed = np.linalg.norm(df, axis=0)
    path_speed_derivative = np.sum(df * ddf, axis=0) / path_speed
    distance_dot = arc_fraction_dot * _PATH_LENGTH
    distance_ddot = arc_fraction_ddot * _PATH_LENGTH
    sd = distance_dot / path_speed
    sdd = (distance_ddot / path_speed
           - distance_dot**2 * path_speed_derivative / path_speed**3)

    # Spostamento costante che porta il primo punto dell'otto in p0.
    # Non cambia velocità e accelerazione (la derivata di una costante è zero).
    f0, _, _ = path(0.0)
    offset = np.asarray(p0, dtype=float) - f0
    offset = offset.reshape((3,) + (1,) * np.ndim(t))   # funziona sia con t scalare sia con array

    pos = f + offset
    vel = df * sd
    acc = ddf * sd**2 + df * sdd
    return pos, vel, acc
