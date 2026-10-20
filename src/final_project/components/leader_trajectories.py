"""
Simple leader trajectories for the benchmark: hover, straight line, circle, vertical step.

Same interface as TrajectoryHandler (figure-8): the whole trajectory is sampled once,
then update_trajectory() sends one velocity + acceleration setpoint per tick to maneuver.
Every move uses a smooth ramp (velocity profile 3u² − 2u³) whose peak acceleration is `acc`.
"""

import numpy as np


def _time_law(length, speed, acc, period):
    """Arc length s(t), ṡ, s̈ to travel `length` m: smooth ramp up, cruise at `speed`,
    smooth ramp down. Peak acceleration of the ramps = acc. Shorter paths use a lower speed."""
    # the ramp lasts T_r = 1.5·v/a and covers v·T_r/2 metres
    speed = min(speed, np.sqrt(length * acc / 1.5))
    t_ramp = 1.5 * speed / acc
    t_cruise = (length - speed * t_ramp) / speed
    T = 2 * t_ramp + t_cruise
    t = np.arange(int(np.ceil(T / period)) + 1) * period

    s, sd, sdd = np.empty_like(t), np.empty_like(t), np.empty_like(t)
    for i, ti in enumerate(t):
        if ti < t_ramp:                                   # ramp up
            u = ti / t_ramp
            s[i] = speed * t_ramp * (u**3 - 0.5 * u**4)
            sd[i] = speed * (3 * u**2 - 2 * u**3)
            sdd[i] = speed * (6 * u - 6 * u**2) / t_ramp
        elif ti < t_ramp + t_cruise:                      # cruise
            s[i] = 0.5 * speed * t_ramp + speed * (ti - t_ramp)
            sd[i], sdd[i] = speed, 0.0
        else:                                             # ramp down (mirror of the ramp up)
            u = min((T - ti) / t_ramp, 1.0) if ti < T else 0.0
            s[i] = length - speed * t_ramp * (u**3 - 0.5 * u**4)
            sd[i] = speed * (3 * u**2 - 2 * u**3)
            sdd[i] = -speed * (6 * u - 6 * u**2) / t_ramp
    return s, sd, sdd


def _segment(p_a, p_b, speed, acc, period):
    """Straight move from p_a to p_b, starting and ending at rest."""
    p_a, p_b = np.asarray(p_a, float), np.asarray(p_b, float)
    d = p_b - p_a
    u = d / np.linalg.norm(d)
    s, sd, sdd = _time_law(np.linalg.norm(d), speed, acc, period)
    return p_a + np.outer(s, u), np.outer(sd, u), np.outer(sdd, u)


def _hold(p, duration, period):
    """Hover on p for `duration` s."""
    n = int(round(duration / period))
    return np.tile(np.asarray(p, float), (n, 1)), np.zeros((n, 3)), np.zeros((n, 3))


def _circle(p0, radius, laps, speed, acc, period):
    """Circle in the horizontal plane through p0, tangent to +x at p0 (center at p0 + R·ŷ),
    travelled counter-clockwise."""
    s, sd, sdd = _time_law(2 * np.pi * radius * laps, speed, acc, period)
    th = s / radius - np.pi / 2                   # angle of the point seen from the center
    c, sn = np.cos(th), np.sin(th)
    center = np.asarray(p0, float) + np.array([0.0, radius, 0.0])
    pos = center + radius * np.column_stack([c, sn, np.zeros_like(th)])
    tangent = np.column_stack([-sn, c, np.zeros_like(th)])
    normal = -np.column_stack([c, sn, np.zeros_like(th)])     # towards the center
    vel = tangent * sd[:, None]
    acc_v = tangent * sdd[:, None] + normal * (sd**2 / radius)[:, None]
    return pos, vel, acc_v


class SampledTrajectory:
    """Sampled trajectory sent to maneuver one tick at a time (as TrajectoryHandler)."""

    def __init__(self, parts, period):
        self.period = period
        self._pos = np.vstack([p[0] for p in parts]).T     # 3 x N, as in TrajectoryHandler
        self._vel = np.vstack([p[1] for p in parts]).T
        self._acc = np.vstack([p[2] for p in parts]).T
        self.n_steps = self._pos.shape[1] - 1
        self.duration = self.n_steps * period
        self._step = 0
        self._stop_sent = False

    @property
    def is_complete(self):
        return self._step > self.n_steps

    @property
    def peak_speed(self):
        return float(np.linalg.norm(self._vel, axis=0).max())

    @property
    def peak_acc(self):
        return float(np.linalg.norm(self._acc, axis=0).max())

    def update_trajectory(self, target_maneuver):
        """Sends ONE setpoint (velocity + acceleration) and advances by one tick."""
        if self.is_complete:
            if not self._stop_sent:
                target_maneuver.velocity(0, 0, 0, 0, 0, 0, 0, 0)
                self._stop_sent = True
            return False
        v, a = self._vel[:, self._step], self._acc[:, self._step]
        target_maneuver.velocity(v[0], v[1], v[2], 0, a[0], a[1], a[2], 0)
        self._step += 1
        return True


def build_leader_trajectory(kind, p0, period, cfg):
    """SampledTrajectory for kind in ('hover', 'line', 'circle', 'vstep'), starting at rest on p0."""
    p0 = np.asarray(p0, float)
    v, a = cfg['leader_speed'], cfg['leader_traj_acc']
    if kind == 'hover':
        parts = [_hold(p0, cfg['hover_time'], period)]
    elif kind == 'line':
        parts = [_segment(p0, p0 + [cfg['line_length'], 0.0, 0.0], v, a, period)]
    elif kind == 'circle':
        parts = [_circle(p0, cfg['circle_radius'], cfg['circle_laps'], v, a, period)]
    elif kind == 'vstep':
        top = p0 + [0.0, 0.0, cfg['vstep_dz']]
        hold = cfg['vstep_hold']
        parts = [_hold(p0, hold, period), _segment(p0, top, v, a, period),
                 _hold(top, hold, period), _segment(top, p0, v, a, period),
                 _hold(p0, hold, period)]
    else:
        raise ValueError(f"unknown leader trajectory {kind!r}")
    return SampledTrajectory(parts, period)
