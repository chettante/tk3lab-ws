import numpy as np
from final_project.graphics.trajectory_plotter import plot_trajectory


class TrajectoryHandler:
    def __init__(self, period=0.05, duration=40.0,
                 A=5, B=5, H=5, P0=np.array([-1, 0, 1]),
                 arc_resolution=20_001, position_reader=None,
                 leader_body_name="QR4_leading"):
        self.period = period
        self.duration = duration
        self.n_steps = int(round(duration / period))
        self.trajectory_points = []
        self.real_positions = []
        self.position_reader = position_reader
        self.leader_body_name = leader_body_name
        self.A = A
        self.B = B
        self.H = H
        # Starting point of the figure-8 (the trajectory is translated so it starts here).
        # Note: not copied, the array is shared with the caller.
        self.P0 = P0

        # Resolution of the grid used to approximate the arc length
        # (see _build_arc_length_table). It is not tied to 'period':
        # it only sets how finely the GEOMETRY of the figure-8 is sampled,
        # independently of how it is then travelled in time.
        self.arc_resolution = arc_resolution

        # Arc-length table: keeps the actual drone speed constant during
        # the cruise phase, also on the curved parts of the figure-8.
        # It depends on A, B, H (through otto_path), so it must be
        # (re)built whenever they change: here it is computed once
        # in __init__, not at every call of generate_otto_trajectory.
        self._ARC_S, self._ARC_LENGTH, self._PATH_LENGTH = self._build_arc_length_table()

        # Full trajectory (pos/vel/acc for EVERY instant from 0 to T),
        # computed only once here: it depends only on A, B, H, P0,
        # duration, period, which are fixed for this instance.
        # update_trajectory() walks through it one step at a time.
        self._pos, self._vel, self._acc = self.generate_otto_trajectory()
        self._step = 0  # index of the next point to send
        self._stop_sent = False

    @classmethod
    def from_speed(cls, speed, **kwargs):
        """Creates the figure-8 travelled at cruise speed `speed` [m/s]:
        the duration is derived from the figure-8 length (inverse of cruise_speed)."""
        geometry = {k: kwargs[k] for k in ("A", "B", "H", "P0", "arc_resolution") if k in kwargs}
        probe = cls(duration=1.0, period=1.0, **geometry)    # only for the path length
        duration = probe._PATH_LENGTH / (speed * (1.0 - 0.5 * 0.15))
        return cls(duration=duration, **kwargs)

    @property
    def cruise_speed(self):
        """Cruise speed [m/s] along the figure-8 (see otto_time_law, ramp_fraction=0.15)."""
        return self._PATH_LENGTH / (self.duration * (1.0 - 0.5 * 0.15))

    @property
    def is_complete(self):
        """True when all trajectory samples have been sent."""
        return self._step > self.n_steps

    def _sample_times(self):
        """Returns the sample instants k * period."""
        return np.arange(self.n_steps + 1) * self.period

    def _build_arc_length_table(self):
        """Builds the table (s -> travelled arc length) for the
        figure-8 defined by the current A, B, H of this instance.

        Returns:
            arc_s      : grid of s values in [0, 1]
            arc_length : cumulative travelled length, same shape as arc_s
            path_length: total length of the whole figure-8 (scalar)
        """
        arc_s = np.linspace(0.0, 1.0, self.arc_resolution)
        _, arc_df, _ = self.otto_path(arc_s)
        arc_segment = np.linalg.norm(arc_df, axis=0)
        arc_length = np.concatenate((
            [0.0],
            np.cumsum(0.5 * (arc_segment[1:] + arc_segment[:-1]) * np.diff(arc_s)),
        ))
        path_length = arc_length[-1]
        return arc_s, arc_length, path_length

    def otto_time_law(self, t=None, ramp_fraction=0.15):
        """Time law: smooth acceleration, then cruise speed.

        The drone starts at rest, reaches its maximum speed along the
        arc length after ``ramp_fraction * T`` and keeps it until ``T``.
        Hence at ``t = T`` the speed is still non-zero: to stop exactly
        at the final point a deceleration must be added.
        """
        T = self.duration
        if t is None:
            t = self._sample_times()
        if T <= 0:
            raise ValueError("T must be greater than zero")
        if not 0 < ramp_fraction < 1:
            raise ValueError("ramp_fraction must be between 0 and 1")

        t = np.clip(t, 0.0, T)
        ramp_time = ramp_fraction * T

        # The area under s_dot must be 1: during the ramp the average
        # speed is half of the cruise speed.
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

    def otto_path(self, s):
        """Figure-8 geometry and its derivatives with respect to s."""
        c1, c2 = 2*np.pi, 4*np.pi
        f = np.array([
            (self.B/2) * np.sin(c2*s),
            -self.A * np.cos(c1*s),
            16*self.H * s**2 * (1 - s)**2,
        ])
        df = np.array([                       # df/ds
            (self.B/2) * c2 * np.cos(c2*s),
            self.A * c1 * np.sin(c1*s),
            32*self.H * s * (1 - s) * (1 - 2*s),
        ])
        ddf = np.array([                      # d²f/ds²
            -(self.B/2) * c2**2 * np.sin(c2*s),
            self.A * c1**2 * np.cos(c1*s),
            32*self.H * (1 - 6*s + 6*s**2),
        ])
        return f, df, ddf

    def generate_otto_trajectory(self):
        """
        Figure-8 setpoints at all sample times (0 <= t <= T), translated so
        that it starts and ends in p0. Returns pos, vel, acc (each with 3
        rows: x, y, z).
        """
        t = self._sample_times()
        p0 = self.P0

        arc_fraction, arc_fraction_dot, arc_fraction_ddot = self.otto_time_law(t)
        distance = arc_fraction * self._PATH_LENGTH
        s = np.interp(distance, self._ARC_LENGTH, self._ARC_S)
        f, df, ddf = self.otto_path(s)

        path_speed = np.linalg.norm(df, axis=0)
        path_speed_derivative = np.sum(df * ddf, axis=0) / path_speed
        distance_dot = arc_fraction_dot * self._PATH_LENGTH
        distance_ddot = arc_fraction_ddot * self._PATH_LENGTH
        sd = distance_dot / path_speed
        sdd = (distance_ddot / path_speed
               - distance_dot**2 * path_speed_derivative / path_speed**3)

        # Constant offset that moves the first point of the figure-8 to p0.
        # It does not change velocity and acceleration (the derivative of a constant is zero).
        f0, _, _ = self.otto_path(0.0)
        offset = np.asarray(p0, dtype=float) - f0
        offset = offset.reshape((3,) + (1,) * np.ndim(t))   # works with both scalar and array t

        pos = f + offset
        vel = df * sd
        acc = ddf * sd**2 + df * sdd
        return pos, vel, acc

    def update_trajectory(self, target_maneuver):
        """Executes ONE step of the trajectory (one x,y,z point + velocity/
        acceleration command) and advances the internal state by one tick.

        Must be called once per iteration of the real-time loop. The point
        is read from the trajectory precomputed in __init__ instead of
        being recomputed every time.
        """
        if self.is_complete:
            # The loop keeps running after the figure-8 ends, so the last
            # command (which at time T still has cruise speed) is cancelled once.
            if not self._stop_sent:
                target_maneuver.velocity(0, 0, 0, 0, 0, 0, 0, 0)
                self._stop_sent = True
            return False

        k = self._step
        p = self._pos[:, k]
        v = self._vel[:, k]
        a = self._acc[:, k]

        self.trajectory_points.append((p[0], p[1], p[2]))

        if self.position_reader is not None:
            position, _ = self.position_reader(self.leader_body_name)
            if position is not None:
                self.real_positions.append(tuple(np.asarray(position, dtype=float)))

        target_maneuver.velocity(v[0], v[1], v[2], 0, a[0], a[1], a[2], 0)

        self._step += 1
        return True

    def plot_trajectories(self, planned_path, real_path, show):
        plot_trajectory(self.trajectory_points, dt=self.period, filename=planned_path, show=show)
        plot_trajectory(self.real_positions, dt=self.period, filename=real_path, show=show)