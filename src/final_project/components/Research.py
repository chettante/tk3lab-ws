import numpy as np

# ----------------------------------------------------------------------
# Building blocks — same logic as time_law/generate_trajectory in
# otto.py, made generic for any straight segment.
# ----------------------------------------------------------------------

def time_law(t, T):
    """Smoothstep s(t): starts and ends with zero velocity and acceleration."""
    tau = np.clip(t / T, 0.0, 1.0)
    s   = 10*tau**3 - 15*tau**4 + 6*tau**5
    sd  = 30 * tau**2 * (1 - tau)**2 / T
    sdd = 60 * tau * (1 - tau) * (1 - 2*tau) / T**2
    return s, sd, sdd


def _ptp(t, T, p_start, p_end):
    """pos, vel, acc for a smoothstep straight segment between two 3D points.
    If p_start == p_end returns a constant pos and vel = acc = 0 (hover)."""
    s, sd, sdd = time_law(t, T)
    p_start = np.asarray(p_start, dtype=float).reshape((3,) + (1,) * np.ndim(t))
    p_end   = np.asarray(p_end,   dtype=float).reshape((3,) + (1,) * np.ndim(t))
    delta = p_end - p_start
    pos = p_start + delta * s
    vel = delta * sd
    acc = delta * sdd
    return pos, vel, acc


def _wrap(a):
    """Wraps an angle to (-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def _yaw_ptp(t, T, yaw_start, yaw_end):
    """yaw, yaw_rate for a smoothstep rotation along the shortest direction."""
    s, sd, sdd = time_law(t, T)
    dyaw = _wrap(yaw_end - yaw_start)
    yaw = yaw_start + dyaw * s
    yaw_rate = dyaw * sd
    return _wrap(yaw), yaw_rate


def _min_duration(d, v_max, a_max):
    """
    Minimum T to cover d with the smoothstep profile without exceeding
    v_max (velocity peak = 1.875*d/T) or a_max (acceleration peak
    = 5.7735*d/T², i.e. 10/sqrt(3)). Also valid for angles.
    """
    d = abs(d)
    if d < 1e-6:
        return 0.0
    return max(1.875 * d / v_max, np.sqrt(5.7735 * d / a_max))


def _ptp_duration(p_start, p_end, v_max, a_max):
    """Duration of a straight segment between two points (see _min_duration)."""
    d = np.linalg.norm(np.asarray(p_end, dtype=float) - np.asarray(p_start, dtype=float))
    return _min_duration(d, v_max, a_max) if d > 1e-6 else 0.5


# ----------------------------------------------------------------------
# Main class
# ----------------------------------------------------------------------

class LeaderSearchManeuver:
    """
    Generates the follower setpoint when it loses sight of the leader.
    The plan is (re)built with update_leader() when entering SEARCHING,
    then .step() must be called once per cycle: it advances BY ITSELF by one dt.

    Sequence:
      R   recovery: a single smoothstep segment computed on the PREDICTED
          leader position p_pred = p_L + v_L·min(t_lost + T, pred_horizon)
          (straight-line prediction, trusted only for pred_horizon), seen in the
          camera frame (x forward, y left, z up):
            - rotates by Δψ = atan2(y, x)          → leader centered horizontally
            - moves towards p_pred by the part of
              horizontal distance beyond approach_dist
            - goes to the predicted leader altitude → leader centered vertically
          each correction is zero if that FOV constraint is already satisfied
      Rs  one full turn on the spot at omega_scan: looks in every direction within
          the camera range, wherever the leader went (turned, braked, reversed)
      G1  go to the LAST KNOWN leader position (altitude in [z_min, z_max]), G2 hover
      G3  climb by delta_z_global while SPINNING, G4 hover at the top
      G5  descend while SPINNING -> end (done=True)

    yaw0 is the CAMERA heading (drone yaw + mounting): the commanded yaw rate
    is the same for drone and camera, because the offset is constant.

    Main parameters:
      v_max          peak velocity in transfers [m/s]
      v_scan         peak velocity in the vertical scan segments [m/s]
      delta_z_global vertical excursion of the scan [m]
      omega_scan     rotation speed during the scan [rad/s]
      yaw_rate_max   peak yaw rate allowed during recovery [rad/s]
      a_max          peak acceleration in all segments [m/s²]
      yaw_acc_max    peak angular acceleration during recovery [rad/s²]
      t_hold         duration of each hover [s]
      T_rec          minimum recovery duration [s]
      pred_horizon   max time the straight-line prediction is extrapolated [s]
      approach_dist  horizontal distance from the predicted leader to reach [m]
      z_min, z_max   altitude limits during recovery [m]
    """

    def __init__(self, p_follower0, yaw0, p_leader_last, v_leader_last,
                 dt=0.05, v_max=3.0, v_scan=1.0, delta_z_global=10.0,
                 omega_scan=1.0, yaw_rate_max=2.0, t_hold=3.0,
                 a_max=1.5, yaw_acc_max=2.0, T_rec=2.5, approach_dist=3.0, z_min=0.5, z_max=12.0,
                 pred_horizon=2.0, t_lost=0.0):
        self.dt = dt
        self.v_max = v_max
        self.v_scan = v_scan
        self.delta_z_global = delta_z_global
        self.omega_scan = omega_scan
        self.yaw_rate_max = yaw_rate_max
        self.a_max = a_max
        self.yaw_acc_max = yaw_acc_max
        self.t_hold = t_hold
        self.T_rec = T_rec
        self.approach_dist = approach_dist
        self.z_min = z_min
        self.z_max = z_max
        self.pred_horizon = pred_horizon

        self.update_leader(p_leader_last, v_leader_last,
                           p_follower0=p_follower0, yaw0=yaw0, t_lost=t_lost)

    # ------------------------------------------------------------------
    # Flight plan construction (run once per plan)
    # ------------------------------------------------------------------
    def _add(self, name, T, p_end, yaw_mode="const", yaw_end=None):
        """
        Appends a segment that starts where the previous one ended.
        yaw_mode:
          "const" yaw held at the value reached so far
          "turn"  turns (smoothstep) to yaw_end
          "spin"  rotates at constant speed omega_scan
        Keeps track of the final position and yaw, so the next segment
        starts exactly from there (no jump between segments).
        """
        p_end = np.asarray(p_end, dtype=float)
        if yaw_mode == "const":
            yaw_end = self._yaw_build
        elif yaw_mode == "spin":
            yaw_end = _wrap(self._yaw_build + self.omega_scan * T)

        self._segments.append({
            "name": name, "t_start": self._t_build, "T": T,
            "p_start": self._p_build.copy(), "p_end": p_end,
            "yaw_mode": yaw_mode,
            "yaw_start": self._yaw_build, "yaw_end": yaw_end,
        })
        self._t_build += T
        self._t_ends.append(self._t_build)
        self._p_build = p_end
        self._yaw_build = yaw_end

    def _recovery_target(self, T):
        """
        End point and rotation of a recovery lasting T.
        Returns p_pred, p_rec, dyaw.
        """
        p_f0 = self.p_follower0
        p_pred = self.p_leader_last + self.v_leader_last * min(self.t_lost + T, self.pred_horizon)
        r = p_pred - p_f0

        # r in the camera frame (yaw only): x forward, y left
        c, s = np.cos(self.yaw0), np.sin(self.yaw0)
        x_c = c * r[0] + s * r[1]
        y_c = -s * r[0] + c * r[1]

        d = np.linalg.norm(r[:2])
        if d > 0.3:
            dyaw = float(np.arctan2(y_c, x_c))     # predicted leader centered horizontally
            step_xy = max(0.0, d - self.approach_dist) * r[:2] / d
        else:
            dyaw = 0.0                             # leader almost above/below: bearing undefined
            step_xy = np.zeros(2)

        z_rec = float(np.clip(p_pred[2], self.z_min, self.z_max))
        p_rec = np.array([p_f0[0] + step_xy[0], p_f0[1] + step_xy[1], z_rec])
        return p_pred, p_rec, dyaw

    def _recovery_duration(self, p_rec, dyaw):
        """Minimum T that satisfies the linear and yaw velocity and acceleration limits."""
        T_pos = _min_duration(np.linalg.norm(p_rec - self.p_follower0), self.v_max, self.a_max)
        T_yaw = _min_duration(dyaw, self.yaw_rate_max, self.yaw_acc_max)
        return max(self.T_rec, T_pos, T_yaw)

    def _plan_recovery(self):
        """
        Plans the recovery: if the drone limits stretch the maneuver beyond
        T_rec, the leader meanwhile gets farther, so the target is recomputed
        once with the new duration.
        """
        T = self.T_rec
        p_pred, p_rec, dyaw = self._recovery_target(T)
        T_new = self._recovery_duration(p_rec, dyaw)
        if T_new > T:
            T = T_new
            p_pred, p_rec, dyaw = self._recovery_target(T)
            T = self._recovery_duration(p_rec, dyaw)
        return p_pred, p_rec, dyaw, T

    def _build_schedule(self):
        # ---- R: recovery on the predicted leader position ----
        self.p_pred, self.p_rec, self.dyaw_rec, self.T_rec_eff = self._plan_recovery()
        self._add("R_recovery", self.T_rec_eff, self.p_rec, "turn",
                  _wrap(self.yaw0 + self.dyaw_rec))
        self._add("R_scan", 2 * np.pi / self.omega_scan, self.p_rec, "spin")

        # ---- G: last known leader position, hover, climb, hover, descent ----
        p_home = np.array([self.p_leader_last[0], self.p_leader_last[1],
                           np.clip(self.p_leader_last[2], self.z_min, self.z_max)])
        self._add("G1_go_last_seen",
                  _ptp_duration(self.p_rec, p_home, self.v_max, self.a_max), p_home)
        self._add("G2_hold", self.t_hold, p_home)
        p_top = p_home + np.array([0.0, 0.0, self.delta_z_global])
        T_scan = _ptp_duration(p_home, p_top, self.v_scan, self.a_max)
        self._add("G3_scan_up", T_scan, p_top, "spin")
        self._add("G4_hold_top", self.t_hold, p_top)
        self._add("G5_scan_down", T_scan, p_home, "spin")

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    def _segment_index(self, t):
        return int(np.searchsorted(self._t_ends, t, side="left"))

    def phase_name(self, t=None):
        t = self.t if t is None else t
        idx = self._segment_index(t)
        if idx >= len(self._segments):
            return "search_done"
        return self._segments[idx]["name"]

    # ------------------------------------------------------------------
    # Setpoint at time t
    # ------------------------------------------------------------------
    def _evaluate(self, t):
        """pos, vel, acc, yaw, yaw_rate at time t (since the start of the search)."""
        idx = self._segment_index(t)
        if idx >= len(self._segments):
            # search finished: hold at the last point
            return self._p_build.copy(), np.zeros(3), np.zeros(3), self._yaw_build, 0.0

        seg = self._segments[idx]
        tt = t - seg["t_start"]          # time local to the segment
        pos, vel, acc = _ptp(tt, seg["T"], seg["p_start"], seg["p_end"])

        if seg["yaw_mode"] == "turn":
            yaw, yaw_rate = _yaw_ptp(tt, seg["T"], seg["yaw_start"], seg["yaw_end"])
        elif seg["yaw_mode"] == "spin":
            yaw = _wrap(seg["yaw_start"] + self.omega_scan * tt)
            yaw_rate = self.omega_scan
        else:
            yaw, yaw_rate = seg["yaw_start"], 0.0
        return pos, vel, acc, yaw, yaw_rate

    def step(self):
        """
        To be called once per control cycle (every self.dt seconds).
        Advances BY ITSELF through all the segments.
        Returns: pos, vel, acc, yaw, yaw_rate, done.
        After done=True it keeps returning the final state (at rest on the last point).
        """
        pos, vel, acc, yaw, yaw_rate = self._evaluate(self.t)
        self.done = self.t >= self.T_total
        if not self.done:
            self.t += self.dt
        return pos, vel, acc, yaw, yaw_rate, self.done,

    def update_leader(self, p_leader_last, v_leader_last,
                      p_follower0=None, yaw0=None, t_lost=0.0):
        """
        Reconfigures the search with new data from the last sighting and
        rebuilds the flight plan from scratch, WITHOUT recreating the instance.

        p_leader_last, v_leader_last: last known leader position/velocity
        p_follower0, yaw0: follower position and camera heading. If not
                           given, it restarts from where the internal trajectory
                           is NOW (self._evaluate(self.t)).
        t_lost: seconds elapsed since the last sighting of the leader
        """
        if p_follower0 is not None and yaw0 is not None:
            p_start = np.asarray(p_follower0, dtype=float)
            yaw_start = float(yaw0)
        else:
            pos, _, _, yaw, _ = self._evaluate(self.t)
            p_start = pos.copy()
            yaw_start = yaw

        self.p_follower0 = p_start
        self.yaw0 = yaw_start
        self.p_leader_last = np.asarray(p_leader_last, dtype=float)
        self.v_leader_last = np.asarray(v_leader_last, dtype=float)
        self.t_lost = float(t_lost)

        self._segments = []
        self._t_ends = []
        self._t_build = 0.0
        self._p_build = self.p_follower0.copy()
        self._yaw_build = self.yaw0

        self._build_schedule()

        self._t_ends = np.array(self._t_ends)
        self.T_total = self._t_build
        self.t = 0.0
        self.done = False
