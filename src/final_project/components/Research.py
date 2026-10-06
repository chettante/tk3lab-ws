import numpy as np

# ----------------------------------------------------------------------
# Blocchi di base — stessa logica di time_law/generate_trajectory in
# otto.py, resa generica per un tratto rettilineo qualsiasi.
# ----------------------------------------------------------------------

def time_law(t, T):
    """s(t) smoothstep: parte e arriva con velocità e accelerazione nulle."""
    tau = np.clip(t / T, 0.0, 1.0)
    s   = 10*tau**3 - 15*tau**4 + 6*tau**5
    sd  = 30 * tau**2 * (1 - tau)**2 / T
    sdd = 60 * tau * (1 - tau) * (1 - 2*tau) / T**2
    return s, sd, sdd


def _ptp(t, T, p_start, p_end):
    """pos, vel, acc per un tratto rettilineo smoothstep tra due punti 3D.
    Se p_start == p_end restituisce pos costante e vel = acc = 0 (attesa)."""
    s, sd, sdd = time_law(t, T)
    p_start = np.asarray(p_start, dtype=float).reshape((3,) + (1,) * np.ndim(t))
    p_end   = np.asarray(p_end,   dtype=float).reshape((3,) + (1,) * np.ndim(t))
    delta = p_end - p_start
    pos = p_start + delta * s
    vel = delta * sd
    acc = delta * sdd
    return pos, vel, acc


def _wrap(a):
    """Riporta un angolo in (-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def _yaw_ptp(t, T, yaw_start, yaw_end):
    """yaw, yaw_rate per una rotazione smoothstep sul verso più breve."""
    s, sd, sdd = time_law(t, T)
    dyaw = _wrap(yaw_end - yaw_start)
    yaw = yaw_start + dyaw * s
    yaw_rate = dyaw * sd
    return _wrap(yaw), yaw_rate


def _min_duration(d, v_max, a_max):
    """
    T minimo per percorrere d con il profilo smoothstep senza superare
    v_max (picco di velocità = 1.875*d/T) né a_max (picco di accelerazione
    = 5.7735*d/T², cioè 10/sqrt(3)). Vale anche per gli angoli.
    """
    d = abs(d)
    if d < 1e-6:
        return 0.0
    return max(1.875 * d / v_max, np.sqrt(5.7735 * d / a_max))


def _ptp_duration(p_start, p_end, v_max, a_max):
    """Durata di un tratto rettilineo tra due punti (vedi _min_duration)."""
    d = np.linalg.norm(np.asarray(p_end, dtype=float) - np.asarray(p_start, dtype=float))
    return _min_duration(d, v_max, a_max) if d > 1e-6 else 0.5


# ----------------------------------------------------------------------
# Classe principale
# ----------------------------------------------------------------------

class LeaderSearchManeuver:
    """
    Genera il setpoint del follower quando perde di vista il leader.
    Il piano si (ri)costruisce con update_leader() all'ingresso in SEARCHING,
    poi .step() va chiamata una volta per ciclo: avanza DA SOLA di un dt.

    Sequenza:
      R   recupero: un solo tratto smoothstep calcolato sulla posizione
          PREVISTA del leader p_pred = p_L + v_L·(t_lost + T), vista nel
          frame camera (x avanti, y sinistra, z su):
            - ruota di Δψ = atan2(y, x)            → leader al centro in orizzontale
            - avanza verso p_pred della parte di
              distanza orizzontale oltre approach_dist
            - va alla quota prevista del leader    → leader al centro in verticale
          ogni correzione è nulla se quel vincolo del FOV è già rispettato
      Ra  fermo t_hold secondi a guardare
      G1  vai in p_home, G2 attesa
      G3  sali di delta_z_global RUOTANDO, G4 attesa in cima
      G5  scendi fino a p_home RUOTANDO -> fine (done=True)

    yaw0 è lo heading della CAMERA (yaw del drone + montaggio): lo yaw rate
    comandato è lo stesso per drone e camera, perché l'offset è costante.

    Parametri principali:
      v_max          velocità di picco nei trasferimenti [m/s]
      v_scan         velocità di picco nei tratti verticali di scansione [m/s]
      delta_z_global ampiezza dell'escursione verticale della scansione [m]
      omega_scan     velocità di rotazione durante la scansione [rad/s]
      yaw_rate_max   yaw rate di picco ammesso nel recupero [rad/s]
      a_max          accelerazione di picco in tutti i tratti [m/s²]
      yaw_acc_max    accelerazione angolare di picco nel recupero [rad/s²]
      t_hold         durata di ogni attesa da fermo [s]
      T_rec          durata minima del recupero e orizzonte di previsione [s]
      approach_dist  distanza orizzontale dal leader previsto a cui portarsi [m]
      z_min, z_max   limiti di quota durante il recupero [m]
      p_home         punto da cui parte la scansione globale
    """

    def __init__(self, p_follower0, yaw0, p_leader_last, v_leader_last,
                 dt=0.05, v_max=3.0, v_scan=1.0, delta_z_global=10.0,
                 omega_scan=1.0, yaw_rate_max=2.0, t_hold=3.0,
                 a_max=1.5, yaw_acc_max=2.0, T_rec=2.5, approach_dist=3.0, z_min=0.5, z_max=12.0,
                 p_home=(0.0, 0.0, 1.0), t_lost=0.0):
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
        self.p_home = np.asarray(p_home, dtype=float)

        self.update_leader(p_leader_last, v_leader_last,
                           p_follower0=p_follower0, yaw0=yaw0, t_lost=t_lost)

    # ------------------------------------------------------------------
    # Costruzione del piano di volo (eseguita una volta sola)
    # ------------------------------------------------------------------
    def _add(self, name, T, p_end, yaw_mode="const", yaw_end=None):
        """
        Aggiunge un segmento che parte da dove è finito il precedente.
        yaw_mode:
          "const" yaw fermo al valore raggiunto finora
          "turn"  gira (smoothstep) fino a yaw_end
          "spin"  ruota a velocità costante omega_scan
        Tiene traccia di posizione e yaw finali, così il segmento dopo
        parte esattamente da lì (nessun salto tra un tratto e l'altro).
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
        Punto e rotazione di fine recupero per un recupero di durata T.
        Ritorna p_pred, p_rec, dyaw.
        """
        p_f0 = self.p_follower0
        p_pred = self.p_leader_last + self.v_leader_last * (self.t_lost + T)
        r = p_pred - p_f0

        # r nel frame camera (solo yaw): x avanti, y sinistra
        c, s = np.cos(self.yaw0), np.sin(self.yaw0)
        x_c = c * r[0] + s * r[1]
        y_c = -s * r[0] + c * r[1]

        d = np.linalg.norm(r[:2])
        if d > 0.3:
            dyaw = float(np.arctan2(y_c, x_c))     # leader previsto al centro in orizzontale
            step_xy = max(0.0, d - self.approach_dist) * r[:2] / d
        else:
            dyaw = 0.0                             # leader quasi sopra/sotto: bearing indefinito
            step_xy = np.zeros(2)

        z_rec = float(np.clip(p_pred[2], self.z_min, self.z_max))
        p_rec = np.array([p_f0[0] + step_xy[0], p_f0[1] + step_xy[1], z_rec])
        return p_pred, p_rec, dyaw

    def _recovery_duration(self, p_rec, dyaw):
        """T minimo che rispetta i limiti di velocità e accelerazione, lineari e di yaw."""
        T_pos = _min_duration(np.linalg.norm(p_rec - self.p_follower0), self.v_max, self.a_max)
        T_yaw = _min_duration(dyaw, self.yaw_rate_max, self.yaw_acc_max)
        return max(self.T_rec, T_pos, T_yaw)

    def _plan_recovery(self):
        """
        Pianifica il recupero: se i limiti del drone allungano la manovra oltre
        T_rec, il leader nel frattempo va più lontano, quindi si ricalcola
        una volta il bersaglio con la nuova durata.
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
        # ---- R: recupero sulla posizione prevista del leader ----
        self.p_pred, self.p_rec, self.dyaw_rec, self.T_rec_eff = self._plan_recovery()
        self._add("R_recupero", self.T_rec_eff, self.p_rec, "turn",
                  _wrap(self.yaw0 + self.dyaw_rec))
        self._add("R_attesa", self.t_hold, self.p_rec)

        # ---- G: home, attesa, salita, attesa, discesa ----
        p_home = self.p_home
        self._add("G1_vai_home",
                  _ptp_duration(self.p_rec, p_home, self.v_max, self.a_max), p_home)
        self._add("G2_attesa_home", self.t_hold, p_home)
        p_top = p_home + np.array([0.0, 0.0, self.delta_z_global])
        T_scan = _ptp_duration(p_home, p_top, self.v_scan, self.a_max)
        self._add("G3_salita_scan", T_scan, p_top, "spin")
        self._add("G4_attesa_in_cima", self.t_hold, p_top)
        self._add("G5_discesa_scan", T_scan, p_home, "spin")

    # ------------------------------------------------------------------
    # Utilità
    # ------------------------------------------------------------------
    def _segment_index(self, t):
        return int(np.searchsorted(self._t_ends, t, side="left"))

    def phase_name(self, t=None):
        t = self.t if t is None else t
        idx = self._segment_index(t)
        if idx >= len(self._segments):
            return "fine_ricerca"
        return self._segments[idx]["name"]

    def print_schedule(self):
        print(f"{'segmento':24s} {'t_inizio':>8s} {'durata':>7s}  {'yaw':>5s}   da -> a")
        for seg in self._segments:
            print(f"{seg['name']:24s} {seg['t_start']:8.2f} {seg['T']:7.2f}  "
                  f"{seg['yaw_mode']:>5s}   "
                  f"{np.round(seg['p_start'], 2)} -> {np.round(seg['p_end'], 2)}")
        print(f"{'TOTALE':24s} {'':8s} {self.T_total:7.2f} s")

    # ------------------------------------------------------------------
    # Setpoint al tempo t
    # ------------------------------------------------------------------
    def _evaluate(self, t):
        """pos, vel, acc, yaw, yaw_rate al tempo t (dall'inizio ricerca)."""
        idx = self._segment_index(t)
        if idx >= len(self._segments):
            # ricerca finita: fermo nell'ultimo punto
            return self._p_build.copy(), np.zeros(3), np.zeros(3), self._yaw_build, 0.0

        seg = self._segments[idx]
        tt = t - seg["t_start"]          # tempo locale al segmento
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
        Da chiamare una volta per ciclo di controllo (ogni self.dt secondi).
        Avanza DA SOLA attraverso tutti i segmenti.
        Ritorna: pos, vel, acc, yaw, yaw_rate, done.
        Dopo done=True continua a restituire lo stato finale (fermo in 0,0,0).
        """
        pos, vel, acc, yaw, yaw_rate = self._evaluate(self.t)
        self.done = self.t >= self.T_total
        if not self.done:
            self.t += self.dt
        return pos, vel, acc, yaw, yaw_rate, self.done, 

    def update_leader(self, p_leader_last, v_leader_last,
                      p_follower0=None, yaw0=None, t_lost=0.0):
        """
        Riconfigura la ricerca con nuovi dati dell'ultimo avvistamento e
        rigenera il piano di volo da zero, SENZA ricreare l'istanza.

        p_leader_last, v_leader_last: ultima posizione/velocità note del leader
        p_follower0, yaw0: posizione del follower e heading della camera. Se non
                           passati, si riparte da dove si trova ORA la
                           traiettoria interna (self._evaluate(self.t)).
        t_lost: secondi trascorsi dall'ultimo avvistamento del leader
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
