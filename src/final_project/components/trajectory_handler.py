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
        # Mantiene lo stesso punto iniziale del generatore usato dallo script
        # di prova. La copia evita di condividere un array mutabile tra istanze.
        self.P0 = P0

        # Risoluzione della griglia usata per approssimare la lunghezza
        # d'arco (vedi _build_arc_length_table). Non è legata a 'period':
        # è solo una questione di quanto finemente vogliamo campionare
        # la GEOMETRIA dell'otto, indipendentemente da come poi lo si
        # percorre nel tempo.
        self.arc_resolution = arc_resolution

        # Tabella di lunghezza d'arco: rende costante la velocita' reale
        # del drone nella fase di crociera anche nelle parti curve
        # dell'otto. Dipende da A, B, H (tramite otto_path), quindi va
        # (ri)costruita ogni volta che questi cambiano: qui la calcoliamo
        # una sola volta in __init__, non ad ogni chiamata di
        # generate_otto_trajectory.
        self._ARC_S, self._ARC_LENGTH, self._PATH_LENGTH = self._build_arc_length_table()

        # Traiettoria completa (pos/vel/acc per OGNI istante da 0 a T),
        # calcolata una sola volta qui: dipende solo da A, B, H, P0,
        # duration, period, che sono fissi per questa istanza.
        # update_trajectory() la scorre un passo alla volta.
        self._pos, self._vel, self._acc = self.generate_otto_trajectory()
        self._step = 0  # indice del prossimo punto da inviare
        self._stop_sent = False

    @property
    def is_complete(self):
        """True quando tutti i campioni della traiettoria sono stati inviati."""
        return self._step > self.n_steps

    def _sample_times(self):
        """Restituisce gli stessi istanti k * period dello script di prova."""
        return np.arange(self.n_steps + 1) * self.period

    def _build_arc_length_table(self):
        """Costruisce la tabella (s -> lunghezza d'arco percorsa) per
        l'otto definito dagli A, B, H correnti di questa istanza.

        Ritorna:
            arc_s      : griglia di valori di s in [0, 1]
            arc_length : lunghezza percorsa cumulata, stessa forma di arc_s
            path_length: lunghezza totale dell'intero otto (scalare)
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
        """Legge temporale: accelerazione liscia, poi velocita' di crociera.

        Il drone parte da fermo, raggiunge la massima velocita' lungo la
        lunghezza d'arco dopo ``ramp_fraction * T`` e la mantiene fino a ``T``.
        Di conseguenza a ``t = T`` la velocita' e' ancora non nulla: chi vuole
        fermarsi esattamente nel punto finale deve aggiungere una decelerazione.
        """
        T = self.duration
        if t is None:
            t = self._sample_times()
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

    def otto_path(self, s):
        """Geometria dell'otto e sue derivate rispetto a s."""
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
        Setpoint dell'otto al tempo t (0 <= t <= T), traslato in modo che
        parta e arrivi in p0. Restituisce pos, vel, acc (ciascuno con 3
        righe: x, y, z).
        """
        T = self.duration
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

        # Spostamento costante che porta il primo punto dell'otto in p0.
        # Non cambia velocità e accelerazione (la derivata di una costante è zero).
        f0, _, _ = self.otto_path(0.0)
        offset = np.asarray(p0, dtype=float) - f0
        offset = offset.reshape((3,) + (1,) * np.ndim(t))   # funziona sia con t scalare sia con array

        pos = f + offset
        vel = df * sd
        acc = ddf * sd**2 + df * sdd
        return pos, vel, acc

    def update_trajectory(self, target_maneuver):
        """Esegue UN passo della traiettoria (un punto x,y,z + comando di
        velocita'/accelerazione) e avanza lo stato interno di un tick.

        Va chiamato una volta per ogni iterazione del loop a tempo reale,
        esattamente come faceva ``generate_trajectory(t, T)`` nel main()
        originale — ma qui il punto viene letto dalla traiettoria
        precalcolata in __init__ invece di essere ricalcolato ogni volta.
        """
        if self.is_complete:
            # Lo script di prova termina qui. Nella state machine il loop
            # prosegue, quindi annulliamo una sola volta l'ultimo comando,
            # che al tempo T ha ancora velocita' di crociera.
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

    def plot_trajectories(self):
        plot_trajectory(self.trajectory_points, dt=self.period, filename="otto.png", show=True)
        plot_trajectory(self.real_positions, dt=self.period, filename="real_pos.png", show=True)