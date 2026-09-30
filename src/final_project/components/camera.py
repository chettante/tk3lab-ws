import numpy as np
import time
import logging
from typing import Optional
from genomix.event import GenoMError
from final_project.components.velocity_estimator import VelocityEstimator


logger = logging.getLogger("FOVCamera")


def quat_to_rot(q):
    """Matrice di rotazione body→world da quaternione (qw, qx, qy, qz)."""
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


class FOVPyramid:
    """
    FOV di una camera pinhole (come quella di Gazebo): tronco di piramide a base
    rettangolare con apex nella camera. Asse ottico = x del drone ruotato di
    yaw_offset attorno a z (montaggio della camera).
      - semi-aperture orizzontale h e verticale v (asse ottico ↔ facce della piramide)
      - range = profondità lungo l'asse ottico, tra i piani near e far (max_range)
    gimbal=True: camera stabilizzata, la misura ruota solo con lo yaw del follower;
    gimbal=False: camera rigida sul corpo (come in Gazebo), ruota con roll/pitch/yaw.
    """

    def __init__(self, half_angle_h_deg: float = 45.0, half_angle_v_deg: float = 36.87,
                 max_range: float = 10.0, gimbal: bool = False, near: float = 0.1,
                 yaw_offset_deg: float = 0.0):
        """
        Args:
            half_angle_h_deg: semi-apertura orizzontale (gradi, asse ottico ↔ faccia laterale)
            half_angle_v_deg: semi-apertura verticale (gradi, asse ottico ↔ faccia sup./inf.)
            max_range: piano far, profondità massima (metri)
            gimbal: True = solo yaw, False = assetto completo
            near: piano near (metri)
            yaw_offset_deg: yaw di montaggio della camera rispetto all'asse x del drone
        """
        self.max_range = max_range
        self.near = near
        self.gimbal = gimbal
        self.yaw_offset = np.radians(yaw_offset_deg)    # rad, heading camera = yaw drone + offset
        self.tan_h = np.tan(np.radians(half_angle_h_deg))
        self.tan_v = np.tan(np.radians(half_angle_v_deg))

    def contains(self, point_in_body_frame: np.ndarray) -> bool:
        """
        True se il punto è nel FOV.

        Args:
            point_in_body_frame: [x avanti, y sinistra, z su] nel frame della camera
        """
        x, y, z = np.asarray(point_in_body_frame, dtype=float)
        if not (self.near <= x <= self.max_range):
            logger.debug(f"Point out of depth range: x={x:.2f}m (near {self.near}, far {self.max_range})")
            return False
        return abs(y) <= x * self.tan_h and abs(z) <= x * self.tan_v

    def measure(self, leader_pos, follower_pos, follower_quat) -> np.ndarray:
        """
        "Misura della camera": posizione del leader nel frame della camera.
        Con gimbal ruota solo con lo yaw del follower, senza con l'assetto completo.
        """
        rel = np.asarray(leader_pos, dtype=float) - np.asarray(follower_pos, dtype=float)
        R = quat_to_rot(follower_quat)
        if self.gimbal:
            # solo yaw: yaw del drone + yaw di montaggio della camera
            return self.global_to_body(rel, np.zeros(3), np.arctan2(R[1, 0], R[0, 0]) + self.yaw_offset)
        body = R.T @ rel                                # leader negli assi del drone
        # dagli assi del drone agli assi della camera (ruotata di yaw_offset attorno a z)
        return self.global_to_body(body, np.zeros(3), self.yaw_offset)

    def get_yaw_error(self, pos_relative, follower_yaw=0.0):
        """
        Calcola errore di heading: quanto ruotare per centrare il leader
        
        Args:
            pos_relative: [dx, dy, dz] posizione leader nel frame globale
                        (non body frame!)
            follower_yaw: orientamento attuale del follower (radianti)
        
        Returns:
            yaw_error: angolo di rotazione (radianti)
                    positivo = ruota CCW (left)
                    negativo = ruota CW (right)
        """
        # Angolo del leader rispetto al follower
        leader_angle = np.arctan2(pos_relative[1], pos_relative[0])
        
        # Errore = dove dovrebbe guardare - dove guarda
        yaw_error = leader_angle - follower_yaw
        
        # Limita in [-π, π]
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
        
        return yaw_error

    def global_to_body(self, pos_leader: np.ndarray, pos_follower: np.ndarray, follower_yaw: float) -> np.ndarray:
        """Trasforma la posizione relativa dal Frame Globale al Body Frame del follower."""
        pos_rel = pos_leader - pos_follower
        cos_y, sin_y = np.cos(follower_yaw), np.sin(follower_yaw)
        
        R_w2b = np.array([
            [ cos_y,  sin_y, 0.0],
            [-sin_y,  cos_y, 0.0],
            [   0.0,    0.0, 1.0]
        ])
        return R_w2b @ pos_rel


class TrackingController:
    """
    Il follower tiene il leader centrato nel FOV: yaw puntato sul leader (bearing),
    posizione follow_distance dietro al leader lungo il bearing, alla stessa quota.
    """

    def __init__(self, follower, velocity_estimator=None,
                 kp_xy=0.3, kp_z=0.6, follow_distance=1.5, cmd_acc_max=1.0,
                 yaw_priority_start_deg=25.0, yaw_priority_end_deg=45.0,
                 max_velocity=2.5, *, yaw_rate_max):
        self.follower = follower
        # priorità allo yaw: sopra start la correzione xy cala, a end è nulla
        self.yaw_prio = np.radians([yaw_priority_start_deg, yaw_priority_end_deg])
        self.follow_distance = follow_distance   # m, distanza orizzontale dal leader
        self.cmd_acc_max = cmd_acc_max           # m/s², max variazione del comando di velocità
        self.max_velocity = max_velocity         # m/s, limite del drone (come il leader)
        self.velocity_estimator = velocity_estimator or VelocityEstimator()
        self.kp = np.array([kp_xy, kp_xy, kp_z])
        self.yaw_rate_max = yaw_rate_max
        self.t_prev = None
        self.last_cmd = np.zeros(3)

    def reset(self):                    # da chiamare in TrackingState.enter()
        self.t_prev = None; self.last_cmd[:] = 0

    def compute_command(self, leader_pos, follower_pos, follower_yaw,
                        camera, leader_vel, kp_angle=1.0, follower_vel=None):
        now = time.monotonic()
        dt = 0.05 if self.t_prev is None else np.clip(now - self.t_prev, 1e-3, 0.2)
        self.t_prev = now

        leader_pos = np.asarray(leader_pos, dtype=float)
        leader_vel = np.zeros(3) if leader_vel is None else np.asarray(leader_vel)

        # punto desiderato: follow_distance dietro al leader lungo il bearing
        # orizzontale follower→leader (stessa quota del leader)
        rel = leader_pos - follower_pos
        d_xy = np.linalg.norm(rel[:2])
        if d_xy > 1e-3:
            u = rel[:2] / d_xy
        else:                                   # sovrapposti: indietreggia lungo lo yaw
            u = np.array([np.cos(follower_yaw), np.sin(follower_yaw)])
        target = leader_pos - self.follow_distance * np.array([u[0], u[1], 0.0])

        # P + feedforward: v = v_leader + kp·(target − p)
        v_corr = self.kp * (target - follower_pos)

        # priorità allo yaw: se il leader si avvicina al bordo del FOV, prima si ruota
        # per ricentrarlo e solo dopo si corregge la posizione nel piano
        yaw_err = camera.get_yaw_error(rel, follower_yaw)
        lo, hi = self.yaw_prio
        prio = np.clip((hi - abs(yaw_err)) / (hi - lo), 0.0, 1.0)
        v_corr[:2] *= prio

        v_cmd = leader_vel + v_corr             # FF pieno (gain = 1)

        # rate limiter: il comando non può cambiare più di cmd_acc_max*dt per tick
        dv = v_cmd - self.last_cmd
        dv_max = self.cmd_acc_max * dt
        n_dv = np.linalg.norm(dv)
        if n_dv > dv_max:
            v_cmd = self.last_cmd + dv * (dv_max / n_dv)

        # limite di velocità del drone (lo stesso del leader)
        n = np.linalg.norm(v_cmd)
        if n > self.max_velocity:
            v_cmd = v_cmd * (self.max_velocity / n)

        # yaw: leader centrato nel FOV = P sul bearing + feedforward della sua velocità angolare
        # velocità relativa con la velocità MISURATA del follower: l'ultimo comando
        # può differire molto dal moto reale e dare un FF nel verso sbagliato
        v_f = self.last_cmd if follower_vel is None else np.asarray(follower_vel)
        rel_v = leader_vel - v_f
        r2 = max(rel[0]**2 + rel[1]**2, 0.25)
        bearing_rate = (rel[0]*rel_v[1] - rel[1]*rel_v[0]) / r2
        yaw_cmd = np.clip(kp_angle * yaw_err + bearing_rate,
                          -self.yaw_rate_max, self.yaw_rate_max)

        self.last_cmd = v_cmd.copy()
        return v_cmd, yaw_cmd
    
    def send_command(self, velocity_cmd, yaw_cmd: float = 0.0, max_acc: float = 3.0):
        """Invia il comando al follower (già saturato in compute_command)."""
        velocity_cmd = np.asarray(velocity_cmd, dtype=float)
        try:
            self.follower.velocity(
                vx=float(velocity_cmd[0]),
                vy=float(velocity_cmd[1]),
                vz=float(velocity_cmd[2]),
                wz=float(yaw_cmd),
                ax=max_acc, ay=max_acc, az=max_acc,
                duration=0.0,
            )
        except GenoMError as e:
            # es. ::genom::interrupted: un singolo comando perso non deve fermare il loop
            logger.warning(f"follower.velocity fallito: {e}")