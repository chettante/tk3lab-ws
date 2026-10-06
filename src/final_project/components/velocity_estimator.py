import numpy as np
import time
import logging
from final_project.config import config_path


logger = logging.getLogger("VelocityEstimator")


class VelocityEstimator:
    """Stima la velocità del leader dalla misura della camera via differenziazione numerica"""

    def __init__(self, alpha=1.0, log=True):
        """
        Args:
            alpha: filtro EMA sulla velocità (1.0 = nessun filtro)
            log: se True scrive su CONFIG['velocity_log'] (vedi final_project/config.py)
        """
        self.alpha = alpha
        self.old_pos = None                 # ultima posizione in world (None = nessuna misura)
        self.velocity = np.zeros(3)
        self.last_update_time = time.time()

        self.log_path = config_path('velocity_log') if log else None

    def reset(self):
        """Dimentica la misura precedente: il prossimo update riparte da zero."""
        self.old_pos = None
        self.velocity = np.zeros(3)

    def _log_to_file(self, timestamp: float, velocity: np.ndarray):
        """Appende una riga con timestamp e velocità al file di log"""
        with open(self.log_path, 'a') as f:
            f.write(f"{timestamp:.6f}, {velocity[0]:.6f}, {velocity[1]:.6f}, {velocity[2]:.6f}\n")

    def update(self, pos_in_cam, follower_pos, follower_quat, camera, wasnt_in_fov):
        """
        Aggiorna la stima di velocità.
        Args:
            pos_in_cam: posizione del leader nel frame della camera (camera.measure)
            follower_pos, follower_quat: posa del follower
            camera: FOVPyramid, per riportare la misura in world
            wasnt_in_fov: True se il leader è appena (ri)entrato nel FOV
        Returns:
            (velocità stimata in world [m/s], posizione del leader ricostruita in world)
        """
        # la derivata va fatta in world: il frame camera ruota con il follower
        position = camera.to_world(pos_in_cam, follower_pos, follower_quat)
        return self.update_world(position, reset=wasnt_in_fov), position

    def update_world(self, position, reset=False):
        """
        Aggiorna la stima di velocità da una posizione già in world.
        Args:
            position: posizione in world
            reset: True per ripartire da zero (es. leader appena rientrato nel FOV)
        Returns:
            velocità stimata in world [m/s]
        """
        position = np.asarray(position, dtype=float)
        current_time = time.time()

        # prima misura o reset: riparte da zero
        if reset or self.old_pos is None:
            self.old_pos = position
            self.last_update_time = current_time
            self.velocity = np.zeros(3)
            return self.velocity

        # campione OptiTrack non ancora aggiornato: tieni la stima precedente
        # (altrimenti la velocità alterna 0 e 2v e il feedforward fa scattare il drone)
        if np.allclose(position, self.old_pos, rtol=0.0, atol=1e-9):
            return self.velocity

        dt = current_time - self.last_update_time
        if dt < 0.001:
            logger.debug("dt too small, skipping update")
            return self.velocity
        dt = min(dt, 0.2)

        v = (position - self.old_pos) / dt
        self.velocity = self.alpha * v + (1 - self.alpha) * self.velocity
        self.old_pos = position
        self.last_update_time = current_time

        # Salva velocità e istante di tempo su file prima di ritornare
        if self.log_path is not None:
            self._log_to_file(current_time, self.velocity)
        return self.velocity
