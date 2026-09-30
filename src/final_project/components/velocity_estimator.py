import numpy as np
import time
import logging
from final_project.config import config_path


logger = logging.getLogger("VelocityEstimator")


class VelocityEstimator:
    """Stima la velocità del leader via differenziazione numerica"""

    def __init__(self):
        """Il file di log è CONFIG['velocity_log'] (vedi final_project/config.py)."""
        self.old_pos = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.last_update_time = time.time()

        self.log_path = config_path('velocity_log')

    def _log_to_file(self, timestamp: float, velocity: np.ndarray):
        """Appende una riga con timestamp e velocità al file di log"""
        try:
            with open(self.log_path, "a") as f:
                f.write(f"{timestamp:.6f}, {velocity[0]:.6f}, {velocity[1]:.6f}, {velocity[2]:.6f}\n")
        except IOError as e:
            logger.warning(f"Impossibile scrivere sul file di log: {e}")

    def update(self, position: np.ndarray, wasnt_in_fov) -> np.ndarray:
        """
        Aggiorna la stima di velocità.
        Args:
            position: posizione attuale [x, y, z]
        Returns:
            Velocità stimata (m/s)
        """
        position = np.asarray(position, dtype=float)
        current_time = time.time()

        # leader appena (ri)entrato nel FOV: riparte da zero
        if wasnt_in_fov:
            self.old_pos = position
            self.last_update_time = current_time
            self.velocity = np.zeros(3)
            return self.velocity, False

        # campione OptiTrack non ancora aggiornato: tieni la stima precedente
        # (altrimenti la velocità alterna 0 e 2v e il feedforward fa scattare il drone)
        if np.array_equal(position, self.old_pos):
            return self.velocity, False

        dt = current_time - self.last_update_time
        if dt < 0.001:
            logger.debug("dt too small, skipping update")
            return self.velocity, False
        dt = min(dt, 0.2)

        instant_velocity = (position - self.old_pos) / dt
        self.old_pos = position
        self.last_update_time = current_time
        self.velocity = instant_velocity

        # Salva velocità e istante di tempo su file prima di ritornare
        self._log_to_file(current_time, self.velocity)
        return self.velocity, False
