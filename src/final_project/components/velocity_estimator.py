import numpy as np
import time
import os
from collections import deque
import logging


logger = logging.getLogger("VelocityEstimator")


class VelocityEstimator:
    """Stima la velocità del leader via differenziazione numerica con smoothing"""

    def __init__(self, log_filename: str = "velocity_log.txt"):
        """
        Args:
            log_filename: nome del file di log; per default viene salvato nella cartella graphics
        """
        self.old_pos = np.array([0.0, 0.0, 0.0])
        self.new_pos = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.last_update_time = time.time()

        # Percorso del file di log: final_project/graphics/velocity_log.txt
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        graphics_dir = os.path.join(project_root, "graphics")
        os.makedirs(graphics_dir, exist_ok=True)

        if os.path.isabs(log_filename):
            self.log_path = log_filename
        else:
            self.log_path = os.path.join(graphics_dir, log_filename)

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
        # Check if the leader was not in the field of view (FOV) in the previous update
        if wasnt_in_fov:
            self.old_pos = position
            wasnt_in_fov = False
            return np.array([0.0, 0.0, 0.0]), wasnt_in_fov
        
        wasnt_in_fov = False

        # dt calculation
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        if dt > 0.12:
            dt = 0.1
        if dt < 0.001:
            logger.debug("dt too small, skipping update")
            return self.velocity, wasnt_in_fov

        # Compute velocity estimation
        self.new_pos = position
        instant_velocity = (self.new_pos - self.old_pos) / dt
        self.old_pos = self.new_pos
        self.velocity = instant_velocity

        # Salva velocità e istante di tempo su file prima di ritornare
        self._log_to_file(current_time, self.velocity)
        return self.velocity, wasnt_in_fov