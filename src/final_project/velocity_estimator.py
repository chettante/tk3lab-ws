import numpy as np
import time
import os
from collections import deque
import logging


logger = logging.getLogger("VelocityEstimator")


class VelocityEstimator:
    """Stima la velocità del leader via differenziazione numerica con smoothing"""

    def __init__(self, window_size: int = 5, alpha: float = 0.7, log_filename: str = "velocity_log.txt"):
        """
        Args:
            window_size: numero di campioni per la media mobile
            alpha: fattore di smoothing low-pass (0.5-0.9)
            log_filename: nome del file di log (salvato nella stessa cartella dello script)
        """
        self.position_history = deque(maxlen=window_size)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.alpha = alpha
        self.last_update_time = time.time()

        # Percorso del file di log, sempre nella stessa cartella di questo .py
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.log_path = os.path.join(script_dir, log_filename)

    def _log_to_file(self, timestamp: float, velocity: np.ndarray):
        """Appende una riga con timestamp e velocità al file di log"""
        try:
            with open(self.log_path, "a") as f:
                f.write(f"{timestamp:.6f}, {velocity[0]:.6f}, {velocity[1]:.6f}, {velocity[2]:.6f}\n")
        except IOError as e:
            logger.warning(f"Impossibile scrivere sul file di log: {e}")

    def update(self, position: np.ndarray) -> np.ndarray:
        """
        Aggiorna la stima di velocità.
        Args:
            position: posizione attuale [x, y, z]
        Returns:
            Velocità stimata (m/s)
        """
        # Conversione a numpy array se necessario
        if not isinstance(position, np.ndarray):
            position = np.array(position)

        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Evita divisione per zero
        if dt < 0.001:
            logger.debug("dt too small, skipping update")
            return self.velocity

        self.position_history.append(position)

        if len(self.position_history) >= 2:
            # Velocità istantanea tra ultimi due campioni
            old_pos = self.position_history[0]
            new_pos = self.position_history[-1]
            instant_velocity = (new_pos - old_pos) / dt

            # Low-pass filter per smoothing
            self.velocity = (
                self.alpha * instant_velocity +
                (1.0 - self.alpha) * self.velocity
            )

        # Salva velocità e istante di tempo su file prima di ritornare
        self._log_to_file(current_time, self.velocity)
        print("VELOCITA PORCODDIO", self.velocity)
        return self.velocity





