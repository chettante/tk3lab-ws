import numpy as np
import time
from collections import deque
import logging


logger = logging.getLogger("VelocityEstimator")


class VelocityEstimator:
    """Stima la velocità del leader via differenziazione numerica con smoothing"""
 
    def __init__(self, window_size: int = 5, alpha: float = 0.7):
        """
        Args:
            window_size: numero di campioni per la media mobile
            alpha: fattore di smoothing low-pass (0.5-0.9)
        """
        self.position_history = deque(maxlen=window_size)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.alpha = alpha
        self.last_update_time = time.time()
 
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
        
        return self.velocity