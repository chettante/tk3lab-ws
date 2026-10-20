import numpy as np
import time
from final_project.config import config_path


class VelocityEstimator:
    """Estimates the leader velocity from the camera measurement by numerical differentiation"""

    def __init__(self, alpha=1.0, log=True):
        """
        Args:
            alpha: EMA filter on the velocity (1.0 = no filtering)
            log: if True writes to CONFIG['velocity_log'] (see final_project/config.py)
        """
        self.alpha = alpha
        self.old_pos = None                 # last position in world (None = no measurement)
        self.velocity = np.zeros(3)
        self.last_update_time = time.time()

        self.log_path = config_path('velocity_log') if log else None

    def reset(self):
        """Forgets the previous measurement: the next update starts from zero."""
        self.old_pos = None
        self.velocity = np.zeros(3)

    def _log_to_file(self, timestamp: float, velocity: np.ndarray):
        """Appends a line with timestamp and velocity to the log file"""
        with open(self.log_path, 'a') as f:
            f.write(f"{timestamp:.6f}, {velocity[0]:.6f}, {velocity[1]:.6f}, {velocity[2]:.6f}\n")

    def update(self, pos_in_cam, follower_pos, follower_quat, camera, wasnt_in_fov):
        """
        Updates the velocity estimate.
        Args:
            pos_in_cam: leader position in the camera frame (camera.measure)
            follower_pos, follower_quat: follower pose
            camera: FOVPyramid, to bring the measurement back to world
            wasnt_in_fov: True if the leader has just (re)entered the FOV
        Returns:
            (estimated velocity in world [m/s], reconstructed leader position in world)
        """
        # the derivative must be taken in world: the camera frame rotates with the follower
        position = camera.to_world(pos_in_cam, follower_pos, follower_quat)
        return self.update_world(position, reset=wasnt_in_fov), position

    def update_world(self, position, reset=False):
        """
        Updates the velocity estimate from a position already in world.
        Args:
            position: position in world
            reset: True to restart from zero (e.g. leader that just re-entered the FOV)
        Returns:
            estimated velocity in world [m/s]
        """
        position = np.asarray(position, dtype=float)
        current_time = time.time()

        # first measurement: velocity 0. Reset (leader back in the FOV): new reference position,
        # but the last estimate is kept as feedforward until the next sample (0 would make
        # the follower brake just while it has to catch up)
        if reset or self.old_pos is None:
            if self.old_pos is None:
                self.velocity = np.zeros(3)
            self.old_pos = position
            self.last_update_time = current_time
            return self.velocity

        # OptiTrack sample not updated yet: keep the previous estimate
        # (otherwise the velocity alternates between 0 and 2v and the feedforward makes the drone jerk)
        if np.allclose(position, self.old_pos, rtol=0.0, atol=1e-9):
            return self.velocity

        dt = current_time - self.last_update_time
        if dt < 0.001:
            return self.velocity
        dt = min(dt, 0.2)

        v = (position - self.old_pos) / dt
        self.velocity = self.alpha * v + (1 - self.alpha) * self.velocity
        self.old_pos = position
        self.last_update_time = current_time

        # Save velocity and timestamp to file before returning
        if self.log_path is not None:
            self._log_to_file(current_time, self.velocity)
        return self.velocity
