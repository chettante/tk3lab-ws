import numpy as np
import time
import logging
from typing import Optional
from velocity_estimator import VelocityEstimator


logger = logging.getLogger("FOVCamera")


class FOVPyramid:
    """Geometria del FOV come piramide con apex nel body frame del follower"""
 
    def __init__(self, half_angle_deg: float = 45.0, max_range: float = 10.0):
        """
        Args:
            half_angle_deg: semi-apertura angolare della piramide (gradi)
            max_range: distanza massima di tracking (metri)
        """
        self.half_angle = np.radians(half_angle_deg)
        self.max_range = max_range
        self.half_angle_deg = half_angle_deg
 
    def contains(self, point_in_body_frame: np.ndarray) -> bool:
        """
        Controlla se un punto è dentro la piramide FOV.
 
        Args:
            point_in_body_frame: [x, y, z] nel frame del follower
                                 (x pointing forward, y-z lateral)
 
        Returns:
            True se il punto è nel FOV
        """
        # Convert to numpy array if needed
        if not isinstance(point_in_body_frame, np.ndarray):
            point_in_body_frame = np.array(point_in_body_frame)
        
        # Range check
        distance = np.linalg.norm(point_in_body_frame)
        
        if distance > self.max_range or distance < 0.1:
            logger.debug(f"Point is out of range: {distance:.2f}m (max: {self.max_range}m)")
            return False
 
        # Angle check: il punto deve essere davanti (x > 0)
        if point_in_body_frame[0] <= 0:
            logger.debug(f"Point is behind the camera: x={point_in_body_frame[0]:.2f}")
            return False
 
        # Angolo dal centerline (asse x)
        cos_angle = point_in_body_frame[0] / distance
        
        # Proteggi da errori numerici
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle_from_center = np.arccos(cos_angle)
        
        logger.debug(f"Angle from center: {np.degrees(angle_from_center):.1f}° (max: {self.half_angle_deg}°)")
 
        return angle_from_center <= self.half_angle
 
    def get_angle_from_center(self, point_in_body_frame: np.ndarray) -> float:
        """Ritorna l'angolo dal centerline (radianti)"""
        if not isinstance(point_in_body_frame, np.ndarray):
            point_in_body_frame = np.array(point_in_body_frame)
        
        distance = np.linalg.norm(point_in_body_frame)
        if distance < 0.1:
            return 0.0
        cos_angle = np.clip(point_in_body_frame[0] / distance, -1.0, 1.0)
        return np.arccos(cos_angle)


class TrackingController:
    """Image-Based Visual Servoing (IBVS) con predizione"""
 
    def __init__(
        self,
        follower,
        kp: float = 0.5,
        velocity_estimator: Optional[VelocityEstimator] = None,
    ):
        """
        Args:
            follower: maneuver component per il follower (maneuver_f)
            kp: guadagno proporzionale (0.3-0.7)
            velocity_estimator: stimatore di velocità del leader
        """
        self.follower = follower
        self.kp = kp
        self.velocity_estimator = velocity_estimator or VelocityEstimator()
        self.last_command_time = time.time()
        self.command_period = 0.05  # 20 Hz
 
    def compute_command(
        self,
        leader_pos: np.ndarray,
        follower_pos: np.ndarray,
        leader_vel: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Calcola il comando di velocità via visual servoing.
 
        Args:
            leader_pos: posizione assoluta del leader
            follower_pos: posizione assoluta del follower
            leader_vel: velocità stimata del leader (opzionale)
 
        Returns:
            velocity_command: [vx, vy, vz] da inviare via maneuver.velocity()
        """
        # Conversione a numpy array se necessario
        if not isinstance(leader_pos, np.ndarray):
            leader_pos = np.array(leader_pos)
        if not isinstance(follower_pos, np.ndarray):
            follower_pos = np.array(follower_pos)
        
        # Posizione relativa nel frame globale
        relative_pos = leader_pos - follower_pos
        
        # Proporzionale puro
        velocity_cmd = self.kp * relative_pos
 
        # Feedforward della velocità predetta del leader
        if leader_vel is not None:
            if not isinstance(leader_vel, np.ndarray):
                leader_vel = np.array(leader_vel)
            
            prediction_horizon = 0.1  # 100 ms
            velocity_cmd += 0.5 * leader_vel * prediction_horizon
 
        return velocity_cmd
 
    def send_command(self, velocity_cmd: np.ndarray):
        """
        Invia il comando al drone follower via maneuver.velocity()
        
        Args:
            velocity_cmd: [vx, vy, vz] velocità da inviare (m/s)
        """
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_command_time < self.command_period:
            return
 
        self.last_command_time = current_time
 
        try:
            if not isinstance(velocity_cmd, np.ndarray):
                velocity_cmd = np.array(velocity_cmd)
            
            # Invia comando via maneuver.velocity()
            self.follower.velocity(
                vx=float(velocity_cmd[0]),
                vy=float(velocity_cmd[1]),
                vz=float(velocity_cmd[2]),
                wz=0.0,                    # No yaw control
                ax=0.0, ay=0.0, az=0.0,   # No acceleration
                duration=1              # Refresh rate
            )
            logger.debug(
                f"Velocity cmd sent: vx={velocity_cmd[0]:.2f}, "
                f"vy={velocity_cmd[1]:.2f}, vz={velocity_cmd[2]:.2f}"
            )
        except Exception as e:
            logger.error(f"Error sending velocity command: {e}")