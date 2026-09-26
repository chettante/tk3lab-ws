import numpy as np
import time
import logging
from typing import Optional
from final_project.components.velocity_estimator import VelocityEstimator


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
    """Image-Based Visual Servoing (IBVS) con predizione + controllo angolare"""
 
    def __init__(
        self,
        follower,
        kp: float = 0.1,
        velocity_estimator: Optional[VelocityEstimator] = None,
    ):
        """
        Args:
            follower: maneuver component per il follower (maneuver_f)
            kp: guadagno proporzionale distanza (0.1-0.3)
            velocity_estimator: stimatore di velocità del leader
        """
        self.follower = follower
        self.kp = kp
        self.velocity_estimator = velocity_estimator or VelocityEstimator()
        self.last_command_time = time.time()
        self.command_period = 0.05  # 20 Hz limit
 
    def compute_command_with_angular_control(
        self,
        leader_pos,
        follower_pos,
        follower_yaw,
        camera,
        leader_vel=None,
        kp_distance=0.1,
        kp_angle=0.5
    ):
        """
        Controllo ibrido: distanza + heading alignment
        
        Args:
            leader_pos: posizione assoluta leader
            follower_pos: posizione assoluta follower
            follower_yaw: orientamento follower (rad) da mocap
            camera: FOVPyramid instance
            leader_vel: velocità stimata leader (optional)
            kp_distance: guadagno controllo distanza (0.1-0.3)
            kp_angle: guadagno controllo yaw (0.5-2.0)
        
        Returns:
            velocity_cmd: [vx, vy, vz] m/s
            yaw_cmd: velocità angolare wz (rad/s)
        """
        
        # ===== CONTROLLO DISTANZA =====
        relative_pos = leader_pos - follower_pos
        
        # Proporzionale sulla distanza
        velocity_cmd = kp_distance * relative_pos
        
        # Feedforward velocità leader
        if leader_vel is not None:
            velocity_cmd += leader_vel
        
        # ===== CONTROLLO ANGOLARE =====
        # Calcola errore di heading usando il metodo della camera
        yaw_error = camera.get_yaw_error(relative_pos, follower_yaw)
        
        # Controllo proporzionale su yaw
        yaw_cmd = kp_angle * yaw_error
        
        # Saturazione yaw (max rotazione 1.0 rad/s)
        yaw_cmd = np.clip(yaw_cmd, -1.0, 1.0)
        
        return velocity_cmd, yaw_cmd
 
    def send_command(self, velocity_cmd: np.ndarray, yaw_cmd: float = 0.0, max_velocity: float = 1.0):
        """
        Invia il comando al drone follower via maneuver.velocity()
        
        Args:
            velocity_cmd: [vx, vy, vz] velocità lineare (m/s)
            yaw_cmd: velocità angolare wz (rad/s)
            max_velocity: limite di velocità massima (m/s)
        """
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_command_time < self.command_period:
            return
 
        self.last_command_time = current_time
 
        try:
            if not isinstance(velocity_cmd, np.ndarray):
                velocity_cmd = np.array(velocity_cmd)
            
            # Saturazione velocità lineare
            cmd_magnitude = np.linalg.norm(velocity_cmd)
            if cmd_magnitude > max_velocity:
                velocity_cmd = (velocity_cmd / cmd_magnitude) * max_velocity
                logger.debug(f"Velocity saturated to {max_velocity:.2f} m/s")
            
            # Parametri POSIZIONALI: vx, vy, vz, ax, ay, az, duration, wz
            self.follower.velocity(
                vx=float(velocity_cmd[0]),  # vx
                vy=float(velocity_cmd[1]),  # vy
                vz=float(velocity_cmd[2]),  # vz
                wz=float(yaw_cmd),  # wz
                ax=2.0, ay=2.0, az=2.0,          # ax, ay, az (accelerazione per planner)
                duration=0.0,                     # duration (1 secondo)
            )
            logger.debug(
                f"Command sent: v=[{velocity_cmd[0]:.2f}, {velocity_cmd[1]:.2f}, {velocity_cmd[2]:.2f}] m/s, "
                f"wz={yaw_cmd:.2f} rad/s"
            )
        except Exception as e:
            logger.error(f"Error sending command: {e}")