"""
TrackingState: main IBVS tracking loop.
Transizioni:
  - → SEARCHING se leader esce dal FOV (perdita traccia)
  - → IDLE se manual stop or timeout
"""

import logging
import time
import numpy as np
from final_project.state_machine.state_machine import StateType
from final_project.components.functions import _quat_to_yaw

logger = logging.getLogger("TrackingState")


class TrackingState:
    """
    Stato di tracking: esegue IBVS (Image-Based Visual Servoing) con controllo
    proporzionale su distanza e angolo di heading.
    """

    def __init__(
        self,
        camera,
        tracker,
        optitrack_helper,
        follower_bodies,
        kp_distance=0.2,
        kp_angle=0.5,
        max_velocity=2.0,
        out_of_fov_timeout=2.0,
        tracking_timeout=120.0,
    ):
        """
        Args:
            camera: FOVPyramid instance
            tracker: TrackingController instance
            optitrack_helper: funzione che legge le posizioni da OptiTrack
            follower_bodies: tuple ('QR4_leading', 'QR4_following')
            kp_distance: guadagno proporzionale per la distanza
            kp_angle: guadagno proporzionale per l'orientamento (yaw)
            max_velocity: limite di velocità massima (m/s)
            out_of_fov_timeout: secondi fuori dal FOV prima di passare a SEARCHING
            tracking_timeout: timeout globale di tracking (secondi)
        """
        self.camera = camera
        self.tracker = tracker
        self.optitrack_helper = optitrack_helper
        self.leader_body_name, self.follower_body_name = follower_bodies
        self.kp_distance = kp_distance
        self.kp_angle = kp_angle
        self.max_velocity = max_velocity
        self.out_of_fov_timeout = out_of_fov_timeout
        self.tracking_timeout = tracking_timeout
        
        self.enter_time = None
        self.last_fov_time = None
        self.wasnt_in_fov = True
        
        self.loop_count = 0
        self.in_fov_count = 0
        self.out_fov_count = 0


    def update(self) -> StateType or None:
        """
        Logica TRACKING:
        1. Leggi posizioni e orientamenti da OptiTrack
        2. Controlla se leader è nel FOV
        3. Se sì, calcola comando di velocità e invia
        4. Se no, arresta il drone inviando velocità nulla e controlla timeout per SEARCHING
        5. Se tracking_timeout elapsed, transiziona a IDLE
        """
        self.loop_count += 1


        # Leggi posizioni da OptiTrack
        leader_pos, leader_quat = self.optitrack_helper(self.leader_body_name)
        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)

        if leader_pos is None or follower_pos is None:
            logger.debug("Could not read positions")
            # Invia velocità nulla per sicurezza se la misura fallisce
            self.tracker.send_command(np.array([0.0, 0.0, 0.0]), yaw_cmd=0.0)
            return None

        # Calcola yaw del follower
        follower_yaw = _quat_to_yaw(follower_quat)

        # Trasforma in body frame
        pos_in_body = self.camera.global_to_body(leader_pos, follower_pos, follower_yaw)

        # Controlla FOV
        is_in_fov = self.camera.contains(pos_in_body)

        if is_in_fov:
            self.in_fov_count += 1
            self.last_fov_time = time.time()  # Reset del timer di presenza nel FOV

            # Aggiorna stima velocità leader
            leader_vel, self.wasnt_in_fov = self.tracker.velocity_estimator.update(leader_pos, self.wasnt_in_fov)

            # Calcola comando con controllo distanza + angolo
            velocity_cmd, yaw_cmd = self.tracker.compute_command_with_angular_control(
                leader_pos=leader_pos,
                follower_pos=follower_pos,
                follower_yaw=follower_yaw,
                camera=self.camera,
                leader_vel=leader_vel,
                kp_distance=self.kp_distance,
                kp_angle=self.kp_angle,
            )

            # Invia comando
            self.tracker.send_command(
                velocity_cmd=velocity_cmd,
                yaw_cmd=yaw_cmd,
                max_velocity=self.max_velocity,
            )
            
            self.wasnt_in_fov = False
            if self.loop_count % 20 == 0:
                dist = np.linalg.norm(pos_in_body)
                logger.debug(
                    f"[TRACKING] Leader in FOV, distance: {dist:.2f}m, "
                    f"v_cmd: {velocity_cmd}, yaw_cmd: {yaw_cmd:.2f}"
                )

        else:
            # Leader fuori dal FOV: arresta immediatamente il drone
            self.out_fov_count += 1
            elapsed_out_of_fov = time.time() - self.last_fov_time
            wasnt_in_fov = True

            # Invia azzeramento velocità a ogni ciclo fuori dal FOV
            self.tracker.send_command(np.array([0.0, 0.0, 0.0]), yaw_cmd=0.0)

            if self.loop_count % 10 == 0:
                logger.debug(
                    f"[TRACKING] Leader OUT of FOV for {elapsed_out_of_fov:.1f}s (drone stopped)"
                )

            # Transiziona a SEARCHING se il timeout è superato
            if elapsed_out_of_fov > self.out_of_fov_timeout:
                logger.warning(
                    f"Leader lost (out of FOV for {elapsed_out_of_fov:.1f}s > {self.out_of_fov_timeout}s), "
                    "transitioning to SEARCHING"
                )
                # Ferma i motori appena prima del cambio di stato, così il drone
                # non continua a muoversi mentre entra nello stato di ricerca.
                self.tracker.send_command(np.array([0.0, 0.0, 0.0]), yaw_cmd=0.0, max_velocity=self.max_velocity)
                return StateType.SEARCHING

        # Check timeout globale
        elapsed = time.time() - self.enter_time
        if elapsed > self.tracking_timeout:
            logger.warning(f"TRACKING timeout ({self.tracking_timeout}s) elapsed, transitioning to IDLE")
            return StateType.IDLE   

        return None