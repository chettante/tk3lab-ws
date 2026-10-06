"""
TrackingState: inseguimento del leader con controllore PI + feedforward.
Transizioni:
  - → SEARCHING se il leader resta fuori dal FOV oltre out_of_fov_timeout
  - → IDLE se scatta tracking_timeout
"""

import logging
import time
import numpy as np
from final_project.state_machine.state_machine import StateType
from final_project.components.functions import _quat_to_yaw
from final_project.components.velocity_estimator import VelocityEstimator

logger = logging.getLogger("TrackingState")


class TrackingState:
    def __init__(
        self,
        camera,
        tracker,
        optitrack_helper,
        follower_bodies,
        kp_angle=1.0,
        max_acc=3.0,
        out_of_fov_timeout=0.6,
        tracking_timeout=120.0,
    ):
        """
        Args:
            camera: FOVPyramid instance
            tracker: TrackingController instance
            optitrack_helper: funzione che legge le posizioni da OptiTrack
            follower_bodies: tuple ('QR4_leading', 'QR4_following')
            kp_angle: guadagno proporzionale sullo yaw
            max_acc: limite di accelerazione passato al planner (m/s²)
            out_of_fov_timeout: secondi fuori FOV prima di passare a SEARCHING
            tracking_timeout: timeout globale di tracking (secondi)
        """
        self.camera = camera
        self.tracker = tracker
        self.optitrack_helper = optitrack_helper
        self.leader_body_name, self.follower_body_name = follower_bodies
        self.kp_angle = kp_angle
        self.max_acc = max_acc
        self.out_of_fov_timeout = out_of_fov_timeout
        self.tracking_timeout = tracking_timeout

        # impostato da enter(), chiamato dalla StateMachine a ogni transizione
        self.enter_time = None

        self.last_fov_time = None
        self.wasnt_in_fov = True
        self.last_leader_pos = None
        self.last_leader_vel = np.zeros(3)
        # velocità misurata del follower (OptiTrack) per il FF dello yaw
        self.follower_vel_estimator = VelocityEstimator(log=False)

        self.loop_count = 0
        self.in_fov_count = 0
        self.out_fov_count = 0

    def _stop(self):
        self.tracker.send_command(
            np.zeros(3), yaw_cmd=0.0,
            max_acc=self.max_acc,
        )

    def enter(self):
        """Chiamato dalla StateMachine a ogni ingresso in TRACKING: azzera lo stato del ciclo precedente."""
        self.enter_time = time.time()
        self.last_fov_time = self.enter_time
        self.wasnt_in_fov = True
        self.last_leader_pos = None
        self.last_leader_vel = np.zeros(3)
        self.follower_vel_estimator.reset()
        self.tracker.reset()

    def update(self):
        """
        1. Legge le pose da OptiTrack
        2. Se il leader è nel FOV: aggiorna la stima di velocità e inseguilo
        3. Se è fuori: continua sulla posizione estrapolata fino a out_of_fov_timeout,
           poi ferma il drone e passa a SEARCHING
        4. Se scatta tracking_timeout: ferma il drone e passa a IDLE
        """
        self.loop_count += 1
        now = time.time()

        leader_pos, _ = self.optitrack_helper(self.leader_body_name)
        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)

        if leader_pos is None or follower_pos is None:
            logger.debug("Could not read positions")
            self._stop()
            return None

        # heading della camera = yaw del drone + yaw di montaggio: il controllo di yaw
        # centra il leader sull'asse ottico, non sull'asse x del drone
        camera_yaw = _quat_to_yaw(follower_quat) + self.camera.yaw_offset
        pos_in_body = self.camera.measure(leader_pos, follower_pos, follower_quat)
        follower_vel = self.follower_vel_estimator.update_world(follower_pos)

        if self.camera.contains(pos_in_body):
            self.in_fov_count += 1
            self.last_fov_time = now

            # posizione e velocità del leader dalla misura della camera
            leader_vel, leader_cam_pos = self.tracker.velocity_estimator.update(
                pos_in_body, follower_pos, follower_quat, self.camera, self.wasnt_in_fov
            )
            self.wasnt_in_fov = False

            self.last_leader_pos = leader_cam_pos
            self.last_leader_vel = leader_vel
            leader_est = self.last_leader_pos
            vel_est = self.last_leader_vel

        else:
            self.out_fov_count += 1
            self.wasnt_in_fov = True
            elapsed_out = now - self.last_fov_time

            if elapsed_out > self.out_of_fov_timeout or self.last_leader_pos is None:
                logger.warning(
                    f"Leader lost (out of FOV for {elapsed_out:.2f}s), "
                    "transitioning to SEARCHING"
                )
                self._stop()
                return StateType.SEARCHING

            # grace period: continua sulla posizione estrapolata
            leader_est = self.last_leader_pos + self.last_leader_vel * elapsed_out
            vel_est = self.last_leader_vel

            if self.loop_count % 10 == 0:
                logger.debug(f"[TRACKING] Leader OUT of FOV for {elapsed_out:.2f}s (predicting)")

        velocity_cmd, yaw_cmd = self.tracker.compute_command(
            leader_pos=leader_est,
            follower_pos=follower_pos,
            follower_yaw=camera_yaw,
            camera=self.camera,
            leader_vel=vel_est,
            kp_angle=self.kp_angle,
            follower_vel=follower_vel,
        )
        self.tracker.send_command(
            velocity_cmd, yaw_cmd,
            max_acc=self.max_acc,
        )

        if self.loop_count % 20 == 0:
            logger.debug(
                f"[TRACKING] dist={np.linalg.norm(pos_in_body):.2f}m "
                f"v_cmd={velocity_cmd} yaw_cmd={yaw_cmd:.2f}"
            )

        if now - self.enter_time > self.tracking_timeout:
            logger.warning(f"TRACKING timeout ({self.tracking_timeout}s), transitioning to IDLE")
            self._stop()
            return StateType.IDLE

        return None