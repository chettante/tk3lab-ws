"""
TrackingState: inseguimento del leader con controllore PI + feedforward.
Transizioni:
  - → SEARCHING se il leader resta fuori dal FOV oltre out_of_fov_timeout
  - → IDLE se scatta tracking_timeout
"""

import logging
import time
from typing import Optional
import numpy as np
from final_project.state_machine.state_machine import StateType
from final_project.components.functions import _quat_to_yaw

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

        # impostato dalla StateMachine a ogni transizione
        self.enter_time = None
        self._entered_at = None     # ultimo enter_time già gestito

        self.last_fov_time = None
        self.wasnt_in_fov = True
        self.last_leader_pos = None
        self.last_leader_vel = np.zeros(3)
        self._f_prev = None             # (posizione, tempo) precedenti del follower
        self.follower_vel = np.zeros(3)

        self.loop_count = 0
        self.in_fov_count = 0
        self.out_fov_count = 0

    def _stop(self):
        self.tracker.send_command(
            np.zeros(3), yaw_cmd=0.0,
            max_acc=self.max_acc,
        )

    def _reset_if_new_entry(self, now):
        # La StateMachine imposta enter_time a ogni transizione:
        # se è cambiato, siamo appena entrati in TRACKING.
        if self._entered_at != self.enter_time:
            self._entered_at = self.enter_time
            self.last_fov_time = now
            self.wasnt_in_fov = True
            self.last_leader_pos = None
            self.last_leader_vel = np.zeros(3)
            self._f_prev = None
            self.follower_vel = np.zeros(3)
            self.tracker.reset()

    def _update_follower_vel(self, follower_pos, now, alpha=0.5):
        """Velocità misurata del follower (differenza finita + EMA) per il FF dello yaw."""
        p = np.asarray(follower_pos, dtype=float)
        if self._f_prev is not None:
            p_prev, t_prev = self._f_prev
            if np.array_equal(p, p_prev):       # campione OptiTrack non aggiornato
                return self.follower_vel
            dt = now - t_prev
            if dt > 1e-3:
                v = (p - p_prev) / min(dt, 0.2)
                self.follower_vel = alpha * v + (1 - alpha) * self.follower_vel
        self._f_prev = (p, now)
        return self.follower_vel

    def update(self) -> Optional[StateType]:
        """
        1. Legge le pose da OptiTrack
        2. Se il leader è nel FOV: aggiorna la stima di velocità e inseguilo
        3. Se è fuori: continua sulla posizione estrapolata fino a out_of_fov_timeout,
           poi ferma il drone e passa a SEARCHING
        4. Se scatta tracking_timeout: ferma il drone e passa a IDLE
        """
        self.loop_count += 1
        now = time.time()
        self._reset_if_new_entry(now)

        leader_pos, _ = self.optitrack_helper(self.leader_body_name)
        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)

        if leader_pos is None or follower_pos is None:
            logger.debug("Could not read positions")
            self._stop()
            return None

        follower_yaw = _quat_to_yaw(follower_quat)
        pos_in_body = self.camera.global_to_body(leader_pos, follower_pos, follower_yaw)
        follower_vel = self._update_follower_vel(follower_pos, now)

        if self.camera.contains(pos_in_body):
            self.in_fov_count += 1
            self.last_fov_time = now

            leader_vel, self.wasnt_in_fov = self.tracker.velocity_estimator.update(
                leader_pos, self.wasnt_in_fov
            )
            if leader_vel is None:
                leader_vel = np.zeros(3)
            self.wasnt_in_fov = False

            self.last_leader_pos = np.asarray(leader_pos, dtype=float)
            self.last_leader_vel = np.asarray(leader_vel, dtype=float)
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
            follower_yaw=follower_yaw,
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