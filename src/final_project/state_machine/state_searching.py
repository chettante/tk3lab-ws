"""
SearchingState: il leader è uscito dal FOV, il follower esegue la manovra di ricerca
(LeaderSearchManeuver) un passo per ciclo del main loop.

Transizioni:
  - → TRACKING se il leader rientra nel FOV
  - → IDLE a fine manovra (leader non ritrovato)
"""

import logging
import time
import numpy as np
from final_project.state_machine.state_machine import StateType
from final_project.components.functions import _quat_to_yaw


logger = logging.getLogger("SearchingState")


class SearchingState:
    def __init__(
        self,
        camera,
        optitrack_helper,
        vel_estimator,
        tracking_state,
        follower_bodies,
        search,
        maneuver_follower,
    ):
        """
        Args:
            camera: FOVPyramid instance
            optitrack_helper: funzione che legge le posizioni da OptiTrack
            vel_estimator: VelocityEstimator del leader (lo stesso del tracker)
            tracking_state: TrackingState, per l'istante dell'ultimo avvistamento
            follower_bodies: tuple ('QR4_leading', 'QR4_following')
            search: LeaderSearchManeuver instance
            maneuver_follower: componente maneuver del follower
        """
        self.camera = camera
        self.optitrack_helper = optitrack_helper
        self.leader_body_name, self.follower_body_name = follower_bodies
        self.search = search
        self.vel_estimator = vel_estimator
        self.tracking_state = tracking_state
        self.maneuver_follower = maneuver_follower

        # impostato da enter(), chiamato dalla StateMachine a ogni transizione
        self.enter_time = None
        self._planned = False           # manovra già pianificata per questa ricerca
        self._phase = None              # fase corrente, per loggare i cambi

        self.loop_count = 0

    def _stop(self):
        self.maneuver_follower.velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

    def enter(self):
        """Chiamato dalla StateMachine a ogni ingresso in SEARCHING: la manovra va ripianificata."""
        self.enter_time = time.time()
        self._planned = False
        self._phase = None

    def update(self):
        """
        1. Legge le pose da OptiTrack e rifà la misura della camera
        2. Se il leader è nel FOV → TRACKING
        3. Al primo ciclo pianifica la manovra dall'ultima stima del leader
        4. Esegue un passo della manovra; a fine manovra ferma il drone → IDLE
        """
        self.loop_count += 1

        leader_pos, _ = self.optitrack_helper(self.leader_body_name)
        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)

        pos_in_body = self.camera.measure(leader_pos, follower_pos, follower_quat)
        if self.camera.contains(pos_in_body):
            logger.info("Leader re-acquired! Transitioning back to TRACKING")
            return StateType.TRACKING

        if not self._planned:
            # ultima stima del leader dal VelocityEstimator: il leader è fuori FOV,
            # quindi non c'è una nuova misura da differenziare
            p_leader_last = self.vel_estimator.old_pos
            camera_yaw = _quat_to_yaw(follower_quat) + self.camera.yaw_offset
            last_seen = self.tracking_state.last_fov_time
            t_lost = time.time() - last_seen if last_seen is not None else 0.0
            self.search.update_leader(
                p_leader_last=p_leader_last,
                v_leader_last=self.vel_estimator.velocity,
                p_follower0=follower_pos,
                yaw0=camera_yaw,
                t_lost=t_lost,
            )
            self._planned = True
            logger.info(
                f"Search planned ({self.search.T_total:.1f}s): t_lost={t_lost:.2f}s "
                f"p_pred={np.round(self.search.p_pred, 2)} "
                f"dyaw={np.degrees(self.search.dyaw_rec):.0f}deg "
                f"p_rec={np.round(self.search.p_rec, 2)} T_rec={self.search.T_rec_eff:.2f}s"
            )

        phase = self.search.phase_name()
        if phase != self._phase:
            logger.info(f"[SEARCHING] phase: {phase}")
            self._phase = phase

        _, vel, acc, _, yaw_rate, done = self.search.step()

        if done:
            logger.warning("Search finished without finding the leader, transitioning to IDLE")
            self._stop()
            return StateType.IDLE

        # un NaN mandato a maneuver fa crashare il suo planner (kdtp): meglio fermarsi
        if not (np.all(np.isfinite(vel)) and np.all(np.isfinite(acc)) and np.isfinite(yaw_rate)):
            logger.error(f"Non-finite search command (vel={vel}, acc={acc}, wz={yaw_rate}), "
                         "stopping and transitioning to IDLE")
            self._stop()
            return StateType.IDLE

        self.maneuver_follower.velocity(
            vel[0], vel[1], vel[2], yaw_rate, acc[0], acc[1], acc[2], 0
        )
        return None
