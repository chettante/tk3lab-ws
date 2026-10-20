"""
SearchingState: the leader has left the FOV, the follower runs the search maneuver
(LeaderSearchManeuver) one step per main loop cycle.

Transitions:
  - → TRACKING if the leader re-enters the FOV
  - → IDLE at the end of the maneuver (leader not found)
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
            optitrack_helper: function that reads the poses from OptiTrack
            vel_estimator: leader VelocityEstimator (the same as the tracker's)
            tracking_state: TrackingState, for the time of the last sighting
            follower_bodies: tuple ('QR4_leading', 'QR4_following')
            search: LeaderSearchManeuver instance
            maneuver_follower: follower maneuver component
        """
        self.camera = camera
        self.optitrack_helper = optitrack_helper
        self.leader_body_name, self.follower_body_name = follower_bodies
        self.search = search
        self.vel_estimator = vel_estimator
        self.tracking_state = tracking_state
        self.maneuver_follower = maneuver_follower

        # reset by enter(), called by the StateMachine on every transition
        self._planned = False           # maneuver already planned for this search
        self._phase = None              # current phase, to log the changes

    def _stop(self):
        self.maneuver_follower.velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

    def enter(self):
        """Called by the StateMachine on every entry into SEARCHING: the maneuver must be replanned."""
        self._planned = False
        self._phase = None

    def update(self):
        """
        1. Reads the poses from OptiTrack and repeats the camera measurement
        2. If the leader is in the FOV → TRACKING
        3. On the first cycle plans the maneuver from the last leader estimate
        4. Runs one step of the maneuver; at the end stops the drone → IDLE
        """

        leader_pos, _ = self.optitrack_helper(self.leader_body_name)
        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)

        pos_in_body = self.camera.measure(leader_pos, follower_pos, follower_quat)
        if self.camera.contains(pos_in_body):
            logger.info("Leader re-acquired! Transitioning back to TRACKING")
            return StateType.TRACKING

        if not self._planned:
            # last leader estimate from the VelocityEstimator: the leader is out of the FOV,
            # so there is no new measurement to differentiate
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

        # a NaN sent to maneuver crashes its planner (kdtp): better to stop
        if not (np.all(np.isfinite(vel)) and np.all(np.isfinite(acc)) and np.isfinite(yaw_rate)):
            logger.error(f"Non-finite search command (vel={vel}, acc={acc}, wz={yaw_rate}), "
                         "stopping and transitioning to IDLE")
            self._stop()
            return StateType.IDLE

        self.maneuver_follower.velocity(
            vel[0], vel[1], vel[2], yaw_rate, acc[0], acc[1], acc[2], 0
        )
        return None
