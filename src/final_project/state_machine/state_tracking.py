"""
TrackingState: chases the leader with a P + feedforward controller.
Transitions:
  - → SEARCHING if the leader stays out of the FOV longer than out_of_fov_timeout
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
        kp_angle,
        out_of_fov_timeout,
        acc_limits,
    ):
        """
        Args:
            camera: FOVPyramid instance
            tracker: TrackingController instance
            optitrack_helper: function that reads the poses from OptiTrack
            follower_bodies: tuple ('QR4_leading', 'QR4_following')
            kp_angle: proportional gain on yaw
            out_of_fov_timeout: seconds out of the FOV before switching to SEARCHING
            acc_limits: (tracking, free, yaw) acceleration limits of the follower maneuver.
                maneuver.velocity() reaches the command as fast as its limit allows, so in TRACKING
                the limit (per axis) is the tilt budget; free = follower_max_acc for the other states
        """
        self.camera = camera
        self.tracker = tracker
        self.optitrack_helper = optitrack_helper
        self.leader_body_name, self.follower_body_name = follower_bodies
        self.kp_angle = kp_angle
        self.out_of_fov_timeout = out_of_fov_timeout
        self.acc_tracking, self.acc_free, self.yaw_acc = acc_limits

        # reset by enter(), called by the StateMachine on every transition
        self.last_fov_time = None
        self.ticks_since_enter = 0
        self.wasnt_in_fov = True
        self.last_leader_pos = None
        self.last_leader_vel = np.zeros(3)
        # measured follower velocity (OptiTrack) for the yaw FF
        self.follower_vel_estimator = VelocityEstimator(log=False)

        self.loop_count = 0

    def enter(self):
        """Called by the StateMachine on every entry into TRACKING: resets the state of the previous round."""
        self.last_fov_time = time.time()
        self.ticks_since_enter = 0
        self.wasnt_in_fov = True
        self.last_leader_pos = None
        self.last_leader_vel = np.zeros(3)
        self.follower_vel_estimator.reset()
        self.tracker.reset()
        self.tracker.follower.set_acceleration_limit(self.acc_tracking, self.yaw_acc)

    def update(self):
        """
        1. Reads the poses from OptiTrack
        2. If the leader is in the FOV: updates the velocity estimate and chases it
        3. If it is out: keeps going on the extrapolated position until out_of_fov_timeout,
           then switches to SEARCHING
        """
        self.loop_count += 1
        now = time.time()

        leader_pos, _ = self.optitrack_helper(self.leader_body_name)
        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)

        if leader_pos is None or follower_pos is None:
            return None

        # camera heading = drone yaw + mounting yaw: the yaw control
        # centers the leader on the optical axis, not on the drone x axis
        camera_yaw = _quat_to_yaw(follower_quat) + self.camera.yaw_offset
        pos_in_body = self.camera.measure(leader_pos, follower_pos, follower_quat)
        follower_vel = self.follower_vel_estimator.update_world(follower_pos)
        self.ticks_since_enter += 1

        if self.camera.contains(pos_in_body):
            self.last_fov_time = now

            # leader position and velocity from the camera measurement
            leader_vel, leader_cam_pos = self.tracker.velocity_estimator.update(
                pos_in_body, follower_pos, follower_quat, self.camera, self.wasnt_in_fov
            )
            self.wasnt_in_fov = False

            self.last_leader_pos = leader_cam_pos
            self.last_leader_vel = leader_vel
            leader_est = self.last_leader_pos
            vel_est = self.last_leader_vel

        else:
            self.wasnt_in_fov = True
            elapsed_out = now - self.last_fov_time

            if elapsed_out > self.out_of_fov_timeout or self.last_leader_pos is None:
                logger.warning(
                    f"Leader lost (out of FOV for {elapsed_out:.2f}s), "
                    "transitioning to SEARCHING"
                )
                self.tracker.follower.set_acceleration_limit(self.acc_free, self.yaw_acc)
                return StateType.SEARCHING

            # grace period: keep going on the extrapolated position
            leader_est = self.last_leader_pos + self.last_leader_vel * elapsed_out
            vel_est = self.last_leader_vel

            if self.loop_count % 10 == 0:
                logger.debug(f"[TRACKING] Leader OUT of FOV for {elapsed_out:.2f}s (predicting)")

        # bumpless transfer: on the first cycle the follower velocity is not measurable yet
        # (two samples are needed), so the last command (searching/exploring) is left active;
        # on the second one the rate limiter starts from the measured velocity instead of from rest
        if self.ticks_since_enter == 1:
            return None
        if self.ticks_since_enter == 2:
            self.tracker.reset(follower_vel)

        velocity_cmd, yaw_cmd = self.tracker.compute_command(
            leader_pos=leader_est,
            follower_pos=follower_pos,
            follower_yaw=camera_yaw,
            camera=self.camera,
            leader_vel=vel_est,
            kp_angle=self.kp_angle,
            follower_vel=follower_vel,
        )
        self.tracker.send_command(velocity_cmd, yaw_cmd)

        return None