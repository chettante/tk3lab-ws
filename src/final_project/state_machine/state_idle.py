import time
import logging
import numpy as np
from final_project.state_machine.state_machine import StateType
from final_project.components.functions import _quat_to_yaw


logger = logging.getLogger("IdleState")


class IdleState:
    def __init__(
        self,
        camera,
        optitrack_helper,
        follower_bodies,
        idle_timeout,
        maneuver_follower=None,
        spawn_lift_height=1.0,
    ):
        self.camera = camera
        self.optitrack_helper = optitrack_helper
        self.leader_body_name, self.follower_body_name = follower_bodies
        self.maneuver_follower = maneuver_follower
        self.spawn_lift_height = spawn_lift_height
        self.enter_time = None
        self.idle_timeout = idle_timeout
        self.fov_check_count = 0
        self.spawn_lift_done = False

    def _spawn_lift(self):
        if self.maneuver_follower is None:
            return

        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)
        if follower_pos is None or follower_quat is None:
            return

        x, y, z = follower_pos
        yaw = _quat_to_yaw(follower_quat)
        target_z = z + self.spawn_lift_height

        self.maneuver_follower.goto(
            x,
            y,
            target_z,
            yaw,
            0,
        )
        logger.info(
            "Idle spawn lift: initial z=%.2f m, target z=%.2f m",
            z,
            target_z,
        )

    def update(self):
        self.fov_check_count += 1

        if not self.spawn_lift_done:
            self._spawn_lift()
            self.spawn_lift_done = True

        leader_pos, leader_quat = self.optitrack_helper(self.leader_body_name)
        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)

        if leader_pos is None or follower_pos is None:
            return None

        pos_in_body = self.camera.measure(leader_pos, follower_pos, follower_quat)

        if self.camera.contains(pos_in_body):
            logger.info("Leader detected in FOV! Transitioning to TRACKING")
            return StateType.TRACKING

        elapsed = time.time() - self.enter_time
        if elapsed > self.idle_timeout:
            logger.info("Idle timeout reached. Transitioning to EXPLORING")
            return StateType.EXPLORING

        return None