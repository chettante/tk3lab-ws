import time
import logging
import numpy as np
from final_project.state_machine.state_machine import StateType
from final_project.components.functions import _quat_to_yaw


logger = logging.getLogger("IdleState")


class IdleState:
    def __init__(self,
                 camera, 
                 optitrack_helper, 
                 follower_bodies
    ):
        self.camera = camera
        self.optitrack_helper = optitrack_helper
        self.leader_body_name, self.follower_body_name = follower_bodies
        self.enter_time = None
        self.idle_timeout = 60.0
        self.fov_check_count = 0

    def update(self):
        self.fov_check_count += 1
        leader_pos, leader_quat = self.optitrack_helper(self.leader_body_name)
        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)

        if leader_pos is None or follower_pos is None:
            return None

        follower_yaw = _quat_to_yaw(follower_quat)
        pos_in_body = self.camera.global_to_body(leader_pos, follower_pos, follower_yaw)

        if self.camera.contains(pos_in_body):
            logger.info("Leader detected in FOV! Transitioning to TRACKING")
            return StateType.TRACKING

        elapsed = time.time() - self.enter_time
        if elapsed > self.idle_timeout:
            logger.info("Idle timeout reached. Transitioning to SEARCHING")
            #return StateType.SEARCHING

        return None