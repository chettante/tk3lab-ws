import time
import logging
import numpy as np
from final_project.state_machine.state_machine import StateType


logger = logging.getLogger("IdleState")


class IdleState:
    def __init__(self,
                 camera, 
                 optitrack_helper, 
                 follower_bodies,
                 idle_timeout
    ):
        self.camera = camera
        self.optitrack_helper = optitrack_helper
        self.leader_body_name, self.follower_body_name = follower_bodies
        self.enter_time = None
        self.idle_timeout = idle_timeout
        self.fov_check_count = 0

    def update(self):
        self.fov_check_count += 1
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
            logger.info("Idle timeout reached. Transitioning to SEARCHING")
            #return StateType.SEARCHING

        return None