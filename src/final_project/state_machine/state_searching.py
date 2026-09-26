"""
SearchingState: placeholder per la fase di ricerca.
Logica di ricerca (loiter orbit, spiral, etc.) ancora da implementare.

Transizioni:
  - → TRACKING se leader riaquistato
  - → IDLE se search_timeout elapsed
"""

import logging
import time
import numpy as np
from final_project.state_machine.state_machine import StateType
from final_project.components.functions import _quat_to_yaw

logger = logging.getLogger("SearchingState")


class SearchingState:
    """
    Stato di ricerca: il leader è andato fuori dal FOV e il follower lo sta cercando.
    
    TODO: implementare strategia di ricerca (loiter orbit, spiral, etc.)
    """

    def __init__(
        self,
        camera,
        optitrack_helper,
        follower_bodies,
        search_timeout=30.0,
    ):
        """
        Args:
            camera: FOVPyramid instance
            optitrack_helper: funzione che legge le posizioni da OptiTrack
            follower_bodies: tuple ('QR4_leading', 'QR4_following')
            search_timeout: tempo massimo per la ricerca (secondi)
        """
        self.camera = camera
        self.optitrack_helper = optitrack_helper
        self.leader_body_name, self.follower_body_name = follower_bodies
        self.search_timeout = search_timeout
        
        self.enter_time = None
        self.loop_count = 0

    def update(self) -> StateType or None:
        """
        Logica SEARCHING (TODO):
        1. Implementare strategia di ricerca (loiter orbit, spiral, etc.)
        2. Monitorare se leader torna nel FOV → TRACKING
        3. Se timeout → IDLE
        
        Per ora: just check timeout and return to IDLE
        """
        self.loop_count += 1

        # Leggi posizioni da OptiTrack
        leader_pos, leader_quat = self.optitrack_helper(self.leader_body_name)
        follower_pos, follower_quat = self.optitrack_helper(self.follower_body_name)

        if leader_pos is None or follower_pos is None:
            logger.debug("Could not read positions")
            return None

        # Calcola yaw del follower
        follower_yaw = _quat_to_yaw(follower_quat)

        # Trasforma in body frame
        pos_in_body = self.camera.global_to_body(leader_pos, follower_pos, follower_yaw)

        # TODO: eseguire movimento di ricerca (loiter, spiral, etc.)
        # Per ora: print debug
        if self.loop_count % 20 == 0:
            dist = np.linalg.norm(pos_in_body)
            logger.debug(f"[SEARCHING] Leader distance: {dist:.2f}m (position only)")

        # Controlla se leader è tornato nel FOV
        if self.camera.contains(pos_in_body):
            logger.info("Leader re-acquired! Transitioning back to TRACKING")
            return StateType.TRACKING

        # Check timeout
        elapsed = time.time() - self.enter_time
        if elapsed > self.search_timeout:
            logger.warning(
                f"SEARCHING timeout ({self.search_timeout}s) elapsed, "
                "transitioning to IDLE"
            )
            return StateType.IDLE

        return None