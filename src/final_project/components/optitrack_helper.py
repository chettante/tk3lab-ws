"""
Helper to read positions and orientations from OptiTrack robustly.
Hides the details of the genomix API.
"""

import logging
import numpy as np

logger = logging.getLogger("OptiTrackHelper")


class OptiTrackHelper:
    """
    Wrapper around the OptiTrack API to read positions and quaternions.
    """

    def __init__(self, optitrack_component):
        """
        Args:
            optitrack_component: 'optitrack' component loaded by genomix
        """
        self.optitrack = optitrack_component

    def read_body(self, body_name: str):
        """
        Reads position and orientation of a body from OptiTrack.

        Args:
            body_name: body name ('QR4_leading', 'QR4_following', etc.)

        Returns:
            (position, quaternion) tuple
            - position: np.array([x, y, z])
            - quaternion: np.array([qw, qx, qy, qz])

            On error: (None, None)
        """
        try:
            body_data = self.optitrack.bodies(body_name)

            # Extract position
            pos_dict = body_data['bodies']['pos']
            position = np.array([
                pos_dict['x'],
                pos_dict['y'],
                pos_dict['z']
            ])

            # Extract quaternion (orientation)
            att_dict = body_data['bodies']['att']
            quaternion = np.array([
                att_dict['qw'],
                att_dict['qx'],
                att_dict['qy'],
                att_dict['qz']
            ])

            return position, quaternion

        except Exception as e:
            logger.warning(f"Error reading body '{body_name}': {e}")
            return None, None


def create_optitrack_reader(optitrack_component):
    """
    Factory that creates an OptiTrack reading function.

    Args:
        optitrack_component: 'optitrack' component from genomix

    Returns:
        A function optitrack_reader(body_name) → (position, quaternion)
    """
    helper = OptiTrackHelper(optitrack_component)
    return helper.read_body
