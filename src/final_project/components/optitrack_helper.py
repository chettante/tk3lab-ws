"""
Helper per leggere posizioni e orientamenti da OptiTrack in modo robusto.
Astrae i dettagli dell'API di genomix.
"""

import logging
import numpy as np

logger = logging.getLogger("OptiTrackHelper")


class OptiTrackHelper:
    """
    Wrapper intorno all'API di OptiTrack per leggere posizioni e quaternioni.
    """

    def __init__(self, optitrack_component):
        """
        Args:
            optitrack_component: componente 'optitrack' caricato da genomix
        """
        self.optitrack = optitrack_component

    def read_body(self, body_name: str):
        """
        Legge posizione e orientamento di un body da OptiTrack.
        
        Args:
            body_name: nome del body ('QR4_leading', 'QR4_following', etc.)
        
        Returns:
            (position, quaternion) tuple
            - position: np.array([x, y, z])
            - quaternion: np.array([qw, qx, qy, qz])
            
            Se errore: (None, None)
        """
        try:
            body_data = self.optitrack.bodies(body_name)
            
            # Estrai posizione
            pos_dict = body_data['bodies']['pos']
            position = np.array([
                pos_dict['x'],
                pos_dict['y'],
                pos_dict['z']
            ])
            
            # Estrai quaternione (orientamento)
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
    Factory per creare una funzione di lettura OptiTrack.
    
    Args:
        optitrack_component: componente 'optitrack' da genomix
    
    Returns:
        Una funzione optitrack_reader(body_name) → (position, quaternion)
    """
    helper = OptiTrackHelper(optitrack_component)
    return helper.read_body
