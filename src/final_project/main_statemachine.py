#!/usr/bin/env python3
"""
Main script con state machine: IDLE → TRACKING → SEARCHING.

Orchestrazione:
1. Setup di GenoM3 e droni
2. Inizializzazione state machine
3. Main loop con cicli di update della state machine
4. Cleanup
"""

import os
import sys
import time
import logging
import numpy as np
import signal

# Allow direct execution from the project folder: python3 main_statemachine.py
if __package__ in (None, ""):
    project_root = os.path.dirname(os.path.abspath(__file__))
    src_root = os.path.dirname(project_root)
    if src_root not in sys.path:
        sys.path.insert(0, src_root)

# Import config and core modules
from final_project.components.functions import *
from final_project.components.camera import FOVPyramid, TrackingController
from final_project.components.velocity_estimator import VelocityEstimator
from final_project.components.optitrack_helper import create_optitrack_reader
from final_project.state_machine.state_machine import StateMachine, StateType
from final_project.state_machine.state_idle import IdleState
from final_project.state_machine.state_tracking import TrackingState
from final_project.state_machine.state_searching import SearchingState
from final_project.components.trajectory_handler import *

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(name)s] %(levelname)s: %(message)s'
)
logger = logging.getLogger("MainControl")


# ============================================================================
# CONFIGURATION (sarà spostato in config.py in seguito)
# ============================================================================

CONFIG = {
    # FOV camera
    'fov_half_angle_deg': 45.0,
    'fov_max_range': 10.0,
    
    # Tracking gains (distanza e angolo/yaw)
    'tracking_kp_distance': 0.6,  
    'tracking_kp_angle': 0.2,    
    'tracking_max_velocity': 2.0,
    
    # State machine timeouts
    'idle_timeout': 10.0,
    'tracking_timeout': 120.0,
    'search_timeout': 30.0,
    'out_of_fov_timeout': 0.2,
    
    # Main loop
    # Deve coincidere con il campionamento della traiettoria del leader.
    'control_loop_period': 0.05,  # 20 Hz
    'max_loops': 1000,
}

# ============================================================================
# MAIN
# ============================================================================

def main():

    # ===== SETUP =====
    logger.info("[*] Setting up GenoM3 components...")
    
    setup()

    logger.info("[*] Starting motors and servo...")

    start()

    # ===== INITIALIZATION =====
    logger.info("[*] Initializing components...")

    # Camera FOV
    camera = FOVPyramid(
        half_angle_deg=CONFIG['fov_half_angle_deg'],
        max_range=CONFIG['fov_max_range']
    )

    # Velocity estimator
    vel_estimator = VelocityEstimator()

    # Tracking controller
    tracker = TrackingController(
        follower=maneuver_f,
        velocity_estimator=vel_estimator
    )

    # OptiTrack reader
    optitrack_reader = create_optitrack_reader(optitrack)

    logger.info("[*] Initializing state machine...")

    follower_bodies = ('QR4_leading', 'QR4_following')

    # Registra i tre stati
    idle_state = IdleState(
        camera=camera,
        optitrack_helper=optitrack_reader,
        follower_bodies=follower_bodies
    )

    tracking_state = TrackingState(
        camera=camera,
        tracker=tracker,
        optitrack_helper=optitrack_reader,
        follower_bodies=follower_bodies,
        kp_distance=CONFIG['tracking_kp_distance'],
        kp_angle=CONFIG['tracking_kp_angle'],
        max_velocity=CONFIG['tracking_max_velocity'],
        out_of_fov_timeout=CONFIG['out_of_fov_timeout'],
        tracking_timeout=CONFIG['tracking_timeout'],
    )

    searching_state = SearchingState(
        camera=camera,
        optitrack_helper=optitrack_reader,
        follower_bodies=follower_bodies,
        search_timeout=CONFIG['search_timeout'],
    )

    # State Machine
    sm = StateMachine(idle_state)
    sm.idle_state = idle_state
    sm.tracking_state = tracking_state
    sm.searching_state = searching_state

    # ===== SET VELOCITY LIMITS =====
    logger.info("[*] Setting velocity limits...")
    maneuver_f.set_velocity_limit(CONFIG['tracking_max_velocity'], 1)
    maneuver_l.set_velocity_limit(1.0, 1)
    logger.info(f"  Follower max velocity: {CONFIG['tracking_max_velocity']} m/s")
    logger.info(f"  Leader max velocity: 1.0 m/s")

    # ===== MAIN CONTROL LOOP =====
    logger.info("[*] Starting main control loop")
    logger.info(f"     Period: {CONFIG['control_loop_period']}s ({1/CONFIG['control_loop_period']:.0f} Hz)")
    logger.info(f"     Max loops: {CONFIG['max_loops']}")
    logger.info("=" * 80)

    graphics_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "graphics")
    os.makedirs(graphics_dir, exist_ok=True)
    pom_l.log_state(os.path.join(graphics_dir, "pom.log"))
    loop_count = 0
    next_t = time.monotonic()
    PERIOD = CONFIG['control_loop_period']
    maneuver_l.goto(-1,0,1,0,0)

    # select the trajectory for the leader
    th = TrajectoryHandler(
        period=PERIOD,
        duration=40.0,
        position_reader=optitrack_reader,
    )

    while loop_count < CONFIG['max_loops']:
        # Aggiorna la traiettoria del leader
        th.update_trajectory(maneuver_l)

        # Esegui ciclo di update della state machine
        current_state = sm.get_current_state()
        sm.update()

        # Log dello stato ogni N cicli
        if loop_count % 100 == 0:
            logger.debug(f"[Loop {loop_count:04d}] State: {current_state.value}")

        # Rate limiting
        next_t += PERIOD
        sleep = next_t - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.monotonic()

        loop_count += 1

    stop()
    th.plot_trajectories()
    logger.info("[+] Drone chase control finished")


if __name__ == "__main__":
    main()
