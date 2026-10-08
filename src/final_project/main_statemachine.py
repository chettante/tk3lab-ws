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
from final_project.state_machine.state_exploring import ExploringState
from final_project.components.trajectory_handler import *
from final_project.config import CONFIG, config_path
from final_project.components.Research import *
from final_project.graphics.trajectory_plotter import plot_trajectory

# Setup logging
logging.basicConfig(
    level=getattr(logging, str(CONFIG['log_level']).upper(), logging.INFO),
    format='[%(asctime)s] [%(name)s] %(levelname)s: %(message)s'
)
logger = logging.getLogger("MainControl")


def save_real_follower_path(path_points, period, filename=None):
    """Funzione temporanea per salvare il percorso reale del follower."""
    if not path_points:
        logger.warning("[TEMP] Nessun percorso del follower da plottare.")
        return

    out_file = filename or config_path('plot_real_trajectory')
    plot_trajectory(path_points, dt=period, filename=out_file, show=False)
    logger.info(f"[TEMP] Percorso reale del follower salvato in {out_file}")


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
        half_angle_h_deg=CONFIG['fov_half_angle_h_deg'],
        half_angle_v_deg=CONFIG['fov_half_angle_v_deg'],
        max_range=CONFIG['fov_max_range'],
        gimbal=CONFIG['camera_gimbal'],
        yaw_offset_deg=CONFIG['camera_yaw_deg'],
    )

    # Velocity estimator
    vel_estimator = VelocityEstimator()

    # Tracking controller
    tracker = TrackingController(
    follower=maneuver_f,
    velocity_estimator=vel_estimator,
    kp_xy=CONFIG['tracking_kp_xy'],
    kp_z=CONFIG['tracking_kp_z'],
    follow_distance=CONFIG['tracking_follow_distance'],
    cmd_acc_max=CONFIG['tracking_cmd_acc_max'],
    yaw_priority_start_deg=CONFIG['tracking_yaw_priority_start_deg'],
    yaw_priority_end_deg=CONFIG['fov_half_angle_h_deg'],   # fine priorità = bordo del FOV
    max_velocity=CONFIG['max_velocity'],                  # stessi limiti del leader
    yaw_rate_max=CONFIG['max_yaw_rate'],
    )

    #Research Strategy
      
    search = LeaderSearchManeuver(
           p_follower0 = CONFIG['follower_start_pos'], yaw0=CONFIG['follower_start_yaw'], p_leader_last=CONFIG['leader_start_pos'], v_leader_last=CONFIG['leader_start_velocity'], dt=CONFIG['control_loop_period'],
           v_max=CONFIG['max_velocity'], v_scan=CONFIG['search_v_scan'],
           delta_z_global=CONFIG['search_delta_z_global'], omega_scan=CONFIG['search_omega_scan'],
           yaw_rate_max=CONFIG['max_yaw_rate'], t_hold=CONFIG['search_t_hold'],
           a_max=CONFIG['search_a_max'], yaw_acc_max=CONFIG['search_yaw_acc_max'],
           T_rec=CONFIG['search_T_rec'], approach_dist=CONFIG['search_approach_dist'],
           z_min=CONFIG['search_z_min'], z_max=CONFIG['search_z_max'], p_home=CONFIG['search_home'],
       )
    

    # OptiTrack reader
    optitrack_reader = create_optitrack_reader(optitrack)

    logger.info("[*] Initializing state machine...")

    follower_bodies = (CONFIG['leader_body'], CONFIG['follower_body'])

    # Registra i tre stati
    idle_state = IdleState(
        camera=camera,
        optitrack_helper=optitrack_reader,
        follower_bodies=follower_bodies,
        idle_timeout=CONFIG['idle_timeout'],
        maneuver_follower=maneuver_f,
        spawn_lift_height=1.0,
    )

    tracking_state = TrackingState(
    camera=camera,
    tracker=tracker,
    optitrack_helper=optitrack_reader,
    follower_bodies=follower_bodies,
    kp_angle=CONFIG['tracking_kp_angle'],
    max_acc=CONFIG['tracking_max_acc'],
    out_of_fov_timeout=CONFIG['out_of_fov_timeout'],
    tracking_timeout=CONFIG['tracking_timeout'],
    )

    searching_state = SearchingState(
        camera=camera,
        search=search,
        vel_estimator = vel_estimator,
        tracking_state=tracking_state,
        optitrack_helper=optitrack_reader,
        follower_bodies=follower_bodies,
        maneuver_follower=maneuver_f,
    )

    exploring_state = ExploringState(
        camera=camera,
        optitrack_helper=optitrack_reader,
        follower_bodies=follower_bodies,
        maneuver_follower=maneuver_f,
        explore_bounds=CONFIG['explore_bounds'],
        ring_clearance=CONFIG['explore_ring_clearance'],
        top_clearance=CONFIG['explore_top_clearance'],
        max_velocity=CONFIG['max_velocity'],
        dt=CONFIG['control_loop_period'],
        explore_timeout=CONFIG['exploring_timeout'],
        explore_velocity=CONFIG['exploring_velocity'],
        explore_yaw_rate=CONFIG['exploring_yaw_rate'],
    )

    # State Machine
    sm = StateMachine(idle_state)
    idle_state.enter_time = time.time()
    sm.idle_state = idle_state
    sm.tracking_state = tracking_state
    sm.searching_state = searching_state
    sm.exploring_state = exploring_state

    # ===== SET VELOCITY LIMITS =====
    logger.info("[*] Setting velocity limits...")
    # droni identici: stessi limiti (lineare m/s, angolare rad/s) per leader e follower
    for m in (maneuver_l, maneuver_f):
        m.set_velocity_limit(CONFIG['max_velocity'], CONFIG['max_yaw_rate'])
    logger.info(f"  Max velocity (both): {CONFIG['max_velocity']} m/s, "
                f"max yaw rate: {CONFIG['max_yaw_rate']} rad/s")

    PERIOD = CONFIG['control_loop_period']

    

    # select the trajectory for the leader
    th = TrajectoryHandler(
        period=PERIOD,
        duration=CONFIG['traj_duration'],
        A=CONFIG['traj_A'],
        B=CONFIG['traj_B'],
        H=CONFIG['traj_H'],
        P0=np.array(CONFIG['traj_P0'], dtype=float),
        position_reader=optitrack_reader,
        leader_body_name=CONFIG['leader_body'],
    )
    if th.cruise_speed > CONFIG['max_velocity']:
        logger.warning(
            f"  traj_duration={CONFIG['traj_duration']}s richiede {th.cruise_speed:.2f} m/s "
            f"> max_velocity={CONFIG['max_velocity']} m/s: il leader non riuscirà "
            "a seguire la traiettoria (aumenta traj_duration o max_velocity)"
        )

    # il loop dura quanto la traiettoria del leader + una coda finale
    random_traj = CONFIG['leader_random_trajectory']
    traj_time = (15.0 * CONFIG['random_traj_points'] + 10.0) if random_traj else CONFIG['traj_duration']
    max_loops = int(np.ceil((traj_time + CONFIG['traj_tail_time']) / PERIOD))

    # ===== MAIN CONTROL LOOP =====
    logger.info("[*] Starting main control loop")
    logger.info(f"     Period: {PERIOD}s ({1/PERIOD:.0f} Hz)")
    if random_traj:
        logger.info(f"     Leader trajectory: RANDOM, {CONFIG['random_traj_points']} waypoints "
                    f"({traj_time:.0f}s), tail {CONFIG['traj_tail_time']}s -> {max_loops} loops")
    else:
        logger.info(f"     Leader trajectory: {CONFIG['traj_duration']}s "
                    f"(cruise {th.cruise_speed:.2f} m/s), tail {CONFIG['traj_tail_time']}s "
                    f"-> {max_loops} loops")
    logger.info("=" * 80)

    pom_l.log_state(config_path('pom_leader_log'))
    pom_f.log_state(config_path('pom_follower_log'))
    loop_count = 0
    follower_path = []
    x0, y0, z0 = CONFIG['leader_start_pos']
    maneuver_l.goto(x0, y0, z0, CONFIG['leader_start_yaw'], CONFIG['leader_start_duration'])
    
    if random_traj:
        # accoda i waypoint in maneuver_l: vengono eseguiti da maneuver, senza comandi dal loop
        random_trajectory(CONFIG['random_traj_points'])
    next_t = time.monotonic()

    try:
        while loop_count < max_loops:
            #nhfc_f.servo(ack=True)
            #nhfc_l.servo(ack=True)
            # Aggiorna la traiettoria del leader (solo per l'otto)
            if not random_traj:
                th.update_trajectory(maneuver_l)

            # Esegui ciclo di update della state machine
            current_state = sm.get_current_state()
            sm.update()

            follower_pos, _ = optitrack_reader(CONFIG['follower_body'])
            if follower_pos is not None:
                follower_path.append(tuple(np.asarray(follower_pos, dtype=float)))

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
    finally:
        # anche in caso di eccezione: ferma i droni e chiudi i log
        logger.info(f"[*] Loop terminato dopo {loop_count} cicli, stop dei droni")
        stop()

    save_real_follower_path(follower_path, PERIOD, config_path('plot_real_trajectory'))

    if not random_traj:     # con i waypoint casuali non c'è una traiettoria pianificata da plottare
        #th.plot_trajectories(
            planned_path=config_path('plot_planned_trajectory'),
            real_path=config_path('plot_real_trajectory'),
            show=CONFIG['show_plots'],
        #)
    logger.info("[+] Drone chase control finished")
    


if __name__ == "__main__":
    main()