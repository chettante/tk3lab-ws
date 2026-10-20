#!/usr/bin/env python3
"""
Main script with state machine: IDLE → EXPLORING → TRACKING → SEARCHING.

Orchestration:
1. GenoM3 and drone setup
2. State machine initialization
3. Main loop running the state machine update cycles
4. Cleanup
"""

import os
import sys
import time
import logging
import numpy as np

# Allow direct execution from the project folder: python3 main_statemachine.py
if __package__ in (None, ""):
    project_root = os.path.dirname(os.path.abspath(__file__))
    src_root = os.path.dirname(project_root)
    if src_root not in sys.path:
        sys.path.insert(0, src_root)

# Import config and core modules
from final_project.components.functions import (
    setup, start, stop, random_trajectory, _quat_to_yaw,
    optitrack, pom_l, pom_f, maneuver_l, maneuver_f,
)
from final_project.components.camera import FOVPyramid, TrackingController
from final_project.components.velocity_estimator import VelocityEstimator
from final_project.components.optitrack_helper import create_optitrack_reader
from final_project.state_machine.state_machine import StateMachine
from final_project.state_machine.state_idle import IdleState
from final_project.state_machine.state_tracking import TrackingState
from final_project.state_machine.state_searching import SearchingState
from final_project.state_machine.state_exploring import ExploringState
from final_project.components.trajectory_handler import TrajectoryHandler
from final_project.components.leader_trajectories import build_leader_trajectory
from final_project.config import CONFIG, config_path
from final_project.components.Research import LeaderSearchManeuver
from final_project.graphics.trajectory_plotter import plot_trajectory

# Setup logging
logging.basicConfig(
    level=getattr(logging, str(CONFIG['log_level']).upper(), logging.INFO),
    format='[%(asctime)s] [%(name)s] %(levelname)s: %(message)s'
)
logger = logging.getLogger("MainControl")


def save_real_follower_path(path_points, period, filename=None):
    """Temporary function to save the actual follower path."""
    if not path_points:
        logger.warning("[TEMP] No follower path to plot.")
        return

    out_file = filename or config_path('plot_real_trajectory')
    plot_trajectory(path_points, dt=period, filename=out_file, show=False)
    logger.info(f"[TEMP] Actual follower path saved to {out_file}")


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

    # tracking acceleration from the tilt budget: a = g·tan(tilt)
    tracking_acc = 9.81 * np.tan(np.radians(CONFIG['tracking_max_tilt_deg']))

    # Tracking controller
    tracker = TrackingController(
    follower=maneuver_f,
    velocity_estimator=vel_estimator,
    kp_xy=CONFIG['tracking_kp_xy'],
    kp_z=CONFIG['tracking_kp_z'],
    follow_distance=CONFIG['tracking_follow_distance'],
    cmd_acc_max=tracking_acc,
    yaw_priority_start_deg=CONFIG['tracking_yaw_priority_start_deg'],
    yaw_priority_end_deg=CONFIG['fov_half_angle_h_deg'],   # priority ends at the FOV edge
    vprio_start_deg=CONFIG['tracking_vprio_start_deg'],
    vprio_end_deg=CONFIG['fov_half_angle_v_deg'],           # vertical priority ends at the FOV edge
    max_velocity=CONFIG['follower_max_velocity'],
    yaw_rate_max=CONFIG['max_yaw_rate'],
    )

    # Search strategy
    search = LeaderSearchManeuver(
           # neutral initial values: the plan is recomputed in SearchingState.update()
           p_follower0=np.zeros(3), yaw0=0.0, p_leader_last=CONFIG['leader_start_pos'],
           v_leader_last=np.zeros(3), dt=CONFIG['control_loop_period'],
           v_max=CONFIG['follower_max_velocity'], v_scan=CONFIG['search_v_scan'],
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

    # Create the four states
    idle_state = IdleState(
        camera=camera,
        optitrack_helper=optitrack_reader,
        follower_bodies=follower_bodies,
        idle_timeout=CONFIG['idle_timeout'],
    )

    tracking_state = TrackingState(
    camera=camera,
    tracker=tracker,
    optitrack_helper=optitrack_reader,
    follower_bodies=follower_bodies,
    kp_angle=CONFIG['tracking_kp_angle'],
    out_of_fov_timeout=CONFIG['out_of_fov_timeout'],
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
        max_velocity=CONFIG['follower_max_velocity'],
        max_acc=CONFIG['exploring_acc_max'],
        dt=CONFIG['control_loop_period'],
        explore_velocity=CONFIG['exploring_velocity'],
        explore_yaw_rate=CONFIG['exploring_yaw_rate'],
    )

    # State Machine
    sm = StateMachine(idle_state)
    sm.idle_state = idle_state
    sm.tracking_state = tracking_state
    sm.searching_state = searching_state
    sm.exploring_state = exploring_state

    # ===== SET VELOCITY LIMITS =====
    logger.info("[*] Setting velocity limits...")
    maneuver_l.set_velocity_limit(CONFIG['leader_max_velocity'], CONFIG['max_yaw_rate'])
    maneuver_l.set_acceleration_limit(CONFIG['leader_max_acc'], CONFIG['max_yaw_acc'])
    maneuver_f.set_velocity_limit(CONFIG['follower_max_velocity'], CONFIG['max_yaw_rate'])
    maneuver_f.set_acceleration_limit(CONFIG['follower_max_acc'], CONFIG['max_yaw_acc'])

    PERIOD = CONFIG['control_loop_period']

    # select the trajectory for the leader (th = None: random waypoints queued after takeoff)
    kind = CONFIG['leader_trajectory']
    p0 = np.array(CONFIG['leader_start_pos'], dtype=float)
    if kind == 'otto':
        th = TrajectoryHandler.from_speed(
            CONFIG['leader_speed'],
            period=PERIOD,
            A=CONFIG['traj_A'],
            B=CONFIG['traj_B'],
            H=CONFIG['traj_H'],
            P0=p0,                          # the figure-8 starts where the leader takes off
            position_reader=optitrack_reader,
            leader_body_name=CONFIG['leader_body'],
        )
        leader_peak_v = float(np.linalg.norm(th._vel, axis=0).max())
        leader_peak_a = float(np.linalg.norm(th._acc, axis=0).max())
    elif kind == 'random':
        th = None
        leader_peak_v, leader_peak_a = CONFIG['leader_max_velocity'], CONFIG['leader_max_acc']
    else:
        th = build_leader_trajectory(kind, p0, PERIOD, CONFIG)
        leader_peak_v, leader_peak_a = th.peak_speed, th.peak_acc

    tilt = CONFIG['tracking_max_tilt_deg']
    logger.info(f"  Leader: trajectory '{kind}', peak {leader_peak_v:.1f} m/s, {leader_peak_a:.1f} m/s² "
                f"(limits {CONFIG['leader_max_velocity']} m/s, {CONFIG['leader_max_acc']} m/s²)")
    logger.info(f"  Follower: limits {CONFIG['follower_max_velocity']} m/s, {CONFIG['follower_max_acc']} m/s², "
                f"tracking acc {tracking_acc:.2f} m/s² (tilt {tilt}°)")
    logger.info(f"  Max yaw rate (both): {CONFIG['max_yaw_rate']} rad/s")

    # consistency of the limits, with the velocity/acceleration the leader actually reaches
    if kind != 'random' and (leader_peak_v > CONFIG['leader_max_velocity']
                             or leader_peak_a > CONFIG['leader_max_acc']):
        logger.warning("  the trajectory exceeds the leader maneuver limits: "
                       "the leader will not follow it exactly")
    if CONFIG['follower_max_velocity'] < 1.3 * leader_peak_v:
        logger.warning(f"  follower_max_velocity={CONFIG['follower_max_velocity']} m/s < 1.3·leader peak "
                       f"velocity ({leader_peak_v:.1f} m/s): little margin to catch up with the leader")
    if tracking_acc < leader_peak_a:
        logger.warning(f"  tracking acceleration {tracking_acc:.2f} m/s² < leader peak acceleration "
                       f"{leader_peak_a:.1f} m/s²: the leader can pull away during its maneuvers")
    # rigid camera: tilting by `tilt` moves a leader at the same altitude by `tilt` in the image
    if tilt >= CONFIG['tracking_vprio_start_deg']:
        logger.warning(f"  tracking_max_tilt_deg={tilt}° >= tracking_vprio_start_deg="
                       f"{CONFIG['tracking_vprio_start_deg']}°: at full acceleration the approach is cut "
                       "even with the leader at the same altitude")

    pom_l.log_state(config_path('pom_leader_log'))
    pom_f.log_state(config_path('pom_follower_log'))
    loop_count = 0
    follower_path = []
    # takeoff: leader to leader_start_pos, follower straight up above its spawn (same yaw)
    pf, qf = optitrack_reader(CONFIG['follower_body'])
    targets = {CONFIG['leader_body']: np.array(CONFIG['leader_start_pos'], dtype=float),
               CONFIG['follower_body']: np.array([pf[0], pf[1], CONFIG['follower_takeoff_z']])}
    maneuver_f.goto(*targets[CONFIG['follower_body']], _quat_to_yaw(qf), 0, ack=True)
    maneuver_l.goto(*targets[CONFIG['leader_body']], CONFIG['leader_start_yaw'], 0)   # duration 0 = as fast as possible
    # the follower goto (ack=True) returns before the drone has arrived (the leader one blocks):
    # wait for the actual positions, then hover
    t_end = time.monotonic() + CONFIG['leader_takeoff_timeout']
    while time.monotonic() < t_end:
        ps = [optitrack_reader(b)[0] for b in targets]
        if all(p is not None and np.linalg.norm(np.asarray(p) - t) < CONFIG['leader_takeoff_tol']
               for p, t in zip(ps, targets.values())):
            break
        time.sleep(PERIOD)
    time.sleep(CONFIG['leader_takeoff_hold'])

    if th is None:
        # queue the waypoints in maneuver_l: maneuver executes them, without commands from the loop
        traj_time, final_point = random_trajectory(
            CONFIG['random_traj_points'], start=p0,
            v_max=CONFIG['leader_max_velocity'], a_max=CONFIG['leader_max_acc'],
            z_range=CONFIG['random_traj_z_range'], xy_range=CONFIG['random_traj_xy_range'],
            seed=CONFIG['random_seed'],
        )
    else:
        traj_time = th.duration

    # the loop ends traj_tail_time after the leader trajectory: for the random waypoints
    # when the leader reaches the last one (twice the estimated duration at most)
    tail_loops = int(round(CONFIG['traj_tail_time'] / PERIOD))
    max_loops = int(np.ceil((traj_time if th is not None else 2 * traj_time) / PERIOD)) + tail_loops
    end_loop = None
    stall_pos, stall_t = None, None     # leader stopped too long (random waypoints)

    # ===== MAIN CONTROL LOOP =====
    logger.info("[*] Starting main control loop")
    logger.info(f"     Period: {PERIOD}s ({1/PERIOD:.0f} Hz)")
    logger.info(f"     Leader trajectory: '{kind}', about {traj_time:.0f}s, "
                f"tail {CONFIG['traj_tail_time']}s, at most {max_loops} loops")
    logger.info("=" * 80)
    # the IDLE timeout starts now, not before the takeoff (which can last longer than idle_timeout)
    idle_state.enter_time = time.time()
    next_t = time.monotonic()

    try:
        while loop_count < max_loops and (end_loop is None or loop_count < end_loop):
            # Update the leader trajectory (random waypoints: already queued in maneuver)
            if th is not None:
                th.update_trajectory(maneuver_l)

            # Run one state machine update cycle
            sm.update()

            follower_pos, _ = optitrack_reader(CONFIG['follower_body'])
            if follower_pos is not None:
                follower_path.append(tuple(np.asarray(follower_pos, dtype=float)))

            # end of the leader trajectory: start the final tail
            if end_loop is None:
                if th is not None:
                    finished = th.is_complete
                else:
                    leader_pos, _ = optitrack_reader(CONFIG['leader_body'])
                    finished = (leader_pos is not None and np.linalg.norm(
                        np.asarray(leader_pos) - final_point) < CONFIG['leader_takeoff_tol'])
                    # leader stopped before the last waypoint (maneuver did not execute the queue):
                    # end the run instead of hovering until twice the estimated duration
                    if leader_pos is not None and not finished:
                        if stall_pos is None or np.linalg.norm(np.asarray(leader_pos) - stall_pos) > 0.2:
                            stall_pos, stall_t = np.asarray(leader_pos, dtype=float), time.monotonic()
                        elif time.monotonic() - stall_t > CONFIG['leader_stall_time']:
                            logger.warning(f"Leader still for {CONFIG['leader_stall_time']}s at "
                                           f"{np.round(stall_pos, 2)}, before the last waypoint "
                                           f"{np.round(final_point, 2)}: ending the run")
                            finished = True
                if finished:
                    end_loop = loop_count + tail_loops

            # Log the state every N cycles
            if loop_count % 100 == 0:
                logger.debug(f"[Loop {loop_count:04d}] State: {sm.get_current_state().value}")

            # Rate limiting
            next_t += PERIOD
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

            loop_count += 1
    finally:
        # also on exceptions: stop the drones and close the logs
        logger.info(f"[*] Loop ended after {loop_count} cycles, stopping the drones")
        stop()

    save_real_follower_path(follower_path, PERIOD, config_path('plot_real_trajectory'))

    logger.info("[+] Drone chase control finished")



if __name__ == "__main__":
    main()
