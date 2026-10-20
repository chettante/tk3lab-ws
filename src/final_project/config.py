"""
Central project configuration.

Edit the values below and save: they are loaded automatically the next time
main_statemachine.py (and the plotters in graphics/) start.

Paths:
  - *relative* file names (e.g. 'pom.log') are taken relative to 'output_dir'
  - *absolute* paths (e.g. '/home/user/run1/pom.log') are used as they are
  - '~' is expanded; missing folders are created
"""

import os

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

CONFIG = {
    # ------------------------------------------------------------------
    # Camera FOV (pinhole, as in Gazebo: sync_camera_fov.py copies these values into model.sdf)
    # ------------------------------------------------------------------
    # half-angles = angle between the optical axis and the side (h) / top-bottom (v) faces
    'fov_half_angle_h_deg': 43.5,   # horizontal (left/right)
    'fov_half_angle_v_deg': 29.0,  # vertical (up/down)
    'fov_max_range': 10.0,          # m, far plane (depth along the optical axis)
    'camera_yaw_deg': 45.0,         # camera mounting: yaw relative to the drone x axis
                                    # (+45° = bisector between arm 1 front/red and arm 2)
    'camera_gimbal': False,         # True = stabilized camera (yaw only),
                                    # False = camera rigidly fixed to the body (roll/pitch/yaw), as in Gazebo

    # ------------------------------------------------------------------
    # Tracking controller, leader centered in the FOV
    # ------------------------------------------------------------------
    # P + feedforward: v = v_leader + kp·(target − p), target = follow_distance
    # behind the leader along the bearing. Velocity/yaw limits: follower_max_velocity
    # and max_yaw_rate (nhfc section).
    'tracking_kp_xy': 0.3,
    'tracking_kp_z': 0.6,
    'tracking_follow_distance': 3.0,  # m, horizontal distance from the leader (along the bearing);
                                      # larger = the bearing turns more slowly when the leader curves
    'tracking_kp_angle': 1.5,       # 1/s: steady-state yaw lag ≈ bearing rate / kp_angle
    # tracking acceleration as a tilt budget: the camera is rigid, so every degree of
    # follower tilt moves the leader by one degree in the image (at any distance).
    # The main computes a = g·tan(tilt) and uses it both for the command rate limiter
    # and as the ax, ay, az passed to maneuver.velocity(). 6° ≈ 1.0 m/s², 10° ≈ 1.7 m/s², 15° ≈ 2.6 m/s²
    'tracking_max_tilt_deg': 10.0,
    # yaw priority: the position (xy) correction decreases linearly from 100% at
    # |yaw error| = yaw_priority_start down to 0% at the FOV edge (fov_half_angle_h_deg)
    'tracking_yaw_priority_start_deg': 25.0,
    # vertical priority: beyond this vertical angle of the leader (camera frame) the in-plane
    # approach decreases to 0 at the edge (fov_half_angle_v_deg); when too close it backs off.
    # Approaching from close range makes the vertical angle (atan(Δz/d)) grow a lot, from far almost not at all
    'tracking_vprio_start_deg': 23.0,

    # ------------------------------------------------------------------
    # State machine timeouts
    # ------------------------------------------------------------------
    'idle_timeout': 5.0,
    'exploring_velocity': 10.0,
    'exploring_yaw_rate': 2.0,
    'exploring_acc_max': 1.5,       # m/s², peak acceleration of the connector segment to the ring
    'explore_bounds': ((-10.0, 10.0), (-10.0, 10.0), (5.0, 15.0)),
    'explore_ring_clearance': 1.0,
    'explore_top_clearance': 1.0,
    'out_of_fov_timeout': 2.0,      # grace period with prediction

    # ------------------------------------------------------------------
    # Control loop and terminal logging
    # ------------------------------------------------------------------
    'control_loop_period': 0.05,
    'log_level': 'INFO',            # levels: DEBUG, INFO, WARNING, ERROR

    # ------------------------------------------------------------------
    # Low-level flight control (nhfc), IDENTICAL for leader and follower
    # ------------------------------------------------------------------
    # Kp* position, Kv* velocity, Kq* attitude (xy = roll/pitch, z = yaw),
    # Kw* angular velocity, Ki* integral. Yaw: ψ'' = -Kqz·e - Kwz·e'
    # (original values: 1.0/1.4 on yaw made the run worse, probably by taking
    # motor margin away from translation)
    'nhfc_gains': {
        'Kpxy': 5, 'Kpz': 5, 'Kqxy': 4, 'Kqz': 0.1,
        'Kvxy': 6, 'Kvz': 6, 'Kwxy': 1, 'Kwz': 0.1,
        'Kixy': 0, 'Kiz': 0,
    },
    # drone limits (maneuver). The leader cruises at leader_speed (Leader section);
    # the follower must be able to overtake it: follower_max_velocity ≳ 1.3·leader_speed
    'leader_max_velocity': 50.0,     # m/s, leader maneuver limit (≥ leader_speed)
    'leader_max_acc': 50.0,          # m/s², leader maneuver limit
    'follower_max_velocity': 50.0,   # m/s, follower maneuver limit + tracking/exploring/search saturation
    'max_yaw_rate': 2.0,            # rad/s, both drones
    'max_yaw_acc': 2.0,             # rad/s², leader angular acceleration limit
    # flight volume for the maneuver planner: 220 m x 220 m x 50 m
    'maneuver_bounds': {
        'xmin': -110, 'xmax': 110,
        'ymin': -110, 'ymax': 110,
        'zmin': 0,   'zmax': 50,
        'yawmin': -3.14, 'yawmax': 3.14,
    },

    # ------------------------------------------------------------------
    # Drones / OptiTrack
    # ------------------------------------------------------------------
    'leader_body': 'QR4_leading',
    'follower_body': 'QR4_following',

    # follower spawn in Gazebo (the leader starts at (-1, 0) in the world). Applied by
    # sync_camera_fov.py when quad_simulation.sh starts, together with the yaw that points
    # the camera at the leader: the simulation must be restarted to change it.
    #   'near' = leader in the FOV at spawn: goes straight to TRACKING (tests the controller)
    #   'far'  = leader out of reach: IDLE -> EXPLORING -> ... (tests the exploration)
    'follower_spawn_mode': 'far',
    'follower_spawn_near_xy': (-3.0, 0.0),
    'follower_spawn_far_xy': (20.0, 5.0),

    # ------------------------------------------------------------------
    # Leader: initial position and figure-8 trajectory
    # ------------------------------------------------------------------
    'leader_start_pos': (-1.0, 0.0, 5.0),   # initial goto (x, y, z) [m]
    'leader_start_yaw': 0.0,                # rad
    'follower_takeoff_z': 1.0,              # m, the follower takes off vertically above its spawn together with the leader
    'leader_takeoff_tol': 0.15,             # m, distance from the takeoff target (both drones)
    'leader_takeoff_timeout': 5.0,         # s, after this the run starts anyway
    'leader_takeoff_hold': 2.0,             # s, hover on leader_start_pos before the trajectory

    # leader cruise speed, both on the figure-8 and on the random waypoints:
    # the trajectory duration is derived from it
    'leader_speed': 40.0,                    # m/s

    'leader_random_trajectory': True,       # TEMP: random waypoints (random_trajectory) instead of the figure-8
    'random_traj_points': 100,               # number of random waypoints (+ one final point at rest)
    'random_traj_z_range': (5.0, 15.0),      # m, waypoint altitudes (inside explore_bounds)
    'random_traj_xy_range': 100.0,           # m, x and y drawn in [-range, +range]

    'traj_tail_time': 15.0,                 # s, the main keeps running for this long after the trajectory ends
    'traj_A': 5.0,                          # amplitude along y [m]
    'traj_B': 5.0,                          # amplitude along x (semi-axis = B/2) [m]
    'traj_H': 5.0,                          # maximum altitude above P0 [m]

    # ------------------------------------------------------------------
    # Follower: search strategy parameters
    # ------------------------------------------------------------------
    # recovery: one segment towards the PREDICTED leader position
    # p_pred = p_last + v_last·(t_lost + T): turns to center it, moves closer
    # if it is beyond approach_dist, goes to its predicted altitude
    'search_T_rec': 0.2,            # s, minimum recovery duration and prediction horizon
    'search_approach_dist': 3.0,    # m, horizontal distance from the predicted leader (< fov_max_range)
    'search_z_min': 5.5,            # m, altitude limits during recovery
    'search_z_max': 15.0,
    # global scan: home, climb by delta_z_global while spinning, descend while spinning
    'search_home': (0.0, 0.0, 5.0),
    'search_delta_z_global': 3.0,   # top = search_home.z + delta, aligned with the exploration ceiling
    'search_v_scan': 5.0,           # m/s, peak velocity while climbing/descending
    'search_omega_scan': 2.0,       # rad/s, rotation during the scan
    'search_t_hold': 1.0,           # s, duration of each hover
    'search_a_max': 1.5,            # m/s², peak acceleration of the search segments
    'search_yaw_acc_max': 2.0,      # rad/s², peak angular acceleration during recovery


    # ------------------------------------------------------------------
    # Paths: logs and images
    # ------------------------------------------------------------------
    'output_dir': os.path.join(PROJECT_ROOT, 'graphics'),

    # logs saved by main_statemachine.py (relative to output_dir)
    'pom_leader_log': 'log/pom.log',
    'pom_follower_log': 'log/pom_f.log',     # GenoM accepts paths of at most 64 characters
    'velocity_log': 'log/velocity_log.txt',

    # images (relative to output_dir)
    'plot_planned_trajectory': 'plot/otto.png',
    'plot_real_trajectory': 'plot/real_pos.png',
    'plot_velocity': 'plot/velocity_comparison.png',
    'plot_tracking': 'plot/tracking_analysis.png',
    'show_plots': True,             # False on machines without a display

    # folder for the internal logs of the GenoM3 components (pom/rotorcraft/nhfc/maneuver/optitrack)
    'genom_log_dir': '/tmp',
}


# ============================================================================
# Path helpers (no need to edit them)
# ============================================================================

def resolve_output(path):
    """Absolute path of `path`: if relative, it is taken relative to output_dir.
    Creates the destination folder if missing."""
    p = os.path.expanduser(str(path))
    if not os.path.isabs(p):
        p = os.path.join(os.path.expanduser(CONFIG['output_dir']), p)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    return p


def config_path(key):
    """Absolute path of the file given by CONFIG[key] (see resolve_output)."""
    return resolve_output(CONFIG[key])


def genom_log_path(filename):
    """Path of an internal GenoM3 log inside 'genom_log_dir'."""
    d = os.path.expanduser(CONFIG['genom_log_dir'])
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, filename)
