"""
Benchmark scenarios (no gimbal). Every value not listed here comes from config.py and stays
the same in every run: follower limits, gains, follow distance, timeouts, camera.

Each scenario: id, phase, label, x (value on the x axis of the phase plot), overrides of CONFIG.
"""

# parametric trajectories: leader maneuver limits high enough not to cut the trajectory
_PARAM = {'leader_max_velocity': 15.0, 'leader_max_acc': 8.0, 'follower_spawn_mode': 'near'}
# figure-8 20 m (y) x 10 m (x): with the default 10 m x 5 m the curves need more acceleration
# than the leader has already at 4 m/s
_OTTO = {'traj_A': 10.0, 'traj_B': 10.0}

# random waypoints: the leader flies at its maneuver limits = aggressiveness levels
LEVELS = {
    1: (5.0, 1.5),     # slower than the follower (tracking 3.6 m/s² per axis, tilt 20°)
    2: (8.0, 3.0),     # about the same acceleration
    3: (12.0, 5.0),    # about twice, at the follower velocity margin (15/1.3)
    4: (15.0, 8.0),    # outside the envelope
}


def _random(level, seed, xy, spawn):
    v, a = LEVELS[level]
    return {'leader_trajectory': 'random', 'leader_max_velocity': v, 'leader_max_acc': a,
            'random_seed': seed, 'random_traj_xy_range': xy, 'random_traj_z_range': [5.0, 10.0],
            'random_traj_points': 20, 'follower_spawn_mode': spawn}


SCENARIOS = [
    # A - reference: hover and straight line (leader ramps at 2 m/s² < follower 3.6 m/s²)
    dict(id='A01', phase='A', label='hover', x=0.0,
         overrides={**_PARAM, 'leader_trajectory': 'hover', 'hover_time': 15.0}),
    *[dict(id=f'A{i + 2:02d}', phase='A', label=f'line {v:g} m/s', x=v,
           overrides={**_PARAM, 'leader_trajectory': 'line', 'leader_speed': v, 'line_length': L})
      for i, (v, L) in enumerate(((3.0, 50.0), (6.0, 100.0), (10.0, 150.0)))],

    # B - curves: circle R = 10 m (limit v = sqrt(3.6·10) ≈ 6 m/s) and flat figure-8
    *[dict(id=f'B{i + 1:02d}', phase='B', label=f'circle {v:g} m/s', x=v,
           overrides={**_PARAM, 'leader_trajectory': 'circle', 'leader_speed': v,
                      'circle_radius': 10.0, 'circle_laps': 1.5})
      for i, v in enumerate((3.0, 5.0, 7.0))],
    *[dict(id=f'B{i + 4:02d}', phase='B', label=f'figure-8 {v:g} m/s', x=v,
           overrides={**_PARAM, **_OTTO, 'leader_trajectory': 'otto', 'leader_speed': v, 'traj_H': 0.0})
      for i, v in enumerate((2.0, 3.0))],

    # C - acceleration across vs along the line of sight, same values: slalom at 4 m/s with peak
    # lateral acceleration 2 / 4 / 6 m/s², shuttle (back and forth at 6 m/s, stop at each end)
    # with ramps at 2 / 4 / 6 m/s². Tracking limit: 3.6 m/s² per axis (tilt 20°)
    *[dict(id=f'C{i + 1:02d}', phase='C', label=f'slalom {a:g} m/s²', x=a,
           overrides={**_PARAM, 'leader_trajectory': 'slalom', 'leader_speed': 4.0, 'slalom_lat_acc': a})
      for i, a in enumerate((2.0, 4.0, 6.0))],
    *[dict(id=f'C{i + 4:02d}', phase='C', label=f'shuttle {a:g} m/s²', x=a,
           overrides={**_PARAM, 'leader_trajectory': 'shuttle', 'leader_speed': 6.0, 'leader_traj_acc': a})
      for i, a in enumerate((2.0, 4.0, 6.0))],

    # D - aggressiveness: random waypoints with stops, 4 levels x 3 seeds
    *[dict(id=f'D{lv}{s}', phase='D', label=f'level {lv} seed {s}', x=lv,
           overrides=_random(lv, s, 30.0, 'near'))
      for lv in LEVELS for s in (1, 2, 3)],

    # E - search: follower spawned far, leader inside the exploration area
    *[dict(id=f'E{lv}{s}', phase='E', label=f'level {lv} seed {s}', x=lv,
           overrides=_random(lv, s, 10.0, 'far'))
      for lv in (1, 2) for s in (1, 2)],
]
