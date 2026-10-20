#!/usr/bin/env python3
"""
Aligns the camera of the Gazebo model with the FOV in the config:
  - <horizontal_fov> = 2 * fov_half_angle_h_deg          (in radians)
  - <height>         = <width> · tan(v) / tan(h)         (Gazebo derives the vertical
                       aperture from the image height/width ratio)
  - <far>            = fov_max_range                     (camera clip, m)
  - <pose> (yaw)     = camera_yaw_deg                    (camera mounting)
The image width stays the one already in the SDF (only the ratio matters).

In the world it also updates the follower spawn yaw, so that at startup the optical
axis points straight at the leader: yaw = bearing(follower → leader) − camera_yaw_deg.

Gazebo reads the SDF only at startup: quad_simulation.sh calls this before 'gz sim'.
The camera exists only in the follower model (mrsim-quadrotor): the leader uses
mrsim-quadrotor-leader, identical but without camera, so it is not rendered.
Changes to the drone physics must be made in both model.sdf files.
camera_gimbal has no effect on the SDF: in Gazebo the camera stays rigid on the body.

Usage:
    python3 sync_camera_fov.py [PATH_MODEL_SDF [PATH_WORLD]]
"""

import math
import os
import re
import sys

# allows direct execution reading final_project/config.py:
# walks up the folders until it finds the one containing final_project/ (i.e. src/)
_SRC_ROOT = os.path.dirname(os.path.abspath(__file__))
while not os.path.isfile(os.path.join(_SRC_ROOT, 'final_project', 'config.py')):
    parent = os.path.dirname(_SRC_ROOT)
    if parent == _SRC_ROOT:
        sys.exit("Cannot find final_project/config.py walking up from " + __file__)
    _SRC_ROOT = parent
if _SRC_ROOT not in sys.path:
    sys.path.insert(0, _SRC_ROOT)
from final_project.config import CONFIG

DEFAULT_SDF = os.path.join(os.path.dirname(_SRC_ROOT),
                           'gazebo', 'models', 'mrsim-quadrotor', 'model.sdf')
DEFAULT_WORLD = os.path.join(os.path.dirname(_SRC_ROOT), 'gazebo', 'worlds', 'example.world')


def _include_pose(world, name):
    """Match of the <pose> line of the <include> with <name>name</name>."""
    m = re.search(r'<include>(?:(?!</include>).)*?<name>' + re.escape(name) + r'</name>'
                  r'(?:(?!</include>).)*?(<pose>([^<]*)</pose>)', world, re.S)
    if m is None:
        sys.exit(f"No <include> with <name>{name}</name> and <pose> in the world")
    return m


def sync_spawn_yaw(path, leader='qr4_leading', follower='qr4_following'):
    """Follower spawn position from follower_spawn_mode (near/far) and
    yaw = bearing to the leader − camera mounting yaw."""
    with open(path) as f:
        world = f.read()
    pl = [float(v) for v in _include_pose(world, leader).group(2).split()]
    mf = _include_pose(world, follower)
    pf = [float(v) for v in mf.group(2).split()]
    mode = CONFIG['follower_spawn_mode']
    if mode not in ('near', 'far'):
        sys.exit(f"follower_spawn_mode must be 'near' or 'far', not {mode!r}")
    pf[0], pf[1] = CONFIG[f'follower_spawn_{mode}_xy']
    bearing = math.atan2(pl[1] - pf[1], pl[0] - pf[0])
    yaw = math.atan2(math.sin(bearing - math.radians(CONFIG['camera_yaw_deg'])),
                     math.cos(bearing - math.radians(CONFIG['camera_yaw_deg'])))
    pf[5] = yaw
    new = world[:mf.start(2)] + " ".join(f"{v:g}" for v in pf[:5]) + f" {yaw:.6f}" + world[mf.end(2):]
    if new != world:
        with open(path, 'w') as f:
            f.write(new)
    print(f"World {path}: spawn {follower} ({mode}) in ({pf[0]:g}, {pf[1]:g}), "
          f"yaw {math.degrees(yaw):.1f}° "
          f"(bearing to {leader} {math.degrees(bearing):.1f}° − camera {CONFIG['camera_yaw_deg']:g}°)")


def sync(path):
    h = CONFIG['fov_half_angle_h_deg']
    v = CONFIG['fov_half_angle_v_deg']
    rng = CONFIG['fov_max_range']
    fov_rad = math.radians(2 * h)

    with open(path) as f:
        sdf = f.read()

    # only inside the <sensor type="camera">...</sensor> block
    m = re.search(r'<sensor type="camera".*?</sensor>', sdf, re.S)
    if m is None:
        sys.exit(f"No <sensor type=\"camera\"> in {path}")
    cam = m.group(0)

    width = int(re.search(r'<width>\s*(\d+)\s*</width>', cam).group(1))
    height = max(1, round(width * math.tan(math.radians(v)) / math.tan(math.radians(h))))
    v_real = math.degrees(math.atan(math.tan(math.radians(h)) * height / width))

    cam = re.sub(r'<horizontal_fov>[^<]*</horizontal_fov>',
                 f'<horizontal_fov>{fov_rad:.6f}</horizontal_fov>', cam)
    cam = re.sub(r'<height>[^<]*</height>', f'<height>{height}</height>', cam)
    cam = re.sub(r'<far>[^<]*</far>', f'<far>{rng:g}</far>', cam)
    # mounting yaw: replaces the 6th value of the sensor <pose> (x y z roll pitch yaw)
    pm = re.search(r'(<pose>)([^<]*)(</pose>)', cam)
    pose = pm.group(2).split()
    pose[5] = f"{math.radians(CONFIG['camera_yaw_deg']):.6f}"
    cam = cam[:pm.start(2)] + " ".join(pose) + cam[pm.end(2):]
    cam = re.sub(r'<!-- Pose: .*?-->',
                 f"<!-- Pose: X Y Z Roll Pitch Yaw -> optical axis rotated by "
                 f"{CONFIG['camera_yaw_deg']:g}° around z relative to the drone x axis, from config.py -->", cam)
    # the patterns also match the old Italian comments, so older SDF files are updated too
    cam = re.sub(r'<!-- (?:FOV orizzontale totale|Total horizontal FOV):.*?-->',
                 f'<!-- Total horizontal FOV: {h:g}° left + {h:g}° right = {2*h:g}° '
                 f'({fov_rad:.6f} rad), from config.py -->', cam)
    cam = re.sub(r'<!-- Max range (?:di|of) .*?-->', f'<!-- Max range of {rng:g} meters, from config.py -->', cam)
    new = sdf[:m.start()] + cam + sdf[m.end():]
    new = re.sub(r'<!-- (?:TELECAMERA|CAMERA)[^(]*\((?:FOV TOTALE|TOTAL FOV) .*?\) -->',
                 f'<!-- CAMERA (TOTAL FOV {2*h:g}°x{2*v:g}°, RANGE {rng:g}M) -->', new)

    if new != sdf:
        with open(path, 'w') as f:
            f.write(new)
    print(f"Camera {path}: FOV {2*h:g}° x {2*v:g}° (image {width}x{height} → "
          f"effective vertical {2*v_real:.2f}°), range {rng:g} m")


if __name__ == '__main__':
    sync(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SDF)
    sync_spawn_yaw(sys.argv[2] if len(sys.argv) > 2 else DEFAULT_WORLD)
