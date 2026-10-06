#!/usr/bin/env python3
"""
Allinea la camera del modello Gazebo al FOV del config:
  - <horizontal_fov> = 2 * fov_half_angle_h_deg          (in radianti)
  - <height>         = <width> · tan(v) / tan(h)         (Gazebo ricava l'apertura
                       verticale dal rapporto altezza/larghezza dell'immagine)
  - <far>            = fov_max_range                     (clip della camera, m)
  - <pose> (yaw)     = camera_yaw_deg                    (montaggio della camera)
La larghezza dell'immagine resta quella già presente nell'SDF (conta solo il rapporto).

Nel world aggiorna anche lo yaw di spawn del follower, così che all'avvio l'asse
ottico punti dritto sul leader: yaw = bearing(follower → leader) − camera_yaw_deg.

Gazebo legge l'SDF solo all'avvio: quad_simulation.sh lo chiama prima di 'gz sim'.
La camera c'è solo nel modello del follower (mrsim-quadrotor): il leader usa
mrsim-quadrotor-leader, identico ma senza camera, per non renderizzarla.
Le modifiche alla fisica del drone vanno fatte in entrambi i model.sdf.
camera_gimbal non ha effetto sull'SDF: in Gazebo la camera resta rigida sul corpo.

Uso:
    python3 sync_camera_fov.py [PATH_MODEL_SDF [PATH_WORLD]]
"""

import math
import os
import re
import sys

# permette l'esecuzione diretta leggendo final_project/config.py:
# risale le cartelle finché trova quella che contiene final_project/ (cioè src/)
_SRC_ROOT = os.path.dirname(os.path.abspath(__file__))
while not os.path.isfile(os.path.join(_SRC_ROOT, 'final_project', 'config.py')):
    parent = os.path.dirname(_SRC_ROOT)
    if parent == _SRC_ROOT:
        sys.exit("Impossibile trovare final_project/config.py risalendo da " + __file__)
    _SRC_ROOT = parent
if _SRC_ROOT not in sys.path:
    sys.path.insert(0, _SRC_ROOT)
from final_project.config import CONFIG

DEFAULT_SDF = os.path.join(os.path.dirname(_SRC_ROOT),
                           'gazebo', 'models', 'mrsim-quadrotor', 'model.sdf')
DEFAULT_WORLD = os.path.join(os.path.dirname(_SRC_ROOT), 'gazebo', 'worlds', 'example.world')


def _include_pose(world, name):
    """Match della riga <pose> dell'<include> con <name>name</name>."""
    m = re.search(r'<include>(?:(?!</include>).)*?<name>' + re.escape(name) + r'</name>'
                  r'(?:(?!</include>).)*?(<pose>([^<]*)</pose>)', world, re.S)
    if m is None:
        sys.exit(f"Nessun <include> con <name>{name}</name> e <pose> nel world")
    return m


def sync_spawn_yaw(path, leader='qr4_leading', follower='qr4_following'):
    """Yaw di spawn del follower = bearing verso il leader − yaw di montaggio della camera."""
    with open(path) as f:
        world = f.read()
    pl = [float(v) for v in _include_pose(world, leader).group(2).split()]
    mf = _include_pose(world, follower)
    pf = [float(v) for v in mf.group(2).split()]
    bearing = math.atan2(pl[1] - pf[1], pl[0] - pf[0])
    yaw = math.atan2(math.sin(bearing - math.radians(CONFIG['camera_yaw_deg'])),
                     math.cos(bearing - math.radians(CONFIG['camera_yaw_deg'])))
    pf[5] = yaw
    new = world[:mf.start(2)] + " ".join(f"{v:g}" for v in pf[:5]) + f" {yaw:.6f}" + world[mf.end(2):]
    if new != world:
        with open(path, 'w') as f:
            f.write(new)
    print(f"World {path}: spawn {follower} yaw {math.degrees(yaw):.1f}° "
          f"(bearing verso {leader} {math.degrees(bearing):.1f}° − camera {CONFIG['camera_yaw_deg']:g}°)")


def sync(path):
    h = CONFIG['fov_half_angle_h_deg']
    v = CONFIG['fov_half_angle_v_deg']
    rng = CONFIG['fov_max_range']
    fov_rad = math.radians(2 * h)

    with open(path) as f:
        sdf = f.read()

    # solo dentro il blocco <sensor type="camera">...</sensor>
    m = re.search(r'<sensor type="camera".*?</sensor>', sdf, re.S)
    if m is None:
        sys.exit(f"Nessun <sensor type=\"camera\"> in {path}")
    cam = m.group(0)

    width = int(re.search(r'<width>\s*(\d+)\s*</width>', cam).group(1))
    height = max(1, round(width * math.tan(math.radians(v)) / math.tan(math.radians(h))))
    v_real = math.degrees(math.atan(math.tan(math.radians(h)) * height / width))

    cam = re.sub(r'<horizontal_fov>[^<]*</horizontal_fov>',
                 f'<horizontal_fov>{fov_rad:.6f}</horizontal_fov>', cam)
    cam = re.sub(r'<height>[^<]*</height>', f'<height>{height}</height>', cam)
    cam = re.sub(r'<far>[^<]*</far>', f'<far>{rng:g}</far>', cam)
    # yaw di montaggio: sostituisce il 6° valore della <pose> del sensore (x y z roll pitch yaw)
    pm = re.search(r'(<pose>)([^<]*)(</pose>)', cam)
    pose = pm.group(2).split()
    pose[5] = f"{math.radians(CONFIG['camera_yaw_deg']):.6f}"
    cam = cam[:pm.start(2)] + " ".join(pose) + cam[pm.end(2):]
    cam = re.sub(r'<!-- Pose: .*?-->',
                 f"<!-- Pose: X Y Z Roll Pitch Yaw -> asse ottico ruotato di "
                 f"{CONFIG['camera_yaw_deg']:g}° attorno a z rispetto all'asse x del drone, da config.py -->", cam)
    cam = re.sub(r'<!-- FOV orizzontale totale:.*?-->',
                 f'<!-- FOV orizzontale totale: {h:g}° a sx + {h:g}° a dx = {2*h:g}° '
                 f'({fov_rad:.6f} rad), da config.py -->', cam)
    cam = re.sub(r'<!-- Max range di .*?-->', f'<!-- Max range di {rng:g} metri, da config.py -->', cam)
    new = sdf[:m.start()] + cam + sdf[m.end():]
    new = re.sub(r'<!-- TELECAMERA[^(]*\(FOV TOTALE .*?\) -->',
                 f'<!-- TELECAMERA (FOV TOTALE {2*h:g}°x{2*v:g}°, RANGE {rng:g}M) -->', new)

    if new != sdf:
        with open(path, 'w') as f:
            f.write(new)
    print(f"Camera {path}: FOV {2*h:g}° x {2*v:g}° (immagine {width}x{height} → "
          f"verticale effettiva {2*v_real:.2f}°), range {rng:g} m")


if __name__ == '__main__':
    sync(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SDF)
    sync_spawn_yaw(sys.argv[2] if len(sys.argv) > 2 else DEFAULT_WORLD)
