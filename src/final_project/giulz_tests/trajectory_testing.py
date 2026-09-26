#!/usr/bin/env python3

import time
import numpy as np
from functions import *
from camera import *
from trajectories_generator import *

import numpy as np
from plotting import plot_trajectory


def main():
    """Main loop per demo FOV + tracking"""
    print("[*] Setup...")
    setup()
    print("[*] Start...")
    start()

    PERIOD = 0.05                      # 10 Hz
    T = 40.0                          # durata della traiettoria [s]
    n_steps = int(round(T / PERIOD))
    otto = []
    real_pos = []
    reference_port = nhfc_l.reference('trajectory_gen')  # porta di input del riferimento della traiettoria
    nhfc_l.connect_port({
        'local': 'reference', 'remote': 'trajectory_gen'
      })
    

    next_t = time.monotonic()         # serve SOLO a scandire il ciclo
    for k in range(n_steps + 1):      # +1: include il punto finale a t = T
        t = k * PERIOD                # tempo della traiettoria: 0, 0.1, ..., T
        p, v, a = generate_trajectory(t, T)
        trajectory = ({
                'reference': {
                'ts':        {'sec': t, 'nsec': 0},
                'intrinsic': 0,
                'pos':       {'x': p[0], 'y': p[1], 'z': p[2]},
                'att':       {'qw': 0, 'qx': 0, 'qy': 0, 'qz': 0},
                'vel':       {'vx': v[0], 'vy': v[1], 'vz': v[2]},
                'avel':      {'wx': 0, 'wy': 0, 'wz': 0},
                'acc':       {'ax': a[0], 'ay': a[1], 'az': a[2]},
                'aacc':      None,
                'jerk':      None,
                'snap':      None,
                    }
                })
        otto.append((p[0], p[1], p[2]))
        leader_body = optitrack.bodies('QR4_leading')
        pos_leader = np.array([
                leader_body['bodies']['pos']['x'],
                leader_body['bodies']['pos']['y'],
                leader_body['bodies']['pos']['z']])
        real_pos.append((pos_leader[0], pos_leader[1], pos_leader[2]))
        reference_port(trajectory)

        print(maneuver_l.desired())

        next_t += PERIOD
        sleep = next_t - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.monotonic()   # overrun: resync

    plot_trajectory(otto, dt=PERIOD, filename="otto.png", show=True)
    plot_trajectory(real_pos, dt=PERIOD, filename="real_pos.png", show=True)


if __name__ == "__main__":

    main()
    pom_l.log_stop()
