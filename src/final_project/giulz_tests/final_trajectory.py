#!/usr/bin/env python3

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


from final_project.components.functions import *
from final_project.components.camera import *
from final_project.trajectories_generator import *
from final_project.graphics.trajectory_plotter import plot_trajectory

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

    next_t = time.monotonic()         # serve SOLO a scandire il ciclo
    for k in range(n_steps + 1):      # +1: include il punto finale a t = T
        t = k * PERIOD                # tempo della traiettoria: 0, 0.1, ..., T
        p, v, a = generate_trajectory(t, T)
        otto.append((p[0], p[1], p[2]))
        leader_body = optitrack.bodies('QR4_leading')
        pos_leader = np.array([
                leader_body['bodies']['pos']['x'],
                leader_body['bodies']['pos']['y'],
                leader_body['bodies']['pos']['z']])
        real_pos.append((pos_leader[0], pos_leader[1], pos_leader[2]))
        maneuver_l.velocity(v[0], v[1], v[2], 0, a[0], a[1], a[2], 0)
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
