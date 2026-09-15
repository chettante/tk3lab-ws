from functions import *

setup()
start()
random_trajectory(n_points=3)
PERIOD = 0.1  # seconds
next_t = time.monotonic()
while True:  # run for 30 seconds
    # --- your periodic work here ---
    leader = optitrack.bodies('QR4_leading')
    follower = optitrack.bodies('QR4_following')
    pos_leader = leader['bodies']['pos']
    pos_follower = follower['bodies']['pos']
    print("Leader position: ", pos_leader)
    print("Follower position: ", pos_follower)
    target = compute_follower_target_3d(pos_leader, pos_follower, step=0.5)
    print("Follower target: ", target)
    maneuver_f.goto(target[0], target[1], target[2], 0, 0)

    # schedule the next tick and sleep only the remaining time
    next_t += PERIOD
    sleep = next_t - time.monotonic()
    if sleep > 0:
        time.sleep(sleep)
    else:
        # work took longer than PERIOD — we're behind. Skip missed ticks
        # so we don't spiral, and resync to the current time.
        next_t = time.monotonic()

maneuver_l.wait()
stop()
