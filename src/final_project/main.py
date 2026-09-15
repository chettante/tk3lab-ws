from functions import *

setup()
start()
random_trajectory(n_points=3)
vmax = 1
maneuver_f.set_velocity_limit(vmax, 1)
f_limits = maneuver_f.get_limits()
# print("Follower limits: ", f_limits)
maneuver_l.set_velocity_limit(1, 1)
l_limits = maneuver_l.get_limits()
# print("Leader limits: ", l_limits)
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
    target = compute_follower_velocity_target_3d(pos_leader, pos_follower, vmax)
    print("Follower target: ", target)
    maneuver_f.velocity(target[0], target[1], target[2], 1, 1, 1, 1, 0)
    pos_leader_prev = pos_leader

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
