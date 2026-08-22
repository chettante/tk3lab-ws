# Copyright (c) 2026 IRISA/CNRS-INRIA
# All rights reserved.
#
# Redistribution  and  use  in  source  and binary  forms,  with  or  without
# modification, are permitted provided that the following conditions are met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
# THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
# WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
# MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
# ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
# WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
# IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#                                         Gianluca Corsini on Thu Apr 16 2026

# ############################################
#  PLACE HERE YOUR NECESSARY FUNCTION IMPORTS
# ############################################

import genomix
import math
import numpy as np
import os
import time
from quadrotor import Quadrotor

# --- setup ----------------------------------------------------------------
def setup():
    # ###############################################
    #  PLACE HERE YOUR NECESSARY SETUP LINES OF CODE
    # ###############################################

    # Write data to input port
    client = genomix.connect()
    nhfc = client.load("nhfc")
    my_state_port = nhfc.state('my_state')

    # connect nhfc to state port which will be updated by our simulator
    nhfc.connect_port({ 'local': 'state', 'remote': 'my_state' })

    return my_state_port

# --- start ----------------------------------------------------------------
def start():
    # ########################################
    #  FILL THIS FUNCTION, THEN REMOVE 'pass'
    # ########################################
    pass


# --- stop -----------------------------------------------------------------
def stop():
    # ########################################
    #  FILL THIS FUNCTION, THEN REMOVE 'pass'
    # ########################################
    pass

# --- state_to_nhfc --------------------------------------------------------
def state_to_nhfc(state_port, state: np.array):
    def _get_time():
        # returns a tuple of the type (<sec>,<nsec>)
        now = math.modf(time.clock_gettime(time.CLOCK_REALTIME))
        return (int(now[1]), int(now[0]*1e9))

    # https://git.openrobots.org/projects/libmrsim/repository/libmrsim/revisions/master/entry/src/sim.c#L66
    pstddev = 1e-3
    qstddev = 1e-3
    vstddev = 1e-3
    wstddev = 3e-3
    astddev = 2e-2

    """ covariances """
    # Covariances documentation at
    # https://git.openrobots.org/projects/openrobots-idl/repository/openrobots-idl/revisions/master/entry/pose/t3d.idl#L41

    pos_cov = [(pstddev)**2, 0, (pstddev)**2, 0, 0, (pstddev)**2]

    att_cov = [0 for i in range(10)]
    # https://git.openrobots.org/projects/mrsim-genom3/repository/mrsim-genom3/revisions/master/entry/codels/sim.c#L52
    qw = state[3]
    qx = state[4]
    qy = state[5]
    qz = state[6]
    att_cov[0] = (qstddev**2) * (1 - qw*qw);
    att_cov[1] = (qstddev**2) * -qw*qx;
    att_cov[2] = (qstddev**2) * (1 - qx*qx);
    att_cov[3] = (qstddev**2) * qw*qy;
    att_cov[4] = (qstddev**2) * -qx*qy;
    att_cov[5] = (qstddev**2) * (1 - qy*qy);
    att_cov[6] = (qstddev**2) * -qw*qz;
    att_cov[7] = (qstddev**2) * -qx*qz;
    att_cov[8] = (qstddev**2) * -qy*qz;
    att_cov[9] = (qstddev**2) * (1 - qz*qz);

    att_pos_cov = [0 for i in range(4*3)]

    vel_cov = [(vstddev)**2, 0, (vstddev)**2, 0, 0, (vstddev)**2]
    avel_cov = [(wstddev)**2, 0, (wstddev)**2, 0, 0, (wstddev)**2]
    acc_cov = [(astddev)**2, 0, (astddev)**2, 0, 0, (astddev)**2]

    aacc_cov = [0 for i in range(6)]

    now = _get_time()

    # port and message descriptions at
    # https://git.openrobots.org/projects/openrobots-idl/repository/openrobots-idl/revisions/master/entry/pose/pose_estimator.gen
    # https://git.openrobots.org/projects/openrobots-idl/repository/openrobots-idl/revisions/master/entry/pose/t3d.idl
    data = { "state": {
            "ts" : {"sec": now[0], "nsec": now[1]},
            "intrinsic": False,
            "pos": ({"x": state[0], "y": state[1], "z": state[2]}),
            "att": ({"qw": state[3], "qx": state[4], "qy": state[5], "qz": state[6]}),
            "vel": ({"vx": state[7], "vy": state[8], "vz": state[9]}),
            "avel": ({"wx": state[10], "wy": state[11], "wz": state[12]}),
            "acc": ({"ax": state[13], "ay": state[14], "az": state[15]}),
            "aacc": ({"awx": state[16], "awy": state[17], "awz": state[18]}),
            "pos_cov": ({"cov": pos_cov}),
            "att_cov": ({"cov": att_cov}),
            "att_pos_cov": ({"cov": att_pos_cov}),
            "vel_cov": ({"cov": vel_cov}),
            "avel_cov": ({"cov": avel_cov}),
            "acc_cov": ({"cov": acc_cov}),
            "aacc_cov": ({"cov": aacc_cov})
            }
        }

    if not state_port:
        print("port 'sim_state_port' is not set")
        return
    state_port(data)

# --- rotor_speeds_from_nhfc -----------------------------------------------
def rotor_speeds_from_nhfc(c_f, n_act=4):
    desired_speeds = np.zeros(n_act)

    data = nhfc.rotor_input()["rotor_input"]
    # sometimes None may appear
    # if that happens for i-th rotor, then skip its speed
    for i, s in enumerate(data["desired"]):
        if s:
            desired_speeds[i] = data["desired"][i]
    return desired_speeds

# --- speed_to_thrust ------------------------------------------------------
def speed_to_thrust(speed: np.array, c_f):
    return np.square(speed) * c_f

# --- get_time_now_ms ------------------------------------------------------
def get_time_now_ms():
    return time.clock_gettime_ns(time.CLOCK_REALTIME)*1e-6 # return ms

################################################################################
g = genomix.connect()

g.rpath(os.environ['HOME'] + '/openrobots/lib/genom/pocolibs/plugins')

nhfc = g.load('nhfc')

state_port = setup()

input("start simulation?")

# ############################
#  INITIALIZE SIMULATION HERE
# ############################

x = 0
u0 = 0

t0 = 0
tf = 10
dt = 0.001 # the time step is 1ms

# preallocate arrays to store all simulation data
# more efficient than dynamic allocation
N = math.ceil((tf-t0)/dt)
tt = np.linspace(t0, tf, N)
x_log = np.zeros((N, x0.shape[0]))
u_log = np.zeros((N, u0.shape[0]))
t_log = np.zeros(N) # (N,)
tc_log = np.zeros(N) # (N,)

quadrotor = Quadrotor()

start()

# give it some time
time.sleep(0.1)

set_first_wp = True
set_second_wp = True
for i, ts in enumerate(tt):
    if set_first_wp and ts >= 0:
        nhfc.set_position(1,1,1,0)
        set_first_wp = False
    elif set_second_wp and ts >= 10:
        nhfc.set_position(0,0,0,0)
        set_second_wp = False

    c_f = 3e-6  # thrust coefficient, to be tuned for your drone
    u = speed_to_thrust(rotor_speeds_from_nhfc(), c_f)
    t1 = get_time_now_ms()

    # ################################
    #  UPDATE SIMULATION: make 1 step
    # ################################
    
    # drone model here to compute the states
    x = quadrotor.step(u, dt)
    xdot = quadrotor.dynamics(x_prev, u)

    # save data
    t_log[i] = ts
    x_log[i, :] = x.reshape(-1)
    u_log[i, :] = u.reshape(-1)

    # print simulation time but every 1k iterations
    if (int(ts/dt) % 1000) == 0:
        print(f"t: {ts}")

    # if necessary, wait to match dt
    t2 = get_time_now_ms()
    elapsed_ms = t2-t1
    tc_log[i] = elapsed_ms
    if elapsed_ms > 0:
        time.sleep(elapsed_ms*1e-3)
    elif elapsed_ms < 0:
        print(f"delay of: {dt - elapsed_ms}ms")

    # update nhfc state
    state = np.hstack((x, xdot[-6:])).reshape(-1)
    state_to_nhfc(state)
    x_prev = x

stop()

# ############################################
#  SAVE DATA TO DISK:
#  generate log files as GenoM3 components do
# ############################################
