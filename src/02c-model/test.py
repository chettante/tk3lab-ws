from quadrotor import Quadrotor
from lib import *
from utils import *

quad = Quadrotor(mass=1.0, dt=0.001)
save_dir = "plots"

# CASE 1: HOVER
thrust = quad.mass * quad.g
force_wrench = np.array([0, 0, thrust, 0, 0, 0])

quad.reset()

for t in range(5000):  # 5 seconds
    quad.step(force_wrench)
    quad.log_state(t * quad.dt)

plot_states(quad.history, title="quad_states_hover", save_path=save_dir, show_time=3)
plot_trajectory(quad.history, title="quad_traj_hover", save_path=save_dir, show_time=3)


# CASE 2: UPWARD ACCELERATION (+1N)
thrust = quad.mass * quad.g + 1.0
force_wrench = np.array([0, 0, thrust, 0, 0, 0])

quad.reset()

for t in range(5000):  # 5 seconds
    quad.step(force_wrench)
    quad.log_state(t * quad.dt)

plot_states(quad.history, title="quad_states_upward", save_path=save_dir, show_time=3)
plot_trajectory(quad.history, title="quad_traj_upward", save_path=save_dir, show_time=3)