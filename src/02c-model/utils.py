from lib import *


###############################################
# Utility functions for quadrotor model class #
###############################################

def quaternion_product(q1, q2):
    """Hamilton product of two quaternions q1 ⊗ q2 (scalar first)"""
    s1, v1 = q1[0], q1[1:4]
    s2, v2 = q2[0], q2[1:4]
    
    s = s1 * s2 - np.dot(v1, v2)
    v = s1 * v2 + s2 * v1 + np.cross(v1, v2)
    return np.hstack([s, v])


def quaternion_normalize(q):
    """Normalize quaternion to unit norm"""
    norm = np.linalg.norm(q)
    return q / norm if norm > 0 else q


def skew_matrix(v):
    """Skew-symmetric matrix from vector v"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def rk4_step(f, x, u, dt):
    """RK4 integration step"""
    k1 = f(x, u)
    k2 = f(x + dt/2 * k1, u)
    k3 = f(x + dt/2 * k2, u)
    k4 = f(x + dt * k3, u)
    return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)


def rodrigues_rotation_matrix(q):
    """Convert quaternion (scalar first) to rotation matrix"""
    qn = quaternion_normalize(q)
    s, v = qn[0], qn[1:4]
    vx = skew_matrix(v)
    R = np.eye(3) + np.sin(s)*vx + (1-np.cos(s))*np.dot(vx, vx)
    return R


def quat_to_euler(q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        R = rodrigues_rotation_matrix(q)
        
        sin_pitch = -R[2, 0]
        sin_pitch = np.clip(sin_pitch, -1, 1)
        pitch = np.arcsin(sin_pitch)
        
        if np.abs(np.cos(pitch)) > 1e-6:
            roll = np.arctan2(R[2, 1], R[2, 2])
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[0, 1], R[1, 1])
            yaw = 0
        
        return np.array([roll, pitch, yaw])



##################
# Plotting utils #
##################

def plot_states(history, title="states", save_path=None, show_time=0):
    t = np.array(history['time'])

    pos = np.array(history['position'])
    vel = np.array(history['velocity'])
    avel = np.array(history['ang_velocity'])
    acc = np.array(history['acceleration'])
    euler = np.array([quat_to_euler(q) for q in history['quaternion']])

    colors = ['blue', 'red', 'green']
    labels = ['x', 'y', 'z']

    fig, axs = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle(title)

    # Position
    axs[0,0].plot(t, pos[:, 1], colors[1], label=labels[1])
    axs[0,0].plot(t, pos[:, 2], colors[2], label=labels[2])
    axs[0,0].plot(t, pos[:, 0], colors[0], label=labels[0])
    axs[0,0].set_title("Position")
    axs[0,0].legend()
    axs[0,0].grid()

    # Attitude
    axs[0,1].plot(t, euler[:, 0], colors[0], label=labels[0])
    axs[0,1].plot(t, euler[:, 1], colors[1], label=labels[1])
    axs[0,1].plot(t, euler[:, 2], colors[2], label=labels[2])
    axs[0,1].set_title("Attitude")
    axs[0,1].legend()
    axs[0,1].grid()

    # Velocity
    axs[1,0].plot(t, vel[:, 1], colors[1], label=labels[1])
    axs[1,0].plot(t, vel[:, 2], colors[2], label=labels[2])
    axs[1,0].plot(t, vel[:, 0], colors[0], label=labels[0])
    axs[1,0].set_title("Velocity")
    axs[1,0].legend()
    axs[1,0].grid()

    # Angular velocity
    axs[1,1].plot(t, avel[:, 1], colors[1], label=labels[1])
    axs[1,1].plot(t, avel[:, 2], colors[2], label=labels[2])
    axs[1,1].plot(t, avel[:, 0], colors[0], label=labels[0])
    axs[1,1].set_title("Angular velocity")
    axs[1,1].legend()
    axs[1,1].grid()

    # Acceleration
    axs[2,0].plot(t, acc[:, 1], colors[1], label=labels[1])
    axs[2,0].plot(t, acc[:, 2], colors[2], label=labels[2])
    axs[2,0].plot(t, acc[:, 0], colors[0], label=labels[0])
    axs[2,0].set_title("Acceleration")
    axs[2,0].legend()
    axs[2,0].grid()

    axs[2,1].axis('off')

    plt.tight_layout()

    # Save
    if save_path is not None:
        os.makedirs(save_path, exist_ok=True)
        filename = os.path.join(save_path, f"{title}.png")
        plt.savefig(filename, dpi=300)

    # Show optionally
    if show_time > 0:
        plt.show(block=False)
        plt.pause(show_time)
        plt.close()
    else:
        plt.close()


def plot_trajectory(history, title="trajectory", save_path=None, show_time=0):
    pos = np.array(history['position'])

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    fig.suptitle(title)

    ax.plot(pos[:,0], pos[:,1], pos[:,2], label="trajectory")
    ax.scatter(pos[0,0], pos[0,1], pos[0,2], color='green', label="start")
    ax.scatter(pos[-1,0], pos[-1,1], pos[-1,2], color='red', label="end")

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.legend()
    ax.grid()

    plt.tight_layout()

    # Save
    if save_path is not None:
        os.makedirs(save_path, exist_ok=True)
        filename = os.path.join(save_path, f"{title}.png")
        plt.savefig(filename, dpi=300)

    # Show optionally
    if show_time > 0:
        plt.show(block=False)
        plt.pause(show_time)
        plt.close()
    else:
        plt.close()