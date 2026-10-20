import matplotlib.pyplot as plt
import numpy as np

def plot_trajectory(points, dt=None, filename="otto.png", show=True):
    """
    Shows the trajectory: 3D view, top view (x-y) and altitude over time.

    points   : list of (x, y, z) tuples, e.g. the `otto` list
    dt       : time step between points [s]; if None the horizontal axis
               of the third plot is the point index
    filename : if not None the image is saved
    show     : opens the window (set it to False on a machine without a display)
    """
    P = np.asarray(points, dtype=float)
    if P.ndim != 2 or P.shape[1] != 3 or len(P) == 0:
        raise ValueError("points must be a list of (x, y, z) tuples")
    x, y, z = P.T
    t = np.arange(len(P)) * dt if dt else np.arange(len(P))

    line, start, end = "#2E6FD8", "#1F9D55", "#C8372D"
    fig = plt.figure(figsize=(15, 5))

    # --- 3D view
    ax = fig.add_subplot(1, 3, 1, projection="3d")
    ax.plot(x, y, z, color=line, lw=2)
    ax.scatter(*P[0], color=start, s=40, label="Start", depthshade=False)
    ax.scatter(*P[-1], color=end, s=40, marker="s", label="End", depthshade=False)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.set_box_aspect(np.maximum(np.ptp(P, axis=0), 1e-3))  # same meters on the 3 axes
    ax.set_title("3D view")
    ax.legend(loc="upper left")

    # --- Top view
    ax = fig.add_subplot(1, 3, 2)
    ax.plot(x, y, color=line, lw=2)
    ax.plot(x[0], y[0], "o", color=start, ms=8)
    ax.plot(x[-1], y[-1], "s", color=end, ms=8)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Top view (x-y)")
    ax.grid(alpha=0.3)

    # --- Altitude over time
    ax = fig.add_subplot(1, 3, 3)
    ax.plot(t, z, color=line, lw=2)
    ax.set_xlabel("t [s]" if dt else "point index"); ax.set_ylabel("z [m]")
    ax.set_title("Altitude")
    ax.grid(alpha=0.3)

    fig.tight_layout()
    if filename:
        fig.savefig(filename, dpi=150)
        print(f"[*] Plot saved to {filename}")
    if show:
        plt.show()
    plt.close(fig)
