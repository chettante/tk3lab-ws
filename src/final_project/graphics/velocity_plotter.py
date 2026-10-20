"""
Compares the estimated velocity (velocity_log.txt) with the "true" velocity
measured by pom (pom.log), restricted to the time interval common to the
two files, and produces a single image with 3 subplots (Vx, Vy, Vz).
 
Usage:
    python3 velocity_plotter.py [pom_log_path] [velocity_log_path] [output_path]
 
If no arguments are given, it uses the paths defined in final_project/config.py
(pom_leader_log, velocity_log, plot_velocity).
"""
 
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# allows direct execution (python3 velocity_plotter.py) reading final_project/config.py
_SRC_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _SRC_ROOT not in sys.path:
    sys.path.insert(0, _SRC_ROOT)
from final_project.config import config_path
 
 
def load_pom_log(path):
    """
    Parses the pom.log file.
    Format: comment lines starting with '#', empty lines to skip,
    header with the column names preceded by '#', then space-separated data lines.
    Columns of interest (based on the header):
        ts vx vy vz  -> indices 0, 13, 14, 15
    """
    timestamps = []
    vx, vy, vz = [], [], []
 
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            # valid data line: must have at least 16 columns (index 15 = vz)
            if len(parts) < 16:
                continue
            try:
                ts = float(parts[0])
                v_x = float(parts[13])
                v_y = float(parts[14])
                v_z = float(parts[15])
            except ValueError:
                continue
            timestamps.append(ts)
            vx.append(v_x)
            vy.append(v_y)
            vz.append(v_z)
 
    return (np.array(timestamps), np.array(vx), np.array(vy), np.array(vz))
 
 
def load_velocity_log(path):
    """
    Parses the velocity_log.txt file.
    Format: timestamp, vx, vy, vz (comma separated)
    """
    timestamps, vx, vy, vz = [], [], [], []
 
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 4:
                continue
            try:
                ts, v_x, v_y, v_z = (float(p) for p in parts[:4])
            except ValueError:
                continue
            timestamps.append(ts)
            vx.append(v_x)
            vy.append(v_y)
            vz.append(v_z)
 
    return (np.array(timestamps), np.array(vx), np.array(vy), np.array(vz))
 
 
def clip_to_common_interval(t_a, t_b):
    """Returns (t_start, t_end) of the time interval common to two time series."""
    t_start = max(t_a.min(), t_b.min())
    t_end = min(t_a.max(), t_b.max())
    return t_start, t_end
 
 
def nearest_match(t_ref, t_pom, vx_pom, vy_pom, vz_pom):
    """
    For each timestamp in t_ref (the one that logs less frequently),
    finds the closest pom sample in time and takes its value.
    t_pom must be sorted in increasing order (the logs already are by construction).
    """
    idx = np.searchsorted(t_pom, t_ref)
    idx = np.clip(idx, 1, len(t_pom) - 1)
 
    left = idx - 1
    right = idx
    # pick the closest between the previous and the next sample
    use_left = np.abs(t_pom[left] - t_ref) <= np.abs(t_pom[right] - t_ref)
    best_idx = np.where(use_left, left, right)
 
    return vx_pom[best_idx], vy_pom[best_idx], vz_pom[best_idx]
 
 
def main():
 
    pom_path = sys.argv[1] if len(sys.argv) > 1 else config_path('pom_leader_log')
    vel_path = sys.argv[2] if len(sys.argv) > 2 else config_path('velocity_log')
    out_path = sys.argv[3] if len(sys.argv) > 3 else config_path('plot_velocity')
 
    # Load the two files
    t_pom, vx_pom, vy_pom, vz_pom = load_pom_log(pom_path)
    t_est, vx_est, vy_est, vz_est = load_velocity_log(vel_path)
 
    if len(t_pom) == 0:
        raise RuntimeError(f"No valid data found in {pom_path}")
    if len(t_est) == 0:
        raise RuntimeError(f"No valid data found in {vel_path}")
 
    # Common time interval
    t_start, t_end = clip_to_common_interval(t_pom, t_est)
    print(f"Common interval: [{t_start:.3f}, {t_end:.3f}]  (duration: {t_end - t_start:.2f} s)")
 
    mask_est = (t_est >= t_start) & (t_est <= t_end)
    t_est_c, vx_est_c, vy_est_c, vz_est_c = (t_est[mask_est], vx_est[mask_est],
                                              vy_est[mask_est], vz_est[mask_est])
 
    # For each velocity_log timestamp (less frequent), take the closest
    # pom sample in time, instead of keeping all the pom points
    vx_pom_c, vy_pom_c, vz_pom_c = nearest_match(t_est_c, t_pom, vx_pom, vy_pom, vz_pom)
 
    # Relative time (starting at 0) for readability on the x axis
    t0 = t_start
    t_pom_rel = t_est_c - t0   # same instants as velocity_log
    t_est_rel = t_est_c - t0
 
    # Plot: 3 vertical subplots, one per component
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
 
    components = [
        ("Vx", vx_pom_c, vx_est_c, axes[0]),
        ("Vy", vy_pom_c, vy_est_c, axes[1]),
        ("Vz", vz_pom_c, vz_est_c, axes[2]),
    ]
 
    for label, pom_data, est_data, ax in components:
        ax.plot(t_pom_rel, pom_data, label="pom (ground truth)", color="tab:green", linewidth=1.5)
        ax.plot(t_est_rel, est_data, label="estimate (velocity_log)", color="tab:red",
                linestyle="none", marker="o", markersize=3)
        ax.set_ylabel(f"{label} [m/s]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
 
    axes[-1].set_xlabel("Time [s] (relative to the start of the common interval)")
    fig.suptitle("Velocity comparison: pom (ground truth) vs estimate (velocity_log)", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
 
    fig.savefig(out_path, dpi=150)
    print(f"Plot saved to: {out_path}")
 
 
if __name__ == "__main__":
    main()