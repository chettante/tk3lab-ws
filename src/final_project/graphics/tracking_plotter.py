"""
Analizza l'inseguimento leader/follower a partire dai log del pom
(nessun file extra da produrre nel main).

Legge (percorsi e parametri FOV di default da final_project/config.py):
  - pom del leader   (pom_leader_log,   scritto da pom_l.log_state nel main)
  - pom del follower (pom_follower_log, scritto da pom_f.log_state nel main)

Produce un'unica figura:
  (1) distanza leader-follower su x, y, z (frame globale) + norma
  (2) errore di yaw (bearing del leader - yaw del follower)
  (3) vista dall'alto (x-y) di leader e follower

Sfondo rosso nei grafici temporali = leader fuori dal FOV (calcolato
con la stessa geometria di FOVPyramid.contains).

Uso:
    python3 tracking_plotter.py [--leader PATH] [--follower PATH] [--out PATH]
                                [--fov-deg DEG] [--max-range M] [--rate 20]
                                [--crop T_START T_END]     (secondi relativi)
                                [--cols x y z yaw]  (indici colonna, default 7 8 9 12)
"""

import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# permette l'esecuzione diretta (python3 velocity_plotter.py) leggendo final_project/config.py
_SRC_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _SRC_ROOT not in sys.path:
    sys.path.insert(0, _SRC_ROOT)
from final_project.config import CONFIG, config_path, genom_log_path

# Formato pom-genom3: ts i posp attp velp avelp accp x y z roll pitch yaw vx ...
DEFAULT_COLS = [7, 8, 9, 12]          # x y z yaw
NAMES = ["x", "y", "z", "yaw"]


def find_columns(header_tokens):
    names = [h.lower() for h in header_tokens]
    if all(n in names for n in NAMES):
        return [names.index(n) for n in NAMES]
    return DEFAULT_COLS


def load_pom_pose(path, cols=None):
    """Ritorna t[N], pos[N,3], yaw[N] (rad, unwrapped) da un pom.log."""
    header, rows = None, []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            toks = line.lstrip("#").split()
            # l'header di pom ("ts i posp ...") può comparire con o senza '#'
            if toks and toks[0].lower() == "ts":
                header = toks
                continue
            if line.startswith("#"):
                continue
            rows.append(toks)

    if cols is None:
        cols = find_columns(header) if header else DEFAULT_COLS
    # colonna 'i' (intrinsic): teniamo solo le righe in frame world (i == 0)
    i_col = header.index("i") if header and "i" in header else None

    need = max(cols) + 1
    t, pos, yaw = [], [], []
    for parts in rows:
        if len(parts) < need:
            continue
        try:
            if i_col is not None and int(float(parts[i_col])) == 1:
                continue
            t.append(float(parts[0]))
            pos.append([float(parts[cols[i]]) for i in range(3)])
            yaw.append(float(parts[cols[3]]))
        except ValueError:
            continue
    if not t:
        raise RuntimeError(f"Nessun dato valido in {path}")
    return np.array(t), np.array(pos), np.unwrap(np.array(yaw))


def resample(t_src, pos, yaw, t_grid):
    p = np.column_stack([np.interp(t_grid, t_src, pos[:, i]) for i in range(3)])
    y = np.interp(t_grid, t_src, yaw)
    return p, y


def in_fov_mask(rel, yaw, half_deg, max_range):
    """Stessa logica di FOVPyramid.contains, vettorializzata."""
    c, s = np.cos(yaw), np.sin(yaw)
    xb = c * rel[:, 0] + s * rel[:, 1]
    dist = np.linalg.norm(rel, axis=1)
    ok = (dist <= max_range) & (dist >= 0.1) & (xb > 0)
    ang = np.arccos(np.clip(xb / np.maximum(dist, 1e-9), -1, 1))
    return ok & (ang <= np.radians(half_deg))


def segments(mask, t):
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = t[i]
        elif not m and start is not None:
            out.append((start, t[i])); start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--leader", default=config_path("pom_leader_log"))
    ap.add_argument("--follower", default=config_path("pom_follower_log"))
    ap.add_argument("--out", default=config_path("plot_tracking"))
    ap.add_argument("--fov-deg", type=float, default=CONFIG["fov_half_angle_deg"])
    ap.add_argument("--max-range", type=float, default=CONFIG["fov_max_range"])
    ap.add_argument("--rate", type=float, default=20.0, help="Hz della griglia comune")
    ap.add_argument("--crop", type=float, nargs=2, metavar=("T0", "T1"),
                    help="tieni solo [T0, T1] secondi dall'inizio dell'intervallo comune")
    ap.add_argument("--cols", type=int, nargs=4, metavar=("X", "Y", "Z", "YAW"))
    args = ap.parse_args()

    tl, pl, yl = load_pom_pose(args.leader, args.cols)
    tf, pf, yf = load_pom_pose(args.follower, args.cols)

    t_start, t_end = max(tl.min(), tf.min()), min(tl.max(), tf.max())
    if t_end <= t_start:
        raise RuntimeError("I due log non hanno un intervallo di tempo comune")
    print(f"Intervallo comune: [{t_start:.3f}, {t_end:.3f}]  (durata: {t_end - t_start:.2f} s)")

    if args.crop:
        t_start, t_end = t_start + args.crop[0], min(t_end, t_start + args.crop[1])

    tg = np.arange(t_start, t_end, 1.0 / args.rate)
    L, _ = resample(tl, pl, yl, tg)
    F, yaw = resample(tf, pf, yf, tg)
    t = tg - tg[0]

    rel = L - F
    dist = np.linalg.norm(rel, axis=1)
    bearing = np.arctan2(rel[:, 1], rel[:, 0])
    yaw_err = np.degrees(np.arctan2(np.sin(bearing - yaw), np.cos(bearing - yaw)))
    fov = in_fov_mask(rel, yaw, args.fov_deg, args.max_range)

    fig = plt.figure(figsize=(15, 8))
    gs = fig.add_gridspec(2, 2, width_ratios=[2, 1])
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[1, 0], sharex=ax1)
    ax3 = fig.add_subplot(gs[:, 1])

    def shade(ax):
        for a, b in segments(~fov, t):
            ax.axvspan(a, b, color="tab:red", alpha=0.15, lw=0)

    for i, (lab, c) in enumerate(zip("xyz", ["tab:blue", "tab:green", "tab:purple"])):
        ax1.plot(t, rel[:, i], color=c, lw=1.3, label=f"Δ{lab}")
    ax1.plot(t, dist, color="k", lw=1.8, label="‖Δ‖")
    ax1.axhline(0, color="gray", lw=0.6)
    ax1.axhline(args.max_range, color="k", ls=":", lw=1, label=f"max range FOV ({args.max_range:g} m)")
    ax1.axhline(CONFIG["tracking_follow_distance"], color="k", ls="--", lw=1,
                label=f"distanza desiderata ({CONFIG['tracking_follow_distance']:g} m)")
    shade(ax1)
    ax1.set_ylabel("Leader − follower [m]")
    ax1.set_title("Distanza dal leader (frame globale)")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="upper right", ncol=5, fontsize=8)

    ax2.plot(t, yaw_err, color="tab:red", lw=1.4)
    ax2.axhline(0, color="gray", lw=0.6)
    ax2.axhline(args.fov_deg, color="k", ls=":", lw=1)
    ax2.axhline(-args.fov_deg, color="k", ls=":", lw=1, label=f"±{args.fov_deg:g}° (semi-apertura FOV)")
    shade(ax2)
    ax2.set_ylabel("Errore di yaw [deg]")
    ax2.set_xlabel("Tempo [s] (relativo all'inizio dell'intervallo)")
    ax2.set_title("Errore di yaw (>0: leader a sinistra, il follower deve ruotare CCW)")
    ax2.grid(alpha=0.3)
    ax2.legend(loc="upper right", fontsize=8)

    ax3.plot(L[:, 0], L[:, 1], color="tab:blue", lw=1.6, label="Leader")
    ax3.plot(F[:, 0], F[:, 1], color="tab:orange", lw=1.6, label="Follower")
    step = max(1, int(round(2.0 * args.rate)))          # collegamento ogni ~2 s
    for i in range(0, len(t), step):
        ax3.plot([L[i, 0], F[i, 0]], [L[i, 1], F[i, 1]], color="gray", lw=0.6, alpha=0.6)
    for P, c in ((L, "tab:blue"), (F, "tab:orange")):
        ax3.plot(*P[0, :2], "o", color=c, ms=8)
        ax3.plot(*P[-1, :2], "s", color=c, ms=8)
    # heading del follower ogni ~1 s: freccia lungo lo yaw + bordi del FOV
    h_step = max(1, int(round(1.0 * args.rate)))
    idx = np.arange(0, len(t), h_step)
    span = np.ptp(np.vstack([L[:, :2], F[:, :2]]), axis=0).max()
    arrow = 0.06 * max(span, 1.0)                        # lunghezza freccia in m
    half = np.radians(args.fov_deg)
    for s in (-half, half):
        ax3.plot(np.vstack([F[idx, 0], F[idx, 0] + 0.7 * arrow * np.cos(yaw[idx] + s)]),
                 np.vstack([F[idx, 1], F[idx, 1] + 0.7 * arrow * np.sin(yaw[idx] + s)]),
                 color="tab:red", lw=0.6, alpha=0.5)
    ax3.quiver(F[idx, 0], F[idx, 1], np.cos(yaw[idx]), np.sin(yaw[idx]),
               color="tab:red", angles="xy", scale_units="xy", scale=1.0 / arrow,
               width=0.004, zorder=3)
    ax3.plot([], [], color="tab:red", lw=1.5, label=f"Heading follower (±{args.fov_deg:g}° FOV)")
    ax3.set_aspect("equal", adjustable="datalim")
    ax3.set_xlabel("x [m]"); ax3.set_ylabel("y [m]")
    ax3.set_title("Vista dall'alto\n(○ start, □ fine, grigio ogni ~2 s, heading ogni ~1 s)", fontsize=10)
    ax3.grid(alpha=0.3)
    ax3.legend(loc="best", fontsize=8)

    fig.legend(handles=[Patch(color="tab:red", alpha=0.3, label="Leader fuori FOV")],
               loc="lower center", fontsize=9)
    fig.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(args.out, dpi=150)
    print(f"Plot salvato in: {args.out}")

    if fov.any():
        print(f"[Leader nel FOV, {fov.sum()} campioni]")
        print(f"  RMS Δx,Δy,Δz [m] : {np.sqrt((rel[fov] ** 2).mean(axis=0)).round(3)}")
        print(f"  distanza media   : {dist[fov].mean():.3f} m  (max {dist[fov].max():.3f} m)")
        print(f"  RMS yaw err      : {np.sqrt(np.nanmean(yaw_err[fov] ** 2)):.2f} deg")
    print(f"Frazione di tempo fuori FOV: {100 * (~fov).mean():.1f} %")


if __name__ == "__main__":
    main()