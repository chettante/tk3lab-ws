"""Explanatory figures for the README (English, no config values in labels).

Usage: python3 docs/make_figures.py <repo root>
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Wedge, Arc, Circle, FancyArrowPatch

REPO = sys.argv[1]
OUT = os.path.join(REPO, "docs")
os.makedirs(OUT, exist_ok=True)
sys.path.insert(0, os.path.join(REPO, "src"))
from final_project.config import CONFIG as C  # values used for geometry only

BLUE, GREEN, ORANGE, GRAY, PURPLE, RED, DARK, SLATE = (
    "#3B6FB6", "#2E7D4F", "#C0742B", "#8A93A6", "#8E44AD", "#C0392B", "#1F2733", "#5B6475")
FOV_FILL = "#E4EAF5"
plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 11, "axes.edgecolor": SLATE,
    "axes.labelcolor": DARK, "xtick.color": SLATE, "ytick.color": SLATE,
    "axes.spines.top": False, "axes.spines.right": False, "savefig.dpi": 150,
    "savefig.bbox": "tight", "savefig.facecolor": "white",
})


def save(fig, name):
    fig.savefig(os.path.join(OUT, name))
    plt.close(fig)


def box(ax, xy, w, h, title, sub, color, fs=15):
    x, y = xy
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0,rounding_size=0.12",
                                fc=color, ec="none", zorder=3))
    ax.text(x, y + (0.12 if sub else 0), title, ha="center", va="center", color="white",
            fontsize=fs, weight="bold", zorder=4)
    if sub:
        ax.text(x, y - 0.22, sub, ha="center", va="center", color="white", fontsize=10.5, zorder=4)


def arrow(ax, p, q, color=DARK, label=None, lpos=0.5, ls="-", lw=2, rad=0.0, lc=None, off=(0, 0)):
    a = FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=16, color=color, lw=lw, ls=ls,
                        connectionstyle=f"arc3,rad={rad}", zorder=2, shrinkA=0, shrinkB=0)
    ax.add_patch(a)
    if label:
        mx = p[0] + (q[0] - p[0]) * lpos + off[0]
        my = p[1] + (q[1] - p[1]) * lpos + off[1]
        ax.text(mx, my, label, ha="center", va="center", fontsize=10.5, color=lc or color,
                bbox=dict(fc="white", ec="#D5D9E0", boxstyle="round,pad=0.25"), zorder=5)


def quad(ax, x, y, color, yaw=0.0, s=0.45, lw=2):
    for k in range(4):
        a = yaw + np.pi / 4 + k * np.pi / 2
        px, py = x + s * np.cos(a), y + s * np.sin(a)
        ax.plot([x, px], [y, py], color=color, lw=lw, zorder=5)
        ax.add_patch(Circle((px, py), s * 0.45, fc="white", ec=color, lw=lw, zorder=6))
    ax.add_patch(Circle((x, y), s * 0.28, fc=color, ec=color, zorder=7))


# ------------------------------------------------------------------ 1. architecture
fig, ax = plt.subplots(figsize=(12, 5.6))
ax.set_xlim(-0.2, 12.2); ax.set_ylim(0, 5.6); ax.axis("off")
W, H = 2.5, 0.95
P = {"py": (1.2, 2.8), "man": (4.1, 4.6), "nhfc": (7.9, 4.6), "rc": (10.9, 2.8),
     "opt": (4.1, 0.9), "pom": (7.9, 0.9)}
box(ax, P["py"], W, H, "Python", "main_statemachine.py", BLUE)
box(ax, P["man"], W, H, "maneuver", "trajectory planner", GREEN)
box(ax, P["nhfc"], W, H, "nhfc", "flight controller", ORANGE)
box(ax, P["rc"], W, H, "rotorcraft", "motor driver", GRAY)
box(ax, P["opt"], W, H, "optitrack", "motion capture (shared)", "#5D6675")
box(ax, P["pom"], W, H, "pom", "state estimator (EKF)", PURPLE)
arrow(ax, (2.2, 3.28), (3.3, 4.12), label="velocity() · goto()\nwaypoint()", lpos=0.45, off=(-0.35, 0.15))
arrow(ax, (5.35, 4.6), (6.65, 4.6), label="desired →\nreference", off=(0, 0.45))
arrow(ax, (9.15, 4.3), (10.6, 3.28), label="rotor_input", lpos=0.4, off=(0.35, 0.15))
arrow(ax, (10.35, 3.28), (8.9, 4.12), label="rotor_measure", lpos=0.55, off=(-0.1, -0.35))
arrow(ax, (10.6, 2.33), (9.15, 1.2), label="imu", lpos=0.5, off=(0.3, 0))
arrow(ax, (5.35, 0.9), (6.65, 0.9), label="body pose\n→ mocap", off=(0, -0.45))
arrow(ax, (7.9, 1.38), (7.9, 4.13), label="state", lpos=0.5)
arrow(ax, (7.3, 1.38), (4.6, 4.13), label="state", lpos=0.45)
arrow(ax, (2.9, 1.2), (1.9, 2.33), label="bodies()\n(camera model input)", lpos=0.5, off=(-0.45, -0.1))
ax.text(10.9, 4.3, "Gazebo\n(simulated quadrotor)", ha="center", color=SLATE, fontsize=10.5)
ax.plot([10.9, 10.9], [3.3, 3.95], color=SLATE, ls=":", lw=1.5)
ax.set_title("One component chain per drone (leader and follower), optitrack shared",
             fontsize=14, weight="bold", color=DARK)
save(fig, "architecture.png")

# ------------------------------------------------------------------ 2. camera FOV
h = np.radians(C["fov_half_angle_h_deg"]); v = np.radians(C["fov_half_angle_v_deg"])
R = C["fov_max_range"]; cy = np.radians(C["camera_yaw_deg"])
fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.6), gridspec_kw=dict(width_ratios=[1.1, 1]))
ax = a1
ax_dir = np.array([np.cos(cy), np.sin(cy)]); lat = np.array([-np.sin(cy), np.cos(cy)])
c1 = R * ax_dir + R * np.tan(h) * lat; c2 = R * ax_dir - R * np.tan(h) * lat
ax.add_patch(Polygon([[0, 0], c1, c2], fc=FOV_FILL, ec="#C9D4EA", zorder=1))
ax.plot([0, 0.8 * R * ax_dir[0]], [0, 0.8 * R * ax_dir[1]], ls="--", color=BLUE)
ax.text(0.8 * R * ax_dir[0] + 0.2, 0.8 * R * ax_dir[1], "optical axis", color=BLUE, fontsize=10.5)
ax.plot([0, 3.2], [0, 0], ls=":", color=SLATE)
ax.text(3.3, -0.1, "drone x axis", color=SLATE, fontsize=10, va="center")
ax.add_patch(Arc((0, 0), 4.6, 4.6, theta1=0, theta2=np.degrees(cy), color=SLATE, lw=1.5))
ax.text(2.5, 1.1, "camera mounting yaw", color=SLATE, fontsize=10)
ax.add_patch(Arc((0, 0), 9, 9, theta1=np.degrees(cy), theta2=np.degrees(cy + h), color=BLUE, lw=1.5))
ax.text(0.6, 5.2, "horizontal\nhalf-angle", color=BLUE, fontsize=10)
quad(ax, 0, 0, "#2B4C7E", yaw=0, s=0.5)
ax.text(0, -1.4, "follower", ha="center", color="#2B4C7E", weight="bold")
ax.plot(3, 7.5, "o", color=RED, ms=12); ax.text(3.3, 8.6, "inside: measurement\navailable", color=RED, ha="center", fontsize=10)
ax.plot(9, -1.6, "o", mfc="white", mec=RED, mew=2, ms=12); ax.text(9.4, -1.6, "outside: no\nmeasurement", color=RED, va="center", fontsize=10)
ax.text(np.mean([c1[0], c2[0]]) + 1.2, np.mean([c1[1], c2[1]]) + 1.2, "far plane", color=BLUE, fontsize=10)
ax.set_aspect("equal"); ax.set_xlim(-2.5, 15); ax.set_ylim(-2.5, 15)
ax.set_xlabel("x world [m]"); ax.set_ylabel("y world [m]"); ax.set_title("Top view", color=DARK)
ax = a2
ax.add_patch(Polygon([[0, 0], [R, R * np.tan(v)], [R, -R * np.tan(v)]], fc=FOV_FILL, ec="#C9D4EA"))
ax.plot([0, R * 1.08], [0, 0], ls="--", color=BLUE)
ax.add_patch(Arc((0, 0), 8, 8, theta1=0, theta2=np.degrees(v), color=BLUE, lw=1.5))
ax.text(4.2, 0.8, "vertical\nhalf-angle", color=BLUE, fontsize=10)
pt = np.radians(10); Rot = np.array([[np.cos(pt), np.sin(pt)], [-np.sin(pt), np.cos(pt)]])
tri = np.array([[0, 0], [R, R * np.tan(v)], [R, -R * np.tan(v)], [0, 0]]) @ Rot.T
ax.plot(tri[:, 0], tri[:, 1], ls="--", color=ORANGE, lw=1.6)
ax.text(-0.8, -6.6, "same pyramid with the drone pitched:\nthe camera is rigid, so tilting\nmoves the leader in the image",
        color=ORANGE, ha="left", fontsize=10)
ax.plot([-0.6, 0.6], [0, 0], color="#2B4C7E", lw=5); ax.plot(0, 0, "o", color="#2B4C7E", ms=8)
ax.text(0, -0.8, "follower", ha="center", color="#2B4C7E", weight="bold")
ax.plot(7, 3.2, "o", color=RED, ms=12); ax.text(7.3, 3.7, "leader", color=RED, fontsize=10)
ax.set_xlim(-1, 11.5); ax.set_ylim(-8, 7)
ax.set_xlabel("depth along the optical axis [m]"); ax.set_ylabel("relative altitude [m]")
ax.set_title("Side view", color=DARK)
fig.suptitle("Simulated camera: pinhole pyramid rigidly mounted on the follower", weight="bold", color=DARK, fontsize=14)
save(fig, "camera_fov.png")

# ------------------------------------------------------------------ 3. state machine
fig, ax = plt.subplots(figsize=(11, 7.2))
ax.set_xlim(0, 11); ax.set_ylim(0, 7.2); ax.axis("off")
S = {"IDLE": (1.6, 3.6), "EXPLORING": (5.5, 5.8), "TRACKING": (9.4, 3.6), "SEARCHING": (5.5, 1.3)}
box(ax, S["IDLE"], 2.5, 1.15, "IDLE", "hover in place,\nwatch the FOV", GRAY, fs=17)
box(ax, S["EXPLORING"], 2.6, 1.15, "EXPLORING", "orbit the flight volume\nlooking at its centre", BLUE, fs=17)
box(ax, S["TRACKING"], 2.5, 1.15, "TRACKING", "P + feedforward,\nleader centred", GREEN, fs=17)
box(ax, S["SEARCHING"], 2.9, 1.15, "SEARCHING", "recovery on the predicted\npoint + vertical scan", ORANGE, fs=17)
for t in ax.texts:
    if t.get_text() in S: t.set_position((t.get_position()[0], t.get_position()[1] + 0.08))
arrow(ax, (2.85, 3.6), (8.15, 3.6), GREEN, "leader in FOV")
arrow(ax, (1.8, 4.18), (4.2, 5.85), BLUE, "no sighting\nfor a while", rad=-0.25, lpos=0.42, off=(-0.55, 0.15))
arrow(ax, (4.2, 5.45), (2.3, 4.18), GRAY, "lap completed,\nleader not found", ls="--", rad=0.0, lpos=0.5, off=(0.55, -0.25))
arrow(ax, (6.8, 5.9), (9.2, 4.18), GREEN, "leader in FOV", rad=-0.25, lpos=0.5, off=(0.6, 0.35))
arrow(ax, (9.3, 3.02), (6.95, 1.2), ORANGE, "out of FOV\nlonger than a timeout", rad=-0.2, lpos=0.5, off=(0.9, -0.35))
arrow(ax, (6.95, 1.6), (8.75, 3.02), GREEN, "back in FOV", rad=0.0, lpos=0.5)
arrow(ax, (4.05, 1.15), (1.9, 3.02), GRAY, "scan finished,\nleader not found", ls="--", rad=-0.2, lpos=0.5, off=(-0.1, -0.55))
ax.plot(0.8, 2.35, "o", color=DARK, ms=10); arrow(ax, (0.8, 2.4), (0.95, 3.02), DARK)
ax.text(0.8, 2.05, "start", ha="center", color=SLATE)
ax.text(5.5, 7.05, "Follower state machine — every control cycle the current state reads the poses "
        "and returns the next state", ha="center", color=SLATE, fontsize=11)
save(fig, "state_machine.png")

# ------------------------------------------------------------------ 4. tracking command
fd = C["tracking_follow_distance"]
fig, ax = plt.subplots(figsize=(8.5, 7.2))
L = np.array([7.0, 3.0]); F = np.array([0.0, 0.0]); cam = np.radians(10)
ax.add_patch(Wedge((0, 0), 9.5, np.degrees(cam - h), np.degrees(cam + h), fc="#EAEFF7", ec="#D6DFEE"))
ax.plot([0, 9 * np.cos(cam)], [0, 9 * np.sin(cam)], ls="--", color=BLUE)
ax.text(9 * np.cos(cam) + 0.1, 9 * np.sin(cam) - 0.35, "optical axis", color=BLUE, fontsize=10.5)
u = (L - F) / np.linalg.norm(L - F); T = L - fd * u
ax.plot([0, L[0]], [0, L[1]], ls=":", color=SLATE)
ax.add_patch(Circle(L, fd, fill=False, ls=":", ec=SLATE))
ax.plot(*T, "X", color=DARK, ms=13, zorder=8)
ax.text(T[0] - 0.2, T[1] + 0.45, "desired point:\nfixed distance\nbehind the leader", fontsize=10.5, color=DARK)
quad(ax, *L, RED); ax.text(L[0] + 0.75, L[1] - 0.55, "leader", color=RED, ha="left", weight="bold")
vl = np.array([1.2, 2.6])
ax.annotate("", L + vl, L, arrowprops=dict(arrowstyle="-|>", color=PURPLE, lw=2.5, mutation_scale=16))
ax.text(*(L + vl + [0.15, 0.05]), "estimated\nleader velocity", color=PURPLE, fontsize=10.5)
ff = vl * 0.9; corr = 0.3 * (T - F); vc = ff + corr
ax.annotate("", ff, (0, 0), arrowprops=dict(arrowstyle="-|>", color=PURPLE, lw=2.2, ls="--", mutation_scale=15))
ax.text(ff[0] - 2.3, ff[1] - 1.0, "feedforward", color=PURPLE, fontsize=10.5)
ax.annotate("", vc, ff, arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=2.5, mutation_scale=15))
ax.text(ff[0] - 1.2, ff[1] + 0.6, "kp·(target − p)", color=ORANGE, fontsize=10.5)
ax.annotate("", vc, (0, 0), arrowprops=dict(arrowstyle="-|>", color=DARK, lw=3.2, mutation_scale=18))
ax.text(vc[0] + 0.2, vc[1] + 0.05, "v_cmd", color=DARK, weight="bold", fontsize=12)
b = np.arctan2(L[1], L[0])
ax.add_patch(Arc((0, 0), 5.2, 5.2, theta1=np.degrees(cam), theta2=np.degrees(b), color=DARK, lw=1.8))
ax.text(2.7, 0.95, "yaw error e_ψ", fontsize=10.5)
ax.annotate("", (1.3, 0.55), (1.2, -0.6), arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=2,
                                                        connectionstyle="arc3,rad=0.3", mutation_scale=14))
ax.text(1.6, -0.85, "ω = kp_angle·e_ψ + bearing rate", color=BLUE, fontsize=10.5)
quad(ax, 0, 0, "#2B4C7E", yaw=cam, s=0.4); ax.text(0, -1.0, "follower", ha="center", color="#2B4C7E", weight="bold")
ax.text(-2.1, 8.0, "v_cmd = v̂_leader + kp·(target − p_follower)\n  → yaw / vertical priorities scale the correction\n"
        "  → rate limiter |Δv| ≤ g·tan(θ_max)·dt  → velocity saturation",
        family="DejaVu Sans Mono", fontsize=10, va="top", bbox=dict(fc="#F4F6FA", ec="#D5D9E0", boxstyle="round,pad=0.4"))
ax.set_aspect("equal"); ax.set_xlim(-2.3, 11); ax.set_ylim(-2.3, 8.3)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("TrackingController command (top view)", weight="bold", color=DARK, fontsize=14)
save(fig, "tracking_command.png")

# ------------------------------------------------------------------ 5. priorities
fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.4))
for ax, s, e, col, xl, ttl, note in [
    (a1, C["tracking_yaw_priority_start_deg"], C["fov_half_angle_h_deg"], BLUE, "|yaw error|",
     "Yaw priority", "near the side edge the xy correction\nfades out: first turn to re-centre,\nthen move"),
    (a2, C["tracking_vprio_start_deg"], C["fov_half_angle_v_deg"], ORANGE, "|vertical angle of the leader|",
     "Vertical priority", "near the top/bottom edge only the\nin-plane approach fades out;\naltitude and backing off stay full")]:
    x = np.linspace(0, e * 1.2, 400); y = np.clip((e - x) / (e - s), 0, 1)
    ax.plot(x, y, color=col, lw=3.2)
    ax.axvspan(e, e * 1.2, color="#F3DEDB"); ax.axvline(s, color=SLATE, ls=":", lw=1)
    ax.text(e * 1.01, 0.5, "outside\nthe FOV", color=RED, fontsize=10)
    ax.text(s, 1.08, "start", ha="center", color=SLATE, fontsize=10)
    ax.text(e, 1.08, "FOV edge", ha="center", color=RED, fontsize=10)
    ax.text(e * 0.04, 0.3, note, fontsize=10, color=DARK)
    ax.set_xlim(0, e * 1.2); ax.set_ylim(-0.05, 1.18); ax.set_xticks([])
    ax.set_xlabel(xl); ax.set_title(ttl, color=DARK)
a1.set_ylabel("factor on the correction")
save(fig, "priorities.png")

# ------------------------------------------------------------------ 6. searching
fig = plt.figure(figsize=(14, 6.2))
ax = fig.add_axes([0.05, 0.1, 0.48, 0.78])
Fp = np.array([0, 0]); last = np.array([7.5, 1.0]); vhat = np.array([0.25, 1.1]); pred = last + vhat * 3
ad = C["search_approach_dist"]
yaw0 = np.radians(20)
ax.add_patch(Wedge((0, 0), 7, np.degrees(yaw0 - h), np.degrees(yaw0 + h), fc="#EEF1F8", ec="#D6DFEE", ls="--"))
d = pred - Fp; dn = np.linalg.norm(d); rec = Fp + (dn - ad) * d / dn
b = np.degrees(np.arctan2(d[1], d[0]))
ax.add_patch(Wedge(rec, 7, b - np.degrees(h), b + np.degrees(h), fc=FOV_FILL, ec="#C9D4EA"))
ax.add_patch(Circle(pred, ad, fill=False, ls=":", ec=SLATE))
ax.text(pred[0] - ad * 0.6, pred[1] + ad + 0.2, "approach distance", color=SLATE, fontsize=10)
ax.annotate("", rec, (0.3, 0.15), arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=3, mutation_scale=16))
ax.text(2.4, 2.2, "recovery:\nturn + approach\n+ go to its altitude", color=ORANGE, ha="right", fontsize=10.5)
ax.annotate("", pred, last, arrowprops=dict(arrowstyle="-|>", color=PURPLE, lw=2, ls="--", mutation_scale=14))
ax.text(last[0] + 0.5, last[1] + 1.3, "v̂·(t_lost + T)", color=PURPLE, fontsize=10.5)
ax.plot(*last, "o", mfc="white", mec=RED, mew=2, ms=12); ax.text(last[0] + 0.3, last[1] - 0.4, "last sighting", color=RED, fontsize=10.5)
ax.plot(*pred, "o", color=RED, ms=13); ax.text(pred[0] + 0.35, pred[1], "predicted position", color=RED, fontsize=10.5, va="center")
quad(ax, 0, 0, "#2B4C7E", yaw=yaw0, s=0.4); ax.text(0, -1.1, "follower", ha="center", color="#2B4C7E", weight="bold")
ax.set_aspect("equal"); ax.set_xlim(-2.5, 13); ax.set_ylim(-2.5, 9)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_title("Phase R: recovery on the predicted position", color=DARK)
ax2 = fig.add_axes([0.57, 0.08, 0.42, 0.8]); ax2.axis("off"); ax2.set_xlim(0, 10); ax2.set_ylim(0, 8)
steps = [("R  recovery", ORANGE, "smoothstep to the recovery point,\npredicted leader centred"),
         ("R  hold", "#E2B78A", "hover and look"),
         ("G1  go home", GRAY, "back to the search home"),
         ("G2  hold", "#C9CDD8", ""),
         ("G3  climb spinning", BLUE, "camera sweeps 360°"),
         ("G4  hold", "#AFC1E2", ""),
         ("G5  descend spinning", BLUE, "back home, still spinning")]
for i, (n, c, note) in enumerate(steps):
    y = 7.3 - i * 1.0
    ax2.add_patch(FancyBboxPatch((0.2, y - 0.35), 4.2, 0.7, boxstyle="round,pad=0,rounding_size=0.08", fc=c, ec="none"))
    ax2.text(2.3, y, n, ha="center", va="center", color="white", weight="bold", fontsize=12)
    ax2.text(4.65, y, note, va="center", fontsize=10, color=DARK)
    if i < len(steps) - 1:
        ax2.annotate("", (2.3, y - 0.62), (2.3, y - 0.38), arrowprops=dict(arrowstyle="-|>", color=SLATE))
ax2.text(0.2, 0.25, "At any time: leader in FOV → TRACKING.   End of sequence → IDLE.", color=GREEN, weight="bold", fontsize=11)
ax2.set_title("Search manoeuvre sequence", color=DARK)
fig.suptitle("SEARCHING: entered when the leader stays out of the FOV during TRACKING", weight="bold", color=DARK, fontsize=14)
save(fig, "searching.png")

# ------------------------------------------------------------------ 7. exploring
(xm, xM), (ym, yM), (zm, zM) = C["explore_bounds"]
rr = 0.5 * np.hypot(xM - xm, yM - ym) + C["explore_ring_clearance"]
zl, zh = zm + 1.0, zM - C["explore_top_clearance"]
fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw=dict(width_ratios=[1, 1.15]))
ax = a1
ax.add_patch(plt.Rectangle((xm, ym), xM - xm, yM - ym, fc="#F0F3F8", ec="#DCE2EC"))
ax.text(xm + 0.4, yM - 1.0, "exploration volume", color=BLUE, fontsize=10.5)
t = np.linspace(0, 2 * np.pi, 300); cx, cyy = (xm + xM) / 2, (ym + yM) / 2
ax.plot(cx + rr * np.cos(t), cyy + rr * np.sin(t), color=BLUE, lw=3)
for a in np.linspace(0, 2 * np.pi, 9)[:-1] + 0.3:
    p = np.array([cx + rr * np.cos(a), cyy + rr * np.sin(a)]); q = p + 0.25 * (np.array([cx, cyy]) - p)
    ax.annotate("", q, p, arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=1.3))
ax.annotate("", (cx - 0.5, cyy + rr), (cx + 0.5, cyy + rr), arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=3, mutation_scale=22))
ax.plot(cx, cyy, "+", color=DARK, ms=18, mew=2)
ax.text(cx, cyy - 3.6, "centre: the camera\nalways points here", ha="center", color=BLUE, fontsize=10.5)
fs_ = np.array([20.0, 5.0]); ang = np.arctan2(fs_[1] - cyy, fs_[0] - cx); ent = np.array([cx + rr * np.cos(ang), cyy + rr * np.sin(ang)])
ax.plot([fs_[0], ent[0]], [fs_[1], ent[1]], ls="--", color=DARK, lw=2)
ax.text(17.5, 1.2, "connector", ha="center", fontsize=10.5)
quad(ax, *fs_, "#2B4C7E", s=0.75); ax.text(fs_[0], fs_[1] + 1.4, "follower", ha="center", color="#2B4C7E", weight="bold")
ax.set_aspect("equal"); ax.set_xlim(-rr - 2, 24); ax.set_ylim(-rr - 2, rr + 2)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_title("Top view: ring around the volume", color=DARK)
ax = a2
th = np.linspace(0, 360, 400); tau = np.where(th <= 180, th / 180, (th - 180) / 180)
s5 = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
z = np.where(th <= 180, zl + (zh - zl) * s5, zh - (zh - zl) * s5)
ax.axhspan(zm, zM, color="#F0F3F8")
ax.plot(th, z, color=BLUE, lw=3.2)
ax.axhline(zl, ls=":", color=SLATE, lw=1); ax.axhline(zh, ls=":", color=SLATE, lw=1)
ax.text(5, zl - 0.6, "low altitude", color=SLATE, fontsize=10); ax.text(5, zh + 0.3, "high altitude", color=SLATE, fontsize=10)
ax.set_xticks([0, 90, 180, 270, 360]); ax.set_xlim(0, 360); ax.set_ylim(zm - 1.5, zM + 1.5)
ax.set_xlabel("angle travelled on the ring [°]"); ax.set_ylabel("altitude [m]")
ax.set_title("Altitude along the lap: climbs in the first half, descends in the second", color=DARK)
fig.suptitle("EXPLORING: entered when the leader has not been seen for a while in IDLE", weight="bold", color=DARK, fontsize=14)
save(fig, "exploring.png")
print(sorted(os.listdir(OUT)))
