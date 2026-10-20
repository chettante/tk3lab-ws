#!/usr/bin/env python3
"""
Analyzes a benchmark campaign written by run_benchmark.py.

For every run it reads the pom logs of leader and follower (same FOV geometry as
graphics/tracking_plotter.py) and the main log (state transitions), then writes to
results/<campaign>/report/:
  - summary.csv / summary.md   one row per run with the metrics below
  - runs/<id>.png              the tracking_plotter figure of every run
  - phase_<X>.png              comparison plots per phase

Metrics (from the start of the leader trajectory = leader on leader_start_pos + takeoff hold):
  fov_pct          % of time with the leader in the FOV
  t_first_s        time to the first acquisition
  losses           exits from the FOV after the first acquisition (longer than 0.1 s)
  long_losses      losses longer than out_of_fov_timeout (-> SEARCHING)
  loss_mean_s / loss_max_s   duration of the losses
  exit_side / exit_vertical / exit_range / exit_behind   where the leader left the FOV
  dist_err_rms_m   RMS of (horizontal distance - follow_distance) with the leader in the FOV
  dist_max_m       maximum 3D distance
  yaw_err_rms_deg  RMS of the yaw error with the leader in the FOV
  elev_p95_deg     95th percentile of |vertical angle| of the leader in the image (in FOV)
  tilt_p95_deg / tilt_max_deg   follower tilt
  leader_v_max / leader_a_p95   speed and acceleration the leader actually had
  searching / reacquired / exploring / gave_up / cmd_failed   counts from main.log

Usage:
    python3 analyze_benchmark.py results/<campaign> [--no-run-plots]
"""

import argparse
import csv
import json
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt    # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
sys.path.insert(0, os.path.dirname(PROJECT))
from final_project.config import CONFIG    # noqa: E402
from final_project.graphics.tracking_plotter import (    # noqa: E402
    load_pom_pose, resample, camera_coords, in_fov_mask, segments)

RATE = 20.0
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']          # categorical, fixed order
LEVEL_RAMP = {1: '#86b6ef', 2: '#3987e5', 3: '#1c5cab', 4: '#0d366b'}   # ordinal (blue ramp)
INK, MUTED, GRID = '#1f1f1e', '#6b6a64', '#e4e3df'
LOG_EVENTS = {
    'searching': 'transitioning to SEARCHING',
    'reacquired': 'Leader re-acquired',
    'exploring': 'Transitioning to EXPLORING',
    'gave_up': 'finished without finding the leader',
    'cmd_failed': 'follower.velocity failed',
}


def style(ax):
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)


def load_run(run_dir):
    with open(os.path.join(run_dir, 'scenario.json')) as f:
        sc = json.load(f)
    with open(os.path.join(run_dir, 'overrides.json')) as f:
        cfg = dict(CONFIG, **json.load(f))

    tl, pl, al = load_pom_pose(os.path.join(run_dir, 'log', 'pom.log'))
    tf, pf, af = load_pom_pose(os.path.join(run_dir, 'log', 'pom_f.log'))
    t_a, t_b = max(tl.min(), tf.min()), min(tl.max(), tf.max())
    tg = np.arange(t_a, t_b, 1.0 / RATE)
    L, _ = resample(tl, pl, al, tg)
    F, att = resample(tf, pf, af, tg)

    # start of the leader trajectory: leader on its start position, plus the hover
    near = np.linalg.norm(L - np.asarray(cfg['leader_start_pos']), axis=1) < 0.3
    i0 = int(np.argmax(near)) if near.any() else 0
    i0 = min(i0 + int(cfg['leader_takeoff_hold'] * RATE), len(tg) - 2)
    t = tg[i0:] - tg[i0]
    L, F, att = L[i0:], F[i0:], att[i0:]

    rel = L - F
    cam_yaw = np.radians(cfg['camera_yaw_deg'])
    cam = camera_coords(rel, att, cfg['camera_gimbal'], cam_yaw)
    fov = in_fov_mask(cam, cfg['fov_half_angle_h_deg'], cfg['fov_half_angle_v_deg'], cfg['fov_max_range'])
    bearing = np.arctan2(rel[:, 1], rel[:, 0])
    yaw = att[:, 2] + cam_yaw
    yaw_err = np.degrees(np.arctan2(np.sin(bearing - yaw), np.cos(bearing - yaw)))
    elev = np.degrees(np.arctan2(cam[:, 2], cam[:, 0]))
    tilt = np.degrees(np.arccos(np.clip(np.cos(att[:, 0]) * np.cos(att[:, 1]), -1, 1)))
    d_xy = np.linalg.norm(rel[:, :2], axis=1)

    # leader velocity/acceleration from the positions, smoothed over 0.25 s
    k = np.ones(5) / 5
    vl = np.column_stack([np.convolve(np.gradient(L[:, i], t), k, 'same') for i in range(3)])
    al_ = np.column_stack([np.convolve(np.gradient(vl[:, i], t), k, 'same') for i in range(3)])

    m = dict(id=sc['id'], phase=sc['phase'], label=sc['label'], x=sc['x'], duration_s=t[-1])
    m['fov_pct'] = 100 * fov.mean()
    first = int(np.argmax(fov)) if fov.any() else None
    m['t_first_s'] = t[first] if first is not None else np.nan
    lost = [(a, b) for a, b in segments(~fov, t)
            if first is not None and a > t[first] and b - a > 0.1]
    gaps = [b - a for a, b in lost]
    m['losses'] = len(gaps)
    # where the leader left the FOV, from the first out-of-FOV sample of each loss:
    # behind = closer than the near plane or behind the camera, range = beyond fov_max_range,
    # side = horizontal FOV edge, vertical = vertical FOV edge
    for k in ('exit_behind', 'exit_range', 'exit_side', 'exit_vertical'):
        m[k] = 0
    tan_h, tan_v = np.tan(np.radians([cfg['fov_half_angle_h_deg'], cfg['fov_half_angle_v_deg']]))
    for a, _ in lost:
        x, y, z = cam[np.searchsorted(t, a)]
        if x < 0.1:
            m['exit_behind'] += 1
        elif x > cfg['fov_max_range']:
            m['exit_range'] += 1
        elif abs(y) > x * tan_h:
            m['exit_side'] += 1
        else:
            m['exit_vertical'] += 1
    m['long_losses'] = sum(g > cfg['out_of_fov_timeout'] for g in gaps)
    m['loss_mean_s'] = np.mean(gaps) if gaps else 0.0
    m['loss_max_s'] = max(gaps) if gaps else 0.0
    inside = fov if fov.any() else np.ones_like(fov)
    m['dist_err_rms_m'] = np.sqrt(np.mean((d_xy[inside] - cfg['tracking_follow_distance']) ** 2))
    m['dist_max_m'] = np.linalg.norm(rel, axis=1).max()
    m['yaw_err_rms_deg'] = np.sqrt(np.mean(yaw_err[inside] ** 2))
    m['elev_p95_deg'] = np.percentile(np.abs(elev[inside]), 95)
    m['tilt_p95_deg'] = np.percentile(tilt, 95)
    m['tilt_max_deg'] = tilt.max()
    m['leader_v_max'] = np.linalg.norm(vl, axis=1).max()
    m['leader_a_p95'] = np.percentile(np.linalg.norm(al_, axis=1), 95)

    log_path = os.path.join(run_dir, 'main.log')
    text = open(log_path, errors='replace').read() if os.path.isfile(log_path) else ''
    for key, pattern in LOG_EVENTS.items():
        m[key] = text.count(pattern)

    series = dict(t=t, d_xy=d_xy, elev=elev, fov=fov, follow=cfg['tracking_follow_distance'],
                  vprio=cfg['tracking_vprio_start_deg'], fov_v=cfg['fov_half_angle_v_deg'],
                  crop=tg[i0] - t_a)
    return m, series


def run_plot(run_dir, out_png, crop):
    """Figure of graphics/tracking_plotter.py, with the same overrides as the run."""
    env = dict(os.environ, DRONE_CHASE_OVERRIDES=os.path.join(run_dir, 'overrides.json'), MPLBACKEND='Agg')
    subprocess.run([sys.executable, os.path.join(PROJECT, 'graphics', 'tracking_plotter.py'),
                    '--leader', os.path.join(run_dir, 'log', 'pom.log'),
                    '--follower', os.path.join(run_dir, 'log', 'pom_f.log'),
                    '--out', out_png, '--crop', f'{crop:.2f}', '1e9'],
                   env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def plot_time_series(rows, series, key, ylabel, title, out, hlines=(), ylim=None):
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for i, r in enumerate(rows[:len(SERIES)]):
        s = series[r['id']]
        ax.plot(s['t'], s[key], color=SERIES[i], lw=1.6, label=f"{r['id']} {r['label']}")
    for y, lab in hlines:
        ax.axhline(y, color=MUTED, ls='--', lw=1)
        ax.annotate(lab, (0, y), xytext=(4, 3), textcoords='offset points', fontsize=8, color=MUTED)
    style(ax)
    if ylim:
        ax.set_ylim(*ylim)
    ax.set_xlabel('Time from the start of the leader trajectory [s]', color=INK)
    ax.set_ylabel(ylabel, color=INK)
    ax.set_title(title, color=INK, loc='left')
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)


def plot_vs_x(rows, groups, metrics, xlabel, title, out, vline=None):
    """One panel per metric; one series per group (label prefix), points at x."""
    fig, axes = plt.subplots(1, len(metrics), figsize=(4.2 * len(metrics), 3.8))
    for ax, (key, ylab) in zip(np.atleast_1d(axes), metrics):
        for i, g in enumerate(groups):
            rs = sorted([r for r in rows if r['label'].startswith(g)], key=lambda r: r['x'])
            if rs:
                ax.plot([r['x'] for r in rs], [r[key] for r in rs], '-o', color=SERIES[i], lw=2,
                        ms=8, mec='white', mew=2, label=g)
        if vline:
            ax.axvline(vline[0], color=MUTED, ls='--', lw=1)
            ax.annotate(vline[1], (vline[0], 1), xycoords=('data', 'axes fraction'), xytext=(4, -12),
                        textcoords='offset points', fontsize=8, color=MUTED)
        style(ax)
        ax.set_xlabel(xlabel, color=INK)
        ax.set_ylabel(ylab, color=INK)
    if len(groups) > 1:
        np.atleast_1d(axes)[0].legend(fontsize=8, frameon=False)
    fig.suptitle(title, color=INK, x=0.01, ha='left')
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)


def plot_levels(rows, title, out, metrics):
    """Phases D/E: each seed is a point, the line joins the mean per level."""
    fig, axes = plt.subplots(1, len(metrics), figsize=(4.2 * len(metrics), 3.8))
    for ax, (key, ylab) in zip(np.atleast_1d(axes), metrics):
        levels = sorted({r['x'] for r in rows})
        means = [np.nanmean([r[key] for r in rows if r['x'] == lv]) for lv in levels]
        ax.plot(levels, means, '-', color=MUTED, lw=1.5, zorder=1, label='mean')
        for r in rows:
            ax.plot(r['x'], r[key], 'o', color=LEVEL_RAMP.get(int(r['x']), INK), ms=9,
                    mec='white', mew=2, zorder=2)
        style(ax)
        ax.set_xticks(levels)
        ax.set_xlabel('Leader level (1 slow → 4 aggressive)', color=INK)
        ax.set_ylabel(ylab, color=INK)
    fig.suptitle(title, color=INK, x=0.01, ha='left')
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('campaign_dir')
    ap.add_argument('--no-run-plots', action='store_true', help='skip the per-run figures')
    args = ap.parse_args()

    report = os.path.join(args.campaign_dir, 'report')
    os.makedirs(os.path.join(report, 'runs'), exist_ok=True)
    rows, series = [], {}
    for name in sorted(os.listdir(args.campaign_dir)):
        run_dir = os.path.join(args.campaign_dir, name)
        if not os.path.isfile(os.path.join(run_dir, 'log', 'pom_f.log')):
            continue
        try:
            m, s = load_run(run_dir)
        except Exception as e:                       # one broken run must not stop the report
            print(f'{name}: skipped ({e})')
            continue
        rows.append(m)
        series[m['id']] = s
        if not args.no_run_plots:
            run_plot(run_dir, os.path.join(report, 'runs', f'{name}.png'), s['crop'])
        print(f"{name}: FOV {m['fov_pct']:.0f}%, {m['losses']} losses ({m['long_losses']} long)")
    if not rows:
        sys.exit('No runs with logs found')

    keys = [k for k in rows[0] if k not in ('x',)]
    with open(os.path.join(report, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    md_keys = ['id', 'label', 'fov_pct', 't_first_s', 'losses', 'long_losses', 'loss_max_s',
               'exit_side', 'exit_vertical', 'exit_range', 'exit_behind',
               'dist_err_rms_m', 'yaw_err_rms_deg', 'elev_p95_deg', 'tilt_p95_deg', 'leader_a_p95']
    with open(os.path.join(report, 'summary.md'), 'w') as f:
        f.write('| ' + ' | '.join(md_keys) + ' |\n|' + '---|' * len(md_keys) + '\n')
        for r in rows:
            f.write('| ' + ' | '.join(f'{r[k]:.2f}' if isinstance(r[k], float) else str(r[k])
                                      for k in md_keys) + ' |\n')

    by = {p: [r for r in rows if r['phase'] == p] for p in 'ABCDE'}
    a_track = 9.81 * np.tan(np.radians(CONFIG['tracking_max_tilt_deg']))
    if by['A']:
        f0 = series[by['A'][0]['id']]['follow']
        plot_time_series(by['A'], series, 'd_xy', 'Horizontal distance [m]',
                         'A - hover and straight line: distance from the leader',
                         os.path.join(report, 'phase_A.png'),
                         hlines=[(f0, 'follow distance'), (CONFIG['fov_max_range'], 'FOV range')])
    if by['B']:
        plot_vs_x(by['B'], ['circle', 'figure-8'],
                  [('fov_pct', 'Time in FOV [%]'), ('dist_err_rms_m', 'Distance error RMS [m]'),
                   ('yaw_err_rms_deg', 'Yaw error RMS [deg]')],
                  'Leader speed [m/s]', 'B - curves', os.path.join(report, 'phase_B.png'),
                  vline=(np.sqrt(a_track * 10.0), 'circle limit √(a·R)'))
    if by['C']:
        s0 = series[by['C'][0]['id']]
        plot_time_series(by['C'], series, 'elev', 'Vertical angle of the leader [deg]',
                         'C - vertical motion: leader position in the image',
                         os.path.join(report, 'phase_C.png'),
                         hlines=[(s0['fov_v'], 'FOV edge'), (-s0['fov_v'], ''),
                                 (s0['vprio'], 'vertical priority')],
                         ylim=(-1.5 * s0['fov_v'], 1.5 * s0['fov_v']))
    if by['D']:
        plot_levels(by['D'], 'D - leader aggressiveness (random waypoints, 3 seeds per level)',
                    os.path.join(report, 'phase_D.png'),
                    [('fov_pct', 'Time in FOV [%]'), ('long_losses', 'Losses -> SEARCHING'),
                     ('loss_mean_s', 'Mean loss duration [s]')])
    if by['E']:
        plot_levels(by['E'], 'E - search from far spawn',
                    os.path.join(report, 'phase_E.png'),
                    [('t_first_s', 'Time to first acquisition [s]'), ('fov_pct', 'Time in FOV [%]'),
                     ('long_losses', 'Losses -> SEARCHING')])
    print(f'Report in {report}')


if __name__ == '__main__':
    main()
