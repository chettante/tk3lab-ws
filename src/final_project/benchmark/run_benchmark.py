#!/usr/bin/env python3
"""
Runs the benchmark scenarios one after the other, each in a fresh simulation:
  1. writes the overrides of the scenario (CONFIG values that change) to a JSON file
  2. starts simulation/quad_simulation.sh (it also syncs camera and follower spawn)
  3. waits for the components, runs main_statemachine.py
  4. stops the simulation and copies logs and plots to results/<campaign>/<id>/

Usage (inside the tk3lab container, NOT with the simulation already running):
    python3 run_benchmark.py                       # all scenarios, campaign named by date/time
    python3 run_benchmark.py --campaign c1 --phase A B
    python3 run_benchmark.py --campaign c1 --only D21 D22
    python3 run_benchmark.py --campaign c1 --resume    # skip the runs already completed
Then: python3 analyze_benchmark.py results/<campaign>
"""

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)                     # final_project/
SRC = os.path.dirname(PROJECT)
sys.path.insert(0, SRC)
from final_project.benchmark.scenarios import SCENARIOS    # noqa: E402

SIM_SCRIPT = os.path.join(PROJECT, 'simulation', 'quad_simulation.sh')
MAIN = os.path.join(PROJECT, 'main_statemachine.py')
TMP = '/tmp/bm'                     # short paths: GenoM accepts log paths of at most 64 characters


def stop_sim(proc):
    """Ctrl-C to the whole simulation (its trap kills components, Gazebo and h2), then force."""
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(30)
        return
    except ProcessLookupError:
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(10)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    # after a SIGKILL the trap does not run: clean up what it would have cleaned
    subprocess.run('pkill -f "gz sim"; h2 end', shell=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run_once(sc, run_dir, args):
    """One attempt: returns (main return code, main duration [s])."""
    tmp = os.path.join(TMP, sc['id'])
    shutil.rmtree(tmp, ignore_errors=True)
    overrides = dict(sc['overrides'], output_dir=tmp, show_plots=False,
                     genom_log_dir=os.path.join(TMP, 'genom'))
    ov_path = os.path.join(run_dir, 'overrides.json')
    with open(ov_path, 'w') as f:
        json.dump(overrides, f, indent=2)
    env = dict(os.environ, DRONE_CHASE_OVERRIDES=ov_path, MPLBACKEND='Agg')

    with open(os.path.join(run_dir, 'sim.log'), 'w') as sim_log:
        sim = subprocess.Popen(args.sim_cmd.split(), env=env, stdout=sim_log,
                               stderr=subprocess.STDOUT, start_new_session=True)
        try:
            time.sleep(args.sim_startup)
            t0 = time.monotonic()
            with open(os.path.join(run_dir, 'main.log'), 'w') as main_log:
                try:
                    rc = subprocess.run([sys.executable, MAIN], env=env, stdout=main_log,
                                        stderr=subprocess.STDOUT, timeout=args.timeout).returncode
                except subprocess.TimeoutExpired:
                    rc = 'timeout'
            duration = time.monotonic() - t0
        finally:
            stop_sim(sim)
    time.sleep(args.pause)

    for sub in ('log', 'plot'):
        if os.path.isdir(os.path.join(tmp, sub)):
            shutil.copytree(os.path.join(tmp, sub), os.path.join(run_dir, sub), dirs_exist_ok=True)
    return rc, duration


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default=time.strftime('%Y%m%d_%H%M'))
    ap.add_argument('--phase', nargs='+', help='only these phases (A B C D E)')
    ap.add_argument('--only', nargs='+', help='only these scenario ids')
    ap.add_argument('--resume', action='store_true', help='skip the runs with status ok')
    ap.add_argument('--sim-startup', type=float, default=20.0,
                    help='seconds between starting the simulation and running the main')
    ap.add_argument('--pause', type=float, default=3.0, help='seconds after stopping the simulation')
    ap.add_argument('--timeout', type=float, default=900.0, help='seconds before the main is stopped')
    ap.add_argument('--sim-cmd', default=f'sh {SIM_SCRIPT}', help='command that starts the simulation')
    args = ap.parse_args()

    todo = [s for s in SCENARIOS
            if (not args.phase or s['phase'] in args.phase) and (not args.only or s['id'] in args.only)]
    out = os.path.join(HERE, 'results', args.campaign)
    print(f'{len(todo)} scenarios -> {out}')

    for i, sc in enumerate(todo, 1):
        run_dir = os.path.join(out, sc['id'])
        status_path = os.path.join(run_dir, 'status.json')
        if args.resume and os.path.isfile(status_path):
            with open(status_path) as f:
                if json.load(f).get('ok'):
                    print(f'[{i}/{len(todo)}] {sc["id"]} already done')
                    continue
        os.makedirs(run_dir, exist_ok=True)
        with open(os.path.join(run_dir, 'scenario.json'), 'w') as f:
            json.dump(sc, f, indent=2)

        print(f'[{i}/{len(todo)}] {sc["id"]} {sc["label"]} ...', end=' ', flush=True)
        rc, duration = run_once(sc, run_dir, args)
        if rc != 0 and duration < 30:
            # usually the components were not ready yet: retry once with a longer wait
            print(f'failed after {duration:.0f}s (rc={rc}), retrying ...', end=' ', flush=True)
            args_retry = argparse.Namespace(**{**vars(args), 'sim_startup': 2 * args.sim_startup})
            rc, duration = run_once(sc, run_dir, args_retry)
        ok = rc == 0 and os.path.isfile(os.path.join(run_dir, 'log', 'pom.log'))
        with open(status_path, 'w') as f:
            json.dump({'ok': ok, 'returncode': rc, 'main_duration_s': round(duration, 1)}, f)
        print(f'{"ok" if ok else "FAILED"} ({duration:.0f}s)')

    print(f'Done. Analysis: python3 {os.path.join(HERE, "analyze_benchmark.py")} {out}')


if __name__ == '__main__':
    main()
