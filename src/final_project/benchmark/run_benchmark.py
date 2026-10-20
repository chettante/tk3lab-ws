#!/usr/bin/env python3
"""
Runs the benchmark scenarios one after the other in ONE simulation.

The simulation (Gazebo + GenoM3 components) is started once; the drones take off once and,
between two scenarios, fly back to the start positions (main_statemachine.go_to_start), so
also the follower spawn (near / far) changes without restarting Gazebo. The simulation is
restarted only if a run leaves the drones unable to reach the start (e.g. after a crash).

Usage (inside the tk3lab container):
    python3 run_benchmark.py --campaign c1 --only A01 A03     # a few scenarios
    python3 run_benchmark.py --campaign c1 --resume           # all of them, skipping the done ones
    python3 run_benchmark.py --campaign c1 --sim-running      # use a simulation started by hand
Then: python3 analyze_benchmark.py results/<campaign>

Each run is in results/<campaign>/<id>/: main.log, log/pom*.log, overrides.json, status.json.
"""

import argparse
import copy
import json
import logging
import os
import shutil
import signal
import subprocess
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)                     # final_project/
SRC = os.path.dirname(PROJECT)
sys.path.insert(0, SRC)
from final_project.benchmark.scenarios import SCENARIOS    # noqa: E402

SIM_SCRIPT = os.path.join(PROJECT, 'simulation', 'quad_simulation.sh')
TMP = '/tmp/bm'                     # short paths: GenoM accepts log paths of at most 64 characters
EVENTS = {'tracking': 'Transitioning to TRACKING', 'lost': 'transitioning to SEARCHING',
          'reacquired': 'Leader re-acquired', 'exploring': 'Transitioning to EXPLORING'}
# the simulation is ready when the 9 GenoM3 components are running and Gazebo has created both drones
READY_COMPONENTS = 9
READY_GAZEBO = 'mrsim created /tmp/pty-qr4_following'
SESSION_BROKEN = 3                  # exit code of the worker: the drones cannot reach the start
MAX_ATTEMPTS = 2                    # per scenario


# ---------------------------------------------------------------------------
# simulation
# ---------------------------------------------------------------------------

def cleanup():
    """Gazebo and pocolibs devices left by a previous simulation: with them 'h2 init' fails.
    pkill gets the pattern as an argument (a shell running 'pkill -f "gz sim"' would kill itself)."""
    for cmd in (['pkill', '-f', 'gz sim'], ['h2', 'end']):
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def start_sim(log_path, max_wait, settle):
    """Starts quad_simulation.sh (Gazebo unpaused) and waits until it is ready. Returns the process."""
    cleanup()
    log = open(log_path, 'w')
    proc = subprocess.Popen(['sh', SIM_SCRIPT], env=dict(os.environ, GZ_AUTORUN='1'), stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    t_end = time.monotonic() + max_wait
    while time.monotonic() < t_end and proc.poll() is None:
        with open(log_path, errors='replace') as f:
            text = f.read()
        if text.count('setup and running') >= READY_COMPONENTS and READY_GAZEBO in text:
            time.sleep(settle)
            return proc
        time.sleep(0.5)
    stop_sim(proc)
    raise RuntimeError(f'simulation not ready after {max_wait:.0f}s, see {log_path}')


def stop_sim(proc):
    """Ctrl-C to the whole simulation (its trap kills components, Gazebo and h2), then force."""
    if proc is None:
        return
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(10)
        except ProcessLookupError:
            pass
    cleanup()


# ---------------------------------------------------------------------------
# worker: one simulation, several scenarios
# ---------------------------------------------------------------------------

def wait_genomix(timeout=30.0):
    """genomixd accepts connections a bit after the components are up."""
    import genomix
    t_end = time.monotonic() + timeout
    while True:
        try:
            genomix.connect()
            return
        except OSError:
            if time.monotonic() > t_end:
                raise
            time.sleep(1.0)


def worker(campaign_dir, ids):
    wait_genomix()
    from final_project.config import CONFIG
    base = copy.deepcopy(CONFIG)
    import final_project.main_statemachine as ms        # connects to genomix
    from final_project.components.functions import setup, start, stop

    setup()
    start()
    root = logging.getLogger()
    try:
        for sc in (s for s in SCENARIOS if s['id'] in ids):
            run_dir = os.path.join(campaign_dir, sc['id'])
            status = read_status(run_dir)
            if status.get('ok'):
                continue
            tmp = os.path.join(TMP, sc['id'])
            shutil.rmtree(tmp, ignore_errors=True)
            overrides = dict(sc['overrides'], output_dir=tmp, show_plots=False,
                             genom_log_dir=os.path.join(TMP, 'genom'))
            with open(os.path.join(run_dir, 'overrides.json'), 'w') as f:
                json.dump(overrides, f, indent=2)
            CONFIG.clear()
            CONFIG.update(copy.deepcopy(base), **overrides)

            handler = logging.FileHandler(os.path.join(run_dir, 'main.log'), mode='w')
            handler.setFormatter(logging.Formatter('[%(asctime)s] [%(name)s] %(levelname)s: %(message)s'))
            root.addHandler(handler)
            t0 = time.monotonic()
            broken = False
            try:
                ms.run_chase()
                error = None
            except Exception as e:                     # one run must not stop the others
                error = repr(e)
                logging.getLogger('Benchmark').error(traceback.format_exc())
                # the drones did not reach the start: this simulation is not usable any more
                broken = isinstance(e, RuntimeError) and 'not on their targets' in str(e)
            finally:
                root.removeHandler(handler)
                handler.close()
            for sub in ('log', 'plot'):
                if os.path.isdir(os.path.join(tmp, sub)):
                    shutil.copytree(os.path.join(tmp, sub), os.path.join(run_dir, sub), dirs_exist_ok=True)
            ok = error is None and os.path.isfile(os.path.join(run_dir, 'log', 'pom.log'))
            write_status(run_dir, ok=ok, error=error, duration_s=round(time.monotonic() - t0, 1),
                         attempts=status.get('attempts', 0) + 1)
            print_result(sc, run_dir)
            if broken:
                return SESSION_BROKEN
    finally:
        CONFIG.clear()
        CONFIG.update(base)
        stop()
    return 0


# ---------------------------------------------------------------------------
# bookkeeping
# ---------------------------------------------------------------------------

def read_status(run_dir):
    path = os.path.join(run_dir, 'status.json')
    if os.path.isfile(path):
        with open(path) as f:
            return json.load(f)
    return {}


def write_status(run_dir, **status):
    with open(os.path.join(run_dir, 'status.json'), 'w') as f:
        json.dump(status, f)


def print_result(sc, run_dir):
    st = read_status(run_dir)
    path = os.path.join(run_dir, 'main.log')
    text = open(path, errors='replace').read() if os.path.isfile(path) else ''
    events = ', '.join(f'{k} {text.count(v)}' for k, v in EVENTS.items())
    print(f'  {sc["id"]} {sc["label"]}: {"ok" if st.get("ok") else "FAILED"} '
          f'({st.get("duration_s", 0):.0f}s) - {events}', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default=time.strftime('%Y%m%d_%H%M'))
    ap.add_argument('--phase', nargs='+', help='only these phases (A B C D E)')
    ap.add_argument('--only', nargs='+', help='only these scenario ids')
    ap.add_argument('--resume', action='store_true', help='skip the runs already completed')
    ap.add_argument('--sim-running', action='store_true',
                    help='use the simulation already running (started by hand), never restart it')
    ap.add_argument('--sim-startup', type=float, default=90.0, help='max seconds to wait for the simulation')
    ap.add_argument('--settle', type=float, default=3.0, help='seconds after the simulation is ready')
    ap.add_argument('--worker', nargs='+', help=argparse.SUPPRESS)       # internal
    args = ap.parse_args()

    out = os.path.join(HERE, 'results', args.campaign)
    if args.worker:
        sys.exit(worker(out, args.worker))

    todo = [s for s in SCENARIOS
            if (not args.phase or s['phase'] in args.phase) and (not args.only or s['id'] in args.only)]
    for sc in todo:
        run_dir = os.path.join(out, sc['id'])
        os.makedirs(run_dir, exist_ok=True)
        if not (args.resume and read_status(run_dir).get('ok')):
            write_status(run_dir, ok=False, attempts=0)
        with open(os.path.join(run_dir, 'scenario.json'), 'w') as f:
            json.dump(sc, f, indent=2)
    print(f'{len(todo)} scenarios -> {out}')

    session = 0
    while True:
        left = [s['id'] for s in todo if not read_status(os.path.join(out, s['id'])).get('ok')
                and read_status(os.path.join(out, s['id'])).get('attempts', 0) < MAX_ATTEMPTS]
        if not left:
            break
        if args.sim_running and session > 0:
            print('The simulation started by hand is not usable any more: restart it and use --resume')
            break
        session += 1
        print(f'Session {session}: {len(left)} scenarios', flush=True)
        sim = None
        try:
            if not args.sim_running:
                sim = start_sim(os.path.join(out, f'sim_{session}.log'), args.sim_startup, args.settle)
            rc = subprocess.run([sys.executable, os.path.abspath(__file__), '--campaign', args.campaign,
                                 '--worker', *left], env=dict(os.environ, MPLBACKEND='Agg')).returncode
        finally:
            stop_sim(sim)
        if rc not in (0, SESSION_BROKEN):
            print(f'Worker exited with code {rc}, see the main.log of the last run')
            # count the attempt of the scenario that was running, so the loop cannot spin forever
            for sid in left:
                st = read_status(os.path.join(out, sid))
                if not st.get('ok'):
                    write_status(os.path.join(out, sid), **{**st, 'attempts': st.get('attempts', 0) + 1})
                    break

    failed = [s['id'] for s in todo if not read_status(os.path.join(out, s['id'])).get('ok')]
    print(f'Done, {len(todo) - len(failed)}/{len(todo)} ok' + (f', failed: {" ".join(failed)}' if failed else ''))
    print(f'Analysis: python3 {os.path.join(HERE, "analyze_benchmark.py")} {out}')


if __name__ == '__main__':
    main()
