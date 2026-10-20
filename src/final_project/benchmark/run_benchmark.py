#!/usr/bin/env python3
"""
Runs the benchmark scenarios one after the other, doing what you do by hand when testing:

  1. start Gazebo (simulation/quad_simulation.sh)    - only when the follower spawn changes
  2. Restart (world reset) and Play in Gazebo        - before every run
  3. run main_statemachine.py with the scenario config
  4. let it finish, or Ctrl+C after --timeout seconds

The config of each run is passed to the main as a JSON file (DRONE_CHASE_OVERRIDES, read
at the end of config.py): config.py itself is never modified. Gazebo restart and play are
sent with 'gz service' to /world/<name>/control, as the Gazebo buttons do.

Usage (inside the tk3lab container):
    python3 run_benchmark.py --campaign c1 --only A01 A03     # a few scenarios
    python3 run_benchmark.py --campaign c1 --resume           # all of them, skipping the done ones
    python3 run_benchmark.py --campaign c1 --sim-running near # Gazebo started by hand (spawn 'near')
Then: python3 analyze_benchmark.py results/<campaign>
"""

import argparse
import json
import os
import re
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
WORLD = os.path.join(os.path.dirname(SRC), 'gazebo', 'worlds', 'example.world')
MAIN = os.path.join(PROJECT, 'main_statemachine.py')
TMP = '/tmp/bm'                     # short paths: GenoM accepts log paths of at most 64 characters
EVENTS = {'tracking': 'Transitioning to TRACKING', 'lost': 'transitioning to SEARCHING',
          'reacquired': 'Leader re-acquired', 'exploring': 'Transitioning to EXPLORING'}
# Gazebo is ready when the 9 GenoM3 components are running and both drones have been created
READY_COMPONENTS = 9
READY_GAZEBO = 'mrsim created /tmp/pty-qr4_following'


# ---------------------------------------------------------------------------
# Gazebo
# ---------------------------------------------------------------------------

def cleanup():
    """Everything a previous simulation may have left: Gazebo, the GenoM3 components, genomixd
    and the pocolibs devices. With them still alive the new components cannot start and the
    main fails at once. pkill gets the patterns as arguments (a shell running
    'pkill -f "gz sim"' would match and kill itself)."""
    for cmd in (['pkill', '-f', 'gz sim'],
                ['pkill', '-f', '^(optitrack|nhfc|pom|rotorcraft|maneuver)-pocolibs'],
                ['pkill', '-x', 'genomixd']):
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)
    subprocess.run(['h2', 'end'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def start_sim(spawn, log_path, max_wait):
    """quad_simulation.sh with follower_spawn_mode = spawn (sync_camera_fov.py writes it in the
    world). Returns the process once components and drones are up."""
    cleanup()
    ov = os.path.join(os.path.dirname(log_path), f'sim_{spawn}.json')
    with open(ov, 'w') as f:
        json.dump({'follower_spawn_mode': spawn}, f)
    log = open(log_path, 'w')
    proc = subprocess.Popen(['sh', SIM_SCRIPT], env=dict(os.environ, DRONE_CHASE_OVERRIDES=ov),
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    t_end = time.monotonic() + max_wait
    while time.monotonic() < t_end and proc.poll() is None:
        with open(log_path, errors='replace') as f:
            text = f.read()
        if text.count('setup and running') >= READY_COMPONENTS and READY_GAZEBO in text:
            return proc
        time.sleep(0.5)
    stop_sim(proc)
    raise RuntimeError(f'simulation not ready after {max_wait:.0f}s, see {log_path}')


def stop_sim(proc):
    """Ctrl+C to the simulation (its trap kills components, Gazebo and h2), then force."""
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


def gz_env():
    """Same Gazebo transport partition/IP as quad_simulation.sh, so 'gz service' reaches the server."""
    ip = os.environ.get('GZ_IP') or subprocess.run(['hostname', '-I'], capture_output=True,
                                                    text=True).stdout.split()[0]
    return dict(os.environ, GZ_PARTITION=os.environ.get('GZ_PARTITION', 'tk3lab'), GZ_IP=ip)


def gz_control(world, request):
    """WorldControl request, as the Gazebo GUI buttons: 'reset: {all: true}' = Restart,
    'pause: false' = Play."""
    r = subprocess.run(['gz', 'service', '-s', f'/world/{world}/control',
                        '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000', '--req', request],
                       env=gz_env(), capture_output=True, text=True)
    if 'true' not in r.stdout:
        raise RuntimeError(f"gz service '{request}' failed: {r.stdout.strip()} {r.stderr.strip()}")


def restart_and_play(world, settle):
    gz_control(world, 'pause: true, reset: {all: true}')     # Restart
    time.sleep(1.0)
    gz_control(world, 'pause: false')                        # Play
    time.sleep(settle)


# ---------------------------------------------------------------------------
# one run
# ---------------------------------------------------------------------------

def run_main(sc, run_dir, args):
    """Runs the main with the scenario config; Ctrl+C after args.timeout. Returns (rc, duration)."""
    tmp = os.path.join(TMP, sc['id'])
    shutil.rmtree(tmp, ignore_errors=True)
    overrides = dict(sc['overrides'], output_dir=tmp, show_plots=False,
                     genom_log_dir=os.path.join(TMP, 'genom'))
    ov_path = os.path.join(run_dir, 'overrides.json')
    with open(ov_path, 'w') as f:
        json.dump(overrides, f, indent=2)
    env = dict(os.environ, DRONE_CHASE_OVERRIDES=ov_path, MPLBACKEND='Agg')

    t0 = time.monotonic()
    with open(os.path.join(run_dir, 'main.log'), 'w') as log:
        proc = subprocess.Popen([sys.executable, MAIN], env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            rc = proc.wait(args.timeout)
        except KeyboardInterrupt:
            # Ctrl+C in the terminal reaches the main too: let it stop the motors, then stop here
            try:
                proc.wait(20)
            except subprocess.TimeoutExpired:
                proc.kill()
            raise
        except subprocess.TimeoutExpired:
            proc.send_signal(signal.SIGINT)                  # Ctrl+C: the main stops the motors
            try:
                proc.wait(20)
            except subprocess.TimeoutExpired:
                proc.kill()
            rc = 'timeout'
    duration = time.monotonic() - t0

    for sub in ('log', 'plot'):
        if os.path.isdir(os.path.join(tmp, sub)):
            shutil.copytree(os.path.join(tmp, sub), os.path.join(run_dir, sub), dirs_exist_ok=True)
    return rc, duration


def read_status(run_dir):
    path = os.path.join(run_dir, 'status.json')
    if os.path.isfile(path):
        with open(path) as f:
            return json.load(f)
    return {}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default=time.strftime('%Y%m%d_%H%M'))
    ap.add_argument('--phase', nargs='+', help='only these phases (A B C D E)')
    ap.add_argument('--only', nargs='+', help='only these scenario ids')
    ap.add_argument('--resume', action='store_true', help='skip the runs already completed')
    ap.add_argument('--sim-running', choices=('near', 'far'),
                    help='use Gazebo started by hand with this follower spawn: only the matching '
                         'scenarios are run and Gazebo is never restarted')
    ap.add_argument('--sim-startup', type=float, default=90.0, help='max seconds to wait for Gazebo')
    ap.add_argument('--settle', type=float, default=3.0, help='seconds between Play and the main')
    ap.add_argument('--timeout', type=float, default=600.0, help='seconds before Ctrl+C to the main')
    args = ap.parse_args()

    todo = [s for s in SCENARIOS
            if (not args.phase or s['phase'] in args.phase) and (not args.only or s['id'] in args.only)]
    if args.sim_running:
        todo = [s for s in todo if s['overrides']['follower_spawn_mode'] == args.sim_running]
    # all 'near' scenarios first, then the 'far' ones: Gazebo restarts only once
    todo.sort(key=lambda s: s['overrides']['follower_spawn_mode'] != 'near')
    out = os.path.join(HERE, 'results', args.campaign)
    os.makedirs(out, exist_ok=True)
    with open(WORLD) as f:
        world = re.search(r'<world\s+name="([^"]+)"', f.read()).group(1)
    print(f'{len(todo)} scenarios -> {out}')

    sim, sim_spawn = None, args.sim_running
    try:
        for i, sc in enumerate(todo, 1):
            run_dir = os.path.join(out, sc['id'])
            if args.resume and read_status(run_dir).get('ok'):
                print(f'[{i}/{len(todo)}] {sc["id"]} already done')
                continue
            os.makedirs(run_dir, exist_ok=True)
            with open(os.path.join(run_dir, 'scenario.json'), 'w') as f:
                json.dump(sc, f, indent=2)

            spawn = sc['overrides']['follower_spawn_mode']
            if spawn != sim_spawn:                           # spawn changed: Gazebo must restart
                stop_sim(sim)
                print(f'Starting Gazebo (follower spawn {spawn}) ...', flush=True)
                sim = start_sim(spawn, os.path.join(out, f'sim_{spawn}.log'), args.sim_startup)
                sim_spawn = spawn

            print(f'[{i}/{len(todo)}] {sc["id"]} {sc["label"]} ...', end=' ', flush=True)
            restart_and_play(world, args.settle)
            rc, duration = run_main(sc, run_dir, args)
            if rc != 0 and duration < 30:
                # the main did not even reach the takeoff: Restart + Play again and retry once
                for name in ('main.log',):
                    os.replace(os.path.join(run_dir, name), os.path.join(run_dir, 'failed_' + name))
                print(f'failed after {duration:.0f}s (rc={rc}), retrying ...', end=' ', flush=True)
                restart_and_play(world, 3 * args.settle)
                rc, duration = run_main(sc, run_dir, args)

            ok = rc == 0 and os.path.isfile(os.path.join(run_dir, 'log', 'pom.log'))
            with open(os.path.join(run_dir, 'status.json'), 'w') as f:
                json.dump({'ok': ok, 'returncode': rc, 'main_duration_s': round(duration, 1)}, f)
            text = open(os.path.join(run_dir, 'main.log'), errors='replace').read()
            events = ', '.join(f'{k} {text.count(v)}' for k, v in EVENTS.items())
            print(f'{"ok" if ok else "FAILED"} ({duration:.0f}s) - {events}', flush=True)
    except KeyboardInterrupt:
        print('\nInterrupted: stopping Gazebo and its components (wait, do not press Ctrl+C again) ...')
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)        # the cleanup must not be interrupted
        if not args.sim_running:
            stop_sim(sim)

    print(f'Done. Analysis: python3 {os.path.join(HERE, "analyze_benchmark.py")} {out}')


if __name__ == '__main__':
    main()
