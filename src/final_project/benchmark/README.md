# Benchmark (camera without gimbal)

The follower is the system under test: its limits, gains, follow distance, timeouts and
camera stay as in `config.py` for every run. Only the leader changes, through the
overrides listed in `scenarios.py`.

| Phase | What it tests | Runs |
|---|---|---|
| A | hover, straight line at 3 / 6 / 10 m/s | 4 |
| B | circle R = 10 m at 3 / 5 / 7 m/s (limit ≈ 5.1 m/s), flat figure-8 at 2 / 3 m/s | 5 |
| C | slalom (lateral acceleration) and shuttle (acceleration along the line of sight), 2 / 4 / 6 m/s² | 6 |
| D | random waypoints with stops, 4 leader levels x 3 seeds | 12 |
| E | far spawn: exploration and search, levels 1-2 x 2 seeds | 4 |

Leader levels (maneuver limits): 1 = 5 m/s, 1.5 m/s² · 2 = 8 m/s, 3 m/s² ·
3 = 12 m/s, 5 m/s² · 4 = 15 m/s, 8 m/s². Follower tracking acceleration: 2.6 m/s² (tilt 15°).

## Running

The runner does what you do by hand when testing, without touching `config.py`:

1. starts Gazebo (`simulation/quad_simulation.sh`) - only once for all the `near` scenarios and
   once for the `far` ones, because the follower spawn is read by Gazebo at startup;
2. before every run: **Restart** (world reset) and **Play**, sent with `gz service` to
   `/world/mrsim/control` as the Gazebo buttons do;
3. runs `main_statemachine.py` with the config of the scenario (a JSON file named by
   `DRONE_CHASE_OVERRIDES`, applied at the end of `config.py`);
4. lets it finish, or sends Ctrl+C after `--timeout` seconds.

```sh
cd ~/tk3lab-ws/src/final_project/benchmark
python3 run_benchmark.py --campaign c1 --only A01 A03     # a few scenarios
python3 run_benchmark.py --campaign c1 --resume           # all of them, skipping the completed ones
python3 analyze_benchmark.py results/c1
```

- `--phase A B` or `--only D21 D22` run a subset; `--resume` skips the runs already completed.
- `--sim-running near` (or `far`) uses Gazebo already started by hand with that follower spawn:
  only the matching scenarios are run and Gazebo is never restarted.
- If the main fails within 30 s, the runner restarts Gazebo and the components and retries once;
  the log of the failed attempt is kept as `failed_main.log`. After any failed run (also a timeout)
  the next run starts with a fresh Gazebo, because a stuck GenoM3 component survives a World Restart.
- After each scenario the runner prints the state transitions (`tracking 1, lost 0, ...`): with the
  `near` spawn (phases A-D) `exploring` must be 0.
- Each run is in `results/<campaign>/<id>/`: `main.log`, `log/pom*.log`, `overrides.json`,
  `status.json`; the Gazebo output is in `results/<campaign>/sim_<spawn>_<n>.log` (one per Gazebo start).
- The report is in `results/<campaign>/report/`: `summary.csv`, `summary.md`, `phase_*.png`,
  `runs/<id>.png` (the `tracking_plotter.py` figure of each run). `results/` is not committed.
