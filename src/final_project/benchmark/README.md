# Benchmark (camera without gimbal)

The follower is the system under test: its limits, gains, follow distance, timeouts and
camera stay as in `config.py` for every run. Only the leader changes, through the
overrides listed in `scenarios.py`.

| Phase | What it tests | Runs |
|---|---|---|
| A | hover, straight line at 3 / 6 / 10 m/s | 4 |
| B | circle R = 10 m at 3 / 5 / 7 m/s (limit ≈ 5.1 m/s), flat figure-8 at 2 / 3 m/s | 5 |
| C | altitude step of 2 / 5 m, figure-8 with H = 5 m | 3 |
| D | random waypoints with stops, 4 leader levels x 3 seeds | 12 |
| E | far spawn: exploration and search, levels 1-2 x 2 seeds | 4 |

Leader levels (maneuver limits): 1 = 5 m/s, 1.5 m/s² · 2 = 8 m/s, 3 m/s² ·
3 = 12 m/s, 5 m/s² · 4 = 15 m/s, 8 m/s². Follower tracking acceleration: 2.6 m/s² (tilt 15°).

## Running

Inside the tk3lab container. The runner starts **one** simulation for the whole campaign
(`simulation/quad_simulation.sh`, Gazebo unpaused): the drones take off once and between two
scenarios they fly back to the start positions, the follower first climbing to `reposition_z`
so it never crosses the leader. Also the follower spawn (`near` / `far`) is reached in flight,
so Gazebo never needs a restart. The simulation is restarted only if the drones cannot reach the
start any more (e.g. after a crash), and every scenario is tried at most twice.

```sh
cd ~/tk3lab-ws/src/final_project/benchmark
python3 run_benchmark.py --campaign c1 --only A01 A03     # a few scenarios
python3 run_benchmark.py --campaign c1 --resume           # all of them, skipping the completed ones
python3 analyze_benchmark.py results/c1
```

- `--phase A B` or `--only D21 D22` run a subset; `--resume` skips the runs already completed.
- `--sim-running` uses a simulation you started by hand (`sh quad_simulation.sh`, press Play):
  nothing is restarted, so if the drones crash the runner stops and you restart it with `--resume`.
- After each scenario the runner prints the state transitions (`tracking 1, lost 0, ...`): with the
  `near` spawn (phases A-D) `exploring` must be 0.
- Each run is in `results/<campaign>/<id>/`: `main.log`, `log/pom*.log`, `overrides.json`,
  `status.json`; the simulation output is in `results/<campaign>/sim_<n>.log`.
- The report is in `results/<campaign>/report/`: `summary.csv`, `summary.md`, `phase_*.png`,
  `runs/<id>.png` (the `tracking_plotter.py` figure of each run). `results/` is not committed.
